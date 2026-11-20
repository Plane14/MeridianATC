"""Autonomous flight control for the First Officer.

When enabled, the First Officer manages the entire flight from startup to
shutdown using the simulator's autopilot and aircraft systems. This includes
all phases: preflight, pushback, taxi, takeoff, climb, cruise, descent,
approach, landing, taxi-in, and shutdown.

Two autopilot management modes are supported (``autonomous_ap_mode``):
- "managed": the aeroplane's own managed modes -- flight-level change for
  the climb and descent, NAV for the route, APR for the approach, and the
  autothrottle's TO/GA where the model has one.
- "direct": headings, levels, vertical speeds and indicated airspeeds set
  explicitly, for the aeroplanes whose autopilots do not fly managed.

The controls go through :class:`~meridianatc.atc.ap.FlightDirector`, which
tries the cleanest means first -- autopilot modes, then the ``*_SET``
control events for the moments the AP cannot fly (rotation, the flare,
the go-around's pitch) -- and the geometry through
:mod:`meridianatc.atc.auto_nav`.

When chained flights are enabled, the First Officer will automatically
select random destinations within the aircraft's range at airports that
support the aircraft type and have ground services (fuel, parking, etc.).
"""

from __future__ import annotations

import contextlib
import logging
import math
import random
import re
import sqlite3
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import StrEnum

from ...config import Config
from ...navdata.build import haversine_nm, project
from ...navdata.db import Airport, NavDB
from ...sim.capabilities import autofly_limit, capabilities
from ...sim.source import SimConnectSource
from .. import approach_flight, auto_nav, procedure_legs, vspeeds
from .. import flying as fly
from .. import interphone as ip
from ..ap import FlightDirector
from ..checklist import holds
from ..controller import ControllerBrain
from ..crew import CrewDirector
from ..session import FlightSession, Phase, station_key

log = logging.getLogger(__name__)


class AutonomousPhase(StrEnum):
    """Internal autonomous flight phases."""
    INIT = "init"
    PREFLIGHT = "preflight"
    REQUEST_CLEARANCE = "request_clearance"
    PUSHBACK = "pushback"
    TAXI_OUT = "taxi_out"
    HOLDING_SHORT = "holding_short"
    LINE_UP = "line_up"
    TAKEOFF = "takeoff"
    CLIMB = "climb"
    CRUISE = "cruise"
    DESCENT = "descent"
    APPROACH = "approach"
    LANDING = "landing"
    TAXI_IN = "taxi_in"
    PARKED = "parked"
    SHUTDOWN = "shutdown"
    CHAIN_NEXT = "chain_next"
    COMPLETE = "complete"
    ERROR = "error"


# The sub-steps inside the phases that are sequences rather than waits.
# They live on the state as ``sub`` and the phase methods dispatch on it.
SUB_STATES = (
    # pushback
    "engines", "after_start", "request", "wait",
    # taxi
    "taxi", "hold_point",
    # line up
    "align",
    # takeoff
    "wait_clearance", "roll", "rotate", "initial_climb",
    # approach
    "intercept", "final", "go_around",
    # landing
    "flare", "rollout",
    # shutdown
    "secure", "cooldown", "engines_off", "electrics_off",
)


@dataclass
class AutonomousState:
    """State of the autonomous flight."""
    phase: AutonomousPhase = AutonomousPhase.INIT
    previous_phase: AutonomousPhase = AutonomousPhase.INIT
    phase_entered_at: float = 0.0
    # Flight plan and route
    has_ifr_clearance: bool = False
    clearance_read_back: bool = False
    # When the first clearance ask actually went out on the frequency --
    # the timeout's clock, so a radio that was still dead at phase entry
    # is not counted against the controller's answer.
    clearance_asked_at: float = 0.0
    departure_runway: str = ""
    arrival_runway: str = ""
    assigned_altitude_ft: float = 0.0
    assigned_heading: float = 0.0
    assigned_speed_kt: float = 0.0
    # The sub-step inside a phase, and when it began.
    sub: str = ""
    sub_entered_at: float = 0.0
    # Pushback tracking: where the simulator says nothing about the tug
    # the phase watches the groundspeed instead -- moving means the push
    # is under way, stationary again for a while means it is done.
    push_requested: bool = False
    moved_since_push: bool = False
    still_since: float = 0.0
    # When the push first moved the aeroplane -- the clock an open-ended
    # tug (X-Plane's) is let go on -- and whether that let-go was sent.
    push_moved_at: float = 0.0
    push_stop_sent: bool = False
    # Engines: when the start was asked for, so a model that never runs
    # them is an error rather than an endless wait.
    engines_started_at: float = 0.0
    start_retried: bool = False
    push_retries: int = 0
    # Whether the push is being asked for on the frequency -- an approval
    # that sends the tug -- rather than pressed for on the simulator.
    push_radio: bool = False
    # Taxi: progress along session.taxi_path and where it is held.
    taxi_index: int = 0
    hold_point_m: float = 0.0
    # Line up: the runway's own numbers, once found.
    runway_lat: float = 0.0
    runway_lon: float = 0.0
    runway_heading: float = 0.0
    runway_key: tuple[str, str] = ()
    runway_length_ft: float = 0.0
    runway_width_ft: float = 0.0
    aligned: bool = False
    # Takeoff: when rotation began, whether the wheels are up, and the
    # rejected-takeoff watch on the roll.
    rotate_at: float = 0.0
    airborne_at: float = 0.0
    rotate_speed_kt: float = 0.0
    rejected: bool = False
    # The rejected takeoff's own "holding" word, said once.
    abort_announced: bool = False
    # The speeds this leg is flown at, from vspeeds.
    v1_kt: float = 0.0
    vr_kt: float = 0.0
    v2_kt: float = 0.0
    vapp_kt: float = 0.0
    climb_kt: float = 0.0
    # Climb/cruise: the top of climb, and the step the controllers are on.
    top_of_climb: bool = False
    # Descent/approach: the planned top of descent, the configuration
    # stage, the stabilized-approach verdict, the minima, and the
    # go-arounds flown.
    tod_nm: float = 0.0
    configured: str = ""
    stabilized: bool = False
    minima_ft: float = 200.0
    go_arounds: int = 0
    go_around_at: float = 0.0
    missed_alt_ft: float = 0.0
    # See-and-avoid: the heading an airborne traffic conflict put the
    # aeroplane on (true, like the track it is worked from), the track
    # it left, how long the deviation stands, and which contact caused
    # it -- so the watch emits once rather than every tick.
    avoid_heading: float = -1.0
    avoid_from: float = -1.0
    avoid_until: float = 0.0
    avoid_key: str = ""
    avoid_since: float = 0.0
    # The hold the controller gave, as it is being flown: which fix,
    # which leg ("to_fix", "outbound", "inbound"), when the outbound
    # leg's clock started, and the nearest the fix has been on the way
    # in -- a fix passed abeam is still a fix passed.
    hold_fix: str = ""
    hold_leg: str = ""
    hold_leg_at: float = 0.0
    hold_nearest_nm: float = 0.0
    # "Proceed direct": the fix being flown to, the clearance it came
    # from as last seen (so the same fix cleared again is a new one),
    # its place on the route -- an ident a route names twice resolves to
    # the first occurrence, which is not the one a later leg is flown to
    # -- and the nearest it has been. Kept past the cleared fix, one fix
    # after another: the simulator's FMS was never told of the shortcut
    # and its active leg is still the one that was cut. An index of
    # len(route.fixes) is the field itself, the target once the route's
    # last fix is reached.
    direct_fix: str = ""
    direct_cleared: str = ""
    direct_index: int = -1
    direct_nearest_nm: float = 0.0
    # The tower's orbit: the altitude it is flown at, the track last
    # seen and how far round the aeroplane has turned by its own count.
    orbit_alt_ft: float = 0.0
    orbit_track: float = -1.0
    orbit_turned: float = 0.0
    # The cleared approach as a flyable recipe (approach_flight.resolve),
    # resolved once and kept until the field or the runway changes; whether
    # NAV1 has been tuned to its navaid yet, when that was last tried, and
    # the circling pattern's path once the circle has started.
    approach_recipe: object = None
    approach_recipe_tried: bool = False
    approach_recipe_key: tuple = ()
    approach_tuned: bool = False
    nav_tune_at: float = 0.0
    circling: bool = False
    circle_path: list = field(default_factory=list)
    # Filed/cleared SID and STAR guidance: the chosen transition's legs
    # walked by ProcedureNavigator, keyed so a new clearance or field is
    # a new navigator. ``procedure_suspended`` is the controller's
    # "resume own navigation" for the active procedure.
    sid_nav: object = None
    star_nav: object = None
    sid_key: tuple = ()
    star_key: tuple = ()
    procedure_suspended: str = ""
    # Landing: the flare's clock and the rollout's.
    flare_at: float = 0.0
    touchdown_at: float = 0.0
    rollout_at: float = 0.0
    # Said once, before departure: the field's density altitude when the
    # day is hot enough or high enough to make the takeoff numbers lie.
    density_alt_said: bool = False
    vacated: bool = False
    vacated_call: bool = False
    reversers_out: bool = False
    # The stand the aeroplane is parked against: whether the "still
    # attached" note was said, whether GSX's flight-deck-ready prompt
    # was answered, and how far the arrival's stand services got.
    stand_hold_said: bool = False
    push_confirmed: bool = False
    ramp_step: int = 0
    # The last circuit leg reported on a broadcast frequency, where
    # nobody clears: each leg is said once as it is flown.
    circuit_said: str = ""
    # Shutdown: the stage clock.
    shutdown_at: float = 0.0
    # Emergencies: what was declared, to where, and whether the divert is
    # already set up.
    emergency: str = ""
    emergency_declared: bool = False
    emergency_destination: Airport | None = None
    # An emergency descent gets down now, not at the plan's top of
    # descent; a gear that stayed up gets one go-around, then the belly.
    emergency_descent: bool = False
    gear_ga_done: bool = False
    gear_up_said: bool = False
    # Chain flights
    chain_flight_index: int = 0
    chain_flights: list[dict] = field(default_factory=list)
    # Aircraft performance for chain planning
    aircraft_range_nm: float = 0.0
    aircraft_type: str = ""
    # The sim's own word that the airframe carries floats -- a landplane
    # code does not say (aircraft_class.amphibious).
    aircraft_floats: bool = False
    # Error tracking
    error_message: str = ""
    error_count: int = 0
    # Autopilot bookkeeping the director keeps itself; these flags remain
    # for anything that reads them.
    ap_master: bool = False
    ap_hdg_mode: bool = False
    ap_alt_mode: bool = False
    ap_apr_mode: bool = False
    ap_nav_mode: bool = False
    ap_vs_mode: bool = False
    ap_speed_mode: bool = False
    # Checklist tracking
    checklists_completed: set[str] = field(default_factory=set)
    # The radios, worked by the FO itself: COM1 has been put on the
    # departure field's position, which station was last checked in with,
    # and what the session's squawk/altimeter were when they were last
    # dialled in -- a new value is a new instruction to action.
    comms_tuned: bool = False
    checked_in_with: str = ""
    squawk_dialled: str = ""
    altimeter_dialled: float = 0.0
    # -inf: a 0.0 seed reads as "radio just now" while the host uptime
    # is still inside any of the quiet beats below it.
    last_radio_at: float = float("-inf")
    # The turnaround between chained legs: the plan being serviced for,
    # whether the engines were shut and the fuel asked for, and where the
    # diversion field is that the fuel figure was built on.
    plan_flight: dict | None = None
    service_started_at: float = 0.0
    gsx_asked: bool = False
    gsx_busy: bool = False
    fuel_wrote: bool = False
    fuel_wrote_at: float = 0.0
    service_warned: bool = False
    alternate: Airport | None = None
    # The warnings a flight gets once: overweight at the hold, the flap
    # speed the schedule asked for and could not have, minimum fuel
    # already said on the radio, and the short-of-diversion note at the
    # hold.
    mtow_warned: bool = False
    flap_gate_warned: bool = False
    gear_gate_warned: bool = False
    minimum_fuel_said: bool = False
    depart_fuel_warned: bool = False
    # What the weather has already been reacted to: the wind that held a
    # departure, the icing the systems were turned on for, the rough air
    # the speed was pulled back through, the approach conditions that
    # sent the flight around, and how many times the weather won.
    wind_hold_said: bool = False
    wind_hold_since: float = 0.0
    convective_hold_said: bool = False
    icing_said: bool = False
    ice_warned: bool = False
    turbulence_until: float = 0.0
    turbulence_said: bool = False
    windshear_since: float = 0.0
    weather_gas: int = 0
    weather_ga_active: bool = False
    # The pop-up IFR pickup: when the sim first reported the aeroplane
    # in cloud on a VFR leg -- a wisp of cloud does not file anything,
    # a stay in it does.
    imc_since: float = 0.0
    # The engine watch (x_the_engines): a limit exceedance already said
    # once, and whether the carburetted engine's heat is a lever the FO
    # put on itself -- the flag that lets it come off when the sky dries.
    eng_warned: bool = False
    carb_set: bool = False
    # The cooling watch: where the FO last put the cowl flaps (-1 =
    # never), whether the CHT opened them itself, and whether the
    # "nearing limit" call was already made.
    cowl_set: float = -1.0
    cht_flaps: bool = False
    cht_warned: bool = False
    # The load-factor watch (x_the_load): whether a limit is being
    # approached or has been exceeded, when the exceedance started for
    # the clear-down hysteresis, whether the warning was emitted, and
    # the G the last touchdown arrived with. ``demeanor`` is how hard
    # the FO is currently allowed to fly -- smooth, gentle or urgent.
    g_limited: bool = False
    g_limited_since: float = 0.0
    g_warned: bool = False
    touchdown_g: float = 0.0
    demeanor: str = "smooth"
    # The stall watch (x_the_stall): when the trigger first held, how
    # long the instruments have agreed the wing is flying again, whether
    # the recovery currently owns the axes, and the calls already made.
    stall_since: float = 0.0
    stall_clear_since: float = 0.0
    stall_recovering: bool = False
    stall_warned: bool = False
    stall_count: int = 0
    # The terrain watch (x_shared_helpers): how long a pull-up off low
    # ground owns the climb, and the level it climbs to -- worked out
    # from the elevation the sim itself reports under the aeroplane,
    # not a database.
    terrain_escape_until: float = 0.0
    terrain_escape_alt: float = 0.0
    # The VS bug as last commanded, and when, for the slew-limited
    # descent entries of _apply_flight_director; and the AP bank cap
    # last asked for, so it is not re-sent every pass.
    vs_bug_cmd: float = 0.0
    vs_bug_at: float = 0.0
    bank_cap_deg: float = 0.0
    # Timing
    last_action_at: float = 0.0


# Which closed-loop channels a sub-step entering resets, so a stale
# integrator or a wound output from the last regime never flies the new
# one. Only the hand-flown axes are listed; the flight-director modes
# own the rest.
_SUB_RESETS = {
    "rotate": ("rotation",),
    "flare": ("flare",),
    "taxi": ("steer", "taxi"),
    "roll": ("steer",),
    "rollout": ("steer", "taxi"),
    "align": ("steer", "taxi"),
    # A go-around leaves the vertical-path loop behind: whatever it had
    # wound belongs to the approach it abandoned, not the next one.
    "go_around": ("vertical",),
}

# The groundspeed edges the learned yaw authorities are kept against, in
# knots. A nosewheel's authority grows with the speed -- the same tiller
# travel turns the nose faster the faster the wheels roll -- and the
# rudder's with its square, so a figure measured on the apron is not the
# figure that holds on the roll. Six bands, indexed 0-5.
_GS_BAND_EDGES = (8.0, 16.0, 26.0, 45.0, 80.0)


def _gs_band(gs: float) -> int:
    """Which groundspeed band a speed lands in, 0-5."""
    band = 0
    for edge in _GS_BAND_EDGES:
        if gs >= edge:
            band += 1
    return band


def _nws_fade(gs: float) -> float:
    """How much of the demanded tiller the nosewheel still gets at this
    groundspeed: all of it at taxi pace, none of it once the rudder is
    the authority.

    A steering wheel that still bought full nosewheel travel at seventy
    knots would be the accident it is scheduled away from being -- on a
    transport the tiller's authority is faded out by the speed the
    rudder takes over (the A320's handwheel goes from about 75 degrees
    at rest to nothing by 70 kt; a 737's pedals never buy more than
    seven). The fade spans 20-70 kt: below it the nosewheel is the
    steering, through it the two share, above it the rudder alone
    answers.
    """
    if gs <= 20.0:
        return 1.0
    if gs >= 70.0:
        return 0.0
    return 1.0 - (gs - 20.0) / 50.0


def _yaw_rate_cap(gs: float) -> float:
    """The fastest turn the taxi steering law may order, in deg/s.

    Fifteen degrees a second is a brisk ramp turn at walking speed; the
    same demand at twenty-five knots is the swerve this cap exists to
    prevent, so it schedules down with the knots the way a pilot's hand
    asks for less as the ride speeds up.
    """
    if gs <= 8.0:
        return 15.0
    if gs >= 26.0:
        return 6.0
    return 15.0 - (gs - 8.0) * 0.5




__all__ = [
    "log",
    "AutonomousPhase",
    "SUB_STATES",
    "AutonomousState",
    "_SUB_RESETS",
    "_GS_BAND_EDGES",
    "_gs_band",
    "_nws_fade",
    "_yaw_rate_cap",
    "contextlib",
    "logging",
    "math",
    "random",
    "re",
    "sqlite3",
    "time",
    "Callable",
    "dataclass",
    "field",
    "StrEnum",
    "Config",
    "haversine_nm",
    "project",
    "Airport",
    "NavDB",
    "autofly_limit",
    "capabilities",
    "SimConnectSource",
    "approach_flight",
    "auto_nav",
    "procedure_legs",
    "vspeeds",
    "fly",
    "ip",
    "FlightDirector",
    "holds",
    "ControllerBrain",
    "CrewDirector",
    "FlightSession",
    "Phase",
    "station_key",
]
