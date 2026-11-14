from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    AutonomousState,
    Callable,
    Config,
    ControllerBrain,
    CrewDirector,
    FlightDirector,
    FlightSession,
    NavDB,
    SimConnectSource,
    fly,
    vspeeds,
)
from .x_chain_flights_unchanged import _Chain_flights_unchanged
from .x_circling import _Circling
from .x_emergencies import _Emergencies
from .x_entry import _Entry
from .x_filed_cleared_sid import _Filed_cleared_sid
from .x_fuel_planning import _Fuel_planning
from .x_helpers_kept_from import _Helpers_kept_from
from .x_plumbing import _Plumbing
from .x_shared_helpers import _Shared_helpers
from .x_taxi import _Taxi
from .x_the_approach_s import _The_approach_s
from .x_the_checklist_answered import _The_checklist_answered
from .x_the_cleared_approach import _The_cleared_approach
from .x_the_engines import _The_engines
from .x_the_envelope import _The_envelope
from .x_the_load import _The_load
from .x_the_loop import _The_loop
from .x_the_phase_machine import _The_phase_machine
from .x_the_radios import _The_radios
from .x_the_stall import _The_stall
from .x_the_turnaround_between import _The_turnaround_between
from .x_weather import _Weather


class AutonomousFlight(_Plumbing, _Entry, _The_loop, _Shared_helpers, _The_envelope, _The_load, _The_stall, _The_engines, _The_radios, _The_checklist_answered, _The_phase_machine, _The_turnaround_between, _Taxi, _Filed_cleared_sid, _The_approach_s, _The_cleared_approach, _Circling, _Emergencies, _Fuel_planning, _Weather, _Chain_flights_unchanged, _Helpers_kept_from):
    """Manages autonomous flight from startup to shutdown."""

    def __init__(
        self,
        config: Config,
        navdb: NavDB,
        sim_source: SimConnectSource | None,
        session: FlightSession,
        crew: CrewDirector,
        brain: ControllerBrain,
        get_state: Callable[[], object],
        on_event: Callable[[str, str, str], None],
        use_route: Callable[[object], None] | None = None,
    ):
        self.config = config
        self.navdb = navdb
        self.sim = sim_source
        self.session = session
        self.crew = crew
        self.brain = brain
        self.get_state = get_state
        self.on_event = on_event
        self.use_route = use_route

        self.state = AutonomousState()
        self._enabled = config.immersion.autonomous_flight
        # The rotorcraft controller, when the aeroplane in the sim is a
        # helicopter: it flies the whole leg itself and this class only
        # delegates (meridianatc.atc.autonomous_heli).
        self._heli = None
        self._is_helicopter = False
        self._chained = config.immersion.autonomous_chained_flights
        self._ap_mode = config.immersion.autonomous_ap_mode
        self._chain_delay = config.immersion.autonomous_chain_delay_s
        self._request_ifr = config.immersion.autonomous_request_ifr
        self._use_sim_ai = config.immersion.autonomous_use_sim_ai
        # The FO's own jobs while it flies: the radios, the turnaround
        # fuelling between legs, and the reserve the legs are planned to.
        self._manage_radio = getattr(
            config.immersion, "autonomous_manage_comms", True)
        self._turnaround_service = getattr(
            config.immersion, "autonomous_service_turnaround", True)
        self._reserve_min = float(getattr(
            config.immersion, "autonomous_divert_reserve_min", 45.0) or 45.0)
        # Whether the weather is reacted to at all. Off, the FO flies the
        # leg it was given whatever falls out of the sky on it.
        self._weather_reactions = getattr(
            config.immersion, "autonomous_weather", True)
        # The biggest fuel_total_gal the model has ever shown: the
        # capacity for a model that declares no FUEL TOTAL CAPACITY.
        self._fuel_capacity_seen = 0.0
        # Which checklist item the FO is working, and since when -- an
        # item that cannot be made to hold is answered after the wait,
        # not waited on for ever.
        self._checklist_item_id = 0
        self._checklist_item_since = 0.0
        # When the squawk/altimeter were last pushed into the radios, so a
        # dial that does not take is retried on a clock, not every tick;
        # and whether COM1 is showing the frequency it should, which is
        # what holds an ask across a retune still settling.
        self._dialled_at = 0.0
        self._comms_settled = True
        self._call_tried_at = 0.0
        self._dir: FlightDirector | None = None
        self._perf: fly.Performance | None = None
        self._speeds: vspeeds.Speeds | None = None
        self._fuel_known = False
        self._ga_flag_seen = False
        # The last few headwind samples on final, for the windshear
        # swing the watch works off.
        self._wind_z_hist: list = []
        self._last_state = None
        # The throttle servo's last commanded lever position, 0..1, and
        # when -- the slew limiter works off the pair.
        self._last_lever = 0.0
        self._lever_at = 0.0
        # When the aeroplane last refused to move under taxi power.
        self._ground_still_since = 0.0
        # Whether the hold the taxi is stopped at is a cut route's split
        # taxiway rather than the runway's holding point.
        self._held_at_split = False
        # The closed-loop axes. Gains are resolved per aircraft like the
        # performance figures (class bundle, autofly profile refinements);
        # the controllers are made lazily, one per channel, and the
        # immersion.autonomous_pid switch can return every axis to the
        # open-loop rules they replaced for a model that misbehaves.
        self._gains = None
        self._pids: dict = {}
        # What this airframe has taught the loops before -- trims, gain
        # scales, control polarities -- per aircraft model, kept on disk.
        # None until begin_flight resolves the model it is flying.
        self._adapt = None

        # Compute aircraft range for chain planning. The engine builds this
        # before a flight exists, so the session can be None.
        if session is not None:
            self._compute_aircraft_performance(session)

        # Timing constants
        self.ACTION_INTERVAL = 1.0  # seconds between autonomous actions
        # Four minutes for the clearance or the release: a busy delivery
        # frequency answers within it, and a flight that still has no
        # paper after it is a failure -- never a quiet re-file to VFR.
        self.CLEARANCE_TIMEOUT = 240.0
        self.ENGINE_START_TIMEOUT = 240.0
        self.TAKEOFF_TIMEOUT = 240.0
        self.CLIMB_TIMEOUT = 3600.0
        self.DESAT_TIMEOUT = 3600.0
        self.APPROACH_TIMEOUT = 1200.0
        self.LANDING_TIMEOUT = 600.0
        self.SHUTDOWN_COOLDOWN_S = 90.0

def create_autonomous_flight(
    config: Config,
    navdb: NavDB,
    sim_source: SimConnectSource | None,
    session: FlightSession,
    crew: CrewDirector,
    brain: ControllerBrain,
    get_state: Callable[[], object],
    on_event: Callable[[str, str, str], None],
    use_route: Callable[[object], None] | None = None,
) -> AutonomousFlight:
    """The First Officer's machine, always built.

    It used to exist only when the setting was on at startup, which made
    the panel's engage button dead until a restart. The object is cheap
    and inert until engaged -- ``_enabled`` (the setting, default off)
    is what gates the tick, not the object's existence.
    """
    return AutonomousFlight(
        config, navdb, sim_source, session, crew, brain, get_state,
        on_event, use_route
    )

__all__ = ["AutonomousFlight", "AutonomousPhase", "AutonomousState", "create_autonomous_flight"]
