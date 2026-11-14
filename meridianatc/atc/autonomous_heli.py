"""Autonomous flight for rotorcraft: the helicopter's own controller.

:class:`~meridianatc.atc.autonomous.AutonomousFlight` delegates to this when the
aeroplane in the sim is a helicopter (``aircraft_class.HELICOPTER``). It is
a separate controller rather than more branches in the fixed-wing one because
a rotorcraft is flown differently in every phase: there is no takeoff roll,
no rotation, no flaps or gear, and the "autopilot" is the pilot's own hands
on the cyclic, the collective and the pedals.

**VFR only.** There are no helicopter instrument approach procedures in the
navdata this program carries, and the fixed-wing IFR phases (runway lineup,
ILS capture) do not apply. An IFR session is downgraded to VFR with an event,
never flown through the airplane logic.

**Best effort.** MSFS has no universal helicopter start or shutdown event --
every add-on wires its own -- so the engine sequence is: the generic
``ENGINE_AUTO_START``/``ENGINE_AUTO_SHUTDOWN`` events, the rotor governor and
clutch switches where the model answers them, and then watching
``engines_running`` and ``rotor_rpm_pct`` with timeouts rather than hoping.
The same goes for the flight controls: ``AXIS_COLLECTIVE_SET``,
``AXIS_CYCLIC_*_SET``, ``AXIS_STEERING_SET`` and the helicopter throttle
events are sent and their effect is judged from the state, not assumed.
``AUTO_HOVER_*`` is documented by the SDK as not implemented and is never
sent.

Every gain below is UNVERIFIED: no running 2024 has been flown with this.
The controllers are deliberately soft -- small proportional terms inside
tight clamps -- so a model that answers the axes differently from the
assumption degrades into a bad flight the pilot can see, not an unrecoverable
one. ``logs`` and the ``autonomous.*`` events say what was tried.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from .aircraft_class import HELICOPTER
from .autonomous import AutonomousPhase
from .powerplant import limits_for
from .vspeeds import speeds_for

log = logging.getLogger(__name__)

# --- the axes -------------------------------------------------------------
# MSFS 2024 helicopter key events (SDK "Helicopter Specific Events"). The
# collective and the tail rotor take 0..16384; the cyclics and the steering
# axis take -16384..16384. ``_send`` masks to a DWORD, which is the
# two's-complement form SimConnect expects for the signed axes.
AXIS_MAX = 16384
COLLECTIVE_SET = b"AXIS_COLLECTIVE_SET"
CYCLIC_LONG_SET = b"AXIS_CYCLIC_LONGITUDINAL_SET"
CYCLIC_LAT_SET = b"AXIS_CYCLIC_LATERAL_SET"
STEERING_SET = b"AXIS_STEERING_SET"
# The pedals are anti-torque, not nosewheel: ``ROTOR_AXIS_TAIL_ROTOR_SET``
# is the helicopter's yaw channel on the SDK's signed axis scale. The
# 2024 reference also lists ``AXIS_TAIL_ROTOR_SET`` -- a tail-rotor
# *speed* command on a 0..16384 range, which cannot carry a left-pedal
# input -- so it is the fallback only, never the first ask; the steering
# axis only matters to the models that ground-taxi on wheels.
PEDAL_SET = b"ROTOR_AXIS_TAIL_ROTOR_SET"
TAIL_ROTOR_SET = b"AXIS_TAIL_ROTOR_SET"
HELI_THROTTLE_SET = b"AXIS_HELICOPTER_THROTTLE_SET"
# Named throttle events as fallbacks, per the same page: "set all
# throttles", and the per-engine forms a twin might answer.
THROTTLE_ALL_SET = b"HELICOPTER_THROTTLE_SET"
THROTTLE_ALL_FULL = b"HELICOPTER_THROTTLE_FULL"
THROTTLE_ALL_CUT = b"HELICOPTER_THROTTLE_CUT"
ROTOR_GOV_SET = b"ROTOR_GOV_SWITCH_SET"
ROTOR_CLUTCH_SET = b"ROTOR_CLUTCH_SWITCH_SET"
ROTOR_BRAKE = b"ROTOR_BRAKE"
ROTOR_BRAKE_ON = b"ROTOR_BRAKE_ON"
ENGINE_START = b"ENGINE_AUTO_START"
ENGINE_STOP = b"ENGINE_AUTO_SHUTDOWN"

# --- the flight, conservatively -------------------------------------------
# UNVERIFIED everywhere: chosen inside the light singles' envelopes the
# FLIGHT Checklist profiles fly (H125, Bell 407, EC135, UH-1H).
HOVER_AGL_FT = 30.0            # the check hover, and the departure hover
TRANSITION_KT = 40.0           # out of ground effect, through the H-V curve
CLIMB_VS_FPM = 800.0           # best-rate-ish, well inside the profiles' 700-1000
MAX_VS_FPM = 1200.0            # the envelope: never command more
MAX_IAS_KT = 120.0             # a light single's VNE is near here; never above
PATTERN_AGL_FT = 800.0         # the altitude the circuit is flown at
FINAL_AGL_FT = 150.0           # where the approach becomes the landing
LANDING_VS_FPM = -300.0        # the descent onto the spot
FLARE_AGL_FT = 25.0            # collective comes up under this
CRUISE_CAP_FT = 6000.0         # a light single does not cruise at 39,000
START_TIMEOUT_S = 240.0        # spool-up on a cold turbine is slow
ROTOR_MIN_PCT = 80.0           # "flight RPM" for takeoff, lower than the 85 call
SHUTDOWN_COOLDOWN_S = 120.0    # best-effort idle cool-down before the brake
ROTOR_BRAKE_BELOW_PCT = 40.0   # apply the rotor brake under this, where fitted

# --- the controllers ------------------------------------------------------
# Collective: feet of altitude error per per-cent of collective, and the
# vertical-speed damping. UNVERIFIED gains; the clamps are the protection.
COLL_KP_PER_FT = 0.12          # per cent collective per foot of error
COLL_KD_PER_FPM = 0.04         # per cent collective per fpm of sink/climb
COLL_TRIM_PCT = 55.0           # the hover collective, guessed then adapted
COLL_ADAPT = 0.002             # how fast the trim learns, per second
# Cyclic longitudinal: axis units per knot of airspeed error. The SIGN is
# UNVERIFIED -- which way the axis pitches the model -- so the constant is
# written once, here, and flipped if a flight shows it backwards.
CYCLIC_KT_PER_AXIS = 30.0
CYCLIC_MAX = 6000              # about a third of full deflection
# Pedals/steering: axis units per degree of heading error.
PEDAL_DEG_PER_AXIS = 8.0
PEDAL_MAX = 5000
# The trim the cyclic starts from each regime, in axis units: a little
# forward to fly, centred to hover.
CYCLIC_TRIM_AIRBORNE = -1500
CYCLIC_TRIM_HOVER = 0
# How hard the engines are asked for in the air: the helicopter throttle
# event, 0..16384, full for takeoff and climb, governed in cruise.
THROTTLE_CLIMB = AXIS_MAX
THROTTLE_CRUISE = int(AXIS_MAX * 0.75)

# --- per-type envelopes ---------------------------------------------------
# Each rotorcraft is not the same aeroplane: an R44 and an S-76 do not
# transition at the same speed, climb at the same rate or see the same
# ceiling. The figures below are derived from the EUROCONTROL ATCPFDB
# entries for the type (indicative, not operational data) -- cruise TAS
# for the cruise speed, initial-climb ROC for the climb rate, approach
# IAS for the approach speed, ceiling and range as published. Where the
# database's helicopter rows are fixed-wing-looking (the H25x entries
# publish jet figures) or absent, the code maps to the nearest sane row
# or keeps the DEFAULT profile. UNVERIFIED like everything else here.


@dataclass(frozen=True)
class HeliProfile:
    """One rotorcraft's envelope: speeds, rates, the ceiling and the
    rotor's load limits.

    ``pos_g``/``neg_g`` bound the normal load factor; ``low_g`` is the
    caution floor below which a teetering rotor mast-bumps -- the R44
    POH's "avoid low-G" made a number. UNVERIFIED like every figure
    here: the defaults are the light single's book values."""

    ias_cruise_kt: float
    ias_approach_kt: float
    climb_vs_fpm: float
    max_vs_fpm: float
    max_ias_kt: float
    ceiling_ft: float
    transition_kt: float
    landing_vs_fpm: float
    pos_g: float = 2.5
    neg_g: float = -0.5
    low_g: float = 0.2
    # The autorotation figures: the speed the rotor keeps energy best at
    # (the type's published auto speed where known, else the approach
    # figure) and the height the flare starts spending it.
    auto_kt: float = 0.0
    auto_flare_ft: float = 40.0


def _heli_profile(cruise_tas: float, approach_ias: float, roc_fpm: float,
                  ceiling_ft: float, low_g: float = 0.2,
                  auto_kt: float = 0.0,
                  auto_flare_ft: float = 40.0) -> HeliProfile:
    """Derive a profile from the database's raw figures, clamped to what
    a light helicopter envelope looks like.

    ``low_g`` is the teetering-rotor mast-bump floor: 0.4 on the
    Robinsons and Bells with a two-blade seesaw rotor, the articulated
    default elsewhere."""
    def clamp(v, lo, hi):
        return min(hi, max(lo, v))

    cruise = clamp(cruise_tas, 60.0, 140.0)
    approach = clamp(approach_ias, 45.0, 90.0)
    climb = clamp(roc_fpm * 0.75, 600.0, 1400.0)
    return HeliProfile(
        ias_cruise_kt=cruise,
        ias_approach_kt=approach,
        climb_vs_fpm=climb,
        max_vs_fpm=min(climb * 1.5, 2000.0),
        max_ias_kt=min(cruise + 30.0, 160.0),
        ceiling_ft=ceiling_ft,
        transition_kt=clamp(cruise * 0.4, 30.0, 60.0),
        landing_vs_fpm=-clamp(roc_fpm * 0.3, 200.0, 400.0),
        low_g=low_g, auto_kt=auto_kt, auto_flare_ft=auto_flare_ft)


DEFAULT_HELI_PROFILE = HeliProfile(
    ias_cruise_kt=80.0, ias_approach_kt=60.0, climb_vs_fpm=CLIMB_VS_FPM,
    max_vs_fpm=MAX_VS_FPM, max_ias_kt=MAX_IAS_KT, ceiling_ft=CRUISE_CAP_FT,
    transition_kt=TRANSITION_KT, landing_vs_fpm=LANDING_VS_FPM)

# Keyed by the type code the session carries; where MeridianATC's code is not
# the database's, the row is the nearest equivalent (B430 stands in for
# the AS350 family, EC35 for the EC130/135/145 family, B412 for the
# medium twins, AS32 for the Super Puma). UNVERIFIED mappings.
_HELI_PROFILES: dict[str, HeliProfile] = {
    # Robinson R44: piston, 110 kt, 1,000 fpm. Teetering rotor. The POH's
    # autorotation speed is ~65 KIAS with the flare at about 40 ft.
    "R44": _heli_profile(110, 80, 1000, 14000, low_g=0.4,
                         auto_kt=65.0, auto_flare_ft=40.0),
    # Airbus H125 / Leonardo H125 and the AS350 family via Bell 430.
    "H125": _heli_profile(130, 60, 1400, 14600, auto_kt=65.0),
    "B407": _heli_profile(130, 60, 1400, 14600, auto_kt=65.0),
    "R66": _heli_profile(130, 60, 1400, 14600, low_g=0.4, auto_kt=65.0),
    # Leonardo H145 / EC145 via the EC135 row.
    "H145": _heli_profile(135, 90, 1500, 20000, auto_flare_ft=75.0),
    # Airbus H160 has no database row; EC35 is its closest published kin.
    "H160": _heli_profile(135, 90, 1500, 20000, auto_flare_ft=75.0),
    # Eurocopter EC130/135/145 family.
    "EC30": _heli_profile(135, 90, 1500, 20000),
    "EC35": _heli_profile(135, 90, 1500, 20000, auto_flare_ft=75.0),
    "EC45": _heli_profile(135, 90, 1500, 20000, auto_flare_ft=75.0),
    # Airbus AS365 Dauphin via the medium-twin row.
    "AS50": _heli_profile(125, 60, 1300, 20000, auto_flare_ft=75.0),
    # Bell 206 light single. Teetering rotor; the auto speed is inside
    # the 52-75 KIAS window the RFM draws.
    "B06": _heli_profile(120, 80, 1350, 13500, low_g=0.4, auto_kt=60.0),
    # Bell 412 / Agusta A139 medium twin. The 412 is also teetering.
    "A139": _heli_profile(125, 60, 1300, 20000, auto_flare_ft=75.0),
    "B412": _heli_profile(125, 60, 1300, 20000, low_g=0.4,
                          auto_flare_ft=75.0),
    # Sikorsky S-76 medium-lift twin.
    "S76": _heli_profile(150, 60, 1800, 13800, auto_flare_ft=75.0),
    # Leonardo AW109 light-medium twin.
    "A109": _heli_profile(140, 100, 1900, 19600),
    # Airbus AS332 Super Puma heavy twin.
    "AS32": _heli_profile(140, 90, 1500, 17000, auto_flare_ft=75.0),
    # Bell UH-1 utility single. Teetering rotor; the book's auto entry
    # is 75-90 KIAS.
    "UH1": _heli_profile(110, 60, 1755, 19000, low_g=0.4,
                         auto_kt=75.0, auto_flare_ft=60.0),
}


def heli_profile_for(type_code: str) -> HeliProfile:
    """The envelope for this rotorcraft, or the conservative default."""
    return _HELI_PROFILES.get((type_code or "").strip().upper(),
                              DEFAULT_HELI_PROFILE)


# The internal machine. The owner-visible phase (AutonomousPhase) is a
# coarser view of the same flight.
STAGES = ("preflight", "start", "hover", "departure", "climb", "cruise",
          "descent", "approach", "landing", "shutdown", "done",
          "autorotate")

# stage -> the AutonomousPhase the rest of the program sees
_STAGE_PHASE = {
    "preflight": AutonomousPhase.PREFLIGHT,
    "start": AutonomousPhase.PREFLIGHT,
    "hover": AutonomousPhase.TAXI_OUT,
    "departure": AutonomousPhase.TAKEOFF,
    "climb": AutonomousPhase.CLIMB,
    "cruise": AutonomousPhase.CRUISE,
    "descent": AutonomousPhase.DESCENT,
    "approach": AutonomousPhase.APPROACH,
    "landing": AutonomousPhase.LANDING,
    "autorotate": AutonomousPhase.DESCENT,
    "shutdown": AutonomousPhase.SHUTDOWN,
    # "done" is deliberately absent: entering it must not park the owner in
    # COMPLETE, or the chain step in ``_done`` would never run.
}


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _heading_error(want: float, have: float) -> float:
    """The shortest way round, -180..180."""
    return ((want - have + 540.0) % 360.0) - 180.0


def _pedal_dir(self) -> int:
    """The sign this model's pedal axis actually answers in."""
    return -1 if getattr(self, "_pedal_flip", False) else 1


def _pedal_check(self, demand: int, hdg: float) -> None:
    """Flip the pedal sign when the axis answers backwards.

    The same check the fixed-wing steering runs: a pedal axis on which
    the answer and the demand disagree is the inverted-axis model,
    where every correction enlarges its error -- the hover version of
    the apron pivot. The yaw rate answers the demand already on the
    wire, so the vote is whether the achieved yaw agrees with the
    *law's* demand from last call, not the signed output.
    """
    prev = getattr(self, "_pedal_demand", None)
    prev_hdg = getattr(self, "_pedal_hdg", None)
    prev_at = getattr(self, "_pedal_at", 0.0)
    now = time.monotonic()
    self._pedal_demand = demand
    self._pedal_hdg = hdg
    self._pedal_at = now
    if prev is None or prev_hdg is None or not prev_at:
        return
    dt = now - prev_at
    if dt <= 0.5:
        return
    rate = _heading_error(hdg, prev_hdg) / dt
    if abs(prev) < PEDAL_MAX * 0.3 or abs(rate) < 5.0:
        return
    votes = getattr(self, "_pedal_votes", 0)
    votes += 1 if (rate > 0.0) == (prev > 0) else -1
    if votes <= -3:
        self._pedal_flip = not getattr(self, "_pedal_flip", False)
        votes = 3
        log.warning("pedal axis answers backwards on this model; "
                    "pedal sign flipped")
    self._pedal_votes = votes


def _committed_error(err: float, commit: int) -> tuple[float, int]:
    """A near-reversal committed to one side.

    At ~180 degrees of heading error the shortest way round flips on a
    degree of wobble, and a pedal loop free to re-pick yaws the machine
    left, then right, then left again around the point instead of
    through it -- the same pivot the fixed-wing steering showed on the
    apron. Once the error is genuinely big the side it first showed is
    kept until the turn is mostly made; the caller holds the commit in
    its own state between ticks.
    """
    if commit and abs(err) < 60.0:
        commit = 0
    if not commit and abs(err) > 150.0:
        commit = 1 if err > 0.0 else -1
    if commit:
        err = commit * max(60.0, abs(err))
    return err, commit


@dataclass
class HeliState:
    """What the rotorcraft controller remembers between ticks."""

    stage: str = "preflight"
    stage_entered_at: float = 0.0
    # What the flight is for.
    dest_lat: float = 0.0
    dest_lon: float = 0.0
    dest_elev_ft: float = 0.0
    dest_ident: str = ""
    cruise_alt_ft: float = 0.0
    pattern_alt_ft: float = 0.0
    ias_cruise_kt: float = 80.0
    ias_approach_kt: float = 60.0
    # The learned hover collective, per cent depressed.
    coll_trim: float = COLL_TRIM_PCT
    last_agl_ft: float = 0.0
    last_ias_kt: float = 0.0
    # One-shots.
    systems_sent: bool = False
    start_sent: bool = False
    gov_sent: bool = False
    rotor_brake_sent: bool = False
    shutdown_sent: bool = False
    # IFR was seen and downgraded, once.
    ifr_downgraded: bool = False
    # The shutdown cool-down clock.
    shutdown_at: float = 0.0


class RotorcraftFlight:
    """Flies the helicopter the fixed-wing controller cannot.

    Same lifecycle as :class:`AutonomousFlight`: ``begin_flight`` once per
    leg, ``tick`` from the situation loop. The owner's ``state.phase`` is
    kept on the coarse :class:`AutonomousPhase` for anything that watches.
    """

    def __init__(self, owner):
        self.owner = owner
        self.config = owner.config
        self.session = owner.session
        self.sim = owner.sim
        self.get_state = owner.get_state
        self.crew = owner.crew
        self.on_event = owner.on_event
        self.state = HeliState()
        self._profile = DEFAULT_HELI_PROFILE
        # The type's published engine/rotor limits (atc.powerplant):
        # class-pattern figures until begin() resolves the real type.
        self._limits = limits_for("", HELICOPTER)
        self._transitioned_owner = ""
        self._last_tick = 0.0
        # The closed-loop axes follow the same switch as the fixed-wing
        # ones: immersion.autonomous_pid off returns every channel to the
        # open-loop constants above.
        self._gains = None
        self._pids: dict = {}
        self._tq_warned = False
        self._nr_warned = ""
        # The situation loop runs faster than a pilot's hand can; the
        # controls are closed-loop, so a few passes a second is plenty and
        # every pass is a SimConnect round trip.
        self.TICK_S = 0.25

    @property
    def _use_pids(self) -> bool:
        """The owner's switch, read live."""
        return bool(getattr(self.owner, "_use_pids", True))

    def _pid(self, channel: str):
        """The loop on one axis, made on first use, gains the type's."""
        pids = getattr(self, "_pids", None)
        if pids is None:
            pids = self._pids = {}
        pid = pids.get(channel)
        if pid is None:
            if getattr(self, "_gains", None) is None:
                from .control_gains import rotor_gains
                code = getattr(getattr(self.session, "aircraft", None),
                               "type_code", "") or ""
                self._gains = rotor_gains(code)
            from .control import Pid
            pid = Pid(getattr(self._gains, channel))
            pids[channel] = pid
        return pid

    # -- plumbing ---------------------------------------------------------

    def _send(self, name: bytes, value: int) -> bool:
        return self.owner._send_event(name, value)

    def _emit(self, kind: str, text: str, key: str = "") -> None:
        self.owner._emit(kind, text, key=key)

    def _goto(self, stage: str) -> None:
        self.state.stage = stage
        self.state.stage_entered_at = time.monotonic()
        for pid in getattr(self, "_pids", {}).values():
            pid.reset()
        shown = _STAGE_PHASE.get(stage)
        if shown is not None and shown != self.owner.state.phase:
            self.owner._transition(shown)
        log.info("rotorcraft stage: %s", stage)

    def _checklist(self, name: str) -> None:
        done = self.owner.state.checklists_completed
        if name not in done:
            self.crew.checklists.start(name)
            done.add(name)

    def _elapsed(self) -> float:
        return time.monotonic() - self.state.stage_entered_at

    def handoff(self) -> None:
        """The owner's disengage: the controls to neutral.

        Cyclic and pedals centre; the collective and throttle are left
        where the last tick put them -- sending the collective to zero in
        the air would be a command, not a release.
        """
        try:
            self._send(CYCLIC_LONG_SET, 0)
            self._send(CYCLIC_LAT_SET, 0)
            self._send(PEDAL_SET, 0)
            self._send(TAIL_ROTOR_SET, 0)
            self._send(STEERING_SET, 0)
        except Exception:
            log.debug("rotorcraft handoff raised", exc_info=True)

    # -- entry ------------------------------------------------------------

    def begin(self, session) -> None:
        """A new leg (or a chained one): everything is re-planned."""
        self.session = session
        self.state = HeliState()
        # The type's own envelope where the table knows it -- an R44 and
        # an S-76 do not share a transition speed or a ceiling.
        self._profile = heli_profile_for(
            getattr(session.aircraft, "type_code", ""))
        # The published ceilings this leg's protections compare against:
        # torque and rotor RPM where the model reports them.
        self._limits = limits_for(
            getattr(session.aircraft, "type_code", ""), HELICOPTER)
        self._tq_warned = False
        self._nr_warned = ""
        dest = session.destination
        if dest is not None:
            self.state.dest_lat = dest.lat
            self.state.dest_lon = dest.lon
            self.state.dest_elev_ft = dest.elev_ft
            self.state.dest_ident = dest.ident
        cruise = session.cruise_altitude_ft or 3000.0
        self.state.cruise_alt_ft = min(cruise, self._profile.ceiling_ft)
        self.state.pattern_alt_ft = min(
            max(self.state.dest_elev_ft + PATTERN_AGL_FT, 1500.0),
            self.state.cruise_alt_ft)
        speeds = speeds_for(getattr(session.aircraft, "type_code", ""),
                            HELICOPTER)
        # The model's or the table's approach speed first, the type's own
        # envelope for the cruise it does not declare.
        self.state.ias_approach_kt = (
            float(speeds.vapp or 0.0) or self._profile.ias_approach_kt)
        self.state.ias_cruise_kt = self._profile.ias_cruise_kt
        self._goto("preflight")

    # -- the loop ---------------------------------------------------------

    def tick(self, state) -> None:
        """One pass of the flight, at whatever rate the situation loop runs."""
        if state is None or not getattr(state, "connected", False):
            return
        # The owner gave up (a timeout, an exception): hands off, whatever
        # stage this was in.
        if self.owner.state.phase == AutonomousPhase.ERROR:
            return
        now = time.monotonic()
        if now - self._last_tick < self.TICK_S:
            return
        self._last_tick = now
        # A chained leg: the owner's chain phase has loaded the next flight
        # plan and put the session back to PREFLIGHT; this controller
        # re-plans for it rather than starting a new object.
        if self.state.stage == "done":
            phase = self.owner.state.phase
            if phase == AutonomousPhase.PREFLIGHT:
                self.begin(self.session)
                return
            if phase == AutonomousPhase.CHAIN_NEXT:
                # Drive the owner's chain step here: the fixed-wing phase
                # loop is bypassed for a helicopter.
                self.owner._phase_chain_next(state, time.monotonic())
                phase = self.owner.state.phase
                if phase == AutonomousPhase.PREFLIGHT:
                    self.begin(self.session)
                elif phase in (AutonomousPhase.SHUTDOWN,
                               AutonomousPhase.COMPLETE):
                    # The chain found nothing more: the flight is over.
                    self._finish()
                return
            if phase in (AutonomousPhase.COMPLETE, AutonomousPhase.ERROR):
                return
            # SHUTDOWN (or anything else): the leg ended; chain or finish.
            self._done(state)
            return
        stage = self.state.stage
        # An engine that stops in flight is an autorotation, whatever the
        # stage was: the rotor's stored energy is spent getting down under
        # control, not holding the plan.
        if stage not in ("preflight", "start", "shutdown", "done",
                         "autorotate") and self._engine_lost(state):
            if not self.owner.state.emergency_declared:
                self.owner._declare_emergency("engine", state, mayday=True)
            self._goto("autorotate")
            stage = "autorotate"
        try:
            if stage == "autorotate":
                self._autorotate(state)
            elif stage == "preflight":
                self._preflight(state)
            elif stage == "start":
                self._start(state)
            elif stage == "hover":
                self._hover(state)
            elif stage == "departure":
                self._departure(state)
            elif stage == "climb":
                self._climb(state)
            elif stage == "cruise":
                self._cruise(state)
            elif stage == "descent":
                self._descent(state)
            elif stage == "approach":
                self._approach(state)
            elif stage == "landing":
                self._landing(state)
            elif stage == "shutdown":
                self._shutdown(state)
            elif stage == "done":
                self._done(state)
        except Exception as exc:                       # pragma: no cover
            log.exception("rotorcraft stage %s failed: %s", stage, exc)
            self.owner._set_error(f"rotorcraft {stage}: {exc}")

    # -- preflight and start ----------------------------------------------

    def _preflight(self, state) -> None:
        if self.session.ifr and not self.state.ifr_downgraded:
            # Explicit, not silent: the flight is flown VFR because that is
            # all a rotorcraft gets here, and the log says why.
            self.state.ifr_downgraded = True
            self.session.ifr = False
            self._emit("info",
                       "Autonomous rotorcraft is VFR only; the flight is "
                       "flown VFR", "autonomous.rotorcraft.vfr_only")
        if self.session.destination is None:
            self.owner._set_error("no destination for the rotorcraft leg")
            return
        self._checklist("before_start")
        self._goto("start")

    def _start(self, state) -> None:
        """Best-effort engine start: the generic events, then watch."""
        if not self.state.systems_sent:
            self.state.systems_sent = True
            if not getattr(state, "battery_master", False):
                self._send(b"MASTER_BATTERY_SET", 1)
            if not getattr(state, "avionics_master", False):
                self._send(b"AVIONICS_MASTER_SET", 1)
        if not self.state.start_sent:
            self.state.start_sent = True
            # The governor up for the start where the model has one; the
            # clutch is the model's own decision on most turbines, so it is
            # only touched if nothing is running after a while.
            self._send(ROTOR_GOV_SET, 1)
            self.state.gov_sent = True
            self._send(ENGINE_START, 1)
            self._emit("info", "Engine start requested (best effort)",
                       "autonomous.rotorcraft.start")
        running = bool(getattr(state, "engines_running", False))
        rpm = float(getattr(state, "rotor_rpm_pct", 0.0) or 0.0)
        # Rotor RPM is published by some models and not others: where it is
        # 0 all flight, engines_running and the timeout carry the phase.
        rotor_ok = rpm >= ROTOR_MIN_PCT or (rpm == 0.0 and running)
        if running and rotor_ok:
            self._checklist("after_start")
            self._goto("hover")
            return
        if self._elapsed() > START_TIMEOUT_S:
            self.owner._set_error(
                "rotorcraft engine start timed out; the model may need its "
                "own start event")

    # -- the flight --------------------------------------------------------

    def _hover(self, state) -> None:
        """The start checks: hold the spot until the rotor is flight RPM
        and the clock says the pilot has looked."""
        self._hold_hover(state)
        rpm = float(getattr(state, "rotor_rpm_pct", 0.0) or 0.0)
        rotor_ok = rpm >= ROTOR_MIN_PCT or (rpm == 0.0 and self._elapsed() > 30.0)
        if rotor_ok and self._elapsed() > 8.0:
            self._checklist("before_takeoff")
            self._goto("departure")
        elif self._elapsed() > 120.0:
            # A model that never reports rotor RPM still goes; the spool is
            # judged by the engine and the clock, not by a missing variable.
            self._checklist("before_takeoff")
            self._goto("departure")

    def _departure(self, state) -> None:
        """Transition through the H-V curve: accelerate at skid height in
        ground effect to the transition speed, *then* climb away.

        The old version climbed at the profile rate the whole way, which
        is the profile for a helicopter that is already flying -- a
        departing one accelerates level first. Where the type publishes
        a takeoff-power airspeed ceiling (the H125's 40 kt), the accel
        stays under it."""
        agl = float(getattr(state, "altitude_agl_ft", 0.0) or 0.0)
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        prof = self._profile
        accel_to = min(prof.transition_kt,
                       self._limits.top_max_ias_kt or prof.transition_kt)
        # The gate to the climb is the accel target itself, not the
        # transition figure: a type whose takeoff-power ceiling sits
        # under the transition speed (the H125's 40 kt) accelerates to
        # its own number and climbs on what it has -- gating on the
        # transition speed would hold it level for ever.
        if ias < accel_to - 2.0:
            self._fly(state, agl_target=HOVER_AGL_FT,
                      ias_target=accel_to,
                      vs_target=0.0, throttle=THROTTLE_CLIMB)
        else:
            self._fly(state, agl_target=HOVER_AGL_FT + 20.0,
                      ias_target=prof.transition_kt,
                      vs_target=prof.climb_vs_fpm, throttle=THROTTLE_CLIMB)
        if agl > 100.0 and ias >= prof.transition_kt:
            self._checklist("after_takeoff")
            self._goto("climb")
        elif self._elapsed() > 90.0:
            # Never rotated: something the controller cannot see. Say so
            # rather than charge into the climb.
            self.owner._set_error("rotorcraft transition timed out")

    def _climb(self, state) -> None:
        self._fly(state, agl_target=None,
                  alt_target=self.state.cruise_alt_ft,
                  ias_target=self.state.ias_cruise_kt,
                  vs_target=self._profile.climb_vs_fpm,
                  throttle=THROTTLE_CLIMB)
        if (state.altitude_ft or 0.0) >= self.state.cruise_alt_ft - 150.0 \
                and abs(getattr(state, "vertical_speed_fpm", 0.0) or 0.0) < 200:
            self._goto("cruise")
        elif self._elapsed() > 900.0:
            self.owner._set_error("rotorcraft climb timed out")

    def _cruise(self, state) -> None:
        self._checklist("cruise")
        dist = self._distance_nm(state)
        if dist is not None and dist < 5.0:
            self._checklist("descent")
            self._goto("descent")
            return
        self._fly(state, agl_target=None,
                  alt_target=self.session.assigned_altitude_ft
                  or self.state.cruise_alt_ft,
                  ias_target=self.session.assigned_speed_kt
                  or self.state.ias_cruise_kt,
                  vs_target=0.0, throttle=THROTTLE_CRUISE)

    def _descent(self, state) -> None:
        agl = float(getattr(state, "altitude_agl_ft", 0.0) or 0.0)
        self._fly(state, agl_target=None,
                  alt_target=self.state.pattern_alt_ft,
                  ias_target=self.state.ias_approach_kt,
                  vs_target=-self._profile.climb_vs_fpm,
                  throttle=THROTTLE_CRUISE)
        dist = self._distance_nm(state)
        if agl <= PATTERN_AGL_FT and (dist is None or dist < 3.0):
            self._checklist("approach")
            self._goto("approach")
        elif self._elapsed() > 900.0:
            self.owner._set_error("rotorcraft descent timed out")

    def _approach(self, state) -> None:
        agl = float(getattr(state, "altitude_agl_ft", 0.0) or 0.0)
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        # Down the profile's approach: slow to approach speed first, then
        # descend into the hover over the spot.
        if ias > self.state.ias_approach_kt + 5.0 or agl > FINAL_AGL_FT + 100:
            self._fly(state, agl_target=FINAL_AGL_FT + 100.0,
                      ias_target=self.state.ias_approach_kt,
                      vs_target=-400.0, throttle=THROTTLE_CRUISE)
        else:
            # The walk-down: the speed bleeds off with the height --
            # the approach figure at the top of the descent, a fast
            # walk over the spot -- rather than a chop to the flare
            # speed at one altitude.
            top = FINAL_AGL_FT + 100.0
            span = top - 50.0
            frac = _clamp((agl - 50.0) / span, 0.0, 1.0) if span > 0 \
                else 0.0
            ias_t = 20.0 + frac * max(
                0.0, self.state.ias_approach_kt - 20.0)
            self._fly(state, agl_target=0.0,
                      ias_target=max(ias_t, 15.0),
                      vs_target=self._profile.landing_vs_fpm,
                      throttle=THROTTLE_CRUISE)
        if agl <= FLARE_AGL_FT + 20.0:
            self._checklist("before_landing")
            self._goto("landing")

    def _landing(self, state) -> None:
        agl = float(getattr(state, "altitude_agl_ft", 0.0) or 0.0)
        if getattr(state, "on_ground", False):
            # Touchdown: collective down, cyclic and pedals centred, and the
            # aircraft's own brakes where it has them.
            self._collective(5.0)
            self._send(CYCLIC_LONG_SET, CYCLIC_TRIM_HOVER)
            self._send(CYCLIC_LAT_SET, 0)
            self._send(PEDAL_SET, 0)
            self._send(TAIL_ROTOR_SET, 0)
            self._send(STEERING_SET, 0)
            self._send(b"PARKING_BRAKE_SET", 1)
            self._checklist("after_landing")
            self._goto("shutdown")
            return
        # The flare: the rate comes off under FLARE_AGL_FT so the skids
        # meet the ground rather than the airframe meeting it fast.
        vs_target = (self._profile.landing_vs_fpm if agl > FLARE_AGL_FT
                     else -120.0)
        self._fly(state, agl_target=0.0,
                  ias_target=max(self.state.ias_approach_kt - 15.0, 35.0),
                  vs_target=vs_target, throttle=THROTTLE_CRUISE)
        if self._elapsed() > 300.0:
            self.owner._set_error("rotorcraft landing timed out")

    # -- the failure ---------------------------------------------------------

    def _demeanor(self, state) -> str:
        """How hard the axes may travel: smooth, gentle or urgent.

        The precision stages -- the hover, the approach, the landing --
        spend the last of the cyclic carefully. An autorotation, an
        engine that has stopped, or an emergency the aeroplane's FO has
        declared gets everything the clamps allow.
        """
        if self.state.stage == "autorotate" or self._engine_lost(state) \
                or getattr(self.owner.state, "emergency", ""):
            return "urgent"
        if self.state.stage in ("hover", "approach", "landing"):
            return "gentle"
        return "smooth"

    def _engine_lost(self, state) -> bool:
        """Whether every engine the sim reports has stopped, airborne."""
        if getattr(state, "on_ground", True):
            return False
        out = self.owner._engines_out(state)
        count = int(getattr(state, "engine_count", 0) or 0) or 1
        return 0 < count <= out

    def _autorotate(self, state) -> None:
        """The engine-out landing: the rotor's energy spent to touch down.

        A real autorotation is three moves -- the collective goes down at
        once to keep the rotor turning, the cyclic holds the speed the
        rotor likes (near the type's approach figure), and at the bottom
        the flare trades speed for rotor and the collective comes up to
        spend what is left against the last few feet. That is flown here:
        low collective and approach speed down to FLARE_AGL_FT, the flare
        and the pull under it, then the ground.
        """
        agl = float(getattr(state, "altitude_agl_ft", 0.0) or 0.0)
        if getattr(state, "on_ground", False):
            # Down: collective all the way, the rest centred, the leg over.
            self._collective(0.0)
            self._send(CYCLIC_LONG_SET, 0)
            self._send(CYCLIC_LAT_SET, 0)
            self._send(PEDAL_SET, 0)
            self._send(TAIL_ROTOR_SET, 0)
            self._checklist("after_landing")
            self._goto("shutdown")
            return
        self._throttle(0)
        flare_agl = self._profile.auto_flare_ft or FLARE_AGL_FT
        if agl > flare_agl:
            # Preserving the rotor: near-bottom collective, the type's
            # autorotation speed on the cyclic, pedals holding the track
            # to the spot.
            coll = 10.0
            if self._use_pids:
                # The rotor is the engine now: where the model reports
                # Nr, a governor trades collective against it -- droop
                # unloads the blades, a windmilling rotor takes pitch.
                # The target sits just above the type's power-off floor.
                rpm = float(getattr(state, "rotor_rpm_pct", 0.0) or 0.0)
                if rpm > 0.0:
                    nr_want = max(90.0,
                                  (self._limits.nr_auto_min_pct or 90.0)
                                  + 5.0)
                    coll += self._pid("rotor").update(rpm - nr_want)
            self._collective(coll)
            ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
            auto_kt = self._profile.auto_kt or self.state.ias_approach_kt
            err = auto_kt - ias
            if self._use_pids:
                axis = CYCLIC_TRIM_AIRBORNE + self._pid("cyclic") \
                    .update(err, measurement=ias)
            else:
                axis = CYCLIC_TRIM_AIRBORNE + CYCLIC_KT_PER_AXIS * err
            self._send(CYCLIC_LONG_SET,
                       int(_clamp(axis, -CYCLIC_MAX, CYCLIC_MAX)))
            self._send(CYCLIC_LAT_SET, self._lateral_trim_axis(state))
            want_hdg = self._desired_track(state)
            if want_hdg:
                err_h = _heading_error(
                    want_hdg, float(getattr(state, "heading_true", 0.0)
                                    or 0.0))
                err_h, self._turn_commit = _committed_error(
                    err_h, getattr(self, "_turn_commit", 0))
                demand = int(_clamp(-PEDAL_DEG_PER_AXIS * err_h,
                                    -PEDAL_MAX, PEDAL_MAX))
                _pedal_check(self, demand, float(
                    getattr(state, "heading_true", 0.0) or 0.0))
                pedal = demand * _pedal_dir(self)
                if not self._send(PEDAL_SET, pedal):
                    self._send(TAIL_ROTOR_SET, pedal)
        elif agl > 10.0:
            # The flare: aft cyclic trades speed for rotor, and the
            # collective starts up to cushion what is left of the descent.
            self._send(CYCLIC_LONG_SET, -CYCLIC_TRIM_AIRBORNE)
            self._collective(45.0)
        else:
            # The pull: everything the rotor has left, against the last
            # feet -- then level the skids for the touchdown.
            self._collective(85.0)
            self._send(CYCLIC_LONG_SET, 0)
            self._send(CYCLIC_LAT_SET, 0)
        if self._elapsed() > 300.0:
            self.owner._set_error("rotorcraft autorotation timed out")

    # -- shutdown -----------------------------------------------------------

    def _shutdown(self, state) -> None:
        """Cool-down, rotor brake where fitted, then the engine events."""
        rpm = float(getattr(state, "rotor_rpm_pct", 0.0) or 0.0)
        running = bool(getattr(state, "engines_running", False))
        if not self.state.shutdown_at:
            self.state.shutdown_at = time.monotonic()
            # The cool-down only means anything at idle: the last _fly
            # pass left the throttle wherever the flight put it, so it
            # is brought back before the clock starts counting -- the
            # ITT figure below cannot be reached at cruise throttle.
            self._throttle(0)
        elapsed = time.monotonic() - self.state.shutdown_at
        if running and not self.state.rotor_brake_sent \
                and getattr(state, "rotor_brake_available", False) \
                and (rpm == 0.0 or rpm < ROTOR_BRAKE_BELOW_PCT):
            # ROTOR_BRAKE is a stepper; the explicit ON goes first and the
            # stepper is the fallback for a model that maps neither.
            if not self._send(ROTOR_BRAKE_ON, 0):
                self._send(ROTOR_BRAKE, 1)
            self.state.rotor_brake_sent = True
        itt = float(getattr(state, "eng_itt_c", 0.0) or 0.0)
        idle_itt = self._limits.itt_idle_c
        cooled = (elapsed >= SHUTDOWN_COOLDOWN_S or (rpm and rpm < 30.0)
                  or (idle_itt and 0.0 < itt <= idle_itt))
        if running and cooled and not self.state.shutdown_sent:
            self.state.shutdown_sent = True
            self._send(ENGINE_STOP, 1)
            self._send(b"AVIONICS_MASTER_SET", 0)
            self._send(b"MASTER_BATTERY_SET", 0)
            self._emit("info", "Engine shutdown requested (best effort)",
                       "autonomous.rotorcraft.shutdown")
        if (not running and self.state.shutdown_sent) \
                or (self.state.shutdown_sent and elapsed > 60.0) \
                or elapsed > 300.0:
            self._goto("done")

    def _done(self, state) -> None:
        """The leg is over: chain the next or finish, as the owner does."""
        if self.owner._chained:
            if self.owner.state.phase != AutonomousPhase.CHAIN_NEXT:
                self.owner._transition(AutonomousPhase.CHAIN_NEXT)
            return
        self._finish()

    def _finish(self) -> None:
        """Emit the completion once and park the owner in COMPLETE."""
        if self.owner.state.phase != AutonomousPhase.COMPLETE:
            self._emit("info", "Autonomous rotorcraft flight completed",
                       "autonomous.completed")
            self.owner._transition(AutonomousPhase.COMPLETE)

    # -- the controls -------------------------------------------------------

    def _fly(self, state, *, agl_target=None, alt_target=None,
             ias_target=None, vs_target=0.0, throttle=None) -> None:
        """One pass of the three axes, in the order a pilot thinks of them:
        collective for height, cyclic for speed, pedals for heading."""
        agl = float(getattr(state, "altitude_agl_ft", 0.0) or 0.0)
        alt = float(getattr(state, "altitude_ft", 0.0) or 0.0)
        vs = float(getattr(state, "vertical_speed_fpm", 0.0) or 0.0)
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        hdg = float(getattr(state, "heading_true", 0.0) or 0.0)
        airborne = not bool(getattr(state, "on_ground", True))
        # How hard the axes may travel this pass: the precision stages
        # spend the last of the cyclic carefully; the urgent ones get
        # everything the clamps allow.
        axis_max = int(CYCLIC_MAX
                       * (0.7 if self._demeanor(state) == "gentle" else 1.0))
        pedal_max = int(PEDAL_MAX * (axis_max / CYCLIC_MAX))

        # -- collective: height, or rate of climb --------------------------
        if agl_target is not None:
            err = agl_target - agl
            if self._use_pids:
                # Hover/landing: feet above the ground, damped towards the
                # wanted vertical speed rather than towards none.
                coll = self.state.coll_trim + self._pid("collective") \
                    .update(err, rate=(vs_target or 0.0) - vs)
            else:
                coll = self.state.coll_trim + COLL_KP_PER_FT * err \
                    + COLL_KD_PER_FPM * ((vs_target or 0.0) - vs)
            # Learn the trim only in a steady hover, so a climb does not
            # teach the collective the wrong number.
            if airborne and abs(vs) < 150.0 and abs(err) < 40.0:
                self.state.coll_trim += COLL_ADAPT * (
                    float(getattr(state, "collective_position", 0.0) or 0.0)
                    - self.state.coll_trim) * 0.1
        elif alt_target is not None:
            # Cruise/climb/descent: feet to the level, rate of climb damped.
            # The caller's target vertical speed bounds the ask where it
            # gives one -- a climb at eight hundred feet a minute is not
            # the envelope's twelve hundred.
            err = alt_target - alt
            limit = (min(self._profile.max_vs_fpm, abs(vs_target))
                     if vs_target else self._profile.max_vs_fpm)
            want_vs = _clamp(err * 2.0, -limit, limit)
            if self._use_pids:
                coll = self.state.coll_trim + self._pid("collective") \
                    .update(err * 0.1, rate=(want_vs - vs) * 0.1)
            else:
                coll = self.state.coll_trim + COLL_KP_PER_FT * err / 10.0 \
                    + COLL_KD_PER_FPM * (want_vs - vs) / 10.0
        else:
            coll = self.state.coll_trim
        self._collective(coll)

        # -- cyclic longitudinal: airspeed ----------------------------------
        if ias_target is not None and airborne:
            # Nothing is chased past the type's never-exceed: the target
            # itself is held under it, not merely the overspeed pulled
            # back from.
            ias_target = min(float(ias_target),
                             self._profile.max_ias_kt * 0.95)
            err = ias_target - ias
            if self._use_pids:
                axis = CYCLIC_TRIM_AIRBORNE + self._pid("cyclic") \
                    .update(err, measurement=ias)
            else:
                axis = CYCLIC_TRIM_AIRBORNE + CYCLIC_KT_PER_AXIS * err
            # A helicopter cannot climb vertically forever: bleed speed
            # protection -- if the collective is asking hard and the speed
            # is low, lower the nose rather than stall in the H-V curve.
            if ias < 45.0 and agl > 200.0:
                axis -= 1500
            self._send(CYCLIC_LONG_SET, int(_clamp(axis, -axis_max,
                                                 axis_max)))
        elif not airborne:
            self._send(CYCLIC_LONG_SET, CYCLIC_TRIM_HOVER)
        # Cyclic lateral: the model's trim, plus a drift correction the
        # way a pilot adds a touch of into-wind cyclic -- track right of
        # the nose is wind from the left, so the answer is left bank.
        # Only forward flight: below ~15 kt a track is not a thing yet.
        # UNVERIFIED sign, like every cyclic axis.
        lat = self._lateral_trim_axis(state)
        if airborne and ias >= 15.0:
            drift = _heading_error(
                float(getattr(state, "heading_true", 0.0) or 0.0),
                float(getattr(state, "track_true", 0.0) or 0.0))
            lat += int(_clamp(drift * 150.0, -2000.0, 2000.0))
        self._send(CYCLIC_LAT_SET, int(_clamp(lat, -axis_max, axis_max)))

        # -- pedals: heading -------------------------------------------------
        want_hdg = self._desired_track(state)
        if want_hdg:
            err = _heading_error(want_hdg, hdg)
            err, self._turn_commit = _committed_error(
                err, getattr(self, "_turn_commit", 0))
            if self._use_pids:
                demand = int(_clamp(-self._pid("pedals").update(err),
                                    -pedal_max, pedal_max))
            else:
                demand = int(_clamp(-PEDAL_DEG_PER_AXIS * err, -pedal_max,
                                    pedal_max))
            _pedal_check(self, demand, hdg)
            pedal = demand * _pedal_dir(self)
            if not self._send(PEDAL_SET, pedal):
                self._send(TAIL_ROTOR_SET, pedal)
            self._send(STEERING_SET, pedal)

        # -- throttle ---------------------------------------------------------
        if throttle is not None:
            self._throttle(throttle)

        # -- envelope -----------------------------------------------------------
        self._protect(state, vs, ias, agl)
        self.state.last_agl_ft = agl
        self.state.last_ias_kt = ias

    def _collective(self, pct: float) -> None:
        pct = _clamp(pct, 0.0, 100.0)
        # The collective axis is signed like a joystick's: -16383 is the
        # floor. A bare 0..16384 put zero collective at mid-stick.
        self._send(COLLECTIVE_SET, int(-16383.0 + pct / 100.0 * 32766.0))

    def _throttle(self, value: int) -> None:
        """The helicopter throttle events, best effort: the axis where the
        model has it, the named events where it does not. The axis is
        signed -- a unipolar value leaves the lowest throttle at half."""
        axis = int(-16383.0
                   + _clamp(value, 0, AXIS_MAX) / AXIS_MAX * 32766.0)
        if not self._send(HELI_THROTTLE_SET, axis):
            self._send(THROTTLE_ALL_SET, value)
        if value >= AXIS_MAX * 0.9:
            self._send(THROTTLE_ALL_FULL, 0)

    def _protect(self, state, vs: float, ias: float, agl: float) -> None:
        """The envelope: never more than the type's max VS either way,
        never above its max IAS, never a big collective at skid height --
        and never past the torque the engine is rated for, where the
        model reports it."""
        max_vs = self._profile.max_vs_fpm
        autorotating = self.state.stage == "autorotate"
        if vs > max_vs:
            self._collective(max(0.0, self.state.coll_trim - 10.0))
        elif vs < -max_vs and not autorotating:
            # A sink past the envelope is met with collective -- except in
            # the autorotation, where the steep descent is the design and
            # the rotor's stored speed is spent at the flare, not here.
            self._collective(min(100.0, self.state.coll_trim + 15.0))
        if ias > self._profile.max_ias_kt and not autorotating:
            self._send(CYCLIC_LONG_SET, CYCLIC_MAX)   # nose down, UNVERIFIED sign
        torque = float(getattr(state, "eng_torque_pct", 0.0) or 0.0)
        # The variable is documented as a percent but some models publish
        # torque in foot-pounds on the same scale; a figure far past the
        # red line is not a percent, and limiting on it would hold the
        # collective down all flight. The line itself is the type's own:
        # the published takeoff figure (atc.powerplant), not a generic
        # 95 -- an EC135's 75% is a different red line from an S-76's.
        tq_red = self._limits.tq_takeoff_pct or 100.0
        if tq_red * 0.97 <= torque <= tq_red * 1.30:
            # At red line the collective comes off, whatever else wants.
            self._collective(max(0.0, self.state.coll_trim - 8.0))
            if not self._tq_warned:
                self._tq_warned = True
                self._emit("info",
                           f"Torque {torque:.0f}% at the limit "
                           f"({tq_red:.0f}%): easing the collective",
                           "autonomous.rotorcraft.torque")
        elif torque < tq_red * 0.90:
            self._tq_warned = False

        # Rotor speed, where the model publishes it: droop unloads the
        # blades so the governor recovers (collective down), an overspeed
        # loads the rotor to bring it back (collective up). The floors
        # are the type's power-on figures; in an autorotation the
        # rotor-PID above owns this job and the watch stands down.
        rpm = float(getattr(state, "rotor_rpm_pct", 0.0) or 0.0)
        nr_min = self._limits.nr_min_pct or 90.0
        nr_max = self._limits.nr_max_pct or 104.0
        if self.state.stage != "autorotate" and 40.0 < rpm < nr_min:
            self._collective(max(0.0, self.state.coll_trim - 12.0))
            if self._nr_warned != "droop":
                self._nr_warned = "droop"
                self._emit("info",
                           f"Rotor droop ({rpm:.0f}% < {nr_min:.0f}%): "
                           "collective down to recover",
                           "autonomous.rotorcraft.droop")
        elif self.state.stage != "autorotate" and rpm > nr_max:
            self._collective(min(100.0, self.state.coll_trim + 10.0))
            if self._nr_warned != "overspeed":
                self._nr_warned = "overspeed"
                self._emit("info",
                           f"Rotor overspeed ({rpm:.0f}% > "
                           f"{nr_max:.0f}%): loading the rotor",
                           "autonomous.rotorcraft.overspeed")
        elif nr_min <= rpm <= nr_max:
            self._nr_warned = ""
        if agl < 15.0 and not getattr(state, "on_ground", True):
            # Effectively landed but not flagged: kill the climb input.
            self._collective(min(self.state.coll_trim, 30.0))

        # The rotor's load. High G is offloaded on the collective the
        # way the torque is; low G -- the mast-bump edge on a teetering
        # rotor -- is reloaded with a gentle aft cyclic, which is the
        # recovery the Robinson POH teaches (UNVERIFIED sign, like the
        # cyclic axis it writes). An autorotation wants the rotor
        # unloaded on purpose, so the watch stands down for it.
        g = float(getattr(state, "g_force", 1.0) or 0.0)
        if getattr(self.owner, "_envelope_protection", True) \
                and self.state.stage != "autorotate" \
                and not getattr(state, "on_ground", True) \
                and "g_force" not in getattr(state, "unavailable", ()):
            if g > self._profile.pos_g * 0.85:
                self._collective(max(0.0, self.state.coll_trim - 8.0))
                if not getattr(self, "_g_warned", False):
                    self._g_warned = True
                    self._emit("info",
                               f"High rotor load ({g:.1f} G): easing the collective",
                               "autonomous.rotorcraft.g_limit")
            if g < self._profile.low_g:
                self._send(CYCLIC_LONG_SET, int(_clamp(
                    CYCLIC_TRIM_HOVER + 2000.0, -CYCLIC_MAX, CYCLIC_MAX)))
                if not getattr(self, "_low_g_warned", False):
                    self._low_g_warned = True
                    self._emit("info",
                               f"Low G ({g:.1f}): gentle aft cyclic",
                               "autonomous.rotorcraft.low_g")
            if g < self._profile.pos_g * 0.7 \
                    and g > self._profile.low_g + 0.1:
                self._g_warned = False
                self._low_g_warned = False

    def _lateral_trim_axis(self, state=None) -> int:
        """The model's own lateral cyclic trim as an axis value.

        ROTOR LATERAL TRIM PCT is the trim the model carries; 50 is
        centred, and the departure from it is the bias a wings-level
        hover needs. UNVERIFIED: the sign and the scale are the
        simulator's, and a wrong one shows as a slow roll, not a departure.
        """
        if state is None:
            state = self.get_state()
        trim = float(getattr(state, "rotor_lateral_trim_pct", 0.0)
                     or 0.0) if state is not None else 0.0
        # Only a value that could be a percent is trusted; anything else
        # is a unit the conversion did not expect.
        if not 0.0 < trim < 100.0:
            return 0
        return int(_clamp((trim - 50.0) * 163.84, -3000, 3000))

    # -- navigation ---------------------------------------------------------

    def _distance_nm(self, state) -> float | None:
        if not getattr(state, "has_position", False) \
                or not self.state.dest_lat:
            return None
        return state.distance_to(self.state.dest_lat, self.state.dest_lon)

    def _desired_track(self, state) -> float:
        """Where to point, in degrees true: the controller's heading, else
        the destination. An assigned heading is spoken magnetic, and the
        pedal loop steers ``heading_true`` -- without the conversion the
        pedals hold a heading off by the local variation."""
        assigned = getattr(self.session, "assigned_heading", None)
        if assigned:
            if getattr(state, "has_position", False):
                from ..navdata.magvar import to_true, variation
                return to_true(float(assigned),
                               variation(state.latitude, state.longitude))
            return float(assigned)
        # The live destination, not the one begin() planned: an emergency
        # divert repoints the whole flight, and the pedals should go the
        # same way the declaration said.
        dest = getattr(self.session, "destination", None)
        lat = getattr(dest, "lat", 0.0) or self.state.dest_lat
        lon = getattr(dest, "lon", 0.0) or self.state.dest_lon
        if not getattr(state, "has_position", False) or not lat:
            return float(getattr(state, "heading_true", 0.0) or 0.0)
        return state.bearing_to(lat, lon)

    def _hold_hover(self, state) -> None:
        """Feet on the ground, rotor turning: centred controls."""
        self._collective(self.state.coll_trim)
        self._send(CYCLIC_LONG_SET, CYCLIC_TRIM_HOVER)
        self._send(CYCLIC_LAT_SET, self._lateral_trim_axis(state))
        self._send(STEERING_SET, 0)


__all__ = ["RotorcraftFlight", "HELI_THROTTLE_SET", "STAGES"]
