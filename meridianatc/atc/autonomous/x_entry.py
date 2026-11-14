from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    AutonomousState,
    FlightSession,
    Phase,
    autofly_limit,
    log,
    time,
)


class _Entry:
    # -- entry -------------------------------------------------------------

    def _detect_helicopter(self, session: FlightSession) -> bool:
        """Whether the aeroplane in the sim is a helicopter.

        The simulator's own classification first (CATEGORY / ENGINE TYPE /
        the Doc 8643 designator); the pilot's helicopter profile as the
        fallback for a session begun before the state has answered.
        """
        from .. import aircraft_class as ac

        try:
            state = self.get_state()
        except Exception:
            state = None
        if state is not None and getattr(state, "connected", False):
            found = ac.of_state(state)
            self._aircraft_class = found.name
            return found.name == ac.HELICOPTER
        if getattr(session.profile, "kind", "") == "helicopter":
            self._aircraft_class = ac.HELICOPTER
            return True
        found = ac.classify(
            getattr(session.aircraft, "type_code", ""),
            airline=getattr(session.profile, "airliner", False)
        )
        self._aircraft_class = found.name
        return found.name == ac.HELICOPTER

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        self._enabled = True
        self.config.immersion.autonomous_flight = True
        # A re-engage must not resume a dead phase: finished, failed or
        # never started is joined where the aeroplane actually is, the
        # same way ``engage`` enters mid-flight.
        if self.session is not None and self.state.phase in (
                AutonomousPhase.INIT, AutonomousPhase.COMPLETE,
                AutonomousPhase.ERROR, AutonomousPhase.SHUTDOWN):
            # PARKED and CHAIN_NEXT are live phases, not dead ones:
            # PARKED is mid-turnaround and CHAIN_NEXT holds the legs
            # still to fly -- re-beginning either replaces the state
            # wholesale and the chain's bookkeeping goes with it.
            self.begin_flight(self.session)

    def disable(self) -> None:
        """Stop the FO's hands, then hand the aeroplane back.

        The flag goes first so no tick runs after the handoff has centred
        the controls; a FO that lets go leaving the elevator deflected is
        how a disengage becomes an upset.
        """
        self._enabled = False
        self.config.immersion.autonomous_flight = False
        self._handoff()

    def engage(self, session: FlightSession | None = None) -> bool:
        """The panel's engage, live: start the machine fresh wherever the
        aeroplane already is. False when there is no flight to fly yet --
        the next ``begin_flight`` picks the flag up instead."""
        self.enable()
        session = session or self.session
        if session is None:
            return False
        self.begin_flight(session)
        return True

    def disengage(self) -> None:
        """The panel's stop: same as ``disable`` -- off, controls handed
        back, config flag cleared so a restart does not fly itself."""
        self.disable()

    def _handoff(self) -> None:
        """Leave the aeroplane controllable the instant the FO lets go:
        surfaces centred, and every mode *this machine* engaged back off.

        The surfaces centre unconditionally -- a deflected elevator left
        from a flare is the worst souvenir there is. The AP and AT come
        off only where the FO turned them on: an autopilot the pilot
        engaged before the FO did is the pilot's, not this machine's to
        switch.
        """
        heli = self._heli
        if heli is not None and hasattr(heli, "handoff"):
            try:
                heli.handoff()
            except Exception:
                log.debug("rotorcraft handoff raised", exc_info=True)
        d = self._dir
        if d is None:
            return
        try:
            d.elevator(0.0)
            d.aileron(0.0)
            d.rudder(0.0)
            d.steering(0.0)
            d.center_controls()
            if self.state.ap_master:
                d.autopilot(False)
            if self.state.ap_apr_mode:
                d.approach(False)
                d.localizer(False)
            if self.state.ap_vs_mode:
                d.vs_hold(False)
            if self.state.ap_alt_mode:
                d.alt_hold(False)
            if self.state.ap_speed_mode:
                d.airspeed_hold(False)
                d.autothrottle_disconnect()
            if self._ap_mode == "managed" and self.state.ap_master:
                d.level_change(False)
            state = self._last_state
            if state is None or getattr(state, "on_ground", True):
                d.throttle(0.0)
                d.throttle_idle()
                d.brakes(0.0)
            d.reset()
        except Exception:
            log.debug("autonomous handoff raised", exc_info=True)
        self._last_lever = 0.0
        self._pedal_flip = False
        self._pedal_votes = 0
        self._pedal_demand = 0.0
        self._turn_commit = 0
        self._turn_rate_dps = 0.0
        self._last_hdg = None
        self.state.ap_master = False
        self.state.ap_hdg_mode = False
        self.state.ap_nav_mode = False
        self.state.ap_alt_mode = False
        self.state.ap_vs_mode = False
        self.state.ap_speed_mode = False
        self.state.ap_apr_mode = False

    def status(self) -> dict:
        """What the panel shows about the FO: on or off, and if on, what
        the machine is doing."""
        return {
            "available": self.sim is not None,
            "enabled": self._enabled,
            "helicopter": self._is_helicopter,
            "phase": self.state.phase.value,
            "sub": self.state.sub or "",
            "error": (self.state.error_message
                      if self.state.phase == AutonomousPhase.ERROR else ""),
        }

    def _entry_phase(self, session: FlightSession):
        """Where an engage joins the flight, by where the aeroplane is.

        A ground start walks the whole machine from preflight; an engage
        in the air must not -- a FO asked to take over on climbout who
        began by running the before-start checklist would fly the
        aeroplane into whatever was in front of it.
        """
        try:
            state = self.get_state()
        except Exception:
            state = None
        if state is None or not getattr(state, "connected", False) \
                or getattr(state, "on_ground", True):
            return AutonomousPhase.PREFLIGHT, "systems"
        session_phase = getattr(session, "phase", Phase.PREFLIGHT)
        if session_phase in (Phase.APPROACH, Phase.LANDING):
            return AutonomousPhase.APPROACH, "intercept"
        if session_phase == Phase.DESCENT:
            return AutonomousPhase.DESCENT, ""
        target = self._altitude_target(state)
        altitude = float(getattr(state, "altitude_ft", 0.0) or 0.0)
        if target and altitude > target + 500.0:
            return AutonomousPhase.DESCENT, ""
        if target and abs(altitude - target) <= 500.0:
            return AutonomousPhase.CRUISE, ""
        return AutonomousPhase.CLIMB, ""

    def _reset_scratch(self) -> None:
        """The instance-level scratch a new leg must not inherit.

        ``self.state`` is replaced outright for a new flight; these are
        the fields that live on the machine itself -- the control loops'
        memory (an integrator wound by the last leg's go-around would
        start this leg's throttle where that one ended), the radio and
        checklist bookkeeping, and the push/taxi flags. Left over, they
        are how the second leg flies the first leg's ghosts.
        """
        self._pids = {}
        self._wind_z_hist.clear()
        self._ga_flag_seen = False
        self._emergency_level = 0
        self._fuel_known = False
        self._fuel_capacity_seen = 0.0
        self._checklist_item_id = 0
        self._checklist_item_since = 0.0
        self._checklist_perform_since = 0.0
        self._checklist_sent = set()
        self._dialled_at = 0.0
        self._comms_settled = True
        self._call_tried_at = 0.0
        self._last_lever = 0.0
        self._flaps_asked = 0.0
        self._lever_at = 0.0
        self._ground_still_since = 0.0
        self._held_at_split = False

    def begin_flight(self, session: FlightSession) -> None:
        """Start a new autonomous flight."""
        self.session = session
        # The config's own word on chaining, re-read each flight: an
        # emergency's kill is this flight's, not the program's.
        self._chained = bool(
            self.config.immersion.autonomous_chained_flights)
        aircraft = self.get_state()
        if self._enabled and aircraft is not None and autofly_limit(aircraft):
            self._enabled = False
            self._emit("info", autofly_limit(aircraft), "log.autofly_unavailable")
        self.state = AutonomousState()
        self.state.phase, sub = self._entry_phase(session)
        self.state.phase_entered_at = time.monotonic()
        self._goto_sub(sub)
        if self._dir is not None:
            self._dir.reset()
        self._perf = None
        self._speeds = None
        self._heli = None
        self._reset_scratch()
        # Compute aircraft performance for chain planning
        self._compute_aircraft_performance(session)
        self._is_helicopter = self._detect_helicopter(session)
        if self._is_helicopter:
            from ..autonomous_heli import RotorcraftFlight

            self._heli = RotorcraftFlight(self)
            self._heli.begin(session)
        if self._use_sim_ai:
            # There is no SimConnect path to the simulator's own AI pilot;
            # the flag is kept for the config surface, and the flight is
            # flown on the First Officer's own autopilot regardless.
            self._emit("info", "Simulator AI assist is not available; "
                       "flying with the First Officer's own autopilot",
                       "autonomous.sim_ai.unused")
        # The engine calls this for every new session, engaged or not --
        # only the engaged machine gets to announce itself.
        if self._enabled:
            self._emit("info", "Autonomous flight started",
                       "autonomous.started")

    def set_chain_flights(self, flights: list[dict]) -> None:
        """Set up chained flights from a list of flight plans."""
        self.state.chain_flights = flights
        self.state.chain_flight_index = 0

