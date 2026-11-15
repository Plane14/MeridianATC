from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    Phase,
    autofly_limit,
    capabilities,
    log,
    time,
)


class _The_loop:
    # -- the loop ------------------------------------------------------------

    def tick(self) -> None:
        """Main autonomous flight loop. Called from the situation loop."""
        if not self._enabled:
            return

        state = self.get_state()
        if state is None or not state.connected:
            return
        reason = autofly_limit(state)
        if reason:
            self._enabled = False
            self._handoff()
            self._emit("info", reason, "log.autofly_unavailable")
            return
        self._last_state = state
        # What the loops have learned about this model is written down
        # on a slow clock while the flight runs, not only when it ends.
        if getattr(self, "_adapt", None) is not None:
            from .. import adaptation
            adaptation.save()
        if not self._can_fly_phase(state):
            self._manual_control_required()
            return
        if not capabilities(state.source).flight_axes and not self.state.ap_master:
            self.state.ap_master = self.director.autopilot(True)

        now = time.monotonic()

        # A helicopter is flown by its own controller, on every pass of the
        # loop rather than the fixed-wing action interval: the cyclic,
        # collective and pedals are closed-loop controls, not once-a-second
        # mode selections. The radio and the checklists are a FO's jobs
        # whatever it flies, so they run from here either way.
        self._manage_comms(state, now)
        self._drive_checklist(state, now)
        # The weather watch is a FO's job whatever is flown, like the
        # radios: a helicopter ices and is blown around as an aeroplane
        # does. The go-around parts are the fixed-wing phases' own.
        self._monitor_weather(state, now)
        # The engine watch runs on the same fast cadence: a needle past
        # the type's published red line is said once, and the lever cap
        # the throttle code reads is what keeps it off the line.
        self._watch_engines(state, now)
        # The cowl flaps ride the same watch: a lever change belongs to
        # the phase, a CHT running hot cannot wait for the next one.
        self._manage_cooling(state)
        if not self._enabled:
            return
        # Emergencies outrank the phase: a MAYDAY changes what the flight
        # is for before it changes where the aeroplane is pointed. The
        # rotorcraft watch runs here too -- its controller's tick is what
        # flies the autorotation that follows.
        self._monitor_emergency(state, now)
        # The load watch runs on the same fast cadence as the weather and
        # the emergencies -- a pull past the limit cannot wait for the
        # one-second mode selections, and its touchdown report edges off
        # the air->ground change wherever it happens.
        self._monitor_load(state, now)
        # The stall watch runs on the same cadence: a trigger held past
        # a gust hands the axes to the recovery, which flies them every
        # tick until the wing is flying again -- no mode selection waits
        # on the one-second interval for that.
        self._monitor_stall(state, now)
        if self.state.stall_recovering:
            self._fly_stall_recovery(state, now)
            return
        if self._heli is not None:
            self._heli.tick(state)
            return

        # The phases that are flown by hand -- rotation, the flare, the
        # go-around's pitch, the taxi -- want a faster hand than the 1 Hz
        # mode selections the rest of the flight needs.
        interval = self.ACTION_INTERVAL
        fast = self.state.phase in (AutonomousPhase.TAKEOFF,
                                    AutonomousPhase.LANDING,
                                    AutonomousPhase.TAXI_OUT,
                                    AutonomousPhase.TAXI_IN,
                                    AutonomousPhase.LINE_UP) \
            and self.state.sub in ("rotate", "flare", "taxi",
                                   "align", "roll", "rollout")
        # The go-around is hand-flown too, from the APPROACH phase: its
        # pitch and power answer wants the same quarter-second hand.
        if fast or self.state.sub == "go_around":
            interval = 0.25
        if now - self.state.last_action_at < interval:
            return

        # Pushback progress, which the phase reads: a push that has moved the
        # aeroplane and then stopped is done, however the tug was attached.
        # The threshold is under a knot: a tug walks the aeroplane, and two
        # knots is faster than the push ever gets.
        if self.state.phase == AutonomousPhase.PUSHBACK:
            if (state.ground_speed_kt or 0.0) > 0.8:
                if not self.state.moved_since_push:
                    self.state.push_moved_at = now
                self.state.moved_since_push = True
                self.state.still_since = now
            elif self.state.moved_since_push and self.state.still_since == 0.0:
                self.state.still_since = now

        self._execute_phase(state, now)

        self.state.last_action_at = now

    def _can_fly_phase(self, state) -> bool:
        return capabilities(state.source).flight_axes or (
                not self._is_helicopter and self.state.phase in (
                    AutonomousPhase.CLIMB, AutonomousPhase.CRUISE,
                    AutonomousPhase.DESCENT, AutonomousPhase.APPROACH)
                and self.state.sub != "go_around")

    def _manual_control_required(self) -> None:
        self._enabled = False
        self._handoff()
        self._emit("info", "Take manual control: this simulator cannot hand "
                   "over these flight phases.", "log.autofly_manual")

    def _execute_phase(self, state, now: float) -> None:
        """Execute logic for the current autonomous phase."""
        phase = self.state.phase
        if not self._can_fly_phase(state):
            self._manual_control_required()
            return

        # The session can move under a phase that is not watching it: a
        # clearance the tower takes back becomes a taxi to a stand while
        # the FO is still holding for the strip in ``align`` -- which
        # left it parked on the runway for the rest of the session.
        # Ground phases before the takeoff follow the session to the
        # stand, the way the takeoff's own ``await`` already does.
        if self.session.phase == Phase.TAXI_IN \
                and self.session.taxi_path \
                and phase in (AutonomousPhase.HOLDING_SHORT,
                              AutonomousPhase.LINE_UP,
                              AutonomousPhase.TAKEOFF):
            self._transition(AutonomousPhase.TAXI_IN)
            phase = AutonomousPhase.TAXI_IN

        try:
            if phase == AutonomousPhase.PREFLIGHT:
                self._phase_preflight(state, now)
            elif phase == AutonomousPhase.REQUEST_CLEARANCE:
                self._phase_request_clearance(state, now)
            elif phase == AutonomousPhase.PUSHBACK:
                self._phase_pushback(state, now)
            elif phase == AutonomousPhase.TAXI_OUT:
                self._phase_taxi_out(state, now)
            elif phase == AutonomousPhase.HOLDING_SHORT:
                self._phase_holding_short(state, now)
            elif phase == AutonomousPhase.LINE_UP:
                self._phase_line_up(state, now)
            elif phase == AutonomousPhase.TAKEOFF:
                self._phase_takeoff(state, now)
            elif phase == AutonomousPhase.CLIMB:
                self._phase_climb(state, now)
            elif phase == AutonomousPhase.CRUISE:
                self._phase_cruise(state, now)
            elif phase == AutonomousPhase.DESCENT:
                self._phase_descent(state, now)
            elif phase == AutonomousPhase.APPROACH:
                self._phase_approach(state, now)
            elif phase == AutonomousPhase.LANDING:
                self._phase_landing(state, now)
            elif phase == AutonomousPhase.TAXI_IN:
                self._phase_taxi_in(state, now)
            elif phase == AutonomousPhase.PARKED:
                self._phase_parked(state, now)
            elif phase == AutonomousPhase.SHUTDOWN:
                self._phase_shutdown(state, now)
            elif phase == AutonomousPhase.CHAIN_NEXT:
                self._phase_chain_next(state, now)
            elif phase == AutonomousPhase.ERROR:
                self._phase_error(state, now)
        except Exception as e:
            log.exception("autonomous flight error in phase %s: %s", phase, e)
            self._set_error(f"Phase {phase.value}: {e}")

