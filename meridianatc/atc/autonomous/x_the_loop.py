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
            self._disabled_reason = reason
            self.config.immersion.autonomous_flight = False
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
        self._log_telemetry(state, now)

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
        # Neither can a tank running dry: an engine fed from an empty
        # tank with fuel still aboard is a selector position, and it is
        # moved now rather than after the engine has quit.
        self._manage_fuel_tanks(state)
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
        # A flap stage the speed gate refused is selected as soon as the
        # aeroplane has slowed to it.
        self._configure_owed(state)
        # See-and-avoid rides the same watch: a meeting a minute out is
        # a turn made now, not a go-around made at the merge.
        self._watch_traffic(state, now)
        # The stall watch runs on the same cadence: a trigger held past
        # a gust hands the axes to the recovery, which flies them every
        # tick until the wing is flying again -- no mode selection waits
        # on the one-second interval for that.
        self._monitor_stall(state, now)
        if self.state.stall_recovering:
            # The recovery flies the axes itself -- a sim AI left
            # engaged underneath it is two pilots on one yoke.
            self._sim_ai_release(state)
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
                                   "align", "roll", "rollout",
                                   "initial_climb")
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

        # The flag's delegation, where it applies: the quiet en-route
        # legs are handed to the sim's own AI pilot while the FO keeps
        # the radios, the watch and the mission; every other moment --
        # taxi, takeoff, the approach, the flare, an emergency, a stall
        # -- is the FO's own loops, delegation or not. A leg the write
        # could not engage is flown like any other.
        if self._sim_ai_wanted(state):
            self._sim_ai_hold(state, now)
            if self._delegated:
                self.state.last_action_at = now
                return
        else:
            self._sim_ai_release(state)

        self._execute_phase(state, now)
        self._fly_direct(state)
        self._fly_hold(state, now)
        self._fly_avoidance(state)

        self.state.last_action_at = now

    # How often the sim's own numbers are written down: dense enough to
    # rebuild a ground track from, sparse enough to leave the rest of
    # the log readable.
    TELEMETRY_S = 5.0

    def _log_telemetry(self, state, now: float) -> None:
        """The simulator's own numbers, on a slow clock.

        Every other line in this log is what the FO *asked for*; this
        one is what the aeroplane *answered* -- where it is, how it is
        moving, the switches the sim reports -- the half of the story a
        "the model ignored the write" report is made of.
        """
        if now - getattr(self, "_telemetry_at", 0.0) < self.TELEMETRY_S:
            return
        self._telemetry_at = now

        def g(name):
            return getattr(state, name, 0.0) or 0.0
        log.info(
            "telem: lat %.5f lon %.5f hdg %.0f trk %.0f "
            "gs %.1f ias %.0f vs %+.0f alt %.0f agl %.0f "
            "pitch %+.1f bank %+.1f thr %.0f%% v %.1f gnd %s ap %s "
            "wind %.0f@%.0f",
            g("latitude"), g("longitude"),
            g("heading_true"), g("track_true"),
            g("ground_speed_kt"), g("indicated_airspeed_kt"),
            g("vertical_speed_fpm"), g("altitude_ft"),
            g("altitude_agl_ft"),
            g("pitch_deg"), g("bank_deg"), g("throttle_pct"),
            g("main_bus_volts"),
            1 if g("on_ground") else 0, 1 if g("autopilot") else 0,
            g("ambient_wind_direction"), g("ambient_wind_velocity"),
            extra={
                "telem": {
                    "lat": round(g("latitude"), 6),
                    "lon": round(g("longitude"), 6),
                    "hdg": round(g("heading_true"), 1),
                    "trk": round(g("track_true"), 1),
                    "gs_kt": round(g("ground_speed_kt"), 2),
                    "ias_kt": round(g("indicated_airspeed_kt"), 1),
                    "vs_fpm": round(g("vertical_speed_fpm")),
                    "alt_ft": round(g("altitude_ft")),
                    "agl_ft": round(g("altitude_agl_ft")),
                    "pitch_deg": round(g("pitch_deg"), 1),
                    "bank_deg": round(g("bank_deg"), 1),
                    "throttle_pct": round(g("throttle_pct"), 1),
                    "main_bus_volts": round(g("main_bus_volts"), 1),
                    "on_ground": bool(g("on_ground")),
                    "autopilot": bool(g("autopilot")),
                    "autothrottle": bool(g("autothrottle")),
                    "flaps_index": int(g("flaps_index")),
                    "gear_down": bool(g("gear_down")),
                    "parking_brake": bool(g("parking_brake")),
                    "paused": bool(g("paused")),
                    "wind_dir": round(g("ambient_wind_direction")),
                    "wind_kt": round(g("ambient_wind_velocity"), 1),
                    "source": getattr(state, "source", "") or "",
                }})

    # -- sim-AI delegation ---------------------------------------------------

    # The legs a sim's own pilot can be asked to fly: en-route, where
    # holding a heading, a level and a descent rate is the whole job.
    # Everything closer to the ground stays the FO's -- X-Plane's
    # ai_flies_aircraft does a straight-and-level job it cannot taxi or
    # land, and MSFS's delegate is not trusted below the approach gate.
    DELEGABLE_PHASES = (AutonomousPhase.CLIMB,
                        AutonomousPhase.CRUISE,
                        AutonomousPhase.DESCENT)

    def _sim_ai_wanted(self, state) -> bool:
        """Whether this tick should hand the flying to the sim's AI."""
        # A vector or an armed avoidance turn outranks the delegation:
        # neither reaches a sim's own pilot, so the FO takes the leg
        # back and flies the instruction itself.
        return bool(getattr(self, "_use_sim_ai", False)
                    and not self._is_helicopter
                    and self.state.phase in self.DELEGABLE_PHASES
                    and self.state.sub != "go_around"
                    and not self.state.stall_recovering
                    and not self.state.emergency
                    and not self.state.emergency_descent
                    and self.session.assigned_heading is None
                    and self.state.avoid_heading < 0.0)

    def _sim_ai_set(self, state, on: bool) -> bool:
        """Ask the sim to fly the leg itself (or stop), source by source.

        X-Plane: ``sim/operation/prefs/ai_flies_aircraft`` -- the sim's
        internal pilot flies the user's aircraft while it stands, and a
        zero hands the flying model back cleanly. MSFS: ``DELEGATE
        CONTROLS TO AI`` -- a toggle sent only on the transitions, and
        unverified on the models that ignore it (the FO's own control
        law picks the leg back up regardless, which a dead write leaves
        the sim's AI still nominally flying: a bug the FO's own writes
        fight -- so a failed or unverifiable send does not delegate).
        """
        try:
            if state.source == "xplane":
                return bool(self.sim.dref(
                    "sim/operation/prefs/ai_flies_aircraft",
                    1.0 if on else 0.0))
            if state.source == "simconnect":
                return self._send_event(b"DELEGATE CONTROLS TO AI", 1)
        except Exception:
            log.debug("sim AI delegation write failed", exc_info=True)
        return False

    def _sim_ai_hold(self, state, now: float) -> None:
        """Engage the delegation once, then keep the mission honest."""
        if not getattr(self, "_delegated", False):
            if not self._sim_ai_set(state, True):
                # No delegation path on this source (or the write fell
                # off): the flag stops asking and the leg is flown like
                # any other -- the not-available line once per flight.
                self._use_sim_ai = False
                if not getattr(self, "_sim_ai_said", False):
                    self._sim_ai_said = True
                    self._emit("info",
                               "Simulator AI assist is not available; "
                               "flying with the First Officer's own "
                               "autopilot",
                               "autonomous.sim_ai.unused")
                return
            self._delegated = True
            self._emit("info",
                       "Flying delegated to the simulator's AI for this "
                       "leg",
                       "autonomous.sim_ai.delegated")
        if self._delegated:
            self._delegate_mission(state, now)

    def _sim_ai_release(self, state) -> None:
        """Take the leg back from the sim's pilot, if it had one."""
        if not getattr(self, "_delegated", False):
            return
        self._delegated = False
        if state is not None:
            self._sim_ai_set(state, False)
            self._emit("info",
                       "First Officer flying again",
                       "autonomous.sim_ai.resumed")

    def _delegate_mission(self, state, now: float) -> None:
        """The mission bookkeeping a delegated leg still owes.

        The sim's pilot holds heading, level and rate; it owns no
        clearance, no checklist, no top of descent. The phase advances
        on the same terms the flown legs do -- an assigned approach or
        descent, the level-off at the climb's target, the plan's TOD --
        and the descent gets an AGL floor so a sim AI's late level-off
        cannot sail the aeroplane past the approach it never starts.
        """
        if self.session.phase == Phase.DESCENT \
                and self.state.phase != AutonomousPhase.DESCENT:
            self._transition(AutonomousPhase.DESCENT)
            return
        if self.session.phase == Phase.APPROACH:
            self._transition(AutonomousPhase.APPROACH)
            return
        if self.state.emergency_descent:
            self._transition(AutonomousPhase.DESCENT)
            return
        if self.state.phase == AutonomousPhase.CLIMB:
            target = self._altitude_target(state)
            if target and abs((state.altitude_ft or 0.0) - target) < 200:
                self._transition(AutonomousPhase.CRUISE)
        elif self.state.phase == AutonomousPhase.CRUISE:
            if not self.state.tod_nm:
                self._plan_tod(state)
            distance = self._distance_to_destination_nm(state)
            if distance is not None and self.state.tod_nm > 0.0 \
                    and distance <= self.state.tod_nm:
                self._transition(AutonomousPhase.DESCENT)
        elif self.state.phase == AutonomousPhase.DESCENT:
            agl = state.altitude_agl_ft or 0.0
            if 0.0 < agl < 1500.0:
                self._transition(AutonomousPhase.APPROACH)

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

