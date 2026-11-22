from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    FlightSession,
    log,
    time,
)


class _Helpers_kept_from:
    # -- helpers kept from the original ----------------------------------------

    def _compute_aircraft_performance(self, session: FlightSession) -> None:
        """Compute aircraft range and performance for chain flight planning."""
        type_code = session.aircraft.type_code.upper()
        self.state.aircraft_type = type_code
        self.state.aircraft_floats = bool(
            getattr(session.aircraft, "on_floats", False))

        ranges = {
            "C152": 350, "C172": 550, "C182": 750, "C208": 800,
            "PA28": 500, "PA34": 700, "DA40": 600, "DA42": 800,
            "SR22": 700, "BE36": 800, "TB20": 700,
            "TBM9": 1500, "PC12": 1300, "AT76": 800, "DH8D": 900,
            "C25A": 1200, "C25C": 1400, "C700": 1800,
            "C525": 1500, "C560": 1800, "C680": 2200, "C750": 2500,
            "E55P": 1500, "E50P": 1800, "F2TH": 2000, "F900": 2500,
            "GLEX": 3500, "GLF5": 3500, "GLF6": 4000, "G280": 2000,
            "H25B": 1800, "LJ45": 1800, "LJ60": 2000,
            "A319": 3200, "A320": 3000, "A321": 3000, "A20N": 3500, "A21N": 3500,
            "B737": 2800, "B738": 3000, "B739": 3000, "B38M": 3500, "B39M": 3500,
            "B752": 3500, "B753": 3300,
            "A332": 5500, "A333": 5500, "A339": 6000, "A343": 6500, "A346": 7000,
            "A359": 7000, "A35K": 8000, "A388": 8000,
            "B763": 5500, "B764": 5500, "B772": 5500, "B77W": 7500, "B77L": 8500,
            "B77X": 8000, "B788": 7500, "B789": 7500, "B78X": 6500,
            "B744": 7000, "B748": 7500,
            "CRJ7": 1500, "CRJ9": 1500, "CRJX": 1500,
            "E170": 1800, "E175": 1800, "E190": 2200, "E195": 2200,
        }

        # The perftable's own row first -- it is the maintained source of
        # still-air ranges -- then the local list, which carries the
        # business jets and heavies the perftable does not. The class
        # figure is last: a profile says what the flight looks like, not
        # what the aeroplane is, and an airliner flight on a piston twin
        # used to give the Baron two and a half thousand miles of legs.
        from .. import perftable

        perf = perftable.type_perf(type_code)
        range_nm = perf.range_nm if perf else 0.0
        if not range_nm:
            range_nm = ranges.get(type_code, 0)
        if not range_nm:
            for prefix, r in ranges.items():
                if type_code.startswith(prefix):
                    range_nm = r
                    break
        if not range_nm:
            # A type nobody lists takes the class figure -- but the class
            # of airframe the simulator says it is, not the flight's
            # profile: an unknown piston twin or rotorcraft does not fly
            # an airliner's legs.
            from .. import aircraft_class as ac

            cls = ""
            try:
                state = self.get_state()
                if state is not None:
                    cls = ac.of_state(state).name
            except Exception:
                pass
            light = cls in (ac.PISTON_SINGLE, ac.PISTON_TWIN, ac.HELICOPTER)
            range_nm = perftable.range_for(type_code, light=light)
            if cls in (ac.TURBOPROP_SINGLE, ac.TURBOPROP_TWIN):
                range_nm = min(range_nm, 1200.0)

        self.state.aircraft_range_nm = range_nm
        log.info("Aircraft %s: estimated range %d NM", type_code, range_nm)

    def _transition(self, new_phase: AutonomousPhase) -> None:
        """Transition to a new autonomous phase."""
        log.info("Autonomous phase: %s -> %s", self.state.phase.value, new_phase.value)
        self.state.previous_phase = self.state.phase
        self.state.phase = new_phase
        self.state.phase_entered_at = time.monotonic()
        self.state.sub = ""
        self.state.sub_entered_at = time.monotonic()
        self._emit("info", f"Autonomous: {new_phase.value}", f"autonomous.phase.{new_phase.value}")

    def _set_error(self, message: str) -> None:
        """Set error state."""
        self.state.error_message = message
        self.state.error_count += 1
        log.error("autonomous flight error in %s: %s",
                  self.state.phase.value, message)
        self._transition(AutonomousPhase.ERROR)

    def _emit(self, kind: str, text: str, key: str = "") -> None:
        """Emit an event."""
        # An event the pilot sees is not a log line: a rejected takeoff's
        # reason used to reach the panel and leave nothing in the log to
        # explain why the aeroplane stopped.
        if kind in ("error", "warning"):
            log.warning("%s", text)
        self.on_event(kind, text, "Autonomous", key=key)

    def _ensure_systems_on(self, state) -> None:
        """Ensure battery, avionics, and essential systems are on."""
        if not getattr(state, "battery_master", False):
            self._send_event(b"MASTER_BATTERY_SET", 1)
        self.director.battery(True)
        self.director.avionics(True)

    def _start_engines(self, state) -> None:
        """Start engines using starter events."""
        if not state.engines_running:
            # Props forward and mixture rich before the crank -- the
            # levers a prop start is primed with, and the ones a low
            # quadrant position would leave the start without.
            self._prop_levers()
            self._send_event(b"ENGINE_AUTO_START", 1)
            self.director.starter(True)
            self.director.all_starters(True)

    def _release_parking_brake(self) -> None:
        self._send_event(b"PARKING_BRAKE_SET", 0)
        self.director.parking_brake(False)

    def _set_parking_brake(self) -> None:
        self._send_event(b"PARKING_BRAKE_SET", 1)
        self.director.parking_brake(True)

    def _pushback_skippable(self) -> bool:
        """Whether this aeroplane can leave the stand under its own power
        when no tug ever answers: a light single or twin on a ramp taxis
        straight out, where an airliner nose-in at a gate cannot. The
        class comes from the aircraft table; anything heavier stays an
        error so a real failure is never silently taxied through a
        building."""
        from .. import aircraft_class as ac
        return getattr(self, "_aircraft_class", "airliner") in (
            ac.PISTON_SINGLE, ac.PISTON_TWIN, ac.TURBOPROP_SINGLE,
            ac.HELICOPTER)

    def _request_pushback(self) -> None:
        """Request pushback through the ramp, then the simulator's own tug."""
        # The tug cannot move the aeroplane with the parking brake set,
        # but a tug still on its way is no reason to release it: the brake
        # that held the start holds the wait too, and comes off only once
        # the tug is attached -- an idling aeroplane creeps without it.
        # The brake goes back on again after the push has stopped.
        ground = getattr(self.brain, "ground", None)
        if ground is not None:
            try:
                said = ground.start_pushback()
            except Exception as e:
                log.debug("ramp pushback request failed: %s", e)
            else:
                if said:
                    log.info("autonomous pushback: %s", said)
                    self._emit("info", said, "autonomous.pushback")
                    return
        if self.sim is not None and getattr(self.sim, "connected", False):
            try:
                if self.sim.pushback():
                    log.info("autonomous pushback: simulator tug")
                    return
            except Exception as e:
                log.debug("simulator pushback failed: %s", e)
        if self.director.pushback():
            log.info("autonomous pushback: director event")

    def _request_clearance_via_fo(self) -> bool:
        """The clearance or the release the flight's rules need, asked
        for through the first officer. True when the call went out on
        the frequency; a refused one is asked again."""
        return self._fo_say("clearance" if self.session.ifr else "vfr")

    def _engage_climb(self, state) -> None:
        """The climb's guidance, once the initial climb is flown."""
        target = self._altitude_target(state)
        assigned_hdg = self.session.assigned_heading
        speed = self._speed_target(state)
        if self._ap_mode == "managed":
            d = self.director
            d.autopilot(True)
            self.state.ap_master = True
            if assigned_hdg is not None:
                d.heading(assigned_hdg)
                self.state.ap_hdg_mode = True
            else:
                own = self._route_lateral(state)
                if own >= 0.0:
                    d.heading(own)
                    self.state.ap_hdg_mode = True
            d.speed(speed)
            d.airspeed_hold(True)
            # Speed hold only drives the levers where an autothrottle is
            # armed -- aircraft.cfg's autothrottle_arming_required makes
            # the arm a separate ask, sent once and only while the sim
            # says the AT is not already live.
            if not getattr(state, "autothrottle", False):
                d.autothrottle_arm()
            if target:
                d.altitude(target)
            d.level_change(True)
            self._manage_throttle(state, speed)
        else:
            self._apply_flight_director(
                state, altitude=target,
                vs_fpm=self.performance.climb_fpm,
                heading=(assigned_hdg if assigned_hdg is not None
                         else self._route_lateral(state)),
                speed_kt=speed)

    def _level_off_at_cruise(self, state) -> None:
        """Level off at cruise altitude."""
        # FLCH off first, before the holds go on: the mode left engaged
        # past altitude capture keeps the levers at the climb/descent
        # extremes it was flying -- idle where it levelled off -- and
        # the speed decays toward the stall before the holds pull the
        # aeroplane out of it (the mode is for the level *change*; the
        # cruise belongs to ALT + SPD).
        self.director.level_change(False)
        self.director.alt_hold(True)
        own = self._route_lateral(state)
        if own >= 0.0:
            self.director.heading(own)
        self.director.airspeed_hold(True)
        self.director.speed(self._speed_target(state))
        if not getattr(state, "autothrottle", False):
            self.director.autothrottle_arm()
        self.director.vs_hold(False)
        self.state.ap_alt_mode = True
        self.state.ap_speed_mode = True

    def _shutdown_engines(self) -> None:
        """Shutdown engines."""
        self._send_event(b"ENGINE_AUTO_SHUTDOWN", 1)
        self.director.throttle_cut()
        self.director.throttle_idle()

    def _send_event(self, event_name: bytes, value: int) -> bool:
        """Send a SimConnect event."""
        if not self.sim or not self.sim.connected:
            return False
        try:
            return self.sim._send(event_name, value)
        except Exception as e:
            log.debug("Failed to send event %s: %s", event_name, e)
            return False


