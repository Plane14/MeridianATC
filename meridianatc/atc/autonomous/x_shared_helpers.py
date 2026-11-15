from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    Phase,
    log,
    time,
)

# How fast the VS bug may move toward a new demand, feet per minute per
# second: a step on the bug is what makes the autopilot pitch abruptly,
# so the demanded rate arrives at the demeanor's rate instead.
_VS_SLEW_FPM_S = {"smooth": 400.0, "gentle": 200.0, "urgent": 1500.0}

# The AP bank cap each demeanor asks for, degrees: passenger legs keep
# the roll gentle, the precision legs gentler, and only the escapes ask
# for the thirty the autopilot offers at most. The aeroplane's own
# figure, where a profile declares one, bounds the ask below these.
_BANK_CAP_DEG = {"smooth": 20.0, "gentle": 15.0, "urgent": 30.0}


class _Shared_helpers:
    # -- shared helpers ------------------------------------------------------

    @staticmethod
    def _altimeter_hpa(value) -> float:
        """The altimeter setting in hPa, whatever the dialect gave.

        The session stores the setting as it was said: FAA positions
        give inches of mercury, everyone else hectopascals -- a figure
        under a hundred is inches. The Kohlsman event and the cockpit's
        own readback are both hectopascals, so the conversion lives in
        one place rather than at each call.
        """
        value = float(value or 0.0)
        return value * 33.8639 if 0.0 < value < 100.0 else value

    def _demeanor(self, state) -> str:
        """How hard the FO is flying right now: smooth, gentle or urgent.

        Urgent is the safety legs -- a declared emergency, an escape
        climb, a windshear that has just been met, or the load limiter
        holding the commands: those get the fast hands and the wide
        clamps. Gentle is the precision legs flown by hand, where the
        last of the travel is spent carefully. Everything else is
        smooth, at passenger rates.
        """
        s = self.state
        demeanor = "gentle" if s.sub in (
            "rotate", "flare", "roll", "rollout") else "smooth"
        if demeanor == "smooth" and time.monotonic() < s.turbulence_until:
            # Rough air gets the smaller corrections too.
            demeanor = "gentle"
        if s.g_limited or s.emergency or s.emergency_descent \
                or s.weather_ga_active or s.sub == "go_around" \
                or (s.windshear_since
                    and time.monotonic() - s.windshear_since < 30.0):
            demeanor = "urgent"
        s.demeanor = demeanor
        return demeanor

    def _vs_slew(self, state, want_fpm: float, now: float) -> float:
        """The VS bug moved toward its new demand at the demeanor's rate.

        While the load flag is up the demand is level flight -- the
        limiter, not the schedule, is what the bug chases -- and the
        urgent rate is what gets it there.
        """
        if self.state.g_limited:
            want_fpm = 0.0
        rate = _VS_SLEW_FPM_S.get(self._demeanor(state), 400.0)
        bug = float(self.state.vs_bug_cmd)
        step = rate * max(0.0, now - (self.state.vs_bug_at or now))
        bug += max(-step, min(step, want_fpm - bug))
        self.state.vs_bug_cmd = bug
        self.state.vs_bug_at = now
        return bug

    def _pitch_target_deg(self) -> float:
        """The nose-up attitude the rotation and the go-around aim at.

        By the same cruise-speed ladder the envelope sorts aeroplanes
        with -- ten degrees for the light aeroplane, twelve for the
        turboprop, fifteen for the jet -- trimmed toward the attitude
        this model actually lifted off at last time, where the store
        has measured one. The learned figure is let three degrees off
        the class's, no more: a tail-scraper is met halfway, not
        followed into the runway.
        """
        cruise = self.performance.cruise_kt
        if cruise >= 230.0:
            target = 15.0
        elif cruise >= 140.0:
            target = 12.0
        else:
            target = 10.0
        adapt = getattr(self, "_adapt", None)
        learned = adapt.value("rotate.pitch", 0.0) \
            if adapt is not None else 0.0
        if learned > 0.0:
            target = max(target - 3.0, min(target + 3.0,
                                           0.5 * target + 0.5 * learned))
        return target

    def _altitude_target(self, state) -> float:
        """The level to fly now: what ATC last assigned, else the plan's."""
        return (self.session.assigned_altitude_ft
                or self.session.cruise_altitude_ft
                or self.session.initial_altitude_ft or 0.0)

    def _penetration_kt(self, state) -> float:
        """The speed rough air is ridden out at: a margin off the red
        line and off the stall, the way the books write it."""
        cruise = self.performance.cruise_kt or 200.0
        lim = self._limits(state)
        pen = cruise * 0.75
        if lim["stall_kt"] > 0.0:
            pen = max(pen, lim["stall_kt"] * 1.5)
        return max(60.0, min(pen, lim["max_ias_kt"]))

    def _speed_target(self, state) -> float:
        target = 0.0
        if self.session.assigned_speed_kt:
            target = self.session.assigned_speed_kt
        # In the cruise the model's own estimate of its cruise speed is
        # the best figure there is -- it is this aeroplane's, not a class's
        # -- and the class number is the fallback for a model that
        # declares none.
        elif self.state.phase == AutonomousPhase.CRUISE:
            estimated = float(getattr(state, "estimated_cruise_kt", 0.0)
                              or 0.0)
            if 50.0 < estimated < 600.0:
                # The variable may be true rather than indicated speed;
                # never command past the model's own red line.
                redline = float(getattr(state, "barber_pole_kt", 0.0)
                                or 0.0)
                if redline > 0:
                    estimated = min(estimated, redline * 0.95)
                target = estimated
        if not target:
            target = self.state.climb_kt or self.performance.cruise_kt
        if time.monotonic() < self.state.turbulence_until:
            # Rough air beats the assignment, the way a crew asks for a
            # slower speed rather than ride it out at it.
            target = min(target, self._penetration_kt(state))
        # Two-fifty below ten thousand: the speed limit the departures
        # and arrivals both fly, for the aeroplanes fast enough to break
        # it. A light aeroplane's own speeds are already under it.
        if (self.performance.cruise_kt >= 200.0
                and self.state.phase in (AutonomousPhase.CLIMB,
                                         AutonomousPhase.DESCENT)
                and 0.0 < float(getattr(state, "altitude_ft", 0.0) or 0.0)
                < 10_000.0):
            target = min(target, 250.0)
        # An engine that is out changes what the wing is for: all out is
        # the best-glide figure -- about a one-point-four over the stall
        # -- and one of several is the drift-down speed a twin comes down
        # at on the one it has left.
        if self.state.emergency == "engine":
            lim = self._limits(state)
            glide = max((lim["stall_kt"] or 0.0) * 1.4,
                        self.performance.approach_kt * 0.9)
            return self._safe_speed(state, glide, lim)
        if self.state.emergency == "engine-out":
            return min(target, self.state.climb_kt
                       or self.performance.climb_kt)
        return target

    def _apply_flight_director(self, state, altitude: float = 0.0,
                             vs_fpm: float = 0.0, heading: float = -1.0,
                             speed_kt: float = 0.0) -> None:
        """Put the assigned numbers on the autopilot, managed or direct.

        ``heading`` < 0 means "not commanded"; ``vs_fpm`` nonzero means a
        rate is wanted (climb or descent in progress); zero means level.
        """
        d = self.director
        # The sim's own flag is the truth: a pilot who bumped the AP off
        # mid-engage gets it back rather than a machine that "remembers"
        # engaging it once and never asks again. AUTOPILOT_ON is
        # idempotent, so a model that never reports the variable just
        # gets asked every pass.
        if not getattr(state, "autopilot", False):
            d.autopilot(True)
        self.state.ap_master = True
        # The yaw damper rides the AP legs where the model has one -- the
        # Dutch roll the passengers never feel. An aeroplane without the
        # system answers nothing to the event and flies as it did.
        d.yaw_damper(True)
        lim = self._limits(state)
        if altitude:
            d.altitude(self._safe_altitude(state, altitude, lim))
        if heading >= 0.0:
            d.heading(heading)
            self.state.ap_hdg_mode = True
        if speed_kt:
            speed_kt = self._safe_speed(state, speed_kt, lim)
            d.speed(speed_kt)
            self.state.ap_speed_mode = True
            self._manage_throttle(state, speed_kt)
        now = time.monotonic()
        if vs_fpm:
            vs_fpm = max(lim["vs_dn_fpm"], min(float(vs_fpm),
                                             lim["vs_up_fpm"]))
            # The bug is slewed to the demand, not stepped: the AP's
            # pitch answer follows a ramp instead of a stair.
            vs_fpm = self._vs_slew(state, vs_fpm, now)
            d.vertical_speed(vs_fpm)
            d.vs_hold(True)
            self.state.ap_vs_mode = True
        else:
            self.state.vs_bug_cmd = 0.0
            self.state.vs_bug_at = now
            d.vs_hold(False)
            self.state.ap_vs_mode = False
            if altitude:
                d.alt_hold(True)
                self.state.ap_alt_mode = True
        # The AP's own roll limit: the demeanor's cap, or the load
        # limiter's tighter one while that flag is up -- and never more
        # than the aeroplane's own declared limit.
        demeanor = self._demeanor(state)
        cap = 15.0 if self.state.g_limited else _BANK_CAP_DEG.get(
            demeanor, 20.0)
        own = float(getattr(self.performance, "max_bank_deg", 0.0) or 0.0)
        if own > 0.0:
            cap = min(cap, own)
        self._cap_bank(cap)
        if self._ap_mode == "managed":
            # Managed legs climb and descend on the flight-level-change
            # mode where the model has it; direct legs use the VS bug.
            if vs_fpm and self.state.phase in (
                    AutonomousPhase.CLIMB, AutonomousPhase.DESCENT):
                d.level_change(True)
            elif self.state.phase in (AutonomousPhase.CLIMB,
                                      AutonomousPhase.DESCENT) \
                    and abs(vs_fpm) < 1:
                d.level_change(False)

    def _manage_throttle(self, state, target_kt: float) -> None:
        """The FO's own throttle, for the aeroplanes with no autothrottle.

        On a model whose autopilot cannot hold speed the lever stays
        where takeoff or the go-around left it -- full power all the way
        to the red line. This servo moves the levers only when the speed
        is genuinely off target, so a working autothrottle, which keeps
        the speed inside the deadband on its own, is never fought.
        """
        if not target_kt or state.on_ground or not state.engines_running:
            return
        # A live autothrottle owns the levers: the servo's job only exists
        # where there is none, and two hands on one lever is a porpoise.
        if getattr(state, "autothrottle", False):
            return
        if self.state.phase == AutonomousPhase.APPROACH:
            # The go-around owns the levers, and the pattern right after
            # it is still a climb to the missed altitude, not an arrival
            # at approach speed.
            if self.state.sub == "go_around":
                return
            missed = self.state.missed_alt_ft or 0.0
            if missed and (state.altitude_ft or 0.0) < missed - 150.0:
                # Below the missed-approach height the climb wants full
                # power, whatever the calendar says: the lever is left
                # where the go-around put it until the level-off.
                return
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        if ias <= 20.0:
            return
        lever = self._lever_position(state)
        barber = float(getattr(state, "barber_pole_kt", 0.0) or 0.0)
        max_mach = float(getattr(state, "mach_max_operate", 0.0)
                         or getattr(state, "barber_pole_mach", 0.0) or 0.0)
        mach = float(getattr(state, "airspeed_mach", 0.0) or 0.0)
        if (barber > 0.0 and ias >= barber * 0.97) \
                or (max_mach > 0.0 and mach >= max_mach - 0.005):
            # At the red line -- knots low down, mach high up -- the lever
            # comes down, whatever the target.
            lever = min(lever - 0.10, 0.40)
        elif self._use_pids:
            # The closed loop owns the lever: a speed error moved against,
            # the trim the model needs learned by the integrator, and the
            # deadband inside it so the last knot is not chased forever.
            pid = self._pid("throttle")
            if pid.output is None:
                pid.reset(output=lever)
            lever = pid.update(target_kt - ias, measurement=ias)
        else:
            err = ias - target_kt
            if err > 8.0:
                lever -= min(0.20, 0.02 + err * 0.006)
            elif err < -15.0:
                lever += min(0.15, 0.02 + (-err) * 0.006)
            elif self._engine_limit_cap(state) >= 1.0:
                # On target and no gauge over its red line: nothing to
                # move. An exceedance inside the deadband is NOT a
                # deadband case -- it falls through to the cap below.
                return
        lever = max(0.05, min(0.97, lever))
        # The engine watch's ceiling: a gauge past its published red line
        # gets power off, whatever the speed loop wanted.
        lever = min(lever, self._engine_limit_cap(state))
        self._last_lever = lever
        self.director.throttle(lever)

    def _surface_position(self, positions: tuple[str, ...]):
        """The departure field's station in one of these positions, where
        one is actually published. ``station_for`` cannot answer this:
        its fallback chain hands back Tower for Ground, and what is
        being asked here is whether the position itself exists at all.
        """
        session = self.session
        airport = getattr(session, "departure", None) \
            or self._get_current_airport()
        if airport is None or getattr(self, "navdb", None) is None:
            return None
        try:
            for position in positions:
                for station in self.navdb.stations(airport.ident,
                                                   position) or ():
                    if getattr(station, "mhz", 0.0):
                        return station
        except Exception:
            return None
        return None

    def _stand_clear(self) -> tuple[bool, str]:
        """Whether the ramp says the aeroplane may move, and why not.

        A jetway or a set of stairs still on the aeroplane, a fuel hose
        attached, a service still running or a tug not let go yet all
        read the same to a pilot: not clear. Where nothing is watching
        the ramp there is nothing in the way by definition.
        """
        ground = getattr(self.brain, "ground", None)
        gstate = getattr(ground, "state", None) if ground is not None else None
        if gstate is None or not getattr(gstate, "available", False):
            return True, ""
        try:
            return bool(gstate.can_taxi), str(gstate.why_not() or "")
        except Exception:
            return True, ""

    def _free_the_stand(self, why: str) -> None:
        """Ask the ramp to take its equipment off the aeroplane, once.

        The jetway and the stairs are toggles on the same GSX menu, so
        asking for what is connected is asking for it to come off. A
        fuel hose or a running service is the truck's own business and
        is only waited on, not walked.
        """
        if self.state.stand_hold_said:
            return
        self.state.stand_hold_said = True
        ground = getattr(self.brain, "ground", None)
        gstate = getattr(ground, "state", None) if ground is not None else None
        asked = ""
        if ground is not None and getattr(ground, "can_ask", False):
            try:
                if getattr(gstate, "jetway_connected", False):
                    asked = ground.request("jetway") or ""
                elif getattr(gstate, "stairs_connected", False):
                    asked = ground.request("stairs") or ""
            except Exception:
                asked = ""
        self._emit("info", f"Holding the stand: {why}"
                   + (f" -- {asked}" if asked else ""),
                   "autonomous.stand.hold")

    def _ground_confirm_push(self) -> bool:
        """Answer the tug's "is the flight deck ready?" once.

        GSX pauses a push with the tug attached to ask exactly that; the
        answer it wants is its own menu's confirm, not a radio call.
        Asked once per leg -- a prompt that is not up is simply skipped.
        """
        if self.state.push_confirmed:
            return False
        ground = getattr(self.brain, "ground", None)
        gstate = getattr(ground, "state", None) if ground is not None else None
        if gstate is None or not getattr(gstate, "being_pushed", False):
            return False
        try:
            said = ground.request("confirm")
        except Exception:
            said = ""
        if not said:
            return False
        self.state.push_confirmed = True
        self._emit("info", f"Push confirmed: {said}",
                   "autonomous.pushback.confirm")
        return True

    def _clearance_position(self):
        """The staffed position that can issue a clearance or a release
        on the departure field, or None where nobody can -- a broadcast
        frequency, or an advisory service that only ever advises.
        ``station_for``'s fallbacks are not what is asked here: a tower
        working the field's ground is clearance authority, a CTAF is
        not, and an AFIS gives the runway at the pilot's discretion.
        """
        issues = getattr(getattr(self, "brain", None),
                         "issues_clearances", None)
        for position in ("DEL", "GND", "TWR"):
            station = self._surface_position((position,))
            if station is None:
                continue
            if issues is None or issues(station):
                return station
        return None

    def _clearance_done(self) -> bool:
        """Whether the paper the stand was waiting for is read back.

        IFR: the controller marked ``clearance_read_back`` when it
        accepted the clearance readback. A VFR release sets no flag --
        it is the session's last instruction, and its accepted readback
        is the release held.
        """
        if self.session.ifr:
            return bool(self.session.clearance_read_back)
        last = getattr(self.session, "last_instruction", None)
        return (getattr(last, "kind", "") == "vfr_release"
                and bool(getattr(last, "read_back", False)))

    def _pushback_by_radio(self) -> bool:
        """Whether the push is somebody's to approve on the frequency.

        With ground control switched off the request is unanswered on
        every frequency by design, so the FO presses for the tug itself;
        at a field with no surface position there is nobody to ask, and
        the same is true. The tower alone does not count: it does not
        take the pushback call here, and a wait on it is a wait for a
        voice that never comes -- and nor is a published position
        nobody is actually working.
        """
        brain = getattr(self, "brain", None)
        if not getattr(brain, "ground_control", False):
            return False
        station = self._surface_position(("GND", "DEL", "RMP"))
        if station is None:
            return False
        staffed = getattr(brain, "staffed", None)
        return staffed is None or staffed(station)

    def _cleared_to_taxi(self) -> bool:
        """Whether the taxi instruction has been issued -- or nobody
        here issues one.

        The session's own phase is the truth: TAXI_OUT or further means
        a route was cleared, and a ``taxi_path`` is the route itself.
        Where somebody works the surface the aeroplane waits for the
        instruction -- taxiing without it is the thing a holding point
        exists to prevent, and the calls that ask for it are worked by
        ``_work_the_calls`` until the taxi watchdog says the wait
        itself is the failure. Where nobody can issue one -- a
        broadcast frequency, an advisory service, a published position
        nobody is working -- the FO's call is the announcement a real
        pilot makes before moving, so after a moment for it to go out
        the aeroplane taxis on its own navigation.
        """
        # A clearance does not move the ramp: an aeroplane still on the
        # jetway, the stairs, a fuel hose or the tug does not taxi on
        # any instruction. Said once -- GSX's state can lie and a pilot
        # walking the menu by hand clears it while it does.
        clear, why = self._stand_clear()
        if not clear:
            if not self.state.stand_hold_said:
                self.state.stand_hold_said = True
                self._emit("info", f"Taxi held: {why}",
                           "autonomous.stand.hold")
            return False
        session = self.session
        # TAXI_OUT alone is not the clearance: the phase also flips on
        # groundspeed alone, which is how an aeroplane pushed back fast
        # taxied out on no instruction at all. ``taxi_route`` is the
        # names the controller gave and ``taxi_path`` the line they
        # planned; either one is an instruction received, and the
        # phases past taxi are its consequences.
        if session.phase in (Phase.HOLDING_SHORT, Phase.LINED_UP,
                             Phase.TAKEOFF) \
                or getattr(session, "taxi_path", None) \
                or getattr(session, "taxi_route", None):
            return True
        if self._taxi_issuing_position() is not None:
            # Somebody owns the surface: the instruction is theirs
            # to give, and the aeroplane waits for it.
            return False
        return self._sub_age() > 10.0

    def _taxi_issuing_position(self):
        """The staffed position a real taxi route comes from, if any.

        Ground, delivery or tower -- the positions an actual taxi
        instruction is issued on. An AFIS advises ("at your
        discretion") and a CTAF hears only announcements: neither owes
        a route, and waiting on either is a wait for a voice that
        never comes.
        """
        staffed = getattr(getattr(self, "brain", None), "staffed", None)
        for position in ("GND", "DEL", "TWR"):
            station = self._surface_position((position,))
            if station is not None \
                    and (staffed is None or staffed(station)):
                return station
        return None

    def _surface_contacts(self) -> list:
        """Every aeroplane the controller's picture holds, for yielding to.

        The invented traffic, the live aeroplanes this program flies, and
        the simulator's own -- which obey nobody. A picture that cannot be
        asked is treated as empty: an absent picture is no reason to hold
        position for ever.
        """
        brain = getattr(self, "brain", None)
        contacts = getattr(brain, "contacts", None)
        if contacts is None:
            return []
        try:
            return list(contacts() or [])
        except Exception:
            log.debug("could not read the traffic picture", exc_info=True)
            return []

    def _runway_blocked(self, runway: str = "", airport=None) -> bool:
        """Whether anybody is on the strip the flight is about to use.

        The departure runway unless told otherwise, resolved the way
        :meth:`_find_runway` resolves it, and measured against every
        contact -- the tower's view of occupancy
        (:func:`meridianatc.atc.occupancy.on_the_runway`), not just the
        traffic this program is flying.
        """
        session = self.session
        name = runway or getattr(session, "departure_runway", "") \
            or (self.state.runway_key[1] if self.state.runway_key else "")
        airport = airport or getattr(session, "departure", None) \
            or getattr(session, "destination", None) \
            or self._get_current_airport()
        if not name or airport is None:
            return False
        try:
            from .. import occupancy
            return bool(occupancy.on_the_runway(
                self.navdb, airport, name, self._surface_contacts()))
        except Exception:
            log.debug("the runway occupancy check failed", exc_info=True)
            return False

