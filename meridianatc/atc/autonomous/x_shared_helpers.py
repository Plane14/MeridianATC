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

    # How far off the bearing to the fix the active leg runs to the track
    # may sit before the armed NAV is judged to be steering nothing, and
    # how long it may sit there before the FO flies the heading itself.
    # A NAV mode takes tens of seconds to roll into an intercept, so a
    # course change is not a failure -- keeping on a heading that goes
    # nowhere near the route for this long is.
    NAV_OFF_DEG = 80.0
    NAV_OFF_S = 45.0

    # How fast over the bug a held speed may sit before the thing on the
    # levers is judged no autothrottle, and for how long. A working AT
    # keeps within a handful of knots of its figure; a change or a gust
    # passes in seconds -- fifteen over for fifteen straight is the
    # model's own logic on the engines, not a window being held.
    _AT_ERR_KT = 15.0
    _AT_ERR_S = 15.0

    def _nav_fallback_heading(self, state, now: float) -> float:
        """A heading to fly where the armed NAV is tracking nothing, -1 else.

        The airborne phases arm the simulator's NAV and trust the plan
        behind it; nothing else checks the plan produced a course, and a
        NAV with no course leaves the aeroplane on whatever heading it
        last had -- N45XS flew its SID leg for twelve minutes into
        terrain on a diverted plan the box never steered. Watched
        against the bearing to the fix the active leg runs to: pointed
        anywhere near it is tracking, persistently not is not -- and
        the fallback is that heading, engaged as the HDG mode's number
        until the track comes round and NAV can be trusted again.

        And where the plan cannot be placed at all -- progress None,
        which is what the box's stale diverted plan looks like -- the
        bearing it is checked against is the destination's, because
        pointed somewhere is still better than pointed at the last
        stale course.
        """
        route = getattr(getattr(self, "brain", None), "route", None)
        session = getattr(self, "session", None)
        progress = None
        if route is not None and getattr(state, "has_position", False):
            try:
                progress = route.progress(
                    state.latitude, state.longitude,
                    float(getattr(state, "ground_speed_kt", 0.0) or 0.0),
                    hint=getattr(session, "route_direct_index", -1))
            except Exception:
                progress = None
        want = None
        if progress is not None:
            fix = progress.to_fix
            want = state.bearing_to(fix.lat, fix.lon)
        else:
            # The route cannot be placed -- the box's plan is the thing
            # steering now, and the check that remains is whether the
            # track is going where the flight goes at all.
            destination = getattr(session, "destination", None)
            if destination is not None \
                    and getattr(state, "has_position", False):
                try:
                    want = state.bearing_to(destination.lat,
                                            destination.lon)
                except Exception:
                    want = None
        if want is None:
            self._nav_off_since = 0.0
            return -1.0
        have = float(getattr(state, "track_true", 0.0) or 0.0) \
            or float(getattr(state, "heading_true", 0.0) or 0.0)
        err = abs((want - have + 180.0) % 360.0 - 180.0)
        if err <= self.NAV_OFF_DEG:
            self._nav_off_since = 0.0
            return -1.0
        if not getattr(self, "_nav_off_since", 0.0):
            self._nav_off_since = now
            return -1.0
        if now - self._nav_off_since < self.NAV_OFF_S:
            return -1.0
        # The bug it is handed to reads magnetic; the bearing is true,
        # like the runway course the go-around converts the same way.
        return self._magnetic(want, state)

    def _own_nav_heading(self, state) -> float:
        """The magnetic bearing to the fix the active leg runs to, -1 else.

        The same aim the NAV watchdog checks its bearing against, taken
        as the steering itself: where the box's NAV cannot be trusted
        with the plan the heading bug carries this instead, refreshed
        every tick the way the procedure legs are already flown. The
        destination's bearing answers where the plan cannot be placed
        at all, for the same reason the fallback keeps it.
        """
        route = getattr(getattr(self, "brain", None), "route", None)
        session = getattr(self, "session", None)
        want = None
        if route is not None and getattr(state, "has_position", False):
            try:
                progress = route.progress(
                    state.latitude, state.longitude,
                    float(getattr(state, "ground_speed_kt", 0.0) or 0.0),
                    hint=getattr(session, "route_direct_index", -1))
            except Exception:
                progress = None
            if progress is not None:
                fix = progress.to_fix
                want = state.bearing_to(fix.lat, fix.lon)
        if want is None:
            destination = getattr(session, "destination", None)
            if destination is not None \
                    and getattr(state, "has_position", False):
                try:
                    want = state.bearing_to(destination.lat,
                                            destination.lon)
                except Exception:
                    want = None
        if want is None:
            return -1.0
        return self._magnetic(want, state)

    def _route_lateral(self, state, now=None) -> float:
        """The heading the director flies where the route steers, -1 NAV.

        Where the sim's own NAV can be trusted with the plan --
        ``box_nav`` -- it is armed first and the watchdog's fallback is
        the heading handed back, so the course wins the engagement the
        way the climb has always meant it. X-Plane's FMS is not that
        NAV: a plan it cannot load is dropped without a word and one it
        can is followed on an active leg it picks itself and will not
        always sequence, so the steering is the bearing to the active
        fix on the bug instead. With no fix and no destination to aim
        at, the heading the wings already have is held -- a course that
        was never steering is no better than the one the compass reads.
        """
        if getattr(self.sim, "box_nav", True):
            self.director.nav(True)
            self.state.ap_nav_mode = True
            return self._nav_fallback_heading(
                state, now if now is not None else time.monotonic())
        own = self._own_nav_heading(state)
        if own >= 0.0:
            return own
        hdg = float(getattr(state, "heading_true", 0.0) or 0.0)
        return self._magnetic(hdg, state) if hdg else -1.0

    def _vector_fms(self, state) -> None:
        """A vectored plan, told to the FMS once per heading.

        Flying the bug is only half of a vector: the box still wants the
        fix the heading bypassed, and a "resume own navigation" later
        would steer back at a point already behind -- so the plan in the
        FMS is rewritten the way a crew loads it, present position direct
        the leg the vector rejoins, and its marker cleared when ATC hands
        the route back.
        """
        vector = getattr(self.session, "assigned_heading", None)
        if vector is None:
            self._vector_fms_hdg = None
            return
        if getattr(self, "_vector_fms_hdg", None) == vector:
            return
        self._vector_fms_hdg = vector
        sim = getattr(self, "sim", None)
        loader = getattr(sim, "load_fms_plan", None) \
            if sim is not None else None
        if loader is None \
                or not getattr(state, "has_position", False) \
                or getattr(state, "on_ground", False):
            return
        route = getattr(self.brain, "route", None)
        fixes = tuple(getattr(route, "fixes", ()) or ())
        if len(fixes) < 3:
            return
        try:
            progress = route.progress(
                state.latitude, state.longitude,
                float(getattr(state, "ground_speed_kt", 0.0) or 0.0),
                hint=getattr(self.session, "route_direct_index", -1))
        except Exception:
            progress = None
        if progress is None:
            return
        trimmed = self._fms_direct_route(state, route)
        if trimmed is not None and loader(trimmed):
            # The next status read carries this plan back: marked ours,
            # the adoption pass leaves the live session alone.
            sim._fms_written = True

    def _fms_direct_route(self, state, route):
        """The plan as a direct from here to the leg being flown.

        Departure field, present position, then the route's active fix and
        everything after it: what the box holds is the same flight, only
        drawn from where the aeroplane is instead of where it was --
        which is what keeps a fresh plan from steering back over the
        field to intercept a course line that starts there.
        """
        fixes = tuple(getattr(route, "fixes", ()) or ())
        if len(fixes) < 3:
            return None
        try:
            progress = route.progress(
                state.latitude, state.longitude,
                float(getattr(state, "ground_speed_kt", 0.0) or 0.0),
                hint=getattr(self.session, "route_direct_index", -1))
        except Exception:
            progress = None
        if progress is None:
            return None
        from ..route import Route, RouteFix
        ppos = RouteFix(ident="PPOS", lat=state.latitude,
                        lon=state.longitude, kind="wpt")
        return Route(fixes=tuple([fixes[0], ppos]
                                 + list(fixes[progress.leg:])),
                     origin=route.origin,
                     destination=route.destination,
                     cruise_ft=route.cruise_ft, source="sim")

    def _fms_departure_direct(self, state) -> None:
        """Straight out after takeoff: the first fix, not back overhead.

        The box's course line starts at the departure field: NAV armed
        on it fresh out of the roll intercepts that line by returning
        over the airport. A direct from present position to the first
        fix is the same plan as the box flies it -- written once, when
        the climb's NAV first goes on.
        """
        if getattr(self, "_fms_direct_done", False):
            return
        self._fms_direct_done = True
        sim = getattr(self, "sim", None)
        loader = getattr(sim, "load_fms_plan", None) \
            if sim is not None else None
        if loader is None \
                or not getattr(state, "has_position", False) \
                or getattr(state, "on_ground", False):
            return
        route = getattr(self.brain, "route", None)
        if route is None:
            return
        trimmed = self._fms_direct_route(state, route)
        if trimmed is not None and loader(trimmed):
            sim._fms_written = True

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

    @staticmethod
    def _flown_altitude_ft(state) -> float:
        """The level the altimeter reads -- what a clearance is flown to.

        ``altitude_ft`` is true MSL; the pilot flies the Kollsman window,
        and the radar's corrected mode-C agrees with it. On a QNH-1018
        day the two are a hundred-plus feet apart, and a level held on
        the true figure is the wrong one on every gauge that matters.
        """
        return float(getattr(state, "indicated_altitude_ft", 0.0)
                     or getattr(state, "altitude_ft", 0.0) or 0.0)

    def _altitude_target(self, state) -> float:
        """The level to fly now: what ATC last assigned, else the plan's.

        Order matters: a clearance beats the filed figure, and the
        route's own cruise -- what the paperwork actually filed, kept
        by ``use_route`` -- answers where neither the session's fields
        nor a controller has one, so a SimBrief-level plan is flown to
        its filed level rather than the pre-flight guess the session
        was seated with.
        """
        route = getattr(getattr(self, "brain", None), "route", None)
        filed = float(getattr(route, "cruise_ft", 0.0) or 0.0)
        if self.session.assigned_altitude_ft:
            return self.session.assigned_altitude_ft
        return self._under_vfr_ceiling(
            self.session.cruise_altitude_ft
            or filed
            or self.session.initial_altitude_ft or 0.0, state)

    # How far from the field its "not above" still binds: the reach of
    # the widest shelf a tower's ceiling is drawn under.
    VFR_CEILING_REACH_NM = 30.0

    def _under_vfr_ceiling(self, target: float, state) -> float:
        """``target``, kept at or below the VFR ceiling ATC gave.

        "Not above two thousand five hundred" was read back and the
        climb carried on to the filed level, through the airspace the
        figure exists to keep the flight under. It binds a VFR flight
        until the controller lifts it or the field is well behind.
        """
        session = self.session
        ceiling = float(getattr(session, "vfr_ceiling_ft", 0.0) or 0.0)
        if not ceiling or not target or getattr(session, "ifr", True):
            return target
        field = getattr(session, "departure", None)
        if field is not None and getattr(state, "has_position", False):
            try:
                if state.distance_to(field.lat, field.lon)                         > self.VFR_CEILING_REACH_NM:
                    return target
            except Exception:
                pass
        return min(target, ceiling)

    def _route_level(self, state) -> tuple[float, str]:
        """(level, stage) the plan's next fix is filed at, else 0."""
        level, _dist, stage = self._route_level_ahead(state)
        return level, stage

    def _route_level_ahead(self, state) -> tuple[float, float, str]:
        """(level, nm, stage) the plan's next fix is filed at, else 0.

        A flight plan's vertical profile is a chain of crossing levels:
        the first fix still ahead of the aeroplane that files one is the
        one that constrains it. Markers file nothing by design, and a
        route that levels nothing answers 0 -- unchanged behaviour --
        rather than a constraint made up. The miles are measured along
        the legs, the distance that is actually still to fly: a STAR
        that doglegs past the field it started abeam does not file a
        shorter path than the aeroplane has left.
        """
        brain = getattr(self, "brain", None)
        route = getattr(brain, "route", None)
        if route is None:
            return 0.0, 0.0, ""
        progress = getattr(brain, "route_progress", None)
        if progress is None and getattr(state, "has_position", False):
            try:
                progress = route.progress(
                    state.latitude, state.longitude,
                    float(getattr(state, "ground_speed_kt", 0.0) or 0.0))
            except Exception:
                progress = None
        if progress is None:
            return 0.0, 0.0, ""
        for idx in range(progress.leg, len(route.fixes)):
            fix = route.fixes[idx]
            if not fix.is_marker and fix.alt_ft > 0:
                dist = progress.to_next_nm \
                    + route._along(progress.leg, idx)
                return float(fix.alt_ft), max(0.0, dist), fix.stage or ""
        return 0.0, 0.0, ""

    def _penetration_kt(self, state) -> float:
        """The speed rough air is ridden out at: a margin off the red
        line and off the stall, the way the books write it."""
        cruise = self._cruise_speed_kt(state) or 200.0
        lim = self._limits(state)
        pen = cruise * 0.75
        if lim["stall_kt"] > 0.0:
            pen = max(pen, lim["stall_kt"] * 1.5)
        return max(60.0, min(pen, lim["max_ias_kt"]))

    def _cruise_speed_kt(self, state) -> float:
        """The cruise figure this leg is flown and priced at: the model's
        own estimate where it publishes one (MSFS does, X-Plane does
        not), else the better of the type's Vno pulled back to a cruise
        power setting and the performance figure -- a matched autofly
        profile carries the POH's number, and a model with none still
        flies at its own speed rather than the class's.
        """
        estimated = float(getattr(state, "estimated_cruise_kt", 0.0)
                          or 0.0)
        if 50.0 < estimated < 600.0:
            return estimated
        own = 0.75 * float(getattr(state, "design_vc_kt", 0.0) or 0.0)
        return max(own, self.performance.cruise_kt) \
            or self.state.climb_kt

    def _flap_up_kt(self) -> float:
        """The airspeed the takeoff flap comes in at.

        V2+20 is the transport schedule -- the second-segment figure a
        jet's own climb speed already exceeds. A light aeroplane's book
        climb sits under it -- a Cessna profile climbs at 74 where the
        gate asks 75 -- and the flap's own drag is half of why it never
        gets there, so a gate the climb cannot reach leaves the flap out
        for the leg. Its retract number is the climb figure less the
        slack a dirty wing still owes, floored just over rotation.
        """
        gate = (self.state.v2_kt or self.state.vr_kt or 0.0) + 20.0
        climb = float(getattr(self.performance, "climb_kt", 0.0) or 0.0)
        if climb and gate > climb:
            gate = max((self.state.vr_kt or 0.0) + 5.0, climb - 15.0)
        return gate

    def _speed_target(self, state) -> float:
        target = 0.0
        if self.session.assigned_speed_kt:
            target = self.session.assigned_speed_kt
        elif self.state.phase == AutonomousPhase.CRUISE:
            target = self._cruise_speed_kt(state)
            if target:
                # The estimate may be true rather than indicated speed;
                # never command past the model's own red line.
                redline = float(getattr(state, "barber_pole_kt", 0.0)
                                or 0.0)
                if redline > 0:
                    target = min(target, redline * 0.95)
        elif self.state.phase == AutonomousPhase.CLIMB:
            climb = self.state.climb_kt or self.performance.cruise_kt
            agl = float(getattr(state, "altitude_agl_ft", 0.0) or 0.0)
            cruise = self.performance.cruise_kt or climb
            if agl > 1500.0 and cruise > climb:
                # Vy is for the ground's reach: past the noise height a
                # crew accelerates to the enroute climb, a share of the
                # way from the book figure to the cruise -- the POH's
                # "cruise climb" column the single figure cannot hold.
                climb += 0.35 * (cruise - climb)
            target = climb
        if not target:
            # Any other phase: the climb figure was first here, so a
            # descent with no answer of its own came down at Vy -- the
            # same slow-leg bug the cruise fallback had. CLIMB is the
            # only phase the climb speed belongs to, and it has its own
            # branch above; the rest of flight gets the descent-profile
            # figure, part way back from cruise the way a crew flies it.
            target = 0.85 * self._cruise_speed_kt(state)
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
            clean_stall = float(getattr(state, "design_vs1_kt", 0.0)
                                or lim["stall_kt"] or 0.0)
            glide = float(getattr(self.performance, "best_glide_kt", 0.0)
                          or 0.0)
            if glide <= 0.0:
                glide = max(clean_stall * 1.4,
                            self.performance.approach_kt)
            return self._safe_speed(state, glide, lim)
        if self.state.emergency == "engine-out":
            return min(target, self.state.climb_kt
                       or self.performance.climb_kt)
        return target

    def _apply_flight_director(self, state, altitude: float = 0.0,
                             vs_fpm: float = 0.0, heading: float = -1.0,
                             speed_kt: float = 0.0,
                             prefer_vs: bool = False) -> None:
        """Put the assigned numbers on the autopilot, managed or direct.

        ``heading`` < 0 means "not commanded"; ``vs_fpm`` nonzero means a
        rate is wanted (climb or descent in progress); zero means level.
        ``prefer_vs`` asks the managed autopilot to fly the leg on the
        vertical-speed hold rather than level-change -- a continuous
        descent is pitch-for-rate, the way VNAV rides a path, where
        level-change's pitch-for-speed can only dive for a floor.
        """
        d = self.director
        self._vector_fms(state)
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
        avoid = self._avoid_heading_bug(state)
        if avoid >= 0.0:
            # See-and-avoid outranks the phase's own steering -- and an
            # assigned one -- for the seconds the threat is ahead: the
            # avoidance heading is what is flown, and the clearance is
            # picked back up when the meeting is past.
            heading = avoid
        if heading >= 0.0:
            d.heading(heading)
            self.state.ap_hdg_mode = True
        if speed_kt:
            speed_kt = self._safe_speed(state, speed_kt, lim)
            d.speed(speed_kt)
            self.state.ap_speed_mode = True
            self._manage_throttle(state, speed_kt)
        now = time.monotonic()
        flc = False
        # The demand, not the bug: the slew opens at zero on its first
        # tick, and a mode released on the slewed figure is released in
        # the same breath it was asked for.
        vs_demanded = bool(vs_fpm)
        if vs_fpm:
            vs_fpm = max(lim["vs_dn_fpm"], min(float(vs_fpm),
                                             lim["vs_up_fpm"]))
            vs_fpm = self._climb_within_speed(state, vs_fpm, lim)
            # The bug is slewed to the demand, not stepped: the AP's
            # pitch answer follows a ramp instead of a stair.
            vs_fpm = self._vs_slew(state, vs_fpm, now)
            d.vertical_speed(vs_fpm)
            if self._ap_mode == "managed" and self.state.phase in (
                    AutonomousPhase.CLIMB, AutonomousPhase.DESCENT,
                    AutonomousPhase.CRUISE):
                flc = self._flc_owning(state, now, prefer_vs)
            # One vertical mode at a time: where level-change has been
            # confirmed on the axis the VS hold stays out of it -- the
            # two are exclusive, and both asserted is the autopilot
            # flipping FL and VS a write apart, the flicker a descent
            # flew the whole way down. ``None`` is the probe still out:
            # VS is held off so it cannot knock the asked-for mode back
            # off before its status answers.
            d.vs_hold(flc is False)
            self.state.ap_vs_mode = flc is False
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
        cap = 15.0 if (self.state.g_limited
                       or now < getattr(self.state, "turbulence_until",
                                        0.0)) \
            else _BANK_CAP_DEG.get(demeanor, 20.0)
        own = float(getattr(self.performance, "max_bank_deg", 0.0) or 0.0)
        if own > 0.0:
            cap = min(cap, own)
        self._cap_bank(cap)
        if self._ap_mode == "managed" and not vs_demanded \
                and self.state.phase in (AutonomousPhase.CLIMB,
                                         AutonomousPhase.DESCENT,
                                         AutonomousPhase.CRUISE):
            # Managed legs climb and descend on the flight-level-change
            # mode where the model has it -- a step inside the cruise
            # too, the way the real mode flies a small level change on
            # proportional thrust rather than the VS bug (Boeing FLCH).
            # It is OFF the moment level is captured: latched on, it
            # holds the levers at the climb/descent extreme it was
            # flying and the speed decays toward the stall before the
            # holds pull it back.
            d.level_change(False)
            self._flc_asked_at = 0.0

    # Where the mode's own status is readable, this long is an engage
    # asked for and never taken -- the model has no level-change, and
    # asking again every tick knocks the VS hold it does have off the
    # axis a write at a time.
    _FLC_PROBE_S = 2.0

    def _flc_owning(self, state, now: float,
                    prefer_vs: bool = False) -> bool:
        """Whether flight-level-change owns the vertical axis this tick.

        ``True`` -- the mode is read back engaged and owns the axis;
        ``None`` -- the engage is still out and nothing else may assert
        the axis while its answer is waited on; ``False`` -- the VS
        hold is the vertical mode to fly, either because the model
        showed it has no level-change or because there is no source to
        read the mode back from at all (the legacy both-asked ordering,
        where the last write wins on models that have the mode and VS
        simply persists on those that do not). ``prefer_vs`` asks for
        the VS hold even where the mode exists -- a flown path wants
        pitch-for-rate the whole way down, not a speed the floor
        answers early and levels for.
        """
        sim = getattr(getattr(self, "director", None), "sim", None)
        read = getattr(sim, "value", None)
        if read is None:
            # No source or no readback on it at all: the legacy
            # both-asked ordering -- FLC where the model takes it, VS
            # asserted alongside and winning where it does not.
            self.director.level_change(True)
            return False
        xp = getattr(sim, "flc_pitch_only", False) is True
        engaged_at = getattr(sim, "ap_status_engaged", 1.0)
        if not isinstance(engaged_at, (int, float)) or engaged_at <= 0.0:
            engaged_at = 1.0
        if xp or prefer_vs:
            # X-Plane's level-change pitches for speed -- and so does
            # the throttle manager: two controllers on one airspeed is
            # a phugoid, and the descent leg that flew it porpoised the
            # aeroplane into the ground. A continuous descent asks for
            # the same pairing on any sim: pitch for rate (the VS bug)
            # with thrust for speed is the decoupled way a path is
            # held, so VS owns the axis here outright. A mode left on
            # is brought off under the same two-second probe, reversed,
            # before VS asserts -- expired unanswered, VS stays latched
            # rather than pulsing the axis every other tick.
            if not getattr(self, "_flc_vs", False):
                if (read("control:speed_status") or 0.0) >= engaged_at:
                    self._flc_off_at = \
                        getattr(self, "_flc_off_at", 0.0) or now
                    self.director.level_change(False)
                    if now - self._flc_off_at <= self._FLC_PROBE_S:
                        return None
                self._flc_off_at = 0.0
                self._flc_vs = True
            return False
        self._flc_off_at = 0.0
        self._flc_vs = False
        if getattr(self, "_flc_refused", False):
            return False
        engaged = bool((read("control:speed_status") or 0.0)
                       >= engaged_at)
        if engaged:
            # Keep it up -- the mode write is idempotent once engaged;
            # a later drop re-opens the probe below on a fresh clock.
            self._flc_asked_at = 0.0
            self.director.level_change(True)
            return True
        # Still asking: the answer lands on the next reads or it does
        # not. While the probe is out the VS hold stays OFF the axis --
        # asserted alongside, its engage knocks the level-change off a
        # write later and the probe can never see the mode it asked
        # for. A model that never shows the mode is called refused.
        self._flc_asked_at = getattr(self, "_flc_asked_at", 0.0) or now
        self.director.level_change(True)
        if now - self._flc_asked_at > self._FLC_PROBE_S:
            self._flc_refused = True
            return False
        return None

    def _climb_within_speed(self, state, vs_fpm: float, lim) -> float:
        """A climb rate the aeroplane has the airspeed for.

        A vertical-speed mode holds its rate and lets the speed pay for
        it: hot, heavy or high, the book rate is more than the engine
        has, the airspeed bleeds a knot at a time and the autopilot
        pitches up after it into the stall. A crew watches the speed
        and gives the rate away first -- all of it by fifteen per cent
        over the stall, and below that the nose comes down for speed.
        """
        if vs_fpm <= 0.0 or getattr(state, "on_ground", False):
            return vs_fpm
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        # Clean, it is the clean stall that counts; a model that
        # publishes neither is given its approach speed's (1.3 Vs0).
        stall = float(lim.get("stall_kt") or 0.0)
        if getattr(self, "_flaps_asked", 0.0) <= 0.0:
            stall = float(getattr(state, "design_vs1_kt", 0.0) or stall)
        if stall <= 0.0:
            stall = float(self.performance.approach_kt or 0.0) / 1.3
        if ias <= 0.0 or stall <= 0.0:
            return vs_fpm
        floor = stall * 1.15
        margin = max(8.0, stall * 0.15)
        if ias >= floor + margin:
            self._climb_speed_warned = False
            return vs_fpm
        if not getattr(self, "_climb_speed_warned", False):
            self._climb_speed_warned = True
            self._emit("info",
                       f"Climb rate reduced: {ias:.0f} kt is close to "
                       "the stall", "autonomous.limits.climb_speed")
        if ias <= floor:
            return -300.0
        return max(100.0, vs_fpm * (ias - floor) / margin)

    def _manage_throttle(self, state, target_kt: float) -> None:
        """The FO's own throttle, for the aeroplanes with no autothrottle.

        On a model whose autopilot cannot hold speed the lever stays
        where takeoff or the go-around left it -- full power all the way
        to the red line. This servo moves the levers only when the speed
        is genuinely off target, so a working autothrottle, which keeps
        the speed inside the deadband on its own, is never fought.
        """
        if not target_kt or state.on_ground or self._all_engines_out(state):
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
        # A live autothrottle owns the levers: the servo's job only
        # exists where there is none, and two hands on one lever is a
        # porpoise. Owns them while it holds the bug, that is: a model
        # whose autothrottle drives its own target -- a King Air holding
        # max-continuous power to 255 knots against a 210 window -- is
        # the throttle's lever read at whatever it likes while the
        # engines never hear it. The speed sitting over the figure it
        # was given, for longer than a change or a gust takes, is the
        # call that the thing on the levers is not an autothrottle: the
        # disconnect goes once, the flag is distrusted while it stays
        # up, and the levers come back to this servo.
        if getattr(state, "autothrottle", False) \
                and not getattr(self, "_at_distrusted", False):
            if ias > target_kt + self._AT_ERR_KT:
                now = time.monotonic()
                since = getattr(self, "_at_bad_since", 0.0) or now
                self._at_bad_since = since
                if now - since < self._AT_ERR_S:
                    return
                self._at_distrusted = True
                self._at_bad_since = 0.0
                self.director.autothrottle_disconnect()
                self._emit(
                    "info",
                    "Autothrottle not holding the speed "
                    f"({ias:.0f} kt on a {target_kt:.0f} bug): "
                    "taking the levers",
                    "autonomous.at")
            else:
                self._at_bad_since = 0.0
                return
        elif not getattr(state, "autothrottle", False):
            self._at_distrusted = False
            self._at_bad_since = 0.0
        lever = self._lever_position(state)
        # The lever the sim reports is the only position a slew can
        # start from: seeded anywhere else, the first write of a pass
        # is the jump the rate limit exists to prevent.
        self._last_lever = lever
        # A climb that still owes height keeps climb power: the speed is
        # the pitch's and the bug's to mind up there -- a servo that
        # idles the engine six hundred feet up for being ten knots fast
        # is the mush into the trees off the end.
        climb_floor = 0.0
        if self.state.phase == AutonomousPhase.CLIMB:
            target_alt = self._altitude_target(state)
            if target_alt and float(getattr(state, "altitude_ft", 0.0)
                                    or 0.0) < target_alt - 200.0:
                climb_floor = 0.90
        # And the mirror: a descent that still owes height is flown at
        # descent power -- near idle for a light aeroplane -- the pitch
        # holding the rate and the lever the speed. Left where the
        # climb set it, the lever stayed at ninety-odd percent down the
        # whole descent: speed on target meant the servo never moved,
        # and the shallow powered descent ended in a chopped dive.
        descent_cap = 0.0
        if self.state.phase == AutonomousPhase.DESCENT:
            target_alt = self._altitude_target(state)
            if target_alt and self._flown_altitude_ft(state) \
                    > target_alt + 250.0 \
                    and ias > target_kt - 5.0:
                descent_cap = 0.20
        barber = float(getattr(state, "barber_pole_kt", 0.0) or 0.0)
        max_mach = float(getattr(state, "mach_max_operate", 0.0)
                         or getattr(state, "barber_pole_mach", 0.0) or 0.0)
        mach = float(getattr(state, "airspeed_mach", 0.0) or 0.0)
        overspeed = (barber > 0.0 and ias >= barber * 0.97) \
            or (max_mach > 0.0 and mach >= max_mach - 0.005)
        if overspeed:
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
                lever -= min(0.06, 0.02 + err * 0.003)
            elif err < -15.0:
                lever += min(0.06, 0.02 + (-err) * 0.003)
            elif self._engine_limit_cap(state) >= 1.0 \
                    and (not descent_cap or lever <= descent_cap):
                # On target and no gauge over its red line: nothing to
                # move -- unless a descent owes the lever back, which a
                # deadband return left riding at climb power forever.
                return
        lever = max(0.05, min(0.97, lever))
        # The engine watch's ceiling: a gauge past its published red line
        # gets power off, whatever the speed loop wanted.
        lever = min(lever, self._engine_limit_cap(state))
        if descent_cap:
            lever = min(lever, descent_cap)
        if not overspeed:
            lever = max(lever, climb_floor)
        # The servo moves the lever at a hand's speed like the takeoff
        # ramp does: a step to where the speed wants it is a slammed
        # fuel unit, and a turbine answers the slam with the EGT spike
        # the limit watch reads straight back as an exceedance.
        self.director.throttle(self._slew_lever(lever, up_per_s=0.30))

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
            if not getattr(self, "_traffic_read_warned", False):
                self._traffic_read_warned = True
                log.warning("could not read the traffic picture",
                            exc_info=True)
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
            if not getattr(self, "_runway_check_warned", False):
                self._runway_check_warned = True
                log.warning("the runway occupancy check failed",
                            exc_info=True)
            return False

    # How far out an arrival still closes the runway for a "line up and
    # wait": the couple of miles where the landing is seconds away and
    # the bars are where a crew stays. Past it the controller is telling
    # the arrival what to do, not the departure where to sit.
    LINEUP_FINAL_NM = 2.5

    def _runway_inbound(self, runway: str = "", airport=None) -> bool:
        """Whether an arrival has the strip next, threshold's view.

        "Line up and wait behind landing traffic" is not on the runway
        -- it is on final, inside the cone, and ``_runway_blocked``
        cannot see it until the wheels touch. This is the part of the
        instruction that says wait: an aeroplane in the final cone
        inside ``LINEUP_FINAL_NM`` of the threshold.
        """
        try:
            session = self.session
            name = runway or getattr(session, "departure_runway", "") \
                or (self.state.runway_key[1] if self.state.runway_key else "")
            airport = airport or getattr(session, "departure", None) \
                or getattr(session, "destination", None) \
                or self._get_current_airport()
            if not name or airport is None:
                return False
            from .. import occupancy
            return bool(occupancy.on_final(
                self.navdb, airport, name, self._surface_contacts(),
                within_nm=self.LINEUP_FINAL_NM))
        except Exception:
            if not getattr(self, "_final_check_warned", False):
                self._final_check_warned = True
                log.warning("the final-approach check failed",
                            exc_info=True)
            return False

    # How far ahead the FO looks for somebody in the way, how close a
    # meeting counts as a conflict, and the turn made about it -- the
    # see-and-avoid a crew owes the aeroplane nobody is separating it
    # from. The figures are a crew's, not a centre's: it is the pilot's
    # own eyes and the few seconds a turn still changes something.
    TRAFFIC_LOOKAHEAD_S = 45.0
    TRAFFIC_CONFLICT_NM = 0.6
    TRAFFIC_CONFLICT_FT = 400.0
    TRAFFIC_AVOID_TURN_DEG = 30.0
    TRAFFIC_HEAD_ON_DEG = 20.0
    TRAFFIC_AVOID_S = 30.0
    # The turn is made once and held, not lived on: a meeting a
    # geometry keeps alive past this long is taken as a fresh look --
    # the watch ends the deviation and, if the other aeroplane is still
    # a conflict, works out a new turn rather than an endless vector.
    TRAFFIC_AVOID_MAX_S = 180.0

    def _watch_traffic(self, state, now: float) -> None:
        """See and avoid: look ahead for a meeting nobody has called.

        The phases it runs in are the enroute ones: the final has its
        own answers -- sequencing, and the go-around for somebody on
        the runway -- and the surface has the give-way. Out here the
        answer is the one the rules of the air give a pilot: a meeting
        is turned away from, head-on to the right, and the route is
        picked up again once the other aeroplane is opening.

        The turn is made once and held. The question asked while it is
        held is whether the track the aeroplane left would still meet
        the other one -- not whether the avoidance track does, which is
        "no" the moment the turn works and would hand the aeroplane
        straight back to the conflict.
        """
        if self.state.phase not in (AutonomousPhase.CLIMB,
                                    AutonomousPhase.CRUISE,
                                    AutonomousPhase.DESCENT) \
                or not getattr(state, "has_position", False) \
                or state.on_ground:
            self._end_avoidance()
            return
        track = getattr(state, "track_true", None) \
            or getattr(state, "heading_true", None) or 0.0
        if not track:
            return
        own_speed = float(getattr(state, "ground_speed_kt", 0.0) or 0.0)
        if own_speed < 40.0:
            return

        from ...navdata.build import bearing_deg

        contacts = [c for c in self._surface_contacts() if not c.on_ground]
        avoiding = self.state.avoid_key
        if avoiding and now - self.state.avoid_since \
                > self.TRAFFIC_AVOID_MAX_S:
            self._end_avoidance()
            avoiding = ""
        # The one already turned away from is judged first: a turn that
        # still meets it is held rather than swapped for whichever other
        # contact lists next -- two standing conflicts must not alternate
        # the turn's direction on every look.
        contacts.sort(key=lambda c: (c.written or "traffic") != avoiding)
        threat = next((c for c in contacts
                       if self._meets(state, c, track, own_speed)), None)
        if threat is None and avoiding:
            # Clear on the avoidance track. Whether the one turned away
            # from is past is asked of the track that was left.
            left = self.state.avoid_from
            if left < 0.0:
                left = track
            if any((c.written or "traffic") == avoiding
                   and self._meets(state, c, left, own_speed)
                   for c in contacts):
                self.state.avoid_until = now + self.TRAFFIC_AVOID_S
                return
        if threat is None:
            # The meeting is past: back on the route's own heading.
            self._end_avoidance()
            return

        key = threat.written or "traffic"
        if key == avoiding and self.state.avoid_heading >= 0.0:
            # Still closing on the one already turned away from: the
            # turn stands, it is not added to on every look.
            self.state.avoid_until = now + self.TRAFFIC_AVOID_S
            return
        bearing = bearing_deg(state.latitude, state.longitude,
                              threat.latitude, threat.longitude)
        off = (bearing - track + 540.0) % 360.0 - 180.0
        # Head-on is a right turn -- the rules of the air's answer, and
        # "head-on" is a few degrees either side of the nose; anything
        # else is a turn away from the side it is on.
        side = 1.0 if off <= self.TRAFFIC_HEAD_ON_DEG else -1.0
        if not avoiding:
            self.state.avoid_from = track
        self.state.avoid_heading = (track + side
                                    * self.TRAFFIC_AVOID_TURN_DEG) % 360.0
        self.state.avoid_until = now + self.TRAFFIC_AVOID_S
        self.state.avoid_key = key
        self.state.avoid_since = now
        self._emit(
            "info",
            f"Traffic {key}: turning {('right' if side > 0 else 'left')}"
            " to avoid", "autonomous.traffic_avoid")

    def _meets(self, state, contact, track: float, own_speed: float) -> bool:
        """Whether flying ``track`` from here meets this contact."""
        from ..trafficinfo import closest_approach
        try:
            miss, t, vgap = closest_approach(
                state.latitude, state.longitude,
                float(state.altitude_ft or 0.0), track, own_speed,
                contact.latitude, contact.longitude,
                float(contact.altitude_ft or 0.0),
                float(contact.heading or 0.0),
                float(contact.ground_speed_kt or 0.0),
                own_vs_fpm=float(
                    getattr(state, "vertical_speed_fpm", 0.0) or 0.0),
                vs_fpm=float(contact.vertical_rate_fpm or 0.0),
                horizon_s=self.TRAFFIC_LOOKAHEAD_S)
        except Exception:
            if not getattr(self, "_avoid_check_warned", False):
                self._avoid_check_warned = True
                log.warning("the see-and-avoid check failed",
                            exc_info=True)
            return False
        return (t > 0.0 and miss < self.TRAFFIC_CONFLICT_NM
                and vgap < self.TRAFFIC_CONFLICT_FT)

    def _end_avoidance(self) -> None:
        self.state.avoid_key = ""
        self.state.avoid_heading = -1.0
        self.state.avoid_from = -1.0
        self.state.avoid_until = 0.0
        self.state.avoid_since = 0.0

    def _avoid_heading_bug(self, state) -> float:
        """The see-and-avoid heading as the bug wants it (magnetic), or
        -1 when no avoidance stands. The watch keeps it as a true track,
        the frame the geometry is worked in."""
        if self.state.avoid_until <= time.monotonic() \
                or self.state.avoid_heading < 0.0:
            return -1.0
        return self._magnetic(self.state.avoid_heading, state)

    # Over the fix: this close is "crossed" to a crew timing a hold by
    # hand; and the turns are to the right, the standard pattern the
    # controller's words give.
    HOLD_OVER_FIX_NM = 1.0
    HOLD_PASSED_NM = 3.0

    def _fly_hold(self, state, now: float) -> None:
        """Fly the hold the controller gave: to the fix, then the
        racetrack -- outbound for the leg's time, a right turn back to
        the fix, and round again until it is cleared to leave.

        Nothing flew it before: the hold was read back and the route
        flown straight on past the fix, into the "verify holding" the
        controller keeps for a flight that wandered off the pattern.
        It is the last lateral word of the tick but one -- only the
        see-and-avoid turn outranks it.
        """
        st = self.state
        enroute = st.phase in (AutonomousPhase.CLIMB, AutonomousPhase.CRUISE,
                               AutonomousPhase.DESCENT)
        hold = getattr(getattr(self, "session", None), "hold", None) \
            if enroute else None
        if hold is None \
                or not getattr(state, "has_position", False) \
                or state.on_ground:
            st.hold_fix, st.hold_leg = "", ""
            st.hold_leg_at, st.hold_nearest_nm = 0.0, 0.0
            return

        from ...navdata.build import bearing_deg, haversine_nm

        if st.hold_fix != hold.fix or not st.hold_leg:
            st.hold_fix, st.hold_leg = hold.fix, "to_fix"
            st.hold_leg_at, st.hold_nearest_nm = 0.0, 0.0
        distance = haversine_nm(state.latitude, state.longitude,
                                hold.lat, hold.lon)
        to_fix = bearing_deg(state.latitude, state.longitude,
                             hold.lat, hold.lon)
        track = float(getattr(state, "track_true", None)
                      or getattr(state, "heading_true", None) or 0.0)
        outbound = (hold.inbound_true + 180.0) % 360.0

        if st.hold_leg in ("to_fix", "inbound"):
            nearest = st.hold_nearest_nm or distance
            st.hold_nearest_nm = min(nearest, distance)
            passed = (st.hold_nearest_nm <= self.HOLD_PASSED_NM
                      and distance > st.hold_nearest_nm + 0.5)
            if distance <= self.HOLD_OVER_FIX_NM or passed:
                st.hold_leg, st.hold_leg_at = "outbound", 0.0
                st.hold_nearest_nm = 0.0
        want = to_fix
        if st.hold_leg == "outbound":
            want = outbound
            off = abs((track - outbound + 540.0) % 360.0 - 180.0)
            if not st.hold_leg_at and off < 30.0:
                # The leg is timed from wings level outbound.
                st.hold_leg_at = now
            if st.hold_leg_at and now - st.hold_leg_at \
                    >= float(hold.leg_minutes or 1.0) * 60.0:
                st.hold_leg, want = "inbound", to_fix
                st.hold_nearest_nm = 0.0
        if st.hold_leg != "to_fix":
            # A heading bug turns the short way round; the hold turns in
            # its own direction. A half-turn is led round by the bug kept
            # ahead of the nose until the short way is the right way.
            turn_left = getattr(hold, "turn", "R") == "L"
            right = (want - track) % 360.0
            left = (track - want) % 360.0
            if not turn_left and 150.0 < right < 300.0:
                want = (track + 90.0) % 360.0
            elif turn_left and 150.0 < left < 300.0:
                want = (track - 90.0) % 360.0
        self.director.heading(self._magnetic(want, state))
        self.state.ap_hdg_mode = True

    # A fix this near is reached; one that came this near and is now
    # falling behind was passed abeam.
    DIRECT_REACHED_NM = 2.0
    DIRECT_PASSED_NM = 5.0

    def _end_direct(self) -> None:
        st = self.state
        st.direct_fix, st.direct_index = "", -1
        st.direct_nearest_nm = 0.0

    def _direct_index(self, route, ident: str, session, state) -> int:
        """Where on the route a freshly cleared direct fix is.

        The index the granter kept when it named the fix answers a route
        that visits it twice; without one, the first occurrence still
        ahead of the aeroplane does -- resolved inside the part of the
        route not yet flown, so "direct BBBBB" means the BBBBB to come
        and never the one behind.
        """
        hint = getattr(session, "route_direct_index", -1)
        after = 0
        progress = getattr(getattr(self, "brain", None),
                           "route_progress", None)
        if progress is None and getattr(state, "has_position", False):
            try:
                progress = route.progress(
                    state.latitude, state.longitude,
                    float(getattr(state, "ground_speed_kt", 0.0) or 0.0))
            except Exception:
                progress = None
        if progress is not None:
            # The leg's index is the fix it runs to: the first point
            # still ahead.
            after = max(0, int(progress.leg))
        return route.fix_index(ident, hint=hint, after=after)

    def _direct_destination(self, route, session):
        """Where the last leg of a direct points once the fixes run out:
        the field -- the session's airport, or the plan's last point."""
        dest = getattr(session, "destination", None)
        if dest is not None and getattr(dest, "lat", None) is not None \
                and getattr(dest, "lon", None) is not None:
            return dest
        return route.fixes[-1] if route.fixes else None

    def _fly_direct(self, state) -> None:
        """Fly the "proceed direct" the controller gave: straight to
        the fix, then on along the route from it.

        Nothing flew it before: the shortcut was read back and NAV went
        on round the legs it had cut, on a route the controller no
        longer expected the flight to be on. The simulator's FMS is
        never told, so NAV handed back at the fix would turn for the
        waypoint behind; the rest of the route is flown from here fix
        to fix -- kept as an index, so a fix the route names twice is
        the occurrence the direct meant -- until the arrival procedure
        or the approach takes the aeroplane. Past the last fix the field
        itself is the target: handing NAV back still legs-deep in the
        shortcut would fly the legs it cut. A vector or a hold given
        meanwhile is flown instead and the route picked up again after
        it.
        """
        st = self.state
        session = getattr(self, "session", None)
        route = getattr(getattr(self, "brain", None), "route", None)
        cleared = (getattr(session, "route_direct", "") or "").upper()
        enroute = st.phase in (AutonomousPhase.CLIMB, AutonomousPhase.CRUISE,
                               AutonomousPhase.DESCENT)
        if not enroute or route is None                 or not getattr(state, "has_position", False)                 or state.on_ground:
            self._end_direct()
            st.direct_cleared = ""
            return
        if cleared and cleared != st.direct_cleared:
            st.direct_index = self._direct_index(route, cleared,
                                                 session, state)
            st.direct_fix = cleared if st.direct_index >= 0 else ""
            st.direct_nearest_nm = 0.0
        st.direct_cleared = cleared
        if st.direct_index < 0:
            return
        if not cleared and getattr(st, "star_nav", None) is not None:
            # Past the shortcut and on the arrival: the procedure's.
            self._end_direct()
            return
        if getattr(session, "assigned_heading", None) is not None                 or getattr(session, "hold", None) is not None:
            return

        from ...navdata.build import bearing_deg, haversine_nm

        if st.direct_index >= len(route.fixes):
            # Beyond the route's last fix the direct still holds: the
            # descent would re-engage NAV on the leg the shortcut left,
            # and the aeroplane would turn back round the legs it cut.
            fix = self._direct_destination(route, session)
        else:
            i = route.fix_index(st.direct_fix, hint=st.direct_index)
            if i < 0:
                # A new plan without the fix ends the direct.
                self._end_direct()
                return
            st.direct_index = i
            fix = route.fixes[i]
            distance = haversine_nm(state.latitude, state.longitude,
                                    fix.lat, fix.lon)
            nearest = st.direct_nearest_nm or distance
            st.direct_nearest_nm = min(nearest, distance)
            passed = (st.direct_nearest_nm <= self.DIRECT_PASSED_NM
                      and distance > st.direct_nearest_nm + 0.5)
            if distance <= self.DIRECT_REACHED_NM or passed:
                nxt = next((j for j in range(i + 1, len(route.fixes))
                            if not route.fixes[j].is_marker), None)
                if nxt is not None:
                    st.direct_fix = route.fixes[nxt].ident.upper()
                st.direct_index = nxt if nxt is not None \
                    else len(route.fixes)
                st.direct_nearest_nm = 0.0
                if nxt is not None:
                    fix = route.fixes[nxt]
                else:
                    # The route's last fix: on to the field itself.
                    fix = self._direct_destination(route, session)
        if fix is None:
            self._end_direct()
            return
        self.director.heading(self._magnetic(
            bearing_deg(state.latitude, state.longitude, fix.lat, fix.lon),
            state))
        self.state.ap_hdg_mode = True

    # Below this an orbit is not begun: on short final the answer to
    # traffic too close ahead is the go-around the approach already has.
    ORBIT_MIN_AGL_FT = 500.0

    def _fly_orbit(self, state) -> bool:
        """Fly the tower's orbit: one level turn all the way round, to
        the side it was given, at the height it was given at.

        Nothing flew it before: "orbit left" was read back and the
        final flown on down behind the traffic it was meant to make
        room for. The tower counts the turn off the track and calls it
        complete; the aeroplane's own count ends it once round all the
        same, should nobody be watching.
        """
        st = self.state
        session = getattr(self, "session", None)
        side = getattr(session, "orbiting", "") or ""
        if not side or state.on_ground:
            st.orbit_alt_ft, st.orbit_track, st.orbit_turned = 0.0, -1.0, 0.0
            return False
        track = float(getattr(state, "track_true", None)
                      or getattr(state, "heading_true", None) or 0.0)
        if not st.orbit_alt_ft:
            agl = float(getattr(state, "altitude_agl_ft", 0.0) or 0.0)
            if agl < self.ORBIT_MIN_AGL_FT:
                return False
            st.orbit_alt_ft = float(state.altitude_ft or 0.0)
            st.orbit_track, st.orbit_turned = track, 0.0
        step = abs((track - st.orbit_track + 540.0) % 360.0 - 180.0)
        st.orbit_turned += min(step, 90.0)
        st.orbit_track = track
        if st.orbit_turned >= 360.0:
            return False
        # A heading bug turns the short way: kept a quarter ahead of
        # the nose on the orbit's side, it is a turn that way all round.
        want = (track + (90.0 if side == "right" else -90.0)) % 360.0
        self._hold_level(state, st.orbit_alt_ft, self._magnetic(want, state),
                         self._approach_speed(state))
        return True

    def _fly_avoidance(self, state) -> None:
        """Put the avoidance heading on the autopilot, last word of the
        tick: a phase that flies NAV, a procedure or nothing lateral at
        all has just asked for that again, and the turn the crew was
        told about is flown regardless."""
        bug = self._avoid_heading_bug(state)
        if bug >= 0.0:
            self.director.heading(bug)
            self.state.ap_hdg_mode = True

