from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    approach_flight,
    auto_nav,
    capabilities,
    contextlib,
    time,
)


class _Circling:
    # -- circling -----------------------------------------------------------

    def _circling(self, state, recipe) -> bool:
        """Whether the aeroplane is in the circle, entering it once.

        The published approach is done when the aeroplane is at or below
        its minima or has passed the threshold; from there the pattern to
        the landing runway takes over -- unless it is already lined up on
        that runway's final, which the ordinary approach sees home.
        """
        if self.state.circling:
            return not self._on_landing_final(state, recipe)
        agl = state.altitude_agl_ft or 0.0
        distance = state.distance_to(recipe.threshold_lat,
                                     recipe.threshold_lon) \
            if getattr(state, "has_position", False) else None
        # The floor is the circling MDA: below the approach's own minima
        # is already too low to be circling at. An offset final's version
        # of "there" is the missed-approach point itself -- the charted
        # final ends off the runway's axis, which is where the visual
        # manoeuvre (the VPT where one is published) begins.
        floor = recipe.circle_minima_ft or recipe.minima_ft or 200.0
        at_map = getattr(recipe, "offset_final", False) \
            and getattr(recipe, "map_lat", 0.0) \
            and getattr(state, "has_position", False) \
            and state.distance_to(recipe.map_lat, recipe.map_lon) < 1.2
        entered = agl <= floor + 100.0 \
            or (distance is not None and distance < 1.5) or at_map
        if not entered:
            return False
        path = approach_flight.circle_legs(recipe, self.navdb, state)
        if not path:
            return False
        self.state.circling = True
        self.state.circle_path = path
        # Below this the go-around triggers are the circling minima's.
        self.state.minima_ft = recipe.circle_minima_ft or recipe.minima_ft
        self._emit("info", "Circling to land",
                   "autonomous.circling")
        return True

    def _on_landing_final(self, state, recipe) -> bool:
        """On the landing runway's final, close enough to land.

        The recipe's own course, not ``state.runway_heading``: for a
        circle that heading can still be the approach runway's.
        """
        if recipe.final_course <= 0 \
                or not getattr(state, "has_position", False):
            return False
        track = getattr(state, "track_true", None) \
            or state.heading_true or 0.0
        err = abs((recipe.final_course - track + 540.0)
                  % 360.0 - 180.0)
        distance = state.distance_to(recipe.threshold_lat,
                                     recipe.threshold_lon)
        return err < 20.0 and distance < 4.0

    def _fly_circle(self, state, recipe) -> bool:
        """The circling pattern: hold the circling MDA, fly the rectangle
        to the landing runway's final."""
        if not self.state.circle_path:
            path = approach_flight.circle_legs(recipe, self.navdb, state)
            if not path:
                self.state.circling = False
                self.state.circle_path = []
                return False
            self.state.circle_path = path
        d = self.director
        # Whatever was tracking the approach runway's needles is done with
        # them: the pattern is flown on the heading bug.
        d.localizer(False)
        if self.state.ap_apr_mode:
            d.approach(False)
            self.state.ap_apr_mode = False
        steer = auto_nav.path_steering(
            state.latitude, state.longitude, self.state.circle_path,
            lookahead_m=2500.0)
        target = recipe.field_elev_ft + \
            (recipe.circle_minima_ft or recipe.minima_ft)
        alt = state.altitude_ft or 0.0
        vs = 0.0
        if alt > target + 80.0:
            vs = -250.0
        elif alt < target - 80.0:
            vs = 150.0
        heading = steer.get("heading")
        if heading is None:
            heading = recipe.final_course
        self._apply_flight_director(
            state, altitude=int(target), vs_fpm=vs,
            heading=self._magnetic(heading, state),
            speed_kt=(self._vapp(state))
            + 5.0)
        return True

    def _field_elevation(self, state) -> float:
        airport = self.session.destination or self.session.departure
        if airport is not None:
            return float(getattr(airport, "elev_ft", 0.0) or 0.0)
        return 0.0

    def _distance_to_destination_nm(self, state) -> float | None:
        dest = self.session.destination
        if dest is None or not getattr(state, "has_position", False):
            return None
        return state.distance_to(dest.lat, dest.lon)

    def _plan_tod(self, state) -> None:
        """Where the descent begins: the height to lose on a three-degree
        path, plus a margin for the configuration."""
        distance = self._distance_to_destination_nm(state)
        if distance is None:
            self.state.tod_nm = -1.0
            return
        agl = (state.altitude_ft or 0.0) - self._field_elevation(state)
        # 318 ft per NM is the standard path; the TOD is where that
        # path, started now, reaches the field -- plus the room the
        # speed takes to come back to the descent figure: about a mile
        # for every ten knots a level cruise would still be carrying,
        # the "three times the thousands plus the slowdown" rule of
        # thumb a crew plans by.
        cruise = self._cruise_speed_kt(state)
        descent_kt = max(self.performance.approach_kt, 0.85 * cruise)
        slow = max(0.0, cruise - descent_kt) / 10.0
        self.state.tod_nm = max(0.0, agl / 318.0 + slow)

    def _descent_floor(self, state) -> float:
        """The lowest level the descent may level at: pattern height."""
        elev = self._field_elevation(state)
        # A safe floor for the descent itself: pattern height over the
        # field -- or the level the plan's own profile bottoms at where
        # it files one, so "descend via the STAR" that ends at seven
        # thousand is not walked below it. Anything lower than either
        # is the approach phase's job, once it has the runway in sight
        # or the needles captured.
        route = getattr(getattr(self, "brain", None), "route", None)
        filed_floor = float(getattr(route, "arrival_floor_ft", 0.0) or 0.0)
        return max(elev + 3000.0, filed_floor)

    def _configure_final(self, state) -> None:
        """The approach configuration, by distance to the threshold."""
        d = self.director
        distance = self._distance_to_destination_nm(state)
        agl = (state.altitude_ft or 0.0) - self._field_elevation(state)

        # Gear down on final, flaps through the stages, speedbrake armed.
        # Only a model that has said the gear is down already is not
        # asked: up or unanswered both get the command. The inverted
        # check used to ask only the aeroplane already configured.
        if (agl < 2000 or (distance is not None and distance < 10.0)) \
                and state.gear_down is not True \
                and self._gear_allowed(state):
            d.gear(True)
        if distance is not None and distance < 6.0 and agl < 2500:
            self._set_flaps(state, 0.75)
            self.state.configured = "flaps_landing"
        if distance is not None and distance < 3.0:
            self._set_flaps(state, self.performance.landing_flaps or 1.0)
            self.state.configured = "flaps_full"
        d.spoilers_arm(True)
        # Approach speed: Vapp where known, the type's figure otherwise.
        vapp = self._vapp(state)
        self.director.speed(vapp)

    def _check_stabilized(self, state) -> None:
        """The stabilized-approach call, once, below a thousand feet."""
        if self.state.stabilized:
            return
        # The runway this judges against is the arrival's, and the pick
        # made leaving home is still on the state unless refreshed:
        # without this the line-up's threshold stands in for the field
        # ahead and no final is ever aligned, let alone stable.
        self._find_runway(state)
        agl = state.altitude_agl_ft or 0.0
        if agl > 1000.0 or agl <= 0.0:
            return
        vs = abs(state.vertical_speed_fpm or 0.0)
        ias = state.indicated_airspeed_kt or 0.0
        target = self._vapp(state)
        bank = abs(getattr(state, "bank_deg", 0.0) or 0.0)
        aligned = True
        if self.state.runway_heading > 0 and state.on_ground is False:
            err = abs((self.state.runway_heading
                       - (state.heading_true or 0.0) + 540.0)
                      % 360.0 - 180.0)
            aligned = err < 15.0
            # On the runway's line, not merely pointing its way: within
            # a twentieth of the distance still to run (about three
            # degrees of the extended centreline), never tighter than a
            # runway's half width.
            offset = self._centreline_offset_m(state)
            if offset is not None and getattr(state, "has_position", False):
                to_go_m = state.distance_to(
                    self.state.runway_lat, self.state.runway_lon) * 1852.0
                aligned = aligned and abs(offset) < max(25.0, to_go_m * 0.05)
                # And near the path down to it: far above a three-degree
                # slope this close in is a dive at the runway or a
                # landing halfway along it, whatever the needles of the
                # moment read.
                above = (state.altitude_ft or 0.0)                     - self._field_elevation(state)
                on_path = approach_flight.TCH_FT                     + to_go_m / 1852.0 * approach_flight.FT_PER_NM
                aligned = aligned and above < on_path * 1.4 + 250.0
        bank_max = self.performance.max_bank_deg or 25.0
        if vs < 1000.0 and abs(ias - target) < 12.0 \
                and bank < bank_max and aligned:
            self.state.stabilized = True
            self._emit("info", "Stabilized on final",
                       "autonomous.stabilized")

    def _should_go_around(self, state) -> bool:
        """The criteria that send an approach around, conservatively."""
        # The tower called it: a missed approach is the aeroplane's, not
        # the controller's, to refuse. The flag stays up until the
        # controller sees the aeroplane back on a final, so it is honoured
        # on its rising edge only -- once per call, not once per tick.
        flag = bool(self.session.going_around)
        if flag and not self._ga_flag_seen:
            self._ga_flag_seen = True
            return True
        if not flag:
            self._ga_flag_seen = False
        agl = state.altitude_agl_ft or 0.0
        # The weather's gates, below a thousand feet: the wind past the
        # limits on this runway, or the field's own report below what the
        # approach can be flown to. Counted once an approach -- the flag
        # lifts when the next one is set up.
        if (self._weather_reactions and 0.0 < agl < 1000.0
                and not self.state.weather_ga_active
                and (self._landing_wind_bad(state)
                     or self._weather_below_minima(state))):
            self.state.weather_gas += 1
            self.state.weather_ga_active = True
            return True
        # Below minimums without the runway: a go-around, not a dive.
        if (agl < (self.state.minima_ft or 200.0) and agl > 0
                and not self.session.cleared_landing and self.session.ifr):
            return True
        # Unstable low down: sink rate, bank, or speed far off.
        if 0 < agl < 500.0:
            vs = state.vertical_speed_fpm or 0.0
            if vs < -1200.0:
                return True
            bank = abs(getattr(state, "bank_deg", 0.0) or 0.0)
            if bank > (self.performance.max_bank_deg or 25.0) + 10.0:
                return True
            ias = state.indicated_airspeed_kt or 0.0
            target = self._vapp(state)
            if target and abs(ias - target) > 20.0:
                return True
        # Gear that never came down: once is a go-around to work the
        # handle and the checklist; the second time it is the belly
        # landing, announced -- a pilot does not circuit for ever on a
        # gear that will not come out.
        if 0.0 < agl < 1000.0 and state.gear_down is False \
                and not self.state.gear_ga_done:
            self.state.gear_ga_done = True
            self._emit("info",
                       "Gear not down on final: going around",
                       "autonomous.gear_up_goaround")
            return True
        if 0.0 < agl < 500.0 and state.gear_down is False \
                and self.state.gear_ga_done \
                and not self.state.gear_up_said:
            self.state.gear_up_said = True
            self._emit("info", "Gear still up: landing gear-up",
                       "autonomous.gear_up_landing")
        # Somebody on the runway this close to it is a missed approach,
        # not a landing on top of them -- the tower's call or not, and
        # it applies whether or not the landing clearance ever came.
        if 0.0 < agl < 500.0 and self._runway_blocked(
                getattr(self.session, "arrival_runway", "") or "",
                airport=getattr(self.session, "destination", None)
                or getattr(self.session, "departure", None)):
            return True
        # Still in the air with the touchdown zone behind: the runway
        # left is not enough to land on. A third of it gone and the
        # wheels not yet in the flare is a go-around, not a long float
        # toward the far fence.
        if 60.0 < agl < 500.0 and self.state.runway_heading > 0                 and (self.state.runway_length_ft or 0.0) > 0.0                 and getattr(state, "has_position", False)                 and state.on_ground is False:
            where = auto_nav.align(
                state.latitude, state.longitude, self.state.runway_lat,
                self.state.runway_lon, self.state.runway_heading)
            length_m = self.state.runway_length_ft * 0.3048
            if (length_m / 3.0 < where["along_m"] < length_m
                    and abs(where["offset_m"]) < 300.0):
                return True
        # Never stabilized by the flare's gate is a go-around however
        # the numbers looked on the way down.
        if 0.0 < agl < 200.0 and not self.state.stabilized:
            return True
        # Drifting toward the ground without a flare started. On the
        # runway itself is a landing, not a go-around: wheels on the
        # ground never get the power-and-pitch treatment.
        return bool(0.0 < agl < 30.0 and not self.session.cleared_landing
                    and state.on_ground is not True)

    def _start_go_around(self, state) -> None:
        """The go-around: power, pitch, configuration, climb away."""
        if not capabilities(state.source).flight_axes:
            self._manual_control_required()
            return
        if self.state.go_arounds >= 3:
            # A pilot does not circuit for ever: three go-arounds without
            # the weather's count is the field not taking the aeroplane,
            # and the alternate is what it is there for.
            self._divert_after_go_arounds(state)
            return
        self.state.go_arounds += 1
        self.state.circling = False
        self.state.circle_path = []
        self.state.go_around_at = time.monotonic()
        # The flare's clock belongs to the approach being left behind:
        # keeping it would send the next pass around the moment it sank
        # below sixty feet.
        self.state.flare_at = 0.0
        # Missed approach altitude: what was assigned, else the
        # procedure's own where the recipe knows it, else a safe pattern
        # height -- but never below the aeroplane. A stale low
        # assignment would end the missed approach on its first tick.
        recipe = self.state.approach_recipe
        floor = max(self._field_elevation(state) + 2000.0,
                    (state.altitude_ft or 0.0) + 500.0)
        self.state.missed_alt_ft = max(
            floor,
            self.session.assigned_altitude_ft or 0.0,
            (getattr(recipe, "missed_alt_ft", 0.0) or 0.0))
        # From the landing flare the go-around is an approach again: the
        # phase owns the sub-step only from APPROACH down.
        if self.state.phase == AutonomousPhase.LANDING:
            self._transition(AutonomousPhase.APPROACH)
        self._goto_sub("go_around")
        d = self.director
        # A fresh TO/GA press: the same one-shot events the takeoff roll
        # fired have to be allowed to fire again.
        d.forget("takeoff_power", "toga")
        d.go_around()
        d.takeoff_power()
        self._prop_levers()
        d.throttle(1.0)
        self._last_lever = 1.0
        # What was tracking the approach's needles is done with them: a
        # go-around flown with APR still armed hunts the glide it just
        # left. The modes it armed go with it -- a VS descent still
        # engaged commands the elevator the go-around's own pull is
        # fighting, the fight the stall monitor drops the AP for.
        d.approach(False)
        if self.state.ap_master or self.state.ap_speed_mode \
                or self.state.ap_vs_mode:
            d.autopilot(False)
            d.airspeed_hold(False)
            d.vs_hold(False)
            d.localizer(False)
            self.state.ap_master = False
            self.state.ap_speed_mode = False
            self.state.ap_vs_mode = False
            self.state.ap_alt_mode = False
            self.state.ap_hdg_mode = False
            self.state.ap_nav_mode = False
        # The flare's surface demands are the runway's, not the
        # climb's: a de-crab pedal or an into-wind wing still commanded
        # into the missed approach is a skid and a bank at fifty feet.
        d.rudder(0.0)
        d.steering(0.0)
        d.aileron(0.0)
        d.elevator(self._g_pull_cap(state,
                                    self._go_around_pull(state, 0.3)))
        # The book's order is power, pitch, flap, THEN gear on the
        # positive rate -- the wheels stay down while the aeroplane can
        # still touch, and come up once the climb is established.
        # Flaps one stage up, not all the way: the go-around flap setting.
        if state.flaps_index >= 2:
            d.flaps(0.5)
        elif state.flaps_index == 1:
            d.flaps(0.25)
        self.state.stabilized = False
        self.state.configured = ""
        self.state.ap_apr_mode = False
        # The tower hears it: a missed approach is called, not just flown.
        if self._manage_radio:
            self._fo_say("going_around")
        self._emit("info", "Going around", "autonomous.go_around")

    def _go_around_pull(self, state, want: float) -> float:
        """Bound a go-around pull by the airspeed's say in the matter.

        A crew pitches for the climb attitude and flies the speed first:
        under the go-around figure the nose does not come up more, and
        well under it comes down, because a missed approach stalled is
        a stall, not a missed approach.
        """
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        floor = float(getattr(self.performance, "goaround_kt", 0.0)
                      or 0.0) \
            or float(getattr(self.state, "climb_kt", 0.0) or 0.0) \
            or self._vapp(state)
        if not (ias and floor):
            return want
        if ias < floor - 10.0:
            # Speed badly decayed: unload the wing, do not pull it up.
            return min(want, -0.1)
        if ias < floor:
            return min(want, 0.0)
        return want

    def _fly_go_around(self, state, now: float) -> None:
        """Climb away on the missed approach, then set up again."""
        d = self.director
        elapsed = now - self.state.go_around_at
        # Initial: pitch and power; after positive rate, clean up. The
        # pull eases as the climb establishes, the way a pilot relaxes
        # the back pressure once the VSI agrees.
        if elapsed < 8.0:
            pull = (0.25 if (state.vertical_speed_fpm or 0.0) < 500.0
                    else 0.10)
            # The go-around attitude bounds the pull, the way the
            # rotation's does: at the mark the hand comes off, a dead
            # stick, not a lingering pinch that keeps pitching past it.
            pitch = float(getattr(state, "pitch_deg", 0.0) or 0.0)
            if pitch >= self._pitch_target_deg():
                pull = 0.0
            pull = self._go_around_pull(state, pull)
            d.rudder(0.0)
            d.steering(0.0)
            d.aileron(0.0)
            d.elevator(self._g_pull_cap(state, pull))
            self._prop_levers()
            d.throttle(1.0)
            self._last_lever = 1.0
            # Hand-flown for the first seconds, the way the book flies
            # it: the autopilot stays out of it until the attitude and
            # the climb are there, or pitch mode and the elevator write
            # fight each other.
            return
        d.elevator(0.0)
        if state.gear_down and (state.vertical_speed_fpm or 0.0) > 0.0:
            d.gear(False)
        # Flap comes up a stage at a time, behind the speed -- the
        # all-at-once pull to zero at a few hundred feet used to run
        # here.
        if state.flaps_index >= 2:
            self._set_flaps(state, 0.5)
        elif state.flaps_index:
            self._set_flaps(state, 0.0)
        # Wings-level climb on the runway heading or the assigned heading.
        # The heading bug is magnetic: the runway's is degrees true off
        # the threshold's coordinates, so it is converted like every
        # other course this hands the autopilot.
        hdg = self.session.assigned_heading
        if hdg is None and self.state.runway_heading > 0:
            hdg = self._magnetic(self.state.runway_heading, state)
        self._apply_flight_director(state, altitude=self.state.missed_alt_ft,
                                    vs_fpm=self.performance.climb_fpm,
                                    heading=hdg if hdg is not None else -1.0,
                                    speed_kt=(self.performance.goaround_kt
                                              or self.state.climb_kt
                                              or 180.0))
        # Once level-ish at the missed altitude, reset for another pass --
        # unless the weather sent this one and the one before it, in
        # which case the destination is out and the alternate gets it.
        alt = state.altitude_ft or 0.0
        if alt >= self.state.missed_alt_ft - 150.0 \
                and abs(state.vertical_speed_fpm or 0.0) < 300:
            self.state.stabilized = False
            self.state.configured = ""
            # The missed is flown: the next approach's throttle is its
            # own, not held at climb power under the old missed height.
            self.state.missed_alt_ft = 0.0
            if self.state.weather_gas >= 2 and self._weather_reactions:
                self._divert_for_weather(state)
                return
            self.state.weather_ga_active = False
            self._goto_sub("intercept")
            self._emit("info", "Missed approach complete; setting up again",
                       "autonomous.go_around_done")
        elif elapsed > 300.0:
            self._set_error("Go-around did not climb")

    def _divert_after_go_arounds(self, state) -> None:
        """The fourth missed approach is not flown: the alternate is.

        Where there is a field in reach that can take the aeroplane the
        destination becomes it, re-planned the way an emergency's divert
        is; where there is none the flight is over, because the approach
        has already said everything it can.
        """
        dest = self._pick_alternate(state)
        if dest is None:
            self._set_error("Three go-arounds and no alternate in reach")
            return
        self._emit("info", f"Three go-arounds: diverting to {dest.ident}",
                   "autonomous.divert.go_arounds")
        if self.crew.transmit:
            with contextlib.suppress(Exception):
                self.crew.transmit(f"unable to land, diverting to "
                                   f"{dest.ident}")
        self.session.destination = dest
        self._route_diversion()

