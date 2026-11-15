from __future__ import annotations

from ._shared import (
    approach_flight,
    auto_nav,
    log,
    project,
    sqlite3,
)


class _The_cleared_approach:
    # -- the cleared approach, as a recipe ---------------------------------

    def _recipe_key(self, state) -> tuple:
        """What the recipe was resolved for: a new field, runway or
        request is a new recipe."""
        dest = getattr(self.session, "destination", None)
        return (getattr(dest, "ident", "") or "",
                getattr(self.session, "arrival_runway", "") or "",
                getattr(self.session, "requested_approach", "") or "")

    def _approach_recipe(self, state):
        """The cleared approach resolved for flying, once per leg.

        approach_flight.resolve reads the controller's own choice off the
        session -- "ILS Zulu", "RNAV Yankee", "VOR-A" -- and turns it into
        a course, a final approach fix, minima and the navaid to tune.
        """
        key = self._recipe_key(state)
        if self.state.approach_recipe_tried \
                and key == self.state.approach_recipe_key:
            return self.state.approach_recipe
        self.state.approach_recipe_tried = True
        self.state.approach_recipe_key = key
        try:
            recipe = approach_flight.resolve(
                self.session, self.brain, self.navdb, state)
        except (sqlite3.Error, AttributeError, TypeError, ValueError,
                KeyError, IndexError, RuntimeError) as exc:
            log.info("approach recipe failed: %s", exc)
            recipe = None
        self.state.approach_recipe = recipe
        if recipe is not None:
            # The approach's own minima, not the ILS's 200 ft every
            # approach used to be judged against.
            self.state.minima_ft = recipe.minima_ft
            from ...logs import decision
            decision("autonomous_approach", airport=recipe.ident,
                     approach=recipe.name, family=recipe.family,
                     line=recipe.line or None,
                     approach_runway=recipe.approach_runway,
                     landing_runway=recipe.landing_runway,
                     navaid=recipe.navaid.ident if recipe.navaid else None,
                     freq_mhz=(round(recipe.navaid.freq_mhz, 3)
                               if recipe.navaid else None),
                     minima_ft=recipe.minima_ft,
                     circle_minima_ft=recipe.circle_minima_ft or None,
                     why=recipe.why or None)
        return recipe

    def _maybe_tune_nav(self, state, recipe, now: float) -> None:
        """NAV1 to the approach's navaid, once, with readback.

        The localizer, the VOR or the NDB the procedure is flown on -- the
        needles and the station callouts are only there if the radio is
        tuned, and until now nothing ever tuned it. Tried again every ten
        seconds until the frequency reads back; an aeroplane whose radios
        the FO cannot reach simply keeps the old guidance.
        """
        nav = getattr(recipe, "navaid", None) if recipe is not None else None
        if nav is None or not nav.freq_mhz:
            return
        sim = self.sim
        if sim is None or not getattr(sim, "connected", False):
            return
        tuned = float(getattr(state, "nav1_freq", 0.0) or 0.0)
        if tuned and abs(tuned - nav.freq_mhz) < 0.005:
            if not self.state.approach_tuned:
                self.state.approach_tuned = True
                self._emit("info",
                           f"NAV1 showing {nav.ident} "
                           f"{nav.freq_mhz:.3f} MHz",
                           "autonomous.nav_tuned")
            return
        if now - self.state.nav_tune_at < 10.0:
            return
        self.state.nav_tune_at = now
        if hasattr(sim, "set_nav") and sim.set_nav(nav.freq_mhz, 1):
            self._emit("info",
                       f"Tuning NAV1 to {nav.ident} {nav.freq_mhz:.3f} MHz",
                       "autonomous.nav_tune")

    @staticmethod
    def _magnetic(true_heading: float, state) -> float:
        """A true course as the heading bug wants it: magnetic."""
        from ...navdata.magvar import to_magnetic, variation

        return to_magnetic(true_heading,
                           variation(state.latitude, state.longitude)) % 360.0

    def _fly_recipe(self, state, recipe, now: float) -> bool:
        """The cleared approach's own guidance; False for the fallback.

        What the aeroplane can actually fly decides the mode: an ILS with
        live needles gets the autopilot's approach mode; a localizer
        without a glideslope gets LOC mode and the path's vertical speed;
        everything else -- VOR, NDB, RNAV, an ILS whose needles never came
        alive -- is the computed final: the gate-FAF-threshold path flown
        by heading and a three-degree vertical speed.
        """
        d = self.director
        family = recipe.family
        has_loc = bool(getattr(state, "nav1_has_localizer", False))
        has_gs = bool(getattr(state, "nav1_has_glideslope", False))

        # A circle, once it has started, owns the aeroplane until it is
        # back on the landing runway's final.
        if recipe.circling and self._circling(state, recipe):
            return self._fly_circle(state, recipe)

        if family == "ils" and has_loc and has_gs and self.session.ifr:
            d.approach(True)
            self.state.ap_apr_mode = True
            d.vs_hold(False)
            self.state.ap_vs_mode = False
            # The needles own the flight path, not the levers: approach
            # mode with no speed answer is a decelerating aeroplane on
            # the glide, whatever the descent left the throttle at.
            speed = self._vapp(state)
            if speed:
                d.speed(speed)
                d.airspeed_hold(True)
                self.state.ap_speed_mode = True
                self._manage_throttle(state, speed)
            return True
        if family == "loc" and has_loc and self.session.ifr \
                and not getattr(recipe, "back_course", False):
            # The localizer for the lateral, the path for the vertical:
            # there is no glideslope on this approach to capture. A back
            # course is excluded: its needles sense reversed, and a
            # front-course hold on them flies away from the runway --
            # it takes the computed final like a VOR.
            d.localizer(True)
            self.state.ap_apr_mode = False
            self._fly_path_vertical(state, recipe)
            return True
        if family in ("ils", "loc", "vor", "ndb", "rnav"):
            self._fly_computed(state, recipe)
            return True
        return False

    def _path_vertical(self, state, recipe) -> float:
        """The vertical speed the three-degree path asks for here."""
        return self._three_degree_vs(
            state, recipe.threshold_lat, recipe.threshold_lon,
            recipe.field_elev_ft)

    def _three_degree_vs(self, state, thr_lat: float, thr_lon: float,
                         elev_ft: float) -> float:
        """The vertical speed that holds a three-degree path to a
        threshold: the path's own sink at this groundspeed, corrected by
        where the aeroplane sits against it. Shared by every final that
        has no glideslope needle -- the computed procedures and the
        visual alike -- so none of them descends by the clock alone."""
        distance = state.distance_to(thr_lat, thr_lon) \
            if getattr(state, "has_position", False) else None
        desired = (elev_ft + approach_flight.TCH_FT
                   + max(0.0, distance if distance is not None else 10.0)
                   * approach_flight.FT_PER_NM)
        alt = state.altitude_ft or 0.0
        vs = approach_flight.sink_fpm(state.ground_speed_kt or 0.0)
        if self._use_pids:
            # The path error to a sink correction on top of the profile's
            # own figure: above the path the loop deepens the descent,
            # below it the answer climbs back -- the piecewise steps
            # flown as one continuous law, with the integrator carrying
            # any steady offset the profile itself got wrong. The lower
            # bound is a climb: below the path is regained, not dived
            # through.
            corr = self._pid("vertical").update(alt - desired,
                                                measurement=alt)
            return max(-900.0, min(250.0, -vs - corr))
        # Capture: well above the path it comes down harder; below it the
        # aeroplane climbs back to the path rather than keep sinking
        # toward the ground under it.
        if alt > desired + 300.0:
            vs = min(900.0, vs * 1.5)
        elif alt < desired - 150.0:
            return 200.0
        return -vs

    def _fly_path_vertical(self, state, recipe) -> None:
        """Vertical speed and speed for a mode that owns the lateral."""
        self._apply_flight_director(
            state, vs_fpm=self._path_vertical(state, recipe),
            speed_kt=self._vapp(state))

    def _fly_computed(self, state, recipe) -> None:
        """The computed final: path steering for the lateral, the
        three-degree profile for the vertical. What an RNAV, a VOR, an NDB
        or an ILS whose needles never arrived all share -- the geometry is
        the same, the minima and the tuned station are what differ."""
        d = self.director
        d.localizer(False)
        if self.state.ap_apr_mode:
            # APR was flying this approach a moment ago; the needles are
            # gone or the procedure changed, and approach mode with no
            # guidance behind it is worse than none.
            d.approach(False)
            self.state.ap_apr_mode = False
        steer = auto_nav.path_steering(
            state.latitude, state.longitude, approach_flight.legs(recipe),
            lookahead_m=max(400.0, min(2500.0, 926.0 * state.distance_to(
                recipe.threshold_lat, recipe.threshold_lon))))
        heading = steer.get("heading")
        if heading is None:
            heading = recipe.final_course
        self._apply_flight_director(
            state, vs_fpm=self._path_vertical(state, recipe),
            heading=self._magnetic(heading, state),
            speed_kt=self._vapp(state))

    def _fly_visual_final(self, state) -> bool:
        """The final with no procedure behind it: the extended centreline
        tracked like a procedure leg and the three-degree path held by
        reference to the distance still to run. False where the runway
        is not known -- there is nothing to aim at.

        The descent used to be the path's sink rate flown from wherever
        the aeroplane happened to be: begun low, it met the ground short
        of the runway at exactly three degrees."""
        if self.state.runway_heading <= 0:
            self._find_runway(state)
        if self.state.runway_heading <= 0:
            return False
        rwy_hdg = self.state.runway_heading
        heading = self._magnetic(rwy_hdg, state)
        if getattr(state, "has_position", False):
            gate = project(self.state.runway_lat, self.state.runway_lon,
                           (rwy_hdg + 180.0) % 360.0, 10.0)
            far = project(self.state.runway_lat, self.state.runway_lon,
                          rwy_hdg, approach_flight.PAST_THRESHOLD_NM)
            steer = auto_nav.path_steering(
                state.latitude, state.longitude, [gate, far],
                lookahead_m=self._final_lookahead_m(state))
            if steer and steer.get("heading") is not None:
                heading = self._magnetic(steer["heading"], state)
        self._apply_flight_director(
            state,
            vs_fpm=self._three_degree_vs(
                state, self.state.runway_lat, self.state.runway_lon,
                self._field_elevation(state)),
            heading=heading,
            speed_kt=self._vapp(state) or self.performance.approach_kt)
        return True

    def _final_lookahead_m(self, state) -> float:
        """How far ahead the final's steering point sits: long out on the
        approach, drawn in close to the runway so the last mile is flown
        on the centreline rather than toward a point beyond it."""
        if not getattr(state, "has_position", False) \
                or self.state.runway_heading <= 0:
            return 2000.0
        to_go_m = state.distance_to(self.state.runway_lat,
                                    self.state.runway_lon) * 1852.0
        return max(400.0, min(2500.0, to_go_m * 0.5))

    def _fly_short_final(self, state, now: float) -> None:
        """Below five hundred feet and cleared to land: the same guidance
        the approach flew, carried to the flare."""
        d = self.director
        recipe = self._approach_recipe(state)
        if recipe is not None and recipe.family != "visual"                 and not recipe.circling                 and self._fly_recipe(state, recipe, now):
            return
        has_ils = getattr(state, "nav1_has_localizer", False) and             getattr(state, "nav1_has_glideslope", False)
        if has_ils and self.session.ifr:
            d.approach(True)
            self.state.ap_apr_mode = True
            vapp = self._vapp(state) or self.performance.approach_kt
            d.speed(vapp)
            d.airspeed_hold(True)
            self.state.ap_speed_mode = True
            self._manage_throttle(state, vapp)
            return
        self._fly_visual_final(state)

    def _centreline_offset_m(self, state) -> float | None:
        """Metres right of the landing runway's centreline, or None where
        the runway or the position is not known."""
        if self.state.runway_heading <= 0                 or not getattr(state, "has_position", False):
            return None
        return auto_nav.align(
            state.latitude, state.longitude, self.state.runway_lat,
            self.state.runway_lon, self.state.runway_heading)["offset_m"]

    def _centreline_heading(self, state, max_cut_deg: float) -> float:
        """The true heading that holds the runway and closes on its
        centreline: the runway's own, cut toward the line by a degree
        for every few metres off it, capped -- a heading hold alone
        keeps the aeroplane parallel to the runway, which is how a
        rollout ends in the grass beside it."""
        rwy = self.state.runway_heading
        offset = self._centreline_offset_m(state)
        if offset is None or abs(offset) < 1.0:
            return rwy
        cut = max(-max_cut_deg, min(max_cut_deg, offset * 0.3))
        return (rwy - cut) % 360.0

