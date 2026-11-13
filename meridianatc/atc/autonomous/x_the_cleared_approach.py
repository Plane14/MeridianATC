from __future__ import annotations

from ._shared import (
    approach_flight,
    auto_nav,
    log,
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
        distance = state.distance_to(recipe.threshold_lat,
                                     recipe.threshold_lon) \
            if getattr(state, "has_position", False) else None
        desired = approach_flight.profile_alt(
            recipe, distance if distance is not None else 10.0)
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
            lookahead_m=2500.0)
        heading = steer.get("heading")
        if heading is None:
            heading = recipe.final_course
        self._apply_flight_director(
            state, vs_fpm=self._path_vertical(state, recipe),
            heading=self._magnetic(heading, state),
            speed_kt=self._vapp(state))

