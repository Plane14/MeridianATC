from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    contextlib,
    log,
    math,
    re,
    time,
)


class _Weather:
    # -- weather ---------------------------------------------------------------

    def _monitor_weather(self, state, now: float) -> None:
        """What the weather is doing, answered the way a crew answers it.

        Nothing is invented: the icing call is made on the cloud and the
        temperature the sim is rendering, the rough air on the wind the
        airframe is actually in, the wind limits on the same METAR the
        controllers quote. The departure and approach checks live at the
        decisions they gate; what is watched here is what the airframe is
        in, which is nobody's phase to own.
        """
        # An emergency already declared does not calm the air: the
        # windshear on the PAN PAN's diversion approach is still a
        # windshear.
        if not self._weather_reactions:
            return
        self._watch_icing(state, now)
        self._watch_turbulence(state, now)
        self._watch_density_altitude(state)

    def _watch_density_altitude(self, state) -> None:
        """The hot-and-high briefing, before the takeoff it matters to.

        Density altitude is pressure altitude corrected for temperature
        -- the altitude the airframe performs at, not the one the field
        sits at. A runway at five thousand feet on a thirty-degree day
        flies like eight thousand, and a crew that does not say so is a
        crew that rotates into a longer takeoff than they briefed. Said
        once, on the ground, where the numbers still come cheap.
        """
        if getattr(self.state, "density_alt_said", False) \
                or not state.on_ground:
            return
        elev = self._field_elevation(state)
        if elev <= 0.0:
            return
        oat_raw = getattr(state, "ambient_temperature_c", None)
        # Zero Celsius is a reading, not a missing one -- ``or 15`` would
        # warm a freezing day into a standard one.
        oat = 15.0 if oat_raw is None else float(oat_raw)
        isa = 15.0 - 2.0 * (elev / 1000.0)
        density_alt = elev + 120.0 * (oat - isa)
        if density_alt - elev < 2500.0 and density_alt < 7000.0:
            return
        self.state.density_alt_said = True
        self._emit(
            "info",
            f"Density altitude {density_alt:,.0f} ft (field "
            f"{elev:,.0f} ft, {oat:.0f} C): expect a longer takeoff",
            "autonomous.weather.density")

    def _watch_icing(self, state, now: float) -> None:
        """Ice protection on in visible moisture near freezing -- the
        kit the type actually carries, not every switch the sim has.

        TAT is the figure a crew watches -- it is warmer than ambient at
        speed, which is the same reason an aeroplane ices in cloud that
        is not strictly below freezing. What gets switched is the type's
        own list (atc.powerplant.ice_kit_for): a carburetted piston gets
        its induction heat on its own envelope, a turbine its engine and
        inlet anti-ice, an unprotected airframe nothing at all -- a
        write that does not exist does no harm, but a switch the type
        never had is not asked for either.
        """
        from ..powerplant import ice_kit_for
        code = (getattr(state, "type_code", "") or ""
                or getattr(getattr(self.session, "aircraft", None),
                           "type_code", "") or "")
        kit = ice_kit_for(code,
                          getattr(self, "_aircraft_class", "") or "")
        tat_raw = getattr(state, "total_air_temperature_c", None)
        ambient_raw = getattr(state, "ambient_temperature_c", None)
        tat = (float(ambient_raw) if ambient_raw is not None else 15.0) \
            if tat_raw is None else float(tat_raw)
        ambient = 15.0 if ambient_raw is None else float(ambient_raw)
        moisture = (bool(getattr(state, "ambient_in_cloud", False))
                    or int(getattr(state, "ambient_precip_state", 0) or 0) > 0)
        icing = (not state.on_ground) and moisture \
            and -30.0 < min(tat, ambient) < 6.0
        d = self.director
        engines = range(1, int(getattr(state, "engine_count", 0) or 1) + 1)
        if icing and not self.state.icing_said:
            self.state.icing_said = True
            if kit.pitot:
                d.pitot_heat(True)
            if kit.engine_inlet or kit.inertial_separator:
                # One SimVar serves the cowl anti-ice, the inlet lip and
                # the PT6's inertial separator: whichever the model wired
                # is the one that answers.
                for index in engines:
                    d.write(f"GENERAL ENG ANTI ICE POSITION:{index}", 1.0)
            if kit.prop_deice:
                d.write("PROP DEICE SWITCH:1", 1.0)
            if kit.airframe_deice:
                d.write("STRUCTURAL DEICE SWITCH", 1.0)
            if kit.windshield:
                d.write("WINDSHIELD DEICE SWITCH:1", 1.0)
            self._emit("info", "Icing conditions: anti-ice and pitot "
                       "heat on", "autonomous.weather.icing")
        elif not icing:
            # Out of the envelope -- dried off or too warm/cold -- the
            # advisory is re-armed so a re-entry is said again. The
            # switches stay on: heat left on in clear air does no harm,
            # and taking it off second-guesses a condition the model
            # may just not have reported yet.
            self.state.icing_said = False
        # Carburetted induction ice obeys its own envelope -- broader
        # than the structural one, and low power all by itself -- and a
        # carburetted engine wants its heat on the ground too, for the
        # run-up. Tracked on ``carb_set`` so the FO's own application
        # comes back off when the sky dries; a lever the pilot set
        # himself is never touched.
        if kit.carb_heat and getattr(state, "engines_running", False):
            want = self._carb_heat_wanted(state)
            if want != self.state.carb_set:
                self.state.carb_set = want
                # Every carburetted engine gets its lever: writing only
                # :1 left a twin's second engine to ice on its own.
                for index in engines:
                    d.write(f"GENERAL ENG ANTI ICE POSITION:{index}",
                            1.0 if want else 0.0)
                if want:
                    self._emit("info", "Carburettor heat on",
                               "autonomous.weather.carb")
        # Ice that is accreting anyway is a route problem, not a switch
        # problem: it is said once so the pilot hears it -- earlier on an
        # airframe that was never certified to carry it.
        ice = float(getattr(state, "structural_ice_pct", 0.0) or 0.0)
        warn_at = 40.0 if kit.known_ice else 25.0
        if ice >= warn_at and not self.state.ice_warned:
            self.state.ice_warned = True
            note = "" if kit.known_ice else \
                " (not certified for known icing)"
            self._emit("info",
                       f"Ice accreting on the airframe ({ice:.0f}%)"
                       f"{note}: consider leaving this altitude",
                       "autonomous.weather.ice")

    def _watch_turbulence(self, state, now: float) -> None:
        """Rough air, as the airframe reports it.

        The vertical wind is the only convective evidence the sim
        publishes, and it is honest evidence: a ten-knot up or downdraft
        is a thousand feet a minute. In it the speed is pulled back to
        the penetration figure; on final a swing of it is a windshear,
        and a windshear is a go-around, not a recovery attempt.
        """
        if state.on_ground:
            return
        updraft = abs(getattr(state, "aircraft_wind_y_kt", 0.0) or 0.0)
        if updraft >= 12.0:
            self.state.turbulence_until = now + 30.0
            if not self.state.turbulence_said:
                self.state.turbulence_said = True
                self._emit("info",
                           "Rough air: pulled back toward the turbulence "
                           "penetration speed",
                           "autonomous.weather.turbulence")
        elif now > self.state.turbulence_until:
            self.state.turbulence_said = False

        # Windshear is a fixed-wing problem, twice over: on the approach
        # it is a go-around, on the departure it is the escape manoeuvre
        # -- max power, the climb attitude, and the configuration held.
        if self._heli is not None:
            return
        agl = float(getattr(state, "altitude_agl_ft", 0.0) or 0.0)
        if not 0.0 < agl < 1500.0:
            self._wind_z_hist.clear()
            return
        # The headwind, kept for a few polls: fifteen knots appearing or
        # vanishing across it is the textbook shear figure.
        longitudinal = float(getattr(state, "aircraft_wind_z_kt", 0.0) or 0.0)
        hist = self._wind_z_hist
        hist.append(longitudinal)
        del hist[:len(hist) - 8]
        swing = (max(hist) - min(hist)) if len(hist) >= 4 else 0.0
        if not (swing >= 15.0 or (updraft >= 10.0 and agl < 500.0)) \
                or now - self.state.windshear_since <= 20.0:
            return
        if self.state.phase == AutonomousPhase.APPROACH \
                and self.state.sub in ("intercept", "final"):
            self.state.windshear_since = now
            self.state.weather_gas += 1
            self._emit("info", "Windshear: going around",
                       "autonomous.weather.windshear")
            self._start_go_around(state)
        elif self.state.phase in (AutonomousPhase.TAKEOFF,
                                  AutonomousPhase.CLIMB):
            # The departure escape: the flag that makes the FO urgent
            # and holds the configuration, full power, and the climb
            # the phase is already flying -- the book says not to change
            # anything until clear of the shear.
            self.state.windshear_since = now
            self.director.takeoff_power()
            self._last_lever = 1.0
            self._emit("info",
                       "Windshear escape: max power, configuration held",
                       "autonomous.weather.windshear_departure")

    def _field_weather(self, airport):
        """The weather the controllers are quoting for a field, or None.

        It is the engine's merged figure -- the sim's where the field is
        close enough to speak for itself, the observation's elsewhere --
        so the FO's decision matches the wind the tower just read out.
        """
        if airport is None:
            return None
        weather_for = getattr(self.brain, "weather_for", None)
        if weather_for is None:
            return None
        for ident in (getattr(airport, "icao", "") or "",
                      getattr(airport, "ident", "") or ""):
            if not ident:
                continue
            try:
                wx = weather_for(ident)
            except Exception:
                log.debug("weather for %s failed", ident, exc_info=True)
                continue
            if wx is not None:
                return wx
        return None

    @staticmethod
    def _convective(weather) -> bool:
        """Whether the report's sky is convective: a thunderstorm, hail,
        squall or funnel in the present weather, or a CB in the clouds.
        The raw body is scanned as well as the parsed group -- a storm
        in the vicinity still counts."""
        if weather is None:
            return False
        if getattr(weather, "cb", False):
            return True
        text = ((getattr(weather, "raw", "") or "")
                .split(" RMK", 1)[0].upper())
        present = getattr(weather, "present", "") or ""
        if present:
            text += " " + present.upper()
        return bool(re.search(r"(?:^|\s)[-+]?(?:VCTS|TS\w*|GR|SQ|FC)",
                              text))

    def _wind_limits(self) -> tuple[float, float]:
        """(crosswind, tailwind) in knots this class is demonstrated to.

        Nobody publishes the model's own figures, so they are the class's
        -- a transport's thirty off the side and fifteen up the tail, a
        turboprop's twenty-five and ten, a light aircraft's fifteen and
        ten. A helicopter has no runway wind that matters this way.
        """
        if self._is_helicopter:
            return 99.0, 99.0
        cruise = float(getattr(self.performance, "cruise_kt", 0.0) or 0.0)
        if cruise >= 230.0:
            return 30.0, 15.0
        if cruise >= 140.0:
            return 25.0, 10.0
        return 15.0, 10.0

    @staticmethod
    def _runway_wind(weather, heading: float) -> tuple[float, float, float]:
        """(headwind, crosswind, tailwind) in knots on a runway heading.

        The gust is counted, not reported separately: it is the number
        the airframe sees at the worst moment of the flare. A variable
        wind is nobody's crosswind -- it is worked as calm, the same way
        the runway chooser works it.
        """
        if weather is None or getattr(weather, "variable", False):
            return 0.0, 0.0, 0.0
        wind = max(float(getattr(weather, "wind_kt", 0.0) or 0.0),
                   float(getattr(weather, "gust_kt", 0.0) or 0.0))
        if wind < 0.5 or heading <= 0.0:
            return 0.0, 0.0, 0.0
        off = math.radians((float(getattr(weather, "wind_dir", 0.0) or 0.0)
                            - float(heading)) % 360.0)
        head = wind * math.cos(off)
        return max(0.0, head), abs(wind * math.sin(off)), max(0.0, -head)

    def _vapp(self, state) -> float:
        """The approach speed as a crew would fly it: the figure, plus
        half the gust, capped -- the additive a gusty day asks for, so
        the speed that is flown covers the speed that gusts."""
        base = (self.state.vapp_kt
                or (self.performance.touchdown_kt + 5.0
                    if self.performance.touchdown_kt else 0.0)
                or self.performance.approach_kt)
        if not base or not self._weather_reactions:
            return base
        wx = self._field_weather(self.session.destination)
        gust = float(getattr(wx, "gust_kt", 0.0) or 0.0)
        steady = float(getattr(wx, "wind_kt", 0.0) or 0.0)
        if gust <= steady:
            return base
        return base + max(0.0, min((gust - steady) * 0.5, 15.0))

    def _route_diversion(self) -> None:
        """The destination changed under the flight: the plan filed again
        direct, the old field's procedures dropped, and the approach
        machinery reset so the new field is worked out fresh rather
        than on the old one's numbers."""
        self.session.sid = ""
        self.session.descend_via = ""
        self.session.star_join = ""
        self.state.sid_nav = None
        self.state.star_nav = None
        self.state.sid_key = ()
        self.state.star_key = ()
        self.state.procedure_suspended = ""
        if self.use_route is not None:
            try:
                from ..route import from_string, resolver
                dest = self.session.destination
                # The leg is re-planned from where the aeroplane is, not
                # from the new destination back to itself.
                origin = (self._get_current_airport()
                          or self.session.departure)
                origin_ident = getattr(origin, "ident", "") or dest.ident
                route = from_string(
                    f"DCT {dest.ident}", origin_ident, dest.ident,
                    resolver(self.navdb))
                self.use_route(route)
            except Exception:
                log.debug("autonomous enroute route build failed",
                          exc_info=True)
        # The old field's instructions are done with: the assignments,
        # the landing clearance and the approach the tower gave are not
        # carried to the new field.
        session = self.session
        session.assigned_heading = None
        session.assigned_speed_kt = None
        session.assigned_altitude_ft = 0.0
        session.cleared_landing = False
        session.going_around = False
        session.arrival_runway = ""
        if self.state.phase in (AutonomousPhase.CLIMB,
                                AutonomousPhase.CRUISE,
                                AutonomousPhase.DESCENT,
                                AutonomousPhase.APPROACH):
            self.state.runway_heading = 0.0
            self.state.aligned = False
            self.state.stabilized = False
            self.state.configured = ""
            self.state.minima_ft = 200.0
            self.state.approach_recipe = None
            self.state.approach_recipe_tried = False
            self.state.approach_recipe_key = ()
            self.state.approach_tuned = False
            self.state.circling = False
            self.state.circle_path = []
            self.state.tod_nm = 0.0
            self.state.weather_gas = 0
            self.state.weather_ga_active = False
            # The old field's approach scars are done with too: its
            # go-around count, its flare clock, its gear-up call and the
            # missed-approach height the throttle gate still reads. A
            # stale flare clock is the next pass going around the moment
            # it sinks under sixty feet.
            self.state.go_arounds = 0
            self.state.flare_at = 0.0
            self.state.missed_alt_ft = 0.0
            self.state.gear_ga_done = False
            self.state.gear_up_said = False
            self._transition(AutonomousPhase.DESCENT)
            self._goto_sub("")

    def _departure_wind_ok(self, state) -> bool:
        """Whether the wind and the cells let the aeroplane go.

        A crosswind or a tailwind the class is not demonstrated for is
        held for, as a crew holds short for a gusty wind rather than
        rolling in it; a thunderstorm over the field is waited out. Both
        are held for twenty minutes, then called -- a wind that is not
        coming back is a plan that is not flying.
        """
        weather = self._field_weather(
            self.session.departure or self._get_current_airport())
        if weather is None:
            self.state.convective_hold_said = False
            self.state.wind_hold_said = False
            self.state.wind_hold_since = 0.0
            return True
        now = time.monotonic()
        if self._convective(weather):
            if not self.state.convective_hold_said:
                self.state.convective_hold_said = True
                self.state.wind_hold_since = now
                self._emit("info",
                           "Holding departure: thunderstorm at the field",
                           "autonomous.weather.hold")
            elif now - self.state.wind_hold_since > 1200.0:
                self._set_error("Thunderstorm over the field for twenty "
                                "minutes; not departing into it")
            return False
        _head, cross, tail = self._runway_wind(
            weather, self.state.runway_heading)
        xlim, tlim = self._wind_limits()
        if cross > xlim or tail > tlim:
            if not self.state.wind_hold_said:
                self.state.wind_hold_said = True
                self.state.wind_hold_since = now
                self._emit(
                    "info",
                    f"Holding departure: {cross:.0f} kt crosswind, "
                    f"{tail:.0f} kt tailwind (limits {xlim:.0f}/"
                    f"{tlim:.0f})", "autonomous.weather.hold")
            elif now - self.state.wind_hold_since > 1200.0:
                self._set_error("Departure wind beyond limits and not "
                                "improving")
            return False
        # Cleared: the clocks go back to zero, so the next cell or gust
        # is a new hold -- not an instant failure on the old one's time.
        self.state.wind_hold_said = False
        self.state.convective_hold_said = False
        self.state.wind_hold_since = 0.0
        return True

    def _landing_wind_bad(self, state) -> bool:
        """Whether the wind on the landing runway is past the limits."""
        if not self.state.runway_heading:
            return False
        weather = self._field_weather(self.session.destination)
        _head, cross, tail = self._runway_wind(
            weather, self.state.runway_heading)
        xlim, tlim = self._wind_limits()
        return cross > xlim or tail > tlim

    def _weather_below_minima(self, state) -> bool:
        """Whether the field's own report puts the approach out: the
        ceiling under the minima, or the visibility under what the
        approach can be flown to. A field that does not report is not a
        field below minima -- only a report can say that."""
        weather = self._field_weather(self.session.destination)
        if weather is None or not getattr(weather, "raw", ""):
            return False
        ceiling = min((h for cover, h in getattr(weather, "clouds", [])
                       if cover in ("BKN", "OVC", "VV")), default=None)
        minima = float(self.state.minima_ft or 200.0)
        if ceiling is not None and ceiling + 50.0 < minima:
            return True
        recipe = self.state.approach_recipe
        floor_sm = 1.5 if getattr(recipe, "circling", False) else (
            0.75 if getattr(recipe, "family", "") in ("ils", "rnav")
            else 1.0)
        visibility = getattr(weather, "visibility_sm", None)
        # Zero visibility is a report, not a missing one.
        return (10.0 if visibility is None else float(visibility)) < floor_sm

    def _vmc_for_vfr(self, state) -> bool:
        """Whether a VFR leg could actually be released here: the weather
        reported at both ends is at or above the control-zone minima the
        controller judges a release by (``atc.svfr``). A field that does
        not report, or a realism that never refuses, holds nobody back.
        """
        realism = getattr(getattr(self, "brain", None), "realism", None)
        if realism is not None and not getattr(realism, "special_vfr", True):
            return True
        region_for = getattr(getattr(self, "brain", None), "region_for", None)
        for airport in (getattr(self.session, "departure", None)
                        or self._get_current_airport(),
                        getattr(self.session, "destination", None)):
            if airport is None:
                continue
            weather = self._field_weather(airport)
            if weather is None or not getattr(weather, "raw", ""):
                continue
            try:
                from ..svfr import conditions
                region = (region_for(airport.ident)
                          if region_for is not None else "icao")
                if not conditions(
                        weather, region,
                        float(getattr(airport, "lat", 0.0) or 0.0),
                        float(getattr(airport, "lon", 0.0) or 0.0)).vmc:
                    return False
            except Exception:
                log.debug("VMC check for %s failed",
                          getattr(airport, "ident", "?"), exc_info=True)
        return True

    def _ifr_required(self, state) -> bool:
        """Whether the leg is filed and flown IFR, decided the way a crew
        files it: the aircraft's operation first, then the weather it
        would fly in.

        A transport never goes VFR -- the airliner, the business jet and
        the turboprop twin file IFR whatever the sky; a schedule is not
        cancelled for a clear one. A light aeroplane truly may go VFR,
        and does where the weather reported at both ends leaves the
        control zones at VMC -- the same minima the controller answers a
        release by. Below VMC there is no VFR to be had, so the flight
        files IFR. The pilot's own preference (``autonomous_request_ifr``)
        comes first of all: off, only VFR is ever flown, and where the
        weather refuses a release the flight does not go at all.
        """
        if self._is_helicopter:
            return False
        if not self._request_ifr:
            return False
        # The filing outranks the weather and the airframe: a flight
        # plan (SimBrief, the chain's own, or the rules the pilot set
        # on the flight) that says IFR is flown IFR -- a C172 on an IFR
        # plan asks for the clearance it was filed under, not a VFR
        # release the sky happens to allow.
        if getattr(self.session, "ifr", False):
            return True
        from .. import aircraft_class as ac
        cls = getattr(self, "_aircraft_class", "") or ac.AIRLINER
        if cls in (ac.AIRLINER, ac.BUSINESS_JET, ac.TURBOPROP_TWIN):
            return True
        return not self._vmc_for_vfr(state)

    def _divert_for_weather(self, state) -> None:
        """Two attempts at a destination the weather keeps shutting: the
        alternate gets the flight instead, said like a pilot would."""
        dest = self._pick_alternate(state)
        if dest is None:
            self.state.weather_gas = 0
            self._emit("error",
                       "Destination weather below limits and no alternate "
                       "in reach; trying the approach again",
                       "autonomous.weather.no_alternate")
            return
        self._emit("info",
                   f"Weather below limits at the destination: diverting "
                   f"to {dest.ident}", "autonomous.weather.divert")
        if self.crew.transmit:
            with contextlib.suppress(Exception):
                self.crew.transmit(
                    f"diverting to {dest.ident}, weather below minima")
        self.session.destination = dest
        self.state.weather_gas = 0
        self._route_diversion()

