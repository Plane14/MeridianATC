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
        self._watch_ifr_pickup(state, now)
        self._watch_enroute_convection(state, now)

    def _enroute_weather_cells(self, state):
        """Current enroute hazards: simulator-native where it exists, and
        explicit external convective evidence as the MSFS strategic fallback."""
        from ...weather.enroute import xplane_cells

        cells = []
        radar_points = getattr(getattr(self, "sim", None),
                               "weather_radar_points", None)
        if callable(radar_points):
            try:
                cells.extend(xplane_cells(
                    radar_points(), getattr(state, "altitude_ft", 0.0)))
            except Exception:
                log.debug("X-Plane weather-radar scan failed", exc_info=True)
        provider = getattr(self, "enroute_weather", None)
        if provider is not None and getattr(state, "has_position", False):
            provider.update(state.latitude, state.longitude,
                            getattr(state, "altitude_ft", 0.0))
            cells.extend(provider.cells())
        # Current-point evidence is an immediate corroborating override, not
        # a scan: hail or a published CB type is convective evidence, while
        # MSFS rain alone is not. On MSFS the heavy-rain rate has to agree
        # with a strong world-vertical draft before it is treated as a cell.
        from ...weather.enroute import HazardCell

        point_hail = float(getattr(state, "hail_ratio", 0.0) or 0.0) > 0.05
        point_cb = float(getattr(state, "cloud_type_max", 0.0) or 0.0) >= 2.6 \
            and (float(getattr(state, "turbulence_factor", 0.0) or 0.0) >= 4.0
                 or int(getattr(state, "ambient_precip_state", 0) or 0) > 0)
        msfs_convection = getattr(state, "source", "") == "simconnect" \
            and float(getattr(state, "ambient_precip_rate_mmh", 0.0) or 0.0) >= 4.0 \
            and abs(float(getattr(state, "ambient_wind_y_kt", 0.0) or 0.0)) >= 8.0
        if (point_hail or point_cb or msfs_convection) \
                and getattr(state, "has_position", False):
            reason = "hail/CB evidence at the aircraft" \
                if point_hail or point_cb else "heavy rain with a strong vertical draft"
            cells.append(HazardCell(
                state.latitude, state.longitude, 5.0, 0.8, "point-evidence",
                reason, top_ft=float(getattr(state, "altitude_ft", 0.0) or 0.0)))
        return cells

    @staticmethod
    def _weather_source_label(plan) -> str:
        """Human provenance for a hazard source.

        AWC evidence is useful strategy, not the simulator's own precipitation
        field, so it is never allowed to masquerade as a cockpit radar scan.
        """
        if str(getattr(plan, "source", "")).startswith("awc-"):
            return "external approximate weather"
        return str(getattr(plan, "source", "weather"))

    def _enroute_weather_plan(self, state):
        """The avoidance decision ahead of the aircraft, if there is one."""
        from ...weather.enroute import avoidance_plan

        if not getattr(state, "has_position", False):
            return None
        course = float(getattr(state, "track_true", 0.0)
                       or getattr(state, "heading_true", 0.0) or 0.0)
        return avoidance_plan(
            self._enroute_weather_cells(state), state.latitude,
            state.longitude, course, getattr(state, "altitude_ft", 0.0))

    def _watch_enroute_convection(self, state, now: float) -> None:
        """Scan ahead and ask ATC for a weather deviation before entering
        a convective cell, as an IFR crew should. If penetration is imminent
        and no answer has come, safety outranks the clearance and the crew
        deviates while telling ATC."""
        if self._is_helicopter or self._heli is not None \
                or self.state.phase not in (AutonomousPhase.CLIMB,
                                            AutonomousPhase.CRUISE,
                                            AutonomousPhase.DESCENT) \
                or getattr(state, "on_ground", True):
            return
        session = self.session
        if session is None:
            return
        active = bool(getattr(session, "deviating", "")
                      and getattr(session, "weather_deviation_at", 0.0))
        if active:
            self._weather_deviation_requested_at = 0.0
            return
        plan = self._enroute_weather_plan(state)
        if plan is None or not plan.threatening:
            self._weather_deviation_requested_at = 0.0
            return
        asked = float(getattr(self, "_weather_deviation_requested_at", 0.0) or 0.0)
        transmission_failed = False
        if not asked:
            self._weather_deviation_plan = plan
            transmitted = self._fo_say("weather_deviation")
            # Start the clock even when nobody can take the call. Otherwise a
            # CTAF, a dead frequency, or a congested one would leave the crew
            # flying into the cell while it waited for a timer that never ran.
            self._weather_deviation_requested_at = now
            asked = now
            if transmitted:
                self._emit("info",
                           f"Convective weather {plan.nearest_nm:.0f} miles ahead "
                           f"({self._weather_source_label(plan)}): requesting {plan.degrees} degrees {plan.side}",
                           "autonomous.weather.deviation_request")
                return
            transmission_failed = True
        # A quiet frequency is acceptable when the weather is still far away;
        # it is not acceptable with the cell at the windshield. Maneuver and
        # inform ATC, as the pilot-in-command ultimately must. When the call
        # cannot be made at all and the cell is already close, there is no
        # controller whose answer is worth waiting for.
        close = plan.nearest_nm <= 12.0
        if (not transmission_failed or not close) and (
                now - asked < 18.0 or not close):
            return
        course = float(getattr(state, "track_true", 0.0)
                       or getattr(state, "heading_true", 0.0) or 0.0)
        session.deviating = "approved"
        session.weather_deviation_side = plan.side
        session.weather_deviation_degrees = plan.degrees
        session.weather_deviation_miles_nm = plan.miles_nm
        session.weather_deviation_course = course
        session.weather_deviation_at = now
        session.weather_deviation_urgent = True
        self._urgent_deviation_side = plan.side
        self._fo_say("deviating_weather")
        self._emit("warning",
                   "No ATC answer with convection imminent: deviating and advising",
                   "autonomous.weather.deviation_urgent")

    def _watch_ifr_pickup(self, state, now: float) -> None:
        """A VFR leg flown into cloud files IFR, the pop-up pickup.

        The sim's word is the honest one: ``ambient_in_cloud`` is the
        sky the aeroplane is actually in, not the METAR it was filed
        against. A wisp of cloud is not a reason to change the flight's
        rules -- the clock must see a stay in it -- but a leg flown into
        sustained IMC on a VFR filing is a flight that needs a clearance,
        so the FO files it: ``session.ifr`` flips, and the ask goes out
        on the frequency the way the ground one did (``_work_the_calls``
        makes the call until somebody issues it, or announces it once
        where nobody can). A helicopter, a circuit flight, a leg with
        nowhere to be cleared to, and the pilot's "VFR only" preference
        are all left alone.
        """
        session = self.session
        if (session is None or self._is_helicopter or self._heli is not None
                or not self._request_ifr
                or getattr(session, "ifr", False)
                or getattr(session, "pattern_work", False)
                or getattr(session, "destination", None) is None
                or getattr(state, "on_ground", True)
                or self.state.phase not in (AutonomousPhase.CLIMB,
                                            AutonomousPhase.CRUISE,
                                            AutonomousPhase.DESCENT)):
            self.state.imc_since = 0.0
            return
        if not getattr(state, "ambient_in_cloud", False):
            self.state.imc_since = 0.0
            return
        if not self.state.imc_since:
            self.state.imc_since = now
            return
        if now - self.state.imc_since < 25.0:
            return
        session.ifr = True
        flight_cfg = getattr(getattr(self, "config", None), "flight", None)
        if flight_cfg is not None:
            # The flight is filed IFR from here: the settings screen
            # should say so, the way the preflight filing writes back.
            flight_cfg.ifr = True
        self._emit("info", "In cloud on a VFR leg: picking up IFR",
                   "autonomous.weather.ifr_pickup")

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
        density_alt = self._density_altitude(state)
        if density_alt is None:
            return
        oat_raw = getattr(state, "ambient_temperature_c", None)
        oat = 15.0 if oat_raw is None else float(oat_raw)
        if density_alt - elev < 2500.0 and density_alt < 7000.0:
            return
        self.state.density_alt_said = True
        self._emit(
            "info",
            f"Density altitude {density_alt:,.0f} ft (field "
            f"{elev:,.0f} ft, {oat:.0f} C): expect a longer takeoff",
            "autonomous.weather.density")

    def _density_altitude(self, state):
        """The field's density altitude in feet, or None unsolved.

        Pressure altitude corrected for temperature: the altitude the
        airframe performs at rather than the one it sits at -- a five-
        thousand-foot runway on a thirty-degree day flies like eight.
        """
        elev = self._field_elevation(state)
        if elev <= 0.0:
            return None
        oat_raw = getattr(state, "ambient_temperature_c", None)
        # Zero Celsius is a reading, not a missing one -- ``or 15`` would
        # warm a freezing day into a standard one.
        oat = 15.0 if oat_raw is None else float(oat_raw)
        isa = 15.0 - 2.0 * (elev / 1000.0)
        return elev + 120.0 * (oat - isa)

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
        if kit.carb_heat and self._any_engine_running(state):
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
        vertical = float(getattr(state, "ambient_wind_y_kt", 0.0) or 0.0) \
            if "ambient_wind_y_kt" not in getattr(state, "unavailable", ()) \
            else float(getattr(state, "aircraft_wind_y_kt", 0.0) or 0.0)
        updraft = abs(vertical)
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

    def _fly_weather_deviation(self, state) -> None:
        """Fly the lateral limits of an approved weather deviation.

        The clearance controls only lateral manoeuvring: the phase machine and
        altitude logic keep their normal vertical assignment. When the approved
        lateral distance has been covered, the route is resumed direct the
        clearance's rejoin fix where one was issued.
        """
        session = getattr(self, "session", None)
        if session is None or not getattr(session, "weather_deviation_at", 0.0):
            return
        if getattr(state, "on_ground", True):
            return
        miles = float(getattr(session, "weather_deviation_miles_nm", 0.0) or 15.0)
        gs = max(80.0, float(getattr(state, "ground_speed_kt", 0.0) or 80.0))
        elapsed = time.monotonic() - float(session.weather_deviation_at)
        needed = miles / gs * 3600.0 + 60.0
        clear = True
        if elapsed >= needed:
            try:
                ahead = self._enroute_weather_plan(state)
                clear = ahead is None or not ahead.threatening
            except Exception:
                log.debug("weather-deviation completion scan failed",
                          exc_info=True)
                clear = False
        # "When able" means when the route ahead is clear, not merely when a
        # timer ran out. A hard upper bound keeps a stale external strategic
        # polygon from owning the rest of the flight; that fallback is said in
        # the event rather than silently treated as a precision radar track.
        timed_out = elapsed >= needed + 600.0
        if elapsed >= needed and (clear or timed_out):
            rejoin = getattr(session, "weather_deviation_rejoin", "") or ""
            if rejoin:
                session.route_direct = rejoin
            session.deviating = ""
            session.weather_deviation_side = ""
            session.weather_deviation_degrees = 0
            session.weather_deviation_miles_nm = 0
            session.weather_deviation_rejoin = ""
            session.weather_deviation_at = 0.0
            session.weather_deviation_course = -1.0
            session.weather_deviation_urgent = False
            self._fo_say("clear_of_weather")
            self._emit("info",
                       "Clear of the weather deviation: resuming the route",
                       "autonomous.weather.deviation_complete")
            return
        degrees = int(getattr(session, "weather_deviation_degrees", 0) or 30)
        side = (getattr(session, "weather_deviation_side", "") or "left").lower()
        sign = -1.0 if side.startswith("l") else 1.0
        course = float(getattr(session, "weather_deviation_course", -1.0) or -1.0)
        if course < 0.0:
            course = float(getattr(state, "track_true", 0.0)
                           or getattr(state, "heading_true", 0.0) or 0.0)
        target = (course + sign * degrees + 360.0) % 360.0
        self.director.heading(self._magnetic(target, state))
        self.state.ap_hdg_mode = True

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
        # The tower's orbit was spacing at the old field: the emergency
        # arrival at the new one does not resume it, at the height it
        # began or the turn it had left. The direct to a fix on the old
        # route is not the new route's either.
        session.orbiting = ""
        session.orbit_track = -1.0
        session.orbit_turned = 0.0
        session.route_direct = ""
        session.route_direct_index = -1
        self.state.orbit_alt_ft = 0.0
        self.state.orbit_track = -1.0
        self.state.orbit_turned = 0.0
        self._end_direct()
        self.state.direct_cleared = ""
        # The rejoin circuit and wind crab were learned on the old
        # field's geometry: left standing, the approach would fly the
        # old airport's downwind point.
        self._rejoin_path = None
        self._crab_deg = 0.0
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
            self.state.circle_index = 0
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
        # A cell overhead is waited out while the pilot's setting says to
        # wait; flown through when it says to go -- the demonstrated wind
        # limits below are the envelope and hold either way. Read live:
        # flipping it off mid-hold releases the flight without a restart.
        if getattr(self.config.immersion, "autonomous_departure_hold",
                   True) and self._convective(weather):
            mode = str(getattr(self.config.immersion,
                               "autonomous_departure_weather_mode", "hold")
                       or "hold").lower()
            if mode == "deviate":
                # A cell over the airfield is not automatically a cell on the
                # first climb-out miles. Only a demonstrably clear short
                # corridor releases the departure; an incomplete or positive
                # scan keeps the hold.
                from ...weather.enroute import scan_route
                course = float(getattr(self.state, "runway_heading", 0.0) or 0.0)
                # Only a fresh simulator radar scan can demonstrate that the
                # first climb-out miles are clear. External METAR/SIGMET
                # evidence is strategic: it can add a hazard, but its absence
                # cannot prove a thunderstorm-scale hole in the corridor.
                wall_now = time.time()
                evidence = False
                radar_points = getattr(getattr(self, "sim", None),
                                       "weather_radar_points", None)
                if callable(radar_points):
                    try:
                        evidence = any(wall_now - point.at < 120.0
                                       for point in radar_points())
                    except Exception:
                        evidence = False
                cells = self._enroute_weather_cells(state)
                if evidence:
                    # Where the simulator's own scan is fresh, a coarse METAR
                    # point at the field is superseded as local geometry; the
                    # tactical cells and explicit larger-scale polygons count.
                    cells = [cell for cell in cells
                             if cell.source != "awc-metar"]
                hits, _coverage = scan_route(
                    cells, state.latitude, state.longitude, course,
                    ahead_nm=15.0, width_nm=7.0)
                if not hits and getattr(state, "has_position", False) and evidence:
                    if not getattr(self, "_departure_deviation_said", False):
                        self._departure_deviation_said = True
                        self._emit("info",
                                   "Thunderstorm at the field, but the departure corridor is clear: departing",
                                   "autonomous.weather.departure_deviate")
                    # The convective hold is answered; only the demonstrated
                    # wind checks below still apply.
                    mode = "bypass"
                else:
                    mode = "hold"
            if mode != "bypass":
                if not self.state.convective_hold_said:
                    self.state.convective_hold_said = True
                    self.state.wind_hold_since = now
                    log.info("holding departure: thunderstorm at the field")
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
                log.info("holding departure: %.0f kt crosswind, %.0f kt "
                         "tailwind (limits %.0f/%.0f)",
                         cross, tail, xlim, tlim)
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
                from ..svfr import _VMC, _VMC_DEFAULT, visibility_m
                region = (region_for(airport.ident)
                          if region_for is not None else "icao")
                vmc_ceiling, vmc_visibility = _VMC.get(
                    region, _VMC_DEFAULT)
                # The filing's ceiling is the observation's, not the
                # simulator's sky: ``ceiling_ft`` blanks the layers
                # when the merged weather is sim-sourced because the
                # *release* is judged against the sky the plane flies
                # in -- but the filing is paperwork, and the paperwork
                # is the METAR. A field reporting OVC004 files IFR even
                # if someone painted the sim's sky clear.
                ceiling = min(
                    (float(h) for cover, h in
                     getattr(weather, "clouds", None) or []
                     if cover in ("BKN", "OVC", "VV")),
                    default=float("inf"))
                if ceiling < vmc_ceiling \
                        or visibility_m(weather) < vmc_visibility:
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
        if not self._request_ifr:
            return False
        # The filing outranks the weather and the airframe: a flight
        # plan (SimBrief, the chain's own, or the rules the pilot set
        # on the flight) that says IFR is flown IFR -- a C172 on an IFR
        # plan asks for the clearance it was filed under, not a VFR
        # release the sky happens to allow, and a helicopter's plan is
        # as much a plan as a wing's is.
        if getattr(self.session, "ifr", False):
            return True
        if self._is_helicopter:
            return False
        from .. import aircraft_class as ac
        cls = getattr(self, "_aircraft_class", "") or ac.AIRLINER
        if cls in (ac.AIRLINER, ac.BUSINESS_JET, ac.TURBOPROP_TWIN):
            return True
        # Night is the other thing a light pilot files around: outside
        # the US the rating to fly VFR in the dark is its own paper
        # (SERA.5010 -- the FAA's 91.155 keeps night VFR ordinary), so
        # the Skyhawk over Calais after sunset goes on a plan like the
        # one it would file for cloud.
        if cls in (ac.PISTON_SINGLE, ac.PISTON_TWIN, ac.TURBOPROP_SINGLE):
            airport = (getattr(self.session, "departure", None)
                       or self._get_current_airport())
            region_for = getattr(getattr(self, "brain", None),
                                 "region_for", None)
            region = (region_for(getattr(airport, "ident", "") or "")
                      if region_for is not None else "icao")
            if region != "faa":
                from ..svfr import is_night
                lat = (float(getattr(state, "latitude", 0.0) or 0.0)
                       or float(getattr(airport, "lat", 0.0) or 0.0))
                lon = (float(getattr(state, "longitude", 0.0) or 0.0)
                       or float(getattr(airport, "lon", 0.0) or 0.0))
                if is_night(lat, lon, region):
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

