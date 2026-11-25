from __future__ import annotations

from ._shared import (
    Aircraft,
    AircraftState,
    FlightProfile,
    FlightSession,
    NotAFlightPlan,
    Phase,
    autofly_limit,
    capabilities,
    decision,
    every_language,
    haversine_nm,
    log,
    wake_suffix,
)


class _Session:
    # ------------------------------------------------------------------
    # session
    # ------------------------------------------------------------------

    # How far the aeroplane may be from the field the flight says it is
    # departing before the field is treated as wrong rather than the aeroplane
    # as taxiing. Generous: a long taxi at a big airport is a few miles, and a
    # flight that is already airborne on the way somewhere is not a mistake.
    DEPARTURE_TOLERANCE_NM = 30.0

    def _plausible_departure(self, airport) -> bool:
        """Whether the aircraft could actually be departing this field.

        Airborne is always plausible: a flight can be an hour out of the field
        it departed. It is only an aeroplane sitting on the ground somewhere
        else that says the departure field is stale.
        """
        if airport is None or not self.state.has_position:
            return True
        if not self.state.on_ground:
            return True
        distance = haversine_nm(self.state.latitude, self.state.longitude,
                                airport.lat, airport.lon)
        return distance <= self.DEPARTURE_TOLERANCE_NM

    def use_region(self) -> None:
        """A region pinned or unpinned on the screen, from the next
        transmission, without rebuilding the flight."""
        profile = FlightProfile.of(self.config.flight)
        self.brain.profile = profile
        self.traffic.profile = profile
        if self.session is not None:
            self.session.profile = profile

    def _rules_for(self, flight) -> bool:
        """The rules a rebuilt session flies under.

        Every save of the flight, every reconnection of the simulator and
        every setting under ``flight`` rebuilds the session, and each one
        used to put the rules back to the setting -- so a VFR release, or
        "IFR" said on the radio, lasted until the pilot next pressed Save.
        What the radio changed is kept unless the setting itself has moved
        since the session was built, which is the pilot changing them.
        """
        old = self.session
        if old is not None and old.ifr_setting == flight.ifr:
            return old.ifr
        return flight.ifr

    def begin_session(self) -> FlightSession:
        """Build the flight session from the sim state and the config."""
        with self._text_lock:
            return self._begin_session()

    def _begin_session(self) -> FlightSession:
        flight = self.config.flight
        profile = FlightProfile.of(flight)
        # The simulator is the authority on what the aeroplane is (the type
        # correction below), and what it says can disagree with the profile
        # outright: a Baron is not an odd airliner, it is a wrong profile.
        # Where what is flown says so plainly the profile follows it, is
        # written back so the settings screen stops disagreeing, and is
        # told -- the way the type and the departure are. An unknown class
        # says nothing (``questioned`` returns "" for a guess).
        from ..atc import aircraft_class

        reported = aircraft_class.classify(
            self.state.type_code or flight.aircraft_type or "",
            engine_type=(self.state.engine_type
                         if getattr(self.state, "engine_type", -1) >= 0
                         else None),
            engines=getattr(self.state, "engine_count", 0) or 0,
            category=getattr(self.state, "category", ""),
            title=getattr(self.state, "title", ""),
            airline=profile.airliner)
        question = profile.questioned(reported)
        if question:
            self.emit(
                "info",
                f"The simulator has a {reported.name.replace('_', ' ')}, "
                f"so the {profile.kind.replace('_', ' ')} profile is now "
                f"{question.replace('_', ' ')}.",
                key="log.profile_moved", profile=question,
                was=profile.kind, aircraft_class=reported.name)
            self.config.flight.profile = question
            flight = self.config.flight
            profile = FlightProfile.of(flight)
        self.brain.profile = profile
        self.traffic.profile = profile
        # Whether this is an airline flight is the pilot's to say. In general
        # aviation the simulator's airline-and-flight-number callsign is the
        # livery's, not the flight's, so it is the tail -- unless the pilot
        # typed a company callsign themselves (an EJA123 business jet).
        if profile.airliner:
            raw_callsign = flight.callsign or self.state.callsign or "N123AB"
            as_airline = True
        else:
            raw_callsign = (flight.callsign or self.state.tail_number
                            or self.state.callsign or "N123AB")
            as_airline = bool(flight.callsign)

        # The aircraft type follows the simulator, not the config file.
        #
        # This is not a preference the way the callsign is. The type decides
        # the wake-turbulence suffix, how the controller sequences you and
        # what it expects you to be able to do, and there is no version of
        # "I am a Cessna 172" that is true while the simulator is holding an
        # A321. A typed type is the fallback for when the simulator is not
        # there, and it is corrected -- and written back, so the settings
        # screen stops disagreeing with the aeroplane -- when it is.
        type_code = self.state.type_code or flight.aircraft_type or ""
        if (self.state.type_code
                and flight.aircraft_type != self.state.type_code):
            if flight.aircraft_type:
                self.emit("info",
                          f"The simulator has a {self.state.type_code}, not a "
                          f"{flight.aircraft_type}.",
                          key="log.type_moved", type=self.state.type_code,
                          was=flight.aircraft_type)
            self.config.flight.aircraft_type = self.state.type_code
            flight = self.config.flight
            try:
                # The file is corrected too: an in-memory write-back left
                # the saved type naming the last aeroplane, and every
                # cold start announced it until the simulator answered.
                self.config.save()
            except Exception as exc:
                log.warning("could not write config.yaml: %s", exc)

        departure = self.navdb.airport(flight.departure) if flight.departure else None
        if self.state.has_position:
            under = self.navdb.home_airport(self.state.latitude, self.state.longitude)
            if departure is None:
                # The field below answers "where did it start" only on the
                # ground: airborne, the aeroplane is over whatever is under
                # it, and that field is not where it came from -- leave it
                # unset rather than naming the wrong departure.
                if self.state.on_ground:
                    departure = under
            elif not self._plausible_departure(departure):
                # The aeroplane is not where the flight says it is starting.
                # The simulator is the authority on where it is, so the typed
                # field is corrected rather than believed -- otherwise the
                # panel reports an airport half a continent away, resolves
                # every frequency against it, and nothing on the radio works.
                moved = under or self.navdb.nearest_airport(
                    self.state.latitude, self.state.longitude, max_nm=60.0)
                if moved is not None and moved.ident != departure.ident:
                    self.emit(
                        "info",
                        f"The aircraft is at {moved.ident}, not "
                        f"{departure.ident}. Using {moved.ident}.",
                        key="log.departure_moved",
                        airport=moved.ident, was=departure.ident,
                    )
                    departure = moved
                    self.config.flight.departure = moved.ident
                    flight = self.config.flight

        # Without a simulator there is no position, so nothing resolves and the
        # radio is dead. If a departure airport was named, park the aircraft
        # there: it is the only place the flight could sensibly be starting.
        if (departure is not None and not self.state.has_position
                and hasattr(self.source, "update")):
            self.source.update(
                latitude=departure.lat, longitude=departure.lon,
                altitude_ft=departure.elev_ft, on_ground=True,
            )
            self.refresh_state()
            self.emit("info", f"No simulator, so placing the aircraft at "
                              f"{departure.ident}.",
                      key="log.placed", airport=departure.ident)

        # The callsign in every language a controller might use, built once.
        # Which one gets used is decided per transmission, by where the
        # aeroplane is and what the pilot said.
        #
        # Built here rather than earlier because it needs the departure field,
        # and the departure field is not settled until the aeroplane has been
        # found: where the flight starts decides whether English groups the
        # flight number into words -- "Delta twelve thirty four" -- or reads it
        # out in numerals. The group form is a US rule and was being applied
        # everywhere, so an English-speaking controller in Czech airspace said
        # it the American way.
        alternatives = every_language(
            raw_callsign, type_code,
            dialect=profile.dialect_for(departure.ident if departure else ""),
            airline=as_airline)
        callsign = alternatives["en"]
        aircraft = Aircraft(callsign, type_code, wake_suffix(type_code),
                            callsigns=alternatives,
                            wake_every=(profile.dialect_for(
                                departure.ident if departure else "")
                                == "faa"),
                            on_floats=bool(getattr(self.state,
                                                   "on_floats", False)))

        destination = (
            self.navdb.airport(flight.destination) if flight.destination else None
        )

        session = FlightSession(
            aircraft=aircraft,
            departure=departure,
            destination=destination,
            destination_input=flight.destination,
            cruise_altitude_ft=max(
                float(flight.cruise_altitude_ft or 0), 0.0),
            ifr=self._rules_for(flight),
            ifr_setting=flight.ifr,
            profile=profile,
        )
        # A VFR flight sets its region's conspicuity code at power-up --
        # 1200 across North America/Australia/NZ, 1400 in Japan, the
        # ICAO 7000 anywhere else (navdata.db.vfr_squawk_code).
        from ..navdata.db import vfr_squawk_code
        session.squawk = vfr_squawk_code(
            getattr(departure, "ident", "") or "")
        session.phase = Phase.PREFLIGHT if self.state.on_ground else Phase.ENROUTE
        self.session = session

        if self.history is not None:
            try:
                self.history.open_session(
                    departure=getattr(departure, "ident", "") or "",
                    destination=getattr(destination, "ident", "")
                    or flight.destination or "",
                    callsign=callsign.spoken_full
                    if hasattr(callsign, "spoken_full") else str(callsign),
                    aircraft=type_code or "",
                    rules="ifr" if session.ifr else "vfr")
            except Exception:
                log.debug("history session open failed", exc_info=True)

        self.understanding.set_callsign(aircraft.variants())
        # A new flight is a new crew: whatever the last one had already said
        # is forgotten, or the safety briefing never happens again.
        self.crew.begin(session, self.last_language)
        # Initialize autonomous flight for this session
        if self.autonomous_flight:
            self.autonomous_flight.begin_flight(session)
        self._check_announcements()
        # And a new runway, so a departure held on the last flight is not
        # still waiting for it to clear on this one.
        self.departures.began()
        self._centre_looked_at = 0.0
        self._centre_checked_at = 0.0
        self._sector_noted = None

        where = departure.ident if departure else "an unknown field"
        self.emit(
            "info",
            f"Flight {callsign.written} ({callsign.spoken_full}) at {where}"
            + (f", destination {destination.ident}." if destination else "."),
            key=("log.flight_to" if destination else "log.flight"),
            callsign=callsign.written, spoken=callsign.spoken_full,
            airport=where,
            destination=(destination.ident if destination else ""),
        )
        if departure is not None:
            self.follow_languages(departure.ident, announce=True)
        self.route_progress = self.brain.route_progress = None
        if not self.route and session.ifr:
            # Said, because every distance the controller uses is then to the
            # airport's reference point rather than along a route.
            decision("route", event="none",
                     why="no flight plan loaded: distances are to the airport")
        return session

    def engage_autonomous(self) -> tuple[bool, str]:
        """The panel's engage: the First Officer takes the aeroplane now.

        The machine is always built; what an engage does is start it
        wherever the flight already is -- parked, taxiing or airborne --
        rather than only at a session's beginning.
        """
        message = autofly_limit(self.state)
        if message:
            self.emit("info", message, key="log.autofly_unavailable")
            return False, message
        fo = self.autonomous_flight
        if fo is None:
            return False, "autonomous flight is not available"
        try:
            if not fo.engage(self.session):
                # No session yet: the flag is set and the next
                # begin_session starts the machine -- the button is not
                # refused for want of a flight already in progress.
                fo.enable()
        except Exception as exc:
            log.debug("autonomous engage failed", exc_info=True)
            return False, str(exc)
        if capabilities(self.state.source).flight_axes:
            self.emit("info", "First Officer flying", key="fo.engaged")
        else:
            self.emit("info", "First Officer airborne autopilot assistance engaged; "
                      "taxi, takeoff, landing, go-around and rotorcraft controls remain manual.",
                      key="fo.ap_only_engaged")
        return True, ""

    def disengage_autonomous(self) -> tuple[bool, str]:
        """The panel's stop: the FO lets go, controls handed back neutral."""
        fo = self.autonomous_flight
        if fo is None:
            return True, ""
        try:
            fo.disengage()
        except Exception as exc:
            log.debug("autonomous disengage failed", exc_info=True)
            return False, str(exc)
        self.emit("info", "First Officer has stopped flying",
                  key="fo.disengaged")
        return True, ""

    def autonomous_status(self) -> dict:
        """What the panel shows in the FO button."""
        reason = autofly_limit(self.state)
        if reason:
            return {"available": False, "enabled": False, "phase": "", "sub": "",
                    "helicopter": self.state.category == "Helicopter", "error": reason}
        fo = self.autonomous_flight
        if fo is None:
            return {"available": False, "enabled": False, "phase": "",
                    "sub": "", "helicopter": False, "error": ""}
        try:
            return fo.status()
        except Exception:
            log.debug("autonomous status raised", exc_info=True)
            return {"available": False, "enabled": False, "phase": "",
                    "sub": "", "helicopter": False, "error": ""}

    def use_route(self, route) -> None:
        """Take a route as the flight's, and say so in the transcript."""
        self.route = route if route else None
        self.brain.route = self.route
        self.route_progress = self.brain.route_progress = None
        if self.route is not None:
            # Whatever the plan filed or was cleared for: the field's
            # procedure list is asked once, on the background thread.
            self._ask_for_terminal_procedures(self.route.origin)
            self._ask_for_terminal_procedures(self.route.destination)
        if self.route is None:
            decision("route", event="none", source=getattr(route, "source", ""),
                     why="the plan has fewer than two placed fixes")
            return
        decision("route", event="loaded", source=route.source,
                 route=route.described(), fixes=len(route.fixes),
                 sid=route.sid, sid_runway=route.sid_runway, star=route.star,
                 star_runway=route.star_runway, approach=route.approach,
                 cruise_ft=route.cruise_ft,
                 toc=getattr(route.top_of_climb, "ident", ""),
                 tod=getattr(route.top_of_descent, "ident", ""),
                 floor_ft=route.arrival_floor_ft or 0,
                 notes=list(route.notes))
        self._upload_fms_plan(self.route)

    def load_route_file(self, path) -> str:
        """A simulator flight plan (``.pln``) on disk as the flight's route."""
        from pathlib import Path

        return self.load_route_text(
            Path(path).read_text(encoding="utf-8-sig", errors="replace"))

    def load_route_text(self, text: str) -> str:
        """A simulator flight plan (``.pln``) as the flight's route: the
        file's text, which is what the panel's button sends.

        Each fix the file does not place itself is asked of the simulator,
        then of the database (atc/route.py ``resolver``); what neither knows
        is listed in the route's notes. Returns the route, described. Raises
        :class:`NotAFlightPlan` for a file that is not one, so the panel can
        say so in the pilot's language rather than show a parser's error.
        """
        from xml.etree.ElementTree import ParseError

        from ..atc.route import from_pln, resolver

        sim = None
        if self.config.sim.enabled:
            from ..sim.capabilities import capabilities
            if capabilities(self.state.source).facilities:
                from ..integrations.fixes import SimFixes

                sim = SimFixes(self.config.sim.simconnect_dll)
            elif self.state.source == "xplane":
                from ..integrations.xpground import XPlaneFixes

                sim = XPlaneFixes(
                    sysroot=getattr(self.simtraffic, "sysroot", ""))
        departure = self.navdb.airport(self.config.flight.departure or "")
        near = (departure.lat, departure.lon) if departure else None
        try:
            route = from_pln(text, resolver(self.navdb, sim, near=near))
        except ParseError as exc:
            raise NotAFlightPlan(str(exc)) from exc
        if not (route.origin or route.destination or route.fixes):
            raise NotAFlightPlan("no departure, destination or fix")
        flight = self.config.flight
        if route.origin:
            flight.departure = route.origin
        if route.destination:
            flight.destination = route.destination
        if route.cruise_ft and route.cruise_ft > 0:
            flight.cruise_altitude_ft = route.cruise_ft
        self.brain.plan_runways = {}
        if route.origin:
            self.brain.plan_runways[route.origin] = {
                "departing": route.sid_runway, "sid": route.sid}
        if route.destination:
            self.brain.plan_runways.setdefault(route.destination, {})
            self.brain.plan_runways[route.destination]["landing"] = \
                route.star_runway
        self.use_route(route)
        if self.session is not None:
            self.begin_session()
        return route.described()

    def _follow_route(self, state: AircraftState) -> None:
        """Where the aeroplane is along the route; a row at each new leg."""
        route = self.route
        if not route or state.on_ground:
            return
        last = self.route_progress
        found = route.progress(state.latitude, state.longitude,
                               state.ground_speed_kt or 0.0,
                               hint=last.leg if last is not None else 1)
        if found is None:
            return
        self.route_progress = found
        self.brain.route_progress = found
        if last is None or found.leg != last.leg:
            decision("route", event="leg", leg=found.leg_name,
                     via=found.to_fix.via,
                     cross_track_nm=round(found.cross_track_nm, 1),
                     to_go_nm=round(found.to_go_nm, 1),
                     altitude_ft=round(state.altitude_ft))

