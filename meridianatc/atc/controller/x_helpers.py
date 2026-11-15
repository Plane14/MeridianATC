from __future__ import annotations

from ._shared import (
    DEFAULT_TRANSITION_FT,
    DEPARTING_WITHIN_NM,
    FlightSession,
    Instruction,
    Phase,
    Reply,
    Station,
    _degrees_said,
    _side_said,
    _similar,
    bearing_deg,
    decision,
    flowmod,
    language_of,
    math,
    normalize_runway,
    re,
    station_key,
    time,
    transition_altitude,
)


class _Helpers:
    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _station_name(station: Station, phraseology, short: bool = False) -> str:
        """A station's callup name in the language currently being spoken."""
        if station is None:
            return ""
        return station.callsign_in(language_of(phraseology), short=short)

    def _planned_sid(self, airport, weather) -> str:
        """The plan's SID, while the runway it was planned for is in use."""
        planned = self.plan_runways.get(getattr(airport, "ident", ""), {})
        sid = planned.get("sid", "")
        if not sid or weather is None:
            return ""
        runway = planned.get("departing", "")
        if runway and self._departure_runway(airport, weather) != runway:
            return ""
        return sid

    def _planned(self, airport, weather, which: str) -> str:
        """The plan's runway at this field, if the wind leaves it usable."""
        planned = self.plan_runways.get(getattr(airport, "ident", ""), {})
        runway = planned.get(which, "")
        if not runway or weather is None:
            return ""
        if not self.navdb.runway_within_limits(
                airport.ident, runway, weather.wind_dir, weather.wind_kt,
                gust_kt=weather.gust_kt, variable=weather.variable):
            return ""
        return runway

    def _flow_runway(self, airport, weather, *, departing: bool) -> str:
        """The scenery flow's first end for the direction, or "".

        X-Plane's apt.dat runway-in-use rules are the airport's own "which
        runway when": where one passes it outranks a wind guess, which is
        all the fallback ever was. Nothing under MSFS -- its facility data
        carries no flows -- or where no flow applies.
        """
        try:
            layout = self._layout_for(airport.ident)
        except Exception:
            layout = None
        flow = flowmod.pick_flow(getattr(layout, "flows", None), weather)
        ends = flow.ends(departing) if flow is not None else []
        return normalize_runway(ends[0]) if ends else ""

    def _departure_runway(self, airport, weather) -> str:
        if airport is None:
            return "01"
        planned = self._planned(airport, weather, "departing")
        if planned:
            return planned
        flow = self._flow_runway(airport, weather, departing=True)
        if flow:
            return flow
        return self._wind_runway(airport, weather, departing=True)

    def _arrival_runway(self, airport, weather) -> str:
        if airport is None:
            return "01"
        planned = self._planned(airport, weather, "landing")
        if planned:
            return planned
        flow = self._flow_runway(airport, weather, departing=False)
        if flow:
            return flow
        return self._wind_runway(airport, weather, departing=False)

    def _wind_runway(self, airport, weather, *, departing: bool) -> str:
        """The runway the wind gives, kept until the wind takes it away.

        It was worked out afresh on every call, so a wind veering through
        the crossover flipped the runway on the next transmission and back
        on the one after, and nobody was ever told. A tower keeps the runway
        in use while the wind leaves it usable -- a tailwind of 5 kt or less,
        gusts included, which is the figure both ICAO PANS-ATM (Doc 4444,
        aerodrome control, runway in use) and the FAA's calm-wind runway use
        work to; UNVERIFIED: the paragraph numbers -- and changes it only
        when it does not (``NavDB.runway_within_limits``).
        """
        best = self.navdb.best_runway(
            airport.ident, weather.wind_dir, weather.wind_kt, departing=departing,
            variable=weather.variable, gust_kt=weather.gust_kt,
        )
        choice = best[0] if best else "01"
        if not best or not self.realism.runway_change:
            return choice
        key = (airport.ident, departing)
        current = self.runway_in_use.get(key)
        if current and current != choice:
            if self.navdb.runway_within_limits(
                    airport.ident, current, weather.wind_dir, weather.wind_kt,
                    gust_kt=weather.gust_kt, variable=weather.variable):
                return current
            decision("runway_change", airport=airport.ident,
                     departing=departing, was=current, now=choice,
                     wind_dir=round(float(weather.wind_dir or 0.0)),
                     wind_kt=round(float(weather.wind_kt or 0.0)),
                     gust_kt=round(float(weather.gust_kt or 0.0)) or None,
                     variable=bool(weather.variable),
                     weather=getattr(weather, "source", "") or None)
            # The landing direction is the one announced; a departure-only
            # change (a preferential system) is announced when it is the
            # only one.
            if not departing or airport.ident not in self.runway_changes \
                    or self.runway_changes[airport.ident][1] != choice:
                self.runway_changes[airport.ident] = (current, choice)
        self.runway_in_use[key] = choice
        return choice

    @classmethod
    def _departing(cls, session, state) -> bool:
        """Whether a flight is outbound from where it took off.

        The phase says so, or the aeroplane is still near the field it
        left and is not an arrival back at it.
        """
        if session.phase.outbound:
            return True
        if session.phase in (Phase.DESCENT, Phase.APPROACH, Phase.LANDING):
            return False
        origin = session.departure
        if origin is None or state is None or not getattr(state, "has_position", False):
            return False
        return state.distance_to(origin.lat, origin.lon) <= DEPARTING_WITHIN_NM

    @classmethod
    def _transition_altitude(cls, airport) -> int:
        """Where altitudes become flight levels.

        The table moved to :mod:`meridianatc.navdata.db`, beside the dialect and
        the registration prefix, because the controller was not the only thing
        that needed it. The other aeroplanes on the frequency say their own
        levels, and they were reading every one of them against eighteen
        thousand feet -- so a controller over London issued a flight level and
        the aircraft read it back in thousands of feet.
        """
        if airport is None:
            return DEFAULT_TRANSITION_FT
        return transition_altitude(airport.ident)

    @staticmethod
    def _default_cruise(session: FlightSession) -> float:
        """A plausible cruise level when the pilot never filed one."""
        if session.cruise_altitude_ft:
            return session.cruise_altitude_ft
        wake = session.aircraft.wake
        if wake in ("heavy", "super"):
            return 35000.0
        if session.profile.airliner:
            return 33000.0
        return 9000.0

    def _valid_level(self, feet: float, state, airport, upward: bool) -> float:
        """The nearest level an aeroplane can be cleared to, above or below.

        Whole thousands; and above the transition, the semicircular rule on
        the magnetic track (ICAO Annex 2 Appendix 3, 14 CFR 91.179): 000-179
        odd, 180-359 even, and above FL410 the RVSM steps of four thousand
        (odd 410, 450...; even 430, 470...). "Request climb" at 17,143 ft
        used to give flight level two one one.
        """
        transition = self._transition_altitude(airport)
        if feet <= transition:
            step = 1000.0
            base = 0.0
        else:
            from ...navdata.magvar import to_magnetic, variation
            from ..final import course_over_ground

            track = course_over_ground(state) if state is not None else None
            if track is not None and getattr(state, "has_position", False):
                track = to_magnetic(track, variation(state.latitude, state.longitude))
            east = track is None or track < 180.0
            if feet > 41000.0:
                step, base = 4000.0, (41000.0 if east else 43000.0)
            else:
                step, base = 2000.0, (1000.0 if east else 0.0)
        if upward:
            level = math.ceil((feet - base) / step) * step + base
        else:
            level = math.floor((feet - base) / step) * step + base
        return float(level)

    def _next_level_up(self, session, state) -> float:
        """The next level up, valid for the direction, never below where the
        aeroplane is, and the filed cruise level at most."""
        current = getattr(state, "altitude_ft", 0.0) or session.assigned_altitude_ft
        target = session.cruise_altitude_ft or self._default_cruise(session)
        step = 4000 if current < 18000 else 10000
        airport = self.navdb.airport(getattr(session.departure, "ident", "") or "") \
            if session.departure else None
        wanted = max(current + step, 3000.0)
        level = self._valid_level(wanted, state, airport, upward=True)
        # Inside the band near the top the filed level is the answer, valid
        # for the direction or not: it is what was filed and approved.
        if level >= target:
            return float(target)
        return level

    def _next_level_down(self, session, state) -> float:
        """The next level down, rounded to a usable altitude.

        Descents are issued in the steps a controller actually uses -- large
        ones in the flight levels, smaller ones below -- and never to a level
        the aircraft is already at or beneath.
        """
        current = getattr(state, "altitude_ft", 0.0) or session.assigned_altitude_ft
        if current <= 0:
            return 10000.0
        step = 10000 if current > 24000 else 4000
        target = current - step
        # A level, not a number: whole, and valid for the direction above
        # the transition.
        airport = session.destination or session.departure
        level = self._valid_level(min(target, current - 1000), state, airport,
                                  upward=False)
        return float(max(3000, level))

    # A step climb is refused for an aeroplane this close laterally that sits
    # between the level the flight is leaving and the one it asked for, or
    # less than the RVSM thousand feet above that one. UNVERIFIED as a
    # figure: real controllers judge it off the closure, and ten miles is
    # what two airliners closing head-on cover in under a minute.
    STEP_CLIMB_TRAFFIC_NM = 10.0

    def _climb_blocked_by(self, state, target_ft: float):
        """The contact a climb to ``target_ft`` would go through, or None."""
        if state is None or not getattr(state, "has_position", False):
            return None
        current = float(getattr(state, "altitude_ft", 0.0) or 0.0)
        nearest = None
        for contact in self.contacts():
            if contact.on_ground:
                continue
            if not current + 500.0 < contact.altitude_ft < target_ft + 1000.0:
                continue
            miles = state.distance_to(contact.latitude, contact.longitude)
            if miles > self.STEP_CLIMB_TRAFFIC_NM:
                continue
            if nearest is None or miles < nearest[0]:
                nearest = (miles, contact)
        return nearest[1] if nearest else None

    # A request for descent this long or more before it is due gets "expect
    # lower"; nearer, the descent itself (after LocalTC: 15 minutes).
    DESCENT_ASK_MIN = 15.0
    # The speed limit below 10,000 ft (14 CFR 91.117(a), SERA.6001).
    SPEED_LIMIT_KT = 250
    SPEED_LIMIT_BELOW_FT = 10000.0

    def _answer_speed(self, parsed, session, station, p, state) -> Reply:
        """A speed, or no restriction: unable below 10,000 ft above 250 kt,
        and unable while there is somebody to follow."""
        asked = int(parsed.get("speed_kt") or 0)
        altitude = float(getattr(state, "altitude_ft", 0.0) or 0.0)
        aircraft = session.aircraft
        if asked > self.SPEED_LIMIT_KT and altitude < self.SPEED_LIMIT_BELOW_FT:
            decision("request", asked="speed", answer="unable",
                     why="speed limit", knots=asked)
            return Reply(p.unable_because(aircraft, "speed_limit"), station)
        destination = session.destination
        if (session.speed_given and not session.speed_lifted
                and self.arrivals is not None and destination is not None
                and state is not None and getattr(state, "has_position", False)
                and self.arrivals._traffic_ahead(
                    state, destination,
                    state.distance_to(destination.lat, destination.lon))):
            decision("request", asked="speed", answer="unable",
                     why="traffic ahead")
            return Reply(p.unable_because(aircraft, "traffic_ahead"), station)
        session.speed_lifted = bool(session.speed_given)
        decision("request", asked="speed", answer="no restriction",
                 knots=asked or None)
        return Reply(p.no_speed_restrictions(aircraft), station)

    def _answer_hold(self, session, station, p, state) -> Reply:
        """The hold itself, over the arrival's last fix, wherever the
        flight is; unable without a fix to hold over."""
        session.hold_requested = True
        destination = session.destination
        reply = None
        if self.arrivals is not None and destination is not None \
                and state is not None and getattr(state, "has_position", False):
            reply = self.arrivals._give_hold(
                session, station, state, destination, "requested",
                self.arrivals._arrivals_queued(state, destination), p,
                time.time())
        if reply is None:
            session.hold_requested = False
            decision("request", asked="hold", answer="unable",
                     why="no holding fix")
            return Reply(p.unable_because(session.aircraft, "no_hold_fix"),
                         station)
        decision("request", asked="hold", answer="hold")
        return reply

    def _descent_asked_early(self, session, station, p, state) -> Reply | None:
        """"Request descent" before the descent is due: "expect lower in N
        minutes" well ahead of it, the descent itself nearer; None leaves
        the old answer (the descent is already under way, or no route)."""
        from ..arrival import FIRST_DESCENT_KEY

        destination = session.destination
        if (not self.realism.ifr_descent or self.arrivals is None
                or destination is None or FIRST_DESCENT_KEY in session.advisories
                or state is None or not getattr(state, "has_position", False)):
            return None
        speed = float(getattr(state, "ground_speed_kt", 0.0) or 0.0)
        if speed < 100.0:
            return None
        level = max(float(state.altitude_ft or 0.0),
                    float(session.assigned_altitude_ft or 0.0))
        to_run = state.distance_to(destination.lat, destination.lon)
        to_go, start, lead = self.arrivals._descent_point(
            destination, state, level, to_run, session)
        minutes = (to_go - start - lead) / speed * 60.0
        if minutes > self.DESCENT_ASK_MIN:
            decision("request", asked="descent", answer="expect lower",
                     minutes=round(minutes))
            text = p.expect_lower(session.aircraft, int(round(minutes)))
            return Reply(text, station, instruction=session.remember(
                Instruction("expect_lower", station_key(station), text, {},
                            readback_required=False)))
        decision("request", asked="descent", answer="the descent now",
                 minutes=round(minutes))
        return self.arrivals._first_descent(
            session, station, state, destination, to_run,
            getattr(self, "weather_for", None), p, force=True)

    def _airport_named(self, parsed):
        """An airport said by its ICAO code, written or in the alphabet."""
        from ..speech import _ALPHABET_REVERSE

        text = getattr(parsed, "text", "") or ""
        # Spelled first: "Lima India Romeo Alfa" is LIRA, and "Lima" on its
        # own is a word of the alphabet, not the code LIMA.
        words = re.findall(r"[a-z]+", text.lower())
        letters = [_ALPHABET_REVERSE.get(w, "") for w in words]
        for i in range(len(letters) - 3):
            group = letters[i:i + 4]
            if all(len(g) == 1 for g in group):
                found = self.navdb.airport("".join(group).upper())
                if found is not None:
                    return found
        # Then a code typed in capitals.
        for token in re.findall(r"\b[A-Z]{4}\b", text):
            found = self.navdb.airport(token)
            if found is not None:
                return found
        return None

    def _divert(self, parsed, session, station, p, state, back: bool) -> Reply:
        """Back to the field left, or on to the alternate (or the airport
        named): "cleared to (airport) via direct, maintain ...", and the
        arrival worked from here to there."""
        aircraft = session.aircraft
        if back:
            target = session.departure
        else:
            target = self._airport_named(parsed)
            alternate = getattr(self.route, "alternate", "") if self.route else ""
            if target is None and alternate:
                target = self.navdb.airport(alternate)
        if target is None:
            decision("request", asked="return" if back else "diversion",
                     answer="unable", why="no airport")
            return Reply(p.unable_because(aircraft, "no_alternate"), station)
        self._new_destination(session, target)
        altitude = float(session.assigned_altitude_ft
                         or round(float(getattr(state, "altitude_ft", 0.0) or 0.0), -2))
        session.assigned_altitude_ft = altitude
        decision("request", asked="return" if back else "diversion",
                 answer="cleared", to=target.ident, altitude_ft=int(altitude))
        text = p.cleared_to_airport(
            aircraft, target.spoken, altitude,
            transition_altitude=self._transition_altitude(target))
        instruction = session.remember(Instruction(
            "diversion", station_key(station), text,
            {"altitude_ft": int(altitude), "airport": target.spoken},
            required_items={"altitude_ft"}))
        return Reply(text, station, instruction=instruction)

    @staticmethod
    def _new_destination(session, airport) -> None:
        """Everything the arrival kept about the old destination, dropped."""
        session.destination = airport
        if not session.phase.on_ground:
            # An arrival from here on, not a climb-out: departure must not
            # hand it to the centre as one.
            session.phase = Phase.DESCENT
        session.arrival_runway = ""
        session.approach_choice, session.approach_key = None, ()
        session.vector_leg, session.vector_side, session.vector_at = "", 0.0, 0.0
        session.hold, session.hold_requested = None, False
        session.star_join, session.descend_via = "", ""
        session.speed_given, session.speed_lifted = "", False
        session.field_in_sight = False
        session.advisories = {key for key in session.advisories
                              if not key.startswith(("arrival:", "hold:"))
                              and key not in ("handoff:approach",
                                              "handoff:tower")}

    def _approve_deviation(self, parsed, session, station, p, state) -> Reply:
        """"Request deviation twenty degrees left for weather": approved,
        with the first route fix forty miles on to rejoin at (after
        LocalTC). The weather is the pilot's to see, so it is not refused;
        the off-route watch stands down until the flight is back on its
        route (atc/routing.py)."""
        from ..routing import rejoin_fix

        text_heard = parsed.normalized or ""
        side = _side_said(text_heard)
        degrees = _degrees_said(text_heard, getattr(parsed, "language", "en"))
        fix = None
        if state is not None and getattr(state, "has_position", False):
            fix = rejoin_fix(self.route, self.route_progress, state)
        session.deviating = "approved"
        session.establish(station)
        decision("route", event="deviation", station=station.callsign,
                 side=side or None, degrees=degrees or None,
                 rejoin=getattr(fix, "ident", None))
        text = p.deviation_approved(session.aircraft, degrees, side,
                                    getattr(fix, "ident", ""))
        values = {"fix": fix.ident} if fix is not None else {}
        instruction = session.remember(Instruction(
            "deviation", station_key(station), text, values,
            readback_required=False))
        return Reply(text, station, instruction=instruction)

    def _direct_on_the_plan(self, parsed, session, station, p, state) -> Reply:
        """A direct asked of a centre, with a flight plan loaded.

        Granted to a fix still ahead on the route; anything else is unable.
        Without the plan the controller matched the word against every
        navaid within 250 miles and cleared the flight to whichever sounded
        nearest, which over Vietnam was a beacon in China.
        """
        from ..final import course_over_ground

        aircraft = session.aircraft
        heard = (parsed.get("fix", "") or "").upper()
        best, score = None, 0.0
        for fix in self.plan_fixes:
            alike = max(_similar(heard, fix.ident),
                        _similar(heard, (fix.name or "").upper()))
            if alike > score:
                best, score = fix, alike
        if not heard or best is None or score < 0.6:
            decision("enroute_direct", station=station.callsign, fix=heard,
                     event="direct refused", granted=False,
                     reason=f"{heard} is not on the flight plan")
            if not heard:
                return Reply(p.unable_unknown_fix(aircraft), station)
            return Reply(p.unable_direct(aircraft, heard), station)
        track = course_over_ground(state) if state is not None else None
        if track is None:
            track = float(getattr(state, "heading_true", 0.0) or 0.0)
        bearing = bearing_deg(state.latitude, state.longitude, best.lat, best.lon)
        off = abs(((bearing - track) + 540.0) % 360.0 - 180.0)
        if off > 90.0:
            decision("enroute_direct", station=station.callsign,
                     fix=best.ident, event="direct refused", granted=False,
                     reason=f"{best.ident} is already behind the flight")
            return Reply(p.unable_direct(aircraft, best.ident), station)
        decision("enroute_direct", station=station.callsign, fix=best.ident,
                 event="direct granted", granted=True,
                 reason=f"{best.ident} is ahead on the flight plan")
        spoken = best.name if best.name and best.name.upper() != best.ident else ""
        text = p.direct_to(aircraft, best.ident, fix_spoken=spoken)
        instruction = session.remember(
            Instruction("direct", station_key(station), text, {},
                        readback_required=False))
        return Reply(text, station, instruction=instruction)

    def _resolve_fix(self, heard: str, state) -> tuple[str, str]:
        """Match a mis-heard fix name against real navaids nearby.

        Recognition regularly turns CAMRN into "cammen". Rather than refusing,
        the heard word is matched against navaids in the area, which is what a
        controller does when a readback is close but not exact.
        """
        heard = (heard or "").upper()
        if not heard or state is None or not getattr(state, "has_position", False):
            return heard, ""

        candidates = self.navdb.nearest_navaids(
            state.latitude, state.longitude, limit=40, max_nm=250.0
        )
        best_ident, best_name, best_score = heard, "", 0.0
        for navaid, _d in candidates:
            ident = (navaid.get("ident") or "").upper()
            name = (navaid.get("name") or "")
            score = max(_similar(heard, ident), _similar(heard, name.upper()))
            if score > best_score:
                best_ident, best_name, best_score = ident, name, score
        if best_score >= 0.6:
            return best_ident, best_name
        return heard, ""

    @staticmethod
    def _bearing_to(state, airport) -> tuple[int, float]:
        """Where a field is from the aircraft, as a clock position and a range.

        Zero and zero when the position or the heading is unknown, which reads
        as "no position given" and leaves the phrase without the clause.
        """
        if state is None or airport is None or not getattr(state, "has_position", False):
            return 0, 0.0
        # An explicit None: a heading of north is 0, and "or" took it for no
        # heading and read the magnetic one instead.
        heading = getattr(state, "heading_true", None)
        if heading is None:
            heading = getattr(state, "heading_magnetic", None)
        distance = state.distance_to(airport.lat, airport.lon)
        if heading is None:
            return 0, distance
        import math

        lat1 = math.radians(state.latitude)
        lat2 = math.radians(airport.lat)
        dlon = math.radians(airport.lon - state.longitude)
        y = math.sin(dlon) * math.cos(lat2)
        x = (math.cos(lat1) * math.sin(lat2)
             - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        bearing = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
        relative = (bearing - float(heading) + 360.0) % 360.0
        oclock = int(round(relative / 30.0)) or 12
        return (oclock if oclock <= 12 else oclock - 12), distance

    @staticmethod
    def _distance_to(state, airport) -> float:
        """How far the aircraft is from a field, in nautical miles.

        Zero when either is unknown, which reads as "at the field" and keeps
        the no-simulator case working the way it always has.
        """
        if state is None or airport is None or not getattr(state, "has_position", False):
            return 0.0
        return state.distance_to(airport.lat, airport.lon)

    def _nearest_airport(self, state):
        if state is None or not getattr(state, "has_position", False):
            return None
        return self.navdb.nearest_airport(
            state.latitude, state.longitude, max_nm=60.0, min_rank=1
        )

    def _destination_tower(self, session, station) -> Station | None:
        target = session.destination
        if target is None:
            return None
        return self.controller_at(target.ident, "TWR",
                                  runway=session.arrival_runway)


# How far from the field it left a flight can still be a departure whatever
# the phase tracker says: the tracker calls it en route at ten thousand feet.
