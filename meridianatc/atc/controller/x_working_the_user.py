from __future__ import annotations

from ._shared import (
    Instruction,
    Phase,
    Reply,
    decision,
    final_for,
    language_of,
    logging,
    normalize_runway,
    runway_length_nm,
    say_words,
    sid_first_fix,
    station_key,
    time,
)

log = logging.getLogger(__name__)


class _Working_the_user:
    # ------------------------------------------------------------------
    # working the user into the traffic
    # ------------------------------------------------------------------

    # Set by the engine when something is reporting the ground handling. A
    # controller that can see the tug does not clear the aeroplane to taxi
    # while it is still attached to it.
    ground = None

    def _stuck_on_the_stand(self) -> str:
        """Why the aeroplane cannot move, or an empty string if it can."""
        if self.ground is None:
            return ""
        try:
            state = self.ground.state
            return "" if state.can_taxi else state.why_not()
        except Exception:
            log.warning("could not read the ground state; treating the "
                        "stand as clear", exc_info=True)
            return ""

    # Set by the engine when the traffic injector is running. Without it every
    # method below falls back to the way it worked when the frequency was
    # empty, which is what the tests and a traffic-free session get.
    traffic = None

    def _traffic_ahead(self, arriving: bool):
        """The AI flights in front of the user, nearest first."""
        if self.traffic is None:
            return []
        try:
            return self.traffic.traffic_ahead(arriving)
        except Exception:
            log.warning("could not read the traffic picture; treating "
                        "the approach lane as empty", exc_info=True)
            return []

    # How close an arrival has to be before nobody else may be cleared to
    # go. Four miles is about ninety seconds, which is not enough to line up,
    # roll and be airborne with anything to spare.
    RUNWAY_CONFLICT_NM = 4.0

    # A departure with an arrival between RUNWAY_CONFLICT_NM and this on the
    # final goes now or not at all: "cleared for immediate takeoff".
    IMMEDIATE_NM = 6.0
    # An arrival still told to continue, or cleared behind a departure that
    # has not gone, is sent round inside this of the threshold.
    GO_AROUND_NM = 0.6

    def landing_on_top(self, airport=None, runway: str = ""):
        """The arrival too close for anybody else to be sent down the runway:
        the invented traffic's, or -- given the field and the runway --
        anything in the picture on that final (atc.occupancy)."""
        mine = next((f for f in self._traffic_ahead(True)
                     if f.phase == "final"
                     and f.distance_nm <= self.RUNWAY_CONFLICT_NM), None)
        if mine is not None or airport is None or not runway \
                or not self.realism.conditional_clearances:
            return mine
        seen = self.arrivals_on_final(airport, runway, self.RUNWAY_CONFLICT_NM)
        return seen[0] if seen else None

    def runway_available(self, airport=None, runway: str = "") -> bool:
        """Whether the runway is free enough to clear somebody for takeoff.

        Public because the tower has to answer this question twice: once when
        the pilot asks, and again on its own, for an aeroplane it has already
        put on the runway and told to wait. See
        :class:`~meridianatc.atc.departure.DepartureWatch`.
        """
        return (not self._runway_blocked(airport, runway)
                and self.landing_on_top(airport, runway) is None)

    def _runway_blocked(self, airport=None, runway: str = "") -> bool:
        """Whether somebody else is on the runway right now: the invented
        traffic's reservation, or anything in the picture on the strip."""
        if self.traffic is not None:
            try:
                if self.traffic.runway_is_busy(time.monotonic()):
                    return True
            except Exception:
                log.warning("could not read the runway reservation; "
                            "falling back to the occupancy picture",
                            exc_info=True)
        return bool(self.runway_occupants(airport, runway))

    def arrivals_ahead(self, airport, runway: str, state) -> list:
        """Arrivals on the same final nearer its threshold than this
        flight: the landing clearance waits for them (Phase 5 (e))."""
        if not self.realism.tower_continue or state is None \
                or not getattr(state, "has_position", False):
            return []
        final = final_for(self.navdb, getattr(airport, "ident", ""), runway)
        if final is None:
            return []
        mine = final.distance(state)
        return [one for one in self.arrivals_on_final(
                    airport, runway, self.LANDING_CLEARANCE_NM + 5.0)
                if one.distance_nm < mine - 0.3]

    def runway_occupants(self, airport, runway: str) -> list:
        """Everybody on ``runway``'s strip (atc.occupancy.on_the_runway)."""
        if airport is None or not runway or not self.realism.conditional_clearances:
            return []
        from ..occupancy import on_the_runway

        return on_the_runway(self.navdb, airport, runway, self.contacts())

    def arrivals_on_final(self, airport, runway: str, within_nm: float) -> list:
        """The arrivals on ``runway``'s final inside ``within_nm``."""
        if airport is None or not runway:
            return []
        from ..occupancy import on_final

        return on_final(self.navdb, airport, runway, self.contacts(), within_nm)

    def landing_or_withheld(self, session, station, weather, p, airport,
                            runway: str, *, station_name: str = "",
                            sequence: str = "", caution: str = "",
                            state=None) -> Reply | None:
        """What the tower says instead of a plain landing clearance while
        somebody is on the runway, or None when it is clear.

        In daylight in the UK and the ICAO and German regions, "land after"
        the aeroplane on it (CAP 413). In the US, a departure still on the
        runway gets the anticipated-separation clearance. Otherwise, and for
        circuits, "continue approach": the clearance comes when the runway is
        free, and a go-around if it is not free in time
        (``ArrivalWatch``). Every one of them is logged with what was on the
        runway and where.
        """
        if not self.realism.conditional_clearances:
            return None
        from ..svfr import is_night
        from ..traffic import runway_traffic

        occupants = self.runway_occupants(airport, runway)
        if not occupants:
            session.landing_withheld = False
            return None
        first = occupants[0]
        language = language_of(p)
        aircraft = session.aircraft
        abbreviated = bool(session.established_with(station))
        region = getattr(p, "region", "icao")
        final = final_for(self.navdb, airport.ident, runway)
        departing = bool(final is not None and abs(((first.contact.heading
                         - final.course) + 540.0) % 360.0 - 180.0) <= 30.0
                         and first.along_nm < 0.5 * max(0.3, runway_length_nm(
                             self.navdb, airport.ident, runway)))
        log_fields = dict(airport=airport.ident, runway=runway,
                          occupant=first.written or None,
                          occupant_type=first.contact.type_code or None,
                          along_nm=round(first.along_nm, 2), moving=first.moving,
                          observed=first.contact.observed,
                          threshold_nm=(round(final.distance(state), 2)
                                        if final is not None and state is not None
                                        and getattr(state, "has_position", False)
                                        else None))
        if (region in ("uk", "icao", "de") and not session.pattern_work
                and not is_night(airport.lat, airport.lon, region)):
            text = p.land_after(aircraft, runway, weather,
                                traffic=first.describe(language),
                                station=station_name, abbreviated=abbreviated,
                                caution=caution)
            decision("landing_clearance", said="land_after", **log_fields)
            return self._landing_given(session, station, runway, text,
                                       weather)
        if region == "faa" and departing and not session.pattern_work:
            text = p.cleared_to_land(
                aircraft, runway, weather, station=station_name,
                sequence=sequence, abbreviated=abbreviated,
                caution=", ".join(c for c in (caution, p.departing_prior()) if c))
            session.landing_anticipated = first.written
            decision("landing_clearance", said="anticipated", **log_fields)
            return self._landing_given(session, station, runway, text,
                                       weather)
        session.landing_withheld = True
        session.phase = Phase.LANDING
        text = p.continue_approach(aircraft, runway_traffic(language))
        decision("landing_clearance", said="continue", **log_fields)
        instruction = session.remember(
            Instruction("continue_approach", station_key(station), text, {},
                        readback_required=False))
        return Reply(text, station, instruction=instruction)

    def _landing_given(self, session, station, runway, text,
                       weather=None) -> Reply:
        session.cleared_landing = True
        session.landing_withheld = False
        session.phase = Phase.LANDING
        session.allow_runway(runway)
        session.note_clearance_wind(weather)
        instruction = session.remember(
            Instruction("landing_clearance", station_key(station), text,
                        {"runway": runway}))
        return Reply(text, station, instruction=instruction)

    def order_go_around(self, session, station, weather, p, airport,
                        why: str) -> Reply:
        """The tower sending an arrival round because the runway is not
        clear: "go around, traffic on the runway", then what to fly (the
        circuit again, or the missed approach)."""
        from ..traffic import runway_traffic

        occupants = self.runway_occupants(airport, session.arrival_runway)
        decision("go_around_ordered", airport=getattr(airport, "ident", ""),
                 runway=session.arrival_runway, why=why,
                 occupant=occupants[0].written if occupants else None)
        session.landing_withheld = False
        session.landing_anticipated = ""
        then = self.go_around(session, station, weather, p, airport, told=True)
        order = p.go_around(session.aircraft,
                            reason=runway_traffic(language_of(p)))
        then.text = f"{order} {then.text}"
        return then

    def _issue_takeoff(self, session, station, weather, p, parsed,
                       state=None) -> Reply:
        airport = self.navdb.airport(station.ident)
        asked = parsed.get("runway")
        if asked and airport is not None:
            ends = {name for name, _hdg, _rw
                    in self.navdb.runway_ends(airport.ident)}
            # A runway the field does not have is not granted: the ask is
            # ignored and the wind's runway is used instead -- clearing
            # somebody for takeoff on pavement that does not exist is how
            # the request for a 36 at a field with one strip reads as
            # approved.
            if ends and str(asked).upper() not in ends:
                asked = None
        runway = (
            session.departure_runway
            or asked
            or self._departure_runway(airport, weather)
        )
        session.departure_runway = runway

        # A clearance that is still standing is not reissued. Asking twice is
        # usually a pilot who did not hear it, so the runway is repeated, but
        # a second "cleared for takeoff" for a clearance they already hold
        # reads as a new one and is exactly how a runway gets two aircraft.
        if session.cleared_takeoff:
            return Reply(
                p.already_cleared_for_takeoff(session.aircraft, runway), station
            )

        # A takeoff clearance is issued to an aeroplane at the runway, not
        # to one still taxiing to it -- "cleared for takeoff" heard on the
        # apron is rolled onto wherever it happened to be pointing. At it
        # reaches past the pavement runway_at reads: the bars a departure
        # holds at are outside that rectangle, as are a CAT III point and
        # the connector beside it, so the ask is whether the aeroplane is
        # beside this runway, at its end of it -- not on it. Where position
        # is not reported at all this cannot be told, so the ask is trusted
        # the way it always was. `lined_up` and the engine's own holding
        # point skip the check: the aeroplane's place is already known.
        if (state is not None and getattr(state, "has_position", False)
                and airport is not None and not session.lined_up
                and session.phase is not Phase.HOLDING_SHORT):
            standing = self.navdb.runway_at(
                airport.ident, state.latitude, state.longitude,
                heading=getattr(state, "heading_true", None))
            if (normalize_runway(standing or "") != normalize_runway(runway)
                    and not self.navdb.at_the_runway(
                        airport.ident, runway,
                        state.latitude, state.longitude)):
                instruction = session.remember(
                    Instruction("taxi", station_key(station), "",
                                {"runway": runway}))
                return Reply(
                    p.taxi_to_runway(
                        session.aircraft, self._station_name(station, p),
                        runway, (), abbreviated=True),
                    station, instruction=instruction)

        # Below VFR minima nothing VFR takes off without Special VFR, and
        # circuits are not flown in it at all.
        refused = self._below_vmc(session, station, weather, p, airport,
                                  asked="takeoff",
                                  offer=not session.pattern_work)
        if refused is not None:
            return refused

        # The runway is one aeroplane wide. If somebody else is rolling on it,
        # or is about to land on it, the user is lined up and held rather than
        # cleared -- which is the whole of what a tower does with a departure
        # queue, and it is why a takeoff clearance sometimes takes a minute.
        landing = self.landing_on_top(airport, runway)
        blocked = self._runway_blocked(airport, runway)
        if blocked or landing is not None:
            if session.lined_up or session.held_for_landing:
                # Already on the runway and still waiting. Asking again does
                # not make the aeroplane in front of you land any sooner.
                session.tell_to_wait(hold=True)
                return Reply(p.hold_position(session.aircraft), station)
            if not blocked and self.realism.conditional_clearances:
                return self._held_for_landing(session, station, p, runway,
                                              landing)
            reason = ""
            if landing is not None:
                from ..traffic import landing_traffic

                reason = landing_traffic(landing.describe(language_of(p)),
                                         language_of(p))
            # Lining up off an intersection, the taxiway is named too --
            # 7110.65 3-7-1 wants it in both the line-up and the takeoff
            # clearance, because a pilot who rolls from the wrong entry
            # is the incident the rule is written against.
            intersection = (say_words(
                station.ident, [session.departure_intersection])[0]
                if session.departure_intersection else "")
            text = p.line_up_and_wait(
                session.aircraft, runway, reason=reason,
                station=self._station_name(station, p)
                if not session.established_with(station) else "",
                abbreviated=session.established_with(station),
                intersection=intersection,
            )
            session.lined_up = True
            session.allow_runway(runway)
            instruction = session.remember(
                Instruction("line_up_and_wait", station_key(station), text,
                            {"runway": runway,
                             "intersection": intersection})
            )
            return Reply(text, station, instruction=instruction)

        station_name = (
            self._station_name(station, p)
            if not session.established_with(station) else ""
        )
        if session.pattern_work:
            # Staying in the circuit, and coming back to the runway it left
            # from. The circuit's landings were taken from the wind, so
            # circuits asked for on two eight were flown back to three five.
            session.arrival_runway = runway
            # Staying in the circuit, so the clearance says which way round it
            # goes instead of sending the aircraft off on runway heading.
            text = p.closed_traffic_approved(
                session.aircraft, runway,
                self._pattern_direction(session, parsed, p, airport, runway),
                weather, station=station_name,
                abbreviated=session.established_with(station),
            )
        else:
            # An arrival on the final with room for one departure in front of
            # it, and no more: the departure goes now (ICAO Doc 4444 chapter
            # 12, "cleared for immediate take-off"), told why.
            soon = (self.arrivals_on_final(airport, runway, self.IMMEDIATE_NM)
                    if self.realism.conditional_clearances else [])
            traffic = ""
            if soon:
                described = (soon[0].indefinite(language_of(p))
                             or soon[0].describe(language_of(p)))
                traffic = p.traffic_on_final(described, soon[0].distance_nm)
                decision("takeoff", airport=getattr(airport, "ident", ""),
                         runway=runway, said="immediate",
                         arrival=soon[0].written or None,
                         arrival_nm=round(soon[0].distance_nm, 1),
                         observed=soon[0].contact.observed)
            intersection = (say_words(
                station.ident, [session.departure_intersection])[0]
                if session.departure_intersection else "")
            text = p.cleared_for_takeoff(
                session.aircraft, runway, weather,
                caution=self._caution(session, weather, p,
                                      self._departing_airliner(airport),
                                      "departing"),
                station=station_name,
                after_departure=self._after_takeoff(session, airport, runway, p),
                abbreviated=session.established_with(station),
                traffic=traffic, immediate=bool(soon),
                intersection=intersection,
                remaining_m=session.departure_remaining_m,
            )
        session.held_for_landing = ""
        session.cleared_takeoff = True
        session.cleared_takeoff_at = time.monotonic()
        session.phase = Phase.TAKEOFF
        session.allow_runway(runway)
        session.note_clearance_wind(weather)
        instruction = session.remember(
            Instruction("takeoff_clearance", station_key(station), text,
                        {"runway": runway,
                         "intersection": (say_words(
                             station.ident,
                             [session.departure_intersection])[0]
                             if session.departure_intersection else "")})
        )
        return Reply(text, station, instruction=instruction)

    def _after_takeoff(self, session, airport, runway: str, p) -> str:
        """What the takeoff clearance says the flight does next.

        A SID cleared is the route, and an ICAO tower says nothing more. A
        US one names an RNAV SID's first fix ("RNAV to MERIT", 7110.65
        4-3-2) when it is a path from the runway, within 15 nm. No SID is
        runway heading, which departure turns on course with a direct.
        """
        if not session.ifr:
            return ""
        if not self.realism.ifr_departure:
            return p.RUNWAY_HEADING
        session.runway_heading = False
        if session.sid:
            first = None
            if airport is not None:
                first = sid_first_fix(self.route, airport.lat, airport.lon)
            said = p.rnav_to(first.ident) if first is not None else ""
            decision("takeoff", airport=getattr(airport, "ident", ""),
                     runway=runway, said="rnav_to" if said else "sid",
                     sid=session.sid,
                     first_fix=first.ident if first is not None else None)
            return said
        session.runway_heading = True
        decision("takeoff", airport=getattr(airport, "ident", ""),
                 runway=runway, said="runway_heading",
                 why="no SID cleared")
        return p.RUNWAY_HEADING

    def _held_for_landing(self, session, station, p, runway, landing) -> Reply:
        """A departure with an arrival too close in front of it.

        Outside the US, lined up behind it: "behind the landing Cessna, line
        up runway two four, behind" (ICAO Doc 4444 chapter 12, conditional
        clearances) -- not "line up and wait" in front of it, which put the
        departure on the runway with an aeroplane landing on it. In the US,
        which gives no conditional clearances, held short. Either way the
        takeoff clearance is the tower's to give once it has landed
        (departure.DepartureWatch).
        """
        language = language_of(p)
        aircraft = session.aircraft
        abbreviated = session.established_with(station)
        name = self._station_name(station, p) if not abbreviated else ""
        traffic = landing.describe(language)
        miles = float(getattr(landing, "distance_nm", 0.0) or 0.0)
        session.held_for_landing = getattr(landing, "written", "") or "traffic"
        if getattr(p, "region", "") == "faa":
            from ..traffic import landing_traffic

            held = p.hold_short_instruction(aircraft, runway)
            text = f"{held[:-1]}, {landing_traffic(traffic, language)}."
            kind, values = "hold_short", {"hold_short": runway}
            said = "hold_short"
        else:
            text = p.line_up_behind(aircraft, runway,
                                    p.landing_traffic_named(traffic),
                                    station=name, abbreviated=abbreviated)
            session.lined_up = True
            session.allow_runway(runway)
            kind, values = "line_up_and_wait", {"runway": runway}
            said = "line_up_behind"
        decision("takeoff", airport=station.ident, runway=runway, said=said,
                 arrival=session.held_for_landing, arrival_nm=round(miles, 1))
        instruction = session.remember(
            Instruction(kind, station_key(station), text, values))
        return Reply(text, station, instruction=instruction)

