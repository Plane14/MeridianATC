from __future__ import annotations

from ._shared import (
    Any,
    Instruction,
    Phase,
    Reply,
    agrees,
    bearing_deg,
    choose_join,
    circuit_for,
    decision,
    final_for,
    haversine_nm,
    language_of,
    log,
    math,
    sp,
    station_key,
)


class _The_circuit:
    # ------------------------------------------------------------------
    # the circuit
    # ------------------------------------------------------------------
    #
    # A tower without radar keeps VFR aircraft apart with reporting points and
    # a sequence, not with vectors and an approach clearance. Everything below
    # is that: where to join, what to report next, who to follow, and the
    # clearance at the end of it.

    # Further out than this an aircraft is joining the circuit; inside it, it
    # is already in one and is sequenced rather than given a join.
    PATTERN_JOIN_NM = 6.0
    # Without a leg reported, the aeroplane is on final -- and is cleared to
    # land -- only inside this of the threshold, in the final-approach cone
    # (:mod:`meridianatc.atc.final`), below PATTERN_FINAL_BELOW_AGL_FT and
    # descending. Three miles on its own cleared an aeroplane on the
    # downwind, one at four thousand feet over the field and one climbing
    # away. 1,500 ft is above any circuit height in use (1,000 ft is the
    # usual light-aircraft figure in the FAA AIM 4-3-3 and the UK and French
    # AIPs) with room for a pilot high on a long final.
    PATTERN_FINAL_NM = 3.0
    PATTERN_FINAL_BELOW_AGL_FT = 1500.0

    def _on_final(self, state, airport, runway: str) -> bool:
        """Whether the aeroplane is on final for ``runway``, by where it is."""
        if state is None or airport is None or not getattr(state, "has_position", False):
            return False
        if getattr(state, "on_ground", False) or not state.descending:
            return False
        elevation = float(getattr(airport, "elev_ft", 0.0) or 0.0)
        above = (getattr(state, "altitude_agl_ft", 0.0)
                 or float(state.altitude_ft or 0.0) - elevation)
        if above > self.PATTERN_FINAL_BELOW_AGL_FT:
            return False
        final = final_for(self.navdb, airport.ident, runway)
        if final is None:
            # No threshold to measure from: the old radius, still with the
            # height and the descent.
            distance = self._distance_to(state, airport)
            return bool(distance) and distance <= self.PATTERN_FINAL_NM
        return final.in_cone(state, self.PATTERN_FINAL_NM)

    # Whether the transmission being answered has written its circuit row,
    # so that :meth:`respond` writes one only for those that have not.
    _circuit_logged = False

    # How far round the circuit an aeroplane has to be behind the one it
    # follows: about a minute at a light aircraft's circuit speed. Closer
    # than that on the downwind, it is extended and its base called later.
    CIRCUIT_SPACING_NM = 1.5
    # On base or final, where there is nothing left to extend: closer than
    # this behind the one ahead, about thirty seconds, it is given an orbit.
    # UNVERIFIED: the figure.
    ORBIT_SPACING_NM = 0.8
    # As many aeroplanes in the circuit as this and an arrival is held at a
    # reporting point until one has landed. UNVERIFIED: the figure; ATZ
    # circuits are commonly run with four to six.
    CIRCUIT_FULL = 4

    def _circuit_at(self, session, airport, runway: str):
        """The circuit to ``runway``, flown the way this session flies it."""
        if airport is None or not runway:
            return None
        if not session.pattern_direction_said:
            published = self.navdb.circuit_data(airport.ident, runway)
            if published is not None and published.side:
                session.pattern_direction = published.side
        height, _source = self.circuit_height(airport, runway)
        return circuit_for(self.navdb, airport.ident, runway,
                           side=session.pattern_direction, height_agl_ft=height)

    @staticmethod
    def _where_in(circuit, state):
        """Where the user is in ``circuit``, or None if that cannot be seen."""
        if circuit is None or state is None:
            return None
        if not getattr(state, "has_position", False) or getattr(state, "on_ground", False):
            return None
        return circuit.locate(state)

    # Set by the engine: ``() -> list[Seen]``, every aeroplane the simulator
    # reports (FSLTL, its own traffic, other AI), the pilot's own excluded.
    sim_traffic = None

    def contacts(self) -> list:
        """Every aeroplane in the controller's picture
        (:class:`~meridianatc.atc.trafficinfo.Contact`): the invented traffic
        and everything the simulator reports, each once.

        The sequencing and the traffic information read this. They read the
        invented traffic alone, so a pilot flying with FSLTL -- or with the
        invented traffic switched off -- had a controller blind to every
        aeroplane out of the window.
        """
        from ..trafficinfo import from_flight, from_seen

        found, ours = [], set()
        if self.traffic is not None:
            for flight in list(getattr(self.traffic, "flights", []) or []):
                if getattr(flight, "done", False):
                    continue
                if getattr(flight, "stuck", False):
                    # Frozen and about to be taken away (TrafficInjector
                    # ._watchdog): nothing may wait on it moving -- no "land
                    # after", no "line up behind" -- and the simulator's copy
                    # of it, read below, is left out with it.
                    ours.add(flight.written)
                    continue
                if (getattr(flight, "observed", False)
                        and getattr(flight, "source", "") != "live"
                        and self.sim_traffic is not None):
                    continue        # read below, from the simulator itself
                # A live one (atc/live.py) is drawn by this program, so the
                # simulator never reports it as somebody else's: it is taken
                # from its strip, and its callsign keeps the simulator's copy
                # of it out below.
                contact = from_flight(flight)
                found.append(contact)
                ours.add(contact.written)
        if self.sim_traffic is not None:
            try:
                seen = self.sim_traffic() or []
            except Exception:
                if not getattr(self, "_sim_traffic_warned", False):
                    self._sim_traffic_warned = True
                    log.warning("could not read the simulator's traffic; "
                                "the frequency looks empty", exc_info=True)
                seen = []
            for one in seen:
                if not getattr(one, "positioned", True):
                    continue
                contact = from_seen(one)
                if contact.written and contact.written not in ours:
                    found.append(contact)
                    ours.add(contact.written)
        return found

    def circuit_traffic(self, circuit) -> list:
        """The traffic in ``circuit``, as ``(position, contact)``, the one
        with the least still to fly first -- invented or the simulator's,
        placed by where it is rather than by what it was told."""
        if circuit is None:
            return []
        placed = []
        for contact in self.contacts():
            if contact.on_ground:
                continue
            here = circuit.locate_at(contact.latitude, contact.longitude,
                                     contact.altitude_ft, contact.heading,
                                     contact.vertical_rate_fpm)
            if here.in_circuit:
                placed.append((here, contact))
        placed.sort(key=lambda pair: pair[0].to_go_nm)
        return placed

    def _caution(self, session, weather, p, ahead_type: str = "",
                 role: str = "") -> str:
        """Wake turbulence behind an airliner, a wet runway in rain.

        ``role`` is what the aeroplane making the wake is doing -- the
        clearance says who it is ("caution wake turbulence, heavy Boeing
        departing", JO 7110.65 2-1-20)."""
        from ..trafficinfo import caution, needs_wake_caution, runway_wet, wake_traffic

        wake = needs_wake_caution(ahead_type, session.aircraft.type_code,
                                  light=not session.profile.airliner)
        wet = runway_wet(weather)
        if wake or wet:
            decision("caution", wake=wake, wet=wet, ahead=ahead_type or None)
        traffic = (wake_traffic(ahead_type, role, language_of(p))
                   if wake else "")
        return caution(p, wet=wet, wake=wake, wake_traffic=traffic)

    def _departing_airliner(self, airport) -> str:
        """An airliner just off this field, climbing out: what a departure
        behind it is cautioned about."""

        if airport is None:
            return ""
        for contact in self.contacts():
            if contact.on_ground or contact.vertical_rate_fpm < 300.0:
                continue
            if haversine_nm(airport.lat, airport.lon, contact.latitude,
                            contact.longitude) <= 3.0 and sp.is_airliner(contact.type_code):
                return contact.type_code
        return ""

    def circuit_sequence(self, circuit, here, p) -> tuple[str, Any, Any]:
        """``("number two, follow the Cessna on base", position, flight)``
        for the aeroplane immediately ahead, or ``("", None, None)``."""
        if here is None:
            return "", None, None
        ahead = [pair for pair in self.circuit_traffic(circuit)
                 if pair[0].to_go_nm < here.to_go_nm]
        if not ahead:
            return "", None, None
        there, flight = ahead[-1]
        who = p.traffic_on_leg(flight.describe(language_of(p)), p.leg(there.leg))
        return p.sequence_number(len(ahead) + 1, who), there, flight

    def note_circuit(self, session, station, state, said: str) -> None:
        """The arrival-log row for a tower transmission that is not one of
        the circuit's own -- a takeoff clearance, a landing clearance seen
        rather than asked for, a rejoin after a go-around -- so that every
        one the tower makes to a VFR flight carries the leg it saw."""
        if station is None or station.position != "TWR" or not session.visual:
            return
        if state is None or getattr(state, "on_ground", True):
            return
        airport = self.navdb.airport(station.ident)
        here = self._where_in(self._circuit_at(
            session, airport, session.arrival_runway), state)
        self.log_circuit(session, station, here, said=said)

    def log_circuit(self, session, station, here, **fields) -> None:
        """One row of the arrival log for a tower transmission to a VFR
        flight: where the tower saw it, and what it made of that."""
        self._circuit_logged = True
        decision("circuit", airport=station.ident,
                 runway=session.arrival_runway,
                 leg=here.describe() if here is not None else "unseen",
                 to_go_nm=round(here.to_go_nm, 2) if here is not None else None,
                 along_nm=round(here.along_nm, 2) if here is not None else None,
                 out_nm=round(here.out_nm, 2) if here is not None else None,
                 height_agl_ft=(round(here.height_agl_ft)
                                if here is not None else None),
                 inside=here.inside if here is not None else None,
                 **fields)

    def _work_the_circuit(self, session, station, weather, p, parsed,
                          state=None) -> Reply:
        """Answer a VFR aircraft in, or joining, the traffic pattern.

        What the pilot says is checked against where the aeroplane is
        (:mod:`meridianatc.atc.circuit`) rather than believed: the join is
        chosen from the position, a reported leg two legs away from the one
        the tower can see is queried, and the sequence is the traffic
        actually in the circuit.
        """
        aircraft = session.aircraft
        airport = self.navdb.airport(station.ident)
        runway = (
            session.arrival_runway
            or parsed.get("runway")
            or self._arrival_runway(airport, weather)
        )
        session.arrival_runway = runway
        session.allow_runway("")
        # Joining, so no longer only passing through: the arrival watch
        # leaves a transit alone, and this one has asked to land.
        session.transition_approved = False

        direction = self._pattern_direction(session, parsed, p, airport, runway)
        reported = (parsed.get("pattern_leg") or "").lower()
        first_call = not session.established_with(station)
        station_name = self._station_name(station, p) if first_call else ""
        circuit = self._circuit_at(session, airport, runway)
        here = self._where_in(circuit, state)
        note = {"reported": reported or None}

        # Joining from outside into weather below VFR minima: kept out of
        # the zone, and told Special VFR can be asked for.
        if not reported and not session.cleared_landing and (
                here is None or not here.in_circuit):
            refused = self._below_vmc(session, station, weather, p, airport,
                                      asked="join", outside=True)
            if refused is not None:
                return refused

        # --- a leg the tower can see is not where the aeroplane is ---------
        if reported and here is not None and not agrees(reported, here.leg):
            text = p.confirm_leg(aircraft, p.leg(here.leg))
            self.log_circuit(session, station, here, said="confirm_leg", **note)
            instruction = session.remember(
                Instruction("confirm_position", station_key(station), text, {},
                            readback_required=False))
            return Reply(text, station, instruction=instruction)

        if reported:
            session.pattern_leg = reported
            # The report the tower was waiting for has come, or a later one.
            session.expected_report = ""
            session.expected_report_chased = False
        if session.orbiting and not reported:
            # "Orbit complete": the report asked for after it.
            return self.orbit_complete(session, station, p, circuit, here,
                                       said_by="pilot")
        if session.expected_vrp and self._mentions(parsed, session.expected_vrp, p):
            # Over the point it was asked to report: now it is given the join.
            decision("vrp", airport=station.ident, use="reported",
                     point=session.expected_vrp)
            session.vrp_reported = session.expected_vrp
            session.expected_vrp = ""

        # Who is in front, by where everybody is in the circuit. Without a
        # picture of the user, the arrival stream as it was before.
        if here is not None:
            sequence, there, followed = self.circuit_sequence(circuit, here, p)
        else:
            there = followed = None
            distance = self._distance_to(state, airport)
            ahead = [f for f in self._traffic_ahead(True)
                     if not distance or f.distance_nm < distance]
            sequence = (p.sequence_number(len(ahead) + 1,
                                          ahead[-1].describe(language_of(p)))
                        if ahead else "")
        note["sequence"] = sequence or None

        # --- on base or final: this is the landing clearance ---------------
        on_final = (reported in ("base", "final")
                    or self._on_final(state, airport, runway))
        # On base or final and too close behind the one ahead to land after
        # it: an orbit, ended by the geometry (ArrivalWatch) or the report.
        if (on_final and not session.cleared_landing and self.realism.delays
                and not session.orbiting and followed is not None
                and here is not None and there is not None
                and here.to_go_nm - there.to_go_nm < self.ORBIT_SPACING_NM):
            side = "right" if session.pattern_direction == "left" else "left"
            session.orbiting = side
            session.orbit_turned = 0.0
            session.orbit_track = -1.0
            text = p.orbit(aircraft, p.side(side), sequence)
            self.log_circuit(session, station, here, said="orbit", side=side,
                             behind=followed.written,
                             gap_nm=round(here.to_go_nm - there.to_go_nm, 2),
                             **note)
            instruction = session.remember(
                Instruction("circuit_report", station_key(station), text, {},
                            readback_required=False))
            return Reply(text, station, instruction=instruction)
        if on_final and not session.cleared_landing:
            self.log_circuit(session, station, here, said="clearance", **note)
            return self._clear_the_circuit(
                session, station, weather, p, runway, sequence, station_name,
                caution=self._caution(session, weather, p,
                                      followed.type_code if followed is not None else "",
                                      "landing"),
                low_approach="low approach" in parsed.normalized)

        # --- in the circuit: sequence them and give them the next report ---
        if reported in ("overhead", "deadside"):
            # Letting down on the dead side after an overhead join: the
            # downwind is next.
            session.phase = Phase.APPROACH
            text = p.report_position(aircraft, p.join_report())
            session.expected_report = "downwind"
            session.expected_report_chased = False
            self.log_circuit(session, station, here, said="circuit_report",
                             **note)
            instruction = session.remember(
                Instruction("circuit_report", station_key(station), text, {},
                            readback_required=False))
            return Reply(text, station, instruction=instruction)

        if reported in ("downwind", "crosswind", "upwind"):
            session.phase = Phase.APPROACH
            after_downwind, expect_after = p.report_after_downwind()
            if reported != "downwind":
                text = p.report_position(aircraft, p.join_report())
                session.expected_report = "downwind"
            elif followed is not None and here is not None and (
                    here.to_go_nm - there.to_go_nm < self.CIRCUIT_SPACING_NM):
                # Too close behind to turn base where it would: extended, and
                # the base is the tower's to call (ArrivalWatch.circuit).
                text = p.extend_downwind(aircraft, sequence)
                session.base_owed_behind = followed.written
                session.expected_report = ""
            else:
                # The FAA asks for the base, CAP 413 and ICAO for the final,
                # the DGAC for the "dernier virage" (report_after_downwind).
                text = p.report_position(aircraft, after_downwind,
                                         sequence=sequence)
                session.expected_report = expect_after
            session.expected_report_chased = False
            self.log_circuit(session, station, here, said="circuit_report",
                             **note)
            instruction = session.remember(
                Instruction("circuit_report", station_key(station), text, {},
                            readback_required=False))
            return Reply(text, station, instruction=instruction)

        if on_final and session.cleared_landing:
            # The landing clearance is already issued: a final report now
            # is the rollout talking, not a request to rejoin the circuit.
            return Reply(p.roger(session.aircraft), station)

        # --- everything else is a join, from where the aeroplane is --------
        session.phase = Phase.APPROACH
        if here is not None:
            join = choose_join(here, circuit, self.region_for(station.ident))
        else:
            distance = self._distance_to(state, airport)
            join = ("straight_in" if distance and distance > self.PATTERN_JOIN_NM * 2
                    else "downwind")
        if join == "overhead":
            # Over the field two thousand feet above it, on the QNH, then
            # down on the dead side and into the circuit (the UK's Standard
            # Overhead Join; France's verticale terrain).
            elevation = float(getattr(airport, "elev_ft", 0.0) or 0.0)
            overhead_ft = math.ceil((elevation + 2000.0) / 100.0) * 100.0
            session.expected_report = "downwind"
            session.expected_report_chased = False
            text = p.join_overhead(aircraft, station_name, runway, overhead_ft,
                                   weather.qnh_hpa, direction)
            self.log_circuit(session, station, here, said="join", join=join,
                             spoken_join=join, overhead_ft=overhead_ft, **note)
            instruction = session.remember(
                Instruction("pattern_entry", station_key(station), text,
                            {"runway": runway,
                             "pattern_direction": session.pattern_direction}))
            return Reply(text, station, instruction=instruction)
        spoken = join
        leg, report, expect = {
            "straight_in": ("", p.REPORT_FINAL, "final"),
            "base": (p.leg("base"), p.REPORT_BASE, "base"),
            "crosswind": (p.leg("crosswind"), p.join_report(), "downwind"),
            "downwind": (p.leg("downwind"), p.join_report(), "downwind"),
        }[spoken]
        session.expected_report = expect
        session.expected_report_chased = False
        # Outside the published points, the first report is the point on the
        # aeroplane's side of the field (the plan's "report November"),
        # wherever the field has points a controller can say. Not in the US,
        # whose charted VFR waypoints are codes nobody says on the radio.
        point = (self.vrp_ahead(airport, state)
                 if p.region != "faa" and here is not None and not here.in_circuit
                 and not session.vrp_reported else None)
        if point is not None and self.realism.delays and circuit is not None:
            busy = len(self.circuit_traffic(circuit))
            if busy >= self.CIRCUIT_FULL:
                # Full: held at the point, and called in from it when there
                # is room (ArrivalWatch).
                session.holding_at = point.name
                session.expected_report = ""
                text = p.hold_at(aircraft, station_name, p.vrp(point.name), runway)
                self.log_circuit(session, station, here, said="hold",
                                 point=point.name, in_circuit=busy, **note)
                instruction = session.remember(
                    Instruction("hold_at", station_key(station), text, {}))
                return Reply(text, station, instruction=instruction)
        if point is not None:
            report = p.vrp(point.name, point.altitude_ft)
            session.expected_vrp = point.name
            session.expected_report = ""
            decision("vrp", airport=airport.ident, use="entry", point=point.name,
                     source=point.source)
        text = p.enter_pattern(
            aircraft, station_name, runway,
            leg=leg, direction=direction, report=report,
            weather=weather if first_call else None,
            sequence=sequence, straight_in=spoken == "straight_in",
        )
        self.log_circuit(session, station, here, said="join", join=join,
                         spoken_join=spoken, **note)
        instruction = session.remember(
            Instruction("pattern_entry", station_key(station), text,
                        {"runway": runway,
                         "pattern_direction": session.pattern_direction}))
        return Reply(text, station, instruction=instruction)

    def orbit_complete(self, session, station, p, circuit, here,
                       said_by: str) -> Reply:
        """Round the orbit: back into the sequence, and the final asked for."""
        session.orbiting = ""
        session.expected_report = "final"
        session.expected_report_chased = False
        sequence = self.circuit_sequence(circuit, here, p)[0] if here is not None else ""
        text = p.report_position(session.aircraft, p.REPORT_FINAL,
                                 sequence=sequence)
        self.log_circuit(session, station, here, said="orbit_complete",
                         by=said_by, turned_deg=round(session.orbit_turned))
        instruction = session.remember(
            Instruction("circuit_report", station_key(station), text, {},
                        readback_required=False))
        return Reply(text, station, instruction=instruction)

    def _clear_the_circuit(self, session, station, weather, p, runway,
                           sequence, station_name, caution: str = "",
                           low_approach: bool = False) -> Reply:
        """The clearance at the end of a circuit.

        An aircraft doing circuits gets "cleared for the option", which covers
        the touch-and-go, the low approach and the full stop without the tower
        having to know which one the pilot has decided on. One that asked for
        a low approach by name is given it; one that is coming in to land gets
        a landing clearance.
        """
        aircraft = session.aircraft
        abbreviated = bool(session.established_with(station))
        withheld = self.landing_or_withheld(
            session, station, weather, p, self.navdb.airport(station.ident),
            runway, station_name=station_name, sequence=sequence,
            caution=caution)
        if withheld is not None:
            return withheld
        if low_approach:
            # Asked for by name: "cleared low approach", not the option.
            # It is still a runway clearance -- the phase and the wind
            # snapshot below treat it like any other.
            text = p.cleared_low_approach(aircraft, runway)
            kind = "low_approach_clearance"
            session.cleared_landing = True
        elif session.pattern_work:
            text = p.cleared_touch_and_go(
                aircraft, runway, weather, sequence=sequence,
                station=station_name, abbreviated=abbreviated, caution=caution)
            kind = "option_clearance"
            session.cleared_option = True
        else:
            text = p.cleared_to_land(
                aircraft, runway, weather, station=station_name,
                sequence=sequence, abbreviated=abbreviated, caution=caution)
            kind = "landing_clearance"
            session.cleared_landing = True
        session.phase = Phase.LANDING
        session.allow_runway(runway)
        session.note_clearance_wind(weather)
        instruction = session.remember(
            Instruction(kind, station_key(station), text, {"runway": runway}))
        return Reply(text, station, instruction=instruction)

    @staticmethod
    def _mentions(parsed, name: str, p) -> bool:
        """Whether a transmission names a reporting point, said the way the
        controller said it ("Whiskey") or written ("W")."""
        text = f" {parsed.normalized} "
        said = p.vrp(name).lower()
        return f" {said} " in text or f" {name.lower()} " in text

    def _issue_transition(self, session, station, weather, p, parsed,
                          state=None) -> Reply:
        """A VFR flight asking to cross the airspace on its way past.

        Approved with a ceiling and a reporting point, which is the shape of
        the instruction in every state. Refused while the runway is occupied,
        because a tower with an aeroplane on the pavement has nothing to spare
        for somebody crossing overhead, and a controller that can only ever
        say yes is not a controller.
        """
        aircraft = session.aircraft
        airport = self.navdb.airport(station.ident)
        airspace = p.airspace(self._airspace_class(airport, station))
        first_call = not session.established_with(station)

        refused = self._below_vmc(session, station, weather, p, airport,
                                  asked="transit", outside=True, offer=False)
        if refused is not None:
            return refused

        if station.position == "TWR" and self._runway_blocked(
                airport, self._departure_runway(airport, weather)
                if airport is not None else ""):
            text = p.remain_clear_of_airspace(
                aircraft, airspace,
                self._station_name(station, p) if first_call else "")
            instruction = session.remember(
                Instruction("remain_clear", station_key(station), text, {},
                            readback_required=False))
            return Reply(text, station, instruction=instruction)

        # Approved up to the shelf, not to whatever was asked: a request for
        # five thousand through a shelf at four is a transit approved to
        # four, and an approval read back at five takes the flight into the
        # airspace above somebody else's.
        ceiling = min(float(parsed.get("altitude_ft")
                            or self._vfr_ceiling(airport)),
                      float(self._vfr_ceiling(airport)))
        session.vfr_ceiling_ft = float(ceiling)
        session.transition_approved = True
        if self._airspace_class(airport, station) == "B":
            # Cleared through a Bravo is cleared into it (FAA JO 7110.65
            # chapter 7): the airspace watch lets the aeroplane in.
            session.cleared_bravo = airport.ident
        route = ""
        if state is not None and getattr(state, "has_position", False):
            point = self.vrp_toward(
                airport, bearing_deg(airport.lat, airport.lon,
                                     state.latitude, state.longitude),
                departing=False) if airport is not None else None
            if point is not None:
                # French says "transit autorisé par Sierra Echo"; the
                # preposition is the phrase's own.
                route = (p.vrp(point.name) if language_of(p) == "fr"
                         else f"via {p.vrp(point.name)}")
        text = p.cleared_through_airspace(
            aircraft, airspace, altitude_ft=ceiling, route=route,
            report=p.clear_of(airspace),
            transition_altitude=self._transition_altitude(airport),
        )
        instruction = session.remember(
            Instruction("transition", station_key(station), text,
                        {"vfr_ceiling_ft": float(ceiling)}))
        return Reply(text, station, instruction=instruction)

    # How close an arrival has to be before a landing clearance is issued.
    # Further out than this a tower says "continue approach" and clears the
    # aircraft later, because a clearance given at forty miles has to be
    # cancelled the moment anything changes.
    LANDING_CLEARANCE_NM = 10.0

    def _tower_check_in_on_final(self, parsed, session, station, weather, p,
                                 state) -> Reply:
        """The landing clearance if it can be given now; else "runway 27
        center, continue, I'll call your clearance", and the arrival watch
        clears it once the runway is seen empty and nobody is ahead on the
        final (occupancy.py, through landing_or_withheld)."""
        airport = self.navdb.airport(station.ident)
        runway = session.arrival_runway or self._arrival_runway(airport, weather)
        session.arrival_runway = runway
        ahead = self.arrivals_ahead(airport, runway, state)
        if ahead and not session.cleared_landing:
            # Somebody ahead on the same final: the clearance waits for them
            # (arrival.py _runway_owed gives it once nobody is).
            session.landing_withheld = True
            session.phase = Phase.LANDING
        else:
            reply = self._issue_landing(session, station, weather, p, parsed,
                                        state)
            if reply.instruction is None \
                    or reply.instruction.kind != "continue_approach":
                return reply
        text = p.tower_continue(
            session.aircraft, runway,
            station=(self._station_name(station, p)
                     if not session.established_with(station) else ""),
            # Its place behind whoever is ahead on the same final: "number
            # two" behind one (live traffic, Phase 5).
            sequence=p.sequence_number(len(ahead) + 1) if ahead else "")
        decision("tower", event="continue", runway=runway,
                 station=station.callsign,
                 ahead=ahead[0].contact.written if ahead else None)
        instruction = session.remember(Instruction(
            "continue_approach", station_key(station), text, {},
            readback_required=False))
        return Reply(text, station, instruction=instruction)

    def _issue_landing(self, session, station, weather, p, parsed,
                       state=None) -> Reply:
        airport = self.navdb.airport(station.ident)
        runway = (
            session.arrival_runway
            or parsed.get("runway")
            or self._arrival_runway(airport, weather)
        )
        session.arrival_runway = runway

        if state is not None and getattr(state, "on_ground", False):
            # Nothing on the ground can be cleared to land, whatever it asked
            # for. Saying so is more use than a clearance it cannot fly.
            return Reply(p.already_on_the_ground(session.aircraft), station)

        if session.cleared_landing:
            return Reply(
                p.already_cleared_to_land(session.aircraft, runway), station
            )

        distance = self._distance_to(state, airport)
        if distance > self.LANDING_CLEARANCE_NM or (
                state is not None and not self.re_established(session, airport, state)):
            text = p.continue_approach(session.aircraft)
            session.phase = Phase.APPROACH
            instruction = session.remember(
                Instruction("continue_approach", station_key(station), text, {},
                            readback_required=False)
            )
            return Reply(text, station, instruction=instruction)

        # Where the user comes in the arrival stream. "Number two, follow the
        # Airbus" is only worth saying when there is an Airbus to follow, so
        # the sequence is built from the traffic that actually exists --
        # and on the same ruler: the traffic's distance_nm is to the
        # threshold, so the user's is measured there too, not to the
        # field's centre a mile off it.
        final = final_for(self.navdb, airport.ident, runway)
        if final is not None and state is not None \
                and getattr(state, "has_position", False):
            threshold_nm = final.distance(state)
            if math.isfinite(threshold_nm):
                distance = threshold_nm
        sequence = ""
        ahead = [f for f in self._traffic_ahead(True) if f.distance_nm < distance]
        if ahead:
            sequence = p.sequence_number(len(ahead) + 1,
                                         ahead[-1].describe(language_of(p)))
        withheld = self.landing_or_withheld(
            session, station, weather, p, airport, runway,
            station_name=(self._station_name(station, p)
                          if not session.established_with(station) else ""),
            sequence=sequence,
            caution=self._caution(session, weather, p,
                                  ahead[-1].type_code if ahead else "",
                                  "landing"),
            state=state)
        if withheld is not None:
            return withheld

        text = p.cleared_to_land(
            session.aircraft, runway, weather,
            station=self._station_name(station, p) if not session.established_with(station) else "",
            sequence=sequence,
            abbreviated=session.established_with(station),
            caution=self._caution(session, weather, p,
                                  ahead[-1].type_code if ahead else "",
                                  "landing"),
        )
        session.cleared_landing = True
        session.phase = Phase.LANDING
        session.allow_runway(runway)
        session.note_clearance_wind(weather)
        instruction = session.remember(
            Instruction("landing_clearance", station_key(station), text,
                        {"runway": runway})
        )
        return Reply(text, station, instruction=instruction)

