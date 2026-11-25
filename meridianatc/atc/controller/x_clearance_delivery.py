from __future__ import annotations

from .. import procedure_legs
from ._shared import (
    Any,
    Instruction,
    Intent,
    Reply,
    _airborne,
    decision,
    initial_altitude,
    normalize_runway,
    procedure_heard,
    procedure_near,
    re,
    required_readback_items,
    same_channel,
    squawk_for,
    station_key,
    terrain,
    written_procedure,
)


class _Clearance_delivery:
    # ------------------------------------------------------------------
    # clearance delivery
    # ------------------------------------------------------------------

    def _send_on(self, session, station, p, *positions) -> Reply | None:
        """Hand the aircraft to the first of these positions that is staffed.

        ``None`` when none of them is, which means the position holding the
        aircraft has to deal with the request itself rather than pass it to
        somebody who is not there.
        """
        for position in positions:
            found = self.controller_at(station.ident, position)
            if found is None or station_key(found) == station_key(station):
                continue
            return Reply(
                p.handoff(session.aircraft,
                          self._station_name(found, p, short=True), found.mhz),
                station, handoff_to=found,
            )
        return None

    def _delivery(self, parsed, session, station, weather, p, state) -> Reply:
        aircraft = session.aircraft

        # Delivery works aircraft at the gate. One that is already flying wants
        # a controller who can see it on radar, so it is sent to one rather
        # than issued a departure clearance it has no use for.
        if _airborne(state) and parsed.intent is not Intent.RADIO_CHECK:
            return self._hand_off_departure(session, station, p)

        if parsed.intent in (Intent.REQUEST_CLEARANCE, Intent.CHECK_IN,
                             Intent.POSITION_REPORT):
            # A VFR flight has no IFR clearance to be given. Reading one to it
            # was the clearest case of the controller using the wrong half of
            # the phrasebook: it is released, not cleared to a destination.
            if not session.ifr:
                return self._issue_vfr_release(session, station, p, parsed, weather)
            return self._issue_clearance(session, station, weather, p)

        if parsed.intent is Intent.REQUEST_VFR_DEPARTURE:
            return self._issue_vfr_release(session, station, p, parsed, weather)

        if parsed.intent is Intent.REQUEST_PATTERN_WORK:
            # Circuits are a tower matter, and the aircraft has to taxi first.
            # They are flown visually (FlightSession.visual) without changing
            # the rules the flight was filed under.
            session.pattern_work = True
            ground = self.controller_at(station.ident, "GND")
            if ground and not same_channel(ground.mhz, station.mhz, 0):
                return Reply(
                    p.handoff(aircraft,
                              self._station_name(ground, p, short=True),
                              ground.mhz),
                    station, handoff_to=ground,
                )
            return self._issue_taxi(session, station, weather, p, state)

        if parsed.intent in (Intent.REQUEST_TAXI,
                             Intent.REQUEST_TAXI_TO_PARKING):
            ground = self.controller_at(station.ident, "GND")
            if ground and not same_channel(ground.mhz, station.mhz, 0):
                return Reply(
                    p.handoff(aircraft, self._station_name(ground, p, short=True), ground.mhz),
                    station, handoff_to=ground,
                )
            # No ground position: delivery at a small field does its
            # work, in the direction the aeroplane is going.
            if parsed.intent is Intent.REQUEST_TAXI_TO_PARKING:
                return self._issue_taxi_in(session, station, weather, p,
                                           state)
            return self._issue_taxi(session, station, weather, p, state)

        # Delivery pushes nobody back and launches nobody, but it is the
        # frequency the aircraft is on, so it answers -- by naming whoever
        # does that job, or by doing it itself at a field with nobody else.
        # Handing the request back as a list of other calls was the one reply
        # that left the pilot with nowhere to go.
        if parsed.intent is Intent.REQUEST_PUSHBACK:
            sent = self._send_on(session, station, p, "GND")
            if sent is not None:
                return sent
            reason = self._stuck_on_the_stand()
            if reason:
                session.tell_to_wait(hold=True)
                return Reply(p.hold_position(aircraft) + f" You are {reason}.",
                             station)
            return Reply(p.pushback_approved(aircraft), station,
                         ground="pushback")

        if parsed.intent is Intent.REQUEST_TAKEOFF:
            if _airborne(state):
                return Reply(p.already_airborne(aircraft), station)
            sent = self._send_on(session, station, p, "TWR", "GND")
            if sent is not None:
                return sent
            return self._issue_takeoff(session, station, weather, p, parsed, state)

        if parsed.intent is Intent.RADIO_CHECK:
            return Reply(p.radio_check(aircraft), station)

        if parsed.intent in (Intent.ACKNOWLEDGE, Intent.AFFIRMATIVE):
            return Reply("", station, silent=True)

        return self._not_understood(session, station, p, state, parsed=parsed)

    def _issue_clearance(self, session, station, weather, p) -> Reply:
        airport = self.navdb.airport(station.ident)
        destination = session.destination
        dest_name = (
            destination.spoken if destination
            else (session.destination_input or p.UNKNOWN_DESTINATION)
        )

        session.squawk = squawk_for(session, station.ident)
        cruise = (session.cruise_altitude_ft
                  if session.cruise_altitude_ft > 0
                  else self._default_cruise(session))

        departure_station = (
            self.navdb.station_for(station.ident, "DEP")
            or self.navdb.station_for(station.ident, "APP")
        )
        departure_freq = departure_station.mhz if departure_station else 0.0

        # No open dataset publishes SIDs, so the clearance is issued with
        # radar vectors, which is correct phraseology in its own right --
        # unless the flight plan names one for the runway in use, or the
        # box's own legs walk one, which is what a controller would clear.
        sid = self._planned_sid(airport, weather)
        runway = self._departure_runway(airport, weather) if airport else ""
        worked = self.realism.ifr_departure
        if worked:
            if not sid:
                sid = self._fms_sid(airport, runway)
            initial, expect_minutes = self._departure_altitudes(
                session, airport, sid, runway, cruise)
            sid = self._sid_for_runway(airport, sid, runway, weather)
        else:
            initial, expect_minutes = session.initial_altitude_ft or 5000.0, 10
        session.initial_altitude_ft = float(initial)
        # As the chart prints it, ROUSY 1L, which is what is said and what a
        # readback is held to; the plan's six-letter ROUS1L is neither.
        sid_name = written_procedure(sid, self._plan_idents()) if sid else ""
        session.sid = sid_name
        text = p.ifr_clearance(
            session.aircraft,
            self._station_name(station, p) if not session.established_with(station) else "",
            dest_name,
            departure_procedure=sid_name,
            route=p.ROUTE_AS_FILED if sid else p.ROUTE_RADAR_VECTORS,
            initial_altitude=initial,
            cruise_altitude=cruise,
            expect_minutes=expect_minutes,
            departure_freq=departure_freq,
            squawk=session.squawk,
            transition_altitude=self._transition_altitude(airport),
            runway=runway if worked else "",
        )

        session.clearance_issued = True
        # The runway named in the clearance is the runway the taxi is
        # for. Left unset it is picked again when ground is asked, and a
        # wind that has come around in between sends the aeroplane to
        # the other end of the strip it was cleared off.
        if worked and runway:
            session.departure_runway = runway
        values: dict[str, Any] = {"altitude_ft": int(initial), "squawk": session.squawk}
        if departure_freq:
            values["frequency"] = departure_freq
        # A clearance readback that gets the altitude and the code right is
        # accepted. Pilots routinely leave the departure frequency out of the
        # readback and no controller challenges it. The SID is not left out:
        # it is the route the aeroplane is about to fly (ICAO Doc 4444
        # 4.5.7.5.1 a), the route clearance, is a readback item).
        required = {"altitude_ft", "squawk"}
        if sid_name and worked:
            values["sid"] = sid_name
            required.add("sid")
        instruction = session.remember(
            Instruction("ifr_clearance", station_key(station), text, values,
                        required_items=required)
        )
        return Reply(text, station, instruction=instruction)

    def _hear_procedure(self, parsed, session) -> None:
        """A SID or a STAR owed in a readback, found in what was said.

        The parser has no slot for a procedure's name -- it would have to
        know every SID there is -- so the one that is owed is looked for in
        the words (atc/procedures.py ``heard``) and put with the rest of the
        readback's items.
        """
        pending = session.pending
        if pending is None:
            return
        text = getattr(parsed, "text", "") or ""
        for key in ("sid", "star"):
            if key not in (pending.values or {}):
                continue
            expected = str(pending.values[key])
            if procedure_heard(text, expected, self._plan_idents()):
                parsed.readback_items[key] = expected
            elif (key == "sid" and self._unsure(parsed)
                  and procedure_near(text, expected, self._plan_idents())):
                parsed.nearly.add(key)

    def _confirm_below(self) -> float:
        return float(getattr(self.hearing, "confirm_below", 0.0) or 0.0)

    def _unsure(self, parsed) -> bool:
        """The recogniser was less sure of these words than
        ``speech.confirm_below``; typed text is always sure."""
        return getattr(parsed, "heard_confidence", 1.0) < self._confirm_below()

    def _plan_idents(self) -> list[str]:
        idents = [f.ident for f in getattr(self.route, "fixes", ()) or ()]
        base = idents or [getattr(f, "ident", "") for f in self.plan_fixes]
        # What the sim's own FMS has loaded corroborates: the procedure a
        # name resolves to is the same either way, but the box's fixes are
        # sometimes the ones a six-letter designator was named after.
        seen = set(base)
        return base + [i for i in self.fms_idents or ()
                       if i and i not in seen]

    def _departure_altitudes(self, session, airport, sid: str, runway: str,
                             cruise: float) -> tuple[int, int]:
        """The clearance's initial altitude and the minutes to the cruise.

        The altitude is the field's own (atc/climb.py), lifted over the
        terrain round it; the minutes are the plan's time to its top of
        climb, else the climb worked out from the cruise (atc/route.py).
        """
        from ..route import Route

        elevation = float(getattr(airport, "elev_ft", 0.0) or 0.0)
        floor = 0.0
        if airport is not None:
            floor = terrain.floor_ft(airport.lat, airport.lon, elevation)
        initial = initial_altitude(
            elevation, getattr(airport, "ident", ""), sid, runway,
            session.aircraft.callsign.written, floor_ft=floor)
        # Never cleared above the level the flight filed: a "climb to
        # FL080" to an aeroplane whose plan says 5,000 is an altitude it
        # cannot use, and no strip says otherwise. The terrain floor
        # still wins -- a filed level below it cannot be vectored to --
        # and where no terrain covers the field, a thousand over its
        # elevation stands in, so a nonsense filed level can never
        # clear an aeroplane into the ground it sits on.
        if cruise > 0:
            safe = max((int(floor) + 999) // 1000 * 1000,
                       (int(elevation) + 1999) // 1000 * 1000)
            initial = min(initial, max(int(cruise), safe))
        route = self.route if self.route else Route(())
        minutes = route.minutes_to_cruise(cruise)
        decision("clearance", airport=getattr(airport, "ident", ""),
                 initial_ft=initial, cruise_ft=int(cruise),
                 terrain_floor_ft=round(floor),
                 field_ft=round(elevation), sid=sid, runway=runway,
                 expect_minutes=minutes,
                 minutes_from=("plan" if route and route.top_of_climb
                               and route.top_of_climb.time_s else
                               "route" if route else "rate"))
        return initial, minutes

    def _fms_sid(self, airport, runway: str) -> str:
        """The SID the box has loaded, when the plan never said one.

        A procedure typed into the FMS never reaches ``plan_runways`` --
        it is fed by plans MeridianATC itself read -- but X-Plane's
        plugin ships the loaded legs' idents, and the field's decoded
        procedures say which SID's transition walks them. The leading
        legs of the loaded plan matching one departure's transition in
        order is the SID the pilot briefed, and the one "as filed"
        clears by name.
        """
        if airport is None:
            return ""
        procs = getattr(self, "terminal_procedures", {}).get(airport.ident)
        idents = [i for i in (self.fms_idents or ()) if i]
        if procs is None or not idents:
            return ""
        origin = getattr(airport, "ident", "") or ""
        # The loaded plan's leading idents are the departure's: the field
        # itself first, then the procedure's fixes. A short head is
        # plenty -- a SID's own fixes all sit at the front.
        head = [i for i in idents if i != origin][:8]
        if not head:
            return ""
        best, best_key = "", None
        for name, proc in procs.departures.items():
            trans = procedure_legs._transition_for(
                proc, "sid", runway, "", head)
            if trans is None:
                continue
            fixes = [leg.fix_ident.upper() for leg in trans.legs
                     if leg.fix_ident]
            if not fixes:
                continue
            at = 0
            hits = 0
            first_at = -1
            for fix in fixes:
                for i in range(at, len(head)):
                    if head[i] == fix:
                        hits += 1
                        if first_at < 0:
                            first_at = i
                        at = i + 1
                        break
            # Two fixes walking in order, or the only fix a short
            # transition has: anything less is a shared waypoint's luck.
            if hits < min(2, len(fixes)):
                continue
            key = (hits, -first_at)
            if best_key is None or key > best_key:
                best, best_key = name, key
        if best:
            decision("sid", airport=getattr(airport, "ident", ""),
                     cleared=best, runway=runway,
                     why="the FMS's loaded legs walk its transition")
        return best

    def _sid_for_runway(self, airport, sid: str, runway: str, weather) -> str:
        """The plan's SID while its runway is the one in use; said why not.

        A SimBrief plan names one SID, for one runway: another runway in use
        is radar vectors, and the log says which runway the SID was for.
        """
        planned = self.plan_runways.get(getattr(airport, "ident", ""), {})
        filed = planned.get("sid", "")
        if filed and not sid:
            decision("sid", airport=getattr(airport, "ident", ""), filed=filed,
                     planned_runway=planned.get("departing", ""),
                     runway=runway, cleared="radar vectors",
                     why="the plan's SID is for another runway")
        elif sid:
            decision("sid", airport=getattr(airport, "ident", ""), filed=sid,
                     runway=runway, cleared=sid)
        return sid

    # How high a VFR departure is kept until it is clear of the circuit. A
    # real one is published per field; this is the value that fits almost all
    # of them and is what keeps the aircraft under the arriving traffic.
    VFR_CEILING_AGL_FT = 2500.0

    # The requests a pilot names a runway in. Deliberately not every intent
    # that can carry one: a position report saying "runway two four in sight"
    # is not a request to change the runway in use, and a readback of the
    # runway just issued is the opposite of asking for another.
    _RUNWAY_REQUESTS = frozenset({
        Intent.REQUEST_CLEARANCE, Intent.REQUEST_TAXI, Intent.REQUEST_TAKEOFF,
        Intent.REQUEST_PUSHBACK, Intent.REQUEST_VFR_DEPARTURE,
        Intent.REQUEST_APPROACH, Intent.REQUEST_LANDING,
        Intent.REQUEST_PATTERN_WORK,
    })

    def _note_runway(self, parsed, session, station,
                     asked: bool = False) -> None:
        """Use the runway the pilot asked for, where the field has one.

        The runway was decided from the wind and nothing else, so a pilot at
        Prague asking for three zero was cleared for zero six every time --
        and asking again changed nothing, because there was nowhere for the
        request to be written down. A real controller is asked for a runway
        all the time and normally approves it: it is the pilot who knows how
        long their aeroplane needs and which way they are going.

        Only a runway the field actually publishes. One it has not got is left
        alone rather than acted on, so a misheard number gets the runway in
        use rather than a clearance to pavement that is not there.
        """
        if not asked and parsed.intent not in self._RUNWAY_REQUESTS:
            return
        wanted = parsed.get("runway")
        if wanted is None:
            return
        wanted = normalize_runway(str(wanted))
        if not wanted:
            return
        airport = self.navdb.airport(station.ident)
        if airport is None:
            return
        published = {name for name, _heading, _runway
                     in self.navdb.runway_ends(airport.ident)}
        if wanted not in published:
            return
        if session.phase.outbound:
            session.departure_runway = wanted
        else:
            session.arrival_runway = wanted

    # The position a callup names, by its last word ("Centennial Ground").
    # The field's own positions only: Approach and Departure are one radar
    # room at most fields, and "Boston Departure" said on the Approach
    # frequency is answered there, not sent away.
    _CALLED = {"ground": "GND", "tower": "TWR", "clearance": "DEL",
               "delivery": "DEL"}
    # How far into a transmission the callup is, in words, when the pilot
    # did not pause after it: the field's name and the position.
    _CALLUP_WORDS = 4

    def _called_elsewhere(self, session, station, parsed, p) -> Reply | None:
        """The pilot named another of this field's positions on this one's
        frequency: say where that one is, and do nothing else.

        At Centennial a call to "Centennial Ground" made on the Delivery
        frequency was answered by Delivery with a VFR release, as if it had
        been addressed -- which tells the pilot nothing about having dialled
        the wrong number, and has Delivery doing Ground's job. The one
        exception is what this position does at this field anyway: in the
        US a VFR departure from inside a Class B or C collects its squawk
        from Delivery, whoever the pilot happened to name.
        """
        if station is None or not parsed.understood \
                or parsed.intent in self._NOT_A_REQUEST:
            return None
        head = re.split(r"[,.;]", parsed.text or "", maxsplit=1)[0].lower()
        words = re.findall(r"[a-z]+", head)[:self._CALLUP_WORDS]
        called = next((self._CALLED[w] for w in words if w in self._CALLED), "")
        if not called or called == station.position                 or station.position not in ("DEL", "GND", "TWR"):
            return None
        other = self.controller_at(station.ident, called)
        if other is None or not self.staffed(other) \
                or same_channel(other.mhz, station.mhz, 4):
            return None
        airport = self.navdb.airport(station.ident)
        if (station.position == "DEL"
                and parsed.intent is Intent.REQUEST_VFR_DEPARTURE
                and p.region == "faa"
                and self._airspace_class(airport, station) in ("B", "C")):
            return None
        decision("called_elsewhere", station=station.callsign,
                 called=other.callsign, mhz=other.mhz)
        return Reply(p.called_elsewhere(
            session.aircraft, self._station_name(station, p),
            self._station_name(other, p), other.mhz), station)

    # What the pilot says back is the controller's words, not a request:
    # "squawk VFR" read back is not a VFR flight.
    _NOT_A_REQUEST = frozenset({Intent.READBACK, Intent.ACKNOWLEDGE})

    @classmethod
    def _note_rules(cls, session, parsed) -> None:
        """Believe the pilot about which rules they are flying under.

        The rules live on the session, and the session gets them from the
        flight it was set up with -- which is a setting, and one a pilot may
        never have looked at. So a flight left at the default called up, asked
        for an IFR clearance in as many words, and was answered with a VFR
        release; and because the release itself writes VFR onto the session,
        every later request was answered the same way. The one transmission
        that said what was actually filed was the one thing being thrown away.

        Only when it was said. A bare "request clearance" leaves the session
        where it is, which is what keeps a VFR flight collecting its release
        from delivery rather than being read an airways clearance it did not
        file. And to any station: it was Delivery and Ground only, so "IFR
        to Boston" said to a tower that issues its own clearances was not
        heard. A cancellation says "IFR" too, and is the radar's to take.
        """
        if (parsed.intent in cls._NOT_A_REQUEST
                or parsed.intent is Intent.CANCEL_IFR):
            return
        if parsed.rules == "ifr":
            session.ifr = True
        elif parsed.rules == "vfr":
            session.ifr = False

    def _issue_vfr_release(self, session, station, p, parsed=None,
                           weather=None) -> Reply:
        """Release a VFR flight, in the direction it asked to go.

        Not a clearance. It is an approval to leave, a ceiling while it is
        under somebody else's traffic, and a code -- and the code is the
        conspicuity one unless a radar position is going to work the flight,
        because nobody hands a discrete code to an aircraft they will never
        see.
        """
        session.ifr = False
        airport = self.navdb.airport(station.ident)
        refused = self._below_vmc(session, station, weather, p, airport,
                                  asked="vfr_release")
        if refused is not None:
            return refused
        departure_station = (
            self.navdb.station_for(station.ident, "DEP")
            or self.navdb.station_for(station.ident, "APP")
        )
        freq = departure_station.mhz if departure_station else 0.0
        session.squawk = (
            squawk_for(session, station.ident) if freq else p.vfr_squawk
        )

        asked = self._direction_asked(parsed)
        direction = p.bearing(asked)
        # The published way out in that direction, where the field has one
        # a controller can say ("leave the zone via Sierra Echo").
        exit_point = ""
        if asked in self._COMPASS_DEG and airport is not None:
            point = self.vrp_toward(airport, self._COMPASS_DEG[asked],
                                    departing=True)
            if point is not None:
                exit_point = p.vrp(point.name)
                session.expected_exit = point.name
                decision("vrp", airport=airport.ident, use="exit",
                         point=point.name, source=point.source)
        ceiling = self._vfr_ceiling(airport)
        session.vfr_ceiling_ft = ceiling
        # The pressure is read in the release below; ground need not
        # say it again with the taxi.
        session.altimeter_issued = weather is not None

        pressure = (0.0 if weather is None else
                    weather.altimeter_inhg if p.region == "faa"
                    else weather.qnh_hpa)
        text = p.vfr_departure_clearance(
            session.aircraft,
            self._station_name(station, p) if not session.established_with(station) else "",
            airspace=p.airspace(self._airspace_class(airport, station)),
            departure_freq=freq,
            squawk=session.squawk,
            altitude_restriction=ceiling,
            direction=direction,
            exit_point=exit_point,
            # The pressure is in every release: a VFR departure on the wrong
            # setting is the one a release exists to prevent. The way out of
            # the zone is the direction until there are reporting points to
            # name (Phase 4 of the VFR plan).
            pressure=pressure,
        )
        values: dict[str, Any] = {"squawk": session.squawk}
        if pressure:
            values["altimeter"] = pressure
        if freq:
            values["frequency"] = freq
        if ceiling:
            values["vfr_ceiling_ft"] = float(ceiling)
        if session.expected_exit and exit_point:
            values["exit_point"] = session.expected_exit
        instruction = session.remember(
            Instruction("vfr_release", station_key(station), text, values,
                        required_items=required_readback_items(values, p.dialect))
        )
        return Reply(text, station, instruction=instruction)

