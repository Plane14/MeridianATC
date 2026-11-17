from __future__ import annotations

from ._shared import (
    FlightSession,
    Instruction,
    Intent,
    ParsedIntent,
    Phase,
    Phraseology,
    Reply,
    Station,
    _airborne,
    decision,
    language_of,
    re,
    same_channel,
    station_key,
)


class _What_the_controller:
    # ------------------------------------------------------------------
    # what the controller is waiting to hear
    # ------------------------------------------------------------------
    #
    # "Say again" tells a pilot that they were not understood and nothing
    # else. On a frequency they have never worked, in a language they are
    # learning, that is the least useful thing a controller could say -- and
    # the controller knows perfectly well what call would make sense, because
    # it knows the position it is working and where the flight has got to.
    # So it says that instead.

    def expected_calls(self, session: FlightSession, station: Station,
                       state=None, language: str = "en") -> list[str]:
        """The calls that would make sense on this frequency, right now."""
        from ..pilot import BOOKS, ENGLISH

        book = BOOKS.get((language or "en")[:2], ENGLISH)
        position = station.position if station else ""
        phase = session.phase
        airborne = _airborne(state)
        inbound = phase in (Phase.LANDED, Phase.TAXI_IN, Phase.PARKED)

        wanted: list[str] = []
        if position == "DEL":
            if session.clearance_issued:
                # It has been given. Naming it again is the one call that
                # cannot help, and it was the call this kept naming.
                wanted = [book.request_pushback, book.request_taxi]
            elif not session.ifr:
                wanted = [book.request_vfr_departure, book.request_pushback]
            elif session.destination and session.destination.spoken:
                wanted = [book.request_clearance.format(
                    dest=session.destination.spoken).strip(),
                    book.request_pushback]
            else:
                # No destination filed. The call still has to be nameable, and
                # "request IFR clearance to" with nothing after it is not a
                # call anybody can make.
                wanted = [book.request_clearance_plain
                          or book.request_clearance.format(dest="").strip(),
                          book.request_pushback]
        elif position == "GND":
            wanted = ([book.request_taxi_to_parking] if inbound
                      else [book.request_taxi, book.request_pushback])
        elif position == "TWR":
            if airborne:
                wanted = [book.request_landing, book.field_in_sight]
                if not session.ifr:
                    wanted.insert(0, book.request_pattern_work)
            elif inbound:
                wanted = [book.request_taxi_to_parking]
            else:
                wanted = [book.ready_for_departure_plain]
                if not session.ifr:
                    wanted.append(book.request_pattern_work)
        elif position in ("DEP", "APP", "CTR"):
            wanted = [book.check_in]
            if session.ifr:
                if phase in (Phase.DESCENT, Phase.APPROACH):
                    wanted.append(book.request_landing)
            else:
                wanted += [book.request_flight_following,
                           book.request_transition]
        if not self.ground_control:
            # Suggesting a call nobody will answer is worse than not
            # suggesting it.
            wanted = [call for call in wanted if call != book.request_pushback]
        return [call for call in wanted if call][:3]

    def _repeat(self, instruction: Instruction, station: Station, p,
                aircraft) -> str:
        """An instruction said again, all in one language.

        In the language being spoken now when it can be rebuilt from its
        values; otherwise whole, in the language it was issued in. The stored
        sentence used to go inside the current language's announcement --
        "je répète, descend to flight level two five zero".
        """
        from ..restate import restate

        spoken = language_of(p)
        if not instruction.language or instruction.language == spoken:
            return p.say_again_instruction(aircraft, instruction.text)
        rebuilt = restate(instruction, p, aircraft)
        if rebuilt:
            return p.say_again_instruction(aircraft, rebuilt)
        original = self.for_station(station, instruction.language)
        aircraft.language = instruction.language
        return original.say_again_instruction(aircraft, instruction.text)

    # What the first call after a handoff is read as when it is really the
    # check-in. On a radar frequency the readings that come of the words a
    # check-in carries: the altitudes ("climbing seven thousand five
    # hundred", read as a climb request or a readback), and "VFR to the
    # north", read as asking for a VFR departure. A tower or ground keeps
    # the requests it works ("ready for departure", "five miles final"), so
    # only what it could not read at all is taken as the check-in there.
    _CHECK_IN_READINGS = frozenset({Intent.UNKNOWN, Intent.READBACK,
                                    Intent.ACKNOWLEDGE})
    _RADAR_CHECK_IN_READINGS = _CHECK_IN_READINGS | {
        Intent.REQUEST_VFR_DEPARTURE, Intent.REPORT_LEAVING,
        Intent.REQUEST_CLIMB, Intent.REQUEST_DESCENT}
    _ASKING = re.compile(r"\b(?:request|requesting|like)\b")

    def _checking_in_after_handoff(self, session: FlightSession,
                                   station: Station,
                                   parsed: ParsedIntent) -> bool:
        """Whether this is the first call from an aeroplane sent here.

        After 26 October's departure from Centennial: the tower's handoff
        was read back, and "Denver Departure, Diamond one seven two Sierra
        Papa, six thousand five hundred climbing seven thousand five
        hundred, VFR to the north" was answered "expecting with you, or
        request flight following, or request transition" -- the controller
        listing the check-in to a pilot who had just made it. A controller
        handed an aeroplane is waiting for its call, and the first one with
        the callsign in it is the check-in.
        """
        handed = session.handoff_pending
        if (handed is None or not parsed.callsign_matched
                or not same_channel(handed.mhz, station.mhz, 0)
                or session.established_with(station)):
            return False
        pending = session.pending
        if (pending is not None and pending.readback_required
                and not pending.read_back
                and pending.station_key == station_key(station)):
            return False
        radar = station.position in ("DEP", "APP", "CTR")
        readings = (self._RADAR_CHECK_IN_READINGS if radar
                    else self._CHECK_IN_READINGS)
        if parsed.intent not in readings:
            return False
        return not (parsed.intent in (Intent.REQUEST_CLIMB, Intent.REQUEST_DESCENT) and self._ASKING.search(parsed.normalized))

    # The pilot's altitude and the Mode C disagree by this much or more:
    # the controller says what it sees before telling it to descend.
    MODE_C_DISAGREES_FT = 300.0

    def _altitude_answered(self, session: FlightSession, station: Station,
                           parsed: ParsedIntent, p, state) -> Reply | None:
        """The answer to "say altitude", asked of a VFR flight above the
        altitude this radar holds it under (watch.RestrictionWatch).

        It was "roger", whatever the pilot said: an aeroplane 700 ft over
        its restriction reported itself under it and was left there. The
        answer is read as an altitude and set against the Mode C -- the
        same, and it is told to descend; 300 ft or more apart, it is told
        what the controller sees first, because one of the two is wrong and
        the pilot is the one who can tell which.
        """
        from ..watch import RestrictionWatch

        key = station_key(station)
        if (not session.restriction_asked_at or session.radar_service != key
                or session.ifr or not session.vfr_ceiling_ft):
            return None
        heard = parsed.get("altitude_ft")
        shown = getattr(state, "altitude_ft", None) if state is not None else None
        if not heard or shown is None:
            return None
        ceiling = float(session.vfr_ceiling_ft)
        if shown < ceiling + RestrictionWatch.ABOVE_FT:
            # Complied: the next time it is above the ceiling is a new
            # violation, and a controller warns on every one of them.
            session.advisories.discard(f"restriction:descend:{key}")
            return None
        if not session.advise_once(f"restriction:descend:{key}"):
            return None
        session.establish(station)
        disagree = abs(float(heard) - float(shown)) >= self.MODE_C_DISAGREES_FT
        text = p.descend_at_or_below(
            session.aircraft, ceiling,
            transition_altitude=self._transition_altitude(
                self.navdb.airport(station.ident)),
            shown_ft=round(float(shown) / 100.0) * 100.0 if disagree else 0.0)
        decision("vfr_restriction", station=station.callsign, said="descend",
                 heard_ft=heard, mode_c_ft=round(float(shown)),
                 ceiling_ft=round(ceiling), disagree=disagree)
        instruction = session.remember(Instruction(
            "vfr_ceiling", key, text, {"vfr_ceiling_ft": ceiling}))
        return Reply(text, station, instruction=instruction)

    def _not_understood(self, session: FlightSession, station: Station,
                        p: Phraseology, state=None) -> Reply:
        """The answer to a transmission the controller could not act on."""
        return Reply(
            p.expecting(session.aircraft,
                        self.expected_calls(session, station, state,
                                            language_of(p))),
            station,
        )

