from __future__ import annotations

from ._shared import (
    _ASKS_FOR_RUNWAY,
    _REPORT_WORD,
    FlightSession,
    Intent,
    ParsedIntent,
    Phase,
    Reply,
    Station,
    Weather,
    _airborne,
    decision,
    language_of,
    log,
    station_key,
)


class _Entry_point:
    # ------------------------------------------------------------------
    # entry point
    # ------------------------------------------------------------------

    def respond(
        self,
        parsed: ParsedIntent,
        session: FlightSession,
        station: Station,
        weather: Weather,
        state=None,
    ) -> Reply:
        """Produce the controller's reply to one pilot transmission."""
        language = getattr(parsed, "language", "en")
        p = self.for_station(station, language)
        aircraft = session.aircraft
        # The callsign is spoken differently in each language, so the aircraft
        # is told which one this exchange is in before any template runs.
        aircraft.language = language_of(p)
        self._hear_procedure(parsed, session)
        self._current_runway_heard(parsed, session, station)

        # An aerodrome traffic frequency has nobody on it, so nothing is said
        # on it -- including "say again". Answering only the transmissions it
        # failed to parse was the worst of both: silence when the pilot got it
        # right, and a voice when they did not.
        if not self.staffed(station):
            return Reply("", station, silent=True)

        if session.pending is not None and session.pending.station_key != station_key(station):
            # A readback owed to the frequency the pilot just left cannot be
            # collected here, but the instruction was still given: what it
            # changed is applied before it is dropped -- only the chase for
            # its readback is lost, not the instruction.
            if session.pending.values:
                session.apply(session.pending.values)
            session.clear_pending()

        # With Ground switched off nobody approves a pushback, and nobody
        # redirects it either: "contact Tower" for a pushback is the tower
        # doing Ground's job by another name. The request goes unanswered on
        # every frequency.
        if not self.ground_control and parsed.intent is Intent.REQUEST_PUSHBACK:
            return Reply("", station, silent=True)

        # A distress call is answered by whoever hears it, before anything
        # else is considered -- including an outstanding readback, and
        # including whether the request makes sense for the position. It used
        # to fall through every handler to the not-understood reply.
        if parsed.intent in (Intent.EMERGENCY, Intent.URGENCY,
                             Intent.MINIMUM_FUEL):
            session.establish(station)
            return self._emergency(parsed, session, station, weather, p, state)

        # The answer to the controller's questions after a MAYDAY or PAN.
        # Kept, and answered with the priority and the runway -- it used to
        # fall through to the list of calls the controller was "expecting".
        if parsed.intent is Intent.EMERGENCY_DETAILS:
            session.establish(station)
            session.emergency_details.update({
                key: parsed.values[key] for key in ("souls", "fuel_minutes")
                if key in parsed.values})
            if not session.emergency:
                return Reply(p.roger(aircraft), station)
            runway = self._emergency_runway(session, station, weather, state)
            return Reply(p.emergency_priority(aircraft, runway), station)

        # The reporting point an arrival was asked to report, named on its own
        # ("Whiskey"), is that report; no rule table knows a field's points.
        if (not parsed.understood and session.expected_vrp
                and self._mentions(parsed, session.expected_vrp, p)):
            parsed.intent = Intent.POSITION_REPORT
        if (not parsed.understood and session.expected_exit
                and self._mentions(parsed, session.expected_exit, p)):
            parsed.intent = Intent.POSITION_REPORT

        # The rules the pilot says they are flying under, to any station.
        self._note_rules(session, parsed)

        # Another of this field's positions, called on this one's frequency.
        elsewhere = self._called_elsewhere(session, station, parsed, p)
        if elsewhere is not None:
            return elsewhere

        if self._checking_in_after_handoff(session, station, parsed):
            parsed.intent = Intent.CHECK_IN

        answered = self._altitude_answered(session, station, parsed, p, state)
        if answered is not None:
            return answered

        # A transmission we could not understand gets a "say again", exactly as
        # it would on a real frequency. Guessing would be worse.
        if not parsed.understood and parsed.intent is not Intent.READBACK:
            if not parsed.normalized.strip():
                return Reply("", station, silent=True)
            # A readback owed here, garbled: ask for it again by name. The
            # list of calls the controller was "expecting" is the answer to a
            # pilot who has said nothing useful, not to one reading back.
            pending = session.pending
            if (pending is not None and pending.readback_required
                    and pending.station_key == station_key(station)):
                session.establish(station)
                # Unrecognised, but carrying exactly what is owed: that is
                # the readback, whatever words came with it.
                heard = parsed.readback_items
                if heard and pending.owed_items and not pending.mismatches(heard):
                    return self._check_readback(parsed, session, station, p, state)
                return self._insist(session, station, p)
            # The callsign was heard, so somebody is on frequency and has
            # called: that is two-way contact, whatever the rest was. Without
            # it an unreadable first call ("descending flight level eight
            # zero" before it was parsed) never counted, and every arrival
            # call that waits for a check-in waited for ever.
            if parsed.callsign_matched:
                session.establish(station)
                return Reply(p.say_again(aircraft), station)
            return self._not_understood(session, station, p, state)

        # "Final, runway two four" carries a runway and no request verb, so it
        # parses as a readback -- and with nothing waiting to be read back it
        # was answered "roger" on every frequency, the tower's included.
        # With nothing pending, a call that names a leg is where the
        # aeroplane is.
        if (parsed.intent is Intent.READBACK and session.pending is None
                and parsed.get("pattern_leg")
                and not _REPORT_WORD.search(parsed.normalized)):
            parsed.intent = Intent.POSITION_REPORT

        # A readback is checked before anything else, because an incorrect one
        # has to be corrected before the flight moves on.
        if parsed.intent is Intent.READBACK:
            if session.pending is not None:
                return self._check_readback(parsed, session, station, p, state)
            # Reading back something we never issued: acknowledge rather than
            # asking them to say again, which would be a pointless loop.
            #
            # Except that some of those are not readbacks at all. "Request
            # runway three zero" carries a runway and no request verb the
            # rules know, so it lands here -- and a bare "roger" to it is
            # indistinguishable from not having heard it. Nothing is pending,
            # so nothing here can be a readback of anything; a transmission
            # that asks for a runway in as many words is asking for one.
            session.establish(station)
            if _ASKS_FOR_RUNWAY.search(parsed.normalized):
                self._note_runway(parsed, session, station, asked=True)
                runway = (session.departure_runway if session.phase.outbound
                          else session.arrival_runway)
                if runway:
                    return Reply(p.roger_runway(aircraft, runway), station)
            return Reply(p.roger(aircraft), station)

        # An outstanding readback is settled before any new request is worked.
        # A pilot who has been told to hold short of a runway and answers with
        # "request taxi" has not been told to hold short in any sense that
        # matters, and a controller does not move on until they have said it.
        #
        # This comes before "say again" on purpose. German uses "wiederholen
        # Sie" for both "read back" and "say again", so a pilot echoing a
        # correction was heard as asking the controller to repeat itself and
        # the exchange could not be ended. A transmission carrying the values
        # a readback is waiting for is a readback, whatever else those words
        # could have meant.
        insisted = self._settle_pending(parsed, session, station, p, state)
        if insisted is not None:
            session.establish(station)
            return insisted

        # "Say again" from the pilot asks this controller to repeat itself,
        # which it can do exactly, because it wrote down what it said. What it
        # cannot do is repeat somebody else's transmission: the last
        # instruction of the flight may have come from a frequency the
        # aeroplane has left, and reading it out here would be saying again
        # something that was never said on this one.
        if parsed.intent is Intent.SAY_AGAIN:
            here = station_key(station)
            last = session.pending or session.last_instruction
            if last is not None and last.text and last.station_key == here:
                return Reply(self._repeat(last, station, p, aircraft),
                             station)
            return self._not_understood(session, station, p, state)

        # A runway the pilot asked for, before any handler decides which one
        # they are getting. After the readback handling above, so reading back
        # the runway you were just given is never mistaken for asking for a
        # different one.
        self._note_runway(parsed, session, station)

        # The answer to a traffic call. In sight is the end of it -- and for a
        # VFR flight that is all it is: see-and-avoid is its own business, so
        # it is not told to maintain visual separation, which is what an IFR
        # flight is given (FAA JO 7110.65 7-2-1). Looking, or not seeing it,
        # is acknowledged, and the watch says when it is no factor.
        if parsed.intent in (Intent.REPORT_TRAFFIC_IN_SIGHT,
                             Intent.REPORT_TRAFFIC_LOOKING,
                             Intent.REPORT_TRAFFIC_NOT_IN_SIGHT):
            state_word = {Intent.REPORT_TRAFFIC_IN_SIGHT: "seen",
                          Intent.REPORT_TRAFFIC_LOOKING: "looking",
                          Intent.REPORT_TRAFFIC_NOT_IN_SIGHT: "negative"}[parsed.intent]
            # The answer names no traffic, so it belongs to the call still
            # open -- the most recent one: two called contacts are not both
            # in sight because one report said so.
            open_calls = [name for name, said
                          in session.traffic_called.items()
                          if said in ("called", "looking", "negative")]
            if open_calls:
                session.traffic_called[open_calls[-1]] = state_word
            if parsed.intent is Intent.REPORT_TRAFFIC_IN_SIGHT and session.ifr:
                session.establish(station)
                return Reply(p.maintain_visual_separation(aircraft), station)
            if session.traffic_called or not session.ifr:
                session.establish(station)
                return Reply(p.roger(aircraft), station)

        # A VFR flight plan is flight service's to close. A controller asked
        # to close one passes it on; an information service closes it itself
        # (afis), so it is not answered here.
        if (parsed.intent is Intent.CLOSE_FLIGHT_PLAN
                and station.position not in ("INFO", "CTAF")):
            session.establish(station)
            if not session.flight_plan_open:
                return Reply(p.roger(aircraft), station)
            session.flight_plan_open = False
            decision("flight_plan", station=station.callsign, said="relayed_close")
            return Reply(p.flight_plan_relayed(aircraft), station)

        # The departure over the point it was released by, or saying it is
        # leaving: the last thing the tower wanted from it, so it is let go.
        if (session.expected_exit and station.position == "TWR"
                and self.realism.reporting_points and _airborne(state)
                and (parsed.intent is Intent.REPORT_LEAVING
                     or self._mentions(parsed, session.expected_exit, p))):
            decision("vrp", airport=station.ident, use="exit_reported",
                     point=session.expected_exit)
            session.expected_exit = ""
            # Let go: the level it was kept under went with the zone.
            session.vfr_ceiling_ft = 0.0
            session.establish(station)
            return Reply(p.frequency_change_approved(aircraft), station)

        # A full stop ends the circuits. Asking for them set the flag and
        # nothing ever cleared it, so "downwind, full stop" was answered on
        # final with "cleared for the option" and the tower never cleared the
        # aeroplane to land of its own accord.
        if session.pattern_work and parsed.get("full_stop"):
            session.pattern_work = False
            decision("circuits_ended", airport=station.ident, why="pilot",
                     said=parsed.normalized)

        handler = {
            "DEL": self._delivery,
            "GND": self._ground,
            "TWR": self._tower,
            "DEP": self._radar,
            "APP": self._radar,
            "CTR": self._radar,
            "CTAF": self._ctaf,
            "INFO": self._info,
        }.get(station.position)

        if handler is None:
            return Reply("", station, silent=True)
        if parsed.intent is Intent.REPORT_RUNWAY_VACATED:
            session.occupying_runway = ""
            destination = session.destination
            if (self.realism.tower_continue and station.position == "GND"
                    and destination is not None
                    and station.ident == destination.ident
                    and session.phase in (Phase.LANDING, Phase.LANDED,
                                          Phase.TAXI_IN)):
                # An arrival off the runway on Ground's frequency wants its
                # stand: given the taxi in, where "roger" left it sitting
                # on the taxiway with nothing to do (the IFR plan, 5 (e)).
                decision("ground", event="taxi in on vacating",
                         station=station.callsign)
                parsed.intent = Intent.REQUEST_TAXI_TO_PARKING
            else:
                # Acknowledged by whoever is listening: a report, not a
                # request.
                session.establish(station)
                return Reply(p.roger(session.aircraft), station)
        # Special VFR is the zone's: whoever releases or joins a VFR flight
        # there gives it.
        if (parsed.intent is Intent.REQUEST_SPECIAL_VFR
                and (station.position in ("DEL", "GND", "TWR")
                     or _airborne(state))):
            # On a radar position the request can only come from the
            # air, and the airborne half of _special_vfr is the join
            # into the zone -- a ground position answers the out half.
            handler = self._special_vfr

        # "Affirm"/"negative" answers an open offer of assistance on any
        # frequency but ground's, where the taxi offer is answered with the
        # new route itself.
        if session.assistance_offered("flight"):
            if parsed.intent is Intent.AFFIRMATIVE:
                session.answer_assistance()
                return Reply(p.roger(session.aircraft), station)
            if parsed.intent is Intent.NEGATIVE:
                session.answer_assistance()
                return Reply(p.roger(session.aircraft), station)

        # The pressure setting can be asked of whoever is on the
        # frequency: it is not an instruction, and the answer is the
        # weather, not the position.
        if parsed.intent is Intent.REQUEST_ALTIMETER:
            value = (weather.altimeter_inhg if p.dialect == "faa"
                     else weather.qnh_hpa)
            session.establish(station)
            decision("request", asked="altimeter", value=value,
                     station=station.callsign)
            return Reply(p.altimeter_setting(session.aircraft, value),
                         station)

        # Two-way contact is only established *after* this transmission has
        # been handled: the reply to a first call uses the full callsign and,
        # for a radar position, opens with "radar contact".
        self._circuit_logged = False
        reply = handler(parsed, session, station, weather, p, state)
        if reply.speak and not self._circuit_logged:
            self.note_circuit(session, station, state,
                              reply.instruction.kind if reply.instruction
                              else parsed.intent.value)
        if reply.speak:
            told = self._old_information(parsed, session, station, p)
            if told:
                reply.text = f"{reply.text} {told[0].upper()}{told[1:]}"
        session.establish(station)
        if reply.instruction is not None and not reply.instruction.from_intent:
            reply.instruction.from_intent = parsed.intent.value
        return reply

    # Set by the engine: ``(ident, language) -> AtisReport | None``, the
    # broadcast the field is putting out now. Unset, an ATIS letter in a call
    # is not checked against anything, which is what tests and a field
    # without an engine get.
    atis = None

    def _old_information(self, parsed, session, station, p) -> str:
        """The current ATIS letter, when the pilot reported another.

        The letter in a first call was parsed, kept, and never compared with
        anything, so a pilot holding a broadcast an hour old was worked on its
        runway and its pressure setting without a word.
        """
        heard = str(parsed.get("atis_letter") or "").strip().upper()
        if not heard or self.atis is None:
            return ""
        if station.position in ("APP", "DEP", "CTR"):
            field = session.destination
        else:
            field = self.navdb.airport(station.ident)
        # Only where there is a broadcast to have heard. A letter reported at
        # a field without one is not wrong; it is the pilot's own business.
        if field is None or self.navdb.station_for(field.ident, "ATIS") is None:
            return ""
        try:
            report = self.atis(field.ident, language_of(p))
        except Exception:
            log.debug("could not read the ATIS for %s", field.ident,
                      exc_info=True)
            return ""
        if report is None or not report.letter or report.letter.upper() == heard:
            return ""
        weather = report.weather
        pressure = (weather.altimeter_inhg if p.dialect == "faa"
                    else weather.qnh_hpa)
        decision("atis_letter_old", airport=field.ident, heard=heard,
                 current=report.letter)
        return p.information_current(report.letter, pressure)

