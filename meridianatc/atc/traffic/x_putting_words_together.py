from __future__ import annotations

from ._shared import (
    READBACK_GAP_S,
    Call,
    Flight,
    PilotSpeech,
    Station,
    Weather,
    log,
    phraseology_for,
)


class _Putting_words_together:
    # ------------------------------------------------------------------
    # putting words together
    # ------------------------------------------------------------------

    @property
    def uncontrolled(self) -> bool:
        """Whether this field has no controller: worked by nobody, on its
        traffic frequency, or by an information service, which gives
        information and clears nobody."""
        return not self.field.stations and (
            self.field.advisory is not None or self.field.afis is not None)

    @property
    def _announcing_on(self) -> str:
        """The position an uncontrolled field's traffic is heard on."""
        return "INFO" if self.field.afis is not None else "CTAF"

    def _aerodrome_information(self, ident: str) -> Station | None:
        from .. import afis

        for station in self.navdb.stations(ident):
            if (station.position == "INFO" and station.mhz
                    and afis.kind_of(self.navdb, station) == afis.AERODROME):
                return station
        return None


    def _exchange(self, flight: Flight, now: float, position: str, *,
                  pilot_first=None, controller=None, pilot_back=None,
                  announce=None, command=None, urgent: bool = False) -> None:
        """Queue a complete exchange, spaced the way a real one sounds.

        At an uncontrolled field there is no exchange. There is one aeroplane
        saying what it is about to do, and everybody else listening -- so the
        controller's half is dropped, and so is the readback of it, which is a
        readback of something nobody said.

        ``announce`` is what to say instead, for the exchanges where neither
        half of the real one works on its own: reading back a takeoff
        clearance is not an announcement a pilot could make at a field where
        nobody issued one.

        ``command`` is ``(kind, values)``: what the aeroplane does about it,
        timed to the end of the readback rather than to the start of the
        instruction. That gap is what makes the traffic look like it is
        listening -- you hear "go around", you hear it acknowledged, and then
        the nose comes up.

        ``urgent`` is for the instructions that cannot wait behind a queue of
        pleasantries. A go-around is one of them: it goes to the front, and
        the aeroplane acts on it whether or not the frequency was clear enough
        for the words to be heard.
        """
        if self.uncontrolled and self.field.afis is not None:
            self._afis_exchange(flight, now, pilot_first=pilot_first,
                                announce=announce, command=command)
            return
        if self.uncontrolled:
            say = announce or pilot_first or pilot_back
            if say is not None:
                self._queue(flight, now, "CTAF", True, say)
            if command is not None:
                # Nobody said it, but the aeroplane is still flying: at a field
                # with no tower a pilot who has to go around goes around.
                self._tell(flight, command[0], now + 1.0, **command[1])
            return

        if getattr(self, "radio_silence", False) and not urgent:
            # Somebody has declared mayday: the frequency stays clear for
            # them. The aeroplane still does what it was told -- the
            # words are what stay unsaid.
            if command is not None:
                self._tell(flight, command[0], now + 1.0, **command[1])
            return

        self._exchanges += 1
        mark = self._exchanges
        at = now
        halves: list[Call | None] = []
        if pilot_first is not None:
            halves.append(self._queue(flight, at, position, True, pilot_first,
                                      exchange=mark))
            at += READBACK_GAP_S + self.random.uniform(0.4, 2.0)
        if controller is not None:
            halves.append(self._queue(flight, at, position, False, controller,
                                      urgent=urgent, exchange=mark))
            at += READBACK_GAP_S + self.random.uniform(0.2, 1.2)
        if pilot_back is not None:
            halves.append(self._queue(flight, at, position, True, pilot_back,
                                      urgent=urgent, exchange=mark))

        # All of it or none of it. A half that would not compose -- a
        # phrasebook with no wording for this in this language, or a value the
        # sentence needed and did not have -- used to be dropped on its own,
        # which left an aeroplane calling a controller who never answered, or
        # a controller answering nobody. The aeroplane still does what it was
        # told: it is the words that failed, not the clearance.
        if any(half is None for half in halves):
            self.pending = [c for c in self.pending if c.exchange != mark]

        if command is not None:
            self._tell(flight, command[0], at, **command[1])

    def _afis_exchange(self, flight: Flight, now: float, *, pilot_first=None,
                       announce=None, command=None) -> None:
        """An exchange with an AFIS: the pilot calls up and is given the
        field (runway in use, wind, QNH, traffic), or calls ready or on final
        and is told the runway is free, the wind, and "at your discretion";
        the pilot acknowledges. Nothing the pilot would only have said back
        to a controller -- a readback of a takeoff clearance, "line up and
        wait" -- is said at all. UNVERIFIED: the acknowledgement as a plain
        "roger".
        """
        kind = command[0] if command is not None else ""
        runway = (command[1].get("runway") if command is not None else "") \
            or flight.runway
        weather = self.field.weather or Weather()
        if kind == "takeoff":
            def opening(p, a, s):
                return p.ready_for_departure(a, runway, station=s)
            def answer(c, a, s):
                return c.afis_takeoff(a, weather)
        elif kind in ("land", "touch_and_go"):
            opening = pilot_first or announce
            def answer(c, a, s):
                return c.afis_land(a, weather)
        elif kind in ("taxi", "pattern") and pilot_first is not None:
            opening = pilot_first
            def answer(c, a, s):
                return c.afis_field(a, s, runway, weather)
        elif kind == "line_up":
            opening = answer = None
        else:
            opening, answer = announce or pilot_first, None
        at = now
        if opening is not None and answer is not None:
            self._exchanges += 1
            mark = self._exchanges
            halves = [self._queue(flight, at, "INFO", True, opening,
                                  exchange=mark)]
            at += READBACK_GAP_S + self.random.uniform(0.4, 2.0)
            halves.append(self._queue(flight, at, "INFO", False, answer,
                                      exchange=mark))
            at += READBACK_GAP_S + self.random.uniform(0.2, 1.2)
            halves.append(self._queue(flight, at, "INFO", True,
                                      lambda p, a, s: p.roger(a),
                                      exchange=mark))
            if any(half is None for half in halves):
                self.pending = [c for c in self.pending if c.exchange != mark]
        elif opening is not None:
            self._queue(flight, now, "INFO", True, opening)
        if command is not None:
            self._tell(flight, command[0], max(at, now + 1.0), **command[1])

    def _say(self, flight: Flight, now: float, position: str, *,
             from_pilot: bool, make) -> None:
        if self.uncontrolled:
            if not from_pilot:
                return
            position = self._announcing_on
        self._queue(flight, now, position, from_pilot, make)

    def _queue(self, flight: Flight, at: float, position: str,
               from_pilot: bool, make, urgent: bool = False,
               exchange: int = 0) -> Call | None:
        """Compose one transmission and queue it. None if there were no words.

        The return value is what lets :meth:`_exchange` refuse to say half of
        something: a sentence that will not compose is a silence, and a
        silence in the middle of an exchange is heard as a lost transmission.
        """
        station = self.field.stations.get(position) or (
            self.field.advisory if position == "CTAF"
            else self.field.afis if position == "INFO" else None)
        station_name = station.callsign_in(flight.language) if station else ""
        speaker = (flight.aircraft.for_language(flight.language).spoken_short
                   if from_pilot else station_name)
        try:
            if from_pilot:
                speaker_book = self._pilot(flight.language)
                speaker_book.greet = position not in flight.greeted
                text = make(speaker_book, flight.aircraft, station_name)
                flight.greeted.add(position)
            else:
                text = make(self._phrase(flight.language),
                            flight.aircraft, station_name)
        except Exception:
            log.debug("could not compose a %s transmission for %s",
                      position, flight.aircraft.callsign.written,
                      exc_info=True)
            return None
        if not text:
            return None
        call = Call(
            text=text, language=flight.language, from_pilot=from_pilot,
            position=position, speaker=speaker or station_name,
            flight=flight, at=at, urgent=urgent, exchange=exchange,
        )
        self.pending.append(call)
        return call

    def _phrase(self, language: str):
        key = (language, self.profile.region)
        found = self._phraseology.get(key)
        if found is None:
            found = phraseology_for(language, self.field.ident,
                                    region=self.profile.region)
            self._phraseology[key] = found
        return found

    def _pilot(self, language: str) -> PilotSpeech:
        found = self._pilots.get(language)
        if found is None:
            found = PilotSpeech(language, self._phrase(language))
            self._pilots[language] = found
        return found

