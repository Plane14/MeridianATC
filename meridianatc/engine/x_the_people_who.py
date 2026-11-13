from __future__ import annotations

from ._shared import (
    RADIO,
    Intent,
    ParsedIntent,
    Station,
    by_words,
    clearly_in,
    gsx,
    interphone,
    language_of_text,
    languages_for,
    log,
    station_key,
    timed,
)


class _The_people_who:
    # ------------------------------------------------------------------
    # the people who are not on the radio
    # ------------------------------------------------------------------

    def _read(self, text: str, tagged: str | None, session,
              station: Station) -> tuple[ParsedIntent, str]:
        """Read one transmission, typed or spoken, the same way.

        Once a frequency is worked in a language it is read in that one,
        unless the words are in another over several of them. The
        recogniser's tag does not move it: it is a guess from the sound, and
        at Kortrijk it called an English taxi call French, which the French
        grammar made nothing of -- "say again" to a sentence that typed got
        "roger".

        On a frequency not worked yet the words decide when they say
        anything (:func:`detect.by_words`), and the tag only when they do
        not. At Heubach "Heubach Tower, DEHUH, request radio check" was
        tagged German, from the sound of the field's name, and answered in
        German.

        Where that reading makes nothing of it, it is read in the field's
        other languages, and the first that understands it is the language
        it was in: answered in it, and the frequency goes over to it
        (:meth:`_frequency_language`), save for the one-word replies
        ("répétez", "affirm") that never move it. That is how a pilot
        changes language mid-session. At Heubach, before it, a frequency
        answered once in German heard every English call after as nothing,
        and nothing could bring it back.

        Returns what was understood and the language it was read in.
        """
        ident = station.ident
        established = self._languages_by_frequency.get(station_key(station))
        worked = [self._language_for(ident, lang)
                  for lang in languages_for(ident)]
        words = self._language_for(
            ident, language_of_text(text, languages_for(ident)))
        tag = self._language_for(ident, tagged) if tagged else ""
        if established:
            first = (words if clearly_in(text, words, established)
                     else established)
        else:
            decided = by_words(text, worked)
            first = (self._language_for(ident, decided) if decided
                     else tag or words)
        others = [words, tag, established or self._language_for(ident),
                  *worked]
        context = self._situation(session, station)
        with timed("understanding"):
            parsed = self.understanding.parse(text, first, context=context)
        for other in dict.fromkeys(o for o in others if o and o != first):
            if parsed.intent is not Intent.UNKNOWN:
                break
            with timed("understanding"):
                again = self.understanding.parse(text, other, context=context)
            if again.intent is not Intent.UNKNOWN:
                return again, other
        return parsed, first

    # What one word in another language does not change: these are answered
    # in the frequency's language. A radio check is not on it: it is the
    # call a pilot opens with, and at Heubach the one an English pilot made
    # on a frequency stuck in German.
    _NOT_A_CHANGE_OF_LANGUAGE = frozenset({
        Intent.SAY_AGAIN, Intent.ACKNOWLEDGE, Intent.AFFIRMATIVE,
        Intent.NEGATIVE, Intent.STANDBY, Intent.UNKNOWN,
    })

    def _frequency_language(self, station: Station, heard_in: str,
                            parsed) -> str:
        """The language to answer in on this frequency."""
        key = station_key(station)
        established = self._languages_by_frequency.get(key)
        if (established and heard_in != established
                and parsed.intent in self._NOT_A_CHANGE_OF_LANGUAGE):
            return established
        self._languages_by_frequency[key] = heard_in
        return heard_in

    def answer_the_aeroplane(self, text: str, language: str | None) -> bool:
        """Answer a transmission that was addressed to somebody on board.

        True when it was, and when it was, nothing goes out over the air and
        the controller never hears it. See
        :mod:`meridianatc.atc.interphone` for what counts as an address and why
        it has to be one.
        """
        settings = self.config.immersion
        if not (settings.enabled and settings.interphone):
            return False
        session = self.session
        # A bare checklist answer is the first officer's only while they
        # are waiting for one, no readback is owed to the controller, and
        # they spoke after the controller did.
        expecting = (self.crew.waiting_for_the_pilot()
                     and (session is None or session.pending is None)
                     and self.crew.spoke_after(self._atc_said_at))
        try:
            said = interphone.parse(
                text, self.crew.language, names=self.crew.names(),
                first_officer=self.crew.first_officer_names(),
                checklists=self.crew.checklist_titles(),
                expecting=expecting, offered=bool(self.crew.offering()))
        except Exception:
            log.debug("could not read the interphone", exc_info=True)
            return False
        if said is None and expecting:
            said = self._checklist_answer_from_the_model(text)
        if said is None:
            return False

        # Written to the transcript under the names the transcript already
        # reads: who was talking, and what the machine made of it. A row
        # nobody can read is the same as no row.
        RADIO.write("interphone", text=text, speaker=said.who,
                    intent=said.what, addressed=said.addressed,
                    language=self.crew.language)
        if said.who == interphone.GROUND:
            self.ask_the_ground(said)
        else:
            self.crew.ask(said)
        # True whether or not anybody answered. A pilot who addressed their
        # cabin and has the cabin switched off gets silence and a row in the
        # crew log saying which switch it was; what they must not get is the
        # controller being handed a drinks order and asking them to say
        # again. The whole of the decision was made by the address.
        return True

    def _checklist_answer_from_the_model(self, text: str):
        """The understanding layer's model, asked whether a transmission
        the rules made nothing of is a checklist answer. None for no model,
        or for "not an answer", which is the controller's."""
        try:
            kind = self.understanding.checklist_answer(text)
        except Exception:
            log.debug("the model could not read a checklist answer",
                      exc_info=True)
            return None
        if not kind:
            return None
        return interphone.Said(who=interphone.FIRST_OFFICER,
                               what=interphone.ANSWER, text=text,
                               language=self.crew.language,
                               answers=(kind,))

    def _read_now(self, var: str):
        """One simulator variable, read now, or None (see
        SimConnectSource.read_now)."""
        reader = getattr(self.source, "read_now", None)
        if not callable(reader):
            return None
        return reader(var)

    def ask_the_ground(self, said) -> bool:
        """Ask the ground handling for whatever the pilot just asked for.

        The words are the crew's and the doing is GSX's, and the two are kept
        apart here: the ground crew acknowledge what was actually asked for,
        and where nothing could be asked -- no bridge, no GSX, or a switch
        turned off -- they say so instead of promising a truck that is never
        coming.
        """
        settings = self.config.gsx
        answer = ""
        wanted = said.what in gsx.SERVICES
        if wanted and not (settings.enabled and settings.control
                           and settings.voice):
            answer = "ground_none"
        elif wanted:
            try:
                notice = self.gsx.request(said.what)
            except Exception:
                log.debug("could not ask the ground handling", exc_info=True)
                notice = ""
            if notice:
                self.emit("info", notice, key="log.ground_handling",
                          detail=notice)
            else:
                answer = "ground_none"
        return self.crew.ask(said, answer=answer)

