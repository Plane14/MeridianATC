from __future__ import annotations

from ._shared import (
    LANGUAGE_NAMES,
    RADIO,
    Intent,
    Reply,
    SpeechModelMissing,
    as_written,
    decision,
    letters,
    log,
    replace,
    threading,
    time,
    timed,
)


class _Transmission:
    # ------------------------------------------------------------------
    # transmission
    # ------------------------------------------------------------------

    def begin_transmission(self) -> None:
        if self.microphone.recording:
            return
        try:
            self.microphone.start()
            # A streamed recogniser (Deepgram over a websocket) hears the
            # transmission as it is captured, so its answer is already back
            # when the button comes up rather than starting its journey then.
            # Anything without a stream to offer -- the local model -- leaves
            # the tap None and nothing about the capture changes.
            try:
                # The stream's keyterms are fixed when its socket opens, so
                # the context refresh runs here as well as before the
                # transcript: a station changed between calls belongs in it.
                self._tell_the_recogniser()
                starter = getattr(self.recognizer, "start_stream", None)
                self.microphone.on_chunk = (
                    starter(self.microphone.rate) if starter else None)
            except Exception:
                log.debug("could not start streamed recognition",
                          exc_info=True)
                self.microphone.on_chunk = None
            self.emit("info", "Transmitting...", key="log.transmitting")
        except Exception as exc:
            # Logged as well as shown. A panel line is gone as soon as the
            # next one arrives, and this is the failure that then explains
            # everything else that happens for the rest of the session.
            log.warning("could not open the microphone: %s", exc, exc_info=True)
            self.emit("error", f"Could not open the microphone: {exc}",
                       key="log.mic_failed", detail=exc)

    # The longest a transmission may run before the program stops believing
    # in it. Well past the configured limit on a transmission's length, so
    # this only ever fires on a transmit state that is stuck rather than on a
    # pilot who is being thorough.
    STUCK_TRANSMIT_FACTOR = 2.0
    STUCK_TRANSMIT_MIN_S = 60.0

    def _watch_transmission(self) -> None:
        """End a transmission that has plainly stopped being one.

        A transmit state that never ends is the most expensive stuck flag in
        the program, because so much waits on it politely. The cabin crew and
        the first officer will not speak over the frequency, and the
        controller volunteers nothing while the pilot has the bar down -- so a
        microphone that is wedged open does not announce itself, it just
        quietly removes half the aeroplane and every unprompted call, for the
        rest of the flight, with the radio still answering normally when
        spoken to. That is unfalsifiable from the flight deck.

        The causes are real and none of them are the pilot: a Bluetooth
        headset changing profile under the stream, a device unplugged
        mid-transmission, a joystick that reported the button down and then
        stopped reporting anything. So it is bounded rather than trusted.
        """
        limit = max(self.STUCK_TRANSMIT_MIN_S,
                    self.config.audio.max_transmission_s
                    * self.STUCK_TRANSMIT_FACTOR)
        held = self.microphone.recording_for
        if held < limit:
            return
        log.warning("the transmit button has been down for %.0fs; ending the "
                    "transmission", held, extra={"held_s": round(held, 1)})
        self.emit("error",
                  f"The transmit button has been down for {held:.0f} seconds, "
                  f"so the transmission was ended. Nothing else could be said "
                  f"while it was.",
                  key="log.transmit_stuck", seconds=f"{held:.0f}")
        try:
            self.end_transmission()
        except Exception:
            log.debug("could not end the stuck transmission", exc_info=True)
        # Whatever happened above, the state must not survive it: everything
        # this method exists to protect is waiting on that one flag.
        if self.microphone.recording:
            self.microphone.abandon()
        # A stream opened for this transmission is abandoned with it: nothing
        # will ever ask for its transcript, and a socket left open still
        # hears the rest of the flight.
        abort = getattr(self.recognizer, "abort_stream", None)
        if abort is not None:
            try:
                abort()
            except Exception:
                log.debug("could not abandon the recognition stream",
                          exc_info=True)

    def _drain_plugin_commands(self) -> None:
        """The X-Plane plugin's key bindings, sent up its UDP channel.

        The traffic renderer answers datagrams on the same socket the
        snapshots go out on, so a command is a JSON packet arriving there:
        transmit_com1/2 pick the radio the pilot is working, the talk and
        push-to-talk bindings drive the microphone path directly, and
        copy_error puts the last error where it can be pasted.
        """
        traffic = getattr(self, "simtraffic", None)
        if traffic is None or not getattr(traffic, "enabled", False):
            return
        try:
            getter = getattr(traffic, "command", None)
            while getter is not None:
                command = getter()
                if command is None:
                    break
                self._plugin_command(command)
        except Exception:
            log.debug("plugin command handling raised", exc_info=True)

    def _plugin_command(self, command: dict) -> None:
        cmd = str(command.get("cmd") or "")
        if cmd in ("transmit_com1", "transmit_com2"):
            self._radio_talk = 1 if cmd.endswith("1") else 2
            self.emit("info", f"Transmitting on COM{self._radio_talk}.",
                      key=f"log.com{self._radio_talk}_tx")
            return
        if cmd in ("mic_push_to_talk", "talk", "talk_copilot"):
            if command.get("down", True):
                self._talk_speaker = ("first_officer"
                                      if cmd == "talk_copilot" else "")
                self.begin_transmission()
            else:
                self.end_transmission()
            return
        if cmd == "copy_error":
            self._copy_last_error()
            return
        log.debug("unknown plugin command %r", cmd)

    def _copy_last_error(self) -> None:
        text = self._last_error or "No error has been reported."
        copied = False
        try:
            import tkinter
            root = tkinter.Tk()
            root.withdraw()
            root.clipboard_clear()
            root.clipboard_append(text)
            root.update()
            root.destroy()
            copied = True
        except Exception:
            log.debug("clipboard copy failed", exc_info=True)
        self.emit(
            "info",
            ("Last error copied to clipboard: " if copied
             else "The last error was: ") + text,
            key="log.copy_error")

    def end_transmission(self) -> None:
        if not self.microphone.recording:
            return
        captured = self.microphone.stop()
        if captured.too_quiet:
            self.emit("info", "Nothing heard on that transmission.",
                      key="log.nothing_heard")
            return
        threading.Thread(
            target=self._handle_transmission, args=(captured,), daemon=True
        ).start()

    def _handle_transmission(self, captured) -> None:
        if not self._busy.acquire(blocking=False):
            # This transmission is dropped whole. A stream opened for it has
            # no transcribe coming to collect its finals, so it is ended here
            # rather than left to its own lifetime bound.
            abort = getattr(self.recognizer, "abort_stream", None)
            if abort is not None:
                try:
                    abort()
                except Exception:
                    log.debug("could not abandon the recognition stream",
                              exc_info=True)
            self.emit("info", "Still working on the last transmission.",
                      key="log.still_working")
            return
        try:
            self._tell_the_recogniser()
            with timed("recognition", warn_over_ms=4000):
                transcript = self.recognizer.transcribe(
                    captured.audio, captured.sample_rate)
            # The trainer's "when you heard X, I meant Y" pairs apply to the
            # raw text before anything else reads it.
            corrected = self.trainer.apply(transcript.text)
            if corrected != transcript.text:
                transcript = replace(transcript, text=corrected)
            if transcript.empty:
                self.emit("info", "Transmission was unreadable.",
                          key="log.unreadable")
                RADIO.write("pilot", text="", heard=False,
                            seconds=round(captured.seconds, 2)
                            if hasattr(captured, "seconds") else None)
                return

            # "spoken": the panel empties its box, so a draft left in it is
            # not sent again on the end of the next typed call.
            self.emit("pilot", transcript.text, action="spoken",
                      radio=f"com{self._radio_talk}")
            if transcript.confidence < self.config.speech.min_confidence:
                log.info("low confidence %.2f: %s", transcript.confidence,
                         transcript.text,
                         extra={"confidence": round(transcript.confidence, 3),
                                "heard": transcript.text})

            speaker = self._talk_speaker
            self._talk_speaker = ""
            self.handle_text(transcript.text, transcript.language,
                             speaker=speaker,
                             confidence=transcript.confidence)
        except SpeechModelMissing:
            self.emit("error", "The speech recognition has not been downloaded "
                      "yet. Open Settings, then Components, and press Download.",
                      key="log.speech_missing", action="setup")
        except Exception as exc:
            log.exception("transmission handling failed")
            self.emit("error", f"Error handling transmission: {exc}",
                       key="log.transmission_failed", detail=exc)
        finally:
            self._busy.release()

    # How far a station may be and still be one the pilot might call.
    RECOGNISER_STATIONS_NM = 40.0

    def _tell_the_recogniser(self) -> None:
        """The pilot's own callsign and the stations in reach, as hotwords.

        Measured on sixty pilot calls (scripts/stt_vocabulary_bench.py) they
        bring the callsign back whole in 98% of transmissions against 38%:
        "Oscar Oscar Alpha Bravo Shirley" was a callsign the recogniser had
        not been told about.
        """
        try:
            session = self.session
            callsign = (session.aircraft.callsign.spoken_full
                        if session is not None else "")
            names = []
            if self.station is not None:
                names.append(self.station.callsign)
            state = self.state
            if state.has_position:
                for one in self.navdb.all_stations_near(
                        state.latitude, state.longitude,
                        self.RECOGNISER_STATIONS_NM):
                    if one.position != "ATIS":          # never called
                        names.append(one.callsign)
            # The nearest first; a dozen is all the budget is worth.
            names = list(dict.fromkeys(names))[:12]
            self.recognizer.set_context(callsign, names)
        except Exception:
            log.debug("could not give the recogniser its context",
                      exc_info=True)

    def _answer_from_traffic(self, session, station, parsed) -> bool:
        """Let a called crew answer the pilot. False if nobody was called."""
        if self.traffic is None:
            return False
        try:
            return self.traffic.answer_the_user(
                session, parsed, station.position,
                humor=self.config.traffic.humor)
        except Exception:
            log.debug("a crew answering the user failed", exc_info=True)
            return False

    def handle_text(self, text: str, language: str | None = None,
                    speaker: str = "", confidence: float = 1.0) -> Reply | None:
        """Process one pilot transmission that has already been transcribed.

        Exposed separately from the audio path so the whole ATC stack can be
        driven from typed input, which is how the tests exercise it.
        ``language`` is what the pilot spoke; the controller answers in the
        same one when the facility works it. ``speaker`` is "first_officer"
        for a call the pilot handed over, written to the radio log so the
        transcript says who spoke. ``confidence`` is the recogniser's own
        (1.0 for typed text): a name heard nearly is asked about, not
        refused, when the recogniser was unsure of it.
        """
        # The recogniser's spacing is not the keyboard's; the same words are
        # the same transmission.
        text = " ".join((text or "").split())
        if self.session is None:
            self.begin_session()
        session = self.session
        assert session is not None

        self.refresh_state()

        # Somebody in the aeroplane, rather than somebody on the frequency.
        # Before the station is resolved on purpose: a pilot can ask their
        # cabin crew for a coffee at a field with nothing published on the
        # box, and being told that nobody is listening would be wrong twice.
        if speaker != "first_officer":
            if self.answer_the_aeroplane(text, language):
                return None
            # A go-around reported to the tower is also the call on the
            # flight deck; the transmission still goes to the tower.
            try:
                self.crew.heard_on_the_radio(text)
            except Exception:
                log.debug("the first officer raised", exc_info=True)

        # A dead radio sends nothing. Said rather than silently ignored: a
        # pilot who keys a switched-off aeroplane should know why nobody
        # answers, and that it is the aeroplane and not the program.
        if not self.radio_powered:
            self.emit("info", "The radio has no power: nothing was "
                      "transmitted. Switch on the battery and the avionics.",
                      key="log.radio_no_power")
            return None

        # The radio the pilot is keyed on decides who hears it: COM1 the
        # tuned station, COM2 its own. Falling back keeps a dead COM2 from
        # silencing a call.
        station = ((self.station2 or self.station)
                   if self._radio_talk == 2 else self.station) \
            or self._current_station()
        if station is None:
            self.emit("info", "No station is listening on this frequency.",
                      key="log.nobody_listening")
            return None

        tagged = language
        # A letter heard as a word ("Bravo, Shirley") is read as the letter
        # where a letter belongs; the log keeps what was heard.
        heard = text
        text, _ = letters.snap(text)
        parsed, heard_in = self._read(text, tagged, session, station)
        parsed.heard_confidence = float(confidence)
        if not parsed.callsign_matched and self._take_the_callsign_heard(
                session, text):
            parsed.callsign_matched = True
        # The frequency keeps the language it is being worked in. Read in
        # whatever the recogniser heard, so "répétez" on an English frequency
        # is still a request to repeat -- but answered in English, because
        # one word in another language is not a change of language. Only a
        # transmission of substance in the other language moves it.
        spoken = self._frequency_language(station, heard_in, parsed)
        parsed.language = spoken
        if spoken != self.last_language:
            self.last_language = spoken
            if len(self.languages) > 1:
                self.emit("info", f"Speaking {LANGUAGE_NAMES.get(spoken, spoken)}.",
                          key="log.speaking", language=spoken)

        # The transcript records the reading as well as the words. "Why did it
        # say that" is a question about the intent and the numbers, and neither
        # of them survives in a log of the sentence alone.
        pending = session.pending
        RADIO.write(
            "pilot", text=text, language=spoken,
            speaker=speaker or "pilot",
            # The recogniser's own guess, for a spoken one: the language it
            # was read in is the frequency's unless the words said otherwise.
            **({"heard_as": tagged} if tagged else {}),
            **({"heard": heard} if heard != text else {}),
            station=station.callsign, frequency=round(station.mhz, 3),
            intent=parsed.intent.value, confidence=round(parsed.confidence, 3),
            callsign_matched=parsed.callsign_matched,
            values=dict(parsed.values), phase=session.phase.value,
            owed=pending.owed_items if pending is not None else [],
        )

        # An ATIS letter the pilot reports is remembered so the controller can
        # tell whether they have the current information.
        if parsed.get("atis_letter"):
            session.atis_acknowledged = parsed.get("atis_letter")

        if self.human is not None:
            # Somebody real is working this frequency. What the pilot said is
            # shown, because they said it, and nothing is transmitted over the
            # person who is going to answer it.
            self.emit("info",
                      f"{self.human.describe()} is working this frequency.",
                      key="log.human_online",
                      mhz=f"{station.mhz:.3f}", station=self.human.callsign,
                      network=self.human.network.upper())
            return None

        # Addressed to another aeroplane on the frequency, the answer is that
        # aeroplane's: "Delta four five, say your position" is not the
        # controller's to take.
        if self._answer_from_traffic(session, station, parsed):
            return None

        # Somebody else's exchange is half way through on this frequency:
        # the first call of a flight the controller has not worked yet may
        # go unanswered, and any other is answered "stand by" now and in full
        # once the exchange is over.
        congested = self._congested(station, parsed)
        if congested and not session.established_with(station) \
                and self._first_call_missed(session, station):
            return Reply("", station, silent=True)

        weather = self.weather_for(station.ident)
        with timed("controller"):
            reply = self.brain.respond(parsed, session, station, weather,
                                       self.state)

        if reply.instruction is not None:
            session.remember(reply.instruction)

        if congested and reply.speak and parsed.intent is Intent.READBACK:
            # A readback is not a request: "standby" to it told the pilot
            # their correct readback was something to wait on. Its answer
            # waits, unannounced, for the other exchange to end.
            self._held_reply = (reply, station, spoken, time.monotonic())
            decision("congestion", station=station.callsign, said="nothing",
                     held="readback")
            reply = Reply("", station, silent=True)
        elif congested and reply.speak:
            self._held_reply = (reply, station, spoken, time.monotonic())
            p = self.brain.for_station(station, spoken)
            session.aircraft.language = getattr(p, "language", "en")
            decision("congestion", station=station.callsign, said="standby",
                     held=(reply.instruction.kind
                           if reply.instruction is not None else "answer"))
            session.tell_to_wait()
            reply = Reply(p.standby(session.aircraft), station)

        if reply.speak:
            # An uplink is sent, not said: the message is in the log and
            # on the panel, and nothing goes out on the voice circuit.
            datalink = reply.instruction is not None \
                and reply.instruction.delivery == "cpdlc"
            self.emit("cpdlc" if datalink else "atc",
                      reply.text, station.callsign)
            session.note("atc", as_written(reply.text))
            RADIO.write(
                "atc", text=reply.text, language=spoken,
                station=station.callsign, frequency=round(station.mhz, 3),
                instruction=(reply.instruction.kind
                             if reply.instruction is not None else ""),
                owed=(reply.instruction.owed_items
                      if reply.instruction is not None else []),
                handoff_to=(reply.handoff_to.callsign
                            if reply.handoff_to is not None else ""),
                delivery=("cpdlc" if datalink else "voice"),
            )
            if not datalink:
                self.speak(reply.text, station, language=spoken)

        if reply.handoff_to is not None:
            session.handoff_pending = reply.handoff_to
            session.handoff_offered_at = time.monotonic()

        # An approval that reaches the ramp. A controller who says "push-back
        # approved" and then leaves the pilot to go and find the tug in a
        # menu has broken the one thing this program is for.
        if reply.ground == "pushback":
            self._start_pushback()

        # And the other seat says the numbers back to you, on the interphone.
        # After the controller, never over them, and never on the air.
        if reply.instruction is not None:
            try:
                self.crew.confirm(reply.instruction)
            except Exception:
                log.debug("the first officer raised", exc_info=True)

        return reply

