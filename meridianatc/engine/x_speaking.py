from __future__ import annotations

from ._shared import (
    PROFILE_ATIS,
    PROFILE_HF,
    AtisReport,
    Station,
    VoicesMissing,
    as_written,
    log,
    time,
)


class _Speaking:
    # ------------------------------------------------------------------
    # speaking
    # ------------------------------------------------------------------

    def spoken_form(self, text: str) -> str:
        """What the synthesiser is actually given.

        The phraseology composes what a controller says, which is a quantity
        one numeral at a time: "one one eight decimal three one". Some people
        would rather the figures were read instead, and ``atc.spoken_numerals``
        hands over "118.310" for the voice to make what it will of. The same
        converter the transcript uses, pointed at the other output, so the two
        can never disagree about what counts as a number -- and callsigns stay
        as words in both, because "Delta twelve thirty four" is a name.
        """
        if not self.config.atc.spoken_numerals:
            return text
        return as_written(text)

    def speak(self, text: str, station: Station, atis: bool = False,
              language: str = "en") -> None:
        """Render and queue a controller transmission.

        The render runs on the synthesiser's own lane: a CPU engine takes
        seconds per transmission and this method used to spend them on the
        situation loop, which was the worst of the long ticks. The audio
        lands on the player when the lane's drain runs it back, ahead of
        anything the cabin asked for first.
        """
        # The last guard: whatever path got here, a dead radio plays nothing.
        if not self.radio_powered:
            return
        if not atis:
            self._atc_said_at = time.monotonic()
        altitude = self.state.altitude_ft
        radio_fx = self.config.audio.radio_effects
        # The path the voice came down: a recording for ATIS, SSB and
        # static for an oceanic sector's HF, plain VHF otherwise.
        if atis:
            profile = PROFILE_ATIS
        else:
            from ..atc.enroute import is_oceanic
            profile = PROFILE_HF if is_oceanic(station) else None
        def render():
            phrased = self.phrasing.render(text) if self.phrasing is not None else text
            return self.synth.speak(
                self.spoken_form(phrased),
                facility=station.facility or station.ident,
                position=station.position, ident=station.ident,
                language=language, distance_nm=station.distance_nm,
                altitude_ft=altitude, profile=profile,
                radio=radio_fx)
        self._radio_renders += 1
        self.synth.render_soon(
            render,
            on_done=lambda tr, err: self._spoken(tr, err, station),
            priority=0)

    def _spoken(self, transmission, error, station: Station) -> None:
        """A rendered transmission handed back on the loop: play it."""
        self._radio_renders = max(0, self._radio_renders - 1)
        if isinstance(error, VoicesMissing):
            # Not a fault: a download that has not happened. It reads as one
            # every time the controller opens their mouth, so it is said once
            # per session and then the panel is asked to put the download
            # dialog up, which is the only thing that actually fixes it.
            first = not self._voices_reported
            self._voices_reported = True
            log.warning("no controller voices: %s", error.where())
            self.emit(
                "error",
                "The controller voices have not been downloaded yet. "
                "Open Settings, then Components, and press Download.",
                key="log.voices_missing",
                action="setup" if first else "",
            )
            return
        if error is not None:
            log.error("synthesis failed", exc_info=error)
            self.emit("error",
                      f"Could not synthesise the transmission: {error}",
                      key="log.synthesis_failed", detail=error)
            return
        if transmission is None:
            return
        self.player.play(
            transmission.audio, transmission.sample_rate, label=station.callsign
        )

    def play_atis(self, airport_ident: str | None = None,
                  language: str | None = None) -> AtisReport | None:
        """Broadcast the ATIS for a field."""
        if airport_ident is None:
            station = self.station
            airport_ident = station.ident if station else (
                self.session.departure.ident
                if self.session and self.session.departure else ""
            )
        if not airport_ident:
            return None
        if not self.radio_powered:
            self.emit("info", "The radio has no power: nothing was "
                      "transmitted. Switch on the battery and the avionics.",
                      key="log.radio_no_power")
            return None

        spoken = self._language_for(airport_ident, language)
        report = self.atis_for(airport_ident, spoken)
        if report is None:
            self.emit("info", f"No ATIS available for {airport_ident}.",
                      key="log.no_atis", airport=airport_ident)
            return None

        atis_station = self.navdb.station_for(airport_ident, "ATIS")
        if atis_station is None:
            airport = self.navdb.airport(airport_ident)
            atis_station = Station(
                ident=airport_ident, position="ATIS", mhz=0.0,
                facility=airport.spoken if airport else airport_ident,
                dialect=airport.dialect if airport else "faa",
            )

        self.emit("atis", report.text, f"{airport_ident} ATIS {report.letter}")
        self.speak(report.text, atis_station, atis=True, language=spoken)
        return report

