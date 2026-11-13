from __future__ import annotations

from ._shared import (
    Flight,
    landing_traffic,
)


class _Small_facts_about:
    # ------------------------------------------------------------------
    # small facts about the field
    # ------------------------------------------------------------------

    def _freq(self, position: str) -> float:
        station = self.field.stations.get(position)
        return station.mhz if station else 0.0

    def _next_station(self, position: str,
                      language: str = "en") -> tuple[str, float]:
        """The next frequency, named the way the current exchange is being run.

        A French handoff hands you to "Orly Sol", not to "Orly Ground": the
        station's name is part of the transmission and follows its language.
        Empty where nobody works that position, which is what stops an
        aeroplane being sent to a frequency that cannot answer it.
        """
        station = self.field.stations.get(position)
        if station is None:
            return "", 0.0
        return station.callsign_in(language), station.mhz

    def _taxi_route(self) -> list[str]:
        """A short taxi route, as letters.

        The phraseology spells them, so they leave here as "A", "B" -- handing
        it "Alpha" would get it spelled a second time, letter by letter.
        """
        letters = list("ABCDEFGHJKLMNPQRSTVWY")
        count = self.random.choice((1, 2, 2, 3))
        return self.random.sample(letters, count)

    def _arrival_inside(self, miles: float) -> Flight | None:
        for flight in self.traffic_ahead(True):
            if flight.phase == "final" and flight.distance_nm <= miles:
                return flight
        return None

    def _describe_arrival(self, flight: Flight | None, language: str) -> str:
        if flight is None:
            return ""
        return landing_traffic(flight.describe(language), language)

    def _sequence_text(self, flight: Flight, ahead: Flight) -> str:
        phrase = self._phrase(flight.language)
        try:
            return phrase.sequence_number(flight.sequence,
                                          ahead.describe(flight.language))
        except Exception:
            return ""


