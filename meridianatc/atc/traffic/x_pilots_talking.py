"""Talking to the traffic: crews answer when called, and call the pilot.

Two halves. When the user's transmission names an invented aeroplane --
"Delta four five, say your position" -- that aeroplane answers rather than
the controller, out of its own flight plan rather than a script. And on an
uncontrolled frequency, where crews police themselves, a crew that has not
heard the pilot before asks their position and intentions, as one does.

All of it is deterministic: what a crew knows is what its :class:`Flight`
holds, and when it would not know the answer it says so.
"""

from __future__ import annotations

import random

from ..intents import normalize
from ._shared import log

# A crew says where it is in the words its own strip would use. Position is
# never invented: a flight that has not left the apron is on the apron.
_POSITION = {
    "waiting": "on the apron",
    "ready": "ready for departure, holding short {runway}",
    "lineup": "lining up runway {runway}",
    "takeoff": "rolling runway {runway}",
    "upwind": "on the upwind runway {runway}",
    "crosswind": "on the crosswind leg",
    "downwind": "on downwind runway {runway}",
    "base": "on base runway {runway}",
    "final": "on final runway {runway}",
    "go_around": "going around, back on climbout",
    "departed": "climbing out en route to {destination}",
    "joining": "inbound, {distance} miles out",
    "taxi": "taxiing to runway {runway}",
    "taxi_in": "taxiing in to the apron",
    "landed": "just landed, clearing the runway",
    "parked": "parked on the apron",
    "clearance": "on the apron, start-up shortly",
}

_HUMOR = [
    "and it is a beautiful day up here",
    "and we would be faster if they let us",
    "say again all after the callsign",
]


class PilotsTalking:
    """Mixin on the injector: answers and calls between crews.

    ``advisor`` may hold an ``llm.decisions.Decisions`` the engine wires in;
    it only words the deterministic answer, never decides what one says."""

    advisor = None

    def flight_called(self, text: str):
        """The aeroplane a transmission names, if it names one of ours.

        Matches the spoken callsign on word boundaries -- "delta four five"
        finds DAL45, "delta" alone does not.
        """
        norm = normalize(text or "")
        if not norm:
            return None
        for flight in getattr(self, "flights", []):
            if flight.done:
                continue
            callsign = flight.aircraft.for_language(flight.language)
            for form in (callsign.spoken_full, callsign.spoken_short):
                form = normalize(form)
                if form and f" {form} " in f" {norm} ":
                    return flight
        return None

    def _position_words(self, flight) -> str:
        pattern = _POSITION.get(flight.phase, "in the vicinity")
        return pattern.format(
            runway=flight.runway or "the runway",
            destination=flight.destination or "our destination",
            distance=max(1, round(flight.distance_nm)),
        )

    def answer_the_user(self, session, parsed, station_position: str,
                        humor: bool = False) -> bool:
        """Have the called crew answer the pilot. True if one did."""
        text = parsed.text or ""
        if getattr(parsed, "callsign_matched", False):
            return False  # addressed to ATC, not another aeroplane
        flight = self.flight_called(text)
        if flight is None:
            return False
        user_cs = session.aircraft.for_language(
            flight.language).spoken_short or ""
        norm = parsed.normalized or normalize(text)
        wants_landing = "landing" in norm or "runway" in norm
        wants_weather = ("weather" in norm or "wind" in norm
                         or "qnh" in norm or "altimeter" in norm)
        own = flight.aircraft.for_language(flight.language).spoken_short \
            or flight.written
        if wants_weather:
            wx = self.field.weather
            wind = (f"wind {wx.wind_dir:.0f} degrees at "
                    f"{wx.wind_kt:.0f} knots" if wx else "no reported wind")
            reply = f"{user_cs}, {own}: {wind} here"
        elif wants_landing and flight.arriving:
            reply = (f"{user_cs}, {own}: "
                     f"landing runway {flight.runway or 'in use'}, "
                     f"{self._position_words(flight)}")
        elif wants_landing:
            reply = (f"{user_cs}, {own}: departing, "
                     f"{self._position_words(flight)}")
        else:
            reply = (f"{user_cs}, {own}: "
                     f"{self._position_words(flight)}")
            if humor and random.random() < 0.25:
                reply += f", {random.choice(_HUMOR)}"
        reply = self._advised(reply, flight, user_cs, own, text,
                              humor=humor)
        self._say(flight, self._now(), station_position, from_pilot=True,
                  make=lambda _b, _a, _n, r=reply: r)
        return True

    def _advised(self, reply, flight, user_cs, own, asked, *, humor):
        """A model-worded version of the crew's answer, or the answer itself.

        Every fact in a kept draft must already be in the answer or the
        flight's own data -- the advisor can give a crew some personality,
        never anything to say that was not already true.
        """
        advisor = self.advisor
        if advisor is None or not (reply or "").strip():
            return reply
        facts = " ".join(part for part in (
            own, flight.written, user_cs,
            self._position_words(flight),
            f"runway {flight.runway}" if flight.runway else "",
            f"destination {flight.destination}" if flight.destination else "",
        ) if part)
        return advisor.draft(
            "traffic_pilot", asked, facts, reply,
            persona=f"the pilot of traffic aeroplane {own}, talking to "
                    f"another pilot over the radio"
                    + (", being a little playful" if humor else ""))

    def maybe_call_the_user(self, session, station_position: str,
                            now: float) -> bool:
        """One crew asks the pilot's intentions, once per arrival episode.

        Only on frequencies where crews coordinate among themselves -- an
        uncontrolled field's advisory channel. Controlled fields go through
        the controller; a pilot-to-pilot call there would be noise.
        """
        if station_position not in ("CTAF", "INFO"):
            return False
        last = getattr(self, "_user_called_at", 0.0)
        if now - last < 240.0:
            return False
        for flight in getattr(self, "flights", []):
            # A moving aeroplane is the one that wonders who else is on the
            # frequency; somebody parked or shutting down does not care.
            if flight.done or flight.phase in (
                    "parked", "waiting", "clearance", "landed", "taxi_in"):
                continue
            own = flight.aircraft.for_language(flight.language).spoken_short
            theirs = session.aircraft.for_language(
                flight.language).spoken_short or "aircraft calling"
            reply = (f"{theirs}, {own}: say your position and intentions")
            self._say(flight, now, station_position, from_pilot=True,
                      make=lambda _b, _a, _n, r=reply: r)
            self._user_called_at = now
            log.debug("traffic crew %s calling the user", flight.written)
            return True
        return False

    def _now(self) -> float:
        import time
        return time.monotonic()
