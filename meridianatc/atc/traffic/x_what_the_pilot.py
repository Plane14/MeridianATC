from __future__ import annotations

from ._shared import (
    Call,
    Flight,
    same_channel,
)


class _What_the_pilot:
    # ------------------------------------------------------------------
    # what the pilot hears
    # ------------------------------------------------------------------

    @property
    def quiet(self) -> bool:
        """Whether there is nothing here to run at all.

        The density says how much to *invent*, not whether the frequency is
        worked. A pilot running FSLTL with the invented traffic turned right
        down has a full airport and a silent radio otherwise -- which was the
        first thing that went wrong when the two sources were put together.
        """
        if not self.field.ident:
            return True
        # One the pilot put there by hand is worked whatever the density:
        # turning the invented traffic off is not asking for that one to
        # stand frozen on the runway.
        return (self.density <= 0 and not self.observed_count
                and not self.flights and not self._asked_for
                and not self._materialising)

    def advance(self, now: float) -> None:
        """Move everybody along without collecting what they said.

        For when there is nothing to hear it on: the aeroplanes are still
        there, still taxiing and still landing, and whatever they transmit
        waits in the queue until somebody tunes in or it goes stale.
        """
        if self.quiet:
            return
        self._spawn(now)
        self._advance(now)
        self.pending = [c for c in self.pending if c.at > now]
        self.flights = [f for f in self.flights if not f.done]

    def poll(self, now: float, listening_on: str = "",
             mhz: float = 0.0) -> list[Call]:
        """Advance every flight and return what is due on this frequency.

        ``listening_on`` is the position the user is tuned to. Transmissions on
        any other frequency still happen -- the flights keep moving -- but the
        user does not hear them, exactly as on a real radio.

        ``mhz`` is the channel tuned, when known, and then decides instead:
        a call is heard when it is said on the same channel. Ground and
        Tower sharing one are both heard; a field's second Approach
        frequency does not hear the first's.
        """
        if self.quiet:
            return []

        if self._first_poll is None:
            self._first_poll = now
        self._spawn(now)
        self._advance(now)

        due: list[Call] = []
        keep: list[Call] = []
        for call in self.pending:
            if call.at > now:
                keep.append(call)
            elif self._heard_on(call, listening_on, mhz):
                due.append(call)
            # A call on another frequency is simply dropped: it happened, the
            # user was not listening to it.
        self.pending = keep
        self.flights = [f for f in self.flights if not f.done]
        return due

    def _heard_on(self, call: Call, listening_on: str, mhz: float) -> bool:
        if not listening_on and not mhz:
            return True
        if not call.mhz:
            call.mhz = self._freq(call.position)
        if mhz and call.mhz:
            return same_channel(call.mhz, mhz, 0)
        return not listening_on or call.position == listening_on

    def traffic_ahead(self, arriving: bool = True) -> list[Flight]:
        """The flights in front of the user, nearest first.

        Used by the controller to sequence the user into the stream: "number
        two, follow the Airbus on a four mile final" is only honest if there is
        an Airbus on a four mile final.
        """
        if arriving:
            found = [f for f in self.flights
                     if f.arriving and not f.stuck
                     and f.phase in ("inbound", "final")]
            found.sort(key=lambda f: f.distance_nm)
        else:
            found = [f for f in self.flights
                     if not f.arriving and not f.stuck
                     and f.phase in ("holding", "ready", "waiting", "lineup")]
            found.sort(key=lambda f: f.sequence)
        return found

    def runway_is_busy(self, now: float) -> bool:
        """Whether somebody is on the runway right now."""
        return now < self.runway_free_at

