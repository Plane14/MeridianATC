from __future__ import annotations

from ._shared import (
    CONCURRENT_BY_RANK,
    fleets,
    livemod,
    roster_for,
    time,
)


class _Making_aeroplanes:
    # ------------------------------------------------------------------
    # making aeroplanes
    # ------------------------------------------------------------------

    # How busy a field really is when the weather is on the ground and
    # grey: instruments or not, a ceiling or a visibility that low is
    # fewer movements an hour, not the same ones slightly damper.
    _IMC_RATE = 0.6
    # What VFR needs to be legal, roughly: fifteen hundred feet of cloud
    # base and three miles to see by, the figures a special VFR is issued
    # under rather than flown over.
    _VMC_CEILING_FT = 1500
    _VMC_VIS_SM = 3.0

    def _vmc(self) -> bool:
        """Whether the field is in visual conditions, on its own report.

        A field that reports nothing is the field it always was; only a
        report can call the weather bad.
        """
        weather = self.field.weather
        if weather is None or not getattr(weather, "raw", ""):
            return True
        if getattr(weather, "visibility_sm", 10.0) < self._VMC_VIS_SM:
            return False
        ceiling = min((h for cover, h in getattr(weather, "clouds", [])
                       if cover in ("BKN", "OVC", "VV")), default=None)
        return ceiling is None or ceiling >= self._VMC_CEILING_FT

    def _wanted(self) -> int:
        """How many invented aeroplanes this field should have.

        Less whatever somebody else has already put there -- FSLTL's, or the
        real ones from the live feed -- counted in its own category only
        (:meth:`_room`). A pilot running FSLTL has a full airport and does
        not want a second one invented on top of it; but a field with two
        real aeroplanes on it is still a quiet field, so the invented traffic
        fills the rest rather than standing down the moment anything is seen.

        And less when the weather is down: an IMC rate is not the VMC one.
        """
        wanted = sum(self._room().values())
        if not self._vmc():
            wanted = int(round(wanted * self._IMC_RATE))
        if self.max_invented is not None:
            wanted = min(wanted, self.max_invented)
        return wanted

    def _density_by_category(self) -> dict[str, int]:
        """The field's in-play figure, split into the categories its invented
        traffic is made of (airline and cargo, or light aircraft)."""
        kind = self.field.kind
        roster = ()
        if kind is None or kind.airline:
            key = (self.field.ident, self.field.size_rank)
            if getattr(self, "_roster_for", None) != key:
                self._roster_for = key
                self._roster = roster_for(self.field.ident,
                                          self.field.size_rank)
            roster = self._roster
        return livemod.wanted_by_category(kind, self._in_play(kind), roster)

    def _observed_by_category(self) -> dict[str, int]:
        """The aeroplanes somebody else flies that work this field, by
        category. What is overflying never had a strip (_works_here)."""
        counted = {c: 0 for c in livemod.CATEGORIES}
        for f in self.flights:
            if f.observed and not f.done and not f.stuck:
                counted[livemod.flight_category(f)] += 1
        return counted

    def _room(self) -> dict[str, int]:
        """How many invented aeroplanes each category still has room for:
        the field's density in that category less the real ones in it. A
        real Cessna crossing a big airport's zone takes nothing from its
        airliners, and a real airliner nothing from an airfield's Cessnas."""
        return livemod.room(self._density_by_category(),
                            self._observed_by_category())

    def _invented_by_category(self) -> dict[str, int]:
        counted = {c: 0 for c in livemod.CATEGORIES}
        for f in (self.flights + self._materialising
                  + [s for s in self.statics if s.boarding_at]):
            if not f.observed and not f.done:
                counted[livemod.flight_category(f)] += 1
        return counted

    def _steer(self) -> str | None:
        """The category the next invented aeroplane should be, when real ones
        have filled the others; None leaves the draw as it always was."""
        if not any(self._observed_by_category().values()):
            return None
        room = self._room()
        invented = self._invented_by_category()
        short = [c for c in livemod.CATEGORIES if room[c] - invented[c] > 0]
        return short[0] if len(short) == 1 else None

    def _in_play(self, kind: fleets.FieldKind | None) -> int:
        """Aeroplanes in play at once at this field, scaled by the density:
        the field's kind for a general-aviation field (fleet.classify), the
        size rank's for an airport, as it always was."""
        base = (kind.in_play if kind is not None and kind.in_play
                else CONCURRENT_BY_RANK.get(self.field.size_rank, 3))
        return int(round(base * self.density * self._diurnal()))

    def _movement_gap_s(self, kind: fleets.FieldKind | None) -> float:
        """Seconds between one aeroplane starting and the next."""
        base = (kind.movement_gap_s if kind is not None and kind.movement_gap_s
                else 55.0)
        gap = base / max(0.25, self.density)
        # The small hours are spaced out on their own: the same tower
        # working a bank and working the night does not work them at the
        # same tempo.
        gap /= max(0.15, self._diurnal())
        # Long finals and careful taxiing in the murk: a tower works the
        # same rules with bigger gaps when nobody can see far.
        if not self._vmc():
            gap *= 1.4
        return gap

    def _diurnal(self) -> float:
        """What the hour does to how busy the field is, 0.15 to 1.0.

        A real field has a rhythm: quiet before five, building to the
        morning bank, steady through the day, a second bank before
        dinner, then down to the night flights and the freighters. The
        curve is the local hour's, linear between its knots -- a field
        does not go to sleep at a stroke, it winds down.
        """
        if not getattr(self, "diurnal", True):
            return 1.0
        hour = time.localtime().tm_hour + time.localtime().tm_min / 60.0
        knots = ((0.0, 0.15), (5.0, 0.15), (7.0, 0.7), (9.0, 1.0),
                 (16.0, 0.8), (18.5, 1.0), (21.0, 0.45), (23.0, 0.15),
                 (24.0, 0.15))
        for (h0, f0), (h1, f1) in zip(knots, knots[1:], strict=False):
            if h0 <= hour <= h1:
                span = h1 - h0
                return f0 + (f1 - f0) * ((hour - h0) / span) if span else f0
        return 1.0

    def _booked(self) -> int:
        """Departures booked off the apron, not yet moving. They are in play
        already: counted only once they moved, two booked on top of two in
        the air made four at Kortrijk."""
        return sum(1 for f in self.statics if f.boarding_at)

    def _spawn(self, now: float) -> None:
        if not self.invented:
            # Switched off: nothing new is invented and what was flying
            # stops existing. The real and observed aeroplanes are
            # somebody else's and stay -- their work is not the spawn's.
            self._drop_invented()
            return
        if self.requires_sim and not self.can_invent:
            self._drop_invented()
            return
        if self._asked_for:
            asked, self._asked_for = self._asked_for, []
            for flight in asked:
                self._admit(flight, now)
        if self.schedule or any(f.boarding_at for f in self.statics):
            # Without the schedule only the board button ever books one.
            self._work_the_boarding(now)
        self._fill_stands(now)
        self._retire_at_gate()
        self._say_the_blend()
        invented = (sum(1 for f in self.flights if not f.observed)
                    + len(self._materialising) + self._booked())
        if invented >= self._wanted() or now < self._spawn_at:
            return
        flight = self._new_flight(now, category=self._steer())
        if flight is not None:
            self._admit(flight, now)
        # Spacing between one starting and the next: the field's own, scaled
        # by how busy the pilot asked for it to be.
        gap = self._movement_gap_s(self.field.kind)
        self._spawn_at = now + self.random.uniform(gap * 0.6, gap * 1.4)

