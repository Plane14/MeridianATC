from __future__ import annotations

from ._shared import (
    OBSERVED_LIMIT,
    PARKED_RANGE_NM,
    TAXIING_KT,
    decision,
    haversine_nm,
)


class _Aeroplanes_somebody_else:
    # ------------------------------------------------------------------
    # aeroplanes somebody else is flying
    # ------------------------------------------------------------------

    def set_observed(self, seen, now: float) -> None:
        """Take the aeroplanes the simulator is holding onto the frequency.

        ``seen`` is what :class:`meridianatc.integrations.simaircraft.SimAircraft`
        reports: everything the simulator has, most of which is not this
        airport's business. What survives :meth:`_works_here` becomes a
        :class:`Flight` like any other, and from there the whole of the rest of
        this module treats it as traffic -- it holds the runway, it takes its
        place in the landing sequence, and it is the aeroplane the user is told
        to follow.

        What it never becomes is a subordinate. It is flown by whoever injected
        it, so the flight model does not touch it and the tower does not
        instruct it. The only thing said to it is a description of what it is
        already doing; see :meth:`_advance_observed`.
        """
        if not self.field.ident:
            return
        # The parked aeroplanes are taken up only once the pilot's own
        # position is known: the pilot is in the simulator's list too, and
        # until they can be recognised by where they are, their own
        # aeroplane is a parked airliner on a stand -- which, measured, got
        # its doors opened by this program for the ten seconds it took.
        user_known = self.user is not None and self.user.has_position
        self._observed_read = seen is not None and user_known
        if not self.live_managed:
            # The managed switch off again: every live aeroplane that was
            # adopted is handed back to the feed, wherever the real one
            # now is -- before `known` is built, so the feed's copy lands
            # back on the same strip rather than a second one.
            for f in list(self.flights) + list(self.statics):
                if not f.observed and f.source == "live":
                    self._unadopt(f, now)
        known = {f.observed_id: f for f in self.flights if f.observed}
        here: set[int] = set()
        # The ones this program put there itself. They are in the simulator
        # now, so the simulator reports them, under the callsign they were
        # created with -- and read back as somebody else's they would be
        # worked twice, once flown and once watched.
        ours = {f.written.upper()
                for f in self.flights + self._materialising + self._asked_for
                + self.statics
                if not f.observed}
        # The live ones this program draws: the simulator reports each of
        # those back too, under the same callsign, and that copy is ours.
        live_ours = ({f.written.upper() for f in self.flights
                      if f.observed and f.source == "live"}
                     | {one.callsign for one in self._parked_observed.values()
                        if one.live})
        seen = [one for one in self._rejoin_the_feed(seen or (), now)
                if getattr(one, "source", "") == "live"
                or (one.callsign or "").strip().upper() not in live_ours]
        # A copy of ours just given back to the feed is no longer ours.
        ours -= {f.written.upper() for f in self.flights
                 if f.done and not f.observed}
        # The pilot's own real aeroplane is dropped before the pose is
        # fixed up: snapped to a stand it could sit far enough from the
        # pilot's true position that ``_is_user`` stopped recognising it,
        # and the frequency gained a ghost of the person flying.
        seen = [self._live_pose(one, now)
                if getattr(one, "source", "") == "live" else one
                for one in seen
                if getattr(one, "source", "") != "live"
                or not self._is_user(one)]
        # The live ones the feed hears now: a parked one is kept on its stand
        # only while it does not (_note_parked).
        self._live_heard = {one.object_id for one in seen
                            if getattr(one, "source", "") == "live"}
        # A real flight with a callsign an invented one already has (same
        # airline, same number): the invented one finishes what it is doing
        # and goes at its next gate, and the real one joins once it has --
        # never two aeroplanes with one callsign, and nobody taken off the
        # frequency mid-exchange.
        for one in seen or ():
            if getattr(one, "source", "") != "live":
                continue
            written = (one.callsign or "").strip().upper()
            if written in ours:
                for f in (self.flights + self._materialising + self.statics):
                    if not f.observed and f.written.upper() == written \
                            and not f.retire:
                        f.retire = True
                        decision("blend", event="retire", callsign=written,
                                 why="the real flight has this callsign")
        # Somebody else's aeroplanes standing on this airport. Not worked,
        # but in the way of anything this puts on a stand -- and on the map,
        # and handled, because the simulator does nothing for them.
        standing = [
            one for one in (seen if user_known else ())
            if getattr(one, "positioned", False) and one.on_ground
            and one.ground_speed_kt < TAXIING_KT
            and (one.callsign or "").strip().upper() not in ours
            and not self._is_user(one)
            and haversine_nm(self.field.latitude, self.field.longitude,
                             one.latitude, one.longitude) < PARKED_RANGE_NM]
        self._parked_seen = [(one.latitude, one.longitude) for one in standing]
        self._note_parked(standing, known, now)

        # Nearest first, so a field with more aeroplanes on it than can be
        # worked keeps the ones that are about to use the runway.
        wanted = [one for one in (seen or ())
                  if (one.callsign or "").strip().upper() not in ours
                  and not self._is_user(one)
                  and self._works_here(one, one.object_id in known)]
        wanted.sort(key=lambda one: haversine_nm(
            self.field.latitude, self.field.longitude,
            one.latitude, one.longitude))

        # OBSERVED_LIMIT is the budget for what the simulator reports;
        # the live aeroplanes have a budget of their own
        # (traffic.live.limit, applied by the engine before they are
        # handed over), not a share of it -- the nearest 300 at the very
        # outside, where a wide feed would otherwise flood the frequency.
        live_wanted = [one for one in wanted
                       if getattr(one, "source", "") == "live"]
        others = [one for one in wanted
                  if getattr(one, "source", "") != "live"]
        for one in live_wanted[:300] + others[:OBSERVED_LIMIT]:
            here.add(one.object_id)
            flight = known.get(one.object_id)
            if flight is None:
                flight = self._observed_flight(one)
                if flight is None:
                    continue
                self.flights.append(flight)
                if flight.source == "live":
                    # Its first appearance, for logs radio: which real
                    # aeroplane, from which source, going where.
                    decision("live_traffic", event="appeared",
                             callsign=flight.written, type=flight.type_code,
                             feed=getattr(one, "feed", "") or "",
                             category=flight.category,
                             origin=flight.origin,
                             destination=flight.destination,
                             on_ground=bool(one.on_ground))
            if (self.live_managed and getattr(one, "source", "") == "live"
                    and flight.observed and flight.source == "live"):
                self._adopt_live(flight, one, now)
            if not flight.observed:
                # Adopted this pass or adopted already: ours flies it,
                # the feed no longer writes its position.
                continue
            self._read_observed(flight, one, now)

        # Anything that has gone is gone. It landed and taxied out of range,
        # it was despawned, or the pilot flew away from the field -- and in
        # none of those cases is it still on this frequency.
        for object_id, flight in known.items():
            if object_id not in here:
                flight.done = True
        self.flights = [f for f in self.flights if not f.done]
        self.observed_count = sum(1 for f in self.flights if f.observed)

