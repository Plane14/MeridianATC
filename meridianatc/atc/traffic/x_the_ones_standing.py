from __future__ import annotations

from ._shared import (
    CONCURRENT_BY_RANK,
    Flight,
    fleets,
    livemod,
)


class _The_ones_standing:
    # ------------------------------------------------------------------
    # the ones standing at the gates
    # ------------------------------------------------------------------

    # How often an emptied stand is filled again once the apron has been
    # set out, and how far from the pilot it has to be: an aeroplane that
    # appears on the next stand is a conjuring trick, one that appears
    # across the apron is one that was towed there while nobody looked.
    RESTOCK_S = 90.0
    # How long the first fill waits for the simulator's own aeroplanes to
    # have been read, when a reader is attached. Two polls and a margin.
    OBSERVED_GRACE_S = 8.0
    RESTOCK_CLEAR_M = 150.0

    def _parked_wanted(self) -> int:
        """How many aeroplanes should be standing at the gates.

        As many as were asked for, but never so many that the moving traffic
        has nowhere to go: an arrival with no stand free rolls off the
        runway and is taken away, so enough are left empty for everything
        that could be on the move at once, and a couple over.
        """
        if self.ground is None or self.parked <= 0:
            return 0
        airline_field = self._airline_field
        stands = len(self.ground.stands_for(airliner=airline_field))
        spare = CONCURRENT_BY_RANK.get(self.field.size_rank, 3) + 2
        # The field's share of ``traffic.parked``, which is an airport's:
        # twelve Cessnas stood at Kortrijk.
        wanted = fleets.parked_for(self.field.kind, self.parked)
        # Less the real ones standing here in the categories this field
        # parks: a live airliner at the gate is one fewer to invent, a light
        # aeroplane on the grass at a big airport is none fewer.
        mine = livemod.field_categories(self.field.kind)
        wanted -= sum(1 for one in self._parked_observed.values()
                      if (one.category or livemod.category_of(
                          one.callsign, one.type_code)) in mine)
        return max(0, min(wanted, stands - spare))

    def _fill_stands(self, now: float) -> None:
        """Set the apron out, then keep it about as full as it was.

        All at once the first time -- the pilot is loading in and has not
        looked yet -- and one at a time after that, away from the pilot.
        Most of the restocking is done by the traffic itself: an arrival
        that has parked stays, as one of these, and a departure boards one.
        """
        if now < self._parked_at:
            return
        if (self.await_observed and not self._observed_read
                and self._first_poll is not None
                and now - self._first_poll < self.OBSERVED_GRACE_S):
            return                     # the simulator has not said yet
        wanted = self._parked_wanted()
        first = not self._stands_filled
        while len(self.statics) < wanted:
            flight = self._new_static(now, clear_of_user=not first)
            if flight is None:
                break
            self.statics.append(flight)
            if not first:
                break
        self._stands_filled = True
        self._parked_at = now + self.RESTOCK_S

    def _new_static(self, now: float,
                    clear_of_user: bool = False) -> Flight | None:
        """An aeroplane on a free stand, which will one day be a departure."""
        if self.ground is None:
            return None
        away = []
        user = self.user
        if clear_of_user and user is not None and user.has_position:
            away = [(*self.ground.to_xy(user.latitude, user.longitude),
                     self.RESTOCK_CLEAR_M)]
        flight = self._new_flight(now, arriving=False, parked=False)
        if flight is None:
            return None
        stand = self._free_stand(airliner=flight.operator is not None
                                 or flight.ifr, also=away)
        if stand is None:
            self._used_callsigns.discard(flight.written)
            return None
        self._park(flight, stand)
        self._make_static(flight)
        return flight

    def _make_static(self, flight: Flight) -> None:
        flight.static = True
        flight.phase = "parked"
        flight.step = 0
        flight.sequence = 0
        flight.next_at = float("inf")
        flight.ground_speed_kt = 0.0
        flight.target.clear()
        flight.target.intent = "parked"
        flight.doing = None
        flight.commands.clear()

    # How many parked aeroplanes may be booked or boarding at once, under
    # the schedule. A departure asked for while that many are in hand is
    # one already on its way.
    MAX_BOARDING = 2

    def _book_boarding(self, now: float) -> bool:
        """The schedule's departure: book a parked aeroplane to board.

        It boards after ``boarding_lead_s`` -- the minutes in which its model
        can be changed with every door shut -- for ``boarding_s``, and only
        then calls for its clearance (:meth:`_work_the_boarding`). True when
        the departure is in hand, booked now or already; False when there is
        nobody parked to book, and a departure has to be found another way.
        """
        booked = [f for f in self.statics if f.boarding_at]
        if len(booked) >= self.MAX_BOARDING:
            return True
        vmc = self._vmc()
        ready = [f for f in self.statics
                 if (f.drawn or not self.requires_sim) and not f.boarding_at
                 and (f.ifr or vmc)]
        if not ready:
            return bool(booked)
        flight = self.random.choice(ready)
        flight.boarding_at = now + self.boarding_lead_s
        return True

    def _work_the_boarding(self, now: float) -> None:
        """Start the boarding that is due, and send the boarded off."""
        vmc = self._vmc()
        for flight in list(self.statics):
            if not flight.boarding_at:
                continue
            if not flight.ifr and not vmc:
                # Booked in visual conditions and the field went down
                # under it: the passengers are sent home, and the
                # aeroplane stays where it is parked.
                flight.boarding_at = 0.0
                flight.boarding = False
                continue
            if not flight.boarding and now >= flight.boarding_at:
                flight.boarding = True
            if flight.boarding and now >= flight.boarding_at + self.boarding_s:
                boarded = self._board_static(now, flight)
                if boarded is not None:
                    self._admit(boarded, now)

    def _board_static(self, now: float,
                      flight: Flight | None = None) -> Flight | None:
        """A departure from an aeroplane already standing at a gate.

        The same aeroplane, in the same simulator object, so nothing
        appears: the one the pilot has been parked beside is the one that
        calls for its clearance and pushes back. ``flight`` is the one the
        schedule booked; without it, one is chosen now.
        """
        if flight is None:
            vmc = self._vmc()
            ready = [f for f in self.statics
                     if (f.drawn or not self.requires_sim)
                     and (f.ifr or vmc)]
            if not ready:
                return None
            flight = self.random.choice(ready)
        self.statics.remove(flight)
        flight.boarding_at = 0.0
        flight.boarding = False
        flight.static = False
        flight.arriving = False
        flight.runway = self.field.departure_runway
        if not flight.destination:
            # A static aeroplane was routed when it was parked; only an
            # older one without a destination needs one now.
            if flight.ifr:
                flight.destination, flight.destination_icao = \
                    self._route_end(
                        flight.operator, flight.type_code,
                        getattr(flight.performance, "range_nm", 0.0) or 0.0)
            else:
                flight.destination = (self._nearby_field()
                                      or self._elsewhere())
        flight.parked_at = 0.0
        flight.next_at = now + self.random.uniform(2.0, 12.0)
        flight.sequence = len(self.traffic_ahead(False)) + 1
        flight.target.intent = ""
        flight.phase = ("clearance" if flight.ifr
                        and "DEL" in self.field.stations else "taxi")
        return flight

    # How many aeroplanes may be committed to the runway at once. One a
    # minute is what a single runway does, and the approach is that long: any
    # more than this and the tower is manufacturing a queue it will have to
    # send half of round again. Departures are not limited the same way -- an
    # aeroplane at a holding point costs nothing.
    MAX_ON_APPROACH = 4

