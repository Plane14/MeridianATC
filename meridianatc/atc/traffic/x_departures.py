from __future__ import annotations

from ._shared import (
    _CIRCUIT_PHASES,
    AIRBORNE_BY_FT,
    INITIAL_CLIMB_FT,
    Flight,
    _said_route,
    _spell_route,
)


class _Departures:
    # -- departures ----------------------------------------------------

    # How long a departure with a door still open waits before looking again.
    PUSH_RETRY_S = 10.0

    def _may_push(self, flight: Flight) -> bool:
        """Whether this departure's doors are shut, as far as anybody knows."""
        if self.can_push is None:
            return True
        try:
            return bool(self.can_push(flight.written))
        except Exception:
            return True

    def _advance_departure(self, flight: Flight, now: float) -> None:
        phase = flight.phase

        if phase == "clearance":
            # The initial level is an altitude, MSL: what the flight
            # model climbs to, not the bare three thousand above sea
            # level that would sit under a high field.
            initial = self.field.elevation_ft + INITIAL_CLIMB_FT
            self._exchange(
                flight, now, "DEL",
                pilot_first=lambda p, a, s: p.request_clearance(
                    a, s, flight.destination,
                    atis_letter=self._atis_letter(flight.language),
                    type_name=self._type_of(flight)),
                controller=lambda c, a, s: c.ifr_clearance(
                    a, s, flight.destination,
                    departure_procedure=flight.sid,
                    transition=flight.sid_transition,
                    initial_altitude=initial, squawk=flight.squawk,
                    departure_freq=self._freq("DEP")),
                pilot_back=lambda p, a, s: p.clearance(
                    a, flight.destination, flight.squawk, initial),
                command=("clearance", {"altitude_ft": initial}),
            )
            flight.phase = "taxi"
            flight.next_at = now + self.random.uniform(25.0, 70.0)
            return

        if phase == "taxi" and not self._may_push(flight):
            flight.next_at = now + self.PUSH_RETRY_S
            return

        if phase == "taxi" and flight.on_layout:
            planned = self._plan_departure(flight)
            if planned is None:
                # Nowhere to go from here. An invented one stays on its
                # stand and is dropped, rather than being put somewhere
                # it cannot reach; a live one is handed back to the
                # feed, where the real aeroplane's day continues.
                if flight.source == "live":
                    self._unadopt(flight, now,
                                  why="nowhere to go from the stand")
                else:
                    flight.done = True
                return
            if self._crosses(planned[0], flight):
                # Somebody is coming the other way along it. It waits on
                # the stand until they are past.
                flight.next_at = now + 10.0
                return
            flight.planned = planned
            route, hold_at, rest, where = _said_route(
                self.ground, planned[0], self.field.ident, inbound=False)
            route = route or self._taxi_route()
            flight.taxi_rest, flight.taxi_split_xy = rest, where
            letter = self._atis_letter(flight.language)
            type_name = self._type_of(flight)
            if flight.ifr:
                def first(p, a, s):
                    return p.request_taxi(a, s, atis_letter=letter,
                                          type_name=type_name)
            elif flight.pattern_work:
                def first(p, a, s):
                    return p.call(
                                    a, s, p.book.request_pattern_work, greeting=True)
            else:
                def first(p, a, s):
                    return p.request_vfr_departure(a, s, atis_letter=letter,
                                                   type_name=type_name)
            self._exchange(
                flight, now, "GND",
                pilot_first=first,
                controller=lambda c, a, s: c.taxi_to_runway(
                    a, s, flight.runway, route, hold_short_taxiway=hold_at),
                # The readback is the taxiways, where it holds and the
                # runway: never the whole clearance said back.
                pilot_back=lambda p, a, s: p.taxi(
                    a, flight.runway, route=_spell_route(route),
                    hold_short_taxiway=_spell_route([hold_at]) if hold_at
                    else ""),
                command=("taxi", {"runway": flight.runway}),
            )
            flight.phase = "ready"
            flight.next_at = now + 5.0
            return

        if phase == "ready" and flight.on_layout and (
                flight.taxi is not None or flight.planned is not None):
            # Still taxiing. It asks for the runway when it is at the
            # holding point, not when a timer says it should be -- and is
            # given the rest of a cut taxi at the taxiway it held short of.
            self._rest_of_taxi(flight, now, inbound=False)
            flight.next_at = now + 2.0
            return

        if phase == "taxi":
            route = self._taxi_route()
            letter = self._atis_letter(flight.language)
            type_name = self._type_of(flight)
            if flight.ifr:
                def first(p, a, s):
                    return p.request_taxi(a, s, atis_letter=letter,
                                          type_name=type_name)
            elif flight.pattern_work:
                # No clearance to collect: a VFR departure asks to taxi and,
                # if it is staying in the circuit rather than going somewhere,
                # says so.
                def first(p, a, s):
                    return p.call(
                                    a, s, p.book.request_pattern_work, greeting=True)
            else:
                def first(p, a, s):
                    return p.request_vfr_departure(a, s, atis_letter=letter,
                                                   type_name=type_name)
            self._exchange(
                flight, now, "GND",
                pilot_first=first,
                controller=lambda c, a, s: c.taxi_to_runway(
                    a, s, flight.runway, route),
                pilot_back=lambda p, a, s: p.taxi(
                    a, flight.runway, route=_spell_route(route)),
                command=("taxi", {"runway": flight.runway}),
            )
            flight.phase = "ready"
            flight.next_at = now + self.random.uniform(50.0, 140.0)
            return

        if phase == "ready":
            # At the holding point and asking for the runway. What happens
            # next is not this flight's decision. At an AFIS the call opens
            # the exchange that gives the runway (``_afis_exchange``), so it
            # is not made twice.
            if not (self.uncontrolled and self.field.afis is not None):
                self._say(flight, now, "TWR", from_pilot=True,
                          make=lambda p, a, s: p.ready_for_departure(
                              a, flight.runway, s,
                              intersection=self._said_intersection(flight)))
            flight.phase = "waiting"
            flight.next_at = now + self.random.uniform(4.0, 12.0)
            return

        if phase in ("waiting", "lineup"):
            self._work_the_departure(flight, now)
            return

        if phase == "takeoff":
            # Rolling, or climbing away. The runway is left behind when the
            # aeroplane has actually left it.
            climbing = (flight.altitude_ft
                        > self.field.elevation_ft + AIRBORNE_BY_FT)
            if flight.on_ground or not climbing:
                flight.next_at = now + 2.0
                return
            if flight.pattern_work or flight.last_circuit:
                self._exchange(
                    flight, now, "TWR",
                    controller=lambda c, a, s: c.report_position(
                        a, c.REPORT_MIDFIELD_DOWNWIND),
                    pilot_back=lambda p, a, s: p.wilco(a),
                )
                flight.phase = "upwind"
                flight.step = 0
                flight.next_at = now + 20.0
                return
            flight.phase = "departed"
            flight.next_at = now + self.random.uniform(15.0, 40.0)
            return

        if phase == "departed" and flight.handed_off:
            # Flying away. It goes when it is out of sight, not the moment
            # it has changed frequency.
            if (flight.distance_nm > self.radius_nm
                    or flight.altitude_ft > self.field.elevation_ft
                    + self.OUT_OF_SIGHT_FT):
                if flight.source == "live":
                    # An adopted one out of sight is the feed's again,
                    # where the real aeroplane still is: dropped here, it
                    # comes back inside the feed's wider reach and is
                    # adopted and handed off all over again.
                    self._unadopt(flight, now, why="out of sight")
                else:
                    flight.done = True
            flight.next_at = now + 5.0
            return

        if phase == "departed":
            if not flight.ifr:
                # Nobody to hand a VFR departure to at an airfield. It is
                # turned on course and then it is on its own.
                self._exchange(
                    flight, now, "TWR",
                    controller=lambda c, a, s: c.turn_out_approved(a),
                    pilot_back=lambda p, a, s: p.leaving(a),
                    command=("turn_out", {}),
                )
                flight.handed_off = True
                flight.next_at = now + 5.0
                return
            station, mhz = self._next_station("DEP", flight.language)
            if station:
                self._exchange(
                    flight, now, "TWR",
                    controller=lambda c, a, s: c.handoff(a, station, mhz),
                    pilot_back=lambda p, a, s: p.contact(a, station, mhz),
                    command=("handoff", {}),
                )
            flight.handed_off = True
            flight.next_at = now + 5.0
            return

        if phase in _CIRCUIT_PHASES:
            # Round the circuit after a departure: the legs fly themselves and
            # the aeroplane becomes an arrival when it turns final.
            flight.next_at = now + 4.0
            return

        if phase in ("go_around", "missed_hold"):
            self._after_the_go_around(flight, now)

    # Where a departure that has been handed on stops being drawn: far enough
    # out, or high enough, that nobody on the field could still see it. The
    # range itself is the injector's ``radius_nm`` (traffic.radius_nm); the
    # figure here is only its default for a caller that never set one, and
    # the altitude that ends it whichever way the range is set.
    OUT_OF_SIGHT_NM = 12.0
    OUT_OF_SIGHT_FT = 9000.0

