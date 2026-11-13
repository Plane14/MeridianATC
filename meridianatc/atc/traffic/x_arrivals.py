from __future__ import annotations

from ._shared import (
    _CIRCUIT_PHASES,
    JOIN_REACHED_NM,
    LANDING_ROLL_S,
    Flight,
    _said_route,
)


class _Arrivals:
    # -- arrivals ------------------------------------------------------

    # Where on the approach each thing happens, in miles to run.
    HANDOFF_TO_TOWER_NM = 12.0
    LANDING_CLEARANCE_NM = 8.0
    TOUCHDOWN_NM = 0.15

    def _advance_arrival(self, flight: Flight, now: float) -> None:
        phase = flight.phase

        if phase == "inbound" and not flight.ifr:
            # A VFR arrival calls the tower up itself from outside the circuit
            # and is given a join, not a handoff and an instrument approach.
            if flight.step == 0:
                direction = flight.pattern_direction

                def joined(c, a, s):
                    return c.enter_pattern(
                        a, s, flight.runway, leg=c.leg("downwind"),
                        direction=c.side(direction),
                        report=c.REPORT_MIDFIELD_DOWNWIND)

                def read_back(p, a, s):
                    phrase = self._phrase(flight.language)
                    return p.pattern_entry(
                        a, flight.runway, phrase.leg("downwind"),
                        phrase.side(direction))

                self._exchange(
                    flight, now, "TWR",
                    pilot_first=lambda p, a, s: p.inbound_for_landing(a, s),
                    controller=joined,
                    pilot_back=read_back,
                    command=("pattern", {"leg": "downwind"}),
                )
                flight.step = 1
                flight.phase = "joining"
                flight.next_at = now + 4.0
                return
            flight.next_at = now + 4.0
            return

        if phase == "joining":
            # Flying to the point the downwind starts from. It joins when it
            # gets there, not when a timer says so.
            if self._distance_to_join(flight) > JOIN_REACHED_NM:
                flight.next_at = now + 3.0
                return
            flight.phase = "downwind"
            flight.arriving = False
            flight.step = 0
            flight.next_at = now + 3.0
            return

        if phase in _CIRCUIT_PHASES:
            flight.next_at = now + 3.0
            return

        if phase == "inbound":
            # One check-in with approach, then it flies the arrival until the
            # tower takes it. The flying is done by the model, at its own
            # speed.
            #
            # Approach answers it. This used to be the one transmission in
            # the whole roster that nobody replied to, and a pilot sitting on
            # the approach frequency heard an aeroplane check in and then
            # minutes of silence -- which does not read as a quiet frequency,
            # it reads as a radio that is dropping transmissions.
            if flight.step == 0:
                flight.step = 1
                self._exchange(
                    flight, now, "APP",
                    pilot_first=lambda p, a, s: p.checking_in(
                        a, s, flight.altitude_ft, climbing=False),
                    controller=lambda c, a, s: c.radar_contact(a, s),
                )
            if flight.distance_nm > self.HANDOFF_TO_TOWER_NM:
                flight.next_at = now + 4.0
                return
            station, mhz = self._next_station("TWR", flight.language)
            if station:
                self._exchange(
                    flight, now, "APP",
                    controller=lambda c, a, s: c.handoff(a, station, mhz),
                    pilot_back=lambda p, a, s: p.contact(a, station, mhz),
                    command=("handoff", {}),
                )
            flight.phase = "final"
            flight.step = 0
            flight.next_at = now + 3.0
            return

        if phase == "final":
            self._work_the_arrival(flight, now)
            return

        if phase == "landed":
            if flight.ground_speed_kt > 30.0:
                flight.next_at = now + 2.0
                return
            if self.ground is not None and not flight.on_layout:
                self._plan_arrival(flight)
            self._stop_flying(flight)
            station, mhz = self._next_station("GND", flight.language)
            if station:
                self._exchange(
                    flight, now, "TWR",
                    controller=lambda c, a, s: c.handoff(a, station, mhz),
                    pilot_back=lambda p, a, s: p.contact(a, station, mhz),
                    command=("handoff", {}),
                )
            flight.phase = "taxi_in"
            flight.next_at = now + self.random.uniform(20.0, 50.0)
            return

        if phase == "taxi_in" and flight.on_layout:
            if flight.step < 10:
                route, hold_at, rest, where = _said_route(
                    self.ground, flight.taxi, self.field.ident, inbound=True)
                route = route or self._taxi_route()
                flight.taxi_rest, flight.taxi_split_xy = rest, where
                self._exchange(
                    flight, now,
                    "GND" if "GND" in self.field.stations else "TWR",
                    controller=lambda c, a, s: c.taxi_to_parking(
                        a, s, route, hold_short_taxiway=hold_at),
                    pilot_back=lambda p, a, s: p.leaving(a),
                    command=("taxi_in", {}),
                )
                flight.step = 10
            self._rest_of_taxi(flight, now, inbound=True)
            # Parked a while, and then off the radio -- the passengers are off
            # and it is nobody's business any more. It stays on its stand as
            # one of the parked aeroplanes if the apron has room for it, and
            # is its next departure; it used to vanish, in front of a pilot
            # who had just watched it taxi in.
            if flight.parked_at and now - flight.parked_at > self.PARKED_S:
                self._release_all(flight)
                if len(self.statics) < self._parked_wanted():
                    self._make_static(flight)
                    return
                flight.done = True
            flight.next_at = now + 5.0
            return

        if phase == "taxi_in":
            route = self._taxi_route()
            self._exchange(
                flight, now, "GND" if "GND" in self.field.stations else "TWR",
                controller=lambda c, a, s: c.taxi_to_parking(a, s, route),
                pilot_back=lambda p, a, s: p.leaving(a),
                command=("taxi_in", {}),
            )
            flight.done = True
            return

        if phase == "go_around":
            self._after_the_go_around(flight, now)

    # How long an arrival stays on its stand before it is taken away.
    PARKED_S = 180.0

    def _plan_arrival(self, flight: Flight) -> None:
        """Off the runway at the first exit and on to a free stand.

        With no stand free it rolls off the runway and is taken away there,
        which is better than being parked on top of somebody.
        """
        runway = self._runway_xy(flight.runway)
        layout = self.ground
        if runway is None or layout is None:
            return
        threshold, heading = runway
        stand = self._free_stand(airliner=flight.operator is not None
                                 or flight.ifr, prefer=flight.planned_stand)
        if stand is None:
            return
        x, z = layout.to_xy(flight.latitude, flight.longitude)
        # Round whoever is standing still -- the pilot, a departure at its
        # holding point -- where there is a way round. An exit planned
        # through a holding point left the arrival on the runway waiting
        # for the departure, and the departure waiting for the runway.
        standing = self._avoid_user() | self._avoid_standing(flight)
        taxi = None
        for avoid in (standing, self._avoid_user(), set()):
            taxi = layout.vacate(x, z, flight.heading, threshold, heading,
                                 self.RUNWAY_LENGTH_M, stand, avoid=avoid)
            if taxi is not None:
                break
        if taxi is None:
            return
        flight.on_layout = True
        flight.stand = stand.index
        flight.planned_stand = None
        flight.taxi = taxi

    def _work_the_arrival(self, flight: Flight, now: float) -> None:
        """An aeroplane on final: clear it, then watch it land."""
        if flight.step == 0:
            if flight.distance_nm > self.LANDING_CLEARANCE_NM:
                flight.next_at = now + 3.0
                return
            flight.step = 1
            ahead = [f for f in self.traffic_ahead(True)
                     if f is not flight and f.distance_nm < flight.distance_nm]
            flight.sequence = len(ahead) + 1
            sequence = ""
            if flight.sequence > 1 and ahead:
                sequence = self._sequence_text(flight, ahead[-1])
            following = self._user_ahead_of(flight)
            if flight.pattern_work:
                # Round again: the option covers the touch-and-go without the
                # tower having to know which one the pilot has chosen.
                self._exchange(
                    flight, now, "TWR",
                    pilot_first=lambda p, a, s: p.on_leg(
                        a, self._phrase(flight.language).leg("base"),
                        following=following),
                    controller=lambda c, a, s: c.cleared_touch_and_go(
                        a, flight.runway, self.field.weather,
                        sequence=sequence, station=s),
                    pilot_back=lambda p, a, s: p.option(a, flight.runway),
                    command=("touch_and_go", {"runway": flight.runway}),
                )
            elif not flight.ifr:
                self._exchange(
                    flight, now, "TWR",
                    pilot_first=lambda p, a, s: p.on_leg(
                        a, self._phrase(flight.language).leg("final"),
                        following=following),
                    controller=lambda c, a, s: c.cleared_to_land(
                        a, flight.runway, self.field.weather, station=s,
                        sequence=sequence),
                    pilot_back=lambda p, a, s: p.land(a, flight.runway),
                    command=("land", {"runway": flight.runway}),
                )
            else:
                self._exchange(
                    flight, now, "TWR",
                    pilot_first=lambda p, a, s: p.established(a, flight.runway),
                    controller=lambda c, a, s: c.cleared_to_land(
                        a, flight.runway, self.field.weather, station=s,
                        sequence=sequence),
                    pilot_back=lambda p, a, s: p.land(a, flight.runway),
                    command=("land", {"runway": flight.runway}),
                )
            flight.next_at = now + 3.0
            return

        if flight.distance_nm > self.TOUCHDOWN_NM or not flight.on_ground:
            flight.next_at = now + 2.0
            return

        self.runway_free_at = max(self.runway_free_at, now + LANDING_ROLL_S)
        # Down. Everything it was given for the approach is finished with.
        flight.assigned_speed_kt = None
        flight.assigned_heading = None
        flight.assigned_altitude_ft = None
        if flight.pattern_work:
            # A touch-and-go is on the runway for a moment and then round
            # again, which is what makes a circuit sound like a circuit.
            flight.phase = "takeoff"
            flight.arriving = False
            flight.step = 0
            # Whether to ask for the option again. When the answer is no it
            # still flies one more circuit -- and lands off that one, which is
            # what "last one" means to a pilot in the circuit.
            flight.pattern_work = self.random.random() < 0.7
            flight.last_circuit = not flight.pattern_work
            flight.next_at = now + 4.0
            return
        flight.phase = "landed"
        flight.next_at = now + 4.0

