from __future__ import annotations

import time
from types import SimpleNamespace

from .. import holds as hold_rules
from ._shared import (
    _CIRCUIT_PHASES,
    JOIN_REACHED_NM,
    LANDING_ROLL_S,
    Flight,
    _runway_key,
    _said_route,
    _strips_parallel,
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
                # On a STAR the check-in is answered with its descent;
                # with only an approach named it is told what to expect;
                # a field with neither answers the way it always did.
                if flight.star:
                    def controller(c, a, s):
                        return c.descend_via(a, flight.star, station=s,
                                             level_ft=flight.altitude_ft)
                elif flight.approach:
                    def controller(c, a, s):
                        return c.expect_approach(a, flight.approach,
                                                 flight.runway)
                else:
                    def controller(c, a, s):
                        return c.radar_contact(a, s)
                self._exchange(
                    flight, now, "APP",
                    pilot_first=lambda p, a, s: p.checking_in(
                        a, s, flight.altitude_ft, climbing=False),
                    controller=controller,
                )
            # The queue ahead may be longer than the runway takes. Then
            # approach holds it, the way it holds the pilot, over the last
            # fix of its arrival; and lets it go when the queue has shrunk
            # to where it fits again.
            if flight.step >= 1:
                if flight.hold is None and self._needs_the_hold(flight):
                    self._give_the_hold(flight, now)
                elif flight.hold is not None and not self._needs_the_hold(
                        flight):
                    self._leave_the_hold(flight, now)
            if (flight.distance_nm > self.HANDOFF_TO_TOWER_NM
                    or flight.hold is not None):
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

    def _arrivals_ahead(self, flight: Flight) -> int:
        """How many land before it on its strip.

        A held aeroplane keeps the place it was holding -- its distance
        hardly moves orbiting the fix, so whoever was behind it still is.
        A parallel final is another runway's queue; a crossing one is
        this one's, the strip being shared either way.
        """
        strip = _runway_key(flight.runway or self.field.arrival_runway)
        count = 0
        for other in self._approaching(ignore=flight):
            theirs = _runway_key(other.runway or self.field.arrival_runway)
            if theirs != strip and _strips_parallel(theirs, strip):
                continue
            if other.distance_nm <= flight.distance_nm:
                count += 1
        return count

    def _needs_the_hold(self, flight: Flight) -> bool:
        """Whether the queue is longer than the runway takes.

        Holds only exist for the IFR stream, and only with a margin in
        the middle -- issuing at the threshold it is released from again
        would have it told to hold and told to leave in the same breath.
        """
        if not flight.ifr or flight.phase != "inbound":
            return False
        sequence = hold_rules.sequence_hold_ahead(self.field.ident)
        ahead = self._arrivals_ahead(flight)
        if flight.hold is not None:
            return ahead > sequence - 2
        return ahead >= sequence

    def _hold_fix_for(self, flight: Flight):
        """Where it is held: the last fix of its STAR in the ring the
        books call a hold fix, else its own position -- "hold at present
        position" is a real clearance too."""
        legs = ()
        if flight.procedure_pair is not None:
            _proc, trans = flight.procedure_pair
            legs = trans.legs
        fixes = [SimpleNamespace(ident=leg.fix_ident, lat=leg.lat,
                                 lon=leg.lon, is_marker=False, kind="")
                 for leg in legs if leg.has_fix and leg.fix_ident]
        fix = hold_rules.hold_fix(fixes, (), self.field.latitude,
                                  self.field.longitude)
        if fix is None:
            fix = SimpleNamespace(ident="", lat=flight.latitude,
                                  lon=flight.longitude)
        return fix

    def _give_the_hold(self, flight: Flight, now: float) -> None:
        """The clearance, the racetrack the model flies, and the EFC."""
        fix = self._hold_fix_for(flight)
        ahead = self._arrivals_ahead(flight)
        altitude = round(float(flight.altitude_ft or 0.0), -2)
        inbound = hold_rules.inbound_course(fix.lat, fix.lon,
                                            self.field.latitude,
                                            self.field.longitude)
        legs = hold_rules.leg_minutes(altitude)
        hold = hold_rules.Hold(
            fix=fix.ident, lat=fix.lat, lon=fix.lon, inbound_true=inbound,
            side=hold_rules.side_of_fix(inbound), leg_minutes=legs,
            altitude_ft=altitude, reason="sequence",
            efc_hhmm=hold_rules.efc(
                time.time(), hold_rules.sequence_delay_min(ahead)),
            number=ahead + 1)
        flight.hold = hold
        flight.hold_leg = "in"
        flight.hold_leg_at = now
        self._exchange(
            flight, now, "APP",
            controller=lambda c, a, s: c.hold_clearance(
                a, hold.fix or "your present position", hold.side,
                int(round(inbound)) or 360, altitude, legs,
                hold.efc_hhmm, station=s),
            pilot_back=lambda p, a, s: p.wilco(a),
            command=("hold", {"fix": hold.fix}))

    def _leave_the_hold(self, flight: Flight, now: float) -> None:
        """Its slot has come: out of the racetrack and on with the
        arrival, the STAR's navigator resuming from wherever the
        pattern left it."""
        hold = flight.hold
        flight.hold = None
        flight.hold_leg = ""
        if hold is not None:
            self._exchange(
                flight, now, "APP",
                controller=lambda c, a, s: c.leave_the_hold(
                    a, hold.fix or "the field"),
                pilot_back=lambda p, a, s: p.wilco(a),
                command=("leave_hold", {}))

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

        self._hold_runway(flight.runway, now, LANDING_ROLL_S)
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

