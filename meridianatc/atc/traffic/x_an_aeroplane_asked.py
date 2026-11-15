from __future__ import annotations

from ._shared import (
    Flight,
    groundwork,
)


class _An_aeroplane_asked:
    # ------------------------------------------------------------------
    # an aeroplane asked for
    # ------------------------------------------------------------------

    def spawn_on_runway(self, now: float) -> tuple[Flight | None, str]:
        """Put a departure at the runway in use, for the panel's button.

        An ordinary flight from the field's own roster that starts at the
        last step before the roll, so from here the tower works it like any
        other: cleared for takeoff when the runway and the approach allow it,
        handed to departure once it is climbing.

        Lined up when the runway really is free, and at the holding point
        when it is not. The first version refused instead, and a refusal
        reads as the program being wrong: the pilot looks at an empty
        runway -- the aeroplane in the way may be one the simulator never
        drew -- and is told it is occupied. A busy runway is a queue at a
        real field, not a reason to have no aeroplane. And lining one up
        with an arrival inside :attr:`LINEUP_BLOCK_NM` is the mistake the
        tower itself is written not to make: it sent the arrival around.

        Returns the flight and an empty reason, or None and the reason, as a
        key the panel can translate. Whether it was lined up is its phase.

        The flight waits in ``_asked_for`` until the next tick moves it onto
        the roster. The panel calls this from a different thread from the
        one running the traffic, and the tick rebuilds ``flights`` whole, so
        an aeroplane appended to it from here could vanish in between. Where
        the simulator has to draw it first (``requires_sim``), it goes from
        there to waiting for that, like any other invented aeroplane.
        """
        if not self.field.ident:
            return None, "no_field"
        if self.requires_sim and not self.can_invent:
            # Nothing to put it in. An aeroplane on the radio and not on the
            # runway is exactly what the pilot asked this never to do.
            return None, "not_in_sim"
        ends = getattr(self.field, "departure_runways", None) \
            or [self.field.departure_runway]
        runway = next((e for e in ends if self._threshold(e) is not None),
                      None)
        if runway is None:
            return None, "no_runway"
        free = not (any(f.phase == "lineup"
                        for f in self._asked_for + self._materialising)
                    or self._somebody_lined_up(runway=runway)
                    or self.runway_occupied(now, runway=runway)
                    or self._traffic_on_final(self.LINEUP_BLOCK_NM,
                                              runway=runway)
                    is not None)
        flight = self._new_flight(now, arriving=False, parked=False)
        if flight is None:
            return None, "no_callsign"
        if not flight.ifr and not self._vmc():
            # The pilot asked for a departure and the field is in the
            # clouds: what launches is on a clearance, not circuits.
            flight.ifr = True
            flight.pattern_work = False
            flight.squawk = self._new_squawk()
            self._assign_route(flight, arriving=False)
        flight.sequence = 0
        legs = self._legs_from_the_ramp(flight)
        if not free:
            # Asked for the runway already; the tower answers it the way it
            # answers every other departure at the holding point.
            flight.phase = "waiting"
            flight.step = 0
            flight.next_at = now + self.random.uniform(3.0, 6.0)
            if legs is not None:
                # At the airport's own holding point, facing the runway.
                out, onto = legs
                self._put(flight, out.points[-1],
                          groundwork._heading(onto.points[1][0]
                                              - out.points[-1][0],
                                              onto.points[1][1]
                                              - out.points[-1][1]))
                flight.line_up = onto
                onto.index = 1
            else:
                self._place(flight)
            self._asked_for.append(flight)
            return flight, ""
        if legs is not None:
            # On the runway where a real line-up ends, not at a point worked
            # out from the threshold.
            _out, onto = legs
            self._put(flight, onto.points[-1], onto.face)
        flight.phase = "lineup"
        flight.step = 2
        # Recorded as already said and done, so the strip reads "lined up"
        # and the tower does not tell it to line up a second time.
        lined_up = self._tell(flight, "line_up", now, runway=flight.runway)
        lined_up.applied = True
        flight.doing = lined_up
        self._start_flying(flight)
        # Long enough for the pilot to find it out of the window before it
        # asks to go.
        flight.next_at = now + self.random.uniform(8.0, 15.0)
        self._asked_for.append(flight)
        return flight, ""

