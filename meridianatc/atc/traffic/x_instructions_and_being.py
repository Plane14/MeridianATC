from __future__ import annotations

from ._shared import (
    OWN_DECISIONS,
    RUNWAY_ROLL_S,
    Command,
    Flight,
    bearing_deg,
)


class _Instructions_and_being:
    # ------------------------------------------------------------------
    # instructions, and being obeyed
    # ------------------------------------------------------------------

    def _tell(self, flight: Flight, kind: str, at: float, **values) -> Command:
        """Record an instruction, to take effect when the readback ends.

        Nothing here says it -- the words are the phraseology's job and are
        queued alongside. This is the other half: what the aeroplane will
        actually do about it, and when.
        """
        command = Command(kind=kind, at=at, values=values,
                          shown=(OWN_DECISIONS.get(kind, "")
                                 if self.uncontrolled else ""))
        flight.commands.append(command)
        # Only the last few are kept. A strip shows what an aeroplane is
        # doing, not everything it has ever been told.
        if len(flight.commands) > 8:
            del flight.commands[:-8]
        return command

    def _already_told(self, flight: Flight, kind: str) -> bool:
        """Whether this aeroplane is already dealing with an instruction.

        An instruction takes a few seconds to be read back, and the tower
        looks at the field every second. Without this, an aeroplane on short
        final to a blocked runway is told to go around six times before the
        first one takes effect -- and goes around six times.
        """
        if flight.doing is not None and flight.doing.kind == kind:
            return True
        return any(command.kind == kind and not command.applied
                   for command in flight.commands)

    def _obey(self, now: float) -> None:
        """Do what has been read back and is now due."""
        for flight in self.flights:
            if flight.done:
                continue
            for command in flight.commands:
                if command.applied or command.at > now:
                    continue
                command.applied = True
                flight.doing = command
                self._carry_out(flight, command, now)

    def _carry_out(self, flight: Flight, command: Command, now: float) -> None:
        """One instruction, turned into something the aeroplane is doing."""
        kind = command.kind
        values = command.values

        if kind in ("line_up", "takeoff", "touch_and_go"):
            # Released: no longer waiting, and no longer a reason for the
            # tower to hold the arrivals off.
            flight.held_since = 0.0
        if kind == "touch_and_go" and flight.arriving:
            # Cleared for the option on base or final: the clearance is used
            # on the runway, by the touchdown in _work_the_arrival, which is
            # where a circuit becomes a takeoff. Taken as a takeoff here, it
            # put an arrival two miles out into a phase nothing on the
            # arrival side moves on from, and it flew the runway heading for
            # the rest of the session (Centennial, 26 October: five at once).
            return
        if kind == "taxi" and flight.on_layout and flight.planned is not None:
            # The taxi clearance has been read back: off the stand and on
            # the way to the holding point.
            flight.taxi, flight.line_up = flight.planned
            flight.planned = None
            flight.stand = None
        elif kind in ("line_up", "takeoff", "touch_and_go") \
                and flight.on_layout and flight.line_up is not None:
            # Driven onto the runway rather than put there; the roll starts
            # when it is lined up (see _fly).
            if flight.taxi is None:
                flight.taxi = flight.line_up
            flight.phase = "lineup" if kind == "line_up" else "takeoff"
            if kind != "line_up":
                flight.assigned_heading = None
                flight.assigned_altitude_ft = None
                flight.assigned_speed_kt = None
                self.runway_free_at = max(self.runway_free_at,
                                          now + RUNWAY_ROLL_S + 30.0)
        elif kind == "line_up":
            if not flight.airborne_model:
                self._start_flying(flight)
            flight.phase = "lineup"
        elif kind in ("takeoff", "touch_and_go"):
            if not flight.airborne_model:
                self._start_flying(flight)
            flight.phase = "takeoff"
            flight.assigned_heading = None
            flight.assigned_altitude_ft = None
            flight.assigned_speed_kt = None
            self.runway_free_at = max(self.runway_free_at, now + RUNWAY_ROLL_S)
        elif kind == "go_around":
            # The one instruction that has to work the instant it is read
            # back: the aeroplane stops descending, climbs away on the runway
            # heading, and rejoins the circuit for another go.
            flight.phase = "go_around"
            flight.arriving = False
            flight.extending = False
            flight.go_arounds += 1
            flight.step = 0
            flight.assigned_heading = None
            flight.assigned_altitude_ft = None
            flight.assigned_speed_kt = None
            flight.next_at = now + self.random.uniform(20.0, 35.0)
        elif kind == "turn_out":
            # "Turn out approved" is a heading: at the field it filed
            # for where the navdata can point at it, and off to the
            # circuit's side where it cannot -- so the departure
            # visibly goes its own way rather than holding the runway
            # heading until it is out of sight.
            airport = (self.navdb.airport(flight.destination_icao)
                       if flight.destination_icao else None)
            if airport is not None:
                flight.assigned_heading = bearing_deg(
                    flight.latitude, flight.longitude,
                    airport.lat, airport.lon)
            else:
                side = (-1.0 if flight.pattern_direction != "right"
                        else 1.0)
                end = self._threshold(flight.runway)
                base = end[2] if end is not None else flight.heading
                flight.assigned_heading = (
                    base + side * self.random.uniform(25.0, 55.0)) % 360.0
        elif kind == "extend":
            flight.extending = True
        elif kind == "continue":
            flight.extending = False
        elif kind == "heading":
            flight.assigned_heading = float(values.get("heading", flight.heading))
            flight.target.turn = values.get("turn", "")
        elif kind == "altitude":
            flight.assigned_altitude_ft = float(values.get("altitude_ft", 0.0))
        elif kind == "speed":
            flight.assigned_speed_kt = float(values.get("speed_kt", 0.0))
        elif kind == "resume":
            flight.assigned_heading = None
            flight.assigned_speed_kt = None
        elif kind == "hold_short":
            flight.target.hold = True

