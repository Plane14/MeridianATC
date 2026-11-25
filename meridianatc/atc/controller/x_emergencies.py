from __future__ import annotations

from ._shared import (
    Instruction,
    Intent,
    Phase,
    Reply,
    _airborne,
    station_key,
)


class _Emergencies:
    # ------------------------------------------------------------------
    # emergencies
    # ------------------------------------------------------------------

    # How close an aircraft in trouble has to be before the tower stops
    # talking and clears it to land.
    EMERGENCY_LANDING_NM = 25.0

    def _emergency(self, parsed, session, station, weather, p, state) -> Reply:
        """Answer a distress or urgency call, from any position.

        The first one is acknowledged and the two questions asked -- what is
        wrong, and how many people and how much fuel are on board -- because
        those are what everything after depends on. After that the answer is
        priority and a runway, and at the tower, once the aeroplane is close
        enough, it is a landing clearance.
        """
        aircraft = session.aircraft

        # Minimum fuel is an advisory, not a declaration: AIM 5-5-15 is
        # explicit that it is not an emergency and implies no traffic
        # priority. Remember it for delay handling, acknowledge it, and
        # leave the ordinary arrival/departure service running.
        if parsed.intent is Intent.MINIMUM_FUEL:
            session.minimum_fuel = True
            return Reply(p.minimum_fuel_acknowledged(aircraft), station)

        kind = parsed.intent.value
        first = not session.emergency
        session.emergency = kind

        if first:
            return Reply(
                p.emergency_acknowledged(
                    aircraft,
                    self._station_name(station, p)
                    if not session.established_with(station) else "",
                ),
                station,
            )

        runway = self._emergency_runway(session, station, weather, state)
        airport = self.navdb.airport(station.ident)
        close_in = self._distance_to(state, airport) <= self.EMERGENCY_LANDING_NM

        if station.position == "TWR" and _airborne(state) and close_in:
            session.cleared_landing = True
            session.arrival_runway = runway
            session.phase = Phase.LANDING
            session.allow_runway(runway)
            text = p.emergency_cleared_to_land(aircraft, runway, weather)
            instruction = session.remember(
                Instruction("landing_clearance", station_key(station), text,
                            {"runway": runway}))
            return Reply(text, station, instruction=instruction)

        return Reply(p.emergency_priority(aircraft, runway), station)

    def _emergency_runway(self, session, station, weather, state) -> str:
        """The runway to send somebody in trouble to: the nearest one there is."""
        airport = (self.navdb.airport(station.ident)
                   or self._nearest_airport(state) or session.destination)
        if airport is None:
            return session.arrival_runway or session.departure_runway or "01"
        return (session.arrival_runway
                or self._arrival_runway(airport, weather))

