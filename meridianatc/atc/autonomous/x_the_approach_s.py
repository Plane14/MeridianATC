from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    auto_nav,
)


class _The_approach_s:
    # -- the approach's own pieces --------------------------------------------

    def _find_runway(self, state) -> None:
        """The departure/arrival runway's threshold and heading, once."""
        # The field the leg is bound for: the destination's once the
        # aeroplane is on the way in -- the departure's geometry is no
        # use to an approach, whatever runway the tower named -- and the
        # departure's until then.
        arrival = self.state.phase in (
            AutonomousPhase.APPROACH, AutonomousPhase.LANDING,
            AutonomousPhase.TAXI_IN)
        airport = (self.session.destination if arrival
                   else (getattr(self.session, "departure", None)
                         or self.session.destination))
        if airport is None:
            state_now = self.get_state()
            if state_now is not None and getattr(state_now, "has_position", False):
                airport = self.navdb.home_airport(state_now.latitude,
                                                  state_now.longitude)
        if airport is None:
            self.state.runway_heading = 0.0
            self.state.runway_key = ()
            return
        # The runway named for this end of the flight is the answer:
        # the arrival's for an aeroplane coming in, the departure's
        # until then -- a departure asking after the destination's
        # runway either finds nothing or, worse, a same-numbered one
        # pointed the wrong way.
        name = ((self.session.arrival_runway if arrival
                 else self.session.departure_runway) or "")
        key = (airport.ident, name, arrival)
        if self.state.runway_heading > 0 and self.state.runway_key == key:
            return
        self.state.runway_heading = 0.0
        self.state.runway_key = ()
        runways = list(self.navdb.runways(airport.ident) or [])
        if not name and runways:
            # The tower's choice stands in session; the longest runway is
            # the honest guess when nothing was said. A name that was
            # said and does not resolve stays unresolved -- guessing a
            # heading is how an aeroplane lands on the reciprocal.
            longest = max(runways, key=lambda r: r.length_ft or 0)
            name = getattr(longest, "le_ident", "") or ""
        found = auto_nav.runway_end(runways, name)
        if found is None:
            return
        self.state.runway_lat, self.state.runway_lon, \
            self.state.runway_heading, self.state.runway_length_ft = found
        self.state.runway_heading = self.state.runway_heading or 360.0
        self.state.runway_key = key

