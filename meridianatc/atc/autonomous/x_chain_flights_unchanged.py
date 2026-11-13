from __future__ import annotations

import dataclasses

from ._shared import (
    Airport,
    AutonomousState,
    fly,
    haversine_nm,
    log,
    random,
)


class _Chain_flights_unchanged:
    # -- chain flights (unchanged behaviour) -----------------------------------

    def _select_next_destination(self) -> None:
        """Select a random destination within aircraft range with ground services."""
        current = self._get_current_airport() or self.session.departure
        if not current:
            log.warning("Cannot select next destination: no current airport")
            return

        candidates = self._find_suitable_destinations(current)
        if not candidates:
            log.warning("No suitable destinations within range of %s", current.ident)
            return

        destination = self._pick_destination(candidates)
        if not destination:
            return

        flight_plan = self._create_flight_plan(current, destination)
        self.state.chain_flights.append(flight_plan)
        log.info("Chained flight: %s -> %s (%d NM)", current.ident, destination.ident,
                 flight_plan.get("distance_nm", 0))

    def _get_current_airport(self) -> Airport | None:
        """Get the airport the aircraft is currently at."""
        state = self.get_state()
        if state and state.has_position:
            return self.navdb.home_airport(state.latitude, state.longitude)
        return self.session.destination or self.session.departure

    def _find_suitable_destinations(self, origin: Airport) -> list[tuple[Airport, float]]:
        """Find airports within range that can host the aircraft with ground services."""
        candidates = []
        max_range = self.state.aircraft_range_nm * 0.8

        for airport, distance in self.navdb.nearest_airports(
                origin.lat, origin.lon, limit=5000, max_nm=max_range):
            if airport.ident == origin.ident:
                continue
            if not self._airport_suitable(airport):
                continue
            if not self._has_ground_services(airport):
                continue
            candidates.append((airport, distance))

        candidates.sort(key=lambda x: abs(x[1] - max_range * 0.5))
        return candidates[:20]

    def _airport_suitable(self, airport: Airport) -> bool:
        """Check if airport can handle the aircraft type."""
        from .. import aircraft_class as ac

        return ac.airport_usable(
            airport, self.navdb.runways(airport.ident),
            self.state.aircraft_type, helicopter=self._is_helicopter)

    def _has_ground_services(self, airport: Airport) -> bool:
        """Check if airport has ground services (fuel, parking, handling)."""
        if self._is_helicopter:
            return airport.type in ("heliport", "small_airport",
                                    "medium_airport", "large_airport")
        if airport.type in ("large_airport", "medium_airport"):
            return True
        if self.state.aircraft_type in fly._LIGHT_TYPES:
            return airport.type in ("small_airport", "medium_airport", "large_airport")
        return False

    def _pick_destination(self, candidates: list[tuple[Airport, float]]) -> Airport | None:
        """Pick a destination from candidates with some randomness."""
        if not candidates:
            return None
        max_range = self.state.aircraft_range_nm * 0.8
        weights = []
        for _, dist in candidates:
            ideal = max_range * 0.5
            diff = abs(dist - ideal) / max_range
            weight = 1.0 / (1.0 + diff * 2.0)
            weights.append(weight)
        total = sum(weights)
        weights = [w / total for w in weights]
        return random.choices([c[0] for c in candidates], weights=weights, k=1)[0]

    def _create_flight_plan(self, origin: Airport, destination: Airport) -> dict:
        """Create a flight plan dictionary for the leg."""
        distance = haversine_nm(origin.lat, origin.lon,
                                destination.lat, destination.lon)

        if distance < 200:
            cruise_ft = 15000
        elif distance < 500:
            cruise_ft = 25000
        elif distance < 1000:
            cruise_ft = 35000
        else:
            cruise_ft = 39000

        if self._is_helicopter:
            cruise_ft = min(cruise_ft, 5000)
        elif self.state.aircraft_type in ("TBM9", "PC12", "C25A", "C25C", "AT76", "DH8D"):
            cruise_ft = min(cruise_ft, 28000)
        elif self.state.aircraft_type in fly._LIGHT_TYPES:
            cruise_ft = min(cruise_ft, 12000)

        return {
            "origin": origin.ident,
            "destination": destination.ident,
            "origin_name": origin.name,
            "destination_name": destination.name,
            "distance_nm": round(distance),
            "cruise_altitude_ft": cruise_ft,
            "ifr": not self._is_helicopter,
            "aircraft_type": self.state.aircraft_type,
        }

    # What survives a leg change on the session: who is flying, where the
    # leg is going (set from the plan below), the rules it was filed
    # under, and the transcript log, which is the pane's history of the
    # whole day rather than of one leg. Everything else the dataclass
    # declares is per-flight: leg one's readback owed, squawk, QNH,
    # contacts made and advisories said do not belong to leg two.
    _SESSION_KEPT = {
        "aircraft", "departure", "destination", "destination_input",
        "cruise_altitude_ft", "ifr", "ifr_setting", "profile", "log",
    }

    def _reset_session_for_leg(self, flight_plan: dict) -> None:
        """A fresh flight on the same session object.

        The session is shared with the controllers, so it cannot be
        replaced outright the way ``self.state`` is below -- the fields
        are walked off the dataclass instead and each per-flight one put
        back to its declared default. A field list written by hand grew
        long and still missed the ones added later, and every miss was
        leg one flown into leg two.
        """
        session = self.session
        for f in dataclasses.fields(session):
            if f.name in self._SESSION_KEPT:
                continue
            if f.default is not dataclasses.MISSING:
                setattr(session, f.name, f.default)
            elif f.default_factory is not dataclasses.MISSING:
                setattr(session, f.name, f.default_factory())
        session.destination = self.navdb.airport(flight_plan["destination"])
        session.destination_input = flight_plan["destination"]
        session.departure = self.navdb.airport(flight_plan["origin"])
        session.cruise_altitude_ft = flight_plan["cruise_altitude_ft"]
        session.ifr = flight_plan.get("ifr", True)
        session.ifr_setting = session.ifr

    def _load_next_flight(self, flight_plan: dict) -> None:
        """Load the next flight in the chain."""
        self.config.flight.departure = flight_plan["origin"]
        self.config.flight.destination = flight_plan["destination"]
        self.config.flight.cruise_altitude_ft = flight_plan["cruise_altitude_ft"]
        self.config.flight.ifr = flight_plan.get("ifr", True)

        self._reset_session_for_leg(flight_plan)

        # A fresh state for the leg: the list of fields one by one grew
        # long and still missed the ones added later -- a go-around's
        # clock, the weather holds, the gear-up flags -- and each miss
        # is the last leg flown into this one. What survives is the
        # chain's own bookkeeping and the aircraft's figures.
        chain_flights = self.state.chain_flights
        chain_index = self.state.chain_flight_index
        aircraft_type = self.state.aircraft_type
        aircraft_range_nm = self.state.aircraft_range_nm
        self.state = AutonomousState()
        self.state.chain_flights = chain_flights
        self.state.chain_flight_index = chain_index
        self.state.aircraft_type = aircraft_type
        self.state.aircraft_range_nm = aircraft_range_nm
        self._reset_scratch()
        if self._dir is not None:
            self._dir.reset()
        self._speeds = None
        self._perf = None

        # The crew is re-begun for the leg the way a new session would
        # begin it: its once-per-flight bookkeeping -- the said-set, the
        # callouts, the checklist runner's offered and done -- is leg
        # one's otherwise, and leg two flies silent.
        self.crew.begin(
            self.session,
            getattr(self.crew, "_language_heard", "en") or "en")

        from ..route import from_string, resolver

        route = from_string(
            f"DCT {flight_plan['destination']}",
            flight_plan["origin"], flight_plan["destination"],
            resolver(self.navdb), cruise_ft=flight_plan["cruise_altitude_ft"])
        if self.use_route is not None:
            self.use_route(route)

        self._emit("info", f"Next flight: {flight_plan['origin']} -> {flight_plan['destination']}",
                   "autonomous.chain.next")

