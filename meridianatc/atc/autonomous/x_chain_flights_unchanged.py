from __future__ import annotations

from ._shared import (
    Airport,
    AutonomousState,
    Phase,
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

    def _load_next_flight(self, flight_plan: dict) -> None:
        """Load the next flight in the chain."""
        self.config.flight.departure = flight_plan["origin"]
        self.config.flight.destination = flight_plan["destination"]
        self.config.flight.cruise_altitude_ft = flight_plan["cruise_altitude_ft"]
        self.config.flight.ifr = flight_plan.get("ifr", True)

        self.session.destination = self.navdb.airport(flight_plan["destination"])
        self.session.cruise_altitude_ft = flight_plan["cruise_altitude_ft"]
        self.session.ifr = flight_plan.get("ifr", True)
        self.session.phase = Phase.PREFLIGHT

        self.session.clearance_issued = False
        self.session.clearance_read_back = False
        self.session.assigned_altitude_ft = 0.0
        self.session.assigned_heading = None
        self.session.assigned_speed_kt = None
        self.session.cleared_takeoff = False
        self.session.cleared_landing = False
        self.session.lined_up = False
        self.session.sid = ""
        self.session.descend_via = ""
        self.session.star_join = ""
        self.session.departure_runway = ""
        self.session.arrival_runway = ""
        self.session.hold_short_of = ""
        self.session.taxi_path = []
        self.session.taxi_route = []
        self.session.taxi_nodes = []
        self.session.taxi_crossings = []
        self.session.taxi_rest = []
        self.session.taxi_split_at = None
        self.session.going_around = False

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

        self.session.departure = self.navdb.airport(flight_plan["origin"])

        from ..route import from_string, resolver

        route = from_string(
            f"DCT {flight_plan['destination']}",
            flight_plan["origin"], flight_plan["destination"],
            resolver(self.navdb), cruise_ft=flight_plan["cruise_altitude_ft"])
        if self.use_route is not None:
            self.use_route(route)

        self._emit("info", f"Next flight: {flight_plan['origin']} -> {flight_plan['destination']}",
                   "autonomous.chain.next")

