from __future__ import annotations

from ._shared import (
    _DESTINATIONS,
    AIRLINE_RANK,
    DESTINATION_AIRPORTS,
    FREIGHT,
    LIGHT_FLEET,
    REGIONAL,
    WIDEBODY,
    Flight,
    Operator,
    fleets,
    groundwork,
    haversine_nm,
    livemod,
    perftable,
    project,
    registration_prefix,
    roster_for,
)


class _The_ground_driven:
    # -- the ground, driven ----------------------------------------------

    def _on_ground_obstacles(self, ignore: Flight | None = None,
                             with_user: bool = True
                             ) -> list[tuple[float, float, float]]:
        """Everything on the ground, as (x, z, radius) in the layout plane:
        the pilot and every other aeroplane."""
        found: list[tuple[float, float, float]] = []
        layout = self.ground
        if layout is None:
            return found
        user = self.user
        if with_user and user is not None and user.has_position                 and user.on_ground:
            found.append((*layout.to_xy(user.latitude, user.longitude), 25.0))
        for other in self.flights + self._materialising + self._asked_for:
            if other is ignore or other.done or not other.positioned:
                continue
            if not other.on_ground:
                continue
            found.append((*layout.to_xy(other.latitude, other.longitude),
                          20.0))
        return found

    def _stands_taken(self) -> set[int]:
        everyone = (self.flights + self._materialising + self._asked_for
                    + self.statics)
        return ({f.stand for f in everyone
                 if f.stand is not None and not f.done}
                | {f.planned_stand for f in everyone
                   if f.planned_stand is not None and not f.done})

    def _free_stand(self, airliner: bool, also=(), prefer: int | None = None):
        """A free stand, chosen at random -- or ``prefer``, the one an
        arrival has held since it was invented, while it is still free."""
        occupied = self._on_ground_obstacles() + list(also) + [
            (*self.ground.to_xy(lat, lon), 20.0)
            for lat, lon in self._parked_seen]
        taken = self._stands_taken()
        choose = self.random.choice
        if prefer is not None:
            taken.discard(prefer)

            def choose(options):
                for option in options:
                    if option.index == prefer:
                        return option
                return self.random.choice(options)
        return self.ground.free_stand(airliner, occupied, taken, choose=choose)

    def ground_occupancy(self, layout: groundwork.GroundLayout
                         ) -> tuple[list[tuple[float, float, float]], set[int]]:
        """Where every other aeroplane is on ``layout``'s ground, for the
        pilot's own taxi (:mod:`meridianatc.atc.taxi`).

        (x, z, radius) for each aeroplane on the ground -- the ones driven
        here and the ones somebody else put in the simulator -- and the stands
        handed out to the traffic, which a stand number only means anything
        on the layout the traffic is using. The pilot is not in it: they are
        the one being taxied.
        """
        if layout is None or self.ground is not layout:
            return [], set()
        occupied = self._on_ground_obstacles(with_user=False) + [
            (*layout.to_xy(lat, lon), 20.0) for lat, lon in self._parked_seen]
        return occupied, self._stands_taken()

    def _park(self, flight: Flight, stand) -> None:
        flight.on_layout = True
        flight.stand = stand.index
        flight.latitude, flight.longitude = self.ground.to_latlon(stand.x,
                                                                  stand.z)
        flight.heading = stand.heading
        flight.altitude_ft = self.field.elevation_ft
        flight.ground_speed_kt = 0.0
        flight.on_ground = True

    def _airline_flight(self, cargo: bool | None = None
                        ) -> tuple[str, str, Operator] | None:
        """An airline's flight. ``cargo`` True or False keeps to freighters or
        to the others, where real traffic has filled the other category;
        a field with none of the kind asked for draws from all of them."""
        candidates = roster_for(self.field.ident, self.field.size_rank)
        if cargo is not None:
            kept = [c for c in candidates
                    if (c.operator.fleet == livemod.FREIGHT) == cargo]
            candidates = kept or candidates
        if not candidates:
            return None
        operator = self.random.choices(
            [c.operator for c in candidates],
            weights=[c.weight for c in candidates],
        )[0]
        low, high = operator.number_range
        # An aeroplane the runway cannot take is never drawn however the
        # field is classified: the scheduled-service flag it was believed
        # on is sometimes years stale (fleet.fits_runway).
        runway_m = (self.field.kind.runway_m if self.field.kind is not None
                    else fleets.longest_runway_m(self.navdb,
                                                 self.field.ident))
        fleet = tuple(t for t in operator.fleet
                      if fleets.fits_runway(t, runway_m))
        if not fleet:
            return None
        if self.has_livery is not None:
            # Only where there is a painted one to choose: an airline with
            # none keeps its whole fleet rather than having no aeroplanes.
            fleet = tuple(t for t in fleet
                          if self.has_livery(operator.icao, t)) or fleet
        for _ in range(20):
            written = f"{operator.icao}{self.random.randint(low, high)}"
            if written not in self._used_callsigns:
                return written, self.random.choice(fleet), operator
        return None

    def _light_flight(self, airline_fleet: bool = False
                      ) -> tuple[str, str, None] | None:
        """A registration of the shape used where this airport is.

        Every national scheme here is five characters once the dash is taken
        out -- F-GXYZ, G-ABCD, D-EFGH, HB-ABC -- so the prefix decides how many
        letters are still to be drawn. The US is the exception and is numeric.

        ``airline_fleet`` gives it an airliner instead of a light single,
        which is what a large field gets when the pilot has asked for
        registrations rather than telephony names. The registration is the
        thing being chosen here; the aeroplane under it should still be the
        one the airport would have.
        """
        prefix = registration_prefix(self.field.ident)
        letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        needed = 5 - len(prefix.replace("-", ""))
        kind = self.field.kind or fleets.classify(self.navdb, self.field.ident)
        airliners = self._airline_fleet() if airline_fleet else ()
        for _ in range(20):
            if prefix == "N":
                tail = (f"N{self.random.randint(100, 999)}"
                        f"{self.random.choice(letters)}"
                        f"{self.random.choice(letters)}")
            else:
                tail = prefix + "".join(
                    self.random.choice(letters) for _ in range(needed))
            if tail not in self._used_callsigns:
                type_code = (self.random.choice(airliners) if airliners
                             else fleets.light_type(kind, self.random))
                return tail, type_code, None
        return None

    @property
    def _airline_field(self) -> bool:
        kind = self.field.kind
        if kind is None:
            return self.field.size_rank >= AIRLINE_RANK
        return kind.airline

    def _nearby_field(self) -> str:
        """A field a light aeroplane might have come from or be going to:
        one with a runway, ten to eighty miles away. Empty with none."""
        if not (self.field.latitude or self.field.longitude):
            return ""
        cached = getattr(self, "_nearby_cache", None)
        if cached is None or cached[0] != self.field.ident:
            try:
                found = self.navdb.nearest_airports(
                    self.field.latitude, self.field.longitude, limit=40,
                    max_nm=80.0, min_rank=1, require_runway=True)
            except Exception:
                found = []
            names = [a.spoken or a.name for a, d in found
                     if a.ident != self.field.ident and d >= 10.0]
            cached = (self.field.ident, names)
            self._nearby_cache = cached
        return self.random.choice(cached[1]) if cached[1] else ""

    def _airline_fleet(self) -> tuple[str, ...]:
        """The types the operators at this field actually fly.

        Taken from the roster rather than written down again, so a field that
        only sees regional jets does not suddenly produce a 777 because the
        pilot asked for registrations.
        """
        runway_m = (self.field.kind.runway_m if self.field.kind is not None
                    else fleets.longest_runway_m(self.navdb,
                                                 self.field.ident))
        seen: list[str] = []
        for candidate in roster_for(self.field.ident, self.field.size_rank):
            for type_code in candidate.operator.fleet:
                if (type_code not in seen
                        and fleets.fits_runway(type_code, runway_m)):
                    seen.append(type_code)
        return tuple(seen) or LIGHT_FLEET

    def _elsewhere(self) -> str:
        """Somewhere plausible to be going to or coming from."""
        return self.random.choice(_DESTINATIONS)

    def _field_airport(self):
        """This field as a navdata airport, or None."""
        if not self.field.ident:
            return None
        try:
            return self.navdb.airport(self.field.ident)
        except Exception:
            return None

    def _route_end(self, operator: Operator | None, type_code: str,
                   range_nm: float) -> tuple[str, str]:
        """The far end of an IFR flight's route: (spoken name, ICAO).

        Chosen from the navdata inside what the aeroplane can fly and what
        the operator actually flies -- a SkyWest regional jet does not file
        to Warsaw -- with the airport's scheduled service, size and the
        operator's own region biasing the draw. Only when the field is
        somewhere the database has no neighbours does the old global list
        answer, so a frequency never goes quiet for want of a route.
        """
        home = self._field_airport()
        if home is None or not home.lat:
            name = self._elsewhere()
            return name, DESTINATION_AIRPORTS.get(name, "")
        reach = range_nm if range_nm > 0 else perftable.range_for(type_code)
        cap = reach * 0.75
        low = 30.0
        ideal = cap * 0.45
        prefixes: tuple[str, ...] = ()
        regional = False
        if operator is not None:
            prefixes = operator.home + operator.region
            fleet = set(operator.fleet)
            regional = bool(fleet) and fleet <= set(REGIONAL)
            heavy = bool(fleet & (set(WIDEBODY) | set(FREIGHT)))
            if operator.long_haul:
                ideal = min(cap * 0.6, reach * 0.5)
                if heavy:
                    low = 200.0
            elif regional:
                # A regional carrier's legs are its feed: a couple of
                # hundred miles, not an ocean.
                cap = min(cap, 800.0)
                ideal = 350.0
                low = 15.0
            elif operator.leisure or operator.charter:
                cap = min(cap, 1800.0)
                ideal = 800.0
            elif heavy:
                low = 200.0
                ideal = min(cap * 0.5, 1600.0)
        # Reach for points across the whole leg, not just the nearest
        # fields: a widebody's destination is hundreds of miles away, and
        # a nearest-first query never sees it. Each try picks a bearing
        # and a distance biased towards the ideal leg, then asks what
        # scheduled airports are near that point.
        min_rank = 1 if regional else 2
        seen: dict[str, object] = {}
        for _ in range(14):
            bearing = self.random.uniform(0.0, 360.0)
            dist = max(low, min(cap, ideal
                                 + (self.random.random() - 0.5) * cap))
            lat, lon = project(home.lat, home.lon, bearing, dist)
            try:
                near = self.navdb.nearest_airports(
                    lat, lon, limit=6, max_nm=150.0, min_rank=min_rank,
                    require_runway=True)
            except Exception:
                continue
            for airport, _d in near:
                if airport.ident == home.ident or airport.ident in seen:
                    continue
                if operator is not None and not airport.scheduled:
                    continue
                seen[airport.ident] = airport
        weighted: list[tuple[float, object]] = []
        for airport in seen.values():
            true_dist = haversine_nm(home.lat, home.lon,
                                     airport.lat, airport.lon)
            if true_dist < low:
                continue
            weight = 1.0 / (1.0 + abs(true_dist - ideal) / max(cap, 1.0))
            weight *= 0.5 + 0.5 * airport.size_rank
            if airport.scheduled:
                weight *= 1.5
            if prefixes and airport.ident.startswith(prefixes):
                weight *= 1.4
            weighted.append((weight, airport))
        if not weighted:
            name = self._elsewhere()
            return name, DESTINATION_AIRPORTS.get(name, "")
        pick = self.random.choices(
            [a for _, a in weighted], weights=[w for w, _ in weighted],
            k=1)[0]
        return pick.spoken or pick.name, pick.ident

    def _assign_route(self, flight: Flight, arriving: bool) -> None:
        """Give an IFR flight a plausible origin and destination.

        This field is one end -- the destination of an arrival, the origin
        of a departure -- and the other is what :meth:`_route_end` picks.
        The spoken names go on the radio calls; the ICAO pair is what the
        map's route line is drawn to.
        """
        home = self._field_airport()
        field_name = ((home.spoken or home.name) if home is not None
                      else self.field.ident or "")
        field_icao = home.ident if home is not None else ""
        far_name, far_icao = self._route_end(
            flight.operator, flight.type_code,
            getattr(flight.performance, "range_nm", 0.0) or 0.0)
        if arriving:
            flight.origin, flight.origin_icao = far_name, far_icao
            flight.destination, flight.destination_icao = field_name, \
                field_icao
        else:
            flight.origin, flight.origin_icao = field_name, field_icao
            flight.destination, flight.destination_icao = far_name, far_icao

