from __future__ import annotations

from ._shared import (
    _CIRCUIT_PHASES,
    CIRCUIT_UPWIND_NM,
    CIRCUIT_WIDTH_NM,
    CLIMB_TO_FT,
    DESTINATION_AIRPORTS,
    GLIDE_FT_PER_NM,
    INITIAL_CLIMB_FT,
    RUNWAY_ROLL_S,
    Flight,
    bearing_deg,
    glide_altitude,
    log,
    make_of,
    project,
)


class _Where_it_is:
    # -- where it is going, for the map ------------------------------------

    # How far ahead a path on the map runs once an aeroplane is in the air
    # and has nothing nearer to aim at.
    PATH_OUT_NM = 25.0

    def route_ahead(self, flight: Flight) -> dict:
        """The rest of this aeroplane's way, as the map draws it.

        ``path`` is points of [latitude, longitude, altitude in feet] from
        where it is to where it is going: the taxi route it is on, the
        runway, the climb out towards its destination; or the approach, the
        runway and the stand. ``waypoints`` names the ones worth naming, as
        a kind and a value the panel puts into words. Nothing is invented
        for an aeroplane somebody else is flying: the simulator does not say
        where FSLTL's are going, and a line to nowhere would be a guess.
        """
        extra = {
            "origin": flight.origin, "destination": flight.destination,
            "origin_icao": flight.origin_icao or DESTINATION_AIRPORTS.get(
                flight.origin, ""),
            "destination_icao": flight.destination_icao
            or DESTINATION_AIRPORTS.get(flight.destination, ""),
            "squawk": flight.squawk,
            "make": make_of(flight.type_code),
            "airline": (flight.operator.icao if flight.operator else ""),
            "telephony": flight.aircraft.for_language("en").spoken_full,
            "stand": (self.ground.stands[flight.stand].number
                      if self.ground is not None and flight.stand is not None
                      and flight.stand < len(self.ground.stands) else None),
            "path": [], "waypoints": [],
        }
        if flight.observed:
            return extra
        try:
            path, waypoints = self._path(flight)
        except Exception:
            log.debug("could not work out %s's path", flight.written,
                      exc_info=True)
            return extra
        extra["path"] = [[round(lat, 6), round(lon, 6), round(alt)]
                         for lat, lon, alt in path]
        extra["waypoints"] = waypoints
        return extra

    def _path(self, flight: Flight):
        elevation = self.field.elevation_ft
        here = (flight.latitude, flight.longitude, flight.altitude_ft)
        path = [here]
        waypoints: list[dict] = []
        end = self._threshold(flight.runway)

        def mark(kind: str, value: str, lat: float, lon: float) -> None:
            waypoints.append({"kind": kind, "value": value,
                              "lat": round(lat, 6), "lon": round(lon, 6)})

        def along(taxi) -> None:
            if taxi is None or self.ground is None:
                return
            for x, z in taxi.points[max(1, taxi.index):]:
                lat, lon = self.ground.to_latlon(x, z)
                path.append((lat, lon, elevation))

        def out_of_the_circuit(lat: float, lon: float, alt: float,
                               heading: float) -> None:
            """Along the runway heading and then on course, climbing."""
            lat2, lon2 = project(lat, lon, heading, 6.0)
            path.append((lat2, lon2, max(alt, elevation + INITIAL_CLIMB_FT)))
            course = heading
            target = (flight.destination_icao
                      or DESTINATION_AIRPORTS.get(flight.destination))
            airport = self.navdb.airport(target) if target else None
            if airport is not None:
                course = bearing_deg(lat2, lon2, airport.lat, airport.lon)
            lat3, lon3 = project(lat2, lon2, course, self.PATH_OUT_NM)
            path.append((lat3, lon3, elevation + CLIMB_TO_FT))
            mark("destination", target or flight.destination, lat3, lon3)

        departing = not flight.arriving and flight.phase not in _CIRCUIT_PHASES
        if departing and flight.phase in ("clearance", "taxi", "ready",
                                          "waiting", "holding", "lineup",
                                          "takeoff") and not (
                flight.phase == "takeoff" and not flight.on_ground):
            planned = flight.planned
            to_hold, onto = ((planned[0], planned[1]) if planned is not None
                             else (flight.taxi, flight.line_up))
            if to_hold is not onto:
                along(to_hold)
                if self.ground is not None and len(path) > 1 \
                        and flight.phase in ("clearance", "taxi", "ready"):
                    # The holding point: the end of the taxi out, before the
                    # line-up leg onto the runway.
                    mark("hold", flight.runway, path[-1][0], path[-1][1])
            along(onto)
            if end is not None:
                lat, lon, heading = end
                roll_from = path[-1]
                lift = project(roll_from[0], roll_from[1], heading, 1.2)
                path.append((lift[0], lift[1], elevation + 400.0))
                mark("runway", flight.runway, roll_from[0], roll_from[1])
                out_of_the_circuit(lift[0], lift[1], elevation + 400.0,
                                   heading)
            return path, waypoints

        if flight.phase in ("takeoff", "departed", "go_around") \
                and not flight.arriving and end is not None:
            out_of_the_circuit(flight.latitude, flight.longitude,
                               flight.altitude_ft, end[2])
            return path, waypoints

        if flight.phase in _CIRCUIT_PHASES and end is not None:
            lat, lon, heading = end
            side = -1.0 if flight.pattern_direction != "right" else 1.0
            base = self._base_nm(flight)
            wide = (heading + 90.0 * side) % 360.0
            legs = {
                "upwind": project(lat, lon, heading, CIRCUIT_UPWIND_NM),
                "crosswind": project(*project(lat, lon, heading,
                                              CIRCUIT_UPWIND_NM), wide,
                                     CIRCUIT_WIDTH_NM),
                "downwind": project(*project(lat, lon, heading, -base), wide,
                                    CIRCUIT_WIDTH_NM),
                "base": project(lat, lon, heading, -base),
            }
            order = ["upwind", "crosswind", "downwind", "base"]
            pattern = elevation + flight.performance.pattern_agl_ft
            for leg in order[order.index(flight.phase):]:
                point = legs[leg]
                path.append((point[0], point[1], pattern))
                mark("leg", leg, point[0], point[1])
            path.append((lat, lon, elevation))
            mark("threshold", flight.runway, lat, lon)
            return path, waypoints

        if flight.arriving and end is not None and flight.phase in (
                "inbound", "joining", "final", "landed"):
            lat, lon, heading = end
            if flight.phase != "landed":
                behind = project(lat, lon, (heading + 180.0) % 360.0,
                                 min(6.0, max(0.0, flight.distance_nm)))
                path.append((behind[0], behind[1],
                             glide_altitude(elevation, min(
                                 6.0, max(0.0, flight.distance_nm)),
                                 GLIDE_FT_PER_NM)))
                path.append((lat, lon, elevation + 50.0))
                mark("threshold", flight.runway, lat, lon)
            rollout = project(lat, lon, heading, 0.8)
            path.append((rollout[0], rollout[1], elevation))
            along(flight.taxi)
            return path, waypoints

        if flight.phase == "taxi_in":
            along(flight.taxi)
        if flight.stand is not None and self.ground is not None \
                and flight.stand < len(self.ground.stands):
            stand = self.ground.stands[flight.stand]
            lat, lon = self.ground.to_latlon(stand.x, stand.z)
            mark("stand", str(stand.number), lat, lon)
        return path, waypoints

    def _plan_departure(self, flight: Flight):
        """The way from where it is to the holding point, and onto the
        runway: two legs, because they are two clearances."""
        runway = self._runway_xy(flight.runway)
        if runway is None:
            return None
        threshold, heading = runway
        layout = self.ground
        stand = (layout.stands[flight.stand] if flight.stand is not None
                 else None)
        here = layout.to_xy(flight.latitude, flight.longitude)
        return (layout.departure(stand, here, threshold, heading,
                                 self.RUNWAY_LENGTH_M,
                                 avoid=self._avoid_user())
                or layout.departure(stand, here, threshold, heading,
                                    self.RUNWAY_LENGTH_M))

    # An arrival inside this is close enough that a departure is kept at the
    # holding point rather than put on the runway in front of it. Lining one
    # up with somebody on a two-mile final is how a tower ends up sending the
    # arrival around, which is the mistake this exists to not make.
    LINEUP_BLOCK_NM = 4.0
    # And the slack on top of the roll itself. A departure released with
    # exactly enough time is a departure that goes wrong.
    DEPARTURE_MARGIN_S = 25.0

    def _runway_taken_by_departure(self, flight: Flight) -> bool:
        """Whether another departure holds the runway: told to line up or
        to take off, and still on the ground."""
        for other in self.flights:
            if other is flight or other.done or other.arriving:
                continue
            if not other.on_ground:
                continue
            if other.phase in ("lineup", "takeoff") or any(
                    command.kind in ("line_up", "takeoff")
                    and not command.applied for command in other.commands):
                return True
        return False

    def _somebody_lined_up(self, ignore: Flight | None = None) -> bool:
        """Whether the runway already has an aeroplane sitting on it."""
        return any(
            other is not ignore and not other.done
            and (other.phase == "lineup"
                 or self._already_told(other, "line_up"))
            for other in self.flights)

    def _seconds_to_threshold(self, flight: Flight) -> float:
        """How long this arrival has left to run, at the speed it is doing."""
        return flight.distance_nm / max(30.0, flight.ground_speed_kt) * 3600.0

    def _nearest_arrival_s(self, flight: Flight) -> float:
        """How long before the closest thing on the approach is on the runway."""
        approaching = self._approaching(flight)
        if not approaching:
            return 1e6
        return min(self._seconds_to_threshold(other) for other in approaching)

    def _room_to_depart(self, flight: Flight, now: float) -> bool:
        """Whether a departure has time to be gone before anybody lands.

        A tower does not decide this in miles, it decides it in seconds: the
        runway has to be free, the roll takes most of a minute, and whatever
        is on final has to still be on final at the end of it. Working in
        miles is what let a departure be lined up in front of an arrival that
        then had to be sent around.
        """
        needed = (max(0.0, self.runway_free_at - now)
                  + RUNWAY_ROLL_S + self.DEPARTURE_MARGIN_S)
        for other in self._approaching(flight):
            if self._seconds_to_threshold(other) < needed:
                return False
        return True

    def _work_the_departure(self, flight: Flight, now: float) -> None:
        """Decide what to do with an aeroplane that wants the runway.

        The runway is one aeroplane wide, and it is not only AI aeroplanes on
        it: the person flying counts, and so does one of them on short final.
        Nothing is released while either is true, and what the departure is
        told depends on which of the two it is.
        """
        # An aeroplane already sitting on the runway has priority over one
        # still at the holding point: it cannot get out of the way, so the
        # moment the runway is free it goes, as long as there is time for the
        # roll. Making it wait for the full margin instead was what left a
        # departure stuck on the runway with an arrival closing on it -- and
        # the arrival was then the one that had to go round.
        pressing = (flight.phase == "lineup"
                    and not self.runway_occupied(now, ignore=flight)
                    and self._nearest_arrival_s(flight) >= RUNWAY_ROLL_S)

        flight.held_since = flight.held_since or now
        if flight.phase != "lineup" and self._runway_taken_by_departure(flight):
            # Somebody else already has the runway -- lined up, entering it,
            # or rolling. One departure at a time: the clearance that used to
            # go out here put a second aeroplane on the runway behind a
            # lined-up one, and the two rolled together.
            flight.next_at = now + self.random.uniform(4.0, 8.0)
            return

        if not pressing and not self._room_to_depart(flight, now):
            blocking = self._traffic_on_final(self.LINEUP_BLOCK_NM,
                                              ignore=flight)
            # Somebody is landing. The departure stays where it is: held at
            # the holding point if it is still there, and held on the runway
            # if it was already lined up before the arrival got this close.
            if (flight.phase != "lineup" and flight.step == 0
                    and blocking is not None):
                flight.step = 1
                self._exchange(
                    flight, now, "TWR",
                    controller=lambda c, a, s: c.hold_short_instruction(
                        a, flight.runway),
                    pilot_back=lambda p, a, s: p.wilco(a),
                    command=("hold_short", {"runway": flight.runway}),
                )
            flight.next_at = now + self.random.uniform(6.0, 14.0)
            return

        if self.runway_occupied(now, ignore=flight):
            # Somebody is rolling, and will be gone in half a minute. That is
            # what lining up and waiting is for -- for one aeroplane. A runway
            # with three aeroplanes lined up on it is not a runway, and it was
            # what sent the next arrival round.
            if (flight.phase != "lineup" and flight.step < 2
                    and not self._somebody_lined_up(flight)):
                flight.step = 2
                self._exchange(
                    flight, now, "TWR",
                    controller=lambda c, a, s: c.line_up_and_wait(
                        a, flight.runway, station=s),
                    pilot_back=lambda p, a, s: p.line_up(a, flight.runway),
                    command=("line_up", {"runway": flight.runway}),
                )
            flight.next_at = now + self.random.uniform(6.0, 14.0)
            return

        def taking_the_runway(p, a, s):
            return p.line_up(a, flight.runway)
        if flight.pattern_work:
            self._exchange(
                flight, now, "TWR",
                controller=lambda c, a, s: c.closed_traffic_approved(
                    a, flight.runway, c.side(flight.pattern_direction),
                    self.field.weather, station=s),
                pilot_back=lambda p, a, s: p.takeoff(a, flight.runway),
                announce=taking_the_runway,
                command=("takeoff", {"runway": flight.runway}),
            )
        else:
            self._exchange(
                flight, now, "TWR",
                controller=lambda c, a, s: c.cleared_for_takeoff(
                    a, flight.runway, self.field.weather, station=s),
                pilot_back=lambda p, a, s: p.takeoff(a, flight.runway),
                announce=taking_the_runway,
                command=("takeoff", {"runway": flight.runway}),
            )
        self.runway_free_at = now + RUNWAY_ROLL_S
        flight.next_at = now + RUNWAY_ROLL_S * 0.5

    def _after_the_go_around(self, flight: Flight, now: float) -> None:
        """What happens to an aeroplane that has just been sent around.

        It is climbing away on the runway heading. Once it is high enough and
        far enough out it is put back into the circuit, which it then flies --
        so a go-around costs it a full circuit, exactly as one does.
        """
        pattern_ft = (self.field.elevation_ft
                      + flight.performance.pattern_agl_ft)
        if (flight.altitude_ft < pattern_ft - 200.0
                and flight.distance_nm < CIRCUIT_UPWIND_NM):
            flight.next_at = now + 3.0
            return
        flight.phase = "upwind"
        flight.arriving = False
        flight.step = 0
        flight.next_at = now + 5.0

