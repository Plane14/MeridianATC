from __future__ import annotations

from ..ground import INTERSECTION_M
from ..taxi import say_words
from ._shared import (
    _CIRCUIT_PHASES,
    CIRCUIT_UPWIND_NM,
    CIRCUIT_WIDTH_NM,
    CLIMB_TO_FT,
    DESTINATION_AIRPORTS,
    GLIDE_FT_PER_NM,
    INITIAL_CLIMB_FT,
    ROLLING_KT,
    RUNWAY_ROLL_S,
    Flight,
    _runway_key,
    _strips_parallel,
    bearing_deg,
    glide_altitude,
    landing_traffic,
    log,
    make_of,
    project,
    wake_suffix,
    wind_snapshot,
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
                # A procedure the model is steering draws ahead as the
                # legs it is on -- the published path, offset final and
                # all -- rather than the straight-in there would be
                # without them.
                rest = getattr(getattr(flight, "procedure_nav", None),
                               "points", None)
                if rest:
                    for plat, plon, palt, _speed in rest:
                        path.append((plat, plon,
                                     palt or elevation + 50.0))
                else:
                    behind = project(
                        lat, lon, (heading + 180.0) % 360.0,
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

    def _plan_departure(self, flight: Flight, extra_avoid=None):
        """The way from where it is to the holding point, and onto the
        runway: two legs, because they are two clearances.

        ``extra_avoid`` is more ground to keep out of the way -- the
        stretch a give-way reroute is staying off, which every fallback
        has to keep to or the new way meets the same nose."""
        runway = self._runway_xy(flight.runway)
        if runway is None:
            return None
        threshold, heading = runway
        layout = self.ground
        stand = (layout.stands[flight.stand] if flight.stand is not None
                 else None)
        if extra_avoid is not None:
            # A reroute starts where the aeroplane sits, not on the stand
            # it left -- and there is no pushback left to drive.
            stand = None
        here = layout.to_xy(flight.latitude, flight.longitude)
        closed = self._closed_runway_edges(
            flight.runway or self.field.departure_runway)
        from .. import aircraft_class
        need_m = aircraft_class.min_runway_ft(
            flight.type_code or "") * 0.3048
        # The runway's published length where it is published -- entries
        # beyond it do not exist, and a remaining distance counted from a
        # made-up figure is a wrong number on the frequency.
        surface = getattr(self.field, "surfaces", {}).get(
            flight.runway or self.field.departure_runway)
        length_m = (surface[2] * self._M_PER_NM
                    if surface else self.RUNWAY_LENGTH_M)
        extra = set(extra_avoid or ())
        legs = (layout.departure(stand, here, threshold, heading,
                                 length_m,
                                 avoid=self._avoid_user() | extra,
                                 closed=closed, need_m=need_m)
                or layout.departure(stand, here, threshold, heading,
                                    length_m, avoid=extra or None,
                                    closed=closed,
                                    need_m=need_m)
                or layout.departure(stand, here, threshold, heading,
                                    length_m, avoid=extra or None,
                                    closed=closed,
                                    need_m=need_m))
        # Joining the runway anywhere but the threshold is an
        # intersection departure: the taxiway it commits on is the edge
        # it steps onto the pavement with (the route's last name when
        # that edge is unlabeled), and what the tower then says in the
        # line-up and the takeoff clearance (JO 7110.65 3-7-1).
        flight.intersection = ""
        flight.remaining_m = 0.0
        if legs is not None:
            along = getattr(legs[1], "entry_along_m", None)
            if along is not None and along > INTERSECTION_M:
                names = layout.names_along(legs[0].nodes)
                joined_on = getattr(legs[1], "entry_name", "")
                name = joined_on or (names[-1] if names else "")
                if name:
                    flight.intersection = name
                    flight.remaining_m = max(0.0, length_m - along)
        return legs

    def _said_intersection(self, flight: Flight) -> str:
        """The taxiway an intersection departure joins, said the way it
        is read back ("A" as alpha, not a raw map label)."""
        if not flight.intersection:
            return ""
        return say_words(self.field.ident, [flight.intersection])[0]

    # An arrival inside this is close enough that a departure is kept at the
    # holding point rather than put on the runway in front of it. Lining one
    # up with somebody on a two-mile final is how a tower ends up sending the
    # arrival around, which is the mistake this exists to not make.
    LINEUP_BLOCK_NM = 4.0
    # And the slack on top of the roll itself. A departure released with
    # exactly enough time is a departure that goes wrong.
    DEPARTURE_MARGIN_S = 25.0

    def _conflicts(self, runway: str, other: Flight,
                   fallback: str = "") -> bool:
        """Whether ``other`` is on this runway's stream.

        Its own strip conflicts, and so does a crossing one -- the strips
        share pavement. A parallel is another runway's queue, the way the
        arrival count and the roll window already treat it.
        """
        strip = _runway_key(runway or fallback)
        theirs = _runway_key(other.runway or fallback)
        return theirs == strip or not _strips_parallel(theirs, strip)

    def _wake_departure_wait_s(self, flight: Flight, now: float) -> float:
        """Seconds still owed to the last roll's wake on this runway.

        7110.65 3-9-7: two minutes behind a heavy, three behind a super,
        before the next aeroplane lines up or is cleared on the same strip
        or a crossing one -- the wake lies on the pavement and under the
        climb-out either way.
        """
        rolled = getattr(self, "_wake_rolled", None)
        if not rolled:
            return 0.0
        strip = _runway_key(flight.runway or self.field.departure_runway)
        wait = 0.0
        for other_strip, (rolled_at, wake) in rolled.items():
            if other_strip != strip                     and _strips_parallel(other_strip, strip):
                continue
            need = 180.0 if wake == "super"                 else (120.0 if wake == "heavy" else 0.0)
            wait = max(wait, rolled_at + need - now)
        return max(0.0, wait)

    def _runway_taken_by_departure(self, flight: Flight) -> bool:
        """Whether another departure holds the runway: told to line up or
        to take off, and still on the ground."""
        strip = flight.runway or self.field.departure_runway
        for other in self.flights:
            if other is flight or other.done or other.arriving:
                continue
            if not other.on_ground:
                continue
            if not self._conflicts(strip, other,
                                   self.field.departure_runway):
                continue
            if other.phase in ("lineup", "takeoff") or any(
                    command.kind in ("line_up", "takeoff")
                    and not command.applied for command in other.commands):
                return True
        return False

    def _runway_occupier(self, flight: Flight) -> Flight | None:
        """The aeroplane on the runway in front of a waiting departure --
        the one whose roll the departure is waiting behind."""
        return next(
            (other for other in self.flights
             if other is not flight and not other.done
             and other.on_ground and other.runway == flight.runway
             and other.phase in ("takeoff", "landed")),
            None)

    @staticmethod
    def _lineup_reason(occupier: Flight | None, language: str) -> str:
        """Why the departure is being lined up behind somebody: a rolling
        departure ahead of it, or a landing clearing the runway -- the
        "departing traffic" / "landing traffic" a tower says when it does
        not clear an aeroplane straight off."""
        if occupier is None:
            return ""
        if occupier.phase == "takeoff":
            return "departing traffic"
        return landing_traffic(occupier.describe(language), language)

    def _somebody_lined_up(self, ignore: Flight | None = None,
                           runway: str = "") -> bool:
        """Whether the runway already has an aeroplane sitting on it."""
        return any(
            other is not ignore and not other.done
            and (other.phase == "lineup"
                 or self._already_told(other, "line_up"))
            and (not runway
                 or self._conflicts(runway, other,
                                    self.field.departure_runway))
            for other in self.flights)

    def _seconds_to_threshold(self, flight: Flight) -> float:
        """How long this arrival has left to run, at the speed it is doing."""
        return flight.distance_nm / max(30.0, flight.ground_speed_kt) * 3600.0

    def _nearest_arrival_s(self, flight: Flight,
                           runway: str = "") -> float:
        """How long before the closest thing on the approach is on the runway."""
        strip = (runway or getattr(flight, "runway", "")
                 or self.field.departure_runway)
        approaching = [other for other in self._approaching(flight)
                       if self._conflicts(strip, other,
                                          self.field.arrival_runway)]
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
        needed = (max(0.0, self._runway_free_at(flight.runway) - now)
                  + RUNWAY_ROLL_S + self.DEPARTURE_MARGIN_S)
        strip = _runway_key(flight.runway or self.field.departure_runway)
        for other in self._approaching(flight):
            theirs = _runway_key(other.runway
                                 or self.field.arrival_runway)
            if theirs != strip and _strips_parallel(theirs, strip):
                # A parallel final does not hold this runway's roll; a
                # crossing one still does.
                continue
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
                    and not self.runway_occupied(
                        now, ignore=flight, runway=flight.runway)
                    and self._nearest_arrival_s(flight) >= RUNWAY_ROLL_S)

        flight.held_since = flight.held_since or now
        if flight.phase != "lineup" and self._runway_taken_by_departure(flight):
            # Somebody else already has the runway -- lined up, entering it,
            # or rolling. One departure at a time: the clearance that used to
            # go out here put a second aeroplane on the runway behind a
            # lined-up one, and the two rolled together.
            flight.next_at = now + self.random.uniform(4.0, 8.0)
            return

        wake_wait = self._wake_departure_wait_s(flight, now)
        if wake_wait > 0.0 and not pressing:
            flight.next_at = now + min(wake_wait, 20.0)
            return

        if not pressing and not self._room_to_depart(flight, now):
            blocking = self._traffic_on_final(self.LINEUP_BLOCK_NM,
                                              ignore=flight,
                                              runway=flight.runway)
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

        if self.runway_occupied(now, ignore=flight, runway=flight.runway):
            user = self.user
            if (user is not None and user.on_ground
                    and user.ground_speed_kt < ROLLING_KT
                    and self._user_on_runway(
                        flight.runway or self.field.departure_runway)):
                # The pilot is on the strip and not rolling: crossing it,
                # backtracking along it, turning round at the far end. A
                # "line up and wait" may only be issued behind somebody
                # leaving by the far end -- the pilot's way out is back
                # past whoever would line up, and the two of them meet
                # nose to nose on the pavement.
                flight.next_at = now + self.random.uniform(6.0, 14.0)
                return
            # Somebody is rolling, and will be gone in half a minute. That is
            # what lining up and waiting is for -- for one aeroplane. A runway
            # with three aeroplanes lined up on it is not a runway, and it was
            # what sent the next arrival round.
            if (flight.phase != "lineup" and flight.step < 2
                    and not self._somebody_lined_up(
                        flight, runway=flight.runway)):
                flight.step = 2
                occupier = self._runway_occupier(flight)
                reason = self._lineup_reason(occupier, flight.language)
                self._exchange(
                    flight, now, "TWR",
                    controller=lambda c, a, s: c.line_up_and_wait(
                        a, flight.runway, reason=reason, station=s,
                        intersection=self._said_intersection(flight)),
                    pilot_back=lambda p, a, s: p.line_up(
                        a, flight.runway,
                        intersection=self._said_intersection(flight)),
                    command=("line_up", {"runway": flight.runway}),
                )
            flight.next_at = now + self.random.uniform(6.0, 14.0)
            return

        def taking_the_runway(p, a, s):
            return p.line_up(a, flight.runway,
                             intersection=self._said_intersection(flight))
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
                    a, flight.runway, self.field.weather, station=s,
                    intersection=self._said_intersection(flight),
                    remaining_m=flight.remaining_m),
                pilot_back=lambda p, a, s: p.takeoff(
                    a, flight.runway,
                    intersection=self._said_intersection(flight)),
                announce=taking_the_runway,
                command=("takeoff", {"runway": flight.runway}),
            )
        flight.wind_at_clearance = wind_snapshot(self.field.weather)
        rolled = getattr(self, "_wake_rolled", None)
        if rolled is None:
            rolled = self._wake_rolled = {}
        rolled[_runway_key(flight.runway
                           or self.field.departure_runway)] = (
            now, wake_suffix(flight.type_code))
        self._hold_runway(flight.runway, now, RUNWAY_ROLL_S)
        flight.next_at = now + RUNWAY_ROLL_S * 0.5

    def _after_the_go_around(self, flight: Flight, now: float) -> None:
        """What happens to an aeroplane that has just been sent around.

        An IFR arrival with a missed coded flies the published procedure
        out to its end and is then put back into the arrival stream for
        another approach, which is what a real go-around buys. Everything
        else -- a VFR flight, or a field whose data codes no missed --
        climbs away on the runway heading and rejoins the circuit once it
        is high enough and far enough out, exactly as one does.
        """
        missed_nav = getattr(flight, "missed_nav", None)
        if missed_nav is not None:
            if not missed_nav.guidance(flight).done:
                flight.next_at = now + 3.0
                return
            if flight.phase == "missed_hold":
                # Orbiting the missed's hold fix: kept until the clock
                # runs out, which is how long the tower takes to work it
                # back into the arrivals.
                if now < flight.missed_hold_until:
                    flight.next_at = now + 3.0
                    return
                flight.missed_hold = False
            elif getattr(flight, "missed_hold", False):
                # The procedure ends in a hold: the aeroplane keeps the
                # racetrack a couple of minutes rather than turning
                # straight back onto final.
                flight.phase = "missed_hold"
                flight.missed_hold_until = \
                    now + self.random.uniform(120.0, 300.0)
                flight.next_at = now + 3.0
                return
            flight.missed_nav = None
            # The missed is flown: back into the stream for another
            # approach rather than round the circuit again.
            flight.phase = "inbound"
            flight.arriving = True
            flight.step = 0
            flight.next_at = now + 5.0
            return
        if flight.phase == "missed_hold":
            # A hold left over with no navigator (cleared in between):
            # back to the arrivals it goes.
            flight.missed_hold = False
            flight.phase = "inbound"
            flight.arriving = True
            flight.step = 0
            flight.next_at = now + 5.0
            return
        pattern_ft = (self.field.elevation_ft
                      + flight.performance.pattern_agl_ft)
        if getattr(flight, "ifr", False):
            # The published missed where the legs are not coded: runway
            # heading up to its altitude, then back into the arrival
            # stream for approach to sequence again -- the circuit is
            # the VFR aeroplane's way round, not this one's.
            goal = getattr(flight, "missed_alt_ft", 0.0) or pattern_ft
            if (flight.altitude_ft < goal - 100.0
                    and flight.distance_nm < 15.0):
                flight.next_at = now + 3.0
                return
            flight.phase = "inbound"
            flight.arriving = True
            flight.step = 0
            flight.next_at = now + 5.0
            return
        if (flight.altitude_ft < pattern_ft - 200.0
                and flight.distance_nm < CIRCUIT_UPWIND_NM):
            flight.next_at = now + 3.0
            return
        flight.phase = "upwind"
        flight.arriving = False
        flight.step = 0
        flight.next_at = now + 5.0

