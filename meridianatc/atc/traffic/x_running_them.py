from __future__ import annotations

from ._shared import (
    RUNWAY_LENGTH_NM,
    Flight,
    groundwork,
    log,
    math,
    offset_from_track,
)


class _Running_them:
    # ------------------------------------------------------------------
    # running them
    # ------------------------------------------------------------------

    # A poll that arrives after a long gap -- the window was minimised, the
    # sim was paused -- must not teleport everybody. An aeroplane moves by at
    # most this many seconds' worth in one step.
    MAX_STEP_S = 5.0

    # How far ahead of a taxiing aeroplane anything stops it. Ninety metres
    # is the braking distance from twenty knots (groundwork.BRAKE_HARD_MS2,
    # about thirty-five metres) plus an airliner's length and a half: room to
    # stop behind one without its tail filling the windscreen. It was seventy
    # when an aeroplane stopped dead the instant it saw something.
    TAXI_LOOK_M = 90.0
    # An intersection is claimed this far before it is reached, for the same
    # reason: what holds an aeroplane short of it has to leave room to brake.
    CLAIM_M = 95.0

    def _taxi(self, flight: Flight, elapsed: float, now: float) -> None:
        """One step along the route, unless something is in the way."""
        layout = self.ground
        taxi = flight.taxi
        if layout is None or taxi is None:
            flight.taxi = None
            return
        x, z = layout.to_xy(flight.latitude, flight.longitude)
        if taxi.done:
            self._arrived(flight, now)
            return
        tx, tz = taxi.points[taxi.index]
        pushing = taxi.index < taxi.push
        moving = groundwork._heading(tx - x, tz - z)
        why = self._taxi_blocked(flight, x, z, moving, now)
        if why == "holding short" and not flight.arriving:
            flight.held_since = flight.held_since or now
        elif not why:
            flight.held_since = 0.0
        if taxi.speed < 0.0:
            # Starting the route at the speed it already has: an arrival
            # turning off the runway rolls on to its taxi.
            taxi.speed = flight.ground_speed_kt / 1.943844
        if why:
            taxi.blocked_since = taxi.blocked_since or now
            if taxi.speed > 0.0:
                # Braking, not stopped dead: something is seen in the look
                # ahead (TAXI_LOOK_M, CLAIM_M), further than an airliner
                # takes to stop from taxi speed, and stopping in an instant
                # was a jolt in the simulator.
                x, z, heading, speed = groundwork.advance(
                    taxi, x, z, flight.heading, elapsed, stop=True)
                flight.latitude, flight.longitude = layout.to_latlon(x, z)
                flight.heading = heading
            flight.ground_speed_kt = taxi.speed * 1.943844
            flight.target.intent = why
            self._break_deadlock(flight, now)
            return
        if (taxi.hold_at is not None and not taxi.cleared
                and taxi.index > taxi.hold_at):
            # Off the runway. On to the stand only if nobody is coming the
            # other way along the same taxiway; otherwise it waits here,
            # clear of the runway, which is where a real one would be told to.
            if self._crosses(taxi, flight):
                taxi.blocked_since = taxi.blocked_since or now
                flight.ground_speed_kt = 0.0
                flight.target.intent = "holding position"
                return
            taxi.cleared = True
        taxi.blocked_since = None
        x, z, heading, speed = groundwork.advance(taxi, x, z, flight.heading,
                                                  elapsed)
        flight.latitude, flight.longitude = layout.to_latlon(x, z)
        flight.heading = heading
        flight.ground_speed_kt = speed * 1.943844
        flight.on_ground = True
        flight.altitude_ft = self.field.elevation_ft
        flight.target.intent = "pushing back" if pushing else "taxiing"
        self._release_behind(flight)
        if taxi.done:
            self._arrived(flight, now)

    def _in_the_way(self, flight: Flight, x: float, z: float
                    ) -> Flight | str | None:
        """Whatever is on the stretch of route this aeroplane is about to use.

        A corridor along the route itself rather than a cone off the nose: a
        cone saw the pilot parked beside a taxilane and stopped for them for
        ever, and missed an aeroplane round the next corner.
        """
        layout = self.ground
        corridor = groundwork.ahead(flight.taxi, x, z, self.TAXI_LOOK_M)
        others: list[tuple[object, float, float]] = []
        user = self.user
        if user is not None and user.has_position and user.on_ground:
            others.append(("you", *layout.to_xy(user.latitude,
                                                user.longitude)))
        for other in self.flights:
            if other is flight or other.done or not other.positioned:
                continue
            if not other.on_ground:
                continue
            others.append((other, *layout.to_xy(other.latitude,
                                                 other.longitude)))
        for who, ox, oz in others:
            if math.hypot(ox - x, oz - z) < 1.0:
                continue
            if (isinstance(who, Flight)
                    and getattr(who, "taxi", None) is not None
                    and getattr(who, "ground_speed_kt", 0.0) < 1.0
                    and getattr(who, "_giving_way", "") == flight.written):
                # Stopped and already giving way to this one: it is the
                # give-way, not the obstacle -- the other side of a head-on
                # is met by creeping past it, the way real aircraft do.
                continue
            # The pilot gets more room than the traffic gives itself: their
            # aeroplane may be anything, parked a little off the stand mark.
            room = (self.USER_CLEARANCE_M if who == "you"
                    else self.TAXI_CORRIDOR_M)
            # Behind it does not count; that one is the other's to watch.
            for px, pz in corridor[1:]:
                if math.hypot(ox - px, oz - pz) < room:
                    return who
        return None

    USER_CLEARANCE_M = 35.0

    def _avoid_standing(self, flight: Flight) -> set[object]:
        """Taxi nodes beside an aeroplane that is not moving."""
        layout = self.ground
        found: set[object] = set()
        for other in self.flights:
            if other is flight or other.done or not other.positioned:
                continue
            if not other.on_ground or other.ground_speed_kt > 1.0:
                continue
            ox, oz = layout.to_xy(other.latitude, other.longitude)
            found |= {node for node, (x, z) in layout.xy.items()
                      if math.hypot(x - ox, z - oz) < self.TAXI_CORRIDOR_M
                      + 5.0}
        return found

    def _avoid_user(self) -> set[object]:
        """The taxi nodes too close to the pilot to route through, if there
        is any other way round."""
        layout = self.ground
        user = self.user
        if layout is None or user is None or not user.has_position \
                or not user.on_ground:
            return set()
        ux, uz = layout.to_xy(user.latitude, user.longitude)
        return {node for node, (x, z) in layout.xy.items()
                if math.hypot(x - ux, z - uz) < self.USER_CLEARANCE_M + 10.0}

    # Half an airliner's span and a margin: anything with its middle closer
    # than this to the route ahead is in the way.
    TAXI_CORRIDOR_M = 28.0
    # Two aeroplanes each waiting for the other this long are not going to
    # sort it out between themselves: the loser is routed round the meeting.
    # A second wait beyond that without a way round is what the tow is for.
    GIVE_WAY_S = 20.0
    DEADLOCK_S = 60.0
    STUCK_ON_RUNWAY_S = 30.0

    def _break_deadlock(self, flight: Flight, now: float) -> None:
        """The last resort when two taxiing aeroplanes face each other.

        The route check stops a ground controller sending them at each
        other, but an aeroplane that has just landed and one pushing back can
        still meet. Given long enough nose to nose the one that matters less
        -- a departure gives way to an arrival that has to get off the
        runway -- is sent round another way; only where the field has no
        other way is it taken away, there being no tug to call.
        """
        taxi = flight.taxi
        if taxi is None or taxi.blocked_since is None:
            return
        layout = self.ground
        x, z = layout.to_xy(flight.latitude, flight.longitude)
        blocker = self._in_the_way(flight, x, z)
        on_runway = self._on_runway_itself(flight)
        if (now - taxi.blocked_since >= self.STUCK_ON_RUNWAY_S
                and on_runway):
            # Stuck on the runway behind somebody standing still. The runway
            # is the one place nobody may wait, so whatever of ours is in the
            # way goes -- never the pilot, never somebody else's aeroplane.
            if blocker == "you":
                # Nose to nose with the pilot: it is ours that moves, and the
                # way it can is the way it came -- off the runway by the
                # first exit behind it, or towed when the pavement offers
                # none. A live one goes back to the feed instead.
                if getattr(flight, "_vacated_for_user", False):
                    self._take_off_the_runway(flight, now)
                elif self._vacate_the_runway(flight, now):
                    flight._vacated_for_user = True
                else:
                    self._take_off_the_runway(flight, now)
                return
            if isinstance(blocker, Flight) and not blocker.observed \
                    and blocker.ground_speed_kt <= 1.0:
                if blocker.source == "live":
                    log.info("%s is stuck on the runway behind %s; %s is "
                             "handed back to the feed", flight.written,
                             blocker.written, blocker.written)
                    self._unadopt(blocker, now, why="stuck on the runway")
                else:
                    log.info("%s is stuck on the runway behind %s; %s is "
                             "taken away", flight.written, blocker.written,
                             blocker.written)
                    blocker.done = True
                    self._release_all(blocker)
                return
        if now - taxi.blocked_since < self.GIVE_WAY_S:
            return
        if not on_runway:
            if blocker == "you":
                # Face to face with the pilot on a taxiway: the give-way is
                # ours to make, and the way it can be made is round another
                # way -- the same vacate as off a runway, without the
                # runway. Where the field has no other way it keeps waiting.
                self._replan_round(flight, self._pilot_avoid())
                return
            oncoming = self._head_on_ahead(flight)
            if oncoming is not None:
                # Held back for one coming the other way that is taking too
                # long about it: route round the meeting where the field
                # has one.
                self._replan_round(flight, self._contested_avoid(
                    flight, oncoming))
                return
        if not isinstance(blocker, Flight) or blocker.taxi is None \
                or blocker.taxi.blocked_since is None:
            return
        ox, oz = layout.to_xy(blocker.latitude, blocker.longitude)
        if self._in_the_way(blocker, ox, oz) is not flight:
            return
        loser = blocker if (flight.arriving and not blocker.arriving) \
            else flight
        if loser.observed:
            loser = flight
        winner = flight if loser is blocker else blocker
        if not on_runway and self._replan_round(
                loser, self._contested_avoid(loser, winner)):
            return
        if now - taxi.blocked_since < self.DEADLOCK_S:
            return
        if loser.source == "live":
            log.info("%s and %s are nose to nose; %s is handed back to "
                     "the feed", flight.written, blocker.written,
                     loser.written)
            self._unadopt(loser, now, why="nose to nose")
            return
        log.info("%s and %s are nose to nose; %s is taken away",
                 flight.written, blocker.written, loser.written)
        loser.done = True
        self._release_all(loser)

    def _pilot_avoid(self) -> set:
        """The taxi nodes beside the pilot, wide enough that a way round is
        genuinely a way round rather than a second meeting."""
        layout = self.ground
        user = self.user
        if layout is None or user is None or not user.has_position:
            return set()
        ux, uz = layout.to_xy(user.latitude, user.longitude)
        return {node for node, (nx, nz) in layout.xy.items()
                if math.hypot(nx - ux, nz - uz)
                < self.USER_CLEARANCE_M + 45.0}

    def _contested_avoid(self, flight: Flight, other: Flight) -> set:
        """Ground to keep out of while routing round a head-on: the stretch
        the two routes share reversed, plus a bubble round the other
        aeroplane wherever it is."""
        avoid: set = set()
        layout = self.ground
        if flight.taxi is not None and other.taxi is not None:
            reversed_mine = {(b, a) for a, b in
                             flight.taxi.remaining_edges()}
            for a, b in other.taxi.remaining_edges() & reversed_mine:
                avoid.update((a, b))
        if layout is not None:
            ox, oz = layout.to_xy(other.latitude, other.longitude)
            avoid |= {node for node, (nx, nz) in layout.xy.items()
                      if math.hypot(nx - ox, nz - oz) < self.TAXI_LOOK_M}
        return avoid

    def _replan_round(self, flight: Flight, avoid: set) -> bool:
        """A fresh way from where it is to where it was going, kept off the
        ground in ``avoid``. False where the field has none -- and the
        caller's fallback then stands. An aeroplane committed to the
        runway is never rerouted: the strip has its own machinery.
        """
        layout = self.ground
        if (layout is None or flight.taxi is None
                or flight.taxi is flight.line_up):
            return False
        x, z = layout.to_xy(flight.latitude, flight.longitude)
        if flight.arriving:
            stand = (layout.stands[flight.stand]
                     if flight.stand is not None
                     and 0 <= flight.stand < len(layout.stands) else None)
            if stand is None:
                return False
            closed = self._closed_runway_edges(
                flight.runway or self.field.arrival_runway)
            taxi = None
            for extra in (avoid | self._avoid_user()
                          | self._avoid_standing(flight),
                          avoid | self._avoid_user(), avoid):
                taxi = layout.to_stand(x, z, stand, avoid=extra or None,
                                       closed=closed)
                if taxi is not None:
                    break
            if taxi is None and closed:
                for extra in (avoid | self._avoid_user(), avoid):
                    taxi = layout.to_stand(x, z, stand, avoid=extra or None)
                    if taxi is not None:
                        break
            if taxi is None:
                return False
            flight.taxi = taxi
        else:
            legs = self._plan_departure(flight, extra_avoid=avoid)
            if legs is None:
                return False
            flight.taxi, flight.line_up = legs
            flight.planned = None
        flight._giving_way = ""
        log.info("%s gives way and is routed round", flight.written)
        return True

    def _on_runway_itself(self, flight: Flight) -> bool:
        end = self._threshold(flight.runway or self.field.arrival_runway)
        if end is None:
            return False
        along, across = offset_from_track(flight.latitude, flight.longitude,
                                          end[0], end[1], end[2])
        # The runway surface, not the ninety-metre strip RUNWAY_WIDTH_NM
        # describes: with that, an aeroplane eighty metres off the
        # centreline counted as already on the runway, was let "off" it, and
        # drove across in front of a departure on its takeoff roll.
        return abs(across) <= self.ON_RUNWAY_NM and \
            -0.15 <= along <= RUNWAY_LENGTH_NM

    def _vacate_the_runway(self, flight: Flight, now: float) -> bool:
        """Send ours off the runway the way it came, to a stand.

        The resolution for the one deadlock the usual answer cannot touch:
        a departure nose to nose with the pilot on the strip cannot be taken
        away (the pilot sees an aeroplane, not a gap) and cannot go forward
        (the pilot is there). What a tower does is vacate it -- turn round,
        off at the first exit behind, and back to a stand. False when the
        layout offers no way to do it.
        """
        layout = self.ground
        runway = self._runway_xy(
            flight.runway or self.field.departure_runway
            or self.field.arrival_runway)
        if layout is None or runway is None:
            return False
        threshold, heading = runway
        stand = self._free_stand(airliner=flight.operator is not None
                                 or flight.ifr,
                                 gate=self._wants_gate(flight))
        if stand is None:
            return False
        x, z = layout.to_xy(flight.latitude, flight.longitude)
        closed = self._closed_runway_edges(
            flight.runway or self.field.departure_runway)
        taxi = None
        for avoid in (self._avoid_user() | self._avoid_standing(flight),
                      self._avoid_user(), set()):
            taxi = layout.vacate(x, z, flight.heading, threshold, heading,
                                 self.RUNWAY_LENGTH_M, stand, avoid=avoid,
                                 closed=closed, behind_first=True)
            if taxi is not None:
                break
        if taxi is None and closed:
            for avoid in (self._avoid_user() | self._avoid_standing(flight),
                          self._avoid_user(), set()):
                taxi = layout.vacate(x, z, flight.heading, threshold,
                                     heading, self.RUNWAY_LENGTH_M, stand,
                                     avoid=avoid, behind_first=True)
                if taxi is not None:
                    break
        if taxi is None:
            return False
        log.info("%s vacates the runway behind the pilot, to stand %s",
                 flight.written, stand.number)
        flight.on_layout = True
        flight.arriving = True
        flight.stand = stand.index
        flight.planned_stand = None
        flight.line_up = None
        flight.taxi = taxi
        flight.phase = "taxi_in"
        flight.doing = None
        flight.commands = [c for c in flight.commands
                           if c.kind not in ("line_up", "takeoff")]
        return True

    def _take_off_the_runway(self, flight: Flight, now: float) -> None:
        """The last resort for one stuck on the runway with the pilot: off
        it goes, however that has to happen. A live one is handed back to
        the feed -- the real aeroplane is wherever it is -- and an invented
        one is towed away, the runway being the one place nobody may wait.
        """
        if flight.source == "live" or flight.observed:
            log.info("%s is stuck on the runway behind the pilot; handed "
                     "back to the feed", flight.written)
            self._unadopt(flight, now)
            return
        log.info("%s is stuck on the runway behind the pilot; %s is taken "
                 "away", flight.written, flight.written)
        flight.done = True
        self._release_all(flight)

    def _head_on_ahead(self, flight: Flight) -> Flight | None:
        """Somebody else coming the other way along what is left of this
        route, or None.

        Two routes that use the same stretch of ground in opposite
        directions meet nose to nose; one of the pair has to wait, and a
        ground controller decides which before they are face to face. A
        route already committed to the runway is exempt -- the strip has
        its own machinery.
        """
        taxi = flight.taxi
        if taxi is None or taxi is flight.line_up:
            return None
        reversed_mine = {(b, a) for a, b in taxi.remaining_edges()}
        if not reversed_mine:
            return None
        for other in self.flights:
            if other is flight or other.done or other.observed \
                    or other.taxi is None or other.taxi.done:
                continue
            # The one that already yields to us does not get our yield back.
            if getattr(other, "_giving_way", "") == flight.written:
                continue
            if not (other.taxi.remaining_edges() & reversed_mine):
                continue
            if self._yields_to(flight, other):
                return other
        return None

    @staticmethod
    def _yields_to(flight: Flight, other: Flight) -> bool:
        """Who waits when two routes meet head on.

        A departure waits for an arrival -- the arrival has a runway's
        worth of momentum behind it and a stand in front, same rule the
        deadlock breaker uses. Between two of the same kind the callsign
        picks one for good: both sides work the same answer out, and it
        cannot flip halfway through the meeting.
        """
        if flight.arriving != other.arriving:
            return not flight.arriving
        return flight.written > other.written

    def _taxi_blocked(self, flight: Flight, x: float, z: float,
                      moving: float, now: float) -> str:
        """Why this aeroplane may not move just now, or "" if it may."""
        layout = self.ground
        # Somebody coming the other way along what is left of the route:
        # sorted out early, before anyone is inside anyone's windscreen.
        oncoming = self._head_on_ahead(flight)
        if oncoming is not None:
            # Give way where we are, early: the junctions a wait of ours
            # would hold are the junctions the other one needs through.
            flight._giving_way = oncoming.written
            self._release_all(flight)
            return "holding for traffic"
        flight._giving_way = ""
        # Anybody ahead: the pilot, another of ours, somebody else's.
        if self._in_the_way(flight, x, z) is not None:
            return "holding for traffic"
        # The next intersection, which is somebody else's until they are
        # through it.
        taxi = flight.taxi
        for position in range(taxi.index, len(taxi.nodes)):
            node = taxi.nodes[position]
            if node is None:
                continue
            nx, nz = layout.xy[node]
            if math.hypot(nx - x, nz - z) > self.CLAIM_M:
                break
            if layout.degree(node) < 3 and node not in layout.runway_nodes:
                continue
            holder = self._reserved.get(node)
            if holder and holder != flight.written:
                return "holding for traffic"
            # The line-up itself was cleared by the tower; every other way
            # onto the runway -- the taxi out included, which carries the
            # line-up leg from the stand onwards -- asks here first.
            if node in layout.runway_nodes and taxi is not flight.line_up \
                    and not self._on_runway_itself(flight) \
                    and self._reserved.get(node) != flight.written:
                # Onto the runway to cross it or to backtrack along it --
                # which at a field like Madeira is the only way to the end.
                # Asked once, for all of it: the stretch it will be on, and
                # how long that takes. An aeroplane that has just landed is
                # always let off the runway it is on.
                if not self._may_cross_runway(flight, position, now):
                    return "holding short"
                self._claim_runway_stretch(flight, position)
            self._reserved[node] = flight.written
        return ""

    # A runway is crossed only with this long before anybody lands on it.
    CROSS_CLEAR_S = 90.0

    def _runway_stretch(self, flight: Flight, position: int) -> list[object]:
        """The runway nodes a route uses from here, in one go."""
        layout = self.ground
        nodes = flight.taxi.nodes
        stretch = []
        for node in nodes[position:]:
            if node is None or node not in layout.runway_nodes:
                break
            stretch.append(node)
        return stretch

    def _crossed_runway(self, stretch: list) -> str:
        """The runway the stretch actually lies on, as its published ident.

        The taxi route can carry an aeroplane over a strip that is not
        the one it was cleared for, and the holds have to name the
        pavement under the wheels -- a crossing guarded only by the
        flight's own runway let one be driven across a live parallel.
        """
        layout = self.ground
        if not stretch or layout is None:
            return ""
        thresholds = getattr(self.field, "thresholds", {}) or {}
        surfaces = getattr(self.field, "surfaces", {}) or {}
        for name, end in thresholds.items():
            try:
                threshold_xy = layout.to_xy(end[0], end[1])
            except (AttributeError, TypeError, ValueError):
                continue
            surface = surfaces.get(name)
            length_m = (surface[2] * self._M_PER_NM
                        if surface else 2000.0)
            for node in stretch[:4]:
                try:
                    along, across = layout.runway_frame(
                        *layout.xy[node], threshold_xy, end[2])
                except (AttributeError, TypeError, ValueError, KeyError):
                    continue
                if -50.0 <= along <= length_m + 50.0 \
                        and abs(across) <= groundwork.RUNWAY_HALF_M:
                    return name
        return ""

    def _may_cross_runway(self, flight: Flight, position: int,
                          now: float) -> bool:
        """Whether it may go onto the runway for the stretch ahead, and if it
        may, the runway is taken for as long as that stretch takes."""
        layout = self.ground
        stretch = self._runway_stretch(flight, position)
        metres = sum(math.dist(layout.xy[a], layout.xy[b])
                     for a, b in zip(stretch, stretch[1:], strict=False))
        # Off the far side as well: the tail has to clear it, not the nose.
        takes = metres / groundwork.TURN_MS + 30.0
        crossing = self._crossed_runway(stretch) or flight.runway
        if (self.runway_occupied(now, ignore=flight, runway=crossing)
                or self._somebody_lined_up(flight, runway=crossing)
                or self._nearest_arrival_s(flight, runway=crossing) < takes
                + self.CROSS_CLEAR_S):
            return False
        self._hold_runway(crossing, now, takes)
        return True

    def _claim_runway_stretch(self, flight: Flight, position: int) -> None:
        for node in self._runway_stretch(flight, position):
            self._reserved[node] = flight.written

    def _release_behind(self, flight: Flight) -> None:
        """Give back the intersections this aeroplane is through."""
        taxi = flight.taxi
        layout = self.ground
        x, z = layout.to_xy(flight.latitude, flight.longitude)
        for node, holder in list(self._reserved.items()):
            if holder != flight.written:
                continue
            ahead = taxi is not None and node in taxi.nodes[taxi.index:]
            nx, nz = layout.xy[node]
            if not ahead and math.hypot(nx - x, nz - z) > 30.0:
                del self._reserved[node]

    def _release_all(self, flight: Flight) -> None:
        for node, holder in list(self._reserved.items()):
            if holder == flight.written:
                del self._reserved[node]

    def _prune_reservations(self) -> None:
        """Drop every reservation whose holder is no longer on the ground.

        A reservation was released by driving past the junction, by parking,
        or by the deadlock breaker -- never by leaving. A departure lined up
        and took off still holding its last junction, and the next aeroplane
        to use it waited for one that no longer existed: Kortrijk's "takeoff,
        0.0 nm to run" for twenty-eight minutes, and the queue behind it.
        Reproduced on Kortrijk's own taxi network (OO-BPH waiting on a
        junction held by OO-XLZ, gone).
        """
        if not self._reserved:
            return
        on_the_ground = {f.written for f in
                         self.flights + self._materialising + self._asked_for
                         if not f.done and f.on_ground}
        for node, holder in list(self._reserved.items()):
            if holder not in on_the_ground:
                del self._reserved[node]

