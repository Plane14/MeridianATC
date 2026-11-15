from __future__ import annotations

from ._shared import (
    _CIRCUIT_PHASES,
    _FLYING_PHASES,
    Flight,
    decision,
    fly_step,
    log,
    math,
)


class _The_watchdog:
    # -- the watchdog ---------------------------------------------------

    # How long an aeroplane may stand still where it should be moving before
    # it is stuck: on a runway, on a taxiway, in the air. The figures asked
    # for after Kortrijk -- longer than any stop this program's own traffic
    # makes there on purpose, which is all they have to be.
    STUCK_AFTER_S = {"runway": 20.0, "taxiway": 60.0, "air": 30.0}
    # Standing still on purpose: told to, or waiting its turn.
    _WAITING_ON_PURPOSE = frozenset({"holding short", "lined up",
                                     "holding position", "parked"})
    _AIRBORNE_PHASES = frozenset({"takeoff", "departed", "go_around",
                                  "inbound", "joining", "final"}) \
        | _CIRCUIT_PHASES
    # Moved: a metre on the ground, ten feet up or down.
    STILL_M = 1.0
    STILL_FT = 10.0

    def _where_moving(self, flight: Flight) -> str:
        """"runway", "taxiway" or "air" for an aeroplane in a phase in which
        it should be moving, or "" for one that may stand still."""
        if flight.done or flight.observed or flight.static:
            return ""
        if not flight.on_ground:
            return "air" if flight.phase in self._AIRBORNE_PHASES else ""
        if flight.taxi is not None and flight.phase in (
                "taxi", "ready", "taxi_in", "takeoff", "lineup", "landed"):
            on_runway = (self.ground is not None
                         and self._on_runway_itself(flight))
            return "runway" if on_runway else "taxiway"
        if flight.phase in ("takeoff", "landed"):
            return "runway"
        return ""

    def _queued(self, flight: Flight, now: float) -> bool:
        """Waiting for somebody who is themselves getting on with it -- the
        aeroplane in front, or the one holding the junction ahead -- which
        is a queue and not a fault. Two that have both stood still for the
        whole limit are waiting on each other, and that is.

        Measured at Madeira: an arrival taxiing in held for a junction
        another of ours had reserved was taken away as stuck while the
        holder was simply waiting its own turn.
        """
        if flight.taxi is None or self.ground is None:
            return False
        x, z = self.ground.to_xy(flight.latitude, flight.longitude)
        ahead = self._in_the_way(flight, x, z)
        if ahead is not None and not isinstance(ahead, Flight):
            return True                     # the pilot, or the simulator's
        waiting_on = [ahead] if isinstance(ahead, Flight) else []
        taxi = flight.taxi
        holders = {holder for node, holder in self._reserved.items()
                   if holder != flight.written and node in taxi.nodes[taxi.index:]}
        waiting_on += [f for f in self.flights if f.written in holders]
        limit = min(self.STUCK_AFTER_S.values())
        return any(not other.stuck and not other.done
                   and now - (other.still_since or now) < limit
                   for other in waiting_on)

    def _watchdog(self, now: float) -> None:
        """Find what has stopped where it should be moving, and deal with it.

        The safety net, not the fix: whatever froze an aeroplane is a bug to
        be found, and this is what keeps the frequency working until it is.
        Stuck for ``STUCK_AFTER_S`` it is flagged -- out of the controller's
        picture from then on -- and given one recovery; still stuck after
        another as long, it is taken out of the traffic, and so out of the
        simulator, with a ``traffic_stuck`` row saying which, where, in
        what phase and why.
        """
        for flight in self.flights:
            where = self._where_moving(flight)
            here = (flight.latitude, flight.longitude, flight.altitude_ft)
            if (not where or flight.target.intent in self._WAITING_ON_PURPOSE
                    or self._queued(flight, now)):
                flight.still_at, flight.still_since = here, now
                flight.stuck = flight.recovered = False
                continue
            if flight.still_at is None or self._moved(flight.still_at, here):
                flight.still_at, flight.still_since = here, now
                if flight.stuck:
                    log.info("traffic: %s is moving again", flight.written)
                flight.stuck = flight.recovered = False
                continue
            still = now - flight.still_since
            if still < self.STUCK_AFTER_S[where]:
                continue
            if not flight.recovered:
                flight.stuck = True
                flight.recovered = True
                self._say_stuck(flight, where, still, "retry")
                self._recover(flight)
                flight.still_since = now
                continue
            self._say_stuck(flight, where, still, "removed")
            flight.done = True
            self._release_all(flight)
            if where == "runway":
                # Whatever it had booked the runway for is not happening.
                self._release_runway(flight.runway, now)

    def _moved(self, was: tuple, here: tuple) -> bool:
        north_m = (here[0] - was[0]) * 111_320.0
        east_m = ((here[1] - was[1]) * 111_320.0
                  * math.cos(math.radians(here[0])))
        return (math.hypot(north_m, east_m) > self.STILL_M
                or abs(here[2] - was[2]) > self.STILL_FT)

    def _recover(self, flight: Flight) -> None:
        """One attempt to get it going: nothing reserved in its name, the
        route's wait forgotten, back in the flight model if it is flying,
        and drawn again from scratch in case the simulator lost it."""
        self._release_all(flight)
        if flight.taxi is not None:
            flight.taxi.blocked_since = None
        flight.target.hold = False
        if (flight.phase in _FLYING_PHASES and flight.taxi is None
                and not flight.airborne_model):
            self._start_flying(flight)
        flight.redraw = True

    def _say_stuck(self, flight: Flight, where: str, still: float,
                   action: str) -> None:
        if where == "runway":
            place = f"runway {flight.runway}".strip()
        elif where == "air":
            place = f"in the air, {flight.altitude_ft:.0f} ft"
        else:
            place = "taxiway"
        why = flight.target.intent or "not moving"
        holders = sorted({holder for node, holder in self._reserved.items()
                          if holder != flight.written and flight.taxi is not None
                          and node in flight.taxi.nodes[flight.taxi.index:]})
        if holders:
            why += f" (route held by {', '.join(holders)})"
        decision("traffic_stuck", callsign=flight.written, phase=flight.phase,
                 where=place, latitude=round(flight.latitude, 5),
                 longitude=round(flight.longitude, 5),
                 altitude_ft=round(flight.altitude_ft), still_s=round(still),
                 why=why, action=action)
        log.info("traffic: %s stuck %s s %s in phase %s (%s): %s",
                 flight.written, round(still), place, flight.phase, why,
                 "trying once more" if action == "retry" else "taken away")

    def _arrived(self, flight: Flight, now: float) -> None:
        """At the end of a route: the holding point, the runway, a stand."""
        taxi = flight.taxi
        flight.taxi = None
        flight.ground_speed_kt = 0.0
        if taxi is not None and taxi.face is not None:
            # On the runway, pointing down it. The flight model takes it from
            # here, from where it actually is.
            flight.line_up = None
            flight.heading = taxi.face
            return
        if flight.arriving and flight.stand is not None:
            flight.parked_at = now
            flight.target.intent = "parked"
            stand = self.ground.stands[flight.stand]
            flight.heading = stand.heading
            self._release_all(flight)
            return
        flight.target.intent = "holding short"

    def _crosses(self, route, flight: Flight) -> bool:
        """Whether this route meets somebody else's head on.

        A ground controller does not send two aeroplanes at each other down
        one taxiway and hope; one of them waits. So does this.
        """
        if route is None:
            return False
        mine = route.remaining_edges()
        reversed_mine = {(b, a) for a, b in mine}
        for other in self.flights:
            if other is flight or other.done or other.taxi is None:
                continue
            if other.taxi.remaining_edges() & reversed_mine:
                return True
        return False

    def _fly(self, now: float) -> None:
        """Move every aeroplane along, at its own rate."""
        elapsed = now - self._flown_at if self._flown_at else 0.0
        self._flown_at = now
        elapsed = max(0.0, min(elapsed, self.MAX_STEP_S))
        if elapsed <= 0.0:
            return
        for flight in self.flights:
            if flight.done or flight.observed:
                # Somebody else owns this one's position. Flying it here would
                # be two hands on the same aeroplane.
                continue
            if not flight.airborne_model:
                if flight.taxi is not None:
                    # On the ground and on its way somewhere: driven, not
                    # placed, and not flown until it gets there.
                    self._taxi(flight, elapsed, now)
                    if flight.taxi is not None:
                        continue
                if flight.phase in _FLYING_PHASES and not (
                        flight.on_layout and flight.line_up is not None):
                    # Anything in an airborne phase belongs to the model,
                    # however it got there.
                    self._start_flying(flight)
                if not flight.airborne_model:
                    if not flight.on_layout:
                        self._place(flight)
                    continue
            if now > getattr(flight, "expedite_until", 0.0):
                flight.expedite_until = 0.0
            self._aim(flight)
            fly_step(flight, flight.target, flight.performance, elapsed,
                     self._surface_ft(flight), energy=self.energy_model)
            self._after_flying(flight)

    def _advance(self, now: float) -> None:
        # What was said a moment ago takes effect first: an aeroplane that has
        # finished reading back a go-around is going around before it is moved
        # anywhere, not a tick later.
        self._obey(now)
        self._fly(now)
        self._prune_reservations()
        self._watchdog(now)
        self._protect_the_pilot(now)
        for flight in self.flights:
            if flight.done:
                continue
            if now >= flight.next_at:
                if flight.observed:
                    self._advance_observed(flight, now)
                elif flight.arriving:
                    self._advance_arrival(flight, now)
                else:
                    self._advance_departure(flight, now)
        # And then the tower looks at what has developed and says something
        # about it, which is the half of this that is not on a timetable.
        self._watch(now)
        # Whoever has just parked for good leaves the frequency for the apron.
        parked = [f for f in self.flights if f.static]
        if parked:
            self.flights = [f for f in self.flights if not f.static]
            for flight in parked:
                # Already in the simulator; it has been all along.
                flight.drawn = True
            self.statics.extend(parked)
            self.pending = [c for c in self.pending
                            if not c.flight.static]

