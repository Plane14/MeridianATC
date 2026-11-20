from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    Phase,
    auto_nav,
    haversine_nm,
    log,
    time,
)


def _pursuit_leg(heading: float, path: list, seg: int) -> float:
    """The leg direction the pursuit's ask is bounded against.

    The ask is joining whichever leg it least departs from, not
    blindly the nearest segment: at a hairpin's apex both legs share
    the vertex, the strict distance tie in ``path_steering`` keeps the
    incoming one, and a bound measured off it clamps the return turn
    away from the route.
    """
    seg = min(seg, len(path) - 2)
    leg = auto_nav.bearing_deg(path[seg][0], path[seg][1],
                               path[seg + 1][0], path[seg + 1][1])
    if seg + 2 < len(path):
        nxt = auto_nav.bearing_deg(path[seg + 1][0], path[seg + 1][1],
                                   path[seg + 2][0], path[seg + 2][1])
        if abs((heading - nxt + 540.0) % 360.0 - 180.0) \
                < abs((heading - leg + 540.0) % 360.0 - 180.0):
            leg = nxt
    return leg


def _bounded_intercept(heading: float, leg_hdg: float,
                       rejoining: bool) -> float:
    """The pursuit's ask, bounded against the leg's own direction.

    A point beside the line is rejoined at a steady intercept, not
    steered at whatever bearing the lookahead happens to publish: the
    unbounded ask is what swung the demand ninety degrees a tick either
    side of the line -- the hunt for the taxi target. Off the line the
    intercept is a rejoin angle; on it, the ask stays inside a sweep
    the nose can make without crossing back through it.
    """
    intercept = ((heading - leg_hdg + 540.0) % 360.0) - 180.0
    cap = 65.0 if rejoining else 45.0
    return (leg_hdg + max(-cap, min(cap, intercept))) % 360.0


class _Taxi:
    # -- taxi -----------------------------------------------------------------

    # Far enough off the cleared route that aiming for it is a diagonal
    # across the apron -- through whatever scenery the taxiway graph does
    # not know is there. Closer than this the aeroplane rejoins; past it
    # it stops, because a stopped aeroplane hits nothing.
    OFF_PATH_STOP_M = 50.0
    REJOIN_M = 15.0

    # The corridor the cleared route is given the benefit of: narrower than
    # a parallel taxiway's spacing, wide enough that an aeroplane ahead on
    # the same line is seen, and the look ahead is far enough to stop for
    # whatever is coming the other way rather than meet it.
    CONFLICT_CORRIDOR_M = 30.0
    CONFLICT_LOOK_M = 90.0
    # The hold is the route's last point -- the holding point the planner
    # picked: stop with the nose just short of the bars, not a taxiway
    # length back from them. The sim reports the aeroplane's datum, not
    # its nose, so the datum's distance is the class's own nose radius
    # plus the margin the paint deserves.
    HOLD_BARS_MARGIN_M = 4.0
    # Held off the cleared line this long, the route is asked for again:
    # planned from where the aeroplane actually is, it is a route it can
    # rejoin rather than one it holds short of for ever. Twice is enough
    # to tell a stuck aeroplane from a slow one.
    REASK_AFTER_S = 60.0
    REASK_MAX = 2

    # The pedal check every taxi begins with: once the aeroplane is
    # rolling, the brakes are squeezed once -- a pedal that answers
    # nothing is found on the ramp, not at the holding bars.
    BRAKE_CHECK_KT = 3.0
    BRAKE_CHECK_S = 1.5
    BRAKE_CHECK_PRESS = 0.4

    def _taxi_along_path(self, state, outbound: bool) -> None:
        """Follow ``session.taxi_path`` at taxi speed, or hold short.

        Outbound the path ends at the holding point; inbound it ends at
        the stand. With no path (an invented route) the aeroplane steers
        for the runway's threshold outbound, and holds once there -- it
        does not invent a taxiway.
        """
        path = list(getattr(self.session, "taxi_path", None) or [])
        if not getattr(state, "taxi_light", False):
            # The light a moving aeroplane shows, whatever the checklist
            # left unsaid -- "as required" answers on some types never
            # reach a switch, and a dark aeroplane on the taxiway is how
            # ground collisions start.
            self.director.taxi_light(True)
        # A runway crossed on the way, granted or simply met, is strobes
        # on until the far side is behind it.
        self._clearance_strobes(state)
        if getattr(self.session, "hold_position", False):
            # "Hold position" is a stop where you are, not a hold at the
            # route's end: the brakes go on until the next movement
            # instruction -- a new route, a crossing grant, a line-up
            # or the takeoff word -- releases it.
            self._stop(state)
            return
        if not path or len(path) < 2:
            self._taxi_without_path(state, outbound)
            return

        # A new clearance is a new walk from the start.
        if getattr(self, "_taxi_route", None) is not self.session.taxi_path:
            self._taxi_route = self.session.taxi_path
            self._taxi_progress = 0
            self._off_path_since = 0.0
            self._taxi_reask = False

        # The walk only moves forward: the nearest segment of what is
        # left, never of what is behind. A whole-path search re-picks a
        # nearer parallel leg or a segment already passed, and the
        # pursuit target teleporting between them is the weave that
        # leaves the taxiway.
        at = min(getattr(self, "_taxi_progress", 0), len(path) - 2)
        remaining_path = path[at:]
        # The pursuit's lead scales with speed -- thirty metres at a
        # standstill, about eight seconds of travel once rolling -- so a
        # slow aeroplane rejoins instead of cutting every junction the
        # way a fixed airliner lookahead does.
        gs = state.ground_speed_kt or 0.0
        lookahead = max(30.0, min(auto_nav.TAXI_LOOKAHEAD_M,
                                gs * 1.852 * 8.0 / 3.6))
        steer = auto_nav.path_steering(state.latitude, state.longitude,
                                       remaining_path, lookahead_m=lookahead)
        if steer is None or steer.get("heading") is None:
            self._stop(state)
            return
        self._taxi_progress = at + steer["nearest"]

        # Off the route the fixed lookahead is a diagonal across the
        # apron. Near the line, rejoin it gently; far from it the line
        # back is scenery the graph does not know, and the aeroplane
        # stops rather than find out what.
        off = abs(steer["cross_track_m"])
        if off > self.OFF_PATH_STOP_M:
            self._stop(state)
            now = time.monotonic()
            if not getattr(self, "_off_path_since", 0.0):
                self._off_path_since = now
            elif now - self._off_path_since > self.REASK_AFTER_S \
                    and getattr(self, "_taxi_reasks", 0) < self.REASK_MAX \
                    and self._taxi_issuing_position() is not None:
                # Stopped where no rejoin is safe: asking for the route
                # again gets one planned from where the aeroplane is,
                # which is a route it can follow rather than a line to
                # hold short of for ever. The clock restarts so the
                # flag is raised once per hold, not once a tick.
                log.info("off the taxi route for %.0f s: asking for "
                         "the route again", now - self._off_path_since)
                self._taxi_reasks = getattr(self, "_taxi_reasks", 0) + 1
                self._taxi_reask = True
                self._off_path_since = now
            warned_at = getattr(self, "_off_path_warned_at", 0.0)
            if now - warned_at > 15.0:
                self._off_path_warned_at = now
                log.warning(
                    "the aeroplane is %.0f m off the taxi route (heading "
                    "%.0f for %.0f, leg %d of %d); holding rather than "
                    "crossing the apron",
                    off, state.heading_true or 0.0,
                    steer["heading"] or 0.0,
                    getattr(self, "_taxi_progress", 0), len(path) - 1)
            return
        self._off_path_since = 0.0
        rejoining = off > self.REJOIN_M
        if rejoining:
            rejoin = auto_nav.path_steering(
                state.latitude, state.longitude, remaining_path,
                lookahead_m=min(30.0, off))
            if rejoin is not None and rejoin.get("heading") is not None:
                steer["heading"] = rejoin["heading"]

        # The pursuit's ask, bounded against the leg's own direction: a
        # point beside the line is rejoined at a steady intercept, not
        # steered at whatever bearing the lookahead happens to publish.
        leg_hdg = _pursuit_leg(steer["heading"], remaining_path,
                               steer["nearest"])
        steer["heading"] = _bounded_intercept(
            steer["heading"], leg_hdg, rejoining)

        # The corridor the rest of the route runs through: whatever is on
        # it ahead -- head-on, crossing, or stopped -- is given way to
        # rather than met. The AI gives the pilot the same courtesy at
        # 35 m; the FO stops further out than that, so in a head-on it is
        # the one already holding when the other aeroplane arrives.
        if self._yield_to_traffic(state, remaining_path):
            self._stop(state)
            warned_at = getattr(self, "_yield_warned_at", 0.0)
            if time.monotonic() - warned_at > 30.0:
                self._yield_warned_at = time.monotonic()
                log.info("traffic on the taxi route ahead: holding")
            return

        # Slow for the end of the route, and for corners: the turn the
        # path asks for sets the speed before the steering has to make
        # it, so a sharp corner is arrived at slowly instead of scrubbed
        # through fast.
        remaining = steer["remaining_m"]
        # The route's last point, as the crow flies: the outbound hold is
        # judged by distance to the bars, not path left, so a curved last
        # leg still stops with the nose at them.
        end_m = haversine_nm(state.latitude, state.longitude,
                             path[-1][0], path[-1][1]) * 1852.0
        turn = abs((steer["heading"] - (state.heading_true or 0.0)
                    + 540.0) % 360.0 - 180.0)
        target_kt = self._expedite_kt(state, self.performance.taxi_kt)
        if rejoining:
            # Back to the line at walking pace: crossing the apron is a
            # job for the slowest speed the aeroplane will hold.
            target_kt = min(target_kt, 5.0)
        if remaining < 60.0 or turn > 60.0:
            target_kt = max(4.0, target_kt * 0.5)
        if turn > 90.0:
            # Nearly reversed: the turn itself is the job, at walking
            # pace. Driven faster the rotation outruns the stale ticks
            # and the aeroplane orbits the point instead of lining up.
            target_kt = min(target_kt, 2.5)
        if turn > 20.0:
            target_kt = min(target_kt, max(
                4.0, target_kt * (1.0 - (turn - 20.0) / 80.0)))
        if outbound:
            # Creep the last metres to the bars instead of stopping a
            # taxiway short of them.
            if end_m <= self._bars_stop_m():
                target_kt = 0.0
            elif end_m < 60.0:
                target_kt = min(target_kt, 4.0)
        elif remaining < 25.0:
            target_kt = 0.0

        if target_kt <= 0.0 \
                or self._ground_still(state, time.monotonic()):
            self._stop(state)
        else:
            now = time.monotonic()
            # The brake check rides the steering rather than replacing
            # it: the pedals are squeezed once as the roll begins, and
            # the aeroplane keeps tracking the line through it.
            until = getattr(self, "_brake_check_until", 0.0)
            if not getattr(self, "_brake_checked", False):
                if not until and gs >= self.BRAKE_CHECK_KT:
                    until = self._brake_check_until = now + \
                        self.BRAKE_CHECK_S
                if until and now >= until:
                    self._brake_checked = True
                    self._brake_check_until = 0.0
                    until = 0.0
            desired = steer["heading"]
            if not rejoining and turn < 20.0 and gs > 4.0:
                from .. import tailwheel as _tw
                if _tw.is_tailwheel(state):
                    # The nose hides the taxiway ahead: a taildragger
                    # taxis a slow S so the pilot sees past it.
                    desired = (desired
                               + _tw.s_turn_offset(now)) % 360.0
            # The situation's press on the steering ask: coming back to
            # the line from off it, or round a real corner, wants the
            # nose moved now; tracking the centre of a straight leg
            # wants the soft hand that does not chase the last degree.
            if rejoining or turn > 45.0:
                demand = 1.3
            elif turn < 5.0 and off < 4.0:
                demand = 0.8
            else:
                demand = 1.0
            self._steer(state, desired, target_kt, demand=demand)
            if until:
                self.director.throttle(0.0)
                self.director.brakes_split(self.BRAKE_CHECK_PRESS,
                                           self.BRAKE_CHECK_PRESS)

        # Outbound: stop short of a runway crossing the route was cleared
        # to hold at, and of the split point of a cut taxi.
        if outbound:
            if self.session.taxi_split_at is not None:
                lat, lon = self.session.taxi_split_at
                if haversine_nm(state.latitude, state.longitude,
                                lat, lon) * 1852.0 < 60.0:
                    # A taxiway hold, not the runway's: ground gives the
                    # rest of the route here, and HOLDING_SHORT is for
                    # the end of it.
                    self._held_at_split = True
                    self._goto_sub("hold_point")
                    return
            # At the bars -- or stopped just short of them and unable to
            # close the last metres: holding either way. HOLDING_SHORT
            # runs the before-takeoff checklist and waits for the line-up
            # clearance.
            bars_m = self._bars_stop_m()
            if end_m <= bars_m or (
                    end_m <= bars_m * 3
                    and (state.ground_speed_kt or 0.0) <= 1.0
                    and self._sub_age() > 10.0):
                self._held_at_split = False
                self._goto_sub("hold_point")
                self._transition(AutonomousPhase.HOLDING_SHORT)
                return
        else:
            # Inbound: the stand is the end of the path.
            if steer.get("done"):
                self._goto_sub("park")
                return

    def _bars_stop_m(self) -> float:
        """Metres the datum keeps from the bars for the nose to sit just
        short of them.

        The sim reports where the datum is; the nose is the class's own
        radius ahead of it -- a piston's a couple of strides, an
        airliner's a bus length -- so the fixed metre stop that puts a
        transport at the paint parks a light aeroplane halfway down the
        link. Wake-heavy and super fly a little more nose.
        """
        from .. import aircraft_class as ac
        cls = getattr(self, "_aircraft_class", "") or ac.PISTON_SINGLE
        nose = ac.stand_radius_m(cls)
        wake = getattr(getattr(self.session, "aircraft", None), "wake", "")
        if wake == "super":
            nose += 10.0
        elif wake == "heavy":
            nose += 6.0
        return nose + self.HOLD_BARS_MARGIN_M

    def _expedite_kt(self, state, base_kt: float) -> float:
        """The taxi speed the tower's "expedite" buys, while it stands.

        The call has a shelf life (``session.expedite_until``): while it
        stands, and only on the runway it was called for, the aeroplane
        is taxied briskly -- about double the book's figure, capped
        where a laden aeroplane still holds the turnoffs. It decays back
        to the book's figure across the last seconds of the window
        rather than stepping off it, and it is nothing once the window
        or the runway has gone: the corners, the rejoin and the bars
        keep their own slowdowns on top of whatever this answers.
        """
        until = float(getattr(self.session, "expedite_until", 0.0) or 0.0)
        left = until - time.monotonic()
        if left <= 0.0 or not self._on_the_runway_now(state):
            return base_kt
        brisk = min(30.0, base_kt * 2.2)
        blend = min(1.0, left / 15.0)
        return base_kt + (brisk - base_kt) * blend

    def _on_the_runway_now(self, state) -> bool:
        """Whether the aeroplane is on a runway it was assigned.

        Where that cannot be told -- no position, no runway named --
        the expedite's own context answers for it: "expedite" is a
        runway call, so the benefit of the doubt is the runway's.
        """
        if not getattr(state, "has_position", False):
            return True
        session = self.session
        runways = ((session.departure, session.departure_runway),
                   (session.destination, session.arrival_runway))
        named = any(r for _, r in runways)
        if not named:
            return True
        from types import SimpleNamespace

        from ..occupancy import on_the_runway
        me = SimpleNamespace(
            latitude=state.latitude, longitude=state.longitude,
            altitude_ft=float(getattr(state, "altitude_ft", 0.0) or 0.0),
            on_ground=True,
            heading=float(getattr(state, "heading_true", 0.0)
                          or getattr(state, "heading_magnetic", 0.0) or 0.0),
            ground_speed_kt=0.0)
        return any(
            field is not None and runway
            and on_the_runway(self.navdb, field, runway, [me])
            for field, runway in runways)

    def _clearance_strobes(self, state) -> None:
        """Strobes for a clearance, or a runway, the aeroplane is on.

        The switch is the clearance and the pavement, not the phase: on
        the moment the line-up, takeoff or runway-crossing clearance is
        in hand -- the call a crew makes before the bars -- and on for
        any runway the aeroplane stands on, granted or simply met,
        because the strip itself is what they warn of. A light the
        crossing burned goes out once the far side is behind it; one
        the checklist or the departure burned stays for whoever owns
        it -- the rollout's own clear-of-runway rule, or the parking
        checklist.
        """
        session = self.session
        departing = getattr(session, "lined_up", False) \
            or getattr(session, "cleared_takeoff", False) \
            or getattr(session, "phase", None) in (
                Phase.LINED_UP, Phase.TAKEOFF)
        crossing = bool(getattr(session, "crossing_granted", "")) \
            or bool(getattr(session, "occupying_runway", ""))
        lit = bool(getattr(state, "strobe_light", False))
        if (departing or crossing) and not lit:
            self.director.strobes(True)
            self._strobe_for_crossing = crossing and not departing
        elif lit and not departing and not crossing:
            if getattr(self, "_strobe_for_crossing", False):
                # Vacated the strip the grant was for: the crossing's
                # own lights go out with it.
                self.director.strobes(False)
            self._strobe_for_crossing = False
        elif lit:
            # Lit some other way -- the checklist's -- but burning
            # through a crossing all the same: the crossing keeps the
            # off owed.
            self._strobe_for_crossing = crossing and not departing

    def _yield_to_traffic(self, state, path) -> object | None:
        """The first aeroplane on the corridor the route uses, if any.

        ``path`` is what is left of the route in lat/lon. On any failure
        to look, the answer is none: a picture that cannot be read is not
        a reason to park for ever.
        """
        try:
            found = auto_nav.traffic_ahead(
                state.latitude, state.longitude, list(path),
                self._surface_contacts(),
                corridor_m=self.CONFLICT_CORRIDOR_M,
                look_m=self.CONFLICT_LOOK_M)
        except Exception:
            log.debug("could not look up the taxi route's traffic",
                      exc_info=True)
            return None
        return found[0][0] if found else None

    def _taxi_without_path(self, state, outbound: bool) -> None:
        """No simulator taxiway route: steer at the runway, hold short."""
        self._clearance_strobes(state)
        if outbound:
            self._find_runway(state)
            if self.session.cleared_takeoff or self.session.phase in (
                    Phase.HOLDING_SHORT, Phase.LINED_UP, Phase.TAKEOFF):
                self._transition(AutonomousPhase.LINE_UP)
                return
            if self.state.runway_heading <= 0:
                return
            # Where nobody issues a route -- an empty frequency, an
            # advisory service -- the taxi is the crew's own navigation,
            # and where the airport's own taxiways are known it is
            # flown on them rather than straight across the apron.
            if self._taxi_issuing_position() is None:
                if self._self_plan_taxi(state):
                    return
                # Still reading the field's taxiways is not "there are
                # none": the straight line is for the read that came
                # back empty, not the one that has not come back yet.
                if self._layout_still_coming():
                    self._stop(state)
                    return
            rwy_lat = self.state.runway_lat
            rwy_lon = self.state.runway_lon
            if rwy_lat and rwy_lon:
                # A heading is not a route: the runway's bearing from
                # where the aeroplane sits is what reaches it, and the
                # threshold itself is what it holds short of. ``hold_point``
                # does the stopping and the move to HOLDING_SHORT, the
                # same end a planned route arrives at.
                if haversine_nm(state.latitude, state.longitude,
                                rwy_lat, rwy_lon) * 1852.0 <= 90.0:
                    self._goto_sub("hold_point")
                    return
                # No route, so the corridor is the way being steered.
                hdg = auto_nav.bearing_deg(state.latitude, state.longitude,
                                           rwy_lat, rwy_lon)
                if self._yield_to_traffic(
                        state, [(state.latitude, state.longitude),
                                (rwy_lat, rwy_lon)]):
                    self._stop(state)
                    return
                self._steer(state, hdg,
                            self._expedite_kt(state, self.performance.taxi_kt))
            else:
                self._steer(state, self.state.runway_heading,
                            self._expedite_kt(state, self.performance.taxi_kt))
        else:
            # Inbound with no route yet: the tower has not said where to
            # park, so the aeroplane finishes its rollout and waits --
            # stopping short on the runway is worse than rolling out.
            if self.session.phase == Phase.PARKED:
                self._stop(state)
                if (state.ground_speed_kt or 0.0) <= 1.0:
                    self._transition(AutonomousPhase.PARKED)
            elif (state.ground_speed_kt or 0.0) > self._expedite_kt(
                    state, self.performance.taxi_kt):
                self.director.throttle(0.0)
                self.director.throttle_idle()
                self.director.spoilers(1.0)
            else:
                # Slow with nowhere to go: stop and wait for the taxi
                # instruction rather than roll the runway's length.
                self._stop(state)

    # A failed self-plan is not retried every tick -- the answer is the
    # same until the layout arrives or the aeroplane has moved on.
    SELF_PLAN_RETRY_S = 10.0

    def _self_plan_taxi(self, state) -> bool:
        """Plan the taxi-out on the airport's own taxiways, where nobody
        will give a route.

        At a field with no staffed surface position the way to the
        runway is the crew's own navigation: a real crew follows the
        airport diagram, not the crow. Where the simulator has said
        what the taxiways are, the aeroplane is taxied on them -- and
        where it has not, this answers False and the old rule holds:
        steer for the threshold. Returns True once a path is on the
        session for ``_taxi_along_path`` to pick up.
        """
        now = time.monotonic()
        if now - getattr(self, "_self_plan_at", float("-inf")) < self.SELF_PLAN_RETRY_S:
            return False
        self._self_plan_at = now
        ask = getattr(getattr(self, "brain", None), "ground_layout", None)
        airport = getattr(self.session, "departure", None) \
            or self._get_current_airport()
        if ask is None or airport is None or self.navdb is None:
            return False
        runway = self.session.departure_runway \
            or (self.state.runway_key[1] if self.state.runway_key else "")
        if not runway:
            return False
        try:
            layout = ask(airport.ident)
        except Exception:
            log.debug("could not ask for %s's taxiways", airport.ident,
                      exc_info=True)
            return False
        if layout is None:
            return False
        from .. import taxi as taxiplan
        try:
            lines = taxiplan.runway_lines(
                layout, self.navdb.runways(airport.ident))
            busy = taxiplan.busy_runways(
                layout, self.navdb, airport.ident,
                (getattr(self.session, "departure_runway", ""),
                 getattr(self.session, "arrival_runway", ""), runway),
                weather=self._field_weather(airport))
            from .. import aircraft_class
            need_m = aircraft_class.min_runway_ft(
                getattr(state, "type_code", "") or "") * 0.3048
            plan, _why = taxiplan.plan_departure(
                layout, state.latitude, state.longitude, runway, lines,
                busy=busy, need_m=need_m)
        except Exception:
            log.debug("self-planned taxi at %s raised", airport.ident,
                      exc_info=True)
            return False
        if plan is None or len(plan.points or ()) < 2:
            return False
        self.session.taxi_path = list(plan.points)
        self.session.lineup_path = list(getattr(plan, "lineup", ()))
        self.session.taxi_crossings = list(plan.crossings)
        self.session.crossing_index = 0
        self.session.crossing_granted = ""
        log.info("%s: taxiing on the airport's own taxiways "
                 "(no route issued)", airport.ident)
        return True

    def _layout_still_coming(self) -> bool:
        """Whether this field's taxiways are still being read -- an
        answer that is coming, told apart from one that never did."""
        pending = getattr(getattr(self, "brain", None),
                          "ground_pending", None)
        if pending is None:
            return False
        airport = getattr(self.session, "departure", None) \
            or self._get_current_airport()
        if airport is None:
            return False
        try:
            return bool(pending(airport.ident))
        except Exception:
            return False

