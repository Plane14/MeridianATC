from __future__ import annotations

from ._shared import (
    _CIRCUIT_PHASES,
    ACCELERATE_AT_FT,
    AIRLINER_FOOTPRINT_M,
    CIRCUIT_CENTRELINE_NM,
    CIRCUIT_UPWIND_NM,
    CIRCUIT_WIDTH_NM,
    CLIMB_TO_FT,
    FLARE_FPM,
    FLARE_FT,
    GLIDE_FT_PER_NM,
    INITIAL_CLIMB_FT,
    INTERCEPT_LEAD_NM,
    INTERCEPT_LIMIT_DEG,
    LIFTOFF_AFTER_S,
    LIGHT_FOOTPRINT_M,
    LIGHT_PERFORMANCE,
    TAXIING_KT,
    Flight,
    _turn_radius_nm,
    approach_speed,
    bearing_deg,
    circuitry,
    glide_altitude,
    haversine_nm,
    math,
    project,
)


class _Flying_them:
    # ------------------------------------------------------------------
    # flying them
    # ------------------------------------------------------------------

    def _start_flying(self, flight: Flight) -> None:
        """Hand an aeroplane to the flight model.

        Everything after this point is integrated -- a heading swings round at
        three degrees a second, a runway takes half a minute to roll down --
        so this only has to say where it is the moment the model takes over.
        """
        end = self._threshold(flight.runway)
        if end is None:
            return
        lat, lon, heading = end
        elevation = self.field.elevation_ft
        threshold_ft = self._threshold_ft(flight.runway)
        flight.airborne_model = True
        flight.heading = heading

        if flight.arriving and not flight.ifr and flight.phase == "inbound":
            # A light aircraft arriving at an airfield is not on an instrument
            # approach and does not appear on the centreline eight miles out.
            # It comes in from wherever it has been, at circuit height, and
            # joins where the tower tells it to.
            side = -1.0 if flight.pattern_direction != "right" else 1.0
            from_where = (heading + 180.0 + 45.0 * side) % 360.0
            flight.latitude, flight.longitude = project(
                lat, lon, from_where, max(1.0, flight.distance_nm))
            flight.altitude_ft = (elevation
                                  + flight.performance.pattern_agl_ft + 500.0)
            flight.ground_speed_kt = flight.performance.cruise_kt
            flight.heading = (from_where + 180.0) % 360.0
            flight.on_ground = False
            return

        if flight.arriving:
            # An IFR arrival on a STAR starts at the arrival's own entry
            # fix and is flown down it; everything else is on the
            # extended centreline at the distance it is reporting, on a
            # three-degree path, at the speed that distance implies.
            if flight.procedure_pair is not None and self._begin_star(
                    flight):
                return
            behind = (heading + 180.0) % 360.0
            flight.latitude, flight.longitude = project(
                lat, lon, behind, max(0.2, flight.distance_nm))
            flight.altitude_ft = glide_altitude(threshold_ft,
                                                flight.distance_nm,
                                                GLIDE_FT_PER_NM)
            flight.ground_speed_kt = approach_speed(flight.distance_nm,
                                                    flight.performance)
            flight.on_ground = False
            return

        # A departure joins the model at the threshold, stopped -- or, having
        # taxied onto the runway itself, wherever it now is.
        if not flight.on_layout:
            flight.latitude, flight.longitude = project(lat, lon, heading,
                                                        0.04)
        flight.altitude_ft = self._surface_ft(flight)
        flight.ground_speed_kt = 0.0
        flight.on_ground = True

    def _stop_flying(self, flight: Flight) -> None:
        """Take an aeroplane off the model and back onto a taxiway."""
        flight.airborne_model = False
        flight.assigned_heading = None
        flight.assigned_altitude_ft = None
        flight.assigned_speed_kt = None
        flight.procedure_nav = None
        flight.procedure_pair = None
        flight.target.clear()

    def _begin_star(self, flight: Flight) -> bool:
        """Put an IFR arrival at its STAR's entry fix with the legs to
        follow, and the navigator that steers them.

        False when the transition's path cannot be built at all -- the
        caller then spawns it the generic way, exactly as a field with no
        procedures does.
        """
        from ..procedure_legs import ProcedureNavigator

        proc, trans = flight.procedure_pair
        nav = ProcedureNavigator(
            proc, trans,
            fallback_altitude_ft=flight.altitude_ft,
            fallback_speed_kt=flight.performance.cruise_kt)
        if not nav.active:
            return False
        first = trans.first_fix
        if first is not None and first.has_fix:
            flight.latitude, flight.longitude = first.lat, first.lon
        else:
            flight.latitude, flight.longitude = nav.points[0][:2]
        # The level it checks in at: the STAR's own constraint where the
        # legs give one, the arrival height the spawn worked out where
        # they do not.
        level = next((leg.altitude_constraint or leg.fix_alt_ft
                      for leg in trans.legs
                      if leg.altitude_constraint or leg.fix_alt_ft), 0.0)
        if level:
            flight.altitude_ft = level
        if len(nav.points) > 1:
            flight.heading = bearing_deg(
                flight.latitude, flight.longitude, *nav.points[1][:2])
        flight.ground_speed_kt = flight.performance.cruise_kt
        flight.on_ground = False
        flight.procedure_nav = nav
        end = self._threshold(flight.runway)
        if end is not None:
            flight.distance_nm = haversine_nm(
                flight.latitude, flight.longitude, end[0], end[1])
        return True

    def _aim(self, flight: Flight) -> None:
        """What this aeroplane is trying to do, right now.

        The phase says what it would be doing if nobody had said anything; an
        assigned heading, level or speed says what it was told, and outranks
        it. That order is the whole of what makes an instruction an
        instruction: the aeroplane goes back to flying the phase only when the
        assignment is taken away.
        """
        target = flight.target
        target.hold = False
        target.turn = ""
        target.accel_kt_s = None
        target.max_descent_fpm = None
        end = self._threshold(flight.runway)
        heading = end[2] if end is not None else flight.heading
        elevation = self.field.elevation_ft
        # Where the wheels are, which on a sloping runway is not the field's
        # published elevation; see _surfaces.
        surface = self._surface_ft(flight)
        performance = flight.performance
        phase = flight.phase

        if phase == "lineup":
            target.heading = heading
            target.altitude_ft = surface
            target.speed_kt = 0.0
            target.hold = True
            target.intent = "lined up"
        elif phase == "takeoff":
            target.heading = heading
            target.speed_kt = performance.climb_kt
            if flight.on_ground:
                target.altitude_ft = surface
                target.accel_kt_s = performance.takeoff_kt_s
                target.intent = "rolling"
            else:
                target.altitude_ft = elevation + INITIAL_CLIMB_FT
                target.intent = "climbing out"
                if flight.altitude_ft < surface + ACCELERATE_AT_FT:
                    # V2 plus a little to the acceleration height, nose high,
                    # rather than flattening out to gather speed off the
                    # runway.
                    target.speed_kt = min(performance.climb_kt,
                                          performance.rotate_kt + 20.0)
        elif phase == "departed":
            target.heading = heading
            target.speed_kt = performance.cruise_kt
            target.altitude_ft = elevation + CLIMB_TO_FT
            target.intent = "climbing out"
        elif phase in _CIRCUIT_PHASES:
            self._circuit(flight, heading, elevation)
        elif phase == "go_around":
            target.heading = heading
            target.speed_kt = performance.climb_kt
            # An IFR go-around climbs to the plate's missed altitude on
            # the runway heading; a VFR one to pattern height.
            target.altitude_ft = (
                getattr(flight, "missed_alt_ft", 0.0)
                or elevation + performance.pattern_agl_ft)
            target.intent = "going around"
            missed_nav = getattr(flight, "missed_nav", None)
            if missed_nav is not None:
                # The published missed steers, the way the SID and STAR
                # legs do elsewhere; done or not, _after_the_go_around
                # decides what comes after it.
                guide = missed_nav.guidance(flight)
                if not guide.done:
                    if guide.heading is not None:
                        target.heading = guide.heading
                    if guide.altitude_ft:
                        target.altitude_ft = guide.altitude_ft
                    if guide.speed_kt:
                        target.speed_kt = guide.speed_kt
                    target.intent = "missed approach"
        elif phase == "missed_hold":
            # Orbiting the missed's hold fix: the navigator's own
            # racetrack, looped so it is flown round and round until the
            # tower's clock lets it back into the arrivals.
            nav = getattr(flight, "missed_nav", None)
            bare = list(getattr(nav, "_bare", None) or [])
            from ..auto_nav import path_steering
            steer = path_steering(
                flight.latitude, flight.longitude,
                bare + bare[:4] if len(bare) > 1 else bare) if bare else None
            target.heading = (steer or {}).get("heading") or heading
            target.altitude_ft = flight.missed_hold_alt_ft
            target.speed_kt = performance.climb_kt * 0.9
            target.intent = "holding on the missed approach"
        elif phase == "joining":
            target.heading = self._bearing_to_join(flight, heading)
            target.speed_kt = performance.cruise_kt * 0.85
            target.altitude_ft = elevation + performance.pattern_agl_ft
            target.intent = "joining downwind"
        elif phase in ("inbound", "final"):
            target.heading = self._intercept(flight, heading)
            target.speed_kt = approach_speed(flight.distance_nm, performance)
            target.altitude_ft = glide_altitude(
                self._threshold_ft(flight.runway), flight.distance_nm,
                GLIDE_FT_PER_NM)
            if phase == "inbound" \
                    and self._missed_return_active(flight, heading):
                # The reversal is not the glide: hold the missed's
                # level until the turn-in and the intercept can take
                # it down.
                target.altitude_ft = (
                    getattr(flight, "missed_alt_ft", 0.0)
                    or elevation + performance.pattern_agl_ft)
            target.intent = "on approach" if phase == "inbound" else "on final"
            height = flight.altitude_ft - surface
            if phase == "final" and height < FLARE_FT:
                # The flare: the descent eased off from the glide path's rate
                # to a gentle touchdown over the last forty feet.
                glide_fpm = flight.ground_speed_kt * GLIDE_FT_PER_NM / 60.0
                target.max_descent_fpm = max(
                    FLARE_FPM, glide_fpm * max(0.0, height) / FLARE_FT)
                # Down to the runway under it, not to the threshold's
                # height: the flare carries it past the threshold, and on a
                # runway that falls away (Madeira's 23 drops 45 ft) it held
                # the threshold's altitude above the slope for ever.
                target.altitude_ft = surface
        elif phase == "landed":
            target.heading = heading
            target.altitude_ft = surface
            target.speed_kt = 0.0
            target.intent = "landing roll"
        else:
            target.speed_kt = 0.0
            target.hold = True
            target.intent = phase

        # A SID or STAR's legs steer the phase the way a "climb via" or
        # "descend via" does: the procedure's path is what it flies until
        # the legs run out or the tower says something else. A hold is
        # something else: the clearance, for as long as it lasts, is the
        # racetrack rather than the legs -- which resume when it ends.
        if flight.hold is not None:
            self._steer_hold(flight, target)
        else:
            self._steer_procedure(flight, target)
        # And what it was told, which beats all of that.
        if flight.assigned_heading is not None:
            target.heading = flight.assigned_heading
        if flight.assigned_altitude_ft is not None:
            target.altitude_ft = flight.assigned_altitude_ft
        if (flight.assigned_speed_kt is not None and not target.hold
                and not flight.on_ground):
            target.speed_kt = flight.assigned_speed_kt
        if getattr(flight, "expedite_until", 0.0):
            # "Expedite": a higher aim speed, and a firmer push into it,
            # until the moment passes or the runway is given back.
            if not target.hold and target.speed_kt is not None:
                cap = (performance.cruise_kt if not flight.on_ground
                       else performance.rotate_kt * 0.6)
                target.speed_kt = min(target.speed_kt * 1.25 + 10.0, cap)
            if flight.on_ground:
                target.accel_kt_s = max(target.accel_kt_s or 0.0,
                                        performance.takeoff_kt_s * 1.3)

    def _steer_procedure(self, flight: Flight, target) -> None:
        """The SID or STAR legs as this tick's aim.

        Built the first time the model asks, from wherever the aeroplane
        then is -- for a departure that is the end of the runway, for an
        arrival the entry fix it spawned at. The legs' own levels and
        speed limits come with the path; when they are done the aeroplane
        goes back to flying its phase, and anything the tower assigns
        still outranks it afterwards.
        """
        if (flight.procedure_pair is None or flight.on_ground
                or not flight.airborne_model):
            return
        if flight.phase not in ("takeoff", "departed", "inbound", "final"):
            return
        nav = flight.procedure_nav
        if nav is None:
            from ..procedure_legs import ProcedureNavigator

            proc, trans = flight.procedure_pair
            nav = ProcedureNavigator(
                proc, trans, start=(flight.latitude, flight.longitude),
                fallback_altitude_ft=target.altitude_ft
                or self.field.elevation_ft + CLIMB_TO_FT,
                fallback_speed_kt=target.speed_kt
                or flight.performance.cruise_kt)
            flight.procedure_nav = nav
        guide = nav.guidance(flight)
        if guide.done:
            flight.procedure_nav = None
            flight.procedure_pair = None
            return
        if guide.heading is None:
            return
        target.heading = guide.heading
        if guide.altitude_ft:
            target.altitude_ft = guide.altitude_ft
        if guide.speed_kt:
            target.speed_kt = min(guide.speed_kt, flight.performance.cruise_kt)
        target.intent = (f"on the {flight.sid} departure" if flight.sid
                         else f"on the {flight.star} arrival" if flight.star
                         else target.intent)

    # Faster than this a held aeroplane's pattern stops looking like a
    # hold and starts looking like a tour of the county. ICAO's tables
    # cap holding speeds far lower; this is the number that looks right
    # on the map for anything from a turboprop up.
    HOLD_SPEED_KT = 220.0
    # Inside this of the fix the inbound leg is done and the turn out
    # begins -- a little anticipation, the way a pilot leads the turn.
    HOLD_FIX_REACH_NM = 0.4

    def _steer_hold(self, flight: Flight, target) -> None:
        """The racetrack: in to the fix, a turn out, the leg, a turn in.

        Right-hand pattern, legs as long as the clearance said -- a
        minute at or below 14,000 ft, a minute and a half above, the same
        numbers the pilot's own holds are worked with. The inbound legs
        are steered at the fix rather than on the course, which pulls the
        pattern back onto the fix whatever the wind did to the turn.
        """
        hold = flight.hold
        if (hold is None or flight.on_ground or not flight.airborne_model
                or flight.phase not in ("inbound", "final")):
            return
        now = self._now()
        to_fix = haversine_nm(flight.latitude, flight.longitude,
                              hold.lat, hold.lon)
        if flight.hold_leg == "out":
            # Away from the fix on the reciprocal for as long as the leg
            # runs; the turn at the end of it is the flight model's, at
            # its own rate.
            if now - flight.hold_leg_at >= hold.leg_minutes * 60.0:
                flight.hold_leg = "in"
                flight.hold_leg_at = now
            else:
                target.heading = (hold.inbound_true + 180.0) % 360.0
        if flight.hold_leg != "out":
            # Inbound: at the fix, or pointing at it from wherever the
            # last turn left it. Over the top of it the outbound leg
            # begins.
            target.heading = bearing_deg(flight.latitude, flight.longitude,
                                         hold.lat, hold.lon)
            if to_fix < self.HOLD_FIX_REACH_NM:
                flight.hold_leg = "out"
                flight.hold_leg_at = now
        target.speed_kt = min(flight.performance.cruise_kt,
                              self.HOLD_SPEED_KT)
        target.altitude_ft = hold.altitude_ft
        target.intent = (f"holding at {hold.fix}" if hold.fix
                         else "holding")

    def _circuit(self, flight: Flight, runway_heading: float,
                 elevation: float) -> None:
        """Fly the circuit, one leg at a time.

        The legs are decided by where the aeroplane actually is rather than by
        a timer, in the runway's own coordinates: how far along the takeoff
        direction it is from the threshold, and how far off to the side. That
        is what makes "extend downwind" a real instruction -- the aeroplane
        goes past the point it would have turned, and keeps going until it is
        told to turn base.
        """
        target = flight.target
        performance = flight.performance
        pattern_ft = elevation + performance.pattern_agl_ft
        along, across = self._runway_frame(flight, runway_heading)
        side = -1.0 if flight.pattern_direction != "right" else 1.0
        # A circuit speed, not a cruise: an airliner going round at two
        # hundred and twenty knots flew a circuit four miles wide.
        target.speed_kt = min(performance.cruise_kt * 0.8,
                              performance.approach_kt * 1.45)
        target.altitude_ft = pattern_ft

        if flight.phase == "upwind":
            target.heading = runway_heading
            target.intent = "upwind"
            if along >= CIRCUIT_UPWIND_NM:
                flight.phase = "crosswind"
                # A new circuit: the last one's short approach is spent.
                flight.short_approach = False
                flight.tight_base = False
        elif flight.phase == "crosswind":
            target.heading = (runway_heading + 90.0 * side) % 360.0
            target.intent = "crosswind"
            if across * side >= CIRCUIT_WIDTH_NM:
                flight.phase = "downwind"
        elif flight.phase == "downwind":
            target.heading = (runway_heading + 180.0) % 360.0
            target.intent = ("extending downwind" if flight.extending
                             else "downwind")
            # An extension that has been transmitted but not yet read back is
            # still an extension. Turning base in the gap would be a crew that
            # heard the tower and did it anyway.
            holding = flight.extending or self._already_told(flight, "extend")
            if not holding and along <= -self._base_nm(flight):
                if self._room_on_final(
                        -along, ignore=flight,
                        within_s=self._rollout_s(flight, across)):
                    flight.phase = "base"
                else:
                    # No gap by the time it would be there. It carries on
                    # down the downwind rather than starting a turn it would
                    # have to abandon a mile from the centreline -- inside
                    # its own turning circle, which is across the approach.
                    flight.extending = True
        elif flight.phase == "base":
            target.heading = (runway_heading - 90.0 * side) % 360.0
            target.intent = "base"
            target.altitude_ft = elevation + performance.pattern_agl_ft * 0.6
            target.speed_kt = performance.approach_kt * 1.15
            # Rolling out on the centreline rather than through it: the turn
            # starts one turn radius short of it.
            if abs(across) <= max(CIRCUIT_CENTRELINE_NM,
                                  _turn_radius_nm(flight.ground_speed_kt)):
                # Committed: the gap was judged for this moment when the base
                # turn began. Turning away from here, inside its own turning
                # circle, is what took the last one across the approach; if
                # the spacing has closed anyway, the tower's watch on final
                # sends one of them round.
                flight.phase = "final"
                flight.step = 0
                flight.arriving = True
                flight.distance_nm = max(1.0, -along)

    def _join_point(self, flight: Flight,
                    runway_heading: float) -> tuple[float, float] | None:
        """Where the downwind starts, for an aeroplane joining the circuit.

        Abeam the upwind end, out on the circuit side. A VFR arrival told to
        join left downwind flies to this point and turns onto the leg, which
        is what it was told to do -- rather than appearing on it.
        """
        end = self._threshold(flight.runway)
        if end is None:
            return None
        return circuitry.join_point(end[0], end[1], runway_heading,
                                    flight.pattern_direction)

    def _bearing_to_join(self, flight: Flight, runway_heading: float) -> float:
        """Which way to point to get to the start of the downwind."""
        point = self._join_point(flight, runway_heading)
        if point is None:
            return flight.heading
        return bearing_deg(flight.latitude, flight.longitude, point[0], point[1])

    def _distance_to_join(self, flight: Flight) -> float:
        """How far it still has to fly before it is on the downwind."""
        end = self._threshold(flight.runway)
        point = self._join_point(flight, end[2]) if end else None
        if point is None:
            return 0.0
        return haversine_nm(flight.latitude, flight.longitude,
                            point[0], point[1])

    def _intercept(self, flight: Flight, runway_heading: float) -> float:
        """The heading that closes the centreline, rather than parallels it.

        An aeroplane that rolls out of a turn a few hundred yards wide and
        then flies the runway heading stays a few hundred yards wide all the
        way to the threshold. This aims it at a point on the centreline ahead
        of it, which brings it in and then keeps it there.
        """
        along, across = self._runway_frame(flight, runway_heading)
        if self._missed_return_active(flight, runway_heading):
            # The course reversal: back over the field and out the
            # other end on the reciprocal, until there is room to turn
            # in -- the way a missed without legs (or a vectored
            # re-sequence) actually comes round. Aiming at the final
            # alone just orbited the strip.
            end = self._threshold(flight.runway)
            if end is None:
                return flight.heading
            if along > 0.0:
                point = project(end[0], end[1],
                                (runway_heading + 180.0) % 360.0,
                                circuitry.LONG_FINAL_NM + 2.0)
                return bearing_deg(flight.latitude, flight.longitude,
                                   point[0], point[1])
            return (runway_heading + 180.0) % 360.0
        angle = math.degrees(math.atan2(-across, INTERCEPT_LEAD_NM))
        angle = max(-INTERCEPT_LIMIT_DEG, min(INTERCEPT_LIMIT_DEG, angle))
        return (runway_heading + angle) % 360.0

    def _missed_return_active(self, flight: Flight,
                              runway_heading: float) -> bool:
        """The missed-approach return still in its course reversal.

        True while the aeroplane is upfield of the threshold, and while
        it is outbound past it with the turn-in still to do -- called
        off once it is out far enough that the intercept onto final
        closes. Only the legless return flies this; coded legs, VFR
        and ordinary arrivals keep their own steerings.
        """
        if getattr(flight, "phase", "") != "inbound" \
                or not getattr(flight, "missed_alt_ft", 0.0) \
                or getattr(flight, "missed_nav", None) is not None:
            return False
        along, _ = self._runway_frame(flight, runway_heading)
        if along > 0.0:
            return True
        err = abs((runway_heading - flight.heading + 180.0) % 360.0
                  - 180.0)
        return err > 60.0 \
            and along > -(circuitry.LONG_FINAL_NM + 2.0)

    def _runway_frame(self, flight: Flight,
                      runway_heading: float) -> tuple[float, float]:
        """Where an aeroplane is in the runway's own coordinates.

        ``(along, across)`` in nautical miles from the threshold: along is
        positive in the direction of takeoff, across is positive to the right
        of it. Every decision about the circuit, the approach and whether
        somebody is on the runway is made in those two numbers.
        """
        end = self._threshold(flight.runway)
        if end is None:
            return 0.0, 0.0
        return circuitry.runway_frame(flight.latitude, flight.longitude,
                                      end[0], end[1], runway_heading)

    def _after_flying(self, flight: Flight) -> None:
        """Read back off the aeroplane whatever the rest of the module asks it.

        The distance to run and whether it is on the ground are facts about
        where it now is, not numbers something else has to remember to keep in
        step with it.
        """
        end = self._threshold(flight.runway)
        if end is None:
            return
        along, _across = self._runway_frame(flight, end[2])
        if flight.arriving or flight.phase in _CIRCUIT_PHASES:
            flight.distance_nm = max(0.0, -along)
        else:
            flight.distance_nm = max(0.0, along)

        performance = flight.performance
        if flight.phase == "takeoff":
            if not flight.on_ground:
                pass                     # once off, it stays off
            elif flight.ground_speed_kt < performance.rotate_kt:
                flight.rotate_at = None
            else:
                now = self._flown_at
                if flight.rotate_at is None:
                    flight.rotate_at = now
                # Rotated, and off once the nose is high enough.
                if now - flight.rotate_at >= LIFTOFF_AFTER_S:
                    flight.on_ground = False
        elif flight.phase in ("lineup", "landed"):
            flight.on_ground = True
        else:
            flight.on_ground = (flight.altitude_ft
                                <= self._surface_ft(flight) + 3.0)

    def positions(self, invented_only: bool = False,
                  parked: bool = False) -> list[dict]:
        """Where every live aeroplane is, for the map and the simulator.

        ``invented_only`` leaves out the ones the simulator is already
        holding. The map wants all of them -- they are all out of the window
        -- and the injector in :mod:`meridianatc.integrations.aitraffic` wants
        only the ones it is responsible for, or it would create a second copy
        of every FSLTL aeroplane a foot away from the first.

        ``parked`` adds the aeroplanes standing at the gates, this program's
        and everybody else's. The map wants those too; the strips do not,
        because a dozen aeroplanes that will say nothing would bury the
        three that will.
        """
        # The live ones (atc/live.py) are observed -- nobody instructs them --
        # but this program draws them, so they go to the injector with the
        # invented ones; FSLTL's and the simulator's never do.
        found = [f.snapshot() for f in self.flights
                 if not f.done and f.positioned
                 and not (invented_only and f.observed and f.source != "live")
                 and not (invented_only and f.redraw)]
        if invented_only:
            # A recovery's redraw (``_recover``): left out of the simulator's
            # list once, so its object is removed, and created again on the
            # next tick at the same spot.
            for flight in self.flights:
                flight.redraw = False
        if parked and not invented_only:
            # The apron as well: this program's own parked aeroplanes, and
            # the ones FSLTL and the simulator stood there. Neither is on
            # the radio, both are out of the window.
            found += [f.snapshot() for f in self.statics
                      if f.positioned and (f.drawn or not self.requires_sim)]
            found += [one.snapshot(self.field) for one in
                      self._parked_observed.values()]
        if not invented_only:
            # The map also wants where each one is going: the rest of its
            # route, as the aeroplane itself would fly it. The simulator does
            # not, and is asked for these a dozen times a second.
            by_callsign = {f.written: f for f in self.flights}
            for snapshot in found:
                flight = by_callsign.get(snapshot["callsign"])
                if flight is not None:
                    snapshot.update(self.route_ahead(flight))
        if invented_only:
            # What the simulator still has to make. It is asked for these and
            # nobody else hears of them until it has.
            found += [f.snapshot() for f in self._materialising
                      if f.positioned]
            # The parked ones last, so that where the simulator can only draw
            # so many it is the ones standing still that go without. They
            # are for the simulator only: the panel's list and its maps are
            # of the traffic on the frequency, and a dozen aeroplanes that
            # will say nothing would bury the three that will.
            found += [f.snapshot() for f in self.statics if f.positioned]
            # And the live ones standing at the gates, which this program
            # draws as well.
            found += [one.snapshot(self.field) for one in
                      self._parked_observed.values() if one.live]
            # And how each is moving, unrounded, with the way ahead on the
            # ground: the simulator draws them between ticks from this
            # (meridianatc.atc.follow), rather than jumping once a tick.
            by_callsign = {f.written: f for f in self.flights}
            by_callsign.update({f.written: f for f in self._materialising})
            by_callsign.update({f.written: f for f in self.statics})
            for snapshot in found:
                flight = by_callsign.get(snapshot["callsign"])
                if flight is not None:
                    snapshot["motion"] = self.motion_of(flight)
            # The simulator draws the first traffic.in_simulator_limit of
            # these: the ones that matter to the pilot first (the pilot's
            # rule, 30 Oct 2026) -- the runway and the final, then the rest
            # of the air, then the taxiways, then the parked ones nearest
            # first. An aeroplane on final is never left out for one parked
            # across the airport.
            found.sort(key=self._relevance)
        return found

    def _relevance(self, snapshot: dict) -> tuple:
        phase = snapshot.get("phase", "")
        on_ground = bool(snapshot.get("on_ground"))
        if phase in ("final", "takeoff", "lineup", "landed", "go_around") \
                or (not on_ground and phase in ("departed", "circuit")):
            rank = 0
        elif not on_ground:
            rank = 1
        elif phase not in ("parked",) and (
                snapshot.get("ground_speed_kt", 0) >= TAXIING_KT
                or phase in ("taxi", "taxi_in", "pushback", "holding")):
            rank = 2
        else:
            rank = 3
        user = self.user
        if user is not None and user.has_position:
            here = (user.latitude, user.longitude)
        else:
            here = (self.field.latitude, self.field.longitude)
        far = haversine_nm(here[0], here[1], snapshot.get("lat", here[0]),
                           snapshot.get("lon", here[1]))
        return rank, far

    #: The most route points handed to the injector's follower: a few
    #: hundred metres of taxiway, all it can draw ahead within a tick.
    MOTION_POINTS = 12

    def motion_of(self, flight: Flight) -> dict:
        """How an aeroplane is moving, for drawing it between ticks."""
        motion = {
            "t": self._flown_at or 0.0,
            "lat": flight.latitude, "lon": flight.longitude,
            "alt": flight.altitude_ft, "heading": flight.heading,
            "gs": flight.ground_speed_kt, "vs": flight.vertical_rate_fpm,
            "on_ground": bool(flight.on_ground),
            "arriving": bool(flight.arriving),
            # A light aircraft rotates less and climbs flatter.
            "light": flight.performance.climb_fpm <= LIGHT_PERFORMANCE.climb_fpm,
            "rotate_at": flight.rotate_at,
            "surface_ft": (self._surface_ft(flight) if flight.runway
                           else self.field.elevation_ft),
            "route": [], "push": 0,
        }
        taxi = flight.taxi
        layout = self.ground
        if (taxi is not None and layout is not None and not taxi.done
                and not flight.airborne_model):
            ahead = taxi.points[taxi.index:taxi.index + self.MOTION_POINTS]
            motion["route"] = [layout.to_latlon(x, z) for x, z in ahead]
            # And the two points behind it, so a corner it is half way round
            # is the same curve from one tick to the next.
            motion["behind"] = [layout.to_latlon(x, z) for x, z in
                                taxi.points[max(0, taxi.index - 2):taxi.index]]
            motion["push"] = max(0, taxi.push - taxi.index)
            if taxi.face is not None and taxi.index + len(ahead) >= len(taxi.points):
                motion["face"] = taxi.face
        return motion

    def standing(self) -> list:
        """Every invented aeroplane on a stand, for the ground handling.

        The ones set out at the gates, an arrival that has just stopped on
        its stand, and a departure boarded and waiting for its push: each
        with its stand's geometry and which of those it is, in the shape
        :class:`meridianatc.atc.handling.Standing` wants. Somebody else's
        aeroplanes are never in it -- the simulator handles its own -- and
        neither is anything that has started to move.
        """
        from ..handling import ARRIVED, BOARDING, DEPARTING, PARKED, Standing, is_light

        if self.ground is None:
            return []
        found: list = []

        def add(flight: Flight, status: str) -> None:
            if flight.stand is None or not flight.positioned:
                return
            if not 0 <= flight.stand < len(self.ground.stands):
                return
            stand = self.ground.stands[flight.stand]
            x, z = self.ground.to_xy(flight.latitude, flight.longitude)
            # Boarding, its doors close when the boarding does: the holds
            # first, and the bags must be on by then.
            closes = (flight.boarding_at + self.boarding_s
                      if status == BOARDING and flight.boarding_at else None)
            found.append(Standing(
                callsign=flight.written, x=x, z=z, heading=flight.heading,
                radius=stand.radius, status=status, since=flight.parked_at,
                light=is_light(flight.type_code, flight.operator is None),
                gate=stand.gate, type_code=flight.type_code,
                operator=flight.operator.icao if flight.operator else "",
                closes_at=closes))

        for flight in self.statics:
            if flight.drawn or not self.requires_sim:
                add(flight, BOARDING if flight.boarding else PARKED)
        for one in self._parked_observed.values():
            # On the stand it is nearest, if it is on one; otherwise on its
            # own footprint, which is all the geometry a vehicle needs.
            x, z = self.ground.to_xy(one.latitude, one.longitude)
            stand = self.ground.stand_at(x, z)
            light = is_light(one.type_code, not one.type_code)
            # A live one is drawn by this program, so it is handled as one of
            # ours: found by its callsign, not by somebody else's object.
            found.append(Standing(
                callsign=one.callsign if one.live else one.key,
                x=x, z=z, heading=one.heading,
                radius=(stand.radius if stand is not None
                        else (LIGHT_FOOTPRINT_M if light
                              else AIRLINER_FOOTPRINT_M)),
                status=ARRIVED if one.since else PARKED, since=one.since,
                light=light, gate=stand.gate if stand is not None else False,
                type_code=one.type_code, observed=not one.live,
                object_id=0 if one.live else one.object_id, title=one.title))
        for flight in self.flights:
            if flight.observed or flight.done or flight.static:
                continue
            if not flight.on_ground or flight.ground_speed_kt > 0.5:
                continue
            if flight.taxi is not None:
                continue
            if flight.arriving:
                if flight.parked_at:
                    add(flight, ARRIVED)
            else:
                add(flight, DEPARTING)
        return found

    def flight_named(self, callsign: str) -> Flight | None:
        """One of this program's aeroplanes by callsign, wherever it is."""
        for flight in (self.statics + self.flights + self._materialising
                       + self._asked_for):
            if flight.written == callsign and not flight.done:
                return flight
        return None

    def apron_state(self, callsign: str
                    ) -> tuple[str, tuple[float, float] | None]:
        """Where one of this program's aeroplanes is in its day, for the
        schedule's choice of model, and the stand it turns round on.

        One of "static" (standing on its stand), "booked" (to depart,
        boarding not started), "boarding", "departing" (boarded, on its
        stand, not yet moving), "arrived" (blocked in, still on the
        radio), "arriving" (on its way to a stand it holds) or "moving".
        The stand is in the layout's metres, or None.
        """
        def stand_xy(index: int | None) -> tuple[float, float] | None:
            if (index is None or self.ground is None
                    or not 0 <= index < len(self.ground.stands)):
                return None
            stand = self.ground.stands[index]
            return stand.x, stand.z

        for flight in self.statics:
            if flight.written == callsign:
                kind = ("boarding" if flight.boarding
                        else "booked" if flight.boarding_at else "static")
                return kind, stand_xy(flight.stand)
        for flight in self.flights + self._materialising + self._asked_for:
            if flight.written != callsign or flight.done:
                continue
            still = flight.on_ground and flight.ground_speed_kt <= 0.5
            if flight.arriving:
                if flight.parked_at and still:
                    return "arrived", stand_xy(flight.stand)
                return "arriving", stand_xy(flight.planned_stand
                                            if flight.planned_stand is not None
                                            else flight.stand)
            if still and flight.taxi is None and flight.stand is not None:
                return "departing", stand_xy(flight.stand)
            return "moving", None
        return "moving", None

    # How long the simulator has to produce an invented aeroplane before it
    # is dropped. Long enough for a refused title and a second one to be
    # tried; short enough that a flight nobody will ever see does not sit in
    # the queue for the next one.
    MATERIALISE_S = 30.0

    def _admit(self, flight: Flight, now: float) -> None:
        """Put a new invented aeroplane where it belongs: on the roster, or
        waiting for the simulator to draw it first."""
        if self.requires_sim:
            self._materialising.append(flight)
            self._materialise_by[flight.written] = now + self.MATERIALISE_S
        else:
            self.flights.append(flight)

    def confirm(self, drawn, failed, now: float
                ) -> tuple[list[Flight], list[Flight]]:
        """Admit what the simulator has drawn and drop what it has not.

        ``drawn`` and ``failed`` answer for a callsign: whether the
        simulator has the aeroplane, and whether it has given up on it.
        Returns the flights that have just become real and the ones that
        were dropped, so the caller can say which.
        """
        confirmed: list[Flight] = []
        dropped: list[Flight] = []
        waiting: list[Flight] = []
        for flight in self._materialising:
            if drawn(flight.written):
                # It has been sitting still while the simulator made it; it
                # does not then say two things in the same second.
                flight.next_at = max(flight.next_at, now + 1.0)
                self.flights.append(flight)
                confirmed.append(flight)
            elif (failed(flight.written)
                  or now > self._materialise_by.get(flight.written, now)):
                if flight.source == "live":
                    # The sim cannot draw ours, but the real aeroplane
                    # is still on the feed: the strip reads it there
                    # rather than vanishing and coming back as a new one.
                    self._unadopt(flight, now,
                                  why="simulator cannot draw it")
                    self.flights.append(flight)
                    self._materialise_by.pop(flight.written, None)
                    continue
                flight.done = True
                dropped.append(flight)
            else:
                waiting.append(flight)
        self._materialising = waiting
        for flight in confirmed + dropped:
            self._materialise_by.pop(flight.written, None)
        # A parked one the simulator cannot draw is simply not there. It is
        # not reported: nobody was told about it.
        for flight in self.statics:
            flight.drawn = bool(drawn(flight.written))
        self.statics = [f for f in self.statics
                        if f.drawn or not failed(f.written)]
        return confirmed, dropped

    def _drop_invented(self) -> None:
        """Take every invented aeroplane away: the simulator is not drawing
        them, so they are not there. What somebody else put in the
        simulator stays -- it is out of the window whatever this does."""
        self._asked_for.clear()
        self._materialising.clear()
        self._materialise_by.clear()
        # The live ones adopted under traffic.live.managed are not
        # invented: a real aeroplane stays what it is for the invented
        # switch, and is dropped only when traffic is off altogether.
        keep = ("live", "taken over")
        self.statics = [f for f in self.statics if f.source in keep]
        self._stands_filled = False
        gone = {id(f) for f in self.flights
                if not f.observed and f.source not in keep}
        if gone:
            for f in self.flights:
                if id(f) in gone:
                    self._release_held(f)
            self.flights = [f for f in self.flights if id(f) not in gone]
            self.pending = [c for c in self.pending
                            if id(c.flight) not in gone]

    def _wind_runway(self) -> str:
        """The runway in use for the wind, or the longest one if there is none."""
        weather = self.field.weather
        found = self.navdb.best_runway(
            self.field.ident,
            weather.wind_dir if weather else 0.0,
            weather.wind_kt if weather else 0.0,
            variable=bool(weather and weather.variable),
            gust_kt=weather.gust_kt if weather else 0.0,
        )
        return found[0] if found else ""

