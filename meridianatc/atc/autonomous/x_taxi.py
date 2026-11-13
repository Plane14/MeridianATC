from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    Phase,
    auto_nav,
    time,
)


class _Taxi:
    # -- taxi -----------------------------------------------------------------

    def _taxi_along_path(self, state, outbound: bool) -> None:
        """Follow ``session.taxi_path`` at taxi speed, or hold short.

        Outbound the path ends at the holding point; inbound it ends at
        the stand. With no path (an invented route) the aeroplane steers
        for the runway's threshold outbound, and holds once there -- it
        does not invent a taxiway.
        """
        path = list(getattr(self.session, "taxi_path", None) or [])
        if not path or len(path) < 2:
            self._taxi_without_path(state, outbound)
            return

        steer = auto_nav.path_steering(state.latitude, state.longitude, path)
        if steer is None or steer.get("heading") is None:
            self._stop(state)
            return

        # Slow for the end of the route, and for corners: the turn the
        # path asks for sets the speed before the steering has to make
        # it, so a sharp corner is arrived at slowly instead of scrubbed
        # through fast.
        remaining = steer["remaining_m"]
        turn = abs((steer["heading"] - (state.heading_true or 0.0)
                    + 540.0) % 360.0 - 180.0)
        target_kt = self.performance.taxi_kt
        if remaining < 60.0 or turn > 60.0:
            target_kt = max(4.0, target_kt * 0.5)
        if turn > 20.0:
            target_kt = min(target_kt, max(
                4.0, target_kt * (1.0 - (turn - 20.0) / 80.0)))
        if remaining < 25.0:
            target_kt = 0.0

        if target_kt <= 0.0 \
                or self._ground_still(state, time.monotonic()):
            self._stop(state)
        else:
            self._steer(state, steer["heading"], target_kt)

        # Outbound: stop short of a runway crossing the route was cleared
        # to hold at, and of the split point of a cut taxi.
        if outbound:
            if self.session.taxi_split_at is not None:
                lat, lon = self.session.taxi_split_at
                if state.distance_to(lat, lon) * 1852.0 < 60.0:
                    # A taxiway hold, not the runway's: ground gives the
                    # rest of the route here, and HOLDING_SHORT is for
                    # the end of it.
                    self._held_at_split = True
                    self._goto_sub("hold_point")
                    return
            # The path is done: at the hold point. HOLDING_SHORT runs the
            # before-takeoff checklist and waits for the line-up clearance.
            if steer.get("done") or remaining < 30.0:
                self._held_at_split = False
                self._goto_sub("hold_point")
                self._transition(AutonomousPhase.HOLDING_SHORT)
                return
        else:
            # Inbound: the stand is the end of the path.
            if steer.get("done"):
                self._goto_sub("park")
                return

    def _taxi_without_path(self, state, outbound: bool) -> None:
        """No simulator taxiway route: steer at the runway, hold short."""
        if outbound:
            self._find_runway(state)
            if self.session.cleared_takeoff or self.session.phase in (
                    Phase.HOLDING_SHORT, Phase.LINED_UP, Phase.TAKEOFF):
                self._transition(AutonomousPhase.LINE_UP)
                return
            if self.state.runway_heading <= 0:
                return
            rwy_lat = self.state.runway_lat
            rwy_lon = self.state.runway_lon
            if rwy_lat and rwy_lon:
                # A heading is not a route: the runway's bearing from
                # where the aeroplane sits is what reaches it, and the
                # threshold itself is what it holds short of. ``hold_point``
                # does the stopping and the move to HOLDING_SHORT, the
                # same end a planned route arrives at.
                if state.distance_to(rwy_lat, rwy_lon) * 1852.0 <= 90.0:
                    self._goto_sub("hold_point")
                    return
                hdg = auto_nav.bearing_deg(state.latitude, state.longitude,
                                           rwy_lat, rwy_lon)
                self._steer(state, hdg, self.performance.taxi_kt)
            else:
                self._steer(state, self.state.runway_heading,
                            self.performance.taxi_kt)
        else:
            # Inbound with no route yet: the tower has not said where to
            # park, so the aeroplane finishes its rollout and waits --
            # stopping short on the runway is worse than rolling out.
            if self.session.phase == Phase.PARKED:
                self._stop(state)
                if (state.ground_speed_kt or 0.0) <= 1.0:
                    self._transition(AutonomousPhase.PARKED)
            elif (state.ground_speed_kt or 0.0) > self.performance.taxi_kt:
                self.director.throttle(0.0)
                self.director.throttle_idle()
                self.director.spoilers(1.0)
            else:
                # Slow with nowhere to go: stop and wait for the taxi
                # instruction rather than roll the runway's length.
                self._stop(state)

