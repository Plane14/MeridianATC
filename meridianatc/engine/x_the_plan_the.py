from __future__ import annotations

from ._shared import (
    log,
    os,
    time,
)


class _The_plan_the:
    # ------------------------------------------------------------------
    # the plan the simulator itself is flying
    # ------------------------------------------------------------------

    _SIM_PLAN_POLL_S = 10.0

    def _poll_sim_plan(self) -> None:
        """Ask the simulator which flight plan it has loaded, and take it.

        ``RequestSystemState("FlightPlan")`` is answered on the wrapper's
        dispatch thread, so the ask and the answer are split over two
        passes of this loop -- the loop never waits on the reply.
        """
        if not getattr(self.config.immersion, "autonomous_read_sim_plan",
                       True):
            return
        source = self.source
        ask = getattr(source, "request_flight_plan", None)
        take = getattr(source, "flight_plan_reply", None)
        if ask is None or take is None \
                or not getattr(source, "connected", False):
            return
        now = time.monotonic()
        if self._sim_plan_requested:
            reply = take()
            if reply is not None:
                self._sim_plan_requested = False
                self._sim_plan_seen = reply
            elif now - self._sim_plan_at > self._SIM_PLAN_POLL_S:
                self._sim_plan_requested = False    # the answer never came
            else:
                return
        if not self._sim_plan_requested \
                and now - self._sim_plan_at >= self._SIM_PLAN_POLL_S:
            self._sim_plan_at = now
            self._sim_plan_requested = bool(ask())
        # Adopted on the ground only: a plan the sim swapped to
        # mid-flight is picked up after the landing, not flown from under
        # the leg already being flown.
        if self._sim_plan_seen and getattr(self.state, "on_ground", True):
            self._use_sim_plan(self._sim_plan_seen)

    def _use_sim_plan(self, path: str) -> None:
        """The simulator's active .pln as the flight's route.

        Loaded once per file: the same path at the same modified time is
        the same plan, and never reparsed. A route the pilot loaded by
        hand is only replaced when the simulator's plan itself changes --
        which is exactly the moment the sim's plan is the newer one.
        """
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return
        if path == self._sim_plan_path and mtime == self._sim_plan_mtime:
            return
        try:
            described = self.load_route_file(path)
        except Exception as exc:
            log.info("simulator flight plan %s could not be loaded: %s",
                     path, exc)
            return
        self._sim_plan_path = path
        self._sim_plan_mtime = mtime
        self.emit("info", f"Flight plan from the simulator: {described}",
                  key="log.simplan")

