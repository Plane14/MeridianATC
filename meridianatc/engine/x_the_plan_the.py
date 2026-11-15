from __future__ import annotations

from ._shared import (
    decision,
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

    def _use_sim_plan(self, plan) -> None:
        """What the simulator is flying, as the flight's route.

        MSFS answers with the .pln path on disk; X-Plane's plugin reads
        the FMS itself and answers with the entry list.
        """
        # The route this adopts is the box's own plan coming back: the
        # flag keeps use_route's upload from sending it straight back in,
        # which on MSFS would rewrite the file the read is parked on.
        self._adopting_sim_plan = True
        try:
            if isinstance(plan, (list, tuple)):
                self._use_fms_plan(plan)
                return
            self._use_pln_plan(plan)
        finally:
            self._adopting_sim_plan = False

    # The nav types an FMS entry can be and still be a route point --
    # approach elements (ILS, localizer, glideslope, markers) are skipped.
    _FMS_ROUTE_TYPES = {1: "apt", 2: "ndb", 4: "vor", 512: "wpt",
                        1024: "wpt", 2048: "wpt", 4096: "vor"}

    def _use_fms_plan(self, entries) -> None:
        """The plugin's FMS entry list as the flight's route.

        Every entry carries its own position, so nothing needs resolving;
        the fingerprint keeps the same plan from being rebuilt every
        pass of the poll.
        """
        fingerprint = tuple(
            (str(e.get("id", "")), round(float(e.get("lat", 0) or 0), 5),
             round(float(e.get("lon", 0) or 0), 5), int(e.get("alt", 0) or 0))
            for e in entries if isinstance(e, dict))
        old = getattr(self, "_sim_plan_fp", None)
        if fingerprint == old:
            return
        if old is not None and len(fingerprint) < len(old) \
                and old[len(old) - len(fingerprint):] == fingerprint:
            # The FMS sequenced past its head entries -- the same plan
            # with legs already passed consumed. That is this plan being
            # flown, not a new one: take the shorter fingerprint so the
            # next sequence is seen too, but keep the route and the
            # session the flight was built around.
            self._sim_plan_fp = fingerprint
            return
        route = self._route_from_fms(entries)
        if route is None:
            return
        self._sim_plan_fp = fingerprint
        current = self.route
        if current is not None and [
                (f.ident, round(f.lat, 5), round(f.lon, 5))
                for f in route.fixes] == [
                (f.ident, round(f.lat, 5), round(f.lon, 5))
                for f in current.fixes]:
            # The plan the box shows is already the flight's -- this is
            # an upload of ours landing, or the box re-reading itself.
            # Re-adopting it as a new flight would restart a session in
            # the middle of the taxi it is already on.
            return
        self.use_route(route)
        if self.session is not None:
            self.begin_session()
        self.emit("info", f"Flight plan from the simulator: {route.described()}",
                  key="log.simplan")

    def _route_from_fms(self, entries):
        """FMS entry dicts (id/t/alt/lat/lon/active) as a Route."""
        from ..atc.route import Route, RouteFix

        fixes = []
        for e in entries[:512]:
            if not isinstance(e, dict):
                continue
            try:
                kind = self._FMS_ROUTE_TYPES.get(int(e.get("t", 0) or 0))
                lat, lon = float(e.get("lat", 0) or 0), \
                    float(e.get("lon", 0) or 0)
                alt = int(e.get("alt", 0) or 0)
            except (TypeError, ValueError):
                continue
            if kind is None \
                    or not (-90 <= lat <= 90 and -180 <= lon <= 180):
                continue
            ident = str(e.get("id") or "").strip().upper() \
                or f"{lat:+.4f}{lon:+.4f}"
            fixes.append(RouteFix(ident=ident, lat=lat, lon=lon,
                                  alt_ft=alt,
                                  kind=kind))
        if len(fixes) < 2:
            return None
        airports = [f.ident for f in fixes if f.kind == "apt"]
        return Route(fixes=tuple(fixes),
                     origin=airports[0] if airports else "",
                     destination=airports[-1] if airports else "",
                     source="sim")

    def _use_pln_plan(self, path: str) -> None:
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
        if os.path.abspath(path) == getattr(
                self.source, "_pln_uploaded", ""):
            # The active .pln is the one this engine wrote for
            # ``FlightPlanLoad`` -- the echo of the upload, not a plan
            # the pilot loaded. Adopting it would reset a live session
            # over its own upload.
            self._sim_plan_path = path
            self._sim_plan_mtime = mtime
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

    def _upload_fms_plan(self, route) -> None:
        """The same plan into the simulator's own box, so the FMS flies
        what the engine thinks was filed -- the chained legs included.

        Every route adoption lands here through ``use_route``: the OFP,
        a .pln loaded by hand, a typed string, the next leg's DCT, the
        en-route re-plan. Two of them are skipped: a route the simulator
        itself reported (``source="sim"``, or anything adopted under the
        flag -- on MSFS it would otherwise rewrite the very file the
        readback is parked on), and a link that cannot take a plan, whose
        sources simply lack ``load_fms_plan``.
        """
        if route is None or route.source == "sim" or not route.fixes:
            return
        if getattr(self, "_adopting_sim_plan", False):
            return
        loader = getattr(self.source, "load_fms_plan", None)
        if not callable(loader):
            return
        try:
            loaded = loader(route)
        except Exception:
            log.debug("the simulator's FMS load raised", exc_info=True)
            return
        decision("route", event="fms_loaded" if loaded else "fms_refused",
                 source=route.source, route=route.described())

