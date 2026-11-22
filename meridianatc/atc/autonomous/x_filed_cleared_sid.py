from __future__ import annotations

from ._shared import (
    auto_nav,
    procedure_legs,
)


class _Filed_cleared_sid:
    # -- filed/cleared SID/STAR guidance -----------------------------------

    def _route_idents(self) -> tuple:
        route = getattr(self.brain, "route", None)
        filed = [] if route is None else [
            getattr(fix, "ident", "") or ""
            for fix in getattr(route, "fixes", ()) or ()]
        # What the sim's own FMS has loaded is read back as a second
        # source (X-Plane's plugin reports it in its status): the filed
        # route stays first so it wins the ties, and whatever is in the
        # box that the briefing did not mention still corroborates.
        seen = set(filed)
        extra = [ident for ident in
                 dict.fromkeys(getattr(self.brain, "fms_idents", ()) or ())
                 if ident and ident not in seen]
        return tuple(filed) + tuple(extra)

    def _airport_ident(self, airport, fallback: str = "") -> str:
        if airport is not None:
            ident = getattr(airport, "ident", "") or ""
            if ident:
                return ident
        return fallback

    def _runway_start(self, ident: str, runway: str,
                      procedures) -> tuple:
        """Where the SID's first leg begins: the runway end where the
        navdata knows it, the airport reference where it does not."""
        found = auto_nav.runway_end(self.navdb.runways(ident), runway)
        if found is not None:
            return found[0], found[1]
        return (getattr(procedures, "lat", 0.0) or 0.0,
                getattr(procedures, "lon", 0.0) or 0.0)

    def _sid_nav(self, state):
        """The filed/cleared departure procedure's guidance, if any.

        Only a SID the route or the clearance actually names is flown --
        an arbitrary procedure is never invented. The transition is the
        one matching the assigned or planned runway, else the only one
        the procedure has.
        """
        if self.state.procedure_suspended == "sid":
            return None
        if getattr(self.session, "assigned_heading", None) is not None:
            # A vector is off the procedure until ATC says resume own
            # navigation; the heading bug is the clearance, not the leg.
            return None
        route = getattr(self.brain, "route", None)
        ident = self._airport_ident(
            getattr(self.session, "departure", None),
            getattr(route, "origin", "") or "")
        if not ident:
            return None
        procs = getattr(self.brain, "terminal_procedures", {}).get(ident)
        if not procs:
            return None
        name = (getattr(self.session, "sid", "")
                or (getattr(route, "sid", "") if route else "") or "")
        if not name:
            return None
        runway = (getattr(self.session, "departure_runway", "")
                  or self.state.departure_runway
                  or (getattr(route, "sid_runway", "") if route else "")
                  or "")
        transition = (getattr(route, "sid_transition", "")
                      if route else "") or ""
        proc, trans = procedure_legs.choose_sid(
            procs, name, runway, transition, self._route_idents())
        if trans is None:
            return None
        key = (ident, proc.name, trans.name, trans.runway, runway)
        if self.state.sid_key != key or self.state.sid_nav is None:
            start = self._runway_start(ident, runway or trans.runway, procs)
            self.state.sid_nav = procedure_legs.ProcedureNavigator(
                proc, trans, start,
                fallback_altitude_ft=self._altitude_target(state),
                fallback_speed_kt=self._speed_target(state))
            self.state.sid_key = key
            self._emit("info", f"Flying SID {proc.name} "
                       f"{trans.name or trans.runway}".strip(),
                       "autonomous.sid")
        nav = self.state.sid_nav
        nav.fallback_altitude_ft = self._altitude_target(state)
        nav.fallback_speed_kt = self._speed_target(state)
        guidance = nav.guidance(
            state, climb_fpm=self.performance.climb_fpm,
            descend_fpm=self.performance.descend_fpm)
        if guidance.done:
            self.state.sid_nav = None
            self.state.sid_key = ()
            return None
        return guidance

    def _star_nav(self, state):
        """The filed/cleared arrival procedure's guidance, if any."""
        if self.state.procedure_suspended == "star":
            return None
        if getattr(self.session, "assigned_heading", None) is not None:
            return None
        route = getattr(self.brain, "route", None)
        ident = self._airport_ident(
            getattr(self.session, "destination", None),
            getattr(route, "destination", "") or "")
        if not ident:
            return None
        procs = getattr(self.brain, "terminal_procedures", {}).get(ident)
        if not procs:
            return None
        name = (getattr(self.session, "descend_via", "")
                or (getattr(route, "star", "") if route else "") or "")
        if not name:
            return None
        runway = (getattr(self.session, "arrival_runway", "")
                  or self.state.arrival_runway
                  or (getattr(route, "star_runway", "") if route else "")
                  or "")
        transition = (getattr(route, "star_transition", "")
                      if route else "") or ""
        proc, trans = procedure_legs.choose_star(
            procs, name, runway, transition, self._route_idents())
        if trans is None:
            return None
        key = (ident, proc.name, trans.name, trans.runway, runway)
        if self.state.star_key != key or self.state.star_nav is None:
            self.state.star_nav = procedure_legs.ProcedureNavigator(
                proc, trans,
                (state.latitude, state.longitude),
                fallback_altitude_ft=(
                    getattr(self.session, "assigned_altitude_ft", 0.0)
                    or self._descent_floor(state)),
                fallback_speed_kt=(
                    getattr(self.session, "assigned_speed_kt", 0.0)
                    or (self.state.vapp_kt
                        or self.performance.approach_kt) + 20.0))
            self.state.star_key = key
            self._emit("info", f"Descending via STAR {proc.name} "
                       f"{trans.name or trans.runway}".strip(),
                       "autonomous.star")
        nav = self.state.star_nav
        nav.fallback_altitude_ft = (
            getattr(self.session, "assigned_altitude_ft", 0.0)
            or self._descent_floor(state))
        nav.fallback_speed_kt = (
            getattr(self.session, "assigned_speed_kt", 0.0)
            or (self.state.vapp_kt
                or self.performance.approach_kt) + 20.0)
        guidance = nav.guidance(
            state, climb_fpm=self.performance.climb_fpm,
            descend_fpm=self.performance.descend_fpm)
        if guidance.done:
            self.state.star_nav = None
            self.state.star_key = ()
            return None
        return guidance

    def _fly_procedure(self, state, guidance) -> None:
        """Put a procedure leg's targets on the autopilot.

        An assigned heading, level or speed still outranks the
        procedure's own targets -- ATC's word is the clearance.
        """
        assigned_hdg = self.session.assigned_heading
        heading = assigned_hdg if assigned_hdg is not None else (
            self._magnetic(guidance.heading, state)
            if guidance.heading is not None else -1.0)
        assigned_alt = self.session.assigned_altitude_ft or 0.0
        altitude = assigned_alt or guidance.altitude_ft \
            or self._altitude_target(state)
        if assigned_alt:
            current = self._flown_altitude_ft(state)
            if current < assigned_alt - 250.0:
                vs = self.performance.climb_fpm
            elif current > assigned_alt + 250.0:
                vs = -self.performance.descend_fpm
            else:
                vs = 0.0
        else:
            vs = guidance.vs_fpm
        speed = (self.session.assigned_speed_kt
                 or guidance.speed_kt or self._speed_target(state))
        self._apply_flight_director(
            state, altitude=altitude, vs_fpm=vs,
            heading=heading, speed_kt=speed)

