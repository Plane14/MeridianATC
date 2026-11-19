from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    log,
)


class _The_turnaround_between:
    # -- the turnaround between legs ------------------------------------------

    def _service_turnaround(self, state, now: float) -> bool:
        """Fuel and service for the next leg; True when it may be loaded.

        The engines go off before a hose comes anywhere near the
        aeroplane. The fuel figure is the leg's own plus the diversion to
        the alternate and the reserve -- what a crew would sign for, not
        merely enough to reach the destination. GSX is asked first where
        the stand can ask; a model whose tanks are writable is fuelled
        directly where GSX cannot. A leg that cannot be fuelled for is
        not departed on: the chain ends at the stand rather than in the
        air short of fuel.
        """
        if not self._turnaround_service:
            return True
        if self.state.service_started_at == 0.0:
            self.state.service_started_at = now

        # Engines off first: nobody refuels a running aeroplane, and a
        # rotor that is still turning is worse than a jet that is.
        if getattr(state, "engines_running", False):
            self._shutdown_engines()
            if now - self.state.service_started_at < 60.0:
                return False
            self._emit("info",
                       "Turnaround: engines still running, servicing anyway",
                       "autonomous.service.engines")
        rotor = float(getattr(state, "rotor_rpm_pct", 0.0) or 0.0)
        if (self._is_helicopter and rotor > 1.0
                and now - self.state.service_started_at < 90.0):
            return False

        plan = self.state.plan_flight or {}
        dest = (self.navdb.airport(plan.get("destination", "") or "")
                or self.session.destination)
        if self.state.alternate is None:
            self.state.alternate = self._pick_alternate(state, dest)
        needed = self._fuel_needed_gal(state, destination=dest,
                                       alternate=self.state.alternate,
                                       estimate=True)
        fuel_gal = float(getattr(state, "fuel_total_gal", 0.0) or 0.0)
        self._fuel_capacity_seen = max(
            self._fuel_capacity_seen, fuel_gal,
            float(getattr(state, "fuel_capacity_gal", 0.0) or 0.0))
        if needed is not None:
            target = needed["total_gal"]
        elif self._fuel_capacity_seen > 0.0:
            # No burn figure to plan by: fill most of what it holds.
            target = self._fuel_capacity_seen * 0.9
        else:
            target = fuel_gal
        if self._fuel_capacity_seen > 0.0:
            target = min(target, self._fuel_capacity_seen)
        short = target - fuel_gal

        # What is already aboard may be enough -- tanks that were filled
        # for a longer leg cover a shorter one.
        if short <= 0.5:
            self._emit("info",
                       f"Turnaround: {fuel_gal:.0f} gal aboard covers the "
                       "leg, the diversion and the reserve",
                       "autonomous.service.enough")
            return True

        # GSX first, where the stand can ask for a truck.
        if not self.state.gsx_asked:
            self.state.gsx_asked = True
            asked = self._ask_gsx_refuel()
            if asked:
                self.state.gsx_busy = True
                self._emit("info", f"Turnaround: {asked}",
                           "autonomous.service.gsx")
                return False
            # No truck to ask: the tanks are written where the model's
            # fuel system accepts a write. A write is not the fuel --
            # it is verified against the gauges on the next beat below.
            if self._write_fuel(target):
                self.state.fuel_wrote_at = now
                return False
            self._fuel_or_hold(state, needed, fuel_gal, target)
            return self.state.phase != AutonomousPhase.SHUTDOWN

        # A truck was asked: wait for GSX to say the fuel is on, then
        # check the figure the tanks now report.
        if self.state.gsx_busy:
            if self._gsx_refuel_done() \
                    or now - self.state.service_started_at > 300.0:
                self.state.gsx_busy = False
                fuel_gal = float(getattr(state, "fuel_total_gal", 0.0)
                                 or 0.0)
                if fuel_gal >= target - 0.5:
                    return True
                if self._write_fuel(target):
                    self.state.fuel_wrote_at = now
                    return False
                self._fuel_or_hold(state, needed, fuel_gal, target)
                return self.state.phase != AutonomousPhase.SHUTDOWN
            return False

        # A write the model accepted is believed once the tanks carry
        # it: the uplifts are cross-checked against the gauges before
        # the leg is signed for, and a write that changed nothing --
        # an FMS-side fuel system, a read-only model -- is nobody
        # bringing the fuel, with the same shortfall rules as a truck
        # that never came.
        if self.state.fuel_wrote:
            if fuel_gal >= target - 0.5:
                return True
            if now - self.state.fuel_wrote_at < 15.0:
                return False
            self._fuel_or_hold(state, needed, fuel_gal, target)
            return self.state.phase != AutonomousPhase.SHUTDOWN
        return False

    def _ask_gsx_refuel(self) -> str:
        """Ask GSX for the fuel truck; the words it was asked in, or ""."""
        ground = getattr(self.brain, "ground", None)
        if ground is None or not getattr(ground, "can_ask", False):
            return ""
        try:
            return ground.request("refuel") or ""
        except Exception:
            log.debug("GSX refuel request failed", exc_info=True)
            return ""

    def _gsx_refuel_done(self) -> bool:
        """Whether GSX says the refuelling is finished."""
        ground = getattr(self.brain, "ground", None)
        gstate = getattr(ground, "state", None) if ground is not None else None
        if gstate is None or not getattr(gstate, "available", False):
            return False
        from ...integrations.gsx import COMPLETED, FINISHED
        return gstate.refuelling in (COMPLETED, FINISHED)

    def _write_fuel(self, gallons: float) -> bool:
        """The tanks written to the figure, where the model allows it."""
        if self.sim is None or not getattr(self.sim, "connected", False):
            return False
        try:
            if self.sim.set_fuel(gallons):
                self.state.fuel_wrote = True
                self._emit("info",
                           f"Turnaround: fuelled to {gallons:.0f} gal",
                           "autonomous.service.fuel")
                return True
        except Exception:
            log.debug("turnaround refuel failed", exc_info=True)
        return False

    def _fuel_or_hold(self, state, needed, fuel_gal: float,
                      target: float) -> None:
        """Nobody could bring fuel: go on what is aboard, or do not go.

        Enough for the leg and the reserve -- the diversion figure aside
        -- is departed with, said once; less than the leg itself is not
        departed on at all.
        """
        if needed is not None and fuel_gal >= needed["leg_gal"] \
                + needed["reserve_gal"]:
            self._emit("info",
                       f"Turnaround: no fuel to be had -- departing on "
                       f"{fuel_gal:.0f} gal, short of the diversion "
                       "figure", "autonomous.service.short")
            return
        if needed is None and fuel_gal > 0.0:
            self._emit("info",
                       f"Turnaround: no fuel to be had -- departing on "
                       f"{fuel_gal:.0f} gal", "autonomous.service.short")
            return
        self._emit("error",
                   f"Turnaround: cannot fuel for the leg ({fuel_gal:.0f} "
                   f"gal aboard, {target:.0f} needed) -- ending here",
                   "autonomous.service.nofuel")
        self._transition(AutonomousPhase.SHUTDOWN)

    def _ask_the_ramp(self, state) -> None:
        """The stand's welcome on arrival: a bridge or the stairs, then
        the passengers off, one GSX ask at a time.

        Each menu request abandons a walk still in the menu, so the
        asks are stepped while the engines finish down -- what has not
        been asked by shutdown's end simply is not asked, and a chained
        leg's turnaround does its own servicing later. A ramp nobody
        can ask stays quiet, exactly as a strip with no services does.
        """
        if self._chained:
            return
        ground = getattr(self.brain, "ground", None)
        if ground is None or not getattr(ground, "can_ask", False):
            return
        if getattr(ground, "asking", ""):
            return
        step = self.state.ramp_step
        if step == 0:
            gstate = getattr(ground, "state", None)
            want = ("stairs" if getattr(gstate, "stairs_connected", False)
                    else "jetway")
            try:
                said = ground.request(want) or ""
            except Exception:
                log.debug("ramp %s request failed", want, exc_info=True)
                said = ""
            if said:
                self._emit("info", f"On the stand: {said}",
                           "autonomous.ramp")
            self.state.ramp_step = 1
        elif step == 1:
            try:
                said = ground.request("deboarding") or ""
            except Exception:
                log.debug("ramp deboarding request failed", exc_info=True)
                said = ""
            if said:
                self._emit("info", f"On the stand: {said}",
                           "autonomous.ramp.deboard")
            self.state.ramp_step = 2

    def _phase_error(self, state, now: float) -> None:
        """Error state - disable autonomous flight."""
        self._emit("error", f"Autonomous flight error: {self.state.error_message}",
                   "autonomous.error")
        self.disable()

