from __future__ import annotations

from ._shared import (
    AutonomousPhase,
)


class _Emergencies:
    # -- emergencies ------------------------------------------------------------

    # A projected shortage has to persist to be declared: the need is
    # priced on the flow of the moment, and climb power in the first
    # cruise minute reads triple what the rest of the leg burns -- a
    # projection priced at it cries emergency over healthy tanks the
    # same way takeoff and climb do. "Cannot reach the field" settles
    # faster than "below leg + diversion + reserve": a MAYDAY is held
    # back less than a PAN PAN.
    LEG_FUEL_PERSIST_S = 45.0
    MIN_FUEL_PERSIST_S = 120.0
    # A bus dying stays collapsed through a transient the way a real
    # failure does; a combustion flag is a slower read, so a blip of
    # that size is the one a frame of noise makes.
    ELECTRICAL_PERSIST_S = 8.0
    ENGINES_PERSIST_S = 5.0

    def _monitor_emergency(self, state, now: float) -> None:
        """Watch for the failures the state can actually see.

        Fuel, the bus and per-engine combustion are readable; what cannot
        be seen is not invented. All engines out is a MAYDAY; one out of
        several is a PAN PAN and a diversion, because a twin on one is an
        urgency, not yet a distress. Fuel is judged the way a crew plans
        it: under the leg alone is the MAYDAY; under leg + diversion +
        reserve is minimum fuel, a PAN PAN.
        """
        # ``emergency_declared`` does not end the watch: a PAN PAN can
        # still become a MAYDAY -- the minimum-fuel aeroplane that then
        # loses its last engine -- and ``_declare_emergency`` itself
        # decides whether what is new outranks what was said.
        if self.state.phase in (
                AutonomousPhase.PREFLIGHT, AutonomousPhase.PUSHBACK,
                AutonomousPhase.TAXI_OUT, AutonomousPhase.SHUTDOWN,
                AutonomousPhase.COMPLETE, AutonomousPhase.ERROR):
            return
        # Parked is parked: a tank near empty on the stand is a fuelling
        # order rather than a distress, and nothing on the ground is
        # declared onto a frequency the urgency does not belong on.
        if state.on_ground:
            return

        # Electrical: the main bus collapsing in the air. A 14-volt bus
        # reads under twelve when it is gone; a 28-volt bus -- most
        # turbines' -- sits in the high teens as it dies, which a
        # twelve-volt floor alone never sees.
        volts = getattr(state, "main_bus_volts", -1.0)
        collapsed = volts is not None and volts >= 0.0 and (
            volts < 12.0 or 15.0 < volts < 20.0)
        self._electrical_since = \
            (getattr(self, "_electrical_since", 0.0) or now) \
            if collapsed else 0.0
        if collapsed and not state.on_ground \
                and now - self._electrical_since \
                >= self.ELECTRICAL_PERSIST_S:
            self._declare_emergency("electrical", state, mayday=True)
            return

        # Engines: combustion per engine where the model reports it.
        # "All out" is against everything the sim knows about -- the
        # reported count where it answers, the reported combustions
        # otherwise -- so a twin whose count the model never publishes
        # does not read one failure as two. And a frame of noise is not
        # a failure either: the flags have to stay failed a few seconds
        # before the frequency hears about them.
        if not state.on_ground:
            engines_out, engines_seen = self._engines_out(state)
            self._engines_since = \
                (getattr(self, "_engines_since", 0.0) or now) \
                if engines_out else 0.0
            count = int(getattr(state, "engine_count", 0) or 0)
            if engines_out \
                    and now - self._engines_since \
                    < self.ENGINES_PERSIST_S:
                return
            if engines_out and engines_out >= max(count, engines_seen):
                self._declare_emergency("engine", state, mayday=True)
                return
            if engines_out > 0:
                self._declare_emergency("engine-out", state, mayday=False)
                return

        # Fuel: nothing left, or so little the destination cannot be
        # reached. A sim that never reports fuel reads zero forever, so
        # the check only fires once a nonzero figure has been seen.
        fuel_lb = getattr(state, "fuel_total_lb", 0.0) or 0.0
        fuel_gal = getattr(state, "fuel_total_gal", 0.0) or 0.0
        if fuel_gal <= 0.0 and fuel_lb > 0.0:
            # A model that reports only pounds still gets the gallons
            # every figure below is priced in.
            fuel_gal = fuel_lb / self._lb_per_gal(state)
        capacity = float(getattr(state, "fuel_capacity_gal", 0.0) or 0.0)
        self._fuel_capacity_seen = max(self._fuel_capacity_seen, fuel_gal,
                                       capacity)
        if fuel_lb > 0.0 or fuel_gal > 0.0:
            self._fuel_known = True
        if not self._fuel_known:
            return
        if fuel_lb <= 0.0 and fuel_gal <= 0.0:
            self._declare_emergency("fuel", state, mayday=True)
            return
        # Tanks at or near full cannot be a fuel emergency -- nothing
        # was left behind, so a projection past them is the leg's
        # planning figure, not a distress.
        if capacity > 0.0 and fuel_gal >= capacity * 0.9:
            return
        # Nor is the roll or the climb the place to project it: takeoff
        # and climb burn are not what the rest of the leg burns, and a
        # projection priced at them cries emergency over healthy tanks.
        if self.state.phase in (AutonomousPhase.TAKEOFF,
                                AutonomousPhase.CLIMB):
            return
        needed = self._fuel_needed_gal(state)
        if needed is None or fuel_gal <= 0.0:
            self._fuel_leg_since = self._fuel_min_since = 0.0
            if 0.0 < fuel_gal < 8.0:
                self._declare_emergency("fuel", state, mayday=True)
            return
        leg_short = fuel_gal < needed["leg_gal"]
        total_short = fuel_gal < needed["total_gal"]
        self._fuel_leg_since = \
            (getattr(self, "_fuel_leg_since", 0.0) or now) \
            if leg_short else 0.0
        self._fuel_min_since = \
            (getattr(self, "_fuel_min_since", 0.0) or now) \
            if total_short else 0.0
        if leg_short \
                and now - self._fuel_leg_since >= self.LEG_FUEL_PERSIST_S:
            self._declare_emergency("fuel", state, mayday=True)
        elif total_short \
                and now - self._fuel_min_since >= self.MIN_FUEL_PERSIST_S \
                and not self.state.minimum_fuel_said:
            self.state.minimum_fuel_said = True
            self._declare_emergency("minimum fuel", state, mayday=False)

    def _engines_out(self, state) -> tuple[int, int]:
        """How many engines are not combusting, and how many the sim
        reports at all.

        ``engines_running`` is engine one's combustion; a model that
        reports no combustion variables answers nothing, and nothing is
        invented.
        """
        out = 0
        seen = 0
        for flag in (getattr(state, "engines_running", None),
                     getattr(state, "eng2_combustion", None),
                     getattr(state, "eng3_combustion", None),
                     getattr(state, "eng4_combustion", None)):
            if flag is not None:
                seen += 1
                if not flag:
                    out += 1
        return out, seen

