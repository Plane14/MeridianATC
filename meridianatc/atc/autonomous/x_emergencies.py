from __future__ import annotations

from ._shared import (
    AutonomousPhase,
)
from ...sim.state import combustion


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
    # Height above the ground below which a declared emergency holds
    # its heading and climbs: a diversion turn made below this is the
    # stall the emergency did not cause.
    DIVERT_MIN_AGL_FT = 400.0
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
        # The healthiest figure the bus has ever shown is the healthy
        # bus: a model whose main-bus index reads something else -- an
        # aux bus, a battery -- sits at odd volts all flight, and a
        # fixed band calls that healthy bus dead. Learned before the
        # guards below so the ground phases, where the bus is well,
        # are what teach it.
        volts_now = getattr(state, "main_bus_volts", -1.0)
        # Only a plausibly-live bus teaches: a rail that never once
        # leaves zero is the channel the model does not wire, not a bus
        # at all, and a spike past forty is no bus either. The odd rails
        # between -- the aux bus a model maps here -- are welcome: their
        # collapse is still the fraction of what they always showed.
        if volts_now is not None and 8.0 <= volts_now <= 40.0:
            self._bus_volts_ref = max(
                getattr(self, "_bus_volts_ref", 0.0) or 0.0, volts_now)

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

        # A distress declared in the first seconds of flight has the
        # call and the squawk, but the turn toward the divert waits for
        # height: the aeroplane under the maneuvering floor climbs
        # first -- aviate, navigate, communicate, in that order.
        if getattr(self, "_divert_pending", False):
            agl = float(getattr(state, "altitude_agl_ft", 0.0) or 0.0)
            if agl >= self.DIVERT_MIN_AGL_FT:
                self._divert_pending = False
                self._execute_diversion(state)

        # Electrical: the main bus collapsing in the air. The floor is
        # the learned healthy figure's -- a collapse to sixty-five per
        # cent of what the bus ran at is dying on any system, while a
        # model whose index reads an odd rail stays legal at whatever
        # number it always shows. A bus never once seen alive is not a
        # collapsed bus, it is a channel the model does not publish --
        # the model whose index sits at zero volts from the ramp to the
        # flare would read a MAYDAY out of thin air, the same way the
        # fuel watch waits for a nonzero figure before it counts.
        ref = getattr(self, "_bus_volts_ref", 0.0) or 0.0
        collapsed = ref > 0.0 and volts_now is not None \
            and volts_now >= 0.0 and volts_now < 0.65 * ref
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
        invented. A reported engine count is the roster: on SimConnect
        an out-of-range index still answers, and it answers False --
        ghost failures a single would otherwise never get past V1
        with. Where the count is not reported, a slot past the first
        is an engine once the sim has shown it combusting; a flag
        that has never once been up is no evidence of a failure.
        """
        flags = combustion(state)
        live = getattr(self, "_engines_live", None)
        if live is None:
            live = self._engines_live = set()
        live.update(i for i, flag in enumerate(flags, start=1) if flag)
        count = int(getattr(state, "engine_count", 0) or 0)
        if count > 0:
            live.intersection_update(range(1, count + 1))
        out = 0
        seen = 0
        for i, flag in enumerate(flags, start=1):
            if flag is None or (count > 0 and i > count) \
                    or (count <= 0 and i > 1 and i not in live):
                continue
            seen += 1
            if not flag:
                out += 1
        return out, seen

    def _any_engine_running(self, state) -> bool:
        """At least one reported engine is combusting -- the 'still
        turning' gate a shutdown, a refuel or a push must not ignore on
        a twin whose number two is the one lit."""
        out, seen = self._engines_out(state)
        return seen > out

    def _all_engines_out(self, state) -> bool:
        """Nothing reported is combusting -- also true where the model
        publishes no combustion flags at all, matching the way a lone
        ``engines_running`` read answered before."""
        out, seen = self._engines_out(state)
        return out >= seen

