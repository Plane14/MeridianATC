from __future__ import annotations

from ._shared import (
    _SUB_RESETS,
    FlightDirector,
    fly,
    time,
    vspeeds,
)


class _Plumbing:
    # -- plumbing ---------------------------------------------------------

    @property
    def director(self) -> FlightDirector:
        """The hands, made on first use so a missing sim is not an error.

        The engine swaps ``self.sim`` when the simulator reconnects, so a
        director holding the old source is remade rather than kept.
        """
        if self._dir is None or self._dir.sim is not self.sim:
            self._dir = FlightDirector(self.sim, self._ap_mode)
        return self._dir

    @property
    def performance(self) -> fly.Performance:
        if getattr(self, "_perf", None) is None:
            session = getattr(self, "session", None)
            code = getattr(getattr(session, "aircraft", None),
                           "type_code", "") or ""
            light = getattr(getattr(session, "profile", None),
                            "kind", "") in ("piston_single", "piston_twin",
                                            "light")
            base = fly.performance_for(code, light=light)
            title = getattr(getattr(session, "aircraft", None),
                            "title", "") or ""
            from ..autofly import override
            self._perf = override(code, title, base)
        return self._perf

    @property
    def _use_pids(self) -> bool:
        """Whether the hand-flown axes run the closed-loop laws.

        Read live rather than remembered: a model that misbehaves under a
        loop can be handed back to the open-loop rules mid-flight, and a
        test harness that never ran ``__init__`` gets the default rather
        than an AttributeError.
        """
        return bool(getattr(
            getattr(self, "config", None), "immersion", None)
            and getattr(self.config.immersion, "autonomous_pid", True))

    @property
    def gains(self):
        """The closed-loop gains for what is being flown.

        Resolved where the performance figures are: the class bundle,
        refined channel by channel where an autofly profile's handling
        block matches this add-on's title.
        """
        if getattr(self, "_gains", None) is None:
            from ..autofly import handling
            from ..control_gains import fixed_wing_gains
            code = getattr(getattr(self.session, "aircraft", None),
                           "type_code", "") or ""
            title = getattr(getattr(self.session, "aircraft", None),
                            "title", "") or ""
            self._gains = handling(
                code, title, fixed_wing_gains(self.performance))
        return self._gains

    def _pid(self, channel: str):
        """The loop on one hand-flown axis, made on first use."""
        pids = getattr(self, "_pids", None)
        if pids is None:
            pids = self._pids = {}
        pid = pids.get(channel)
        if pid is None:
            from ..control import Pid
            pid = Pid(getattr(self.gains, channel))
            pids[channel] = pid
        return pid

    def _compute_speeds(self, state=None) -> vspeeds.Speeds:
        """This leg's V-speeds, once: the model's own where it declares
        them, the table where it does not."""
        if self._speeds is not None:
            return self._speeds
        session = self.session
        code = getattr(getattr(session, "aircraft", None), "type_code", "") or ""
        if state is None:
            state = self.get_state()
        cls = getattr(self, "_aircraft_class", "") or "airliner"
        self._speeds = vspeeds.speeds_for(
            code, cls,
            min_rotation_kt=float(getattr(state, "min_rotation_kt", 0.0) or 0.0),
            design_takeoff_kt=float(getattr(state, "design_takeoff_kt", 0.0)
                                    or 0.0))
        s = self.state
        s.v1_kt = float(self._speeds.v1 or 0.0)
        s.vr_kt = float(self._speeds.vr or 0.0)
        s.v2_kt = float(self._speeds.v2 or 0.0)
        s.vapp_kt = float(self._speeds.vapp or 0.0)
        # The model's own stall speed is a floor under the approach speed:
        # a table figure below 1.25 times Vs1 is a bug in the table, not a
        # fast approach.
        vs1 = float(getattr(state, "design_vs1_kt", 0.0) or 0.0)
        if vs1 > 0 and (not s.vapp_kt or s.vapp_kt < vs1 * 1.25):
            s.vapp_kt = vs1 * 1.25
        s.climb_kt = self.performance.climb_kt
        return self._speeds

    def _goto_sub(self, sub: str) -> None:
        """A new sub-step inside the phase."""
        self.state.sub = sub
        self.state.sub_entered_at = time.monotonic()
        # The hand-flown axes start each sub clean: a flare does not want
        # the integrator the taxi wound, and a new rotation does not want
        # the rejected one's.
        for channel in _SUB_RESETS.get(sub, ()):
            pid = getattr(self, "_pids", {}).get(channel)
            if pid is not None:
                pid.reset()

    def _sub_age(self) -> float:
        return time.monotonic() - self.state.sub_entered_at

