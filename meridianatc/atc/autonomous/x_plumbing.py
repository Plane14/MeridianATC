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

    def _type_code_now(self) -> str:
        """The ICAO type the session currently believes the model is.

        Read live: the identification settles a read or two after the
        first state arrives -- a BE58 answer corrected to the C172 the
        ramp actually holds is a normal session start. Caches stamped
        with the early guess are dropped when the code changes, so the
        figures a takeoff is flown with belong to the aeroplane's own
        name, not the first guess's.
        """
        return getattr(getattr(getattr(self, "session", None),
                               "aircraft", None), "type_code", "") or ""

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
        code = self._type_code_now()
        # An unstamped figure is trusted -- it was put there on purpose.
        # Only a figure this property computed for another code is a
        # wrong-aeroplane cache, dropped when the code moves on.
        if getattr(self, "_perf", None) is None \
                or (getattr(self, "_perf_code", "")
                    and getattr(self, "_perf_code", "") != code):
            session = getattr(self, "session", None)
            light = getattr(getattr(session, "profile", None),
                            "kind", "") in ("piston_single", "piston_twin",
                                            "light")
            base = fly.performance_for(code, light=light)
            title = getattr(getattr(session, "aircraft", None),
                            "title", "") or ""
            # What the model says about itself, for a borrowed profile to
            # be retuned by: the profile nearest this aeroplane is flown
            # at this aeroplane's own speeds, not the profile's donor's.
            declared: dict = {}
            try:
                state = self.get_state()
            except Exception:
                state = None
            if state is not None:
                # A negative answer is the sim's "not set", not a speed:
                # it must not stand in front of the takeoff fallback.
                min_rot = float(getattr(state, "min_rotation_kt", 0.0)
                                or 0.0)
                takeoff = float(getattr(state, "design_takeoff_kt", 0.0)
                                or 0.0)
                declared = {
                    # A design takeoff figure is the climb-away speed,
                    # not the unstick: rotating at it flies the ground
                    # run on. Borrowed, it is pulled back to the rotate
                    # speed the aeroplane's own number is nearest to.
                    "rotate_kt": (min_rot if min_rot > 0.0
                                  else takeoff * 0.9),
                    # A design figure is the model's Vno, not its cruise
                    # speed: borrowed at face value a profile's owner was
                    # cruised at the top of the green arc. Pulled back to
                    # a cruise setting it is the aeroplane's own figure.
                    "cruise_kt": float(getattr(state, "estimated_cruise_kt",
                                               0.0)
                                       or 0.75
                                       * float(getattr(state,
                                                       "design_vc_kt",
                                                       0.0) or 0.0)),
                    "approach_kt": float(getattr(state, "design_vs1_kt", 0.0)
                                         or 0.0) * 1.3,
                    "descend_fpm": float(
                        getattr(state, "typical_descent_fpm", 0.0) or 0.0),
                }
                declared = {k: v for k, v in declared.items() if v > 0}
            from ..autofly import override
            self._perf = override(
                code, title, base,
                aircraft_class=getattr(self, "_aircraft_class", "") or "",
                declared=declared)
        self._perf_code = code
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
    def _adaptive(self) -> bool:
        """Whether the loops may learn and remember this model's
        peculiarities, read live like ``_use_pids``."""
        return bool(getattr(
            getattr(self, "config", None), "immersion", None)
            and getattr(self.config.immersion, "autonomous_adaptive", True))

    @property
    def gains(self):
        """The closed-loop gains for what is being flown.

        Resolved where the performance figures are: the class bundle,
        refined channel by channel where an autofly profile's handling
        block matches this add-on's title.
        """
        code = self._type_code_now()
        if getattr(self, "_gains", None) is None \
                or (getattr(self, "_gains_code", "")
                    and getattr(self, "_gains_code", "") != code):
            from ..autofly import handling
            from ..control_gains import fixed_wing_gains
            title = getattr(getattr(self.session, "aircraft", None),
                            "title", "") or ""
            self._gains = handling(
                code, title, fixed_wing_gains(self.performance),
                aircraft_class=getattr(self, "_aircraft_class", "") or "")
        self._gains_code = code
        return self._gains

    def _pid(self, channel: str):
        """The loop on one hand-flown axis, made on first use."""
        pids = getattr(self, "_pids", None)
        if pids is None:
            pids = self._pids = {}
        pid = pids.get(channel)
        if pid is None:
            adapt = getattr(self, "_adapt", None)
            if adapt is not None:
                from ..adaptation import AdaptivePid
                pid = AdaptivePid(channel, adapt,
                                  getattr(self.gains, channel))
            else:
                from ..control import Pid
                pid = Pid(getattr(self.gains, channel))
            pids[channel] = pid
        return pid

    def _compute_speeds(self, state=None) -> vspeeds.Speeds:
        """This leg's V-speeds, once: the model's own where it declares
        them, the table where it does not."""
        code = self._type_code_now()
        # As with ``performance``: an unstamped figure is trusted, only a
        # figure computed under another code is a wrong-aeroplane cache.
        if getattr(self, "_speeds", None) is not None \
                and not (getattr(self, "_speeds_code", "")
                         and getattr(self, "_speeds_code", "") != code):
            self._speeds_code = code
            return self._speeds
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
        self._speeds_code = code
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

