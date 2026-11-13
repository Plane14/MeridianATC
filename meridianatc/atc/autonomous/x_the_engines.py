from __future__ import annotations

from ..powerplant import limits_for


class _The_engines:
    """The engine watch: the levers stay inside the type's red lines.

    The published figures live in ``atc.powerplant`` -- the gas producer,
    the turbine temperature, the crankshaft RPM -- and the comparisons
    run on the model's own instruments: a model that does not publish
    ``TURB ENG N1`` or ``TURB ENG ITT`` simply has no limit checked, the
    same way the envelope watch stands down on a missing G read.

    Two jobs: ``_watch_engines`` says a limit once when the instruments
    cross it (whether or not the FO holds the levers), and
    ``_engine_limit_cap`` gives the throttle code a ceiling it can pull
    back to -- the reflex a pilot's hand has the moment the needle
    touches the red line.
    """

    # How far over a limit the reading has to be before it counts: a
    # needle kissing the line is normal, dwelling past it is not.
    _MARGIN_NG = 0.5        # per cent Ng
    _MARGIN_ITT = 8.0       # degrees C
    _MARGIN_RPM = 30.0      # crankshaft RPM
    _MARGIN_TQ = 1.0        # per cent torque

    def _engine_limits(self):
        """The type's published figures, resolved once per session."""
        key = (getattr(getattr(self.session, "aircraft", None),
                       "type_code", "") or "",
               getattr(self, "_aircraft_class", "") or "")
        if getattr(self, "_eng_limit_key", None) != key:
            self._eng_limit_key = key
            self._eng_limits = limits_for(*key)
        return self._eng_limits

    def _engines_over_limit(self, state, *, takeoff: bool = False) -> list[str]:
        """Which published ceilings the model's own gauges are past.

        ``takeoff`` selects the takeoff rating as the temperature
        ceiling -- the figure that is legal for the roll's five minutes
        rather than the continuous one. Everything returns names, so the
        warning and the lever cap tell the same story.
        """
        lim = self._engine_limits()
        over = []
        ng = float(getattr(state, "eng_n1_pct", 0.0) or 0.0)
        if lim.ng_max_pct and ng > lim.ng_max_pct + self._MARGIN_NG:
            over.append("ng")
        itt = float(getattr(state, "eng_itt_c", 0.0) or 0.0)
        ceiling = (lim.itt_takeoff_c if takeoff and lim.itt_takeoff_c
                   else lim.itt_max_c or lim.itt_takeoff_c)
        if ceiling and itt > ceiling + self._MARGIN_ITT:
            over.append("itt")
        rpm = float(getattr(state, "eng_rpm", 0.0) or 0.0)
        if lim.rpm_max and rpm > lim.rpm_max + self._MARGIN_RPM:
            over.append("rpm")
        tq = float(getattr(state, "eng_torque_pct", 0.0) or 0.0)
        tq_ceiling = (lim.tq_takeoff_pct if takeoff and lim.tq_takeoff_pct
                      else lim.tq_cont_pct or lim.tq_takeoff_pct)
        if tq_ceiling and tq > tq_ceiling + self._MARGIN_TQ:
            over.append("tq")
        return over

    def _engine_limit_cap(self, state, *, takeoff: bool = False) -> float:
        """The highest lever the engines may take right now, 0..1.

        1.0 where every gauge is inside its limit or no gauge is
        published; otherwise a pull toward ~85 per cent of the travel --
        the reflex, not the shutdown: an engine over its ceiling gets
        power off it, never a kill.
        """
        over = self._engines_over_limit(state, takeoff=takeoff)
        if not over:
            return 1.0
        if not getattr(self.state, "eng_warned", False):
            self.state.eng_warned = True
            self._emit("info",
                       "Engine limit reached (" + ", ".join(over) +
                       "): easing the power off",
                       "autonomous.engines.limit")
        return 0.85

    def _watch_engines(self, state, now: float) -> None:
        """The monitor pass: limits are said once, whoever holds the levers.

        Where the FO's own throttle servo or the takeoff sequence holds
        the levers, ``_engine_limit_cap`` already does the retarding;
        this pass exists for the aircraft whose autothrottle or AP owns
        them -- a needle past the red line is still called, once, so the
        pilot hears it.
        """
        if not getattr(state, "engines_running", False):
            self.state.eng_warned = False
            return
        takeoff = self.state.sub in ("roll", "rotate")
        heli = getattr(self, "_heli", None)
        if heli is not None:
            # The rotorcraft's takeoff rating lives in its own stage
            # machine -- the hover and the departure are the legs the
            # five-minute figure was written for, not the fixed-wing
            # sub-phases, which a helicopter never visits.
            takeoff = getattr(getattr(heli, "state", None), "stage", "") \
                in ("hover", "departure")
        over = self._engines_over_limit(state, takeoff=takeoff)
        if over and not self.state.eng_warned:
            self.state.eng_warned = True
            self._emit("info",
                       "Engine over limit (" + ", ".join(over) + ")",
                       "autonomous.engines.overlimit")
        elif not over:
            self.state.eng_warned = False

    # -- carburetted induction ice --------------------------------------

    def _carb_heat_wanted(self, state) -> bool:
        """Whether a carburetted engine wants its heat on.

        The book's envelope, per the Robinson and Cessna POHs: ice can
        form with the OAT anywhere between about -4C and 30C where the
        air is moist -- approximated by humidity, since the sim does not
        publish a dew point -- and at any low power setting regardless,
        because the gage cannot be trusted under ~18 in MAP. Low power
        covers the ground idle as well as the descent, which is how the
        run-up gets its pre-heat while the takeoff roll gets the heat
        OFF the way the POH wants it.
        """
        if not getattr(state, "engines_running", False):
            return False
        oat_raw = getattr(state, "ambient_temperature_c", None)
        rh_raw = getattr(state, "ambient_humidity_pct", None)
        # Zero is a reading in either: ``or``-defaults would warm a
        # freezing day and dry out a dry one.
        oat = 15.0 if oat_raw is None else float(oat_raw)
        rh = 60.0 if rh_raw is None else float(rh_raw)
        moist = (bool(getattr(state, "ambient_in_cloud", False))
                 or int(getattr(state, "ambient_precip_state", 0) or 0) > 0
                 or rh >= 60.0)
        in_window = -4.0 <= oat <= 30.0 and moist
        if getattr(self, "_is_helicopter", False):
            # A helicopter's gageless case is the low collective: under
            # ~18 in MAP the CAT gage lies, so the POHs want full heat
            # on any big reduction. The collective, per cent, stands in;
            # an unpublished one reads zero -- the idle the heat is for.
            low_power = float(getattr(state, "collective_position", 0.0)
                              or 0.0) < 35.0
        else:
            low_power = self._lever_position(state) < 0.35
        return in_window or low_power
