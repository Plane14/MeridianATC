from __future__ import annotations

import time

from ...sim.state import engine_vars
from ..notices import (
    LEFT_TANKS,
    LOW_TANK_GAL,
    LOW_TANK_SHARE,
    OTHER_TANK_GAL,
    RIGHT_TANKS,
)
from ..powerplant import cooling_for, limits_for
from ._shared import AutonomousPhase


class _The_engines:
    """The engine watch: the levers stay inside the type's red lines.

    The published figures live in ``atc.powerplant`` -- the gas producer,
    the turbine temperature, the crankshaft RPM -- and the comparisons
    run on the model's own instruments: a model that does not publish
    ``TURB ENG N1`` or ``TURB ENG ITT`` simply has no limit checked, the
    same way the envelope watch stands down on a missing G read.

    Three jobs: ``_watch_engines`` says a limit once when the instruments
    cross it (whether or not the FO holds the levers), and
    ``_engine_limit_cap`` gives the throttle code a ceiling it can pull
    back to -- the reflex a pilot's hand has the moment the needle
    touches the red line. ``_manage_cooling`` works the type's cowl
    flaps the way the POH does, for the types that have them.
    """

    # How far over a limit the reading has to be before it counts: a
    # needle kissing the line is normal, dwelling past it is not.
    _MARGIN_NG = 0.5        # per cent Ng
    _MARGIN_ITT = 8.0       # degrees C
    _MARGIN_RPM = 30.0      # crankshaft RPM
    _MARGIN_TQ = 1.0        # per cent torque
    _MARGIN_CHT = 5.0       # degrees C of CHT

    # The slack an N-m torque comparison gets, where the per-cent margin
    # above is of the scale: a King Air's 1,315 ft-lb ceiling in N-m is
    # 1,783, and 40 is a couple of per cent of it.
    _MARGIN_TQ_NM = 40.0

    # How long a gauge must read past its ceiling before the watch and
    # the lever cap believe it: a feed that jitters across the line
    # between reads is noise, not an over-temp -- a live probe watched a
    # King Air's ENGN_EGT_c swing a hundred and fifty degrees between
    # quarter-second samples, which flapped the cap on and off every
    # tick and surged the power in the climb.
    _OVER_PERSIST_S = 2.0

    # The lever ceiling's positions while a gauge sits past its red
    # line: first the reflex pull, then the walk down a hand does while
    # the needle is still out -- a fixed cap only stops things getting
    # worse, and a model whose engines still run hot at 85 per cent
    # needs the lever brought lower until the reading is inside. The
    # floor keeps power on the aeroplane: off, never a kill.
    _LIMIT_CAP_FIRST = 0.85
    _LIMIT_CAP_STEP = 0.07
    _LIMIT_CAP_FLOOR = 0.30
    _LIMIT_STEP_S = 2.0
    # Clean seconds the gauges must hold before the ceiling is let back
    # out: a dip between reads is not the needle inside.
    _LIMIT_CLEAR_S = 10.0

    # The CHT the flaps open at: this far under the red line. Closing
    # again is allowed a further 20C of hysteresis so a wandering needle
    # does not pump the levers.
    _CHT_OPEN_MARGIN = 15.0
    _CHT_HYSTERESIS = 20.0

    def _engine_limits(self):
        """The type's published figures, resolved once per session."""
        key = (getattr(getattr(self.session, "aircraft", None),
                       "type_code", "") or "",
               getattr(self, "_aircraft_class", "") or "")
        if getattr(self, "_eng_limit_key", None) != key:
            self._eng_limit_key = key
            self._eng_limits = limits_for(*key)
        return self._eng_limits

    def _cooling(self):
        """The type's cooling kit, resolved once per session like the limits."""
        key = (getattr(getattr(self.session, "aircraft", None),
                       "type_code", "") or "",
               getattr(self, "_aircraft_class", "") or "")
        if getattr(self, "_cooling_key", None) != key:
            self._cooling_key = key
            self._cooling_cached = cooling_for(*key)
        return self._cooling_cached

    def _engines_over_limit(self, state, *, takeoff: bool = False) -> list[str]:
        """Which published ceilings the model's own gauges are past.

        ``takeoff`` selects the takeoff rating as the temperature
        ceiling -- the figure that is legal for the roll's five minutes
        rather than the continuous one. Everything returns names, so the
        warning and the lever cap tell the same story.
        """
        lim = self._engine_limits()
        # The ceilings once, the same for every engine: the model's own
        # red lines where it publishes them -- an X-Plane author who
        # filled in red_hi_ITT knew the figure better than the class
        # table does -- the type's or the class's where it does not.
        itt_ceiling = (float(getattr(state, "eng_itt_takeoff_c", 0.0)
                             or 0.0)
                       if takeoff else
                       float(getattr(state, "eng_itt_limit_c", 0.0) or 0.0))
        if not itt_ceiling:
            itt_ceiling = (lim.itt_takeoff_c
                           if takeoff and lim.itt_takeoff_c
                           else lim.itt_max_c or lim.itt_takeoff_c)
        tq_ceiling = (lim.tq_takeoff_pct if takeoff and lim.tq_takeoff_pct
                      else lim.tq_cont_pct or lim.tq_takeoff_pct)
        # No per-cent reference gets an absolute ceiling: the model's
        # own published one (green_hi_TRQ / acf_max_TRQ), else the
        # type's book figure -- the King Air whose torque gauge paints
        # a 1,315 ft-lb red line onto a dataref that reads 1.0 gets its
        # ceiling from the POH, not the paint.
        nm_ceiling = 0.0
        if not tq_ceiling:
            nm_ceiling = (float(getattr(
                state,
                "eng_tq_takeoff_nm" if takeoff else "eng_tq_limit_nm",
                0.0) or 0.0)
                or (lim.tq_takeoff_nm if takeoff and lim.tq_takeoff_nm
                    else lim.tq_max_nm or lim.tq_takeoff_nm))
        over = []
        # Every engine the sim reports: engine one's figures are the
        # scalars, the rest carry the eng2_.../eng16_ names, and a slot
        # that never answered is no engine. The tag
        # names the engine for the call, and its own persistence clock
        # -- a twin's second PT6 running hot is "itt #2", not "itt".
        for name in engine_vars("eng_n1_pct"):
            ng = getattr(state, name, None)
            tag = "" if name == "eng_n1_pct" else " #" + name[3:].split("_", 1)[0]
            if ng is not None and lim.ng_max_pct \
                    and float(ng) > lim.ng_max_pct + self._MARGIN_NG:
                over.append(f"ng{tag}")
        for name in engine_vars("eng_itt_c"):
            itt = getattr(state, name, None)
            tag = "" if name == "eng_itt_c" else " #" + name[3:].split("_", 1)[0]
            if itt is not None and itt_ceiling \
                    and float(itt) > itt_ceiling + self._MARGIN_ITT:
                over.append(f"itt{tag}")
        for name in engine_vars("eng_rpm"):
            rpm = getattr(state, name, None)
            tag = "" if name == "eng_rpm" else " #" + name[3:].split("_", 1)[0]
            if rpm is not None and lim.rpm_max \
                    and float(rpm) > lim.rpm_max + self._MARGIN_RPM:
                over.append(f"rpm{tag}")
        for name in engine_vars("eng_torque_pct"):
            tq = getattr(state, name, None)
            tag = "" if name == "eng_torque_pct" else " #" + name[3:].split("_", 1)[0]
            if tq is not None and tq_ceiling \
                    and float(tq) > tq_ceiling + self._MARGIN_TQ:
                over.append(f"tq{tag}")
        if not tq_ceiling:
            for name in engine_vars("eng_torque_nm"):
                trq_nm = getattr(state, name, None)
                tag = "" if name == "eng_torque_nm" else " #" + name[3:].split("_", 1)[0]
                if trq_nm is not None and nm_ceiling \
                        and float(trq_nm) > nm_ceiling + self._MARGIN_TQ_NM:
                    over.append(f"tq{tag}")
        # Cylinder-head temperature stays a single line: the scalar is
        # already the hottest head the engines report.
        cht = float(getattr(state, "eng_cht_c", 0.0) or 0.0)
        ceiling = self._cht_limit(state)
        if ceiling and cht > ceiling + self._MARGIN_CHT:
            over.append("cht")
        return over

    def _over_limits(self, state, now: float | None = None, *,
                     takeoff: bool = False) -> list[str]:
        """The gauges that have held past their red lines.

        The instantaneous set aged long enough to be believed: a gauge
        that only touches the line between reads never makes the list,
        so a noisy feed cannot flap the lever cap -- and a gauge that
        drops back under restarts its own clock, so a hovering needle
        does not accumulate its way to an excursion it never flew.
        """
        over = self._engines_over_limit(state, takeoff=takeoff)
        if now is None:
            now = time.monotonic()
        since = getattr(self, "_over_since", None)
        if since is None:
            since = self._over_since = {}
        held = []
        for name in over:
            if now - since.setdefault(name, now) >= self._OVER_PERSIST_S:
                held.append(name)
        for name in tuple(since):
            if name not in over:
                del since[name]
        return held

    def _cht_limit(self, state) -> float:
        """The CHT ceiling this engine's heads are worked against, C."""
        published = float(getattr(state, "eng_cht_limit_c", 0.0) or 0.0)
        return published or self._cooling().cht_max_c

    def _engine_limit_cap(self, state, *, takeoff: bool = False) -> float:
        """The highest lever the engines may take right now, 0..1.

        1.0 where every gauge is inside its limit or no gauge is
        published. Past one, the ceiling starts at the reflex pull and
        keeps walking down while the needle stays out -- the hand's
        travel to wherever the gauge comes back inside, floored where
        power stays on the aeroplane.
        """
        over = self._over_limits(state, takeoff=takeoff)
        now = time.monotonic()
        if not over:
            if now - getattr(self, "_limit_clean", now) \
                    >= self._LIMIT_CLEAR_S:
                self._limit_cap = 1.0
            return getattr(self, "_limit_cap", 1.0)
        self._limit_clean = now
        if not getattr(self.state, "eng_warned", False):
            self.state.eng_warned = True
            self._emit("info",
                       "Engine limit reached (" + ", ".join(over) +
                       "): easing the power off",
                       "autonomous.engines.limit")
        cap = getattr(self, "_limit_cap", 1.0)
        if cap >= 1.0:
            cap = self._LIMIT_CAP_FIRST
            self._limit_step = now
        elif now - getattr(self, "_limit_step", now) >= self._LIMIT_STEP_S:
            self._limit_step = now
            cap = max(self._LIMIT_CAP_FLOOR,
                      cap - self._LIMIT_CAP_STEP)
        self._limit_cap = cap
        return cap

    def _watch_engines(self, state, now: float) -> None:
        """The monitor pass: limits are said once, whoever holds the levers.

        Where the FO's own throttle servo or the takeoff sequence holds
        the levers, ``_engine_limit_cap`` already does the retarding;
        this pass exists for the aircraft whose autothrottle or AP owns
        them -- a needle past the red line is still called, once, so the
        pilot hears it.
        """
        if self._all_engines_out(state):
            self.state.eng_warned = False
            self._over_since = {}
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
        over = self._over_limits(state, now, takeoff=takeoff)
        if over and not self.state.eng_warned:
            self.state.eng_warned = True
            self._emit("info",
                       "Engine over limit (" + ", ".join(over) + ")",
                       "autonomous.engines.overlimit")
        elif not over:
            self.state.eng_warned = False
        self._lean_mixture(state)

    def _lean_mixture(self, state) -> None:
        """The red knob's cruise position on a piston, once per climb.

        Full rich burns fuel the thin air cannot use: above three
        thousand feet AGL the POH leans for economy, and the knob goes
        rich again for the takeoff, the approach and the go-around by
        ``_prop_levers``' own calls -- this only ever pulls it out, and
        only where the air is thin enough to want it lean.
        """
        from .. import aircraft_class as ac
        if getattr(self, "_aircraft_class", "") not in (
                ac.PISTON_SINGLE, ac.PISTON_TWIN):
            return
        if self.state.phase not in (AutonomousPhase.CLIMB,
                                    AutonomousPhase.CRUISE):
            if getattr(self, "_mixture_leaned", False) \
                    and self._any_engine_running(state):
                # Rich again for the descent and the approach -- a knob
                # left lean to the flare is the go-around that answers
                # with a cough instead of power.
                self._mixture_leaned = False
                self.director.mixture(1.0)
                self._emit("info", "Mixture rich",
                           "autonomous.engines.rich")
            else:
                self._mixture_leaned = False
            return
        if getattr(self, "_mixture_leaned", False):
            return
        agl = float(getattr(state, "altitude_agl_ft", 0.0) or 0.0)
        if agl < 3000.0:
            return
        self._mixture_leaned = True
        self.director.mixture(0.75)
        self._emit("info", "Mixture leaned for altitude",
                   "autonomous.engines.lean")

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
        if self._all_engines_out(state):
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

    # -- engine cooling: cowl flaps --------------------------------------

    def _cowl_flap_wanted(self, state) -> float:
        """Where the cowl flaps belong right now, 0 closed .. 1 open.

        The schedule every cowl-flap POH publishes: open where power is
        high and speed is low -- the ground, the takeoff, the climb and
        the go-around -- closed in cruise, descent and approach. On top
        of the schedule sits the gauge: a CHT running toward its red
        line opens them wherever the flight is, and the hysteresis keeps
        a needle hovering near the mark from pumping the levers.
        """
        open_now = bool(state.on_ground) or self.state.sub == "go_around" \
            or self.state.phase in (AutonomousPhase.TAKEOFF,
                                    AutonomousPhase.CLIMB)
        cht = float(getattr(state, "eng_cht_c", 0.0) or 0.0)
        limit = self._cht_limit(state)
        hot = cht > 0.0 and limit > 0.0 \
            and cht > limit - self._CHT_OPEN_MARGIN
        if hot:
            if not self.state.cht_warned:
                self.state.cht_warned = True
                self._emit("info",
                           f"Cylinder head temperature {cht:.0f} C "
                           "nearing limit: cowl flaps open",
                           "autonomous.engines.cht")
            # Sticky: once the gauge opened them they stay open until
            # the heads are comfortably cool again -- the hysteresis is
            # on the flap position, not only the call.
            self.state.cht_flaps = True
        elif cht <= 0.0 or limit <= 0.0 \
                or cht < limit - self._CHT_OPEN_MARGIN - self._CHT_HYSTERESIS:
            self.state.cht_flaps = False
            self.state.cht_warned = False
        if self.state.cht_flaps:
            return 1.0
        return 1.0 if open_now else 0.0

    def _manage_cooling(self, state) -> None:
        """Work the cowl flaps on the types that carry them.

        ``cooling_for`` is the gate: a type whose entry does not say it
        has cowl flaps -- a 172's fixed cowl, a turbine's self-managed
        cooling -- never gets a lever written. Where the sim cannot
        answer whether the levers exist at all (X-Plane's handle ratio
        reads on every model), the type table is what keeps a Cessna
        172's nonexistent flaps untouched. ``cowl_set`` tracks only what
        the FO itself wrote, so a lever the pilot moved stays his.
        """
        if self._all_engines_out(state):
            return
        if not self._cooling().cowl_flaps:
            return
        want = self._cowl_flap_wanted(state)
        if want == self.state.cowl_set:
            return
        if self.director.cowl_flaps(want):
            self.state.cowl_set = want

    # -- fuel: a dry tank is a selector position, not an emergency -------

    # The SimConnect selector positions that feed one side's tank,
    # mirrored: a dry left answers with the matching right -- the move
    # stays inside the aeroplane's own grammar. Aux, tip, centre and
    # external positions are not here: their level is not what the
    # left/right gauges read, so they are only moved on the sim's own
    # tank list, and then to ALL rather than to a position the model
    # may not have. Positions absent from the maps at all -- off, the
    # crossfeeds, isolate -- are either already feeding every tank
    # they can or are a crew's deliberate plumbing, and the FO does
    # not touch them. The second selector a twin carries for its
    # other engine is not published through the abstraction either,
    # so selector 1 -- the master on every single-selector aeroplane
    # -- is what this reads.
    _TANK_MIRROR = {2: 3, 3: 2, 19: 20, 20: 19}
    # Positions whose level only the tank list can say: auxes and tips
    # read the mains' gauges, not their own tank's, and centre and
    # external tanks have no gauge of their own here at all.
    _LIST_POSITIONS = {4, 5, 6, 7, 8, 9, 10, 11, 12, 17}
    # X-Plane's ``m_fuel[]`` order is the Plane-Maker one (left, right,
    # centre, auxes, tips, externals); MSFS's legacy tank names carry
    # the name in the variable itself.
    _XP_TANK_POSITION = {0: 2, 1: 3, 2: 6, 3: 4, 4: 5,
                       5: 12, 6: 11, 7: 9, 8: 10}
    _TANK_NAME_POSITION = {
        "CENTER": 6, "CENTER2": 7, "CENTER3": 8,
        "LEFT MAIN": 19, "RIGHT MAIN": 20,
        "LEFT AUX": 4, "RIGHT AUX": 5,
        "LEFT TIP": 12, "RIGHT TIP": 11,
        "EXTERNAL1": 9, "EXTERNAL2": 10,
        "LEFT": 2, "RIGHT": 3}
    _FEED_ALL = {1, 16}
    _TANK_NAMES = {1: "all", 2: "left", 3: "right", 4: "left aux",
                   5: "right aux", 6: "center", 7: "center2",
                   8: "center3", 9: "external1", 10: "external2",
                   11: "right tip", 12: "left tip", 16: "both",
                   17: "external", 19: "left main", 20: "right main"}

    # How long after a switch the lever is left alone: the gauge needs
    # the seconds to answer, and a feed that jitters the reading across
    # the low line is not a reason to chase the selector back and forth.
    _FUEL_SWITCH_S = 20.0

    def _tank_levels(self, state) -> dict[int, float] | None:
        """The fuel aboard per selector position, where the sim can say.

        ``fuel_tanks()`` answers in gallons on both sources; the names
        are matched back to selector positions -- MSFS's by the tank's
        name in the variable, X-Plane's ``m_fuel`` by the Plane-Maker
        order. A source that cannot say (indexed FUELSYSTEM tanks, no
        source at all) answers ``None`` rather than a guess.
        """
        sim = getattr(self, "sim", None)
        lister = getattr(sim, "fuel_tanks", None)
        if lister is None:
            return None
        try:
            tanks = lister() or []
        except Exception:
            return None
        levels: dict[int, float] = {}
        for name, _capacity, qty in tanks:
            key = str(name).upper()
            position = None
            if "M_FUEL[" in key:
                try:
                    index = int(key.split("[")[1].split("]")[0])
                except (IndexError, ValueError):
                    continue
                position = self._XP_TANK_POSITION.get(index)
            else:
                # Longest tokens first, so "LEFT MAIN" is not read as
                # "LEFT" and "CENTER2" is not read as "CENTER".
                for token, pos in sorted(
                        self._TANK_NAME_POSITION.items(),
                        key=lambda kv: -len(kv[0])):
                    if token in key:
                        position = pos
                        break
            if position is not None:
                levels[position] = max(float(qty or 0.0),
                                       levels.get(position, 0.0))
        return levels

    def _manage_fuel_tanks(self, state) -> None:
        """Move the selector off a tank that has run dry.

        A wing that reads under a gallon while the other holds two is
        the "still on the left tank" call the notices make -- answered
        here instead of only said, because on the types whose selector
        works this is a lever, not a warning. The mirror keeps the move
        inside the aeroplane's own grammar: a left position is answered
        with the matching right, a centre or external position that the
        tank list shows dry is answered with ALL -- the one position
        that feeds from whatever is left without the FO having to guess
        which of a jet's twenty-odd selector stops the model honours.

        The positions the move never makes: OFF, because starvation is
        the failure being prevented; the crossfeeds and isolate, which
        are crew plumbing for imbalance and failure cases this code is
        not qualified to diagnose; and ALL/BOTH themselves, which are
        already feeding from everything aboard.
        """
        if self._all_engines_out(state):
            return
        missing = set(getattr(state, "unavailable", ()) or ())
        if "fuel_selector" in missing:
            return
        selector = int(getattr(state, "fuel_selector", -1) or -1)
        if selector in self._FEED_ALL or selector < 0:
            return
        if selector not in self._TANK_MIRROR \
                and selector not in self._LIST_POSITIONS:
            return
        if time.monotonic() - getattr(self, "_fuel_switch_at", 0.0) \
                < self._FUEL_SWITCH_S:
            return
        want, why = self._fuel_retarget(state, selector, missing)
        if want is None:
            return
        # The clock is latched on the attempt, not the answer: a model
        # that refuses the write is asked again on the same cadence the
        # gauge answers on, not every tick.
        self._fuel_switch_at = time.monotonic()
        if self.director.write("FUEL TANK SELECTOR:1", float(want)):
            self._emit("info",
                       f"Fuel: {why} -- selector to "
                       f"{self._TANK_NAMES.get(want, str(want))}",
                       "autonomous.fuel.tank_switch")

    def _fuel_retarget(self, state, selector: int,
                       missing: set) -> tuple[int | None, str]:
        """(position, why) the dry selection is answered with, else none."""
        # The side gauges judge a side position directly: under a
        # gallon, or under a quarter of the other side with the other
        # side holding two, is the "still on this tank" call the
        # notices make -- acted on here by the mirror.
        side_gauges = "fuel_left_gal" not in missing \
            and "fuel_right_gal" not in missing
        if side_gauges and (selector in LEFT_TANKS
                            or selector in RIGHT_TANKS):
            left = float(getattr(state, "fuel_left_gal", 0.0) or 0.0)
            right = float(getattr(state, "fuel_right_gal", 0.0) or 0.0)
            this, other = ((left, right) if selector in LEFT_TANKS
                           else (right, left))
            if other <= OTHER_TANK_GAL or not (
                    this < LOW_TANK_GAL or this < other * LOW_TANK_SHARE):
                return None, ""
            side = "left" if selector in LEFT_TANKS else "right"
            return (self._TANK_MIRROR[selector],
                    f"{side} tank down to {this:.1f} gal "
                    f"with {other:.1f} on the other side")
        # Everywhere else -- auxes, tips, centre, externals, and the
        # side positions on a sim that does not publish the gauges --
        # only the tank list can say the selected one is the dry one.
        # A guess here is a selector moved onto a tank that still had
        # fuel, so the list missing the position is a no, not a maybe.
        levels = self._tank_levels(state)
        if not levels or selector not in levels:
            return None, ""
        selected = levels[selector]
        if selected >= LOW_TANK_GAL:
            return None, ""
        fullest = max(levels.values(), default=0.0)
        if fullest <= OTHER_TANK_GAL:
            return None, ""
        name = self._TANK_NAMES.get(selector, "selected")
        return 1, (f"{name} tank down to {selected:.1f} gal "
                   f"with {fullest:.1f} aboard")

    # -- the condition lever: the third lever a turboprop carries --------

    # Where each stop sits on the published scale (state.condition_pct):
    # cut-off below the low-idle band, low idle around its detent, high
    # idle at the top of the travel.
    _COND_LOW = 35.0    # under this the fuel cock is shut
    _COND_HIGH = 90.0   # under this the governor still limits the prop
    _COND_WRITE_S = 10.0

    def _manage_condition(self, state) -> None:
        """The condition levers at the detent the phase wants.

        A turboprop's third lever is fuel cock and idle select in one
        handle: CUT-OFF closes the fuel to a running engine, LOW IDLE is
        the ground's detent -- the start, the taxi, the wait at the
        hold -- and HIGH IDLE is the flight's, because the propeller
        governor cannot give takeoff or go-around power from the lower
        stop. The checklist's own sequence says as much: low idle at
        twelve per cent, high idle before the brakes come off.

        Only the floor is enforced: a lever below the detent the phase
        asks is lifted to it, while a pilot's own choice above it -- high
        idle kept on the taxi out, the POH's ground-fine answer -- is
        left alone. The lever only exists on turboprops, which the
        class is the gate for: a piston's red knob is a mixture and a
        jet has neither handle.
        """
        from .. import aircraft_class as ac
        if getattr(self, "_aircraft_class", "") not in (
                ac.TURBOPROP_SINGLE, ac.TURBOPROP_TWIN):
            return
        if self._all_engines_out(state):
            return
        if "condition_pct" in set(getattr(state, "unavailable", ()) or ()):
            return
        if time.monotonic() - getattr(self, "_condition_at", 0.0) \
                < self._COND_WRITE_S:
            return
        # The stops the phase needs under the lever: the flight's high
        # idle from the line-up until the wheels are back on the ground,
        # the ground's low idle before it and after.
        flying = not getattr(state, "on_ground", True) \
            or self.state.phase in (AutonomousPhase.LINE_UP,
                                    AutonomousPhase.TAKEOFF)
        # Every lever the sim reports has to reach the detent: engine
        # one's scalar, and the rest under the eng2_.../eng16_ names --
        # the lowest of them is the lever the phase still owes, and the
        # write below moves them all.
        levers = [v for v in (getattr(state, name, None)
                              for name in engine_vars("condition_pct"))
                  if v is not None]
        lever = min(levers) if levers \
            else float(getattr(state, "condition_pct", 0.0) or 0.0)
        want, floor = (2.0, self._COND_HIGH) if flying \
            else (1.0, self._COND_LOW)
        if lever >= floor:
            return
        # The clock is latched on the attempt like the selector's: a
        # model that refuses is re-asked on the gauge's own cadence.
        self._condition_at = time.monotonic()
        if self.director.condition(want):
            self._emit("info",
                       "Condition levers to "
                       + ("high idle" if want > 1.5 else "low idle")
                       + ("" if lever >= self._COND_LOW
                          else " -- the fuel cock was shut"),
                       "autonomous.condition.lever")
