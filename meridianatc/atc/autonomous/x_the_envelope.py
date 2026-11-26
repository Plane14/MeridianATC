from __future__ import annotations

from ._shared import AutonomousPhase, time


class _The_envelope:
    # -- the envelope ------------------------------------------------------

    # FAA guidance encourages supplemental oxygen above 10,000 ft in an
    # unpressurised aeroplane, and EASA/ICAO rules require flight-crew
    # oxygen for the whole time there. It is a planning ceiling, not a
    # claim that a short legal excursion higher is impossible.
    NO_OXYGEN_CEILING_FT = 10000.0

    # Type inference is only a fallback where the simulator never says.
    # The notable unpressurised turboprops and pressurised pistons are
    # the exceptions to their engine families.
    _UNPRESSURIZED_TURBOPROPS = frozenset({
        "C208", "DHC6", "PC6", "BE99", "D228", "SH33", "SH36",
    })
    _PRESSURIZED_PISTONS = frozenset({
        "PA46", "P46T", "P210", "C340", "C414", "C421", "AEST",
    })

    def _pressurized_airframe(self, state) -> bool:
        """Whether the airframe can keep the cabin below the oxygen levels."""
        if getattr(state, "pressurization_capable", False):
            return True
        if float(getattr(state, "pressurization_diff_psi", 0.0) or 0.0) > 0.2:
            return True
        altitude = float(getattr(state, "pressure_altitude_ft", 0.0)
                         or getattr(state, "altitude_ft", 0.0) or 0.0)
        cabin = float(getattr(state, "cabin_altitude_ft", 0.0) or 0.0)
        if altitude > 10000.0 and cabin and cabin < altitude - 2000.0:
            return True
        code = (getattr(state, "type_code", "") or "").upper()
        raw_engine = getattr(state, "engine_type", -1)
        engine = int(raw_engine if raw_engine is not None else -1)
        if code in self._PRESSURIZED_PISTONS:
            return True
        if code in self._UNPRESSURIZED_TURBOPROPS or engine == 3:
            return False
        if engine in (1, 5):
            return True
        if engine == 0:
            return False
        if code:
            # Manual/no-sim states often know the type but not the engine
            # enumeration. The type table is enough for the pressurisation
            # question: jets and ordinary turboprops are pressure aircraft,
            # pistons are not unless the exception table already answered.
            from .. import aircraft_class as ac
            cls = ac.classify(code).name
            if cls in (ac.BUSINESS_JET, ac.AIRLINER,
                       ac.TURBOPROP_SINGLE, ac.TURBOPROP_TWIN):
                return True
        return False

    @staticmethod
    def _oxygen_system_available(state) -> bool:
        """Whether the model carries usable crew oxygen.

        The .acf capacity proves the system exists; the simulator's
        contents gauge, where it answers, proves there is oxygen in it.
        A missing gauge answer is not an empty bottle -- older models
        never publish one -- so the capacity is enough then.
        """
        capacity = float(getattr(state, "oxygen_capacity_liters", 0.0) or 0.0)
        remaining = float(getattr(state, "oxygen_remaining_liters", 0.0) or 0.0)
        if capacity > 0.0:
            unavailable = getattr(state, "unavailable", ()) or ()
            if "oxygen_remaining_liters" not in unavailable:
                return remaining > 0.0
            return True
        return bool(remaining > 0.0 or getattr(state, "oxygen_on", False))

    def _physiological_ceiling(self, state) -> float:
        """The altitude the airframe's occupants can be planned to, or 0.

        Zero means physiology imposes no lower cap than the airframe's
        own ceiling. An unpressurised aeroplane with no oxygen system is
        not filed into the hypoxia heights merely because its wings and
        engines can physically reach them.
        """
        if self._pressurized_airframe(state) or self._oxygen_system_available(state):
            return 0.0
        return self.NO_OXYGEN_CEILING_FT

    def _model_ceiling(self, state) -> float:
        """The loaded model's operational ceiling, physiology included.

        X-Plane fills ``service_ceiling_ft`` from Plane Maker's
        ``acf/_max_atc_alt_ft``. MSFS cannot read ``ui_certified_ceiling``
        through SimConnect; its documented operational stand-in is
        DESIGN CRUISE ALT, the value its own ATC/flight planner uses.
        """
        declared = (float(getattr(state, "service_ceiling_ft", 0.0) or 0.0)
                    or float(getattr(state, "design_cruise_alt_ft", 0.0) or 0.0))
        physiological = self._physiological_ceiling(state)
        if declared and physiological:
            return min(declared, physiological)
        return declared or physiological

    def _limits(self, state) -> dict:
        """What the aeroplane may be asked to do, in the units it thinks in.

        Every commanded figure goes through here. The red line is the
        aeroplane's own barber pole or design VC where the model declares
        them, else the class's cruise figure bounded up -- a table cruise
        number is a cruise number, not a never-exceed. The stall floor is
        the model's VS1 with margin. A field the model never publishes is
        a conservative class default, not an exception to the envelope.
        """
        perf = self.performance
        barber = float(getattr(state, "barber_pole_kt", 0.0) or 0.0)
        design_vc = float(getattr(state, "design_vc_kt", 0.0) or 0.0)
        vs1 = float(getattr(state, "design_vs1_kt", 0.0) or 0.0)
        vs0 = float(getattr(state, "design_vs0_kt", 0.0) or 0.0)
        max_ias = perf.cruise_kt * 1.35
        if design_vc > 0.0:
            max_ias = min(max_ias, design_vc)
        if barber > 0.0:
            max_ias = min(max_ias, barber * 0.95)
        min_clean = vs1 * 1.3 if vs1 > 0.0 else 0.0
        flap_fraction = 0.0
        detents = int(getattr(state, "flaps_detents", 0) or 0)
        index = int(getattr(state, "flaps_index", 0) or 0)
        if index > 0:
            flap_fraction = min(1.0, index / float(max(1, detents - 1))
                                if detents > 1 else index / 4.0)
        if vs1 > 0.0 and vs0 > 0.0:
            stall = vs1 + (vs0 - vs1) * flap_fraction
        else:
            stall = vs0 or vs1 or 0.0
        # Flap and gear speeds: the model publishes neither, so the
        # figures are the class's -- where the type's own limits are
        # higher, the lower number is still safe.
        flap_max, gear_max = self._config_limits()
        max_mach = float(getattr(state, "mach_max_operate", 0.0)
                         or getattr(state, "barber_pole_mach", 0.0) or 0.0)
        ceiling = (self._model_ceiling(state)
                   or float(getattr(perf, "ceiling_ft", 0.0) or 0.0))
        # The load limits the airframe is certified for. The model
        # publishes neither (MAX/MIN G FORCE are the g-meter's excursion
        # needles, not the red lines), so the figures are the class's:
        # the normal-category numbers the cruise-speed ladder already
        # sorts aeroplanes by. A profile that declares its own says them.
        pos_g = float(getattr(perf, "pos_g", 0.0) or 0.0)
        neg_g = float(getattr(perf, "neg_g", 0.0) or 0.0)
        if not pos_g:
            pos_g, neg_g = self._g_class_limits()
        return {
            "max_ias_kt": max_ias,
            "min_clean_kt": min_clean,
            "stall_kt": stall,
            "flap_max_kt": flap_max,
            "gear_max_kt": gear_max,
            "max_mach": max_mach,
            "ceiling_ft": ceiling,
            "vs_up_fpm": min(perf.climb_fpm, 4000.0),
            "vs_dn_fpm": -min(perf.descend_fpm, 4000.0),
            "g_pos": pos_g,
            "g_neg": neg_g,
        }

    def _g_class_limits(self) -> tuple[float, float]:
        """The class's structural G limits, (positive, negative).

        The normal-category figures: a transport's +2.5/-1.0, a
        light aeroplane's +3.8/-1.5. An aerobatic or utility-category
        model that wants more says so in its autofly profile; here the
        conservative number flies for everyone.
        """
        if self.performance.cruise_kt >= 140.0:
            return 2.5, -1.0
        return 3.8, -1.5

    def _config_limits(self) -> tuple[float, float]:
        """(max speed for any flap, for the gear down), by class.

        An airliner works to roughly 250/265; a light aeroplane's manual
        says 110-ish for the flaps and about 130 for the wheels. The
        cruise figure is the proxy: it is the class's idea of how big the
        aeroplane is.
        """
        if self.performance.cruise_kt >= 230.0:
            return 250.0, 265.0
        if self.performance.cruise_kt >= 140.0:
            return 200.0, 200.0
        return 110.0, 130.0

    def _safe_speed(self, state, kt: float, lim: dict | None = None) -> float:
        """An indicated-airspeed command inside the envelope.

        The ceiling is the red line; the floor is the stall's margin --
        a demand inside it is raised to the approach's thirty per cent
        over the book figure, the way a crew's slowest number is still
        a flying speed. Where the red line sits under the floor the red
        line wins.
        """
        if lim is None:
            lim = self._limits(state)
        value = float(kt or 0.0)
        if value <= 0.0:
            return 0.0
        floor = max(45.0, (lim["stall_kt"] or 0.0) * 1.3)
        return min(max(value, floor), lim["max_ias_kt"])

    def _safe_altitude(self, state, ft: float, lim: dict | None = None) -> float:
        """An altitude command under the ceiling, where one is known."""
        if lim is None:
            lim = self._limits(state)
        value = float(ft or 0.0)
        if lim["ceiling_ft"] > 0.0:
            value = min(value, lim["ceiling_ft"])
        return value

    def _flaps_allowed(self, state) -> bool:
        """Whether a flap may come out now: never past the flap speed."""
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        lim = self._limits(state)
        if ias > lim["flap_max_kt"]:
            if not self.state.flap_gate_warned:
                self.state.flap_gate_warned = True
                self._emit("info",
                           f"Flaps held: {ias:.0f} kt is past the flap "
                           "speed", "autonomous.limits.flaps")
            return False
        return True

    def _gear_allowed(self, state) -> bool:
        """Whether the gear may come out now: never past the gear speed."""
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        lim = self._limits(state)
        if ias > lim["gear_max_kt"]:
            if not self.state.gear_gate_warned:
                self.state.gear_gate_warned = True
                self._emit("info",
                           f"Gear held: {ias:.0f} kt is past the gear "
                           "speed", "autonomous.limits.gear")
            return False
        return True

    def _set_flaps(self, state, fraction: float) -> None:
        """Move the flap lever -- an extension is gated by the flap speed.

        Retraction is always allowed: bringing them in early is a drag
        penalty, never an airframe one. The lever's last request stands
        for where it is now: a request below it is coming in, whatever
        the schedule called the move -- a go-around's first stage asked
        while past the flap speed must not be held as an extension.
        """
        retracting = fraction <= getattr(self, "_flaps_asked", 0.0)
        if retracting or self._flaps_allowed(state):
            self._flaps_asked = fraction
            self._flaps_owed = None
            self.director.flaps(fraction)
        else:
            # Refused for speed: the stage is owed, not done. It is
            # selected the moment the aeroplane is slow enough for it
            # (see _configure_owed) -- a schedule that ticked the stage
            # off here flew the rest of the descent, and sometimes the
            # final, with the flap it never got.
            self._flaps_owed = fraction

    def _configure_owed(self, state) -> None:
        """Select the flap a speed gate refused earlier, now the speed
        allows it. Run every tick. (The gear is asked again by the
        approach itself until the model says it is down.)"""
        if getattr(state, "on_ground", False) \
                or self.state.phase not in (AutonomousPhase.DESCENT,
                                            AutonomousPhase.APPROACH,
                                            AutonomousPhase.LANDING):
            self._flaps_owed = None
            return
        owed = getattr(self, "_flaps_owed", None)
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        if owed is not None \
                and ias <= self._limits(state)["flap_max_kt"]:
            self._flaps_owed = None
            if owed > getattr(self, "_flaps_asked", 0.0):
                self._flaps_asked = owed
                self.director.flaps(owed)

    def _lever_position(self, state) -> float:
        """Where the levers are, 0..1, as the sim last reported them."""
        pct = float(getattr(state, "throttle_pct", 0.0) or 0.0)
        if pct > 0.0:
            return max(0.0, min(1.0, pct / 100.0))
        return self._last_lever

    def _run_checklist_for_phase(self, checklist_name: str) -> None:
        """Trigger a checklist for the current phase."""
        if checklist_name in self.state.checklists_completed:
            return
        runner = self.crew.checklists
        active = bool(getattr(self, "_checklist_active", ""))
        if runner is not None and (runner.waiting or active):
            # The checklist before this one is still being worked: let it
            # finish rather than abandon it mid-item. The callers move on
            # and never ask again, so the debt is queued and the checklist
            # driver picks it up once the runner is free.
            deferred = list(getattr(self.state, "checklist_deferred", []))
            if checklist_name not in deferred:
                deferred.append(checklist_name)
            self.state.checklist_deferred = deferred
            return
        if runner is not None:
            runner.start(checklist_name)
            self._checklist_active = checklist_name
            self._checklist_active_since = time.monotonic()
            self._checklist_seen_running = False
        self.state.checklists_completed.add(checklist_name)

