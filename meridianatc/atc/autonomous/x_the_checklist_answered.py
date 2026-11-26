from __future__ import annotations

from ._shared import (
    _engine_check,
    _gs_band,
    _nws_fade,
    _yaw_rate_cap,
    ip,
    log,
    time,
)


class _The_checklist_answered:
    # The checklist vars whose control is a toggle rather than a set:
    # asking twice flips the switch back off, so each is asked once.
    _TOGGLED_ONCE = frozenset({"seatbelt_sign"})
    # The most the pedal demand may move between writes: at the loop's
    # quarter-second cadence the full throw takes under a second --
    # the continuous correction a hand makes, rather than a kick.
    _PEDAL_SLEW = 0.15

    # -- the checklist, answered by the FO flying ---------------------------

    def _drive_checklist(self, state, now: float) -> None:
        """Work an open checklist: do the item, then answer it.

        A pilotless aeroplane has nobody for the first officer to wait
        on, so when a run is waiting on an item the FO does the item --
        the runner's own check says whether the aeroplane agrees -- and
        answers it itself. A held run is carried on after a pause; a run
        still being read out is left to finish being said. The answer is
        the runner's ``fo_answer``: the same verification a spoken answer
        gets, so a switch the simulator says is still wrong is objected
        to, never waved through.
        """
        runner = self.crew.checklists
        if runner is None:
            return
        item, run_state = runner.pending()
        if item is None:
            self._checklist_item_id = 0
            # A checklist the phase machine asked for while this one was
            # still running is owed; it is started once the runner can
            # take it, not dropped.
            active = getattr(self, "_checklist_active", "")
            if active:
                running = getattr(runner, "running", "")
                done = getattr(runner, "done", set())
                if running == active or runner.waiting:
                    self._checklist_seen_running = True
                    return
                if active in done or getattr(
                        self, "_checklist_seen_running", False):
                    self._checklist_active = ""
                    self._checklist_seen_running = False
                elif now - getattr(self, "_checklist_active_since", 0.0) \
                        < 3.0:
                    # runner.start() is asynchronous. Do not feed the next
                    # card into the same inbox before this one has even
                    # been consumed; the second start would abandon it.
                    return
                else:
                    # A card removed by the aircraft's files is answered
                    # "there is none" and never enters ``done``; that is
                    # not a reason to block every later phase.
                    self._checklist_active = ""
                    self._checklist_seen_running = False
            deferred = list(getattr(self.state, "checklist_deferred", []))
            if deferred and not runner.waiting:
                name = deferred.pop(0)
                self.state.checklist_deferred = deferred
                if name not in self.state.checklists_completed:
                    runner.start(name)
                    self.state.checklists_completed.add(name)
                    self._checklist_active = name
                    self._checklist_active_since = now
                    self._checklist_seen_running = False
            return
        if id(item) != self._checklist_item_id:
            self._checklist_item_id = id(item)
            self._checklist_item_since = now
            self._checklist_perform_since = now
            self._checklist_sent.clear()
            return
        if run_state == "held":
            if now - self._checklist_item_since > 4.0:
                runner.hear(ip.Said(who=ip.FIRST_OFFICER, what=ip.CONTINUE))
                self._checklist_item_since = now
            return
        if run_state != "waiting":
            return
        # Do the item where the check names a variable the FO can reach;
        # the answer goes once the simulator has had a moment to show it.
        # A model that publishes the variable but does not honour the
        # event is not asked forever: a few seconds in, the item is
        # answered as it stands and the runner's own objection machinery
        # says what disagrees, rather than stalling the checklist on it.
        if self._perform_check_item(state, item) \
                and now - self._checklist_perform_since < 4.0:
            self._checklist_item_since = now
            return
        if now - self._checklist_item_since >= 2.0:
            runner.fo_answer()
            # The next pass sees the advance -- or the objection, which a
            # re-performed item then answers.
            self._checklist_item_since = now

    def _perform_check_item(self, state, item) -> bool:
        """Move the switch an item's check names, where it disagrees.

        True where a control was asked for, so the item is answered on a
        later pass once the new value shows. An item whose check names
        nothing the FO can reach, or nothing at all, is not performed --
        the answer simply reports what is.
        """
        checks = getattr(item, "checks", None) or ()
        if not checks:
            return False
        asked_any = False
        for check in checks:
            if self._move_for_check(state, check):
                asked_any = True
        return asked_any

    def _move_for_check(self, state, check) -> bool:
        runner = self.crew.checklists
        try:
            expected = runner._expected(check)
        except Exception:
            expected = check.value
        ok, _actual = _engine_check(state, check, expected)
        if ok:
            return False
        if check.var in self._TOGGLED_ONCE \
                and check.var in self._checklist_sent:
            # A toggle event has no "on": asked again it would flip the
            # switch back off. One ask per item is all it gets.
            return True
        if check.var == "parking_brake" \
                and ((state.ground_speed_kt or 0.0) > 5.0
                     or not getattr(state, "on_ground", True)):
            # The brake is never set while the aeroplane is rolling or
            # flying: an item answered here fought the roll's release
            # every tick, power against the park brake for the whole
            # strip. The answer reports what is; the switch is the
            # phase machine's own.
            return True
        asked = self._switch_for(state, check.var, expected)
        if asked:
            self._checklist_sent.add(check.var)
        return asked

    def _switch_for(self, state, var: str, expected) -> bool:
        """The cockpit control behind a checklist variable."""
        d = self.director
        want_on = (expected is True) or expected == "on" \
            or (isinstance(expected, (int, float))
                and not isinstance(expected, bool) and expected > 0)
        # Switches whose "off" is never asked for while the aeroplane is
        # moving or flying: an item answered there fought the phase
        # machine (the brake) or switched the aeroplane off in the air
        # (the masters). Parked is the only place they are asked for off
        # -- and no state read is not parked.
        parked = bool(getattr(state, "on_ground", False)) \
            and (getattr(state, "ground_speed_kt", 0.0) or 0.0) <= 5.0
        table = {
            "parking_brake": lambda: d.parking_brake(want_on),
            # Gear out goes through the same gear-speed gate the flying
            # uses: a checklist item does not put the wheels out past
            # Vlo. Coming up is allowed airborne only -- never a retraction
            # while the wheels still carry the aeroplane.
            "gear_down": lambda: (
                (want_on and self._gear_allowed(state))
                # Wheels up on the positive rate, the way the book has
                # it: a landing bounce is airborne for a second without
                # being a climb the gear comes up on.
                or (not want_on
                    and not getattr(state, "on_ground", True)
                    and (getattr(state, "vertical_speed_fpm", 0.0)
                         or 0.0) > 300.0)
            ) and d.gear(want_on),
            "landing_light": lambda: d.landing_light(want_on),
            "taxi_light": lambda: d.taxi_light(want_on),
            "nav_light": lambda: d.nav_lights(want_on),
            "strobe_light": lambda: d.strobes(want_on),
            "beacon_light": lambda: d.beacon(want_on),
            "pitot_heat": lambda: d.pitot_heat(want_on),
            "seatbelt_sign": lambda: d.seatbelts(want_on),
            "battery_master": lambda: (want_on or parked) \
                                      and d.battery(want_on),
            "avionics_master": lambda: (want_on or parked) \
                                       and d.avionics(want_on),
            "spoilers_armed": lambda: d.spoilers_arm(want_on),
            "autopilot": lambda: d.autopilot(want_on),
            "autothrottle": lambda: (d.autothrottle_arm() if want_on
                                     else d.autothrottle_disconnect()),
            "gps_drives_nav1": lambda: d.write("GPS DRIVES NAV1",
                                               1.0 if want_on else 0.0),
            # The check asks for "not off", so its expected value is the
            # off position: writing it would close the selector for good.
            "fuel_selector": lambda: d.write("FUEL TANK SELECTOR:1",
                                             float(expected) if float(expected) > 0 else 1.0),
            # The checklist's figure is a percent; the lever is a
            # fraction on every engine -- a twin whose second mixture
            # was never written runs at whatever the pilot left it.
            "mixture_pct": lambda: d.mixture(float(expected) / 100.0),
            # Rotorcraft and APU outcomes are controls upstream of the
            # percentage the checklist verifies: the APU switch for its
            # RPM, the collective lever for its position, and the rotor
            # system's documented switch events on both simulators.
            "apu_pct": lambda: d.write(
                "APU SWITCH", 1.0 if want_on else 0.0),
            "collective_position": lambda: d.send(
                b"AXIS_COLLECTIVE_SET",
                int(-16383.0 + (0.0 if float(expected) <= 10.0
                                else float(expected)) / 100.0
                    * 32766.0)) or d.send(
                b"HELICOPTER_COLLECTIVE_SET",
                int(16384.0 * (0.0 if float(expected) <= 10.0
                               else float(expected)) / 100.0)),
            "rotor_brake_active": lambda: d.send(
                b"ROTOR_BRAKE_ON" if want_on else b"ROTOR_BRAKE_OFF", 0),
            "rotor_clutch_active": lambda: d.send(
                b"ROTOR_CLUTCH_SWITCH_SET", 1 if want_on else 0),
            "rotor_governor_on": lambda: d.send(
                b"ROTOR_GOV_SWITCH_ON" if want_on
                else b"ROTOR_GOV_SWITCH_OFF", 0),
            "engines_running": lambda: self._engines_for_checklist(want_on),
            # The condition lever's check names a percent where its
            # stops live -- under 10 cut-off, ~50 low idle, over 90 high
            # idle -- and the lever is the enum's three detents.
            "condition_pct": lambda: d.condition(
                round(float(expected) / 50.0)),
            # The lever checks name a percent of travel; the servo's own
            # ramp keeps the write from being the brisk lever jump the
            # ITT fix taught the sims to hate.
            "throttle_pct": lambda: d.throttle(
                min(1.0, float(expected) / 100.0)),
            "prop_pct": lambda: d.prop(
                min(1.0, float(expected) / 100.0)),
            # The mode switch's check names one enum value for "stby"
            # and a list for "on or alt": write the first acceptable
            # detent, which is the one the card means.
            "transponder_state": lambda: d.write(
                "TRANSPONDER STATE:1",
                float(expected[0] if isinstance(expected, (list, tuple))
                      else expected)),
            "transponder_code": lambda: d.transponder(str(expected)),
            # The FCU windows the card asks for: dial the figure the
            # controller gave, the way the human would.
            "ap_altitude_ft": lambda: d.altitude(float(expected)),
            "ap_heading_deg": lambda: d.heading(float(expected)),
            "kohlsman_hpa": lambda: d.kohlsman(
                self._altimeter_hpa(expected)),
            # The check asks for a detent, not a travel: the handle index
            # where the sim takes it, the fraction where it does not --
            # and an extension only within the flap speed, the gate the
            # flying path uses too.
            "flaps_index": lambda: self._checklist_flaps(state, expected),
            "spoilers": lambda: d.spoilers(min(1.0, float(expected))),
            # The systems the second checklist sweep checks, written
            # under the same SimConnect names the state reader uses;
            # X-Plane's WRITABLE table maps each to its dataref.
            "fuel_pump_on": lambda: d.write(
                "GENERAL ENG FUEL PUMP SWITCH:1", 1.0 if want_on else 0.0),
            "generator_on": lambda: (want_on or parked) and d.write(
                "GENERAL ENG GENERATOR SWITCH:1", 1.0 if want_on else 0.0),
            "alternator_on": lambda: (want_on or parked) and d.write(
                "GENERAL ENG MASTER ALTERNATOR:1", 1.0 if want_on else 0.0),
            "ignition_on": lambda: d.write(
                "TURB ENG IGNITION SWITCH:1", 1.0 if want_on else 0.0),
            # The key's five stops: X-Plane's single enum answers the
            # one write; SimConnect's two switches are the sum's parts.
            "magneto_pos": lambda: self._magneto(d, 1, expected),
            "carb_heat_pct": lambda: d.write(
                "GENERAL ENG ANTI ICE POSITION:1",
                float(expected) / 100.0),
            "exits_open_pct": lambda: d.write(
                "EXIT OPEN:1", float(expected)),
            "brake_left_pct": lambda: d.write(
                "BRAKE LEFT POSITION", float(expected), unit="Percent"),
            "brake_right_pct": lambda: d.write(
                "BRAKE RIGHT POSITION", float(expected), unit="Percent"),
            "yaw_damper_on": lambda: d.write(
                "AUTOPILOT YAW DAMPER", 1.0 if want_on else 0.0),
            "antiskid_on": lambda: d.write(
                "ANTISKID BRAKES ACTIVE", 1.0 if want_on else 0.0),
            "autobrake_pos": lambda: d.write(
                "AUTO BRAKE SWITCH CB", float(expected)),
            "speedbrake_pct": lambda: d.write(
                "SPOILERS HANDLE POSITION", float(expected)),
            "apu_on": lambda: d.write(
                "APU SWITCH", 1.0 if want_on else 0.0),
            "apu_gen_on": lambda: d.write(
                "APU GENERATOR SWITCH", 1.0 if want_on else 0.0),
            "apu_bleed_on": lambda: d.write(
                "BLEED AIR APU", 1.0 if want_on else 0.0),
            "pack_left_on": lambda: d.write(
                "APU PACKS", 1.0 if want_on else 0.0),
            "bleed_air_on": lambda: d.write(
                "BLEED AIR ENGINE:1", 1.0 if want_on else 0.0),
            "engine_anti_ice_on": lambda: d.write(
                "ENG ANTI ICE:1", 1.0 if want_on else 0.0),
            "wing_deice_on": lambda: d.write(
                "STRUCTURAL DEICE SWITCH", 1.0 if want_on else 0.0),
            "window_heat_on": lambda: d.write(
                "WINDSHIELD DEICE SWITCH", 1.0 if want_on else 0.0),
            "prop_deice_on": lambda: d.write(
                "PROP DEICE SWITCH:1", 1.0 if want_on else 0.0),
            "aoa_heat_on": lambda: d.write(
                "PANEL ANTI ICE SWITCH", 1.0 if want_on else 0.0),
            "hydraulic_pump_on": lambda: d.write(
                "HYDRAULIC SWITCH:1", 1.0 if want_on else 0.0) or d.write(
                # MSFS files the var unindexed; X-Plane's map wants the
                # :1 the first write already tried.
                "HYDRAULIC SWITCH", 1.0 if want_on else 0.0),
            "cabin_altitude_goal_ft": lambda: d.write(
                "PRESSURIZATION CABIN ALTITUDE GOAL", float(expected)),
            "minimums_ft": lambda: d.write(
                "DECISION HEIGHT", float(expected)),
            "no_smoking_sign": lambda: d.write(
                "CABIN NO SMOKING ALERT SWITCH", 1.0 if want_on else 0.0),
            "oxygen_on": lambda: d.write(
                "OXYGEN SWITCH", 1.0 if want_on else 0.0),
            "wiper_pos": lambda: d.write(
                "WIPER SWITCH", float(expected)),
        }
        # The same systems on engines two through sixteen (the sims'
        # highest index): each SimConnect name the scalars use, with
        # the state's engN_ spelling swapped for the :N index.
        for n in range(2, 17):
            table.setdefault(
                f"eng{n}_fuel_pump_on",
                lambda n=n: d.write(f"GENERAL ENG FUEL PUMP SWITCH:{n}",
                                    1.0 if want_on else 0.0))
            table.setdefault(
                f"eng{n}_generator_on",
                lambda n=n: (want_on or parked) and d.write(
                    f"GENERAL ENG GENERATOR SWITCH:{n}",
                    1.0 if want_on else 0.0))
            table.setdefault(
                f"eng{n}_alternator_on",
                lambda n=n: (want_on or parked) and d.write(
                    f"GENERAL ENG MASTER ALTERNATOR:{n}",
                    1.0 if want_on else 0.0))
            table.setdefault(
                f"eng{n}_ignition_on",
                lambda n=n: d.write(f"TURB ENG IGNITION SWITCH:{n}",
                                    1.0 if want_on else 0.0))
            table.setdefault(f"eng{n}_magneto_pos",
                             lambda n=n: self._magneto(d, n, expected))
            table.setdefault(
                f"eng{n}_bleed_air_on",
                lambda n=n: d.write(f"BLEED AIR ENGINE:{n}",
                                    1.0 if want_on else 0.0))
        action = table.get(var)
        if action is None:
            return False
        try:
            # The writes answer whether the sim took them -- a refused
            # one is not the item done.
            return bool(action())
        except Exception:
            log.debug("checklist switch %s failed", var, exc_info=True)
            return False

    def _checklists_settled(self) -> bool:
        """No checklist is running, starting, or owed on a later pass."""
        runner = getattr(getattr(self, "crew", None), "checklists", None)
        if runner is None:
            return True
        return (not getattr(runner, "waiting", False)
                and not getattr(self, "_checklist_active", "")
                and not getattr(self.state, "checklist_deferred", []))

    def _engines_for_checklist(self, want_on: bool) -> bool:
        """The engine-running state a shutdown/start item verifies."""
        if want_on:
            return bool(self._send_event(b"ENGINE_AUTO_START", 1))
        self._shutdown_engines()
        return True

    @staticmethod
    def _magneto(d, engine: int, expected) -> bool:
        """The magneto key's five stops on either sim: X-Plane's
        ignition_key is the enum outright; SimConnect's two switches
        sum to it (start keeps them on both and cranks -- the off,
        left, right and both a checklist asks for are the two)."""
        pos = expected[0] if isinstance(expected, (list, tuple)) \
            else expected
        pos = int(pos)
        if d.write(f"IGNITION_KEY:{engine}", float(pos)):
            return True
        left, right = pos in (1, 3, 4), pos in (2, 3, 4)
        wrote = False
        for name, on in ((f"RECIP ENG LEFT MAGNETO:{engine}", left),
                         (f"RECIP ENG RIGHT MAGNETO:{engine}", right)):
            wrote = d.write(name, 1.0 if on else 0.0) or wrote
        return wrote

    def _checklist_flaps(self, state, expected) -> bool:
        """The flap detent a checklist item names, behind the same
        direction-aware speed gate the flying uses: coming in is always
        allowed, going out never past the flap speed."""
        want = min(1.0, float(expected) / 4.0)
        if want > getattr(self, "_flaps_asked", 0.0) \
                and not self._flaps_allowed(state):
            return False
        self._flaps_asked = want
        d = self.director
        return bool(d.write("FLAPS HANDLE INDEX", float(expected))
                    or d.flaps(want))

    def _configure_for_takeoff(self, state) -> None:
        """Set flaps, trim, lights for takeoff."""
        d = self.director
        # Takeoff flaps: the profile's detent where the type declares
        # one -- detent zero, a King Air's flaps-up takeoff, is a
        # declaration -- else the first detent, whatever the lever was
        # left at, which is the point of setting it.
        takeoff_flaps = self.performance.takeoff_flaps
        d.flaps(takeoff_flaps if takeoff_flaps >= 0.0 else 0.25)
        d.landing_light(True)
        d.strobes(True)

    def _slew_lever(self, want: float, up_per_s: float = 0.20,
                    down_per_s: float = 0.60) -> float:
        """The lever moved toward ``want`` at a hand's speed, not a
        wire's.

        A lever write that jumps the travel commands an acceleration
        the engine cannot give, and the model answers it with a surge
        the passengers would feel. The taxi path calls this at ground
        rates -- a hand easing the levers open -- and the takeoff roll
        at faster ones, because even a brisk hand needs seconds to
        reach the stop. Coming back is always allowed to be quicker:
        retarding late is how the speed runs away.
        """
        now = time.monotonic()
        last_at = getattr(self, "_lever_at", 0.0)
        dt = max(0.0, min(now - last_at, 1.0)) if last_at else 0.0
        self._lever_at = now
        want = max(0.0, min(1.0, want))
        lever = float(getattr(self, "_last_lever", 0.0) or 0.0)
        if want > lever:
            lever = min(want, lever + up_per_s * dt)
        else:
            lever = max(want, lever - down_per_s * dt)
        self._last_lever = lever
        return lever

    def _turn_rate(self, hdg: float) -> float:
        """How fast the nose is coming around, in degrees a second.

        Positive is turning right. The state the laws steer from is a
        tick or two old, so the raw delta between reads is already an
        average over the gap; a light smoothing on top keeps one odd
        sample from being answered as a spin. The measured gap itself
        is kept on ``_steer_dt`` -- the horizon a steering write needs
        to lead by is the loop's own period, not a guess at it.
        """
        now = time.monotonic()
        last_at = getattr(self, "_hdg_at", 0.0)
        prev = getattr(self, "_last_hdg", None)
        self._hdg_at = now
        self._last_hdg = hdg
        if prev is None or not last_at:
            return 0.0
        dt = now - last_at
        if dt <= 0.0:
            return 0.0
        self._steer_dt = dt
        raw = ((hdg - prev + 540.0) % 360.0 - 180.0) / dt
        raw = max(-90.0, min(90.0, raw))
        rate = 0.5 * getattr(self, "_turn_rate_dps", 0.0) + 0.5 * raw
        self._turn_rate_dps = rate
        return rate

    def _steer_lead(self, err: float, rate: float) -> float:
        """The error a steering write will still be answering to when it
        lands, not the one a stale read reports: while the nose is
        already coming around the lead shrinks the ask, so the wheel is
        centred as it arrives rather than thrown over after it has
        passed. The horizon is short and bounded: reads arrive late and
        the measured rate is itself noisy, so projecting it for the
        loop's whole period turns a sampled weave into the next
        reversal. Without the rate term a saturated law under lag
        bang-bangs -- or with the correction arriving late enough,
        orbits the apron.
        """
        # A second or so is about all the writes and reads can honestly
        # predict: the two-to-three second horizon this used to project
        # turned a sampled weave into a steering reversal of its own.
        horizon = max(0.5, min(1.5, 1.5 * getattr(self, "_steer_dt", 2.0)))
        return err - rate * horizon

    def _pedal_dir(self) -> float:
        """The sign this model's steering axis actually answers in."""
        return -1.0 if getattr(self, "_pedal_flip", False) else 1.0

    def _pedal_check(self, demand: float, rate: float,
                     gs: float, vote: bool = True) -> None:
        """Flip the pedal sign when the axis answers backwards.

        A wheel asked right while the nose measures left is an inverted
        axis on the model: every correction enlarges the error it was
        answering, and the aeroplane circles whatever it was told to
        follow. ``rate`` answers the demand already down the wire --
        this tick's write has not landed yet -- so the vote is whether
        the achieved yaw agrees with the *law's* demand from last call,
        not the signed output: after a flip a demanded right turn is
        sent left and a rightward answer is agreement.

        A vote only means something when the evidence is the wheel's
        own: the demand must have held its side across calls, since a
        demand that just flipped or alternates inherits yaw that
        belongs to an older command. Five disagreed answers flip the
        sign; the count is re-seeded positive after a flip so a
        correctly-wired model never chatters between the two.
        """
        prev = getattr(self, "_pedal_demand", 0.0)
        self._pedal_demand = demand
        streak = getattr(self, "_demand_streak", 0)
        streak = streak + 1 if prev and (demand > 0.0) == (prev > 0.0) \
            else 0
        self._demand_streak = streak
        # Gain autoadjust: the degrees a second that one unit of pedal
        # buys on this model, learned from the answer each demand got.
        # Only a held demand is evidence: the live Baron's weave was
        # alternating pedal and rate signs every few seconds, and using
        # absolute magnitudes there counted one side of the oscillation
        # as the other pedal's authority. The answer must therefore
        # agree in direction with the demand and the demand must have
        # held long enough for the nosewheel and airframe lag to be
        # part of the measurement rather than its reversal. The
        # aeroplane must be rolling too: a tug or the wind can put
        # rate on a parked airframe that the pedal never made.
        #
        # The figure is kept per groundspeed band: authority grows with
        # the knots, so a sample taken on the apron is written where it
        # was taken and the rollout's band keeps its own. The legacy
        # whole-speed value is kept updated too -- it is the fallback a
        # band that has no evidence yet reads, and the only figure an
        # old store carries.
        if abs(prev) >= 0.15 and abs(rate) >= 3.0 and gs >= 1.0 \
                and streak >= 2 and rate * prev > 0.0:
            sample = abs(rate) / abs(prev)
            # A sample is only believable as steering's own work while
            # the measured swing is something a pedal could plausibly
            # make: the figure a ground-handling snap or a teleport
            # writes is the authority the next hundred asks are then
            # divided by -- the whispered demands a slow veer eats a
            # runway's width under.
            if 5.0 <= sample <= 50.0 and abs(rate) <= 45.0:
                band = _gs_band(gs)
                bands = getattr(self, "_yaw_authority_b", None)
                if bands is None:
                    bands = self._yaw_authority_b = {}
                learned = bands.get(band, 0.0)
                bands[band] = sample if learned <= 0.0 \
                    else 0.5 * learned + 0.5 * sample
                self._yaw_authority = bands[band]
                adapt = getattr(self, "_adapt", None)
                if adapt is not None:
                    adapt.learn(f"steer.authority.{band}",
                                bands[band], blend=0.5)
                    adapt.learn("steer.authority",
                                bands[band], blend=0.5)
        # The other half of the estimate, the direction the samples
        # never see: a demand answered by much less rate than the
        # figure predicts means the figure flatters the model -- and
        # every ask divided by it stays a whisper, the under-demand a
        # weak-steering aeroplane veers off the strip beneath. The
        # sample gate cannot catch that answer because the rate never
        # reaches it, so the shortfall is measured straight: sustained
        # weak answers decay the band's figure until the asks bite.
        est = self._yaw_authority_at(gs)
        # A weave is not a weak answer: the measured rate spends half of
        # every swing crossing zero, and each crossing scores as the
        # shortfall this decay exists to correct -- the figure decays,
        # the asks grow, and the hunt the bigger pedal excites deepens.
        # Weak is only counted while the answer is actually absent --
        # the rate opposing the demand or the nose dead -- and the
        # streak is longer than the half of a weave's cycle that can
        # fake it, never while the rate swings the demand's own way.
        if abs(prev) >= 0.15 and gs >= 1.0 \
                and abs(rate) < 0.30 * abs(prev) * est \
                and (rate * prev <= 0.0 or abs(rate) <= 0.5):
            weak = getattr(self, "_weak_streak", 0) + 1
        else:
            weak = 0
        self._weak_streak = weak
        if weak >= 6:
            self._weak_streak = 0
            band = _gs_band(gs)
            bands = getattr(self, "_yaw_authority_b", None)
            if bands is not None and bands.get(band, 0.0) > 0.0:
                bands[band] = max(5.0, bands[band] * 0.85)
                learned = bands[band]
            else:
                learned = max(5.0, getattr(
                    self, "_yaw_authority", 0.0) * 0.85 or 25.0)
            self._yaw_authority = learned
            adapt = getattr(self, "_adapt", None)
            if adapt is not None:
                adapt.learn(f"steer.authority.{band}", learned,
                            blend=0.5)
                adapt.learn("steer.authority", learned, blend=0.5)
        if not vote or abs(prev) < 0.3 or abs(rate) < 3.0 \
                or gs < 1.0 or streak < 3:
            return
        adapt = getattr(self, "_adapt", None)
        votes = getattr(self, "_pedal_votes", 0)
        votes += 1 if (rate > 0.0) == (prev > 0.0) else -1
        if votes <= -5:
            self._pedal_flip = not getattr(self, "_pedal_flip", False)
            votes = 3
            log.warning("steering axis answers backwards on this "
                        "model; pedal sign flipped")
            if adapt is not None:
                adapt.set("steer.flip", 1.0 if self._pedal_flip else 0.0)
                # A polarity is the one verdict a crash must not take
                # with it: the next flight on a stored flip is wired
                # right before the first metre is taxied.
                from .. import adaptation
                adaptation.save(force=True)
        self._pedal_votes = votes
        if adapt is not None:
            adapt.learn_votes("steer", votes)

    def _yaw_authority_at(self, gs: float) -> float:
        """The deg/s of yaw one unit of pedal buys at this groundspeed.

        Learned per speed band in ``_pedal_check``, because the same
        travel steers harder the faster the wheels roll: the figure
        measured on the apron applied to a rollout at sixty knots is
        the weave this law exists to stop. The current band's figure
        answers first, then the nearest band that has one, then the
        legacy whole-speed figure, then a middling guess.
        """
        bands = getattr(self, "_yaw_authority_b", None) or {}
        got = bands.get(_gs_band(gs))
        if got is None:
            band = _gs_band(gs)
            for off in range(1, 6):
                got = bands.get(band - off) or bands.get(band + off)
                if got is not None:
                    break
        if not got:
            got = getattr(self, "_yaw_authority", 0.0)
        if got < 5.0:
            got = 30.0
        return got

    def _ground_directional(self, state, want_hdg: float, gs: float,
                            rate_cap: float = 5.0, pedal_max: float = 0.4,
                            demand: float = 1.0, rate_damp: float = 1.0,
                            err_share: float = 0.3,
                            err_gain: float = 0.02) -> float:
        """The signed pedal demand that holds a heading on a fast roll.

        The takeoff roll, the rollout and the flare's de-crab steer by
        rate, the way the taxi law does: the led error asks for a small
        yaw rate -- a few degrees a second is a correction at a hundred
        knots, not a weave -- and the pedal answers the difference
        between that and the measured rate, divided by the authority
        this model has shown at this speed. The bare proportional law
        this replaces had no memory: a model whose rudder answered half
        as strongly as the assumed one weaved until the weave was the
        flight.
        """
        hdg = state.heading_true or 0.0
        err = ((want_hdg - hdg + 540.0) % 360.0 - 180.0)
        rate = self._turn_rate(hdg)
        # ``rate_damp`` de-rates the measured swing wherever the demand
        # would answer it -- in the lead's projection and in the servo
        # difference below. On the roll the figure arrives two beats
        # late: asked at full weight both places it is the weave's own
        # pump, collapsing the ask on the half-cycle the plant already
        # answers, and the troughs that leaves are the ground the
        # standing veer is gained on.
        lead = self._steer_lead(err, rate * rate_damp)
        # The situation's press is on the ask before the cap: a firmer
        # demand reaches the cap sooner, a softer one sits under it --
        # the cap itself is never the thing pressed.
        want_rate = max(-rate_cap, min(
            rate_cap,
            lead / 4.0 * max(0.6, min(1.35, demand))))
        authority = self._yaw_authority_at(gs)
        # The roll's pedal is a damper, not a rate-follower: the asks
        # here are degrees, and the sharpness a feed-forward term adds
        # at roll speed is the weave the sim's own veer is already
        # feeding -- the droop a bare difference leaves is the price
        # of a straight roll, not a wide corner. ``rate_damp`` is the
        # fraction of the measured rate the subtraction keeps: on the
        # takeoff roll the figure arrives two beats late -- a full
        # subtraction is the weave's own pump, each counter-ask landing
        # in time to be the next overshoot's push, and the troughs it
        # leaves are the ground the standing veer is gained on. A
        # third of the answer is enough of the brake; the callers that
        # steer with the lag under a second keep the whole one.
        pedal = max(-pedal_max,
                    min(pedal_max,
                        (want_rate - rate * rate_damp) / authority))
        # The rate law alone droops: the error reaches the pedal only
        # through the authority figure, and a figure that flatters the
        # model leaves a sustained error answered by a whisper -- the
        # slow veer a roll leaves the runway's side on. An
        # error-proportional share is the ask the figure cannot talk
        # out of existence, bounded so it stays a lean -- and only where
        # it agrees with the rate term, so a nose already coming around
        # still gets the counter-pedal that stops the overshoot.
        if pedal * err > 0.0:
            pedal = max(-pedal_max, min(
                pedal_max, pedal + max(-err_share,
                                       min(err_share, err * err_gain))))
        if abs(err) < 1.5 and abs(rate) < 1.0:
            pedal = 0.0
        # Walked like the taxi wheel, and smoothed harder than it: the
        # measured rate this demand answers is itself the weave, and a
        # pedal that chases it beat for beat pumps the swing it exists
        # to damp -- every hard ask arriving in time to be the next
        # overshoot's push. The foot a takeoff roll is flown with is
        # the demand's slow half: the standing pressure the drift
        # costs, held while the fast counter-swings are taken off. A
        # real excursion still arrives -- a beat later, at the pace a
        # leg makes one.
        prev_pedal = getattr(self, "_roll_pedal_out", 0.0)
        pedal = prev_pedal + 0.35 * (pedal - prev_pedal)
        step = pedal - prev_pedal
        step = max(-self._PEDAL_SLEW, min(self._PEDAL_SLEW, step))
        pedal = self._roll_pedal_out = prev_pedal + step
        # No sign vote here: the roll's yaw is not the wheel's own
        # answer. At takeoff power the prop's left-turning moment
        # disagrees with a right demand under either sign, and the
        # vote read that as an inverted axis four times in nine
        # seconds -- the inverted half of the flapping doubling the
        # veer it was meant to cure. The axis's sign is learned where
        # the wheel owns the yaw -- the taxi law's -- and stands for
        # the whole roll. The magnitude half of the estimate still
        # runs: a demand the plant answers weakly decays the authority
        # the asks are divided by, so a rudder that bites far less at
        # roll speed than the tiller the figure was learned on ramps
        # the pedal toward the stop instead of drooping off the edge.
        self._pedal_check(pedal, rate, gs, vote=False)
        return pedal * self._pedal_dir()

    def _right_foot(self, state) -> float:
        """The standing right-rudder a prop takeoff is flown with.

        The engine's left-turning moment -- torque, the prop's
        slipstream, P-factor where the tail is low -- is the roll's
        standing load, and a damper that only answers the yaw it makes
        has already spent the heading it was bought to keep. A pilot
        leads with the right foot as the power comes in and stands it
        down as the airflow builds; this is that, full ask below the
        speed the rudder flies at, faded to nothing at rotate where
        the wings finally take the load the pedals were carrying. The
        moment it answers is standing until the wheels leave -- a fade
        that ends sooner hands the dead band between the nosewheel's
        authority and the rudder's back to the veer.
        """
        throttle = max(0.0, min(1.0,
                                (getattr(state, "throttle_pct", 0.0)
                                 or 0.0) / 100.0))
        ias = (getattr(state, "indicated_airspeed_kt", 0.0)
               or getattr(state, "ground_speed_kt", 0.0) or 0.0)
        rotate = (getattr(self.state, "rotate_speed_kt", 0.0) or 0.0) \
            or 55.0
        return 0.30 * throttle * max(
            0.0, 1.0 - ias / max(1.0, rotate))

    def _roll_aileron(self, state, gs_kt: float, takeoff: bool) -> float:
        """Aileron into the wind on the ground roll -- the AFH technique.

        Takeoff: full into-wind deflection at brake release, eased
        toward neutral as the speed builds -- the surfaces only come
        alive with the airflow, and a yoke still crossed at rotate is
        the wing that drops first. Landing: the mirror of it -- a light
        bias over the numbers growing to full as the aeroplane slows
        and the rudder fades below it, which is what a gusty day rolls
        out on. ``aircraft_wind_x_kt`` carries the gusts, so the input
        is as lively as the air is -- and it is the same sign the
        flare's into-wind bias uses, so a model whose wind axis is
        flipped banks the wrong way the same gentle amount, not into
        the weeds.
        """
        # ``aircraft_wind_x_kt`` is the wind's velocity on the body
        # axis: positive blows toward the right wing -- a crosswind
        # from the left. The yoke goes *into* the wind, the wing down
        # on the side it comes from -- the opposite sign of the drift
        # it is fighting. Held the other way the deflection's adverse
        # yaw leans on the same side the prop's turning moment does,
        # which is how a left-quartering wind ends a roll off the
        # runway's left edge.
        xwind = float(getattr(state, "aircraft_wind_x_kt", 0.0) or 0.0)
        rotate = max(40.0, float(self.performance.rotate_kt or 60.0))
        if takeoff:
            # Full at the brakes, a tenth of it left at rotate.
            scale = max(0.0, 1.0 - 0.9 * gs_kt / rotate)
        else:
            # Light at touchdown speed, full at a fast walk.
            scale = max(0.15, min(1.0, 1.5 - gs_kt / 60.0))
        return max(-0.3, min(0.3, -xwind * 0.02)) * scale

    def _breakaway_lever(self) -> float:
        """The momentary lever that starts the aeroplane rolling.

        A loaded transport wants a third of the travel to break away
        from the stand; a light single wants a fifth -- and this model
        wants whatever it took last time, where the store has measured
        one: the lever the last pull-away actually moved at. Asked for
        only while the wheels are stopped -- once rolling, the speed
        law owns the lever and the power comes back to idle, the way
        the surge ends the moment the aeroplane is moving.
        """
        adapt = getattr(self, "_adapt", None)
        learned = adapt.value("taxi.breakaway", 0.0) \
            if adapt is not None else 0.0
        if learned >= 0.15:
            return max(0.15, min(0.50, learned))
        cruise = self.performance.cruise_kt
        if cruise >= 230.0:
            return 0.38
        if cruise >= 140.0:
            return 0.30
        return 0.22

    def _prop_levers(self, prop: float = 1.0, mixture: float = 1.0) -> None:
        """Props forward, mixture rich -- the levers power is made with,
        on every engine.

        A quadrant lever left at low pitch governs its propeller down to
        an engine that cannot move the aeroplane -- the stand's "power
        applied, nothing happens" -- and the hardware axis holds it
        there every frame unless the FO owns the channel, which these
        writes latch the same way the throttles' do. Only a prop class
        has the levers to move: on a helicopter the prop channel is the
        collective, and a full-forward write there is a climb nobody
        asked for; a jet has neither lever."""
        from .. import aircraft_class as ac
        cls = getattr(self, "_aircraft_class", "")
        if cls not in (ac.PISTON_SINGLE, ac.PISTON_TWIN,
                       ac.TURBOPROP_SINGLE, ac.TURBOPROP_TWIN):
            return
        d = self.director
        d.prop(prop)
        d.mixture(mixture)
        # A turboprop's third lever is its condition lever -- fuel cock
        # and idle select in one -- and high idle is the detent the
        # propeller can take takeoff power from: the lever left at low
        # idle governs the prop down to an engine that cannot give it,
        # the same accident the prop lever's low pitch writes.
        if cls in (ac.TURBOPROP_SINGLE, ac.TURBOPROP_TWIN):
            d.condition(2.0)

    def _set_takeoff_power(self, state=None) -> None:
        """Takeoff thrust: advanced the way a hand moves the levers --
        briskly but progressively, not slammed to the stop -- and TO/GA
        on the autothrottle where the model has one. The one-shot events
        re-arm on each roll and each go-around; the full-power event
        confirms once the levers have walked to the stop. The lever
        itself stays under the engine watch's ceiling: the takeoff
        rating is a limit, not a target."""
        d = self.director
        # Takeoff power on a prop aeroplane is props forward first --
        # full forward is where the lever can actually give it.
        self._prop_levers(mixture=self._takeoff_mixture(state))
        cap = (self._engine_limit_cap(state, takeoff=True)
               if state is not None else 1.0)
        from .. import aircraft_class as ac
        rate = (0.18 if getattr(self, "_aircraft_class", "") in (
                    ac.TURBOPROP_SINGLE, ac.TURBOPROP_TWIN) else 0.45)
        lever = self._slew_lever(min(1.0, cap), up_per_s=rate)
        d.throttle(lever)
        if lever >= 1.0:
            d.takeoff_power()
        d.toga(True)

    def _takeoff_mixture(self, state) -> float:
        """The red knob for the roll: rich everywhere but a piston's
        hot-and-high, where a full-rich sea-level takeoff floods the
        power the density altitude already stole -- the POH's "lean for
        maximum power" above roughly three thousand feet DA."""
        from .. import aircraft_class as ac
        if getattr(self, "_aircraft_class", "") not in (
                ac.PISTON_SINGLE, ac.PISTON_TWIN) or state is None:
            return 1.0
        density_alt = self._density_altitude(state)
        if density_alt is not None and density_alt > 3000.0:
            return 0.8
        return 1.0

    def _airborne_confirmed(self, state, now: float) -> bool:
        """Whether the aeroplane is really off -- the flag held long
        enough to mean it, not a read the surface flicked.

        SIM ON GROUND flickers off over rough pavement and grass, and
        the whole rotate-and-climb path hangs on one read of it: a blip
        at forty knots walks the roll past every speed check to a
        "climb" at ground level. The flag has to stay off for the
        better part of a second before it is believed -- a liftoff
        stays off; a bounce gets its ground read back.
        """
        if state.on_ground is not False:
            self._off_ground_since = 0.0
            return False
        since = getattr(self, "_off_ground_since", 0.0)
        if not since:
            self._off_ground_since = now
            return False
        return now - since >= 0.8

    def _roll_note(self, state, ias: float, rotate_at: float,
                   sub: str) -> None:
        """The takeoff roll's own telemetry line, every tick.

        The roll answers to no steering law, so the steer line goes
        quiet precisely when the aeroplane needs watching most: speed
        against the speed owed, nose attitude, and how much pavement the
        alignment reckons is left. A roll that ends in the overrun is
        the accident the log has to explain.
        """
        gs = state.ground_speed_kt or 0.0
        pitch = float(getattr(state, "pitch_deg", 0.0) or 0.0)
        vs = float(getattr(state, "vertical_speed_fpm", 0.0) or 0.0)
        along = getattr(self, "_roll_along", None)
        left = getattr(self, "_roll_left", None)
        pedal = getattr(self, "_roll_demand", None)
        dist = (f" along {along:.0f} m left {left:.0f} m"
                if along is not None and left is not None else "")
        ask = f" pedal {pedal:+.2f}" if pedal is not None else ""
        log.info(
            "%s: ias %.1f gs %.1f pitch %+.1f vs %+.0f rotate %.0f%s%s",
            sub, ias, gs, pitch, vs, rotate_at, dist, ask)

    def _ground_still(self, state, now: float) -> bool:
        """Whether the aeroplane has refused to move under power for a
        while -- chocked, braked, or boxed in. Taxi power against a
        stationary aeroplane is how the engine ends up at takeoff rpm on
        the ramp, so after a dozen seconds of it the levers come back to
        idle and the phase decides what to do about not moving."""
        if (state.ground_speed_kt or 0.0) > 1.0:
            self._ground_still_since = 0.0
            return False
        if self._ground_still_since == 0.0:
            self._ground_still_since = now
            return False
        stuck = now - self._ground_still_since
        if stuck <= 12.0:
            return False
        # Idle while it refuses to move, but probe with power for a few
        # seconds of every half minute: a chock coming out or a brake
        # releasing should not need a whole new flight to notice. The
        # lever slews up through the window, so eight seconds is what a
        # heavy's breakaway needs.
        return not 22.0 <= stuck % 30.0 < 30.0

    def _stuck_probe(self, state, now: float) -> int:
        """Which pull-away probe a stuck aeroplane is being given.

        0 while it is free to roll -- moving, not yet stuck, or idling
        in the stretch between probes -- and the count of the open probe
        window otherwise, so each retry may ask the engine for more than
        the attempt that failed to move it. The window arithmetic is the
        same ``_ground_still`` does: the probe is the last eight seconds
        of every thirty stuck.
        """
        if (state.ground_speed_kt or 0.0) > 1.0 \
                or not getattr(self, "_ground_still_since", 0.0):
            return 0
        stuck = now - self._ground_still_since
        if not 22.0 <= stuck % 30.0 < 30.0:
            return 0
        return int(stuck // 30) + 1

    def _ground_steer_limit(self, state) -> float:
        """How much nosewheel and rudder the ground may take at this
        speed. Full travel is a parking-stand turn; the same travel at
        taxi speed is the overturn the corner was always going to invite,
        so the cap fades as the knots build."""
        if not state.on_ground:
            return 0.5
        gs = state.ground_speed_kt or 0.0
        if gs >= 35.0:
            return 0.15
        return max(0.15, 0.5 - gs * 0.012)

    def _steer(self, state, desired_hdg: float, target_kt: float,
               max_aileron: float = 0.35, demand: float = 1.0,
               brake_turn: bool = False,
               brake_margin_kt: float = 4.0) -> None:
        """Hand-fly a heading and a speed on the ground or in the air.

        The rudder (which is the nosewheel on the ground) takes the
        heading error; the throttle takes the speed. Used for taxi,
        line-up, the takeoff roll's directional control and the rollout --
        the places an autopilot's heading mode does not reach.

        ``demand`` is the situation's press on the rate the law asks
        for: over one where the caller wants the nose around now -- a
        rejoin from far off the line, a corner -- under one where the
        aeroplane is already tracking it, bounded so neither a firmer
        nor a softer pass becomes a different law. ``brake_turn`` says a
        real corner is being made, so the inside wheel may help it
        around the way it does for the castoring gears.
        ``brake_margin_kt`` is how far over target the speed may drift
        before the brakes answer it -- wide where speed only has to be
        close, narrow where a slowdown is owed by a point on the map.
        """
        d = self.director
        d.parking_brake(False)
        # The levers power is made with, owned the way the throttle is:
        # a prop lever left at low pitch governs its engine to a
        # propeller that cannot pull the aeroplane away, and the
        # pilot's hardware axis holds it there until an override takes
        # it -- the parked Baron that would not taxi.
        self._prop_levers()
        hdg = state.heading_true or 0.0
        # The pursuit point can teleport between legs of the route --
        # where two taxiway edges sit close, the nearest-point pick
        # ping-pongs and the demanded heading jumps tens of degrees a
        # tick, which no loop can converge on. A jump that size is still
        # taken a share at a time, as a brisk turn order rather than a
        # teleport; but a small jump is a real correction, and easing
        # that was the trail the pursuit wove behind the turn -- so the
        # step never comes in under what a tick of turning answers to.
        prev_want = getattr(self, "_steer_want", None)
        if prev_want is not None:
            jump = (desired_hdg - prev_want + 540.0) % 360.0 - 180.0
            step = min(abs(jump), max(10.0, 0.45 * abs(jump)))
            desired_hdg = (prev_want
                           + (step if jump >= 0.0 else -step)) % 360.0
        self._steer_want = desired_hdg
        err = ((desired_hdg - hdg + 540.0) % 360.0 - 180.0)
        # A near-reversal is the shortest-path law's singular case: at
        # ~180 degrees out a degree of wobble flips which way is
        # "shorter", and a loop free to re-pick slams the wheel from
        # stop to stop while the aeroplane pivots in place -- the donut
        # the apron shows. Commit to the side the first big error
        # showed, the way a pilot picks a side for the turnaround, and
        # keep it until the turn is genuinely made.
        commit = getattr(self, "_turn_commit", 0)
        if commit and abs(err) < 60.0:
            commit = 0
        if not commit and abs(err) > 150.0:
            commit = 1 if err > 0.0 else -1
        self._turn_commit = commit
        if commit:
            err = commit * max(60.0, abs(err))
        rate = self._turn_rate(hdg)
        # A rate servo, not a bare proportional law: the error asks for
        # a yaw rate -- capped at a brisk taxi turn -- and the pedal
        # answers the difference between the rate wanted and the rate
        # measured. Braking is then built in: a nose already swinging
        # gets counter-pedal whether or not the error has even shrunk,
        # which is the damping a loop running on seconds-old reads needs
        # and a proportional term cannot supply -- the apron showed the
        # slalom the other shape makes.
        gs = state.ground_speed_kt or 0.0
        tail = False
        castor = False
        if state.on_ground:
            from .. import tailwheel
            tail = tailwheel.is_tailwheel(state)
            castor = tailwheel.is_castoring(state)
        # A weave is a slow-down first: ten knots of taxi cannot be
        # steered out of a six-degree-per-second swing, and adding power
        # back toward the cruise target only gives the next reversal
        # more energy.
        if state.on_ground and gs > 6.0 \
                and abs(rate) > 6.0 and abs(err) > 8.0:
            target_kt = min(target_kt, 4.0)
        # The ask also comes down with speed: fifteen degrees a second
        # is a brisk ramp turn, and the same rate at twenty-five knots
        # is the swerve a fast taxi is lost to.
        rate_cap = _yaw_rate_cap(gs)
        # The gain autoadjusts to the airframe *at this speed*: the
        # degrees a second one unit of pedal buys on this model, learned
        # in ``_pedal_check`` per groundspeed band -- authority grows
        # with the knots, so the apron's figure steered at rollout speed
        # is the weave, not the answer. Until a band has evidence the
        # nearest one -- or a middling guess -- corrects it.
        authority = self._yaw_authority_at(gs)
        limit = self._ground_steer_limit(state)
        # And the ask is capped at what this model's tiller still gives
        # at the limit the ground allows: a rate ordered past that
        # saturates the pedal, and the swing the rate term was meant to
        # brake arrives as the weave itself. A tenth of the travel stays
        # in hand as the counter-pedal that ends the turn.
        rate_cap = min(rate_cap, max(2.0, authority * limit * 0.9))
        # The led error is the one the next write answers to -- the
        # swing is braked before the line is crossed, not corrected
        # after.
        # The situation's press is on the ask before the cap: firmer
        # for the caller's demand and for a committed turnaround,
        # softer under a quarter of each, bounded so a firmer pass is
        # the same law leaning, not a different one -- and never past
        # the authority the cap was just cut to.
        want_rate = max(-rate_cap, min(
            rate_cap,
            self._steer_lead(err, rate) / 4.0
            * max(0.6, min(1.35, demand * (1.2 if commit else 1.0)))))
        # The pedal to make the rate, plus the correction for the rate
        # it differs by: the bare difference this used to be settled
        # halfway to the ask -- every corner twice as wide as its
        # speed made necessary. The full feed-forward -- demand twice
        # and damp once -- delivered the ask in the plant but wove in
        # the sim, whose reads arrive a beat late: half that kick still
        # kills the droop and keeps the extra gain under what the lag
        # can ring.
        rudder = max(-0.5, min(0.5,
                               (1.5 * want_rate - 0.5 * rate) / authority))
        # The same droop the roll's law gets a share against: an error
        # held by an authority figure that flatters the model is never
        # answered at the pedal it needs -- the lineup's centreline
        # approached for a minute at a constant few degrees off. A
        # bounded error share is the ask the estimate cannot divide
        # away -- but only where it agrees with the rate term: a nose
        # already swinging for the mark gets the counter-pedal it was
        # promised, not a lean the other way.
        if rudder * err > 0.0:
            rudder += max(-0.3, min(0.3, err * 0.015))
        rudder = max(-limit, min(limit, rudder))
        if state.on_ground and not castor \
                and not (brake_turn or commit or abs(err) >= 45.0):
            # Straight and gently curved taxiway does not take large
            # nosewheel angles. At ten knots 0.18 of the Baron's
            # forty-degree steering is already seven degrees at the
            # wheel; the old law could command twice that for a
            # twenty-degree correction and start the next overshoot.
            tracking_cap = max(
                0.08, min(limit, 0.07 + min(30.0, abs(err)) * 0.004))
            if gs > 8.0:
                tracking_cap = min(
                    tracking_cap, max(0.09, 0.24 - (gs - 8.0) * 0.008))
            rudder = max(-tracking_cap, min(tracking_cap, rudder))
        if abs(err) < 1.5 and abs(rate) < 1.0:
            # On the heading with no swing: a twitch of tiller is a
            # weave nobody feels, so nothing is written at all.
            rudder = 0.0
        # The wheel is walked to the answer, not thrown: a demand that
        # teleports between ticks is a kick the nose answers with a
        # swing of its own, in whichever sim -- the slew is the
        # continuous correction a hand makes, at the loop's own pace.
        prev_pedal = getattr(self, "_pedal_out", 0.0)
        step = rudder - prev_pedal
        step = max(-self._PEDAL_SLEW, min(self._PEDAL_SLEW, step))
        rudder = self._pedal_out = prev_pedal + step
        sent = rudder * self._pedal_dir()
        log.info(
            "steer: hdg %.0f want %.0f err %+.0f rate %+.1f "
            "pedal %+.2f gs %.1f->%.1f kt at %.5f %.5f thr %.0f",
            hdg, desired_hdg, err, rate, sent, gs, target_kt,
            getattr(state, "latitude", 0.0) or 0.0,
            getattr(state, "longitude", 0.0) or 0.0,
            getattr(state, "throttle_pct", 0.0) or 0.0)
        now = time.monotonic()
        self._pedal_check(rudder, rate, gs)
        d.rudder(sent)
        # The nosewheel follows the same demand where the model steers it
        # separately -- faded out with speed the way a real gear's tiller
        # is, so a full ask at forty knots reaches the tiller as a
        # fraction of itself while the rudder keeps the whole demand. A
        # taildragger on rudder alone and a tricycle gear on STEERING_SET
        # both get their turn.
        d.steering(sent * _nws_fade(gs))
        # Stick position for the wind, the way the AFH has it held on
        # the ground: aileron up on the windward wing in a quartering
        # headwind, everything diving away from a quartering tailwind,
        # and on a taildragger the elevator back so the tail stays down.
        # Taxi-speed only: above it the elevator and aileron belong to
        # the roll, not the wind.
        if state.on_ground and gs < 25.0:
            elevator, aileron = tailwheel.taxi_controls(
                getattr(state, "ambient_wind_direction", 0.0) or 0.0,
                getattr(state, "ambient_wind_velocity", 0.0) or 0.0,
                hdg, tail)
            d.elevator(elevator)
            d.aileron(aileron)
        # Taxi and rollout speeds are throttle jobs, not AP jobs -- and
        # progressive ones: the further below target, the more asked for,
        # and the lever is walked to the answer at a hand's rate rather
        # than put there. A lever write that jumps the travel is the
        # full-throttle departure from the stand this exists to prevent.
        still = self._ground_still(state, now)
        probe = 0 if still else self._stuck_probe(state, now)
        if still:
            want = 0.0
            if self._use_pids:
                # A wound-up integrator is why the next probe comes in
                # as a surge: drop it, so a chock leaving sees the
                # gentle pull-away, not the debt.
                self._pid("taxi").reset(0.0)
        elif self._use_pids:
            want = self._pid("taxi").update(target_kt - gs,
                                            measurement=gs)
            if gs > target_kt + 4.0 and state.on_ground:
                want = 0.0
                # Over target the throttle is off: keeping the integral
                # charged through it is how the speed-off swing starts.
                self._pid("taxi").reset(0.0)
        elif gs < target_kt - 2.0:
            want = max(0.05, min(0.6,
                                 0.10 + (target_kt - gs) * 0.01))
        elif gs > target_kt + 4.0 and state.on_ground:
            want = 0.0
        else:
            want = self._last_lever
        # Breakaway: from a standstill the lever gets the momentary push
        # that starts a loaded aeroplane rolling -- removed again by the
        # law once it is moving, the way the power comes back to idle as
        # soon as the wheels are turning.
        if state.on_ground and target_kt > 0.0 and gs < 2.0 \
                and want > 0.0:
            want = max(want, self._breakaway_lever())
        if probe:
            # A lever that already failed to move the aeroplane is not
            # asked again: each probe wants more, walked up faster than
            # a pull-away's hand would move it, so a heavy or a sticky
            # pair of tyres finds its breakaway rather than dying at the
            # same setting every half minute. Bogged on a corner or off
            # the pavement's edge the ask climbs to the stop itself --
            # a probe that never reaches full travel is a mired
            # aeroplane kept for ever.
            want = max(want, self._breakaway_lever() + 0.20 * (probe - 1))
            lever = self._slew_lever(min(want, 1.0), up_per_s=0.25,
                                     down_per_s=0.15)
        else:
            lever = self._slew_lever(max(0.0, min(0.6, want)),
                                     up_per_s=0.05, down_per_s=0.15)
        # What a pull-away took is what the next one starts with: the
        # lever under way while the wheels were still stopped is noted,
        # and the first pass over two knots writes it down -- bounded,
        # so a pull-away measured against a chock does not poison it.
        if state.on_ground and gs < 2.0 and target_kt > 0.0 \
                and lever > 0.05:
            self._broke_lever = lever
        elif gs >= 2.0 and getattr(self, "_broke_lever", None) is not None:
            took = self._broke_lever
            self._broke_lever = None
            adapt = getattr(self, "_adapt", None)
            if adapt is not None:
                adapt.learn("taxi.breakaway",
                            max(0.15, min(0.50, took)), blend=0.5)
        d.throttle(lever)
        if state.on_ground:
            # Fast is brought down on the brakes, not coasted out on the
            # idle: a squeeze that grows with how far over the speed is,
            # released again once it is not.
            brake = (min(0.5, 0.08 + (gs - target_kt) * 0.03)
                     if gs > target_kt + brake_margin_kt else 0.0)
            # The inside wheel joins the steering where it can help: a
            # taildragger at taxi speed, whose free-castering tail and
            # unblown rudder have nothing to give; a castoring-nosewheel
            # aeroplane, whose tiller never steers at all -- the Cirrus
            # POH's "minimum differential braking" is the whole answer
            # below rudder authority; and the apron pivot turn either
            # gear flies on the inside brake.
            if gs >= 1.0 \
                    and (tail or castor or self._turn_commit
                         or brake_turn):
                # The law's demand, not ``sent``: a model whose pedal
                # axis answers backwards still wants the inside wheel
                # for the turn it is making, not the wire's sign. And
                # the assist is a rolling turn's tool only: braked on
                # the inside wheel at a standstill it is not a pivot,
                # it is a chock -- the aeroplane sat on the runway's
                # edge for over a minute holding exactly the brake that
                # kept the nosewheel's turn from being made.
                add_left, add_right = tailwheel.brake_assist(rudder, gs)
                d.brakes_split(max(brake, add_left),
                               max(brake, add_right))
            else:
                d.brakes(brake)

    def _stop(self, state, target_kt: float = 0.0) -> None:
        """Roll out to a stop: idle, braking, controls centred."""
        d = self.director
        d.throttle(0.0)
        d.throttle_idle()
        # The next pull-away starts the slew from idle, not from
        # wherever the lever happened to be left, and a turnaround in
        # progress is forgotten so the next ask picks its own side.
        self._last_lever = 0.0
        self._turn_commit = 0
        gs = state.ground_speed_kt or 0.0
        if state.on_ground:
            # On the ground the autopilot is never the hand the FO
            # wants: a servo holding a stale heading bug steered every
            # rejected roll's nose left and walked the parked aeroplane
            # after it. Any held stop -- abort, await, hold short,
            # taxi brake -- releases it again, asked every tick because
            # a TO/GA press will put it right back.
            d.autopilot(False)
        if gs <= 1.0:
            d.brakes(0.0)
            d.parking_brake(True)
            d.rudder(0.0)
            d.steering(0.0)
            d.center_controls()
        elif state.on_ground:
            # The squeeze a pilot gives the pedals: enough to bring the
            # speed down, never a stomp that skids the nosewheel.
            d.brakes(min(0.5, 0.10 + gs * 0.02))
            d.rudder(0.0)
            d.steering(0.0)
            # A pull latched from a rotation or a flare is a nosewheel
            # held light through the stop: the stick goes neutral while
            # the wheels are still rolling.
            d.elevator(0.0)
            d.aileron(0.0)

