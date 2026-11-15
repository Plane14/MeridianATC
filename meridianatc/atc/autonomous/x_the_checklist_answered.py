from __future__ import annotations

from ._shared import (
    holds,
    ip,
    log,
    time,
)


class _The_checklist_answered:
    # The checklist vars whose control is a toggle rather than a set:
    # asking twice flips the switch back off, so each is asked once.
    _TOGGLED_ONCE = frozenset({"seatbelt_sign"})

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
        check = getattr(item, "check", None)
        if check is None:
            return False
        runner = self.crew.checklists
        try:
            expected = runner._expected(check)
        except Exception:
            expected = check.value
        actual = getattr(state, check.var, None)
        if holds(check, actual, expected):
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
        # (the masters). Parked is the only place they are asked for off.
        parked = getattr(state, "on_ground", True) \
            and (state.ground_speed_kt or 0.0) <= 5.0
        table = {
            "parking_brake": lambda: d.parking_brake(want_on),
            # Gear out goes through the same gear-speed gate the flying
            # uses: a checklist item does not put the wheels out past
            # Vlo. Coming up is allowed airborne only -- never a retraction
            # while the wheels still carry the aeroplane.
            "gear_down": lambda: (
                (want_on and self._gear_allowed(state))
                or (not want_on and not getattr(state, "on_ground", True))
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
            # The checklist's figure is a percent; the lever variable is
            # a fraction ("Percent over 100"), so a bare write asked for
            # a mixture travel it does not have. A source without named
            # units takes the percent it always did.
            "mixture_pct": lambda: (
                d.write("GENERAL ENG MIXTURE LEVER POSITION:1",
                        float(expected) / 100.0,
                        unit="Percent over 100")
                or d.write("GENERAL ENG MIXTURE LEVER POSITION:1",
                           float(expected))),
            "transponder_code": lambda: d.transponder(str(expected)),
            "kohlsman_hpa": lambda: d.kohlsman(
                self._altimeter_hpa(expected)),
            # The check asks for a detent, not a travel: the handle index
            # where the sim takes it, the fraction where it does not --
            # and an extension only within the flap speed, the gate the
            # flying path uses too.
            "flaps_index": lambda: self._checklist_flaps(state, expected),
            "spoilers": lambda: d.spoilers(min(1.0, float(expected))),
        }
        action = table.get(var)
        if action is None:
            return False
        try:
            action()
        except Exception:
            log.debug("checklist switch %s failed", var, exc_info=True)
            return False
        return True

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
        # Takeoff flaps: the profile's detent where the type declares one,
        # else the first detent -- whatever the lever was left at, which
        # is the point of setting it.
        takeoff_flaps = self.performance.takeoff_flaps or 0.25
        d.flaps(takeoff_flaps)
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
        passed. The horizon is the loop's own measured period -- a
        five-second tick needs the braking started five seconds out.
        Without the rate term a saturated law under lag bang-bangs --
        or with the correction arriving late enough, orbits the apron.
        """
        horizon = max(2.0, 1.5 * getattr(self, "_steer_dt", 2.0))
        return err - rate * horizon

    def _pedal_dir(self) -> float:
        """The sign this model's steering axis actually answers in."""
        return -1.0 if getattr(self, "_pedal_flip", False) else 1.0

    def _pedal_check(self, demand: float, rate: float,
                     gs: float) -> None:
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
        # The magnitude carries the information -- the sign can be
        # backwards while the axis is still being voted on -- and the
        # sample must not wait for the streak the vote does: a hot
        # axis alternates demands fast enough that no streak forms,
        # which is exactly when the estimate matters most. The
        # aeroplane must be rolling too: a tug or the wind can put
        # rate on a parked airframe that the pedal never made.
        if abs(prev) >= 0.15 and abs(rate) >= 3.0 and gs >= 1.0:
            sample = abs(rate) / abs(prev)
            if 5.0 <= sample <= 150.0:
                learned = getattr(self, "_yaw_authority", 0.0)
                self._yaw_authority = sample if learned <= 0.0 \
                    else 0.6 * learned + 0.4 * sample
                adapt = getattr(self, "_adapt", None)
                if adapt is not None:
                    adapt.learn("steer.authority",
                                self._yaw_authority, blend=0.4)
        if abs(prev) < 0.3 or abs(rate) < 3.0 or gs < 1.0 or streak < 3:
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
        self._pedal_votes = votes
        if adapt is not None:
            adapt.learn_votes("steer", votes)

    def _breakaway_lever(self) -> float:
        """The momentary lever that starts the aeroplane rolling.

        A loaded transport wants a third of the travel to break away
        from the stand; a light single wants a fifth. Asked for only
        while the wheels are stopped -- once rolling, the speed law
        owns the lever and the power comes back to idle, the way the
        surge ends the moment the aeroplane is moving.
        """
        cruise = self.performance.cruise_kt
        if cruise >= 230.0:
            return 0.38
        if cruise >= 140.0:
            return 0.30
        return 0.22

    def _set_takeoff_power(self, state=None) -> None:
        """Takeoff thrust: advanced the way a hand moves the levers --
        briskly but progressively, not slammed to the stop -- and TO/GA
        on the autothrottle where the model has one. The one-shot events
        re-arm on each roll and each go-around; the full-power event
        confirms once the levers have walked to the stop. The lever
        itself stays under the engine watch's ceiling: the takeoff
        rating is a limit, not a target."""
        d = self.director
        cap = (self._engine_limit_cap(state, takeoff=True)
               if state is not None else 1.0)
        lever = self._slew_lever(min(1.0, cap))
        d.throttle(lever)
        if lever >= 1.0:
            d.takeoff_power()
        d.toga(True)

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
               max_aileron: float = 0.35) -> None:
        """Hand-fly a heading and a speed on the ground or in the air.

        The rudder (which is the nosewheel on the ground) takes the
        heading error; the throttle takes the speed. Used for taxi,
        line-up, the takeoff roll's directional control and the rollout --
        the places an autopilot's heading mode does not reach.
        """
        d = self.director
        d.parking_brake(False)
        hdg = state.heading_true or 0.0
        # The pursuit point can teleport between legs of the route --
        # where two taxiway edges sit close, the nearest-point pick
        # ping-pongs and the demanded heading jumps tens of degrees a
        # tick, which no loop can converge on. The demand is eased
        # toward each new answer along the arc, so a leg change reads
        # as a brisk turn order rather than a teleport.
        prev_want = getattr(self, "_steer_want", None)
        if prev_want is not None:
            jump = (desired_hdg - prev_want + 540.0) % 360.0 - 180.0
            desired_hdg = (prev_want + 0.45 * jump) % 360.0
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
        want_rate = max(-15.0, min(15.0, err / 4.0))
        # The gain autoadjusts to the airframe: ``_yaw_authority`` is
        # the degrees a second one unit of pedal buys on this model,
        # learned in ``_pedal_check``; until it has evidence the demand
        # assumes a middling answer and the estimate corrects it.
        authority = getattr(self, "_yaw_authority", 0.0)
        if authority < 5.0:
            authority = 30.0
        rudder = max(-0.5, min(0.5, (want_rate - rate) / authority))
        limit = self._ground_steer_limit(state)
        rudder = max(-limit, min(limit, rudder))
        gs = state.ground_speed_kt or 0.0
        sent = rudder * self._pedal_dir()
        log.info(
            "steer: hdg %.0f want %.0f err %+.0f rate %+.1f "
            "pedal %+.2f gs %.1f->%.1f kt",
            hdg, desired_hdg, err, rate, sent, gs, target_kt)
        now = time.monotonic()
        self._pedal_check(rudder, rate, gs)
        d.rudder(sent)
        # The nosewheel follows the same demand where the model steers it
        # separately: a taildragger on rudder alone and a tricycle gear on
        # STEERING_SET both get their turn.
        d.steering(sent)
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
            # same setting every half minute. Still capped -- the probe
            # is a probe, not a held takeoff rating.
            want = max(want, self._breakaway_lever() + 0.12 * (probe - 1))
            lever = self._slew_lever(min(want, 0.75), up_per_s=0.15,
                                     down_per_s=0.15)
        else:
            lever = self._slew_lever(max(0.0, min(0.6, want)),
                                     up_per_s=0.05, down_per_s=0.15)
        d.throttle(lever)
        if state.on_ground:
            # Fast is brought down on the brakes, not coasted out on the
            # idle: a squeeze that grows with how far over the speed is,
            # released again once it is not.
            if gs > target_kt + 4.0:
                d.brakes(min(0.5, 0.08 + (gs - target_kt) * 0.03))
            else:
                d.brakes(0.0)

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

