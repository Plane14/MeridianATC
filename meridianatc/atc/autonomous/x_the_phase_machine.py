from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    Phase,
    auto_nav,
    capabilities,
    fly,
    project,
)


class _The_phase_machine:
    # -- the phase machine ----------------------------------------------------

    def _phase_preflight(self, state, now: float) -> None:
        """Preflight: aircraft configured, checks done, clearance next."""
        d = self.director
        sub = self.state.sub or "systems"

        if sub == "systems":
            self._ensure_systems_on(state)
            # The outside checks a FO runs before the before-start list:
            # transponder on with the assigned squawk, the altimeter set,
            # the lights that belong on for the phase.
            if getattr(state, "transponder_state", -1) not in (3, 4):
                d.transponder(self.session.squawk or "1200")
                # The code is not the box: a squawk written into a
                # standby transponder still answers nothing, so the mode
                # goes to altitude reporting with it.
                d.write("TRANSPONDER STATE:1", 4.0)
            if getattr(state, "kohlsman_hpa", 0.0) > 0 and self.session.altimeter:
                d.kohlsman(self._altimeter_hpa(self.session.altimeter))
            d.beacon(True)
            d.nav_lights(True)
            # The seatbelt event is a toggle, not a set: asking for "on"
            # where it is already on would switch it off.
            if not getattr(state, "seatbelt_sign", False):
                d.seatbelts(True)
            d.parking_brake(True)
            self._goto_sub("checklist")

        elif sub == "checklist":
            self._run_checklist_for_phase("before_start")
            self._goto_sub("clearance")

        elif sub == "clearance":
            if not self.session.destination:
                self._transition(AutonomousPhase.PUSHBACK)
                return
            # What the flight is flown under is decided here, the way a
            # crew files it: the aircraft's operation and the weather
            # reported at both ends. A transport files IFR whatever the
            # sky; a light aeroplane goes VFR only where VMC lets it.
            self.session.ifr = self._ifr_required(state)
            flight_cfg = getattr(getattr(self, "config", None),
                                 "flight", None)
            if flight_cfg is not None:
                flight_cfg.ifr = self.session.ifr
            if self.session.ifr or self._clearance_position() is not None:
                # IFR always has a clearance to collect -- even where the
                # field cannot issue one, the ask is the filing. A VFR
                # departure only needs the release where somebody can
                # give one; on a broadcast field there is nothing to
                # collect before the push.
                self._transition(AutonomousPhase.REQUEST_CLEARANCE)
            else:
                self._transition(AutonomousPhase.PUSHBACK)

    def _phase_request_clearance(self, state, now: float) -> None:
        """Request the clearance or the release the rules need, and hold
        it read back before the push."""
        # An issued clearance is not a held one: every clearance here is
        # written down as an instruction owed a readback, and
        # ``clearance_read_back`` is set by the readback the controller
        # accepts -- not by the transmission that asked. Moving on the
        # issued flag is how a flight pushed back still owing delivery
        # its readback. A VFR release is held to the same standard: it is
        # remembered as the session's last instruction and its read_back
        # is the acceptance.
        if self._clearance_done():
            self._transition(AutonomousPhase.PUSHBACK)
            return
        # A VFR release refused below minima is the weather changing the
        # flight's mind for it: it re-files IFR, exactly what a pilot
        # does who cannot have VFR -- where IFR is allowed at all.
        last = getattr(self.session, "last_instruction", None)
        if not self.session.ifr and self._request_ifr \
                and getattr(last, "kind", "") == "below_minima":
            self.session.ifr = True
            self.state.has_ifr_clearance = False
            self.state.clearance_asked_at = 0.0
            self._emit("info", "VFR refused below minima; filing IFR",
                       "autonomous.clearance.re_filed")
        # The radio is what the ask goes out on: a flight that reached
        # this phase still dark has nothing to call on, and the clock
        # below must not punish silence it could not help.
        self._ensure_systems_on(state)
        # A call the first officer could not make is not a call made: the
        # frequency may be busy or a readback may be owed first, so the ask
        # is retried until one goes out -- and again after that while the
        # answer stays away, the way a pilot repeats a call nobody heard.
        if self._radio_quiet() and (
                not self.state.has_ifr_clearance
                or now - self.state.last_radio_at >= 15.0
        ) and self._request_clearance_via_fo():
            self.state.has_ifr_clearance = True
            if not self.state.clearance_asked_at:
                self.state.clearance_asked_at = now
        if self._clearance_position() is None:
            # Nobody here issues anything -- a broadcast frequency or an
            # advisory service: the call is the announcement, and after a
            # moment for it on the air the departure is the flight's own
            # under whatever rules it is filed. A call that never goes
            # out at all is still bounded by the phase clock.
            if (self.state.last_radio_at
                    and now - self.state.last_radio_at >= 12.0) \
                    or now - self.state.phase_entered_at > self.CLEARANCE_TIMEOUT:
                self._emit("info",
                           "No clearance delivery on this field; "
                           "departing after the announcement",
                           "autonomous.clearance.broadcast")
                self._transition(AutonomousPhase.PUSHBACK)
            return
        if self.state.clearance_asked_at \
                and now - self.state.clearance_asked_at > self.CLEARANCE_TIMEOUT:
            # Somebody who issues is working the field: the wait for the
            # paper is real, and a flight that never gets it is not
            # quietly re-filed VFR to leave on time. The clock is the
            # first ask's and not the phase's -- a radio that took a
            # minute to come alive has not been refused anything yet.
            rules = "IFR clearance" if self.session.ifr else "VFR release"
            self._set_error(f"No {rules} was issued")

    def _phase_pushback(self, state, now: float) -> None:
        """Engine start, pushback, brakes back on."""
        d = self.director
        sub = self.state.sub or "engines"

        if sub == "engines":
            if not state.engines_running:
                if self.state.engines_started_at == 0.0:
                    self._start_engines(state)
                    self.state.engines_started_at = now
                elif now - self.state.engines_started_at > 30.0 \
                        and not self.state.start_retried:
                    self._start_engines(state)
                    self.state.start_retried = True
                elif now - self.state.engines_started_at > self.ENGINE_START_TIMEOUT:
                    self._set_error("Engine start timed out")
                    return
            else:
                self._goto_sub("after_start")

        elif sub == "after_start":
            self._run_checklist_for_phase("after_start")
            d.parking_brake(True)  # set for the push
            self._goto_sub("request")

        elif sub == "request":
            # The stand is let go before the aeroplane asks to be: a
            # jetway or stairs still attached is asked away once and
            # waited on a moment, then taxied out from under anyway --
            # a stuck GSX state cannot hold a flight for ever.
            clear, why = self._stand_clear()
            if not clear and self._sub_age() < 45.0:
                self._free_the_stand(why)
                return
            self.state.stand_hold_said = False
            # A stand the simulator says has no pushback will never send
            # a tug: rather than ask into silence, an aeroplane that can
            # drive out just does. ``pushback_available`` unread -- None
            # -- is not a refusal.
            if getattr(state, "pushback_available", None) is False \
                    and self._pushback_skippable():
                self._emit("info", "No pushback at this stand; "
                                   "taxiing out",
                           "autonomous.pushback.skipped")
                self._goto_sub("taxi")
                self._transition(AutonomousPhase.TAXI_OUT)
                return
            # Where somebody works the surface the push is theirs to
            # approve: the FO asks for it, and the tug the approval sends
            # is what the wait below watches for. The call only counts
            # once it has gone out -- a busy frequency or an owed
            # readback defers it -- and a frequency that never takes it
            # falls back to the simulator's own tug, as before.
            self.state.push_radio = self._pushback_by_radio()
            if self.state.push_radio:
                if self._radio_quiet() and self._fo_say("pushback"):
                    self.state.push_requested = True
                    self._goto_sub("wait")
                elif self._sub_age() > 60.0:
                    self.state.push_radio = False
                    self._request_pushback()
                    self.state.push_requested = True
                    self._goto_sub("wait")
            else:
                self._request_pushback()
                self.state.push_requested = True
                self._goto_sub("wait")

        elif sub == "wait":
            # The aeroplane moved under the tug, then stopped: the push is
            # done. The parking brake goes on before anything moves under
            # its own power -- but not while the tug or the stand is still
            # on the aeroplane; a push paused for the flight deck's
            # "ready?" is a push that has not finished.
            if self.state.moved_since_push and self.state.still_since \
                    and now - self.state.still_since > 5.0:
                clear, _why = self._stand_clear()
                if clear or now - self.state.still_since > 60.0:
                    d.parking_brake(True)
                    self._goto_sub("taxi")
                    self._transition(AutonomousPhase.TAXI_OUT)
                    return
                self._ground_confirm_push()
            if not self.state.moved_since_push:
                # Whether the simulator has a live request: PUSHBACK WAIT
                # while the tug is on its way, PUSHBACK ATTACHED once it
                # is on the nosewheel. The request event is a toggle, so
                # asking again while either is true cancels the push that
                # is coming -- the retry is only for a request that never
                # landed at all.
                engaged = bool(getattr(state, "pushback_wait", False)
                               or getattr(state, "pushback_attached",
                                          False))
                if engaged:
                    # The tug is coming or on the nosewheel -- including
                    # the one a controller's approval sent: the brake
                    # after_start set has to be off for it to push. And
                    # where GSX asks the flight deck whether it is ready,
                    # the FO's answer is yes: the brake is off for it.
                    d.parking_brake(False)
                    if self._sub_age() > 15.0:
                        self._ground_confirm_push()
                else:
                    # A call on the frequency is repeated with patience,
                    # not streamed -- and one never answered falls back
                    # to the simulator's own tug before it is a failure.
                    retry_after = 40.0 if self.state.push_radio else 120.0
                    if self._sub_age() > retry_after \
                            and self.state.push_retries < 3:
                        if self.state.push_radio:
                            self._fo_say("pushback")
                        else:
                            self._request_pushback()
                        self.state.push_retries += 1
                        self.state.sub_entered_at = now
                    elif self._sub_age() > retry_after:
                        if self.state.push_radio:
                            # The calls went unanswered: press for the
                            # tug directly before calling it failed --
                            # its own retries, not the radio's spent ones.
                            self.state.push_radio = False
                            self.state.push_retries = 0
                            self._request_pushback()
                            self.state.sub_entered_at = now
                        elif self._pushback_skippable():
                            self._emit(
                                "info",
                                "No pushback answered at this stand; "
                                "taxiing out",
                                "autonomous.pushback.skipped")
                            self._goto_sub("taxi")
                            self._transition(AutonomousPhase.TAXI_OUT)
                            return
                        else:
                            # Asked for repeatedly and nothing ever
                            # engaged. A light aeroplane on a ramp simply
                            # drives out; what cannot is still an error
                            # rather than a push through the terminal.
                            self._set_error("Pushback did not move the "
                                            "aircraft")
                            return
            if now - self.state.phase_entered_at > self.PUSHBACK_TIMEOUT:
                self._set_error("Pushback timeout")

    def _phase_taxi_out(self, state, now: float) -> None:
        """Taxi to the departure runway along the planned route."""
        self._run_checklist_for_phase("taxi")
        if self.session.phase == Phase.HOLDING_SHORT:
            self._transition(AutonomousPhase.HOLDING_SHORT)
            return

        sub = self.state.sub or "taxi"
        if sub == "taxi":
            # The wheels turn when the instruction to move has been
            # issued -- or where nobody issues one, once the FO's own
            # announcement has had its moment on the frequency.
            if self._cleared_to_taxi():
                self._taxi_along_path(state, outbound=True)
            else:
                self._stop(state)
        elif sub == "hold_point":
            self._stop(state)
            if (state.ground_speed_kt or 0.0) <= 1.0:
                if self._held_at_split:
                    # Held at the taxiway a cut clearance was split at,
                    # not the runway's holding point: the rest of the
                    # route resumes the taxi where HOLDING_SHORT would
                    # sit waiting for a takeoff clearance instead. The
                    # radios keep asking for it while it is owed.
                    if self.session.taxi_split_at is not None:
                        return
                    self._held_at_split = False
                    self._goto_sub("taxi")
                    return
                self._goto_sub("taxi")
                self._transition(AutonomousPhase.HOLDING_SHORT)
                return

        if now - self.state.phase_entered_at > self.TAXI_TIMEOUT:
            self._set_error("Taxi timeout")

    def _phase_holding_short(self, state, now: float) -> None:
        """Holding short of runway, run before-takeoff checklist."""
        if (state.ground_speed_kt or 0.0) > 1.0:
            self._stop(state)
            return
        self._run_checklist_for_phase("before_takeoff")

        # "Line up and wait" puts the aeroplane on the runway; the takeoff
        # clearance is what starts the roll. The aeroplane only goes if it
        # is light enough and the pavement is long enough for it.
        if self.session.lined_up or self.session.cleared_takeoff \
                or self.session.phase in (Phase.LINED_UP, Phase.TAKEOFF):
            if not self._departure_possible(state):
                return
            self._transition(AutonomousPhase.LINE_UP)
            return

        # On a broadcast frequency nobody answers: the first officer's
        # "ready" call was the announcement, and the pilot approves the
        # departure themselves once it has had time to go out and any
        # final to be looked at. ``cleared_takeoff`` is the state
        # machine's name for "the runway may be taken", and on CTAF the
        # person granting that is the one flying.
        station = getattr(self.session, "current_station", None)
        if getattr(station, "position", "") in (
                "CTAF", "UNICOM", "INFO", "AFIS", "RDO") \
                and self.state.last_radio_at >= self.state.phase_entered_at \
                and now - self.state.phase_entered_at > 20.0:
            if not self._departure_possible(state):
                return
            self.session.cleared_takeoff = True
            self.session.cleared_takeoff_at = now
            self._emit("info",
                       "No tower on this frequency; departing after "
                       "self-announcement", "autonomous.depart.broadcast")
            self._transition(AutonomousPhase.LINE_UP)

        # The wait for the clearance is bounded like the taxi that led to
        # it: a tower that never says "cleared for takeoff" is not waited
        # on forever at the holding point.
        if now - self.state.phase_entered_at > self.TAXI_TIMEOUT:
            self._set_error("Held short: no takeoff clearance")

    def _departure_possible(self, state) -> bool:
        """Whether the aeroplane may be taken onto the runway at all.

        The weight the sim reports against the gross the model declares,
        and the pavement against the takeoff run the model suggests. A
        model that publishes neither is taken on trust; one that does is
        held to it -- an overweight aeroplane does not get flown off.
        """
        mtow = float(getattr(state, "max_gross_weight_lb", 0.0) or 0.0)
        weight = float(getattr(state, "total_weight_lb", 0.0) or 0.0)
        if mtow > 0.0 and weight > mtow * 1.02:
            self._set_error(f"Overweight: {weight:.0f} lb for a "
                            f"{mtow:.0f} lb maximum takeoff weight")
            return False
        if self.state.runway_heading <= 0:
            self._find_runway(state)
        need = float(getattr(state, "suggested_runway_takeoff_ft", 0.0)
                     or 0.0)
        have = float(self.state.runway_length_ft or 0.0)
        if need > 0.0 and have > 0.0 and have < need * 0.9 \
                and not self.state.mtow_warned:
            # Warned rather than refused: the model's suggestion is a
            # book figure, not a measured run, and the runway data can
            # equally be the one that is off.
            self.state.mtow_warned = True
            self._emit("info",
                       f"Runway {have:.0f} ft for a suggested {need:.0f} "
                       "ft takeoff run", "autonomous.limits.runway")
        if self._weather_reactions and not self._departure_wind_ok(state):
            return False
        return self._departure_fuel_ok(state)

    def _departure_fuel_ok(self, state) -> bool:
        """Whether the tanks cover the leg with its diversion and reserve.

        What a crew signs for: enough to reach the destination, divert to
        the alternate and land with the reserve. The diversion figure is
        the aim -- under it is said once and flown with care; under the
        leg with the reserve alone is not a departure.
        """
        needed = self._fuel_needed_gal(state, estimate=True)
        if needed is None:
            return True
        fuel_gal = float(getattr(state, "fuel_total_gal", 0.0) or 0.0)
        if fuel_gal <= 0.0:
            return True
        if fuel_gal < needed["leg_gal"] + needed["reserve_gal"]:
            self._set_error(
                f"Not enough fuel: {fuel_gal:.0f} gal for a leg needing "
                f"{needed['total_gal']:.0f} with the diversion")
            return False
        if fuel_gal < needed["total_gal"] \
                and not self.state.depart_fuel_warned:
            self.state.depart_fuel_warned = True
            alt = needed.get("alternate")
            self._emit("info",
                       f"Departing short of the diversion figure: "
                       f"{fuel_gal:.0f} gal, {needed['total_gal']:.0f} "
                       f"planned via {alt.ident if alt else 'no alternate'}",
                       "autonomous.fuel.short")
        return True

    def _phase_line_up(self, state, now: float) -> None:
        """Line up on runway and prepare for takeoff."""
        d = self.director
        sub = self.state.sub or "align"

        if not state.on_ground:
            self._transition(AutonomousPhase.TAKEOFF)
            return

        if sub == "align":
            self._find_runway(state)
            if self.state.runway_heading <= 0:
                # No runway geometry: hold where we are and wait for the
                # clearance rather than steering blind.
                self._stop(state)
                if self.session.cleared_takeoff:
                    self._transition(AutonomousPhase.TAKEOFF)
                return
            # Taxi onto the centreline and align with the runway heading.
            # ``align`` says how far off the extended centreline the
            # aeroplane is and the heading that puts it back on; steering
            # at that heading is the FO's interception, then the runway
            # heading itself once the wings are level on the numbers.
            err = auto_nav.align(state.latitude, state.longitude,
                                 self.state.runway_lat, self.state.runway_lon,
                                 self.state.runway_heading)
            hdg_err = abs((self.state.runway_heading
                            - (state.heading_true or 0.0) + 540.0)
                           % 360.0 - 180.0)
            steer_hdg = (self.state.runway_heading
                         + err["correction_deg"]) % 360.0
            if abs(err["offset_m"]) < 60.0:
                steer_hdg = self.state.runway_heading
            d.parking_brake(False)
            self._steer(state, steer_hdg, self.performance.taxi_kt)
            if abs(err["offset_m"]) < 25.0 and hdg_err < 8.0:
                self.state.aligned = True
                self._goto_sub("set")
                self._emit("info", "Lined up on runway",
                           "autonomous.lined_up")
            elif self._sub_age() > 120.0:
                # Could not line up: hold short rather than roll onto the
                # runway unaligned.
                self._goto_sub("taxi")
                self._transition(AutonomousPhase.HOLDING_SHORT)
                return

        elif sub == "set":
            # Aligned: brakes on, checks complete, wait for the roll.
            d.parking_brake(True)
            self._stop(state)
            self._configure_for_takeoff(state)
            if self.session.cleared_takeoff or self.session.phase == Phase.TAKEOFF:
                self._transition(AutonomousPhase.TAKEOFF)
            elif self._sub_age() > 300.0:
                self._set_error("Lined up but never cleared for takeoff")

    def _phase_takeoff(self, state, now: float) -> None:
        """Takeoff roll, rotation, initial climb."""
        d = self.director
        sub = self.state.sub or "wait_clearance"
        self._compute_speeds(state)

        if sub == "wait_clearance":
            if not state.on_ground:
                self._goto_sub("initial_climb")
                return
            d.parking_brake(True)
            self._stop(state)
            if self.session.cleared_takeoff:
                self.state.rejected = False
                # The power events are one-shot: a new roll is a new
                # press of TO/GA and THROTTLE_FULL.
                self.director.forget("takeoff_power", "toga")
                self._goto_sub("roll")
            elif self._sub_age() > 300.0:
                self._set_error("Takeoff clearance never came")
                return
            return

        if sub == "roll":
            if not state.on_ground:
                self.state.airborne_at = now
                self._goto_sub("rotate")
                return
            # Brakes off, takeoff power, rudder for direction -- the roll.
            d.parking_brake(False)
            # Rejected takeoff: the roll is not accelerating. Below 60% of
            # rotation speed two thirds of the way through the expected
            # roll is an abort, not a rotate.
            vr = self.state.vr_kt or self.performance.rotate_kt
            elapsed = now - self.state.sub_entered_at
            expected = max(10.0, vr / max(self.performance.takeoff_kt_s, 0.5))
            ias = state.indicated_airspeed_kt or 0.0
            if ias < vr * 0.6 and elapsed > expected * 0.66:
                self.state.rejected = True
                self._emit("error", "Rejected takeoff: insufficient acceleration",
                           "autonomous.rejected_takeoff")
                self._goto_sub("abort")
                return
            self._set_takeoff_power(state)
            # Directional control: the nosewheel does the early work, the
            # rudder takes over as the airflow comes alive. Both get the
            # same demand; a model that only honours one still tracks.
            # The demand is kept small: a hard pedal at eighty knots is
            # the swerve a takeoff roll is lost to, not saved by. The
            # band stays proportional out to a real excursion -- a law
            # saturated inside a few degrees under a tick or two of lag
            # is the same weave the apron showed, at eighty knots.
            err = ((self.state.runway_heading - (state.heading_true or 0.0)
                    + 540.0) % 360.0 - 180.0)
            keep = self._slew_rudder(
                0.0 if abs(err) < 2.0
                else max(-0.4, min(0.4, err / 36.0)))
            d.rudder(keep)
            d.steering(keep if (state.indicated_airspeed_kt or 0.0) < 35.0
                       else 0.0)
            rotate_at = (self.state.vr_kt
                         or self.state.min_rotation_kt
                         or self.performance.rotate_kt)
            self.state.rotate_speed_kt = rotate_at
            v1 = self.state.v1_kt or (rotate_at - 10.0)
            if self._engines_out(state) and ias < v1:
                # An engine that quits below V1 is a stop, not a
                # continued takeoff; past it the aeroplane flies.
                self.state.rejected = True
                self._emit("error",
                           "Rejected takeoff: engine failure below V1",
                           "autonomous.rejected_engine")
                self._goto_sub("abort")
                return
            if ias >= rotate_at:
                self._goto_sub("rotate")
                self.state.rotate_at = now
            elif self._sub_age() > self.TAKEOFF_TIMEOUT:
                self._set_error("Takeoff roll timed out")
            return

        if sub == "rotate":
            if not state.on_ground:
                self._goto_sub("initial_climb")
                return
            # Pull to the rotation attitude and hold it until the wheels
            # are up. ELEVATOR_SET is proportional, so the FO can fly the
            # rotation the way a pilot does: a firm pull, then hands off.
            elapsed = now - self.state.rotate_at
            vs_now = float(state.vertical_speed_fpm or 0.0)
            if self._use_pids:
                # The loop closes on the rate coming alive -- the cue a
                # pilot's hand works off -- asking for a positive rate and
                # damping the answer against the one the model reports.
                pull = self._pid("rotation").update(350.0 - vs_now,
                                                    measurement=vs_now)
            else:
                # The pull is the class's too: a light single rotates with
                # less back pressure than a loaded airliner.
                pull_scale = (1.0
                              if self.performance.turn_rate < fly.RATE_LIGHT
                              else 0.8)
                pull = (0.40 if elapsed < 2.5 else 0.28) * pull_scale
                # A pilot stops pulling as the aeroplane comes unstuck: the
                # vertical speed coming alive is the cue, the way a hand
                # comes off the yoke once the nose stops needing help.
                if vs_now > 300.0:
                    pull = 0.10 * pull_scale
            # The attitude, not just the rate: at the rotation attitude
            # the pull stops, the way a pilot's hand comes off once the
            # nose sits at the mark. A model that never publishes pitch
            # answers zero and the rate cue above carries it.
            pitch = float(getattr(state, "pitch_deg", 0.0) or 0.0)
            if pitch >= self._pitch_target_deg():
                pull = min(pull, 0.05)
            # The load watch's ceiling: a rotation that would pull past
            # the limit pulls to it instead.
            d.elevator(self._g_pull_cap(state, pull))
            d.takeoff_power()
            if elapsed > 8.0:
                # Rotated and still on the ground: the model is not
                # flying itself off; lower the nose and reject rather
                # than porpoise down the runway.
                d.elevator(0.0)
                self.state.rejected = True
                self._goto_sub("abort")
            return

        if sub == "abort":
            # Rejected takeoff: idle, maximum braking, off the runway when
            # able, and the flight is over -- a rejected takeoff is not a
            # flight to continue.
            d.throttle_cut()
            d.throttle_idle()
            d.parking_brake(True)
            self._emit("info", "Rejected takeoff complete",
                       "autonomous.rejected_done")
            self._transition(AutonomousPhase.SHUTDOWN)
            self._goto_sub("secure")
            return

        if sub == "initial_climb":
            # Positive rate: flaps and gear come up on schedule, the climb
            # speed is reached, then the AP takes the leg.
            agl = state.altitude_agl_ft or 0.0
            ias = state.indicated_airspeed_kt or 0.0
            elapsed = now - (self.state.airborne_at or now)
            # Flaps up once safely airborne and fast -- except through a
            # windshear escape, where the book's rule is to hold the
            # configuration until clear of the shear.
            shear = (self.state.windshear_since
                     and now - self.state.windshear_since < 15.0)
            flap_up_kt = (self.state.v2_kt or self.state.vr_kt or 0.0) + 20.0
            if state.flaps_index and ias > flap_up_kt and not shear:
                d.flaps(0.0)
            # Gear up after a positive rate, where the model has one.
            # AGL alone is not the rate: an aeroplane settling back is
            # kept on its wheels until the VSI agrees it is climbing.
            if state.gear_down and agl > 200.0 and not shear \
                    and (state.vertical_speed_fpm or 0.0) > 0.0:
                d.gear(False)
            # Checklist and AP engagement once stable.
            if agl > 400.0 or elapsed > 25.0:
                self._run_checklist_for_phase("after_takeoff")
                self._engage_climb(state)
                self._transition(AutonomousPhase.CLIMB)
            return

    def _phase_climb(self, state, now: float) -> None:
        """Climb to the assigned or planned level."""
        target = self._altitude_target(state)
        alt = state.altitude_ft or 0.0
        if not target:
            # Engaged in the air with no level anywhere: a crew levels
            # where it is, it does not climb at a zero bug for an hour.
            target = alt
        # A clearance below is answered by coming down, not by climbing
        # away until the clock runs out.
        vs = self.performance.climb_fpm
        if target and alt > target + 200.0:
            vs = -self.performance.descend_fpm
        assigned_hdg = self.session.assigned_heading
        sid = self._sid_nav(state)
        if self.state.emergency_descent:
            # An emergency declared on the way up is a descent, not a
            # climb against a lowered target.
            self._transition(AutonomousPhase.DESCENT)
            return
        if sid is not None:
            # A filed/cleared SID owns the lateral path until its last
            # leg; the assigned heading still wins where ATC gave one.
            self._fly_procedure(state, sid)
        elif assigned_hdg is not None:
            self._apply_flight_director(state, altitude=target,
                                        vs_fpm=vs,
                                        heading=assigned_hdg,
                                        speed_kt=self._speed_target(state))
        else:
            self._apply_flight_director(state, altitude=target,
                                        vs_fpm=vs,
                                        speed_kt=self._speed_target(state))
            # No SID to walk and no heading assigned is not "wings
            # free": NAV flies the filed route until ATC says otherwise.
            self.director.nav(True)
            self.state.ap_nav_mode = True

        # Level off at the top of the climb.
        if target and abs((state.altitude_ft or 0.0) - target) < 200 \
                and abs(state.vertical_speed_fpm or 0.0) < 200:
            self.state.top_of_climb = True
            self.director.alt_hold(True)
            self.director.vs_hold(False)
            self._level_off_at_cruise(state)
            self._transition(AutonomousPhase.CRUISE)
            return

        if self.session.phase == Phase.DESCENT:
            self._transition(AutonomousPhase.DESCENT)
        elif self.session.phase == Phase.APPROACH:
            self._transition(AutonomousPhase.APPROACH)
        elif now - self.state.phase_entered_at > self.CLIMB_TIMEOUT:
            self._set_error("Climb timed out")

    def _phase_cruise(self, state, now: float) -> None:
        """Cruise: NAV on the route, fuel watched, clearances followed."""
        self._run_checklist_for_phase("cruise")
        sid = self._sid_nav(state)
        if self.state.emergency_descent:
            # An emergency descent does not wait for the plan's top of
            # descent: the boards out, the rate doubled, straight down
            # toward the divert's height. NAV flies the replanned route --
            # the leg now goes where the emergency is being taken.
            self.director.spoilers(0.5)
            self.director.nav(True)
            self._apply_flight_director(
                state, altitude=self._altitude_target(state),
                vs_fpm=-min(self.performance.descend_fpm * 2.0, 5000.0),
                heading=(self.session.assigned_heading
                         if self.session.assigned_heading is not None
                         else -1.0),
                speed_kt=self._speed_target(state))
        elif sid is not None:
            self._fly_procedure(state, sid)
        else:
            self.director.nav(True)
            self.director.airspeed_hold(True)
            target = self._altitude_target(state)
            alt = state.altitude_ft or 0.0
            if target and abs(alt - target) > 300.0:
                # A step clearance answered level is not answered at
                # all: the bug moves and the aeroplane climbs or
                # descends to it, FLCH where the model has one.
                vs = (self.performance.climb_fpm if alt < target
                      else -self.performance.descend_fpm)
                self._apply_flight_director(
                    state, altitude=target, vs_fpm=vs,
                    heading=(self.session.assigned_heading
                             if self.session.assigned_heading is not None
                             else -1.0),
                    speed_kt=self._speed_target(state))
            else:
                # No assigned or planned level is not an instruction to
                # find the sea: a zero on the bug is a descent to it.
                if target:
                    self.director.altitude(target)
                self.director.alt_hold(True)
            self._manage_throttle(state, self._speed_target(state))
            self.director.speed(self._speed_target(state))

        if self.session.phase == Phase.DESCENT:
            self._transition(AutonomousPhase.DESCENT)
        elif self.session.phase == Phase.APPROACH:
            self._transition(AutonomousPhase.APPROACH)

    def _phase_descent(self, state, now: float) -> None:
        """Descend: TOD planning, configuration, approach capture."""
        self._run_checklist_for_phase("descent")
        target = self.session.assigned_altitude_ft or self._descent_floor(state)

        # Top of descent: planned once from the distance to the field,
        # ~318 ft per NM on a three-degree path plus the height to lose.
        if not self.state.tod_nm:
            self._plan_tod(state)

        distance = self._distance_to_destination_nm(state)
        agl = (state.altitude_ft or 0.0) - self._field_elevation(state)

        # Configuration schedule, by height above the field and the
        # distance still to run: flap in stages as the aeroplane slows,
        # the gear with the approach flap, the landing flap on the
        # final. A crew does not put the wheels down at ten thousand
        # feet, whatever a single "approach" switch would do.
        on_final = distance is not None and distance < 5.0
        if agl < 8000 and self.state.configured == "":
            self._set_flaps(state, 0.25)
            self.state.configured = "flaps_1"
        if (agl < 3000 or (distance is not None and distance < 8.0)) \
                and self.state.configured in ("", "flaps_1"):
            self._set_flaps(state, 0.5)
            self.state.configured = "flaps_approach"
            if self._gear_allowed(state):
                self.director.gear(True)
            self.director.spoilers_arm(True)
        if (agl < 1500 or on_final) \
                and self.state.configured != "flaps_final":
            self._set_flaps(state, self.performance.landing_flaps or 1.0)
            self.state.configured = "flaps_final"
            if state.gear_down is not True and self._gear_allowed(state):
                self.director.gear(True)
            self.director.spoilers_arm(True)

        # Speed: assigned, else a sensible step-down -- the type's
        # figure in the descent, approach speed inside three thousand,
        # Vapp on the final.
        speed = self.session.assigned_speed_kt or (
            self._vapp(state) if on_final
            else (self._vapp(state)) + 20.0
            if agl < 3000 else self.performance.approach_kt)
        # The descent rate is the model's own where it declares one --
        # a clean turbofan comes down at three thousand, a piston at eight
        # hundred -- and the class figure where it does not. The sign is
        # taken as a magnitude: a negative or absurd report is a model
        # quirk, not a clearance to climb.
        descent_fpm = abs(float(getattr(state, "typical_descent_fpm", 0.0)
                                or 0.0))
        if not 100.0 <= descent_fpm <= 6000.0:
            descent_fpm = self.performance.descend_fpm
        if self.state.emergency_descent:
            descent_fpm = min(descent_fpm * 2.0, 5000.0)
            self.director.spoilers(0.5)
            if agl < 2500.0:
                # Low enough to land: the boards are stowed and the
                # approach machinery takes the rest of the way down.
                self.state.emergency_descent = False
                self.director.spoilers(0.0)
        # A filed/cleared STAR owns the lateral path until its last leg;
        # an assigned heading still wins where ATC gave one.
        star = self._star_nav(state)
        if star is not None:
            self._fly_procedure(state, star)
        else:
            assigned_hdg = self.session.assigned_heading
            # A re-clearance to a higher level while descending is a
            # climb, whatever the phase is named: an aeroplane does not
            # descend through an assigned level that sits above it.
            vs = (-descent_fpm
                  if (state.altitude_ft or 0.0) > target - 100.0
                  else self.performance.climb_fpm)
            self._apply_flight_director(state, altitude=target,
                                        vs_fpm=vs,
                                        heading=assigned_hdg if assigned_hdg is not None else -1.0,
                                        speed_kt=speed)

        # The approach's own navaid, tuned early enough that the needles
        # are alive by the time the final is flown.
        self._maybe_tune_nav(state, self._approach_recipe(state), now)

        # When the approach phase owns it, or we are close enough that the
        # approach logic should be flying.
        if self.session.phase == Phase.APPROACH or (
                distance is not None and distance < 12.0 and agl < 4000):
            self._transition(AutonomousPhase.APPROACH)
            return
        if now - self.state.phase_entered_at > self.DESAT_TIMEOUT:
            self._set_error("Descent timed out")

    def _phase_approach(self, state, now: float) -> None:
        """The approach: intercept, stabilize, land or go around."""
        self._run_checklist_for_phase("approach")
        if not self.state.sub:
            # Entering the approach names the sub-step: "intercept" is
            # where the windshear and weather gates look for the
            # aeroplane. A local copy of the name used to leave the
            # state's own empty, and those gates never saw the approach.
            self._goto_sub("intercept")
        sub = self.state.sub

        if sub == "go_around":
            self._fly_go_around(state, now)
            return

        self._configure_final(state)

        # The cleared approach, as a recipe: which navaid, which course,
        # which minima. Resolved once and kept until the field or the
        # runway changes; the radio is tuned to its navaid on the way in.
        recipe = self._approach_recipe(state)
        self._maybe_tune_nav(state, recipe, now)

        flown = False
        if recipe is not None and recipe.family != "visual":
            flown = self._fly_recipe(state, recipe, now)
        d = self.director
        if not flown:
            # ILS capture where the needles say it is there; otherwise fly
            # the runway heading to the threshold and let the glide path
            # be flown by reference to distance.
            has_ils = getattr(state, "nav1_has_localizer", False) and \
                getattr(state, "nav1_has_glideslope", False)
            if has_ils and self.session.ifr:
                d.approach(True)
                self.state.ap_apr_mode = True
                d.vs_hold(False)
                self.state.ap_vs_mode = False
            else:
                # Non-precision or visual: track the extended centreline
                # to the threshold like a procedure leg -- an aeroplane
                # coming in at an angle intercepts, it does not fly the
                # runway's heading parallel past the field -- and fly a
                # three-degree path down, groundspeed-scaled the way the
                # VNAV number is made.
                if self.state.runway_heading <= 0:
                    self._find_runway(state)
                if self.state.runway_heading > 0:
                    agl = state.altitude_agl_ft or 0.0
                    gs = state.ground_speed_kt or self._vapp(state) or 120.0
                    vs = -max(300.0, min(1200.0, gs * 5.3)) \
                        if agl > 400.0 else -250.0
                    heading = self._magnetic(self.state.runway_heading,
                                             state)
                    if getattr(state, "has_position", False):
                        faf = project(self.state.runway_lat,
                                      self.state.runway_lon,
                                      (self.state.runway_heading + 180.0)
                                      % 360.0, 10.0)
                        steer = auto_nav.path_steering(
                            state.latitude, state.longitude,
                            [faf, (self.state.runway_lat,
                                   self.state.runway_lon)],
                            lookahead_m=2000.0)
                        if steer and steer.get("heading") is not None:
                            heading = self._magnetic(steer["heading"],
                                                     state)
                    self._apply_flight_director(
                        state, vs_fpm=vs,
                        heading=heading,
                        speed_kt=self.state.vapp_kt
                        or self.performance.approach_kt)

        # Stabilized approach: the numbers a crew actually checks.
        self._check_stabilized(state)

        # Go-around triggers: not stabilized by minimums, sink rate or
        # speed outside limits low down, or no landing clearance when the
        # field is airborne-only.
        if self._should_go_around(state):
            self._start_go_around(state)
            return

        # Cleared and stable enough: hand over to the landing phase for
        # the flare. In the circle or still manoeuvring for the final is
        # neither: the flare is handed the aeroplane that is on it.
        agl = state.altitude_agl_ft or 0.0
        if self.session.cleared_landing and agl < 500.0 \
                and not self.state.circling \
                and (self.state.stabilized or agl < 60.0):
            self._transition(AutonomousPhase.LANDING)
            self._goto_sub("flare")
            return

        if self.session.phase == Phase.LANDING or self.session.phase == Phase.LANDED:
            self._transition(AutonomousPhase.LANDING)
            self._goto_sub("flare")
            return

        if now - self.state.phase_entered_at > self.APPROACH_TIMEOUT:
            self._set_error("Approach timed out")

    def _phase_landing(self, state, now: float) -> None:
        """Flare, touchdown, rollout, vacate."""
        d = self.director
        sub = self.state.sub or "flare"

        if sub == "flare":
            self._run_checklist_for_phase("before_landing")
            if state.on_ground:
                self.state.touchdown_at = now
                # The flare's clock belongs to this landing: a
                # touch-and-go would read it as a fifteen-second float
                # the moment the next pass sank under sixty feet.
                self.state.flare_at = 0.0
                self._goto_sub("rollout")
                return
            agl = state.altitude_agl_ft or 0.0
            # Below the flare window: the autopilot's job is done -- the
            # FO flies the flare by hand, like a pilot does.
            if agl < 60.0:
                if not self.state.flare_at:
                    self.state.flare_at = now
                    d.approach(False)
                    d.autopilot(False)
                    self.state.ap_master = False
                d.throttle_idle()
                d.throttle(0.0)
                self._last_lever = 0.0
                sink = abs(state.vertical_speed_fpm or 0.0)
                if self._use_pids:
                    # A real flare is a decaying sink target, not one
                    # pull: from the approach's descent rate at the flare
                    # height down to a hundred-odd feet a minute at the
                    # numbers. The loop chases that schedule against the
                    # sink the model reports.
                    want = max(100.0, min(750.0, agl * 13.0))
                    pull = 0.05 + self._pid("flare").update(
                        sink - want,
                        measurement=float(
                            state.vertical_speed_fpm or 0.0))
                else:
                    # A pilot flares against the sink he has: enough back
                    # pressure to trade the approach's descent rate for a
                    # hundred-odd feet a minute at the numbers -- not one
                    # fixed pull that floats one model and sinks another.
                    pull = 0.05 + max(0.0, min((sink - 150.0) / 1800.0,
                                               0.45))
                d.elevator(self._g_pull_cap(state, pull))
                # Still aligned with the runway: the de-crab kick of
                # rudder, and the wing held a touch into the wind so the
                # alignment does not buy a drift across the centreline
                # (the wind component's sign is the sim's, UNVERIFIED --
                # a wrong one shows as a slow bank, not a departure).
                if self.state.runway_heading > 0:
                    err = ((self.state.runway_heading
                            - (state.heading_true or 0.0) + 540.0)
                           % 360.0 - 180.0)
                    d.rudder(self._slew_rudder(
                        0.0 if abs(err) < 2.0
                        else max(-0.4, min(0.4, err / 36.0))))
                xwind = float(getattr(state, "aircraft_wind_x_kt", 0.0)
                              or 0.0)
                d.aileron(max(-0.15, min(0.15, xwind * 0.02)))
                # Touchdown must come; a flare that floats is a go-around.
                if now - self.state.flare_at > 15.0 and agl > 10.0:
                    self._start_go_around(state)
                    return
            else:
                # On the glide: on speed, configured. The autothrottle
                # holds Vapp where the model has one; where it does not
                # the lever keeps enough power to carry the glide to the
                # flare height -- a closed throttle at four hundred feet
                # is a landing short, not a flare.
                vapp = self.state.vapp_kt or self.performance.approach_kt
                if getattr(state, "autothrottle", False):
                    d.airspeed_hold(True)
                    self.state.ap_speed_mode = True
                    d.speed(vapp)
                else:
                    d.throttle(0.30)
                    self._last_lever = 0.30
                d.approach(True)
                if self.state.runway_heading <= 0:
                    self._find_runway(state)
            if now - self.state.phase_entered_at > self.LANDING_TIMEOUT:
                self._start_go_around(state)
            return

        if sub == "rollout":
            # Down: spoilers if armed, brakes on, steer to the exit or the
            # taxi path, slow to taxi speed. The aeroplane is on the
            # ground now -- the flight is over, and an autopilot left in
            # speed hold on the ground is what runs the levers to the
            # stops while parked.
            if self.state.ap_master or self.state.ap_speed_mode \
                    or self.state.ap_apr_mode or self.state.ap_vs_mode:
                d.autopilot(False)
                d.airspeed_hold(False)
                d.approach(False)
                d.localizer(False)
                d.vs_hold(False)
                if self._ap_mode == "managed":
                    d.level_change(False)
                self.state.ap_master = False
                self.state.ap_speed_mode = False
                self.state.ap_apr_mode = False
                self.state.ap_vs_mode = False
                self.state.ap_alt_mode = False
                self.state.ap_hdg_mode = False
                self.state.ap_nav_mode = False
            d.spoilers_arm(True)
            d.spoilers(1.0)
            # The flaps stay where the landing put them until the
            # taxi-in: a retraction during the rollout is an after-
            # landing item, not something done at a hundred knots.
            gs = state.ground_speed_kt or 0.0
            if gs > self.performance.taxi_kt + 5.0:
                # Reverse out to sixty knots where a fast aeroplane has
                # it -- then the wheel brakes, graduated with speed the
                # way the autobrake detents a crew actually selects are.
                d.parking_brake(False)
                if (self.performance.cruise_kt >= 180.0
                        and capabilities(state.source).flight_axes):
                    if gs > 60.0 and not self.state.reversers_out:
                        d.send(b"THROTTLE_REVERSE_THRUST_TOGGLE", 0)
                        self.state.reversers_out = True
                    elif gs <= 60.0 and self.state.reversers_out:
                        d.send(b"THROTTLE_REVERSE_THRUST_TOGGLE", 0)
                        self.state.reversers_out = False
                # Reverse does the work above sixty; the wheel brakes
                # are eased while it is in and do the stopping as it
                # comes out -- graduated with speed the way the
                # autobrake detents a crew actually selects are.
                d.brakes(0.4 if gs > 60.0 else 0.7 if gs > 30.0 else 1.0)
            else:
                # Slow enough: brakes off, the boards stowed and
                # disarmed -- a crew does not taxi to the stand with
                # the spoilers out -- taxi in.
                d.brakes(0.0)
                d.spoilers(0.0)
                d.spoilers_arm(False)
                if not self.state.vacated:
                    self.state.vacated = True
                    self._run_checklist_for_phase("after_landing")
                    # Off the runway: the tower, and after it whoever
                    # the frequency belongs to, hears the vacated call
                    # it expects -- once, and only when the FO can
                    # actually transmit.
                    if not self.state.vacated_call \
                            and self._manage_radio \
                            and self._radio_quiet():
                        self._fo_say("vacated")
                        self.state.vacated_call = True
                self._transition(AutonomousPhase.TAXI_IN)
                self._goto_sub("taxi")
                return
            # Keep it on the centreline until slow. Above walking pace
            # the rudder alone holds the line -- the nosewheel thrown in
            # at rollout speed is the swerve a landing is lost to, not
            # saved by.
            if self.state.runway_heading > 0:
                err = ((self.state.runway_heading
                        - (state.heading_true or 0.0) + 540.0)
                       % 360.0 - 180.0)
                rudder = self._slew_rudder(
                    0.0 if abs(err) < 2.0
                    else max(-0.4, min(0.4, err / 36.0)))
                d.rudder(rudder)
                d.steering(rudder if gs < 35.0 else 0.0)
            return

    def _phase_taxi_in(self, state, now: float) -> None:
        """Taxi to the stand after landing."""
        self._run_checklist_for_phase("after_landing")
        sub = self.state.sub or "taxi"
        if sub == "taxi":
            # The after-landing flow a pilot runs on the taxi: flaps
            # back up once the rollout is done with them.
            if state.flaps_index:
                self.director.flaps(0.0)
            self._taxi_along_path(state, outbound=False)
        elif sub == "park":
            self._stop(state)
            if (state.ground_speed_kt or 0.0) <= 1.0:
                self.director.parking_brake(True)
                self._transition(AutonomousPhase.PARKED)

        if self.session.phase == Phase.PARKED:
            self._transition(AutonomousPhase.PARKED)
        elif now - self.state.phase_entered_at > self.TAXI_TIMEOUT:
            if (state.ground_speed_kt or 0.0) > 1.0:
                self._set_error("Taxi-in timed out")
            else:
                # Stopped on the movement area with nowhere left to go:
                # the flight is done where it stands rather than held
                # on the taxi phase for ever.
                self.director.parking_brake(True)
                self._emit("info", "Parked where it stands: no stand "
                           "route came", "autonomous.taxi_in.parked")
                self._transition(AutonomousPhase.PARKED)

    def _phase_parked(self, state, now: float) -> None:
        """At parking position: the shutdown checklist, then the sequence."""
        self.director.parking_brake(True)
        self._run_checklist_for_phase("shutdown")

        if self._chained:
            self._transition(AutonomousPhase.CHAIN_NEXT)
        else:
            self._transition(AutonomousPhase.SHUTDOWN)

    def _phase_shutdown(self, state, now: float) -> None:
        """Shutdown sequence: secure, cool, engines off, electrics off."""
        d = self.director
        sub = self.state.sub or "secure"

        if sub == "secure":
            # The walk-around items the FO does: lights off except the
            # beacon, pitot heat off, controls centred. The autopilot
            # comes off too -- a parked aeroplane has no flight director.
            d.autopilot(False)
            d.airspeed_hold(False)
            d.approach(False)
            d.vs_hold(False)
            self.state.ap_master = False
            self.state.ap_speed_mode = False
            self.state.ap_apr_mode = False
            self.state.ap_vs_mode = False
            d.strobes(False)
            d.landing_light(False)
            d.taxi_light(False)
            d.nav_lights(False)
            d.pitot_heat(False)
            d.center_controls()
            self.state.shutdown_at = now
            self._goto_sub("cooldown")

        elif sub == "cooldown":
            # While the engines finish down the stand gets its asks: a
            # bridge or the stairs to the aeroplane, then the passengers
            # off -- one GSX ask at a time, at the menu's own pace.
            self._ask_the_ramp(state)
            # Turbines cool at idle before shutdown; the clock is the
            # checklist's, not a guess at the model's temperatures.
            if self._is_turbine(state) and \
                    now - self.state.shutdown_at < self.SHUTDOWN_COOLDOWN_S:
                return
            self._goto_sub("engines_off")

        elif sub == "engines_off":
            self._ask_the_ramp(state)
            if state.engines_running:
                self._shutdown_engines()
                if self._sub_age() > 60.0:
                    self._set_error("Engines would not shut down")
                    return
            else:
                self._goto_sub("electrics_off")

        elif sub == "electrics_off":
            self._ask_the_ramp(state)
            if self._sub_age() < 5.0:
                d.avionics(False)
                d.battery(False)
                d.beacon(False)
                return
            self._emit("info", "Autonomous flight completed",
                       "autonomous.completed")
            self._transition(AutonomousPhase.COMPLETE)

    def _phase_chain_next(self, state, now: float) -> None:
        """Turn the aeroplane around and load the next leg of the chain.

        ``pick`` chooses the destination, ``delay`` gives the stand its
        moment, ``service`` shuts the engines down and fuels for the leg
        with its diversion, and ``load`` files the leg and starts it.
        """
        sub = self.state.sub or "pick"

        if sub == "pick":
            if self.state.chain_flight_index >= len(self.state.chain_flights):
                self._select_next_destination()
            if self.state.chain_flight_index >= len(self.state.chain_flights):
                self._emit("info",
                           "No suitable destination found for next flight",
                           "autonomous.chain.no_destination")
                self._transition(AutonomousPhase.SHUTDOWN)
                return
            self.state.plan_flight = self.state.chain_flights[
                self.state.chain_flight_index]
            self._goto_sub("delay")
            return

        if sub == "delay":
            if now - self.state.phase_entered_at < self._chain_delay:
                return
            self._goto_sub("service")

        if sub == "service":
            if self._service_turnaround(state, now):
                self._goto_sub("load")
            return

        if sub == "load":
            self.state.chain_flight_index += 1
            self._load_next_flight(self.state.plan_flight)
            self.state.plan_flight = None
            self._transition(AutonomousPhase.PREFLIGHT)

