from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    Phase,
    _nws_fade,
    approach_flight,
    auto_nav,
    capabilities,
    fly,
    random,
)


class _The_phase_machine:
    # -- the phase machine ----------------------------------------------------

    # How far down the runway the entry is before getting to the departure
    # end counts as a backtrack rather than a line-up -- an intersection
    # this near the numbers is just a lineup; and how close to the end the
    # backtrack turns round: enough pavement for the turn, none wasted.
    BACKTRACK_MIN_M = 250.0
    BACKTRACK_TURN_M = 60.0

    # The chance this crew asks for an intersection departure rather
    # than the full length -- a habit some pilots have and others
    # never take up.
    INTERSECTION_ASK = 0.30

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
            # A crew's habits are settled once, the way the filing below
            # is: this one will take an intersection entry over the long
            # taxi to the numbers, where the runway has a legal one to
            # give. A helicopter departs from wherever it sits and has
            # no intersection to ask for.
            self.session.intersection_preferred = (
                not self._is_helicopter
                and random.random() < self.INTERSECTION_ASK)
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
            if (self.state.last_radio_at > 0.0
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
            # The asks keep going out; the wait is said about, once.
            rules = "IFR clearance" if self.session.ifr else "VFR release"
            self._phase_overdue(
                "clearance", now - self.state.clearance_asked_at, 0.0,
                f"Still waiting for the {rules}; asking again")

    def _phase_pushback(self, state, now: float) -> None:
        """Engine start, pushback, brakes back on."""
        d = self.director
        sub = self.state.sub or "engines"

        if sub == "engines":
            # The start happens on the brake: an idling twin creeps on
            # its own thrust, and it stays on until the tug is attached.
            d.parking_brake(True)
            out, seen = self._engines_out(state)
            if out or not seen:
                if self.state.engines_started_at == 0.0:
                    self._start_engines(state)
                    self.state.engines_started_at = now
                    self.state.start_asked_at = now
                else:
                    # No clock gives up on a start: a model that needs
                    # its own event or a cold-soaked engine is cranked
                    # until it lights -- said once, retried below.
                    self._phase_overdue(
                        "start", now - self.state.engines_started_at,
                        self.ENGINE_START_TIMEOUT,
                        "Engine start is taking a while; still trying")
                    if now - self.state.start_asked_at > 30.0:
                        # The re-ask owes only the engines still out: a
                        # twin whose second engine never lit is cranked
                        # again, not errored into a one-engine taxi.
                        self._start_engines(state)
                        self.state.start_asked_at = now
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
            # An open-ended tug (both sims' push events are toggles that
            # push until told to let go) is let go on a clock: half a
            # minute at walking pace is clear of any stand, and the
            # aeroplane coming to rest is what the normal completion
            # below then reads.
            if self.state.moved_since_push and not self.state.push_stop_sent \
                    and capabilities(state.source).pushback_open_ended \
                    and now - self.state.still_since < 2.0 \
                    and now - self.state.push_moved_at > 30.0:
                # The release is a toggle on both sims: sent only while
                # the aeroplane still moves, so a tug already let go is
                # not toggled back on.
                self.state.push_stop_sent = True
                self.director.pushback_stop()
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
                # The tug on the nosewheel is what lets the brake off:
                # PUSHBACK ATTACHED, or GSX's own word for "on the tug".
                # A tug still on its way (PUSHBACK WAIT -- X-Plane never
                # sets it) is not a reason to release, because an idling
                # aeroplane creeps forward the moment the brake is.
                attached = bool(getattr(state, "pushback_attached", False))
                coming = attached or getattr(state, "pushback_wait", False)
                gstate = getattr(getattr(self.brain, "ground", None),
                                 "state", None)
                if getattr(gstate, "being_pushed", False):
                    attached = coming = True
                # A dispatch that never becomes an attachment -- the
                # tug's WAIT flag stuck on a spawn that failed -- is the
                # tug that never came; ``being_pushed`` and ``attached``
                # both say it is physically there, so they wait as long
                # as they say so.
                stale = coming and not attached and self._sub_age() > 150.0
                if attached:
                    # The tug is on the nosewheel -- including the one a
                    # controller's approval sent: the brake that held the
                    # start has to be off for it to push. And where GSX
                    # asks the flight deck whether it is ready, the FO's
                    # answer is yes: the brake is off for it.
                    d.parking_brake(False)
                    if self._sub_age() > 15.0:
                        self._ground_confirm_push()
                elif coming and not stale:
                    # Dispatched but not on the nosewheel yet: the brake
                    # stays on. The request event is a toggle, so asking
                    # again while a tug is coming would cancel it -- the
                    # wait is simply kept, brakes held.
                    d.parking_brake(True)
                elif stale:
                    if self._pushback_skippable():
                        self._emit(
                            "info",
                            "The pushback tug never arrived; taxiing out",
                            "autonomous.pushback.skipped")
                        self._goto_sub("taxi")
                        self._transition(AutonomousPhase.TAXI_OUT)
                        return
                    self._set_error("Pushback tug never arrived")
                    return
                else:
                    # A call on the frequency is repeated with patience,
                    # not streamed -- and one never answered falls back
                    # to the simulator's own tug before it is a failure.
                    # An answered call is finished being a radio call:
                    # the session's move past "cleared" is the approval,
                    # and a tug that never shows is pressed for directly
                    # rather than asked for on the radio a second time.
                    if self.state.push_radio and getattr(
                            getattr(self, "session", None), "phase", None) \
                            not in (None, Phase.PREFLIGHT, Phase.CLEARED):
                        self.state.push_radio = False
                        self.state.push_retries = 0
                    retry_after = 40.0 if self.state.push_radio else 120.0
                    # The request event is a toggle: a slow push under
                    # ``moved_since_push``'s threshold that its model
                    # never flags as attached would be toggled OFF by a
                    # retry -- only ask again while genuinely standing.
                    if self._sub_age() > retry_after \
                            and self.state.push_retries < 3 \
                            and abs(float(getattr(
                                state, "ground_speed_kt", 0.0) or 0.0)) \
                            < 0.3:
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

    def _phase_holding_short(self, state, now: float) -> None:
        """Holding short of runway, run before-takeoff checklist."""
        # Stopped at the bars is itself the report the session's position
        # guess could not make: a hold reached on the route can sit far
        # from the published end, and the tower works off the session --
        # a flight still "taxiing" is sent the taxi instruction forever.
        if self.session.phase in (Phase.PREFLIGHT, Phase.CLEARED,
                                  Phase.PUSHBACK, Phase.TAXI_OUT):
            self.session.phase = Phase.HOLDING_SHORT
        # The clearance is the switch: a "line up and wait" or a takeoff
        # while the nose is still at the bars is strobes on already.
        self._clearance_strobes(state)
        if (state.ground_speed_kt or 0.0) > 1.0:
            self._stop(state)
            return
        self._run_checklist_for_phase("before_takeoff")

        # "Line up and wait" puts the aeroplane on the runway; the takeoff
        # clearance is what starts the roll. The aeroplane only goes if it
        # is light enough and the pavement is long enough for it.
        if self.session.lined_up or self.session.cleared_takeoff \
                or self.session.phase in (Phase.LINED_UP, Phase.TAKEOFF):
            pending = getattr(self.session, "pending", None)
            if pending is not None and pending.readback_required \
                    and not pending.read_back:
                # The bars are crossed after the line-up or takeoff
                # clearance has been acknowledged, not while that
                # readback is still owed to the tower.
                return
            if not self._departure_possible(state):
                return
            self._backtrack_turned = False
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

    def _runway_half_m(self) -> float:
        """How far off the centreline still counts as on the pavement:
        half the runway's own width, minus the metres the gear tracking
        keeps in hand. Without a width -- one the database or the
        simulator never gave -- a conservative strip's worth."""
        width_m = (self.state.runway_width_ft or 100.0) * 0.3048
        return max(6.0, width_m / 2.0 - 3.0)

    def _phase_line_up(self, state, now: float) -> None:
        """Line up on runway and prepare for takeoff."""
        d = self.director
        sub = self.state.sub or "align"

        if not getattr(state, "taxi_light", False):
            d.taxi_light(True)
        self._clearance_strobes(state)
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
            # Somebody on the strip the aeroplane is cleared onto: wait
            # where the hold is rather than drive on behind them. A
            # departure rolling frees the strip in seconds; an arrival
            # rolling out is gone by the time the lineup is steered.
            if self._runway_blocked():
                self._stop(state)
                return
            # "Line up and wait" is not the takeoff word: the arrival it
            # names is still in the air when the call comes -- invisible
            # to the occupancy check above. The bars are held until its
            # final is no longer landing inside the window; once it is
            # on the pavement the blocked check owns it again.
            if not self.session.cleared_takeoff and self._runway_inbound():
                self._stop(state)
                return
            # Taxi onto the centreline and align with the runway heading.
            # ``align`` says how far off the extended centreline the
            # aeroplane is and the heading that puts it back on; steering
            # at that heading is the FO's interception, then the runway
            # heading itself once the wings are level on the numbers.
            err = auto_nav.align(state.latitude, state.longitude,
                                 self.state.runway_lat, self.state.runway_lon,
                                 self.state.runway_heading,
                                 lookahead_m=120.0)
            hdg_err = abs((self.state.runway_heading
                            - (state.heading_true or 0.0) + 540.0)
                           % 360.0 - 180.0)
            # On the pavement the strobes go on -- the call a crew makes
            # crossing the bars, not only once lined up on the numbers.
            if abs(err["offset_m"]) <= self._runway_half_m() + 2.0 \
                    and not getattr(state, "strobe_light", False):
                d.strobes(True)
            # The correction is applied at every offset: it self-nulls
            # on the line, and a "near enough" band wider than the
            # aligned gate would hold a parallel offset for ever
            # instead of reaching the centreline.
            steer_hdg = (self.state.runway_heading
                         + err["correction_deg"]) % 360.0
            lineup_kt = self._expedite_kt(
                state, self.performance.taxi_kt)
            # Where the taxi plan walked to the bars over a connector,
            # the same plan knows the paved way in: the holding point's
            # own taxiway onto the runway, not the straight intercept to
            # the extended centreline, which cuts the connector's inside
            # corner across the infield -- how the Baron left A3 on the
            # grass. Until that path's end is reached (it finishes on
            # the centreline just past the entry) the aeroplane is still
            # on the connector, and neither the intercept below nor a
            # backtrack's reciprocal heading is the way there.
            lineup = list(getattr(self.session, "lineup_path", None) or [])
            on_path = False
            if len(lineup) >= 2:
                # The walk only moves forward, the way the taxi pursuit
                # does: a whole-path pick re-seats the nearest segment
                # on whatever leg passes closest, and the pursuit
                # teleporting between them is the swing through the
                # numbers the lineup weaves on. A new plan re-seeds it.
                if getattr(self, "_lineup_on", None) != lineup:
                    self._lineup_progress = 0
                    self._lineup_on = lineup
                    self._lineup_done = False
                at = min(getattr(self, "_lineup_progress", 0),
                         len(lineup) - 2)
                paved = None
                # ``done`` is a latch, not a waypoint state: drifting a
                # reach past the last point re-seats the pursuit on the
                # leg behind, and the nose is swung back across the
                # junction it just made -- the wild heading demands a
                # finished path still answered with.
                if not getattr(self, "_lineup_done", False):
                    paved = auto_nav.path_steering(
                        state.latitude, state.longitude, lineup[at:],
                        lookahead_m=30.0)
                if paved is not None \
                        and paved.get("heading") is not None \
                        and not paved.get("done") \
                        and abs(paved.get("cross_track_m") or 0.0) <= 60.0:
                    self._lineup_progress = at + paved["nearest"]
                    steer_hdg = paved["heading"]
                    on_path = True
                    # The path ends on the numbers: roll up to that end
                    # at the pace a stop wants, not still at taxi
                    # speed -- the sail past the centreline that leaves
                    # the roll half the pavement it was counted on.
                    lineup_kt = min(lineup_kt, max(
                        3.0, (paved.get("remaining_m") or 0.0) * 0.10))
                elif paved is not None and paved.get("done") \
                        and abs(err["offset_m"]) \
                        <= self._runway_half_m() + 5.0:
                    self._lineup_done = True
            # Cleared onto the runway while still on the taxi route to
            # the bars: the paved way there is the taxi path, not the
            # diagonal over the infield to the connector -- steering at
            # a lineup leg a field away is how the grass gets crossed.
            if not on_path and abs(err["offset_m"]) > 40.0:
                taxi = list(getattr(self.session, "taxi_path", None) or [])
                if len(taxi) >= 2:
                    # A route to the bars ends beside the runway, never
                    # on it: a path whose last point lies on the
                    # centreline is no taxi route to keep following.
                    ends_at_the_bars = abs(auto_nav.align(
                        *taxi[-1], self.state.runway_lat,
                        self.state.runway_lon,
                        self.state.runway_heading)["offset_m"]) > 15.0
                    # Same forward-only walk the taxi phase took here:
                    # the pursuit cannot jump back onto a leg already
                    # passed and start an orbit at the bars.
                    at = min(getattr(self, "_taxi_progress", 0),
                             len(taxi) - 2)
                    on_taxi = ends_at_the_bars and auto_nav.path_steering(
                        state.latitude, state.longitude, taxi[at:],
                        lookahead_m=30.0)
                    if on_taxi \
                            and on_taxi.get("heading") is not None \
                            and not on_taxi.get("done"):
                        self._taxi_progress = at + on_taxi["nearest"]
                        d.parking_brake(False)
                        ask = abs((on_taxi["heading"]
                                   - (state.heading_true or 0.0)
                                   + 540.0) % 360.0 - 180.0)
                        self._steer(state, on_taxi["heading"], lineup_kt,
                                    brake_turn=ask > 45.0)
                        return
            if not on_path and err["along_m"] > self.BACKTRACK_MIN_M \
                    and abs(err["offset_m"]) <= 60.0 \
                    and not getattr(self, "_backtrack_turned", False) \
                    and not getattr(self.session,
                                    "departure_remaining_m", 0.0):
                # Joined the runway far from the departure end: the way
                # to the threshold is along the pavement itself. Enter
                # facing the way back, drive it, turn round at the end.
                # Except where the taxi plan put it there on purpose --
                # an intersection departure lines up where it joined,
                # on the centreline past the entry, and the pavement
                # ahead of that point is what the clearance counted
                # down to. ``departure_remaining_m`` is that fact; it
                # holds on an unnamed intersection too.
                self._goto_sub("backtrack")
                return
            if not on_path and hdg_err < 30.0 \
                    and (getattr(self, "_lineup_done", False)
                         or abs(err["offset_m"])
                         <= self._runway_half_m()):
                # On the pavement pointed down it: the numbers are
                # rolled up to at walking pace, so the aligned gate a
                # few metres and a few degrees away is caught on the
                # spot it was made for -- not five hundred metres
                # further on at taxi speed, where the roll then starts
                # with the pavement the departure was promised gone.
                lineup_kt = min(lineup_kt, 4.0)
            # The corner's price is the turn the steering asks for and
            # the sharp vertex the path still turns at -- the swing
            # onto the runway heading is made at the join, where the
            # pursuit's ask swings, not crawled toward the whole way in
            # at a hairpin's pace because the heading owed to the
            # runway is large while the connector's is two degrees.
            ask = abs((steer_hdg - (state.heading_true or 0.0)
                       + 540.0) % 360.0 - 180.0)
            for need_deg, corner_kt in self.CORNER_SLOW:
                if ask >= need_deg:
                    lineup_kt = min(lineup_kt, corner_kt)
                    break
            if on_path:
                vertex = next(((at_m, deg) for at_m, deg in
                               auto_nav.turns_ahead(
                                   state.latitude, state.longitude,
                                   lineup[at:], self.CORNER_LOOK_M,
                                   skip_penalty_m=self.NEAREST_SKIP_M)
                               if abs(deg) >= self.CORNER_CUT_DEG),
                              None)
                if vertex is not None:
                    at_m, deg = vertex
                    for need_deg, corner_kt in self.CORNER_SLOW:
                        if abs(deg) >= need_deg:
                            lineup_kt = min(lineup_kt, corner_kt + max(
                                0.0, at_m - self.CORNER_FULL_M)
                                            * self.CORNER_BRAKE_KT_M)
                            break
            d.parking_brake(False)
            self._steer(state, steer_hdg, lineup_kt,
                        brake_turn=ask > 45.0)
            # "Lined up" means on the paint, not merely on the pavement:
            # within a few metres of the centreline and a few degrees of
            # the heading. The old gate -- inside the strip's own
            # half-width -- declared the Baron aligned at the edge of a
            # 45 m runway, which is exactly where the roll then started.
            # ``along_m`` keeps it honest the other way: on the extended
            # centreline but still short of the numbers is the taxiway,
            # not a lineup.
            if abs(err["offset_m"]) < min(3.0, self._runway_half_m()) \
                    and err["along_m"] > -40.0 and hdg_err < 2.0:
                self.state.aligned = True
                self._goto_sub("set")
                self._emit("info", "Lined up on runway",
                           "autonomous.lined_up")

        elif sub == "backtrack":
            # Somebody turning onto the strip while the aeroplane is
            # driving it is met head-on at taxi speed, not at takeoff
            # speed -- which does not make it survivable, only slower.
            # The backtrack holds where it is and the tower hears why.
            blocked = self._runway_blocked()
            if blocked != getattr(self, "_backtrack_blocked", False):
                self._backtrack_blocked = blocked
                if blocked:
                    self._emit("info",
                               "Runway occupied: holding on the strip",
                               "autonomous.backtrack.blocked")
            if blocked:
                self._stop(state)
                return
            # Down the runway toward the departure end, pointed at it:
            # the centreline at the reciprocal heading, until the
            # threshold end is near enough to turn round on.
            back_hdg = (self.state.runway_heading + 180.0) % 360.0
            err = auto_nav.align(state.latitude, state.longitude,
                                 self.state.runway_lat, self.state.runway_lon,
                                 self.state.runway_heading)
            # The centreline is the runway's own line read in the
            # direction being driven: the correction is applied at
            # every offset -- it self-nulls on the paint -- so the
            # backtrack is down the centreline, not a parallel track.
            steer_err = auto_nav.align(
                state.latitude, state.longitude,
                self.state.runway_lat, self.state.runway_lon, back_hdg,
                lookahead_m=120.0)
            steer_hdg = (back_hdg + steer_err["correction_deg"]) % 360.0
            # The end of the drive is a 180 on the pavement that is
            # left: slow toward it the way a crew does, and slower
            # still while a big offset is being corrected.
            to_turn_m = max(0.0, err["along_m"] - self.BACKTRACK_TURN_M)
            target_kt = min(self.performance.taxi_kt,
                            max(4.0, 3.0 + to_turn_m * 0.05))
            if abs(steer_err["offset_m"]) > 20.0:
                target_kt = min(target_kt, 6.0)
            d.parking_brake(False)
            self._steer(state, steer_hdg, target_kt)
            if err["along_m"] <= self.BACKTRACK_TURN_M:
                self._backtrack_turned = True
                self._goto_sub("turn_out")
                self._emit("info", "At the departure end: turning round",
                           "autonomous.backtrack.turn")

        elif sub == "turn_out":
            # The 180 at the end of the backtrack: walking pace, and the
            # committed-reversal law in ``_steer`` picks the side and
            # sees it through. Once the nose is coming round onto the
            # runway heading the turn is no longer the question -- the
            # centreline is: a 180 that swings wide lands beside the
            # strip, so the last of the swing is steered at the line
            # itself and the roll-out finishes on it.
            d.parking_brake(False)
            hdg_err = abs((self.state.runway_heading
                            - (state.heading_true or 0.0) + 540.0)
                          % 360.0 - 180.0)
            steer_hdg = self.state.runway_heading
            if hdg_err < 60.0:
                err = auto_nav.align(
                    state.latitude, state.longitude,
                    self.state.runway_lat, self.state.runway_lon,
                    self.state.runway_heading, lookahead_m=60.0)
                steer_hdg = (steer_hdg + err["correction_deg"]) % 360.0
            self._steer(state, steer_hdg, 3.0)
            if hdg_err < 15.0:
                self._goto_sub("align")

        elif sub == "set":
            # Aligned: brakes on, checks complete, wait for the roll.
            # "Lined up and wait" can be a long wait at a busy field --
            # the clearance is the tower's to give in its time, and an
            # aeroplane parked on the numbers with its engines running
            # is exactly what was asked for, not a failure.
            # Still where the roll wants it? A blown turn or a nudge is
            # a re-align, not a takeoff begun beside the centreline.
            if self.state.runway_heading > 0:
                err = auto_nav.align(state.latitude, state.longitude,
                                     self.state.runway_lat,
                                     self.state.runway_lon,
                                     self.state.runway_heading)
                hdg_err = abs((self.state.runway_heading
                               - (state.heading_true or 0.0) + 540.0)
                              % 360.0 - 180.0)
                # Beside the paint is a re-align, not a takeoff waited
                # for half a lane off -- the slack over the align gate
                # is the creep a hold brake allows, no more.
                if abs(err["offset_m"]) > min(6.0, self._runway_half_m()) \
                        or hdg_err > 5.0:
                    self.state.aligned = False
                    self._goto_sub("align")
                    return
            d.parking_brake(True)
            self._stop(state)
            self._configure_for_takeoff(state)
            if (self.session.cleared_takeoff
                    or self.session.phase == Phase.TAKEOFF) \
                    and not getattr(self.session, "takeoff_cancelled", False) \
                    and not self._runway_blocked():
                self._transition(AutonomousPhase.TAKEOFF)

    def _phase_takeoff(self, state, now: float) -> None:
        """Takeoff roll, rotation, initial climb."""
        d = self.director
        sub = self.state.sub or "wait_clearance"
        self._compute_speeds(state)

        if sub == "wait_clearance":
            if self._airborne_confirmed(state, now):
                self.state.airborne_at = self.state.airborne_at or now
                self._goto_sub("initial_climb")
                return
            d.parking_brake(True)
            self._stop(state)
            # Held for the clearance: as long as the aeroplane is on the
            # numbers with its engines running it is doing exactly what
            # "cleared for takeoff" is waited for. A rejected takeoff
            # latches -- nothing re-rolls after one. The tower's own
            # phase counts too: the lineup's "set" gate already reads a
            # TAKEOFF session phase as the clearance it is, and a flag
            # rescinded without that phase changing (an abort the tower
            # vacated) must not park the aeroplane here for ever.
            if (self.session.cleared_takeoff
                    or self.session.phase == Phase.TAKEOFF) \
                    and not getattr(self.session, "takeoff_cancelled", False) \
                    and not self.state.rejected:
                if self._runway_blocked():
                    # The strip was clear when the clearance came; it is
                    # not now -- wait for it again rather than roll over
                    # whoever arrived in between.
                    return
                pending = getattr(self.session, "pending", None)
                if pending is not None and pending.readback_required \
                        and not pending.read_back:
                    # The clearance is answered before the power comes
                    # up: the readback is the proof the words were
                    # heard to the end, and a crew does not roll
                    # mid-sentence on a clearance it has not
                    # acknowledged.
                    return
                if self.state.runway_heading > 0 \
                        and not self.state.aligned:
                    # Cleared but never made the lineup: the roll starts
                    # on the centreline pointing down it, so the lineup
                    # is made first.
                    self._transition(AutonomousPhase.LINE_UP)
                    return
                # The power events are one-shot: a new roll is a new
                # press of TO/GA and THROTTLE_FULL.
                self.director.forget("takeoff_power", "toga")
                self._goto_sub("roll")
            return

        if sub == "roll":
            if self._airborne_confirmed(state, now):
                self.state.airborne_at = now
                # The roll's into-wind aileron is a ground answer -- in
                # the air a crossed yoke only banks it.
                d.aileron(0.0)
                self._goto_sub("rotate")
                return
            # Brakes off -- all of them: the parking lever and the toe
            # pedals, whose override latches a hold the parking release
            # alone does not undo -- then takeoff power, rudder for
            # direction. The hold's release is asked for every tick
            # because a write the sim never takes is a roll that never
            # starts.
            d.parking_brake(False)
            d.brakes(0.0)
            if getattr(state, "parking_brake", False) \
                    and self._sub_age() > 1.5:
                if not getattr(self, "_brake_stuck_warned", False):
                    self._brake_stuck_warned = True
                    self._emit("info",
                               "Parking brake not releasing on the roll",
                               "autonomous.brake.stuck")
            else:
                self._brake_stuck_warned = False
            # Rejected takeoff: the roll is not accelerating. Not half of
            # rotation speed by the whole expected roll is an abort, not a
            # rotate -- anything nearer the profile is a slow spool-up,
            # which the clock was mistaking for a failure because it also
            # counts the dead seconds of the brake release and spool.
            vr = self.state.vr_kt or self.performance.rotate_kt
            elapsed = now - self.state.sub_entered_at
            expected = max(10.0, vr / max(self.performance.takeoff_kt_s, 0.5))
            # Whichever speed says it is moving: a model that lags the
            # indicated reading spools it up seconds behind the ground
            # truth, and a roll rejected on that lag is an aeroplane
            # stopped on the runway for no fault but a pitot delay. A
            # roll that is not accelerating shows it on both.
            ias = max(float(state.indicated_airspeed_kt or 0.0),
                      float(state.ground_speed_kt or 0.0))
            if ias < vr * 0.5 and elapsed > expected:
                self.state.rejected = True
                why = ("brakes still applied"
                       if getattr(state, "parking_brake", False)
                       else "insufficient acceleration")
                self._emit("error", f"Rejected takeoff: {why}",
                           "autonomous.rejected_takeoff")
                self._goto_sub("abort")
                return
            self._set_takeoff_power(state)
            # The TO/GA press above engages the autopilot on the models
            # that arm one, and an engaged AP is a second hand on the
            # pedals: its nosewheel servo steers for whatever heading
            # bug the airframe was left with -- a stale bug's pull is
            # the left yaw every rejected roll converged on, right down
            # to the parked aeroplane still walking its nose to ~335
            # with the wheels stopped. The FO's foot owns the roll; the
            # release is asked for every tick the way the brakes' is,
            # because a write the sim never takes is the swerve back.
            if state.on_ground:
                d.autopilot(False)
            # Directional control: the nosewheel does the early work, the
            # rudder takes over as the airflow comes alive -- faded
            # across the handover the way the gear's own schedule is,
            # not cut at a number. Both get a demand from the same rate
            # servo the taxi law runs, tightened to a few degrees a
            # second and divided by the authority this model has shown
            # at this speed: a hard pedal at eighty knots is the swerve
            # a takeoff roll is lost to, not saved by, and a model that
            # answers its rudder differently was weaving under a fixed
            # gain.
            gs = state.ground_speed_kt or ias
            keep_hdg = self.state.runway_heading
            if keep_hdg <= 0:
                # No runway figure to keep: the heading the lineup left
                # is the one the roll holds -- a steer for zero degrees
                # drives off the pavement side on any other runway.
                keep_hdg = float(getattr(state, "heading_true", 0.0)
                                 or 0.0)
            if not getattr(self, "_roll_bug_sent", False):
                # And the bug is told the strip's own number: nothing
                # the roll fights is allowed to aim somewhere else.
                self._roll_bug_sent = True
                d.heading(self._magnetic(keep_hdg, state))
            # The V-speeds first: the pavement checks below measure the
            # distance left against the speed still owed.
            rotate_at = (self.state.vr_kt
                         or getattr(state, "min_rotation_kt", 0.0) or 0.0
                         or self.performance.rotate_kt)
            self.state.rotate_speed_kt = rotate_at
            v1 = self.state.v1_kt or (rotate_at - 10.0)
            if int(getattr(state, "engine_count", 0) or 0) == 1:
                v1 = 9999.0
            if self.state.runway_heading > 0 and self.state.runway_lat:
                # Track the paint, not just the heading: a lineup can
                # finish a few metres off the line and runway heading
                # alone would hold that offset for the whole roll.
                # The correction is the centreline's own, capped
                # gentle -- a swerve at speed is the accident a
                # takeoff roll is lost to, not a slow rejoin.
                cl = auto_nav.align(
                    state.latitude, state.longitude,
                    self.state.runway_lat, self.state.runway_lon,
                    self.state.runway_heading, lookahead_m=200.0)
                keep_hdg = (self.state.runway_heading + min(
                    8.0, max(-8.0, cl["correction_deg"]))) % 360.0
                # The roll the books abandon: a heading forty-odd
                # degrees off the strip's own is the swing a takeoff
                # is lost to -- the wheel and rudder demand never asks
                # for it, so its answer is the plant's, and a plant
                # that answers with a swing is not one the aeroplane
                # is flown off behind. The wheels past the pavement's
                # edge is the same verdict measured the other way. A
                # reject is for while the wheels still touch: past
                # liftoff the aeroplane is flown, not stopped.
                if state.on_ground:
                    hdg_off = abs(((state.heading_true or 0.0)
                                   - self.state.runway_heading + 540.0)
                                  % 360.0 - 180.0)
                    off_side = (abs(cl["offset_m"])
                                > self._runway_half_m() + 10.0)
                    if hdg_off > 45.0 or off_side:
                        self.state.rejected = True
                        self._emit("error",
                                   "Rejected takeoff: "
                                   + ("off the side of the runway"
                                      if off_side else
                                      "could not keep runway heading"),
                                   "autonomous.rejected_direction")
                        self._goto_sub("abort")
                        return
                # The runway's own yardstick, from the point the roll
                # began: past the far threshold on the wheels is an
                # overrun whatever the clock said, and more than half
                # the available pavement gone without seventy per cent
                # of the rotate speed is the takeoff the books stop,
                # not continue -- the 70/50 rule a crew keeps for a
                # day the numbers do not feel right. Both stops are
                # V1's: past the decision speed the takeoff is continued
                # even off the end -- the books' answer to a reject that
                # can no longer fit in the pavement.
                length_m = (self.state.runway_length_ft or 0.0) * 0.3048
                cleared_m = float(getattr(
                    getattr(self, "session", None),
                    "departure_remaining_m", 0.0) or 0.0)
                if (length_m > 0.0 or cleared_m > 0.0) and state.on_ground:
                    along = cl["along_m"]
                    if getattr(self, "_roll_stamp", None) \
                            != self.state.sub_entered_at:
                        self._roll_stamp = self.state.sub_entered_at
                        self._roll_along0 = along
                        # The strip this roll may use is the shorter of
                        # the pavement ahead and the distance the
                        # clearance was read back for -- an intersection
                        # departure joins short of the chart's full
                        # length, and its own figure is the one the
                        # tower's "eight hundred metres remaining"
                        # promised.
                        run = (length_m - along
                               if length_m > 0.0 else float("inf"))
                        self._roll_run0 = (min(run, cleared_m)
                                           if cleared_m > 0.0 else run)
                    run0 = self._roll_run0 or 0.0
                    # ``end`` is infinite only where neither the chart
                    # nor the clearance named a distance -- a stamp
                    # taken already past the pavement makes it lie
                    # behind the aeroplane, and the check still answers.
                    end = self._roll_along0 + run0
                    self._roll_along = along - self._roll_along0
                    self._roll_left = end - along
                    if along >= end:
                        if ias >= v1:
                            # Past V1 the stop no longer fits: the
                            # takeoff is continued and the aeroplane
                            # flown off the pavement that is left, the
                            # way the books have it.
                            self._goto_sub("rotate")
                            self.state.rotate_at = now
                            return
                        self.state.rejected = True
                        self._emit("error",
                                   "Rejected takeoff: past the end of "
                                   "the runway",
                                   "autonomous.rejected_distance")
                        self._goto_sub("abort")
                        return
                    if 0.0 < run0 < float("inf") \
                            and self._roll_along > 0.5 * run0 \
                            and ias < v1 and ias < 0.7 * rotate_at:
                        self.state.rejected = True
                        self._emit("error",
                                   "Rejected takeoff: under rotation "
                                   "speed at the midpoint of the "
                                   "available runway",
                                   "autonomous.rejected_distance")
                        self._goto_sub("abort")
                        return
            keep = self._ground_directional(
                state, keep_hdg, gs, rate_cap=5.0, pedal_max=0.8,
                # The measured rate answers two beats late on the roll:
                # subtracting it whole pumped the ~eight-second weave
                # every rejected roll logged, and the troughs were the
                # ground the left veer was gained on. A quarter of it
                # is enough of the brake; the error's own share carries
                # the standing correction a weak rudder keeps owing. It
                # arrives at half the rollout's proportional gain: a roll
                # corrects smoothly, where the larger share made each
                # centreline correction the next swerve's push.
                rate_damp=0.25, err_share=0.5, err_gain=0.01)
            # The right foot a prop takeoff is flown with: the engine's
            # left-turning moment is the roll's standing load, led
            # before the heading can leave -- a damper that waits for
            # the error spends the error first, and the capped share it
            # was spending was half the foot the model wanted.
            keep = max(-1.0, min(1.0, keep + self._right_foot(state)
                                 * self._pedal_dir()))
            self._roll_demand = keep
            d.rudder(keep)
            d.steering(keep * _nws_fade(gs))
            # The yoke into the wind, eased off as the speed builds --
            # the roll's crosswind aileron the AFH teaches, and on a
            # gusty day as lively as the gusts themselves.
            d.aileron(self._roll_aileron(state, gs, takeoff=True))
            self._roll_note(state, ias, rotate_at, "roll")
            if ias < v1 and self._runway_blocked():
                # Somebody on the strip below V1 is a stop, not a
                # continued takeoff -- the same rule the engine gets.
                self.state.rejected = True
                self._emit("error",
                           "Rejected takeoff: traffic on the runway",
                           "autonomous.rejected_runway")
                self._goto_sub("abort")
                return
            if ias < v1 and getattr(
                        self.session, "takeoff_cancelled", False):
                # The tower's own reject: "cancel takeoff clearance" is
                # obeyed the way a crew's is -- a stop while there is
                # still speed to spare. Past V1 the aeroplane flies and
                # the cancel lands the way a real one does, too late.
                self.state.rejected = True
                self._emit("error",
                           "Rejected takeoff: clearance cancelled",
                           "autonomous.rejected_cancelled")
                self._goto_sub("abort")
                return
            engines_out, _ = self._engines_out(state)
            if engines_out and ias < v1:
                # An engine that quits below V1 is a stop, not a
                # continued takeoff; past it the aeroplane flies.
                self.state.rejected = True
                self._emit("error",
                           "Rejected takeoff: engine failure below V1",
                           "autonomous.rejected_engine")
                self._goto_sub("abort")
                return
            # The rotate is an airspeed's, not the optimistic figure the
            # aborts are measured with: lift is made of air, and a
            # groundspeed that leads the pitot through the spool --
            # tailwind, a lagging system -- pulls the aeroplane into
            # the air a dozen knots under the speed its wing flies at,
            # which is a balloon and a settle, not a liftoff. A pitot
            # that reads nothing at all is the exception: ground speed
            # is the only speed there is, so it is the one used.
            indicated = float(state.indicated_airspeed_kt or 0.0)
            flying_speed = (indicated if indicated >= 8.0
                            else float(state.ground_speed_kt or 0.0))
            if flying_speed >= rotate_at:
                self._goto_sub("rotate")
                self.state.rotate_at = now
            else:
                # A roll that will not accelerate is said about once and
                # rolled on -- the aborts above, not a clock, are what
                # end it.
                self._phase_overdue(
                    "roll", self._sub_age(), self.TAKEOFF_TIMEOUT,
                    "The takeoff roll is slow to accelerate; rolling on")
            return

        if sub == "rotate":
            if self._airborne_confirmed(state, now):
                # The attitude that actually flew the wheels off is
                # this model's liftoff attitude -- written down, so the
                # next rotation asks for it rather than only for the
                # class's figure. A tail-scraper and a floater file
                # different numbers.
                adapt = getattr(self, "_adapt", None)
                pitch = float(getattr(state, "pitch_deg", 0.0) or 0.0)
                if adapt is not None and pitch > 0.0:
                    adapt.learn("rotate.pitch", pitch, blend=0.3)
                # The liftoff clock the initial climb is timed from:
                # the roll's own path already stamps it where it fired.
                self.state.airborne_at = self.state.airborne_at or now
                # The roll's into-wind aileron is a ground answer --
                # in the air a crossed yoke only banks it.
                d.aileron(0.0)
                self._goto_sub("initial_climb")
                return
            # Still on the wheels the pavement budget still applies,
            # V1 still the line: below it an aeroplane reaching the end
            # lowers the nose and stops; past it the takeoff is
            # continued -- the pull into the fence a reject there would
            # buy is the accident the rule exists for.
            if state.on_ground and self.state.runway_heading > 0 \
                    and self.state.runway_lat:
                cl = auto_nav.align(
                    state.latitude, state.longitude,
                    self.state.runway_lat, self.state.runway_lon,
                    self.state.runway_heading, lookahead_m=200.0)
                run0 = getattr(self, "_roll_run0", 0.0) or 0.0
                end = (getattr(self, "_roll_along0", 0.0) + run0
                       if 0.0 < run0 < float("inf") else 0.0)
                self._roll_along = cl["along_m"] \
                    - getattr(self, "_roll_along0", 0.0)
                self._roll_left = end - cl["along_m"] if end else 0.0
                ias_now = state.indicated_airspeed_kt or 0.0
                v1_now = (self.state.v1_kt
                          or (self.state.rotate_speed_kt or 0.0) - 10.0)
                if end > 0.0 and cl["along_m"] >= end and ias_now < v1_now:
                    d.elevator(0.0)
                    self.state.rejected = True
                    self._emit("error",
                               "Rejected takeoff: past the end of the "
                               "runway", "autonomous.rejected_distance")
                    self._goto_sub("abort")
                    return
            # Pull to the rotation attitude and hold it until the wheels
            # are up. ELEVATOR_SET is proportional, so the FO can fly the
            # rotation the way a pilot does: a firm pull, then hands off.
            elapsed = now - self.state.rotate_at
            vs_now = float(state.vertical_speed_fpm or 0.0)
            pitch = float(getattr(state, "pitch_deg", 0.0) or 0.0)
            if self._use_pids:
                # The loop closes on the rate coming alive -- the cue a
                # pilot's hand works off -- asking for a positive rate and
                # damping the answer against the one the model reports.
                pull = self._pid("rotation").update(350.0 - vs_now,
                                                    measurement=vs_now)
            else:
                # Without the loop the pull is the attitude gap's: the
                # yoke answers how far the nose still has to come, easing
                # itself off as the attitude arrives the way a hand does.
                # A fixed fraction guesses the model's elevator; this asks
                # for what the nose actually needs -- a weak-elevator
                # model gets a real pull, a touchy one gets the taper.
                pull_scale = (1.0
                              if self.performance.turn_rate < fly.RATE_LIGHT
                              else 0.8)
                err_deg = max(0.0, self._pitch_target_deg() - pitch)
                pull = min(1.0, 0.10 * err_deg) * pull_scale
                # A pilot stops pulling as the aeroplane comes unstuck: the
                # vertical speed coming alive is the cue, the way a hand
                # comes off the yoke once the nose stops needing help.
                if vs_now > 300.0:
                    pull = min(pull, 0.10 * pull_scale)
            # The attitude, not just the rate: at the rotation attitude
            # the pull stops, the way a pilot's hand comes off once the
            # nose sits at the mark. A model that never publishes pitch
            # answers zero and the rate cue above carries it.
            if pitch >= self._pitch_target_deg():
                pull = min(pull, 0.05)
            # The load watch's ceiling: a rotation that would pull past
            # the limit pulls to it instead.
            ias = state.indicated_airspeed_kt or 0.0
            v1 = self.state.v1_kt or (self.state.rotate_speed_kt - 10.0)
            if int(getattr(state, "engine_count", 0) or 0) == 1:
                v1 = 9999.0
            late = elapsed > 8.0
            if late and ias >= v1:
                # Past V1 and slow to unstick: the stop is no longer on
                # the runway that is left, so the takeoff is continued --
                # a firmer pull as the speed builds, the way a heavy or
                # nose-heavy aeroplane is flown off, not a high-speed
                # abort into the overrun.
                pull = max(pull, min(0.65, 0.35 + 0.03 * (elapsed - 8.0)))
            self._roll_note(state, ias,
                            self.state.rotate_speed_kt or 0.0, "rotate")
            d.elevator(self._g_pull_cap(state, pull))
            d.takeoff_power()
            if late and ias < v1:
                # Rotated and still on the ground below V1 with the
                # speed gone (or nothing answering at all): the model
                # is not flying itself off; lower the nose and reject
                # rather than porpoise down the runway. Past V1 the
                # stop is no longer the answer -- the takeoff is
                # continued, which is what V1 means.
                d.elevator(0.0)
                self.state.rejected = True
                self._goto_sub("abort")
            return

        if sub == "abort":
            # Rejected takeoff, stopped the way the QRH has it and not
            # the way a taxiing stop is made: levers closed and the
            # autothrottle out, the boards up, maximum wheel braking
            # with reverse while it still helps, and the centreline
            # held the whole way down -- a drift off the strip during
            # the stop is the overrun the reject was called to prevent.
            d.throttle_cut()
            d.throttle_idle()
            d.autopilot(False)
            gs = state.ground_speed_kt or 0.0
            if gs > 1.0:
                # The tower is told while the aeroplane is still
                # slowing: a crew reports the reject as soon as it is
                # under control, not when the paperwork is done.
                if not self.state.stopping_call:
                    self.state.stopping_call = self._fo_say("stopping")
                d.parking_brake(False)
                d.spoilers(1.0)
                rudder = 0.0
                if self.state.runway_heading > 0:
                    rudder = self._ground_directional(
                        state, self._centreline_heading(state, 6.0), gs,
                        rate_cap=4.0)
                    d.rudder(rudder)
                    d.steering(rudder * _nws_fade(gs))
                if (self.performance.cruise_kt >= 180.0
                        and capabilities(
                            getattr(state, "source", "")).flight_axes):
                    # The event is a toggle: the flag only moves when the
                    # send is accepted, or a refused one leaves the flag
                    # and the buckets a stroke apart for the rest of the
                    # roll.
                    if gs > 60.0 and not self.state.reversers_out:
                        self.state.reversers_out = bool(
                            d.send(b"THROTTLE_REVERSE_THRUST_TOGGLE", 0))
                    elif gs <= 60.0 and self.state.reversers_out:
                        self.state.reversers_out = not bool(
                            d.send(b"THROTTLE_REVERSE_THRUST_TOGGLE", 0))
                # Maximum braking until the aeroplane is stopped on the
                # runway; above sixty the reverse does the work, so the
                # wheels are eased while it is in.
                brake = 0.7 if gs > 60.0 else 1.0
                from .. import tailwheel
                tail = tailwheel.is_tailwheel(state)
                castor = tailwheel.is_castoring(state)
                if not tail and not castor:
                    d.brakes(brake)
                else:
                    # The ground loop's invitation is a stomp on a
                    # taildragger: capped the way the rollout caps it.
                    brake = min(brake, 0.45) if tail else brake
                    add_left, add_right = tailwheel.brake_assist(rudder, gs)
                    d.brakes_split(min(1.0, brake + add_left),
                                   min(1.0, brake + add_right))
                return
            # Stopped: the boards and the reverse go away before the
            # hold -- a crew does not sit on the strip with the
            # speedbrakes out.
            d.spoilers(0.0)
            d.spoilers_arm(False)
            if self.state.reversers_out:
                self.state.reversers_out = not bool(
                    d.send(b"THROTTLE_REVERSE_THRUST_TOGGLE", 0))
            self._stop(state)
            if not self.state.abort_announced:
                self.state.abort_announced = True
                self._emit("info",
                           "Rejected takeoff: holding on the runway",
                           "autonomous.rejected_done")
            self._goto_sub("await")
            return

        if sub == "await":
            # Powered hold where the roll stopped: brakes on, levers
            # idle, waiting on the tower or the pilot to take it. A taxi
            # instruction is the way off -- the runway is vacated along
            # whatever route it clears. The "stopping" call is still
            # owed if the frequency never let it out on the way down.
            self._stop(state)
            if not self.state.stopping_call:
                self.state.stopping_call = self._fo_say("stopping")
            if getattr(self.session, "taxi_path", None):
                if self.session.phase == Phase.TAXI_IN:
                    self._transition(AutonomousPhase.TAXI_IN)
                    return
                if self.session.phase == Phase.TAXI_OUT:
                    self._transition(AutonomousPhase.TAXI_OUT)
                    return
                # A route still on the session from the taxi out is not
                # the way off -- the tower's phase never left the roll's
                # for it, so the hold falls through to the reject reset
                # below rather than wait on a route nobody gave.
            # Nobody's route came -- a rejected takeoff does not park
            # the aeroplane mid-strip. Once it has been held a breath,
            # the reject is worked off like a crew works it: the
            # clearance is gone, the lineup starts over, and the tower
            # hears the ready call again -- the next roll is a fresh
            # clearance's, not a latch on the rejected one.
            if (state.ground_speed_kt or 0.0) <= 1.0 \
                    and self._sub_age() > 10.0:
                self.state.rejected = False
                self.state.abort_announced = False
                self.state.stopping_call = False
                self.state.aligned = False
                self.session.cleared_takeoff = False
                self.session.takeoff_cancelled = False
                # The roll's clearance was spent on the roll that was
                # refused: the session's TAKEOFF would be read back as
                # the clearance it came with, and the next lineup would
                # roll on a word nobody re-gave. The tower's book says
                # what is true -- an aeroplane that stopped on the strip
                # -- until a fresh clearance or a vacate route moves it.
                self.session.phase = Phase.LINED_UP
                self._transition(AutonomousPhase.LINE_UP)
                self._goto_sub("align")
            return

        if sub == "initial_climb":
            # Positive rate: flaps and gear come up on schedule, the climb
            # speed is reached, then the AP takes the leg. Until it does
            # the leg is hand-flown the way the go-around's first seconds
            # are: a sub that issued nothing left the rotation's pull
            # latched on the stick -- the aeroplane ballooned, bled its
            # speed and stalled -- and the levers free to be pulled back
            # to idle by anything else that could reach them.
            agl = state.altitude_agl_ft or 0.0
            ias = state.indicated_airspeed_kt or 0.0
            elapsed = now - (self.state.airborne_at or now)
            # The roll's steering demands are the runway's, not the
            # sky's: a pedal or a tiller still commanded past liftoff is
            # a skid at a hundred feet.
            d.rudder(0.0)
            d.steering(0.0)
            self._set_takeoff_power(state)
            # The roll is flown too: the rotation neutralised the yoke
            # and until this nothing held a wing -- torque, a gust or a
            # heavy tank put twenty-five degrees of bank in at forty
            # feet, and the lift that bank took off the top is the
            # descent into the trees off the end. Wings level toward
            # the runway's heading, the departure every pilot holds
            # until there is height to turn with.
            bank = float(getattr(state, "bank_deg", 0.0) or 0.0)
            runway_hdg = float(getattr(self.state, "runway_heading", 0.0)
                               or 0.0)
            hdg_err = (((runway_hdg
                        - float(getattr(state, "heading_true", 0.0)
                                or 0.0))
                        + 180.0) % 360.0) - 180.0 \
                if runway_hdg > 0.0 else 0.0
            bank_want = max(-12.0, min(12.0, 0.3 * hdg_err))
            d.aileron(max(-0.7, min(0.7,
                                    (bank_want - bank) / 30.0 * 0.8)))
            # The climb attitude is flown, not left where the rotation
            # put it: held while the rate builds, eased once the VSI
            # agrees the aeroplane is climbing -- and the nose is lowered
            # outright under the safe speed, because a climb the wing
            # cannot pay for is a stall, not a climb.
            vs_now = float(state.vertical_speed_fpm or 0.0)
            pitch = float(getattr(state, "pitch_deg", 0.0) or 0.0)
            floor = self.state.v2_kt or self.state.vr_kt or 0.0
            if floor and ias < floor - 5.0 or vs_now < -100.0:
                pull = -0.10
            elif pitch >= self._pitch_target_deg():
                pull = 0.05
            elif vs_now < 400.0:
                pull = 0.25
            else:
                pull = 0.10
            d.elevator(self._g_pull_cap(state, pull))
            # Flaps up once safely airborne and fast -- except through a
            # windshear escape, where the book's rule is to hold the
            # configuration until clear of the shear.
            shear = (self.state.windshear_since
                     and now - self.state.windshear_since < 15.0)
            # Flaps come up no lower than the 400 ft AGL the books give
            # for the first configuration change -- speed alone is not
            # the gate, a low fast aeroplane is not ready either.
            if state.flaps_index and ias > self._flap_up_kt() \
                    and not shear and agl > 400.0:
                d.flaps(0.0)
            # Gear up after a positive rate, where the model has one.
            # AGL alone is not the rate: an aeroplane settling back is
            # kept on its wheels until the VSI agrees it is climbing.
            if state.gear_down and agl > 200.0 and not shear \
                    and vs_now > 0.0:
                d.gear(False)
            # Checklist and AP engagement once stable. The yoke goes
            # neutral first: an AP handed a stick still pulled from the
            # rotation fights the climb it is being asked to fly.
            if agl > 400.0 or elapsed > 25.0:
                d.elevator(0.0)
                self._run_checklist_for_phase("after_takeoff")
                self._engage_climb(state)
                self._transition(AutonomousPhase.CLIMB)
            return

    def _phase_climb(self, state, now: float) -> None:
        """Climb to the assigned or planned level."""
        target = self._altitude_target(state)
        alt = self._flown_altitude_ft(state)
        # The initial-climb window is the flaps' scheduled moment, not
        # their only one: a climb phase that got there dirty -- too slow
        # for the retract before the AP took over -- still comes clean,
        # on the same speed test the window used.
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        shear = (self.state.windshear_since
                 and now - self.state.windshear_since < 15.0)
        if state.flaps_index and ias > self._flap_up_kt() and not shear:
            self.director.flaps(0.0)
        if not target:
            # Engaged in the air with no level anywhere: a crew levels
            # where it is, it does not climb at a zero bug for an hour.
            target = alt
        # The filed profile's next crossing level caps the climb the
        # way the paperwork means it: a plan that files a fix below the
        # cruise is crossed there, not stepped over. It holds while the
        # fix is still ahead -- the small slack is the level-off at it,
        # which is the plan flown rather than the constraint released --
        # and an assigned level beats the paperwork altogether.
        filed, stage = self._route_level(state)
        if (filed and stage != "DSC"
                and not self.session.assigned_altitude_ft
                and alt <= filed + 300.0 and filed < target):
            target = filed
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
            # No SID to walk and no heading assigned is not "wings
            # free": the filed route is the lateral plan until ATC says
            # otherwise -- the sim's NAV where it can be trusted with
            # the plan, the bearing to the active fix on the bug where
            # it cannot.
            self._apply_flight_director(
                state, altitude=target, vs_fpm=vs,
                heading=self._route_lateral(state, now),
                speed_kt=self._speed_target(state))

        # A descent or approach clearance outranks the machine's own top
        # of climb: it is taken first, not after a tick spent thinking
        # the cruise had begun.
        if self.session.phase == Phase.DESCENT:
            self._transition(AutonomousPhase.DESCENT)
            return
        if self.session.phase == Phase.APPROACH:
            self._transition(AutonomousPhase.APPROACH)
            return

        # Level off at the top of the climb.
        if target and abs(alt - target) < 200 \
                and abs(state.vertical_speed_fpm or 0.0) < 200:
            self.state.top_of_climb = True
            self.director.alt_hold(True)
            self.director.vs_hold(False)
            self._level_off_at_cruise(state)
            self._transition(AutonomousPhase.CRUISE)
            return

        self._phase_overdue(
            "climb", now - self.state.phase_entered_at, self.CLIMB_TIMEOUT,
            "The climb is taking a while; still climbing")

    def _phase_cruise(self, state, now: float) -> None:
        """Cruise: NAV on the route, fuel watched, clearances followed."""
        self._run_checklist_for_phase("cruise")
        # A flap out at cruise is a climb leftover, not a cruise
        # configuration: the same test the climb ran, applied here so a
        # climb under the gate does not fly the leg dirty.
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        shear = (self.state.windshear_since
                 and now - self.state.windshear_since < 15.0)
        if getattr(state, "flaps_index", 0) and ias > self._flap_up_kt() \
                and not shear:
            self._set_flaps(state, 0.0)
        sid = self._sid_nav(state)
        if self.state.emergency_descent:
            # An emergency descent does not wait for the plan's top of
            # descent: the boards out, the rate doubled, straight down
            # toward the divert's height. NAV flies the replanned route --
            # the leg now goes where the emergency is being taken.
            self.director.spoilers(0.5)
            self._apply_flight_director(
                state, altitude=self._altitude_target(state),
                vs_fpm=-min(self.performance.descend_fpm * 2.0, 5000.0),
                heading=(self.session.assigned_heading
                         if self.session.assigned_heading is not None
                         else self._route_lateral(state, now)),
                speed_kt=self._speed_target(state))
        elif sid is not None:
            self._fly_procedure(state, sid)
        else:
            vector = self.session.assigned_heading
            route_hdg = None
            if vector is None:
                self._fms_departure_direct(state)
                route_hdg = self._route_lateral(state, now)
            self.director.airspeed_hold(True)
            target = self._altitude_target(state)
            alt = self._flown_altitude_ft(state)
            # The filed profile's next crossing level, while the fix
            # is still ahead and ATC has assigned nothing: a filed
            # crossing below the target is held until the fix passes;
            # a filed step above it is written back as the session's
            # level -- it outlives the fix the way a clearance would
            # (RouteWatch._plan_step latches the same field). A
            # descent-stage fix is the descent phase's number.
            filed, stage = self._route_level(state)
            if (filed and stage != "DSC"
                    and not self.session.assigned_altitude_ft):
                if filed < target and alt <= filed + 300.0:
                    target = filed
                elif filed > target + 500.0:
                    self.session.cruise_altitude_ft = int(filed)
                    target = self._under_vfr_ceiling(filed, state)
            if target and abs(alt - target) > 200.0:
                # A step clearance answered level is not answered at
                # all: the bug moves and the aeroplane climbs or
                # descends to it, FLCH where the model has one. The band
                # is the deviation watch's own two hundred feet: a drift
                # the controller would call "verify altitude" on is one
                # the aeroplane corrects first.
                vs = (self.performance.climb_fpm if alt < target
                      else -self.performance.descend_fpm)
                self._apply_flight_director(
                    state, altitude=target, vs_fpm=vs,
                    heading=(self.session.assigned_heading
                             if self.session.assigned_heading is not None
                             else route_hdg),
                    speed_kt=self._speed_target(state))
            else:
                # No assigned or planned level is not an instruction to
                # find the sea: a zero on the bug is a descent to it.
                if target:
                    self.director.altitude(target)
                self.director.alt_hold(True)
                if vector is not None:
                    # A vector given in level cruise is flown like one
                    # given in the climb: NAV kept on the route with a
                    # heading read back is the readback not flown.
                    self.director.heading(vector)
                    self.state.ap_hdg_mode = True
                elif route_hdg is not None and route_hdg >= 0.0:
                    # Level on the plan is where a dead course is most
                    # quietly wrong: the route's own heading -- the
                    # watchdog's fallback where NAV steers, the bearing
                    # to the active fix where it does not -- keeps the
                    # aeroplane pointed at the leg until the track
                    # comes round.
                    self.director.heading(route_hdg)
                    self.state.ap_hdg_mode = True
            self._manage_throttle(state, self._speed_target(state))
            self.director.speed(self._speed_target(state))

        if self.session.phase == Phase.DESCENT:
            self._transition(AutonomousPhase.DESCENT)
        elif self.session.phase == Phase.APPROACH:
            self._transition(AutonomousPhase.APPROACH)
        else:
            # Nobody is coming to say "descend" on a quiet frequency, so
            # the aeroplane starts down at its own top of descent: the
            # three-degree path the descent plans from, computed here
            # the same way. An assigned level still stands -- the
            # descent's target is that level, so a controller keeping
            # the flight high only moves where it levels, not whether
            # the phase runs.
            if not self.state.tod_nm:
                self._plan_tod(state)
            distance = self._distance_to_destination_nm(state)
            if distance is not None and self.state.tod_nm > 0.0 \
                    and distance <= self.state.tod_nm:
                self._transition(AutonomousPhase.DESCENT)
                return

    def _phase_descent(self, state, now: float) -> None:
        """Descend: TOD planning, configuration, approach capture."""
        self._run_checklist_for_phase("descent")
        target = self.session.assigned_altitude_ft or self._descent_floor(state)
        hold = getattr(self.session, "hold", None)
        if hold is not None and not self.session.assigned_altitude_ft \
                and hold.altitude_ft:
            # "Hold over the fix, maintain": the level the hold was
            # given at is the level it is flown at, not the floor of a
            # descent the hold exists to delay.
            target = float(hold.altitude_ft)
        # The filed profile's next crossing level is a floor too: a
        # plan that files a fix at twelve thousand is not descended
        # through it, whatever pattern height says the floor is. It
        # holds while the fix is still ahead -- leveling at it does not
        # release it -- and an assigned level beats the paperwork.
        if not self.session.assigned_altitude_ft:
            filed, _stage = self._route_level(state)
            if filed and target < filed <= (state.altitude_ft or 0.0) + 300.0:
                target = filed

        # Top of descent: planned once from the distance to the field,
        # ~318 ft per NM on a three-degree path plus the height to lose.
        if not self.state.tod_nm:
            self._plan_tod(state)

        distance = self._distance_to_destination_nm(state)
        agl = (state.altitude_ft or 0.0) - self._field_elevation(state)

        # Configuration schedule, by the approach being made and the
        # speed under each notch's limit -- the book's numbers, not a
        # descent altitude. A Cessna's first notch is a pattern-height,
        # under-110 affair and its 20 and 30 are under-85; a jet's
        # slats meet it inside the terminal area, every stage on speed.
        # Early extension is drag the descent never asked for; fast
        # extension bends the tracks -- the white arc is the gate, not
        # the calendar.
        on_final = distance is not None and distance < 5.0
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        lim = self._limits(state)
        first_agl = (10_000.0 if self.performance.cruise_kt >= 230.0
                     else 5_000.0 if self.performance.cruise_kt >= 140.0
                     else 2_500.0)
        vfe_late = min(lim["flap_max_kt"],
                       (self.performance.approach_kt or 90.0) + 15.0)
        # A flap the climb's own test never took in is a leftover, not
        # the schedule's first stage: it comes up on the same test until
        # the extension gates below own the lever.
        shear = (self.state.windshear_since
                 and now - self.state.windshear_since < 15.0)
        if self.state.configured == "" \
                and getattr(state, "flaps_index", 0) \
                and ias > self._flap_up_kt() and not shear \
                and not (agl < first_agl
                         or (distance is not None and distance < 12.0)):
            self._set_flaps(state, 0.0)
        if self.state.configured == "" and ias <= lim["flap_max_kt"] \
                and (agl < first_agl
                     or (distance is not None and distance < 12.0)):
            self._set_flaps(state, 0.25)
            self.state.configured = "flaps_1"
        if (agl < 2200 or (distance is not None and distance < 6.0)) \
                and self.state.configured in ("", "flaps_1") \
                and ias <= vfe_late:
            self._set_flaps(state, 0.5)
            self.state.configured = "flaps_approach"
            if self._gear_allowed(state):
                self.director.gear(True)
            self.director.spoilers_arm(True)
        if (agl < 1500 or on_final) \
                and self.state.configured != "flaps_final" \
                and ias <= vfe_late:
            self._set_flaps(state, self.performance.landing_flaps or 1.0)
            self.state.configured = "flaps_final"
            if state.gear_down is not True and self._gear_allowed(state):
                self.director.gear(True)
            self.director.spoilers_arm(True)

        # Speed: assigned, else a sensible step-down -- the type's
        # descent figure above three thousand (the approach speed held
        # for the whole descent was the same Vy-for-a-leg bug as the
        # cruise: a Baron coming down at 95 kt), approach speed inside
        # it, Vapp on the final. Below ten thousand nothing fast enough
        # to matter goes over two-fifty.
        descent = max(self.performance.approach_kt,
                      0.85 * self._cruise_speed_kt(state))
        if self.performance.cruise_kt >= 200.0 \
                and 0.0 < float(getattr(state, "altitude_ft", 0.0)
                                or 0.0) < 10_000.0:
            descent = min(descent, 250.0)
        speed = self.session.assigned_speed_kt or (
            self._vapp(state) if on_final
            else (self._vapp(state)) + 20.0
            if agl < 3000 else descent)
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
            # descend through an assigned level that sits above it. A
            # hard level-off band is what it is not: stopping two
            # hundred and fifty feet early leaves the aeroplane parked
            # above the bug, right where the deviation watch lives, and
            # the verify-altitude calls start again. Inside the band the
            # rate is proportional to the error, so the level is
            # captured on the bug rather than at its edge -- and a
            # porpoise has nothing to swing about.
            # The continuous descent, the way the arrival is actually
            # flown: not the book rate down to the floor and a level
            # wait for the field -- a dive and a drive -- but the
            # three-degree path to the threshold, started where the
            # plan's own top of descent put it. Steep to recapture the
            # path from above, the level it has sunk to held while the
            # path drops back to it, and the plan's next crossing
            # composed into the path as "at or above" rather than
            # levelled at miles early. The cases it is not for: an
            # assigned level to capture, a hold being held, an
            # emergency descent, and no position to draw the path on.
            alt = self._flown_altitude_ft(state)
            required = target
            cda = distance is not None and hold is None \
                and not self.session.assigned_altitude_ft \
                and not self.state.emergency_descent
            if cda:
                required = max(
                    required,
                    self._field_elevation(state) + approach_flight.TCH_FT
                    + distance * approach_flight.FT_PER_NM)
                filed_level, filed_nm, _stage = self._route_level_ahead(
                    state)
                if filed_level and filed_level <= alt + 300.0:
                    required = max(
                        required,
                        filed_level + filed_nm * approach_flight.FT_PER_NM)
            err = alt - required
            if err > 250.0:
                vs = -descent_fpm
            elif err < -250.0:
                # Below the level itself is the breach; below only the
                # path's line is a gentle drift back up to it, not a
                # climb at the book's rate.
                vs = (self.performance.climb_fpm
                      if alt < target - 250.0 else 300.0)
            elif required > target:
                # On the path: the rate that follows it down -- the
                # groundspeed's own three-degree figure, the way the
                # VNAV number is made -- trimmed for the error the way
                # a hand on the throttle trims the descent.
                gs = float(getattr(state, "ground_speed_kt", 0.0)
                           or 0.0) or float(
                               getattr(state, "indicated_airspeed_kt",
                                       0.0) or 0.0)
                sink = min(max(gs * approach_flight.FT_PER_NM / 60.0,
                               100.0), descent_fpm)
                vs = max(-descent_fpm, min(300.0, -sink - err * 4.0))
            else:
                vs = max(-300.0, min(300.0, -err * 4.0))
            if assigned_hdg is None:
                # A vector that has ended is not a heading kept for
                # ever: with nothing assigned the route is the lateral
                # plan again, the same ask the climb and the cruise
                # make.
                route_hdg = self._route_lateral(state, now)
            else:
                route_hdg = assigned_hdg
            # VS owns the axis for the whole descent, not just the
            # path's steep part: managed level-change would pitch for
            # speed at the preselect below and the path is never
            # flown -- the flicker the mode asks for is dodged by
            # giving the descent to the rate hold outright.
            self._apply_flight_director(
                state, altitude=target, vs_fpm=vs,
                heading=route_hdg,
                speed_kt=speed, prefer_vs=cda)

        # The approach's own navaid, tuned early enough that the needles
        # are alive by the time the final is flown.
        self._maybe_tune_nav(state, self._approach_recipe(state), now)

        # When the approach phase owns it, or we are close enough that the
        # approach logic should be flying.
        if self.session.phase == Phase.APPROACH or (
                distance is not None and distance < 12.0 and agl < 4000):
            self._transition(AutonomousPhase.APPROACH)
            return
        self._phase_overdue(
            "descent", now - self.state.phase_entered_at, self.DESAT_TIMEOUT,
            "The descent is taking a while; still descending")

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

        if self._fly_orbit(state):
            # Round the orbit the tower gave: nothing of the final is
            # flown, or judged, until the aeroplane is back on it.
            self.state.stabilized = False
            return

        self._configure_final(state)

        # Below ten thousand the landing light is on -- the checklist
        # may have named a switch for it, and where it did not the rule
        # still stands.
        if not getattr(state, "landing_light", False):
            self.director.landing_light(True)

        # The cleared approach, as a recipe: which navaid, which course,
        # which minima. Resolved once and kept until the field or the
        # runway changes; the radio is tuned to its navaid on the way in.
        recipe = self._approach_recipe(state)
        self._maybe_tune_nav(state, recipe, now)

        # Not yet where a final can be flown from: position for it,
        # level, before anything descends toward the runway.
        if not self.state.circling and self._join_final(state, recipe):
            self.state.stabilized = False
            self._phase_overdue(
                "approach", now - self.state.phase_entered_at,
                self.APPROACH_TIMEOUT,
                "The approach is taking a while; still positioning")
            return

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
                self._fly_visual_final(state)

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
        if self.session.cleared_landing and 0.0 < agl < 500.0 \
                and not self.state.circling \
                and (self.state.stabilized or agl < 60.0):
            self._transition(AutonomousPhase.LANDING)
            self._goto_sub("flare")
            return

        if self.session.phase == Phase.LANDING or self.session.phase == Phase.LANDED:
            # The tower marks the session LANDING when the clearance is
            # issued, miles before the flare. That radio phase is not
            # itself a handoff: taken literally it abandons the approach
            # configuration gates early, the live six-mile final that
            # reached the runway with the flaps up. Only the aeroplane's
            # own height -- or the ground itself -- completes the handoff.
            agl = state.altitude_agl_ft or 0.0
            if getattr(state, "on_ground", False) or 0.0 < agl < 500.0:
                self._transition(AutonomousPhase.LANDING)
                self._goto_sub("flare")
            return

        self._phase_overdue(
            "approach", now - self.state.phase_entered_at,
            self.APPROACH_TIMEOUT,
            "The approach is taking a while; still flying it")

    def _phase_landing(self, state, now: float) -> None:
        """Flare, touchdown, rollout, vacate."""
        d = self.director
        sub = self.state.sub or "flare"

        # The flare is short final: the landing light is on -- a crew
        # does not reach the runway with it dark.
        if not getattr(state, "landing_light", False):
            d.landing_light(True)

        if sub == "flare":
            self._run_checklist_for_phase("before_landing")
            # A tower-phase handoff can reach this phase before the
            # approach's distance gates have finished configuring the
            # aeroplane. Airborne short final is still the final: keep
            # the gear-and-flap schedule running here instead of
            # freezing whatever configuration the approach happened
            # to have reached.
            if not getattr(state, "on_ground", False):
                self._configure_final(state)
            if state.on_ground:
                self.state.touchdown_at = now
                # What this flare earned is this model's next flare:
                # the sink the last airborne read showed, turned into
                # the bias the schedule's floor is trimmed by. A hard
                # arrival asks for less sink next time, a floater for
                # more -- the way a pilot adjusts the flare for the
                # aeroplane he has just put down.
                adapt = getattr(self, "_adapt", None)
                sink_td = getattr(self, "_flare_sink", 0.0)
                if adapt is not None and sink_td > 0.0:
                    needed = max(-150.0, min(150.0, 250.0 - sink_td))
                    adapt.learn("flare.bias", needed, blend=0.4)
                self._flare_sink = 0.0
                # The flare's clock belongs to this landing: a
                # touch-and-go would read it as a fifteen-second float
                # the moment the next pass sank under sixty feet.
                self.state.flare_at = 0.0
                self._flare_lever = None
                d.throttle_idle()
                d.throttle(0.0)
                self._last_lever = 0.0
                # The surfaces the flare held go neutral with the wheels
                # down: a pull left latched on the stick is a nosewheel
                # held light through the whole rollout, and the
                # into-wind wing frozen at its last answer.
                d.elevator(0.0)
                d.aileron(0.0)
                self._goto_sub("rollout")
                return
            agl = state.altitude_agl_ft or 0.0
            # Below the flare window: the autopilot's job is done -- the
            # FO flies the flare by hand, like a pilot does. A model that
            # publishes no AGL reads 0 at every height and must not be
            # flared at altitude.
            if 0.0 < agl < 60.0:
                if not self.state.flare_at:
                    self.state.flare_at = now
                    self._flare_lever = None
                    d.approach(False)
                    d.autopilot(False)
                    self.state.ap_master = False
                # The power comes off through the flare, closed by the
                # height a pilot of this class has it closed -- chopped
                # at sixty feet it is a light single dropping the last
                # of its speed onto the runway.
                idle_ft = 30.0 if self.performance.cruise_kt >= 230.0                     else 20.0
                if agl <= idle_ft:
                    lever = 0.0
                else:
                    held = getattr(self, "_flare_lever", None)
                    if held is None:
                        held = self._flare_lever = min(
                            0.6, self._last_lever or 0.0)
                    lever = held * (agl - idle_ft) / (60.0 - idle_ft)
                if lever <= 0.0:
                    d.throttle_idle()
                d.throttle(lever)
                self._last_lever = lever
                sink = abs(state.vertical_speed_fpm or 0.0)
                self._flare_sink = sink
                if self._use_pids:
                    # A real flare is a decaying sink target, not one
                    # pull: from the approach's descent rate at the flare
                    # height down to a hundred-odd feet a minute at the
                    # numbers -- trimmed by what this model's last
                    # touchdowns earned, so a model that keeps arriving
                    # hard is asked for less sink and a floater for
                    # more. The loop chases that schedule against the
                    # sink the model reports.
                    adapt = getattr(self, "_adapt", None)
                    bias = adapt.value("flare.bias", 0.0) \
                        if adapt is not None else 0.0
                    floor = max(50.0, min(250.0, 100.0 + bias))
                    want = max(floor, min(750.0, agl * 13.0))
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
                # alignment does not buy a drift across the centreline.
                if self.state.runway_heading > 0:
                    # The de-crab kick runs the same rate servo the
                    # ground steering does -- a few degrees a second of
                    # yaw answered against the authority this model has
                    # shown, not a bare proportional pedal.
                    # The crab is kept until the power comes off and
                    # kicked out below it: squared up at sixty feet the
                    # aeroplane drifts downwind for the whole flare.
                    want = self._centreline_heading(state, 3.0)
                    if agl > idle_ft:
                        want = self._into_wind(state, want)
                    d.rudder(self._ground_directional(
                        state, want,
                        state.ground_speed_kt
                        or (state.indicated_airspeed_kt or 0.0),
                        rate_cap=4.0))
                # The into-wind wing from the crab the approach is
                # wearing: nose-left-of-track is the wind from the left,
                # which names the bank's side without trusting the sim's
                # body-axis sign convention. A model that gave no drift
                # to read falls back to the reported component.
                crab = getattr(self, "_crab_deg", 0.0)
                if abs(crab) > 0.5:
                    bias = crab * 0.03
                else:
                    xwind = float(getattr(state, "aircraft_wind_x_kt", 0.0)
                                  or 0.0)
                    bias = -xwind * 0.02
                d.aileron(max(-0.15, min(0.15, bias)))
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
                # The guidance that brought it here keeps flying it:
                # the heading and the sink are still steered to the
                # centreline and the path, and the lever still answers
                # the speed -- bugs frozen at five hundred feet drift
                # off the runway on the first gust.
                self._fly_short_final(state, now)
                # The stabilized call is the approach phase's job, but
                # a landing handed over on the tower's own phase word
                # skipped that phase's assessment -- without it running
                # here the flag can never set and every final is a
                # forced go-around at two hundred feet.
                self._check_stabilized(state)
                if self._should_go_around(state):
                    self._start_go_around(state)
                    return
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
            from .. import tailwheel
            tail = tailwheel.is_tailwheel(state)
            castor = tailwheel.is_castoring(state)
            # Keep it on the centreline until slow. The rudder does the
            # holding at rollout pace while the nosewheel's share fades
            # out under it -- thrown in whole at rollout speed it is the
            # swerve a landing is lost to, not saved by. The pedal is
            # computed before the brakes are pressed: on a taildragger
            # the same demand steers the inside brake, so it has to
            # exist first.
            rudder = 0.0
            if self.state.runway_heading > 0:
                rudder = self._ground_directional(
                    state, self._centreline_heading(state, 6.0), gs,
                    rate_cap=4.0)
                d.rudder(rudder)
                d.steering(rudder * _nws_fade(gs))
            # The yoke into the wind the other way round: a touch at
            # rollout speed growing to full as the aeroplane slows and
            # the rudder's authority fades with it -- and gusts move it
            # for real, not on a schedule.
            d.aileron(self._roll_aileron(state, gs, takeoff=False))
            if tail and gs < 40.0:
                # The tail comes up with the aeroplane's speed: the
                # stick stays back as the rollout slows, pinning the
                # wheel that still steers when the rudder has stopped
                # answering.
                d.elevator(0.6)
            if gs > self._expedite_kt(state, self.performance.taxi_kt) + 5.0:
                # Reverse out to sixty knots where a fast aeroplane has
                # it -- then the wheel brakes, graduated with speed the
                # way the autobrake detents a crew actually selects are.
                # An "expedite" on the rollout raises the figure it
                # brakes to, so the exit is taken briskly rather than
                # the strip walked down at the book's taxi speed.
                d.parking_brake(False)
                if (self.performance.cruise_kt >= 180.0
                        and capabilities(state.source).flight_axes):
                    if gs > 60.0 and not self.state.reversers_out:
                        self.state.reversers_out = bool(
                            d.send(b"THROTTLE_REVERSE_THRUST_TOGGLE", 0))
                    elif gs <= 60.0 and self.state.reversers_out:
                        self.state.reversers_out = not bool(
                            d.send(b"THROTTLE_REVERSE_THRUST_TOGGLE", 0))
                # Reverse does the work above sixty; the wheel brakes
                # are eased while it is in and do the stopping as it
                # comes out -- graduated with speed the way the
                # autobrake detents a crew actually selects are.
                brake = 0.4 if gs > 60.0 else 0.7 if gs > 30.0 else 1.0
                if not tail and not castor:
                    d.brakes(brake)
                else:
                    # Full pedal on a taildragger at thirty knots is
                    # the ground loop's invitation: the cap keeps the
                    # tail down. On either castoring gear the inside
                    # wheel takes the steering the fading rudder --
                    # and a wheel that never steered -- cannot.
                    brake = min(brake, 0.45) if tail else brake
                    add_left, add_right = tailwheel.brake_assist(
                        rudder, gs)
                    d.brakes_split(min(1.0, brake + add_left),
                                   min(1.0, brake + add_right))
            else:
                # Slow enough: brakes off, the boards stowed and
                # disarmed -- a crew does not taxi to the stand with
                # the spoilers out -- taxi in.
                d.brakes(0.0)
                d.spoilers(0.0)
                d.spoilers_arm(False)
                off = self._centreline_offset_m(state)
                if not self.state.vacated and (off is None or abs(off) > 30.0):
                    self.state.vacated = True
                    # Clear of the runway: strobes and the landing light
                    # come off -- the after-landing flow a crew runs on
                    # the way out. Still on the pavement at taxi speed
                    # is a backtrack, not a vacating, so none of it is
                    # done until the centreline has been left.
                    d.strobes(False)
                    d.landing_light(False)
                    self._run_checklist_for_phase("after_landing")
                self._transition(AutonomousPhase.TAXI_IN)
                self._goto_sub("taxi")
                return
            return

    def _phase_taxi_in(self, state, now: float) -> None:
        """Taxi to the stand after landing."""
        self._run_checklist_for_phase("after_landing")
        if self.session.phase == Phase.LANDED and (
                getattr(self.session, "taxi_path", None)
                or getattr(self.session, "taxi_route", None)):
            # A path left from the departure is not a taxi-in route.
            # Until the arrival instruction lands there is nowhere valid
            # to follow: keeping the old line made the just-landed
            # aeroplane measure itself against its origin airport and
            # hold for being a hundred kilometres off that route.
            self.session.taxi_route = []
            self.session.taxi_nodes = []
            self.session.taxi_path = []
            self.session.taxi_crossings = []
            self.session.taxi_rest = []
            self.session.taxi_split_at = None
            self.session.taxi_stand = None
            self.session.hold_short_of = ""
            self.session.crossing_index = 0
            self.session.crossing_granted = ""
        # Off the runway: the tower, and after it whoever the frequency
        # belongs to, hears the vacated call it expects. It is retried
        # until it has actually gone out -- a call marked made on a
        # frequency that never let it through was the report nobody
        # heard.
        if not self.state.vacated_call and self._manage_radio \
                and self._radio_quiet():
            off = self._centreline_offset_m(state)
            if off is None or abs(off) > 30.0:
                if not self.state.vacated:
                    # The turn-off after a backtrack lands here: the
                    # rollout's branch only marks the runway left when
                    # the centreline is already behind it.
                    self.state.vacated = True
                    self.director.strobes(False)
                    self.director.landing_light(False)
                # Still on the strip is not vacated, whatever the phase
                # says -- a stopped aeroplane reporting clear is the
                # runway left blocked with nobody told.
                self.state.vacated_call = self._fo_say("vacated")
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
        elif sub == "taxi" \
                and (state.ground_speed_kt or 0.0) <= 1.0:
            # Stopped on the movement area with nowhere left to go:
            # the flight is done where it stands rather than held on
            # the taxi phase for ever. A stand route that exists but
            # never finishes -- a blocked last fifty feet, a stand the
            # path never quite reaches -- gets a longer say before the
            # same answer.
            still = now - getattr(self, "_ground_still_since", 0.0) \
                if getattr(self, "_ground_still_since", 0.0) else 0.0
            has_path = bool(getattr(self.session, "taxi_path", None))
            if self._sub_age() > 90.0 and (not has_path or still > 240.0):
                self.director.parking_brake(True)
                self._emit("info", "Parked where it stands: the stand "
                           "route never finished" if has_path else
                           "Parked where it stands: no stand route came",
                           "autonomous.taxi_in.parked")
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
            if self._any_engine_running(state):
                self._shutdown_engines()
                # A lever that stays live is asked again, not errored on:
                # the shutdown keeps cutting until the engine is out.
                self._phase_overdue(
                    "shutdown", self._sub_age(), 60.0,
                    "An engine is still running; still shutting down")
            else:
                self._goto_sub("electrics_off")

        elif sub == "electrics_off":
            self._ask_the_ramp(state)
            # The type's shutdown card owns the end of the flight. Do
            # not take the battery and avionics away while an item it
            # is still working through needs them powered.
            if not self._checklists_settled():
                return
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
            # The stand's delay is run with the engines off -- they are
            # shut at blocks in the real job, not idled through the
            # turnaround, and the fuelling below wants them cold anyway.
            if self._any_engine_running(state):
                self._shutdown_engines()
            if now - self.state.phase_entered_at < self._chain_delay:
                return
            self._goto_sub("service")

        if sub == "service":
            # Fuelling and the rest of the turnaround begin after the
            # shutdown card, not beside it: a checklist item may still
            # need the electrics the service would otherwise secure.
            if not self._checklists_settled():
                return
            if self._service_turnaround(state, now):
                self._goto_sub("load")
            return

        if sub == "load":
            self.state.chain_flight_index += 1
            self._load_next_flight(self.state.plan_flight)
            self.state.plan_flight = None
            self._transition(AutonomousPhase.PREFLIGHT)

