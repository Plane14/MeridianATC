from __future__ import annotations

from ._shared import (
    CLIMBING_FPM,
    DEPARTURE_REACHING_FT,
    LAST_STEP_FT,
    VISUAL,
    Any,
    Instruction,
    Intent,
    ParsedIntent,
    Phase,
    Reply,
    _airborne,
    _join,
    asked_for,
    decision,
    departure_step,
    departure_top,
    is_oceanic,
    math,
    on_course_fix,
    squawk_for,
    station_key,
)


class _Radar_positions_departure:
    # ------------------------------------------------------------------
    # radar positions: departure, approach, centre
    # ------------------------------------------------------------------

    # Nothing a radar controller can do anything about. Asked for any of them
    # it names whoever can, rather than listing the calls it would rather have
    # heard -- which is what a pilot on the wrong frequency got.
    _SURFACE_ONLY = (
        Intent.REQUEST_PUSHBACK, Intent.REQUEST_TAXI,
        Intent.REQUEST_TAXI_TO_PARKING, Intent.REQUEST_TAKEOFF,
        Intent.REQUEST_CLEARANCE, Intent.REQUEST_PATTERN_WORK,
    )

    def _radar(self, parsed, session, station, weather, p, state) -> Reply:
        aircraft = session.aircraft
        first_contact = not session.established_with(station)

        if parsed.intent in self._SURFACE_ONLY:
            if _airborne(state):
                if parsed.intent is Intent.REQUEST_CLEARANCE:
                    # A clearance asked in the air is the pop-up pickup:
                    # the VFR flight that needs to be IFR gets it here.
                    return self._issue_ifr_pickup(
                        session, station, p, state)
                return Reply(p.already_airborne(aircraft), station)
            sent = self._send_on(session, station, p, "TWR", "GND", "DEL")
            if sent is not None:
                return sent
            return Reply(p.already_on_the_ground(aircraft), station)

        # Radar works aeroplanes that are flying. One still on the ground is
        # not identified, not vectored and certainly not cleared for a visual
        # approach -- all of which it used to be told from the parking stand.
        if not _airborne(state):
            sent = self._send_on(session, station, p, "TWR", "GND", "DEL")
            if sent is not None:
                return sent
            return Reply(p.already_on_the_ground(aircraft), station)

        if is_oceanic(station) and (
                parsed.intent in (Intent.CHECK_IN, Intent.POSITION_REPORT,
                                  Intent.UNKNOWN)
                or "position" in parsed.normalized):
            # An HF station over the ocean has no radar: a position report
            # is taken and acknowledged, never "radar contact", never
            # "identified".
            session.establish(station)
            return Reply(p.roger(aircraft), station)

        if parsed.intent in (Intent.CHECK_IN, Intent.POSITION_REPORT):
            reply = self._radar_check_in(parsed, session, station, p, state,
                                         first_contact)
            said = parsed.normalized or ""
            deviation_reported = any(
                f"deviating {side}" in said for side in ("left", "right", "around"))
            if deviation_reported and self.realism.deviations and session.ifr:
                # A first call often carries two facts: the level check-in
                # and the off-course manoeuvre. The level answer is still
                # owed, and the safety report must not disappear inside it.
                approval = self._approve_deviation(
                    parsed, session, station, p, state)
                return Reply(reply.text + " " + approval.text, station,
                             instruction=reply.instruction or approval.instruction,
                             handoff_to=reply.handoff_to)
            return reply

        if parsed.intent in (Intent.GOING_AROUND, Intent.MISSED_APPROACH):
            # On approach as well as on the tower: a missed approach is
            # often reported to whoever the pilot is still talking to.
            return self.go_around(session, station, weather, p,
                                  session.destination
                                  or self.navdb.airport(station.ident))

        if parsed.intent is Intent.REQUEST_FLIGHT_FOLLOWING:
            return self._issue_flight_following(session, station, p, state)

        if parsed.intent is Intent.REPORT_CLEAR_OF_WEATHER:
            # The pilot's report is the end of the manoeuvre. Normally the
            # FO has already cleared its own executive state before saying
            # it; clearing it here as well keeps a typed/user report from
            # leaving the old deviation steering in charge.
            session.deviating = ""
            session.weather_deviation_side = ""
            session.weather_deviation_degrees = 0
            session.weather_deviation_miles_nm = 0
            session.weather_deviation_rejoin = ""
            session.weather_deviation_at = 0.0
            session.weather_deviation_course = -1.0
            session.weather_deviation_urgent = False
            return Reply(p.roger(session.aircraft), station)

        if parsed.intent in (Intent.REQUEST_DEVIATION, Intent.REPORT_DEVIATION) \
                and self.realism.deviations and session.ifr:
            return self._approve_deviation(parsed, session, station, p, state)

        # The pilot's other requests (the IFR plan, Phase 6): each answered
        # with a clearance, or "unable" and the reason.
        if self.realism.pilot_requests and session.ifr:
            if parsed.intent is Intent.REQUEST_SPEED:
                return self._answer_speed(parsed, session, station, p, state)
            if parsed.intent is Intent.REQUEST_HOLD and self.realism.holds:
                return self._answer_hold(session, station, p, state)
            if parsed.intent is Intent.REQUEST_RETURN:
                return self._divert(parsed, session, station, p, state,
                                    back=True)
            if parsed.intent is Intent.REQUEST_DIVERSION:
                return self._divert(parsed, session, station, p, state,
                                    back=False)
            if parsed.intent is Intent.REQUEST_DESCENT:
                early = self._descent_asked_early(session, station, p, state)
                if early is not None:
                    return early

        if parsed.intent is Intent.REQUEST_TRANSITION:
            return self._issue_transition(session, station, weather, p, parsed,
                                          state)

        # "Request flight level three three zero" names a level and not a
        # direction, and the parser, which cannot see the altimeter, files
        # it as a climb. Below the aeroplane it is a descent.
        if parsed.intent is Intent.REQUEST_CLIMB:
            asked = parsed.get("altitude_ft") or 0
            current = getattr(state, "altitude_ft", 0.0) or 0.0
            if asked and current and asked < current - 200:
                parsed.intent = Intent.REQUEST_DESCENT

        # A VFR flight is not assigned a level. Its altitude is its own affair
        # unless something is above it, so a request for one gets the approval
        # a controller actually gives rather than an IFR level clearance.
        if (not session.ifr
                and parsed.intent in (Intent.REQUEST_CLIMB,
                                      Intent.REQUEST_DESCENT)):
            asked = parsed.get("altitude_ft") or 0
            if session.vfr_ceiling_ft and asked > session.vfr_ceiling_ft:
                return Reply(
                    p.maintain_vfr(aircraft, session.vfr_ceiling_ft), station)
            text = p.vfr_altitude_discretion(aircraft)
            instruction = session.remember(
                Instruction("vfr_altitude", station_key(station), text, {},
                            readback_required=False))
            return Reply(text, station, instruction=instruction)

        if parsed.intent is Intent.REQUEST_CLIMB:
            target = parsed.get("altitude_ft") or self._next_level_up(session, state)
            # A centre's step climb. Departure and approach climb their
            # flights on a plan of their own and are left as they were.
            blocking = None
            if station.position == "CTR":
                blocking = self._climb_blocked_by(state, target)
                decision("enroute_step_climb", station=station.callsign,
                         event="step climb granted" if blocking is None
                         else "step climb refused",
                         from_ft=round(getattr(state, "altitude_ft", 0.0) or 0.0),
                         to_ft=int(target), granted=blocking is None,
                         traffic=getattr(blocking, "written", None),
                         reason=(f"to {int(target)} ft, nothing in the way"
                                 if blocking is None else
                                 f"to {int(target)} ft, {blocking.written} at "
                                 f"{int(blocking.altitude_ft)} ft inside "
                                 f"{self.STEP_CLIMB_TRAFFIC_NM:g} nm"))
            if blocking is not None:
                current = float(getattr(state, "altitude_ft", 0.0) or 0.0)
                return Reply(p.unable_climb_traffic(
                    aircraft, target, round(current / 100.0) * 100.0,
                    transition_altitude=self._transition_altitude(
                        self.navdb.airport(station.ident))), station)
            text = p.climb_maintain(aircraft, target,
                                    transition_altitude=self._transition_altitude(
                                        self.navdb.airport(station.ident)))
            instruction = session.remember(
                Instruction("altitude", station_key(station), text,
                            {"altitude_ft": int(round(target / 100.0) * 100)})
            )
            return Reply(text, station, instruction=instruction)

        if parsed.intent is Intent.REQUEST_DESCENT:
            target = parsed.get("altitude_ft") or self._next_level_down(session, state)
            current = getattr(state, "altitude_ft", 0.0) or 0.0
            if current and target >= current - 200:
                # Already at or below what was asked for. Refusing is the
                # correct answer; issuing a "descent" to a higher level is not.
                return Reply(p.unable_lower(aircraft), station)
            airport = self.navdb.airport(station.ident)
            if not parsed.get("altitude_ft"):
                target = self.out_of_the_layer(airport, weather, target)
            # Never below the floor, whoever asked for it, and issued as a
            # whole thousand.
            target = max(target, math.ceil(self.descent_floor(
                session.destination or airport, state,
                session.arrival_runway) / 1000.0) * 1000.0)
            if current and target >= current - 200:
                return Reply(p.unable_lower(aircraft), station)
            text = p.descend_maintain(
                aircraft, target,
                **self.descent_terms(
                    airport, weather, p,
                    max(current, float(session.assigned_altitude_ft or 0.0)),
                    target))
            instruction = session.remember(
                Instruction("altitude", station_key(station), text,
                            {"altitude_ft": int(round(target / 100.0) * 100)})
            )
            return Reply(text, station, instruction=instruction)

        if parsed.intent is Intent.REQUEST_DIRECT and self.plan_fixes \
                and station.position == "CTR":
            return self._direct_on_the_plan(parsed, session, station, p, state)

        if parsed.intent is Intent.REQUEST_DIRECT:
            fix = parsed.get("fix", "")
            resolved, spoken = self._resolve_fix(fix, state)
            if not resolved:
                return Reply(p.unable_unknown_fix(aircraft), station)
            text = p.direct_to(aircraft, resolved, fix_spoken=spoken)
            instruction = session.remember(
                Instruction("direct", station_key(station), text, {},
                            readback_required=False)
            )
            return Reply(text, station, instruction=instruction)

        if parsed.intent in (Intent.REQUEST_APPROACH, Intent.REQUEST_VECTORS,
                             Intent.REQUEST_LANDING):
            return self._issue_approach(session, station, weather, p, state,
                                        parsed)

        if parsed.intent is Intent.REPORT_FIELD_IN_SIGHT:
            if session.vector_leg and session.ifr and self.realism.vectors:
                # Being vectored: noted, and the visual comes with the last
                # vector onto the final (arrival.py _intercept), not here
                # in the middle of a downwind.
                session.field_in_sight = True
                session.establish(station)
                decision("vectors", event="field in sight", leg=session.vector_leg)
                return Reply(p.roger(aircraft), station)
            return self._issue_visual_approach(session, station, weather, p)

        if parsed.intent is Intent.REPORT_TRAFFIC_IN_SIGHT:
            return Reply(p.maintain_visual_separation(aircraft), station)

        if parsed.intent is Intent.REPORT_ESTABLISHED:
            tower = self._destination_tower(session, station)
            if tower and tower.mhz:
                text = p.handoff(aircraft, self._station_name(tower, p, short=True), tower.mhz)
                instruction = session.remember(
                    Instruction("handoff", station_key(station), text,
                                {"frequency": tower.mhz,
                                 "station": self._station_name(
                                     tower, p, short=True)})
                )
                return Reply(text, station, instruction=instruction, handoff_to=tower)
            return Reply(p.roger(aircraft), station)

        if parsed.intent is Intent.REQUEST_FREQUENCY_CHANGE:
            if not session.ifr and not session.flight_following \
                    and not session.radar_service \
                    and session.squawk != p.vfr_squawk:
                # A VFR flight that was given a code and no advisories: the
                # code comes back with the frequency change.
                session.squawk = p.vfr_squawk
                session.vfr_ceiling_ft = 0.0
                return Reply(p.squawk_vfr(aircraft, frequency_change=True),
                             station)
            # The end of radar advisories, and of the code that went with
            # them; left set, the traffic watch carried on calling traffic
            # to a flight that had left the frequency.
            session.flight_following = False
            session.radar_service = ""
            session.vfr_ceiling_ft = 0.0
            if not session.ifr:
                session.squawk = p.vfr_squawk
                return Reply(p.radar_service_terminated(aircraft), station)
            # IFR flights stay inside the system: the frequency change is
            # approved, the radar service does not end.
            return Reply(p.frequency_change_approved(aircraft), station)

        if parsed.intent is Intent.CANCEL_IFR:
            session.ifr = False
            return Reply(p.ifr_cancelled(aircraft), station)

        if parsed.intent is Intent.RADIO_CHECK:
            return Reply(p.radio_check(aircraft), station)

        if parsed.intent in (Intent.ACKNOWLEDGE, Intent.AFFIRMATIVE):
            return Reply("", station, silent=True)

        return self._not_understood(session, station, p, state, parsed=parsed)

    def _radar_check_in(self, parsed, session, station, p, state, first_contact) -> Reply:
        aircraft = session.aircraft
        transition = self._transition_altitude(self.navdb.airport(station.ident))

        # A check-in on departure gets radar contact and the climb the
        # clearance already promised.
        target = parsed.get("target_altitude_ft") or 0
        current = getattr(state, "altitude_ft", 0.0) or 0.0
        airborne = state is not None and not getattr(state, "on_ground", True)
        cruise = session.cruise_altitude_ft or self._default_cruise(session)

        climb_to = 0.0
        # Never issue a climb to a level the aircraft is already at: an
        # aircraft checking in level at cruise gets "radar contact" and nothing
        # more, which is what a real controller says.
        below_cruise = current <= 0 or current < cruise - 500
        if session.phase in (Phase.TAKEOFF, Phase.DEPARTURE) and below_cruise:
            session.phase = Phase.DEPARTURE
            climb_to = cruise
        elif (airborne and station.position == "DEP" and current < cruise - 1000
              and self._departing(session, state)):
            # Checking in with departure on the way up, whatever the phase
            # tracker thinks: the climb is what the aircraft is waiting for.
            # Only on the way up. Ten fields publish a Departure and no
            # Approach, the centre hands arrivals to it, and an arrival
            # checking in was climbed to cruise and lost as a departure.
            session.phase = Phase.DEPARTURE
            climb_to = cruise
        elif target and target > max(current, session.assigned_altitude_ft):
            climb_to = target

        # A VFR flight checking in is identified and told to stay VFR. It is
        # not climbed to a cruising level it never filed.
        if not session.ifr:
            # Identified: from here this room is watching the flight, and it
            # is this room that ends that (watch.RadarServiceWatch).
            session.radar_service = station_key(station)
            instruction_text = p.maintain_vfr_clause(
                session.vfr_ceiling_ft, transition_altitude=transition)
            if first_contact:
                text = p.radar_contact(
                    aircraft, self._station_name(station, p),
                    instruction=instruction_text, abbreviated=False)
            else:
                text = p.maintain_vfr(aircraft, session.vfr_ceiling_ft)
            instruction = session.remember(
                Instruction("radar_contact", station_key(station), text, {},
                            readback_required=False))
            return Reply(text, station, instruction=instruction)

        if self.realism.climb_steps and self._climbing_out(session, state,
                                                           cruise):
            return self._climbing_check_in(session, station, p, state,
                                           first_contact, transition, cruise)

        if station.position == "CTR" and first_contact and not climb_to:
            return self._centre_check_in(parsed, session, station, p, state,
                                         transition)

        instruction_text = ""
        values: dict[str, Any] = {}
        if climb_to:
            verb = "climb and maintain" if p.dialect == "faa" else "climb to"
            instruction_text = f"{verb} {p.alt(climb_to, transition)}"
            values["altitude_ft"] = int(climb_to)

        if first_contact:
            text = p.radar_contact(
                aircraft, self._station_name(station, p), instruction=instruction_text,
                abbreviated=False,
            )
        elif instruction_text:
            text = p.climb_maintain(aircraft, climb_to, transition_altitude=transition)
        else:
            text = p.roger(aircraft)

        instruction = None
        if values:
            instruction = session.remember(
                Instruction("altitude", station_key(station), text, values)
            )
        return Reply(text, station, instruction=instruction)

    def _climbing_out(self, session, state, cruise: float) -> bool:
        """An IFR flight on its way up to the cruise, wherever it is."""
        if state is None or getattr(state, "on_ground", True):
            return False
        if session.phase in (Phase.DESCENT, Phase.APPROACH, Phase.LANDING):
            return False
        here = float(getattr(state, "altitude_ft", 0.0) or 0.0)
        if not cruise or here >= cruise - 500:
            return False
        climbing = float(getattr(state, "vertical_speed_fpm", 0.0) or 0.0)             >= CLIMBING_FPM
        return session.phase in (Phase.TAKEOFF, Phase.DEPARTURE) or climbing             or self._departing(session, state)

    def _climbing_check_in(self, session, station, p, state,
                           first_contact: bool, transition: int,
                           cruise: float) -> Reply:
        """The answer to a flight checking in on its way up.

        Never "maintain" a level it is already through, and never the whole
        climb at once (atc/climb.py). Departure: radar contact, and the next
        step when the flight is nearly at what the clearance gave -- a flight
        still well below it gets its next step as it nears it, from the
        climb watch -- with "proceed direct" the first fix ahead for a flight
        the tower sent off on runway heading. A centre: its own step, or the
        cruise, once the flight is nearly up to what departure gave; else
        "roger" (FAA) or "identified" (ICAO) and the climb carries on.
        """
        aircraft = session.aircraft
        here = float(getattr(state, "altitude_ft", 0.0) or 0.0)
        assigned = float(session.assigned_altitude_ft
                         or session.initial_altitude_ft or 0.0)
        callsign = aircraft.callsign.written
        step = 0
        direct = None
        why = ""
        if station.position in ("DEP", "APP"):
            session.phase = Phase.DEPARTURE
            short = here < assigned - DEPARTURE_REACHING_FT
            if short and departure_top(cruise) - assigned > LAST_STEP_FT:
                why = "still climbing to the clearance's altitude"
            else:
                step = departure_step(station.callsign, callsign, assigned,
                                      here, cruise)
                why = "the next step"
            if session.runway_heading and self.route:
                direct = on_course_fix(self.route, state.latitude,
                                       state.longitude)
        else:
            if here >= assigned - DEPARTURE_REACHING_FT:
                step = self._centre_step_valid(
                    session, station, state, here, assigned, cruise)
                why = "the centre's step" if step < cruise else "the cruise"
            else:
                why = "still climbing to departure's altitude"
        if step and step <= max(assigned, here) + 500:
            step = 0
        clauses = []
        values: dict[str, Any] = {}
        if step:
            clauses.append(p.climb_clause(step, transition))
            values["altitude_ft"] = int(step)
        if direct is not None:
            clauses.append(p.direct_clause(direct.ident))
            values["fix"] = direct.ident
            session.runway_heading = False
        decision("climb", event="check_in", station=station.callsign,
                 here_ft=round(here), assigned_ft=round(assigned),
                 step_ft=step or None, direct=direct.ident if direct else None,
                 why=why)
        session.establish(station)
        instruction_text = ", ".join(clauses)
        if first_contact and station.position == "CTR" and not instruction_text:
            return Reply(p.centre_check_in(aircraft,
                                           self._station_name(station, p)),
                         station)
        if first_contact:
            text = p.radar_contact(aircraft, self._station_name(station, p),
                                   instruction=instruction_text,
                                   abbreviated=False)
        elif instruction_text:
            text = _join(aircraft.spoken(True), instruction_text) + "."
        else:
            text = p.roger(aircraft)
        instruction = None
        if "altitude_ft" in values:
            instruction = session.remember(
                Instruction("altitude", station_key(station), text, values))
        elif values:
            instruction = session.remember(
                Instruction("direct", station_key(station), text, values,
                            readback_required=False))
        return Reply(text, station, instruction=instruction)

    def _centre_check_in(self, parsed, session, station, p, state,
                         transition: int) -> Reply:
        """A flight checking in with a centre, in the cruise.

        In the region's words: an FAA centre already has the flight on
        radar and says "Boston Center, roger"; an ICAO one says "identified"
        and the level to keep. A level reported that the Mode C does not
        show is asked for -- the check this program already makes of a VFR
        flight under a Bravo -- because one of the two is wrong and only
        the pilot can say which.
        """
        aircraft = session.aircraft
        session.establish(station)
        heard = parsed.get("altitude_ft") or 0
        shown = getattr(state, "altitude_ft", None) if state is not None else None
        if heard and shown is not None \
                and abs(float(heard) - float(shown)) >= self.MODE_C_DISAGREES_FT:
            decision("enroute", event="check_in", station=station.callsign,
                     said="say_altitude", heard_ft=heard,
                     mode_c_ft=round(float(shown)))
            return Reply(p.say_altitude(aircraft), station)
        level = (session.assigned_altitude_ft or heard
                 or (round(float(shown) / 100.0) * 100.0 if shown else 0.0))
        text = p.centre_check_in(aircraft, self._station_name(station, p),
                                 level_ft=float(level or 0.0),
                                 transition=transition)
        decision("enroute", event="check_in", station=station.callsign,
                 said="roger" if p.dialect == "faa" else "identified",
                 heard_ft=heard,
                 mode_c_ft=round(float(shown)) if shown is not None else None)
        return Reply(text, station)

    def _issue_flight_following(self, session, station, p, state) -> Reply:
        """VFR radar advisories, which are asked for and can be refused.

        FAA JO 7110.65 2-1-1: additional services are provided workload
        permitting. Busy means busy, so a sector already sequencing arrivals
        says so instead of pretending it has the capacity.
        """
        aircraft = session.aircraft
        if len(self._traffic_ahead(True)) >= 3:
            return Reply(p.flight_following_unavailable(aircraft), station)

        session.flight_following = True
        session.radar_service = station_key(station)
        session.squawk = squawk_for(session, station.ident)
        text = p.flight_following_approved(
            aircraft,
            self._station_name(station, p) if not session.established_with(station) else "",
            session.squawk,
        )
        instruction = session.remember(
            Instruction("flight_following", station_key(station), text,
                        {"squawk": session.squawk}))
        return Reply(text, station, instruction=instruction)

    def _issue_ifr_pickup(self, session, station, p, state) -> Reply:
        """An airborne VFR flight asking for IFR: the pop-up pickup.

        The ask is the filing: the flight is IFR from the moment the
        clearance is issued, identified, cleared to its destination
        direct, and held at the level it filed -- or the one it is at,
        a pickup is never a descent to the paperwork. Squawk and level
        are readback items, the way the ground clearance's are.
        """
        aircraft = session.aircraft
        session.ifr = True
        session.squawk = squawk_for(session, station.ident)
        current = float(getattr(state, "altitude_ft", 0.0) or 0.0)
        cruise = session.cruise_altitude_ft or self._default_cruise(session)
        airport = self.navdb.airport(station.ident)
        transition = self._transition_altitude(airport)
        # Held at the level it is at, never a descent to the paperwork --
        # but rounded down to a level that is legal for the direction, the
        # same shaping every other level it is cleared to gets.
        level = max(cruise, self._valid_level(
            round(current / 100.0) * 100.0, state, airport, upward=False))
        destination = session.destination
        dest_name = (
            destination.spoken if destination
            else (session.destination_input or p.UNKNOWN_DESTINATION)
        )
        via = "via direct" if p.dialect == "faa" else "direct"
        clause = (f"cleared to {dest_name} {via}, "
                  f"maintain {p.alt(level, transition)}")
        session.establish(station)
        session.clearance_issued = True
        session.radar_service = station_key(station)
        text = (p.radar_contact(
                    aircraft, self._station_name(station, p),
                    instruction=clause, abbreviated=False)
                + " " + p.squawk(aircraft, session.squawk))
        instruction = session.remember(
            Instruction("ifr_clearance", station_key(station), text,
                        {"altitude_ft": int(level),
                         "squawk": session.squawk},
                        required_items={"altitude_ft", "squawk"}))
        return Reply(text, station, instruction=instruction)

    def _issue_approach(self, session, station, weather, p, state,
                        parsed: ParsedIntent | None = None) -> Reply:
        aircraft = session.aircraft
        destination = session.destination or self._nearest_airport(state)
        if destination is None:
            return Reply(p.unable_unknown_destination(aircraft), station)

        if session.hold is not None and session.hold_requested:
            # The hold the pilot asked for is over when they ask for the
            # approach: the arrival watch lets the flight go.
            session.hold_requested = False
        # "Request RNAV two five left": the type, and the runway if one was
        # named and the field has it.
        asked = asked_for(parsed.normalized or parsed.text) if parsed else ""
        named = parsed.get("runway") if parsed else ""
        if named and not self.navdb.has_runway(destination.ident, named) \
                and self.realism.pilot_requests and session.ifr:
            # A runway the field does not have: said, not quietly swapped.
            decision("request", asked="runway", answer="unable",
                     runway=named, why="no such runway")
            return Reply(p.unable_because(aircraft, "no_runway", runway=named),
                         station)
        if named and self.navdb.has_runway(destination.ident, named):
            session.arrival_runway = named
        runway = session.arrival_runway or self._arrival_runway(destination, weather)
        known = self.approach_data.get(destination.ident)
        if (asked and asked != VISUAL and known is not None and session.ifr
                and self.realism.pilot_requests
                and asked not in known.types(runway)):
            # An approach the runway does not have: unable, and the one to
            # expect instead (the choice is not changed).
            other = self.approach_choice(session, destination, runway, weather)
            decision("request", asked="approach", answer="unable",
                     approach=asked, runway=runway,
                     instead=other.approach.name)
            return Reply(p.unable_because(
                aircraft, "no_approach", approach=p.approach_name(asked),
                runway=runway, other=self.approach_said(p, other)), station)
        if asked:
            session.requested_approach = asked
        session.arrival_runway = runway
        session.phase = Phase.APPROACH

        # A VFR flight is not given an instrument approach. It is told where
        # the field is and asked to report it in sight, which is how a radar
        # position hands one to a tower.
        if not session.ifr:
            oclock, distance = self._bearing_to(state, destination)
            text = p.report_field_in_sight(
                aircraft, destination.spoken, oclock, distance)
            instruction = session.remember(
                Instruction("report_field", station_key(station), text, {},
                            readback_required=False))
            return Reply(text, station, instruction=instruction)

        choice = self.approach_choice(session, destination, runway, weather)
        text = p.expect_approach(
            aircraft, self.approach_said(p, choice),
            choice.approach.runway or runway, destination.spoken,
            circle=self.circle_said(p, choice))
        instruction = session.remember(
            Instruction("expect_approach", station_key(station), text, {},
                        readback_required=False)
        )
        return Reply(text, station, instruction=instruction)

    def _issue_visual_approach(self, session, station, weather, p) -> Reply:
        aircraft = session.aircraft
        destination = session.destination
        runway = session.arrival_runway or (
            self._arrival_runway(destination, weather) if destination else ""
        )
        if not runway:
            return Reply(p.roger(aircraft), station)

        session.arrival_runway = runway
        session.phase = Phase.APPROACH
        clearance = p.cleared_visual_approach(
            aircraft, runway, destination.spoken if destination else ""
        )
        tower = self._destination_tower(session, station)
        # One transmission, like the real call: "cleared visual approach
        # runway 09L, contact tower one one eight decimal seven".
        text = (p.handoff(aircraft,
                          self._station_name(tower, p, short=True),
                          tower.mhz, when=clearance)
                if tower and tower.mhz else clearance)
        instruction = session.remember(
            Instruction("visual_approach", station_key(station), text,
                        {"runway": runway})
        )
        return Reply(text, station, instruction=instruction, handoff_to=tower)

