from __future__ import annotations

from ._shared import (
    Any,
    Instruction,
    Intent,
    Phase,
    Reply,
    _airborne,
    decision,
    log,
    normalize_runway,
    say_words,
    station_key,
    taxi_route,
    time,
)


class _Ground:
    # ------------------------------------------------------------------
    # ground
    # ------------------------------------------------------------------

    # Requests that only make sense with the wheels on the ground. Answering
    # any of them from the air used to produce a taxi route for an aircraft at
    # cruise, which is the ground half of the same mistake the tower made.
    #
    # Checking in and asking for circuits are on the list because both reach
    # the taxi code by another route: a check-in on the ground frequency is a
    # request to taxi, and it is still one when the aeroplane is at six
    # thousand feet -- at which point it is not.
    _GROUND_ONLY = (
        Intent.REQUEST_PUSHBACK, Intent.REQUEST_TAXI,
        Intent.REQUEST_TAXI_TO_PARKING, Intent.REQUEST_CLEARANCE,
        Intent.CHECK_IN, Intent.REQUEST_PATTERN_WORK,
        Intent.REQUEST_CROSS_RUNWAY,
    )

    def _ground(self, parsed, session, station, weather, p, state) -> Reply:
        aircraft = session.aircraft

        if parsed.intent in self._GROUND_ONLY and _airborne(state):
            # Being told "you are airborne" and nothing else leaves the pilot
            # on a frequency that cannot help them, so whoever can takes the
            # call -- and that is radar, not the tower. An aeroplane at cruise
            # asking to taxi is not in the circuit, and the tower could do no
            # more for it than ground can.
            sent = self._send_on(session, station, p, "DEP", "APP")
            if sent is not None:
                return sent
            return Reply(p.already_airborne(aircraft), station)

        if parsed.intent is Intent.REQUEST_PUSHBACK:
            return Reply(p.pushback_approved(aircraft), station,
                         ground="pushback")

        if parsed.intent is Intent.REQUEST_TAXI:
            # The ground handling has the aeroplane. Telling it to taxi now
            # would be an instruction it cannot follow.
            reason = self._stuck_on_the_stand()
            if reason:
                return Reply(p.hold_position(aircraft) + f" You are {reason}.",
                             station)

        if parsed.intent is Intent.REQUEST_VFR_DEPARTURE:
            return self._issue_vfr_release(session, station, p, parsed, weather)

        if parsed.intent is Intent.REQUEST_CLEARANCE:
            # A VFR flight has nothing to collect from delivery. Sending it
            # there to be read an airways clearance it did not file is the
            # wrong half of the phrasebook and a wasted frequency change.
            if not session.ifr:
                return self._issue_vfr_release(session, station, p, parsed, weather)
            # A clearance is not a taxi instruction, and answering one with the
            # other is the ground half of the mistake the tower used to make.
            # Where the field has a delivery position, that is whose job it is.
            delivery = self.controller_at(station.ident, "DEL")
            if delivery and station_key(delivery) != station_key(station):
                return Reply(
                    p.handoff(aircraft,
                              self._station_name(delivery, p, short=True),
                              delivery.mhz),
                    station, handoff_to=delivery,
                )
            # No delivery here, so ground issues it, which is what a small
            # field actually does.
            return self._issue_clearance(session, station, weather, p)

        if parsed.intent in (Intent.REQUEST_TAXI, Intent.CHECK_IN):
            if session.phase in (Phase.LANDED, Phase.TAXI_IN):
                return self._issue_taxi_in(session, station, weather, p, state)
            return self._issue_taxi(session, station, weather, p, state)

        if parsed.intent is Intent.REQUEST_TAXI_TO_PARKING:
            return self._issue_taxi_in(session, station, weather, p, state)

        if parsed.intent is Intent.REQUEST_PATTERN_WORK:
            # Circuits are the tower's to approve; ground's job is to get the
            # aircraft to the runway, which is what it asked for in effect.
            session.pattern_work = True
            return self._issue_taxi(session, station, weather, p, state)

        if parsed.intent is Intent.REQUEST_CROSS_RUNWAY:
            reply = self.crossing_clearance(session, station, p, state,
                                            asked=True)
            if reply is not None:
                return reply
            # Nothing left to cross: the pilot is on the rest of the taxi.
            if not session.pending_crossing():
                return Reply(p.continue_taxi(aircraft), station)
            return Reply(p.hold_short_instruction(
                aircraft, session.pending_crossing()), station)

        if parsed.intent is Intent.REQUEST_TAKEOFF:
            tower = self.controller_at(station.ident, "TWR",
                                       runway=session.departure_runway)
            if tower:
                return Reply(
                    p.handoff(aircraft, self._station_name(tower, p, short=True), tower.mhz),
                    station, handoff_to=tower,
                )
            return Reply(p.roger(aircraft), station)

        if parsed.intent is Intent.RADIO_CHECK:
            return Reply(p.radio_check(aircraft), station)

        if parsed.intent in (Intent.ACKNOWLEDGE, Intent.AFFIRMATIVE):
            # An affirmative to the off-route offer is the ask for the new
            # route, which is planned again from where the aeroplane is.
            if parsed.intent is Intent.AFFIRMATIVE \
                    and session.assistance_offered("taxi"):
                session.answer_assistance()
                decision("assist", taken="taxi", station=station.callsign)
                if session.phase is Phase.TAXI_IN:
                    return self._issue_taxi_in(session, station, weather, p, state)
                return self._issue_taxi(session, station, weather, p, state)
            return Reply("", station, silent=True)

        if parsed.intent is Intent.NEGATIVE and session.assistance_offered():
            session.answer_assistance()
            return Reply(p.roger(aircraft), station)

        return self._not_understood(session, station, p, state)

    # Set by the engine: ``ident -> GroundLayout | None``, the simulator's
    # stands and taxiways for a field. It never blocks -- a layout still being
    # read is None, and asking is what starts the read -- and without it, or
    # without an answer, the taxi is the invented one it always was.
    ground_layout = None
    # And ``ident -> bool``: whether a field's read is still in flight.
    # A taxi asked for in that gap is told to stand by rather than cleared
    # on the invented route it would then keep for ever.
    ground_pending = None

    def _layout_for(self, ident: str):
        if self.ground_layout is None:
            return None
        try:
            return self.ground_layout(ident) or None
        except Exception:
            log.debug("the ground layout for %s raised", ident, exc_info=True)
            return None

    def _layout_pending(self, ident: str) -> bool:
        """Whether the field's taxiways are still being read: an answer
        that is coming, as opposed to one that never did."""
        if self.ground_pending is None:
            return False
        try:
            return bool(self.ground_pending(ident))
        except Exception:
            log.debug("the pending check for %s raised", ident, exc_info=True)
            return False

    def _ground_traffic(self, layout):
        """Every other aeroplane on the ground, as (x, z, radius) in the
        layout's plane, and the stands the traffic has handed out."""
        if self.traffic is None:
            return [], set()
        try:
            return self.traffic.ground_occupancy(layout)
        except Exception:
            log.debug("could not ask the traffic where it is", exc_info=True)
            return [], set()

    def _plan_taxi(self, session, station, state, *, inbound: bool,
                   weather=None):
        """The taxi on the simulator's own taxiways, or None and the reason
        it is the invented one instead (see :mod:`meridianatc.atc.taxi`)."""
        from .. import aircraft_class
        from .. import taxi as taxiplan

        layout = self._layout_for(station.ident)
        if layout is None:
            return None, "the simulator has not described this airport"
        if state is None or not getattr(state, "has_position", False):
            return None, "no aircraft position"
        lines = taxiplan.runway_lines(layout, self.navdb.runways(station.ident))
        occupied, taken = self._ground_traffic(layout)
        busy = taxiplan.busy_runways(
            layout, self.navdb, station.ident,
            (session.departure_runway, session.arrival_runway,
             session.touchdown_runway), weather=weather)
        if inbound:
            aircraft = session.aircraft
            # An airline stand for an airline flight, as the pilot said,
            # and inside that divide, one the aeroplane's class fits on.
            airliner = session.profile.airliner
            min_radius = aircraft_class.stand_radius_m(
                aircraft_class.of_state(state).name) \
                if getattr(state, "type_code", "") else 0.0
            return taxiplan.plan_arrival(
                layout, state.latitude, state.longitude, lines,
                airliner=airliner, key=aircraft.callsign.written,
                occupied=occupied, taken=taken,
                runway=session.arrival_runway or session.touchdown_runway,
                busy=busy, min_radius_m=min_radius)
        # An entry is only a way in if the runway left past it still flies
        # the takeoff: the type's own distance figure decides that.
        need_m = 0.0
        if getattr(state, "type_code", ""):
            need_m = aircraft_class.min_runway_ft(state.type_code) * 0.3048
        return taxiplan.plan_departure(
            layout, state.latitude, state.longitude, session.departure_runway,
            lines, avoid=taxiplan.avoid_near(layout, occupied), busy=busy,
            need_m=need_m)

    def _safely_plan(self, session, station, state, *, inbound: bool,
                     weather=None):
        # Scenery is data somebody else wrote, and a layout nobody tested
        # against is the likeliest place for a surprise. A pilot who asked
        # to taxi gets the invented route rather than silence.
        try:
            return self._plan_taxi(session, station, state, inbound=inbound,
                                   weather=weather)
        except Exception as error:
            log.info("planning the taxi at %s raised", station.ident,
                     exc_info=True)
            return None, f"the taxi planner raised {type(error).__name__}"

    @staticmethod
    def _keep_the_rest(session, plan, *, inbound: bool) -> None:
        """A cut taxi's second half, for when the aeroplane gets there."""
        session.taxi_rest = list(plan.rest)
        session.taxi_split_at = plan.split_at if plan.rest else None
        session.taxi_rest_inbound = inbound

    # How close the aeroplane has to be to where it was told to hold short
    # for the rest of a cut taxi to be given. Chosen: a junction's width.
    REST_OF_TAXI_M = 60.0

    def rest_of_taxi(self, session, station, p, state) -> Reply | None:
        """The second half of a taxi cut in two, once the aeroplane is at
        the taxiway it was told to hold short of: the rest of the route to
        the runway, or to the stand."""
        if not session.taxi_rest or session.taxi_split_at is None:
            return None
        if state is None or not state.on_ground or not state.has_position:
            return None
        from ...navdata.build import haversine_nm

        lat, lon = session.taxi_split_at
        apart_m = haversine_nm(state.latitude, state.longitude, lat, lon) * 1852.0
        if apart_m > self.REST_OF_TAXI_M:
            return None
        rest = list(session.taxi_rest)
        session.taxi_rest, session.taxi_split_at = [], None
        session.taxi_route = list(session.taxi_route) + rest
        said = say_words(station.ident, rest)
        decision("taxi_route", airport=station.ident, event="the rest",
                 route=rest, inbound=session.taxi_rest_inbound)
        if session.taxi_rest_inbound:
            text = p.taxi_to_parking(session.aircraft, "", said, p.PARKING,
                                     abbreviated=True)
            return Reply(text, station, instruction=session.remember(
                Instruction("taxi_in", station_key(station), text, {},
                            readback_required=False)))
        runway = session.departure_runway
        text = p.taxi_to_runway(session.aircraft, "", runway, said,
                                hold_short=session.hold_short_of or "",
                                abbreviated=True)
        values: dict[str, Any] = {"runway": runway, "route": rest}
        if session.hold_short_of:
            values["hold_short"] = session.hold_short_of
        return Reply(text, station, instruction=session.remember(
            Instruction("taxi", station_key(station), text, values)))

    @staticmethod
    def _keep_taxi(session, plan) -> None:
        """The route on the session, so the map, the readback and whatever is
        said next use the one that was given."""
        session.taxi_nodes = list(plan.nodes) if plan else []
        session.taxi_path = list(plan.points) if plan else []
        session.taxi_crossings = list(plan.crossings) if plan else []
        session.taxi_stand = (plan.stand.number
                              if plan and plan.stand is not None else None)
        # A new route's crossings are all still to be made, whatever the old
        # one had cleared.
        session.crossing_index = 0
        session.crossing_granted = ""

    def _issue_taxi(self, session, station, weather, p, state=None) -> Reply:
        airport = self.navdb.airport(station.ident)
        runway = session.departure_runway or self._departure_runway(airport, weather)
        session.departure_runway = runway

        plan, why = self._safely_plan(session, station, state, inbound=False,
                                      weather=weather)
        if plan is None and self._layout_pending(station.ident):
            # The field's taxiways are still being read: "stand by" now
            # and the next ask gets the real route, where answering the
            # invented one would have been kept for the whole taxi.
            decision("taxi_route", airport=station.ident, runway=runway,
                     source="waiting", why="the taxiways are still being read")
            return Reply(p.standby(session.aircraft), station)
        self._keep_taxi(session, plan)
        hold_at = ""
        if plan is not None:
            # Five taxiways at most; a longer route is cut, held short of
            # the sixth, and the rest is given there (SurfaceWatch).
            route = plan.first
            hold_at = plan.hold_at
            self._keep_the_rest(session, plan, inbound=False)
            # Every runway the route crosses has to be held short of, and the
            # first is the one the pilot meets first. The rest are for after
            # it has been crossed.
            hold_short = plan.hold_short
            decision("taxi_route", airport=station.ident, runway=runway,
                     source="simulator", route=route, crossings=plan.crossings)
        else:
            route = taxi_route(station.ident, runway)
            # Hold short of a runway that is not the assigned one, when the
            # field has more than one. Crossing clearances are issued
            # separately.
            hold_short = self._hold_short_candidate(airport, runway)
            decision("taxi_route", airport=station.ident, runway=runway,
                     source="invented", route=route, why=why)
        session.taxi_route = route
        session.hold_short_of = hold_short

        altimeter = (
            weather.altimeter_inhg if p.dialect == "faa" else weather.qnh_hpa
        )
        said = say_words(station.ident, route)
        text = p.taxi_to_runway(
            session.aircraft,
            self._station_name(station, p) if not session.established_with(station) else "",
            runway, said, hold_short=hold_short,
            altimeter=altimeter if not session.clearance_issued else 0.0,
            abbreviated=session.established_with(station),
            hold_short_taxiway=(say_words(station.ident, [hold_at])[0]
                                if hold_at else ""),
        )
        session.phase = Phase.TAXI_OUT

        values: dict[str, Any] = {"runway": runway}
        if hold_short:
            values["hold_short"] = hold_short
        if route:
            # Not a readback item, but the first officer's readback is built
            # from these values, and without the route it read the runway
            # back in the hold-short frame -- "hold short runway one nine,
            # hold short runway zero one" -- which the controller refused,
            # four times, on the first IFR reference flight.
            values["route"] = list(route)
        instruction = session.remember(
            Instruction("taxi", station_key(station), text, values)
        )
        return Reply(text, station, instruction=instruction)

    def _issue_taxi_in(self, session, station, weather, p, state=None) -> Reply:
        plan, why = self._safely_plan(session, station, state, inbound=True,
                                      weather=weather)
        if plan is None and self._layout_pending(station.ident):
            decision("taxi_route", airport=station.ident, inbound=True,
                     source="waiting", why="the taxiways are still being read")
            return Reply(p.standby(session.aircraft), station)
        self._keep_taxi(session, plan)
        hold_short = ""
        hold_at = ""
        destination = p.PARKING
        if plan is not None:
            route = plan.first
            hold_at = plan.hold_at
            self._keep_the_rest(session, plan, inbound=True)
            hold_short = plan.hold_short
            if plan.stand.number:
                destination = p.stand(plan.stand.number, plan.stand.gate)
            decision("taxi_route", airport=station.ident, inbound=True,
                     source="simulator", route=route, stand=plan.stand.number,
                     crossings=plan.crossings)
        else:
            route = taxi_route(station.ident, session.arrival_runway or "01",
                               inbound=True)
            decision("taxi_route", airport=station.ident, inbound=True,
                     source="invented", route=route, why=why)
        session.taxi_route = route
        session.hold_short_of = hold_short
        text = p.taxi_to_parking(
            session.aircraft,
            self._station_name(station, p) if not session.established_with(station) else "",
            say_words(station.ident, route), destination,
            hold_short=hold_short,
            abbreviated=session.established_with(station),
            hold_short_taxiway=(say_words(station.ident, [hold_at])[0]
                                if hold_at else ""),
        )
        session.phase = Phase.TAXI_IN
        # A taxi to the stand is not read back, but a hold-short in it is:
        # FAA AIM 4-4-7 and ICAO Doc 4444 4.5.7.5.1 both name it, and a pilot
        # who has not said it back has not agreed to stop.
        values = {"hold_short": hold_short} if hold_short else {}
        instruction = session.remember(
            Instruction("taxi_in", station_key(station), text, values,
                        readback_required=bool(hold_short))
        )
        return Reply(text, station, instruction=instruction)

    def _hold_short_candidate(self, airport, runway: str) -> str:
        """A runway the invented taxi route would plausibly cross.

        Only for the invented route. Where the route comes from the
        simulator's taxiways the hold-short is the first runway it really
        crosses, and a runway it does not cross is never named."""
        if airport is None:
            return ""
        ends = self.navdb.runway_ends(airport.ident)
        target = normalize_runway(runway)
        others = sorted(
            {normalize_runway(e[0]) for e in ends} - {target}
        )
        if not others:
            return ""
        # Only pick a runway that is not the reciprocal of the assigned one.
        target_number = int(target[:2]) if target[:2].isdigit() else 0
        for candidate in others:
            # Not a helicopter strip: an "H" end (de Gaulle's 08H/26H, grass,
            # 440 m) is not a runway a jet taxis across, and the readback
            # "hold short runway two six Hotel" was refused as unreadable on
            # the LFPG-EGLL reference flight.
            if not candidate[:2].isdigit() or candidate.endswith("H"):
                continue
            number = int(candidate[:2])
            if abs(((number - target_number) * 10 + 540) % 360 - 180) > 30:
                return candidate
        return ""

    # How far short of the runway the crossing question is decided. Holding
    # points sit seventy to a hundred metres off the centreline; inside this
    # the next thing the aeroplane does is be on it.
    AT_CROSSING_M = 150.0
    # How long before anybody lands the runway must still be free for a
    # crossing to be granted -- the margin the AI traffic leaves for its own
    # runway crossings (traffic.x_running_them).
    CROSS_CLEAR_S = 90.0
    # Roughly what getting across the strip costs, for the arrival margin.
    CROSSING_TAKES_S = 45.0

    def _crossing_line(self, layout, ident: str, runway: str):
        """The published centreline of the runway held short of, or None."""
        from .. import taxi as taxiplan

        wanted = normalize_runway(runway)
        for line in taxiplan.runway_lines(layout, self.navdb.runways(ident)):
            if wanted in line.ends:
                return line
        return None

    def _crossing_blocked(self, session, layout, line, runway: str,
                          now: float) -> str:
        """Why the crossing cannot be granted: somebody on the strip, or an
        arrival inside the margin. "" when the runway is clear."""
        from .. import taxi as taxiplan

        traffic = self.traffic
        if traffic is None:
            # The runway's use is unknown, which is not a clearance to cross
            # it: a controller who cannot see the strip does not clear
            # anybody across it.
            return "runway traffic unknown"
        active = normalize_runway(session.departure_runway
                                  or session.arrival_runway or "")
        if normalize_runway(runway) == active:
            try:
                if traffic.runway_occupied(now, runway=runway):
                    return "runway occupied"
                if traffic._nearest_arrival_s(None, runway=runway) \
                        < self.CROSSING_TAKES_S + self.CROSS_CLEAR_S:
                    return "arrival inside the window"
            except Exception:
                log.debug("the runway picture raised", exc_info=True)
            return ""
        # Any other runway: nobody lands on it, but somebody can be on it.
        try:
            occupied, _ = traffic.ground_occupancy(layout)
        except Exception:
            log.debug("could not ask the traffic where it is", exc_info=True)
            return ""
        for x, z, radius in occupied:
            if taxiplan._point_to_segment((x, z), line.a, line.b) \
                    <= line.half_width_m + radius:
                return "runway occupied"
        return ""

    def crossing_clearance(self, session, station, p, state, *,
                           asked: bool = False, remind: bool = True
                           ) -> Reply | None:
        """The crossing the aeroplane is held short of: a clearance across
        it when the runway is clear, the hold-short repeated when it is not.

        ``asked`` is the pilot on the frequency -- "request cross runway" --
        which decides the question now wherever the aeroplane is on the way
        to the hold; ``remind`` is the crossing watch asking whether a
        refusal should be heard this look or wait for the next interval.
        """
        runway = session.pending_crossing()
        if not runway:
            return None
        if session.crossing_granted:
            # Already cleared across this one. Asked again, the clearance is
            # the answer again; asked by the watch, nothing more to say.
            if asked:
                return Reply(
                    p.cross_runway(session.aircraft, runway,
                                   then=p.report_vacated()), station)
            return None
        if state is None or not state.on_ground \
                or not getattr(state, "has_position", False):
            return None
        layout = self._layout_for(station.ident)
        if layout is None:
            return None
        line = self._crossing_line(layout, station.ident, runway)
        if line is None:
            return None
        from .. import taxi as taxiplan

        apart = taxiplan.distance_to_line(
            layout, line, state.latitude, state.longitude)
        if not asked and apart > self.AT_CROSSING_M:
            return None
        why = self._crossing_blocked(session, layout, line, runway,
                                     time.monotonic())
        aircraft = session.aircraft
        if why:
            decision("runway_crossing", airport=station.ident,
                     event="refused", runway=runway, why=why,
                     asked=asked, distance_m=round(apart))
            if not asked and not remind:
                return None
            return Reply(p.hold_short_instruction(aircraft, runway), station)
        session.grant_crossing(runway)
        decision("runway_crossing", airport=station.ident,
                 event="granted", runway=runway, asked=asked,
                 distance_m=round(apart))
        text = p.cross_runway(aircraft, runway, then=p.report_vacated())
        instruction = session.remember(Instruction(
            "cross_runway", station_key(station), text, {"runway": runway}))
        return Reply(text, station, instruction=instruction)

