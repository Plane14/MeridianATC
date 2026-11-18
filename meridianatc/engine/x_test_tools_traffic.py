from __future__ import annotations

from ._shared import (
    EMPTY_NOTICE_S,
    Aircraft,
    FlightSession,
    Phase,
    Station,
    command_hint,
    decision,
    every_language,
    haversine_nm,
    heard_registration,
    is_valid_channel,
    log,
    replace,
    same_channel,
    station_key,
    status_log,
    time,
)


class _Test_tools_traffic:
    # -- test tools (traffic.debug) --------------------------------------

    def debug_board_nearest(self) -> tuple[bool, str, str]:
        """The panel's "board nearest now": the parked aeroplane nearest the
        pilot boards now. Off unless traffic.debug.triggers."""
        if not self.config.traffic.debug.triggers:
            return False, "", "debug_off"
        if not self.config.traffic.enabled:
            return False, "", "traffic_off"
        flight, why = self.traffic.board_nearest(time.monotonic())
        return flight is not None, flight.written if flight else "", why

    def debug_arrival_nearest(self) -> tuple[bool, str, str]:
        """The panel's "arrival to nearest stand". Off unless
        traffic.debug.triggers."""
        if not self.config.traffic.debug.triggers:
            return False, "", "debug_off"
        if not self.config.traffic.enabled:
            return False, "", "traffic_off"
        self.traffic.can_invent = self.simtraffic.drawing
        flight, why = self.traffic.arrival_to_nearest_stand(time.monotonic())
        return flight is not None, flight.written if flight else "", why

    def _say_traffic_status(self, now: float) -> None:
        """Every status_every_s, one log line per invented aeroplane, for
        testing the apron (traffic.debug.status). English, like every log."""
        debug = self.config.traffic.debug
        if not debug.status or now - self._status_at < max(
                1.0, float(debug.status_every_s)):
            return
        self._status_at = now
        try:
            lines = self.traffic_status(now)
        except Exception:
            log.debug("the traffic status raised", exc_info=True)
            return
        status_log.info("traffic.debug: %d invented aeroplane(s)", len(lines))
        for line in lines:
            status_log.info("  %s", line)

    # How often the motion loop's own figures are logged with the status on.
    MOTION_STATUS_S = 10.0

    def _say_motion_status(self, now: float) -> None:
        """Every ten seconds with traffic.debug.status, how the loop that
        moves the vehicles and the people really ran: its rate, the time a
        frame took, the actors, the camera, the nearest person's rate, and
        each frame reader's share. The figures are measured on the message
        thread and only read here, so the log costs the frame nothing."""
        if not self.config.traffic.debug.status or                 now - self._motion_status_at < self.MOTION_STATUS_S:
            return
        self._motion_status_at = now
        report = getattr(self.simvehicles, "rates", None)
        if report is None:
            return
        try:
            line = report()
        except Exception:
            log.debug("the motion status raised", exc_info=True)
            return
        if line:
            status_log.info("traffic.debug: %s", line)

    def traffic_status(self, now: float) -> list[str]:
        """What the status dump says: callsign, type, stand, level, window,
        each door, and the next thing that happens to it and when."""
        found = []
        for position in self.traffic.positions(invented_only=True):
            callsign = position["callsign"]
            flight = self.traffic.flight_named(callsign)
            kind, _ = self.traffic.apron_state(callsign)
            window = self._apron_window(callsign, kind, now)
            level = self.simtraffic.target_level(callsign) or "-"
            stand = "-"
            if flight is not None:
                held = (flight.stand if flight.stand is not None
                        else flight.planned_stand)
                stand = str(held) if held is not None else "-"
            found.append(
                f"{callsign:<9} {position.get('type', ''):<5} stand {stand:<4} "
                f"{level:<8} {window:<12} doors {self._doors_said(callsign)}"
                f"  systems {self._systems_said(callsign)}"
                f"  next: {self._next_event(flight, kind, callsign, now)}")
        return found

    def _systems_said(self, callsign: str) -> str:
        """Engines and lights as read back, upper case on: ENG NAV BCN STR
        TAX LDG; a '>' marks one still wanted the other way."""
        simsystems = getattr(self, "simsystems", None)
        object_id = self.simtraffic.object_id(callsign)
        plan = simsystems.plan(object_id) if simsystems and object_id else None
        if plan is None or plan.reading is None:
            return "unread"
        said = []
        for name, short in (("engines", "eng"), ("nav", "nav"), ("beacon", "bcn"),
                            ("strobe", "str"), ("taxi", "tax"), ("landing", "ldg")):
            on = getattr(plan.reading, name)
            word = short.upper() if on else short
            if getattr(plan.goal, name) != on:
                word += ">"
            said.append(word)
        return " ".join(said)

    def _doors_said(self, callsign: str) -> str:
        object_id = self.simtraffic.object_id(callsign)
        plan = self.simdoors.plan(object_id) if object_id else None
        if plan is None:
            return "none (no door animation, or not drawn)"
        said = []
        for role, index in sorted(plan.doors.items()):
            state = plan.is_open(index)
            word = ("deaf" if index in plan.deaf else "open" if state is True
                    else "shut" if state is False else "moving")
            if plan.goal.get(index) and state is not True:
                word += ">open"
            elif not plan.goal.get(index) and state is True:
                word += ">shut"
            said.append(f"{role}={word}")
        return ",".join(said)

    def _next_event(self, flight, kind: str, callsign: str, now: float) -> str:
        def clock(seconds: float) -> str:
            seconds = max(0, int(round(seconds)))
            return f"{seconds // 60}:{seconds % 60:02d}"

        traffic = self.traffic
        if flight is None:
            return "-"
        if kind == "booked":
            return f"boarding in {clock(flight.boarding_at - now)}"
        if kind == "boarding":
            return ("boarded, calls for clearance in "
                    f"{clock(flight.boarding_at + traffic.boarding_s - now)}")
        if kind == "departing":
            tug = self.handling.tug_due_at(callsign)
            if tug is not None:
                return f"pushback (tug) in {clock(tug - now)}"
            if not self.simdoors.can_push(self.simtraffic.object_id(callsign)):
                return "doors closing, then the tug"
            return f"waiting for push clearance ({flight.phase})"
        if kind in ("arrived", "static"):
            ends = self.handling.arrival_ends_at(callsign)
            if ends is not None and ends > now:
                return f"doors shut in {clock(ends - now)}"
            if kind == "arrived":
                return "deplaning"
            return ("parked until booked" if traffic.schedule
                    else "parked until chosen as a departure")
        if kind == "arriving":
            return f"{flight.phase}, {flight.distance_nm:.1f} nm to run"
        return flight.phase

    def _sim_pushback(self) -> bool:
        """The simulator's own tug. Handed to the ground handling as a
        callable so that nothing in ``integrations`` knows what a sim is."""
        push = getattr(self.source, "pushback", None)
        if push is None:
            return False
        try:
            return bool(push())
        except Exception:
            log.debug("the simulator refused the pushback", exc_info=True)
            return False

    def _start_pushback(self) -> None:
        """Call the tug, and say which one answered.

        Never in the way of the transmission. A push that cannot be started
        is reported and nothing else: the aeroplane is where it was, and the
        pilot can still push it themselves.
        """
        said = ""
        if self.config.gsx.enabled:
            try:
                said = self.gsx.start_pushback()
            except Exception:
                log.debug("could not start the pushback", exc_info=True)
        # The ramp that does not answer is not the end of the tug: the
        # simulator's own is still the push the approval promised --
        # "push and start approved" that never called anybody is how the
        # FO ended up asking ground for the same tug four times.
        if not said and self._sim_pushback():
            said = "Pushback tug called"
        if said:
            self.emit("info", said, key="log.ground_handling", detail=said)

    def _room_on_the_channel(self, station: Station | None,
                             mhz: float) -> Station | None:
        """Which of a field's radar positions answers a channel they share.

        Centennial's 132.750 is Denver App/Dep, filed twice, and the resolver
        took the Approach row whoever was calling: the tower sent a
        departure to Denver Departure and Denver Approach answered it. The
        position already being worked keeps the channel; then the one the
        pilot was sent to; then Departure on the way out and Approach on the
        way in.
        """
        if station is None or station.position not in ("APP", "DEP"):
            return station
        same = [s for s in self.navdb.stations(station.ident)
                if s.position in ("APP", "DEP")
                and same_channel(s.mhz, mhz, 0)]
        if len({s.position for s in same}) < 2:
            return station
        previous = self.station
        session = self.session
        handed = session.handoff_pending if session is not None else None
        for held in (previous, handed):
            if (held is not None and held.ident == station.ident
                    and held.position in ("APP", "DEP")
                    and same_channel(held.mhz, mhz, 0)):
                return held
        outbound = session is not None and session.phase.outbound
        wanted = "DEP" if outbound else "APP"
        return next((s for s in same if s.position == wanted), station)

    def tune(self, mhz: float) -> Station | None:
        """Tune a frequency and work out who is listening."""
        with self._text_lock:
            return self._tune(mhz)

    def _tune(self, mhz: float) -> Station | None:
        state = self.state
        lat, lon = (state.latitude, state.longitude) if state.has_position else (0.0, 0.0)
        station = self.navdb.resolve_station(mhz, lat, lon, state.altitude_ft)
        station = self._room_on_the_channel(station, mhz)

        previous = self.station
        # A centre answers anywhere in its sector and carries the FIR's
        # ident whichever field its number is filed at (atc/enroute.py), so
        # retuning it -- five hundred miles on, or past Minnipa to Wudinna,
        # both Melbourne on 125.9 -- finds the same station. That used to
        # take a patch here that kept the old station alive.
        oceanic = (self.session.oceanic
                   if self.session is not None else None)
        if station is None and oceanic is not None:
            # Over the ocean the flight is with an HF station no COM1 can
            # tune; the VHF box is left on whatever the pilot likes.
            station = oceanic
        handed = (self.session.handoff_pending
                  if self.session is not None else None)
        if (handed is not None and handed.position == "CTR" and handed.mhz
                and same_channel(handed.mhz, mhz, 4)):
            # The centre the pilot was just sent to, dialled up to four
            # minutes before its line: the number they were given is theirs,
            # wherever its sector is drawn from here.
            station = handed
            if station_key(handed) != station_key(previous):
                self.session.with_since = time.monotonic()
            pending = self.session.pending
            if (pending is not None and pending.kind == "handoff"
                    and not pending.read_back):
                # Switching is as good as reading it back.
                self.session.clear_pending()
        if station_key(station) != station_key(previous):
            log.info("COM1 %.3f at %.2f, %.2f: %s -> %s", mhz, lat, lon,
                     previous.callsign if previous else "nobody",
                     station.callsign if station else "nobody listening")
        self.station = station
        self._last_com1 = mhz

        if station is None and self.config.atc.answer_unpublished:
            station = self._unpublished_station(mhz, lat, lon)
            if station is not None:
                self.station = station

        if station is None:
            # Explained by the situation loop once COM1 has stayed here
            # (``_explain_empty``), not now: a swap passes through the
            # standby frequency for a moment, and that moment was explained.
            # By channel: 126.630 after 126.625 is the same empty frequency.
            if not same_channel(mhz, self._reported_empty, 0) and (
                    self._empty_pending is None
                    or not same_channel(self._empty_pending[0], mhz, 0)):
                self._empty_pending = (round(mhz, 3), time.monotonic())
            # Nobody on frequency does not mean nowhere. The language still
            # follows the aeroplane, so an empty frequency over Germany is
            # listened to in German and English rather than in whatever was
            # worked at the last airport that answered.
            self._last_region_check = 0.0
            self._follow_region(state)
            if self.session is not None:
                self.session.current_station = None
            return None
        self._reported_empty = 0.0
        self._empty_pending = None
        self._say_invented(station)

        # Somebody real may be working this frequency, in which case this
        # program has nothing to do here.
        self.human = self.online.on_frequency(mhz, lat, lon)

        if previous is None or station_key(previous) != station_key(station) \
                or self.human is not None:
            if self.human is not None:
                self.emit(
                    "info",
                    f"{mhz:.3f} - {self.human.describe()} is working this "
                    "frequency. MeridianATC will stay quiet on it.",
                    key="log.human_online", mhz=f"{mhz:.3f}",
                    station=self.human.callsign,
                    network=self.human.network.upper(),
                )
            # A traffic advisory frequency has nobody on it. Saying so is the
            # difference between a quiet frequency and a broken program: a
            # pilot who tunes one and hears nothing back has no way to tell
            # which of the two they are looking at.
            # Somebody is there only inside their hours: a closed tower or
            # information service is as empty as a traffic frequency.
            elif self.brain.staffed(station):
                self.emit(
                    "info",
                    f"{mhz:.3f} - {station.callsign} "
                    f"({station.ident}, {station.distance_nm:.0f} NM).",
                    key="log.tuned", mhz=f"{mhz:.3f}", station=station.callsign,
                    airport=station.ident, distance=f"{station.distance_nm:.0f}",
                )
            else:
                self.emit(
                    "info",
                    f"{mhz:.3f} - {station.callsign}: an advisory frequency, "
                    "so nobody answers. Announce your position and listen out.",
                    key="log.tuned_advisory", mhz=f"{mhz:.3f}",
                    station=station.callsign, airport=station.ident,
                )
        # The session's word for who COM1 is on: the first officer's
        # check-in and the circuit announcements ask for it. Outside the
        # change check -- a session built mid-flight on the frequency it
        # is already on still needs the station it is with.
        if self.session is not None:
            self.session.current_station = station
        # Whoever is on the frequency decides what is spoken on it.
        self.follow_languages(station.ident)
        if previous is None or station_key(previous) != station_key(station):
            # A new frequency is a new conversation. Whatever was queued was
            # said on the old one and nobody here would have heard it.
            self._traffic_queue.clear()
        return station

    def _resolve_com2(self, state) -> Station | None:
        """Who is on COM2's frequency.

        The same resolver COM1 goes through, without the session
        side-effects tuning carries -- the second radio listens, and
        listening does not establish anything.
        """
        mhz = state.com2_active or 0.0
        if not mhz:
            return None
        lat, lon = ((state.latitude, state.longitude)
                    if state.has_position else (0.0, 0.0))
        try:
            station = self.navdb.resolve_station(
                mhz, lat, lon, state.altitude_ft)
            station = self._room_on_the_channel(station, mhz)
        except Exception:
            log.debug("could not resolve COM2 %.3f", mhz, exc_info=True)
            return self.station2
        previous = self.station2
        if station_key(station) != station_key(previous):
            log.info("COM2 %.3f at %.2f, %.2f: %s -> %s", mhz, lat, lon,
                     previous.callsign if previous else "nobody",
                     station.callsign if station else "nobody listening")
        return station

    def _take_the_callsign_heard(self, session: FlightSession,
                                 text: str) -> bool:
        """The pilot's own callsign, when it is not the one in the settings.

        At Kortrijk the pilot said "Oscar Oscar Alpha Bravo Charlie" on every
        call and was answered "Echo Tango Delta two zero four", the ETD204
        an old flight had left in ``config.yaml``. What the pilot says on the
        radio is who they are: it is taken for the rest of the session, the
        strip follows it, and the change is logged with where the old one
        came from. Only a whole registration near the start of a
        transmission counts (``callsigns.heard_registration``).
        """
        heard = heard_registration(text)
        if not heard:
            return False
        written = session.aircraft.callsign.written
        if heard.replace("-", "") == written.replace("-", "").upper():
            return False
        was_from = ("config.yaml flight.callsign" if self.config.flight.callsign
                    else "the simulator's ATC ID" if self.state.callsign
                    else "the default")
        departure = session.departure
        # A registration heard is a registration, whatever the profile.
        alternatives = every_language(
            heard, session.aircraft.type_code,
            dialect=session.profile.dialect_for(
                departure.ident if departure else ""),
            airline=False)
        old = session.aircraft
        session.aircraft = Aircraft(
            alternatives["en"], old.type_code, old.wake, is_heavy=old.is_heavy,
            callsigns=alternatives, language=old.language)
        self.understanding.set_callsign(session.aircraft.variants())
        decision("callsign_heard", was=written, now=heard, was_from=was_from)
        self.emit("info", f"Callsign {heard}, as said on the radio "
                          f"(was {written}, from {was_from}).",
                  key="log.callsign_heard", callsign=heard, was=written,
                  source=was_from)
        return True

    # Which position answers an unlisted frequency, by where the flight is.
    _POSITION_FOR_PHASE = {
        Phase.PREFLIGHT: "DEL", Phase.CLEARED: "GND", Phase.PUSHBACK: "GND",
        Phase.TAXI_OUT: "GND", Phase.HOLDING_SHORT: "TWR",
        Phase.LINED_UP: "TWR", Phase.TAKEOFF: "TWR", Phase.DEPARTURE: "DEP",
        Phase.ENROUTE: "CTR", Phase.DESCENT: "APP", Phase.APPROACH: "APP",
        Phase.LANDING: "TWR",
    }

    def _unpublished_station(self, mhz: float, lat: float,
                             lon: float) -> Station | None:
        """The controller for where you are, on a frequency nobody publishes.

        The open frequency data is thin at some large airports, and a pilot
        sitting at one of them reading a number off the panel is not making it
        up. If the aeroplane is at a field that staffs positions, that field's
        controller takes the call.
        """
        if not self.state.has_position:
            return None
        # A number no radio could be tuned to is a typo, not a gap in the
        # data, and answering it would hide the mistake.
        if not is_valid_channel(mhz):
            return None
        airport = self.navdb.home_airport(lat, lon)
        if airport is None:
            airport = self.navdb.nearest_airport(
                lat, lon, max_nm=self.config.atc.unpublished_radius_nm,
                min_rank=2)
        if airport is None:
            return None
        distance = haversine_nm(lat, lon, airport.lat, airport.lon)
        if distance > self.config.atc.unpublished_radius_nm:
            return None
        # Only a field that staffs a position of its own. Kortrijk staffs
        # none -- an AFIS and nothing else -- and a "Kortrijk Clearance
        # Delivery" and a "Kortrijk Tower" were invented on 124.850 for it,
        # the second calling a takeoff there a pilot deviation.
        if not any((found := self.navdb.station_for(airport.ident, held))
                   and found.position == held and found.is_controller
                   for held in ("DEL", "GND", "TWR", "APP", "DEP")):
            return None

        phase = self.session.phase if self.session is not None else Phase.PREFLIGHT
        position = self._POSITION_FOR_PHASE.get(phase, "GND")
        if position == "GND" and not self.brain.ground_control:
            # Nobody works Ground, so a frequency assumed for it would be
            # assumed for nobody. The tower is who taxis the aeroplane now.
            position = "TWR"
        station = self.navdb.station_on(airport.ident, mhz, position)
        if station is None:
            return None
        station = replace(station, distance_nm=distance)

        if not same_channel(mhz, self._reported_assumed, 0):
            self._reported_assumed = mhz
            # The command as this installation spells it: there is no
            # "meridianatc" program in a shipped build, and that is what the
            # notice used to tell the pilot to run.
            how = command_hint(f"freq {airport.ident} --add {position} {mhz:.3f}")
            self.emit(
                "info",
                f"{mhz:.3f} is not in the published data for {airport.ident}. "
                f"Answering as {station.callsign}. To keep it: {how}",
                key="log.unpublished", mhz=f"{mhz:.3f}",
                airport=airport.ident, station=station.callsign,
                position=position, command=how,
            )
        return station

    def _situation(self, session: FlightSession, station: Station) -> dict:
        """A few facts that help the AI layer disambiguate a vague request.

        "we\'d like to get going" means taxi to a controller on the ground
        frequency and takeoff to one on tower, and the phase of flight is what
        separates them.
        """
        return {
            "talking to": station.callsign,
            "position": station.position,
            "phase of flight": session.phase.value,
            "on the ground": "yes" if self.state.on_ground else "no",
            "altitude": f"{self.state.altitude_ft:.0f} feet"
            if not self.state.on_ground else "",
            "awaiting readback of": session.pending.kind if session.pending else "",
        }

    def _explain_empty(self, now: float | None = None) -> None:
        """Say that nobody is listening, once COM1 has stayed there.

        Only the active frequency, only once it has been held for
        ``EMPTY_NOTICE_S``, and once per frequency: the situation loop
        re-checks COM1 several times a second, and a swap passes through the
        standby frequency on its way.
        """
        pending = self._empty_pending
        if pending is None or self.station is not None:
            return
        mhz, since = pending
        if round(self.state.com1_active or 0.0, 3) != mhz:
            self._empty_pending = None
            return
        now = time.monotonic() if now is None else now
        if now - since < EMPTY_NOTICE_S:
            return
        self._empty_pending = None
        self._reported_empty = mhz
        state = self.state
        lat, lon = (state.latitude, state.longitude) if state.has_position             else (0.0, 0.0)
        self._say_empty(lat, lon, mhz)

    def _say_empty(self, lat: float, lon: float, mhz: float) -> None:
        """What to say about a frequency nobody answers: the field, what it
        does publish, and the nearest of those when it is a likely mistune.

        One translated line with its values as parameters. It used to be a
        translated head with an English hint glued on as a parameter, half in
        each language, and the hint told the pilot to edit a CSV file and
        run a Python command, which a shipped build has no way to do.
        """
        heard = f"{mhz:.3f}"
        airport = None
        if self.state.has_position:
            airport = self.navdb.home_airport(lat, lon)                 or self.navdb.nearest_airport(lat, lon, max_nm=30.0, min_rank=1)
        stations = self.navdb.stations(airport.ident) if airport else []
        if airport is None or not stations:
            self.emit("info",
                      f"{heard} - nobody is listening on this frequency here.",
                      key="log.empty_frequency_alone", mhz=heard)
            return

        # Every position a pilot can call, the information services included:
        # Kortrijk has nothing else, and its list came out empty.
        order = ["ATIS", "DEL", "GND", "TWR", "DEP", "APP", "CTR",
                 "INFO", "CTAF", "AWOS"]
        seen: dict[str, float] = {}
        for st in sorted(stations, key=lambda s: (s.priority, s.mhz)):
            if st.position in order and st.position not in seen:
                seen[st.position] = st.mhz
        listing = ", ".join(f"{position} {seen[position]:.3f}"
                            for position in order if position in seen)
        field = f"{airport.spoken} ({airport.ident})"
        text = (f"{heard} - nobody is listening on this frequency at {field}."
                f" Published there: {listing}.")
        # A near-miss is worth naming: 127.850 against a published 127.750 is
        # far more likely to be a mistuned radio than a missing frequency.
        closest = min(stations, key=lambda s: abs(s.mhz - mhz))
        if abs(closest.mhz - mhz) <= 0.2:
            self.emit("info", text + f" Did you mean {closest.mhz:.3f}"
                      f" ({closest.callsign})?",
                      key="log.empty_frequency_near", mhz=heard, field=field,
                      listing=listing, near=f"{closest.mhz:.3f}",
                      station=closest.callsign)
            return
        self.emit("info", text, key="log.empty_frequency", mhz=heard,
                  field=field, listing=listing)

