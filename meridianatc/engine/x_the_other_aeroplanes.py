from __future__ import annotations

from ..sim.state import combustion
from ._shared import (
    RADIO,
    STATION_RANGE_NM,
    AircraftState,
    Candidate,
    FlightSession,
    Instruction,
    Intent,
    ParsedIntent,
    Phase,
    Station,
    UserAircraft,
    XPlaneTraffic,
    announcements_missing,
    as_written,
    capabilities,
    command_hint,
    decision,
    fittings_for,
    fuselage_length,
    handling_spots,
    hashlib,
    haversine_nm,
    leaving_departure,
    lod_windows,
    log,
    math,
    normalize_runway,
    replace,
    same_channel,
    spots_for,
    station_key,
    systems_wanted,
    threading,
    time,
    timed,
)
from .x_speaking import _render_count


class _The_other_aeroplanes:
    # ------------------------------------------------------------------
    # the other aeroplanes
    # ------------------------------------------------------------------

    def _run_traffic(self, state: AircraftState, session: FlightSession) -> None:
        """Speak whatever the other traffic is saying on this frequency.

        The injector runs whether or not the user is listening -- aeroplanes
        keep taxiing when you change frequency -- but only what is said on the
        frequency in COM1 reaches the speakers.
        """
        station = self.station
        self.traffic.can_invent = self.simtraffic.drawing
        # The plan actually loaded in the sim's FMS, when X-Plane's
        # plugin can see it: a corroborating route for the First
        # Officer's procedure picks, read off the status channel the
        # plugin already reports on.
        xp = self.simtraffic
        self.brain.fms_idents = (xp.fms_idents()
                                 if isinstance(xp, XPlaneTraffic) else ())
        # A declared emergency clears the frequency for the pilot: the
        # other crews hold their calls until it stands down.
        self.traffic.radio_silence = bool(
            getattr(session, "emergency", "") or "") if session else False
        if station is None or not station.ident:
            # Nobody to talk to, so nothing is said -- but the airport is
            # still there. Whoever is already flying keeps flying, and the
            # field under the aeroplane is worked silently: its stands set
            # out, FSLTL's aeroplanes read off the simulator and handled,
            # all of it on the map. A pilot on 122.800 at the gate used to
            # see an empty airport with nobody on it, because every one of
            # those things waited for a controller to be tuned.
            self._work_the_field_here(state, session)
            self.traffic.advance(time.monotonic())
            self._keep_traffic_real(time.monotonic())
            return

        from ..atc.enroute import is_oceanic

        if station.position == "CTR" and not is_oceanic(station):
            # A centre's frequency is worked by its own traffic
            # (atc/centre_traffic.py); the field's goes on silently, as
            # with nobody tuned. Pointing the field's injector at the FIR
            # was an airfield with no runway and an hour of silence.
            self._work_the_field_here(state, session)
            now = time.monotonic()
            self.traffic.advance(now)
            self._keep_traffic_real(now)
            self._work_the_centre(state, session, station, now)
            return

        # The density is a live setting, so it is read here rather than only
        # at startup: turning the frequency quiet has to take effect now, not
        # on the next run.
        self.traffic.density = self.config.traffic.density
        self.traffic.expedite = getattr(self.config.traffic, "expedite", True)
        self.brain.traffic = self.traffic

        # The aeroplanes somebody else put in the simulator, before anything
        # else: how many of them are on the field is what decides how many
        # more to invent, and a strip for one of them has to exist before the
        # tower looks at the field.
        with timed("tick.traffic.observed", warn_over_ms=500):
            self._take_observed(session)

        # And where the user is, so the traffic works a field with the
        # aeroplane that matters on it rather than around a hole. A departure
        # is held for the user on final, and an AI on short final to a runway
        # the user is sitting on is sent around.
        self.traffic.set_user(UserAircraft(
            callsign=session.aircraft.callsign.written,
            type_code=session.aircraft.type_code,
            latitude=state.latitude,
            longitude=state.longitude,
            altitude_ft=state.altitude_ft,
            heading=state.heading_true,
            ground_speed_kt=state.ground_speed_kt,
            on_ground=state.on_ground,
        ) if state.has_position else None)

        # The traffic uses the runway the user has been given, so the whole
        # field is working one configuration rather than two.
        self.traffic.set_field(
            station.ident,
            departure_runway=session.departure_runway,
            arrival_runway=session.arrival_runway or session.departure_runway,
            weather=self._weather_cache.get(station.ident),
            tuned=station,
        )

        with timed("tick.traffic.ground", warn_over_ms=500):
            self._follow_ground(station.ident)

        now = time.monotonic()
        if self.autonomous_flight is not None:
            self.traffic.enroute_cells = (
                lambda af=self.autonomous_flight, st=state:
                af._enroute_weather_cells(st))
        # Whatever has fallen due joins the back of the queue. It is not spoken
        # here: the frequency may be busy, and a transmission that cannot be
        # heard has to wait rather than disappear.
        with timed("tick.traffic.poll", warn_over_ms=500):
            self._traffic_queue.extend(
                self.traffic.poll(now, station.position, station.mhz))
        # On a frequency without a controller, crews coordinate among
        # themselves -- which includes asking who the new voice is.
        if self.config.traffic.pilots_talk and self.session is not None:
            try:
                self.traffic.maybe_call_the_user(
                    self.session, station.position, now)
            except Exception:
                log.debug("a crew calling the user failed", exc_info=True)
        with timed("tick.traffic.speak", warn_over_ms=500):
            self._speak_from_the_queue(station, now)

        # And the same aeroplanes, where they can be seen. This is every
        # flight the injector is running, not only the ones on this frequency:
        # an aeroplane does not vanish from the taxiway because you changed
        # channel.
        # Only the invented ones: the rest are already in the simulator, and
        # creating a second copy of an FSLTL aeroplane a foot from the first is
        # the one way this could make the window worse rather than better.
        with timed("tick.traffic.real", warn_over_ms=500):
            self._keep_traffic_real(now)

    centre_traffic = None

    def _work_the_centre(self, state: AircraftState, session: FlightSession,
                         station: Station, now: float) -> None:
        """The other flights on the centre frequency, invented and, with
        live traffic on, real; spoken through the same queue as a field's."""
        from ..atc.centre_traffic import CentreTraffic

        if not state.has_position:
            return
        if self.centre_traffic is None:
            self.centre_traffic = CentreTraffic(self.navdb,
                                                profile=self.traffic.profile)
        live = self._live_seen() if self.live_feed is not None else []
        weather_cells = ()
        if self.autonomous_flight is not None:
            try:
                weather_cells = self.autonomous_flight._enroute_weather_cells(state)
            except Exception:
                log.debug("centre weather scan failed", exc_info=True)
        try:
            calls = self.centre_traffic.poll(
                now, station, latitude=state.latitude,
                longitude=state.longitude, altitude_ft=state.altitude_ft,
                own_callsign=session.aircraft.callsign.written, live=live,
                density=self.config.traffic.density,
                weather_cells=weather_cells)
        except Exception:
            log.debug("the centre's traffic raised", exc_info=True)
            return
        self._traffic_queue.extend(calls)
        self._speak_from_the_queue(station, now)

    def ground_layout(self, ident: str):
        """This airport's stands and taxiways, if the simulator has given
        them; asked for in the background if it has not yet."""
        ident = (ident or "").upper()
        layout = self._ground_ready.get(ident)
        if layout is None:
            self._ask_for_ground(ident)
        return layout

    def ground_pending(self, ident: str) -> bool:
        """Whether this airport's taxiways are still being read -- an
        answer that is coming, told apart from one that never did: the
        airport was asked, is not yet read, and its read did not fail."""
        ident = (ident or "").upper()
        return (ident in self._ground_asked
                and ident not in self._ground_ready
                and ident not in self._ground_dead)

    # How close to the destination a layout is asked for whatever the phase
    # says. The read takes seconds on a worker; at 250 knots sixty miles is a
    # quarter of an hour, and a pilot who never called for descent is still
    # descending.
    LAYOUT_AHEAD_NM = 60.0

    def follow_ground_layouts(self, state: AircraftState,
                              session: FlightSession) -> None:
        """Ask for the taxiways the pilot is going to be taxied on.

        The traffic asks only for the field it is working, and only when it
        is switched on, which left the pilot's own departure field unread
        with the traffic off. So: the field the aeroplane is on the ground
        at, as soon as it is there, and the destination during the descent,
        well before the taxi in is asked for -- the read is asynchronous and
        the controller never waits for it.
        """
        if not state.has_position:
            return
        if state.on_ground:
            here = self.navdb.home_airport(state.latitude, state.longitude)
            if here is not None:
                self._ask_for_ground(here.ident)
            return
        destination = session.destination
        if destination is None:
            return
        descending = session.phase in (Phase.DESCENT, Phase.APPROACH,
                                       Phase.LANDING)
        if descending or self.brain._distance_to(state, destination) \
                <= self.LAYOUT_AHEAD_NM:
            self._ask_for_ground(destination.ident)

    def _work_the_field_here(self, state: AircraftState,
                             session: FlightSession) -> None:
        """Point the traffic at the airport the aeroplane is at, with nobody
        tuned to say so, and read the simulator's own traffic there.

        Only when the field can be told: an aeroplane on the ground, or
        within a few miles of one. Airborne and far from anywhere, whatever
        field was being worked stays.
        """
        if not state.has_position:
            return
        airport = self.navdb.home_airport(state.latitude, state.longitude)
        if airport is None:
            return
        self.traffic.set_field(
            airport.ident,
            departure_runway=session.departure_runway,
            arrival_runway=session.arrival_runway or session.departure_runway,
            weather=self._weather_cache.get(airport.ident),
        )
        self._follow_ground(airport.ident)
        self.traffic.set_user(UserAircraft(
            callsign=session.aircraft.callsign.written,
            type_code=session.aircraft.type_code,
            latitude=state.latitude,
            longitude=state.longitude,
            altitude_ft=state.altitude_ft,
            heading=state.heading_true,
            ground_speed_kt=state.ground_speed_kt,
            on_ground=state.on_ground,
        ))
        self._take_observed(session)

    def _follow_ground(self, ident: str) -> None:
        """Give the traffic this airport's taxiways, once the simulator has
        said what they are."""
        ident = (ident or "").upper()
        if not ident or self.traffic.field.ident != ident:
            return
        layout = self._ground_ready.get(ident)
        if layout is not None:
            if self.traffic.ground is None:
                self.traffic.set_ground(layout)
            return
        self._ask_for_ground(ident)

    def _ask_for_ground(self, ident: str) -> None:
        if not ident or ident in self._ground_asked:
            return
        if not self.state.connected:
            return
        # Whichever simulator is asked, it is asked its own way: the
        # facility API on SimConnect, the apt.dat scenery is loaded from
        # on X-Plane.
        source = capabilities(self.state.source)
        if source.facilities:
            reader = self.simground
        elif self.state.source == "xplane":
            reader = self.xpground
        else:
            return
        self._ground_asked.add(ident)

        def read() -> None:
            try:
                found = reader.layout(ident)
            except Exception:
                log.debug("could not read %s's taxiways", ident, exc_info=True)
                found = None
            if found is not None:
                log.info("%s: %d stands and %d taxi nodes from %s",
                         ident, len(found.stands), len(found.xy),
                         "the simulator" if reader is self.simground
                         else "apt.dat")
                self._ground_ready[ident] = found
            else:
                self._ground_dead.add(ident)

        threading.Thread(target=read, name=f"MeridianATC-taxi-{ident}",
                         daemon=True).start()

    runway_cache = None

    def _use_runway_cache(self) -> None:
        from ..navdata.runway_cache import RunwayCache
        from ..paths import data_root

        self.runway_cache = RunwayCache(data_root() / "airports")
        self.navdb.use_runway_cache(self.runway_cache)

    def say_runway_source(self, ident: str) -> None:
        """A row for ``logs radio`` and ``logs arrival``: where the field's
        runways come from (sim, cache or db), and, when the simulator's,
        what they disagree with the database on -- the reason the pilot
        heard "26" where the chart says "25". Once per field and source."""
        if not ident:
            return
        source = self.navdb.runway_source(ident)
        if self._runways_said.get(ident) == source:
            return
        self._runways_said[ident] = source
        ends = sorted({end for rw in self.navdb.runways(ident)
                       for end, *_ in rw.ends()})
        decision("airport", airport=ident, runways_from=source, ends=ends,
                 **self.navdb.runway_differences(ident))

    def _procedure_sources(self):
        """The procedure and approach readers for the active simulator.

        MSFS answers over SimConnect; X-Plane's equivalent is the CIFP
        navdata on its own disk, decoded into the same model. ``(None,
        None)`` when the source has neither, which is what a session
        without either gets.
        """
        if capabilities(self.state.source).facilities:
            return self.simprocedures, self.simapproaches
        if self.state.source == "xplane":
            source = self._cifp_source()
            if source is not None:
                return source, source
        return None, None

    def _cifp_source(self):
        """The CIFP provider for the X-Plane install, built once its
        root is known: the plugin reports its own system path in its
        status, and Laminar's install list is the fallback."""
        if self._cifp is not None:
            return self._cifp
        from pathlib import Path
        roots = []
        candidates = []
        sysroot = getattr(getattr(self, "simtraffic", None), "sysroot", "")
        if sysroot:
            candidates.append(Path(sysroot))
        try:
            from ..xplane_plugin import install_roots
            candidates.extend(install_roots())
        except Exception:
            log.debug("could not list X-Plane installs", exc_info=True)
        for root in candidates:
            for sub in (root / "Custom Data",
                        root / "Resources" / "default data"):
                if sub.is_dir() and sub not in roots:
                    roots.append(sub)
        if not roots:
            return None
        from ..integrations.cifp import CifpSource
        self._cifp = CifpSource(roots)
        log.info("X-Plane procedures from CIFP: %s",
                 ", ".join(str(r) for r in roots))
        return self._cifp

    def _ask_for_terminal_procedures(self, ident: str) -> None:
        """The SIDs and STARs an airport publishes, once per field.

        Asked on its own thread like the approaches: the facility data is
        a paged walk of airport, procedures, transitions and legs, and
        none of it should stall the controller's loop. The answer lands
        in ``brain.terminal_procedures`` for the First Officer.
        """
        if not ident or ident in self._procedures_asked:
            return
        procedures_reader, _ = self._procedure_sources()
        if procedures_reader is None or not self.state.connected:
            return
        self._procedures_asked.add(ident)

        def read() -> None:
            try:
                found = procedures_reader.procedures(ident)
            except Exception:
                log.debug("could not read %s's procedures", ident,
                          exc_info=True)
                return
            if found is not None:
                self.brain.terminal_procedures[ident] = found
                log.info("%s: terminal procedures from %s: "
                         "%d departures, %d arrivals",
                         ident, self.state.source,
                         len(found.departures), len(found.arrivals))

        threading.Thread(target=read, name=f"MeridianATC-procedures-{ident}",
                         daemon=True).start()

    def _ask_for_approaches(self, ident: str) -> None:
        # Said whether or not the simulator can be asked: without it, the
        # row says the cache or the database is what is being used.
        self.say_runway_source(ident)
        self._ask_for_terminal_procedures(ident)
        if not ident or ident in self._approaches_asked:
            return
        _, approaches_reader = self._procedure_sources()
        if approaches_reader is None or not self.state.connected:
            return
        self._approaches_asked.add(ident)

        def read() -> None:
            try:
                found = approaches_reader.approaches(ident)
            except Exception:
                log.debug("could not read %s's approaches", ident, exc_info=True)
                return
            if found is not None:
                log.info("%s: approaches from the simulator: %s", ident,
                         {rw: sorted(kinds) for rw, kinds in found.by_runway.items()})
                self.brain.approach_data[ident] = found
                # Any ATIS already issued for the field named the ILS; the
                # next one is re-issued on what is really there. Marked here
                # and done by atis_for on the engine's own thread: walking
                # the table from this worker raced the engine adding to it.
                with self._atis_lock:
                    self._atis_stale.add(ident)
            # The runways themselves, and from the same answer each one's
            # pattern altitude: the scenery's are the ones the aeroplane
            # rolls on, so from here they are the field's. SimConnect asks
            # the facilities API; X-Plane's equivalent is the apt.dat the
            # scenery packs carry.
            runways_reader = self.simrunways \
                if capabilities(self.state.source).facilities \
                else self.xpground
            try:
                runways = runways_reader.for_airport(ident)
                patterns = runways_reader.patterns(ident)
            except Exception:
                log.debug("could not read %s's runways", ident, exc_info=True)
                return
            if runways:
                from ..integrations.runways import LOCALIZER_KINDS, with_ils_kinds

                if capabilities(self.state.source).facilities:
                    # Which end's navaid is a localizer and which a VOR, from
                    # the approaches read just above.
                    runways = with_ils_kinds(runways, found)
                else:
                    # The apt.dat ends have no navaid idents, so only the
                    # approaches can say an end has a localizer at all.
                    from dataclasses import replace
                    kinds = getattr(found, "by_runway", {}) or {}
                    runways = tuple(
                        replace(rw,
                                le_ils_kind="ILS" if kinds.get(
                                    normalize_runway(rw.le_ident), set())
                                & LOCALIZER_KINDS else "",
                                he_ils_kind="ILS" if kinds.get(
                                    normalize_runway(rw.he_ident), set())
                                & LOCALIZER_KINDS else "")
                        for rw in runways)
                self.navdb.set_sim_runways(ident, runways)
                cache = getattr(self, "runway_cache", None)
                if cache is not None:
                    cache.save(ident, runways)
                self.say_runway_source(ident)
            if patterns:
                self.brain.sim_patterns[ident] = patterns

        threading.Thread(target=read, name=f"MeridianATC-approaches-{ident}",
                         daemon=True).start()

    def _keep_traffic_real(self, now: float) -> None:
        """Have the simulator draw the invented traffic, and keep only what
        it has drawn.

        An aeroplane is asked of the simulator first and joins the frequency
        once it is there; one the simulator cannot make is dropped without a
        word ever having been said about it.
        """
        positions = self.traffic.positions(invented_only=True)
        if self.state.has_position:
            # For how often each is drawn: thirty-five times a second near
            # the pilot, five at ten kilometres.
            self.simtraffic.viewer = (self.state.latitude, self.state.longitude)
            # And how far out any of them are drawn at all
            # (traffic.radius_nm): past it the simulator is not asked, on
            # either simulator.
            radius_nm = float(self.config.traffic.radius_nm or 0.0)
            if radius_nm > 0.0:
                positions = [p for p in positions
                             if haversine_nm(self.state.latitude,
                                             self.state.longitude,
                                             p["lat"], p["lon"]) <= radius_nm]
        self._choose_models(positions, now)
        # Who flies the AI-plane slots the field's own flights did not
        # claim (traffic.xplane_fill, native renderer only): "ambient"
        # fills them with our transit aeroplanes on real legs, "none"
        # leaves them empty, "sim" hands them back to X-Plane's own AI.
        # Either way the exclusive flag tells the plugin not to let the
        # simulator spawn into a gap while the roster is short.
        if getattr(self.simtraffic, "native", False):
            fill = getattr(self.config.traffic, "xplane_fill", "sim")
            self.simtraffic.exclusive = fill in ("ambient", "none")
            self.traffic.ambient_fill = fill == "ambient"
            if self.traffic.ambient_fill:
                room = self.simtraffic.MAX_SLOTS - len(positions)
                if room > 0:
                    positions += self.traffic.ambient_positions(now, room)
        else:
            self.traffic.ambient_fill = False
        with timed("tick.traffic.sync", warn_over_ms=500):
            self.simtraffic.sync(positions, now)
        self._follow_me_car()
        self._taxi_route_overlay()
        confirmed, dropped = self.traffic.confirm(
            self.simtraffic.has, self.simtraffic.failed, now)
        with timed("tick.traffic.handling", warn_over_ms=500):
            self._keep_handling_real(now)
        with timed("tick.traffic.systems", warn_over_ms=500):
            self._keep_systems_real(now)
        self._say_traffic_status(now)
        self._say_motion_status(now)
        # What was already queued from an aeroplane that no longer exists
        # is not said either: a strip finished since the call was made
        # speaks for nobody, drawing or not. And when the simulator is
        # not drawing, nothing of ours is out there to speak for either.
        self._traffic_queue = [c for c in self._traffic_queue
                               if not c.flight.done
                               and (self.simtraffic.drawing or c.flight.observed)]
        for flight in confirmed:
            runway = self._spawned.pop(flight.written, None)
            if runway is None:
                continue
            if flight.phase == "lineup":
                self.emit("info",
                          f"{flight.written} is lined up on runway {runway}.",
                          key="log.traffic_spawned", callsign=flight.written,
                          runway=runway)
            else:
                self.emit("info",
                          f"{flight.written} is holding short of runway "
                          f"{runway} until the runway is free.",
                          key="log.traffic_spawned_holding",
                          callsign=flight.written, runway=runway)
        for flight in dropped:
            if self._spawned.pop(flight.written, None) is not None:
                self.emit("info",
                          f"The simulator could not create {flight.written}, "
                          f"so it is not on the runway.",
                          key="log.traffic_spawn_failed",
                          callsign=flight.written)

    # How far ahead of the aeroplane the follow-me car drives, and how close
    # to the clearance limit counts as arrived.
    FOLLOWME_LEAD_M = 35.0

    def _follow_me_car(self) -> None:
        """The follow-me car on the pilot's cleared route, X-Plane only.

        While a taxi route stands, the car drives it a short way ahead of
        the aeroplane to the clearance limit; the simulator draws it from
        the position the route says, not from anything it can see. Off the
        route it simply waits -- deviation is TaxiWatch's call.
        """
        from ..atc import taxi
        xp = self.simtraffic
        session, state = self.session, self.state
        car = None
        if (self.config.traffic.follow_me and session is not None
                and state is not None and state.has_position
                and state.on_ground
                and session.phase in (Phase.TAXI_OUT, Phase.TAXI_IN)):
            car = taxi.car_ahead(state.latitude, state.longitude,
                                 session.taxi_path, self.FOLLOWME_LEAD_M)
        if car is not None and isinstance(xp, XPlaneTraffic) and xp.available:
            xp.followme(True, *car)
            self._followme_on = True
        elif getattr(self, "_followme_on", False):
            if isinstance(xp, XPlaneTraffic):
                xp.followme(False)
            self._followme_on = False

    # A standing route is re-sent on this clock; the plugin lets one that
    # stops arriving fade, so a silent engine leaves no stale line.
    ROUTE_RESEND_S = 10.0

    def _taxi_route_overlay(self) -> None:
        """The cleared taxi route on X-Plane's own map, X-Plane only.

        While a taxi route stands the plugin draws it as a markings layer,
        from the same ``session.taxi_path`` the follow-me car drives. A
        changed route goes at once; a standing one is re-sent on the
        keep-alive clock the plugin expires it by.
        """
        xp = self.simtraffic
        session = self.session
        path = (list(session.taxi_path)
                if session is not None
                and session.phase in (Phase.TAXI_OUT, Phase.TAXI_IN)
                else [])
        sent = getattr(self, "_route_sent", None)
        if isinstance(xp, XPlaneTraffic) and xp.available:
            now = time.monotonic()
            if path and (path != sent
                         or now - getattr(self, "_route_sent_at", 0.0)
                         > self.ROUTE_RESEND_S):
                if xp.route(path):
                    self._route_sent = path
                    self._route_sent_at = now
            elif not path and sent is not None and xp.route([], on=False):
                self._route_sent = None
        elif sent is not None:
            self._route_sent = None

    def _choose_models(self, positions: list[dict], now: float) -> None:
        """Mark each invented aeroplane with the level of detail it should
        be drawn at, for the injector to follow.

        Only with LOD on: without it no position carries a level and the
        injector draws what it always drew. One not yet drawn is created
        at the level its distance says; one standing on its stand is
        swapped when the decision says, so many a second; one moving is
        left as it is until it parks.
        """
        if isinstance(self.simtraffic, XPlaneTraffic) or not self.lod.enabled:
            return
        if self.lod.scheduled:
            self._choose_models_by_schedule(positions, now)
            return
        try:
            user = self.traffic.user
            user_xy = None
            layout = self.traffic.ground
            if user is not None and user.has_position:
                if layout is not None:
                    user_xy = layout.to_xy(user.latitude, user.longitude)
                else:
                    user_xy = (0.0, 0.0)

            def xy(position: dict) -> tuple[float, float]:
                if layout is not None:
                    return layout.to_xy(position["lat"], position["lon"])
                # Metres east and north of the pilot, near enough.
                from ..navdata.db import haversine_nm
                east = haversine_nm(user.latitude, user.longitude,
                                    user.latitude, position["lon"]) * 1852.0
                north = haversine_nm(user.latitude, user.longitude,
                                     position["lat"], user.longitude) * 1852.0
                return (east if position["lon"] >= user.longitude else -east,
                        north if position["lat"] >= user.latitude else -north)

            detailed_now = sum(
                1 for p in positions
                if self.simtraffic.target_level(p["callsign"]) == "detailed")
            candidates: list[Candidate] = []
            for position in positions:
                callsign = position["callsign"]
                has_detailed = self.simtraffic.has_detailed(
                    position.get("type", ""), position.get("operator", ""))
                if not self.simtraffic.has(callsign):
                    distance = None
                    if user_xy is not None:
                        x, z = xy(position)
                        distance = math.hypot(x - user_xy[0], z - user_xy[1])
                    level = self.lod.level_for_new(distance, detailed_now,
                                                   has_detailed)
                    if level == "detailed":
                        detailed_now += 1
                    position["level"] = level
                    continue
                # Stamped with where it is going, not where it was: on
                # the tick a swap finishes, the old level would be read as
                # an order to swap back.
                level = self.simtraffic.target_level(callsign)
                position["level"] = level
                busy = self.simtraffic.swapping(callsign)
                stationary = bool(position.get("on_ground")) and not position.get(
                    "ground_speed_kt") and (position.get("static")
                                            or position.get("intent") == "parked")
                x, z = xy(position) if user_xy is not None else (0.0, 0.0)
                candidates.append(Candidate(callsign, x, z, level=level,
                                            stationary=stationary,
                                            has_detailed=has_detailed,
                                            busy=busy))
            by_callsign = {p["callsign"]: p for p in positions}
            for callsign, level in self.lod.decide(candidates, user_xy, now):
                by_callsign[callsign]["level"] = level
        except Exception:
            log.debug("choosing the models raised", exc_info=True)

    def _apron_window(self, callsign: str, kind: str, now: float) -> str:
        """Where one of this program's aeroplanes is in its turnaround, from
        what the traffic, the ramp and the doors each know of it."""
        object_id = self.simtraffic.object_id(callsign)
        doors_open = self.simdoors.open_now(object_id)
        if kind == "booked":
            return lod_windows.PRE_BOARDING
        if kind == "boarding":
            return lod_windows.BOARDING
        if kind == "departing":
            # Boarding until every door reads shut, then waiting for the tug.
            if self.simdoors.can_push(object_id) and not doors_open:
                return lod_windows.CLOSED
            return lod_windows.BOARDING
        if kind == "arrived":
            # Blocked in and still on the radio: inside its deplaning round
            # by definition. Asking the ramp instead read it as closed up
            # on the tick it stopped -- the ramp had not seen it yet -- and
            # it was lightened at the gate and detailed again a tick later.
            return lod_windows.ARRIVAL
        if kind == "static":
            if self.handling.arrival_open(callsign, now) or doors_open:
                return lod_windows.ARRIVAL
            return lod_windows.PARKED
        return lod_windows.MOVING

    def _choose_models_by_schedule(self, positions: list[dict],
                                   now: float) -> None:
        """The schedule's choice of model (traffic.lod.mode: schedule).

        Detailed only in the turnaround window, near the pilot and under the
        cap; created detailed where it can be, which is an arrival whose
        stand is near and the departure booked beside the pilot, so that it
        never has to change at the gate. See meridianatc.atc.lod.
        """
        try:
            user = self.traffic.user
            layout = self.traffic.ground
            user_xy = None
            if (user is not None and user.has_position
                    and layout is not None):
                user_xy = layout.to_xy(user.latitude, user.longitude)
            detailed_now = sum(
                1 for p in positions
                if self.simtraffic.target_level(p["callsign"]) == "detailed")
            roles = self.handling.doors(now)
            candidates: list[Candidate] = []
            for position in positions:
                callsign = position["callsign"]
                type_code = position.get("type", "")
                operator = position.get("operator", "")
                has_detailed = self.simtraffic.has_detailed(type_code, operator)
                matches = self.simtraffic.livery_matches(type_code, operator,
                                                         callsign)
                kind, stand = self.traffic.apron_state(callsign)
                if layout is not None:
                    here = layout.to_xy(position["lat"], position["lon"])
                else:
                    here = None
                # Measured from where it will turn round while it is on
                # its way there; from where it is otherwise.
                where = stand if kind == "arriving" and stand else here
                if not self.simtraffic.has(callsign):
                    window = (lod_windows.ARRIVAL if kind == "arriving"
                              and stand else self._apron_window(
                                  callsign, kind, now))
                    distance = None
                    if user_xy is not None and where is not None:
                        distance = math.hypot(where[0] - user_xy[0],
                                              where[1] - user_xy[1])
                    level = self.lod.level_for_new(
                        distance, detailed_now, has_detailed, window=window,
                        livery_matches=matches)
                    if level == "detailed":
                        detailed_now += 1
                    position["level"] = level
                    continue
                level = self.simtraffic.target_level(callsign)
                position["level"] = level
                window = self._apron_window(callsign, kind, now)
                object_id = self.simtraffic.object_id(callsign)
                flight = self.traffic.flight_named(callsign)
                if (flight is not None and flight.by_hand
                        and self.lod.livery != "match"):
                    # Boarded with the test button: its doors are what the
                    # pilot asked to see, so the hide rule does not apply.
                    matches = True
                stationary = bool(position.get("on_ground")) and not position.get(
                    "ground_speed_kt") and kind in ("static", "booked",
                                                    "boarding", "departing",
                                                    "arrived")
                x, z = where if where is not None else (0.0, 0.0)
                candidates.append(Candidate(
                    callsign, x, z, level=level, stationary=stationary,
                    has_detailed=has_detailed,
                    busy=self.simtraffic.swapping(callsign),
                    window=window,
                    doors_open=bool(roles.get(callsign))
                    or self.simdoors.open_now(object_id),
                    doors_moving=self.simdoors.moving(object_id, now),
                    livery_matches=matches))
            by_callsign = {p["callsign"]: p for p in positions}
            for callsign, level in self.lod.decide(candidates, user_xy, now):
                by_callsign[callsign]["level"] = level
        except Exception:
            log.debug("choosing the models by schedule raised", exc_info=True)

    def _read_the_doors(self, standing: list, now: float) -> None:
        """Where each vehicle goes, from each model's own doors: ask the
        simulator once per model, and hand every aeroplane its spots."""
        points_reader = getattr(self, "servicepoints", None)
        if points_reader is None or not points_reader.available:
            return
        titles = {role: self.simvehicles.title_for(role)
                  for role in list(self._service_layout.get("roles", {}))
                  + list(handling_spots.SPOTS)}
        for one in standing:
            if one.observed:
                object_id, title = one.object_id, one.title
            else:
                object_id = self.simtraffic.object_id(one.callsign)
                title = self.simtraffic.title_of(one.callsign)
            if not object_id or not title:
                continue
            points = points_reader.points(title)
            if points is None:
                points_reader.ask(object_id, title, now)
                continue
            length = handling_spots.fuselage_length(one.type_code, one.light)
            fallback = {role: (right, fraction * length + extra, facing)
                        for role, (right, fraction, extra, facing)
                        in handling_spots.SPOTS.items()}
            # How high each door is on this model, and a loader docked by
            # its own belt: both from the points and the height the model
            # stands at (servicepoints.standing).
            # This aeroplane's own height where it is one of ours: the one
            # read with the points is another object's, at another moment.
            standing = None if one.observed else (
                self.simtraffic.height_above_ground(one.callsign))
            pitch = 0.0
            if standing is None:
                standing = points_reader.standing(title)
            else:
                pitch = self.simtraffic.resting_pitch(one.callsign)
            fittings = fittings_for(points, standing,
                                    self._service_layout, titles, pitch,
                                    listed=self.simvehicles.listed_gsx())
            chosen = dict(titles)
            for role, fitting in fittings.items():
                if fitting.get("title"):
                    chosen[role] = fitting["title"]
                    said = (one.callsign, role, fitting["title"])
                    if said not in self._stairs_said:
                        self._stairs_said.add(said)
                        log.info("stairs: %s %s, sill %.2f m: %s (%s)", one.callsign,
                                 role, fitting["sill"], fitting["title"], fitting["why"])
            one.fittings = fittings or None
            one.spots = spots_for(points, self._service_layout, chosen,
                                  one.type_code, fallback, fittings) or None

    def _keep_systems_real(self, now: float) -> None:
        """This program's aeroplanes' engines and lights, by where each is
        in its day (meridianatc.atc.systems). Only objects this program drew;
        never the pilot's, never anybody else's."""
        simsystems = getattr(self, "simsystems", None)
        if simsystems is None or not simsystems.available:
            return
        try:
            simsystems.user_object_id = getattr(self.simwatch,
                                                "user_object_id", None)
            elevation = self.traffic.field.elevation_ft or 0.0
            wanted = {}
            for position in self.traffic.positions(invented_only=True):
                callsign = position["callsign"]
                flight = self.traffic.flight_named(callsign)
                if flight is None:
                    continue
                kind, _ = self.traffic.apron_state(callsign)
                objects = self.simtraffic.objects_of(callsign)
                if not objects:
                    continue
                goal = systems_wanted(
                    kind, flight.phase, now, parked_at=flight.parked_at,
                    pushing=flight.target.intent == "pushing back",
                    doors_shut=self.simdoors.can_push(objects[0]),
                    altitude_agl_ft=max(0.0, flight.altitude_ft - elevation),
                    on_ground=flight.on_ground, callsign=callsign)
                for object_id in objects:
                    wanted[object_id] = (callsign, goal)
            simsystems.sync(wanted, now)
        except Exception:
            log.debug("the engines and lights raised", exc_info=True)

    def _model_swapped(self, callsign: str, old: int, new: int,
                       title: str) -> None:
        """An aeroplane has changed object: its doors go with it."""
        self.simdoors.carry(old, new, title)

    def _model_removed(self, object_id: int, what: str) -> None:
        """An object of ours has been taken out of the simulator. The door
        layer keeps an aeroplane it no longer wants until its doors read
        shut, and one that has gone never reads anything again: at
        Innsbruck (15 October) an arrival removed as its doors were closing
        was asked after every four seconds for the rest of the session,
        each one an UNRECOGNIZED_ID."""
        self.simdoors.forget(object_id)

    def _may_push(self, callsign: str) -> bool:
        """Whether this program's aeroplane may push: every door the door
        layer knows of read shut. One it is not working is not held."""
        return self.simdoors.can_push(self.simtraffic.object_id(callsign))

    def _keep_handling_real(self, now: float) -> None:
        """Send the ground handling to the aeroplanes the simulator has
        drawn on their stands, and keep the vehicles where the turnaround
        says they are.

        Only aeroplanes the simulator has actually made: a baggage loader
        at an empty stand is the one thing worse than an empty one. The
        turnaround runs whether or not the simulator will draw vehicles, so
        the decision is cheap and the drawing fails closed on its own.
        """
        # The turnaround runs whenever handling is on: with vehicles off
        # (another tool draws them) the doors still follow its schedule.
        if not self.config.traffic.handling.enabled:
            return
        try:
            # Somebody else's parked aeroplanes are in the simulator by
            # definition; this program's only once it has drawn them.
            handle_observed = self.config.traffic.handling_observed
            # The pilot's own aeroplane, as the simulator names it: never
            # handled, its doors never touched (see SimDoors.is_the_pilot).
            pilot = getattr(self.simwatch, "user_object_id", None)
            if hasattr(self.simdoors, "user_object_id"):
                self.simdoors.user_object_id = pilot
            standing = [one for one in self.traffic.standing()
                        if ((one.observed and handle_observed)
                            or (not one.observed
                                and self.simtraffic.has(one.callsign)))
                        and not (one.observed and one.object_id in (0, pilot))]
            layout = self.traffic.ground
            user = self.traffic.user
            user_xy = None
            if layout is not None and user is not None and user.latitude:
                user_xy = layout.to_xy(user.latitude, user.longitude)
            # The pilot's aeroplane, for keeping the buses off it: its type
            # if known, else as long as the longest the buses serve.
            self.handling.pilot_footprint = None
            if user_xy is not None and user.on_ground:
                self.handling.pilot_footprint = (
                    user_xy[0], user_xy[1], float(user.heading),
                    fuselage_length(user.type_code) if user.type_code else 45.0)
            self.handling.elevation_ft = self.traffic.field.elevation_ft
            # Whose doors read shut, for the tug: the door layer's answer
            # where it is working, everybody's otherwise.
            ready = None
            if self.config.traffic.handling_doors and self.simdoors.available:
                ready = set()
                for one in standing:
                    object_id = (one.object_id if one.observed
                                 else self.simtraffic.object_id(one.callsign))
                    if self.simdoors.can_push(object_id):
                        ready.add(one.callsign)
            # The motion engine's rates go by how far each actor is from
            # the pilot, in the layout's metres.
            if hasattr(self.simvehicles, "motion"):
                self.simvehicles.motion.viewer_xy = user_xy
            self._read_the_doors(standing, now)
            vehicles = self.handling.positions(standing, layout, user_xy, now,
                                               ready=ready)
            self.simvehicles.sync(vehicles, now)
            if self.config.traffic.handling_doors:
                # Which simulator object each handled aeroplane is, and
                # which model: this program's by what the injector drew it
                # as, somebody else's by what the reader saw.
                roles = self.handling.doors(now)
                wanted = {}
                for one in standing:
                    if one.observed:
                        object_id, title = one.object_id, one.title
                    else:
                        object_id = self.simtraffic.object_id(one.callsign)
                        title = self.simtraffic.title_of(one.callsign)
                    if object_id:
                        wanted[object_id] = (title, one.callsign,
                                             roles.get(one.callsign, set()))
                self.simdoors.sync(wanted, now)
        except Exception:
            # Once at INFO, because a pass that raises every tick looks,
            # from the apron, exactly like one that is not running.
            if not self._handling_raised:
                self._handling_raised = True
                log.info("the ground handling raised; it is being retried "
                         "every tick", exc_info=True)
            else:
                log.debug("the ground handling raised", exc_info=True)

    def _say_observed(self) -> None:
        """Say once that the simulator's own traffic is being worked.

        Only when the count changes by enough to be news. A pilot who has
        started FSLTL wants to know the radio has noticed it -- "I have a
        hundred aeroplanes out there and the controller is talking to none of
        them" is otherwise a question with no answer on screen -- and a line
        every time an aeroplane taxis out of range is not news.
        """
        # Only the simulator's own aeroplanes. Live ADS-B flights are put
        # through the same observed list so they can be worked, but calling
        # them simulator traffic is wrong -- and in a no-sim session there is
        # no simulator for them to have come from at all.
        count = sum(1 for flight in self.traffic.flights
                    if flight.observed and flight.source != "live")
        if count == self._observed_said:
            return
        if count and abs(count - self._observed_said) < 3 and self._observed_said:
            return
        self._observed_said = count
        if not count:
            return
        self.emit("info",
                  f"Working {count} aircraft from the simulator's own traffic.",
                  key="log.traffic_observed", count=str(count))

    # Nobody waits forever. A transmission still queued this long after it fell
    # due has been overtaken by the flight it belongs to, and saying it now
    # would be worse than not saying it.
    TRAFFIC_STALE_S = 25.0

    # How long an exchange is remembered once something has been decided about
    # it. Its halves reach this queue separately, as each falls due, so the
    # decision about the first has to still be here when the last arrives --
    # and the last of a three-part exchange is queued about six seconds after
    # the first, so a minute and a half is generous rather than tight.
    EXCHANGE_MEMORY_S = 90.0

    def _exchange_heard(self, mark: int, heard: bool, now: float) -> None:
        """Remember what was decided about an exchange, and forget old ones."""
        if not mark:
            return
        self._exchanges[mark] = (heard, now)
        if len(self._exchanges) > 64:
            self._exchanges = {
                key: value for key, value in self._exchanges.items()
                if now - value[1] < self.EXCHANGE_MEMORY_S
            }

    # The chance a flight's first call goes unanswered while the frequency
    # is busy with somebody else's exchange -- never when it is not, because
    # then there was nobody to be talking over. Half of them was a frequency
    # that sounded broken rather than busy. UNVERIFIED: 12% is a figure
    # chosen, not measured.
    MISSED_FIRST_CALL = 0.12

    def _congested(self, station: Station, parsed) -> bool:
        """Whether an exchange between the controller and another aeroplane
        is half said on this frequency: its opening heard
        (``_exchanges``), and the rest still to come -- in this queue, or
        still in the traffic's own schedule, where a readback waits until it
        falls due. Reading only this queue ended the wait in the gap before
        the readback. Never for a distress or urgency call, which is
        answered whatever else is going on."""
        if not self.config.immersion.controller.congestion:
            return False
        if parsed.intent in (Intent.EMERGENCY, Intent.URGENCY,
                             Intent.MINIMUM_FUEL):
            return False
        to_come = list(self._traffic_queue)
        if self.traffic is not None:
            to_come += self.traffic.pending
        for call in to_come:
            if not self._on_this_channel(call, station):
                continue
            decided = self._exchanges.get(call.exchange)
            if decided is not None and decided[0]:
                return True
        return False

    def _first_call_missed(self, session: FlightSession, station: Station) -> bool:
        """Whether this first call goes unanswered, decided once per flight
        and station and seeded from both, so the same flight is treated the
        same way every time it is replayed. The next call is answered."""
        key = station_key(station)
        if session.first_call_missed == key:
            return False
        seed = hashlib.sha256(
            f"{session.aircraft.callsign.written}|{key}".encode()).digest()
        missed = seed[0] / 255.0 < self.MISSED_FIRST_CALL
        decision("congestion", station=station.callsign,
                 said="nothing" if missed else "answered", why="first call")
        if missed:
            session.first_call_missed = key
        return missed

    def _say_held_reply(self, now: float) -> bool:
        """The answer held behind somebody else's exchange, once it is over.
        True when it went out."""
        held = self._held_reply
        if held is None:
            return False
        reply, station, spoken, since = held
        if self.station is None or station_key(self.station) != station_key(station):
            self._held_reply = None
            return False
        # The exchange is over when its last half has been said, not when it
        # has been taken off the queue to be said: so not while the radio is
        # still carrying it, nor in the breath the queue leaves after one.
        if (self._congested(station, ParsedIntent()) or not self.quiet_enough()
                or now < self._frequency_clear_at):
            return False
        self._held_reply = None
        session = self.session
        if reply.instruction is not None:
            # Issued now, so the readback is chased from when it was heard.
            reply.instruction.issued_at = time.monotonic()
        decision("congestion", station=station.callsign, said="answered_after",
                 waited_s=round(now - since, 1))
        self.emit("atc", reply.text, station.callsign)
        if session is not None:
            session.note("atc", as_written(reply.text))
        RADIO.write("atc", text=reply.text, language=spoken,
                    station=station.callsign, frequency=round(station.mhz, 3),
                    instruction=(reply.instruction.kind
                                 if reply.instruction is not None else ""),
                    held=True)
        self.speak(reply.text, station, language=spoken)
        if reply.ground == "pushback":
            # The approval that waited out the congestion is still the one
            # that calls the tug -- and the one the first officer reads as
            # answered, so it is never asked for again.
            if session is not None and session.phase in (
                    Phase.PREFLIGHT, Phase.CLEARED):
                session.phase = Phase.PUSHBACK
            self._start_pushback()
        self._frequency_clear_at = now + 0.6
        return True

    def _speak_from_the_queue(self, station: Station, now: float) -> None:
        """Put the next queued transmission on the air, if the air is free.

        Not while a person is working it. The invented traffic is this
        program's, and playing it over a real controller's frequency would put
        aeroplanes on their radio that they cannot see and did not clear.
        """
        if self.stand_down:
            return
        # An answer the pilot was told to stand by for goes before anything
        # new from the traffic.
        if self._say_held_reply(now):
            return
        if not self._traffic_queue:
            return

        # Anything that belongs to a frequency the pilot has since left is
        # dropped, and so is anything that has waited too long -- but by the
        # exchange rather than by the transmission. A call, its clearance and
        # its readback go stale one after another, so a busy frequency used to
        # bin the aeroplane's call and then say the controller's answer to it,
        # or say the call and bin the answer. Either way the frequency sounded
        # like it was losing transmissions, which it was.
        #
        # So: an exchange that has been started is always finished, and one
        # that has gone stale before a word of it was said is dropped whole.
        keep: list = []
        for call in self._traffic_queue:
            # A closed position has no controller for the traffic to talk to
            # either, so its frequency carries nobody's half of an exchange.
            if not self._on_this_channel(call, station) \
                    or self.brain.closed(station):
                continue
            decided = self._exchanges.get(call.exchange)
            if decided is not None and not decided[0]:
                continue                    # its opening was never heard
            if decided is not None or now - call.at < self.TRAFFIC_STALE_S:
                keep.append(call)
                continue
            # Stale, and nothing of it has been said. The whole exchange goes,
            # including the halves that have not reached this queue yet.
            self._exchange_heard(call.exchange, False, now)
        self._traffic_queue = [
            call for call in keep
            if not (call.exchange in self._exchanges
                    and not self._exchanges[call.exchange][0])
        ]
        if not self._traffic_queue:
            return

        # A go-around does not queue behind two taxi instructions. It is the
        # one thing a tower says that the rest of the frequency waits for, and
        # the aeroplane is already climbing away while it is said.
        self._traffic_queue.sort(key=lambda call: not call.urgent)

        # Never on top of the user, never on top of the controller answering
        # the user, and never on top of the cabin.
        if not self.quiet_enough() or now < self._frequency_clear_at:
            return

        call = self._traffic_queue.pop(0)
        # From here the rest of this exchange is heard however long the
        # frequency stays busy: half an exchange is the thing being fixed.
        self._exchange_heard(call.exchange, True, now)
        self._speak_traffic(call, station)
        # Long enough for the transmission to finish before the next one
        # starts, which the player being busy also enforces.
        self._frequency_clear_at = now + 0.6

    @staticmethod
    def _on_this_channel(call, station: Station) -> bool:
        """Whether a queued call is heard on the station tuned: by channel
        where the call knows its own (Ground and Tower on one frequency are
        one frequency), else by position."""
        if getattr(call, "mhz", 0.0) and station.mhz:
            return same_channel(call.mhz, station.mhz, 0)
        return call.position == station.position

    def _speak_traffic(self, call, station: Station) -> None:
        """Put one AI transmission on the air.

        A controller line is spoken by the controller the user is already
        talking to -- same facility, same position, so the same voice. A pilot
        line gets a voice of its own, keyed on the callsign, and drawn from the
        accent pool of the operator's own country: an Air France crew at
        Frankfurt still sounds French.
        """
        self.emit("traffic", call.text, call.speaker)
        RADIO.write(
            "traffic", text=call.text, language=call.language,
            speaker=call.speaker, from_pilot=call.from_pilot,
            station=station.callsign, frequency=round(station.mhz, 3),
            flight=call.flight.written if call.flight else "",
            rules=("IFR" if call.flight and call.flight.ifr else "VFR"),
            phase=call.flight.phase if call.flight else "",
        )
        if call.from_pilot:
            operator = call.flight.operator
            home = (operator.home[0] if operator and operator.home
                    else station.ident)
            args = dict(facility=call.flight.written, position="PILOT",
                        ident=home, language=call.language,
                        distance_nm=6.0,
                        altitude_ft=call.flight.altitude_ft)
        else:
            args = dict(facility=station.facility or station.ident,
                        position=station.position, ident=station.ident,
                        language=call.language,
                        distance_nm=station.distance_nm,
                        altitude_ft=self.state.altitude_ft)
        spoken = self.spoken_form(call.text)
        radio_fx = self.config.audio.radio_effects
        # Same lane, same priority as a real transmission: the frequency is
        # what these aeroplanes are heard on, and the cabin yields to it the
        # moment it is asked for rather than once it can be heard.
        _render_count(self, 1)
        self.synth.render_soon(
            lambda: self.synth.speak(spoken, radio=radio_fx, **args),
            on_done=lambda tr, err: self._traffic_spoken(tr, err, call),
            priority=0)

    def _traffic_spoken(self, transmission, error, call) -> None:
        """An AI transmission back from the lane: put it on the player."""
        _render_count(self, -1)
        if error is not None:
            log.debug("traffic synthesis failed", exc_info=error)
            return
        if transmission is None:
            return
        self.player.play(transmission.audio, transmission.sample_rate,
                         label=call.speaker)

    def _follow_region(self, state: AircraftState) -> None:
        """Keep the language set in step with where the aeroplane is."""
        if not state.has_position:
            return
        now = time.monotonic()
        if now - self._last_region_check < 20.0:
            return
        self._last_region_check = now
        airport = self.navdb.nearest_airport(
            state.latitude, state.longitude, max_nm=250.0, min_rank=1
        )
        if airport is not None:
            self.follow_languages(airport.ident)

    def _watch_surface(self, previous: AircraftState, state: AircraftState,
                       session: FlightSession) -> None:
        """Let the controller react to what the aircraft actually did."""
        station = self.station
        if station is None or self.stand_down:
            return
        spoken = self._language_for(station.ident, self.last_language)
        try:
            reply = self.watch.look(
                session, station, state, previous, language=spoken
            )
        except Exception:
            log.debug("surface watch failed", exc_info=True)
            return
        if reply is None or not reply.speak:
            return

        # A takeoff is one poll wide. Skipping the look while the speaker is
        # busy would lose the whole event rather than delay it, and the player
        # queues anyway, so this transmits into the queue like any other.
        self.emit("atc", reply.text, station.callsign)
        session.note("atc", as_written(reply.text))
        self.speak(reply.text, station, language=spoken)
        self._radio_unprompted(reply.text, station, "surface", spoken,
                               reply.instruction)

    def _watch_crossings(self, previous: AircraftState, state: AircraftState,
                         session: FlightSession) -> None:
        """The runway crossings the real taxi route holds the aeroplane
        short of: granted or refused as it reaches each one
        (:class:`~meridianatc.atc.watch.CrossingWatch`)."""
        station = self.station
        if station is None or self.stand_down:
            return
        spoken = self._language_for(station.ident, self.last_language)
        try:
            reply = self.crossing_watch.look(
                session, station, state, previous, language=spoken
            )
        except Exception:
            log.debug("the crossing watch raised", exc_info=True)
            return
        if reply is None or not reply.speak:
            return
        self.emit("atc", reply.text, station.callsign)
        session.note("atc", as_written(reply.text))
        self.speak(reply.text, station, language=spoken)
        self._radio_unprompted(reply.text, station, "crossing", spoken,
                               reply.instruction)

    def _work_the_departure(self, state: AircraftState,
                            session: FlightSession) -> None:
        """Clear a departure the tower has already put on the runway.

        Not behind the reminders setting, for the same reason the handoffs are
        not: an aeroplane told to line up and wait has been given half an
        instruction, and the other half is the tower's to give.
        """
        station = self.station
        if station is None or self.stand_down or not self.quiet_enough():
            return
        try:
            reply = self.departures.look(
                session, station, state, time.monotonic(),
                weather_for=self.weather_for,
                language=self._language_for(station.ident, self.last_language),
            )
        except Exception:
            log.debug("the departure watch raised", exc_info=True)
            return
        if reply is None or not reply.speak:
            return
        spoken = self._language_for(station.ident, self.last_language)
        self.emit("atc", reply.text, station.callsign)
        session.note("atc", as_written(reply.text))
        self.speak(reply.text, station, language=spoken)
        self._radio_unprompted(reply.text, station, "departure", spoken,
                               reply.instruction)

    def _work_the_climb(self, state: AircraftState,
                        session: FlightSession) -> None:
        """The next step of an IFR climb, a flight off its SID, and the
        centre's side of the route (atc/routing.py)."""
        now = time.monotonic()
        self._say_watched(state, session, "climb",
                          lambda station, spoken: self.climbs.look(
                              session, station, state, now, spoken))
        self._say_watched(state, session, "sid",
                          lambda station, spoken: self.sid_watch.look(
                              session, station, state, now, spoken))
        self._say_watched(state, session, "route",
                          lambda station, spoken: self.route_watch.look(
                              session, station, state, now, spoken))

    def _sim_contacts(self) -> list:
        """The simulator's aeroplanes, without the pilot's own."""
        if not self.config.traffic.from_simulator:
            return []
        own = (self.session.aircraft.callsign.written.upper()
               if self.session is not None else "")
        pilot = getattr(self.simwatch, "user_object_id", None)
        found = [one for one in self.simwatch.poll()
                 if one.object_id != pilot
                 and (one.callsign or "").strip().upper() != own]
        self._log_picture(found)
        return found

    # How often the controller's picture of the simulator's traffic is
    # written to the arrival log when nothing in it has changed.
    PICTURE_EVERY_S = 60.0

    def _log_picture(self, found: list) -> None:
        """A row of what the simulator's traffic feed gave the controller:
        how many, how many placed, and the nearest few with what they were
        read as. Written when the count changes and once a minute otherwise,
        so a flight's log shows whether FSLTL's and the simulator's
        aeroplanes ever reached the tower."""
        now = time.monotonic()
        count = len(found)
        last = getattr(self, "_picture_logged", None)
        if last is not None and last[0] == count and now - last[1] < self.PICTURE_EVERY_S:
            return
        self._picture_logged = (count, now)
        state = self.state
        nearest = []
        if state is not None and state.has_position:
            placed = [one for one in found if getattr(one, "positioned", True)]
            placed.sort(key=lambda one: state.distance_to(one.latitude, one.longitude))
            for one in placed[:5]:
                nearest.append(
                    f"{(one.callsign or one.registration or '?').strip()}"
                    f"/{one.type_code or '?'}"
                    f" {state.distance_to(one.latitude, one.longitude):.1f}nm"
                    f" {round(one.altitude_ft or 0)}ft"
                    f"{' gnd' if one.on_ground else ''}")
        within = (sum(1 for one in found if getattr(one, "positioned", True)
                      and state.distance_to(one.latitude, one.longitude) <= 10.0)
                  if state is not None and state.has_position else None)
        decision("picture", observed=count, within_10nm=within,
                 nearest=nearest or None)

    def _watch_traffic(self, state: AircraftState, session: FlightSession) -> None:
        """Traffic calls for a flight on radar advisories
        (:class:`~meridianatc.atc.trafficinfo.TrafficWatch`)."""
        station = self.station
        if station is None or self.stand_down or not self.quiet_enough():
            return
        spoken = self._language_for(station.ident, self.last_language)
        try:
            reply = self.traffic_watch.look(session, station, state,
                                            language=spoken)
        except Exception:
            log.debug("the traffic watch raised", exc_info=True)
            return
        if reply is None or not reply.speak:
            return
        self.emit("atc", reply.text, station.callsign)
        session.note("atc", as_written(reply.text))
        self.speak(reply.text, station, language=spoken)
        self._radio_unprompted(reply.text, station, "traffic", spoken,
                               reply.instruction)

    def _watch_points(self, state: AircraftState, session: FlightSession) -> None:
        """A reporting point flown past without a report
        (:class:`~meridianatc.atc.watch.ReportingPointWatch`)."""
        station = self.station
        if station is None or self.stand_down or not self.quiet_enough():
            return
        spoken = self._language_for(station.ident, self.last_language)
        try:
            reply = self.point_watch.look(session, station, state,
                                          language=spoken)
        except Exception:
            log.debug("the reporting-point watch raised", exc_info=True)
            return
        if reply is None or not reply.speak:
            return
        self.emit("atc", reply.text, station.callsign)
        session.note("atc", as_written(reply.text))
        self.speak(reply.text, station, language=spoken)
        self._radio_unprompted(reply.text, station, "reporting_point", spoken,
                               reply.instruction)

    def _watch_runway(self, state: AircraftState, session: FlightSession) -> None:
        """The runway in use changing with the wind
        (:class:`~meridianatc.atc.watch.RunwayWatch`)."""
        station = self.station
        if station is None or self.stand_down or not self.quiet_enough():
            return
        spoken = self._language_for(station.ident, self.last_language)
        try:
            reply = self.runway_watch.look(session, station, state,
                                           language=spoken)
        except Exception:
            log.debug("the runway watch raised", exc_info=True)
            return
        if reply is None or not reply.speak:
            return
        self.emit("atc", reply.text, station.callsign)
        session.note("atc", as_written(reply.text))
        self.speak(reply.text, station, language=spoken)
        self._radio_unprompted(reply.text, station, "runway_change", spoken,
                               reply.instruction)

    def _watch_airspace(self, state: AircraftState, session: FlightSession) -> None:
        """A VFR aeroplane about to enter a zone it needs a word for
        (:class:`~meridianatc.atc.watch.AirspaceWatch`)."""
        station = self.station
        if station is None or self.stand_down or not self.quiet_enough():
            return
        spoken = self._language_for(station.ident, self.last_language)
        try:
            reply = self.airspace_watch.look(session, station, state,
                                             language=spoken)
        except Exception:
            log.debug("the airspace watch raised", exc_info=True)
            return
        if reply is None or not reply.speak:
            return
        self.emit("atc", reply.text, station.callsign)
        session.note("atc", as_written(reply.text))
        self.speak(reply.text, station, language=spoken)
        self._radio_unprompted(reply.text, station, "airspace", spoken,
                               reply.instruction)

    def _watch_restriction(self, state: AircraftState,
                           session: FlightSession) -> None:
        """A VFR flight above the altitude radar holds it under
        (:class:`~meridianatc.atc.watch.RestrictionWatch`)."""
        self._say_watched(state, session, "restriction",
                          lambda station, spoken: self.restriction_watch.look(
                              session, station, state, time.monotonic(),
                              language=spoken))

    def _watch_radar_service(self, state: AircraftState,
                             session: FlightSession) -> None:
        """A VFR flight's radar service ended as it leaves
        (:class:`~meridianatc.atc.watch.RadarServiceWatch`)."""
        self._say_watched(state, session, "radar_service_end",
                          lambda station, spoken: self.radar_service_watch.look(
                              session, station, state, language=spoken))

    def _say_watched(self, state: AircraftState, session: FlightSession,
                     why: str, look) -> None:
        station = self.station
        if station is None or self.stand_down or not self.quiet_enough():
            return
        if not self.brain.staffed(station):
            return
        spoken = self._language_for(station.ident, self.last_language)
        try:
            reply = look(station, spoken)
        except Exception:
            log.debug("the %s watch raised", why, exc_info=True)
            return
        if reply is None or not reply.speak:
            return
        self.emit("atc", reply.text, station.callsign)
        session.note("atc", as_written(reply.text))
        self.speak(reply.text, station, language=spoken)
        self._radio_unprompted(reply.text, station, why, spoken,
                               reply.instruction)

    def _watch_transponder(self, state: AircraftState,
                           session: FlightSession) -> None:
        """The squawk the transponder shows against the assigned one
        (:class:`~meridianatc.atc.watch.TransponderWatch`)."""
        self._say_watched(state, session, "squawk",
                          lambda station, spoken: self.transponder_watch.look(
                              session, station, state, language=spoken))

    def _watch_taxi(self, state: AircraftState,
                    session: FlightSession) -> None:
        """The aeroplane off its cleared taxi route
        (:class:`~meridianatc.atc.watch.TaxiWatch`)."""
        self._say_watched(state, session, "taxi",
                          lambda station, spoken: self.taxi_watch.look(
                              session, station, state, language=spoken))

    def _watch_com2(self, state: AircraftState,
                    session: FlightSession) -> None:
        """The second radio's broadcast: tuned to an ATIS, with its audio
        switch on, it says the ATIS -- once when it is tuned, and again
        when the letter changes underneath it."""
        station2 = self.station2
        if station2 is None or station2.position != "ATIS" \
                or not getattr(state, "com2_receive", False):
            self._atis2_played = ""
            return
        try:
            report = self.atis_for(
                station2.ident,
                self._language_for(station2.ident, self.last_language))
        except Exception:
            log.debug("could not read the ATIS for %s", station2.ident,
                      exc_info=True)
            return
        key = f"{station2.ident}:{report.letter if report else ''}"
        if key == self._atis2_played:
            return
        if self.play_atis(station2.ident) is not None:
            self._atis2_played = key

    def _watch_deviation(self, state: AircraftState,
                         session: FlightSession) -> None:
        """One deviation call too many
        (:class:`~meridianatc.atc.watch.DeviationAssistWatch`)."""
        self._say_watched(state, session, "deviation",
                          lambda station, spoken: self.deviation_watch.look(
                              session, station, state, language=spoken))

    def _work_the_arrival(self, previous: AircraftState, state: AircraftState,
                          session: FlightSession) -> None:
        """Let the controller work the arrival instead of waiting to be asked.

        None of this is a reminder for a forgetful pilot, so none of it sits
        behind the setting that governs reminders. An aeroplane may not leave
        its cruising level or fly an approach without being told to, and a
        tower that has to be asked for a landing clearance is not a tower.
        """
        station = self.station
        if station is None or self.stand_down:
            return
        if session.destination is not None:
            self._ask_for_approaches(session.destination.ident)
        # Never over the top of something else: not the speaker, not the
        # pilot with the button down, not the controller working out what the
        # pilot just said, and not the cabin. Unlike a takeoff, none of these
        # is one poll wide -- an arrival eighteen miles out is still eighteen
        # miles out on the next look -- so each of them can wait for the
        # frequency to go quiet rather than talking across it.
        if not self.quiet_enough():
            return
        spoken = self._language_for(station.ident, self.last_language)
        why = "arrival"
        try:
            # The circuit first: a base the tower promised to call, and a
            # report it asked for and did not get. One transmission a look.
            reply = self.arrivals.circuit(session, station, state,
                                          language=spoken)
            if reply is not None and reply.speak:
                why = "circuit"
            else:
                reply = self.arrivals.look(
                    session, station, state, previous,
                    weather_for=self.weather_for, language=spoken,
                )
        except Exception:
            log.debug("the arrival watch raised", exc_info=True)
            return
        if reply is None or not reply.speak:
            return

        if why != "circuit":
            # The circuit's own calls log themselves; anything else the tower
            # says unprompted to a VFR flight gets the same row.
            self.brain.note_circuit(
                session, station, state,
                reply.instruction.kind if reply.instruction else why)
        self.emit("atc", reply.text, station.callsign)
        session.note("atc", as_written(reply.text))
        self.speak(reply.text, station, language=spoken)
        self._radio_unprompted(reply.text, station, why, spoken,
                               reply.instruction)

    #: How close to the departure runway counts as being at the holding
    #: point. A holding point sits a few hundred feet back from the threshold,
    #: which is inside a third of a mile; a taxiway running the length of the
    #: runway is never that close to the *end* it is measured from.
    HOLDING_POINT_NM = 0.35

    #: The phases an aeroplane can still be taxiing out in. Anything later is
    #: already at the runway or past it.
    _TAXIING_OUT = frozenset({Phase.PREFLIGHT, Phase.CLEARED, Phase.PUSHBACK,
                              Phase.TAXI_OUT})

    #: And the ones that mean it has not started rolling yet, so that starting
    #: to roll is a transition out of any of them rather than only out of the
    #: first.
    _NOT_MOVING_YET = frozenset({Phase.PREFLIGHT, Phase.CLEARED,
                                 Phase.PUSHBACK})

    #: How near the route's own holding point -- the first point of the
    #: line-up leg -- counts as the bars. A crew stops short of them, and
    #: the tower's handoff does not turn on the last few metres either way.
    AT_THE_BARS_NM = 0.06

    def _reached_the_holding_point(self, state: AircraftState,
                                   session: FlightSession) -> bool:
        """Whether the aeroplane has taxied up to the runway it is leaving from.

        Where the taxi was given as a route, the holding point is where the
        route says it is -- the first point of the line-up leg -- measured
        to rather than to the runway's end. A stand or an apron bend can
        sit nearer the threshold than the bars do (Vigo's south apron is
        three hundred metres from the 01 threshold, inside the radius
        below): the map test alone marked an aeroplane still on its stand
        as at the hold once its clearance named the runway, and marked a
        taxiing one while it was still on the apron. Without a route, the
        end it is departing from is the only marker -- the taxiway
        running the length of a runway is a few hundred feet from it for
        its whole length, and an aeroplane halfway down that taxiway has
        not reached anything.
        """
        if not (state.on_ground and state.has_position):
            return False
        lineup = getattr(session, "lineup_path", None) or ()
        if lineup:
            lat, lon = lineup[0]
            return haversine_nm(state.latitude, state.longitude,
                                lat, lon) <= self.AT_THE_BARS_NM
        runway = normalize_runway(session.departure_runway or "")
        airport = session.departure
        if not runway or airport is None:
            return False
        for name, _heading, published in self.navdb.runway_ends(airport.ident):
            if name != runway:
                continue
            if name == published.le_ident:
                lat, lon = published.le_lat, published.le_lon
            else:
                lat, lon = published.he_lat, published.he_lon
            if not (lat or lon):
                return False
            return haversine_nm(state.latitude, state.longitude,
                                lat, lon) <= self.HOLDING_POINT_NM
        return False

    # The on-ground flag has to say the same thing this many looks running
    # before it is believed. Two looks is half a second at the usual poll.
    GROUND_SAMPLES = 2

    def _debounced_ground(self, previous: AircraftState,
                          state: AircraftState) -> tuple[AircraftState, AircraftState]:
        """The two readings with the on-ground flag debounced.

        A change is believed once the raw flag has said it GROUND_SAMPLES
        looks running; until then the watchers see the old value.
        """
        if self._ground_seen is None:
            self._ground_seen = bool(previous.on_ground)
        before = self._ground_seen
        if bool(state.on_ground) != self._ground_seen:
            self._ground_disagrees += 1
            if self._ground_disagrees >= self.GROUND_SAMPLES:
                self._ground_seen = bool(state.on_ground)
                self._ground_disagrees = 0
        else:
            self._ground_disagrees = 0
        return (replace(previous, on_ground=before),
                replace(state, on_ground=self._ground_seen))

    # A liftoff this soon after a touchdown, still fast and on the same
    # runway, is off the rollout and not a takeoff. Back on the ground within
    # BOUNCE_S it was a bounce; past it, or past BOUNCE_AGL_FT, it is a
    # touch-and-go or a go-around.
    ROLLOUT_WINDOW_S = 60.0
    ROLLOUT_MIN_KT = 40.0
    BOUNCE_S = 5.0
    BOUNCE_AGL_FT = 200.0

    def _note_rollout_liftoff(self, previous: AircraftState, state: AircraftState,
                              session: FlightSession) -> None:
        """Whether this liftoff is off a landing rollout, decided before the
        surface watch reads it as a takeoff."""
        if not (previous.on_ground and not state.on_ground):
            return
        # Outbound has no rollout to lift off: while the session is the
        # takeoff -- the roll, the climb away, a settling touch between
        # them -- the airborne read is the departure's own, never the
        # end of a landing. A stale touchdown from the last circuit
        # must not say otherwise either.
        if session.phase.outbound:
            return
        if not session.touchdown_at:
            return
        elapsed = time.monotonic() - session.touchdown_at
        runway = ""
        if self.station is not None:
            runway = self.navdb.runway_at(self.station.ident, previous.latitude,
                                          previous.longitude, previous.heading_true)
        same_runway = (not runway or not session.touchdown_runway
                       or normalize_runway(runway) == normalize_runway(
                           session.touchdown_runway))
        if (elapsed <= self.ROLLOUT_WINDOW_S and same_runway
                and session.rollout_min_kt >= self.ROLLOUT_MIN_KT):
            session.rollout_liftoff_at = time.monotonic()
            # The landing clearance covered this landing and is wanted again
            # if it was only a bounce.
            session.cleared_landing = session.touchdown_cleared
            decision("liftoff_after_touchdown", verdict="pending",
                     runway=session.touchdown_runway,
                     since_touchdown_s=round(elapsed, 1),
                     slowest_kt=round(session.rollout_min_kt),
                     speed_kt=round(float(previous.ground_speed_kt or 0.0)))

    def _after_rollout_liftoff(self, previous: AircraftState, state: AircraftState,
                               session: FlightSession) -> None:
        """A bounce, a touch-and-go, or a go-around: what a liftoff off the
        rollout turned out to be."""
        if not session.rollout_liftoff_at:
            return
        airborne_s = time.monotonic() - session.rollout_liftoff_at
        if state.on_ground:
            # Down again: it was a bounce. The touchdown has just been
            # recorded again by _update_phase, with the landing clearance it
            # was restored for, so nothing is said.
            session.rollout_liftoff_at = 0.0
            decision("bounce", runway=session.touchdown_runway,
                     airborne_s=round(airborne_s, 1),
                     speed_kt=round(float(state.ground_speed_kt or 0.0)))
            return
        field = self.navdb.airport(self.station.ident) if self.station else None
        elevation = float(getattr(field, "elev_ft", 0.0) or 0.0)
        above = state.altitude_agl_ft or max(0.0, state.altitude_ft - elevation)
        if airborne_s < self.BOUNCE_S and above < self.BOUNCE_AGL_FT:
            return
        session.rollout_liftoff_at = 0.0
        # Where nobody clears anything -- an AFIS, an unattended field -- a
        # VFR aeroplane off the rollout is flying the circuit it was flying:
        # there was no touch-and-go to clear, so none was recorded, and every
        # one at Kortrijk was taken for a go-around and told to join.
        nobody_clears = not session.ifr and not self.brain.issues_clearances(
            self.station)
        if session.cleared_option or session.pattern_work or nobody_clears:
            # Round the circuit: the touch-and-go it was cleared for.
            session.cleared_option = False
            session.cleared_landing = False
            session.phase = Phase.DEPARTURE
            decision("liftoff_after_touchdown", verdict="touch-and-go",
                     airborne_s=round(airborne_s, 1), agl_ft=round(above))
            return
        # Round again: a go-around after touchdown, worked like any other.
        decision("liftoff_after_touchdown", verdict="go-around",
                 airborne_s=round(airborne_s, 1), agl_ft=round(above))
        station = self.station
        if station is None or field is None or not self.brain.staffed(station):
            session.cleared_landing = False
            session.going_around = True
            session.phase = Phase.APPROACH
            return
        spoken = self._language_for(station.ident, self.last_language)
        p = self.brain.for_station(station, spoken)
        session.aircraft.language = getattr(p, "language", "en")
        reply = self.brain.go_around(session, station,
                                     self.weather_for(field.ident), p, field,
                                     told=True)
        self.brain.note_circuit(session, station, state, "go_around")
        self.emit("atc", reply.text, station.callsign)
        session.note("atc", as_written(reply.text))
        self.speak(reply.text, station, language=spoken)
        self._radio_unprompted(reply.text, station, "go_around_after_touchdown",
                               spoken, reply.instruction)

    # Still on the ground this long after a touchdown, the circuits are over.
    # A touch-and-go is airborne again in well under that; a stop-and-go
    # (which needs a clearance of its own) does not slow below the rollout
    # speed the bounce detection above already uses.
    FULL_STOP_AFTER_S = 20.0

    def _full_stop_after_touchdown(self, state: AircraftState,
                                   session: FlightSession) -> None:
        """End the circuits of an aeroplane that landed and stayed down.

        Asking for circuits set a flag that only a pilot's words could clear,
        and a pilot who simply landed and taxied in kept it -- so the next
        arrival of the session, hours later, was cleared for the option.
        """
        if not session.pattern_work or not session.touchdown_at:
            return
        if session.full_stop_judged_at == session.touchdown_at:
            return
        if not state.on_ground or session.rollout_liftoff_at:
            return
        if time.monotonic() - session.touchdown_at < self.FULL_STOP_AFTER_S:
            return
        session.full_stop_judged_at = session.touchdown_at
        session.pattern_work = False
        decision("circuits_ended", why="landed and stayed down",
                 runway=session.touchdown_runway)

    def _update_phase(self, previous: AircraftState, state: AircraftState,
                      session: FlightSession) -> None:
        # Rolled on the way out: the only witness that separates a holding
        # point reached by taxiing from a stand that happens to sit near
        # the runway end -- the distance to the threshold cannot tell the
        # two apart, so the mark below is made only on its strength.
        # Leaving the taxiing phases ends the journey it counted for.
        if session.phase in self._TAXIING_OUT:
            if state.on_ground and state.ground_speed_kt > 5:
                session.left_the_stand = True
        else:
            session.left_the_stand = False
        # The gear's word on a roll is not a landing's: at takeoff speed the
        # struts unload and touch again, a skipped sim frame reads the same,
        # and taken at face value the pair spends the takeoff clearance on a
        # bounce and writes a touchdown on a roll -- which is how a departure
        # that never left its strip got sent around. Outbound, flight is only
        # the airborne read that is still airborne on the next poll; a single
        # read is the gear answering, and a re-touch inside the roll is the
        # other half of the same bounce, not a landing nobody flew.
        rolling_out = session.phase in (Phase.LINED_UP, Phase.TAKEOFF)
        session.roll_air_ticks = 0 if (
            state.on_ground or not rolling_out
        ) else session.roll_air_ticks + 1
        if previous.on_ground and not state.on_ground:
            # Every liftoff edge, whichever phase read it: the re-touch
            # inside the settling window below is measured from here.
            session.departed_at = time.monotonic()
        if previous.on_ground and not state.on_ground and session.rollout_liftoff_at:
            # Off the rollout, not a takeoff: back to the approach, and the
            # "landed without a clearance" advisory stays spent so a bounce
            # is not told off twice.
            session.phase = Phase.APPROACH
            session.occupying_runway = ""
            return
        if (previous.on_ground and not state.on_ground and not rolling_out) \
                or (rolling_out and session.roll_air_ticks >= 2):
            if session.phase.outbound:
                session.phase = Phase.DEPARTURE
            # The clearance was for this departure and is now spent. Leaving
            # it set meant a circuit could be flown all day on one clearance.
            session.cleared_takeoff = False
            # A new flight from the ground, so the last circuit's legs are
            # history: the tower waits for this one to report its own.
            session.pattern_leg = ""
            session.vrp_reported = ""
            session.expected_vrp = ""
            session.cleared_runway = ""
            session.occupying_runway = ""
            # A new landing is a new event, so its advisory is armed again,
            # and the last one is no longer the one being taxied in from.
            session.advisories.discard("landed_without_clearance")
            session.landed_cleared = False
        elif not previous.on_ground and state.on_ground \
                and not session.re_touch_is_the_takeoffs():
            session.phase = Phase.LANDED
            session.going_around = False
            # Remembered, so a liftoff straight off this rollout can be told
            # from a takeoff.
            session.touchdown_at = time.monotonic()
            session.touchdown_cleared = bool(session.cleared_landing
                                             or session.cleared_option)
            session.rollout_min_kt = float(state.ground_speed_kt or 0.0)
            session.touchdown_runway = (session.arrival_runway
                                        or session.occupying_runway)
            # Kept before it is spent: the welcome on the way to the gate is
            # for an aeroplane that was cleared to be here.
            session.landed_cleared = session.cleared_landing
            session.cleared_landing = False
            session.cleared_takeoff = False
            session.landing_withheld = False
            session.landing_anticipated = ""
            if session.flight_plan_open:
                # Nobody closes it on landing: that is the pilot's call to
                # flight service, or a controller asked to pass it on (FAA
                # AIM 5-1-14). Logged, so a flight plan left open is seen.
                decision("flight_plan", said="still_open_after_landing",
                         airport=getattr(session.destination, "ident", "") or None)
            # The landing clearance covers the rollout, so the runway stays
            # allowed until the aircraft is off it. That holds even for a
            # landing nobody cleared: it has already been called, and calling
            # the rollout again as an incursion would be telling the pilot off
            # twice for one thing.
            session.cleared_runway = (
                session.arrival_runway or session.occupying_runway
                or session.cleared_runway
            )
            session.advisories.discard("departed_without_clearance")
        elif state.on_ground and session.touchdown_at and \
                session.phase is Phase.LANDED and \
                float(state.ground_speed_kt or 0.0) < session.rollout_min_kt:
            # The slowest the rollout got: below forty knots it is taxiing,
            # and a liftoff after that is a takeoff.
            session.rollout_min_kt = float(state.ground_speed_kt or 0.0)
        elif (state.on_ground and state.ground_speed_kt > 5
                and session.phase in self._NOT_MOVING_YET):
            # From any of the phases that mean "at the stand", not only from
            # preflight. An aeroplane that has read back its IFR clearance is
            # in CLEARED, and CLEARED had no way out of it except a taxi
            # clearance -- so a pilot who took their clearance and then simply
            # taxied stayed "at the stand" at twenty knots all the way to the
            # runway. Everything hung on the phase went with it: the holding
            # point was never reached, the tower was never called, and the
            # cabin never did the taxi checklist or the safety demonstration,
            # because those ask whether the aeroplane is taxiing and it said
            # no.
            session.phase = Phase.TAXI_OUT
        elif (state.on_ground and session.phase in self._TAXIING_OUT
                and session.left_the_stand
                and self._reached_the_holding_point(state, session)):
            # Nothing ever set this phase, which is why nobody was sent to the
            # tower and why the cabin was never told to sit down: a flight went
            # from taxiing to airborne with the holding point in between and no
            # name for the part in the middle. The hold still has to be
            # reached by taxiing, though -- a stand near the departure end is
            # inside the same radius, and an aeroplane that has not rolled yet
            # is parked, whatever the map says.
            session.phase = Phase.HOLDING_SHORT
        elif not state.on_ground:
            if session.phase is Phase.DEPARTURE and self._enroute_now(state,
                                                                     session):
                session.phase = Phase.ENROUTE
            elif session.phase is Phase.ENROUTE and state.altitude_ft < 25000:
                if getattr(state, "paused", False):
                    # A paused clock is not descent time: without the reset,
                    # the pause itself counts toward the persistence window.
                    session.descending_since = 0.0
                elif state.descending:
                    now = time.monotonic()
                    session.descending_since = (
                        session.descending_since or now)
                    if now - session.descending_since >= 20.0:
                        session.phase = Phase.DESCENT
                else:
                    session.descending_since = 0.0

        if state.on_ground and session.phase in (Phase.LANDED,
                                                 Phase.TAXI_IN) \
                and state.parking_brake \
                and float(state.ground_speed_kt or 0.0) < 1.0 \
                and not any(bool(f) for f in combustion(state)):
            # Parked is a state, not an instruction: wheels stopped, brake
            # set, engines out -- reached whether or not a taxi clearance
            # ever named the stand. The shutdown checklist keys off it.
            session.phase = Phase.PARKED

    #: Above this, a departure is en route whatever else it is doing. Below
    #: it, the aeroplane has to have stopped climbing and left the circuit.
    ENROUTE_FT = 10000.0

    #: How far from the departure field a levelled-off aeroplane has to be
    #: before it counts as having gone somewhere. A circuit is flown inside
    #: three miles of the runway; this is well outside any of them.
    LEFT_THE_CIRCUIT_NM = 12.0

    def _enroute_now(self, state: AircraftState,
                     session: FlightSession) -> bool:
        """Whether a departure has become a flight going somewhere.

        Ten thousand feet used to be the whole of the test, and it silently
        ended the flight for anybody who never went that high. The phase stays
        DEPARTURE, DEPARTURE is an outbound phase, and every outbound phase is
        excluded from the arrival watch -- so a short hop at eight thousand
        feet flew its whole approach, lined up on final, and was never cleared
        to land, because as far as the controller was concerned it was still
        departing. Nothing in the log said so: the tower simply never spoke.

        So height is one way of qualifying and no longer the only one. An
        aeroplane that has stopped climbing and is a dozen miles from where it
        took off has gone somewhere, whatever its altitude. Circuits are
        excluded by both halves of that -- a circuit is flown within three
        miles of the runway, and by ``pattern_work`` where the pilot has said
        outright that is what they are doing.
        """
        if state.altitude_ft > self.ENROUTE_FT:
            return True
        if session.pattern_work or state.climbing:
            return False
        field = session.departure
        if field is None or not state.has_position:
            return False
        return state.distance_to(field.lat, field.lon) > self.LEFT_THE_CIRCUIT_NM

    def _watch_for_humans(self, state: AircraftState) -> None:
        """Notice a controller logging on or off the frequency in COM1."""
        if not state.com1_active:
            return
        lat, lon = ((state.latitude, state.longitude)
                    if state.has_position else (0.0, 0.0))
        found = self.online.on_frequency(state.com1_active, lat, lon)
        before = self.human
        self.human = found
        if found is not None and (before is None
                                  or before.callsign != found.callsign):
            self.emit(
                "info",
                f"{state.com1_active:.3f} - {found.describe()} is working "
                "this frequency. MeridianATC will stay quiet on it.",
                key="log.human_online", mhz=f"{state.com1_active:.3f}",
                station=found.callsign, network=found.network.upper(),
            )
        elif found is None and before is not None and self.station is not None:
            # They have logged off, so this program has the frequency again.
            self.emit(
                "info",
                f"{state.com1_active:.3f} - {self.station.callsign} "
                f"({self.station.ident}, {self.station.distance_nm:.0f} NM).",
                key="log.tuned", mhz=f"{state.com1_active:.3f}",
                station=self.station.callsign, airport=self.station.ident,
                distance=f"{self.station.distance_nm:.0f}",
            )

    def _check_announcements(self) -> None:
        """Say so if the recorded announcements were asked for and are absent.

        Two ways that happens and both used to be silent. The folder the pilot
        typed is not there under any reading of the name, or it is there and
        holds nothing this program recognises. Either way the switch is on,
        the cabin falls back to the synthesised crew, and a pilot who has just
        installed a sound pack is left listening for it.

        Once per flight, and only when the recordings were actually asked
        for: somebody who has never turned them on does not need a row about
        a folder they have never heard of.
        """
        if not (self.config.immersion.enabled
                and self.config.immersion.recordings):
            return
        missing = announcements_missing(self.config)
        if missing:
            self.emit("info",
                      f"No announcements folder called \"{missing}\". The "
                      f"cabin is using its own voices.",
                      key="log.announcements_missing", folder=missing)
            return
        try:
            found = self.announcements.count
        except Exception:
            log.debug("could not count the announcements", exc_info=True)
            return
        if not found:
            self.emit("info",
                      "Recorded announcements are switched on, but no audio "
                      "files were found. The cabin is using its own voices.",
                      key="log.announcements_empty")

    def _radio_trouble(self, detail: str) -> None:
        """The radio cannot reach the speaker. Say so rather than go quiet.

        A transmission that never finishes playing used to leave the
        frequency busy for the rest of the flight -- controllers, ATIS and
        crew all waiting their turn on a lane that never cleared.
        ``RadioPlayer.busy`` now writes off a wedged item; this is where the
        pilot hears about the device that lost it.
        """
        self.emit("error",
                  "The radio cannot reach the speaker; transmissions are "
                  f"not being heard. ({detail})",
                  key="log.radio_no_audio", detail=detail)

    def _cabin_trouble(self, detail: str) -> None:
        """The cabin cannot reach the speaker. Say so rather than go quiet.

        This is the difference between a bug report that says "the crew
        stopped working" and one that says which device refused. The cabin
        opens a second output stream alongside the radio's, and an audio stack
        can refuse that for reasons this program has no say in -- a device
        held exclusively by the simulator, a sample rate the driver will not
        take. Every one of those used to arrive as a first officer who had
        simply stopped talking, because the failure happened on the player's
        own thread and nothing was reading it.

        The radio is untouched by it, which is why this is a notice rather
        than an error: the controller still answers, and what has been lost is
        the half of the aeroplane that was optional to begin with.
        """
        self.emit("error",
                  "The cabin and the first officer cannot reach the speaker. "
                  f"The radio is unaffected. ({detail})",
                  key="log.cabin_no_audio", detail=detail)

    def radio_busy(self) -> bool:
        """Whether the frequency is in use right now.

        The speaker, the transmit button, and the gap in between while what
        the pilot said is being recognised and answered. A method rather than
        a property because the cabin is handed it as a callable: the crew are
        given a way to ask, not a reference to the engine.
        """
        # A render submitted but not yet on the player counts too: the gap
        # between asking for a transmission and hearing it is still one the
        # cabin must not talk over.
        # ``_text_lock`` held is a typed call or a crew transmission
        # mid-answer -- the same busy the spoken pipeline's ``_busy`` is.
        # ``locked`` is 3.14+; ``_is_owned`` is what it had before.
        held = getattr(self._text_lock, "locked", self._text_lock._is_owned)
        return bool(self.player.busy or self.microphone.recording
                    or self._busy.locked() or held() or self._radio_renders)

    def quiet_enough(self) -> bool:
        """Whether an unprompted transmission can go out now.

        The radio, plus the cabin. An answer to something the pilot just said
        goes out regardless -- they asked, and they are waiting -- but nothing
        the controller volunteers is worth talking over the purser for. The
        two audio paths are separate so that neither has to queue behind the
        other, and this is what keeps that from becoming both at once.
        """
        return not (self.radio_busy() or self.cabin.busy)

    @property
    def stand_down(self) -> bool:
        """Whether a person is working this frequency, so nothing is said.

        Read before every transmission this program would otherwise make --
        an answer, an advisory, a handoff, a readback chased, the other
        aeroplanes. Talking over a real controller is the one failure that
        matters more than any of them.
        """
        return self.human is not None or not self.radio_powered

    @property
    def radio_powered(self) -> bool:
        """Whether the aeroplane's radio has power (atc/radio_power.py).

        Cold and dark, the radio is dead both ways: nothing the pilot says
        goes out and nothing on the frequency comes in -- which is why
        :attr:`stand_down`, read before every transmission, reads this too.
        Each change is a ``radio_power`` row in ``logs radio`` with what it
        was decided from, because "the controller will not answer" is a
        report that has to be answerable.
        """
        if not self.config.atc.radio_needs_power:
            return True
        from ..atc.radio_power import decide

        on, why = decide(self.state)
        if on != getattr(self, "_radio_on", None):
            first = getattr(self, "_radio_on", None) is None
            self._radio_on = on
            RADIO.write("radio_power", powered=on, why=why,
                        com1_status=getattr(self.state, "com1_status", -1),
                        bus_volts=getattr(self.state, "main_bus_volts", -1.0),
                        title=getattr(self.state, "title", ""))
            if not on:
                self.emit("info", "The radio has no power: nothing is heard "
                          "and nothing goes out until the battery and the "
                          "avionics are on.", key="log.radio_off")
            elif not first:
                self.emit("info", "The radio has power.", key="log.radio_on")
        return on

    def _copilot_reads_back(self, session: FlightSession) -> bool:
        """The first officer works the radio, if that is what you asked for.

        Off by default and deliberately slow when it is on. A pilot who is
        practising radio work must never have the transmission taken out of
        their mouth, so the aeroplane waits long enough to be sure that
        somebody who was going to answer has answered.

        What it sends goes through the ordinary text path, so the controller
        checks it exactly as it checks yours -- including finding it wrong. A
        readback that skipped the parser would be a readback nothing could
        disagree with, which is not a readback.
        """
        settings = self.config.immersion
        if not (settings.enabled and settings.copilot
                and settings.radio_readbacks):
            return False
        pending = session.pending
        station = self.station
        if pending is None or station is None or pending.read_back:
            return False
        if pending.since_prompt_s < self.COPILOT_READBACK_WAIT_S:
            return False
        if not self.quiet_enough():
            pending.hold_off()
            return False

        from ..atc.copilot import readback as build_readback

        spoken = self._language_for(station.ident, self.last_language)
        p = self.brain.for_station(station, spoken)
        session.aircraft.language = getattr(p, "language", "en")
        handoff = session.handoff_pending
        text = build_readback(
            pending, session.aircraft, p, spoken,
            station_name=(handoff.callsign_short if handoff is not None else ""),
        )
        if not text:
            return False
        self.emit("pilot", text, "First officer")
        try:
            self.handle_text(text, spoken)
        except Exception:
            log.debug("the first officer's readback failed", exc_info=True)
            return False
        return True

    def _chase_readback(self, session: FlightSession) -> None:
        """Ask again for a readback the pilot has gone quiet on."""
        station = self.station
        if not self.brain.staffed(station) or self.stand_down:
            return
        pending = session.pending
        if pending is None:
            return

        # If the other seat is working the radio, it answers before the
        # controller has to ask.
        if self._copilot_reads_back(session):
            return

        # The pilot cannot answer while the controller is still talking, while
        # they have the transmit button down, or while what they just said is
        # being recognised. The clock is restarted rather than merely paused,
        # so the wait is time the pilot actually had to reply in.
        if (self.player.busy or self.microphone.recording
                or self._busy.locked()):
            pending.hold_off()
            return
        spoken = self._language_for(station.ident, self.last_language)
        p = self.brain.for_station(station, spoken)
        session.aircraft.language = getattr(p, "language", "en")
        try:
            reply = self.brain.chase_readback(session, station, p)
        except Exception:
            log.debug("readback chase failed", exc_info=True)
            return
        if reply is None or not reply.speak:
            return
        self.emit("atc", reply.text, station.callsign)
        session.note("atc", as_written(reply.text))
        self.speak(reply.text, station, language=spoken)
        self._radio_unprompted(reply.text, station, "readback_chase", spoken,
                               reply.instruction)

    # How long the first officer waits before reading a clearance back for
    # you. Shorter than the controller's own patience, so the readback lands
    # before the chase rather than after it -- and long enough that a pilot
    # who was merely thinking still gets there first.
    COPILOT_READBACK_WAIT_S = 12.0

    # How high a departure has to be before the tower is finished with it, and
    # how slow an arrival has to be before it is off the runway and the tower
    # is finished with that. Both are the point at which a real controller
    # stops watching and passes you on.
    HANDOFF_ABOVE_AGL_FT = 700.0
    HANDOFF_BELOW_KT = 35.0
    # The rollout's "when able, contact ground": slow enough that the
    # rollout is made, still on the strip. A C172 lands at sixty, and
    # the call going out the moment the wheels were on was the tower
    # talking over the flare; thirty is the brisk taxi the figure
    # intends -- fast enough the exit is near, slow enough the hands
    # are free.
    ROLLOUT_HANDOFF_BELOW_KT = 30.0

    def _hand_off_on_phase(self, state: AircraftState,
                           session: FlightSession) -> None:
        """Pass the aircraft on when the flight, not the map, says to.

        A tower hands a departure to radar as soon as it is safely climbing
        away and an arrival to ground as soon as it is clear of the runway; a
        departure sector passes the flight to the centre as it leaves the
        terminal area, and the centre passes it on again as it arrives in the
        next one. None of that is a reminder for a forgetful pilot, which is
        why none of it is behind the setting that governs reminders.

        Waiting instead until the aeroplane had flown past ninety per cent of
        the tuned station's range meant the tower never came back after
        departure, and for a centre -- two hundred and seventy miles -- meant
        the flight sat on one frequency from the climb to the flare.
        """
        station = self.station
        if not self.brain.staffed(station) \
                or self.stand_down or not self.quiet_enough():
            return
        # A station that has not been called has nobody to hand on: telling
        # an aeroplane that only tuned approach to contact the centre is a
        # controller talking to a frequency.
        if not session.established_with(station):
            return
        grace = self.config.atc.handoff_grace_s
        if time.monotonic() - session.handoff_offered_at < grace:
            return

        if station.position == "TWR":
            self._watch_the_wind(session, station)
            self._tower_hands_on(state, session, station)
        elif station.position in ("GND", "DEL"):
            self._ground_hands_on(state, session, station)
        elif station.position in ("DEP", "APP", "CTR"):
            self._radar_hands_on(state, session, station)

    def _ground_hands_on(self, state: AircraftState, session: FlightSession,
                         station: Station) -> None:
        """Send a departure to the tower as it reaches the holding point.

        Ground taxis you to the runway and then has nothing left to give you:
        the runway is the tower's. A pilot who taxied up to the hold and was
        left sitting on the ground frequency had to work out for themselves
        that the takeoff clearance was somebody else's to give, and this is
        the one handoff a real ground controller never forgets.

        Not a reminder for a forgetful pilot, so not behind the setting that
        governs reminders -- the same reasoning as the tower passing a
        departure to radar.
        """
        if session.phase is not Phase.HOLDING_SHORT:
            return
        # Its own key: "handoff:tower" is the arrival's, and spending it at
        # the holding point left every arrival on approach's frequency until
        # the pilot called the tower at six miles.
        if session.advise_once("handoff:ground_to_tower"):
            self._pass_to(session, station, "TWR",
                          runway=session.departure_runway)

    def _watch_the_wind(self, session: FlightSession,
                        station: Station) -> None:
        """"The wind is now ..." when it has moved since the clearance.

        A takeoff or landing clearance carries the wind it was issued
        with; when the field's wind moves enough that the ATIS would
        have re-issued, the tower says so -- the same materiality rule
        the ATIS itself uses, and the reason a waiting departure or an
        arrival on final hears "wind is now two five zero at one
        eight". Each issue resets the baseline, so a wind that keeps
        veering is reported again, but only by material steps.
        """
        was = session.wind_at_clearance
        if not was:
            return
        if not (session.cleared_takeoff or session.cleared_landing
                or session.cleared_option):
            session.wind_at_clearance = ()
            return
        weather = self.weather_for(station.ident)
        if weather is None:
            return
        was_dir, was_kt, was_gust = was
        moved = (
            abs(was_kt - (weather.wind_kt or 0.0)) >= 5.0
            or (abs((was_dir - (weather.wind_dir or 0.0) + 540.0)
                    % 360.0 - 180.0) >= 30.0
                and (weather.wind_kt or 0.0) >= 5.0)
            or ((weather.gust_kt or 0.0) - (weather.wind_kt or 0.0) >= 10.0
                and was_gust - was_kt < 10.0)
            # A gust that has died is a wind change too: the tower calls
            # it the same way it called the gust arriving.
            or (was_gust - was_kt >= 10.0
                and (weather.gust_kt or 0.0) - (weather.wind_kt or 0.0) < 10.0))
        if not moved:
            return
        session.wind_at_clearance = (
            float(weather.wind_dir or 0.0), float(weather.wind_kt or 0.0),
            float(weather.gust_kt or 0.0))
        decision("wind_check", airport=station.ident,
                 was=f"{int(was_dir)}/{int(was_kt)}g{int(was_gust)}",
                 now=f"{int(weather.wind_dir or 0)}/"
                     f"{int(weather.wind_kt or 0)}"
                     f"g{int(weather.gust_kt or 0)}")
        spoken = self._language_for(station.ident, self.last_language)
        p = self.brain.for_station(station, spoken)
        text = p.wind_check(weather, session.aircraft)
        self.emit("atc", text, station.callsign)
        self.speak(text, station, language=spoken)
        self._radio_unprompted(text, station, "wind_check", spoken)

    def _tower_hands_on(self, state: AircraftState, session: FlightSession,
                        station: Station) -> None:
        """Off the tower once climbing away, back to ground once clear of it."""
        airport = self.navdb.airport(station.ident)
        elevation = airport.elev_ft if airport else 0.0
        above = state.altitude_agl_ft or max(0.0, state.altitude_ft - elevation)

        # Sent round: back to whoever sequences arrivals, once climbing away.
        # Re-armed by each go-around, so a second one is handed on as well.
        if (session.going_around and not state.on_ground
                and above >= self.HANDOFF_ABOVE_AGL_FT):
            if session.advise_once("handoff:missed"):
                self._pass_to(session, station, "APP", "DEP")
            return

        climbing_out = (not state.on_ground and session.phase.outbound
                        and above >= self.HANDOFF_ABOVE_AGL_FT)
        if climbing_out:
            if self._kept_by_the_tower(session, station):
                # A VFR departure with no code and no advisories is the
                # tower's until it leaves the zone, and then it is let go:
                # handed to Departure, it was a radar service nobody had
                # asked for (and nobody ended).
                if self._in_the_zone(station, state):
                    return
                if session.advise_once("handoff:departure"):
                    decision("vfr_departure", airport=station.ident,
                             said="frequency_change", why="left the zone")
                    self._pass_to_station(session, station, None)
                return
            if session.advise_once("handoff:departure"):
                self._pass_to(session, station, "DEP", "APP")
            return

        # Slow, and off the runway: below 35 knots on the rollout is still on
        # the runway, with the next arrival a mile out, and the tower keeps
        # it until it has turned off.
        clear_of_the_runway = (
            state.on_ground and state.ground_speed_kt < self.HANDOFF_BELOW_KT
            and session.phase in (Phase.LANDED, Phase.TAXI_IN)
            and not self._on_a_runway(station.ident, state))
        # Still rolling but slow enough to be sent on: a tower that has
        # no more for the aeroplane says so during the rollout -- "when
        # able, contact ground" -- rather than making it wait at the
        # exit for a call that was always coming. Whichever of the two
        # gates fires first wins the one handoff.
        rolling_out = (
            state.on_ground
            and session.phase in (Phase.LANDED, Phase.TAXI_IN)
            and state.ground_speed_kt < self.ROLLOUT_HANDOFF_BELOW_KT
            and self._on_a_runway(station.ident, state))
        if rolling_out and session.advise_once("handoff:arrival"):
            self._pass_to(session, station, "GND", when_able=True,
                          welcome=self._welcome_to(station)
                          if session.landed_cleared else "")
        elif clear_of_the_runway and session.advise_once("handoff:arrival"):
            # And a word about where you have arrived, for an aeroplane that
            # was cleared to land here. A tower that has just had to tell
            # somebody they landed without a clearance does not then welcome
            # them to the city, so the one that was not cleared gets the
            # ordinary handoff.
            self._pass_to(session, station, "GND",
                          welcome=self._welcome_to(station)
                          if session.landed_cleared else "")

    # A field's zone where the airspace files have none for it: the radius of
    # an FAA Class D surface area, give or take.
    ZONE_NM = 4.5

    def _kept_by_the_tower(self, session: FlightSession,
                           station: Station) -> bool:
        """A VFR departure the tower does not hand to radar: no advisories
        asked for, no code given."""
        if session.ifr or session.flight_following or session.emergency:
            return False
        p = self.brain.for_station(station, self.last_language)
        return str(session.squawk) == str(p.vfr_squawk)

    def _in_the_zone(self, station: Station, state: AircraftState) -> bool:
        """Whether the aeroplane is still inside the tower's own airspace."""
        def own(volume) -> bool:
            return volume.airport == station.ident and volume.cls in ("B", "C", "D")

        try:
            if any(own(v) for v in self.navdb.airspace_volumes()):
                return any(own(v) for v in self.navdb.airspace_at(
                    state.latitude, state.longitude, state.altitude_ft))
        except Exception:
            log.debug("zone lookup failed at %s", station.ident, exc_info=True)
        where = self._station_position(station)
        return where is not None and state.distance_to(*where) < self.ZONE_NM

    def _on_a_runway(self, ident: str, state: AircraftState) -> bool:
        try:
            return bool(self.navdb.runway_at(ident, state.latitude,
                                             state.longitude))
        except Exception:
            log.debug("runway lookup failed at %s", ident, exc_info=True)
            return False

    # Where each radar position stops being the one working the flight. These
    # are the ordinary shape of a sector rather than anything published: a
    # departure is passed to the centre as it leaves the terminal area, and
    # the centre passes it to approach as it comes into the next one.
    #
    # A departure leaves that area by climbing out of it or by flying out of
    # it, and either will do. Requiring the climb stranded everything that
    # levels off low -- which is most of what flies VFR.
    # How long to leave it before looking again for an enroute frequency
    # there turned out not to be one of. The boundary lookup is a polygon
    # scan and a query, the situation loop runs four times a second, and the
    # ground underneath does not change that fast.
    CENTRE_RETRY_S = 60.0

    HANDOFF_TO_CENTRE_FT = 10000.0
    HANDOFF_TO_CENTRE_NM = 30.0
    HANDOFF_TO_APPROACH_NM = 60.0
    # Below an approach area's ceiling, an arrival this close is handed to
    # approach even outside a small area (after LocalTC: three miles a
    # thousand feet above the field, plus five).
    SMALL_AREA_NM_PER_1000FT = 3.0
    SMALL_AREA_SPARE_NM = 5.0
    HANDOFF_TO_TOWER_NM = 15.0
    TOWER_AT_NM = 6.0
    TOWER_MARGIN_NM = 4.0

    # The phases in which a flight still belongs to the field it left. Enroute
    # is one of them: the phase tracker sets it at ten thousand feet, before
    # anybody has said who to call next.
    _CLIMBING_OUT = (Phase.TAKEOFF, Phase.DEPARTURE, Phase.ENROUTE)

    def _climbing_out_with(self, session: FlightSession,
                           station: Station) -> bool:
        """Whether a radar position is working this flight as a departure.

        ENROUTE is one of the climbing-out phases because the tracker sets
        it at ten thousand feet, before anybody has passed the flight on --
        but it is also what an arrival is in when it checks in with its
        destination's approach a hundred miles out, and counting that as a
        climb-out handed it straight back to the centre it had just left.
        So ENROUTE is a climb-out only on a position that is not the
        destination's, unless the flight is going back where it came from.
        """
        if session.phase is not Phase.ENROUTE:
            return session.phase in self._CLIMBING_OUT
        destination = session.destination
        if destination is None or station.ident != destination.ident:
            return True
        origin = session.departure
        return origin is not None and origin.ident == destination.ident

    def _terminal_area(self, airport):
        """A field's approach area (navdata.approach_areas), when the switch
        is on and the field's coordinates fall in one; else None."""
        if airport is None or not self.brain.realism.approach_areas:
            return None
        return self.navdb.approach_area(airport.ident)

    def _arriving_in_area(self, state: AircraftState, destination,
                          to_run: float) -> str:
        """Why the destination's approach now has the flight, or "".

        Below the area's ceiling, and inside the field's approach area or
        close enough for a small one -- the area alone is not enough, since
        an arrival still at FL300 over it is the centre's. Without an area,
        the old sixty miles.
        """
        if destination is None:
            return ""
        area = self._terminal_area(destination)
        if area is not None:
            if area.contains(state.latitude, state.longitude, state.altitude_ft):
                return f"inside {area.id} below {int(area.ceiling_ft)} ft"
            # A small area and a fast arrival: handed over in time to get
            # down, three miles a thousand feet above the field and room to
            # slow and turn (after LocalTC). Heathrow Director's area is
            # twenty miles across, and without this London Control gave the
            # descent to 3,000 ft and approach took the flight at FL063.
            above = float(state.altitude_ft) - float(destination.elev_ft or 0.0)
            reach = above / 1000.0 * self.SMALL_AREA_NM_PER_1000FT \
                    + self.SMALL_AREA_SPARE_NM
            if state.altitude_ft < area.ceiling_ft and to_run <= reach:
                return (f"below {int(area.ceiling_ft)} ft, {to_run:.0f} nm "
                        f"out, within {reach:.0f} nm of a small area")
            return ""
        if to_run <= self.HANDOFF_TO_APPROACH_NM:
            return f"within {self.HANDOFF_TO_APPROACH_NM:g} nm, no approach area"
        return ""

    def _radar_hands_on(self, state: AircraftState, session: FlightSession,
                        station: Station) -> None:
        """Departure to centre, centre to approach, approach to the tower."""
        if state.on_ground:
            return

        if station.position in ("DEP", "APP") \
                and self._climbing_out_with(session, station):
            if time.monotonic() - self._centre_looked_at < self.CENTRE_RETRY_S:
                return
            home = self.navdb.airport(station.ident)
            out = state.distance_to(home.lat, home.lon) if home else 0.0
            if self.brain.realism.climb_steps and session.ifr:
                # Where departure's airspace ends (atc/climb.py), and only
                # once the flight has checked in: a handoff before the first
                # word is a frequency the pilot never spoke on.
                area = self._terminal_area(session.departure or home)
                why = leaving_departure(
                    state.altitude_agl_ft, out, state.altitude_ft,
                    state.vertical_speed_fpm, session.assigned_altitude_ft,
                    session.cruise_altitude_ft or 0,
                    in_area=(area.covers(state.latitude, state.longitude)
                             if area is not None else None),
                    area_ceiling_ft=getattr(area, "ceiling_ft", 0.0))
                leaving = bool(why) and session.established_with(station)
                if leaving and "handoff:centre" not in session.advisories:
                    decision("climb", event="handoff", station=station.callsign,
                             why=why, here_ft=round(state.altitude_ft),
                             out_nm=round(out, 1),
                             area=getattr(area, "id", None))
            else:
                leaving = (state.altitude_ft >= self.HANDOFF_TO_CENTRE_FT
                           or out >= self.HANDOFF_TO_CENTRE_NM)
            if leaving and session.advise_once("handoff:centre"):
                centre = self.navdb.center_station(
                    state.latitude, state.longitude, state.altitude_ft)
                if centre is None or not centre.mhz \
                        or station_key(centre) == station_key(station):
                    # No enroute frequency on file above this sector, which is
                    # the ordinary case over the United Kingdom and France --
                    # the open data has almost no area control frequencies in
                    # Europe. The flight stays with the radar position it is
                    # already talking to. Releasing it here is what left a
                    # departure out of Manchester with nobody at all: told
                    # "frequency change approved", handed nothing to change
                    # to, and never spoken to again.
                    #
                    # Re-armed rather than spent, because the answer changes
                    # with the ground underneath: the same flight an hour
                    # later, over Germany, does have a centre to call.
                    session.advisories.discard("handoff:centre")
                    self._centre_looked_at = time.monotonic()
                    self._no_enroute_frequency(session, centre)
                    return
                self._pass_to_station(session, station, centre)
            return

        destination = session.destination
        if not state.has_position:
            return
        to_run = (state.distance_to(destination.lat, destination.lon)
                  if destination is not None else float("inf"))

        arriving_why = self._arriving_in_area(state, destination, to_run)
        if station.position == "CTR" and not arriving_why:
            self._centre_hands_on(state, session, station)
            return
        if destination is None:
            return

        if (station.position == "CTR" and arriving_why
                and session.advise_once("handoff:approach")):
            approach = self._approach_at(destination, station)
            decision("handoff", to="approach", station=station.callsign,
                     field=destination.ident, why=arriving_why,
                     to_run_nm=round(to_run, 1),
                     altitude_ft=round(state.altitude_ft),
                     worked_by=("approach" if approach is not None
                                else "the centre: no approach control"))
            if approach is None:
                # No approach control at the field: the centre works the
                # arrival itself -- the descent, the approach clearance
                # and the handoff to the tower, which ArrivalWatch and
                # the tower handoff below give any radar position. It
                # used to say "frequency change approved" here and let
                # the flight go with nobody to call.
                return
            self._pass_to_station(session, station, approach)
            return
        # That has already been said. A field that publishes no approach
        # control is worked by the centre the whole way in, so the flight
        # falls through to the tower handoff below rather than being left
        # on a sector frequency with nobody to clear it to land.

        # The centre is in here as well as the terminal positions. Where a
        # field publishes no approach control the flight is worked by the
        # centre all the way in, and leaving it out meant that flight was
        # never passed to the tower at all -- it arrived on a sector
        # frequency with nobody to clear it to land.
        arriving = (station.position in ("APP", "DEP", "CTR")
                    and not session.phase.outbound
                    and self._on_the_way_in(state, session, destination, to_run))
        if arriving and session.advise_once("handoff:tower"):
            from ..atc.arrival import CLEARED_APPROACH_KEY

            decision("handoff", to="tower", station=station.callsign,
                     field=destination.ident, to_run_nm=round(to_run, 1),
                     why=("cleared and established on the final"
                          if CLEARED_APPROACH_KEY in session.advisories
                          and self.brain.realism.star_final
                          else "in the final's cone or close in"))
            self._pass_to(session, station, "TWR", at=destination.ident,
                          runway=session.arrival_runway)

    # How often a centre looks ahead. Ten seconds is a mile and a third at
    # cruise speed, against a handoff window two minutes wide.
    CENTRE_CHECK_S = 10.0
    # How long before the boundary a centre hands a flight on: a moment in
    # this window, the same one every time for a given flight and sector, so
    # one controller is early and the next a little later, as people are.
    HANDOFF_LEAD_S = (120.0, 240.0)
    # A sector the flight is across in less than this is coordinated by the
    # controllers either side, not handed into.
    SHORT_SECTOR_S = 240.0
    # How far ahead the centre looks: the window, a short sector, and margin.
    LOOK_AHEAD_MIN = 10.0
    # A centre does not hand a flight back to the one it has just left.
    BACK_AGAIN_S = 600.0
    # Climbing or descending faster than this towards its assigned level,
    # a flight is handed to the sector at that level.
    CHANGING_LEVEL_FPM = 500.0
    # Not switched by then, the handoff is said again, once.
    REOFFER_AFTER_S = 90.0
    # However short the time on a frequency, a flight half a minute from the
    # line is handed on.
    LAST_MOMENT_S = 30.0

    def _centre_hands_on(self, state: AircraftState, session: FlightSession,
                         station: Station) -> None:
        """Pass the flight to the next centre before it reaches the boundary.

        The sector ahead along the track (or the SimBrief route) is
        :meth:`EnRoute.crossings`'s answer; the handoff is given two to four
        minutes before the line, to the next controller in the region's
        words. It used to be given only once the aeroplane was inside the
        next sector, which is late by the whole of a real controller's
        margin, and before that it was measured from the field a frequency
        is filed at -- the circle round a country airstrip that took a
        flight across Australia on one frequency.

        A sector crossed in under four minutes is skipped; the handoff never
        goes to the station already worked, nor back to the one just left,
        nor while a readback is owed. Where the moment was missed -- the
        pilot was mid-exchange, or the data put the line closer than the
        window -- the flight is still handed on once it is inside the next
        sector, so it is never left with nobody. Every decision is a row of
        ``logs radio`` with its reason.
        """
        from ..atc.enroute import is_oceanic

        now = time.monotonic()
        if now - self._centre_checked_at < self.CENTRE_CHECK_S:
            return
        self._centre_checked_at = now
        handed = session.handoff_pending
        if handed is not None and station_key(handed) == station_key(station) \
                and session.established_with(station):
            # The flight is with the station it was sent to and has called
            # it: that handoff is done, and the next one may be given.
            session.handoff_pending = None
        if self._reoffer_handoff(session, station, now):
            return
        if session.handoff_pending is not None:
            return
        pending = session.pending
        if pending is not None and pending.readback_required \
                and not pending.read_back:
            return

        enroute = self.navdb.enroute
        lat, lon, alt = state.latitude, state.longitude, state.altitude_ft
        sector = enroute.which(lat, lon, alt)
        # On the way to a level it was given, the flight belongs to the
        # sector at that level, not to each one it passes through: coming
        # down into London it was handed 135.0, 134.9 and 120.525 inside a
        # minute, one stratum after another.
        assigned = float(session.assigned_altitude_ft or 0.0)
        vs = float(state.vertical_speed_fpm or 0.0)
        if assigned and abs(vs) > self.CHANGING_LEVEL_FPM \
                and (assigned - alt) * vs > 0:
            at_level = enroute.which(lat, lon, assigned)
            if at_level is not None:
                if sector is not None and at_level.key != sector.key \
                        and session.advise_once(
                            f"enroute:to_level:{at_level.key}"):
                    decision("enroute", event="to the level",
                             sector=at_level.facility,
                             passing=sector.facility, level_ft=int(assigned))
                sector, alt = at_level, assigned
        self._note_sector(sector)

        # Already inside a sector that is not this station's: late, but
        # handed on rather than left.
        if sector is not None and sector.oceanic:
            if not is_oceanic(station):
                self._go_oceanic(session, station, sector)
                return
        elif sector is None:
            here = self.navdb.center_station(lat, lon, alt)
            if here is not None and here.mhz \
                    and not same_channel(here.mhz, station.mhz, 4):
                self._pass_to_station(session, station, here)
            return

        crossings = enroute.crossings(
            lat, lon, state.heading_true, state.ground_speed_kt, alt,
            self.LOOK_AHEAD_MIN, self._route_points())
        if not sector.oceanic and not sector.answers(station.mhz):
            # Already over the line. Into a sector the flight leaves again
            # within four minutes it is not handed at all, or the late
            # handoff hands it in and straight out again: Maastricht, Reims,
            # Maastricht inside four miles.
            if crossings and crossings[0].minutes * 60.0 < self.SHORT_SECTOR_S:
                if session.advise_once(f"enroute:skipped:{sector.key}"):
                    decision("enroute", event="skipped",
                             sector=sector.facility,
                             reason=f"left again in "
                                    f"{crossings[0].minutes:.1f} min",
                             to=crossings[0].sector.facility)
                target = self._past_short_sectors(session, crossings)
                if target.sector.oceanic:
                    self._go_oceanic(session, station, target.sector)
                else:
                    self._centre_hands_to(session, station, target.sector,
                                          minutes=0.0, late=True)
                return
            self._centre_hands_to(session, station, sector, minutes=0.0,
                                  late=True)
            return
        if not crossings:
            return
        first = crossings[0]
        if session.advise_once(f"enroute:boundary:{first.sector.key}"):
            decision("enroute", event="boundary", sector=first.sector.facility,
                     minutes=round(first.minutes, 1),
                     distance_nm=round(first.distance_nm, 1))
        lead = self._handoff_lead_s(session, first.sector)
        if first.minutes * 60.0 > lead:
            return
        held = time.monotonic() - session.with_since if session.with_since else None
        if (held is not None and held < self.SHORT_SECTOR_S
                and first.minutes * 60.0 > self.LAST_MOMENT_S):
            # Handed in early to a sector not much wider than the window,
            # and out early again, the flight was on a frequency for thirty
            # seconds: Hong Kong Radar on one number, then on another four
            # miles on. A controller who has just taken a flight keeps it
            # four minutes, or until the line is half a minute off.
            if session.advise_once(f"enroute:held:{first.sector.key}"):
                decision("enroute", event="held", station=station.callsign,
                         sector=first.sector.facility,
                         reason=f"on this frequency {held / 60.0:.1f} min")
            return

        nxt = self._past_short_sectors(session, crossings).sector
        if nxt.oceanic:
            if not is_oceanic(station):
                self._go_oceanic(session, station, nxt)
            return
        self._centre_hands_to(session, station, nxt, minutes=first.minutes)

    def _past_short_sectors(self, session: FlightSession, crossings):
        """The first crossing whose sector the flight stays in for four
        minutes or more; those it is across sooner are logged as skipped."""
        target = crossings[0]
        for later in crossings[1:]:
            across = (later.minutes - target.minutes) * 60.0
            if across >= self.SHORT_SECTOR_S:
                break
            if session.advise_once(f"enroute:skipped:{target.sector.key}"):
                decision("enroute", event="skipped",
                         sector=target.sector.facility,
                         reason=f"crossed in {across / 60.0:.1f} min",
                         to=later.sector.facility)
            target = later
        return target

    def _centre_hands_to(self, session: FlightSession, station: Station,
                         sector, minutes: float, late: bool = False) -> None:
        """Say the handoff to ``sector``'s controller, unless it is a rule
        not to: the same station, or the one just left."""
        nxt = sector.station()
        if not nxt.mhz:
            return
        if sector.answers(station.mhz) or station_key(nxt) == station_key(station):
            if session.advise_once(f"enroute:same:{sector.key}"):
                decision("enroute", event="skipped", sector=sector.facility,
                         reason="the station already worked")
            return
        if self._handing_back(nxt):
            if session.advise_once(f"enroute:back:{station_key(nxt)}"):
                decision("enroute", event="skipped", sector=sector.facility,
                         reason="left less than ten minutes ago")
            return
        log.info("centre handoff%s, %.1f min before the line: %s %.3f -> "
                 "%s %.3f", " (late)" if late else "", minutes,
                 station.callsign, station.mhz, nxt.callsign, nxt.mhz)
        state = self.state
        here = (self.navdb.enroute.which(state.latitude, state.longitude,
                                         float(state.altitude_ft or 0.0))
                if state is not None and state.has_position else None)
        decision("enroute", event="offered", station=station.callsign,
                 to=nxt.callsign, frequency=round(nxt.mhz, 3),
                 minutes=round(minutes, 1), late=late,
                 from_sector=here.id if here is not None else None,
                 sector=sector.id,
                 source=sector.frequency_source, invented=sector.invented)
        session.left_at[station_key(station)] = time.monotonic()
        session.oceanic = None
        session.cpdlc = None      # the oceanic link closes with the ocean
        self._pass_to_station(session, station, nxt)

    def _handing_back(self, station: Station) -> bool:
        """Whether ``station`` is one the flight left under ten minutes ago."""
        session = self.session
        if session is None:
            return False
        left = session.left_at.get(station_key(station))
        return left is not None and time.monotonic() - left < self.BACK_AGAIN_S

    def _handoff_lead_s(self, session: FlightSession, sector) -> float:
        """How long before this boundary this flight is handed on.

        Seeded from the callsign and the sector, so a test flight -- or a
        pilot flying the same route twice -- meets the same controller
        habits, while two boundaries in a row are not handed on alike.
        """
        low, high = self.HANDOFF_LEAD_S
        seed = f"{session.aircraft.callsign.written}:{sector.id}:{sector.facility}"
        fraction = int(hashlib.sha1(seed.encode("utf-8")).hexdigest()[:8], 16)
        return low + (high - low) * fraction / 0xFFFFFFFF

    def _note_sector(self, sector) -> None:
        """A row in ``logs radio`` each time the flight enters a sector."""
        key = sector.key if sector is not None else None
        if key == self._sector_noted:
            return
        self._sector_noted = key
        if sector is not None:
            decision("enroute", event="entered", sector=sector.facility,
                     id=sector.id, source=sector.source,
                     frequency=round(sector.mhz, 3), invented=sector.invented)

    def _reoffer_handoff(self, session: FlightSession, station: Station,
                         now: float) -> bool:
        """Say a handoff again when the flight has not switched.

        Ninety seconds after it was given, with the flight still on this
        frequency: "did you get the frequency?", and the handoff once more.
        Once; after that it is the pilot's frequency to find. Any position,
        not only the centre's -- a "contact tower" a pilot never tuned
        would otherwise be announced exactly once and then forgotten.
        """
        handed = session.handoff_pending
        if handed is None or not handed.mhz:
            return False
        if now - session.handoff_offered_at < self.REOFFER_AFTER_S:
            return False
        if not session.advise_once(f"enroute:reoffer:{station_key(handed)}"):
            return False
        spoken = self._language_for(station.ident, self.last_language)
        p = self.brain.for_station(station, spoken)
        session.aircraft.language = getattr(p, "language", "en")
        text = p.handoff_reoffer(session.aircraft,
                                 handed.callsign_in(spoken, short=True),
                                 handed.mhz)
        decision("enroute", event="reoffered", station=station.callsign,
                 to=handed.callsign, frequency=round(handed.mhz, 3))
        self.emit("atc", text, station.callsign)
        session.note("atc", as_written(text))
        self.speak(text, station, language=spoken)
        self._radio_unprompted(text, station, "handoff_reoffer", spoken)
        session.handoff_offered_at = now
        return True

    def _route_points(self) -> list[tuple[float, float]] | None:
        """The loaded route as points, for looking ahead along it."""
        if self.route:
            return self.route.points()
        plan = self.plan
        if plan is None:
            return None
        if self._route_cache is not None and self._route_cache[0] is plan:
            return self._route_cache[1]
        from ..integrations.simbrief import route_points

        points = route_points(plan) or None
        self._route_cache = (plan, points)
        return points

    def _say_invented(self, station: Station) -> None:
        """Say once that a frequency was made up, and ask for the real one.

        A sector with no published frequency in any data this program has is
        given one rather than left silent (atc/enroute.py), and a made-up
        number that looks like a real one is a number nobody corrects.
        """
        from ..atc.enroute import is_invented

        session = self.session
        if session is None or not is_invented(station):
            return
        if not session.advise_once(f"invented:{station_key(station)}"):
            return
        mhz = f"{station.mhz:.3f}"
        self.emit("info",
                  f"No published frequency is known for {station.callsign}; "
                  f"{mhz} is invented. If you know the real one, please "
                  "report it on GitHub.",
                  key="log.invented_frequency", station=station.callsign,
                  mhz=mhz)

    def _go_oceanic(self, session: FlightSession, station: Station,
                    sector) -> None:
        """The last radar controller sends the flight to the ocean's HF station.

        Over the ocean there is no VHF radar, so the honest transmission is
        "radar service terminated, contact ... Radio", with the HF family
        where the data has one. Nothing a simulator's COM radio can tune, so
        the station is held in the session (``session.oceanic``) and taken
        over once the readback is in (:meth:`_settle_oceanic`).
        """
        oceanic = sector.station()
        spoken = self._language_for(station.ident, self.last_language)
        p = self.brain.for_station(station, spoken)
        session.aircraft.language = getattr(p, "language", "en")
        text = p.oceanic_contact(session.aircraft,
                                 oceanic.callsign_in(spoken, short=True),
                                 sector.hf_khz)
        log.info("oceanic: %s hands %s to %s (HF %s)", station.callsign,
                 session.aircraft.callsign.written, oceanic.callsign,
                 sector.hf_khz or "unknown")
        decision("enroute_oceanic", station=station.callsign,
                 to=oceanic.callsign, hf=list(sector.hf_khz))
        self.emit("atc", text, station.callsign)
        session.note("atc", as_written(text))
        self.speak(text, station, language=spoken)
        self._radio_unprompted(text, station, "oceanic", spoken)
        session.oceanic = oceanic
        session.handoff_pending = oceanic
        session.handoff_offered_at = time.monotonic()
        # The data link is how a real flight stays worked out here: where
        # the aircraft carries the box and the switch is on, the oceanic
        # authority's instructions come down CPDLC rather than HF.
        from ..atc import cpdlc
        if getattr(self.config.immersion.controller, "cpdlc", True) \
                and cpdlc.equipped(session):
            session.cpdlc = cpdlc.open_link(station_key(oceanic))
            log.info("oceanic: %s logged %s on to CPDLC", oceanic.callsign,
                     session.aircraft.callsign.written)
        # Nothing in it can be checked: the HF family is not a number the
        # simulator's radio takes. The first call the HF station hears --
        # the readback or a position report -- is its first contact.
        session.remember(Instruction("handoff", station_key(station), text,
                                     {"station": oceanic.callsign},
                                     readback_required=False))

    def _settle_oceanic(self, session: FlightSession) -> None:
        """Once the oceanic handoff is read back, the flight is with HF.

        There is no frequency to tune, so the readback is the switch. The
        station is established at once: a flight checks in with its first
        position report, and nothing in the ocean volunteers anything
        before it.
        """
        from ..atc.enroute import is_oceanic

        oceanic = session.oceanic
        if oceanic is None or is_oceanic(self.station):
            return
        pending = session.pending
        if pending is not None and pending.readback_required \
                and not pending.read_back:
            return
        self.station = oceanic
        session.establish(oceanic)
        log.info("now with %s on HF", oceanic.callsign)

    def cpdlc_respond(self, answer: str) -> bool:
        """The pilot's downlink: WILCO, UNABLE or ROGER on the open uplink.

        The panel POSTs here; the answer settles the instruction the way
        the spoken readback would, so everything downstream of a readback
        -- the applied values, the cleared pending -- behaves identically.
        """
        session = self.session
        link = getattr(session, "cpdlc", None)
        if link is None or session is None:
            return False
        downlink = link.respond(answer, session)
        if downlink is None:
            return False
        log.info("cpdlc: %s answers %s to %s",
                 session.aircraft.callsign.written, answer.upper(),
                 link.authority)
        session.note("cpdlc", downlink.text)
        self.emit("cpdlc", downlink.text,
                  session.aircraft.callsign.written)
        return True

    def cpdlc_view(self) -> dict:
        """The link's state for the panel, or nothing when there is none."""
        session = self.session
        link = getattr(session, "cpdlc", None)
        return link.view() if link is not None else {}

    # Close enough to the field that the tower has it whichever way it is
    # pointing. Inside this an aeroplane flying a visual circuit to the
    # runway, which is never in the cone until short final, is still passed.
    HANDOFF_ANYWAY_NM = 6.0

    def _on_the_way_in(self, state: AircraftState, session: FlightSession,
                       destination, to_run: float) -> bool:
        """Whether an arrival is where the tower takes it on.

        In the final-approach cone, within HANDOFF_TO_TOWER_NM of the
        threshold of the runway it is landing on -- not a radius round the
        field, which handed an aeroplane flying past the far side to a tower
        it was not going to land with. Very close in, anywhere.
        """
        from ..atc.arrival import CLEARED_APPROACH_KEY
        from ..atc.final import final_for

        final = final_for(self.navdb, destination.ident,
                          session.arrival_runway or "")
        if session.going_around:
            # Sent round: approach sequences it again first. Only a new
            # approach clearance and the final give it back to the tower --
            # "within six miles" handed a go-around straight back, climbing
            # out, twenty-five seconds after it had left.
            if CLEARED_APPROACH_KEY not in session.advisories:
                return False
            if final is None:
                return to_run <= self.HANDOFF_TO_TOWER_NM
            return final.in_cone(state, self.HANDOFF_TO_TOWER_NM, aligned=False)
        if self.brain.realism.vectors and session.ifr \
                and CLEARED_APPROACH_KEY not in session.advisories:
            # Vectored arrivals are cleared on the intercept; one handed to
            # the tower before that -- a downwind passes close to the field
            # -- was sent back to approach to be vectored.
            return False
        if to_run <= self.HANDOFF_ANYWAY_NM:
            return True
        if final is None:
            return to_run <= self.HANDOFF_TO_TOWER_NM
        if CLEARED_APPROACH_KEY in session.advisories \
                and self.brain.realism.star_final:
            # Cleared and established: to the tower about TOWER_AT_NM +
            # TOWER_MARGIN_NM out, lined up and tracking down the final
            # (after LocalTC: 6 + 4 nm).
            return final.in_cone(state, self.TOWER_AT_NM + self.TOWER_MARGIN_NM,
                                 aligned=True)
        return final.in_cone(state, self.HANDOFF_TO_TOWER_NM, aligned=False)

    def _no_enroute_frequency(self, session: FlightSession,
                              centre: Station | None) -> None:
        """Tell the pilot why nobody took the flight on, and how to fix it.

        The sector has a name -- the boundary data knows it is London -- and
        no frequency, because the open dataset has almost no area control
        frequencies outside North America. Staying with approach is the right
        thing to do about that and it looks exactly like a program that has
        stopped working, so it is said once, in the panel, with the command
        that adds the frequency for good.
        """
        sector = (centre.facility if centre is not None else "") or ""
        if not sector:
            # No boundary and no frequency: there is nothing to name and
            # nothing useful to say beyond what the pilot can already see.
            return
        if not session.advise_once("centre:unpublished"):
            return
        how = command_hint("freq")
        self.emit(
            "info",
            f"No enroute frequency is published for this area ({sector}). "
            "Staying with the radar controller you are on. Add one with: "
            f"{how} <ICAO> --add CTR <frequency> --name <name>",
            key="log.no_enroute_frequency", sector=sector, command=how,
        )

    def _welcome_to(self, station: Station) -> str:
        """Where the aeroplane has arrived, as a controller would name it.

        The field's spoken name rather than its ident: "welcome to Prague",
        not "welcome to LKPR". Empty where there is no name to use, which
        leaves the handoff exactly as it was.
        """
        airport = self.navdb.airport(station.ident)
        if airport is None:
            return ""
        return (getattr(airport, "municipality", "")
                or getattr(airport, "spoken", "") or "")

    def _radio_unprompted(self, text: str, station: Station, why: str,
                          language: str, instruction=None) -> None:
        """Put a transmission nobody asked for into the transcript.

        Only answers to the pilot were written, so the approach clearance,
        the handoffs and the readback chases -- the calls a pilot most often
        wants to check after a flight -- were missing from ``logs radio``.
        ``why`` is which watch said it.
        """
        RADIO.write(
            "atc", text=text, language=language, unprompted=True, why=why,
            station=station.callsign, frequency=round(station.mhz, 3),
            instruction=instruction.kind if instruction is not None else "",
            owed=instruction.owed_items if instruction is not None else [],
            values=dict(instruction.values) if instruction is not None else {},
        )

    def _approach_at(self, field, station: Station):
        """The field's approach or departure control, from this program's
        station data (never the approach area's name), or None."""
        for position in ("APP", "DEP"):
            found = self.brain.controller_at(field.ident, position)
            if found is not None and station_key(found) != station_key(station):
                return found
        return None

    def _pass_to(self, session: FlightSession, station: Station,
                 *positions: str, at: str = "", welcome: str = "",
                 runway: str = "", when_able: bool = False) -> None:
        """Say who to call next, or release the frequency if nobody is next.

        ``at`` is the field whose positions to look through, which is the one
        being talked to unless the flight is being handed forward to where it
        is going.

        A field with no radar and no ground has nobody to pass you to, and the
        honest answer there is "frequency change approved" rather than silence
        -- which is what the pilot got, and what looked like a program that had
        stopped working.
        """
        nxt = None
        for position in positions:
            found = self.brain.controller_at(at or station.ident, position,
                                             runway=runway)
            if found is not None and station_key(found) != station_key(station):
                nxt = found
                break
        self._pass_to_station(session, station, nxt, welcome=welcome,
                              when_able=when_able)

    def _pass_to_station(self, session: FlightSession, station: Station,
                         nxt: Station | None, welcome: str = "",
                         when_able: bool = False) -> None:
        """Say the handoff, in the language the frequency is being worked in."""
        spoken = self._language_for(station.ident, self.last_language)
        p = self.brain.for_station(station, spoken)
        session.aircraft.language = getattr(p, "language", "en")

        if nxt is not None:
            text = p.handoff(session.aircraft,
                             nxt.callsign_in(spoken, short=True), nxt.mhz,
                             when=p.when_able() if when_able else "",
                             farewell=True, welcome=welcome)
            instruction = Instruction("handoff", station_key(station), text,
                                      {"frequency": nxt.mhz})
        else:
            text = p.frequency_change_approved(session.aircraft)
            instruction = Instruction("frequency_change", station_key(station),
                                      text, {}, readback_required=False)

        self.emit("atc", text, station.callsign)
        session.note("atc", as_written(text))
        self.speak(text, station, language=spoken)
        self._radio_unprompted(text, station, "handoff", spoken, instruction)
        session.handoff_pending = nxt
        session.handoff_offered_at = time.monotonic()
        session.remember(instruction)

    def _offer_handoff(self, state: AircraftState, session: FlightSession) -> None:
        """Prompt the pilot when they have flown past the controller they are on.

        Real controllers hand off before you leave their airspace. Here the
        cue is coverage: once the tuned station is out of plausible range, the
        pilot is told who to call next rather than being left talking to
        somebody who cannot hear them.
        """
        station = self.station
        if not self.brain.staffed(station) or not state.has_position:
            return
        if state.on_ground:
            # A parked aeroplane has not flown past anybody: a tuned
            # station that reads out of range from the stand is a wrong
            # dial, not a handoff to give -- and the first officer, which
            # obeys every handoff it is written down as, was being sent
            # to a radar frequency while it was still asking delivery
            # for its clearance.
            return
        if station.position == "CTR":
            # A centre hands on by sector (_centre_hands_on). Its range from
            # the field its frequency is filed at says nothing about where
            # its airspace ends.
            return
        pending = session.pending
        if pending is not None and pending.station_key == station_key(station):
            # A readback owed to this station is collected before it
            # hands the flight on; one owed to the last frequency does
            # not pin the flight inside this coverage forever.
            return
        if self.stand_down or not self.quiet_enough():
            return
        if time.monotonic() - session.handoff_offered_at < self.config.atc.handoff_grace_s:
            return
        if station.position == "INFO":
            # A VFR flight on an information service is not handed to a
            # radar position. The next information sector, or a free call.
            self._leave_information(session, station, state)
            return

        where = self._station_position(station)
        reach = STATION_RANGE_NM.get(station.position, 60.0)
        # A station the database cannot place cannot bound its own coverage:
        # treat the flight as at the edge of it, so leaving still earns the
        # handoff rather than silence.
        distance = state.distance_to(*where) if where else reach
        if distance <= reach * 0.9:
            return

        nxt = self._next_station(state, session, station)
        if nxt is None and session.flight_following and not session.ifr:
            # Out of range of the advisories with nobody to hand them on to:
            # the service ends, in the region's words, rather than going
            # quiet on a flight that thinks it is still being watched.
            if not session.advise_once(f"ff:end:{station_key(station)}"):
                return
            spoken = self._language_for(station.ident, self.last_language)
            p = self.brain.for_station(station, spoken)
            session.flight_following = False
            session.squawk = p.vfr_squawk
            text = p.radar_service_terminated(session.aircraft)
            decision("flight_following", station=station.callsign, said="terminated")
            self.emit("atc", text, station.callsign)
            self.speak(text, station, language=spoken)
            self._radio_unprompted(text, station, "flight_following_end", spoken)
            return
        if nxt is None or station_key(nxt) == station_key(station):
            return
        if (nxt.callsign == station.callsign
                and same_channel(nxt.mhz, station.mhz, 0)):
            # The same facility filed under another airport -- one approach
            # position serving several fields is a normal thing, and the
            # answer to "you have flown past me" is then the very station
            # and frequency the flight is on. Handing it to itself loops
            # forever, and overwrites a handoff that is still owed.
            return

        spoken = self._language_for(station.ident, self.last_language)
        p = self.brain.for_station(station, spoken)
        text = p.handoff(session.aircraft, nxt.callsign_short, nxt.mhz, farewell=True)
        self.emit("atc", text, station.callsign)
        self.speak(text, station, language=spoken)
        self._radio_unprompted(text, station, "handoff_reminder", spoken)
        session.handoff_pending = nxt
        session.handoff_offered_at = time.monotonic()
        session.remember(
            Instruction("handoff", station_key(station), text,
                        {"frequency": nxt.mhz})
        )

    def _leave_information(self, session: FlightSession, station: Station,
                           state: AircraftState) -> None:
        """An en-route information service letting a flight go at the edge
        of its range: to the next information sector where one can hear it,
        else "squawk 7000, frequency change approved".

        The coverage handoff above sends a flight to a centre or an
        approach, which for a VFR flight talking to Langen Information was
        a radar position that had never heard of it. An aerodrome service
        says nothing: the pilot reports leaving it.
        """
        from ..atc import afis

        if afis.kind_of(self.navdb, station) != afis.FIS:
            return
        if not session.established_with(station):
            return
        where = self._station_position(station)
        if where is None:
            return
        reach = STATION_RANGE_NM.get("INFO", 120.0)
        if state.distance_to(*where) <= reach * 0.9:
            return
        if not session.advise_once(f"fis:left:{station_key(station)}"):
            return
        spoken = self._language_for(station.ident, self.last_language)
        p = self.brain.for_station(station, spoken)
        nxt = self._next_information(station, state)
        if nxt is not None:
            text = p.handoff(session.aircraft,
                             self.brain.station_name(nxt, p, short=True),
                             nxt.mhz, farewell=True)
            session.handoff_pending = nxt
            values = {"frequency": nxt.mhz}
        else:
            session.squawk = p.vfr_squawk
            text = p.frequency_change_approved(session.aircraft,
                                               squawk=p.vfr_squawk)
            values = {}
        decision("fis", station=station.callsign, said="leaving_range",
                 next=nxt.callsign if nxt is not None else None)
        session.handoff_offered_at = time.monotonic()
        session.remember(Instruction("handoff", station_key(station), text,
                                     values, readback_required=bool(values)))
        self.emit("atc", text, station.callsign)
        self.speak(text, station, language=spoken)
        self._radio_unprompted(text, station, "handoff_reminder", spoken)

    def _next_information(self, current: Station,
                          state: AircraftState) -> Station | None:
        """The nearest other en-route information sector in reach."""
        from ..atc import afis

        best = None
        for airport, _distance in self.navdb.nearest_airports(
                state.latitude, state.longitude, limit=25, max_nm=120.0):
            for other in self.navdb.stations(airport.ident):
                if other.position != "INFO" or not other.mhz:
                    continue
                if other.facility.strip().lower() == current.facility.strip().lower():
                    continue
                if afis.kind_of(self.navdb, other) != afis.FIS:
                    continue
                if not self._within_reach(other, state):
                    continue
                distance = state.distance_to(airport.lat, airport.lon)
                if best is None or distance < best[0]:
                    best = (distance, other)
        return best[1] if best else None

    def _station_position(self, station: Station) -> tuple[float, float] | None:
        airport = self.navdb.airport(station.ident)
        return (airport.lat, airport.lon) if airport else None

    def _next_station(self, state, session, current: Station) -> Station | None:
        """Who the flight should be talking to next, and can actually reach.

        Every candidate is range-checked. Without that, a departure out of
        Manchester that had flown past Manchester Approach was told to contact
        Palma de Mallorca Approach -- correct about where the flight was going
        and nine hundred miles wrong about whether anybody there could hear
        it. Saying nothing is the right answer until the destination is in
        range, and this is called again on every pass, so the moment it is,
        the pilot is told.

        In the climb and the cruise only a centre takes the flight on, and
        the destination's approach once it is as close as the centre would
        hand it over. Falling through to whatever was near handed a flight
        at FL350 to an approach underneath it, or to Melbourne Approach from
        a hundred and fifty miles out, because both were in radio range.
        """
        # Descent is still enroute until the destination terminal area
        # actually owns the flight. Treating every descent as terminal
        # traffic made coverage handoffs hop between whatever approach
        # units happened to be underneath -- Santiago to Salamanca while
        # still far from Madrid -- rather than keeping the flight with
        # a centre or waiting for the destination approach.
        en_route = session.phase in (
            Phase.DEPARTURE, Phase.ENROUTE, Phase.DESCENT)
        if en_route:
            centre = self.navdb.center_station(
                state.latitude, state.longitude, state.altitude_ft
            )
            if centre and centre.mhz and self._within_reach(centre, state):
                return centre
            destination = session.destination
            if destination is None or not self._arriving_in_area(
                    state, destination, state.distance_to(
                        destination.lat, destination.lon)):
                return None
            found = self.brain.controller_at(destination.ident, "APP")
            if found is not None and self._within_reach(found, state):
                return found
            return None
        if session.destination is not None:
            for position in ("APP", "TWR"):
                found = self.brain.controller_at(session.destination.ident,
                                                 position)
                if found is not None and self._within_reach(found, state):
                    return found
        nearby = self.navdb.nearest_airport(
            state.latitude, state.longitude, max_nm=60.0, min_rank=2
        )
        if nearby:
            found = self.brain.controller_at(nearby.ident, "APP")
            if found is not None and self._within_reach(found, state):
                return found
        return None

    def _within_reach(self, station: Station, state: AircraftState) -> bool:
        """Whether the aeroplane could raise this station from where it is.

        The same arithmetic :meth:`~meridianatc.navdata.db.NavDB
        .candidate_stations` uses to decide who answers on a tuned frequency,
        so the station named in a handoff is one that will actually be there
        when the pilot dials it. A centre with no airport behind it -- the
        name comes from a boundary, not from a field -- is taken on trust:
        a sector is the size of a country and there is nothing to measure to.
        """
        where = self._station_position(station)
        if where is None or not state.has_position:
            return True
        distance = state.distance_to(*where)
        base = STATION_RANGE_NM.get(station.position, 40.0)
        if station.position in ("DEL", "GND"):
            return distance <= base
        los = 1.23 * math.sqrt(max(0.0, state.altitude_ft))
        return distance <= max(base, min(los, base * 4.0))

    # ------------------------------------------------------------------

    def warm(self) -> None:
        """Load models so the first transmission is not delayed."""
        self.emit("info", "Loading speech recognition...",
                  key="log.loading_stt")
        from ..audio.stt import SpeechModelMissing

        try:
            self.recognizer.warm()
        except SpeechModelMissing:
            # A fresh install: the first-run setup's Downloads screen fetches
            # it, and the recogniser loads on the first transmission after.
            log.info("speech model not downloaded yet; the setup fetches it")
        # Kokoro is loaded whatever ``preload`` says: it is one model for
        # nearly every controller, and paying for it here rather than on the
        # first clearance is the difference between a two-second wait for the
        # taxi instruction and no wait at all. ``preload`` still decides
        # whether the Piper voices behind it are loaded too, because that is
        # the one that costs a gigabyte.
        self.emit("info", "Loading voices...", key="log.loading_voices")
        if self.synth.registry.hq_missing():
            # Asked for the cloned voices and cannot have them. Said here,
            # once, rather than on every transmission: the controllers will
            # answer in the ordinary voices, which is a working radio in the
            # wrong cast, and the pilot who set the tier up should know.
            self.emit(
                "warn",
                "The cloned voices are not installed, so the controllers "
                "are using the standard voices. Download the cloning model "
                f"with: {command_hint('setup --hq')}",
                key="log.hq_missing", command=command_hint("setup --hq"),
            )
        self.synth.warm(piper=self.config.voice.preload)
        self.emit("info", "Ready.", key="log.ready")
