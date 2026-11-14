from __future__ import annotations

from ._shared import (
    AircraftState,
    ControllerMissing,
    ControllersUnavailable,
    Event,
    FlightProfile,
    JoystickPushToTalk,
    Phraseology,
    PushToTalk,
    SimTraffic,
    XPlaneTraffic,
    as_written,
    capabilities,
    log,
    threading,
    time,
)


class _Setup:
    # ------------------------------------------------------------------
    # setup
    # ------------------------------------------------------------------

    def _base_phraseology(self) -> Phraseology:
        # The brain's fallback, for a station with no field. Each controller
        # is built for its own station (ControllerBrain.for_station).
        return Phraseology(
            FlightProfile.of(self.config.flight).dialect_for(""),
            strict_icao_digits=self.config.atc.strict_icao_digits,
        )

    # Kinds that are somebody talking on the radio, and therefore get written
    # down the way a pilot writes on a strip rather than the way it was said.
    #
    # The crew are on the list too. They are not on the radio, but they are
    # somebody speaking, and a first officer calling "ten thousand feet"
    # should read as figures in the transcript for the same reason a
    # controller does.
    _TRANSCRIBED = frozenset({"pilot", "atc", "atis", "traffic", "crew"})

    def emit(self, kind: str, text: str, speaker: str = "", *,
             key: str = "", action: str = "", **params) -> None:
        """Show something to the user.

        A transmission reaches the screen in figures where a pilot would write
        figures -- runway 31, QNH 1013, 127.750 rather than "one two seven
        decimal seven five zero". The spoken form is what goes to the
        synthesiser and is untouched by this; the two are different strings on
        purpose, and ``ui.numerals`` decides only which one is shown.
        """
        if kind in self._TRANSCRIBED:
            text = as_written(text, figures=self.config.ui.numerals)
            # And kept, because this is the only place every transmission goes
            # through. The window has a live connection and does not need it;
            # the panel inside the simulator polls, and a poll that arrives
            # between two transmissions would otherwise never see them.
            entry = {
                "at": time.time(), "kind": kind,
                "speaker": speaker, "text": text,
            }
            if params.get("radio"):
                # "com1"/"com2", so a panel can give each radio its own page.
                entry["radio"] = params["radio"]
            self.transcript.append(entry)
            if self.history is not None:
                # The durable copy: who, what, and on which station, into the
                # session the flight opened. Nothing here reaches back into
                # the engine.
                try:
                    station = getattr(getattr(self, "session", None),
                                      "current_station", None)
                    self.history.record(
                        who=speaker or kind, text=text,
                        station=getattr(station, "ident", ""),
                        intent=key or "",
                        meta=dict(params))
                except Exception:
                    log.debug("history write failed", exc_info=True)
        if kind == "error":
            # The plugin's copy_error binding asks for this one.
            self._last_error = text
        try:
            self.on_event(Event(kind, text, speaker, key=key, action=action,
                                params={k: str(v) for k, v in params.items()}))
        except Exception:
            log.debug("event sink raised", exc_info=True)

    def voices_installed(self) -> None:
        """Something was just downloaded into the voice folder.

        Called by the panel when the installer finishes. The engine is running
        and has already read an empty folder; without this the pilot has to
        restart the program they have only just finished setting up, which is
        the worst moment to be asked to.
        """
        self.synth.refresh()
        self._voices_reported = False
        try:
            self.synth.warm(piper=self.config.voice.preload)
        except Exception:
            log.debug("warming after a download raised", exc_info=True)

    def use_cloned_voices(self) -> None:
        """The cloned voices were asked for from the window, and have arrived.

        Asking for them is the pilot's choice of engine, made the only way a
        pilot without a text editor can make it, so it is written to
        ``config.yaml`` and survives a restart. The synthesiser is switched
        in place (:meth:`Synthesizer.use_engine`), because nothing that is
        downloaded requires a restart to take effect.
        """
        voice = self.config.voice
        if voice.engine != "chatterbox":
            voice.engine = "chatterbox"
            try:
                self.config.save()
            except Exception as exc:
                log.warning("could not write config.yaml: %s", exc)
        self.synth.use_engine(voice.engine, gpu=voice.gpu,
                              hq_model=voice.hq_model,
                              hq_takes=voice.hq_takes)
        self.synth.verify_with(self.recognizer)
        self.voices_installed()

    def load_flight_plan(self, username: str = "") -> str:
        """Fill the flight in from the latest SimBrief plan.

        Returns a line describing what was loaded. Raises
        :class:`~meridianatc.integrations.simbrief.SimBriefError` with something
        worth showing a person if it could not be.
        """
        from ..integrations.simbrief import SimBriefError, fetch

        if self.config.network.offline:
            raise SimBriefError("Offline mode is on.")
        plan = fetch(username or self.config.simbrief.username)
        self.apply_flight_plan(plan)
        self.emit("info", f"SimBrief: {plan.described}",
                  key="log.simbrief", detail=plan.described)
        return plan.described

    def load_plan_on_start(self) -> None:
        """The latest SimBrief plan, when the pilot asked for it at startup.

        The same path as the panel's button (:meth:`load_flight_plan`), on a
        thread of its own because it is a network fetch and the radio must
        not wait for it. The session is begun again with what it filled in,
        exactly as a change to the flight on the screen does.
        """
        self._plan_thread = None
        simbrief = self.config.simbrief
        if not (simbrief.load_on_start and simbrief.username.strip()):
            return

        def fetch() -> None:
            from ..integrations.simbrief import SimBriefError

            try:
                self.load_flight_plan(simbrief.username.strip())
            except SimBriefError as why:
                self.emit("warning", f"SimBrief: {why}",
                          key="log.simbrief_failed", detail=str(why))
                return
            except Exception as exc:
                log.warning("SimBrief at startup failed: %s", exc)
                return
            self.begin_session()

        self._plan_thread = threading.Thread(
            target=fetch, name="MeridianATC-simbrief", daemon=True)
        self._plan_thread.start()

    def apply_flight_plan(self, plan) -> None:
        """Fill the flight in from a parsed plan."""
        flight = self.config.flight
        if plan.callsign:
            flight.callsign = plan.callsign
        if plan.aircraft_type:
            flight.aircraft_type = plan.aircraft_type
        flight.departure = plan.origin
        flight.destination = plan.destination
        if plan.cruise_altitude_ft:
            flight.cruise_altitude_ft = plan.cruise_altitude_ft
        # A filed plan that says which rules it was filed under outranks the
        # setting: a pilot who has loaded a flight plan has said more about
        # this flight than the config file ever did. One that does not say
        # leaves the pilot's choice alone -- it used to make every such plan
        # IFR, whatever the pilot of a VFR flight had chosen.
        if plan.ifr is not None:
            flight.ifr = plan.ifr
        self.plan = plan
        from ..atc.route import from_ofp

        self.use_route(from_ofp(plan))
        # The route's fixes, which are what a centre grants a direct to.
        from ..integrations.simbrief import route_fixes

        self.brain.plan_fixes = route_fixes(plan)
        # The planned runways and SID, which the controllers use while the
        # wind allows them.
        self.brain.plan_runways = {}
        if plan.origin:
            self.brain.plan_runways[plan.origin] = {
                "departing": plan.planned_runway, "sid": plan.sid}
        if plan.destination:
            self.brain.plan_runways.setdefault(plan.destination, {})
            self.brain.plan_runways[plan.destination]["landing"] = \
                plan.arrival_runway

        # A session already under way is rebuilt so the new callsign is what
        # the controller answers to from the next transmission.
        if self.session is not None:
            self.begin_session()

    def attach_simulator_traffic(self) -> bool:
        """Let the other aeroplanes be seen, not only heard.

        Attach the renderer for the connected source. A simulator that will
        not have them leaves the radio alone.
        """
        xplane = capabilities(self.state.source).xplane_traffic
        if xplane != isinstance(self.simtraffic, XPlaneTraffic):
            self.simtraffic.detach()
            options = dict(
                enabled=self.config.traffic.enabled and self.config.traffic.in_simulator,
                limit=self.config.traffic.in_simulator_limit + max(0, int(self.config.traffic.parked)),
                live_limit=self.config.traffic.live.limit,
                settle_s=self.config.traffic.lod.settle_s,
                smooth=self.config.traffic.smooth_motion,
            )
            if xplane:
                self.simtraffic = XPlaneTraffic(
                    **options, host=self.config.traffic.xplane_plugin_host,
                    port=self.config.traffic.xplane_plugin_port)
            else:
                self.simtraffic = SimTraffic(**options)
            self.simtraffic.lod_models = dict(self.config.traffic.lod.models or {})
            self.simtraffic.on_swapped = self._model_swapped
            self.simtraffic.on_removed = self._model_removed
            self._ai_liveries_asked = False
        if xplane:
            self.traffic.await_observed = False
            drawing = self.simtraffic.attach()
            if not drawing and self.simtraffic.enabled:
                self._warn_xplane_traffic()
            return drawing
        simconnect = getattr(self.source, "_sim", None)
        if simconnect is None:
            return False
        # Reading is attached whichever way the writing goes: a pilot who has
        # switched the injected traffic off still has FSLTL running.
        watching = self.simwatch.attach(simconnect)
        # The stands are not set out until the simulator has said what is
        # already standing on them (or has had its chance to).
        self.traffic.await_observed = watching
        self.use_fsltl_liveries()
        # Logged rather than shown. "FSLTL is running and the controller is
        # not seeing it" has four possible causes and none of them is
        # something a pilot can act on from the panel, so it goes where
        # somebody reporting the problem will find it.
        log.info("%s", self.simwatch.describe())
        drawing = self.simtraffic.attach(simconnect)
        if drawing:
            self._read_ai_liveries()
            # Nothing to handle unless the aeroplanes are drawn.
            self.simvehicles.attach(simconnect)
            self.simdoors.attach(simconnect)
            self.simsystems.attach(simconnect)
            self.servicepoints.attach(simconnect)
            log.info("%s", self.simdoors.describe())
        return drawing or watching

    def _warn_xplane_traffic(self) -> None:
        if getattr(self, "_xptraffic_notice", False):
            return
        self._xptraffic_notice = True
        self.emit("info",
                  "X-Plane traffic is not available. Install or enable MeridianATC-XP "
                  "and its CSL models; see xplane/README.md. Traffic stays radio-only.",
                  key="log.xplane_traffic_unavailable")

    # How long the simulator is given to list its aircraft and liveries.
    # A running 2024 lists six thousand in a second or two.
    AI_LIVERIES_S = 20.0

    def _read_ai_liveries(self) -> None:
        """Learn which airlines the simulator's own AI models are painted
        for, on a worker, once per session.

        Until it answers -- and for ever, on a library without the call --
        the detailed models are drawn in their default livery, as they
        always were.
        """
        if self.simtraffic.ai_liveries is not None or self._ai_liveries_asked:
            return
        self._ai_liveries_asked = True
        from ..integrations.ailiveries import AiLiveries
        from ..integrations.groundvehicles import enumerate_simobjects

        def read() -> None:
            try:
                pairs = enumerate_simobjects(
                    2, self.config.sim.simconnect_dll,
                    timeout_s=self.AI_LIVERIES_S,
                    name=b"MeridianATC liveries")
            except Exception:
                log.info("listing the simulator's aircraft raised",
                         exc_info=True)
                return
            if not pairs:
                log.info("the simulator did not list its aircraft; the "
                         "detailed models keep their default livery")
                return
            found = AiLiveries.from_pairs(pairs)
            self.simtraffic.ai_liveries = found
            log.info("AI liveries: %d models painted for airlines, out of "
                     "%d aircraft listed", len(found), len(pairs))

        threading.Thread(target=read, name="MeridianATC-ai-liveries",
                         daemon=True).start()

    def use_fsltl_liveries(self) -> bool:
        """Paint the traffic in its airlines' colours, if FSLTL is here.

        Read once per session: the table is eleven megabytes and does not
        change while the simulator is running.
        """
        if self.simtraffic.liveries is not None:
            return True
        from ..integrations.fsltl import load_installed

        try:
            liveries = load_installed(
                override=self.config.traffic.fsltl_rules)
        except Exception:
            log.info("could not read FSLTL's liveries", exc_info=True)
            liveries = None
        if not liveries:
            log.info("FSLTL is not installed; the traffic uses the "
                     "simulator's own models")
            return False
        self.simtraffic.liveries = liveries
        self.traffic.has_livery = liveries.has_livery
        log.info("FSLTL liveries from %s (%d rules)", liveries.source,
                 len(liveries))
        self.emit("info",
                  f"Using FSLTL's liveries for {liveries.airlines} airlines.",
                  key="log.fsltl_liveries", count=str(liveries.airlines))
        return True

    def open_simulator_frequencies(self) -> bool:
        """Get ready to read airport frequencies out of the simulator.

        Needs a SimConnect library new enough to answer facility questions.
        Without one the published data is used and nothing is said about it,
        because there is nothing the pilot can do differently.
        """
        if not capabilities(self.state.source).facilities or not self.config.sim.frequencies_from_sim:
            return False
        return self.facilities.open()

    def follow_facilities(self, state: AircraftState) -> None:
        """Read the airport under the aeroplane, once, in the background."""
        if not capabilities(state.source).facilities or not self.facilities.available or not state.has_position:
            return
        airport = self.navdb.home_airport(state.latitude, state.longitude)
        if airport is None:
            airport = self.navdb.nearest_airport(
                state.latitude, state.longitude, max_nm=30.0, min_rank=2)
        if airport is None or airport.ident in self._facilities_done:
            return
        self._facilities_done.add(airport.ident)
        threading.Thread(
            target=self._read_facilities, args=(airport.ident,),
            name=f"MeridianATC-freq-{airport.ident}", daemon=True).start()

    def _read_facilities(self, ident: str) -> None:
        """Replace one airport's published frequencies with the simulator's."""
        from ..navdata.localdata import replace_airport

        try:
            found = self.facilities.frequencies(ident)
            if not found:
                return
            added, gone = replace_airport(ident, found)
            if not added:
                return
        except Exception:
            log.debug("could not read %s from the simulator", ident,
                      exc_info=True)
            return

        self.emit("info",
                  f"{ident}: {added} frequencies from the simulator.",
                  key="log.frequencies_from_sim", airport=ident,
                  count=added)
        # Whatever is tuned was resolved against the old list, so it is
        # resolved again -- this is how 123.605 at de Gaulle starts answering.
        if self.state.com1_active:
            try:
                self.tune(self.state.com1_active)
            except Exception:
                log.debug("re-tuning after the import raised", exc_info=True)

    def attach_lvar_bridge(self) -> str:
        """Give the ground handling a way to read the simulator's variables.

        Called when the simulator arrives, because until it has there is
        nothing to open a channel to. Safe to call again: a bridge that is
        already reading is kept, and a failed attempt leaves the ground
        handling exactly where it was -- watching the aeroplane's own
        movement, which needs nothing installed.
        """
        if not capabilities(self.state.source).lvars or not self.config.gsx.enabled:
            return ""
        existing = self.gsx.bridge
        if existing is not None and getattr(existing, "available", False):
            return existing.name

        from ..integrations.lvars import open_bridge

        simconnect = getattr(self.source, "_sim", None)
        bridge = open_bridge(self.config.gsx.bridge,
                             self.config.gsx.lvar_file, simconnect,
                             client=self.config.gsx.client)
        self.gsx.attach(bridge if bridge.available else None)
        if not bridge.available:
            # Not an error and not shown. A pilot without the MobiFlight
            # module has a controller that still watches the ramp through the
            # aeroplane, and there is nothing here for them to act on that
            # ``doctor`` does not say better.
            log.info("nothing can read the simulator's local variables")
            return ""
        log.info("ground handling read through %s", bridge.name)
        return bridge.name

    def start(self, connect_sim: bool = True) -> None:
        """Connect everything and begin the situation loop."""
        self.player.start()
        # The simulator's runways kept from earlier sessions, ahead of the
        # database's (navdata/runway_cache.py). Here and not in __init__, so
        # an engine a test builds reads the database whatever is cached.
        self._use_runway_cache()

        self._sim_wanted = bool(connect_sim and self.config.sim.enabled)
        self._stop.clear()
        if self._sim_wanted:
            self._watch_simulator()
            self.emit(
                "info",
                "Simulator not detected. Waiting for it, and flying from the "
                "manually set aircraft state until it appears.",
                key="log.sim_waiting",
            )
        if self.config.immersion.enabled:
            self.cabin.start()
        self.refresh_state()
        self.begin_session()
        self.load_plan_on_start()
        self.start_live_feed()
        self.emit("info", self.understanding.describe(),
                  key=("set.rules_plus" if self.understanding.model_available
                       else "set.rules_only"),
                  model=(f"{self.understanding.provider.name} "
                         f"({getattr(self.understanding.provider, 'model', '?')})"
                         if self.understanding.model_available else ""))
        self._start_ptt()

        self._stop.clear()
        self._situation_thread = threading.Thread(
            target=self._situation_loop, daemon=True
        )
        self._situation_thread.start()

    def _ptt_controller_lost(self, controller) -> None:
        """The push-to-talk controller was unplugged mid-session. Said, not
        left silently dead: a pilot pressing a button that does nothing
        would otherwise think the radio had failed."""
        self.emit("warning", f"Push-to-talk: {controller.name} was "
                  "disconnected.", key="log.ptt_controller_lost",
                  name=controller.name)

    def _ptt_controller_back(self, controller) -> None:
        self.emit("info", f"Push-to-talk: {controller.name} is connected "
                  "again.", key="log.ptt_controller_back",
                  name=controller.name)

    def _start_ptt(self) -> None:
        mode = self.config.ptt.mode
        if mode == "none":
            return
        try:
            if mode == "joystick":
                self._ptt = JoystickPushToTalk(
                    joystick_index=self.config.ptt.joystick_index,
                    joystick_id=self.config.ptt.joystick_id,
                    joystick_name=self.config.ptt.joystick_name,
                    button=self.config.ptt.joystick_button,
                    on_press=self.begin_transmission,
                    on_release=self.end_transmission,
                    on_lost=self._ptt_controller_lost,
                    on_back=self._ptt_controller_back,
                )
            else:
                self._ptt = PushToTalk(
                    key=self.config.ptt.key,
                    scan_code=self.config.ptt.scan_code,
                    on_press=self.begin_transmission,
                    on_release=self.end_transmission,
                )
            self._ptt.start()
            # The label goes inside a translated sentence, so it is the bare
            # key or button rather than an English phrase around one.
            if mode == "joystick":
                controller = getattr(self._ptt, "controller", None)
                stick = controller.name if controller is not None \
                    else str(self.config.ptt.joystick_index)
                label = f"{stick} button {self.config.ptt.joystick_button}"
                self.emit("info", f"Push to talk: hold {label}.",
                          key="log.ptt_button", stick=stick,
                          button=str(self.config.ptt.joystick_button))
            else:
                label = f"the {self.config.ptt.key} key"
                self.emit("info", f"Push to talk: hold {label}.",
                          key="log.ptt", key_name=self.config.ptt.key)
        except ControllerMissing as exc:
            # The controller it is set to is not plugged in. Said by name,
            # in the pilot's language; it is picked up when it arrives only
            # by setting it again, so the panel's list is where to go.
            self.emit("error", f"Push-to-talk: {exc.wanted} is not connected.",
                      key="log.ptt_controller_missing", name=exc.wanted)
            self._ptt = None
        except ControllersUnavailable as exc:
            if exc.reason == "missing":
                self.emit("error", "Joystick support is missing from this "
                          "installation. Please report it.",
                          key="log.ptt_joystick_missing")
            else:
                self.emit("error", f"Push-to-talk unavailable: {exc.reason}",
                          key="log.ptt_failed", detail=exc.reason)
            self._ptt = None
        except Exception as exc:
            self.emit("error", f"Push-to-talk unavailable: {exc}",
                       key="log.ptt_failed", detail=exc)
            self._ptt = None

