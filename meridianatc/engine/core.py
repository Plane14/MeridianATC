from __future__ import annotations

from ._shared import (
    LOCAL_PROVIDER_NAMES,
    AircraftState,
    AirspaceWatch,
    AnnouncementLibrary,
    Any,
    ArrivalWatch,
    AtisGenerator,
    AutonomousFlight,
    CabinPlayer,
    ClimbWatch,
    Config,
    ControllerBrain,
    CrewDirector,
    CrossingWatch,
    DepartureWatch,
    DeviationAssistWatch,
    DoorTable,
    EventSink,
    FlightSession,
    Future,
    GsxLink,
    Handling,
    IntentParser,
    LandingMonitor,
    Lod,
    ManualSource,
    MicrophoneCapture,
    NavDB,
    OnlineNetwork,
    Phase,
    RadarServiceWatch,
    RadioPlayer,
    Recognizer,
    ReportingPointWatch,
    RestrictionWatch,
    RouteWatch,
    RunwayWatch,
    SidWatch,
    SimAircraft,
    SimDoors,
    SimFacilities,
    SimServicePoints,
    SimSystems,
    SimTraffic,
    SimVehicles,
    StateSource,
    Station,
    SurfaceWatch,
    Synthesizer,
    TaxiWatch,
    TrafficInjector,
    TransponderWatch,
    Understanding,
    Weather,
    WeatherProvider,
    _resolve,
    announcement_roots,
    build_provider,
    create_autonomous_flight,
    deque,
    engine_model,
    gsx_installed,
    imperfections_for,
    load_layout,
    log,
    prosody_for,
    resolve_secret,
    threading,
    time,
    tone_for,
)
from .x_frequency import _Frequency
from .x_session import _Session
from .x_setup import _Setup
from .x_situation_loop import _Situation_loop
from .x_speaking import _Speaking
from .x_state_and_weather import _State_and_weather
from .x_test_tools_traffic import _Test_tools_traffic
from .x_the_language_follows import _The_language_follows
from .x_the_other_aeroplanes import _The_other_aeroplanes
from .x_the_people_who import _The_people_who
from .x_the_plan_the import _The_plan_the
from .x_the_real_aeroplanes import _The_real_aeroplanes
from .x_the_simulator_link import _The_simulator_link
from .x_transmission import _Transmission


class Engine(_Setup, _The_simulator_link, _The_real_aeroplanes, _Session, _The_plan_the, _The_language_follows, _State_and_weather, _Frequency, _Test_tools_traffic, _Transmission, _The_people_who, _Speaking, _Situation_loop, _The_other_aeroplanes):
    """Runs the ATC simulation for one flight."""

    def __init__(
        self,
        config: Config | None = None,
        navdb: NavDB | None = None,
        on_event: EventSink | None = None,
    ):
        self.config = config or Config()
        self.navdb = navdb or NavDB()
        self.on_event = on_event or (lambda event: None)

        self.weather_provider = WeatherProvider(
            refresh_s=self.config.weather.refresh_s,
            online=self.config.weather.online
            and not self.config.network.offline,
        )
        self.synth = Synthesizer(
            voice_dir=_resolve(self.config.voice.voice_dir),
            cache_size=self.config.voice.cache_size,
            seed=self.config.voice.seed,
            engine=self.config.voice.engine,
            prosody_settings=prosody_for(self.config.delivery.prosody),
            imperfection_settings=imperfections_for(
                self.config.delivery.imperfections),
            tone_settings=tone_for(self.config.delivery.tone),
            gpu=self.config.voice.gpu,
            hq_model=self.config.voice.hq_model,
            hq_takes=self.config.voice.hq_takes,
            deepgram_key=resolve_secret(self.config, "speech.deepgram_key", env="DEEPGRAM_API_KEY"),
            deepgram_voice=self.config.speech.deepgram_voice,
        )
        # Said once. Without voices every single transmission fails, and a
        # transcript filling with the same red line is noise rather than
        # information.
        self._voices_reported = False
        # The languages the pilot may speak, which nobody chooses: they follow
        # the aeroplane. English everywhere, plus whatever the state below is
        # worked in, and the set changes as the flight crosses a border. The
        # config can still pin it, which is only useful for keeping the
        # English-only recogniser on a machine that will never leave the US.
        self.pinned_languages = [
            code.lower()[:2] for code in (self.config.speech.languages or [])
        ]
        self.languages = self.pinned_languages or ["en"]
        if self.config.speech.stt_engine == "deepgram" \
                and not self.config.network.offline \
                and self.config.network.cloud_consent:
            from ..integrations.deepgram import DeepgramRecognizer
            self.recognizer = DeepgramRecognizer(
                api_key=resolve_secret(self.config, "speech.deepgram_key", env="DEEPGRAM_API_KEY"),
                model=self.config.speech.deepgram_model,
                stream=self.config.speech.deepgram_stream,
                languages=self.languages,
                fallback=Recognizer(
                    model_size=engine_model(self.config),
                    device=self.config.speech.device,
                    compute_type=self.config.speech.compute_type,
                    beam_size=self.config.speech.beam_size,
                    languages=self.languages,
                    detection_threshold=self.config.speech.detection_threshold),
            )
        else:
            self.recognizer = Recognizer(
                # Where the flight will go is not known at startup, so the
                # model has to be one that can hear anything. Pinning the
                # languages to English is what keeps the smaller
                # English-only model.
                model_size=engine_model(self.config),
                device=self.config.speech.device,
                compute_type=self.config.speech.compute_type,
                beam_size=self.config.speech.beam_size,
                languages=self.languages,
                detection_threshold=self.config.speech.detection_threshold,
            )
        # Never a download from here: the setup screen fetches the model with
        # its own bar. Missing, the recogniser says so (SpeechModelMissing).
        self.recognizer.allow_download = False
        # The cloning tier reads its takes back through this recogniser.
        # Nothing happens here unless that tier is in the cast.
        self.synth.verify_with(self.recognizer)
        self.player = RadioPlayer(
            device=self.config.audio.output_device,
            volume=self.config.audio.volume,
            on_trouble=self._radio_trouble,
        )
        self.microphone = MicrophoneCapture(
            device=self.config.audio.input_device,
            sample_rate=self.config.audio.capture_rate,
            max_seconds=self.config.audio.max_transmission_s,
        )
        # Everybody in this aeroplane who is not a controller: the other
        # seat, and the cabin. On its own audio path, because they are not on
        # the frequency and must not queue behind it.
        self.cabin = CabinPlayer(
            device=self.config.audio.output_device,
            volume=self.config.immersion.volume,
            on_trouble=self._cabin_trouble,
        )
        # Whatever recordings the pilot has put in the announcements folder,
        # in the layout the Fenix packs already use.
        self.announcements = AnnouncementLibrary(announcement_roots(self.config))
        self.crew = CrewDirector(
            self.config, self.synth, self.cabin,
            emit=lambda kind, text, speaker="", **extra: self.emit(
                kind, text, speaker, **extra),
            weather_for=self.weather_for,
            here=self._field_below,
            announcements=self.announcements,
            radio_busy=self.radio_busy,
            # So the cabin knows when the push is over, which is when the
            # cabin manager picks up the handset. Read through a callable
            # rather than handed the object, because the link is built below
            # this and the crew must not hold a reference to something that
            # can be replaced.
            ground=lambda: self.gsx.state,
            # One variable read now, before the first officer says the
            # aeroplane disagrees with a checklist answer. Read only.
            fresh=self._read_now,
        )
        # Where the circuit puts the aeroplane, for the first officer's
        # circuit calls (crew.circuit; the brain is built below).
        self.crew.circuit = self._where_in_the_circuit
        # What the first officer answers questions from, and the radio call
        # it makes when the pilot hands one over (Phase 4).
        self.crew.frequency_for = self._frequency_for
        self.crew.distance_to_field = self._distance_to_field
        self.crew.radio_call = self.first_officer_call
        self.crew.transmit = self.first_officer_transmits

        # Rules first; the AI layer only sees what they could not classify.
        self.parser = IntentParser()
        # A cloud provider named without consent (or in offline mode) is
        # built as "none": the deterministic parser then carries it all.
        ai_provider = self.config.ai.provider
        # Everything but a local provider can reach out (a "custom" endpoint
        # can be anywhere and carries a bearer key), so everything but them
        # wants the first-run consent before a key is sent.
        cloud = ai_provider not in ("", "none") \
            and ai_provider not in LOCAL_PROVIDER_NAMES
        if self.config.network.offline \
                or (cloud and not self.config.network.cloud_consent):
            ai_provider = "none"
        self.understanding = Understanding(
            self.parser,
            build_provider(
                ai_provider,
                model=self.config.ai.model,
                host=self.config.ai.host,
                api_key=resolve_secret(self.config, "ai.api_key",
                                       env="ANTHROPIC_API_KEY"),
                timeout_s=self.config.ai.timeout_s,
            ),
            min_confidence=self.config.ai.min_confidence,
        )
        # Opt-in natural rephrasing of approved transmissions. The provider
        # is the understanding one -- there is no second model -- and the
        # renderer is only built when the pilot asked for it and the
        # provider speaks.
        self.phrasing = None
        # Consent gates cloud reach-out; a local phrasing is a local call
        # and does not need it.
        provider = self.understanding.provider
        is_cloud = provider is not None \
            and getattr(provider, "name", "") not in LOCAL_PROVIDER_NAMES
        if self.config.ai.phrasing and not self.config.network.offline \
                and (not is_cloud or self.config.network.cloud_consent) \
                and provider is not None:
            from ..llm.rendering import PhrasingRenderer
            self.phrasing = PhrasingRenderer(self.understanding.provider)
        # Opt-in model advisory (llm/decisions.py): the model may pick among
        # options the engine enumerated and word replies whose facts are
        # already fixed. It cannot issue or change an instruction, and the
        # deterministic answer stands on any failure -- the same provider and
        # consent rules as phrasing.
        self.decisions = None
        if self.config.ai.decisions and provider is not None \
                and not self.config.network.offline \
                and (not is_cloud or self.config.network.cloud_consent):
            from ..llm.decisions import Decisions
            self.decisions = Decisions(
                provider, shadow=self.config.ai.decisions_shadow)
        self.crew.advisor = self.decisions
        self.brain = ControllerBrain(
            self.navdb, self._base_phraseology(),
            strict_icao_digits=self.config.atc.strict_icao_digits,
            ground_control=self.config.atc.ground_control,
        )
        self.brain.realism = self.config.immersion.controller
        self.brain.hearing = self.config.speech
        # Autonomous flight (First Officer flying the aircraft). Built after
        # the brain it talks to; the session arrives with begin_flight.
        # Always built: the panel's engage button needs the machine to
        # exist before it can turn it on, and the object is inert while
        # its enabled flag (the setting, default off) is clear.
        self.autonomous_flight: AutonomousFlight | None = (
            create_autonomous_flight(
                self.config, self.navdb, None, None, self.crew, self.brain,
                lambda: self.state, self.emit, use_route=self.use_route
            )
        )
        # When the search for an enroute frequency last came up empty, so a
        # flight over a country that publishes none is not made to look for
        # one on every pass of the situation loop.
        self._centre_looked_at = 0.0
        # When a centre last looked at which sector the flight is in.
        self._centre_checked_at = 0.0
        # The sector the flight was last seen in, for the row written as it
        # enters the next; and the SimBrief route as points, per plan.
        self._sector_noted = None
        self._route_cache = None
        # The half of the controller that watches rather than listens: what
        # the aeroplane is doing on the ground, and what it is doing on the
        # way in. Both speak first; nothing in either waits to be asked.
        self.watch = SurfaceWatch(self.navdb, self.brain)
        self.crossing_watch = CrossingWatch(self.navdb, self.brain)
        # And the code the box shows against the one the flight was given.
        self.transponder_watch = TransponderWatch(self.navdb, self.brain)
        self.taxi_watch = TaxiWatch(self.navdb, self.brain)
        self.deviation_watch = DeviationAssistWatch(self.navdb, self.brain)
        self.airspace_watch = AirspaceWatch(self.navdb, self.brain)
        self.point_watch = ReportingPointWatch(self.navdb, self.brain)
        # And the radar half of it for a VFR flight: where its service ends,
        # and whether it is keeping under the altitude it was held at.
        self.radar_service_watch = RadarServiceWatch(self.navdb, self.brain)
        self.restriction_watch = RestrictionWatch(self.navdb, self.brain)
        # Through lambdas, so a replaced weather_for or atis_for is the one
        # asked.
        self.runway_watch = RunwayWatch(
            self.navdb, self.brain, lambda ident: self.weather_for(ident),
            lambda ident, language: self.atis_for(ident, language))
        self.arrivals = ArrivalWatch(self.navdb, self.brain)
        # The controller answers some requests with what the arrival watch
        # would say unprompted: a hold, the first descent (Phase 6).
        self.brain.arrivals = self.arrivals
        self.departures = DepartureWatch(self.navdb, self.brain)
        # The IFR climb in steps and the SID watched (the IFR plan, Phase 1).
        self.climbs = ClimbWatch(self.brain)
        self.sid_watch = SidWatch(self.brain)
        self.route_watch = RouteWatch(self.brain)
        # What the ground handling is doing, if anything is saying -- and
        # the tug, once a push has been approved. The tug is handed over as a
        # callable rather than as the source, so nothing in integrations has
        # to know what a simulator link is.
        self.gsx = GsxLink(enabled=self.config.gsx.enabled,
                           control=self.config.gsx.control,
                           tug=self._sim_pushback)
        self.brain.ground = self.gsx

        # The other aeroplanes on the frequency.
        self.traffic = TrafficInjector(
            self.navdb,
            seed=self.config.traffic.seed or None,
            density=self.config.traffic.density
            if self.config.traffic.enabled else 0.0,
            callsigns=self.config.traffic.callsigns,
            radius_nm=self.config.traffic.radius_nm,
        )
        # Nothing invented exists until the simulator has drawn it. A pilot
        # told there is an aeroplane on the runway looks for it, and the
        # traffic used to be on the radio and the map whether the simulator
        # had made it or not -- which, in 2024, it never had.
        self.traffic.requires_sim = True
        self.traffic.energy_model = getattr(
            self.config.traffic, "energy_model", True)
        self.traffic.expedite = getattr(
            self.config.traffic, "expedite", True)
        self.traffic.diurnal = getattr(
            self.config.traffic, "diurnal_rhythm", True)
        # The same advisor words a traffic crew's answers; PilotsTalking
        # treats it as optional.
        self.traffic.advisor = self.decisions
        self.traffic.max_invented = self.config.traffic.in_simulator_limit
        # The invented aeroplanes' own switch (traffic.invented); the
        # situation loop re-reads it every tick so a mid-flight toggle
        # drops what was flying and makes no more.
        self.traffic.invented = bool(
            getattr(self.config.traffic, "invented", False))
        # And the ones standing at the gates, on a budget of their own.
        self.traffic.parked = max(0, int(self.config.traffic.parked))
        # The aeroplanes the pilot put on the runway, waiting to be said once
        # the simulator has actually made them: callsign -> runway.
        self._spawned: dict[str, str] = {}
        # The controller sequences the user against the same traffic the user
        # can hear, rather than against nothing.
        if self.config.traffic.enabled:
            self.brain.traffic = self.traffic
        # And the traffic flies the same procedures the controller and the
        # First Officer are given: the dicts are populated asynchronously as
        # the provider reads them, so sharing the references is the wiring.
        self.traffic.terminal_procedures = self.brain.terminal_procedures
        self.traffic.approach_data = self.brain.approach_data
        # The same aeroplanes, out of the window. Without a simulator this
        # stays dormant and the radio is unchanged.
        # The simulator's own frequency list, which beats the published one.
        self.facilities = SimFacilities(self.config.sim.simconnect_dll)
        # Airports already read, so each is asked for once per session.
        self._facilities_done: set[str] = set()
        # And their stands and taxiways, for the traffic to drive on rather
        # than be placed on. Read in the background; handed over when ready.
        from ..integrations.taxiways import SimGround

        self.simground = SimGround(self.config.sim.simconnect_dll)
        # The same answer read out of X-Plane's apt.dat when the session
        # is with that simulator instead.
        from ..integrations.xpground import XPlaneGround

        self.xpground = XPlaneGround()
        self._ground_asked: set[str] = set()
        # Fields whose taxiway read came back empty or failed -- dead for
        # the session, so "still coming" and "never came" tell apart.
        self._ground_dead: set[str] = set()
        # And the approaches each airport publishes, so the ILS is only
        # promised where there is one. Asked once per airport, on a worker,
        # and handed to the brain as the answers come in.
        from ..integrations.approaches import SimApproaches

        self.simapproaches = SimApproaches(self.config.sim.simconnect_dll)
        # And the SIDs and STARs each airport publishes, for the First
        # Officer's procedure guidance (integrations.terminal_procedures).
        from ..integrations.terminal_procedures import SimProcedures

        self.simprocedures = SimProcedures(self.config.sim.simconnect_dll)
        # And each runway's pattern altitude, behind the circuit files.
        from ..integrations.runways import SimRunways

        self.simrunways = SimRunways(self.config.sim.simconnect_dll)
        # Where each field's runways have been said to come from (logs radio).
        self._runways_said: dict[str, str] = {}
        self._approaches_asked: set[str] = set()
        self._procedures_asked: set[str] = set()
        # The X-Plane CIFP reader, built by _cifp_source once the
        # install root is known. None means not tried yet.
        self._cifp = None
        self._ground_ready: dict[str, object] = {}
        # The controller taxis the pilot on the same layouts the traffic
        # drives on, and reaches them the way it reaches the traffic: handed
        # a service, so nothing in atc/ imports the engine.
        self.brain.ground_layout = self.ground_layout
        # And whether a field's read is still in flight, so a taxi asked
        # for in the gap is told to stand by rather than given the
        # invented route it would then keep for ever.
        self.brain.ground_pending = self.ground_pending
        # And the ATIS each field is broadcasting, so a call reporting an old
        # letter is told the current one.
        self.brain.atis = self.atis_for
        self.traffic.atis = self.atis_for
        self.brain.weather_for = self.weather_for
        # And everything the simulator reports, the pilot's own excluded,
        # for the controller's picture (ControllerBrain.contacts).
        self.brain.sim_traffic = self._sim_contacts
        from ..atc.trafficinfo import TrafficWatch

        self.traffic_watch = TrafficWatch(self.brain, advisor=self.decisions)
        self.simtraffic = SimTraffic(
            enabled=self.config.traffic.enabled
            and self.config.traffic.in_simulator,
            # The parked aeroplanes come after the moving ones in what it is
            # given, so this is room for them rather than a share of it.
            limit=self.config.traffic.in_simulator_limit
            + max(0, int(self.config.traffic.parked)),
            # The real aeroplanes' budget is their own setting, not a share
            # of the invented ones'.
            live_limit=self.config.traffic.live.limit,
            settle_s=self.config.traffic.lod.settle_s,
            smooth=self.config.traffic.smooth_motion,
        )
        self.simtraffic.lod_models = dict(self.config.traffic.lod.models or {})
        # Which model each parked aeroplane is drawn with, by how far it is
        # from the pilot: the simulator's own, with doors, nearby; FSLTL's
        # painted one further out. The doors follow the aeroplane across.
        lod = self.config.traffic.lod
        self.lod = Lod(enabled=lod.enabled,
                       detailed_m=lod.detailed_m,
                       light_m=lod.light_m,
                       limit=lod.limit,
                       swaps_per_s=lod.swaps_per_s,
                       mode=str(lod.mode).strip().lower(),
                       radius_m=float(lod.detail_radius_km) * 1000.0,
                       hide_m=lod.hide_m,
                       livery=str(lod.livery).strip().lower())
        self._ai_liveries_asked = False
        # The schedule changes the traffic's day as well as the models:
        # departures board before they call, arrivals hold their stand.
        self.traffic.schedule = self.lod.scheduled
        # The test tools (traffic.debug), all off by default. time_scale
        # shortens the turnaround and the schedule's booking and boarding.
        debug = self.config.traffic.debug
        self.time_scale = max(0.01, float(debug.time_scale or 1.0))
        self.traffic.boarding_s = max(0.0, float(lod.boarding_s)) / self.time_scale
        self.traffic.boarding_lead_s = (max(0.0, float(lod.boarding_lead_s))
                                        / self.time_scale)
        self._status_at = 0.0
        self._motion_status_at = 0.0
        self.simtraffic.on_swapped = self._model_swapped
        self.simtraffic.on_removed = self._model_removed
        # A departure asks for taxi only once its doors are read shut.
        self.traffic.can_push = self._may_push
        # A test setting: every invented aeroplane is one of these types.
        # traffic.debug.force_types, or the older traffic.debug_force_types.
        self.traffic.force_types = tuple(
            str(code).strip().upper() for code in
            (debug.force_types or self.config.traffic.debug_force_types or [])
            if str(code).strip())
        if self.traffic.force_types:
            log.info("traffic.debug: every invented aeroplane is one of %s",
                     ", ".join(self.traffic.force_types))
        # And the aeroplanes somebody else put there, which is the other half
        # of the same question: what is actually on this airport.
        # Read whether or not this program draws aeroplanes of its own: a
        # pilot who has switched the invented traffic off still has FSLTL
        # running, and the controller has to see it.
        self.simwatch = SimAircraft(
            enabled=self.config.traffic.from_simulator,
            radius_m=int(self.config.traffic.radius_nm * 1852.0))
        # And the ramp around the aeroplanes this program parks: who calls
        # on them and when is decided here, and drawn by the simulator.
        # Passengers and their buses are GSX's people and GSX's bus: without
        # GSX Pro there is nobody to walk, and an aeroplane's stairs held
        # for a bus that never comes would only keep its tug waiting.
        handling_config = self.config.traffic.handling
        gsx = gsx_installed()
        visible = (max(0, int(handling_config.passengers_visible))
                   if handling_config.passengers and gsx else 0)
        if handling_config.passengers and not gsx:
            log.info("traffic.handling.passengers is on, but GSX Pro is not "
                     "installed: there is nobody to walk, so no buses")
        self.handling = Handling(seed=self.config.traffic.seed or None,
                                 limit=self.config.traffic.handling_limit,
                                 schedule=self.lod.scheduled,
                                 time_scale=self.time_scale,
                                 passengers=visible,
                                 baggage=bool(handling_config.baggage))
        self._handling_raised = False
        self.simvehicles = SimVehicles(
            enabled=self.config.traffic.enabled
            and self.config.traffic.in_simulator
            and self.config.traffic.handling.enabled
            and self.config.traffic.handling.vehicles,
            # The turnaround keeps the budget; this is the ceiling under
            # it, with room for the tugs the turnaround lets through over
            # the budget (see Handling._admit).
            limit=self.config.traffic.handling_limit + 4,
            dll_path=self.config.sim.simconnect_dll,
            prefer_gsx=self.config.traffic.handling.gsx_vehicles
            and gsx,
            # Two buses and their passengers, with room for the ones still
            # walking off as the next appear.
            people=visible > 0 or bool(handling_config.baggage),
            # Two buses and their passengers, two trains (a tractor, two
            # carts and at most two bags on a belt), and room for the ones
            # still going as the next appear.
            people_limit=Handling.BUSES * (visible + 1) + Handling.TRAINS * 5 + 6,
        )
        if self.simvehicles.prefer_gsx:
            log.info("GSX Pro is installed: its vehicles serve this "
                     "program's traffic where the simulator lists them")
        # And the aeroplanes' own doors, opened with the stairs and the
        # loaders and read shut before anything pushes.
        self.simdoors = SimDoors(
            enabled=self.config.traffic.enabled
            and self.config.traffic.in_simulator
            and self.config.traffic.handling_doors,
            table=DoorTable.load(self.config.traffic.doors_table),
            force_after_s=self.config.traffic.doors_force_after_s,
        )
        # And their engines and lights (meridianatc.atc.systems decides,
        # this makes the simulator agree).
        # And where each model's doors are, which is where the vehicles go.
        self.servicepoints = SimServicePoints(
            enabled=self.config.traffic.enabled
            and self.config.traffic.in_simulator
            and self.config.traffic.handling.enabled)
        self._service_layout = load_layout()
        self._stairs_said: set[tuple[str, str, str]] = set()
        self.simsystems = SimSystems(
            enabled=self.config.traffic.enabled
            and self.config.traffic.in_simulator
            and self.config.traffic.engines_and_lights)

        self.source: StateSource = ManualSource()
        self.state = AircraftState()
        self.session: FlightSession | None = None
        # The last SimBrief plan loaded, if any.
        self.plan = None
        # The filed route as a line on the chart (atc/route.py): from the
        # SimBrief plan, or a flight plan file. None is a flight with no plan,
        # worked by distance to the airport as before.
        self.route = None
        # The leg the aeroplane was last found on, and where, for the panel.
        self.route_progress = None
        # How the landing went, and the flight against the plan's schedule.
        # Read through a callable so a plan loaded mid-flight still counts.
        self.landing = LandingMonitor(plan=lambda: self.plan)
        self.station: Station | None = None

        self._atis: dict[tuple[str, str], AtisGenerator] = {}
        # Fields whose ATIS must be re-issued on the next ask, set from the
        # approaches worker.
        self._atis_stale: set[str] = set()
        self._atis_lock = threading.Lock()
        self._weather_cache: dict[str, Weather] = {}
        self._ptt = None
        self._stop = threading.Event()
        self._situation_thread: threading.Thread | None = None
        self._busy = threading.Lock()
        # One pilot call at a time, spoken or typed. The spoken pipeline's
        # lock covers the transcription worker only; a typed call off the
        # web thread or a first officer's transmission on the situation
        # thread used to reach handle_text and retune mid-response, where
        # both callers then ate the same pending readback.
        self._text_lock = threading.RLock()
        # Who is really on the frequency, where a network is configured. The
        # one thing this program must never do is talk over a person.
        # Offline mode: nothing that can reach out is registered -- the poll
        # is configured as "none" rather than merely unpolled, so there is no
        # object that could be asked.
        self.online = OnlineNetwork(
            "none" if config.network.offline else config.network.provider,
            config.network.refresh_s)
        self.human: Any = None
        self._last_com1 = 0.0
        # COM2, the radio that listens: the station it is resolved to, and
        # the frequency last resolved, kept apart from ``self.station`` so a
        # listen-only box never moves the talk side.
        self.station2 = None
        self._last_com2 = 0.0
        # Which radio the pilot is talking on (the plugin's transmit_com1/2
        # commands move it), who the next transmission is from when the
        # plugin keys the copilot's mic, and the last error for copy_error.
        self._radio_talk = 1
        self._talk_speaker = ""
        self._last_error = ""
        # The ATIS COM2 last played, as "ident:letter", so it is not
        # re-read every tick.
        self._atis2_played = ""
        self._reported_empty = 0.0
        # (frequency, since when) of an empty COM1 not yet explained.
        self._empty_pending: tuple[float, float] | None = None
        self._reported_assumed = 0.0
        # The simulator link, watched rather than assumed. Connecting once at
        # startup is not enough: the panel is usually opened before the sim
        # has finished loading, and a sim that is shut down mid-session used
        # to leave the aeroplane frozen wherever it last was, with the panel
        # cheerfully reporting a position that had stopped being true.
        self._sim_wanted = bool(self.config.sim.enabled)
        self._next_sim_try = 0.0
        self._sim_was_connected = False
        # Whether "Simulator not detected" was already said this session --
        # it is only true once a connect attempt has actually failed.
        self._sim_waiting_said = False
        # The live simulator link, when there is one. Held by name rather
        # than recognised by type: the question is whether *this* engine has
        # a simulator source in use, and a type check answers a subtly
        # different one that goes wrong the moment anything wraps the class.
        self._sim_source: StateSource | None = None
        self._sim_attempt: Future | None = None
        self._sim_lock = threading.RLock()
        self._shutdown_lock = threading.Lock()
        self._shutdown_done = False
        # Whether the last tick found the simulator paused, so it is said
        # once rather than four times a second. The second flag records
        # whether the pause line itself was said -- a relocation hold
        # skips the message, so it gets no "back" either.
        self._was_paused = False
        self._paused_said = False
        # A stall is tracked apart from _was_paused so one that starts
        # while the simulator is paused still gets its "back" handling.
        self._was_stalled = False
        # The language each frequency is being worked in (_frequency_language).
        self._languages_by_frequency: dict[str, str] = {}
        # When a controller last spoke, so a bare "say again" after them is
        # theirs and not the first officer's (answer_the_aeroplane).
        self._atc_said_at = 0.0
        # Radio renders the lane is carrying: the cabin reads this through
        # radio_busy so nobody starts a cabin line over a transmission that
        # is on its way to the speaker but not on it yet.
        self._radio_renders = 0
        # The debounced on-ground flag (see _debounced_ground).
        self._ground_seen: bool | None = None
        self._ground_disagrees = 0
        # A frequency this program put into the radio itself. The situation
        # loop reads COM1 back a moment later and must not treat its own
        # write as the pilot having reached for the radio.
        self._tuned_by_us = 0.0
        # The simulator's own loaded flight plan, polled for (the
        # FlightPlanActivated event does not reach this wrapper, and on
        # 2024 it is not to be believed when it does). A request is in the
        # air until its reply is taken; the file's path+mtime dedupe the
        # loads, so an unchanged plan is never reparsed.
        self._sim_plan_requested = False
        self._sim_plan_at = 0.0
        self._sim_plan_seen = ""
        self._sim_plan_path = ""
        self._sim_plan_mtime = 0.0
        # The X-Plane equivalent of path+mtime: the FMS entries' own
        # fingerprint, so an unchanged plan is never rebuilt.
        self._sim_plan_fp = None
        # The language of the last exchange, so an ATIS or a volunteered
        # handoff continues in whatever the pilot has been speaking.
        self.last_language = self.languages[0]
        # When each field's weather was last looked up. Per field: one shared
        # clock meant any field asked about every minute kept every other
        # field's first reading for the rest of the flight.
        self._weather_checked: dict[str, float] = {}
        self.started_at = time.monotonic()
        self._last_region_check = 0.0
        # Everything that has been said on the radio this session, newest
        # last. Short: it is what a panel shows, not an archive, and the log
        # files are where a transmission goes to be kept.
        self.transcript: deque = deque(maxlen=200)
        # The durable transcript (StormATC's session history, ported):
        # SQLite in the data root, opened lazily by the flight that begins.
        self.history = None
        try:
            from ..history import History
            from ..paths import data_root
            self.history = History(data_root() / "history.db")
        except Exception:
            log.debug("history unavailable", exc_info=True)
        # StormATC's voice trainer and mission builder, on the same root.
        from ..atc.continuous import ContinuousFlights
        from ..atc.trainer import VoiceTrainer
        self.trainer = VoiceTrainer(data_root() / "trainer")
        self.continuous = ContinuousFlights(self.navdb)
        # The corrections the trainer taught are applied to every heard
        # transcript before the parser sees it, and its keyterms ride on the
        # recogniser's context.
        recognizer = getattr(self, "recognizer", None)
        if recognizer is not None and hasattr(recognizer, "user_keyterms"):
            recognizer.user_keyterms = self.trainer.keyterms
        # The traffic goes quiet for a moment after anybody else transmits, so
        # two aeroplanes are never heard talking over each other.
        self._frequency_clear_at = 0.0
        # An answer decided while the frequency was busy with somebody
        # else's exchange, said once it is over: (reply, station, language,
        # when it was held).
        self._held_reply = None
        # What the other aeroplanes have said but has not reached the speakers
        # yet, in the order they said it.
        self._traffic_queue: list = []
        # What has been decided about each AI exchange: whether any of it has
        # been put on the air, and when that was decided. Its halves arrive
        # here one at a time, so the decision has to outlive the transmission
        # it was made about.
        self._exchanges: dict[int, tuple[bool, float]] = {}
        # How many aeroplanes of the simulator's own the pilot has been told
        # about, so the line is news rather than a running commentary.
        self._observed_said = 0

    def _watch_landing(self, previous, state, session) -> None:
        """Hand the landing report to whoever is watching, once it exists."""
        try:
            written = self.landing.follow(previous, state)
        except Exception:
            log.debug("the landing monitor raised", exc_info=True)
            return
        if not written:
            return
        report = self.landing.report
        # Where it actually landed, which is the filed destination only when
        # nothing went wrong.
        here = self._field_below()
        report.airport = (here.ident if here is not None else
                          session.destination.ident if session.destination
                          else "")
        log.info("landing: %s", report.describe())
        self.emit("info", f"Landing: {report.describe()}", key="log.landing",
                  fpm=int(round(report.fpm)), score=report.score)

    def _frequency_for(self, name: str):
        """(callsign, MHz) of the station whose name the pilot asked about,
        within reach, or None."""
        from ..atc.interphone import fold

        state = self.state
        if not state.has_position or not name:
            return None
        wanted = fold(name).split()
        best = None
        for station in self.navdb.all_stations_near(
                state.latitude, state.longitude, 120.0):
            if station.position == "ATIS":
                continue
            called = fold(f"{station.callsign} {station.facility} "
                          f"{station.description}")
            hits = sum(1 for w in wanted if w in called.split())
            if hits and (best is None or hits > best[0]
                         or (hits == best[0]
                             and station.distance_nm < best[1].distance_nm)):
                best = (hits, station)
        if best is None:
            return None
        return best[1].callsign, best[1].mhz

    def _distance_to_field(self):
        """(name, nautical miles) to where the flight is going, or None."""
        session, state = self.session, self.state
        if session is None or not state.has_position:
            return None
        airport = session.destination or self._field_below()
        if airport is None:
            return None
        name = getattr(airport, "name", "") or airport.ident
        return (name.split(" ")[0].title() if name else airport.ident,
                state.distance_to(airport.lat, airport.lon))

    def first_officer_call(self, kind: str) -> tuple[str, str]:
        """The one call the pilot handed over: its words, or "" and why not.

        Built with the same PilotSpeech the AI readback uses, so the
        controller reads it as it reads the pilot's. A readback is made only
        when it is what was handed over: a clearance addressed to the pilot
        is theirs to read back.
        """
        from ..atc.copilot import readback as build_readback
        from ..atc.pilot import PilotSpeech

        session = self.session
        station = self.station or self._current_station()
        if session is None or station is None:
            return "", "no station"
        spoken = self._language_for(station.ident, self.last_language)
        p = self.brain.for_station(station, spoken)
        speech = PilotSpeech(spoken, p)
        aircraft = session.aircraft
        name = station.callsign_in(spoken)
        pending = session.pending
        if kind == "readback":
            if pending is None or pending.read_back:
                return "", "nothing to read back"
            handoff = session.handoff_pending
            return build_readback(
                pending, aircraft, p, spoken,
                station_name=(handoff.callsign_short if handoff is not None
                              else "")), ""
        if pending is not None and not pending.read_back:
            return "", "readback"
        phase = session.phase
        if kind == "check_in":
            vs = getattr(self.state, "vertical_speed_fpm", 0.0) or 0.0
            # The level reported is the level the aeroplane is AT, the
            # one the radar screen shows -- not the level the clearance
            # names. Reporting the assigned nine thousand while still
            # through one thousand had the controller verifying an
            # altitude that was never claimed.
            alt = getattr(self.state, "indicated_altitude_ft", 0.0) \
                or getattr(self.state, "altitude_ft", 0.0) or 0.0
            if abs(vs) < 300.0:
                return speech.maintaining(aircraft, name, alt), ""
            return speech.checking_in(aircraft, name, alt,
                                      climbing=vs > 0), ""
        if kind == "going_around":
            return speech.going_around(
                aircraft, session.assigned_altitude_ft or 0.0), ""
        if kind == "any":
            if phase in (Phase.PREFLIGHT, Phase.CLEARED) and session.ifr \
                    and not session.clearance_issued:
                kind = "clearance"
            elif phase in (Phase.PREFLIGHT, Phase.CLEARED, Phase.PUSHBACK,
                           Phase.TAXI_OUT):
                kind = "taxi"
            elif phase in (Phase.HOLDING_SHORT, Phase.LINED_UP):
                kind = "ready"
            else:
                return "", "which"
        if kind == "clearance":
            destination = session.destination
            return speech.request_clearance(
                aircraft, name, getattr(destination, "name", "")
                or getattr(destination, "ident", "")), ""
        if kind == "vfr":
            return speech.request_vfr_departure(aircraft, name), ""
        if kind == "taxi":
            if station.position not in ("GND", "INFO", "CTAF", "UNICOM",
                                        "RDO", "AFIS"):
                # A taxi asked of the tower is refused only where a ground
                # position is actually worked: a printed frequency nobody
                # is on is nobody to send the pilot back to, and refusing
                # left the aeroplane on the ramp until the watchdog bit.
                has_ground = any(
                    s.position == "GND" and self.brain.staffed(s)
                    for s in self.navdb.stations(station.ident))
                if has_ground:
                    return "", "not_on:ground"
            return speech.request_taxi(aircraft, name), ""
        if kind == "pushback":
            if station.position not in ("DEL", "GND", "RMP", "INFO",
                                        "CTAF", "UNICOM", "RDO", "AFIS"):
                # The tug is not the tower's to send: a push asked of a
                # position that cannot give one is refused wherever a
                # surface position is actually worked -- and asked of
                # the tower only where the tower works the surface too.
                has_ground = any(
                    s.position in ("DEL", "GND", "RMP")
                    and self.brain.staffed(s)
                    for s in self.navdb.stations(station.ident))
                if has_ground:
                    return "", "not_on:ground"
            return speech.request_pushback(aircraft, name), ""
        if kind == "ready":
            return speech.ready_for_departure(
                aircraft, session.departure_runway or ""), ""
        if kind in ("downwind", "base", "final"):
            return speech.circuit_report(aircraft, p.leg(kind)), ""
        if kind == "vacated":
            return speech.runway_vacated(aircraft), ""
        return "", "which"

    def first_officer_transmits(self, text: str) -> None:
        """The first officer's call on the frequency: shown as theirs,
        keyed in their voice, and through the ordinary path so the
        controller checks it."""
        station = self.station or self._current_station()
        spoken = self._language_for(station.ident if station else "",
                                    self.last_language)
        self.emit("pilot", text, "First officer")
        try:
            # First: the render lands ahead of the answer handle_text is
            # about to ask for, so the frequency hears the call before the
            # reply to it.
            self._speak_first_officer(text, spoken)
        except Exception:
            log.debug("the first officer's voice failed", exc_info=True)
        try:
            self.handle_text(text, spoken, speaker="first_officer")
        except Exception:
            log.debug("the first officer's call failed", exc_info=True)

    def _where_in_the_circuit(self):
        """Where the circuit puts the aeroplane, for the first officer's
        "downwind", "base", "final, runway 24, cleared": the controller's own
        circuit for the field and runway, so the two never disagree. None on
        the ground, or with nothing to place it by."""
        session, state = self.session, self.state
        if session is None or state is None or state.on_ground \
                or not state.has_position:
            return None
        runway = session.arrival_runway or session.departure_runway
        airport = session.destination or self._field_below()
        if not runway or airport is None:
            return None
        try:
            circuit = self.brain._circuit_at(session, airport, runway)
            return self.brain._where_in(circuit, state)
        except Exception:
            log.debug("could not place the aeroplane in the circuit",
                      exc_info=True)
            return None

    def _field_below(self):
        """The airport the aeroplane is at, if it is at one.

        The cabin announcements use it to fill in a destination nobody
        filed, which is most flights: "welcome to Nantes" beats "welcome to
        our destination" and is no less true.
        """
        state = self.state
        if not state.has_position:
            return None
        return self.navdb.home_airport(state.latitude, state.longitude)

