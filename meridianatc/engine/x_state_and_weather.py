from __future__ import annotations

from ._shared import (
    AircraftState,
    AtisGenerator,
    AtisReport,
    FlightProfile,
    Phase,
    Weather,
    WeatherSample,
    XPlaneTraffic,
    at_the_surface,
    decision,
    in_the_circuit,
    log,
    merge_weather,
    near_the_field,
    parse_metar,
    replace,
    reply_language,
    time,
    variation,
    weather_from_sim,
)


class _State_and_weather:
    # ------------------------------------------------------------------
    # state and weather
    # ------------------------------------------------------------------

    def refresh_state(self) -> AircraftState:
        try:
            self.state = self.source.read()
        except Exception as exc:
            if not getattr(self, "_state_read_failed", False):
                log.warning("state read failed; flying on the last "
                            "numbers: %s", exc, exc_info=True)
            self._state_read_failed = True
        else:
            if getattr(self, "_state_read_failed", False):
                log.info("state reads recovered")
            self._state_read_failed = False
        return self.state

    def weather_for(self, airport_ident: str) -> Weather:
        """Current weather at a field.

        The simulator reports what is around the aeroplane, so it speaks for
        a field only when the aeroplane is at it or about to touch it (see
        :func:`~meridianatc.weather.metar.near_the_field`). Everywhere else the
        field's own observation, then the nearest one, then calm and standard
        with the simulator's sea-level pressure. Laying the ambient values
        over every field read the jet stream out as a destination's surface
        wind and rolled the ATIS letter every thousand feet of descent.

        Never waits for the network: a fetch that is due is started on a
        worker and a later look picks it up.
        """
        cached = self._weather_cache.get(airport_ident)
        now = time.monotonic()
        checked = self._weather_checked.get(airport_ident)
        if cached is not None and checked is not None and (now - checked) < 60.0:
            return cached

        airport = self.navdb.airport(airport_ident)
        station = self.config.weather.station_override or (
            airport.icao if airport and airport.icao else airport_ident
        )
        sample = self.weather_provider.metar(station, wait=False)
        own = self._plugin_metar(station)
        if own is not None:
            sample = own
        if sample.source == "default" and airport is not None:
            sample = self.weather_provider.nearest_metar(
                airport.lat, airport.lon, self.navdb, wait=False)

        sim_weather = (weather_from_sim(self.state)
                       if self.config.weather.prefer_sim else None)
        # A field with no report of its own: in its circuit the simulator's
        # wind is the only one that is this field's, so it is used in the air
        # too (in_the_circuit); beyond it, the nearest observation.
        own_report = (airport is not None and sample.source in ("metar", "cache")
                      and getattr(sample, "station", "")
                      == (airport.icao or airport.ident))
        circuit = (not own_report and sim_weather is not None
                   and in_the_circuit(self.state, airport))
        if sim_weather is not None and (near_the_field(self.state, airport)
                                        or circuit):
            sample = merge_weather(sim_weather, sample,
                                   surface=bool(self.state.on_ground),
                                   wind=(at_the_surface(self.state, airport)
                                         or circuit))
        elif sample.source == "default" and sim_weather is not None:
            # Nothing observed anywhere near. Sea-level pressure is a regional
            # figure and the simulator's is as good as any; its wind and
            # temperature are the aeroplane's and are not used.
            sample = replace(sample, weather=replace(
                sample.weather, qnh_hpa=sim_weather.qnh_hpa,
                altimeter_inhg=sim_weather.altimeter_inhg))

        # A copy, always: the provider caches the object it returned, and the
        # nearest-observation fallback hands the same one to every field near
        # it -- writing this field's variation into it gave them all the last
        # field's.
        weather = replace(sample.weather,
                                      clouds=list(sample.weather.clouds))
        if not weather.wind_from:
            weather.wind_from = dict.fromkeys(
                ("direction", "speed", "gust"),
                f"{sample.source} {getattr(sample, 'station', '') or ''}".strip())
        if airport is not None:
            # Spoken winds are magnetic; the observation is true.
            weather.variation = variation(airport.lat, airport.lon)
        station_name = getattr(sample, "station", "") or ""
        weather.source = {
            "metar": f"metar {station_name}", "cache": f"cached {station_name}",
            "sim": f"sim {station_name}".strip(), "default": "default",
        }.get(sample.source, sample.source)
        if sample.source == "sim":
            weather.source += (" (sim surface wind)"
                               if at_the_surface(self.state, airport)
                               else " (observed wind)")
        if (airport is not None and station_name
                and station_name != (airport.icao or airport.ident)
                and sample.source in ("metar", "cache")):
            weather.source = f"nearest {station_name}"
        self._weather_cache[airport_ident] = weather
        # A miss is looked at again in ten seconds, because the fetch it
        # started will have landed by then; a hit is good for the minute.
        self._weather_checked[airport_ident] = (
            now if sample.source != "default" else now - 50.0)
        return weather

    def _plugin_metar(self, station: str) -> WeatherSample | None:
        """The METAR the simulator itself downloaded, through the plugin.

        X-Plane's downloaded report is the weather it is actually drawing,
        so for the ATIS it outranks a network fetch of the same station.
        None where the plugin is not connected or the sim is not in
        real-weather mode -- ``XPLMGetMETARForAirport`` answers empty then.
        """
        xp = getattr(self, "simtraffic", None)
        if not isinstance(xp, XPlaneTraffic):
            return None
        raw = xp.metar(station)
        if not raw:
            return None
        return WeatherSample(parse_metar(raw), "metar", station)

    def atis_for(self, airport_ident: str,
                 language: str | None = None) -> AtisReport | None:
        """The current ATIS, in the requested language.

        A field is only broadcast in a language it actually works in, so asking
        for French at Kennedy quietly gives English rather than inventing a
        French ATIS that no such airport transmits.
        """
        airport = self.navdb.airport(airport_ident)
        if airport is None:
            return None

        # A controller on the network broadcasting their own ATIS is the
        # authority on this field: it names the runway they are actually
        # using and the letter they are actually expecting to hear back.
        # Generating one over the top would put two truths on one aerodrome.
        online = self._online_atis(airport_ident)
        if online is not None:
            return online

        language = self._language_for(airport_ident, language)
        # The pinned region is the broadcast's too: a pilot who pinned FAA
        # wording does not get ICAO from the ATIS.
        region = FlightProfile.of(self.config.flight).region
        key = (airport_ident, language, region)
        with self._atis_lock:
            if airport_ident in self._atis_stale:
                self._atis_stale.discard(airport_ident)
                for atis_key, stale in self._atis.items():
                    if atis_key[0] == airport_ident:
                        stale.current = None
        generator = self._atis.get(key)
        if generator is None:
            from ..atc.language import phraseology_for

            generator = AtisGenerator(
                self.navdb, airport,
                phraseology_for(language, airport_ident,
                                self.config.atc.strict_icao_digits,
                                region=region),
            )
            # The approach the controller will give: the pilot's own flight's
            # kept choice when it is landing here, and the field's weather,
            # so the broadcast says "ILS Zulu" when approach will.
            generator.approach_for = (
                lambda runway, field=airport: self.brain.approach_for(
                    self.session if self.session is not None
                    and getattr(self.session.destination, "ident", "")
                    == field.ident else None,
                    field, runway, self.weather_for(field.ident)))
            # The runway the controllers are using, flight plan included, so
            # the broadcast never names another.
            generator.runway_for = (
                lambda weather, departing, field=airport: (
                    self.brain.departure_runway(field, weather) if departing
                    else self.brain.arrival_runway(field, weather)))
            # The set the traffic is actually working, parallels and flow
            # runways included, so the broadcast names them all.
            generator.runways_for = (
                lambda weather, departing, field=airport: (
                    self.traffic.runways_in_use(field.ident, departing)
                    if self.traffic is not None else ()))
            generator.vfr_briefing = (
                lambda runway, field=airport, p=generator.phrase:
                self.brain.atis_briefing(field, runway, p))
            self._atis[key] = generator
            self._ask_for_approaches(airport.ident)
        # Not under an aeroplane on final: a new letter there is a readback
        # nobody can give and a runway nobody can change to. It rolls once
        # the aeroplane is down.
        if generator.current is not None and self._on_final_at(airport_ident):
            return generator.current
        before = generator.current
        report = generator.update(self.weather_for(airport_ident))
        if report is not before:
            decision("atis", airport=airport_ident, letter=report.letter,
                     language=language, weather=report.weather.source,
                     wind_from=report.weather.wind_from or None,
                     runways=report.landing_runways[:1],
                     departing=report.departing_runways[:1],
                     approach=report.approach_in_use)
        return report

    # How far out "on final" begins, for holding the ATIS letter.
    FINAL_FOR_ATIS_NM = 15.0

    def _on_final_at(self, ident: str) -> bool:
        session = self.session
        state = self.state
        if session is None or state is None or state.on_ground:
            return False
        if session.phase not in (Phase.APPROACH, Phase.LANDING):
            return False
        field = self.navdb.airport(ident)
        if field is None or not state.has_position:
            return False
        destination = session.destination
        if destination is not None and destination.ident != field.ident \
                and (self.station is None or self.station.ident != field.ident):
            return False
        return state.distance_to(field.lat, field.lon) <= self.FINAL_FOR_ATIS_NM

    def _online_atis(self, ident: str) -> AtisReport | None:
        """The ATIS a network controller is broadcasting for this field."""
        if not self.online.enabled or not self.config.network.use_online_atis:
            return None
        found = self.online.atis_for(ident)
        if found is None or not found.atis:
            return None
        return AtisReport(
            airport=ident,
            letter=found.atis_letter or "",
            text=found.atis,
            weather=self.weather_for(ident),
        )

    def _language_for(self, ident: str, requested: str | None = None) -> str:
        """Resolve a language against what is worked at this facility."""
        wanted = (requested or self.last_language or "en").lower()[:2]
        return reply_language(ident, wanted,
                              self.pinned_languages or None)

