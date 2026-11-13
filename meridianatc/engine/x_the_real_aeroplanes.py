from __future__ import annotations

from ._shared import (
    decision,
    log,
    math,
    threading,
    time,
)


class _The_real_aeroplanes:
    # ------------------------------------------------------------------
    # the real aeroplanes round the pilot (integrations/adsb.py)
    # ------------------------------------------------------------------

    live_feed = None
    _live_said: tuple = ("", "")
    _live_down_said = False

    def start_live_feed(self, run: bool = True):
        """Start asking the ADS-B sources, when the pilot has switched live
        traffic on and this program flies the traffic. With FSLTL flying it
        (traffic.source) the feed is never asked: the two together would put
        a second copy of every real flight in the sky."""
        from ..integrations import adsb
        from ..integrations.fsltl import traffic_source

        live = self.config.traffic.live
        if not live.enabled or self.live_feed is not None:
            return None
        source = traffic_source(self.config.traffic)
        if source != "meridianatc":
            decision("live_feed", event="not started", source=source,
                     why="traffic.source: FSLTL flies the traffic")
            self.emit("info", "Live traffic stays off: FSLTL flies the "
                      "traffic (Settings, Traffic).", key="log.live_fsltl")
            return None
        from ..paths import data_dir

        feed = adsb.Feed(sources=adsb.sources_for(live.own_url),
                         radius_nm=live.radius_nm)
        book = (adsb.FlightBook(data_dir() / "adsb_cache.json")
                if live.routes else None)
        self.live_feed = adsb.LiveFeedThread(feed, book, self._live_centre,
                                             poll_s=live.poll_s)
        decision("live_feed", event="started", radius_nm=feed.radius_nm,
                 poll_s=self.live_feed.poll_s,
                 sources=[s.name for s in feed.sources])
        if run:
            self.live_feed.start()
        return self.live_feed

    def _take_observed(self, session) -> None:
        """Hand the traffic everything somebody else flies: the simulator's
        aeroplanes (FSLTL, AIG, its own) and the real ones from the live feed,
        in one list, because set_observed finishes the strip of anything it
        is not given."""
        self.simwatch.enabled = self.config.traffic.from_simulator
        seen: list = []
        if self.config.traffic.from_simulator:
            self.simwatch.own_callsign = session.aircraft.callsign.written
            try:
                seen = list(self.simwatch.poll() or ())
            except Exception:
                log.debug("could not read the simulator's traffic",
                          exc_info=True)
        live = self._live_seen()
        if not self.config.traffic.from_simulator and self.live_feed is None:
            # Nothing new was read -- but set_observed finishes the strip
            # of anything it is not given, so skipping the call freezes
            # every observed aeroplane at its last position instead of
            # letting the strips go.
            seen = []
        try:
            self.traffic.set_observed(seen + live, time.monotonic())
        except Exception:
            log.debug("could not take the observed traffic", exc_info=True)
        if self.config.traffic.from_simulator:
            self._say_observed()

    live_roster = None

    def _live_seen(self) -> list:
        """The live flights for this moment (atc/live.py): the feed's last
        answer carried on, held through a quiet minute, then only an arrival
        on final, landed by us."""
        if self.live_feed is None:
            return []
        from ..atc.live import LiveRoster

        if self.live_roster is None:
            live = self.config.traffic.live
            self.live_roster = LiveRoster(live.fallback_after_s,
                                          delay_s=live.live_delay_s)
            roster = self.live_roster
            # How our tower holds one short of the pilot's runway, or has it
            # give way, and lets it go (TrafficInjector._protect_the_pilot).
            self.traffic.hold_live = lambda hex_, on: (
                roster.hold(hex_, time.time()) if on
                else roster.release(hex_, time.time()))
        feed = self.live_feed
        self.live_roster.update(feed.flights, feed.answered_wall, feed.routes)
        field = self.traffic.field if self.traffic.field.ident else None
        seen = self.live_roster.seen(time.time(), field)
        # The live aeroplanes have a budget of their own
        # (traffic.live.limit), on top of the invented ones: an over-full
        # answer is kept to the nearest ones, not to whoever the feed
        # listed first.
        limit = max(0, min(300, int(self.config.traffic.live.limit)))
        if len(seen) > limit:
            # Nearest the field being worked, not the pilot: where the
            # two differ the budget must keep the aeroplanes the tower
            # is talking about -- an arrival on this field's final --
            # not the ones merely nearest whoever is listening.
            centre = ((self.traffic.field.latitude,
                       self.traffic.field.longitude)
                      if self.traffic.field.ident
                      else self._live_centre())
            if centre is not None:
                lat0, lon0 = centre
                cos_lat = math.cos(math.radians(lat0))
                seen.sort(key=lambda one: (
                    (one.latitude - lat0) ** 2
                    + ((one.longitude - lon0) * cos_lat) ** 2))
            seen = seen[:limit]
        source = feed.state.source
        for one in seen:
            one.feed = source
        return seen

    def _live_centre(self):
        """Where the feed asks round: the aeroplane, else the field it is
        departing; nothing (and nothing sent) before either is known."""
        state = self.state
        if state is not None and state.has_position:
            return state.latitude, state.longitude
        field = self.session.departure if self.session is not None else None
        if field is not None:
            return field.lat, field.lon
        return None

    def _watch_live_feed(self) -> None:
        """A row each time the feed's state or source changes, and the
        outage said once, on the panel, in the pilot's language."""
        if self.live_feed is None:
            return
        from ..integrations.adsb import DOWN

        state = self.live_feed.state
        now = (state.status, state.source)
        if now == self._live_said:
            return
        self._live_said = now
        decision("live_feed", event="state", status=state.status,
                 source=state.source, count=state.count, age_s=state.age_s,
                 resting=[list(r) for r in state.resting],
                 dropped=list(state.dropped))
        if state.status == DOWN and not self._live_down_said:
            self._live_down_said = True
            self.emit("warning", "Live traffic: no source answering, invented "
                      "traffic in use.", key="log.live_down")
        elif state.status != DOWN and self._live_down_said and state.source:
            self._live_down_said = False
            self.emit("info", f"Live traffic is back, from {state.source}.",
                      key="log.live_back", source=state.source)

    def stop(self) -> None:
        with self._shutdown_lock:
            if self._shutdown_done:
                return
            self._shutdown_done = True
            self._stop_resources()

    def _stop_resources(self) -> None:
        def close(name, component, method):
            if component is None:
                return
            try:
                getattr(component, method)()
            except Exception:
                log.warning("could not close %s", name, exc_info=True)

        self._stop.set()
        with self._sim_lock:
            self._sim_wanted = False
            self._discard_simulator_attempt()
        close("live traffic", self.live_feed, "stop")
        close("cabin", self.cabin, "stop")
        close("push-to-talk", self._ptt, "stop")
        close("microphone", getattr(self, "microphone", None), "abandon")
        close("AI understanding", getattr(self, "understanding", None), "close")
        close("AI phrasing", getattr(self, "phrasing", None), "close")
        close("speech renderer", getattr(self, "synth", None), "stop")
        if hasattr(getattr(self, "recognizer", None), "close"):
            close("speech recognizer", self.recognizer, "close")
        if self._situation_thread is not None and self._situation_thread is not threading.current_thread():
            self._situation_thread.join(timeout=2.0)
            if self._situation_thread.is_alive():
                log.warning("the situation thread is still finishing a tick during shutdown")
        close("autonomous controls", self.autonomous_flight, "disengage")
        close("audio player", self.player, "stop")
        # Take the injected aeroplanes back out before the connection goes, or
        # they are left sitting in the simulator with nobody flying them.
        for name in ("simvehicles", "simdoors", "simsystems", "servicepoints", "simtraffic", "simwatch"):
            close(name, getattr(self, name), "detach")
        close("facilities", self.facilities, "close")
        close("GSX bridge", self.gsx.bridge, "close")
        close("simulator source", self.source, "close")
        close("history session", self.history, "close_session")
        close("history database", self.history, "close")
        close("navigation database", self.navdb, "close")

