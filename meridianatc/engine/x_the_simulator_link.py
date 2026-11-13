from __future__ import annotations

from ._shared import (
    Future,
    ManualSource,
    autofly_limit,
    capabilities,
    log,
    open_source,
    replace,
    threading,
    time,
)


class _The_simulator_link:
    # ------------------------------------------------------------------
    # the simulator link, watched rather than assumed
    # ------------------------------------------------------------------

    def attach_simulator(self) -> bool:
        """Try to take the aircraft state from the simulator, now.

        Returns whether it worked. Safe to call repeatedly: a failed attempt
        leaves whatever source was already in place, so a flight that started
        without a simulator carries on rather than losing its position to a
        half-built connection.
        """
        return self._adopt_simulator(open_source(**self._simulator_options()))

    def _simulator_options(self) -> dict:
        return dict(poll_interval=self.config.sim.poll_interval_s,
                    simulator=self.config.sim.simulator,
                    xplane_host=self.config.sim.xplane_host,
                    xplane_port=self.config.sim.xplane_port,
                    dll_path=self.config.sim.simconnect_dll)

    def _adopt_simulator(self, sim) -> bool:
        if isinstance(sim, ManualSource):
            sim.close()
            return False

        try:
            state = sim.read()
            if not state.connected:
                sim.close()
                return False
        except Exception:
            log.warning("could not read the candidate simulator source", exc_info=True)
            try:
                sim.close()
            except Exception:
                log.warning("could not close the rejected simulator source", exc_info=True)
            return False

        previous = self.source
        self.source = sim
        self._sim_source = sim
        self._sim_was_connected = True
        # Read once before anything is told the sim is here, so the first
        # thing that asks gets the aeroplane's real position rather than the
        # placeholder the manual source was holding.
        self.state = state
        if not capabilities(self.state.source).lvars:
            self._detach_simulator_integrations()
            self.emit("info", "X-Plane uses the browser/desktop panel. GSX, MSFS facilities, "
                      "native TCAS contact import and simulator flight-plan import are unavailable; "
                      "use published airport data and load plans manually or via SimBrief.",
                      key="log.xplane_limits")
        if previous is not sim:
            try:
                previous.close()
            except Exception:
                log.debug("could not close the previous state source",
                          exc_info=True)

        # Update autonomous flight with the new sim source
        if self.autonomous_flight and hasattr(self.autonomous_flight, 'sim'):
            self.autonomous_flight.sim = sim
            if self.config.immersion.autonomous_flight and not autofly_limit(self.state):
                self.autonomous_flight.enable()

        simulator = capabilities(self.state.source).name
        self.emit("info", f"Connected to {simulator}.",
                  key="log.sim_connected_name", simulator=simulator)
        # The ground handling, which needs the simulator to be there before
        # it can open a channel of its own. It used to be written and never
        # called, which is the whole of why GSX appeared to do nothing: the
        # variables were never read and the menu was never reachable.
        try:
            self.attach_lvar_bridge()
        except Exception:
            log.debug("could not attach the local variable bridge",
                      exc_info=True)
        if self.attach_simulator_traffic():
            self.emit("info", "Traffic will be visible in the simulator.",
                      key="log.traffic_visible")
        self.open_simulator_frequencies()
        # The airports whose frequencies were read from the last session are
        # not this session's; ask again.
        self._facilities_done.clear()
        return True

    def _detach_simulator_integrations(self) -> None:
        for integration in (self.simvehicles, self.simdoors, self.simsystems,
                            self.servicepoints, self.simtraffic, self.simwatch):
            integration.detach()
        self.facilities.close()
        bridge = self.gsx.bridge
        if bridge is not None:
            bridge.close()
        self.gsx.attach(None)
        self.traffic.await_observed = False

    def _start_simulator_attempt(self) -> None:
        options = self._simulator_options()
        attempt = self._sim_attempt = Future()

        def connect():
            try:
                attempt.set_result(open_source(**options))
            except Exception as exc:
                attempt.set_exception(exc)

        threading.Thread(target=connect, name="MeridianATC-simulator", daemon=True).start()

    def _discard_simulator_attempt(self) -> None:
        with self._sim_lock:
            self._discard_simulator_attempt_locked()

    def _discard_simulator_attempt_locked(self) -> None:
        attempt = self._sim_attempt
        self._sim_attempt = None
        if attempt is not None:
            def close(result):
                try:
                    result.result().close()
                except Exception:
                    log.debug("discarded simulator attempt", exc_info=True)
            attempt.add_done_callback(close)

    def want_simulator(self, wanted: bool) -> None:
        """Start or stop looking for the simulator, without a restart.

        Turning it on tries immediately rather than waiting out the
        reconnect interval, because a pilot who has just flicked the switch
        is watching for something to happen.
        """
        with self._sim_lock:
            self._want_simulator_locked(wanted)

    def _want_simulator_locked(self, wanted: bool) -> None:
        self._sim_wanted = bool(wanted)
        if not self._sim_wanted:
            self._discard_simulator_attempt()
            if self._sim_source is not None and self.source is self._sim_source:
                self._detach_simulator_integrations()
                self.source.close()
                self.source = ManualSource(replace(self.state, paused=False))
                self._sim_source = None
                self.refresh_state()
            self._next_sim_try = 0.0
            return
        self._next_sim_try = 0.0
        self._watch_simulator()

    # The shortest the engine will wait between attempts, whatever the
    # config says. Asking SimConnect for a connection it cannot give costs a
    # library call and a timeout, and doing that four times a second while
    # the simulator loads is how a panel becomes unresponsive waiting for the
    # thing it is waiting for.
    MIN_SIM_RETRY_S = 2.0

    def _retry_interval(self) -> float:
        return max(self.MIN_SIM_RETRY_S, self.config.sim.reconnect_interval_s)

    def _watch_simulator(self) -> None:
        """Connect when the simulator appears, and notice when it goes.

        This is the whole of the fix for a panel that sat at the departure
        airport with numbers that never moved. The old code connected once,
        during startup, and a pilot who opened this before the simulator had
        finished loading -- which is nearly everybody -- got the manual source
        for the entire flight with no way back.
        """
        with self._sim_lock:
            self._watch_simulator_locked()

    def _watch_simulator_locked(self) -> None:
        if not self._sim_wanted:
            return
        now = time.monotonic()

        link = self._sim_source
        if link is not None and self.source is link:
            if link.connected:
                return
            # It was there and has gone. Say so once, drop back to a manual
            # source holding the last known position, and start looking
            # again. The position is kept rather than zeroed: it is the best
            # guess there is, and zeroing it puts the aeroplane in the
            # Atlantic and every frequency out of range.
            self._sim_was_connected = False
            self.emit("info", "Lost the simulator. Looking for it again.",
                      key="log.sim_lost")
            held = replace(self.state)
            held.connected = True
            held.source = "manual"
            held.paused = False
            self._detach_simulator_integrations()
            try:
                link.close()
            except Exception:
                log.debug("could not close the simulator source", exc_info=True)
            self.source = ManualSource(held)
            self._sim_source = None
            self._next_sim_try = now + self._retry_interval()
            return

        if now < self._next_sim_try:
            return
        if self._sim_attempt is None:
            self._start_simulator_attempt()
            return
        if not self._sim_attempt.done():
            return
        attempt = self._sim_attempt
        self._sim_attempt = None
        self._next_sim_try = now + self._retry_interval()
        try:
            sim = attempt.result()
        except Exception:
            log.debug("simulator connection attempt failed", exc_info=True)
            return
        if self._adopt_simulator(sim):
            # A flight built around a typed departure field is not the flight
            # the simulator is holding. Rebuild it from what is actually
            # there, which is what stops the panel insisting on the old
            # airport once the aeroplane is somewhere else.
            self.begin_session()

    # ------------------------------------------------------------------

    def rebind_ptt(self) -> None:
        """Move push-to-talk onto whatever the settings now say.

        Picking a different button used to save the number and change
        nothing: the listener was built once at startup, so the old button
        went on keying the microphone and the new one did nothing at all.
        From the pilot's side that is indistinguishable from the setting
        being ignored, because it was.

        Also the way a controller plugged in after startup gets used, and the
        way a failed binding recovers -- both leave no listener running, and
        both are fixed by building a new one.
        """
        if self._situation_thread is None or self._stop.is_set():
            return          # not running yet: start() will do this properly
        if self._ptt is not None:
            try:
                self._ptt.stop()
            except Exception as exc:
                log.debug("could not stop push-to-talk: %s", exc)
            self._ptt = None
        # A transmission keyed on the old button has nothing left to unkey
        # it, and would otherwise record until the microphone's own limit.
        self.end_transmission()
        self._start_ptt()

