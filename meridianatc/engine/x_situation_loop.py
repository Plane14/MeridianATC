from __future__ import annotations

from ._shared import (
    Station,
    XPlaneTraffic,
    log,
    time,
    timed,
)


class _Situation_loop:
    # ------------------------------------------------------------------
    # situation loop
    # ------------------------------------------------------------------

    def _current_station(self) -> Station | None:
        if self.state.com1_active:
            return self.tune(self.state.com1_active)
        return self.station

    def _situation_loop(self) -> None:
        failed = False
        while not self._stop.is_set():
            try:
                self._tick()
            except Exception:
                if not failed:
                    log.warning("situation update failed; check the simulator connection and configuration",
                                exc_info=True)
                failed = True
            else:
                if failed:
                    log.info("situation updates recovered")
                failed = False
            self._stop.wait(self.config.sim.poll_interval_s)

    def _tick(self) -> None:
        with timed("tick", warn_over_ms=1500):
            self._one_tick()

    def _one_tick(self) -> None:
        # Before anything is read: is the thing being read from still there?
        self._watch_simulator()
        self._watch_live_feed()

        # And is the microphone still a microphone? First, because a transmit
        # state that is stuck silences everything below this line.
        self._watch_transmission()
        # And what the X-Plane plugin's bound keys asked for since last
        # tick: another radio's talk button, the copilot's mic, copy_error.
        self._drain_plugin_commands()

        previous = self.state
        with timed("tick.read", warn_over_ms=500):
            state = self.refresh_state()
        # Renders the lane finished since the last pass, handed back on this
        # thread: the radio to the player, a crew line to its queue slot.
        self.synth.drain_renders()
        session = self.session
        if session is None:
            return
        # Every watcher below reads the debounced on-ground flag: the raw one
        # flickers on a hard landing, and each flicker was a takeoff and a
        # touchdown. ``self.state`` keeps the raw reading for the panel.
        previous, state = self._debounced_ground(previous, state)

        # Nothing is happening in the aeroplane.
        #
        # While the simulator is paused or the user is in its menus, every
        # watcher below is watching for a change -- a takeoff, a touchdown, a
        # runway entered, a thousand feet passed -- and none of the changes
        # that arrive are things the aeroplane did. A menu that moves you to
        # another airport is not a flight, and a first officer calling the
        # landing checklist at it is the program talking over the top of
        # somebody who is not even flying.
        #
        # The state is still read, so the panel keeps telling the truth about
        # where the aeroplane is, and it keeps being written to ``self.state``
        # -- which is what makes the first tick after the menu compare two
        # readings from after the jump rather than straddling it.
        if state.paused:
            if not self._was_paused:
                self._was_paused = True
                self.crew.hold()
                self.emit("info", "The simulator is paused.",
                          key="log.sim_paused")
            return
        if self._was_paused:
            self._was_paused = False
            self.emit("info", "Back in the aeroplane.",
                      key="log.sim_unpaused")

        # The pilot changed frequency: work out who is now listening.
        retuned = bool(state.com1_active) \
            and abs(state.com1_active - self._last_com1) > 0.004
        if retuned:
            self.tune(state.com1_active)
        # And COM2, which hears rather than speaks: its station is kept for
        # the panel's second radio and for ATIS, and it never decides who
        # the pilot is talking to.
        if abs((state.com2_active or 0.0) - self._last_com2) > 0.004:
            self._last_com2 = state.com2_active or 0.0
            if self._last_com2:
                self.station2 = self._resolve_com2(state)
            else:
                self.station2 = None
        self._watch_com2(state, session)
        self._settle_oceanic(session)
        self._explain_empty()
        if not retuned and self.station is None:
            # Nothing tuned, so the country under the aeroplane decides what
            # the recogniser listens for. Checked rarely: it only changes when
            # a border does.
            self._follow_region(state)

        # The airport under the aeroplane decides what is on the radio there,
        # and the simulator is the authority on that.
        with timed("tick.follow", warn_over_ms=500):
            self.follow_facilities(state)
            # And its taxiways, for the pilot's own taxi, at both ends.
            self.follow_ground_layouts(state, session)

        # And who is on the network, which decides whether this program is
        # entitled to say anything at all. The poll is in the background; the
        # answer is whatever the last one found.
        if self.online.enabled:
            self.online.poll()
            self._watch_for_humans(state)

        # A liftoff off a landing rollout is classified first, so that the
        # surface watch does not take a bounce for a takeoff.
        with timed("tick.phase", warn_over_ms=500):
            self._note_rollout_liftoff(previous, state, session)
            self._follow_route(state)
            # And the simulator's own loaded plan, which is the route when
            # the setting asks for it and nothing newer has replaced it.
            self._poll_sim_plan()

            # What the aircraft is doing is checked before the phase is
            # updated, because a departure and a touchdown are transitions:
            # updating first would clear the very flags that say whether
            # either was cleared.
            self._watch_surface(previous, state, session)
            # And the runway crossings on a real taxi route, worked the same
            # way: granted or held as the aeroplane reaches each one.
            self._watch_crossings(previous, state, session)
            self._watch_taxi(state, session)

            # Track the phase from what the aircraft is actually doing, so
            # the controller stays in step even if the pilot skips a call.
            self._update_phase(previous, state, session)

            # And what the rollout liftoff turns out to be: back on the
            # ground (a bounce), round the circuit, or round again.
            self._after_rollout_liftoff(previous, state, session)
            # Or nothing at all: still on the ground, so it was a full stop.
            self._full_stop_after_touchdown(state, session)

            # The landing, measured. Written up a few seconds after the
            # wheels touch, once a bounce has had time to show itself.
            self._watch_landing(previous, state, session)

        # What the tug and the jetway are doing, so the controller does not
        # clear an aeroplane to move that is bolted to the stand.
        if self.config.gsx.enabled:
            with timed("tick.gsx", warn_over_ms=500):
                self.gsx.poll(state)
            for line in (self.gsx.changed(), self.gsx.notice()):
                if line:
                    self.emit("info", line, key="log.ground_handling",
                              detail=line)

        # A readback that never came. Chasing it is what makes a readback
        # mandatory rather than merely expected: without this the instruction
        # sat pending for the rest of the flight and nothing was ever said.
        self._chase_readback(session)

        # The departure that was told to line up and wait, and the arrival,
        # worked rather than waited for. The arrival is before the handoff on
        # purpose: an aircraft is cleared for the approach by the radar
        # position that then passes it to the tower, not by the tower it
        # arrives on.
        with timed("tick.atc", warn_over_ms=500):
            self._work_the_departure(state, session)
            self._work_the_climb(state, session)
            self._work_the_arrival(previous, state, session)
            self._watch_airspace(state, session)
            self._watch_restriction(state, session)
            self._watch_radar_service(state, session)
            self._watch_transponder(state, session)
            self._watch_deviation(state, session)
            self._watch_traffic(state, session)
            self._watch_runway(state, session)
            self._watch_points(state, session)
        if not self.stand_down:
            self._say_held_reply(time.monotonic())

        # The handoffs the flight itself calls for: off the tower after
        # departure, back to ground after landing. These are not reminders and
        # are not optional, which is why they are not behind the setting.
        self._hand_off_on_phase(state, session)

        if self.config.atc.proactive_handoff:
            self._offer_handoff(state, session)

        # The rest of the aeroplane, following the same state the controller
        # is reading. It can only make noise; it cannot change anything here.
        #
        # The cabin is held under the radio first. Nothing in there ever
        # starts while the frequency is live -- that is the crew's own rule --
        # so what this is for is the announcement already half said when the
        # pilot keys the microphone, and the answer that lands in the middle
        # of it.
        self.cabin.ducked = self.radio_busy()
        try:
            with timed("tick.crew", warn_over_ms=500):
                self.crew.follow(previous, state, session)
        except Exception:
            log.debug("the crew raised", exc_info=True)

        # Autonomous flight tick (First Officer flying the aircraft)
        if self.autonomous_flight:
            try:
                with timed("tick.autonomous", warn_over_ms=500):
                    self.autonomous_flight.tick()
            except Exception:
                log.debug("autonomous flight raised", exc_info=True)

        if self.config.traffic.enabled:
            # The invented aeroplanes' own switch, read live so the
            # panel's toggle takes effect mid-flight: off drops the ones
            # flying and makes no more; the real and observed ones are
            # somebody else's and stay.
            self.traffic.invented = bool(
                getattr(self.config.traffic, "invented", True))
            with timed("tick.traffic", warn_over_ms=500):
                self._run_traffic(state, session)
            # A simulator that will not create AI aircraft says so once,
            # rather than leaving a pilot looking out of the window for
            # aeroplanes that were promised at startup and never made.
            if (self.simtraffic.refused
                    and session.advise_once("sim_traffic_refused")):
                if isinstance(self.simtraffic, XPlaneTraffic):
                    self._warn_xplane_traffic()
                else:
                    self.emit("info",
                              "This simulator will not create AI aircraft, so "
                              "MeridianATC adds no traffic of its own. Aircraft "
                              "already in the simulator are still worked.",
                              key="log.traffic_refused")
        elif self.brain.traffic is not None:
            # Switched off mid-flight: the aeroplanes stop existing, and the
            # controller stops sequencing the user against them.
            self.brain.traffic = None
            self.traffic._release_all_held()
            self.traffic.flights.clear()
            self.traffic.pending.clear()
            self.traffic._taken_over.clear()
            self.traffic._flown_for.clear()
            self.traffic._parked_since.clear()
            self.traffic._drop_invented()
            self._spawned.clear()
            self.traffic.observed_count = 0
            self.traffic.pending.clear()
            self._traffic_queue.clear()
            self.simtraffic.sync([])
            self.simvehicles.sync([])

