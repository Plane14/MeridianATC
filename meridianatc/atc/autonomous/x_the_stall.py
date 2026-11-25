"""The stall watch and recovery.

A stall is the one envelope exceedance no autopilot bug answers: the
wing has to be unloaded -- nose down, power on, wings level -- before
anything else is asked of the aeroplane. ``_monitor_stall`` runs on the
fast tick off the instruments the pilot would hear and see: the model's
own stall warning where it publishes one, the angle of attack against
the warning angle the model declares, and the indicated speed against
the book's stall figure where neither needle is wired. Any of them,
held past a gust, hands the axes to the recovery; all of them agreeing
the wing is flying again, held a moment longer, hands them back -- and
the phase machine takes the aeroplane from where the recovery left it.

A model that publishes none of it simply never triggers. Rotation and
the flare chirp the same switch a stall does, which is what the height
gate and the persistence clocks are for.
"""

# How long a trigger has to hold before it is a stall rather than a
# gust or a wheel chirping the switch in the flare. The speed-only
# trigger -- what fires when no horn and no warning angle is published
# -- asks for longer: it is also what a steep slow turn briefly is, and
# an approach a few knots under Vref is a deliberate thing.
_HORN_S = 0.5
_SPEED_S = 1.0

# Above the runway the horn means something; inside the flare's height
# the same chirp is the arrival. The speed trigger is gated too: below
# the flare's height a push is not a recovery, it is a nose-first arrival.
_MIN_AGL_FT = 50.0

# How long the instruments have to agree the wing is flying again
# before the FO lets the aeroplane go: a gust drops and returns in a
# half-second, a recovery does not.
_CLEAR_S = 1.5

# The recovery's own figures: a firm push -- not a bunt, which answers
# a swept wing's stall by deepening it -- and the wings levelled at a
# rate a wing that just quit still answers.
_PUSH = -0.45
_WINGS_LEVEL = 0.6


class _The_stall:
    """Stall detection and recovery -- the envelope's floor. Mixin on the
    FO."""

    def _monitor_stall(self, state, now: float) -> None:
        """Watch for the stall every fast tick.

        Detection only here; the flying is ``_fly_stall_recovery``'s,
        called from the tick for as long as ``stall_recovering`` stands.
        On the wheels the watch stands down entirely: a stall that met
        the ground is an arrival, not a regime to keep flying.
        """
        if getattr(self, "_heli", None) is not None:
            return
        if bool(getattr(state, "on_ground", True)):
            self.state.stall_since = 0.0
            self.state.stall_clear_since = 0.0
            if self.state.stall_recovering:
                self.state.stall_recovering = False
                self._emit("info",
                           "Stall -- the aeroplane has landed",
                           "autonomous.stall.ground")
            return
        if not self._envelope_protection:
            self.state.stall_since = 0.0
            return

        if self.state.stall_recovering:
            self._watch_recovery(state, now)
            return

        trigger = self._stall_trigger(state)
        if not trigger:
            self.state.stall_since = 0.0
            self.state.stall_warned = False
            return
        if not self.state.stall_since:
            self.state.stall_since = now
            return
        need = _SPEED_S if trigger == "speed" else _HORN_S
        if now - self.state.stall_since < need:
            return

        self.state.stall_recovering = True
        self.state.stall_count += 1
        self.state.stall_clear_since = 0.0
        self._recovery_at = now
        self._last_hdg = getattr(state, "heading_true", None)
        self._last_hdg_at = now
        # The autopilot is part of how it got here -- the altitude or
        # vertical speed it was holding into the stall is what the bug
        # was set to. The recovery flies the aeroplane itself.
        if getattr(self, "director", None) is not None:
            self.director.autopilot(False)
        self.state.ap_master = False
        if not self.state.stall_warned:
            self.state.stall_warned = True
            self._emit("info", f"Stall ({trigger}) -- recovering",
                       "autonomous.stall.enter")
        self._fly_stall_recovery(state, now)

    def _stall_trigger(self, state) -> str:
        """Which instrument is calling the stall, or "": the horn, the
        angle of attack over its warning figure, or -- what fires on a
        model that publishes neither -- the speed under the book's stall
        figure. Horn and alpha want some height before they mean a
        stall; the speed is the airborne figure itself."""
        unavailable = getattr(state, "unavailable", ())
        high = getattr(state, "altitude_agl_ft", 9999.0) > _MIN_AGL_FT
        if high and "stall_warning" not in unavailable \
                and getattr(state, "stall_warning", False):
            return "horn"
        warn = float(getattr(state, "stall_aoa_deg", 0.0) or 0.0)
        if (high and warn > 0.0 and "aoa_deg" not in unavailable
                and float(getattr(state, "aoa_deg", 0.0) or 0.0) >= warn):
            return "alpha"
        stall_kt = float(self._limits(state).get("stall_kt") or 0.0)
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        if high and stall_kt > 0.0 and 0.0 < ias < stall_kt * 0.97:
            return "speed"
        return ""

    def _watch_recovery(self, state, now: float) -> None:
        """While the recovery flies, watch for the wing flying again.

        "Clear" wants every published instrument agreeing: the horn
        quiet, the alpha back under its warning figure -- or under a
        conservative eight degrees on a model that publishes alpha but
        no threshold -- and the speed a margin above the book's stall
        figure. One disagreeing resets the clock.
        """
        unavailable = getattr(state, "unavailable", ())
        clear = True
        if ("stall_warning" not in unavailable
                and getattr(state, "stall_warning", False)):
            clear = False
        if "aoa_deg" not in unavailable:
            warn = float(getattr(state, "stall_aoa_deg", 0.0) or 0.0)
            ceiling = warn - 2.0 if warn > 2.0 else 8.0
            if float(getattr(state, "aoa_deg", 0.0) or 0.0) >= ceiling:
                clear = False
        stall_kt = float(self._limits(state).get("stall_kt") or 0.0)
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        if stall_kt > 0.0 and ias < stall_kt * 1.15:
            clear = False
        if not clear:
            self.state.stall_clear_since = 0.0
            return
        if not self.state.stall_clear_since:
            self.state.stall_clear_since = now
        elif now - self.state.stall_clear_since >= _CLEAR_S:
            self._end_recovery()

    def _end_recovery(self) -> None:
        """Hand the aeroplane back: centres the axes the recovery held
        and lets the phase machine take it from where it is."""
        self.state.stall_recovering = False
        self.state.stall_since = 0.0
        self.state.stall_clear_since = 0.0
        self.state.stall_warned = False
        if getattr(self, "director", None) is not None:
            self.director.center_controls()
        self._emit("info", "Stall recovered -- resuming the flight",
                   "autonomous.stall.recovered")

    def _yaw_rate(self, state, now: float) -> float:
        """Heading change per second, kept between ticks.

        No model on either sim publishes yaw rate on the requests the
        batch already makes, so the rate is the heading's own delta --
        good enough to tell a rotation from a heading change.
        """
        hdg = getattr(state, "heading_true", None)
        last = getattr(self, "_last_hdg", None)
        last_at = getattr(self, "_last_hdg_at", 0.0)
        self._last_hdg, self._last_hdg_at = hdg, now
        if hdg is None or last is None or now <= last_at:
            return 0.0
        delta = (float(hdg) - float(last) + 540.0) % 360.0 - 180.0
        return delta / (now - last_at)

    def _fly_stall_recovery(self, state, now: float) -> None:
        """The recovery itself, every fast tick while it stands: full
        power, the drag in, a firm push, the wings levelled and the
        ball centred. The aeroplane comes out of the stall before it
        is asked to climb out of it."""
        director = getattr(self, "director", None)
        if director is None:
            return
        # The heading's own clockwork tells a spin from a spiral: yawing
        # fast with the wing quit is rotation, and rotation is a spin's
        # first half -- an aileron push into it only stalls the other
        # wing. Past forty degrees a second it is flown by the book:
        # power off, ailerons neutral, rudder against the turn, nose
        # full down.
        yaw_rate = self._yaw_rate(state, now)
        if abs(yaw_rate) > 40.0:
            director.throttle(0.0)
            director.spoilers(0.0)
            director.aileron(0.0)
            director.rudder(-1.0 if yaw_rate > 0.0 else 1.0)
            director.elevator(_PUSH * 1.6)
            return
        # The wing quits on angle of attack alone, so the push comes
        # first and the power after it: full throttle into a stalled,
        # slow prop job adds torque and P-factor the rudder then has to
        # fight. Half power keeps the slipstream over the surfaces
        # while the nose comes down; the wing is let prove it flies --
        # speed back over the book figure -- before it is asked to
        # climb out of the hole.
        stall_kt = float(self._limits(state).get("stall_kt") or 0.0)
        ias = float(getattr(state, "indicated_airspeed_kt", 0.0) or 0.0)
        flying = stall_kt > 0.0 and ias >= stall_kt * 1.05
        director.throttle(1.0 if flying else 0.4)
        director.spoilers(0.0)
        push = _PUSH
        if flying or float(getattr(state, "pitch_deg", 0.0) or 0.0) < -5.0:
            push = 0.0
        if float(getattr(state, "altitude_agl_ft", 9999.0) or 0.0) < 500.0:
            push = max(push, -0.25)
        director.elevator(push)
        bank = float(getattr(state, "bank_deg", 0.0) or 0.0)
        director.aileron(-max(-1.0, min(1.0, bank / 40.0)) * _WINGS_LEVEL)
        director.rudder(0.0)
