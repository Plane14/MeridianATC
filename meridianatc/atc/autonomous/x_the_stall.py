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
# the same chirp is the arrival. The speed trigger is exempt: a speed
# under the stall figure through the flare is the hard landing the
# touchdown report already answers.
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
        if stall_kt > 0.0 and 0.0 < ias < stall_kt * 0.97:
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

    def _fly_stall_recovery(self, state, now: float) -> None:
        """The recovery itself, every fast tick while it stands: full
        power, the drag in, a firm push, the wings levelled and the
        ball centred. The aeroplane comes out of the stall before it
        is asked to climb out of it."""
        director = getattr(self, "director", None)
        if director is None:
            return
        director.throttle(1.0)
        director.spoilers(0.0)
        director.elevator(_PUSH)
        bank = float(getattr(state, "bank_deg", 0.0) or 0.0)
        director.aileron(-max(-1.0, min(1.0, bank / 40.0)) * _WINGS_LEVEL)
        director.rudder(0.0)
