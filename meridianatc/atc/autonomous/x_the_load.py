"""The airframe's load watch.

The envelope module keeps the aeroplane between its speeds; this watch
keeps it between its G limits. ``_monitor_load`` runs on the fast tick
and flags ``state.g_limited`` while the load approaches or exceeds the
airframe's limit -- the phase code then bounds its commands through
``_g_pull_cap`` on the hand-flown legs, and the watch itself eases the
autopilot's bug on the AP legs. Touchdown gets its own report: the G
the wheels arrived with, firm over the threshold, hard over the red
line the model's book never published.

The aeroplane that never reports ``G FORCE`` reads 1.0 -- comfortably
inside every limit -- and the watch simply never triggers.
"""

# Sub-phases whose elevator is hand-flown: the watch does not touch the
# AP on them, it bounds the pull via _g_pull_cap inside the phase code.
_HAND_FLOWN = {"rotate", "flare", "go_around", "roll", "rollout", "taxi",
               "align", "initial_climb"}

# How far inside the limit a soft-margin flag counts from: warn at 85%
# of the book figure, ease commands over the book figure itself.
_SOFT = 0.85

# Seconds comfortably inside before the flag clears -- the hysteresis
# that keeps a gust line from toggling the limiter every tick.
_CLEAR_S = 3.0

# The pull the limiter allows while g_limited: a gentle flare's worth.
_LIMITED_PULL = 0.10

# Touchdown reporting, normal G at the mains.
_FIRM_G = 1.8
_HARD_G = 2.2


class _The_load:
    """G-limit watch and the touchdown report -- the envelope's vertical
    bookend. Mixin on the FO."""

    @property
    def _envelope_protection(self) -> bool:
        """The kill-switch: envelope/load protection on or off. Like
        the weather's reactions, a knob for a FO who is asked to fly
        outside the envelope on purpose."""
        immersion = getattr(getattr(self, "config", None), "immersion", None)
        return bool(getattr(immersion, "autonomous_envelope_protection", True))

    def _monitor_load(self, state, now: float) -> None:
        """Watch the load factor every fast tick.

        Flags ``state.g_limited`` while the G runs over the soft margin
        and eases the aeroplane out of it; flags and reports the
        touchdown G the moment the wheels say ground. The rotorcraft FO
        has its own watch inside ``_protect`` and stays away.
        """
        g = float(getattr(state, "g_force", 1.0) or 0.0)
        on_ground = bool(getattr(state, "on_ground", True))
        prev_ground = getattr(self, "_load_on_ground", None)
        self._load_on_ground = on_ground

        # The touchdown report fires off the air->ground edge even when
        # the protection knob is off: reading the G is free.
        if prev_ground is False and on_ground and g > 0.0:
            self._note_touchdown(state, g)

        if (not self._envelope_protection
                or getattr(self, "_heli", None) is not None
                or on_ground
                or "g_force" in getattr(state, "unavailable", ())):
            return

        lim = self._limits(state)
        pos_lim = float(lim.get("g_pos") or 0.0)
        neg_lim = float(lim.get("g_neg") or 0.0)
        if pos_lim <= 0.0:
            return
        neg_lim = neg_lim if neg_lim < 0.0 else -1.0

        over = g > pos_lim or g < neg_lim
        near = g > pos_lim * _SOFT or g < neg_lim * _SOFT

        if near:
            self.state.g_limited_since = now
            self.state.g_limited = True
            if over:
                if not self.state.g_warned:
                    self.state.g_warned = True
                    self._emit("info",
                               f"Load limit exceeded ({g:.1f} G): easing off",
                               "autonomous.load.g_limit")
                if self.state.sub not in _HAND_FLOWN:
                    self._ease_the_load(state, now)
            elif not self.state.g_warned:
                self.state.g_warned = True
                self._emit("info",
                           f"Approaching the load limit ({g:.1f} G)",
                           "autonomous.load.g_near")
        elif now - self.state.g_limited_since > _CLEAR_S:
            # Three seconds back inside: re-arm the flag and its warning.
            self.state.g_limited = False
            self.state.g_warned = False

    def _ease_the_load(self, state, now: float) -> None:
        """Level the aeroplane out of the exceedance.

        On the AP legs the gentlest answer is altitude: the VS bug goes
        to zero -- the AP stops pulling -- and the bank cap comes down
        so the next turn cannot pull it straight back. The hand-flown
        legs bound their own pull through ``_g_pull_cap`` instead.
        """
        if getattr(self, "director", None) is None:
            return
        self.director.vertical_speed(0.0)
        # The bug alone answers nothing where the AP is flying FLCH,
        # altitude hold or the approach: the zero rate is asked for
        # with the mode that honours it.
        self.director.vs_hold(True)
        self.state.ap_vs_mode = True
        self.state.vs_bug_cmd = 0.0
        self.state.vs_bug_at = now
        self._cap_bank(15.0)

    def _cap_bank(self, degrees: float) -> None:
        """Ask the AP for a bank cap, once per change.

        ``AUTOPILOT MAX BANK`` is read back into the state when the sim
        answers; the send itself is the same event the panel's bank
        limit knob turns. Ignored where the AP has none.
        """
        if degrees == self.state.bank_cap_deg \
                or getattr(self, "director", None) is None:
            return
        # The intent is recorded even where the send goes unanswered --
        # a model without the event is not asked again every pass. The
        # sim's own readback, on ``AUTOPILOT MAX BANK``, is the truth of
        # whether it took.
        self.state.bank_cap_deg = degrees
        self.director.max_bank(degrees)

    def _g_pull_cap(self, state, want: float = 1.0) -> float:
        """The elevator pull a hand-flown leg may ask for this tick.

        Inside the margins the asked pull goes through untouched; while
        ``g_limited`` the answer tops out at the gentle-flare figure, so
        the rotation that would pull past the limit pulls to it instead.
        """
        if not self._envelope_protection:
            return want
        if getattr(self.state, "g_limited", False):
            return min(want, _LIMITED_PULL)
        return want

    def _note_touchdown(self, state, g: float) -> None:
        """Report the G the mains arrived with.

        Firm over the book's firm figure, hard over the heavy-landing
        one -- the engineer's question answered in the radio call the
        crew would make themselves.
        """
        self.state.touchdown_g = g
        if g >= _HARD_G:
            self._emit("info",
                       f"Hard landing ({g:.1f} G) -- log it for the engineers",
                       "autonomous.load.hard_landing")
        elif g >= _FIRM_G:
            self._emit("info",
                       f"Firm touchdown ({g:.1f} G)",
                       "autonomous.load.firm_touchdown")
