"""What the first officer has learned about the aeroplane it is flying.

Every add-on model is a little different: a collective that needs 62 %
rather than the 55 % the gains were tuned at, a steering axis that steers
the wrong way, a throttle that sits higher than the book figure on final.
None of that is in any published number, so the FO measures it. What is
learned on one flight -- a trim, a gain scale, a control polarity, a
steering authority -- is kept per aircraft model and preloaded on the
next, so the second flight in the same add-on starts where the first one
ended rather than from scratch.

What is learned
---------------

Trims
    Where a lever actually rests. Every PID channel with an integral
    term converges to the output that holds its error at zero -- that
    value *is* the trim -- so it is written down at phase changes and
    while the channel sits inside its deadband, and the integrator is
    preloaded with it when the channel next resets. A model whose
    flare lever wants to sit at 18 % rather than 30 % opens there on
    the next landing instead of hunting for it.

Scales
    A multiplier on a channel's gains, grown when the loop saturates
    and the error is not coming in (the aeroplane responds less than
    the tuned model, so ask more) and shrunk when the error keeps
    changing sign under a working lever (it responds more, so ask
    less). Bounded both ways, so a bad model is met with firmer or
    gentler hands, never with an open loop or a dead one.

Assist
    The per-instant version of the same idea, working in ticks where
    the scale works in flights. While an error is not coming in the
    loop presses harder this pass; once it is coming in the hand eases
    back, and an error that has just collapsed under its recent size
    is met softly, so the arrival is braked before the crossing rather
    than hunted after it. It is never written down -- the situation
    that asked for it is gone the next tick.

Signs
    Which way a control works. The taxi code already measures whether
    applying steering turns the nose left or right and flips the input
    to match; that measurement is kept, so a model whose steering is
    wired backwards is known backwards before the first metre is
    taxied on the next flight.

Values
    Anything else a piece of code wants to remember about the model,
    by name: the measured steering authority (deg/s of yaw per unit of
    tiller), the rotorcraft collective trim, and the like.

The store lives at ``data/control_adaptations.json``, one entry per
aircraft title (or ICAO type where no title is known), written at the
end of each leg and once a minute while flying. Everything fails
closed: a file that cannot be read, an add-on that reports no title,
a store that was never written -- each just means the FO flies on the
tuned defaults, exactly as it did before this existed.
"""

from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import replace
from pathlib import Path

from ..paths import data_dir
from .control import Pid, PidTerms

log = logging.getLogger(__name__)

#: Bounds for a learned gain scale: firmer than tuned, never more than
#: two and a half times; gentler, never less than forty per cent.
SCALE_LO, SCALE_HI = 0.4, 2.5

#: How long a channel may sit on its clamp with the error still outside
#: the deadband before its gains are grown a step.
SAT_S = 2.5

#: The gain step on saturation, and its inverse on oscillation. The
#: steps are decisive rather than timid: a model that answers badly is
#: found quickly on the first flight, and the bounds below are what keep
#: a fast verdict from overshooting into the next one.
SCALE_UP, SCALE_DOWN = 1.25, 0.78

#: Error sign flips inside this window while the lever is past half
#: travel mean the loop is chasing itself; the gains come down a step.
OSC_WINDOW_S, OSC_FLIPS = 15.0, 6

#: After a gain step the loop is left alone to show the new gain before
#: another may be taken. Without it the saturation "ask more" and the
#: oscillation "ask less" chase each other -- a lever swinging onto a
#: rail looks saturated while it hunts, so the two verdicts must not be
#: allowed to alternate in quick succession.
BUMP_GAP_S = 7.0

#: How fast a trim observation is blended in: an exponential blend, so a
#: single odd phase cannot move a trim that took a flight to learn.
TRIM_BLEND = 0.10

#: How fast a named scalar is blended in when a caller ``learn``s it.
VALUE_BLEND = 0.5

#: Bounds for the per-instant assist: harder than neutral, never past
#: 1.8 times; softer, never below 0.55. It works in ticks where the
#: learned scale works in flights -- a pass's press, not a model's
#: figure, so it is never written down.
ASSIST_LO, ASSIST_HI = 0.55, 1.8

#: How fast the assist climbs while an error is not coming in, eases
#: off once it has collapsed under its recent size, and drifts back to
#: neutral between the two, per second of evidence.
ASSIST_UP_S, ASSIST_DOWN_S, ASSIST_HOME_S = 0.45, 0.9, 1.5

#: The trailing figure the "is the error coming in" question is asked
#: against: an exponential of |error| at this per-second blend -- slow
#: enough that a tick of noise is not a verdict, fast enough that an
#: error holding its own is seen within a couple of seconds.
ERR_REF_BLEND_S = 0.5

#: The most aircraft models kept: enough for a large fleet, bounded so a
#: pilot who flies everything does not grow the file without limit.
MAX_MODELS = 200


def _store_path() -> Path:
    return data_dir() / "control_adaptations.json"


class Adaptation:
    """One aircraft model's learned control values.

    ``enabled`` False is the no-store case -- an aircraft with no title
    and no type, or the setting off: reads hand back the defaults and
    writes are dropped, so the loops behave exactly as they did before.
    """

    __slots__ = ("enabled", "trims", "scales", "signs", "values", "_touched")

    def __init__(self, enabled: bool = True) -> None:
        self.enabled = enabled
        self.trims: dict[str, float] = {}
        self.scales: dict[str, float] = {}
        self.signs: dict[str, int] = {}
        self.values: dict[str, float] = {}
        self._touched = False

    # -- trims ------------------------------------------------------------

    def trim(self, channel: str, default: float = 0.0) -> float:
        """The learned resting output of a channel, in output units."""
        return self.trims.get(channel, default) if self.enabled else default

    def learn_trim(self, channel: str, output: float,
                   blend: float = TRIM_BLEND) -> None:
        """Blend an observed resting output into the channel's trim."""
        if not self.enabled or not math.isfinite(output):
            return
        old = self.trims.get(channel)
        if old is None:
            self.trims[channel] = output
        else:
            self.trims[channel] = old + (output - old) * blend
        self._touched = True

    # -- scales -----------------------------------------------------------

    def scale(self, channel: str) -> float:
        """The learned gain multiplier of a channel; 1.0 until learned."""
        if not self.enabled:
            return 1.0
        return max(SCALE_LO, min(SCALE_HI, self.scales.get(channel, 1.0)))

    def learn_scale(self, channel: str, scale: float) -> None:
        if not self.enabled or not math.isfinite(scale):
            return
        scale = max(SCALE_LO, min(SCALE_HI, scale))
        if scale != self.scales.get(channel):
            self.scales[channel] = scale
            self._touched = True

    # -- signs ------------------------------------------------------------

    def sign(self, channel: str) -> int:
        """The learned control polarity: +1 normal, -1 reversed, 0 unknown."""
        if not self.enabled:
            return 0
        votes = self.signs.get(channel, 0)
        return (votes > 0) - (votes < 0)

    def vote(self, channel: str, direction: int) -> int:
        """One observation of which way a control worked.

        ``direction`` +1 says the response was the expected one, -1 the
        reversed one. Votes run to five either way; the tally is what is
        stored, so a model already known reversed starts there.
        """
        if not self.enabled:
            return 0
        votes = max(-5, min(5, self.signs.get(channel, 0) + direction))
        if votes != self.signs.get(channel, 0):
            self.signs[channel] = votes
            self._touched = True
        return votes

    def learn_votes(self, channel: str, votes: int) -> None:
        """Write a vote tally back -- the flip code re-seeds the count."""
        if not self.enabled:
            return
        votes = max(-5, min(5, int(votes)))
        if votes != self.signs.get(channel, 0):
            self.signs[channel] = votes
            self._touched = True

    # -- named scalars ----------------------------------------------------

    def value(self, name: str, default: float = 0.0) -> float:
        """A remembered figure -- a steering authority, a collective trim."""
        return self.values.get(name, default) if self.enabled else default

    def set(self, name: str, value: float) -> None:
        """Write a named figure outright -- a learned polarity, a trim."""
        if self.enabled and math.isfinite(value) \
                and self.values.get(name) != value:
            self.values[name] = value
            self._touched = True

    def learn(self, name: str, sample: float,
              blend: float = VALUE_BLEND) -> None:
        if not self.enabled or not math.isfinite(sample):
            return
        old = self.values.get(name)
        if old is None:
            self.values[name] = sample
        else:
            self.values[name] = old + (sample - old) * blend
        self._touched = True

    # -- serialisation ----------------------------------------------------

    def as_dict(self) -> dict:
        return {
            "trims": self.trims, "scales": self.scales,
            "signs": self.signs, "values": self.values,
        }

    @classmethod
    def from_dict(cls, data: dict) -> Adaptation:
        found = cls()
        try:
            found.trims = {str(k): float(v)
                           for k, v in data.get("trims", {}).items()
                           if math.isfinite(float(v))}
            found.scales = {str(k): max(SCALE_LO, min(SCALE_HI, float(v)))
                            for k, v in data.get("scales", {}).items()
                            if math.isfinite(float(v))}
            found.signs = {str(k): max(-5, min(5, int(v)))
                           for k, v in data.get("signs", {}).items()}
            found.values = {str(k): float(v)
                            for k, v in data.get("values", {}).items()
                            if math.isfinite(float(v))}
        except (TypeError, ValueError):
            return cls()
        return found


class AdaptivePid(Pid):
    """A :class:`Pid` whose gains and trim learn from the aircraft.

    ``channel`` names the loop ("throttle", "collective", ...) and
    ``adapt`` is the aircraft's store; a disabled store makes this a
    plain ``Pid`` with a preload step that costs nothing. The learned
    trim preloads the integrator on every reset -- the phase changes
    where the base class clears its memory are exactly where the learned
    figure should be put back.
    """

    def __init__(self, channel: str, adapt: Adaptation, terms: PidTerms,
                 clock=time.monotonic) -> None:
        self.channel = channel
        self._adapt = adapt
        self._base_terms = terms
        self._scale = adapt.scale(channel)
        super().__init__(self._scaled(terms, self._scale), clock)
        self._sat_t = 0.0
        self._last_sign = 0
        self._flips: list[float] = []
        # The last gain step, well in the past so the first evidence may act.
        self._bump_t = -BUMP_GAP_S
        # The instant assist and the error reference it is judged
        # against: neutral and unreferenced until the first pass.
        self._assist = 1.0
        self._err_ref = 0.0
        if terms.ki:
            self.reset()

    @staticmethod
    def _scaled(terms: PidTerms, scale: float) -> PidTerms:
        if scale == 1.0:
            return terms
        return replace(terms, kp=terms.kp * scale, ki=terms.ki * scale,
                       kd=terms.kd * scale)

    def update(self, error: float, dt: float | None = None,
               measurement: float | None = None,
               rate: float | None = None,
               effort: float | None = None) -> float:
        now = self._clock()
        if dt is None:
            dt = 0.0 if self._prev_t is None else now - self._prev_t
        if effort is None:
            effort = (self._assist_for(error, dt)
                      if self._adapt.enabled and 0.0 < dt <= self.DT_MAX
                      else 1.0)
        out = super().update(error, dt=dt, measurement=measurement,
                             rate=rate, effort=effort)
        if not self._adapt.enabled or not (0.0 < dt <= self.DT_MAX):
            return out
        self._learn(error, out, dt, now)
        return out

    __call__ = update

    def _assist_for(self, error: float, dt: float) -> float:
        """This pass's press: how hard the loop leans right now.

        The learned scale is the model's answer over a flight; the
        assist is the situation's answer this tick. An error that is
        holding its own against the loop -- the aeroplane not coming
        round, a stale read, an axis the model answers weakly -- is
        pressed against harder now rather than after the saturation
        verdict lands; an error that has collapsed well under its
        recent size is met softly, the braking a weave needs before
        the crossing and not the correction it chases after it.
        """
        t = self.terms
        err = 0.0 if abs(error) < t.deadband else error
        size = abs(err)
        ref = self._err_ref
        self._err_ref = (size if not ref
                         else ref + (size - ref)
                         * min(1.0, dt * ERR_REF_BLEND_S))
        if not err:
            # In the deadband: the hand is already neutral, no opinion.
            self._assist += (1.0 - self._assist) * min(1.0, dt * 2.0)
        elif ref > 0.0 and size <= ref * 0.45:
            # Collapsed to under half of where it was: arriving -- ease
            # off so it lands rather than drives through.
            self._assist = max(ASSIST_LO, self._assist - ASSIST_DOWN_S * dt)
        elif size >= ref * 0.92:
            # Not coming in -- holding or growing: press harder now.
            self._assist = min(ASSIST_HI, self._assist + ASSIST_UP_S * dt)
        else:
            # Coming in but not yet arrived: back toward a neutral hand.
            self._assist += (1.0 - self._assist) \
                * min(1.0, dt * ASSIST_HOME_S)
        return self._assist

    def _learn(self, error: float, out: float, dt: float, now: float) -> None:
        t = self.terms
        err = 0.0 if abs(error) < t.deadband else error

        # Inside the deadband the output that is holding is the trim.
        if t.ki and not err:
            self._adapt.learn_trim(self.channel, out,
                                   blend=min(dt * TRIM_BLEND, 0.25))

        bounded = t.high < math.inf and t.low > -math.inf
        if not bounded:
            return

        pushing = (out >= t.high - 1e-9 and err > t.deadband) \
            or (out <= t.low + 1e-9 and err < -t.deadband)
        if pushing:
            self._sat_t += dt
        else:
            self._sat_t = max(0.0, self._sat_t - 2.0 * dt)

        # An error that keeps crossing zero while the lever works
        # past half travel is the loop chasing itself: too much
        # gain for this model, not too little.
        sign = (err > t.deadband) - (err < -t.deadband)
        if sign and self._last_sign and sign != self._last_sign:
            span = t.high - t.low
            if abs(out - (t.high + t.low) / 2.0) > 0.25 * span:
                self._flips = [f for f in self._flips
                               if now - f < OSC_WINDOW_S]
                self._flips.append(now)
        if sign:
            self._last_sign = sign

        # The oscillation verdict wins over the saturation one: a lever
        # that swings onto a rail and back looks saturated while it
        # hunts, and growing the gains on that evidence is exactly
        # backwards. Saturation grows them only against a settled sign.
        if len(self._flips) >= OSC_FLIPS:
            self._bump(SCALE_DOWN, now)
            # Spent either way: a rejected bump still means this tally
            # cannot count toward the next one.
            self._flips.clear()
            self._sat_t = 0.0
        elif self._sat_t >= SAT_S:
            self._bump(SCALE_UP, now)
            self._sat_t = 0.0

    def _bump(self, factor: float, now: float) -> bool:
        if now - self._bump_t < BUMP_GAP_S:
            return False
        scale = max(SCALE_LO, min(SCALE_HI, self._scale * factor))
        if scale == self._scale:
            return False
        old_ki = self.terms.ki
        self._scale = scale
        self._bump_t = now
        self._adapt.learn_scale(self.channel, scale)
        self.terms = self._scaled(self._base_terms, scale)
        # The integrator's contribution is ki * _i: rescale _i so the
        # bump itself is not a step into the loop it is calming.
        new_ki = self.terms.ki
        if new_ki and old_ki:
            self._i *= old_ki / new_ki
        return True

    def reset(self, output: float | None = None) -> None:
        # What the loop converged to is learned before it is forgotten.
        if self._adapt.enabled and self._base_terms.ki \
                and self._out is not None:
            self._adapt.learn_trim(self.channel, self._out, blend=0.5)
        super().reset(output)
        # A phase boundary is not evidence of anything: the saturation
        # and oscillation tallies belong to the phase that made them.
        self._sat_t = 0.0
        self._flips.clear()
        self._last_sign = 0
        self._assist = 1.0
        self._err_ref = 0.0
        # The learned trim preloads the integrator only: ``_out`` stays
        # cleared so a caller's bumpless transfer still wins, and the
        # loop's first real update opens near the trim the model wants
        # rather than at zero.
        if output is None and self._adapt.enabled and self._base_terms.ki:
            trim = self._adapt.trim(self.channel, default=0.0)
            if trim:
                t = self.terms
                cap = t.integral_cap / t.ki
                self._i = max(-cap, min(cap, trim / t.ki))


# -- the fleet's stores -----------------------------------------------------

_STORE: dict[str, Adaptation] | None = None
_LAST_SAVE = 0.0
#: The store is written at most this often between saves; the end of a leg
#: forces one regardless.
SAVE_GAP_S = 60.0


def _load_store() -> dict[str, Adaptation]:
    global _STORE
    if _STORE is not None:
        return _STORE
    _STORE = {}
    try:
        raw = json.loads(_store_path().read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return _STORE
    if isinstance(raw, dict):
        for key, data in raw.items():
            if isinstance(data, dict):
                _STORE[str(key)] = Adaptation.from_dict(data)
    return _STORE


def for_aircraft(title: str, type_code: str,
                 enabled: bool = True) -> Adaptation:
    """The learned store for one aircraft model.

    Keyed on the simulator title where there is one -- "PMDG 737-800"
    and "Asobo 737 Max" learn separately -- and on the ICAO type where
    there is not, so two liveries of one add-on share what is learned.
    """
    if not enabled:
        return Adaptation(enabled=False)
    key = (title or "").strip() or (type_code or "").upper()
    if not key:
        return Adaptation(enabled=False)
    store = _load_store()
    found = store.get(key)
    if found is None:
        if len(store) >= MAX_MODELS:
            return Adaptation(enabled=False)
        found = store[key] = Adaptation()
    return found


def for_session(session, enabled: bool = True) -> Adaptation:
    """The store for the session's aircraft; a disabled one if unknown."""
    aircraft = getattr(session, "aircraft", None)
    return for_aircraft(getattr(aircraft, "title", "") or "",
                        getattr(aircraft, "type_code", "") or "",
                        enabled=enabled)


def save(force: bool = False) -> None:
    """Write the learned values that changed since the last write.

    Called once a minute while flying and on every handoff; a store
    that cannot be written is dropped on the floor, because a flight
    that is being handed back has no use for a file error.
    """
    global _LAST_SAVE
    if _STORE is None:
        return
    now = time.monotonic()
    if not force and now - _LAST_SAVE < SAVE_GAP_S:
        return
    if not any(a._touched for a in _STORE.values()):
        return
    _LAST_SAVE = now
    for a in _STORE.values():
        a._touched = False
    try:
        path = _store_path()
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(
            {k: a.as_dict() for k, a in _STORE.items()}, indent=1),
            encoding="utf-8")
        tmp.replace(path)
    except OSError:
        pass


def reset() -> None:
    """Forget everything learned; for tests."""
    global _STORE, _LAST_SAVE
    _STORE = None
    _LAST_SAVE = 0.0


__all__ = ["Adaptation", "AdaptivePid", "for_aircraft", "for_session",
           "save", "reset"]
