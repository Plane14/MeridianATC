"""Closed-loop axis controllers for the autonomous pilots.

What a first officer does with a hand on the lever, written down: a
proportional term for how far off the number is, an integral for the
trim the model never publishes, a derivative for the swing. Each axis
carries the protections that keep a bad model or a bad gain inside the
envelope: output clamps, a slew limit so the controls never jump, an
integral that stops winding when the output is already at its stop, a
derivative taken on the measurement so a step in the target does not
kick the aeroplane, and a deadband so a lever does not chase the last
knot forever.

The gains live in :mod:`meridianatc.atc.control_gains`, one bundle per
aircraft class, refinable per add-on through the autofly profiles.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass


@dataclass(frozen=True)
class PidTerms:
    """The three numbers a channel tunes with.

    Units are the caller's: a throttle channel is lever-fraction per
    knot of error, a collective channel per-cent of collective per foot.
    ``deadband`` is |error| below which nothing accumulates or moves;
    ``integral_cap`` bounds what the integrator may hold, in output
    units; ``low``/``high`` clamp the output; ``slew`` bounds how fast
    it may move, in output units per second; ``wrap`` names a periodic
    error span (360 for degrees) so the derivative does not see a jump
    where the angle wrapped.
    """

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    deadband: float = 0.0
    integral_cap: float = math.inf
    low: float = -math.inf
    high: float = math.inf
    slew: float = math.inf
    wrap: float | None = None


def _wrap_delta(delta: float, span: float | None) -> float:
    """A difference folded onto the short way round a periodic axis."""
    if span is None or span <= 0.0:
        return delta
    half = span / 2.0
    return (delta + half) % span - half


class Pid:
    """One axis of closed-loop control.

    ``update(error, dt=None, measurement=None)`` returns the output,
    clamped and slew-limited. ``dt`` may be omitted: the controller
    keeps its own clock then, which is what the per-tick call sites do.
    A stalled or negative ``dt`` contributes no integral or derivative,
    so a paused simulator does not poison the loop; a very large one is
    folded down rather than trusted.
    """

    # A dt beyond this is a pause/resume, not a sample: the integral is
    # trusted with no more than this, and the derivative is skipped.
    DT_MAX = 5.0

    def __init__(self, terms: PidTerms,
                 clock: Callable[[], float] = time.monotonic):
        self.terms = terms
        self._clock = clock
        self._i = 0.0
        self._prev_error: float | None = None
        self._prev_measure: float | None = None
        self._prev_t: float | None = None
        self._out: float | None = None

    @property
    def output(self) -> float | None:
        """The last output, or None before the first update."""
        return self._out

    def update(self, error: float, dt: float | None = None,
               measurement: float | None = None,
               rate: float | None = None) -> float:
        t = self.terms
        now = self._clock()
        if dt is None:
            dt = 0.0 if self._prev_t is None else now - self._prev_t
        self._prev_t = now
        fresh = 0.0 < dt <= self.DT_MAX
        if dt > self.DT_MAX:
            dt = self.DT_MAX

        # Proportional, deadbanded.
        err = 0.0 if abs(error) < t.deadband else error
        out = t.kp * err

        # Integral, with anti-windup by conditional integration: once the
        # output sits on a clamp and the error would push it further onto
        # it, the integrator holds what it has rather than wind a debt the
        # recovery would have to pay off.
        if t.ki and fresh:
            trial = self._i + err * dt
            # The cap is in output units and the integrator's own units
            # are error-seconds: what it may hold is the cap divided by
            # the gain -- a cap of one lever means ki*_i may reach one
            # lever, not that _i itself may reach one.
            cap = t.integral_cap / t.ki
            trial = max(-cap, min(cap, trial))
            would = t.kp * err + t.ki * trial
            saturating_up = t.high < math.inf and would >= t.high
            saturating_dn = t.low > -math.inf and would <= t.low
            pushing = (saturating_up and err > 0.0) \
                or (saturating_dn and err < 0.0)
            if not pushing:
                self._i = trial
        out += t.ki * self._i

        # Derivative on the measurement where one is given, so a step in
        # the target is a step, not a kick; on the error otherwise. The
        # wrap folds angle deltas, so 179° to -179° is two degrees of
        # movement, not a three-hundred-and-fifty-eight-degree swing. A
        # caller that already holds a rate in error units -- a vertical-
        # speed error under an altitude loop -- hands it in directly and
        # no differencing is done at all.
        if t.kd and fresh:
            if rate is not None:
                d = rate
            elif measurement is not None \
                    and self._prev_measure is not None:
                d = -_wrap_delta(measurement - self._prev_measure,
                                 t.wrap) / dt
            elif self._prev_error is not None:
                d = _wrap_delta(error - self._prev_error, t.wrap) / dt
            else:
                d = 0.0
            out += t.kd * d
        self._prev_error = error
        if measurement is not None:
            self._prev_measure = measurement

        # Output clamp, then the slew limit: the aeroplane gets what the
        # law asked for, but not faster than a hand could move the lever.
        out = max(t.low, min(t.high, out))
        if self._out is not None and t.slew < math.inf and fresh:
            step = t.slew * dt
            out = max(self._out - step, min(self._out + step, out))
            out = max(t.low, min(t.high, out))
        self._out = out
        return out

    __call__ = update

    def reset(self, output: float | None = None) -> None:
        """Drop the memory for a phase change or a bumpless transfer.

        ``output`` re-seeds where the loop starts: the last output, and
        -- where the law has an integral term -- the integrator itself,
        preloaded so the first update comes out at that output rather
        than at whatever the proportional term alone would say. Taking
        over a lever that is already somewhere starts there.
        """
        t = self.terms
        self._i = 0.0
        if output is not None and t.ki:
            self._i = output / t.ki
            cap = t.integral_cap / t.ki
            self._i = max(-cap, min(cap, self._i))
        self._prev_error = None
        self._prev_measure = None
        self._out = output


__all__ = ["Pid", "PidTerms"]
