"""Rotor-speed governing helpers for autonomous rotorcraft.

The shape of the controller follows mature traditional-helicopter governors:
a scheduled/base throttle demand, collective feedforward, bounded droop and
overspeed correction, slow trim with anti-windup, and explicit persistent
underspeed/overspeed faults. It protects the deterministic flight stage; it
is not an unconstrained engine controller.
"""

from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class RotorGovernorOutput:
    """One governor pass."""

    throttle: float          # 0..1 output
    correction: float        # dynamic plus trim correction over the base
    trim: float              # learned steady correction
    fault: str = ""          # underspeed | overspeed | ""


class RotorSpeedGovernor:
    """Bounded rotor-speed feedback around a throttle curve.

    A zero/unpublished rotor speed means "no evidence" and leaves the base
    command unchanged. Persistent limit excursions are faults only after
    ``fault_after_s``, matching the way real governor logic rejects transient
    sensor noise.
    """

    def __init__(self, *, target_pct: float = 100.0,
                 min_pct: float = 95.0, max_pct: float = 104.0,
                 deadband_pct: float = 2.0, proportional: float = 0.006,
                 trim_rate: float = 0.002, feedforward: float = 0.001,
                 max_corr: float = 0.15, fault_after_s: float = 0.5):
        if min_pct >= max_pct or target_pct <= 0.0:
            raise ValueError("rotor limits must bracket a positive target")
        self.target_pct = target_pct
        self.min_pct = min_pct
        self.max_pct = max_pct
        self.deadband_pct = max(0.0, deadband_pct)
        self.proportional = max(0.0, proportional)
        self.trim_rate = max(0.0, trim_rate)
        self.feedforward = max(0.0, feedforward)
        self.max_corr = max(0.0, max_corr)
        self.fault_after_s = max(0.0, fault_after_s)
        self.trim = 0.0
        self._low_s = 0.0
        self._high_s = 0.0

    def reset(self) -> None:
        self.trim = 0.0
        self._low_s = 0.0
        self._high_s = 0.0

    def update(self, base: float, rpm_pct: float, dt: float,
               *, collective_pct: float = 50.0) -> RotorGovernorOutput:
        base = clamp(float(base), 0.0, 1.0)
        rpm = float(rpm_pct or 0.0)
        dt = clamp(float(dt or 0.0), 0.0, 5.0)
        if rpm <= 1.0 or dt <= 0.0:
            self._low_s = 0.0
            self._high_s = 0.0
            return RotorGovernorOutput(base, 0.0, self.trim)

        error = self.target_pct - rpm
        feedback_error = 0.0 if abs(error) <= self.deadband_pct else error
        proportional = self.proportional * feedback_error
        feedforward = (self.feedforward
                       * clamp((collective_pct - 50.0) / 10.0, -2.0, 2.0))
        if feedback_error and rpm < self.max_pct and rpm > self.min_pct:
            self.trim = clamp(self.trim + feedback_error * self.trim_rate * dt,
                              -self.max_corr, self.max_corr)
        correction = clamp(proportional + feedforward + self.trim,
                           -self.max_corr, self.max_corr)

        if rpm < self.min_pct:
            self._low_s += dt
            self._high_s = 0.0
        elif rpm > self.max_pct:
            self._high_s += dt
            self._low_s = 0.0
        else:
            self._low_s = 0.0
            self._high_s = 0.0
        fault = ""
        if self._low_s >= self.fault_after_s:
            fault = "underspeed"
            correction = max(correction, self.max_corr * 0.75)
        elif self._high_s >= self.fault_after_s:
            fault = "overspeed"
            correction = min(correction, -self.max_corr * 0.75)

        throttle = clamp(base + correction, 0.0, 1.0)
        # Anti-windup: pinned output does not continue learning in the same
        # direction. The increment is small by design; this simply prevents a
        # long saturation from becoming a delayed throttle jump.
        if throttle in (0.0, 1.0) and feedback_error:
            pushing_up = throttle == 1.0 and feedback_error > 0.0
            pushing_down = throttle == 0.0 and feedback_error < 0.0
            if pushing_up or pushing_down:
                self.trim = clamp(
                    self.trim - feedback_error * self.trim_rate * dt,
                    -self.max_corr, self.max_corr)
        return RotorGovernorOutput(throttle=throttle,
                                   correction=correction,
                                   trim=self.trim,
                                   fault=fault)


__all__ = ["RotorGovernorOutput", "RotorSpeedGovernor"]
