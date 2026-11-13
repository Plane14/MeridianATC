from __future__ import annotations

import math

import pytest

from meridianatc.atc import autofly
from meridianatc.atc.control import Pid, PidTerms
from meridianatc.atc.control_gains import (
    FIXED_WING_KEYS,
    class_key,
    fixed_wing_gains,
    rotor_gains,
)
from meridianatc.atc.flying import AIRLINER, LIGHT, TURBOPROP


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def step(self, dt):
        self.t += dt


def drive(pid, plant, target, steps=400, dt=0.25, measurement=True):
    """Close a loop on a first-order plant and return the trace."""
    x = 0.0
    trace = []
    for _ in range(steps):
        out = pid.update(target - x, dt=dt,
                         measurement=x if measurement else None)
        x += out * dt
        trace.append(x)
    return x, trace


def test_proportional_converges_on_a_first_order_plant():
    pid = Pid(PidTerms(kp=0.5), clock=Clock())
    x, _ = drive(pid, None, 10.0)
    assert x == pytest.approx(10.0, abs=0.5)


def test_integral_removes_steady_state_error():
    # A plant with a constant disturbance: P alone leaves an offset, the
    # integrator closes it.
    pid = Pid(PidTerms(kp=0.4, ki=0.3), clock=Clock())
    x = 0.0
    for _ in range(800):
        out = pid.update(10.0 - x, dt=0.25, measurement=x)
        x += (out - 0.05) * 0.25        # constant load the P term cannot beat
    assert x == pytest.approx(10.0, abs=0.3)


def test_integral_does_not_wind_up_against_the_clamp():
    pid = Pid(PidTerms(kp=1.0, ki=1.0, low=0.0, high=1.0),
              clock=Clock())
    for _ in range(200):
        pid.update(100.0, dt=0.5)       # held on the stop the whole time
    # The error then steps to something the output can reach: a wound
    # integrator would keep the output pinned high for dozens of ticks.
    out = pid.update(0.5, dt=0.5)
    assert out <= 1.0
    seen = [pid.update(0.5, dt=0.5) for _ in range(20)]
    assert min(seen) < 0.9


def test_derivative_on_measurement_does_not_kick_on_a_setpoint_step():
    pid = Pid(PidTerms(kp=0.0, ki=0.0, kd=10.0), clock=Clock())
    x = 5.0
    pid.update(5.0 - x, dt=1.0, measurement=x)
    # The target steps by 50 with the plant unmoved: the derivative on the
    # measurement sees no movement, so nothing happens.
    kicked = pid.update(55.0 - x, dt=1.0, measurement=x)
    assert kicked == pytest.approx(0.0, abs=1e-9)


def test_derivative_on_measurement_damps_real_movement():
    pid = Pid(PidTerms(kp=0.0, ki=0.0, kd=10.0), clock=Clock())
    pid.update(0.0, dt=1.0, measurement=0.0)
    # The plant moved +2: the derivative opposes it.
    out = pid.update(0.0, dt=1.0, measurement=2.0)
    assert out < 0.0


def test_derivative_on_error_wraps_angles():
    pid = Pid(PidTerms(kp=0.0, kd=1.0, wrap=360.0), clock=Clock())
    pid.update(179.0, dt=1.0)
    # 179 to -179 is a two-degree move through the wrap, not a 358 swing.
    out = pid.update(-179.0, dt=1.0)
    assert out == pytest.approx(2.0, abs=0.01)


def test_output_clamps_hold():
    pid = Pid(PidTerms(kp=10.0, low=-0.5, high=0.5), clock=Clock())
    assert pid.update(100.0, dt=1.0) == 0.5
    assert pid.update(-100.0, dt=1.0) == -0.5


def test_slew_limit_bounds_output_rate():
    pid = Pid(PidTerms(kp=10.0, low=-10.0, high=10.0, slew=1.0),
              clock=Clock())
    pid.update(0.0, dt=1.0)
    out = pid.update(100.0, dt=0.25)
    # Asked for a jump of hundreds, allowed a quarter-unit step.
    assert out <= 0.25 + 1e-9


def test_deadband_holds_the_output():
    pid = Pid(PidTerms(kp=1.0, deadband=5.0), clock=Clock())
    for _ in range(10):
        assert pid.update(3.0, dt=1.0) == 0.0


def test_stalled_and_huge_dt_do_not_explode():
    pid = Pid(PidTerms(kp=0.5, ki=1.0, kd=1.0), clock=Clock())
    a = pid.update(10.0, dt=0.0)      # paused tick: only P may act
    assert a == pytest.approx(5.0)
    b = pid.update(10.0, dt=1e9)      # resume after forever: bounded
    assert math.isfinite(b)


def test_variable_dt_still_converges():
    pid = Pid(PidTerms(kp=0.4, ki=0.4), clock=Clock())
    x = 0.0
    for dt in (0.1, 0.5, 0.05, 1.0, 0.2):
        for _ in range(100):
            out = pid.update(8.0 - x, dt=dt, measurement=x)
            x += out * dt
    assert x == pytest.approx(8.0, abs=0.5)


def test_reset_drops_the_integral_and_history():
    pid = Pid(PidTerms(kp=0.5, ki=1.0, kd=1.0), clock=Clock())
    for _ in range(50):
        pid.update(10.0, dt=0.5)
    pid.reset()
    assert pid.output is None
    # After a reset the integrator is empty: the same first tick as a
    # fresh controller.
    fresh = Pid(PidTerms(kp=0.5, ki=1.0, kd=1.0), clock=Clock())
    assert pid.update(10.0, dt=0.5) == fresh.update(10.0, dt=0.5)


def test_reset_output_is_a_bumpless_transfer():
    pid = Pid(PidTerms(kp=0.01, ki=0.5, low=0.0, high=1.0), clock=Clock())
    # A lever already at 0.6, error inside nothing: the first update must
    # come out near 0.6, not at the proportional term's own answer.
    pid.reset(output=0.6)
    out = pid.update(0.0, dt=0.5)
    assert out == pytest.approx(0.6, abs=0.15)


# -- the gain tables ---------------------------------------------------------

def test_class_key_matches_the_performance_table():
    assert class_key(AIRLINER) == "airliner"
    assert class_key(TURBOPROP) == "turboprop"
    assert class_key(LIGHT) == "light"


def test_every_fixed_wing_bundle_carries_every_channel():
    for perf in (AIRLINER, TURBOPROP, LIGHT):
        gains = fixed_wing_gains(perf)
        for channel in FIXED_WING_KEYS:
            terms = getattr(gains, channel)
            assert isinstance(terms, PidTerms), channel
            assert terms.kp > 0.0 or terms.ki > 0.0, channel


def test_rotor_gains_default_and_twin():
    assert rotor_gains("R44") is rotor_gains("")
    assert rotor_gains("S76") is not rotor_gains("R44")
    for channel in ("collective", "cyclic", "pedals", "rotor"):
        assert getattr(rotor_gains("B412"), channel).kp != 0.0


def test_handling_profile_refines_a_channel(monkeypatch):
    prof = {"titleMatches": ["a320 test"], "handling": {
        "throttle": {"kp": 0.05}}}
    monkeypatch.setattr(autofly, "_profiles", [prof])
    monkeypatch.setattr(autofly, "_loaded", True)
    base = fixed_wing_gains(AIRLINER)
    tuned = autofly.handling("A320", "a320 test add-on", base)
    assert tuned.throttle.kp == 0.05
    assert tuned.steer.kp == base.steer.kp


def test_handling_profile_ignores_unknown_and_missing(monkeypatch):
    prof = {"titleMatches": ["a320 test"], "handling": {
        "not_a_channel": {"kp": 9.0}, "throttle": "not-dict"}}
    monkeypatch.setattr(autofly, "_profiles", [prof])
    monkeypatch.setattr(autofly, "_loaded", True)
    base = fixed_wing_gains(AIRLINER)
    assert autofly.handling("A320", "a320 test", base) is base
    # A profile for another aeroplane leaves the bundle alone too.
    assert autofly.handling("C172", "cessna", base) is base
