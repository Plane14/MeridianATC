from __future__ import annotations

import math

import pytest

from meridianatc.atc.trajectory import (
    AxisShaper,
    TrajectoryLimits,
    energy_speed_height,
    l1_guidance,
    speed_energy_ft,
    stopping_speed_kt,
)


def test_axis_shaper_smooths_a_step_and_converges():
    shaper = AxisShaper(TrajectoryLimits(
        max_velocity=5.0, max_acceleration=2.0, max_jerk=4.0,
        position_gain=1.0))
    positions = []
    velocities = []
    for _ in range(400):
        state = shaper.update(10.0, 0.05)
        positions.append(state.position)
        velocities.append(state.velocity)
    assert positions[-1] == pytest.approx(10.0, abs=0.1)
    assert max(velocities) <= 5.0 + 1e-6
    # The first response is deliberately not a step: jerk limiting starts
    # near zero acceleration.
    assert abs(velocities[0]) < 0.2


def test_axis_shaper_does_not_cross_the_target_in_one_sample():
    shaper = AxisShaper(TrajectoryLimits(
        max_velocity=20.0, max_acceleration=10.0, max_jerk=100.0),
        position=0.0, velocity=2.0)
    state = shaper.update(0.01, 0.1)
    assert state.position <= 0.01 + 1e-9


def test_l1_guidance_turns_back_toward_the_path():
    # Northbound leg at lon 0; aircraft east of it, so the intercept must be
    # left/west of north, not further east.
    guide = l1_guidance(0.0, 0.001, -0.01, 0.0, 0.01, 0.0,
                        90.0, period_s=20.0, damping=0.8)
    assert guide is not None
    assert guide.cross_track_m > 0.0
    assert 270.0 < guide.desired_track_deg < 360.0
    assert guide.bank_deg < 0.0
    assert abs(guide.lateral_accel_mss) <= 9.81


def test_l1_guidance_straightens_on_the_path():
    guide = l1_guidance(0.0, 0.0, -0.01, 0.0, 0.01, 0.0, 90.0)
    assert guide is not None
    assert guide.desired_track_deg == pytest.approx(0.0, abs=1.0)


def test_stopping_speed_schedule_is_continuous_and_capped():
    far = stopping_speed_kt(2.0, 10.0, 1.0, 90.0)
    near = stopping_speed_kt(0.1, 10.0, 1.0, 90.0)
    assert far > near >= 10.0
    assert stopping_speed_kt(10.0, 10.0, 1.0, 90.0) == 90.0
    assert stopping_speed_kt(0.0, 10.0, 1.0, 90.0) == 10.0


def test_stopping_speed_matches_constant_deceleration():
    speed = stopping_speed_kt(1.0, 10.0, 1.0, 200.0)
    # Time at the average speed covers the mile in roughly the same time the
    # deceleration needs; the exact equality is the kinematic equation.
    assert speed == pytest.approx(math.sqrt(100.0 + 7200.0), rel=1e-6)


def test_speed_error_has_height_equivalent_energy():
    assert speed_energy_ft(10.0, 150.0) > 0.0
    assert speed_energy_ft(-10.0, 150.0) < 0.0


def test_energy_allocation_balances_height_and_speed():
    cmd = energy_speed_height(200.0, 0.0, 150.0)
    assert cmd.throttle > 0.0
    assert cmd.pitch > 0.0
    assert cmd.priority == "balanced"


def test_underspeed_protection_is_speed_priority():
    cmd = energy_speed_height(200.0, -15.0, 150.0)
    assert cmd.priority == "speed"
    assert cmd.throttle >= 0.5
    assert cmd.pitch <= -0.15
