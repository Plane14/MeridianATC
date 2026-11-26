from __future__ import annotations

import pytest

from meridianatc.atc.rotor import RotorSpeedGovernor


def governor():
    return RotorSpeedGovernor(target_pct=100.0, min_pct=95.0,
                              max_pct=104.0, deadband_pct=1.0,
                              proportional=0.01, trim_rate=0.0,
                              max_corr=0.2, fault_after_s=0.5)


def test_unreported_rotor_speed_leaves_the_base_throttle():
    out = governor().update(0.75, 0.0, 0.25)
    assert out.throttle == 0.75
    assert out.correction == 0.0
    assert not out.fault


def test_in_band_rotor_speed_leaves_the_base_throttle():
    out = governor().update(0.75, 100.2, 0.25)
    assert out.throttle == pytest.approx(0.75)
    assert not out.fault


def test_droop_adds_bounded_power():
    gov = governor()
    out = gov.update(0.75, 97.0, 0.25, collective_pct=70.0)
    assert out.throttle > 0.75
    assert out.correction <= 0.2


def test_overspeed_removes_power():
    out = governor().update(0.75, 105.0, 0.25, collective_pct=50.0)
    assert out.throttle < 0.75


def test_persistent_droop_is_a_fault_and_demands_power():
    gov = governor()
    gov.update(0.75, 94.0, 0.3)
    out = gov.update(0.75, 94.0, 0.3)
    assert out.fault == "underspeed"
    assert out.correction >= 0.15


def test_persistent_overspeed_is_a_fault_and_unloads_power():
    gov = governor()
    gov.update(0.75, 105.0, 0.3)
    out = gov.update(0.75, 105.0, 0.3)
    assert out.fault == "overspeed"
    assert out.correction <= -0.15


def test_transient_excursion_does_not_fault():
    gov = governor()
    out = gov.update(0.75, 94.0, 0.2)
    assert not out.fault


def test_output_never_leaves_the_lever_range():
    gov = governor()
    low = gov.update(0.05, 105.0, 0.25)
    high = gov.update(0.98, 90.0, 0.25)
    assert 0.0 <= low.throttle <= 1.0
    assert 0.0 <= high.throttle <= 1.0
