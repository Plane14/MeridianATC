"""Simulator-AI delegation must not swallow active ATC instructions."""

from __future__ import annotations

from types import SimpleNamespace

from meridianatc.atc.autonomous import AutonomousFlight, AutonomousPhase


def _fo(**session_values):
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo._use_sim_ai = True
    fo._is_helicopter = False
    fo.state = SimpleNamespace(
        phase=AutonomousPhase.CRUISE,
        sub="",
        stall_recovering=False,
        emergency=False,
        emergency_descent=False,
        avoid_heading=-1.0,
        direct_fix="")
    session = SimpleNamespace(
        assigned_heading=None,
        route_direct="",
        hold=None,
        weather_deviation_at=0.0)
    for name, value in session_values.items():
        setattr(session, name, value)
    fo.session = session
    return fo


def test_quiet_cruise_may_be_delegated():
    state = SimpleNamespace()
    assert _fo()._sim_ai_wanted(state)


def test_a_direct_clearance_is_flown_by_the_first_officer():
    # The delegated sim pilot is never told that the route was cut; while the
    # direct stands, the FO must keep the axes so _fly_direct can steer it.
    state = SimpleNamespace()
    assert not _fo(route_direct="LAMur")._sim_ai_wanted(state)


def test_a_hold_instruction_is_flown_by_the_first_officer():
    # A racetrack is Meridian's own steering, not something X-Plane's AI pilot
    # knows it has been cleared into.
    state = SimpleNamespace()
    hold = SimpleNamespace(fix="ABC")
    assert not _fo(hold=hold)._sim_ai_wanted(state)


def test_the_shortcut_remainder_still_cannot_be_delegated():
    # route_direct clears at the cleared fix, but the rest of the shortened
    # route is still flown by _fly_direct because the simulator FMS was never
    # rewritten. Delegating here would turn back onto the legs ATC cut.
    fo = _fo()
    fo.state.direct_fix = "NEXUS"
    assert not fo._sim_ai_wanted(SimpleNamespace())
