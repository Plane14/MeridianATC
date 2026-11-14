"""The autonomous stall watch and recovery -- without a simulator."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from meridianatc.atc.flying import AIRLINER
from meridianatc.sim.state import AircraftState


def _fo(perf=AIRLINER, **kw):
    """A First Officer built by hand: the state, the director mock the
    axes go through, and no helicopter."""
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.sim = None
    fo._dir = Mock()
    fo._dir.sim = None
    fo._heli = None
    fo._ap_mode = ""
    fo._perf = perf
    fo.session = SimpleNamespace(
        aircraft=None, assigned_heading=None, assigned_altitude_ft=0,
        assigned_speed_kt=0, cruise_altitude_ft=0, initial_altitude_ft=0)
    fo.on_event = Mock()
    fo._last_lever = 0.0
    for key, value in kw.items():
        setattr(fo, key, value)
    return fo


def _emit_keys(fo):
    return [call.kwargs.get("key") for call in fo.on_event.call_args_list]


def _airborne(**kw):
    kw.setdefault("on_ground", False)
    kw.setdefault("altitude_agl_ft", 5000.0)
    kw.setdefault("indicated_airspeed_kt", 250.0)
    return AircraftState(**kw)


# -- detection -----------------------------------------------------------------


def test_the_horn_held_hands_the_axes_to_the_recovery():
    fo = _fo()
    state = _airborne(stall_warning=True)
    fo._monitor_stall(state, now=10.0)
    assert not fo.state.stall_recovering       # first beat starts the clock
    fo._monitor_stall(state, now=10.3)
    assert not fo.state.stall_recovering       # still inside a gust
    fo._monitor_stall(state, now=10.6)
    assert fo.state.stall_recovering
    assert "autonomous.stall.enter" in _emit_keys(fo)
    fo._dir.autopilot.assert_called_with(False)
    fo._dir.throttle.assert_called_with(1.0)
    fo._dir.elevator.assert_called()


def test_a_horn_blip_is_just_a_blip():
    fo = _fo()
    fo._monitor_stall(_airborne(stall_warning=True), now=10.0)
    fo._monitor_stall(_airborne(stall_warning=False), now=10.2)
    assert fo.state.stall_since == 0.0
    fo._monitor_stall(_airborne(stall_warning=False), now=11.0)
    assert not fo.state.stall_recovering


def test_the_horn_in_the_flare_means_nothing():
    fo = _fo()
    state = _airborne(stall_warning=True, altitude_agl_ft=30.0,
                      indicated_airspeed_kt=150.0)
    for beat in range(6):
        fo._monitor_stall(state, now=10.0 + beat * 0.25)
    assert not fo.state.stall_recovering


def test_alpha_over_the_warning_angle_triggers():
    fo = _fo()
    state = _airborne(aoa_deg=15.5, stall_aoa_deg=15.0)
    fo._monitor_stall(state, now=10.0)
    fo._monitor_stall(state, now=10.6)
    assert fo.state.stall_recovering
    assert "autonomous.stall.enter" in _emit_keys(fo)


def test_speed_under_the_book_figure_alone_triggers_slowly():
    fo = _fo()
    # The airliner's book says VS1 130: 97% of it is 126.1.
    state = _airborne(indicated_airspeed_kt=120.0, design_vs1_kt=130.0)
    fo._monitor_stall(state, now=10.0)
    fo._monitor_stall(state, now=10.8)
    assert not fo.state.stall_recovering       # the speed asks for longer
    fo._monitor_stall(state, now=11.1)
    assert fo.state.stall_recovering


def test_no_published_figure_and_a_live_speed_means_no_watch():
    fo = _fo()
    # Nothing publishes a stall figure and the speed is healthy: the
    # watch has nothing to say.
    state = _airborne(indicated_airspeed_kt=250.0)
    state.unavailable = ("stall_warning", "aoa_deg", "stall_aoa_deg")
    for beat in range(6):
        fo._monitor_stall(state, now=10.0 + beat * 0.5)
    assert not fo.state.stall_recovering


def test_the_kill_switch_stands_the_watch_down():
    fo = _fo(config=SimpleNamespace(
        immersion=SimpleNamespace(autonomous_envelope_protection=False)))
    state = _airborne(stall_warning=True)
    for beat in range(6):
        fo._monitor_stall(state, now=10.0 + beat * 0.5)
    assert not fo.state.stall_recovering


# -- the recovery itself --------------------------------------------------------


def test_the_recovery_pushes_powers_and_levels_the_wings():
    fo = _fo()
    state = _airborne(stall_warning=True, bank_deg=30.0)
    fo._monitor_stall(state, now=10.0)
    fo._monitor_stall(state, now=10.6)
    fo._fly_stall_recovery(state, now=10.7)
    fo._dir.elevator.assert_called_with(-0.45)
    fo._dir.spoilers.assert_called_with(0.0)
    fo._dir.rudder.assert_called_with(0.0)
    # 30 degrees right: the counter asks 0.45 left.
    assert fo._dir.aileron.call_args.args[0] == pytest.approx(-0.45)


def test_the_wings_level_ask_caps_at_a_firm_rate():
    fo = _fo()
    fo._fly_stall_recovery(_airborne(bank_deg=-80.0), now=10.0)
    fo._dir.aileron.assert_called_with(0.6)


def test_instruments_agreeing_hands_the_aeroplane_back():
    fo = _fo()
    stalled = _airborne(stall_warning=True)
    fo._monitor_stall(stalled, now=10.0)
    fo._monitor_stall(stalled, now=10.6)
    assert fo.state.stall_recovering
    # The horn goes quiet and the speed is back: a moment of agreement
    # is not yet a recovery.
    flying = _airborne(stall_warning=False, indicated_airspeed_kt=200.0)
    fo._monitor_stall(flying, now=11.0)
    fo._fly_stall_recovery(flying, now=11.0)
    assert fo.state.stall_recovering
    fo._monitor_stall(flying, now=12.0)
    assert fo.state.stall_recovering            # inside the clear window
    fo._monitor_stall(flying, now=12.6)
    assert not fo.state.stall_recovering
    assert "autonomous.stall.recovered" in _emit_keys(fo)
    fo._dir.center_controls.assert_called()


def test_one_disagreeing_instrument_keeps_the_recovery():
    fo = _fo()
    stalled = _airborne(stall_warning=True)
    fo._monitor_stall(stalled, now=10.0)
    fo._monitor_stall(stalled, now=10.6)
    assert fo.state.stall_recovering
    # Horn quiet but the speed is still under the book figure: not yet.
    slow = _airborne(stall_warning=False, indicated_airspeed_kt=120.0,
                     design_vs1_kt=130.0)
    fo._monitor_stall(slow, now=11.0)
    fo._monitor_stall(slow, now=13.0)
    assert fo.state.stall_recovering


def test_the_ground_ends_the_recovery_it_could_not():
    fo = _fo()
    stalled = _airborne(stall_warning=True)
    fo._monitor_stall(stalled, now=10.0)
    fo._monitor_stall(stalled, now=10.6)
    assert fo.state.stall_recovering
    fo._monitor_stall(AircraftState(on_ground=True), now=12.0)
    assert not fo.state.stall_recovering
    assert "autonomous.stall.ground" in _emit_keys(fo)


def test_every_trigger_kind_names_what_called_it():
    fo = _fo()
    assert fo._stall_trigger(_airborne(stall_warning=True)) == "horn"
    assert fo._stall_trigger(
        _airborne(aoa_deg=16.0, stall_aoa_deg=15.0)) == "alpha"
    assert fo._stall_trigger(_airborne(
        indicated_airspeed_kt=120.0, design_vs1_kt=130.0)) == "speed"
    assert fo._stall_trigger(_airborne()) == ""
