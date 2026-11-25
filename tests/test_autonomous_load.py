"""The autonomous load-factor watch, the demeanor it feeds, and the
rotorcraft's rotor-load protection -- without a simulator."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from meridianatc.atc import autofly
from meridianatc.atc.flying import AIRLINER, LIGHT, Performance
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


# -- the limits --------------------------------------------------------------


def test_limits_carry_the_class_g_figures():
    fo = _fo()
    lim = fo._limits(AircraftState(on_ground=False))
    assert lim["g_pos"] == 2.5 and lim["g_neg"] == -1.0
    fo._perf = LIGHT
    lim = fo._limits(AircraftState(on_ground=False))
    assert lim["g_pos"] == 3.8 and lim["g_neg"] == -1.5


def test_limits_fall_back_to_the_class_when_the_profile_is_silent():
    # A Performance that declares no G gets the ladder the cruise figure
    # sorts by: transport numbers over 140 kt, the light book under it.
    fo = _fo(perf=Performance(cruise_kt=280.0))
    assert fo._limits(AircraftState())["g_pos"] == 2.5
    fo._perf = Performance(cruise_kt=110.0)
    lim = fo._limits(AircraftState())
    assert (lim["g_pos"], lim["g_neg"]) == (3.8, -1.5)


def test_autofly_profile_g_limits(monkeypatch):
    monkeypatch.setattr(autofly, "_loaded", True)
    monkeypatch.setattr(autofly, "_profiles", [{
        "canAutofly": True,
        "titleMatches": ["Edge 540"],
        "performance": {"positiveGLimit": 10.0 - 1.0,
                        "negativeGLimit": -4.0,
                        "cruiseSpeedKnots": 200.0},
    }])
    perf = autofly.override("E540", "Zivko Edge 540", AIRLINER)
    assert perf.pos_g == 9.0 and perf.neg_g == -4.0
    assert perf.cruise_kt == 200.0
    # A profile that says nothing about G leaves the class figure alone.
    monkeypatch.setattr(autofly, "_profiles", [{
        "canAutofly": True,
        "titleMatches": ["Edge 540"],
        "performance": {"cruiseSpeedKnots": 200.0},
    }])
    assert autofly.override("E540", "Edge 540", AIRLINER).pos_g == 2.5


# -- the watch ---------------------------------------------------------------


def _airborne(g, **kw):
    return AircraftState(on_ground=False, g_force=g, **kw)


def test_soft_margin_flags_and_warns_before_the_limit():
    fo = _fo()
    fo._load_on_ground = False
    # 85% of +2.5 is 2.125: over the soft margin, under the book figure.
    fo._monitor_load(_airborne(2.3), now=10.0)
    assert fo.state.g_limited and fo.state.g_warned
    assert "autonomous.load.g_near" in _emit_keys(fo)
    fo._dir.vertical_speed.assert_not_called()


def test_exceedance_levels_the_vs_bug_and_caps_the_bank():
    fo = _fo()
    fo._load_on_ground = False
    fo._monitor_load(_airborne(2.8), now=10.0)
    assert fo.state.g_limited
    assert "autonomous.load.g_limit" in _emit_keys(fo)
    fo._dir.vertical_speed.assert_called_with(0.0)
    fo._dir.max_bank.assert_called_with(15)


def test_negative_g_is_watched_too():
    fo = _fo()
    fo._load_on_ground = False
    fo._monitor_load(_airborne(-1.3), now=10.0)
    assert fo.state.g_limited
    assert "autonomous.load.g_limit" in _emit_keys(fo)


def test_hand_flown_subs_take_the_pull_cap_not_the_bug():
    fo = _fo()
    fo._load_on_ground = False
    fo.state.sub = "rotate"
    fo._monitor_load(_airborne(2.8), now=10.0)
    assert fo.state.g_limited
    fo._dir.vertical_speed.assert_not_called()
    assert fo._g_pull_cap(_airborne(2.8), 0.55) == pytest.approx(0.10)
    assert fo._g_pull_cap(_airborne(1.0), 0.55) == pytest.approx(0.10)
    fo.state.g_limited = False
    assert fo._g_pull_cap(_airborne(1.0), 0.55) == pytest.approx(0.55)


def test_the_flag_clears_after_three_seconds_back_inside():
    fo = _fo()
    fo._load_on_ground = False
    fo._monitor_load(_airborne(2.8), now=10.0)
    assert fo.state.g_limited
    fo._monitor_load(_airborne(1.0), now=12.0)
    assert fo.state.g_limited            # still inside the hysteresis
    fo._monitor_load(_airborne(1.0), now=13.5)
    assert not fo.state.g_limited and not fo.state.g_warned


def test_no_g_reading_means_no_watch_and_no_invented_numbers():
    fo = _fo()
    fo._load_on_ground = False
    state = _airborne(9.9)
    state.unavailable = ("g_force",)
    fo._monitor_load(state, now=10.0)
    assert not fo.state.g_limited
    # The default 1.0 a silent model reads is inside every limit.
    fo._monitor_load(_airborne(1.0), now=10.0)
    assert not fo.state.g_limited


def test_the_kill_switch_stands_the_watch_down():
    fo = _fo(config=SimpleNamespace(
        immersion=SimpleNamespace(autonomous_envelope_protection=False)))
    fo._load_on_ground = False
    fo._monitor_load(_airborne(9.9), now=10.0)
    assert not fo.state.g_limited
    assert fo._g_pull_cap(_airborne(9.9), 0.55) == pytest.approx(0.55)


def test_touchdown_g_is_reported_once_on_the_edge():
    fo = _fo()
    fo._load_on_ground = False           # was airborne
    fo._monitor_load(AircraftState(on_ground=True, g_force=2.5), now=10.0)
    assert fo.state.touchdown_g == 2.5
    assert "autonomous.load.hard_landing" in _emit_keys(fo)
    fo.on_event.reset_mock()
    # Still on the ground: no second report off the same touchdown.
    fo._monitor_load(AircraftState(on_ground=True, g_force=2.5), now=10.5)
    assert not _emit_keys(fo)
    # A gentle arrival is logged but not called out.
    fo._load_on_ground = False
    fo._monitor_load(AircraftState(on_ground=True, g_force=1.4), now=11.0)
    assert fo.state.touchdown_g == 1.4
    assert not _emit_keys(fo)


# -- the demeanor and the slew -------------------------------------------------


def test_demeanor_picks_urgent_gentle_smooth():
    fo = _fo()
    state = AircraftState()
    assert fo._demeanor(state) == "smooth"
    fo.state.sub = "flare"
    assert fo._demeanor(state) == "gentle"
    fo.state.emergency = "engine"
    assert fo._demeanor(state) == "urgent"


def test_vs_bug_slews_to_the_demand_at_the_demeanors_rate():
    fo = _fo()
    state = AircraftState()
    # Smooth: 400 fpm/s, so a 2,000 fpm ask moves 100 in a quarter second.
    assert fo._vs_slew(state, 2000.0, now=10.0) == pytest.approx(0.0)
    assert fo._vs_slew(state, 2000.0, now=10.25) == pytest.approx(100.0)
    assert fo._vs_slew(state, 2000.0, now=11.25) == pytest.approx(500.0)
    fo.state.emergency = "engine"
    # Urgent: 1,500 fpm/s -- three quarters of a second moves 1,125
    # toward the new demand.
    assert fo._vs_slew(state, -2500.0, now=12.0) == pytest.approx(-625.0)


def test_the_limiter_wins_the_bug_while_the_flag_is_up():
    fo = _fo()
    fo.state.g_limited = True
    state = AircraftState()
    assert fo._vs_slew(state, 3000.0, now=10.0) == pytest.approx(0.0)


def test_apply_flight_director_slews_the_vs_and_caps_the_bank():
    import time
    fo = _fo()
    state = AircraftState(on_ground=False, autopilot=True)
    # The bug was touched half a second ago, so this pass moves it part
    # of the way to the demand rather than stepping it there.
    fo.state.vs_bug_at = time.monotonic() - 0.5
    fo._apply_flight_director(state, altitude=5000.0, vs_fpm=2000.0,
                              speed_kt=250.0)
    sent = fo._dir.vertical_speed.call_args.args[0]
    assert 0.0 < sent < 2000.0
    assert fo.state.vs_bug_cmd == sent
    fo._dir.max_bank.assert_called_with(20.0)
    fo._dir.yaw_damper.assert_called_with(True)


def test_the_bank_cap_is_sent_once_per_change():
    fo = _fo()
    fo._cap_bank(30.0)
    fo._cap_bank(30.0)
    fo._dir.max_bank.assert_called_once_with(30)
    fo._cap_bank(15.0)
    assert fo._dir.max_bank.call_count == 2
    # A model without the event still records the ask -- no retry storm.
    fo._dir.max_bank.return_value = False
    for _ in range(5):
        fo._cap_bank(15.0)
    assert fo._dir.max_bank.call_count == 2


# -- the vertical-mode exclusivity ---------------------------------------------


def _managed_fo():
    """A first officer in managed AP mode, descending."""
    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _fo()
    fo._ap_mode = "managed"
    fo.state.phase = AutonomousPhase.DESCENT
    return fo


def _probe_sim(fo, status):
    """A sim source with a mode-status readback, on both the officer
    and the director so the director property keeps the mock."""
    sim = Mock()
    sim.value = Mock(return_value=status)
    fo.sim = sim
    fo._dir.sim = sim
    return sim


def test_managed_flc_keeps_the_vs_hold_off_the_axis():
    """Level-change read back engaged: it owns the vertical axis and the
    VS hold stays off -- both asserted was the FL/VS flicker the
    descent flew the whole way down."""
    fo = _managed_fo()
    _probe_sim(fo, 1.0)                            # speed_status: FLC on
    state = AircraftState(on_ground=False, autopilot=True)
    fo._apply_flight_director(state, altitude=3000.0, vs_fpm=-800.0,
                              speed_kt=180.0)
    fo._dir.vs_hold.assert_called_with(False)
    fo._dir.level_change.assert_called_with(True)
    assert fo.state.ap_vs_mode is False


def test_the_probe_holds_vs_off_until_the_mode_answers():
    """While the engage is still out nothing asserts the axis beside it
    -- a VS engage now would knock the asked-for mode off before its
    status could ever read back."""
    fo = _managed_fo()
    _probe_sim(fo, 0.0)                            # nothing engaged yet
    state = AircraftState(on_ground=False, autopilot=True)
    fo._apply_flight_director(state, altitude=3000.0, vs_fpm=-800.0,
                              speed_kt=180.0)
    fo._dir.vs_hold.assert_called_with(False)
    fo._dir.level_change.assert_called_with(True)


def test_a_model_without_level_change_is_flown_on_vs():
    """The probe times out refused: the engage stops knocking the VS
    hold off the axis and VS is the vertical mode from then on."""
    import time
    fo = _managed_fo()
    _probe_sim(fo, 0.0)
    state = AircraftState(on_ground=False, autopilot=True)
    now = time.monotonic()
    assert fo._flc_owning(state, now) is None          # probe armed
    assert fo._flc_owning(state, now + 0.5) is None    # still asking
    assert fo._flc_owning(state, now + 3.0) is False   # refused
    asks = fo._dir.level_change.call_count
    assert fo._flc_owning(state, now + 4.0) is False   # the VS era
    assert fo._dir.level_change.call_count == asks     # asked no more


def test_no_readback_keeps_the_both_asked_ordering():
    """A source with no mode readback at all keeps the legacy path --
    VS asserted alongside the level-change ask, and the last write wins
    on models that have the mode."""
    fo = _managed_fo()
    # fo._dir.sim is None -- no source, no readback on it.
    state = AircraftState(on_ground=False, autopilot=True)
    fo._apply_flight_director(state, altitude=3000.0, vs_fpm=-800.0,
                              speed_kt=180.0)
    fo._dir.vs_hold.assert_called_with(True)
    fo._dir.level_change.assert_called_with(True)
    assert fo.state.ap_vs_mode is True


def test_the_direct_mode_still_flies_the_vs_bug():
    """Direct AP mode never touches the level-change mode."""
    fo = _fo()
    state = AircraftState(on_ground=False, autopilot=True)
    fo._apply_flight_director(state, altitude=3000.0, vs_fpm=-800.0,
                              speed_kt=180.0)
    fo._dir.vs_hold.assert_called_with(True)
    fo._dir.level_change.assert_not_called()
    assert fo.state.ap_vs_mode is True


def _xp_sim(fo, status):
    """An X-Plane source: its level-change pitches for speed and its
    mode statuses run armed at one, engaged at two."""
    sim = _probe_sim(fo, status)
    sim.flc_pitch_only = True
    sim.ap_status_engaged = 2.0
    return sim


def test_xplanes_pitch_only_flc_is_never_asked():
    """X-Plane's level-change pitches for speed -- the same loop the
    throttle manager already runs, and two controllers on one
    airspeed porpoised the aeroplane into terrain. VS owns the axis
    outright; the mode is not even asked."""
    fo = _managed_fo()
    _xp_sim(fo, 0.0)                                 # no mode on
    state = AircraftState(on_ground=False, autopilot=True)
    fo._apply_flight_director(state, altitude=8000.0, vs_fpm=+700.0,
                              speed_kt=90.0)
    fo._dir.vs_hold.assert_called_with(True)
    fo._dir.level_change.assert_not_called()
    assert fo.state.ap_vs_mode is True


def test_xplanes_flc_is_brought_off_before_vs_takes_the_axis():
    """FLC still engaged is disengaged under the same two-second
    probe, reversed, before VS asserts the axis -- both asserted a
    write apart is the flicker the probe exists to stop. Expired
    unanswered, the VS own stays latched -- re-probing a mode that
    will not drop would pulse the axis every other tick."""
    import time
    fo = _managed_fo()
    _xp_sim(fo, 2.0)                                 # FLC still engaged
    state = AircraftState(on_ground=False, autopilot=True)
    now = time.monotonic()
    assert fo._flc_owning(state, now) is None            # coming off
    fo._dir.level_change.assert_called_with(False)
    assert fo._flc_owning(state, now + 1.0) is None
    assert fo._flc_owning(state, now + 3.0) is False
    assert fo._flc_owning(state, now + 4.0) is False      # latched
    fo._flc_off_at = time.monotonic() - 3.0          # probe out in real time
    state2 = AircraftState(on_ground=False, autopilot=True)
    fo._apply_flight_director(state2, altitude=3000.0, vs_fpm=-800.0,
                              speed_kt=180.0)
    fo._dir.vs_hold.assert_called_with(True)
    assert fo.state.ap_vs_mode is True


def test_xplanes_armed_flc_is_not_ownership():
    """An armed status is one on X-Plane, and one is not engaged: an
    armed-but-flying-nothing mode holds nothing off the axis -- VS
    owns from the first look."""
    fo = _managed_fo()
    _xp_sim(fo, 1.0)                                 # armed, not engaged
    state = AircraftState(on_ground=False, autopilot=True)
    assert fo._flc_owning(state, __import__("time").monotonic()) is False
    fo._dir.level_change.assert_not_called()
    fo._apply_flight_director(state, altitude=3000.0, vs_fpm=-800.0,
                              speed_kt=180.0)
    fo._dir.vs_hold.assert_called_with(True)
    assert fo.state.ap_vs_mode is True


def test_msfs_engaged_flc_keeps_the_descent():
    """Status nonzero is the mode flying on the boolean readback: on
    the descent leg that is the level-change doing its work, and VS
    stays off the axis. X-Plane no longer reaches this path at all."""
    fo = _managed_fo()
    _probe_sim(fo, 1.0)                              # engaged (MSFS bool)
    state = AircraftState(on_ground=False, autopilot=True)
    fo._apply_flight_director(state, altitude=3000.0, vs_fpm=-800.0,
                              speed_kt=180.0)
    fo._dir.vs_hold.assert_called_with(False)
    fo._dir.level_change.assert_called_with(True)
    assert fo.state.ap_vs_mode is False


# -- the rotorcraft ------------------------------------------------------------


def _heli(profile=None, stage="cruise", protection=True):
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import AutonomousState
    from meridianatc.atc.autonomous_heli import (
        DEFAULT_HELI_PROFILE,
        HeliState,
        RotorcraftFlight,
    )
    owner = SimpleNamespace(
        config=SimpleNamespace(
            immersion=SimpleNamespace(autonomous_pid=False)),
        session=SimpleNamespace(aircraft=None),
        sim=None, get_state=lambda: None, crew=Mock(), on_event=Mock(),
        state=AutonomousState(),
        _send_event=Mock(return_value=True),
        _emit=Mock(), _engines_out=lambda state: (0, 0),
        _use_pids=False,
        _envelope_protection=protection)
    heli = RotorcraftFlight(owner)
    heli.state = HeliState(stage=stage, coll_trim=50.0)
    heli._profile = profile or DEFAULT_HELI_PROFILE
    return heli


def test_heli_offloads_the_rotor_at_high_g():
    from meridianatc.atc.autonomous_heli import COLLECTIVE_SET
    heli = _heli()
    heli._protect(_airborne(2.4), vs=0.0, ias=80.0, agl=1000.0)
    # 0.85 * 2.5 = 2.125: over the margin -- the collective eases off trim.
    sent = heli.owner._send_event.call_args_list
    assert any(call.args[0] == COLLECTIVE_SET and call.args[1] < 8192
               for call in sent)
    keys = [call.kwargs.get("key") for call in heli.owner._emit.call_args_list]
    assert "autonomous.rotorcraft.g_limit" in keys


def test_heli_recovers_low_g_with_gentle_aft_cyclic():
    from meridianatc.atc.autonomous_heli import CYCLIC_LONG_SET, HeliProfile
    # A teetering profile: the low-G floor is the Robinson's 0.4.
    profile = HeliProfile(110, 80, 1000, 2000, 140, 14000, 45, -300.0,
                          low_g=0.4)
    heli = _heli(profile=profile)
    heli._protect(_airborne(0.1), vs=0.0, ias=80.0, agl=1000.0)
    sent = heli.owner._send_event.call_args_list
    assert any(call.args[0] == CYCLIC_LONG_SET and call.args[1] == 2000
               for call in sent)
    keys = [call.kwargs.get("key") for call in heli.owner._emit.call_args_list]
    assert "autonomous.rotorcraft.low_g" in keys


def test_heli_watch_stands_down_for_the_autorotation_and_the_switch():
    heli = _heli(stage="autorotate")
    heli._protect(_airborne(0.0 + 0.05), vs=0.0, ias=60.0, agl=500.0)
    heli.owner._emit.assert_not_called()
    heli = _heli(protection=False)
    heli._protect(_airborne(9.9), vs=0.0, ias=80.0, agl=1000.0)
    heli.owner._emit.assert_not_called()


def test_heli_demeanor_follows_the_stage():
    heli = _heli(stage="approach")
    assert heli._demeanor(_airborne(1.0)) == "gentle"
    heli.state.stage = "cruise"
    assert heli._demeanor(_airborne(1.0)) == "smooth"
    heli.state.stage = "autorotate"
    assert heli._demeanor(_airborne(1.0)) == "urgent"


def test_heli_profile_table_marks_the_teetering_rotors():
    from meridianatc.atc.autonomous_heli import _HELI_PROFILES
    for teetering in ("R44", "R66", "B06", "UH1", "B412"):
        assert _HELI_PROFILES[teetering].low_g == 0.4
    assert _HELI_PROFILES["H125"].low_g == 0.2


def test_model_published_ceiling_beats_the_class_figure():
    fo = _fo(perf=Performance(cruise_kt=110.0, ceiling_ft=14000.0))
    state = AircraftState(service_ceiling_ft=20688.0,
                          oxygen_capacity_liters=1870.0,
                          oxygen_remaining_liters=1870.0)
    assert fo._limits(state)["ceiling_ft"] == pytest.approx(20688.0)
    state = AircraftState(design_cruise_alt_ft=17000.0,
                          pressurization_capable=True)
    assert fo._limits(state)["ceiling_ft"] == pytest.approx(17000.0)


def test_unpressurized_without_oxygen_is_planned_out_of_hypoxia():
    """FAA guidance recommends oxygen above 10,000 ft in an unpressurised
    aeroplane; EASA/ICAO require flight-crew oxygen for the whole time
    there. A service ceiling is physical performance, not evidence the
    occupants can breathe there."""
    fo = _fo(perf=Performance(cruise_kt=175.0, ceiling_ft=20000.0))
    state = AircraftState(service_ceiling_ft=20688.0, engine_type=0,
                          type_code="BE58")
    assert fo._limits(state)["ceiling_ft"] == pytest.approx(10000.0)
    state.oxygen_capacity_liters = 1870.0
    state.oxygen_remaining_liters = 1870.0
    assert fo._limits(state)["ceiling_ft"] == pytest.approx(20688.0)
    state = AircraftState(service_ceiling_ft=20688.0, engine_type=0,
                          type_code="BE58", pressurization_capable=True)
    assert fo._limits(state)["ceiling_ft"] == pytest.approx(20688.0)


def test_manage_oxygen_uses_the_bottle_before_ten_thousand():
    """The live Baron failure was not absence of equipment: X-Plane's
    .acf carried 1,870 litres and two occupants were plugged in, but the
    valve stayed closed. The FO uses the equipment before the cabin is
    in the hypoxia range."""
    fo = _fo()
    fo._dir.write = Mock(return_value=True)
    state = AircraftState(cabin_altitude_ft=9800.0, on_ground=False,
                          oxygen_capacity_liters=1870.0,
                          oxygen_remaining_liters=1870.0,
                          oxygen_plugged_in=0, oxygen_on=False)
    fo._manage_oxygen(state)
    fo._dir.write.assert_any_call("OXYGEN PLUGGED IN", 2.0)
    fo._dir.write.assert_any_call("OXYGEN FLOW SETTING", 2.0)
    fo._dir.write.assert_any_call("OXYGEN SWITCH", 1.0)
    assert "autonomous.oxygen.on" in _emit_keys(fo)


def test_manage_oxygen_warns_when_no_system_exists():
    fo = _fo()
    fo._dir.write = Mock(return_value=True)
    state = AircraftState(cabin_altitude_ft=10500.0, on_ground=False)
    fo._manage_oxygen(state)
    fo._dir.write.assert_not_called()
    assert "autonomous.oxygen.none" in _emit_keys(fo)


def test_session_cruise_is_capped_when_only_the_wings_can_get_there():
    fo = _fo(perf=Performance(cruise_kt=175.0, ceiling_ft=20000.0))
    session = SimpleNamespace(cruise_altitude_ft=17000.0)
    state = AircraftState(engine_type=0, type_code="BE58",
                          service_ceiling_ft=20688.0)
    fo._cap_cruise_for_physiology(session, state)
    assert session.cruise_altitude_ft == pytest.approx(10000.0)
    session = SimpleNamespace(cruise_altitude_ft=17000.0)
    state.oxygen_capacity_liters = 1870.0
    state.oxygen_remaining_liters = 1870.0
    fo._cap_cruise_for_physiology(session, state)
    assert session.cruise_altitude_ft == pytest.approx(17000.0)
