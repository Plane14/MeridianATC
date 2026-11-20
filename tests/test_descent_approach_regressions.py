"""Regressions for the descent/approach defects read off a live log:

- the check-in named the *cleared* level, not the one flown;
- the throttle servo never brought the power back in the descent, so a
  Cessna rode ninety-odd percent the whole way down and dove at the end;
- a level held on the altimeter was judged against true MSL -- a
  hundred-plus feet apart on a QNH-1018 day;
- a vector into the cone was abandoned for the intercept while the
  watch still expected it flown;
- the approach flap and the gear went down three thousand feet up and
  ten miles out, far ahead of need.
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock


def _engine_for_check_in():
    from meridianatc.atc.phraseology import Phraseology
    from meridianatc.engine.core import Engine
    eng = Engine.__new__(Engine)
    eng.session = SimpleNamespace(
        assigned_altitude_ft=9000.0, pending=None, handoff_pending=None,
        aircraft=SimpleNamespace(
            spoken=lambda abbreviated=False: "x-ray golf sierra"),
        phase=None)
    eng.station = SimpleNamespace(
        ident="LEST", callsign_in=lambda spoken: "Santiago")
    eng._current_station = lambda: eng.station
    eng._language_for = lambda ident, last: "en"
    eng.brain = SimpleNamespace(
        for_station=lambda station, spoken: Phraseology())
    eng.last_language = "en"
    return eng


def test_check_in_reports_the_level_flown_not_the_cleared_one():
    """Through one thousand for nine, the call is "climbing one
    thousand" -- not "flight level niner zero", which invited four
    verify-altitude calls for a level never claimed."""
    eng = _engine_for_check_in()
    eng.state = SimpleNamespace(
        vertical_speed_fpm=1200.0, indicated_altitude_ft=1100.0,
        altitude_ft=970.0)
    text, why = eng.first_officer_call("check_in")
    assert why == ""
    assert "one thousand" in text
    assert "niner" not in text


def test_check_in_level_says_the_altimeters_level():
    """Nearly level, the report is the level the altimeter shows."""
    eng = _engine_for_check_in()
    eng.session.assigned_altitude_ft = 6000.0
    eng.state = SimpleNamespace(
        vertical_speed_fpm=80.0, indicated_altitude_ft=6020.0,
        altitude_ft=5890.0)
    text, why = eng.first_officer_call("check_in")
    assert why == ""
    assert "six thousand" in text


def _fo_descent():
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.state.phase = AutonomousPhase.DESCENT
    fo.session = SimpleNamespace(assigned_altitude_ft=6000.0)
    fo.throttle_writes = []
    fo.sim = None
    fo._dir = SimpleNamespace(
        sim=None,
        throttle=lambda lever: fo.throttle_writes.append(lever))
    fo._last_lever = 0.92
    fo.config = SimpleNamespace(
        immersion=SimpleNamespace(autonomous_pid=False))
    fo._altitude_target = lambda state: 6000.0
    fo._engine_limit_cap = lambda state: 1.0
    return fo


def _airspeed_state(**kw):
    base = dict(
        on_ground=False, engines_running=True, autothrottle=False,
        indicated_airspeed_kt=105.0, throttle_pct=92.0,
        altitude_ft=7900.0, indicated_altitude_ft=7900.0,
        barber_pole_kt=0.0, mach_max_operate=0.0, barber_pole_mach=0.0,
        airspeed_mach=0.0)
    base.update(kw)
    return SimpleNamespace(**base)


def test_a_descent_above_its_level_brings_the_lever_back():
    """Ninety-two percent all the way down was the sawtooth: above the
    cleared level with the speed on target, descent power is idle-ish,
    and the pitch-vs owns the rate."""
    fo = _fo_descent()
    fo._manage_throttle(_airspeed_state(), 105.0)
    assert fo.throttle_writes and fo.throttle_writes[-1] <= 0.20


def test_a_level_leg_in_the_descent_keeps_normal_power():
    """The cap is for losing height, not holding a step: at the cleared
    level the servo behaves as before -- a fast leg gets a normal trim,
    not descent power."""
    fo = _fo_descent()
    fo._manage_throttle(
        _airspeed_state(indicated_airspeed_kt=115.0, altitude_ft=6000.0,
                        indicated_altitude_ft=6000.0), 105.0)
    assert fo.throttle_writes[-1] > 0.50


def test_a_descent_running_slow_gets_its_power_back():
    """Speed sinking under the target lifts the cap -- the lever is the
    speed control while the pitch holds the rate."""
    fo = _fo_descent()
    fo._manage_throttle(
        _airspeed_state(indicated_airspeed_kt=88.0), 105.0)
    assert fo.throttle_writes[-1] > 0.20


def test_the_flown_level_is_the_altimeters_not_true_msl():
    """indicated over true: on a QNH-1018 day they sit a hundred feet
    apart, and the bug's level is the one the needle reads."""
    from meridianatc.atc.autonomous import AutonomousFlight
    state = SimpleNamespace(indicated_altitude_ft=6030.0,
                            altitude_ft=5900.0)
    assert AutonomousFlight._flown_altitude_ft(state) == 6030.0
    state.indicated_altitude_ft = 0.0
    assert AutonomousFlight._flown_altitude_ft(state) == 5900.0


def test_the_watch_judges_what_the_scope_shows():
    """True 5,900 with QNH 1018 reads six thousand on the scope -- level
    as cleared, no verify call earned."""
    from meridianatc.atc.watch import DeviationAssistWatch
    watch = DeviationAssistWatch.__new__(DeviationAssistWatch)
    watch._vs_avg = 0.0
    watch._vs_at = 0.0
    session = SimpleNamespace(
        assigned_altitude_ft=6000.0, assigned_heading=None,
        assigned_speed_kt=None)
    state = SimpleNamespace(
        indicated_altitude_ft=6030.0, altitude_ft=5900.0,
        vertical_speed_fpm=0.0)
    assert "altitude" not in watch._off_now(session, state)
    state.indicated_altitude_ft = 5500.0
    assert "altitude" in watch._off_now(session, state)


def _fo_for_join(assigned, out_nm, course, track, side_nm=0.1):
    """An approach's flight, parked out_nm up the final's course."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    from meridianatc.navdata.build import project
    t_lat, t_lon = 43.295, -8.377          # LECO 03
    lat, lon = project(t_lat, t_lon, course + 180.0, out_nm)
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.state.runway_length_ft = 7900.0
    fo._rejoin_path = None
    fo.session = SimpleNamespace(
        assigned_heading=assigned, assigned_altitude_ft=3000.0,
        assigned_speed_kt=None, ifr=True, pattern_direction="left")
    fo._perf = SimpleNamespace(cruise_kt=110.0)
    fo._final_frame = Mock(
        return_value=(t_lat, t_lon, course, 310.0))
    fo._approach_speed = Mock(return_value=70.0)
    fo._hold_level = Mock()
    fo._emit = Mock()
    # Shift abeam the centreline where a nonzero side is wanted.
    if side_nm:
        lat, lon = project(lat, lon, course + 90.0, side_nm)
    state = SimpleNamespace(
        latitude=lat, longitude=lon, has_position=True,
        track_true=track, heading_true=track,
        altitude_ft=3200.0, indicated_altitude_ft=3200.0,
        altitude_agl_ft=2900.0)
    return fo, state


def test_a_vector_inside_the_cone_is_still_flown():
    """Inside the final's cone but not yet joined, "right heading zero
    zero two" is still the heading -- the intercept is not jumped
    early while the controller owns the vector."""
    fo, state = _fo_for_join(assigned=2.0, out_nm=8.0, course=29.0,
                             track=2.0, side_nm=1.5)
    assert fo._join_final(state, None) is True
    fo._hold_level.assert_called_once()
    assert fo._hold_level.call_args.args[2] == 2.0


def test_the_vector_is_spent_once_the_final_is_joined():
    """On the centreline and roughly on course the vector has done its
    job -- it is released so the approach, and the watch, both move on.
    """
    fo, state = _fo_for_join(assigned=2.0, out_nm=8.0, course=29.0,
                             track=29.0, side_nm=0.0)
    assert fo._join_final(state, None) is False
    assert fo.session.assigned_heading is None


def test_a_vector_that_cannot_intercept_is_released():
    """An assigned heading past sixty off the final's course never
    joins -- the final is flown instead of a vector flown forever."""
    fo, state = _fo_for_join(assigned=150.0, out_nm=8.0, course=29.0,
                             track=90.0, side_nm=1.5)
    assert fo._join_final(state, None) is False
    assert fo.session.assigned_heading is None
