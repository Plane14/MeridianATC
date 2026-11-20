"""The autonomous lights policy and terrain watch -- without a simulator."""

from __future__ import annotations

import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from meridianatc.atc.flying import AIRLINER
from meridianatc.sim.state import AircraftState


def _fo(perf=AIRLINER, **kw):
    """A First Officer built by hand: the state, the director mock the
    axes go through, and no helicopter."""
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import (
        AutonomousFlight, AutonomousState)
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.sim = None
    fo._dir = Mock()
    fo._dir.sim = None
    fo._heli = None
    fo._ap_mode = ""
    fo._perf = perf
    fo._is_helicopter = False
    fo.session = SimpleNamespace(
        aircraft=None, assigned_heading=None, assigned_altitude_ft=0,
        assigned_speed_kt=0, cruise_altitude_ft=0, initial_altitude_ft=0,
        destination=None, arrival_runway="")
    fo.on_event = Mock()
    fo._last_lever = 0.0
    fo._terrain_samples = []
    fo._arrival_final_key = ()
    fo._arrival_final_value = None
    for key, value in kw.items():
        setattr(fo, key, value)
    return fo


def _emit_keys(fo):
    return [call.kwargs.get("key") for call in fo.on_event.call_args_list]


def _airborne(**kw):
    kw.setdefault("on_ground", False)
    kw.setdefault("latitude", 47.26)
    kw.setdefault("longitude", 11.34)
    kw.setdefault("altitude_ft", 4000.0)
    kw.setdefault("altitude_agl_ft", 3000.0)
    kw.setdefault("vertical_speed_fpm", 0.0)
    return AircraftState(**kw)


# -- the lights -----------------------------------------------------------------


def test_landing_and_taxi_lights_off_above_ten_thousand():
    fo = _fo()
    fo.state.phase = "cruise"
    state = _airborne(pressure_altitude_ft=11500.0,
                      altitude_ft=11800.0,
                      landing_light=True, taxi_light=True)
    fo._manage_lights(state)
    fo._dir.landing_light.assert_called_with(False)
    fo._dir.taxi_light.assert_called_with(False)


def test_the_landing_light_is_back_on_below_ten_thousand():
    fo = _fo()
    state = _airborne(pressure_altitude_ft=8900.0,
                      altitude_ft=9200.0,
                      landing_light=False)
    fo._manage_lights(state)
    fo._dir.landing_light.assert_called_with(True)


def test_the_pressure_altitude_is_the_flight_level():
    fo = _fo()
    # Low true altitude over high terrain, but on the flight level the
    # aeroplane is above the line: the lights follow the level.
    state = _airborne(pressure_altitude_ft=10400.0,
                      altitude_ft=9900.0, landing_light=True)
    fo._manage_lights(state)
    fo._dir.landing_light.assert_called_with(False)


def test_on_the_ground_the_lights_are_the_taxi_steps():
    fo = _fo()
    state = AircraftState(on_ground=True, taxi_light=True,
                          landing_light=False)
    fo._manage_lights(state)
    fo._dir.taxi_light.assert_not_called()
    fo._dir.landing_light.assert_not_called()


# -- the terrain watch -----------------------------------------------------------


def test_low_and_not_climbing_off_the_approach_is_a_pull_up():
    fo = _fo()
    fo.state.phase = "descent"
    state = _airborne(altitude_ft=3500.0, altitude_agl_ft=380.0,
                      vertical_speed_fpm=-900.0)
    fo._watch_terrain(state, now=10.0)
    assert fo.state.terrain_escape_until > 10.0
    # The escape climbs over what the sim just read under the
    # aeroplane: 3500 - 380 = 3120 ft of ground, plus the margin.
    assert fo.state.terrain_escape_alt == pytest.approx(
        max(3500.0 + 1500.0, 3120.0 + 1500.0))
    assert "autonomous.terrain_escape" in _emit_keys(fo)


def test_a_healthy_climb_past_the_hills_is_no_warning():
    fo = _fo()
    fo.state.phase = "climb"
    state = _airborne(altitude_ft=9800.0, altitude_agl_ft=500.0,
                      vertical_speed_fpm=1600.0)
    fo._watch_terrain(state, now=10.0)
    assert fo.state.terrain_escape_until == 0.0
    fo.on_event.assert_not_called()


def test_high_agl_alone_is_no_warning():
    fo = _fo()
    state = _airborne(altitude_ft=30000.0, altitude_agl_ft=21000.0,
                      vertical_speed_fpm=-1500.0)
    fo._watch_terrain(state, now=10.0)
    assert fo.state.terrain_escape_until == 0.0


def test_the_landing_phase_is_protected():
    fo = _fo()
    fo.state.phase = "landing"
    state = _airborne(altitude_ft=2100.0, altitude_agl_ft=150.0,
                      vertical_speed_fpm=-650.0)
    fo._watch_terrain(state, now=10.0)
    assert fo.state.terrain_escape_until == 0.0


def test_the_go_around_is_its_own_escape():
    fo = _fo()
    fo.state.phase = "approach"
    fo.state.sub = "go_around"
    state = _airborne(altitude_ft=2500.0, altitude_agl_ft=300.0,
                      vertical_speed_fpm=900.0)
    fo._watch_terrain(state, now=10.0)
    assert fo.state.terrain_escape_until == 0.0


def test_a_declared_emergency_is_left_to_its_own_descent():
    fo = _fo()
    fo.state.phase = "descent"
    fo.state.emergency = "engine_fire"
    state = _airborne(altitude_ft=3000.0, altitude_agl_ft=250.0,
                      vertical_speed_fpm=-4000.0)
    fo._watch_terrain(state, now=10.0)
    assert fo.state.terrain_escape_until == 0.0


def test_inside_the_final_cone_the_path_is_the_answer():
    fo = _fo()
    fo.state.phase = "approach"
    # Innsbruck, runway 08: threshold 47.2602N 11.3437E, course ~79
    # true. The aeroplane is 5 NM out on the centreline -- inside the
    # wedge the approach path, not the watch, owns the height.
    fo.session.destination = SimpleNamespace(ident="LOWI",
                                             lat=47.26, lon=11.34)
    fo.session.arrival_runway = "08"
    fo.state.runway_key = ("LOWI", "08")
    fo.state.runway_lat, fo.state.runway_lon = 47.2602, 11.3437
    fo.state.runway_heading = 79.0
    state = _airborne(latitude=47.235, longitude=11.23,
                      altitude_ft=3000.0, altitude_agl_ft=400.0,
                      vertical_speed_fpm=-800.0)
    fo._watch_terrain(state, now=10.0)
    assert fo.state.terrain_escape_until == 0.0


def test_on_the_ground_the_watch_stands_down():
    fo = _fo()
    fo.state.terrain_escape_until = 99.0
    fo._watch_terrain(AircraftState(on_ground=True), now=10.0)
    assert fo.state.terrain_escape_until == 0.0
    assert fo._terrain_samples == []


def test_the_floor_is_the_seen_ground_plus_the_margin():
    fo = _fo()
    fo.state.phase = "cruise"
    fo._terrain_samples = [(47.0, 11.0, 7200.0, 1.0),
                           (47.1, 11.1, 5400.0, 2.0)]
    assert fo._terrain_floor(_airborne()) == pytest.approx(8000.0)


def test_the_director_is_never_walked_below_the_seen_ground():
    fo = _fo()
    fo.state.phase = "descent"
    fo._terrain_samples = [(47.0, 11.0, 7200.0, time.monotonic())]
    state = _airborne(altitude_ft=9000.0, altitude_agl_ft=2000.0)
    fo._apply_flight_director(state, altitude=4000.0, vs_fpm=-800.0,
                              heading=100.0)
    # The bug goes to the 8000-ft floor, not the 4000 the phase asked.
    assert fo._dir.altitude.call_args.args[0] == pytest.approx(8000.0)


def test_the_escape_flies_a_climb_whatever_the_phase_asked():
    fo = _fo()
    fo.state.phase = "descent"
    fo.state.terrain_escape_until = time.monotonic() + 20.0
    fo.state.terrain_escape_alt = 12000.0
    # The VS bug slews to the demand, so it must not start "just now".
    fo.state.vs_bug_at = time.monotonic() - 10.0
    state = _airborne(altitude_ft=9500.0, altitude_agl_ft=400.0,
                      indicated_airspeed_kt=180.0)
    fo._apply_flight_director(state, altitude=3000.0, vs_fpm=-1000.0,
                              heading=100.0)
    assert fo._dir.altitude.call_args.args[0] == pytest.approx(12000.0)
    vs = fo._dir.vertical_speed.call_args.args[0]
    assert vs > 0.0
