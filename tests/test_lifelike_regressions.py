"""Regressions from the lifelike-behaviour audit: the field's own clock
instead of the computer's, the dead "turn out" clearance, the go-around
rejoin's distance half, heading north as an assignment, and carb heat on
every engine."""

import random
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest


def _flight():
    from meridianatc.atc.autonomous import (
        AutonomousFlight, AutonomousState)
    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    return flight


def _injector():
    from meridianatc.atc.traffic.core import TrafficInjector
    return TrafficInjector.__new__(TrafficInjector)


# -- the field's own clock --------------------------------------------------

def test_the_field_hour_is_utc_plus_the_longitudes_share_of_a_day():
    """Solar hour: exact on the meridian, an hour in either way off it."""
    from meridianatc.atc.local_time import field_hour
    utc = time.struct_time((2024, 6, 1, 17, 0, 0, 5, 153, 0))
    with patch("time.gmtime", return_value=utc):
        assert field_hour(0.0) == pytest.approx(17.0)
        assert field_hour(15.0) == pytest.approx(18.0)
        assert field_hour(-118.4) == pytest.approx(9.106, abs=0.01)
        assert field_hour(165.0) == pytest.approx(4.0)


def test_the_diurnal_follows_the_fields_clock_not_the_computers():
    """A field eight time zones away used to run the rhythm of home."""
    injector = _injector()
    injector.diurnal = True
    utc = time.struct_time((2024, 6, 1, 17, 0, 0, 5, 153, 0))
    # 17:00 UTC is ten in the morning at Los Angeles -- the bank hour --
    # and two in the small hours at Tokyo, which is nearly quiet.
    injector.field = SimpleNamespace(latitude=33.9, longitude=-118.4)
    with patch("time.gmtime", return_value=utc):
        la = injector._diurnal()
    injector.field = SimpleNamespace(latitude=35.7, longitude=139.7)
    with patch("time.gmtime", return_value=utc):
        tokyo = injector._diurnal()
    assert la == pytest.approx(1.0, abs=0.05)
    assert tokyo == pytest.approx(0.15, abs=0.05)


def test_the_crews_clock_is_the_fields_too():
    """The purser's "local time" and the dimmed lights, off longitude."""
    from meridianatc.atc.crew import _after_dark, _clock_now
    assert _after_dark(23.0) and _after_dark(5.0)
    assert not _after_dark(12.0)
    assert "3 30 in the afternoon" in _clock_now("en", 15.5)
    assert _clock_now("de", 15.5) == "15 Uhr 30"


# -- turn out is a real instruction ------------------------------------------

def test_turn_out_is_a_heading_to_the_filed_destination():
    """The approval used to be read back and then never flown."""
    injector = _injector()
    injector.navdb = SimpleNamespace(
        airport=lambda ident: SimpleNamespace(lat=48.0, lon=2.0))
    injector.random = random.Random(0)
    flight = SimpleNamespace(
        destination_icao="LFPG", latitude=51.0, longitude=4.0,
        pattern_direction="left", runway="06", heading=90.0,
        assigned_heading=None, held_since=0.0)
    from meridianatc.atc.flying import Command
    from meridianatc.navdata.db import bearing_deg
    injector._carry_out(flight, Command(kind="turn_out"), 100.0)
    assert flight.assigned_heading == pytest.approx(
        bearing_deg(51.0, 4.0, 48.0, 2.0))


def test_turn_out_without_a_destination_still_leaves_the_runway_heading():
    """With nothing to point at, off to the circuit's side it goes."""
    injector = _injector()
    injector.navdb = SimpleNamespace(airport=lambda ident: None)
    injector.random = random.Random(0)
    injector._threshold = lambda runway: (51.0, 4.0, 90.0)
    flight = SimpleNamespace(
        destination_icao="", latitude=51.0, longitude=4.0,
        pattern_direction="left", runway="06", heading=90.0,
        assigned_heading=None, held_since=0.0)
    from meridianatc.atc.flying import Command
    injector._carry_out(flight, Command(kind="turn_out"), 100.0)
    turn = (90.0 - flight.assigned_heading) % 360.0
    assert 25.0 <= turn <= 55.0


# -- the go-around's way back ------------------------------------------------

def _going_around(distance_nm: float, altitude_ft: float):
    return SimpleNamespace(
        altitude_ft=altitude_ft, distance_nm=distance_nm,
        performance=SimpleNamespace(pattern_agl_ft=1000.0),
        phase="go_around", arriving=True, step=4, next_at=0.0)


def test_a_slow_go_around_that_flies_past_the_upwind_still_rejoins():
    """The "far enough out" half of the rejoin test was dead code."""
    from meridianatc.atc.circuit import CIRCUIT_UPWIND_NM
    injector = _injector()
    injector.field = SimpleNamespace(elevation_ft=0.0)
    flight = _going_around(CIRCUIT_UPWIND_NM + 1.0, 500.0)
    injector._after_the_go_around(flight, 100.0)
    assert flight.phase == "upwind"
    assert flight.arriving is False
    assert flight.next_at == 105.0


def test_a_go_around_still_low_and_close_keeps_climbing_out():
    """Below pattern and still on the upwind leg is still a go-around."""
    from meridianatc.atc.circuit import CIRCUIT_UPWIND_NM
    injector = _injector()
    injector.field = SimpleNamespace(elevation_ft=0.0)
    flight = _going_around(CIRCUIT_UPWIND_NM - 0.5, 500.0)
    injector._after_the_go_around(flight, 100.0)
    assert flight.phase == "go_around"
    assert flight.next_at == 103.0


def test_a_go_around_at_pattern_height_rejoins_wherever_it_is():
    injector = _injector()
    injector.field = SimpleNamespace(elevation_ft=0.0)
    flight = _going_around(0.3, 900.0)
    injector._after_the_go_around(flight, 100.0)
    assert flight.phase == "upwind"


# -- heading north is a heading ------------------------------------------------

def test_heading_north_is_an_assignment_not_its_absence():
    """``or -1.0`` read a stored heading of zero as "no assignment"."""
    from meridianatc.atc import flying as fly
    flight = _flight()
    flight.session = SimpleNamespace(
        assigned_heading=0.0, assigned_altitude_ft=5000.0,
        cruise_altitude_ft=0.0, initial_altitude_ft=0.0,
        assigned_speed_kt=0.0)
    flight._ap_mode = "direct"
    flight._perf = fly.LIGHT
    flight._dir = Mock()
    flight.sim = flight._dir.sim
    flight._apply_flight_director = Mock()
    flight._manage_throttle = Mock()
    state = SimpleNamespace(autothrottle=False)

    flight._engage_climb(state)
    assert flight._apply_flight_director.call_args.kwargs["heading"] == 0.0
    flight._dir.nav.assert_not_called()

    # And nothing assigned is still the route's business.
    flight.session.assigned_heading = None
    flight._apply_flight_director.reset_mock()
    flight._engage_climb(state)
    assert flight._apply_flight_director.call_args.kwargs["heading"] == -1.0
    assert flight.state.ap_nav_mode


# -- carb heat is every engine's ------------------------------------------------

def test_carb_heat_reaches_every_engine_not_just_the_first():
    """A piston twin used to ice its right-hand engine on its own."""
    flight = _flight()
    flight._dir = Mock()
    flight.sim = flight._dir.sim
    flight.session = SimpleNamespace(aircraft=None)
    flight._carb_heat_wanted = lambda state: True
    flight._emit = Mock()
    state = SimpleNamespace(
        on_ground=False, ambient_in_cloud=False, ambient_precip_state=0,
        ambient_temperature_c=20.0, total_air_temperature_c=20.0,
        engines_running=True, engine_count=2, structural_ice_pct=0.0,
        type_code="PA34")
    kit = SimpleNamespace(pitot=False, engine_inlet=False,
                          inertial_separator=False, prop_deice=False,
                          airframe_deice=False, windshield=False,
                          carb_heat=True, known_ice=True)
    with patch("meridianatc.atc.powerplant.ice_kit_for",
               return_value=kit):
        flight._watch_icing(state, 100.0)
    written = {call.args[0] for call in flight._dir.write.call_args_list}
    assert "GENERAL ENG ANTI ICE POSITION:1" in written
    assert "GENERAL ENG ANTI ICE POSITION:2" in written
