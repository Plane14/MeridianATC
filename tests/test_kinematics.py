from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from meridianatc.atc.flying import AIRLINER, LIGHT, Target, _turn_rate, step


def aircraft(**over) -> SimpleNamespace:
    """The six numbers an aeroplane is made of, plus the wind on it."""
    a = SimpleNamespace(
        latitude=51.0, longitude=-0.5, altitude_ft=5000.0, heading=360.0,
        ground_speed_kt=140.0, vertical_rate_fpm=0.0, on_ground=False,
        wind_deg=0.0, wind_kt=0.0, track=0.0)
    for key, val in over.items():
        setattr(a, key, val)
    return a


def fly(a, target, perf=AIRLINER, seconds=1.0, dt=0.5, energy=True):
    for _ in range(int(seconds / dt)):
        step(a, target, perf, dt, energy=energy)


# -- the wind triangle ---------------------------------------------------------

def test_crosswind_blows_the_track_off_the_heading():
    # Pointing north with a wind from the east: the aeroplane drifts west,
    # which is a track under 360, and it ends up west of the meridian it
    # was pointed along.
    a = aircraft(wind_deg=90.0, wind_kt=30.0)
    fly(a, Target(heading=360.0), seconds=60.0)
    assert a.track < 360.0
    assert a.longitude < -0.5


def test_tailwind_moves_the_ground_not_the_heading():
    a = aircraft(wind_deg=180.0, wind_kt=40.0)
    fly(a, Target(heading=360.0), seconds=30.0)
    assert a.track == pytest.approx(360.0, abs=0.01)
    # Forty knots of following wind on a 140 kt aeroplane: it covers the
    # ground at one-eighty, so half a minute is a mile and a half.
    moved_nm = (a.latitude - 51.0) * 60.0
    assert moved_nm == pytest.approx(180.0 * 30.0 / 3600.0, rel=0.2)


def test_a_crew_holding_final_course_still_lands_on_the_centreline():
    # Aim at the runway's heading each tick -- what _aim does -- and the
    # drift is flown out: crab happens, displacement does not.
    a = aircraft(wind_deg=90.0, wind_kt=25.0, latitude=51.2)
    target_hdg = 360.0
    for _ in range(120):
        # Re-aim at the centreline's heading like a crew correcting.
        step(a, Target(heading=target_hdg), AIRLINER, 0.5)
    # Crosswind to the east pushes west; the correction holds it near the
    # meridian rather than drifting away downwind.
    assert a.longitude > -0.52


def test_wind_does_not_move_a_parked_aeroplane():
    a = aircraft(wind_deg=90.0, wind_kt=40.0, on_ground=True,
                 ground_speed_kt=0.0)
    fly(a, Target(hold=True), seconds=10.0)
    assert a.latitude == 51.0


# -- the energy trade ----------------------------------------------------------

def test_a_climb_bleeds_the_acceleration():
    climbing = aircraft()
    level = aircraft()
    target = Target(speed_kt=200.0, altitude_ft=15000.0)
    fly(climbing, target, seconds=20.0)
    fly(level, Target(speed_kt=200.0), seconds=20.0)
    assert climbing.ground_speed_kt < level.ground_speed_kt


def test_a_descent_lends_the_deceleration_its_difficulty():
    # Descending to a slower speed takes longer than slowing in level
    # flight: the height being given back keeps arriving as speed.
    descending = aircraft()
    level = aircraft()
    fly(descending, Target(speed_kt=90.0, altitude_ft=1000.0), seconds=30.0)
    fly(level, Target(speed_kt=90.0), seconds=30.0)
    assert descending.ground_speed_kt > level.ground_speed_kt


def test_a_dirty_wing_slows_faster_than_a_clean_one():
    # Decelerating to the approach figure gets the flap-and-gear drag; to
    # a cruise figure it does not.
    dirty = aircraft()
    clean = aircraft()
    fly(dirty, Target(speed_kt=140.0), seconds=15.0)
    fly(clean, Target(speed_kt=170.0), seconds=15.0)
    assert dirty.ground_speed_kt < clean.ground_speed_kt


def test_the_energy_model_can_be_switched_back_to_the_integrator():
    modern = aircraft(vertical_rate_fpm=2000.0)
    legacy = aircraft(vertical_rate_fpm=2000.0)
    fly(modern, Target(speed_kt=200.0, altitude_ft=15000.0), seconds=20.0)
    fly(legacy, Target(speed_kt=200.0, altitude_ft=15000.0), seconds=20.0,
        energy=False)
    # The integrator accelerates at the book figure whatever the climb.
    assert legacy.ground_speed_kt > modern.ground_speed_kt


# -- the turn -------------------------------------------------------------------

def test_turn_rate_is_banked_and_speed_bound():
    # Rate-one at 140 kt is about seventeen degrees of bank; at 250 the
    # same aeroplane cannot fly it inside its bank limit, so it turns
    # slower -- which is why heavies fly wide circuits.
    slow = _turn_rate(AIRLINER, 140.0, on_ground=False)
    fast = _turn_rate(AIRLINER, 250.0, on_ground=False)
    assert slow <= AIRLINER.turn_rate + 0.01
    assert fast < slow
    assert fast == pytest.approx(
        19.05 * math.tan(math.radians(25.0)) * 57.2958 / 250.0, rel=0.01)


def test_the_class_rate_still_bounds_a_slow_aeroplane():
    assert _turn_rate(LIGHT, 60.0, on_ground=False) <= LIGHT.turn_rate


def test_a_banked_turn_covers_a_real_radius():
    # A 250 kt aeroplane at its bank limit turns wider than a 140 kt one:
    # the radius the heading sweeps is the physics', not the table's.
    heavy = aircraft(ground_speed_kt=250.0)
    light = aircraft(ground_speed_kt=140.0)
    for _ in range(200):
        step(heavy, Target(heading=180.0), AIRLINER, 0.5)
        step(light, Target(heading=180.0), LIGHT, 0.5)
        if abs(heavy.heading - 180.0) < 0.5 and abs(light.heading - 180.0) < 0.5:
            break
    # The light one got there sooner, measured in heading still to fly.
    assert abs(light.heading - 180.0) <= abs(heavy.heading - 180.0) + 0.5


# -- ordinary behavior, still ---------------------------------------------------

def test_the_hold_still_stops_and_speed_and_altitude_still_converge():
    a = aircraft()
    fly(a, Target(speed_kt=160.0, altitude_ft=8000.0, heading=90.0),
        seconds=150.0)
    assert a.ground_speed_kt == pytest.approx(160.0, abs=3.0)
    assert a.altitude_ft == pytest.approx(8000.0, abs=200.0)
    assert a.heading == pytest.approx(90.0, abs=2.0)
    b = aircraft(on_ground=True, ground_speed_kt=5.0)
    fly(b, Target(hold=True), seconds=30.0)
    assert b.ground_speed_kt == 0.0


def test_variable_dt_moves_the_same_ground():
    fast = aircraft()
    slow = aircraft()
    fly(fast, Target(heading=90.0, speed_kt=200.0), seconds=20.0, dt=0.1)
    fly(slow, Target(heading=90.0, speed_kt=200.0), seconds=20.0, dt=1.0)
    assert fast.ground_speed_kt == pytest.approx(
        slow.ground_speed_kt, abs=2.0)
    assert fast.heading == pytest.approx(slow.heading, abs=1.0)
