"""The First Officer flown closed-loop against a small aeroplane.

These assert on where the aeroplane ends up -- on the runway, on the
centreline, at a survivable sink -- rather than on which events left.
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from flight_plant import Plant, PlantDirector

from meridianatc.atc import auto_nav
from meridianatc.atc import flying as fly
from meridianatc.navdata.build import project

RWY_LAT, RWY_LON, RWY_HDG, RWY_LEN_FT = 45.0, 5.0, 90.0, 8000.0


class _NavDB:
    """One airport, one runway: all the final needs to know."""

    def __init__(self, airport):
        self.airport = airport
        far_lat, far_lon = project(RWY_LAT, RWY_LON, RWY_HDG,
                                   RWY_LEN_FT / 6076.12)
        self.runway = SimpleNamespace(
            le_ident="09", he_ident="27", le_lat=RWY_LAT, le_lon=RWY_LON,
            he_lat=far_lat, he_lon=far_lon, length_ft=RWY_LEN_FT,
            width_ft=150.0, le_elevation_ft=0.0, he_elevation_ft=0.0,
            le_heading=RWY_HDG, he_heading=(RWY_HDG + 180.0) % 360.0,
            surface="ASP", closed=False, lighted=True,
            le_ils=None, he_ils=None)

    def runways(self, ident):
        return [self.runway]

    def nearest_airports(self, *a, **k):
        return []

    def home_airport(self, lat, lon):
        return self.airport

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return lambda *a, **k: None


def _fo(plant, type_code="C172", light=True):
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousPhase
    from meridianatc.atc.session import FlightSession
    from meridianatc.config import Config
    from meridianatc.navdata.db import Airport

    airport = Airport(
        ident="LFXX", icao="LFXX", iata="", name="Test", spoken="Test",
        type="medium_airport", lat=RWY_LAT, lon=RWY_LON, elev_ft=0.0,
        country="FR", region="", municipality="", scheduled=0, size_rank=1)
    session = FlightSession(aircraft=SimpleNamespace(
        type_code=type_code, title="", is_heavy=False, spoken="",
        language="en", callsign=SimpleNamespace(spoken="", written="")))
    session.destination = airport
    session.arrival_runway = "09"
    session.cleared_landing = True
    session.ifr = False
    fo = AutonomousFlight(Config(), _NavDB(airport), None, session, Mock(),
                          Mock(), plant.state, Mock())
    fo._weather_reactions = False
    fo._manage_radio = False
    fo._dir = PlantDirector(plant)
    fo._perf = fly.performance_for(type_code, light=light)
    plant.type_code = type_code
    fo.state.phase = AutonomousPhase.APPROACH
    fo.state.sub = "final"
    fo._last_lever = plant.lever
    fo.state.phase_entered_at = 1000.0
    fo._runway_blocked = lambda *a, **k: False
    return fo


def _fly_final(fo, plant, monkeypatch, seconds=900.0, dt=0.25, trace=None):
    """Run the approach and landing phases against the plant until the
    aeroplane is down and slow, the FO gives up, or the clock runs out.
    The FO's own clock is the plant's: its slew limiters and PIDs read
    ``time.monotonic``."""
    import time

    from meridianatc.atc.autonomous import AutonomousPhase
    clock = [1000.0]
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])
    # A Pid binds its clock when the class is defined, so one made after
    # another test imported the module would still read the wall clock.
    made = fo._pid

    def on_the_plants_clock(channel):
        pid = made(channel)
        pid._clock = lambda: clock[0]
        return pid
    fo._pid = on_the_plants_clock
    end = clock[0] + seconds
    while clock[0] < end:
        state = plant.state()
        if fo.state.phase == AutonomousPhase.APPROACH:
            fo._phase_approach(state, clock[0])
        elif fo.state.phase == AutonomousPhase.LANDING:
            fo._phase_landing(state, clock[0])
        else:
            break
        was_airborne = not plant.on_ground
        plant.step(dt)
        clock[0] += dt
        if was_airborne and plant.on_ground:
            plant.touchdown = auto_nav.align(
                plant.lat, plant.lon, RWY_LAT, RWY_LON, RWY_HDG)
        if trace is not None:
            where = auto_nav.align(plant.lat, plant.lon, RWY_LAT, RWY_LON,
                                   RWY_HDG)
            trace.append((clock[0], where["along_m"], where["offset_m"],
                          plant.alt, plant.vs, plant.ias, plant.hdg,
                          plant.lever, str(fo.state.phase), fo.state.sub))
    return plant


def _start(distance_nm, alt_ft, offset_nm=0.0, heading=RWY_HDG, ias=75.0,
           **kw):
    lat, lon = project(RWY_LAT, RWY_LON, (RWY_HDG + 180.0) % 360.0,
                       distance_nm)
    if offset_nm:
        lat, lon = project(lat, lon, (RWY_HDG + 90.0) % 360.0, offset_nm)
    return Plant(lat, lon, alt_ft, heading, ias, **kw)


def _assert_landed_on_the_runway(plant, max_sink_fpm=600.0):
    assert plant.on_ground, "never touched down"
    td = plant.touchdown
    # Past the threshold, inside the first half, within the pavement.
    assert 0.0 < td["along_m"] < RWY_LEN_FT * 0.3048 * 0.5, td
    assert abs(td["offset_m"]) < 20.0, td
    assert abs(plant.touchdown_fpm) < max_sink_fpm, plant.touchdown_fpm
    assert plant.min_ias_airborne > plant.stall_kt * 1.05
    # And the rollout stayed on it.
    end = auto_nav.align(plant.lat, plant.lon, RWY_LAT, RWY_LON, RWY_HDG)
    assert abs(end["offset_m"]) < 20.0, end
    assert end["along_m"] < RWY_LEN_FT * 0.3048, end


def test_a_visual_final_on_the_path_lands_on_the_runway(monkeypatch):
    plant = _start(5.0, 1640.0)
    _fly_final(_fo(plant), plant, monkeypatch)
    _assert_landed_on_the_runway(plant)


def test_a_visual_final_begun_low_does_not_land_short(monkeypatch):
    # Five miles out at six hundred feet: a thousand feet under the path.
    # Descending at the path's rate from here meets the ground two miles
    # short of the runway.
    plant = _start(5.0, 600.0)
    _fly_final(_fo(plant), plant, monkeypatch)
    _assert_landed_on_the_runway(plant)


def test_a_visual_final_begun_off_the_centreline_closes_on_it(monkeypatch):
    plant = _start(6.0, 1900.0, offset_nm=0.5)
    _fly_final(_fo(plant), plant, monkeypatch)
    _assert_landed_on_the_runway(plant)


def test_a_crosswind_final_is_held_to_the_centreline(monkeypatch):
    plant = _start(5.0, 1640.0, wind_from=0.0, wind_kt=12.0)
    _fly_final(_fo(plant), plant, monkeypatch)
    _assert_landed_on_the_runway(plant)


def test_a_final_begun_far_too_high_goes_around_and_flies_a_circuit(
        monkeypatch):
    # Two miles out at eighteen hundred feet: no three-degree path
    # reaches the touchdown zone from here. The approach is thrown away,
    # the aeroplane positions downwind and base by itself, and the
    # second final is the one that lands.
    plant = _start(2.0, 1800.0)
    fo = _fo(plant)
    _fly_final(fo, plant, monkeypatch, seconds=1500.0)
    assert fo.state.go_arounds == 1
    assert plant.min_agl_airborne > 0.0
    _assert_landed_on_the_runway(plant)


def test_a_vector_is_flown_level_until_the_final_is_there(monkeypatch):
    # Abeam the field heading the wrong way with a controller's heading
    # assigned: the vector is flown, level, not a dive at the runway.
    plant = _start(-1.0, 1500.0, offset_nm=2.0, heading=270.0)
    fo = _fo(plant)
    fo.session.assigned_heading = 270.0
    fo.session.assigned_altitude_ft = 1500.0
    _fly_final(fo, plant, monkeypatch, seconds=60.0)
    assert plant.alt == pytest.approx(1500.0, abs=60.0)
    assert abs((plant.hdg - 270.0 + 540.0) % 360.0 - 180.0) < 10.0


def test_a_climb_gives_its_rate_away_before_the_airspeed():
    # A vertical-speed climb the engine cannot pay for bleeds airspeed
    # into the stall; the FO asks for less rate, then for the nose down.
    plant = _start(5.0, 3000.0)
    fo = _fo(plant)
    lim = {"stall_kt": 50.0}
    state = plant.state()
    fo._flaps_asked = 0.5
    state.indicated_airspeed_kt = 90.0
    assert fo._climb_within_speed(state, 700.0, lim) == 700.0
    state.indicated_airspeed_kt = 61.5
    assert 100.0 <= fo._climb_within_speed(state, 700.0, lim) < 700.0
    state.indicated_airspeed_kt = 55.0
    assert fo._climb_within_speed(state, 700.0, lim) < 0.0
    # A descent is never touched.
    assert fo._climb_within_speed(state, -500.0, lim) == -500.0


def test_the_descent_flies_its_path_down_without_a_level_leg(
        monkeypatch):
    """Fourteen miles out on the three-degree line and nothing assigned:
    the descent is continuous -- a dive and a drive would park on a
    level for the minutes it takes the field to come to it -- and the
    approach it hands over to still lands on the runway."""
    import time

    from meridianatc.atc.autonomous import AutonomousPhase
    plant = _start(14.0, 4500.0)
    fo = _fo(plant)
    fo.state.phase = AutonomousPhase.DESCENT
    fo.state.sub = ""
    fo.state.phase_entered_at = 1000.0
    fo.brain = SimpleNamespace(
        route=None, terminal_procedures={}, route_progress=None,
        fms_idents=())
    fo._run_checklist_for_phase = lambda *a, **k: None

    clock = [1000.0]
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])
    made = fo._pid

    def on_the_plants_clock(channel):
        pid = made(channel)
        pid._clock = lambda: clock[0]
        return pid
    fo._pid = on_the_plants_clock

    dt = 0.25
    longest_level = level_run = 0
    for _ in range(int(900.0 / dt)):
        state = plant.state()
        if fo.state.phase != AutonomousPhase.DESCENT:
            break
        fo._phase_descent(state, clock[0])
        plant.step(dt)
        clock[0] += dt
        level_run = level_run + 1 if abs(plant.vs) < 30.0 else 0
        longest_level = max(longest_level, level_run)
    # The approach is taken over on the path's way down, not after a
    # wait at a floor.
    assert fo.state.phase == AutonomousPhase.APPROACH
    # A level segment would sit inside thirty fpm for tens of seconds;
    # twenty steps is five seconds of grace for the capture bands.
    assert longest_level < 20, longest_level * dt
    _fly_final(fo, plant, monkeypatch)
    _assert_landed_on_the_runway(plant)


def test_a_climb_under_the_transport_gate_still_comes_clean():
    """A C172's book climb is 74 kt -- under the Vr+20 retract gate the
    transport schedule asks (75): unreachable, and the takeoff flap
    rode climb, cruise and descent for the whole leg (the live-flight
    regression this covers). The light aeroplane's figure is its climb
    speed less the dirty-wing slack, and the later phases run the same
    test so a leftover cannot stay out for the flight.
    """
    from dataclasses import replace
    from meridianatc.atc.autonomous import AutonomousPhase
    plant = _start(5.0, 3000.0)
    fo = _fo(plant)
    fo.state.vr_kt, fo.state.v2_kt = 55.0, 0.0
    fo._perf = replace(fo._perf, climb_kt=74.0)
    fo.brain = SimpleNamespace(route=None, terminal_procedures={},
                               route_progress=None, fms_idents=())
    fo._run_checklist_for_phase = lambda *a, **k: None
    fo.state.phase = AutonomousPhase.CLIMB
    plant.flaps, plant.ias = 1, 61.0
    fo._phase_climb(plant.state(), 1000.0)
    assert plant.flaps == 0
    # A jet's gate is untouched: the transport schedule still wants
    # V2+20 before the flap moves.
    fo._perf = replace(fo._perf, climb_kt=165.0)
    fo.state.vr_kt, fo.state.v2_kt = 132.0, 141.0
    assert fo._flap_up_kt() == 161.0
    # The same leftover in cruise is a leftover, not a configuration.
    fo.state.vr_kt, fo.state.v2_kt = 55.0, 0.0
    fo._perf = replace(fo._perf, climb_kt=74.0)
    fo.state.phase = AutonomousPhase.CRUISE
    plant.flaps = 1
    fo._phase_cruise(plant.state(), 1000.0)
    assert plant.flaps == 0


def test_a_flap_stage_refused_for_speed_is_selected_once_slow_enough():
    from meridianatc.atc.autonomous import AutonomousPhase
    plant = _start(5.0, 3000.0)
    fo = _fo(plant)
    fo.state.phase = AutonomousPhase.DESCENT
    fo._flaps_asked = 0.0
    state = plant.state()
    fast = fo._limits(state)["flap_max_kt"] + 20.0
    state.indicated_airspeed_kt = fast
    fo._set_flaps(state, 0.5)
    assert plant.flaps == 0.0 and fo._flaps_owed == 0.5
    fo._configure_owed(state)
    assert plant.flaps == 0.0
    state.indicated_airspeed_kt = fast - 40.0
    fo._configure_owed(state)
    assert plant.flaps > 0.0 and fo._flaps_owed is None
