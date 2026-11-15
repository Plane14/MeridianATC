from __future__ import annotations

import math
import threading
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from meridianatc.atc import approaches, auto_nav
from meridianatc.atc.continuous import ContinuousFlights, MissionRequest
from meridianatc.atc.final import Final, course_over_ground
from meridianatc.sim.state import AircraftState


@pytest.mark.parametrize("contents", ["1", '"bad"', '{"airlines": 1}', '{"airlines": {"bad": 1}}', "null", "{"])
def test_malformed_airline_catalogue_degrades_to_empty(tmp_path, contents):
    from meridianatc.atc.airlines import _load
    path = tmp_path / "icao_callsigns.json"
    path.write_text(contents, encoding="utf-8")
    assert _load(path) == []


def test_airline_catalogue_keeps_valid_rows(tmp_path):
    from meridianatc.atc.airlines import _load
    path = tmp_path / "icao_callsigns.json"
    path.write_text('{"airlines": [null, 3, {"icao": "BAW", "callsign": "Speedbird"}]}', encoding="utf-8")
    assert _load(path) == [{"icao": "BAW", "callsign": "Speedbird"}]


@pytest.mark.parametrize("same_station", [False, True])
def test_readback_only_accepts_instructions_from_the_current_controller(same_station):
    pytest.importorskip("scipy")
    from meridianatc.atc.controller import ControllerBrain
    from meridianatc.atc.intents import Intent, ParsedIntent
    from meridianatc.atc.phraseology import Aircraft, Phraseology, Weather
    from meridianatc.atc.session import FlightSession, Instruction, station_key
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.navdata.db import Station

    old = Station("KAAA", "DEL", 121.61, "Alpha")
    station = Station("KAAA", "DEL", 121.61 if same_station else 121.625, "Alpha")
    session = FlightSession(Aircraft(parse_callsign("N123")))
    instruction = Instruction("heading", station_key(old), values={"heading": 90})
    session.pending = instruction
    brain = ControllerBrain(Mock(), Phraseology())
    brain.staffed = lambda station: True
    parsed = ParsedIntent(intent=Intent.READBACK, normalized="heading nine zero", readback_items={"heading": 90})
    brain.respond(parsed, session, station, Weather())
    assert instruction.read_back is same_station
    assert session.assigned_heading == (90 if same_station else None)
    assert session.pending is None


@pytest.mark.parametrize("visibility", [0, .1, None, "bad", float("nan"), float("inf")])
def test_unusable_visibility_never_meets_approach_minima(visibility):
    weather = SimpleNamespace(visibility_sm=visibility, clouds=[])
    assert not approaches._visual_weather(weather)
    assert not approaches._allows(weather, 200, .5)
    field = approaches.FieldApproaches("KAAA")
    field.add("09", "ILS")
    choice = approaches.plan(field, "09", weather, region="faa")
    assert choice.kind == "ILS"


def test_visibility_minima_boundaries_and_missing_weather():
    weather = SimpleNamespace(visibility_sm=10, clouds=[("BKN", 5000)])
    assert approaches._visual_weather(weather)
    assert approaches._allows(weather, 200, .5)
    weather.visibility_sm = 0
    assert approaches._below_basic_vfr(weather)
    assert not approaches._visual_weather(None)
    assert approaches._allows(None, 200, .5)


def test_real_north_track_and_unavailable_track():
    state = AircraftState(source="xplane", heading_true=70, track_true=0)
    assert course_over_ground(state) == 0
    assert Final("36", 40, -74, 0).aligned(state)
    state.unavailable = ("track_true",)
    assert course_over_ground(state) == 70
    state.unavailable = ("track_true", "heading_true")
    assert course_over_ground(state) is None
    state = AircraftState(heading_true=70)
    assert course_over_ground(state) == 70


@pytest.mark.parametrize("track", [None, "bad", float("nan"), float("inf")])
def test_invalid_track_falls_back_to_heading(track):
    assert course_over_ground(SimpleNamespace(track_true=track, heading_true=370)) == 10


@pytest.mark.parametrize("latitude", [None, "bad", float("nan"), float("inf"), 91])
def test_missing_position_cannot_establish_final(latitude):
    state = AircraftState(latitude=latitude, longitude=-74)
    final = Final("36", 40, -74, 0)
    assert math.isinf(final.distance(state))
    assert not final.in_cone(state, within_nm=10)
    assert not final.approach_side(state)


def test_unavailable_position_cannot_establish_final():
    state = AircraftState(latitude=40, longitude=-74, unavailable=("latitude",))
    assert not Final("36", 40, -74, 0).in_cone(state, within_nm=10)


def test_path_signed_cross_track_and_north_heading():
    path = [(40, -74), (40.01, -74)]
    right = auto_nav.path_steering(40.005, -73.999, path)
    left = auto_nav.path_steering(40.005, -74.001, path)
    assert right["cross_track_m"] > 0
    assert left["cross_track_m"] < 0
    assert auto_nav.path_steering(40.005, -74, path)["heading"] == pytest.approx(0)


def test_path_interpolation_across_dateline():
    path = [(0, 179.99), (0, -179.99)]
    steer = auto_nav.path_steering(0, 179.995, path, lookahead_m=1000)
    assert steer["heading"] == pytest.approx(90)
    assert steer["remaining_m"] == pytest.approx(1668, abs=5)
    assert abs(steer["cross_track_m"]) < 1


def test_empty_single_and_duplicate_point_paths_finish():
    assert auto_nav.path_steering(40, -74, [])["done"]
    assert auto_nav.path_steering(40, -74, [(40, -74)])["done"]
    assert not auto_nav.path_steering(40, -74, [(41, -74)])["done"]
    assert auto_nav.path_steering(40, -74, [(40, -74), (40, -74)])["done"]


def runway():
    return SimpleNamespace(le_ident="36", he_ident="18", le_lat=40, le_lon=-74,
                           he_lat=40.01, he_lon=-74, length_ft=6000)


def test_runway_lookup_requires_the_cleared_end_not_nearest_heading():
    assert auto_nav.runway_end([runway()], "36") == (40, -74, 0, 6000, 0.0)
    assert auto_nav.runway_end([runway()], "36L") is None
    assert auto_nav.runway_end([runway()], "09") is None
    assert auto_nav.runway_end([], "36") is None
    assert auto_nav.runway_end([SimpleNamespace(le_lat=None)], "36") is None


def test_the_ground_intercept_aims_down_the_line_not_at_the_threshold():
    # A kilometre down the strip and a wingspan off the line: the
    # threshold-angle law corrects by a degree -- parallel grass for the
    # length of the runway -- the lookahead law points at the line
    # ~120 m ahead and actually closes.
    err = auto_nav.align(41.2531, -8.68387, 41.263302, -8.68524, 172.8)
    assert abs(err["offset_m"]) > 20.0
    assert abs(err["correction_deg"]) < 3.0
    err = auto_nav.align(41.2531, -8.68387, 41.263302, -8.68524, 172.8,
                         lookahead_m=120.0)
    assert err["correction_deg"] * err["offset_m"] < 0.0
    assert abs(err["correction_deg"]) > 10.0


@pytest.mark.parametrize("name", ["min_nm", "max_nm", "reserve_factor", "service_minutes"])
@pytest.mark.parametrize("value", [None, "bad", True, float("nan"), float("inf")])
def test_mission_rejects_nonfinite_and_invalid_values(name, value):
    request = replace(MissionRequest(), **{name: value})
    service = ContinuousFlights(Mock())
    assert service.start(request)
    assert not service.running


def test_mission_rejects_impossible_reserve_band_and_rules():
    assert MissionRequest(min_nm=100, max_nm=150, reserve_factor=2).validate()
    assert MissionRequest(flight_rules="bad").validate()
    assert not MissionRequest().validate()


@pytest.mark.parametrize("restart", [False, True])
def test_stopped_or_restarted_mission_does_not_receive_inflight_plan(restart):
    entered, release = threading.Event(), threading.Event()
    def nearest(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        return [(SimpleNamespace(ident="KBBB"), 50)]
    service = ContinuousFlights(SimpleNamespace(nearest_airports=nearest))
    request = MissionRequest()
    service.start(request)
    result = []
    worker = threading.Thread(target=lambda: result.append(service.plan_next_leg(40, -74, "KAAA", "N123")))
    worker.start()
    try:
        assert entered.wait(3)
        service.stop()
        if restart:
            service.start(request)
    finally:
        release.set()
        worker.join(3)
    assert not worker.is_alive()
    assert result == [None]
    assert service.legs == []


def test_mission_empty_candidates_invalid_position_and_valid_leg():
    navdb = Mock()
    navdb.nearest_airports.return_value = []
    service = ContinuousFlights(navdb)
    assert service.plan_next_leg(40, -74, "KAAA", "N123") is None
    assert not service.start(MissionRequest())
    assert service.plan_next_leg(None, -74, "KAAA", "N123") is None
    navdb.nearest_airports.assert_not_called()
    assert service.plan_next_leg(40, -74, "KAAA", "N123") is None
    navdb.nearest_airports.return_value = [(SimpleNamespace(ident="KBBB"), 50)]
    assert service.plan_next_leg(40, -74, "KAAA", "N123").number == 1


def test_fo_fuel_selector_item_writes_on_not_off():
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import AutonomousFlight
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.sim = None
    fo._ap_mode = ""
    fo._dir = Mock()
    fo._dir.sim = None
    # Every fuel_selector check is "not off" (ne 0): expected is the
    # forbidden value, and writing it closed the selector for good.
    assert fo._switch_for(None, "fuel_selector", 0)
    fo._dir.write.assert_called_once_with("FUEL TANK SELECTOR:1", 1.0)
    fo._dir.write.reset_mock()
    assert fo._switch_for(None, "fuel_selector", 2)
    fo._dir.write.assert_called_once_with("FUEL TANK SELECTOR:1", 2.0)


def test_autonomous_preserves_north_vector_and_arrival_runway():
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.state.phase = AutonomousPhase.APPROACH
    fo.session = SimpleNamespace(assigned_heading=0, assigned_altitude_ft=5000,
                                 assigned_speed_kt=200, departure_runway="09",
                                 arrival_runway="36", destination=SimpleNamespace(ident="KBBB"))
    fo._perf = SimpleNamespace(climb_fpm=1500, descend_fpm=1000)
    fo._apply_flight_director = Mock()
    guidance = SimpleNamespace(heading=90, altitude_ft=4000, vs_fpm=-500, speed_kt=180)
    fo._fly_procedure(AircraftState(altitude_ft=5000), guidance)
    assert fo._apply_flight_director.call_args.kwargs["heading"] == 0
    assert fo._sid_nav(None) is None
    assert fo._star_nav(None) is None
    fo.navdb = SimpleNamespace(runways=lambda ident: [runway()])
    fo._find_runway(None)
    assert fo.state.runway_heading == 360
    assert (fo.state.runway_lat, fo.state.runway_lon) == (40, -74)
    fo.session.arrival_runway = "18"
    fo._find_runway(None)
    assert fo.state.runway_heading == 180
    assert fo.state.runway_lat == 40.01
    fo.session.arrival_runway = "09"
    fo._find_runway(None)
    assert fo.state.runway_heading == 0
