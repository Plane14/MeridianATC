import time
from types import SimpleNamespace
from unittest.mock import Mock

from meridianatc.atc.autonomous import (
    AutonomousFlight,
    AutonomousPhase,
    AutonomousState,
)
from meridianatc.atc.session import FlightSession
from meridianatc.sim.state import AircraftState
from meridianatc.weather.enroute import HazardCell, RadarPoint


def test_fo_flies_approved_weather_deviation_heading():
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.session = FlightSession(aircraft=Mock())
    fo.session.weather_deviation_at = time.monotonic()
    fo.session.weather_deviation_side = "left"
    fo.session.weather_deviation_degrees = 30
    fo.session.weather_deviation_miles_nm = 30
    fo.session.weather_deviation_course = 90.0
    fo.sim = object()
    fo._dir = Mock()
    fo._dir.sim = fo.sim
    fo.state = AutonomousState()
    fo.state.phase = AutonomousPhase.CRUISE
    state = AircraftState(
        latitude=50.0, longitude=8.0, on_ground=False,
        ground_speed_kt=300.0, track_true=90.0, heading_true=90.0)
    fo._fly_weather_deviation(state)
    heading = fo._dir.heading.call_args.args[0]
    assert 55.0 < heading < 65.0  # 090 less thirty degrees, apart from magvar
    assert fo.state.ap_hdg_mode


def test_fo_current_hail_creates_point_hazard():
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.sim = None
    fo.enroute_weather = Mock(cells=lambda: [], update=Mock())
    state = AircraftState(
        latitude=50.0, longitude=8.0, on_ground=False, hail_ratio=.2)
    cells = fo._enroute_weather_cells(state)
    assert any(c.source == "point-evidence" for c in cells)


def test_traffic_weather_scan_never_moves_observed_traffic():
    from meridianatc.atc.traffic.core import TrafficInjector
    from meridianatc.atc.traffic.x_weather_avoidance import _Weather_avoidance

    traffic = TrafficInjector.__new__(TrafficInjector)
    traffic.enroute_cells = lambda: [
        HazardCell(50.3, 8.0, 5.0, .9, "test", "severe cell")]
    traffic._exchange = Mock()
    flight = Mock(
        observed=True, on_ground=False, positioned=True, phase="departed",
        weather_deviation_until=0.0, weather_scan_at=0.0,
        latitude=50.0, longitude=8.0, track=0.0, heading=0.0,
        altitude_ft=30000.0, ground_speed_kt=420.0)
    _Weather_avoidance._avoid_enroute_weather(
        traffic, flight, time.monotonic())
    traffic._exchange.assert_not_called()


def test_traffic_weather_scan_queues_deviation_for_owned_traffic():
    from meridianatc.atc.traffic.core import TrafficInjector
    from meridianatc.atc.traffic.x_weather_avoidance import _Weather_avoidance

    traffic = TrafficInjector.__new__(TrafficInjector)
    traffic.enroute_cells = lambda: [
        HazardCell(50.3, 8.0, 5.0, .9, "test", "severe cell")]
    traffic._exchange = Mock()
    flight = Mock(
        observed=False, on_ground=False, positioned=True, phase="departed",
        weather_deviation_until=0.0, weather_scan_at=0.0,
        latitude=50.0, longitude=8.0, track=0.0, heading=0.0,
        altitude_ft=30000.0, ground_speed_kt=420.0)
    now = time.monotonic()
    _Weather_avoidance._avoid_enroute_weather(traffic, flight, now)
    assert flight.weather_deviation_until > now
    traffic._exchange.assert_called_once()
    assert traffic._exchange.call_args.kwargs["command"][0] == "heading"


def test_localized_weather_deviation_requests_are_understood():
    from meridianatc.atc import intents
    from meridianatc.atc.controller._shared import _degrees_said, _side_said
    from meridianatc.atc.language import phraseology_for
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.pilot import PilotSpeech
    from meridianatc.atc.speech import parse_callsign

    parser = intents.IntentParser(["N123", "November one two three"])
    for language in ("en", "fr", "es", "de", "it", "pt"):
        aircraft = Aircraft(parse_callsign("N123"))
        aircraft.language = language
        pilot = PilotSpeech(language, phraseology_for(language))
        text = pilot.request_deviation(aircraft, "Center", 30, "left")
        parsed = parser.parse(text, language)
        assert parsed.intent is intents.Intent.REQUEST_DEVIATION, (language, text)
        assert _side_said(parsed.normalized) == "left", (language, text)
        assert _degrees_said(parsed.normalized, language) == 30, (language, text)
        readback = pilot.deviation(aircraft, 30, "left")
        assert readback.endswith("."), (language, readback)


def _fo_weather_fixture():
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo._is_helicopter = False
    fo._heli = None
    fo.state = AutonomousState()
    fo.state.phase = __import__("meridianatc.atc.autonomous._shared", fromlist=["AutonomousPhase"]).AutonomousPhase.CRUISE
    fo.session = FlightSession(aircraft=Mock())
    fo._emit = Mock()
    fo._fo_say = Mock(return_value=True)
    return fo


def test_fo_requests_enroute_weather_deviation_through_normal_radio_path():
    from meridianatc.weather.enroute import AvoidancePlan

    fo = _fo_weather_fixture()
    plan = AvoidancePlan(True, "left", 30, 25, 330.0, "cell", "awc-sigmet", 30.0)
    fo._enroute_weather_plan = lambda state: plan
    state = AircraftState(on_ground=False)
    now = time.monotonic()
    fo._watch_enroute_convection(state, now)
    fo._fo_say.assert_called_once_with("weather_deviation")
    assert fo._weather_deviation_requested_at == now
    assert fo.session.weather_deviation_at == 0.0


def test_fo_deviates_and_advises_when_cell_is_imminent_and_atc_is_silent():
    from meridianatc.weather.enroute import AvoidancePlan

    fo = _fo_weather_fixture()
    plan = AvoidancePlan(True, "right", 40, 30, 40.0, "cell", "xplane-radar", 5.0)
    fo._enroute_weather_plan = lambda state: plan
    state = AircraftState(on_ground=False, track_true=0.0, ground_speed_kt=300.0)
    now = time.monotonic()
    fo._weather_deviation_requested_at = now - 20.0
    fo._watch_enroute_convection(state, now)
    assert fo.session.deviating == "approved"
    assert fo.session.weather_deviation_side == "right"
    assert fo.session.weather_deviation_degrees == 40
    assert fo.session.weather_deviation_urgent
    fo._fo_say.assert_called_once_with("deviating_weather")


def test_deviation_is_mentioned_on_localized_checkins():
    from meridianatc.atc import intents
    from meridianatc.atc.language import phraseology_for
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.pilot import PilotSpeech
    from meridianatc.atc.speech import parse_callsign

    parser = intents.IntentParser(["N123", "November one two three"])
    for language in ("en", "fr", "es", "de", "it", "pt"):
        aircraft = Aircraft(parse_callsign("N123"))
        aircraft.language = language
        pilot = PilotSpeech(language, phraseology_for(language))
        text = pilot.checking_in(aircraft, "Center", 12000.0,
                                 climbing=False, deviating_side="left")
        parsed = parser.parse(text, language)
        assert parsed.intent is intents.Intent.CHECK_IN, (language, text)


def _fo_departure_fixture(mode):
    from meridianatc.weather.metar import parse_metar

    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.session = FlightSession(aircraft=Mock())
    fo.state = AutonomousState()
    fo.state.runway_heading = 0.0
    fo.config = SimpleNamespace(immersion=SimpleNamespace(
        autonomous_departure_hold=True,
        autonomous_departure_weather_mode=mode))
    weather = parse_metar(
        "KJFK 242051Z 00000KT 10SM TSRA SCT020CB 25/20 Q1000")
    fo._field_weather = lambda airport: weather
    fo._get_current_airport = lambda: None
    fo._wind_limits = lambda: (99.0, 99.0)
    fo._enroute_weather_cells = lambda state: []
    fo._emit = Mock()
    fo._set_error = Mock()
    fo.sim = None
    fo.enroute_weather = None
    return fo


def test_departure_deviate_mode_holds_until_scan_evidence_exists():
    fo = _fo_departure_fixture("deviate")
    state = AircraftState(latitude=50.0, longitude=8.0,
                          on_ground=True)
    assert not fo._departure_wind_ok(state)
    assert fo.state.convective_hold_said


def test_departure_deviate_mode_departs_when_fresh_scan_is_clear():
    fo = _fo_departure_fixture("deviate")
    fo.sim = SimpleNamespace(weather_radar_points=lambda: [
        RadarPoint(50.03, 8.0, 0.0, 3000.0, 0.1, 0.0, time.time()),
        RadarPoint(50.11, 8.0, 0.0, 3000.0, 0.1, 0.0, time.time()),
        RadarPoint(50.21, 8.0, 0.0, 3000.0, 0.1, 0.0, time.time())])
    state = AircraftState(latitude=50.0, longitude=8.0,
                          on_ground=True)
    assert fo._departure_wind_ok(state)
    assert not fo.state.convective_hold_said


def test_departure_deviate_mode_holds_when_corridor_has_a_cell():
    fo = _fo_departure_fixture("deviate")
    fo.enroute_weather = SimpleNamespace(
        _at=time.time(), _key=(50, 8), EXPIRE_S=900.0,
        _key_for=lambda lat, lon: (round(lat), round(lon)))
    fo.sim = SimpleNamespace(weather_radar_points=lambda: [
        RadarPoint(50.03, 8.0, 0.0, 3000.0, 0.1, 0.0, time.time()),
        RadarPoint(50.11, 8.0, 0.0, 3000.0, 0.1, 0.0, time.time()),
        RadarPoint(50.21, 8.0, 0.0, 3000.0, 0.1, 0.0, time.time())])
    cell = HazardCell(50.15, 8.0, 5.0, .8, "test", "cell")
    fo._enroute_weather_cells = lambda state: [cell]
    state = AircraftState(latitude=50.0, longitude=8.0,
                          on_ground=True)
    assert not fo._departure_wind_ok(state)


def test_departure_bypass_ignores_cell_but_not_wind_limits():
    fo = _fo_departure_fixture("bypass")
    state = AircraftState(latitude=50.0, longitude=8.0,
                          on_ground=True)
    assert fo._departure_wind_ok(state)
    fo._wind_limits = lambda: (0.0, 0.0)
    fo._runway_wind = lambda weather, heading: (0.0, 5.0, 0.0)
    assert not fo._departure_wind_ok(state)
