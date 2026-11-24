import time
from unittest.mock import Mock

from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
from meridianatc.atc.session import FlightSession
from meridianatc.sim.state import AircraftState
from meridianatc.weather.enroute import HazardCell


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
