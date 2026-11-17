"""The pilot's own aeroplane as the runway's immovable occupier.

Two cases the ground model used to leave alone: a departure cleared to
line up and wait *behind* the pilot when the pilot's only way off is back
the way it came -- the head-on on the strip -- and the resulting
nose-to-nose, which the deadlock breaker ignored because ``"you"`` is not
a ``Flight`` it could tow away.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

from meridianatc.atc.traffic import TrafficInjector
from meridianatc.atc.traffic._shared import Flight


def _injector():
    navdb = Mock()
    navdb.airport.return_value = None
    injector = TrafficInjector(navdb, seed=1, density=1.0)
    injector.field.ident = "LPPT"
    injector.field.latitude = 38.77
    injector.field.longitude = -9.13
    injector.field.departure_runway = "17"
    injector.field.arrival_runway = "17"
    injector._exchange = Mock()
    return injector


def _departure(**kw) -> Flight:
    flight = Flight(written="TAP1", type_code="B738", aircraft=Mock(),
                    language="en", operator="TAP", arriving=False)
    flight.ifr = True
    flight.phase = "ready"
    flight.step = 0
    flight.runway = "17"
    flight.latitude = 41.26
    flight.longitude = -8.68
    flight.on_ground = True
    flight.ground_speed_kt = 0.0
    flight.next_at = 0.0
    for key, value in kw.items():
        setattr(flight, key, value)
    return flight


# ------------------------------------------------------------------
# never line up and wait behind a pilot who can only come back at you
# ------------------------------------------------------------------

def _ready_the_gate(injector, *, user_on: bool, user_kt: float = 0.0):
    injector.runway_occupied = lambda now, ignore=None, runway="": True
    injector._runway_taken_by_departure = lambda flight: False
    injector._room_to_depart = lambda flight, now: True
    injector._user_on_runway = lambda runway="": user_on
    injector.user = SimpleNamespace(on_ground=True,
                                    ground_speed_kt=user_kt)


def test_no_line_up_and_wait_behind_a_stopped_pilot():
    injector = _injector()
    _ready_the_gate(injector, user_on=True)
    flight = _departure()
    injector._work_the_departure(flight, 100.0)
    assert flight.next_at > 100.0
    assert not injector._exchange.called


def test_line_up_and_wait_is_still_issued_behind_a_roller():
    injector = _injector()
    _ready_the_gate(injector, user_on=True, user_kt=40.0)
    flight = _departure()
    injector._work_the_departure(flight, 100.0)
    assert flight.step == 2
    assert injector._exchange.called
    kinds = [c.kwargs.get("command", (None,))[0]
             for c in injector._exchange.mock_calls]
    assert "line_up" in kinds


def test_line_up_and_wait_survives_a_pilot_off_the_runway():
    injector = _injector()
    _ready_the_gate(injector, user_on=False)
    flight = _departure()
    injector._work_the_departure(flight, 100.0)
    assert flight.step == 2


# ------------------------------------------------------------------
# nose to nose with the pilot: ours moves, never the pilot
# ------------------------------------------------------------------

def _stuck_on_the_strip(injector, flight):
    flight.taxi = SimpleNamespace(blocked_since=0.0)
    injector.ground = SimpleNamespace(
        to_xy=lambda lat, lon: (lon * 1000.0, lat * 1000.0))
    injector._on_runway_itself = lambda f: True
    injector._in_the_way = lambda f, x, z: "you"


def test_the_standoff_with_the_pilot_is_vacated_not_held():
    injector = _injector()
    flight = _departure()
    _stuck_on_the_strip(injector, flight)
    injector._vacate_the_runway = Mock(return_value=True)
    injector._take_off_the_runway = Mock()
    injector._break_deadlock(flight, 100.0)
    assert injector._vacate_the_runway.called
    assert flight._vacated_for_user
    assert not injector._take_off_the_runway.called
    assert not flight.done


def test_still_there_on_the_next_check_means_off():
    injector = _injector()
    flight = _departure()
    _stuck_on_the_strip(injector, flight)
    flight._vacated_for_user = True
    injector._vacate_the_runway = Mock(return_value=True)
    injector._take_off_the_runway = Mock()
    injector._break_deadlock(flight, 100.0)
    assert injector._take_off_the_runway.called
    assert not injector._vacate_the_runway.called


def test_no_way_back_means_off():
    injector = _injector()
    flight = _departure()
    _stuck_on_the_strip(injector, flight)
    injector._vacate_the_runway = Mock(return_value=False)
    injector._take_off_the_runway = Mock()
    injector._break_deadlock(flight, 100.0)
    assert injector._take_off_the_runway.called


def test_a_live_one_goes_back_to_the_feed():
    injector = _injector()
    flight = _departure(source="live")
    injector._unadopt = Mock()
    injector._release_all = Mock()
    injector._take_off_the_runway(flight, 100.0)
    injector._unadopt.assert_called_once_with(flight, 100.0)
    assert not flight.done


def test_an_invented_one_is_towed():
    injector = _injector()
    flight = _departure()
    injector._unadopt = Mock()
    injector._release_all = Mock()
    injector._take_off_the_runway(flight, 100.0)
    assert flight.done
    assert injector._release_all.called
    assert not injector._unadopt.called


# ------------------------------------------------------------------
# the live ones answer to the same runway the invented ones do
# ------------------------------------------------------------------

def _observed(injector, flight):
    flight.observed = True
    flight.source = "live"
    flight.hex = "4d0123"
    flight.live_held = ""
    injector.flights = [flight]
    injector._rest_of_taxi = Mock()


def test_a_live_line_up_is_not_said_over_an_occupied_strip():
    injector = _injector()
    flight = _departure(observed=True, source="live", phase="lineup")
    _observed(injector, flight)
    injector.runway_occupied = lambda now, ignore=None, runway="": True
    injector._advance_observed(flight, 100.0)
    assert flight.step == 0
    assert not injector._exchange.called


def test_a_live_line_up_is_said_when_only_it_occupies():
    injector = _injector()
    flight = _departure(observed=True, source="live", phase="lineup")
    _observed(injector, flight)
    injector.runway_occupied = lambda now, ignore=None, runway="": False
    injector._runway_occupier = Mock(return_value=None)
    injector._lineup_reason = Mock(return_value="")
    injector._advance_observed(flight, 100.0)
    assert flight.step == 1
    assert injector._exchange.called


def _pilot_protection(injector, *, near: bool):
    injector.user = SimpleNamespace(on_ground=True, ground_speed_kt=0.0)
    injector._user_on_runway = lambda runway="": True
    injector._user_on_final = lambda: None
    injector._on_runway = lambda flight: False
    injector._near_departure_runway = lambda flight: near
    injector._pilot_ahead = lambda flight, now=0.0: False
    injector.hold_live = Mock(return_value=True)


def test_a_taxiing_live_departure_is_held_at_the_strips_edge():
    injector = _injector()
    flight = _departure(observed=True, source="live", phase="taxi")
    _observed(injector, flight)
    _pilot_protection(injector, near=True)
    injector._protect_the_pilot(100.0)
    assert flight.live_held == "runway"
    injector.hold_live.assert_called_once_with(flight.hex, True)


def test_a_taxiing_live_departure_away_from_the_strip_is_left():
    injector = _injector()
    flight = _departure(observed=True, source="live", phase="taxi")
    _observed(injector, flight)
    _pilot_protection(injector, near=False)
    injector._protect_the_pilot(100.0)
    assert flight.live_held == ""
    assert not injector.hold_live.called


def test_a_live_departure_about_to_enter_is_still_held():
    injector = _injector()
    flight = _departure(observed=True, source="live", phase="takeoff")
    _observed(injector, flight)
    _pilot_protection(injector, near=False)
    injector._protect_the_pilot(100.0)
    assert flight.live_held == "runway"
