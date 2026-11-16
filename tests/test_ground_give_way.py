"""The give-way on the ground: a head-on is met once, early, by the one
that should wait -- and resolved round another way before it becomes the
tow.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

from meridianatc.atc.ground import (
    PATH_PARKING,
    PATH_TAXI,
    GroundLayout,
    Taxiing,
)
from meridianatc.atc.traffic import TrafficInjector
from meridianatc.atc.traffic._shared import Flight


def _layout() -> GroundLayout:
    """A single stretch A--B--C with a loop round the south, a stand at
    each end."""
    points = [
        (0, 0, 0.0, 0.0),      # 0 -- west end
        (0, 0, 100.0, 0.0),    # 1 -- middle of the stretch
        (0, 0, 200.0, 0.0),    # 2 -- east end
        (0, 0, 0.0, 100.0),    # 3 -- the loop, west
        (0, 0, 200.0, 100.0),  # 4 -- the loop, east
    ]
    parkings = [
        (8, 0, "G1", "", 1, 0, 90.0, 20.0, 0.0, 20.0),
        (8, 0, "G2", "", 2, 0, 90.0, 20.0, 200.0, 20.0),
    ]
    paths = [
        (PATH_TAXI, 0, 0, "A", 0, 1, -1),
        (PATH_TAXI, 0, 0, "A", 1, 2, -1),
        (PATH_TAXI, 0, 0, "B", 0, 3, -1),
        (PATH_TAXI, 0, 0, "B", 3, 4, -1),
        (PATH_TAXI, 0, 0, "B", 4, 2, -1),
        (PATH_PARKING, 0, 0, "", 3, 0, -1),
        (PATH_PARKING, 0, 0, "", 2, 1, -1),
    ]
    return GroundLayout("TEST", 51.0, 0.0, points, parkings, paths,
                        ["A", "B"])


def _injector(layout: GroundLayout) -> TrafficInjector:
    navdb = Mock()
    navdb.airport.return_value = None
    injector = TrafficInjector(navdb, seed=1, density=1.0)
    injector.field.ident = "TEST"
    injector.field.latitude = 51.0
    injector.field.longitude = 0.0
    injector.field.departure_runway = "17"
    injector.field.arrival_runway = "17"
    injector.ground = layout
    injector.user = None
    return injector


def _arriving(injector, written: str, stand: int, x: float, z: float,
              nodes: list) -> Flight:
    flight = Flight(written=written, type_code="C172", aircraft=Mock(),
                    language="en", operator=None, arriving=True)
    flight.ifr = False
    flight.phase = "taxi_in"
    flight.stand = stand
    flight.latitude, flight.longitude = injector.ground.to_latlon(x, z)
    flight.on_ground = True
    flight.ground_speed_kt = 10.0
    flight.taxi = Taxiing(
        points=[injector.ground.xy[n] for n in nodes],
        nodes=list(nodes))
    return flight


def _head_on_pair(injector):
    """Two arrivals meeting on the A--B--C stretch: AAA1 westward at the
    west end heading east, ZZZ9 eastward at the east end heading west."""
    f1 = _arriving(injector, "AAA1", 1, 0.0, 0.0, [("p", 0), ("p", 1),
                                                 ("p", 2)])
    f2 = _arriving(injector, "ZZZ9", 0, 200.0, 0.0, [("p", 2), ("p", 1),
                                                   ("p", 0)])
    injector.flights = [f1, f2]
    return f1, f2


def test_the_yield_is_decided_before_the_windscreens():
    injector = _injector(_layout())
    f1, f2 = _head_on_pair(injector)
    # The meeting is real: the stretch between them is shared, reversed.
    assert injector._head_on_ahead(f1) is None
    assert injector._head_on_ahead(f2) is f1


def test_the_yielder_holds_and_the_other_goes():
    injector = _injector(_layout())
    f1, f2 = _head_on_pair(injector)
    x2, z2 = injector.ground.to_xy(f2.latitude, f2.longitude)
    assert injector._taxi_blocked(f2, x2, z2, 0.0, 100.0) \
        == "holding for traffic"
    assert f2._giving_way == "AAA1"
    # And the one with the way is not asked to wait for the waiter.
    x1, z1 = injector.ground.to_xy(f1.latitude, f1.longitude)
    assert injector._taxi_blocked(f1, x1, z1, 90.0, 100.0) == ""


def test_a_stopped_yielder_is_the_give_way_not_the_obstacle():
    injector = _injector(_layout())
    f1, f2 = _head_on_pair(injector)
    f2.ground_speed_kt = 0.0
    f2._giving_way = "AAA1"
    # Nose to nose on the shared stretch: the one that goes passes.
    f1.latitude, f1.longitude = injector.ground.to_latlon(80.0, 0.0)
    f2.latitude, f2.longitude = injector.ground.to_latlon(120.0, 0.0)
    x1, z1 = injector.ground.to_xy(f1.latitude, f1.longitude)
    assert injector._in_the_way(f1, x1, z1) is None


def test_the_wait_that_outlasts_its_welcome_routes_round():
    injector = _injector(_layout())
    f1, f2 = _head_on_pair(injector)
    f2.taxi.blocked_since = 50.0
    f2.ground_speed_kt = 0.0
    f2._giving_way = "AAA1"
    injector._break_deadlock(f2, 50.0 + injector.GIVE_WAY_S + 1.0)
    # Routed round the loop, not taken away: the new way does not use the
    # contested stretch.
    assert not f2.done
    nodes = set(f2.taxi.nodes)
    assert ("p", 1) not in nodes
    assert ("k", 0) in nodes


def test_nose_to_nose_with_the_pilot_is_rerouted_not_held():
    injector = _injector(_layout())
    f1, f2 = _head_on_pair(injector)
    # The pilot parked on the stretch instead of f1: f2's way to its stand
    # has the loop round the south for it.
    injector.user = SimpleNamespace(
        on_ground=True, has_position=True,
        latitude=injector.ground.to_latlon(120.0, 0.0)[0],
        longitude=injector.ground.to_latlon(120.0, 0.0)[1])
    injector.flights = [f2]
    f2.taxi.blocked_since = 50.0
    f2.ground_speed_kt = 0.0
    injector._break_deadlock(f2, 50.0 + injector.GIVE_WAY_S + 1.0)
    assert not f2.done
    assert ("p", 1) not in set(f2.taxi.nodes)
