"""Live aeroplanes adopted by traffic.live.managed are flown like the
invented ones, and handed back to the feed when the switch goes off."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest


def _injector(**over):
    """A bare injector with the state _adopt_live/_unadopt/_rejoin_the_feed
    and _drop_invented touch, the helpers they call mocked."""
    from meridianatc.atc.traffic.core import TrafficInjector
    injector = TrafficInjector.__new__(TrafficInjector)
    injector.field = SimpleNamespace(
        ident="EBBR", latitude=50.9, longitude=4.48, elevation_ft=184.0,
        arrival_runway="25R", departure_runway="25R")
    injector.ground = None
    injector.hold_live = Mock()
    injector._taken_over = set()
    injector._flown_for = {}
    injector._parked_since = {}
    injector._live_flew = set()
    injector._live_moving = set()
    injector._used_callsigns = set()
    injector.flights = []
    injector.statics = []
    injector._materialising = []
    injector._asked_for = []
    injector._materialise_by = {}
    injector.pending = []
    injector._stands_filled = False
    injector._final_distance = Mock(return_value=None)
    injector._on_runway = Mock(return_value=False)
    injector._free_stand = Mock(return_value=None)
    injector._threshold = Mock(return_value=(50.901, 4.484, 247.0))
    injector._route_end = Mock(return_value=("Malaga", "LEMG"))
    injector._nearby_field = Mock(return_value=None)
    injector._elsewhere = Mock(return_value="ANTWERP")
    injector._release_all = Mock()
    for key, value in over.items():
        setattr(injector, key, value)
    return injector


def _flight(**over):
    """An observed live strip as _observed_flight leaves it."""
    flight = SimpleNamespace(
        written="BAW123", type_code="A320", category="airline",
        operator="BAW", language="en", ifr=True,
        observed=True, observed_id=12345, source="live",
        origin="EGLL", destination="EBBR", destination_icao="EBBR",
        hex="abc123", squawk="1000",
        arriving=True, runway="25R", phase="final", step=0, next_at=0.0,
        distance_nm=8.0, airborne_model=False, on_ground=False,
        on_layout=False, taxi=None, line_up=None, planned=None, stand=None,
        commands=[], doing=None, lead_at=None, lead_distance_nm=0.0,
        live_held="", assigned_heading=None, assigned_altitude_ft=None,
        assigned_speed_kt=None, target=SimpleNamespace(clear=lambda: None),
        extending=False, held_since=0.0, static=False, parked_at=None,
        done=False, retire=False, drawn=True, boarded_at=None,
        latitude=50.9, longitude=4.5, heading=250.0, altitude_ft=3000.0,
        ground_speed_kt=180.0, vertical_rate_fpm=-800.0, performance=None,
        cleared=False, pattern_direction="left", pattern_work=False)
    for key, value in over.items():
        setattr(flight, key, value)
    return flight


def _seen(**over):
    """A LiveSeen-shaped answer from the feed: a real BAW123 EGLL-EBBR on
    the descent."""
    one = SimpleNamespace(
        object_id=12345, hex="abc123", callsign="BAW123", source="live",
        feed="adsb.lol", on_ground=False,
        latitude=50.9, longitude=4.5, heading=250.0, altitude_ft=3000.0,
        ground_speed_kt=180.0, vertical_speed_fpm=-800.0,
        origin="EGLL", destination="EBBR", track=250, squawk="1000")
    for key, value in over.items():
        setattr(one, key, value)
    return one


def test_a_live_arrival_on_final_is_adopted_into_the_model():
    injector = _injector(_final_distance=Mock(return_value=6.0))
    flight = _flight()
    assert injector._adopt_live(flight, _seen(), 100.0)
    assert flight.observed is False
    assert flight.airborne_model is True
    assert flight.arriving is True
    assert flight.phase == "final"
    assert flight.distance_nm == 6.0
    assert flight.runway == "25R"
    # The feed's copy of it is suppressed until ours parks.
    assert flight.observed_id in injector._taken_over
    assert injector._flown_for[flight.observed_id] is flight
    assert flight.performance is not None
    assert flight.next_at == pytest.approx(101.0)


def test_a_live_arrival_not_yet_lined_up_joins_inbound():
    from meridianatc.navdata.build import haversine_nm
    injector = _injector()
    flight = _flight()
    assert injector._adopt_live(flight, _seen(), 100.0)
    assert flight.phase == "inbound"
    # Distance seeded from where it actually is, not from the approach.
    assert flight.distance_nm == pytest.approx(
        haversine_nm(50.9, 4.5, 50.901, 4.484))


def test_a_live_departure_climbing_out_is_adopted():
    injector = _injector()
    flight = _flight(origin="EBBR", destination="EGLL", arriving=False)
    one = _seen(origin="EBBR", destination="EGLL",
                vertical_speed_fpm=1500.0)
    assert injector._adopt_live(flight, one, 100.0)
    assert flight.arriving is False
    assert flight.phase == "departed"
    assert flight.observed is False


def test_an_overflight_is_left_to_the_feed():
    """No route and not on final: just passing, nobody's to fly it."""
    injector = _injector()
    flight = _flight(origin="", destination="")
    assert injector._adopt_live(flight, _seen(origin="", destination=""),
                                100.0) is False
    assert flight.observed is True
    assert not injector._taken_over


def test_a_parked_live_aeroplane_is_left_to_the_feed():
    """Adopted when the real one starts moving, not on its stand."""
    stand = SimpleNamespace(index=3)
    ground = Mock()
    ground.to_xy.return_value = (0.0, 0.0)
    ground.stand_at.return_value = stand
    injector = _injector(ground=ground)
    flight = _flight(origin="", destination="", on_ground=True)
    one = _seen(on_ground=True, ground_speed_kt=0.0,
                vertical_speed_fpm=0.0, origin="", destination="")
    assert injector._adopt_live(flight, one, 100.0) is False
    assert flight.observed is True


def test_a_taxiing_arrival_is_adopted_onto_a_stand_route():
    stand = SimpleNamespace(index=7)
    taxi = object()
    ground = Mock()
    ground.to_xy.return_value = (100.0, 50.0)
    ground.stand_at.return_value = None
    ground.to_stand.return_value = taxi
    injector = _injector(ground=ground,
                         _free_stand=Mock(return_value=stand))
    injector._live_flew.add("abc123")
    flight = _flight(on_ground=True)
    one = _seen(on_ground=True, ground_speed_kt=15.0,
                vertical_speed_fpm=0.0)
    assert injector._adopt_live(flight, one, 100.0)
    assert flight.phase == "taxi_in"
    assert flight.arriving is True
    assert flight.on_layout is True
    assert flight.taxi is taxi
    assert flight.stand == 7
    # A ground movement, not an airborne one.
    assert not flight.airborne_model


def test_a_taxiing_departure_is_adopted_for_the_tower_to_plan():
    ground = Mock()
    stand = SimpleNamespace(index=11)
    ground.to_xy.return_value = (10.0, 20.0)
    ground.stand_at.return_value = stand
    injector = _injector(ground=ground)
    flight = _flight(origin="EBBR", destination="EGLL", on_ground=True,
                     arriving=False)
    one = _seen(on_ground=True, ground_speed_kt=12.0,
                vertical_speed_fpm=0.0, origin="EBBR", destination="EGLL")
    assert injector._adopt_live(flight, one, 100.0)
    assert flight.phase == "taxi"
    assert flight.arriving is False
    assert flight.on_layout is True
    # The real route's destination is kept, not made up.
    assert flight.destination == "EGLL"


def test_a_departure_without_a_route_is_given_one():
    ground = Mock()
    ground.to_xy.return_value = (10.0, 20.0)
    ground.stand_at.return_value = None
    injector = _injector(ground=ground)
    flight = _flight(origin="EBBR", destination="", on_ground=True,
                     arriving=False)
    one = _seen(on_ground=True, ground_speed_kt=12.0, origin="EBBR",
                destination="")
    assert injector._adopt_live(flight, one, 100.0)
    assert flight.destination == "Malaga"
    assert flight.destination_icao == "LEMG"


def test_a_landed_aeroplane_rolls_out_under_the_model():
    injector = _injector(_on_runway=Mock(return_value=True))
    flight = _flight(on_ground=True)
    one = _seen(on_ground=True, ground_speed_kt=80.0,
                vertical_speed_fpm=0.0)
    assert injector._adopt_live(flight, one, 100.0)
    assert flight.phase == "landed"
    assert flight.airborne_model is True


def test_an_adopted_aeroplane_goes_back_to_the_feed():
    injector = _injector()
    flight = _flight(observed=False, airborne_model=True, on_layout=True,
                     taxi=object())
    injector._taken_over.add(flight.observed_id)
    injector._flown_for[flight.observed_id] = flight
    injector._parked_since[flight.observed_id] = 50.0
    flight.commands.append(object())
    injector.pending.append(SimpleNamespace(flight=flight))
    injector._unadopt(flight, 100.0)
    assert flight.observed is True
    assert flight.airborne_model is False
    assert flight.on_layout is False
    assert flight.taxi is None
    assert flight.commands == []
    assert not injector._taken_over
    assert not injector._flown_for
    assert not injector._parked_since
    # The exchange half said is not said to the feed's aeroplane.
    assert injector.pending == []


def test_an_adopted_parked_aeroplane_leaves_the_stand_to_the_real_one():
    injector = _injector()
    flight = _flight(observed=False, static=True, parked_at=40.0)
    injector.statics = [flight]
    injector._taken_over.add(flight.observed_id)
    injector._flown_for[flight.observed_id] = flight
    injector._unadopt(flight, 100.0)
    assert flight.observed is True
    assert flight.static is False
    assert injector.statics == []


def test_the_invented_switch_leaves_an_adopted_aeroplane_be():
    injector = _injector()
    live = _flight(observed=False, source="live")
    taken = _flight(observed=False, source="taken over")
    made = _flight(observed=False, source="")
    fed = _flight(observed=True, source="live")
    injector.flights = [live, taken, made, fed]
    injector.statics = [_flight(observed=False, source="live"),
                        _flight(observed=False, source="")]
    injector._drop_invented()
    assert live in injector.flights
    assert taken in injector.flights
    assert fed in injector.flights
    assert made not in injector.flights
    assert [f.source for f in injector.statics] == ["live"]


def test_a_moving_adopted_aeroplane_is_not_killed_by_the_feed():
    """Ours parked, boarded and taxiing again: the real aeroplane heard on
    its stand does not put down our departure."""
    injector = _injector()
    ours = _flight(observed=False, source="live")
    injector.flights = [ours]
    injector._flown_for[12345] = ours
    injector._parked_since[12345] = 10.0
    injector._taken_over.add(12345)
    real = _seen()
    out = injector._rejoin_the_feed([real], 100.0)
    assert not ours.done
    assert 12345 in injector._taken_over
    assert real not in out  # the feed's copy stays suppressed


def test_a_parked_adopted_aeroplane_rejoins_when_the_real_one_is_heard():
    injector = _injector()
    ours = _flight(observed=False, source="live", parked_at=50.0)
    injector.flights = [ours]
    injector._flown_for[12345] = ours
    injector._parked_since[12345] = 10.0
    injector._taken_over.add(12345)
    real = _seen()
    out = injector._rejoin_the_feed([real], 100.0)
    assert ours.done
    assert 12345 not in injector._taken_over
    assert real in out  # the feed's aeroplane is heard again
