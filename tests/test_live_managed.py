"""Live aeroplanes adopted by traffic.live.managed are flown like the
invented ones, and handed back to the feed when the switch goes off."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

THRESHOLD_25R = (50.901, 4.484, 247.0)


def _on_strip(threshold, along_nm=0.0, across_nm=0.0):
    """lat/lon ``along_nm`` down the strip and ``across_nm`` right of it."""
    lat0, lon0, course = threshold
    rad = math.radians(course)
    north = along_nm * math.cos(rad) - across_nm * math.sin(rad)
    east = along_nm * math.sin(rad) + across_nm * math.cos(rad)
    return (lat0 + north / 60.0,
            lon0 + east / (60.0 * math.cos(math.radians(lat0))))


def _injector(**over):
    """A bare injector with the state _adopt_live/_unadopt/_rejoin_the_feed
    and _drop_invented touch, the helpers they call mocked."""
    from meridianatc.atc.traffic.core import TrafficInjector
    injector = TrafficInjector.__new__(TrafficInjector)
    injector.field = SimpleNamespace(
        ident="EBBR", latitude=50.9, longitude=4.48, elevation_ft=184.0,
        arrival_runway="25R", departure_runway="25R", stations={},
        thresholds={"25R": THRESHOLD_25R},
        surfaces={"25R": (184.0, 184.0, 2.0)})
    injector.radius_nm = TrafficInjector.OUT_OF_SIGHT_NM
    injector.observed_nm = 30.0
    injector.ground = None
    injector.hold_live = Mock()
    injector._taken_over = set()
    injector._flown_for = {}
    injector._parked_since = {}
    injector._adopted_live = set()
    injector._live_flew = set()
    injector._live_moving = set()
    injector._live_stands = {}
    injector._used_callsigns = set()
    injector.flights = []
    injector.statics = []
    injector._materialising = []
    injector._asked_for = []
    injector._materialise_by = {}
    injector.pending = []
    injector._stands_filled = False
    injector._final_distance = Mock(return_value=None)
    injector._free_stand = Mock(return_value=None)
    injector._threshold = Mock(return_value=(50.901, 4.484, 247.0))
    injector._route_end = Mock(return_value=("Malaga", "LEMG"))
    injector._nearby_field = Mock(return_value=None)
    injector._elsewhere = Mock(return_value="ANTWERP")
    injector._release_all = Mock()
    injector.can_push = None
    injector.random = SimpleNamespace(
        choice=lambda seq: seq[0], uniform=lambda a, b: a,
        sample=lambda seq, k: list(seq)[:k])
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
        cleared=False, pattern_direction="left", pattern_work=False,
        handed_off=False, taxi_rest=[], taxi_split_xy=None)
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


def test_a_flight_bound_elsewhere_is_not_this_arrival():
    """RYR893Z, LIMC to Porto, descended over Vigo inside the shape a
    final makes and was adopted, landed, and sent to the ramp -- for a
    field it never filed. The route the feed looked up outranks the
    coincidence of geometry: bound for somewhere it knows, it is an
    overflight whatever it is lined up on."""
    injector = _injector(_final_distance=Mock(return_value=5.0))
    flight = _flight(origin="LIMC", destination="LPPR", arriving=False)
    one = _seen(origin="LIMC", destination="LPPR",
                vertical_speed_fpm=-1200.0)
    assert injector._adopt_live(flight, one, 100.0) is False
    assert flight.observed is True
    assert not injector._taken_over


def test_a_transit_the_feed_calls_ground_is_still_an_overflight():
    """TAP36RC, Lisbon to Porto, was called on_ground by the feed on
    the low pass over Vigo, and having been seen flying here it was
    adopted a taxi-in and sent to a ramp at a field it never filed.
    The route outranks the wheels' word too: bound for somewhere the
    feed knows, it is an overflight whatever the flags say."""
    ground = Mock()
    ground.to_xy.return_value = (100.0, 50.0)
    ground.stand_at.return_value = None
    ground.to_stand.return_value = object()
    injector = _injector(
        ground=ground,
        _free_stand=Mock(return_value=SimpleNamespace(index=7)))
    injector._live_flew.add("abc123")
    flight = _flight(origin="LPPT", destination="LPPR", on_ground=True)
    one = _seen(on_ground=True, ground_speed_kt=15.0,
                vertical_speed_fpm=0.0, origin="LPPT", destination="LPPR")
    assert injector._adopt_live(flight, one, 100.0) is False
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
    injector = _injector()
    flight = _flight(on_ground=True)
    lat, lon = _on_strip(THRESHOLD_25R, along_nm=0.3)
    one = _seen(on_ground=True, ground_speed_kt=80.0,
                vertical_speed_fpm=0.0, latitude=lat, longitude=lon,
                heading=247.0)
    assert injector._adopt_live(flight, one, 100.0)
    assert flight.phase == "landed"
    assert flight.airborne_model is True


def test_a_live_departure_lined_up_is_adopted_on_the_centreline():
    """A departure holding on the runway is adopted -- snapped onto the
    centreline it was read near, not at the receiver's read off it."""
    injector = _injector()
    flight = _flight(on_ground=True, arriving=False,
                     origin="EBBR", destination="EGLL")
    lat, lon = _on_strip(THRESHOLD_25R, along_nm=0.05, across_nm=0.02)
    one = _seen(on_ground=True, ground_speed_kt=0.0,
                vertical_speed_fpm=0.0, latitude=lat, longitude=lon,
                heading=247.0, origin="EBBR", destination="EGLL")
    assert injector._adopt_live(flight, one, 100.0)
    assert flight.phase == "lineup"
    assert flight.runway == "25R"
    expected_lat, expected_lon = _on_strip(THRESHOLD_25R, along_nm=0.05)
    assert flight.latitude == pytest.approx(expected_lat, abs=1e-5)
    assert flight.longitude == pytest.approx(expected_lon, abs=1e-5)


def test_a_live_departure_read_off_the_side_is_left_to_the_feed():
    """On the runway's wide box but eighty metres off the centreline --
    where what it is really on is a parallel taxiway this read cannot
    tell apart. Adopted, ours rolled the runway's heading down the
    taxiway; the feed's own drawing of it stays instead."""
    injector = _injector()
    flight = _flight(on_ground=True, arriving=False,
                     origin="EBBR", destination="EGLL")
    lat, lon = _on_strip(THRESHOLD_25R, along_nm=0.05, across_nm=0.045)
    one = _seen(on_ground=True, ground_speed_kt=0.0,
                vertical_speed_fpm=0.0, latitude=lat, longitude=lon,
                heading=247.0, origin="EBBR", destination="EGLL")
    assert injector._adopt_live(flight, one, 100.0) is False
    assert flight.observed is True
    assert not injector._taken_over


def test_a_live_aeroplane_crossing_the_runway_is_left_to_the_feed():
    """On the pavement but pointed across it is crossing, not lined up."""
    injector = _injector()
    flight = _flight(on_ground=True, arriving=False,
                     origin="EBBR", destination="EGLL")
    lat, lon = _on_strip(THRESHOLD_25R, along_nm=0.5)
    one = _seen(on_ground=True, ground_speed_kt=15.0,
                vertical_speed_fpm=0.0, latitude=lat, longitude=lon,
                heading=157.0, origin="EBBR", destination="EGLL")
    assert injector._adopt_live(flight, one, 100.0) is False
    assert flight.observed is True


def test_a_live_departure_too_far_down_the_strip_is_left_to_the_feed():
    """Adopted this far along, ours ran its whole roll off what little
    runway was left and out over the far end: a backtracker on the
    feed's own picture stays the feed's."""
    injector = _injector()
    flight = _flight(on_ground=True, arriving=False,
                     origin="EBBR", destination="EGLL")
    lat, lon = _on_strip(THRESHOLD_25R, along_nm=1.8)
    one = _seen(on_ground=True, ground_speed_kt=0.0,
                vertical_speed_fpm=0.0, latitude=lat, longitude=lon,
                heading=247.0, origin="EBBR", destination="EGLL")
    assert injector._adopt_live(flight, one, 100.0) is False
    assert flight.observed is True
    assert not injector._taken_over


def test_a_live_departure_rolling_is_adopted_as_takeoff():
    """Already rolling, it needs only what is left of its own roll --
    that fits wherever the feed caught it."""
    injector = _injector()
    flight = _flight(on_ground=True, arriving=False,
                     origin="EBBR", destination="EGLL")
    lat, lon = _on_strip(THRESHOLD_25R, along_nm=1.2)
    one = _seen(on_ground=True, ground_speed_kt=120.0,
                vertical_speed_fpm=0.0, latitude=lat, longitude=lon,
                heading=247.0, origin="EBBR", destination="EGLL")
    assert injector._adopt_live(flight, one, 100.0)
    assert flight.phase == "takeoff"
    assert flight.runway == "25R"


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


def test_a_live_aeroplane_is_adopted_once():
    """Handed back to the feed it is read rather than flown again: adopted
    a second time it would be worked and handed off over and over for as
    long as the real aeroplane stayed in reach."""
    injector = _injector()
    flight = _flight(origin="EBBR", destination="EGLL", arriving=False)
    one = _seen(origin="EBBR", destination="EGLL",
                vertical_speed_fpm=1500.0)
    assert injector._adopt_live(flight, one, 100.0)
    assert flight.observed_id in injector._adopted_live
    # Its strip back on the feed: not ours to fly again.
    flight.observed = True
    assert injector._adopt_live(flight, one, 200.0) is False
    assert flight.observed is True


def test_an_adopted_departure_out_of_sight_is_handed_back_to_the_feed():
    """A live departure ours has flown out of sight is not dropped: the
    real aeroplane is still out there inside the feed's wider reach, and
    its strip goes back to reading it rather than reappearing to be
    adopted all over again."""
    injector = _injector()
    flight = _flight(observed=False, source="live", arriving=False,
                     phase="departed", handed_off=True, distance_nm=13.0)
    injector._taken_over.add(flight.observed_id)
    injector._flown_for[flight.observed_id] = flight
    injector._adopted_live.add(flight.observed_id)
    injector._advance_departure(flight, 100.0)
    assert flight.observed is True
    assert not flight.done
    assert flight.observed_id not in injector._taken_over
    assert flight.observed_id not in injector._flown_for


def test_an_invented_departure_out_of_sight_is_done():
    injector = _injector()
    flight = _flight(observed=False, source="", arriving=False,
                     phase="departed", handed_off=True, distance_nm=13.0)
    injector._advance_departure(flight, 100.0)
    assert flight.done
    assert flight.observed is False


def test_a_handed_off_departure_back_on_the_feed_is_not_told_twice():
    injector = _injector()
    flight = _flight(handed_off=True, phase="departed", arriving=False)
    injector._advance_observed(flight, 100.0)
    assert injector.pending == []
    assert flight.step == 1


def test_out_of_sight_is_the_configured_radius_not_twelve_miles():
    """A handed-off departure stays while it is inside the draw radius
    the traffic was configured with: thirteen miles was the old fixed
    figure, and at radius_nm=100 it is still ours at twenty."""
    injector = _injector(radius_nm=100.0)
    flight = _flight(observed=False, source="", arriving=False,
                     phase="departed", handed_off=True, distance_nm=20.0,
                     altitude_ft=6000.0)
    injector._advance_departure(flight, 100.0)
    assert not flight.done
    # And still gone the moment it is past the radius.
    flight.distance_nm = 101.0
    injector._advance_departure(flight, 105.0)
    assert flight.done


def test_the_default_radius_is_the_old_figure():
    """A caller that never heard of the setting gets what it always
    got: the departure out of sight at twelve miles."""
    injector = _injector()
    flight = _flight(observed=False, source="", arriving=False,
                     phase="departed", handed_off=True, distance_nm=13.0,
                     altitude_ft=6000.0)
    injector._advance_departure(flight, 100.0)
    assert flight.done


def test_the_feed_gate_is_the_configured_radius_too():
    """The feed is asked out to live.radius_nm (traffic.radius_nm by
    default), but the strip gate was a fixed thirty miles: at
    radius_nm=100 a real aeroplane the feed heard at forty was fetched
    and never shown. The gate follows the setting now."""
    injector = _injector(observed_nm=100.0)
    far = _seen(latitude=51.5, longitude=4.5)        # ~40 nm out
    far.positioned = True
    assert injector._works_here(far)
    # Unset, it keeps the old thirty-mile gate.
    assert not _injector()._works_here(far)


def test_a_live_arrival_sent_to_parking_is_handed_back():
    """A live arrival told to taxi to parking with no layout to drive
    it down was dropped -- and the real aeroplane on the feed put a
    fresh strip up a moment later. It goes back to the feed in place."""
    injector = _injector()
    injector._exchange = Mock()
    flight = _flight(observed=False, source="live", phase="taxi_in",
                     on_layout=False)
    injector._taken_over.add(flight.observed_id)
    injector._flown_for[flight.observed_id] = flight
    injector._adopted_live.add(flight.observed_id)
    injector._advance_arrival(flight, 100.0)
    assert flight.observed is True
    assert not flight.done


def test_a_parked_live_arrival_is_handed_back_not_dropped():
    """Parked long enough a managed arrival leaves the roster -- but a
    live one's real aeroplane is parked right there on the feed, so the
    strip goes back to reading it, never a static double and never a
    vanish-and-fresh-strip."""
    injector = _injector()
    injector._rest_of_taxi = Mock()
    injector._exchange = Mock()
    flight = _flight(observed=False, source="live", phase="taxi_in",
                     on_layout=True, step=10, taxi=object(),
                     parked_at=1.0)
    injector._taken_over.add(flight.observed_id)
    injector._flown_for[flight.observed_id] = flight
    injector._adopted_live.add(flight.observed_id)
    injector._advance_arrival(flight, 100.0 + flight_phase_parked_s())
    assert flight.observed is True
    assert not flight.done
    assert flight.static is False


def flight_phase_parked_s() -> float:
    from meridianatc.atc.traffic.x_arrivals import _Arrivals
    return _Arrivals.PARKED_S + 1.0


def test_a_live_departure_with_nowhere_to_go_is_handed_back():
    """A live departure the taxi planner cannot route anywhere was
    dropped on its stand; the feed's real one reappeared as a fresh
    strip. It is the feed's again instead."""
    injector = _injector(ground=Mock(), _plan_departure=Mock(
        return_value=None))
    flight = _flight(observed=False, source="live", phase="taxi",
                     on_layout=True, arriving=False)
    injector._taken_over.add(flight.observed_id)
    injector._flown_for[flight.observed_id] = flight
    injector._adopted_live.add(flight.observed_id)
    injector._advance_departure(flight, 100.0)
    assert flight.observed is True
    assert not flight.done


def test_a_stuck_live_aeroplane_is_handed_back_not_dropped():
    """The watchdog takes away what cannot be made to move -- but a
    live one still exists on the feed, where its strip reads it rather
    than vanishing and coming back new."""
    injector = _injector()
    injector._where_moving = Mock(return_value="taxiway")
    injector._queued = Mock(return_value=False)
    injector._say_stuck = Mock()
    injector._recover = Mock()
    injector._release_runway = Mock()
    flight = _flight(observed=False, source="live", phase="taxi_in",
                     on_layout=True, still_at=None, still_since=0.0,
                     stuck=False, recovered=False,
                     target=SimpleNamespace(clear=lambda: None,
                                            intent="taxi"))
    injector._taken_over.add(flight.observed_id)
    injector._flown_for[flight.observed_id] = flight
    injector._adopted_live.add(flight.observed_id)
    injector.flights = [flight]
    injector._watchdog(100.0)          # marks it standing still
    injector._watchdog(200.0)          # past the limit: a retry
    injector._watchdog(300.0)          # still: removed -- for live, back
    injector._say_stuck.assert_called_with(
        flight, "taxiway", pytest.approx(100.0), "removed")
    assert flight.observed is True
    assert not flight.done


def test_an_extended_live_arrival_is_handed_back_not_dropped():
    """A live arrival the tower extended past the give-up distance was
    dropped mid-downwind while the real aeroplane flew its approach:
    the strip goes back to reading the feed."""
    injector = _injector(
        _threshold=Mock(return_value=(50.9, 4.48, 250.0)),
        _runway_frame=Mock(return_value=(3.0, 0.5)),
        _room_on_final=Mock(return_value=False),
        _rollout_s=Mock(return_value=60.0))
    flight = _flight(observed=False, source="live", phase="final",
                     arriving=True, extending=True, distance_nm=21.0)
    injector._taken_over.add(flight.observed_id)
    injector._flown_for[flight.observed_id] = flight
    injector._adopted_live.add(flight.observed_id)
    injector._watch_downwind(flight, 100.0)
    assert flight.observed is True
    assert not flight.done


def test_a_live_aeroplane_the_sim_cannot_draw_is_handed_back():
    """The model the simulator could not make was dropped -- and the
    real aeroplane put a fresh strip straight back. It reads the feed
    instead."""
    injector = _injector()
    flight = _flight(observed=False, source="live", drawn=False)
    injector._taken_over.add(flight.observed_id)
    injector._flown_for[flight.observed_id] = flight
    injector._adopted_live.add(flight.observed_id)
    injector._materialising = [flight]
    injector._materialise_by[flight.written] = 200.0
    confirmed, dropped = injector.confirm(
        lambda cs: False, lambda cs: True, 100.0)
    assert flight.observed is True
    assert flight in injector.flights
    assert dropped == []
    assert injector._materialising == []


def test_a_departure_adopted_already_flying_keeps_climbing():
    """UAL145 was adopted off the feed at altitude and drawn back down
    to five thousand over the field -- the invented departure profile.
    A live one keeps climbing the way the real one is, until it is out
    of sight and the feed's copy of it takes over again."""
    injector = _injector()
    flight = _flight(observed=False, source="live", phase="departed",
                     arriving=False, live_airborne=True,
                     altitude_ft=8000.0, heading=120.0, airborne_model=True,
                     hold=None, procedure_pair=None, procedure_nav=None,
                     performance=SimpleNamespace(cruise_kt=430.0),
                     target=SimpleNamespace())
    injector._aim(flight)
    assert flight.target.altitude_ft > flight.altitude_ft
    assert flight.target.altitude_ft > (
        injector.field.elevation_ft + injector.OUT_OF_SIGHT_FT)
    # And on its own course, not the departure runway's heading it
    # never took off from.
    assert flight.target.heading == pytest.approx(120.0)


def test_a_live_departure_off_the_strip_flies_the_runway_out():
    """Adopted on the centreline it did take off here: the runway's
    course is its own. It still climbs out of sight rather than
    levelling at the invented five thousand."""
    injector = _injector()
    flight = _flight(observed=False, source="live", phase="departed",
                     arriving=False, live_airborne=False, heading=247.0,
                     airborne_model=True, hold=None, procedure_pair=None,
                     procedure_nav=None,
                     performance=SimpleNamespace(cruise_kt=430.0),
                     target=SimpleNamespace())
    injector._aim(flight)
    assert flight.target.heading == pytest.approx(247.0)
    assert flight.target.altitude_ft > (
        injector.field.elevation_ft + injector.OUT_OF_SIGHT_FT)


def test_an_invented_departure_still_levels_at_five_thousand():
    injector = _injector()
    flight = _flight(observed=False, source="", phase="departed",
                     arriving=False, heading=247.0, airborne_model=True,
                     hold=None, procedure_pair=None, procedure_nav=None,
                     performance=SimpleNamespace(cruise_kt=430.0),
                     target=SimpleNamespace())
    injector._aim(flight)
    assert flight.target.altitude_ft == pytest.approx(5184.0)


def test_a_departure_adopted_already_past_keeping_is_the_feeds():
    """ANE1114 was adopted above the out-of-sight height and handed
    straight back seven seconds later -- flown by nobody in between.
    One already past where a worked aeroplane is kept stays the
    feed's."""
    injector = _injector()
    flight = _flight(origin="", destination="", arriving=False,
                     altitude_ft=11500.0)
    one = _seen(origin="", destination="", altitude_ft=11500.0,
                vertical_speed_fpm=1500.0)
    assert injector._adopt_live(flight, one, 100.0) is False
    assert flight.observed is True
    assert not injector._taken_over
    # Too far out is the same answer.
    lat, lon = _on_strip(THRESHOLD_25R, along_nm=15.0)
    flight = _flight(origin="", destination="", arriving=False,
                     altitude_ft=5000.0)
    one = _seen(origin="", destination="", altitude_ft=5000.0,
                vertical_speed_fpm=1500.0, latitude=lat, longitude=lon)
    assert injector._adopt_live(flight, one, 100.0) is False
    assert flight.observed is True


def test_a_filed_departure_already_past_keeping_is_not_adopted():
    """The origin match used to answer before the out-of-sight guard:
    a real LECO departure already high was adopted and handed back a
    few seconds later, flown by nobody in between."""
    injector = _injector()
    flight = _flight(origin="EBBR", destination="EGLL", arriving=False,
                     altitude_ft=11500.0)
    one = _seen(origin="EBBR", destination="EGLL", altitude_ft=11500.0,
                vertical_speed_fpm=1500.0)
    assert injector._adopt_live(flight, one, 100.0) is False
    assert flight.observed is True
    assert not injector._taken_over


def test_a_ground_aeroplane_at_the_next_airport_is_not_ours():
    """SWR20D and BEL8HR were taxiing at Porto and each got a Vigo
    strip sixty miles from its runways -- the feed hears that far,
    but a field's own ground is only its own."""
    injector = _injector()
    lat, lon = _on_strip(THRESHOLD_25R, along_nm=6.0)
    far = _seen(on_ground=True, ground_speed_kt=12.0,
                vertical_speed_fpm=0.0, latitude=lat, longitude=lon)
    far.positioned = True
    assert not injector._works_here(far)
    # A strip it already has it keeps; a moving one close in is a strip.
    assert injector._works_here(far, known=True)
    near = _seen(on_ground=True, ground_speed_kt=12.0,
                 vertical_speed_fpm=0.0)
    near.positioned = True
    assert injector._works_here(near)


def test_an_airborne_adoptee_gets_out_of_sight_on_its_own_course():
    """Across the runway's axis the old along-the-strip measure never
    grew -- ours stayed "in sight" at zero miles however far the real
    one went. Its distance is how far it is from the field."""
    injector = _injector()
    lat, lon = _on_strip(THRESHOLD_25R, along_nm=0.0, across_nm=15.0)
    flight = _flight(observed=False, source="live", phase="departed",
                     arriving=False, live_airborne=True,
                     latitude=lat, longitude=lon, runway="25R",
                     altitude_ft=8000.0)
    injector._after_flying(flight)
    assert flight.distance_nm == pytest.approx(15.0, abs=0.5)
