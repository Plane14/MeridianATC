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
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
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


# -- the published missed approach -------------------------------------------

def _ifl_flight():
    return SimpleNamespace(
        ifr=True, runway="17", approach="ILS", latitude=41.25,
        longitude=-8.68, altitude_ft=400.0, heading=170.0,
        ground_speed_kt=120.0, phase="final", arriving=True,
        extending=False, go_arounds=0, step=2,
        assigned_heading=100.0, assigned_altitude_ft=500.0,
        assigned_speed_kt=90.0, next_at=0.0, distance_nm=2.0,
        performance=SimpleNamespace(climb_kt=140.0, pattern_agl_ft=1000.0),
        missed_nav=None)


def _missed_legs():
    from meridianatc.atc.procedure_legs import ProcedureLeg
    return (ProcedureLeg(0, "TF", fix_ident="PR001", lat=41.30, lon=-8.70,
                         alt_desc=2, alt1_ft=2000.0),
            ProcedureLeg(1, "TF", fix_ident="PR002", lat=41.45, lon=-8.75))


def test_an_ifr_go_around_builds_the_published_missed():
    """A missed coded in the approach data is what an IFR go-around flies."""
    from meridianatc.atc.approaches import Approach, FieldApproaches
    from meridianatc.atc.flying import Command
    injector = _injector()
    injector.field = SimpleNamespace(ident="LPPR", elevation_ft=100.0)
    injector.random = random.Random(0)
    known = FieldApproaches("LPPR")
    known.add_procedure(Approach("ILS", "17", missed_ft=2000.0,
                                 missed_legs=_missed_legs()))
    injector.approach_data = {"LPPR": known}
    flight = _ifl_flight()
    injector._carry_out(flight, Command(kind="go_around"), 100.0)
    assert flight.phase == "go_around"
    assert flight.missed_nav is not None
    assert flight.missed_nav.points


def test_an_ifr_go_around_without_missed_data_keeps_the_circuit():
    """No legs after the MAP is the old answer: the circuit."""
    from meridianatc.atc.approaches import Approach, FieldApproaches
    from meridianatc.atc.flying import Command
    injector = _injector()
    injector.field = SimpleNamespace(ident="LPPR", elevation_ft=100.0)
    injector.random = random.Random(0)
    known = FieldApproaches("LPPR")
    known.add_procedure(Approach("ILS", "17", missed_ft=2000.0))
    injector.approach_data = {"LPPR": known}
    flight = _ifl_flight()
    injector._carry_out(flight, Command(kind="go_around"), 100.0)
    assert flight.missed_nav is None


def test_a_vfr_go_around_keeps_the_circuit():
    """VFR has no published missed to fly."""
    from meridianatc.atc.flying import Command
    injector = _injector()
    injector.field = SimpleNamespace(ident="LPPR", elevation_ft=100.0)
    injector.random = random.Random(0)
    injector.approach_data = {}
    flight = _ifl_flight()
    flight.ifr = False
    injector._carry_out(flight, Command(kind="go_around"), 100.0)
    assert flight.missed_nav is None


def test_a_missed_that_is_flown_comes_back_for_another_approach():
    """The missed's end is the arrival stream, not the circuit."""
    injector = _injector()
    injector.field = SimpleNamespace(elevation_ft=0.0)
    flight = _going_around(5.0, 500.0)
    flight.missed_nav = SimpleNamespace(
        guidance=lambda f: SimpleNamespace(done=True))
    injector._after_the_go_around(flight, 100.0)
    assert flight.phase == "inbound"
    assert flight.arriving is True
    assert flight.missed_nav is None


def test_a_missed_still_being_flown_is_left_alone():
    injector = _injector()
    injector.field = SimpleNamespace(elevation_ft=0.0)
    flight = _going_around(5.0, 500.0)
    flight.missed_nav = SimpleNamespace(
        guidance=lambda f: SimpleNamespace(done=False))
    injector._after_the_go_around(flight, 100.0)
    assert flight.phase == "go_around"
    assert flight.next_at == 103.0


def test_an_ifr_go_around_without_legs_climbs_to_the_missed_altitude():
    """No coded legs still has the plate's missed altitude, not a circuit."""
    from meridianatc.atc.approaches import Approach, FieldApproaches
    from meridianatc.atc.flying import Command
    injector = _injector()
    injector.field = SimpleNamespace(ident="LPPR", elevation_ft=100.0)
    injector.random = random.Random(0)
    known = FieldApproaches("LPPR")
    known.add_procedure(Approach("ILS", "17", missed_ft=2000.0))
    injector.approach_data = {"LPPR": known}
    flight = _ifl_flight()
    injector._carry_out(flight, Command(kind="go_around"), 100.0)
    assert flight.missed_nav is None
    assert flight.missed_alt_ft == pytest.approx(2000.0)


def test_an_ifr_go_around_rejoins_inbound_at_the_missed_altitude():
    """Climbs to the published level, then back for another approach."""
    injector = _injector()
    injector.field = SimpleNamespace(elevation_ft=100.0)
    flight = _going_around(5.0, 800.0)
    flight.ifr = True
    flight.missed_alt_ft = 2100.0
    flight.missed_nav = None
    injector._after_the_go_around(flight, 100.0)
    assert flight.phase == "go_around"
    assert flight.next_at == 103.0
    flight.altitude_ft = 2100.0
    injector._after_the_go_around(flight, 104.0)
    assert flight.phase == "inbound"
    assert flight.arriving is True


def test_a_vfr_go_around_still_flies_the_circuit_upwind():
    """The missed-altitude path is the IFR one, not the VFR one's."""
    injector = _injector()
    injector.field = SimpleNamespace(elevation_ft=0.0)
    flight = _going_around(0.3, 900.0)
    flight.ifr = False
    injector._after_the_go_around(flight, 100.0)
    assert flight.phase == "upwind"


def test_the_missed_altitude_is_the_flown_approachs():
    """Two approaches on a runway: the missed level is its own plate's."""
    from meridianatc.atc.approaches import Approach, FieldApproaches
    from meridianatc.atc.flying import Command
    injector = _injector()
    injector.field = SimpleNamespace(ident="LPPR", elevation_ft=0.0)
    injector.random = random.Random(0)
    known = FieldApproaches("LPPR")
    known.add_procedure(Approach("ILS", "17", missed_ft=2000.0))
    known.add_procedure(Approach("NDB", "17", missed_ft=3200.0))
    injector.approach_data = {"LPPR": known}
    flight = _ifl_flight()
    flight.approach = "NDB"
    injector._carry_out(flight, Command(kind="go_around"), 100.0)
    assert flight.missed_alt_ft == pytest.approx(3200.0)


def test_an_upfield_intercept_points_back_down_the_final():
    """Past the runway's upwind end the aim is the extended centreline
    out on final, not thirty degrees more of flying away."""
    injector = _injector()
    injector.field = SimpleNamespace(elevation_ft=0.0)
    injector._threshold = lambda runway: (41.0, -8.0, 360.0)
    flight = SimpleNamespace(latitude=41.03, longitude=-8.0,
                             heading=0.0, runway="36")
    want = injector._intercept(flight, 360.0)
    # ~south, back at the extended final beyond the field.
    assert 150.0 < want < 210.0


def test_an_inbound_still_upfield_holds_the_missed_altitude():
    """Coming back round is not the glide: it holds the missed's level."""
    injector = _injector()
    injector.field = SimpleNamespace(elevation_ft=0.0)
    injector._threshold = lambda runway: (41.0, -8.0, 360.0)
    injector._threshold_ft = lambda runway: 0.0
    injector._surface_ft = lambda flight: 0.0
    flight = SimpleNamespace(
        phase="inbound", latitude=41.03, longitude=-8.0, heading=0.0,
        runway="36", altitude_ft=2100.0, distance_nm=0.0,
        missed_alt_ft=2100.0, on_ground=False, ground_speed_kt=140.0,
        performance=SimpleNamespace(
            approach_kt=90.0, climb_kt=100.0, cruise_kt=140.0,
            pattern_agl_ft=1000.0, rotate_kt=80.0),
        target=SimpleNamespace(), hold=None, procedure_pair=None,
        assigned_heading=None,
        assigned_altitude_ft=None, assigned_speed_kt=None,
        expedite_until=0.0)
    injector._aim(flight)
    assert flight.target.altitude_ft == pytest.approx(2100.0)


# -- level-off and top of descent --------------------------------------------

def test_the_cruise_level_off_releases_flight_level_change():
    """FLCH left on at altitude capture is the idle-thrust stall."""
    flight = _flight()
    flight._dir = Mock()
    flight.sim = flight._dir.sim
    flight._speed_target = Mock(return_value=170.0)
    flight._level_off_at_cruise(SimpleNamespace(autothrottle=False))
    flight._dir.level_change.assert_called_with(False)
    flight._dir.alt_hold.assert_called_with(True)


def test_the_top_of_descent_leaves_room_to_slow_down():
    """The 3-degree path plus a mile per ten knots to lose, not less."""
    flight = _flight()
    flight.session = SimpleNamespace(
        destination=SimpleNamespace(lat=43.3, lon=-8.4))
    flight._perf = SimpleNamespace(approach_kt=90.0)
    flight._cruise_speed_kt = Mock(return_value=200.0)
    flight._field_elevation = Mock(return_value=0.0)
    state = SimpleNamespace(
        altitude_ft=9000.0, has_position=True,
        distance_to=lambda lat, lon: 40.0)
    flight._plan_tod(state)
    # 9000 ft at 318 ft/nm is 28.3 nm; a 200 kt jet needs ~3 more to
    # slow to its descent figure.
    assert flight.state.tod_nm == pytest.approx(28.3 + 3.0, abs=0.6)


# -- crosswind aileron on the roll -------------------------------------------

def _roll_state(xwind_kt: float):
    return SimpleNamespace(aircraft_wind_x_kt=xwind_kt)


def test_the_takeoff_roll_holds_aileron_into_the_wind_and_relaxes():
    flight = _flight()
    flight._perf = SimpleNamespace(rotate_kt=110.0)
    state = _roll_state(15.0)
    start = flight._roll_aileron(state, 0.0, takeoff=True)
    near_vr = flight._roll_aileron(state, 105.0, takeoff=True)
    assert start == pytest.approx(0.3)
    assert abs(near_vr) < abs(start) * 0.3


def test_the_landing_roll_grows_the_into_wind_aileron_as_it_slows():
    flight = _flight()
    flight._perf = SimpleNamespace(rotate_kt=110.0)
    state = _roll_state(15.0)
    fast = flight._roll_aileron(state, 100.0, takeoff=False)
    slow = flight._roll_aileron(state, 25.0, takeoff=False)
    assert abs(fast) == pytest.approx(0.3 * 0.15)
    assert slow == pytest.approx(0.3)


def test_a_nav_stuck_on_a_stale_course_is_flown_to_the_fix():
    """Persistently nowhere near the leg's fix, the FO steers itself."""
    from meridianatc.atc.route import Progress, RouteFix
    flight = _flight()
    flight.brain = SimpleNamespace(
        route=SimpleNamespace(
            progress=lambda *a, **k: Progress(
                leg=1,
                from_fix=RouteFix(ident="A", lat=41.0, lon=-8.0),
                to_fix=RouteFix(ident="B", lat=42.0, lon=-8.0),
                cross_track_nm=2.0, along_leg_nm=0.0, to_go_nm=60.0,
                to_next_nm=60.0, eta_s=())))
    flight.session = SimpleNamespace(
        route_direct_index=-1, destination=None)
    state = SimpleNamespace(
        has_position=True, latitude=41.2, longitude=-8.1,
        ground_speed_kt=150.0, track_true=195.0, heading_true=195.0,
        bearing_to=lambda lat, lon: 30.0)
    now = 1000.0
    assert flight._nav_fallback_heading(state, now) == -1.0
    assert flight._nav_fallback_heading(state, now + 50.0) \
        == pytest.approx(30.0)


def test_a_navless_route_falls_back_to_the_destination_bearing():
    """No route to place: badly off the destination is still corrected."""
    flight = _flight()
    flight.brain = SimpleNamespace(route=None)
    flight.session = SimpleNamespace(
        route_direct_index=-1,
        destination=SimpleNamespace(lat=43.3, lon=-8.4))
    state = SimpleNamespace(
        has_position=True, latitude=40.7, longitude=-7.9,
        ground_speed_kt=150.0, track_true=199.0, heading_true=199.0,
        bearing_to=lambda lat, lon: 340.0)
    now = 2000.0
    assert flight._nav_fallback_heading(state, now) == -1.0
    assert flight._nav_fallback_heading(state, now + 50.0) \
        == pytest.approx(340.0)


# -- expedite -----------------------------------------------------------------

def test_an_expedite_command_hurries_what_the_flight_does_next():
    from meridianatc.atc.flying import Command
    injector = _injector()
    flight = SimpleNamespace(expedite_until=0.0, next_at=999.0)
    injector._carry_out(flight, Command(kind="expedite"), 100.0)
    assert flight.expedite_until == 190.0
    assert flight.next_at == 100.0


def _landed_on_the_runway():
    from meridianatc.atc.session import Phase
    return SimpleNamespace(
        emergency=False, cleared_takeoff=False, cleared_takeoff_at=0.0,
        lined_up=False, departure_runway="", arrival_runway="17",
        phase=Phase.LANDED, expedite_until=0.0,
        advise_once=lambda key: True,
        aircraft=SimpleNamespace(language=""))


def _rollout_state():
    return SimpleNamespace(ground_speed_kt=20.0, on_ground=True,
                           has_position=True, latitude=41.25,
                           longitude=-8.68, altitude_ft=100.0,
                           heading_true=170.0)


def test_an_arrival_rolling_out_in_front_of_final_is_told_to_expedite():
    """A rollout at taxi speed with somebody on final gets the same
    "expedite" the traffic side's aeroplanes get -- and a clock the
    taxi can be brisk for."""
    from meridianatc.atc.departure import DepartureWatch
    p = Mock()
    p.language = "en"
    p.expedite = Mock(return_value="EXPEDITE")
    p.traffic_on_final = Mock(return_value="TRAFFIC")
    coming = SimpleNamespace(
        distance_nm=2.0, written="DAL123",
        indefinite=lambda lang: "a Boeing",
        describe=lambda lang: "DAL123")
    brain = Mock()
    brain.arrivals_on_final = Mock(return_value=[coming])
    brain.for_station = Mock(return_value=p)
    watch = DepartureWatch.__new__(DepartureWatch)
    watch.navdb = SimpleNamespace(
        airport=lambda ident: SimpleNamespace(ident="LPPR"))
    watch.brain = brain
    session = _landed_on_the_runway()
    station = SimpleNamespace(ident="LPPR")
    with patch("meridianatc.atc.occupancy.on_the_runway",
               return_value=[object()]):
        reply = watch._expedite_the_rollout(
            session, station, _rollout_state(), 100.0, "en")
    assert reply is not None
    assert session.expedite_until \
        == 100.0 + DepartureWatch.EXPEDITE_WINDOW_S
    p.expedite.assert_called_once()


def test_the_rollout_expedite_is_not_spent_on_an_empty_final():
    """Nobody behind, nobody told: the word is for the arrival that
    cannot come in while the runway is occupied."""
    from meridianatc.atc.departure import DepartureWatch
    brain = Mock()
    brain.arrivals_on_final = Mock(return_value=[])
    watch = DepartureWatch.__new__(DepartureWatch)
    watch.navdb = SimpleNamespace(
        airport=lambda ident: SimpleNamespace(ident="LPPR"))
    watch.brain = brain
    session = _landed_on_the_runway()
    station = SimpleNamespace(ident="LPPR")
    with patch("meridianatc.atc.occupancy.on_the_runway",
               return_value=[object()]):
        assert watch._expedite_the_rollout(
            session, station, _rollout_state(), 100.0, "en") is None
    assert session.expedite_until == 0.0


def test_an_expedite_on_the_runway_taxis_the_aeroplane_briskly():
    """The word buys a brisk taxi for as long as it stands, and nothing
    once the window has closed."""
    from meridianatc.atc.autonomous.x_taxi import _Taxi
    fo = _Taxi.__new__(_Taxi)
    fo.session = SimpleNamespace(
        expedite_until=time.monotonic() + 90.0,
        departure=SimpleNamespace(ident="LPPR"), departure_runway="17",
        destination=None, arrival_runway="")
    fo.navdb = SimpleNamespace()
    state = SimpleNamespace(has_position=False)
    assert fo._expedite_kt(state, 14.0) > 14.0
    fo.session.expedite_until = time.monotonic() - 1.0
    assert fo._expedite_kt(state, 14.0) == 14.0


def test_strobes_follow_the_clearance_and_the_crossing():
    """A takeoff, line-up or crossing clearance lights the strobes before
    the pavement is touched; vacating the strip puts the crossing's
    light out, while a light the departure owns stays for the roll."""
    from meridianatc.atc.autonomous.x_taxi import _Taxi
    from meridianatc.atc.session import Phase
    fo = _Taxi.__new__(_Taxi)
    calls = []
    fo.director = SimpleNamespace(
        strobes=lambda on: calls.append(on))
    session = SimpleNamespace(
        crossing_granted="", occupying_runway="",
        lined_up=False, cleared_takeoff=False, phase=Phase.TAXI_OUT)
    fo.session = session
    state = SimpleNamespace(strobe_light=False)

    # Nothing in hand: dark.
    fo._clearance_strobes(state)
    assert calls == []

    # Cleared to take off with the nose still at the bars: lit at the
    # word, not at the pavement edge.
    session.cleared_takeoff = True
    fo._clearance_strobes(state)
    assert calls == [True]
    state.strobe_light = True

    # A granted runway crossing lights them the same way -- and owns
    # their going out.
    session.cleared_takeoff = False
    session.crossing_granted = "17"
    session.occupying_runway = "17"
    state.strobe_light = False
    fo._clearance_strobes(state)
    assert calls[-1] is True

    # Off the far side, the grant spent: out they go.
    session.crossing_granted = ""
    session.occupying_runway = ""
    state.strobe_light = True
    fo._clearance_strobes(state)
    assert calls[-1] is False

    # On the pavement without a word for it, they burn anyway -- the
    # strip itself is what they warn of -- and out once vacated.
    session.occupying_runway = "09"
    state.strobe_light = False
    fo._clearance_strobes(state)
    assert calls[-1] is True
    session.occupying_runway = ""
    state.strobe_light = True
    fo._clearance_strobes(state)
    assert calls[-1] is False

    # A light that was already burning for its own reason (the
    # checklist's) is not switched off mid-taxi -- but the crossing
    # adopts the off it is owed.
    session.crossing_granted = "17"
    writes = len(calls)
    fo._clearance_strobes(state)
    assert len(calls) == writes  # no write: already lit
    session.crossing_granted = ""
    fo._clearance_strobes(state)
    assert calls[-1] is False


def test_the_fms_loaded_sid_is_the_one_cleared():
    """A SID typed into the sim's FMS -- never read by a plan MeridianATC
    loaded -- is the departure the clearance names, where a silent plan
    would have been cleared "radar vectors"."""
    from meridianatc.atc.controller import ControllerBrain
    from meridianatc.atc.phraseology import Phraseology
    from meridianatc.atc.procedure_legs import (
        AirportProcedures,
        ProcedureLeg,
        ProcedureTransition,
        TerminalProcedure,
    )
    brain = ControllerBrain(Mock(), Phraseology())
    legs = tuple(ProcedureLeg(index=i, path="TF", fix_ident=f)
                 for i, f in enumerate(("KUBIR", "ROUSY")))
    procs = AirportProcedures(
        ident="KAAA",
        departures={
            "ROUS1L": TerminalProcedure(
                name="ROUS1L", kind="sid",
                transitions=[ProcedureTransition(runway="36", legs=legs)]),
            "OTHER1": TerminalProcedure(
                name="OTHER1", kind="sid",
                transitions=[ProcedureTransition(
                    runway="36",
                    legs=(ProcedureLeg(index=0, path="TF",
                                       fix_ident="NOSIG"),))]),
        })
    brain.terminal_procedures["KAAA"] = procs
    airport = SimpleNamespace(ident="KAAA")

    # The loaded legs walk the transition in order: it is ROUS1L.
    brain.fms_idents = ("KAAA", "KUBIR", "ROUSY", "BOS")
    assert brain._fms_sid(airport, "36") == "ROUS1L"

    # A transition whose whole procedure is the one fix the box shows
    # still names it -- its only leg is the match.
    brain.fms_idents = ("KAAA", "NOSIG", "BOS")
    assert brain._fms_sid(airport, "36") == "OTHER1"

    # Legs in the wrong order walk nothing.
    brain.fms_idents = ("KAAA", "ROUSY", "KUBIR", "BOS")
    assert brain._fms_sid(airport, "36") == ""

    # Nothing loaded, nothing invented.
    brain.fms_idents = ()
    assert brain._fms_sid(airport, "36") == ""


def _cruising_fo(distance_nm: float):
    """A cruise-phase FO on a quiet frequency, ``distance_nm`` out from
    a sea-level destination at ten thousand feet."""
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )
    from meridianatc.atc.session import Phase
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.state.checklists_completed = {"cruise"}
    fo.state.phase = AutonomousPhase.CRUISE
    fo._dir = Mock()
    fo.sim = fo._dir.sim  # the property remakes _dir if sim differs
    fo.get_state = lambda: None  # performance falls back to the class table
    fo.on_event = lambda *a, **kw: None
    fo.brain = SimpleNamespace(
        route=None, terminal_procedures={}, route_progress=None,
        fms_idents=())
    fo.session = SimpleNamespace(
        phase=Phase.ENROUTE, assigned_altitude_ft=0.0,
        assigned_heading=None, assigned_speed_kt=None,
        departure=None, departure_runway="", arrival_runway="",
        sid="", descend_via="", cruise_altitude_ft=10000.0,
        destination=SimpleNamespace(lat=41.0, lon=-8.0, elev_ft=0.0))
    state = SimpleNamespace(
        has_position=True, latitude=40.9, longitude=-8.0,
        altitude_ft=10000.0, altitude_agl_ft=9900.0,
        ground_speed_kt=200.0, heading_true=180.0, autopilot=True,
        vertical_speed_fpm=0.0, estimated_cruise_kt=0.0,
        barber_pole_kt=0.0, climb_kt=0.0,
        on_ground=False, engines_running=True,
        distance_to=lambda lat, lon: distance_nm)
    return fo, state


def test_the_fo_begins_its_own_descent_at_the_top_of_descent():
    """On a frequency that never says "descend", the aeroplane still gets
    down: the cruise's own three-degree point is where the phase is
    entered, not somewhere over the top of the field."""
    from meridianatc.atc.autonomous import AutonomousPhase
    from meridianatc.atc.session import Phase
    # 10 000 ft AGL is about 31 nm of path: inside it, down it goes.
    fo, state = _cruising_fo(20.0)
    fo._phase_cruise(state, 100.0)
    assert fo.state.phase == AutonomousPhase.DESCENT

    # Well outside it, the cruise is kept.
    fo, state = _cruising_fo(80.0)
    fo._phase_cruise(state, 100.0)
    assert fo.state.phase == AutonomousPhase.CRUISE

    # A controller's word still comes first: phase DESCENT on the
    # session is honoured before the FO's own arithmetic is even read.
    fo, state = _cruising_fo(80.0)
    fo.session.phase = Phase.DESCENT
    fo._phase_cruise(state, 100.0)
    assert fo.state.phase == AutonomousPhase.DESCENT


def _descending_fo(altitude_ft: float):
    """A descent-phase FO already at its last assigned level: vectored
    for the approach at nine thousand, with no lower clearance yet."""
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )
    from meridianatc.atc.session import Phase
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.state.phase = AutonomousPhase.DESCENT
    fo.state.tod_nm = 40.0
    fo.state.phase_entered_at = 100.0
    fo.DESAT_TIMEOUT = 600.0
    fo._dir = Mock()
    fo.sim = fo._dir.sim
    fo.get_state = lambda: None
    fo.on_event = lambda *a, **kw: None
    fo.brain = SimpleNamespace(
        route=None, terminal_procedures={}, route_progress=None,
        fms_idents=())
    fo.session = SimpleNamespace(
        phase=Phase.DESCENT, assigned_altitude_ft=9000.0,
        assigned_heading=32.0, assigned_speed_kt=None,
        descend_via="", destination=SimpleNamespace(
            lat=41.0, lon=-8.0, elev_ft=0.0))
    fo._run_checklist_for_phase = Mock()
    fo._descent_floor = Mock(return_value=1800.0)
    fo._route_level = Mock(return_value=(0.0, ""))
    fo._distance_to_destination_nm = Mock(return_value=40.0)
    fo._field_elevation = Mock(return_value=0.0)
    fo._star_nav = Mock(return_value=None)
    fo._approach_recipe = Mock(return_value=None)
    fo._maybe_tune_nav = Mock()
    fo._transition = Mock()
    fo._set_error = Mock()
    fo._apply_flight_director = Mock()
    state = SimpleNamespace(
        has_position=True, latitude=40.9, longitude=-8.0,
        altitude_ft=altitude_ft, altitude_agl_ft=altitude_ft,
        ground_speed_kt=180.0, heading_true=32.0, autopilot=True,
        vertical_speed_fpm=0.0, typical_descent_fpm=800.0,
        gear_down=None, flaps_index=0.0,
        on_ground=False, engines_running=True)
    return fo, state


def test_a_level_the_flight_is_already_at_is_held_not_hunted():
    """Assigned nine thousand and flying at it: the descent phase holds
    the level rather than dipping a hundred feet under and climbing back
    at descent speed -- the porpoise that read as "climbing close to
    stall"."""
    fo, state = _descending_fo(9000.0)
    fo._phase_descent(state, 100.0)
    assert fo._apply_flight_director.call_args.kwargs["vs_fpm"] == 0.0
    assert fo._apply_flight_director.call_args.kwargs["altitude"] == 9000.0


def test_the_descent_still_descends_past_the_band_and_climbs_on_a_reclear():
    fo, state = _descending_fo(9500.0)
    fo._phase_descent(state, 100.0)
    assert fo._apply_flight_director.call_args.kwargs["vs_fpm"] < 0.0

    fo, state = _descending_fo(8600.0)
    fo._phase_descent(state, 100.0)
    assert fo._apply_flight_director.call_args.kwargs["vs_fpm"] > 0.0


def test_the_capture_band_converges_on_the_bug_not_its_edge():
    """Nine thousand two hundred on a nine-thousand bug: still coming
    down, gently. A hard stop at the band's edge parks the aeroplane
    two hundred feet over its level -- the hover that keeps the
    verify-altitude calls coming."""
    fo, state = _descending_fo(9200.0)
    fo._phase_descent(state, 100.0)
    vs = fo._apply_flight_director.call_args.kwargs["vs_fpm"]
    assert -300.0 <= vs < 0.0

    fo, state = _descending_fo(9050.0)
    fo._phase_descent(state, 100.0)
    vs = fo._apply_flight_director.call_args.kwargs["vs_fpm"]
    assert -300.0 <= vs < 0.0


def _arrival_watch(monkeypatch, altitude_ft: float):
    """An approach position vectoring an arrival onto a seven-mile final
    for runway 21 -- the flight sits on the extended centreline at
    ``altitude_ft``, already on a base leg in the pattern."""
    import meridianatc.atc.arrival as arrival
    from meridianatc.atc.final import Final
    from meridianatc.atc.phraseology import Aircraft, Phraseology
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.navdata.build import project
    from meridianatc.navdata.db import Station

    final = Final("21", 42.0, -8.0, 210.0)
    monkeypatch.setattr(arrival, "final_for", lambda *a, **k: final)
    watch = arrival.ArrivalWatch.__new__(arrival.ArrivalWatch)
    watch.navdb = SimpleNamespace()
    watch.brain = SimpleNamespace(
        realism=SimpleNamespace(vectors=True, speed_control=False),
        controller_at=lambda ident, position: False,
        approach_choice=lambda *a, **k: SimpleNamespace(
            kind="ILS", approach=SimpleNamespace(runway="21",
                                                 name="ILS Z 21"),
            runway="21"))
    watch._runway = Mock(return_value="21")
    watch._weather = Mock(return_value=None)
    watch._on_a_star_to_the_final = Mock(return_value=None)
    watch._still_on_star = Mock(return_value=False)
    watch._intercept = Mock(return_value="cleared")

    outbound = (final.course + 180.0) % 360.0
    lat, lon = project(final.lat, final.lon, outbound, 7.0)
    state = SimpleNamespace(
        latitude=lat, longitude=lon, altitude_ft=altitude_ft,
        ground_speed_kt=130.0, heading_true=210.0,
        has_position=True)
    session = FlightSession(Aircraft(parse_callsign("N45XS")))
    session.ifr = True
    session.vector_leg, session.vector_side, session.vector_at = "base", 1, 0.0
    station = Station("LECO", "APP", 120.2, "Approach")
    destination = SimpleNamespace(ident="LECO", lat=42.0, lon=-8.0,
                                  elev_ft=326.0, spoken="Santiago")
    reply = watch._radar_vectors(session, station, state, destination,
                                 20.0, None, Phraseology())
    return watch, session, reply


def test_a_flight_too_high_for_the_gate_is_sent_back_out(monkeypatch):
    """Arriving at the intercept still high is not an approach clearance:
    the controller sends it round again to make the room -- the real
    "unable, maintain heading, expect a re-sequence"."""
    watch, session, reply = _arrival_watch(monkeypatch, 9000.0)
    watch._intercept.assert_not_called()
    assert session.vector_leg == "downwind"
    assert reply is not None


def test_a_flight_low_enough_for_the_gate_is_cleared(monkeypatch):
    watch, session, reply = _arrival_watch(monkeypatch, 2300.0)
    watch._intercept.assert_called_once()


def test_the_assigned_speed_is_flown_to_the_final_segment():
    """"Maintain one-seventy to the marker" is obeyed while the final is
    still far; inside it, the flare's own speed is the one flown."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.get_state = lambda: None
    fo._weather_reactions = False  # Vapp is the book figure, no gust additive
    fo.session = SimpleNamespace(
        assigned_speed_kt=170.0,
        destination=SimpleNamespace(lat=41.0, lon=-8.0, elev_ft=0.0))
    fo.state.vapp_kt = 120.0

    def far_state(agl, nm):
        return SimpleNamespace(
            altitude_agl_ft=agl, has_position=True,
            latitude=40.9, longitude=-8.0,
            distance_to=lambda lat, lon: nm)

    # Eight miles out: the assignment is the speed.
    assert fo._approach_speed(far_state(3000.0, 8.0)) == 170.0
    # Inside four miles the flare's speed takes over.
    assert fo._approach_speed(far_state(3000.0, 3.0)) == 120.0
    # Low is close, wherever the map puts it.
    assert fo._approach_speed(far_state(1200.0, 8.0)) == 120.0
    # Nothing assigned: the checklist's figure.
    fo.session.assigned_speed_kt = None
    assert fo._approach_speed(far_state(3000.0, 8.0)) == 120.0


def _speed_watch(type_code: str, ahead=True, ias_kt: float = 250.0):
    """An approach position with speed control on, vectoring an arrival
    with somebody (or nobody) to follow."""
    import meridianatc.atc.arrival as arrival
    from meridianatc.atc.phraseology import Aircraft, Phraseology
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.navdata.db import Station

    contact = SimpleNamespace(
        on_ground=False, latitude=42.0, longitude=-8.05,
        altitude_ft=2000.0, written="VLG2FJ") if ahead else None
    watch = arrival.ArrivalWatch.__new__(arrival.ArrivalWatch)
    watch.brain = SimpleNamespace(
        realism=SimpleNamespace(speed_control=True),
        contacts=lambda: [contact] if contact else [])
    session = FlightSession(Aircraft(parse_callsign("N45XS"),
                                     type_code=type_code))
    session.vector_leg, session.vector_at = "base", 0.0
    station = Station("LECO", "APP", 120.2, "Approach")
    destination = SimpleNamespace(ident="LECO", lat=42.0, lon=-8.0,
                                  elev_ft=326.0, spoken="Santiago")
    state = SimpleNamespace(
        latitude=42.0, longitude=-8.1, altitude_ft=4000.0,
        indicated_airspeed_kt=ias_kt, type_code=type_code,
        has_position=True)
    return watch, session, station, state, destination, Phraseology()


def test_a_speed_restriction_the_type_cannot_hold_is_never_given():
    """"Reduce speed to two one zero" to a Baron is an instruction it
    cannot fly: the figure is capped at what the type holds, so the
    speed watch does not nag about a deviation the aeroplane cannot fix."""
    watch, session, station, state, destination, p = _speed_watch("BE58")
    reply = watch._speed_control(session, station, state, destination,
                                 20.0, p)
    assert reply is not None
    assert 0 < reply.instruction.values["speed_kt"] < 210


def test_a_speed_restriction_stands_when_the_type_can_hold_it():
    """Past twenty miles the jet gets the full 210; inside them the
    7110.65 figure is 170."""
    watch, session, station, state, destination, p = _speed_watch("B738")
    reply = watch._speed_control(session, station, state, destination,
                                 25.0, p)
    assert reply is not None
    assert reply.instruction.values["speed_kt"] == 210
    from meridianatc.atc.session import FlightSession
    session2 = FlightSession(session.aircraft)
    session2.vector_leg, session2.vector_at = "base", 0.0
    reply = watch._speed_control(session2, station, state, destination,
                                 20.0, p)
    assert reply is not None
    assert reply.instruction.values["speed_kt"] == 170


def test_a_speed_reduction_is_not_asked_of_an_aeroplane_already_slow():
    """A Cessna doing a hundred knots is not told to reduce to 150 --
    the instruction would only speed it up."""
    watch, session, station, state, destination, p = _speed_watch(
        "C172", ias_kt=110.0)
    reply = watch._speed_control(session, station, state, destination,
                                 15.0, p)
    assert reply is None


def test_no_speed_restrictions_clears_the_assigned_speed():
    """The lifted restriction has to come off the session too: left
    assigned, the aeroplane is still being flown and flagged against a
    figure nobody told it any more."""
    watch, session, station, state, destination, p = _speed_watch(
        "BE58", ahead=False)
    session.speed_given = "pattern"
    session.assigned_speed_kt = 140.0
    reply = watch._speed_control(session, station, state, destination,
                                 20.0, p)
    assert reply is not None and session.speed_lifted
    assert session.assigned_speed_kt is None


def test_the_closest_approach_arithmetic():
    """The conflict probe's straight-line look ahead: head-on merges,
    in-trail does not, an opening pair is already past."""
    from meridianatc.atc.trafficinfo import closest_approach
    # Head-on, same height, ~2.5nm out at 200kt each: they meet in ~23s.
    miss, t, vgap = closest_approach(
        40.0, -8.0, 3000.0, 90.0, 200.0,
        40.0, -7.95, 3000.0, 270.0, 200.0)
    assert miss < 0.05
    assert 15.0 < t < 30.0
    assert vgap == 0.0
    # Same heading in trail 3nm ahead, same speed: never merges.
    miss, t, _ = closest_approach(
        40.0, -8.0, 3000.0, 90.0, 200.0,
        40.0, -7.95, 3000.0, 90.0, 200.0)
    assert t == 0.0
    # Head-on but a thousand feet apart and staying there: lateral merge,
    # no vertical conflict.
    miss, t, vgap = closest_approach(
        40.0, -8.0, 3000.0, 90.0, 200.0,
        40.0, -7.95, 4200.0, 270.0, 200.0)
    assert miss < 0.05 and vgap == 1200.0
    # Already behind and opening: the closest point is now.
    miss, t, _ = closest_approach(
        40.0, -8.0, 3000.0, 90.0, 200.0,
        40.0, -8.05, 3000.0, 270.0, 200.0)
    assert t == 0.0


def _enroute_fo(contacts):
    """A cruise FO with a traffic picture, for the see-and-avoid watch."""
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.state.phase = AutonomousPhase.CRUISE
    fo.on_event = lambda *a, **kw: None
    fo.brain = SimpleNamespace(contacts=lambda: contacts)
    fo.session = SimpleNamespace(
        departure=None, destination=None)
    state = SimpleNamespace(
        has_position=True, latitude=40.0, longitude=-8.0,
        altitude_ft=3000.0, track_true=90.0, heading_true=90.0,
        ground_speed_kt=200.0, vertical_speed_fpm=0.0,
        on_ground=False)
    return fo, state


def test_the_fo_turns_away_from_a_meeting_in_the_air():
    """A conflict a minute out gets a see-and-avoid turn; a passed one
    gets the route's heading back."""
    from meridianatc.atc.trafficinfo import Contact
    # Head-on at the same height, meeting in ~45s.
    head_on = Contact(
        written="CS-XXX", type_code="C172", latitude=40.0, longitude=-7.94,
        altitude_ft=3000.0, heading=270.0, ground_speed_kt=200.0)
    fo, state = _enroute_fo([head_on])
    fo._watch_traffic(state, 100.0)
    # Head-on turns right, and the deviation is armed.
    assert fo.state.avoid_key == "CS-XXX"
    assert abs(fo.state.avoid_heading - 120.0) < 0.5
    assert fo.state.avoid_until == 130.0

    # A contact 1200ft up merging laterally is no see-and-avoid case.
    high = Contact(
        written="CS-YYY", type_code="C172", latitude=40.0, longitude=-7.94,
        altitude_ft=4300.0, heading=270.0, ground_speed_kt=200.0)
    fo, state = _enroute_fo([high])
    fo._watch_traffic(state, 100.0)
    assert fo.state.avoid_key == ""

    # Passed traffic clears the deviation back to the route's heading.
    fo, state = _enroute_fo([head_on])
    fo.state.avoid_key = "CS-XXX"
    fo.state.avoid_heading = 120.0
    fo.state.avoid_until = 130.0
    head_on = Contact(
        written="CS-ZZZ", type_code="C172", latitude=40.0, longitude=-8.06,
        altitude_ft=3000.0, heading=270.0, ground_speed_kt=200.0)
    fo.brain.contacts = lambda: [head_on]
    fo._watch_traffic(state, 100.0)
    assert fo.state.avoid_key == ""
    assert fo.state.avoid_heading == -1.0


# -- the rejected takeoff ------------------------------------------------------

def _stopped_departure():
    return SimpleNamespace(
        emergency=False, cleared_takeoff=True, cleared_takeoff_at=50.0,
        lined_up=True, departure_runway="17", arrival_runway="",
        phase=SimpleNamespace(outbound=True),
        advise_once=lambda key: True,
        aircraft=SimpleNamespace(language=""))


def test_a_departure_stopped_on_the_runway_is_worked_off_it():
    """A rejected takeoff used to sit on the strip for the whole session."""
    from meridianatc.atc.departure import DepartureWatch
    brain = Mock()
    brain._issue_taxi_in = Mock(return_value="ROUTE")
    field = SimpleNamespace(ident="LPPR")
    watch = DepartureWatch.__new__(DepartureWatch)
    watch.navdb = SimpleNamespace(airport=lambda ident: field)
    watch.brain = brain
    watch._busy_at = 0.0
    watch._stopped_since = 0.0
    session = _stopped_departure()
    # It rolled under this clearance before it stopped.
    watch._rolled_for = session.cleared_takeoff_at
    state = SimpleNamespace(ground_speed_kt=0.0, on_ground=True,
                            has_position=True, latitude=41.25,
                            longitude=-8.68, altitude_ft=100.0,
                            heading_true=170.0)
    station = SimpleNamespace(ident="LPPR")
    with patch("meridianatc.atc.occupancy.on_the_runway",
               return_value=[object()]):
        # First look starts the clock; the second, past the window,
        # works it off the runway.
        assert watch._vacate_the_stopped(
            session, station, state, 100.0, lambda ident: None, "en") is None
        reply = watch._vacate_the_stopped(
            session, station, state,
            100.0 + DepartureWatch.ABORT_AFTER_S + 1.0,
            lambda ident: None, "en")
    assert reply == "ROUTE"
    assert session.cleared_takeoff is False
    assert session.arrival_runway == "17"
    brain._issue_taxi_in.assert_called_once()


def test_a_stopped_departure_off_the_runway_is_not_taxied_in():
    """Somewhere else is the pilot's call to make, not the tower's."""
    from meridianatc.atc.departure import DepartureWatch
    watch = DepartureWatch.__new__(DepartureWatch)
    watch.navdb = SimpleNamespace(airport=lambda ident: None)
    watch.brain = Mock()
    watch._busy_at = 0.0
    watch._stopped_since = 0.0
    session = _stopped_departure()
    watch._rolled_for = session.cleared_takeoff_at
    state = SimpleNamespace(ground_speed_kt=0.0, on_ground=True,
                            has_position=False)
    station = SimpleNamespace(ident="LPPR")
    assert watch._vacate_the_stopped(
        session, station, state, 200.0, lambda ident: None, "en") is None
    watch.brain._issue_taxi_in.assert_not_called()


def test_a_departure_that_never_rolled_is_not_aborted():
    """Line up and wait is not a rejection: holding on the numbers for a
    departure ahead or a gap is what the clearance is for."""
    from meridianatc.atc.departure import DepartureWatch
    watch = DepartureWatch.__new__(DepartureWatch)
    watch.navdb = SimpleNamespace(airport=lambda ident: None)
    watch.brain = Mock()
    watch._busy_at = 0.0
    watch._stopped_since = 0.0
    watch._rolled_for = 0.0
    session = _stopped_departure()
    state = SimpleNamespace(ground_speed_kt=0.0, on_ground=True,
                            has_position=True, latitude=41.25,
                            longitude=-8.68, altitude_ft=100.0,
                            heading_true=170.0)
    station = SimpleNamespace(ident="LPPR")
    # Stopped far past the abort window but never rolled: no taxi-out.
    assert watch._vacate_the_stopped(
        session, station, state, 200.0, lambda ident: None, "en") is None
    assert session.cleared_takeoff is True
    watch.brain._issue_taxi_in.assert_not_called()


def test_rolling_under_the_clearance_marks_the_abort_watch():
    """The rolling evidence is taken under the clearance it belongs to."""
    from meridianatc.atc.departure import DepartureWatch
    watch = DepartureWatch.__new__(DepartureWatch)
    watch.navdb = SimpleNamespace(airport=lambda ident: None)
    watch.brain = Mock(realism=SimpleNamespace(delays=False))
    watch._busy_at = 0.0
    watch._stopped_since = 0.0
    watch._rolled_for = 0.0
    session = _stopped_departure()
    station = SimpleNamespace(ident="LPPR", position="TWR")
    rolling = SimpleNamespace(ground_speed_kt=60.0, on_ground=True,
                              has_position=False)
    watch.look(session, station, rolling, 60.0)
    assert watch._rolled_for == session.cleared_takeoff_at


# -- stands for the kind of flight ------------------------------------------------

def _gate_and_ramp_layout():
    from meridianatc.atc.ground import PATH_PARKING, GroundLayout
    points = [(0, 0, 0.0, 0.0)]
    parkings = [
        (8, 0, "G1", "", 1, 0, 90.0, 20.0, 50.0, 0.0),    # a gate
        (1, 0, "T1", "", 2, 0, 90.0, 20.0, 100.0, 0.0),   # a ramp big enough too
    ]
    paths = [(PATH_PARKING, 0, 0, "", 0, 0, -1),
             (PATH_PARKING, 0, 0, "", 0, 1, -1)]
    return GroundLayout("TEST", 51.0, 0.0, points, parkings, paths, [])


def test_an_airline_goes_to_the_gate_while_one_is_free():
    layout = _gate_and_ramp_layout()
    stands = layout.free_stands(True, [], set())
    assert [s.index for s in stands] == [0]


def test_a_light_goes_to_the_ramp_while_one_is_free():
    layout = _gate_and_ramp_layout()
    stands = layout.free_stands(False, [], set())
    assert [s.index for s in stands] == [1]


def test_a_light_takes_the_gate_only_when_no_ramp_is_free():
    layout = _gate_and_ramp_layout()
    stands = layout.free_stands(False, [], {1})
    assert [s.index for s in stands] == [0]


def test_an_airliner_uses_the_ramp_when_the_gates_are_full():
    layout = _gate_and_ramp_layout()
    stands = layout.free_stands(True, [], {0})
    assert [s.index for s in stands] == [1]


# -- the missed approach's hold -------------------------------------------------

def test_a_missed_ending_in_a_hold_is_held_not_rejoined():
    from meridianatc.atc.approaches import Approach, FieldApproaches
    from meridianatc.atc.flying import Command
    from meridianatc.atc.procedure_legs import ProcedureLeg
    injector = _injector()
    injector.field = SimpleNamespace(ident="LPPR", elevation_ft=100.0)
    injector.random = random.Random(0)
    known = FieldApproaches("LPPR")
    legs = (ProcedureLeg(0, "TF", fix_ident="PR001", lat=41.30, lon=-8.70),
            ProcedureLeg(1, "HM", fix_ident="PR001", lat=41.30, lon=-8.70))
    known.add_procedure(Approach("ILS", "17", missed_ft=2000.0,
                                 missed_legs=legs))
    injector.approach_data = {"LPPR": known}
    flight = _ifl_flight()
    injector._carry_out(flight, Command(kind="go_around"), 100.0)
    assert flight.missed_hold is True
    assert flight.missed_hold_alt_ft == 2000.0


def test_the_missed_hold_orbits_then_comes_back():
    """A coded hold is orbited a while, then the stream takes it back."""
    injector = _injector()
    injector.field = SimpleNamespace(elevation_ft=0.0)
    injector.random = random.Random(0)
    flight = _going_around(5.0, 500.0)
    flight.missed_nav = SimpleNamespace(
        guidance=lambda f: SimpleNamespace(done=True))
    flight.missed_hold = True
    flight.missed_hold_until = 0.0
    injector._after_the_go_around(flight, 100.0)
    assert flight.phase == "missed_hold"
    assert flight.missed_hold_until > 100.0
    injector._after_the_go_around(flight, flight.missed_hold_until + 1.0)
    assert flight.phase == "inbound"
    assert flight.arriving is True
    assert flight.missed_hold is False
    assert flight.missed_nav is None


def test_a_missed_without_a_hold_rejoins_straight_away():
    """Legs with no hold terminator go back for another approach at once."""
    injector = _injector()
    injector.field = SimpleNamespace(elevation_ft=0.0)
    injector.random = random.Random(0)
    flight = _going_around(5.0, 500.0)
    flight.missed_nav = SimpleNamespace(
        guidance=lambda f: SimpleNamespace(done=True))
    flight.missed_hold = False
    injector._after_the_go_around(flight, 100.0)
    assert flight.phase == "inbound"
    assert flight.missed_nav is None


# -- the departure entry ---------------------------------------------------------

def _runway_layout():
    """A strip with a threshold-side entry and a midfield one, the stand
    beside the midfield and the taxi to the threshold end a long way."""
    from meridianatc.atc.ground import (
        PATH_PARKING,
        PATH_RUNWAY,
        PATH_TAXI,
        GroundLayout,
    )
    points = [
        (0, 0, 0.0, 60.0),       # 0
        (0, 0, 200.0, 0.0),      # 1 junction near the threshold
        (0, 0, 200.0, 60.0),     # 2 taxi node by it
        (0, 0, 1700.0, 0.0),     # 3 midfield junction
        (0, 0, 1700.0, 60.0),    # 4 taxi node by the stand
        (0, 0, 0.0, 0.0),        # 5 runway, threshold end
        (0, 0, 3500.0, 0.0),     # 6 runway, far end
        (0, 0, 0.0, -700.0),     # 7 the detour's first leg
        (0, 0, 1700.0, -700.0),  # 8 the detour's far leg
    ]
    parkings = [(8, 0, "", "", 1, 0, 90.0, 20.0, 1700.0, 80.0)]
    paths = [
        (PATH_RUNWAY, 0, 0, "", 5, 1, -1),
        (PATH_RUNWAY, 0, 0, "", 1, 3, -1),
        (PATH_RUNWAY, 0, 0, "", 3, 6, -1),
        (PATH_TAXI, 0, 0, "", 2, 1, -1),
        (PATH_TAXI, 0, 0, "", 4, 3, -1),
        (PATH_TAXI, 0, 0, "D", 4, 8, -1),
        (PATH_TAXI, 0, 0, "D", 8, 7, -1),
        (PATH_TAXI, 0, 0, "D", 7, 0, -1),
        (PATH_TAXI, 0, 0, "D", 0, 2, -1),
        (PATH_PARKING, 0, 0, "", 4, 0, -1),
    ]
    return GroundLayout("TEST", 51.0, 0.0, points, parkings, paths, ["D"])


def test_the_entry_is_the_cheapest_drive_not_the_nearest_threshold():
    """An entry beside the aeroplane beats the long taxi to the threshold
    end -- a three-kilometre taxi is the slow way to a short backtrack."""
    layout = _runway_layout()
    legs = layout.departure(layout.stands[0], (1700.0, 80.0),
                            (0.0, 0.0), 90.0, 3500.0)
    assert legs is not None
    _taxi_out, line_up = legs
    runway_nodes = [n for n in line_up.nodes
                    if n in layout.runway_nodes]
    assert ("p", 3) in runway_nodes and ("p", 1) not in runway_nodes


def test_an_entry_too_short_for_the_takeoff_is_not_used():
    """The remaining strip past the midfield entry is no departure for a
    heavy; only the threshold-side entry is asked for then."""
    layout = _runway_layout()
    legs = layout.departure(layout.stands[0], (1700.0, 80.0),
                            (0.0, 0.0), 90.0, 3500.0, need_m=2200.0)
    assert legs is not None
    _taxi_out, line_up = legs
    runway_nodes = [n for n in line_up.nodes
                    if n in layout.runway_nodes]
    assert ("p", 1) in runway_nodes


# -- the lifelike-controls-and-phraseology overhaul ---------------------------

def test_the_yaw_rate_cap_schedules_down_with_speed():
    """Fifteen degrees a second is a ramp turn; the same ask at twenty-five
    knots was the swerve."""
    from meridianatc.atc.autonomous._shared import _yaw_rate_cap
    assert _yaw_rate_cap(5.0) == pytest.approx(15.0)
    assert _yaw_rate_cap(8.0) == pytest.approx(15.0)
    assert _yaw_rate_cap(17.0) < 15.0
    assert _yaw_rate_cap(26.0) == pytest.approx(6.0)
    assert _yaw_rate_cap(60.0) == pytest.approx(6.0)


def test_the_nosewheel_fades_to_nothing_while_the_pedals_keep_their_share():
    """The tiller's authority is gone by seventy knots; the demand itself is
    not -- the rudder is still steering."""
    from meridianatc.atc.autonomous._shared import _nws_fade
    assert _nws_fade(10.0) == pytest.approx(1.0)
    assert _nws_fade(20.0) == pytest.approx(1.0)
    assert 0.0 < _nws_fade(45.0) < 1.0
    assert _nws_fade(70.0) == pytest.approx(0.0)
    assert _nws_fade(120.0) == pytest.approx(0.0)


def test_the_groundspeed_bands_split_where_the_authority_changes():
    from meridianatc.atc.autonomous._shared import _gs_band
    assert _gs_band(5.0) == 0
    assert _gs_band(8.0) == 1
    assert _gs_band(16.0) == 2
    assert _gs_band(26.0) == 3
    assert _gs_band(45.0) == 4
    assert _gs_band(80.0) == 5


def _steerable():
    """An AutonomousFlight shell with the steering fields the laws read."""
    flight = _flight()
    flight._pedal_demand = 0.0
    flight._demand_streak = 0
    flight._pedal_votes = 0
    flight._pedal_flip = False
    flight._steer_dt = 0.25
    flight._turn_rate_log = []
    return flight


def test_yaw_authority_is_learned_per_band_and_read_back_nearest():
    flight = _steerable()
    learned = {}
    adapt = SimpleNamespace(
        learn=lambda key, value, blend=0.0: learned.__setitem__(key, value),
        learn_votes=lambda key, votes: None,
        set=lambda key, value: None,
    )
    flight._adapt = adapt
    flight._yaw_authority = 0.0
    flight._yaw_authority_b = {}
    # Two calls at the same band and demand side: one seeds, one blends.
    flight._pedal_check(0.5, 20.0, gs=30.0)
    flight._pedal_check(0.5, 20.0, gs=30.0)
    band = 3  # 26-45 kt
    assert flight._yaw_authority_b[band] == pytest.approx(40.0)
    assert learned["steer.authority.3"] == pytest.approx(40.0)
    assert "steer.authority" in learned     # the legacy figure is kept warm
    # The band answers its own figure; a band with no evidence falls to the
    # nearest measured one rather than the default.
    assert flight._yaw_authority_at(30.0) == pytest.approx(40.0)
    assert flight._yaw_authority_at(60.0) == pytest.approx(40.0)


def test_yaw_authority_defaults_when_nothing_is_learned():
    flight = _steerable()
    flight._yaw_authority = 0.0
    flight._yaw_authority_b = {}
    assert flight._yaw_authority_at(30.0) == pytest.approx(30.0)


def test_the_pedal_check_ignores_evidence_that_is_not_the_wheels():
    """A parked aeroplane nosed round by a tug teaches the steering nothing."""
    flight = _steerable()
    learned = {}
    flight._adapt = SimpleNamespace(
        learn=lambda key, value, blend=0.0: learned.__setitem__(key, value),
        learn_votes=lambda key, votes: None,
        set=lambda key, value: None,
    )
    flight._yaw_authority = 0.0
    flight._yaw_authority_b = {}
    flight._pedal_check(0.0, 20.0, gs=0.0)     # no demand: no sample
    flight._pedal_check(0.5, 1.0, gs=30.0)     # no rate: no sample
    assert flight._yaw_authority_b == {}
    assert learned == {}


def test_the_breakaway_lever_uses_the_learned_pull_away():
    flight = _steerable()
    flight._adapt = SimpleNamespace(
        value=lambda key, default: 0.38 if key == "taxi.breakaway"
        else default)
    assert flight._breakaway_lever() == pytest.approx(0.38)
    # Below the floor the class default stands; above the ceiling it is
    # clamped rather than trusted.
    flight._adapt = SimpleNamespace(
        value=lambda key, default: 0.9 if key == "taxi.breakaway"
        else default)
    assert flight._breakaway_lever() == pytest.approx(0.5)
    flight._adapt = None
    default = flight._breakaway_lever()
    assert 0.0 < default <= 0.5


def test_the_heli_pedal_check_learns_authority_per_band():
    from meridianatc.atc.autonomous_heli import PEDAL_MAX, _pedal_check
    heli = SimpleNamespace(
        _pedal_demand=None, _pedal_hdg=None, _pedal_at=0.0,
        _demand_streak=0, _pedal_votes=0, _pedal_flip=False,
        _pedal_authority_b={})
    learned = {}
    heli._adapt = SimpleNamespace(
        learn=lambda key, value, blend=0.0: learned.__setitem__(key, value),
        learn_votes=lambda key, votes: None,
        set=lambda key, value: None,
    )
    now = time.monotonic()
    # First call seeds the previous demand; the second, a second later,
    # reads the answer. monotonic is real time, so patch the clock the
    # function keeps on self instead.
    _pedal_check(heli, PEDAL_MAX // 2, 100.0, gs=10.0)
    heli._pedal_at = now - 2.0                # a two-second tick
    _pedal_check(heli, PEDAL_MAX // 2, 120.0, gs=10.0)
    # 20 degrees over two seconds = 10 deg/s at half pedal = 20 per full.
    assert heli._pedal_authority_b[1] == pytest.approx(20.0, rel=1e-3)
    assert learned["pedals.authority.1"] == pytest.approx(20.0, rel=1e-3)
    assert "pedals.authority" in learned


def test_the_heli_steering_set_is_faded_with_groundspeed():
    """The code sends the pedal demand through _nws_fade -- verified by the
    fade itself: full at taxi, none by seventy knots, which is where a
    moving helicopter's nosewheel stops being the steering."""
    import inspect

    import meridianatc.atc.autonomous_heli as heli_mod
    from meridianatc.atc.autonomous._shared import _nws_fade
    source = inspect.getsource(heli_mod)
    assert "STEERING_SET, int(pedal * _nws_fade(gs))" in source
    assert _nws_fade(70.0) == 0.0


def test_wind_moved_uses_the_atis_materiality():
    """Five knots or thirty degrees, or a gust spread that was not there --
    what re-letters an ATIS is what earns a wind check."""
    from meridianatc.atc.traffic._shared import wind_moved, wind_snapshot
    weather = SimpleNamespace(wind_dir=260.0, wind_kt=18.0, gust_kt=0.0)
    was = wind_snapshot(weather)
    assert was == (260.0, 18.0, 0.0)
    assert not wind_moved(was, weather)
    moved = SimpleNamespace(wind_dir=260.0, wind_kt=24.0, gust_kt=0.0)
    assert wind_moved(was, moved)                      # +6 kt
    veered = SimpleNamespace(wind_dir=300.0, wind_kt=18.0, gust_kt=0.0)
    assert wind_moved(was, veered)                     # +40 deg
    gusting = SimpleNamespace(wind_dir=260.0, wind_kt=18.0, gust_kt=30.0)
    assert wind_moved(was, gusting)                    # spread appeared
    calm = wind_snapshot(SimpleNamespace(wind_dir=260.0, wind_kt=2.0,
                                         gust_kt=0.0))
    calm_shift = SimpleNamespace(wind_dir=300.0, wind_kt=2.0, gust_kt=0.0)
    assert not wind_moved(calm, calm_shift)            # veered but calm
    assert not wind_moved((), weather)                 # no baseline: silent
    assert not wind_moved(was, None)


def test_the_session_keeps_the_wind_the_clearance_carried():
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    session = FlightSession(Aircraft(parse_callsign("N123")))
    weather = SimpleNamespace(wind_dir=260.0, wind_kt=18.0, gust_kt=25.0)
    session.note_clearance_wind(weather)
    assert session.wind_at_clearance == (260.0, 18.0, 25.0)


def test_the_lineup_reason_names_who_is_on_the_strip():
    injector = _injector()
    departing = SimpleNamespace(phase="takeoff", describe=lambda lang: "a Boeing")
    landing = SimpleNamespace(phase="landed", describe=lambda lang: "a Cessna")
    assert injector._lineup_reason(departing, "en") == "departing traffic"
    assert "landing" in injector._lineup_reason(landing, "en").lower()
    assert injector._lineup_reason(None, "en") == ""


def test_when_able_renders_in_every_handoff():
    from meridianatc.atc.language import phraseology_for
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.speech import parse_callsign
    for language in ("en", "fr", "es", "de", "it", "pt"):
        p = phraseology_for(language)
        a = Aircraft(parse_callsign("N123"))
        a.language = language
        when = p.when_able()
        assert when
        text = p.handoff(a, "Ground", 121.7, when=when)
        assert when.split()[0] in text


def test_the_dormant_phrases_speak_in_every_language():
    from meridianatc.atc.language import phraseology_for
    from meridianatc.atc.phraseology import Aircraft, Weather
    from meridianatc.atc.speech import parse_callsign
    for language in ("en", "fr", "es", "de", "it", "pt"):
        p = phraseology_for(language)
        a = Aircraft(parse_callsign("N123"))
        a.language = language
        weather = Weather(wind_dir=260.0, wind_kt=18.0)
        assert p.wind_check(weather, a)
        assert p.make_short_approach(a, reason="traffic waiting to depart")
        assert p.cleared_low_approach(a, "27")
        assert p.line_up_and_wait(a, "27", reason="departing traffic")


def _pilot(language):
    from meridianatc.atc.language import phraseology_for
    from meridianatc.atc.pilot import PilotSpeech
    return PilotSpeech(language, phraseology_for(language))


def _aircraft(language="en"):
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.speech import parse_callsign
    a = Aircraft(parse_callsign("N123"))
    a.language = language
    return a


def test_a_first_call_carries_the_type_and_the_information():
    speech = _pilot("en")
    a = _aircraft()
    text = speech.request_taxi(a, "Ground", atis_letter="K",
                               type_name="Cessna")
    assert "Cessna" in text
    assert "information" in text and "Kilo" in text
    # Nothing learned, nothing said: the plain call is unchanged.
    plain = speech.request_taxi(a, "Ground")
    assert "information" not in plain


def test_a_first_call_without_the_extras_is_unchanged():
    speech = _pilot("en")
    a = _aircraft()
    assert speech.request_vfr_departure(a, "Tower")
    assert speech.inbound_for_landing(a, "Tower")
    assert speech.checking_in(a, "Approach", 4000.0, climbing=False)


def test_the_information_letter_speaks_every_language():
    for language in ("fr", "es", "de", "it", "pt"):
        speech = _pilot(language)
        a = _aircraft(language)
        text = speech.request_taxi(a, "Sol", atis_letter="K")
        assert text


def test_the_continue_carries_the_expectation_the_book_gives_it():
    # JO 7110.65 3-10-1: "continue" is issued with more -- "expect landing
    # clearance" is its own example, not "I'll call your clearance".
    from meridianatc.atc.language import phraseology_for
    p = phraseology_for("en")
    p.dialect, p.region = "faa", "faa"
    a = _aircraft()
    text = p.tower_continue(a, "27")
    assert "continue" in text
    assert "expect landing clearance" in text
    assert "call your clearance" not in text


def test_the_departing_traffic_says_it_the_way_the_towers_do():
    # "Traffic departing prior to your arrival" is the form working towers
    # use with an anticipated-separation landing clearance.
    from meridianatc.atc.language import phraseology_for
    p = phraseology_for("en")
    assert p.departing_prior() == "traffic departing prior to your arrival"


def test_the_wake_caution_names_the_aeroplane_that_makes_the_wake():
    # JO 7110.65 2-1-20: "CAUTION WAKE TURBULENCE (traffic information)",
    # with "heavy" or "super" in the description when it is one.
    from meridianatc.atc.language import phraseology_for
    from meridianatc.atc.trafficinfo import wake_traffic
    p = phraseology_for("en")
    assert "heavy Boeing" in wake_traffic("B77W", "departing")
    assert "departing" in wake_traffic("B77W", "departing")
    assert "super Airbus" in wake_traffic("A388", "landing")
    assert wake_traffic("B738", "landing").startswith("Boeing")
    assert not wake_traffic("C172", "landing").startswith(("heavy", "super"))
    assert not wake_traffic("", "departing")
    text = p.caution_wake(wake_traffic("B77W", "departing"))
    assert text == "caution wake turbulence, heavy Boeing departing"
    # No traffic known: the caution alone, as before.
    assert p.caution_wake() == "caution wake turbulence"
    assert p.caution_wake("") == "caution wake turbulence"


def test_the_wake_caution_takes_traffic_in_every_language():
    from meridianatc.atc.language import phraseology_for
    from meridianatc.atc.trafficinfo import wake_traffic
    for language in ("en", "fr", "es", "de", "it", "pt"):
        p = phraseology_for(language)
        assert p.caution_wake()
        assert p.caution_wake("Boeing")
        described = wake_traffic("B77W", "departing", language)
        assert "heavy Boeing" in described
        assert p.caution_wake(described).endswith(described)


def _head_on(lon=-7.94, lat=40.0):
    from meridianatc.atc.trafficinfo import Contact
    return Contact(
        written="CS-XXX", type_code="C172", latitude=lat, longitude=lon,
        altitude_ft=3000.0, heading=270.0, ground_speed_kt=200.0)


def test_the_avoiding_turn_is_made_once_and_held_until_the_meeting_is_past():
    """The turn used to be dropped the moment it worked -- the avoidance
    track no longer met anybody -- which handed the aeroplane straight
    back to the conflict, and added to on every look until it did."""
    fo, state = _enroute_fo([_head_on()])
    events = []
    fo.on_event = lambda *a, **kw: events.append(a)
    fo._watch_traffic(state, 100.0)
    assert abs(fo.state.avoid_heading - 120.0) < 0.5
    assert fo.state.avoid_from == 90.0

    # Still closing, same look: the turn stands, said once.
    fo._watch_traffic(state, 101.0)
    assert abs(fo.state.avoid_heading - 120.0) < 0.5
    assert fo.state.avoid_until == 131.0

    # Now established on the avoidance track, which is clear -- but the
    # track that was left still meets them: the turn is kept.
    state.track_true = state.heading_true = 120.0
    fo._watch_traffic(state, 102.0)
    assert fo.state.avoid_key == "CS-XXX"
    assert abs(fo.state.avoid_heading - 120.0) < 0.5
    assert fo.state.avoid_until == 132.0
    assert len(events) == 1

    # Behind and opening: the route is picked up again.
    fo.brain.contacts = lambda: [_head_on(lon=-8.06)]
    fo._watch_traffic(state, 103.0)
    assert fo.state.avoid_key == ""
    assert fo.state.avoid_from == -1.0


def test_an_avoidance_turn_outlived_ends_and_a_real_conflict_turns_anew():
    """The avoid-until refresh alone could hold a deviation forever: a
    contact the geometry kept "meeting" on either track refreshed it
    every look, and the aeroplane sat on the avoidance heading for the
    rest of the flight. Past the hold's cap the deviation ends -- and
    if the other aeroplane still conflicts, the next look turns anew."""
    fo, state = _enroute_fo([_head_on()])
    fo._watch_traffic(state, 100.0)
    assert abs(fo.state.avoid_heading - 120.0) < 0.5

    # Still meeting the abandoned track at +200s: the hold ends anyway,
    # and the still-real meeting is worked out again from this look.
    state.track_true = state.heading_true = 120.0
    fo._watch_traffic(state, 301.0)
    assert fo.state.avoid_since == 301.0 or fo.state.avoid_key == ""

    # No conflict anywhere: the watch stays off.
    fo.brain.contacts = lambda: [_head_on(lon=-8.5)]
    fo.state.avoid_key = "CS-XXX"
    fo.state.avoid_since = 1.0
    fo.state.avoid_until = 60.0
    fo._watch_traffic(state, 301.0)
    assert fo.state.avoid_key == ""
    assert fo.state.avoid_since == 0.0


def test_head_on_a_little_right_of_the_nose_is_still_a_right_turn():
    """Two aeroplanes turning towards each other is the one outcome the
    rules of the air exist to prevent."""
    fo, state = _enroute_fo([_head_on(lat=39.9995)])
    fo._watch_traffic(state, 100.0)
    assert abs(fo.state.avoid_heading - 120.0) < 0.5


def test_the_avoiding_turn_is_flown_over_nav_and_in_the_bugs_own_frame(
        monkeypatch):
    """A cruise on NAV announced the turn and never flew it; and the bug
    is magnetic where the geometry is true."""
    import time as _time

    from meridianatc.navdata import magvar
    fo, state = _enroute_fo([])
    fo._dir = Mock()
    fo.sim = fo._dir.sim
    monkeypatch.setattr(magvar, "variation", lambda lat, lon: -5.0)
    want = magvar.to_magnetic(120.0, -5.0) % 360.0
    assert abs(want - 120.0) > 1.0

    fo._fly_avoidance(state)
    fo.director.heading.assert_not_called()

    fo.state.avoid_key = "CS-XXX"
    fo.state.avoid_heading = 120.0
    fo.state.avoid_until = _time.monotonic() + 30.0
    fo._fly_avoidance(state)
    fo.director.heading.assert_called_once()
    assert abs(fo.director.heading.call_args.args[0] - want) < 0.01

    # Out of date: nothing is flown for a meeting long past.
    fo.director.reset_mock()
    fo.state.avoid_until = _time.monotonic() - 1.0
    fo._fly_avoidance(state)
    fo.director.heading.assert_not_called()


def test_the_avoidance_does_not_outlive_the_enroute_phases():
    from meridianatc.atc.autonomous import AutonomousPhase
    fo, state = _enroute_fo([_head_on()])
    fo._watch_traffic(state, 100.0)
    assert fo.state.avoid_key
    fo.state.phase = AutonomousPhase.APPROACH
    fo._watch_traffic(state, 101.0)
    assert fo.state.avoid_key == "" and fo.state.avoid_heading == -1.0


def test_a_descent_whose_vector_ended_takes_the_route_back():
    """Climb and cruise re-ask for NAV when no heading is assigned; the
    descent kept the last vector's heading for ever."""
    fo, state = _descending_fo(9500.0)
    fo._phase_descent(state, 100.0)
    fo.director.nav.assert_not_called()

    fo.session.assigned_heading = None
    fo._phase_descent(state, 101.0)
    fo.director.nav.assert_called_with(True)
    assert fo._apply_flight_director.call_args.kwargs["heading"] == -1.0


def test_a_vector_in_level_cruise_is_flown_not_left_to_nav():
    """A heading read back at the cruise level was answered with NAV on
    the route: only an aeroplane off its level put the bug where the
    controller asked."""
    fo, state = _cruising_fo(80.0)
    fo._phase_cruise(state, 100.0)
    fo._dir.nav.assert_called_with(True)
    fo._dir.heading.assert_not_called()

    fo, state = _cruising_fo(80.0)
    fo.session.assigned_heading = 250.0
    fo._phase_cruise(state, 100.0)
    fo._dir.nav.assert_not_called()
    fo._dir.heading.assert_called_with(250.0)


def _held_fo(monkeypatch):
    from meridianatc.atc import holds
    from meridianatc.navdata import magvar
    monkeypatch.setattr(magvar, "variation", lambda lat, lon: 0.0)
    fo, state = _enroute_fo([])
    fo._dir = Mock()
    fo.sim = fo._dir.sim
    # The fix dead ahead (east), the field beyond it: inbound 090.
    fo.session.hold = holds.Hold(
        fix="XAMAX", lat=40.0, lon=-7.9, inbound_true=90.0, side="west",
        leg_minutes=1.0, altitude_ft=8000.0, reason="sequence",
        efc_hhmm="1200")
    return fo, state


def _bug(fo):
    return fo._dir.heading.call_args.args[0]


def test_the_hold_the_controller_gave_is_flown(monkeypatch):
    """The hold was read back and the route flown on past the fix, into
    "verify holding". The racetrack: to the fix, right turn outbound,
    the leg's minute, right turn back in."""
    fo, state = _held_fo(monkeypatch)
    fo._fly_hold(state, 100.0)
    assert fo.state.hold_leg == "to_fix"
    assert abs(_bug(fo) - 90.0) < 1.0

    # Over the fix, still pointing inbound: outbound is a half-turn, and
    # it goes round to the right -- the bug leads the nose by a quarter.
    state.longitude = -7.9
    fo._fly_hold(state, 110.0)
    assert fo.state.hold_leg == "outbound"
    assert abs(_bug(fo) - 180.0) < 1.0
    assert fo.state.hold_leg_at == 0.0

    # Round onto the outbound heading: the bug is the leg and the clock
    # starts wings level.
    state.track_true = state.heading_true = 265.0
    state.latitude = 39.96
    fo._fly_hold(state, 150.0)
    assert abs(_bug(fo) - 270.0) < 1.0
    assert fo.state.hold_leg_at == 150.0
    fo._fly_hold(state, 200.0)
    assert fo.state.hold_leg == "outbound"

    # The minute is up: back to the fix, to the right again.
    state.track_true = state.heading_true = 270.0
    state.longitude = -7.97
    fo._fly_hold(state, 211.0)
    assert fo.state.hold_leg == "inbound"
    # (the fix bears 053 from here: 143 degrees round, clockwise)
    assert abs(_bug(fo) - 53.3) < 1.0

    # Cleared to leave: the hold is no longer anybody's heading.
    fo.session.hold = None
    fo._dir.reset_mock()
    fo._fly_hold(state, 220.0)
    fo._dir.heading.assert_not_called()
    assert fo.state.hold_leg == ""


def test_a_hold_is_flown_at_the_level_it_was_given_at():
    from meridianatc.atc import holds
    fo, state = _descending_fo(8000.0)
    fo.session.assigned_altitude_ft = None
    fo.session.hold = holds.Hold(
        fix="XAMAX", lat=40.0, lon=-7.9, inbound_true=90.0, side="west",
        leg_minutes=1.0, altitude_ft=8000.0, reason="sequence",
        efc_hhmm="1200")
    fo._phase_descent(state, 100.0)
    assert fo._apply_flight_director.call_args.kwargs["altitude"] == 8000.0
    assert fo._apply_flight_director.call_args.kwargs["vs_fpm"] == 0.0


def test_a_vfr_ceiling_caps_the_climb_until_it_is_lifted_or_left_behind():
    """"Not above two thousand five hundred" was read back and the FO
    climbed to the filed level through it."""
    fo, state = _enroute_fo([])
    field = SimpleNamespace(lat=40.0, lon=-8.0)
    fo.session = SimpleNamespace(
        ifr=False, assigned_altitude_ft=0.0, cruise_altitude_ft=6500.0,
        initial_altitude_ft=0.0, vfr_ceiling_ft=2500.0, departure=field)
    state.distance_to = lambda lat, lon: 5.0
    assert fo._altitude_target(state) == 2500.0
    # A level the controller assigned is the controller's own word.
    fo.session.assigned_altitude_ft = 4000.0
    assert fo._altitude_target(state) == 4000.0
    fo.session.assigned_altitude_ft = 0.0
    # Well clear of the field, or lifted, or IFR: the filed level.
    state.distance_to = lambda lat, lon: 40.0
    assert fo._altitude_target(state) == 6500.0
    state.distance_to = lambda lat, lon: 5.0
    fo.session.vfr_ceiling_ft = 0.0
    assert fo._altitude_target(state) == 6500.0
    fo.session.vfr_ceiling_ft = 2500.0
    fo.session.ifr = True
    assert fo._altitude_target(state) == 6500.0


# -- "proceed direct" and the tower's orbit, flown ------------------------------

def _direct_fo(monkeypatch):
    from meridianatc.atc.route import Route, RouteFix
    from meridianatc.navdata import magvar
    monkeypatch.setattr(magvar, "variation", lambda lat, lon: 0.0)
    fo, state = _enroute_fo([])
    fo._dir = Mock()
    fo.sim = fo._dir.sim
    # A dog-leg: north to BBBBB, back down to CCCCC due east of here,
    # then on east to DDDDD and the field.
    fixes = [RouteFix("LPAA", 40.0, -8.5, kind="apt"),
             RouteFix("BBBBB", 40.5, -7.8),
             RouteFix("CCCCC", 40.0, -7.5),
             RouteFix("DDDDD", 40.0, -7.0),
             RouteFix("LPZZ", 40.0, -6.5, kind="apt")]
    fo.brain.route = Route(fixes=fixes)
    fo.session.route_direct = ""
    fo.session.assigned_heading = None
    fo.session.hold = None
    return fo, state


def test_a_direct_is_flown_to_its_fix_and_on_along_the_route(monkeypatch):
    """"Proceed direct CCCCC" was read back and NAV flew on up to BBBBB."""
    fo, state = _direct_fo(monkeypatch)
    fo._fly_direct(state)
    fo._dir.heading.assert_not_called()

    fo.session.route_direct = "CCCCC"
    fo._fly_direct(state)
    assert abs(_bug(fo) - 90.0) < 1.0 and fo.state.ap_hdg_mode

    # A vector given meanwhile is the controller's later word.
    fo._dir.heading.reset_mock()
    fo.session.assigned_heading = 120.0
    fo._fly_direct(state)
    fo._dir.heading.assert_not_called()
    fo.session.assigned_heading = None

    # At the fix the controller's direct is over; the FMS still has the
    # leg that was cut as its own, so the next fix is flown from here.
    state.longitude = -7.5
    fo.session.route_direct = ""
    fo._fly_direct(state)
    assert fo.state.direct_fix == "DDDDD"
    assert abs(_bug(fo) - 90.0) < 1.0

    # The last fix before the field: the direct still holds, on to the
    # field itself -- the FMS's active leg is still the cut one, so NAV
    # is not handed back while the aeroplane is enroute.
    state.longitude = -7.0
    fo._dir.heading.reset_mock()
    fo._fly_direct(state)
    assert fo.state.direct_fix == "DDDDD"
    assert fo.state.direct_index == len(fo.brain.route.fixes)
    assert abs(_bug(fo) - 90.0) < 1.0


def test_a_direct_ends_where_the_arrival_or_the_ground_begins(monkeypatch):
    from meridianatc.atc.autonomous import AutonomousPhase
    fo, state = _direct_fo(monkeypatch)
    fo.session.route_direct = "CCCCC"
    fo._fly_direct(state)
    fo.session.route_direct = ""
    fo.state.star_nav = object()
    fo._fly_direct(state)
    assert fo.state.direct_fix == ""

    fo.state.star_nav = None
    fo.session.route_direct = "CCCCC"
    fo._fly_direct(state)
    assert fo.state.direct_fix == "CCCCC"
    fo.state.phase = AutonomousPhase.APPROACH
    fo._fly_direct(state)
    assert fo.state.direct_fix == ""


def _orbiting_fo(monkeypatch, agl=900.0):
    from meridianatc.navdata import magvar
    monkeypatch.setattr(magvar, "variation", lambda lat, lon: 0.0)
    fo, state = _enroute_fo([])
    fo.session.orbiting = "left"
    fo._hold_level = Mock()
    fo._approach_speed = lambda state: 75.0
    state.altitude_ft, state.altitude_agl_ft = 1200.0, agl
    return fo, state


def test_the_towers_orbit_is_flown_level_all_the_way_round(monkeypatch):
    """"Orbit left" was read back and the final flown on down."""
    fo, state = _orbiting_fo(monkeypatch)
    assert fo._fly_orbit(state)
    target, heading, speed = fo._hold_level.call_args.args[1:]
    assert (target, heading, speed) == (1200.0, 0.0, 75.0)

    # Round to the left, the bug a quarter ahead of the nose, the height
    # the orbit began at kept though the aeroplane has sagged.
    turned = 0
    while fo._fly_orbit(state):
        turned += 20
        assert turned <= 380
        state.track_true = state.heading_true = (90.0 - turned) % 360.0
        state.altitude_ft = 1150.0
        assert fo._hold_level.call_args.args[1] == 1200.0
    assert turned >= 360

    # The tower calls it complete: the next orbit is a new one.
    fo.session.orbiting = ""
    assert not fo._fly_orbit(state)
    assert fo.state.orbit_alt_ft == 0.0 and fo.state.orbit_turned == 0.0


def test_an_orbit_is_not_begun_on_short_final(monkeypatch):
    fo, state = _orbiting_fo(monkeypatch, agl=300.0)
    assert not fo._fly_orbit(state)
    fo._hold_level.assert_not_called()


def test_a_direct_through_a_fix_the_route_names_twice(monkeypatch):
    """On AAAAA-BBBBB-CCCCC-BBBBB-DDDDD the second BBBBB used to advance
    from the first: reaching it picked CCCCC again, and the aeroplane
    flew the middle of the route for ever."""
    from meridianatc.atc.route import Route, RouteFix
    from meridianatc.navdata import magvar
    monkeypatch.setattr(magvar, "variation", lambda lat, lon: 0.0)
    fo, state = _enroute_fo([])
    fo._dir = Mock()
    fo.sim = fo._dir.sim
    fixes = [RouteFix("AAAAA", 40.0, -8.0),
             RouteFix("BBBBB", 40.5, -7.5),
             RouteFix("CCCCC", 40.0, -7.0),
             RouteFix("BBBBB", 40.5, -6.5),
             RouteFix("DDDDD", 40.0, -6.0),
             RouteFix("LPZZ", 40.0, -5.5, kind="apt")]
    fo.brain.route = Route(fixes=fixes)
    fo.session.route_direct = ""
    fo.session.assigned_heading = None
    fo.session.hold = None

    fo.session.route_direct = "AAAAA"
    state.longitude = -8.4
    fo._fly_direct(state)
    assert fo.state.direct_fix == "AAAAA"
    assert abs(_bug(fo) - 90.0) < 1.0

    # Fix to fix along the route: the second BBBBB is fixes[3], reached
    # after CCCCC.
    for lat, lon, ident, index in ((40.0, -8.0, "BBBBB", 1),
                                   (40.5, -7.5, "CCCCC", 2),
                                   (40.0, -7.0, "BBBBB", 3)):
        state.latitude, state.longitude = lat, lon
        fo._fly_direct(state)
        assert fo.state.direct_fix == ident
        assert fo.state.direct_index == index

    # At the second BBBBB the next leg is BBBBB-DDDDD -- not a return to
    # CCCCC, which resolving the ident again would pick.
    state.latitude, state.longitude = 40.5, -6.5
    fo._fly_direct(state)
    assert fo.state.direct_fix == "DDDDD"
    assert fo.state.direct_index == 4
    assert 90.0 < _bug(fo) < 180.0


def test_a_direct_to_the_last_fix_keeps_the_field_ahead(monkeypatch):
    """Reaching the last fix before the field used to hand NAV back to
    an FMS whose active leg was still the cut one: the aeroplane turned
    back round the legs the direct skipped. The field is the target
    until the approach, a STAR or a vector takes it."""
    fo, state = _direct_fo(monkeypatch)
    fo.session.destination = SimpleNamespace(
        ident="LPZZ", lat=40.0, lon=-6.5, elev_ft=0.0)

    fo.session.route_direct = "CCCCC"
    fo._fly_direct(state)
    state.longitude = -7.5
    fo.session.route_direct = ""
    fo._fly_direct(state)
    assert fo.state.direct_fix == "DDDDD"

    # Over DDDDD, forty miles from the field, the direct still holds:
    # the bug stays on the field's bearing through every cycle after.
    state.longitude = -7.0
    fo._fly_direct(state)
    assert fo.state.direct_index == len(fo.brain.route.fixes)
    assert abs(_bug(fo) - 90.0) < 2.0
    for _ in range(3):
        fo._dir.heading.reset_mock()
        fo._fly_direct(state)
        assert abs(_bug(fo) - 90.0) < 1.0
        fo._dir.nav.assert_not_called()

    # The approach is where it ends.
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.APPROACH
    fo._dir.heading.reset_mock()
    fo._fly_direct(state)
    assert fo.state.direct_index == -1
    assert fo.state.direct_fix == ""
    fo._dir.heading.assert_not_called()


def test_a_diversion_drops_the_old_fields_orbit(monkeypatch):
    """A mayday mid-orbit used to keep the orbit: entering the new
    field's approach resumed the old tower's turn at the old height."""
    from meridianatc.atc.autonomous import AutonomousPhase
    fo, state = _orbiting_fo(monkeypatch)
    assert fo._fly_orbit(state)
    # Ninety degrees round at the altitude it began.
    state.track_true = state.heading_true = 270.0
    fo._fly_orbit(state)
    assert fo.state.orbit_turned > 0.0

    fo.use_route = None
    fo.state.phase = AutonomousPhase.APPROACH
    fo.session.destination = SimpleNamespace(
        ident="LPSO", lat=41.0, lon=-7.0, elev_ft=0.0)
    fo._route_diversion()

    assert fo.session.orbiting == ""
    assert fo.session.orbit_track == -1.0
    assert fo.session.orbit_turned == 0.0
    assert fo.state.orbit_alt_ft == 0.0
    assert fo.state.orbit_track == -1.0
    assert fo.state.orbit_turned == 0.0
    assert fo.state.phase == AutonomousPhase.DESCENT
    # And the new field's approach flies no orbit at all.
    fo._hold_level.reset_mock()
    assert not fo._fly_orbit(state)
    fo._hold_level.assert_not_called()
