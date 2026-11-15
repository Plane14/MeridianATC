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


# -- expedite -----------------------------------------------------------------

def test_an_expedite_command_hurries_what_the_flight_does_next():
    from meridianatc.atc.flying import Command
    injector = _injector()
    flight = SimpleNamespace(expedite_until=0.0, next_at=999.0)
    injector._carry_out(flight, Command(kind="expedite"), 100.0)
    assert flight.expedite_until == 190.0
    assert flight.next_at == 100.0


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
    from meridianatc.atc.ground import GroundLayout, PATH_PARKING
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
