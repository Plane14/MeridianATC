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
    assert heli._pedal_authority_b[1] == pytest.approx(20.0)
    assert learned["pedals.authority.1"] == pytest.approx(20.0)
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
