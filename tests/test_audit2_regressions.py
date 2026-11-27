"""Regressions from the second audit pass: chain-leg state, altimeter
units, checklist toggles and held live traffic."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest


def _flight():
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    return flight


def test_an_inches_altimeter_is_dialled_in_hectopascals():
    """A "2992" from the controller is not written as 29.92 hPa."""
    from meridianatc.atc.autonomous import AutonomousFlight
    convert = AutonomousFlight._altimeter_hpa
    assert convert(29.92) == pytest.approx(1013.2, abs=0.2)
    assert convert(30.00) == pytest.approx(1015.9, abs=0.2)
    assert convert(1013) == 1013
    assert convert(0) == 0


def test_the_radio_dial_compares_like_units():
    """The dialled check cannot call 29.92 inHg 'already at 29.92 hPa'."""
    flight = _flight()
    flight._dialled_at = 0.0
    flight.state.squawk_dialled = "1200"
    flight.state.altimeter_dialled = 0.0
    flight.sim = SimpleNamespace(set_squawk=lambda code: True)
    flight._dir = SimpleNamespace(kohlsman=Mock(), sim=flight.sim)
    sim_state = SimpleNamespace(
        kohlsman_hpa=1013.0, transponder_code="1200")
    flight.session = SimpleNamespace(squawk="1200", altimeter=29.92)
    # 1013.25 hPa showing, 29.92 given: the two match within half a
    # hectopascal -- nothing is dialled, the figure is remembered.
    flight._dial_radios(sim_state, 100.0)
    flight._dir.kohlsman.assert_not_called()
    assert flight.state.altimeter_dialled == pytest.approx(1013.2, abs=0.2)


def test_a_chained_leg_forgets_the_last_legs_session():
    """The readback owed on leg one is not owed again on leg two."""
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign

    flight = _flight()
    session = FlightSession(Aircraft(parse_callsign("N123")))
    # Leg one's leavings.
    session.squawk = "4567"
    session.altimeter = 29.92
    session.assigned_altitude_ft = 11000.0
    session.assigned_heading = 270.0
    session.cleared_landing = True
    session.going_around = True
    session.emergency = "MAYDAY"
    session.pending = object()
    session.taxi_path = ["A", "B"]
    session.log.append((0.0, "tower", "cleared to land"))
    flight.session = session
    flight.navdb = SimpleNamespace(
        airport=lambda ident: SimpleNamespace(ident=ident))

    flight._reset_session_for_leg({
        "origin": "EGLL", "destination": "LFPG",
        "cruise_altitude_ft": 35000, "ifr": True})

    assert session.squawk == "1200"
    assert session.altimeter == 0.0
    assert session.assigned_altitude_ft == 0.0
    assert session.assigned_heading is None
    assert session.cleared_landing is False
    assert session.going_around is False
    assert session.emergency == ""
    assert session.pending is None
    assert session.taxi_path == []
    # What is kept: the aeroplane, the transcript, the new leg's fields.
    assert session.aircraft.callsign.written == "N123"
    assert session.log == [(0.0, "tower", "cleared to land")]
    assert session.departure.ident == "EGLL"
    assert session.destination.ident == "LFPG"
    assert session.destination_input == "LFPG"
    assert session.cruise_altitude_ft == 35000


def test_a_toggle_switch_is_asked_once_per_checklist_item():
    """A seatbelt toggle re-asked would turn the sign back off."""
    flight = _flight()
    flight._checklist_sent = set()
    flight._switch_for = Mock(return_value=True)
    flight.crew = SimpleNamespace(
        checklists=SimpleNamespace(_expected=lambda c: c.value))
    state = SimpleNamespace(seatbelt_sign=False)
    check = SimpleNamespace(
        var="seatbelt_sign", value=1, op="eq", tolerance=0.0)
    item = SimpleNamespace(checks=(check,))

    assert flight._perform_check_item(state, item) is True
    assert flight._perform_check_item(state, item) is True
    assert flight._switch_for.call_count == 1
    # The next item gets its own ask.
    flight._checklist_sent.clear()
    assert flight._perform_check_item(state, item) is True
    assert flight._switch_for.call_count == 2


def test_a_held_live_aeroplane_is_let_go_when_its_strip_goes():
    """An unreleased hold draws a statue on the feed forever."""
    from meridianatc.atc.traffic.core import TrafficInjector

    injector = TrafficInjector.__new__(TrafficInjector)
    injector.hold_live = Mock()
    held = SimpleNamespace(live_held="runway", hex="abc123")
    injector._release_held(held)
    injector.hold_live.assert_called_once_with("abc123", False)
    assert held.live_held == ""
    # And one never held asks nothing of the feed.
    injector.hold_live.reset_mock()
    injector._release_held(SimpleNamespace(live_held="", hex="def456"))
    injector.hold_live.assert_not_called()


def test_the_turnaround_believes_the_tanks_not_the_write():
    """A tank write the model ignores is nobody bringing the fuel: the
    uplift is checked against the gauges like a real turnaround, and a
    write that changed nothing falls through to the shortfall rules."""
    from meridianatc.atc.autonomous import AutonomousPhase

    flight = _flight()
    flight._turnaround_service = True
    flight._is_helicopter = False
    flight._fuel_capacity_seen = 0.0
    flight.navdb = SimpleNamespace(
        airport=lambda ident: SimpleNamespace(ident=ident))
    flight.session = SimpleNamespace(destination=None)
    flight.brain = SimpleNamespace(ground=None)
    flight._pick_alternate = Mock(return_value=None)
    flight._fuel_needed_gal = Mock(return_value={
        "leg_gal": 50.0, "divert_gal": 10.0, "reserve_gal": 15.0,
        "total_gal": 75.0, "alternate": None})
    flight._write_fuel = Mock(side_effect=lambda gal: (
        setattr(flight.state, "fuel_wrote", True) or True))
    flight.on_event = Mock()
    flight.state.plan_flight = {"destination": "LFPG"}
    state = SimpleNamespace(engines_running=False, rotor_rpm_pct=0.0,
                            fuel_total_gal=10.0, fuel_capacity_gal=100.0)

    # The write is accepted, but the answer comes from the gauges: the
    # stand waits on them a moment.
    assert flight._service_turnaround(state, 100.0) is False
    flight._write_fuel.assert_called_once_with(75.0)
    assert flight._service_turnaround(state, 105.0) is False
    assert flight.state.phase != AutonomousPhase.SHUTDOWN

    # Fuel that never arrives past the grace window is a shortfall like
    # any other: the FO signs the aeroplane off, he does not guess.
    assert flight._service_turnaround(state, 120.0) is False
    assert flight.state.phase == AutonomousPhase.SHUTDOWN


def test_the_turnaround_loads_the_leg_the_tanks_confirm():
    """The write lands where the gauges say so: the leg is loaded on
    the tanks' figure, and a fuller tank still covers a shorter leg."""
    from meridianatc.atc.autonomous import AutonomousPhase  # noqa: F401

    flight = _flight()
    flight._turnaround_service = True
    flight._is_helicopter = False
    flight._fuel_capacity_seen = 0.0
    flight.navdb = SimpleNamespace(
        airport=lambda ident: SimpleNamespace(ident=ident))
    flight.session = SimpleNamespace(destination=None)
    flight.brain = SimpleNamespace(ground=None)
    flight._pick_alternate = Mock(return_value=None)
    flight._fuel_needed_gal = Mock(return_value={
        "leg_gal": 50.0, "divert_gal": 10.0, "reserve_gal": 15.0,
        "total_gal": 75.0, "alternate": None})
    flight._write_fuel = Mock(side_effect=lambda gal: (
        setattr(flight.state, "fuel_wrote", True) or True))
    flight.on_event = Mock()
    flight.state.plan_flight = {"destination": "LFPG"}
    state = SimpleNamespace(engines_running=False, rotor_rpm_pct=0.0,
                            fuel_total_gal=10.0, fuel_capacity_gal=100.0)

    assert flight._service_turnaround(state, 200.0) is False
    # The model takes the write: the next read carries the uplift.
    state.fuel_total_gal = 76.0
    assert flight._service_turnaround(state, 201.0) is True


def test_the_turnaround_shuts_the_engines_at_blocks():
    """A crew that sets the parking brake shuts the engines: the stand
    delay is not spent burning the fuel the next leg was just loaded
    with."""
    from meridianatc.atc.autonomous import AutonomousPhase

    flight = _flight()
    flight.state.phase = AutonomousPhase.CHAIN_NEXT
    flight.state.sub = "delay"
    flight.state.phase_entered_at = 100.0
    flight._chain_delay = 120.0
    flight._shutdown_engines = Mock()
    state = SimpleNamespace(engines_running=True)

    flight._phase_chain_next(state, 110.0)
    flight._shutdown_engines.assert_called_once()

    # Already shut, nothing more is sent.
    state.engines_running = False
    flight._phase_chain_next(state, 111.0)
    flight._shutdown_engines.assert_called_once()


def test_a_leg_change_arms_the_climbs_fms_direct_again():
    """The present-position direct and the vector rewrite are spent on
    the leg's climb: a new leg that inherits 'done' never gets its own
    FMS write."""
    flight = _flight()
    flight._wind_z_hist = []
    flight._adapt = None
    flight._fms_direct_done = True
    flight._vector_fms_hdg = 270.0
    flight._fuel_leg_since = 9.0
    flight._fuel_min_since = 4.0
    flight._electrical_since = 8.0
    flight._engines_since = 6.0
    flight._nav_off_since = 7.0

    flight._reset_scratch()

    assert flight._fms_direct_done is False
    assert flight._vector_fms_hdg is None
    assert flight._fuel_leg_since == 0.0
    assert flight._fuel_min_since == 0.0
    assert flight._electrical_since == 0.0
    assert flight._engines_since == 0.0
    assert flight._nav_off_since == 0.0


def test_a_chained_leg_keeps_the_aircrafts_hull():
    """A floatplane's next leg still lands on water: the aircraft
    figures survive the state reset like the aircraft field itself."""
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.config import Config

    flight = _flight()
    flight.state.aircraft_floats = True
    flight.state.aircraft_type = "DHC2"
    flight.state.aircraft_range_nm = 400.0
    session = FlightSession(Aircraft(parse_callsign("N123")))
    flight.session = session
    flight.config = Config()
    flight.navdb = SimpleNamespace(airport=lambda ident: SimpleNamespace(
        ident=ident, lat=45.0, lon=-75.0))
    flight._dir = None
    flight._heli = None
    flight._adapt = None
    flight._wind_z_hist = []
    flight.use_route = Mock()
    flight.crew = SimpleNamespace(begin=Mock())
    flight.on_event = Mock()

    flight._load_next_flight({"origin": "CYAA", "destination": "CYZZ",
                              "cruise_altitude_ft": 4000, "ifr": True})

    assert flight.state.aircraft_floats is True
    assert flight.state.aircraft_type == "DHC2"
    flight.use_route.assert_called_once()


def test_a_go_around_drops_the_modes_the_approach_armed():
    """A missed approach flown by hand fights an AP still in the
    descent it was armed for: the modes go off with the TO/GA press."""
    from meridianatc.atc.autonomous import AutonomousPhase

    flight = _flight()
    flight.sim = Mock()
    flight._dir = Mock()
    flight._dir.sim = flight.sim
    flight.state.phase = AutonomousPhase.APPROACH
    flight.state.approach_recipe = SimpleNamespace(missed_alt_ft=4000.0)
    flight.state.ap_master = True
    flight.state.ap_vs_mode = True
    flight.state.ap_speed_mode = True
    flight.state.ap_nav_mode = True
    flight._field_elevation = Mock(return_value=0.0)
    flight.session = SimpleNamespace(assigned_altitude_ft=1800.0,
                                     assigned_heading=182.0)
    flight._prop_levers = Mock()
    flight._g_pull_cap = Mock(return_value=0.3)
    flight._perf = SimpleNamespace(goaround_kt=0.0)
    flight._vapp = Mock(return_value=65.0)
    flight._manage_radio = False
    flight.on_event = Mock()
    state = SimpleNamespace(altitude_ft=500.0, flaps_index=2,
                            source="simconnect")

    flight._start_go_around(state)

    flight._dir.autopilot.assert_called_with(False)
    flight._dir.vs_hold.assert_called_with(False)
    flight._dir.airspeed_hold.assert_called_with(False)
    assert flight.state.ap_master is False
    assert flight.state.ap_vs_mode is False
    assert flight.session.assigned_altitude_ft is None
    assert flight.session.assigned_heading is None


def test_a_go_around_pull_yields_to_the_speed():
    """The 2026-11-22 LECO crash: the go-around's hand-flown pull took
    the aeroplane to +15 degrees at 82 kt and the wing gave way. Below
    the go-around figure the nose does not come up; well below it comes
    down -- the speed flies first, the climb attitude second."""
    flight = _flight()
    flight._perf = SimpleNamespace(goaround_kt=80.0)
    flight.state.climb_kt = 0.0
    flight._vapp = Mock(return_value=65.0)

    # Room to climb in: the asked pull goes through.
    fast = SimpleNamespace(indicated_airspeed_kt=100.0)
    assert flight._go_around_pull(fast, 0.3) == 0.3
    # Under the figure the nose stops rising.
    slowing = SimpleNamespace(indicated_airspeed_kt=75.0)
    assert flight._go_around_pull(slowing, 0.25) == 0.0
    # Badly decayed: the wing is unloaded, not pulled through the stall.
    stalling = SimpleNamespace(indicated_airspeed_kt=65.0)
    assert flight._go_around_pull(stalling, 0.25) == -0.1
    # No speed read at all: no invented bound.
    deaf = SimpleNamespace(indicated_airspeed_kt=0.0)
    assert flight._go_around_pull(deaf, 0.3) == 0.3


def test_a_bus_never_seen_alive_is_not_a_collapse():
    """The 2026-11-20 case: the model whose main-bus index read zero
    from the ramp to the flare is a channel it never wired, not a bus
    that died -- the watch learns a live figure first, and only then
    knows what dead looks like."""
    from meridianatc.atc.autonomous import AutonomousPhase

    flight = _flight()
    flight.state.phase = AutonomousPhase.CRUISE
    flight._declare_emergency = Mock()
    flight._electrical_since = 100.0
    flight._fuel_known = False
    flight._fuel_capacity_seen = 0.0
    flight._fuel_needed_gal = Mock(return_value=None)
    state = SimpleNamespace(on_ground=False, main_bus_volts=0.0,
                            fuel_total_gal=100.0, fuel_capacity_gal=200.0)

    flight._monitor_emergency(state, 120.0)
    assert all(call.args[0] != "electrical"
               for call in flight._declare_emergency.call_args_list)

    # A bus that showed its volts and then lost them is the collapse.
    flight._bus_volts_ref = 14.0
    flight._electrical_since = 100.0
    flight._monitor_emergency(state, 120.0)
    flight._declare_emergency.assert_called_with(
        "electrical", state, mayday=True)

    # A variable the model never publishes is not a collapse.
    flight._declare_emergency.reset_mock()
    flight._electrical_since = 0.0
    quiet = SimpleNamespace(on_ground=False, main_bus_volts=-1.0,
                            fuel_total_gal=100.0, fuel_capacity_gal=200.0)
    flight._monitor_emergency(quiet, 120.0)
    assert all(call.args[0] != "electrical"
               for call in flight._declare_emergency.call_args_list)


def test_a_granted_direct_is_the_one_flown():
    """The direct the controller grants is the direct the FO flies:
    the fix is stored where the direct law watches, not left in the
    transcript."""
    from meridianatc.atc.controller.core import ControllerBrain

    ctl = ControllerBrain.__new__(ControllerBrain)
    fix = SimpleNamespace(ident="ALPHA", name="", lat=0.0, lon=1.0)
    ctl.plan_fixes = [fix]
    session = SimpleNamespace(
        aircraft=SimpleNamespace(), route_direct="",
        route_direct_index=-1, remember=lambda i: i)
    station = SimpleNamespace(callsign="CTR", ident="CTR",
                              position="CTR", mhz=123.45)
    p = Mock()
    p.direct_to = Mock(return_value="direct ALPHA")
    state = SimpleNamespace(latitude=0.0, longitude=0.0,
                            heading_true=90.0)

    reply = ctl._direct_on_the_plan({"fix": "ALPHA"}, session, station,
                                    p, state)

    assert session.route_direct == "ALPHA"
    assert session.route_direct_index == 0
    assert reply.text == "direct ALPHA"


def test_the_first_officer_answers_to_his_first_name():
    """A backspace byte in the pattern where a word boundary was meant
    made the name branch match nothing -- the first officer only ever
    answered to "first officer"."""
    from meridianatc.atc.interphone import FIRST_OFFICER, _addressed, fold

    who, words = _addressed(fold("marie can you check the cabin"), "en",
                            names=["julia"], first_officer=["marie"])
    assert who == FIRST_OFFICER
    assert words == "marie"
    # And a name is a whole word, not the start of a longer one.
    who, _ = _addressed(fold("marie can you check the cabin"), "en",
                        names=[], first_officer=["mar"])
    assert who != FIRST_OFFICER


def test_go_around_stops_commanding_climb_at_the_cleared_altitude():
    """The live LECO scan: the tower's 3,000-ft go-around clearance was
    read back correctly, but the vertical-speed hold stayed commanded at
    climb rate and the Baron climbed through the level. Near or above the
    cleared target the director gets altitude hold, not a continuing VS
    climb, and a clearance received after the go-around started updates
    that target."""
    import time

    flight = _flight()
    flight.sim = Mock()
    flight._dir = Mock()
    flight._dir.sim = flight.sim
    flight._apply_flight_director = Mock()
    flight._flaps_up_one_detent = Mock()
    flight._magnetic = Mock(return_value=32.0)
    flight._perf = SimpleNamespace(climb_fpm=1500.0, goaround_kt=90.0)
    flight.session = SimpleNamespace(assigned_altitude_ft=3000.0,
                                     assigned_heading=32.0)
    flight.state.sub = "go_around"
    flight.state.go_around_at = time.monotonic() - 20.0
    flight.state.missed_alt_ft = 2300.0
    flight.state.missed_nav = None
    flight.state.runway_heading = 210.0
    state = SimpleNamespace(altitude_ft=2900.0, vertical_speed_fpm=1400.0,
                            gear_down=False, has_position=False)

    flight._fly_go_around(state, time.monotonic())

    kwargs = flight._apply_flight_director.call_args.kwargs
    assert kwargs["altitude"] == 3000.0
    assert kwargs["vs_fpm"] == 0.0


def test_powered_lift_chain_planning_uses_vertical_aircraft_rules():
    """BETA's ALIA is not an airliner on the chain planner: it uses small
    fields and a powered-lift level rather than the FL390 ladder."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState

    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    flight.state.aircraft_type = "ALIA"
    flight._is_helicopter = False
    flight._perf = SimpleNamespace(ceiling_ft=0.0)
    flight.get_state = lambda: SimpleNamespace(engine_type=-1, title="BETA Alia-250")
    airport = SimpleNamespace(type="small_airport", lat=43.0, lon=-8.0)
    destination = SimpleNamespace(type="small_airport", lat=44.0, lon=-7.0)

    assert flight._has_ground_services(airport) is True
    level = flight._cruise_altitude(airport, destination, 120.0, True)
    assert level <= 10000


def test_stock_light_types_missing_from_legacy_lists_can_chain_from_small_fields():
    """The Aerolite and Stinson are piston singles even though the old
    hand-maintained light-type list never named them."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState

    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    airport = SimpleNamespace(type="small_airport")
    flight._is_helicopter = False
    for code in ("A103", "L5"):
        flight.state.aircraft_type = code
        assert flight._has_ground_services(airport) is True


def test_an_unpowered_glider_does_not_invent_a_powered_chained_leg():
    """The ASK 21 cannot refuel and taxi out on its own; without a tow
    model chaining it would be fiction, so no destination is selected."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState

    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    flight.state.aircraft_type = "AS21"
    flight.get_state = lambda: SimpleNamespace(
        engine_type=2, title="Schleicher ASK 21")
    flight.session = SimpleNamespace(departure=None)
    flight.navdb = None

    flight._select_next_destination()
    assert flight.state.chain_flights == []


def test_no_sim_chain_ranges_use_the_type_even_without_connected_state():
    """A no-sim chain plan must not give every unlisted type an airliner's
    2,500-mile legs: the stock piston, turboprop, rotor and powered-lift
    types keep their own plausible range."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState

    expected_max = {
        "A103": 100, "PA18": 300, "L5": 250, "RV10": 700,
        "BE9L": 1000, "EVOT": 900, "SF50": 1000,
        "R22": 300, "S76": 410, "ALIA": 250,
    }
    for code, maximum in expected_max.items():
        flight = AutonomousFlight.__new__(AutonomousFlight)
        flight.state = AutonomousState()
        session = SimpleNamespace(
            aircraft=SimpleNamespace(type_code=code, on_floats=False))
        flight.get_state = lambda: (_ for _ in ()).throw(RuntimeError("no sim"))
        flight._compute_aircraft_performance(session)
        assert 0 < flight.state.aircraft_range_nm <= maximum + 1, (
            code, flight.state.aircraft_range_nm)


def test_stock_light_and_turboprop_chain_levels_are_class_based():
    """PA-18/RV-10 and the King Air are absent from the legacy light-type
    list, but their chain leg must still use piston/turboprop altitudes."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState

    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    flight._is_helicopter = False
    flight._perf = SimpleNamespace(ceiling_ft=0.0)
    flight.get_state = lambda: SimpleNamespace()
    origin = SimpleNamespace(lat=43.0, lon=-8.0)
    destination = SimpleNamespace(lat=44.0, lon=-7.0)

    for code in ("PA18", "RV10", "A103", "L5"):
        flight.state.aircraft_type = code
        assert flight._cruise_altitude(origin, destination, 120.0, True) <= 6000
    flight.state.aircraft_type = "BE9L"
    assert flight._cruise_altitude(origin, destination, 450.0, True) <= 25000
