"""Exercise the real FO director against the X-Plane UDP translation layer."""

from __future__ import annotations

from unittest.mock import Mock

import pytest
from test_xplane import FakeXPlane, eventually

from meridianatc.atc.ap import FlightDirector
from meridianatc.sim.capabilities import autofly_limit, capabilities
from meridianatc.sim.state import AircraftState
from meridianatc.sim.xpcontrols import AP, FLAPS, HSI, READS, THROTTLE
from meridianatc.sim.xplane import EXTRA, XPlaneSource


@pytest.fixture
def xp():
    peer = FakeXPlane()
    peer.values[READS["control:autothrottle"]] = -1
    source = XPlaneSource(port=peer.port, discovery_s=0, timeout_s=.2)
    try:
        assert source.connect()
        eventually(lambda: all(key in source._values for key in READS))
        yield peer, source
    finally:
        source.close()
        peer.close()


@pytest.mark.parametrize("event,value,path,expected", [
    ("AP_HEADING_BUG_SET_EX1", 8192, AP + "heading_dial_deg_mag_pilot", 180),
    ("AP_HDG_VAR_SET", -10, AP + "heading_dial_deg_mag_pilot", 350),
    ("HEADING_BUG_SET", 370, AP + "heading_dial_deg_mag_pilot", 10),
    ("AP_ALT_VAR_SET_ENGLISH", 12000, AP + "altitude_dial_ft", 12000),
    ("AP_VS_VAR_SET_ENGLISH", -750, AP + "vvi_dial_fpm", -750),
    ("AP_SPD_VAR_SET", 180, AP + "airspeed_dial_kts", 180),
    ("AP_SPD_VAR_SET_EX1", 18025, AP + "airspeed_dial_kts", 180.25),
    ("FLAPS_SET", 8191.5, FLAPS, .5),
    ("AXIS_FLAPS_SET", -16383, FLAPS, 0),
    ("GEAR_SET", 0, "sim/cockpit2/controls/gear_handle_down", 0),
    ("PARKING_BRAKE_SET", 1, "sim/cockpit2/controls/parking_brake_ratio", 1),
    ("AP_NAV_SELECT_SET", 2, HSI, 1),
    ("SPOILERS_ARM_ON", 0, "sim/cockpit2/controls/speedbrake_ratio", -.5),
    ("THROTTLE_FULL", 0, THROTTLE + "_all", 1),
])
def test_event_datarefs(xp, event, value, path, expected):
    peer, source = xp
    assert source._send(event.encode(), value)
    eventually(lambda: any(p == path and v == pytest.approx(expected) for p, v in peer.writes))


@pytest.mark.parametrize("event,command", [
    ("AUTOPILOT_ON", "sim/autopilot/servos_on"),
    ("AUTOPILOT_OFF", "sim/autopilot/servos_off_any"),
    ("GEAR_UP", "sim/flight_controls/landing_gear_up"),
    ("GEAR_DOWN", "sim/flight_controls/landing_gear_down"),
    ("ENGINE_AUTO_START", "sim/operation/auto_start"),
    ("ENGINE_AUTO_SHUTDOWN", "sim/starters/shut_down"),
    ("YAW_DAMPER_ON", "sim/systems/yaw_damper_on"),
    ("AUTO_THROTTLE_TO_GA", "sim/autopilot/take_off_go_around"),
])
def test_event_commands(xp, event, command):
    peer, source = xp
    assert source._send(event.encode(), 0)
    eventually(lambda: peer.commands == [command])


def test_real_director_heading_altitude_vs_speed_and_engagement(xp):
    peer, source = xp
    director = FlightDirector(source)
    assert director.autopilot(True) and director.flight_director(True)
    assert director.heading(270) and director.altitude(15000)
    assert director.vertical_speed(-900) and director.vs_hold(True)
    assert director.speed(220.25) and director.airspeed_hold(True)
    assert director.autothrottle_arm()
    assert director.nav_source(1) and director.nav(True) and director.approach(True)
    assert director.flaps(.5) and director.gear(True) and director.parking_brake(False)
    eventually(lambda: "sim/flight_controls/landing_gear_down" in peer.commands)
    assert "sim/autopilot/servos_on" in peer.commands
    assert "sim/autopilot/heading" in peer.commands
    assert "sim/autopilot/vertical_speed_pre_sel" in peer.commands
    assert "sim/autopilot/NAV" in peer.commands and "sim/autopilot/approach" in peer.commands
    assert (AP + "flight_director_command_bars_pilot", 1) in peer.writes
    assert (AP + "heading_dial_deg_mag_pilot", 270) in peer.writes
    assert (AP + "altitude_dial_ft", 15000) in peer.writes
    assert (AP + "vvi_dial_fpm", -900) in peer.writes
    assert (AP + "airspeed_dial_kts", 220.25) in peer.writes
    assert (AP + "autothrottle_enabled", 1) in peer.writes
    assert (AP + "autothrottle_enabled", 0) not in peer.writes
    assert not any("override" in path for path, _ in peer.writes)


@pytest.mark.parametrize("root,status,command", [
    ("AP_HDG_HOLD", "heading_status", "heading"),
    ("AP_NAV1_HOLD", "nav_status", "NAV"),
    ("AP_LOC_HOLD", "nav_status", "NAV"),
    ("AP_APR_HOLD", "approach_status", "approach"),
    ("AP_ALT_HOLD", "altitude_hold_status", "altitude_hold"),
    ("AP_VS", "vvi_status", "vertical_speed_pre_sel"),
    ("FLIGHT_LEVEL_CHANGE", "speed_status", "level_change"),
])
def test_mode_requests_are_idempotent_before_and_after_feedback(xp, root, status, command):
    peer, source = xp
    assert source._send(root + "_ON", 0)
    assert source._send(root + "_ON", 0)
    eventually(lambda: len(peer.commands) == 1)
    assert peer.commands == ["sim/autopilot/" + command]
    peer.values[AP + status] = 2
    eventually(lambda: source._values["control:" + status] == 2)
    assert source._send(root + "_ON", 0)
    assert source._send(root + "_OFF", 0)
    assert source._send(root + "_OFF", 0)
    eventually(lambda: len(peer.commands) == 2)
    peer.values[AP + status] = 0
    eventually(lambda: source._values["control:" + status] == 0)
    assert source._send(root + "_OFF", 0)
    assert len(peer.commands) == 2


def test_switches_arm_mach_throttle_and_unknown_inputs(xp):
    peer, source = xp
    assert source._send("AUTO_THROTTLE_ARM", 0)
    assert source._send("AP_AIRSPEED_ON", 0)
    assert source._send("AUTO_THROTTLE_ARM", 0)
    assert source._send("AUTO_THROTTLE_DISCONNECT", 0)
    assert source.set_variable("AUTOPILOT MACH HOLD VAR", .78)
    assert source._send("AP_MACH_ON", 0)
    assert source.set_variable("GENERAL ENG THROTTLE LEVER POSITION:2", 65)
    assert source._send("MASTER_BATTERY_SET", 1)
    assert source._send("AVIONICS_MASTER_SET", 1)
    eventually(lambda: ("sim/cockpit2/switches/avionics_power_on", 1) in peer.writes)
    at = [value for path, value in peer.writes if path == AP + "autothrottle_enabled"]
    assert at == [0, 1, -1, 1]
    assert (AP + "airspeed_dial_kts_mach", pytest.approx(.78)) in peer.writes
    assert (THROTTLE + "[1]", pytest.approx(.65)) in peer.writes
    for i in range(8):
        assert (f"sim/cockpit2/electrical/battery_on[{i}]", 1) in peer.writes
    before = len(peer.writes), len(peer.commands)
    for name in (b"ELEVATOR_SET", b"AILERON_SET", b"AXIS_RUDDER_SET", b"STEERING_SET",
                 b"HELICOPTER_COLLECTIVE_SET", b"", b"\xff", b"UNKNOWN"):
        assert not source._send(name, 100)
    assert not source._send("FLAPS_SET", -1)
    assert not source._send("AP_NAV_SELECT_SET", 3)
    assert not source._send("AP_ALT_VAR_SET_ENGLISH", float("inf"))
    assert not source.set_variable("AUTOPILOT ALTITUDE LOCK VAR", None)
    assert not source.set_variable("GENERAL ENG THROTTLE LEVER POSITION:17", 10)
    assert not source.set_variable("AUTOPILOT ALTITUDE LOCK VAR", 5000, unit="radians")
    assert before == (len(peer.writes), len(peer.commands))
    with source._lock:
        source._values.pop("control:nav_status", None)
    assert not source._send("AP_NAV1_HOLD_ON", 0)
    source.close()
    assert not source._send("AUTOPILOT_ON", 0)
    assert not source.set_variable("AUTOPILOT ALTITUDE LOCK VAR", 5000)
    assert not source._controls.pending


def test_altitude_capture_does_not_let_stale_vs_pending_cancel_alt_hold(xp):
    peer, source = xp
    assert source._send("AP_VS_ON", 0)
    peer.values[AP + "altitude_hold_status"] = 2
    eventually(lambda: source._values["control:altitude_hold_status"] == 2)
    assert source._send("AP_ALT_HOLD_ON", 0)
    assert source._send("AP_VS_OFF", 0)
    eventually(lambda: bool(peer.commands))
    assert peer.commands == ["sim/autopilot/vertical_speed_pre_sel"]


def test_flap_index_and_ap_state_require_real_values(xp):
    peer, source = xp
    peer.values[EXTRA["flap_detents"]] = 4
    peer.values[EXTRA["autopilot_mode"]] = 1
    eventually(lambda: source._values.get("flap_detents") == 4)
    assert source.set_variable("FLAPS HANDLE INDEX", 2)
    eventually(lambda: (FLAPS, pytest.approx(2 / 3)) in peer.writes)
    assert not source.read().autopilot  # FD alone does not mean servos engaged.
    peer.values[READS["control:ap_on"]] = 1
    eventually(lambda: source.read().autopilot)


def test_capabilities_are_ap_only_not_axis_control():
    state = AircraftState(source="xplane", connected=True, on_ground=False)
    assert capabilities("xplane").autofly and not capabilities("xplane").flight_axes
    assert not autofly_limit(state)
    state.category = "Helicopter"
    assert "rotorcraft" in autofly_limit(state)
    assert capabilities("simconnect").flight_axes


@pytest.mark.parametrize("phase,sub", [("taxi_out", "taxi"), ("takeoff", "roll"),
                                      ("landing", "flare"), ("approach", "go_around")])
def test_raw_axis_phases_hand_back_before_executing(phase, sub):
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )

    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState(phase=AutonomousPhase(phase), sub=sub)
    fo._is_helicopter = False
    fo._enabled = True
    fo._handoff = Mock()
    fo._emit = Mock()
    aircraft = AircraftState(source="xplane", connected=True, on_ground=False)
    fo._execute_phase(aircraft, 0)
    assert not fo.enabled
    fo._handoff.assert_called_once()
    assert fo._emit.call_args.args[-1] == "log.autofly_manual"


def test_airborne_fo_tick_engages_ap_and_go_around_refuses_axes(xp):
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )

    peer, source = xp
    aircraft = AircraftState(source="xplane", connected=True, on_ground=False)
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState(phase=AutonomousPhase.CRUISE)
    fo.get_state = lambda: aircraft
    fo._enabled = True
    fo._is_helicopter = False
    fo._heli = None
    fo.ACTION_INTERVAL = 1
    fo.sim, fo._ap_mode = source, "managed"
    fo._dir = FlightDirector(source)
    for name in ("_manage_comms", "_drive_checklist", "_monitor_weather", "_monitor_emergency",
                 "_execute_phase", "_handoff", "_emit"):
        setattr(fo, name, Mock())
    fo.tick()
    fo._execute_phase.assert_called_once()
    eventually(lambda: "sim/autopilot/servos_on" in peer.commands)
    assert fo.enabled and fo.state.ap_master
    fo._start_go_around(aircraft)
    assert not fo.enabled
    fo._handoff.assert_called_once()
