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
    ("AP_NAV_SELECT_SET", 2, HSI, 1),
    ("SPOILERS_ARM_ON", 0, "sim/cockpit2/controls/speedbrake_ratio", -.5),
    ("THROTTLE_FULL", 0, THROTTLE + "_all", 1),
    # Trim events keep their own documented ranges: the named elevator
    # event is unipolar 0..16383, the AXIS_ form bipolar, and aileron
    # and rudder trims are percent -- none of them the axes' 16K scale.
    ("AXIS_ELEV_TRIM_SET", 8191, "sim/cockpit2/controls/elevator_trim", 8191 / 16383),
    ("AXIS_ELEV_TRIM_SET", -16383, "sim/cockpit2/controls/elevator_trim", -1),
    ("ELEVATOR_TRIM_SET", 8191, "sim/cockpit2/controls/elevator_trim", 8191 / 8191.5 - 1),
    ("AILERON_TRIM_SET", 50, "sim/cockpit2/controls/aileron_trim", .5),
    ("RUDDER_TRIM_SET", -50, "sim/cockpit2/controls/rudder_trim", -.5),
])
def test_event_datarefs(xp, event, value, path, expected):
    peer, source = xp
    assert source._send(event.encode(), value)
    eventually(lambda: any(p == path and v == pytest.approx(expected) for p, v in peer.writes))


@pytest.mark.parametrize("event,command", [
    ("AUTOPILOT_ON", "sim/autopilot/servos_on"),
    ("AUTOPILOT_OFF", "sim/autopilot/fdir_servos_down_any"),
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


def test_parking_brake_uses_the_xplane_122_commands(xp):
    """A hydraulic parking brake is pressure plus a valve, not a legacy
    ratio write: the documented commands sequence the two correctly."""
    peer, source = xp
    source.version = 120200
    assert source._send(b"PARKING_BRAKE_SET", 1)
    assert source._send(b"PARKING_BRAKE_SET", 0)
    eventually(lambda: peer.commands == [
        "sim/flight_controls/park_brake_set",
        "sim/flight_controls/park_brake_release"])


def test_parking_brake_legacy_write_before_xplane_122(xp):
    peer, source = xp
    source.version = 120100
    assert source._send(b"PARKING_BRAKE_SET", 1)
    eventually(lambda: (
        "sim/cockpit2/controls/parking_brake_ratio", 1.0) in peer.writes)


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
    assert "sim/autopilot/vertical_speed" in peer.commands
    assert "sim/autopilot/NAV" in peer.commands and "sim/autopilot/approach" in peer.commands
    assert (AP + "flight_director_command_bars_pilot", 1) in peer.writes
    assert (AP + "heading_dial_deg_mag_pilot", 270) in peer.writes
    assert (AP + "altitude_dial_ft", 15000) in peer.writes
    assert (AP + "vvi_dial_fpm", -900) in peer.writes
    assert (AP + "airspeed_dial_kts", 220.25) in peer.writes
    assert (AP + "autothrottle_enabled", 1) in peer.writes
    assert (AP + "autothrottle_enabled", 0) not in peer.writes
    # Engaging the AP must not grab the pilot's controls: none of the
    # stick, steering, brake or engine-lever overrides is an AP's to
    # latch. The flap write in this batch legitimately owns its
    # quadrant axis, the same way a lever write does.
    held = ("override_joystick", "override_wheel_steer",
            "override_toe_brakes", "override_throttles",
            "override_prop_pitch", "override_mixture")
    assert not any(any(flag in path for flag in held)
                   for path, _ in peer.writes)


@pytest.mark.parametrize("root,status,command", [
    ("AP_HDG_HOLD", "heading_status", "heading"),
    ("AP_NAV1_HOLD", "nav_status", "NAV"),
    ("AP_LOC_HOLD", "nav_status", "NAV"),
    ("AP_APR_HOLD", "approach_status", "approach"),
    ("AP_ALT_HOLD", "altitude_hold_status", "altitude_hold"),
    ("AP_VS", "vvi_status", "vertical_speed"),
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
    for name in (b"", b"\xff", b"UNKNOWN"):
        assert not source._send(name, 100)
    # The hand-flown channels now write datarefs rather than refuse:
    # the yoke ratios, the steer command and the brake ratios.
    for name, path in ((b"ELEVATOR_SET", "yoke_pitch_ratio"),
                       (b"AILERON_SET", "yoke_roll_ratio"),
                       (b"AXIS_RUDDER_SET", "yoke_heading_ratio"),
                       (b"STEERING_SET", "tire_steer_command_deg[0]"),
                       (b"AXIS_LEFT_BRAKE_SET", "left_brake_ratio"),
                       (b"AXIS_COLLECTIVE_SET", "prop_ratio_all")):
        assert source._send(name, 100)
        eventually(lambda p=path: any(p in w for w, _v in peer.writes))
    before = len(peer.writes), len(peer.commands)
    assert not source._send("FLAPS_SET", -1)
    assert not source._send("AP_NAV_SELECT_SET", 3)
    assert not source._send("AP_ALT_VAR_SET_ENGLISH", float("inf"))
    assert not source._send("STEERING_SET", -16385)
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
    assert peer.commands == ["sim/autopilot/vertical_speed"]


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


def test_capabilities_cover_full_gate_to_gate_control():
    state = AircraftState(source="xplane", connected=True, on_ground=False)
    assert capabilities("xplane").autofly and capabilities("xplane").flight_axes
    assert not autofly_limit(state)
    state.category = "Helicopter"
    assert not autofly_limit(state)
    assert capabilities("simconnect").flight_axes
    assert autofly_limit(AircraftState(source="manual", connected=True))


@pytest.mark.parametrize("phase,sub", [("taxi_out", "taxi"), ("takeoff", "roll"),
                                      ("landing", "flare"), ("approach", "go_around")])
def test_no_axis_sources_hand_back_before_executing(phase, sub):
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
    aircraft = AircraftState(source="manual", connected=True, on_ground=False)
    fo._execute_phase(aircraft, 0)
    assert not fo.enabled
    fo._handoff.assert_called_once()
    assert fo._emit.call_args.args[-1] == "log.autofly_manual"


@pytest.mark.parametrize("event,value,axis,override", [
    ("AILERON_SET", 8191, "yoke_roll_ratio", "override_joystick_roll"),
    ("AILERON_SET", -16383, "yoke_roll_ratio", "override_joystick_roll"),
    ("ELEVATOR_SET", 16384, "yoke_pitch_ratio", "override_joystick_pitch"),
    ("AXIS_RUDDER_SET", -8192, "yoke_heading_ratio", "override_joystick_heading"),
    ("AXIS_CYCLIC_LATERAL_SET", 4096, "yoke_roll_ratio", "override_joystick_roll"),
    ("AXIS_CYCLIC_LONGITUDINAL_SET", -4096, "yoke_pitch_ratio", "override_joystick_pitch"),
    ("ROTOR_AXIS_TAIL_ROTOR_SET", 2048, "yoke_heading_ratio", "override_joystick_heading"),
])
def test_flight_axes_drive_yoke_ratios_and_overrides(xp, event, value, axis, override):
    peer, source = xp
    assert source._send(event.encode(), value)
    expected = max(-1.0, min(1.0, value / 16383.0))
    eventually(lambda: (f"sim/joystick/{axis}", pytest.approx(expected)) in peer.writes)
    assert (f"sim/operation/override/{override}", 1) in peer.writes


def test_director_trims_drive_the_trim_actuators(xp):
    peer, source = xp
    director = FlightDirector(source)
    assert director.trim_elevator(.5)
    assert director.trim_aileron(-.25)
    assert director.trim_rudder(1.0)
    eventually(lambda: ("sim/cockpit2/controls/elevator_trim",
                        pytest.approx(.5, abs=1e-3)) in peer.writes)
    eventually(lambda: ("sim/cockpit2/controls/rudder_trim",
                        pytest.approx(1.0, abs=1e-3)) in peer.writes)
    assert ("sim/cockpit2/controls/aileron_trim",
            pytest.approx(-.25, abs=1e-3)) in peer.writes
    # Trims are the cockpit's own actuators: no joystick override latches.
    assert not any("override_joystick" in path for path, _v in peer.writes)


def test_center_controls_releases_every_override(xp):
    peer, source = xp
    for event in ("AILERON_SET", "ELEVATOR_SET", "AXIS_RUDDER_SET",
                  "STEERING_SET", "AXIS_LEFT_BRAKE_SET"):
        assert source._send(event.encode(), 100)
    assert source._send(b"CENTER_AILER_RUDDER", 0)

    def released(flag):
        prefix = "override_joystick_" if flag in ("pitch", "roll", "heading") else "override_"
        return (f"sim/operation/override/{prefix}{flag}", 0) in peer.writes

    for flag in ("pitch", "roll", "heading", "wheel_steer", "toe_brakes"):
        eventually(lambda f=flag: released(f))


def test_engine_levers_latch_and_release_their_overrides(xp):
    # The takeoff's "power to idle at liftoff" was this: the levers were
    # written while the pilot's throttle quadrant still owned the
    # actuator, and X-Plane re-read the hardware the frame a write
    # stopped. The levers latch the same override flags the stick does.
    peer, source = xp
    override = "sim/operation/override/"
    assert source._send(b"THROTTLE_SET", 8192)
    assert source.set_variable("GENERAL ENG MIXTURE LEVER POSITION:1", 90)
    assert source.set_variable("GENERAL ENG PROPELLER LEVER POSITION:1", 90)
    assert source._send(b"FLAPS_SET", 8192)
    assert source._send(b"SPOILERS_SET", 8192)
    assert source.set_variable("RECIP ENG COWL FLAP POSITION:1", 50)
    latched = ("override_throttles", "override_mixture",
               "override_prop_pitch", "override_flap_handle",
               "override_speedbrakes", "override_cowl_flaps")
    eventually(lambda: all((override + flag, 1) in peer.writes
                           for flag in latched))
    # A centred stick does not hand the quadrant back: a stop or a
    # hold is still the FO's flight, and releasing the levers there is
    # how the parked aeroplane's throttle ended up on the pilot's idle
    # hardware between taxi probes.
    assert source._send(b"CENTER_AILER_RUDDER", 0)
    assert not any((override + flag, 0) in peer.writes for flag in latched)
    # Handing the airplane back releases the levers like the axes --
    # a latched quadrant is as dead as a latched stick.
    assert source._send(b"RELEASE_OVERRIDES", 0)
    eventually(lambda: all((override + flag, 0) in peer.writes
                           for flag in latched))


def test_lever_writes_reach_the_engines_use_path(xp):
    # The parked-Baron failure: with a lever override latched X-Plane's
    # engine reads the family's *_use dataref, not the lever the
    # cockpit draws. Every lever write lands on both.
    peer, source = xp
    # The Baron's governor range, the scale a prop lever is mapped
    # through: forward is fine pitch, the minimum.
    for i in range(16):
        peer.values[f"sim/aircraft/prop/acf_min_pitch[{i}]"] = 8.0
        peer.values[f"sim/aircraft/prop/acf_max_pitch[{i}]"] = 45.0
    eventually(lambda: source._values.get("acf_max_pitch1") == 45.0)
    assert source.set_variable("GENERAL ENG THROTTLE LEVER POSITION:1", 75)
    assert source.set_variable("GENERAL ENG THROTTLE LEVER POSITION:2", 75)
    assert source.set_variable("GENERAL ENG MIXTURE LEVER POSITION:1", 90)
    assert source.set_variable("GENERAL ENG PROPELLER LEVER POSITION:1", 100)
    eng = "sim/flightmodel/engine/"

    def wrote(path, expected):
        return any(p == path and v == pytest.approx(expected)
                   for p, v in peer.writes)

    eventually(lambda: wrote(eng + "ENGN_thro_use[0]", .75)
               and wrote(eng + "ENGN_thro_use[1]", .75)
               and wrote(eng + "ENGN_mixt[0]", .9)
               # Lever forward seeds the governor's blade angle on
               # the coarse side of fine -- the loop walks it to the
               # lever's RPM from there, never the bare minimum stop.
               and wrote(eng + "POINT_pitch_deg_use[0]",
                         8 + 37 * 0.15))
    # The levers still move -- the cockpit's handle and the engine's
    # ear are written together.
    act = "sim/cockpit2/engine/actuators/"
    assert wrote(act + "throttle_ratio[0]", .75)
    assert wrote(act + "mixture_ratio[0]", .9)
    assert wrote(act + "prop_ratio[0]", 1.0)
    # The event forms and the collective reach the same use paths:
    # THROTTLE_SET fans out to every engine, and collective up is more
    # blade, the RPM lever's other direction.
    assert source._send(b"THROTTLE_SET", 8192)
    assert source._send(b"AXIS_COLLECTIVE_SET", 8192)
    eventually(lambda: wrote(eng + "ENGN_thro_use[7]", 8192 / 16383)
               and wrote(eng + "POINT_pitch_deg_use[0]",
                         8 + (8192 + 16383.0) / 32766.0 * 37))
    # Governor range absent: the fallback is the usual 8..45 degrees.
    for i in range(16):
        peer.missing.add(f"sim/aircraft/prop/acf_min_pitch[{i}]")
        peer.missing.add(f"sim/aircraft/prop/acf_max_pitch[{i}]")
        source._values.pop(f"acf_min_pitch{i}", None)
        source._values.pop(f"acf_max_pitch{i}", None)
    # Re-seeded from scratch, the half lever's seed is the fallback
    # range's same coarse-side angle.
    source._controls._prop_deg.clear()
    assert source.set_variable("GENERAL ENG PROPELLER LEVER POSITION:1", 50)
    eventually(lambda: wrote(eng + "POINT_pitch_deg_use[0]",
                             8 + 37 * (0.15 + 0.55 * 0.5)))


def test_the_condition_lever_rides_the_mixture_channel(xp):
    # X-Plane has no condition-lever dataref: on its turboprops the
    # model wires that lever to the mixture channel, so the SimConnect
    # detents go out through the mixture write -- override flag, use
    # path and all.
    peer, source = xp
    act = "sim/cockpit2/engine/actuators/"
    eng = "sim/flightmodel/engine/"

    def wrote(path, expected):
        return any(p == path and v == pytest.approx(expected)
                   for p, v in peer.writes)

    # High idle, engine one: the detent's top stop is the full lever.
    assert source.set_variable("TURB ENG CONDITION LEVER POSITION:1", 2)
    eventually(lambda: wrote(act + "mixture_ratio[0]", 1.0)
               and wrote(eng + "ENGN_mixt[0]", 1.0))
    # Low idle, engine two: half a lever. A stop off the scale is refused.
    assert source.set_variable("TURB ENG CONDITION LEVER POSITION:2", 1)
    eventually(lambda: wrote(act + "mixture_ratio[1]", .5))
    assert not source.set_variable(
        "TURB ENG CONDITION LEVER POSITION:1", 3)


def test_the_software_governor_holds_the_levers_rpm(xp):
    # The override carries no governor: the lever asks an RPM and the
    # loop walks the blade angle to it. The parked Baron's 3000-plus
    # RPM was the fixed angle left at the fine stop under power.
    peer, source = xp
    for i in range(16):
        peer.values[f"sim/aircraft/prop/acf_min_pitch[{i}]"] = 8.0
        peer.values[f"sim/aircraft/prop/acf_max_pitch[{i}]"] = 45.0
        peer.values[f"sim/aircraft/prop/acf_des_rpm_prp[{i}]"] = 2700.0
        peer.values[
            f"sim/cockpit2/engine/indicators/prop_speed_rpm[{i}]"] = 800.0
    eventually(lambda: source._values.get("acf_des_rpm0") == 2700.0
               and source._values.get("prop_rpm0") == 800.0)
    assert source.set_variable(
        "GENERAL ENG PROPELLER LEVER POSITION:1", 100)
    eng = "sim/flightmodel/engine/"

    def last():
        return [v for p, v in peer.writes
                if p == eng + "POINT_pitch_deg_use[0]"][-1]

    eventually(lambda: any(p == eng + "POINT_pitch_deg_use[0]"
                           for p, _v in peer.writes))
    seed = source._controls._prop_deg[0]
    # 800 RPM against a 2700 ask: the angle fines up, the only way a
    # governor adds RPM.
    for _ in range(40):
        source._controls.govern_props()
    assert source._controls._prop_deg[0] < seed
    assert source._controls._prop_deg[0] == pytest.approx(8.0)
    # 3000 on the clock -- the overspeed the fixed angle drew -- and
    # it coarsens back past the seed instead.
    peer.values[
        "sim/cockpit2/engine/indicators/prop_speed_rpm[0]"] = 3000.0
    eventually(lambda: source._values.get("prop_rpm0") == 3000.0)
    for _ in range(60):
        source._controls.govern_props()
    assert source._controls._prop_deg[0] > seed
    # Lever back is the feather ask: the angle winds the whole way to
    # the coarse stop and stays there.
    assert source.set_variable(
        "GENERAL ENG PROPELLER LEVER POSITION:1", 0)
    for _ in range(120):
        source._controls.govern_props()
    assert source._controls._prop_deg[0] == pytest.approx(45.0)
    # The handoff's release stands the governor down with the override.
    source._send(b"RELEASE_OVERRIDES", 0)
    assert not source._controls._prop_lever
    assert not source._controls._prop_deg


def test_steering_scales_by_nosewheel_authority(xp):
    peer, source = xp
    peer.values["sim/aircraft/gear/acf_nw_steerdeg1"] = 40.0
    eventually(lambda: source._values.get("steer_deg") == 40.0)
    assert source._send(b"STEERING_SET", 16383)
    eventually(lambda: any(p.endswith("tire_steer_command_deg[0]")
                           and v == pytest.approx(40.0) for p, v in peer.writes))
    assert ("sim/operation/override/override_wheel_steer", 1) in peer.writes
    assert not source._send(b"STEERING_SET", -16384 - 1)


def test_toe_brakes_and_pushback_and_reverser(xp):
    peer, source = xp
    assert source._send(b"AXIS_LEFT_BRAKE_SET", 16383)
    assert source._send(b"AXIS_RIGHT_BRAKE_SET", -16383)
    assert source._send(b"BRAKES", 0)
    assert source._send(b"THROTTLE_REVERSE_THRUST_TOGGLE", 0)
    assert source._send(b"TOGGLE_PUSHBACK", 0)
    assert source._send(b"PUSHBACK_STOP", 0)
    assert source._send(b"CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE", 0)
    eventually(lambda: "sim/engines/thrust_reverse_toggle" in peer.commands
               and "sim/ground_ops/pushback_straight" in peer.commands
               and "sim/ground_ops/pushback_stop" in peer.commands
               and "sim/systems/seatbelt_sign_toggle" in peer.commands)
    writes = dict()
    for path, value in peer.writes:
        writes[path] = value
    assert writes["sim/cockpit2/controls/left_brake_ratio"] == pytest.approx(1.0)
    assert writes["sim/cockpit2/controls/right_brake_ratio"] == pytest.approx(1.0)
    assert writes["sim/operation/override/override_toe_brakes"] == 1


def test_starters_and_bank_mode_and_rotor_channels(xp):
    peer, source = xp
    assert source._send(b"STARTER_SET", 1)
    assert source._send(b"STARTER2_SET", 1)
    assert source._send(b"STARTER_SET", 0)
    assert source._send(b"AP_MAX_BANK_ANGLE_SET", 20)
    assert source._send(b"AXIS_COLLECTIVE_SET", 16384)
    assert source._send(b"AXIS_HELICOPTER_THROTTLE_SET", 0)
    assert source._send(b"HELICOPTER_THROTTLE_SET", 8192)
    assert source._send(b"HELICOPTER_THROTTLE_FULL", 0)
    assert source._send(b"ROTOR_GOV_SWITCH_SET", 1)
    assert source._send(b"ROTOR_BRAKE_ON", 0)
    eventually(lambda: "sim/engines/engage_starters" in peer.commands
               and "sim/starters/engage_starter_2" in peer.commands
               and any(path == "sim/cockpit2/switches/rotor_brake"
                       for path, _ in peer.writes))
    act = "sim/cockpit2/engine/actuators/"
    assert (act + "ignition_key[1]", 4) in peer.writes
    assert (act + "ignition_key[7]", 4) in peer.writes
    writes = dict()
    for path, value in peer.writes:
        writes[path] = value
    assert writes[act + "ignition_key[0]"] == 3
    assert writes[act + "ignition_key[1]"] == 3
    assert writes["sim/cockpit2/autopilot/bank_angle_mode"] == 4  # 20 deg step
    assert writes[act + "prop_ratio_all"] == pytest.approx(1.0)
    assert writes[act + "throttle_ratio_all"] == pytest.approx(1.0)
    assert writes[act + "governor_on[0]"] == 1
    assert writes["sim/cockpit2/switches/rotor_brake"] == 1


def test_engine_and_ice_variables(xp):
    peer, source = xp
    assert source.set_variable("GENERAL ENG MIXTURE LEVER POSITION:1", 90)
    assert source.set_variable("GENERAL ENG PROPELLER LEVER POSITION:2", 80)
    assert source.set_variable("RECIP ENG COWL FLAP POSITION:1", 50)
    assert source.set_variable("GENERAL ENG ANTI ICE POSITION:1", 1.0)
    assert source.set_variable("PROP DEICE SWITCH:1", 1.0)
    assert source.set_variable("STRUCTURAL DEICE SWITCH", 1.0)
    assert source.set_variable("WINDSHIELD DEICE SWITCH:1", 1.0)
    assert source.set_variable("TRANSPONDER STATE:1", 4.0)
    eventually(lambda: any(
        path == "sim/cockpit2/radios/actuators/transponder_mode"
        for path, _ in peer.writes))
    writes = dict()
    for path, value in peer.writes:
        writes[path] = value
    act = "sim/cockpit2/engine/actuators/"
    ice = "sim/cockpit2/ice/"
    assert writes[act + "mixture_ratio[0]"] == pytest.approx(0.9)
    assert writes[act + "prop_ratio[1]"] == pytest.approx(0.8)
    assert writes[act + "cowl_flap_ratio[0]"] == pytest.approx(0.5)
    assert writes[act + "carb_heat_ratio[0]"] == pytest.approx(1.0)
    assert writes[ice + "ice_inlet_heat_on_per_engine[0]"] == 1
    assert writes[ice + "cowling_thermal_anti_ice_per_engine[0]"] == 1
    assert writes[ice + "ice_prop_heat_on_per_engine[0]"] == 1
    assert writes[ice + "ice_prop_heat_on"] == 1
    assert writes[ice + "ice_surface_boot_on"] == 1
    assert writes[ice + "ice_surface_tks_on"] == 1
    assert writes[ice + "ice_window_heat_on"] == 1
    assert writes["sim/cockpit2/radios/actuators/transponder_mode"] == 3


def test_airborne_fo_tick_runs_phase_logic_with_axes(xp):
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
    assert fo.enabled


def test_close_hands_the_stick_back_before_the_socket_goes():
    peer = FakeXPlane()
    peer.values[READS["control:autothrottle"]] = -1
    source = XPlaneSource(port=peer.port, discovery_s=0, timeout_s=.2)
    try:
        assert source.connect()
        eventually(lambda: all(key in source._values for key in READS))
        assert source._send(b"ELEVATOR_SET", 4000)
        eventually(lambda: (
            "sim/operation/override/override_joystick_pitch", 1)
            in peer.writes)
        source.close()
        # Released on the way out: a latched override is a dead stick in
        # the pilot's hands for the rest of the X-Plane session.
        eventually(lambda: peer.writes[-1:] and (
            "sim/operation/override/override_joystick_pitch", 0)
            in peer.writes)
    finally:
        source.close()
        peer.close()
