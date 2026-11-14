"""Exercise the actual UDP wire format without a simulator or plugin."""

from __future__ import annotations

import socket
import struct
import threading
import time
from unittest.mock import Mock

import pytest

from meridianatc.config import Config, SimConfig
from meridianatc.sim import source as sources
from meridianatc.sim.state import AircraftState
from meridianatc.sim.xplane import DATAREFS, EXTRA, FT, KT, LB, RADIOS, XPlaneSource


def eventually(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        threading.Event().wait(.01)
    assert predicate()


class FakeXPlane:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(.01)
        self.port = self.socket.getsockname()[1]
        self.values = {}
        self.subscriptions = {}
        self.writes = []
        self.commands = []
        self.unsubscribed = set()
        # Paths X-Plane does not know answer nothing, so a read that falls
        # back on a second dataref can be exercised by listing it here.
        self.missing = set()
        self.respond = True
        self.stop = threading.Event()
        self.client = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def string(self, path, value):
        for i, byte in enumerate(value.encode() + b"\0"):
            self.values[f"{path}[{i}]"] = byte

    def run(self):
        last = 0
        while not self.stop.is_set():
            try:
                packet, self.client = self.socket.recvfrom(2048)
                if packet[:5] == b"RREF\0":
                    freq, index, raw = struct.unpack("<ii400s", packet[5:])
                    path = raw.split(b"\0", 1)[0].decode()
                    if freq:
                        self.subscriptions[index] = path
                    else:
                        self.subscriptions.pop(index, None)
                        self.unsubscribed.add(path)
                elif packet[:5] == b"DREF\0":
                    value, raw = struct.unpack("<f500s", packet[5:])
                    self.writes.append((raw.split(b"\0", 1)[0].decode(), value))
                elif packet[:5] == b"CMND\0":
                    self.commands.append(packet[5:].rstrip(b"\0").decode())
            except TimeoutError:
                pass
            if self.respond and self.client and time.monotonic() - last > .02:
                pairs = [(i, p) for i, p in self.subscriptions.items()
                         if p not in self.missing]
                for start in range(0, len(pairs), 100):
                    payload = b"".join(struct.pack("<if", i, self.values.get(path, 0))
                                       for i, path in pairs[start:start + 100])
                    self.socket.sendto(b"RREF," + payload, self.client)
                last = time.monotonic()

    def close(self):
        self.stop.set()
        self.thread.join(1)
        self.socket.close()


@pytest.fixture
def xp():
    server = FakeXPlane()
    link = XPlaneSource(port=server.port, discovery_s=0, timeout_s=.2, disconnect_s=.15)
    try:
        yield server, link
    finally:
        link.close()
        server.close()


def test_connect_converts_state_and_strings(xp):
    server, link = xp
    raw = {
        "latitude": 51.5, "longitude": -.1, "altitude_ft": 1000,
        "altitude_agl_ft": 100, "indicated_altitude_ft": 3400,
        "heading_true": 3, "heading_magnetic": 358, "track_true": 2,
        "ground_speed_kt": 50, "true_airspeed_kt": 60,
        "indicated_airspeed_kt": 110, "vertical_speed_fpm": -700,
        "on_ground": 0, "parking_brake": 1, "gear_down": 1,
        "engine_count": 2, "beacon_light": 1, "seatbelt_sign": 1,
        "com1_active": 123805, "com2_standby": 132010,
        "nav1_freq": 11030, "paused": 1, "fuel_total_lb": 100,
        "ambient_wind_velocity": 10, "ambient_visibility_m": 5,
        "sea_level_pressure_hpa": 101325, "ambient_temperature_c": 12,
    }
    server.values.update({DATAREFS[key][0]: value for key, value in raw.items()})
    server.values.update({EXTRA[key]: value for key, value in {
        "xp_engine_type": 9, "running0": 1, "running1": 1,
        "squawk": 42, "flaps": .5, "flap_detents": 5,
        "transmit": 6, "nav_type": 4,
    }.items()})
    server.string("sim/aircraft/view/acf_ICAO", "BE20")
    server.string("sim/aircraft/view/acf_tailnum", "N123AB")
    server.string("sim/aircraft/view/acf_ui_name", "King Air – test")
    assert link.connect()
    eventually(lambda: link.read().title == "King Air – test")
    state = link.read()
    assert state.connected and state.source == "xplane"
    assert state.latitude == 51.5
    assert state.longitude == pytest.approx(-.1)
    assert state.altitude_ft == pytest.approx(1000 * FT)
    assert state.altitude_agl_ft == pytest.approx(100 * FT)
    assert state.indicated_altitude_ft == 3400
    assert (state.heading_true, state.heading_magnetic, state.track_true) == (3, 358, 2)
    assert state.ground_speed_kt == pytest.approx(50 * KT)
    assert state.true_airspeed_kt == pytest.approx(60 * KT)
    assert state.indicated_airspeed_kt == 110
    assert state.vertical_speed_fpm == -700
    assert not state.on_ground and state.parking_brake and state.gear_down
    assert state.com1_active == pytest.approx(123.805)
    assert state.com2_standby == pytest.approx(132.010)
    assert state.nav1_freq == pytest.approx(110.30)
    assert state.callsign == state.tail_number == "N123AB" and state.type_code == "BE20"
    assert state.engines_running and state.engine_type == 5 and state.engine_count == 2
    assert state.eng2_combustion and state.eng3_combustion is None
    assert state.transponder_code == "0042" and state.flaps_index == 2
    assert state.beacon_light and state.seatbelt_sign and state.paused
    assert state.fuel_total_lb == pytest.approx(100 * LB)
    assert state.fuel_total_gal == pytest.approx(100 * LB / 6)
    assert state.ambient_wind_velocity == pytest.approx(10 * KT)
    assert state.ambient_visibility_m == pytest.approx(5 * 1609.344)
    assert state.sea_level_pressure_hpa == pytest.approx(1013.25)
    assert not state.nav1_has_localizer
    assert link.read_now("latitude") == 51.5
    assert link.read_now("not_a_field") is None


@pytest.mark.parametrize("xp_mode,expected", [(0, 0), (1, 1), (2, 3), (3, 4),
                                              (4, 1), (5, 4), (6, 4), (7, 4)])
def test_transponder_modes(xp, xp_mode, expected):
    server, link = xp
    server.values[EXTRA["xp_transponder"]] = xp_mode
    assert link.connect()
    eventually(lambda: "xp_transponder" in link._values)
    assert link.read().transponder_mode == expected
    server.values[EXTRA["ident"]] = 1
    eventually(lambda: link.read().transponder_mode == 5)


@pytest.mark.parametrize("engine,helo,expected", [(0, 0, 0), (1, 0, 0), (3, 0, 4),
                                                 (5, 0, 1), (7, 0, 1), (9, 1, 3),
                                                 (10, 0, 5), (99, 0, -1)])
def test_engine_types(xp, engine, helo, expected):
    server, link = xp
    server.values.update({EXTRA["xp_engine_type"]: engine, EXTRA["helicopter"]: helo})
    assert link.connect()
    eventually(lambda: "helicopter" in link._values)
    assert link.read().engine_type == expected
    assert (link.read().category == "Helicopter") == bool(helo)


def test_radio_and_command_writes_and_safe_noops(xp):
    server, link = xp
    assert not link.set_com(123.805)
    assert link.connect()
    for radio in (1, 2):
        for standby in (False, True):
            assert link.set_com(123.805, radio, standby)
    assert link.swap_com(1) and link.swap_com(2)
    assert link.set_nav(110.3, 2) and link.set_squawk("0042") and link.pushback()
    eventually(lambda: len(server.writes) == 6 and len(server.commands) == 3)
    for radio in (1, 2):
        for standby in (False, True):
            path = f"com{radio}_" + ("standby_" if standby else "") + "frequency_hz_833"
            assert (RADIOS + path, 123805) in server.writes
    assert (RADIOS + "nav2_frequency_hz", 11030) in server.writes
    assert (RADIOS + "transponder_code", 42) in server.writes
    assert server.commands == ["sim/radios/com1_standy_flip", "sim/radios/com2_standy_flip",
                               "sim/ground_ops/pushback_straight"]
    for value in (None, "bad", float("nan"), 117, 137):
        assert not link.set_com(value)
    assert not link.set_com(123.8, 3) and not link.swap_com(3)
    assert not link.set_nav(None) and not link.set_nav(123)
    assert not link.set_squawk("8888") and not link.set_squawk("42")
    assert not link.set_variable("L:GSX", 1) and not link._send(b"TEST", 1)
    assert link.loaded_flight_plan() is None and link.flight_plan_reply() is None
    assert not link.request_flight_plan() and not link.set_fuel(100)
    assert link.fuel_tanks() == []


@pytest.mark.parametrize("xp_value,msfs", [(0, 0), (1, 2), (2, 6), (3, 3),
                                           (4, 1), (5, 8), (9, -1)])
def test_fuel_selector_enum(xp, xp_value, msfs):
    # X-Plane's tank enum differs from the SimConnect one the state carries
    # (0 none/left/centre/right/all/aft vs 0 off, 1 all, 2 left, 3 right,
    # 6 centre, 8 centre3): left was reaching consumers as "all" and
    # centre as "left".
    server, link = xp
    server.values[DATAREFS["fuel_selector"][0]] = xp_value
    assert link.connect()
    eventually(lambda: "fuel_selector" in link._values)
    assert link.read().fuel_selector == msfs


def test_fuel_selector_write_translates(xp):
    server, link = xp
    assert link.connect()
    assert link.set_variable("FUEL TANK SELECTOR:1", 2)   # SimConnect left
    eventually(lambda: (DATAREFS["fuel_selector"][0], 1) in server.writes)
    assert link.set_variable("FUEL TANK SELECTOR:1", 1)   # SimConnect all
    eventually(lambda: (DATAREFS["fuel_selector"][0], 4) in server.writes)


def test_autopilot_falls_back_to_servos(xp):
    # With the cockpit2 autopilot_on read absent, the built-in servos_on
    # boolean is the answer -- and a boolean is never == 2.
    from meridianatc.sim.xpcontrols import READS

    server, link = xp
    server.missing.add(READS["control:ap_on"])
    server.values[EXTRA["autopilot_mode"]] = 1
    assert link.connect()
    eventually(lambda: "autopilot_mode" in link._values)
    assert link.read().autopilot
    server.values[EXTRA["autopilot_mode"]] = 0
    eventually(lambda: not link.read().autopilot)


def test_load_factor_and_pitch_datarefs(xp):
    # The envelope protection's reads: the normal load factor and the
    # pitch attitude land on the same AircraftState fields MSFS fills
    # from G FORCE and PLANE PITCH DEGREES.
    server, link = xp
    server.values[DATAREFS["g_force"][0]] = 1.42
    server.values[DATAREFS["pitch_deg"][0]] = 8.5
    assert link.connect()
    eventually(lambda: link.read().g_force == pytest.approx(1.42))
    assert link.read().pitch_deg == pytest.approx(8.5)
    assert link.read().ap_max_bank_deg == 0.0


def test_load_datarefs_report_unavailable_when_unanswered(xp):
    server, link = xp
    server.missing.add(DATAREFS["g_force"][0])
    assert link.connect()
    eventually(lambda: link.read().connected)
    assert "g_force" in link.read().unavailable


def test_fuel_distribution(xp):
    server, link = xp
    server.values.update({EXTRA["fuel_capacity_kg"]: 100,
                          EXTRA["tank_ratio0"]: .4, EXTRA["tank_ratio1"]: .6,
                          EXTRA["tank0"]: 20, EXTRA["tank1"]: 30})
    assert link.connect()
    eventually(lambda: len(link.fuel_tanks()) == 2)
    assert sum(cap for _, cap, _ in link.fuel_tanks()) == pytest.approx(100 * LB / 6)
    assert link.set_fuel(50 * LB / 6)
    eventually(lambda: len(server.writes) == 2)
    assert server.writes[0] == ("sim/flightmodel/weight/m_fuel[0]", pytest.approx(20))
    assert server.writes[1] == ("sim/flightmodel/weight/m_fuel[1]", pytest.approx(30))
    assert not link.set_fuel(float("nan")) and not link.set_fuel(-1)


def test_disconnect_and_unsubscribe(xp):
    server, link = xp
    assert link.connect()
    eventually(lambda: len(server.subscriptions) == len(link._subscriptions))
    server.respond = False
    eventually(lambda: not link.connected)
    assert not link.read().connected and link.read_now("latitude") is None
    assert not link.pushback()
    link.close()
    eventually(lambda: len(server.unsubscribed) == len(link._subscriptions))
    assert not server.subscriptions
    link.close()


def test_silent_or_malformed_server_is_not_connected(xp):
    server, link = xp
    server.respond = False
    assert not link.connect()
    assert not link.connected
    assert link.read().type_code == ""
    assert link.read_now("latitude") is None


def test_malformed_and_unknown_values_do_not_refresh_connection(xp):
    server, link = xp
    assert link.connect()
    server.respond = False
    eventually(lambda: not link.connected)
    for packet in (b"RREF\0", b"RREF\0x", b"WRONG" + struct.pack("<if", 0, 1),
                   b"RREF\0" + struct.pack("<if", 999999, 1),
                   b"RREF\0" + struct.pack("<if", 0, float("nan"))):
        server.socket.sendto(packet, server.client)
    assert not link.connected


def test_beacon_and_discovered_endpoint_fallback(xp, monkeypatch):
    server, link = xp
    packet = b"BECN\0" + struct.pack("<BBiiIH", 1, 2, 1, 120100, 1, server.port) + b"sim\0"
    assert link.parse_beacon(packet, ("127.0.0.1", 99)) == ("127.0.0.1", server.port, 120100)
    assert link.parse_beacon(b"BECN\0", ("127.0.0.1", 99)) is None
    assert link.parse_beacon(packet.replace(b"BECN", b"TEST"), ("127.0.0.1", 99)) is None
    monkeypatch.setattr(link, "_discover", lambda: ("127.0.0.1", 1, 120100))
    assert link.connect()
    assert link.version == 120100


@pytest.mark.parametrize("setting,msfs_ok,xp_ok,expected,calls", [
    ("auto", True, True, "msfs", ["msfs"]),
    ("auto", False, True, "xplane", ["msfs", "xplane"]),
    ("auto", False, False, "manual", ["msfs", "xplane"]),
    ("msfs", False, True, "manual", ["msfs"]),
    ("xplane", True, True, "xplane", ["xplane"]),
    ("xplane", True, False, "manual", ["xplane"]),
])
def test_selection(monkeypatch, setting, msfs_ok, xp_ok, expected, calls):
    from meridianatc.sim import xplane

    attempted = []
    msfs, xp = Mock(), Mock()
    msfs.connect.side_effect = lambda: attempted.append("msfs") or msfs_ok
    xp.connect.side_effect = lambda: attempted.append("xplane") or xp_ok
    msfs_factory, xp_factory = Mock(return_value=msfs), Mock(return_value=xp)
    monkeypatch.setattr(sources, "SimConnectSource", msfs_factory)
    monkeypatch.setattr(xplane, "XPlaneSource", xp_factory)
    result = sources.open_source(simulator=setting, xplane_host="192.0.2.1", xplane_port=49001)
    assert attempted == calls
    if expected == "manual":
        assert isinstance(result, sources.ManualSource)
    else:
        assert result is {"msfs": msfs, "xplane": xp}[expected]
    if "xplane" in calls:
        xp_factory.assert_called_once_with(host="192.0.2.1", port=49001, poll_interval=.25)
    if "msfs" in calls and not msfs_ok:
        msfs.close.assert_called_once()


def test_offline_selection_and_defaults():
    assert isinstance(sources.open_source(prefer_sim=False), sources.ManualSource)
    assert SimConfig().simulator == "auto"
    assert (SimConfig().xplane_host, SimConfig().xplane_port) == ("127.0.0.1", 49000)


def test_engine_attachment_and_msfs_only_guards(monkeypatch, xp):
    pytest.importorskip("scipy")
    from meridianatc import engine as module

    attach = Mock(return_value=False)
    monkeypatch.setattr(module.XPlaneTraffic, "attach", attach)
    server, link = xp
    server.values[DATAREFS["on_ground"][0]] = 0
    assert link.connect()
    eventually(lambda: not link.read().on_ground)
    engine = module.Engine.__new__(module.Engine)
    engine.config = Config()
    engine.config.sim.simulator = "xplane"
    engine.config.immersion.autonomous_flight = True
    previous = Mock()
    engine.source = previous
    engine.state = link.read()
    engine.refresh_state = Mock()
    engine.autonomous_flight = Mock()
    engine._facilities_done = set()
    engine.emit = Mock()
    for name in ("simvehicles", "simdoors", "simsystems", "servicepoints", "simtraffic",
                 "simwatch", "facilities", "gsx", "traffic"):
        setattr(engine, name, Mock())
    factory = Mock(return_value=link)
    monkeypatch.setattr(module.x_the_simulator_link, "open_source", factory)
    assert engine.attach_simulator()
    assert engine.source is link and engine.autonomous_flight.sim is link
    engine.autonomous_flight.enable.assert_called_once()
    assert engine._sim_source is link
    previous.close.assert_called_once()
    assert factory.call_args.kwargs["simulator"] == "xplane"
    assert isinstance(engine.simtraffic, module.XPlaneTraffic)
    assert not engine.open_simulator_frequencies()
    assert engine.attach_lvar_bridge() == ""
    engine.simwatch.attach.assert_not_called()
    attach.assert_called_once_with()
    engine.facilities.open.assert_not_called()
    engine.gsx.attach.assert_called_once_with(None)
    assert any("Connected to X-Plane 12" in call.args[1]
               for call in engine.emit.call_args_list)


def test_failed_engine_attachment_keeps_previous_source(monkeypatch):
    pytest.importorskip("scipy")
    from meridianatc import engine as module

    engine = module.Engine.__new__(module.Engine)
    engine.config = Config()
    engine.source = previous = Mock()
    monkeypatch.setattr(module.x_the_simulator_link, "open_source", lambda **kwargs: sources.ManualSource())
    assert not engine.attach_simulator()
    assert engine.source is previous
    previous.close.assert_not_called()


def test_parity_derived_instruments(xp):
    """The reads MSFS has and X-Plane now derives: Mmo and its barber
    pole, COM1 status, the side tanks, torque per cent, the body-axes
    wind, and the FAA heavy line."""
    server, link = xp
    server.values.update({EXTRA[key]: value for key, value in {
        "com1_powered": 1, "fuel_left_kg": 90.0, "fuel_right_kg": 60.0,
        "trq_nm": 2000.0, "trq_red_hi_ftpds": 2000.0,
        "wind_x_ms": 10.0, "wind_y_ms": 5.0, "wind_z_ms": 0.0,
    }.items()})
    server.values[DATAREFS["mach_max_operate"][0]] = 0.82
    server.values[DATAREFS["heading_true"][0]] = 0.0
    # acf_m_max is kilograms; 300,000 lb puts it over the FAA line.
    server.values[DATAREFS["max_gross_weight_lb"][0]] = 300000.0 / LB
    assert link.connect()
    eventually(lambda: "wind_x_ms" in link._values)
    state = link.read()
    assert state.mach_max_operate == pytest.approx(0.82)
    assert state.barber_pole_mach == pytest.approx(0.82)
    assert state.com1_status == 0
    assert state.fuel_left_gal == pytest.approx(90.0 * LB / 6)
    assert state.fuel_right_gal == pytest.approx(60.0 * LB / 6)
    # 2,000 N-m against a 2,000 ft-lb red line: 1,475.6 ft-lbs, ~74%.
    assert state.eng_torque_pct == pytest.approx(
        2000.0 / (2000.0 * 1.35582) * 100.0)
    # Facing north with the wind blowing east: all across, none along.
    assert state.aircraft_wind_x_kt == pytest.approx(10.0 * KT, rel=1e-3)
    assert state.aircraft_wind_z_kt == pytest.approx(0.0, abs=1e-6)
    assert state.aircraft_wind_y_kt == pytest.approx(5.0 * KT, rel=1e-3)
    assert state.atc_heavy
    assert "mach_max_operate" not in state.unavailable
    assert "aircraft_wind_x_kt" not in state.unavailable
    assert "eng_torque_pct" not in state.unavailable


def test_parity_instruments_absent_when_the_model_is_silent(xp):
    """No torque red line: the per cent cannot be computed, so it joins
    unavailable rather than being invented. The figures that always
    answer -- acf_Mmo, the wind -- read 0 on a silent model, which the
    consumers' ">0" gates read as unpublished."""
    server, link = xp
    server.values[EXTRA["trq_nm"]] = 2000.0          # no red line
    assert link.connect()
    eventually(lambda: "trq_nm" in link._values)
    state = link.read()
    assert state.eng_torque_pct == 0.0
    assert "eng_torque_pct" in state.unavailable
    assert state.mach_max_operate == 0.0
    assert state.barber_pole_mach == 0.0


def test_xplane_radio_power():
    from meridianatc.atc.radio_power import decide

    state = AircraftState(source="xplane", connected=True, main_bus_volts=0)
    assert not decide(state)[0]
    state.main_bus_volts = 28
    assert decide(state)[0]
