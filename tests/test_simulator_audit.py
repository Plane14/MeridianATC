"""Regression coverage for provider parity, connection lifecycle and API contracts."""

from __future__ import annotations

import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from test_xplane import FakeXPlane, eventually

from meridianatc.atc.checklist import ChecklistRunner
from meridianatc.atc.checklist_files import Check, Item
from meridianatc.config import Config, SettingRefused, check
from meridianatc.sim import source as sources
from meridianatc.sim import xplane
from meridianatc.sim.capabilities import capabilities
from meridianatc.sim.state import AircraftState
from meridianatc.weather.metar import weather_from_sim


@pytest.fixture
def link():
    peer = FakeXPlane()
    source = xplane.XPlaneSource(port=peer.port, discovery_s=0, timeout_s=.2)
    try:
        yield peer, source
    finally:
        source.close()
        peer.close()


def test_checklist_derived_reads_and_unknown_values(link):
    peer, source = link
    peer.values.update({xplane.EXTRA["flaps"]: .5, xplane.EXTRA["flap_detents"]: 3,
                        xplane.EXTRA["squawk"]: 2000, xplane.EXTRA["xp_transponder"]: 3,
                        xplane.DATAREFS["fuel_selector"][0]: 4})
    assert source.connect()
    eventually(lambda: source.read_now("flaps_index") == 1)
    eventually(lambda: source.read_now("transponder_code") == "2000")
    assert source.read_now("transponder_state") == 4
    # X-Plane's "4" is all; the state carries the SimConnect enum ("1").
    assert source.read_now("fuel_selector") == 1
    assert source.read_now("BRAKE PARKING POSITION") is False
    assert source.read_now("rotor_rpm_pct") is None
    runner = ChecklistRunner(Mock(), fresh=source.read_now)
    runner._state = source.read()
    runner._note = Mock()
    unknown = Item({"en": "Rotor"}, {"en": "set"}, checks=(Check("rotor_rpm_pct", value=0),))
    assert runner._verify(unknown, SimpleNamespace(value=None))[0] is None
    assert not runner._agrees_now(unknown)
    runner._note.assert_called_once()
    gear = Item({"en": "Gear"}, {"en": "down"}, checks=(Check("gear_down", value=True),))
    assert runner._verify(gear, SimpleNamespace(value=None))[0] is False


def test_xplane_switch_writes_weather_and_pause(link):
    peer, source = link
    peer.values.update({xplane.DATAREFS["paused"][0]: 1,
                        xplane.DATAREFS["sea_level_pressure_hpa"][0]: 101325})
    assert source.connect()
    eventually(lambda: source.read().paused)
    eventually(lambda: source.read().sea_level_pressure_hpa > 1000)
    wx = weather_from_sim(source.read())
    assert wx.qnh_hpa == pytest.approx(1013.25)
    assert wx.temperature_c == 0 and wx.dewpoint_c == -20 and wx.visibility_sm == 0
    assert source.set_variable("BRAKE PARKING POSITION", 1)
    assert source.set_variable("GEAR HANDLE POSITION", 0)
    assert source.set_variable("KOHLSMAN SETTING MB", 1013.25)
    assert source.set_variable("AUTOPILOT MASTER", 1)
    assert not source.set_variable("LIGHT BEACON", float("nan"))
    eventually(lambda: len(peer.writes) == 3)
    assert peer.writes[0] == (xplane.DATAREFS["parking_brake"][0], 1)
    assert peer.writes[1] == (xplane.DATAREFS["gear_down"][0], 0)
    assert peer.writes[2][1] == pytest.approx(29.9213, abs=.001)
    peer.values[xplane.DATAREFS["paused"][0]] = 0
    eventually(lambda: not source.read().paused)
    assert source.connected


def test_capabilities_and_honest_autofly_refusal():
    pytest.importorskip("scipy")
    from meridianatc import engine as engines

    assert capabilities("simconnect").facilities and capabilities("simconnect").autofly
    assert capabilities("xplane").live and not capabilities("xplane").lvars
    assert not capabilities("manual").live
    engine = engines.Engine.__new__(engines.Engine)
    engine.state = AircraftState(source="xplane", connected=True)
    engine.autonomous_flight = Mock()
    engine.emit = Mock()
    # X-Plane now answers the full axis set, so the engage is not refused
    # for the simulator -- on the ground or airborne, the FO takes it.
    engine.session = None
    engine.autonomous_flight.engage.return_value = True
    assert engine.engage_autonomous()[0]
    engine.autonomous_flight.engage.assert_called_once_with(None)
    # Only a source that cannot fly at all still refuses.
    engine.state = AircraftState(source="manual", connected=True)
    engine.autonomous_flight.reset_mock()
    ok, message = engine.engage_autonomous()
    assert not ok and "not available" in message
    assert not engine.autonomous_status()["available"]
    engine.autonomous_flight.engage.assert_not_called()


def test_configured_autofly_runs_xplane_controls():
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )

    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo._enabled = True
    fo.state = AutonomousState(phase=AutonomousPhase.CRUISE)
    fo.get_state = lambda: AircraftState(source="xplane", connected=True)
    fo._emit = Mock()
    fo._handoff = Mock()
    for name in ("_manage_comms", "_drive_checklist", "_monitor_weather",
                 "_watch_engines", "_manage_cooling", "_monitor_emergency",
                 "_monitor_load", "_execute_phase"):
        setattr(fo, name, Mock())
    fo._heli = None
    fo._is_helicopter = False
    fo.ACTION_INTERVAL = 0
    fo.tick()
    assert fo.enabled
    fo._manage_comms.assert_called_once()
    fo._execute_phase.assert_called_once()


def test_exception_in_msfs_attempt_still_falls_back(monkeypatch):
    failed = Mock()
    failed.connect.side_effect = OSError("MSFS unavailable")
    connected = Mock()
    connected.connect.return_value = True
    monkeypatch.setattr(sources, "SimConnectSource", lambda **kwargs: failed)
    monkeypatch.setattr(xplane, "XPlaneSource", lambda **kwargs: connected)
    assert sources.open_source() is connected
    failed.close.assert_called_once()


def watcher():
    pytest.importorskip("scipy")
    from meridianatc import engine as engines

    engine = engines.Engine.__new__(engines.Engine)
    engine.config = Config()
    engine._sim_wanted = True
    engine._sim_source = None
    engine._sim_attempt = None
    engine._sim_lock = threading.RLock()
    engine._next_sim_try = 0
    engine.state = AircraftState(source="manual", connected=True)
    engine.source = sources.ManualSource(engine.state)
    engine._adopt_simulator = Mock(return_value=True)
    engine.begin_session = Mock()
    engine._detach_simulator_integrations = Mock()
    engine.refresh_state = Mock()
    engine.emit = Mock()
    return engine


def test_slow_connection_never_blocks_situation_loop_or_duplicates_attempts(monkeypatch):
    engine = watcher()
    entered, release = threading.Event(), threading.Event()
    connected = Mock()

    def connect(**kwargs):
        entered.set()
        assert release.wait(2)
        return connected

    factory = Mock(side_effect=connect)
    monkeypatch.setattr("meridianatc.engine.x_the_simulator_link.open_source", factory)
    engine._watch_simulator()
    assert entered.wait(1)
    attempt = engine._sim_attempt
    try:
        watchers = [threading.Thread(target=engine._watch_simulator) for _ in range(4)]
        for thread in watchers:
            thread.start()
        for thread in watchers:
            thread.join(timeout=1)
            assert not thread.is_alive()
        engine._adopt_simulator.assert_not_called()
        assert factory.call_count == 1
    finally:
        release.set()
    attempt.result(timeout=2)
    engine._watch_simulator()
    engine._adopt_simulator.assert_called_once_with(connected)
    engine.begin_session.assert_called_once()


def test_disabling_discards_pending_connection(monkeypatch):
    engine = watcher()
    release = threading.Event()
    connected = Mock()

    def connect(**kwargs):
        assert release.wait(2)
        return connected

    monkeypatch.setattr("meridianatc.engine.x_the_simulator_link.open_source", connect)
    engine._watch_simulator()
    attempt = engine._sim_attempt
    engine.want_simulator(False)
    release.set()
    attempt.result(timeout=2)
    eventually(lambda: connected.close.call_count == 1)
    assert engine._sim_attempt is None
    engine._adopt_simulator.assert_not_called()


def test_loss_and_disable_release_integrations_and_keep_position():
    engine = watcher()
    engine.state.latitude = 48
    engine.state.paused = True
    engine._sim_source = engine.source = Mock(connected=False, alive=False)
    previous = engine.source
    engine._watch_simulator()
    engine._detach_simulator_integrations.assert_called_once()
    previous.close.assert_called_once()
    assert engine.source.read().latitude == 48 and not engine.source.read().paused
    assert engine._sim_source is None and engine._next_sim_try > 0
    engine._sim_source = engine.source = Mock(connected=True)
    previous = engine.source
    engine.want_simulator(False)
    previous.close.assert_called_once()
    assert engine.source.read().source == "manual"


def test_xplane_timeout_and_reconnect_on_same_source(link):
    peer, source = link
    source.disconnect_s = .1
    assert source.connect()
    peer.respond = False
    eventually(lambda: not source.connected)
    peer.respond = True
    assert source.connect() and source.connected


def test_simulator_settings_roundtrip_and_validation(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("sim:\n  simulator: xplane\n  xplane_port: 49001\ntraffic:\n  xplane_plugin_port: 49011\n", encoding="utf-8")
    config = Config.load(path)
    assert config.sim.simulator == "xplane" and config.traffic.xplane_plugin_port == 49011
    config.sim.xplane_host = "127.0.0.2"
    config.save()
    loaded = Config.load(path)
    assert loaded.sim.xplane_host == "127.0.0.2" and loaded.sim.xplane_port == 49001
    assert loaded.traffic.xplane_plugin_port == 49011
    for key, value in (("sim.simulator", "invalid"), ("sim.xplane_port", 65536),
                       ("traffic.xplane_plugin_port", -1)):
        with pytest.raises(SettingRefused):
            check(key, value)


def test_api_snapshot_and_settings_validation(monkeypatch):
    pytest.importorskip("fastapi")
    pytest.importorskip("scipy")
    from meridianatc.web import server

    engine = Mock()
    engine.state = AircraftState(source="xplane", connected=True)
    engine.config = Config()
    engine.session = engine.station = None
    engine.languages = ["en"]
    engine.last_language = "en"
    engine.crew.checklist_titles.return_value = {}
    engine.gsx = None
    engine.live_feed = engine.landing = None
    engine.started_at = 0
    engine.radio_powered = True
    engine.microphone.recording = engine.player.busy = False
    engine.understanding.describe.return_value = "rules"
    engine.understanding.stats.summary.return_value = {}
    engine.understanding.model_available = False
    engine.autonomous_status.return_value = {"available": False, "enabled": False}
    engine.traffic.positions.return_value = []
    for helper in ("_standby_station", "_profile_view", "_progress"):
        monkeypatch.setattr(server, helper, lambda *args: {})
    result = server.snapshot(engine)
    assert result["source"] == "xplane" and result["simulator"] == "X-Plane 12"
    assert result["connected"] and not result["autonomous"]["available"]
    with pytest.raises(server.SettingsRefused):
        server._check_all({"sim.simulator": "bad", "traffic.xplane_plugin_port": 49010})


def test_a_check_holds_on_every_engine_not_just_the_first():
    """A twin's "throttles ... idle" is only idle where the second
    lever is idle too: the eng2_ slot is checked, and the disagreement
    it names is the second engine's figure, not engine one's."""
    item = Item({"en": "Throttles"}, {"en": "idle"},
                checks=(Check("throttle_pct", op="max", value=5.0),))
    state = AircraftState(throttle_pct=0.0, eng2_throttle_pct=85.0)
    runner = ChecklistRunner(
        Mock(), fresh=lambda name: getattr(state, name, None))
    runner._state = state
    runner._note = Mock()
    verdict, actual, _expected, _again, _failed = runner._verify(
        item, SimpleNamespace(value=None))
    assert verdict is False and actual == 85.0
    # Both levers at the stop: agreement.
    state.eng2_throttle_pct = 0.0
    verdict = runner._verify(item, SimpleNamespace(value=None))[0]
    assert verdict is True
    # No second engine reported at all: engine one's scalar rules alone.
    state.eng2_throttle_pct = None
    verdict = runner._verify(item, SimpleNamespace(value=None))[0]
    assert verdict is True
    # A ghost slot never answers an engine that is not there: a bare
    # state has no eng2_ fields, so nothing phantom can object.
    bare = AircraftState(throttle_pct=0.0)
    runner._state = bare
    assert runner._agrees_now(item)


def test_msfs_registers_the_per_engine_gauges_and_levers():
    """The twin's other three engine slots ride the same SimConnect
    names as engine one's, indexed -- the slow rotation carries them."""
    kinds = {attr: kind for _var, attr, kind, _rate
             in sources.SimConnectSource.REQUESTS}
    for n in (2, 3, 4):
        for attr in ("throttle_pct", "mixture_pct", "prop_pct",
                     "condition_pct", "itt_c", "torque_nm",
                     "torque_pct", "n1_pct", "n2_pct", "rpm"):
            assert kinds.get(f"eng{n}_{attr}"), f"eng{n}_{attr}"
    names = {var for var, _attr, _kind, _rate
             in sources.SimConnectSource.REQUESTS}
    assert "TURB_ENG_ITT:2" in names and "ENG_TORQUE:4" in names
    assert "GENERAL_ENG_THROTTLE_LEVER_POSITION:3" in names


def test_msfs_refuses_standby_channel_bcd_cannot_say(monkeypatch):
    # 118.005 MHz is a legal 8.33 kHz channel that no 25 kHz BCD event
    # can name; accepting it tunes the radio to a frequency ATC never
    # gave.
    source = sources.SimConnectSource()
    source._connected = True
    sent = []
    monkeypatch.setattr(source, "_send",
                        lambda event, value: sent.append(event) or True)
    monkeypatch.setattr(source, "_reads", lambda key, wanted: False)
    monkeypatch.setattr(source, "_took", lambda key, wanted: False)
    hz_event, bcd_event = sources.SimConnectSource._COM_EVENTS[(1, True)]
    assert not source.set_com(118.005, 1, standby=True)
    assert sent == [hz_event]      # the BCD fallback was never sent
    # A 25 kHz channel still gets the BCD probe when Hz did not take.
    assert not source.set_com(118.025, 1, standby=True)
    assert sent[-1] == bcd_event
    sent.clear()
    # And once "bcd" is learned, an 8.33 channel is refused outright.
    source._freq_event_style[(1, True)] = "bcd"
    assert not source.set_com(118.005, 1, standby=True)
    assert sent == []


def test_msfs_attitude_arrives_on_the_pilots_convention():
    """The 2026-11-20 case: SimConnect's attitude angles live on the
    FSX-era body convention -- pitch positive nose-DOWN, bank positive
    LEFT -- while every loop in the app flies nose-up and right-bank
    positive, the convention the axis events are negated into. A
    rotation that read -14 degrees spent the takeoff being pulled at
    and a left bank read as right rolled the level command deeper;
    the read is turned round at the wire like the write."""
    import math

    convert = sources.SimConnectSource._convert
    # A +15-degree rotation is -0.26 rad on the sim's books; a left
    # bank of 20 is +0.35.
    assert convert(math.radians(-15.0), "signed_rad_flip",
                   "pitch_deg") == pytest.approx(15.0)
    assert convert(math.radians(20.0), "signed_rad_flip",
                   "bank_deg") == pytest.approx(-20.0)
    # The attitude rows ride it; the magnitudes do not need it.
    kinds = {attr: kind for _var, attr, kind, _rate
             in sources.SimConnectSource.REQUESTS}
    assert kinds["pitch_deg"] == "signed_rad_flip"
    assert kinds["bank_deg"] == "signed_rad_flip"
    assert kinds["ap_max_bank_deg"] == "signed_rad"


def test_route_from_fms_entries():
    pytest.importorskip("scipy")
    from meridianatc.engine.x_the_plan_the import _The_plan_the

    planner = _The_plan_the()
    route = planner._route_from_fms([
        {"i": 0, "t": 1, "id": "EDDM", "alt": 0,
         "lat": 48.35, "lon": 11.79, "active": False},
        {"i": 1, "t": 4, "id": "MIQ", "alt": 5000,
         "lat": 48.80, "lon": 11.40, "active": True},
        {"i": 2, "t": 512, "id": "BAXRI", "alt": 8000,
         "lat": 50.2, "lon": 8.4, "active": False},
        {"i": 3, "t": 256, "id": "OM", "alt": 0,
         "lat": 51.0, "lon": 0.1, "active": False},   # a marker: skipped
        {"i": 4, "t": 2048, "id": "", "alt": 4000,
         "lat": 51.1, "lon": -0.2, "active": False},
        {"i": 5, "t": 1, "id": "EGLL", "alt": 0,
         "lat": 51.47, "lon": -0.45, "active": False},
    ])
    assert route is not None
    assert route.origin == "EDDM" and route.destination == "EGLL"
    assert route.source == "sim"
    kinds = [fix.kind for fix in route.fixes]
    assert kinds == ["apt", "vor", "wpt", "wpt", "apt"]
    assert route.fixes[1].alt_ft == 5000
    # The unnamed lat/lon entry gets a coordinate for a callsign.
    assert route.fixes[3].ident == "+51.1000-0.2000"
    # An approach-free one-way plan, and fewer than two points, decline.
    assert planner._route_from_fms([{"i": 0, "t": 1, "id": "EDDM", "alt": 0,
                                     "lat": 48.35, "lon": 11.79,
                                     "active": False}]) is None
    assert planner._route_from_fms([]) is None


def test_fms_cruise_is_not_the_departure_fields_height():
    """LECO's plan read back its runway leg at the field's own 327 feet
    and the inferred cruise became the runway's height: departure
    legs -- the runway and anything at or under the field -- are not
    the level, so nothing was filed at all rather than 327."""
    pytest.importorskip("scipy")
    from meridianatc.engine.x_the_plan_the import _The_plan_the

    planner = _The_plan_the()
    route = planner._route_from_fms([
        {"i": 0, "t": 1, "id": "LECO", "alt": 327,
         "lat": 43.30, "lon": -8.38, "active": False},
        {"i": 1, "t": 2048, "id": "RW21", "alt": 327,
         "lat": 43.31, "lon": -8.39, "active": False},
        {"i": 2, "t": 512, "id": "BERAX", "alt": 327,
         "lat": 43.50, "lon": -8.30, "active": False},
        {"i": 3, "t": 512, "id": "FORNO", "alt": 0,
         "lat": 43.70, "lon": -8.10, "active": False},
        {"i": 4, "t": 512, "id": "BBI", "alt": 0,
         "lat": 42.60, "lon": -6.90, "active": False},
        {"i": 5, "t": 1, "id": "LECU", "alt": 2000,
         "lat": 40.37, "lon": -3.78, "active": False},
    ])
    assert route is not None
    # The destination's own elevation is under the departure field's --
    # the read-back's only en-route candidates were departure legs, so
    # the plan files nothing rather than the runway's height.
    assert route.cruise_ft == 0


def test_fms_cruise_takes_the_enroute_leg_levels():
    """A plan with legs genuinely up high still gets its level from
    them -- the field-elevation floor only drops departure legs."""
    pytest.importorskip("scipy")
    from meridianatc.engine.x_the_plan_the import _The_plan_the

    planner = _The_plan_the()
    route = planner._route_from_fms([
        {"i": 0, "t": 1, "id": "LECO", "alt": 327,
         "lat": 43.30, "lon": -8.38, "active": False},
        {"i": 1, "t": 2048, "id": "RW21", "alt": 327,
         "lat": 43.31, "lon": -8.39, "active": False},
        {"i": 2, "t": 512, "id": "FORNO", "alt": 8000,
         "lat": 43.70, "lon": -8.10, "active": False},
        {"i": 3, "t": 512, "id": "BBI", "alt": 20000,
         "lat": 42.60, "lon": -6.90, "active": False},
        {"i": 4, "t": 1, "id": "LECU", "alt": 2000,
         "lat": 40.37, "lon": -3.78, "active": False},
    ])
    assert route is not None
    assert route.cruise_ft == 20000
