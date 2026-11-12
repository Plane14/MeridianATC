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
    assert source.read_now("fuel_selector") == 4
    assert source.read_now("BRAKE PARKING POSITION") is False
    assert source.read_now("rotor_rpm_pct") is None
    runner = ChecklistRunner(Mock(), fresh=source.read_now)
    runner._state = source.read()
    runner._note = Mock()
    unknown = Item({"en": "Rotor"}, {"en": "set"}, check=Check("rotor_rpm_pct", value=0))
    assert runner._verify(unknown, SimpleNamespace(value=None))[0] is None
    assert not runner._agrees_now(unknown)
    runner._note.assert_called_once()
    gear = Item({"en": "Gear"}, {"en": "down"}, check=Check("gear_down", value=True))
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
    assert not source.set_variable("AUTOPILOT MASTER", 1)
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
    ok, message = engine.engage_autonomous()
    assert not ok and "not available" in message
    assert not engine.autonomous_status()["available"]
    engine.autonomous_flight.engage.assert_not_called()


def test_configured_autofly_does_not_send_xplane_controls():
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import AutonomousFlight

    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo._enabled = True
    fo.get_state = lambda: AircraftState(source="xplane", connected=True)
    fo._emit = Mock()
    fo._manage_comms = Mock()
    fo.tick()
    assert not fo.enabled
    fo._manage_comms.assert_not_called()
    fo._emit.assert_called_once()


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
    monkeypatch.setattr("meridianatc.engine.open_source", factory)
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

    monkeypatch.setattr("meridianatc.engine.open_source", connect)
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
    engine._sim_source = engine.source = Mock(connected=False)
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
    source.disconnect_s = .05
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
