"""Traffic sender contract exercised against a real localhost UDP peer."""

from __future__ import annotations

import json
import socket
import threading
import time
from unittest.mock import Mock

import pytest

from meridianatc.config import Config
from meridianatc.integrations.xptraffic import XPlaneTraffic, aircraft_packet
from meridianatc.sim.state import AircraftState


def eventually(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        threading.Event().wait(.005)
    assert predicate()


class FakePlugin:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(.02)
        self.port = self.socket.getsockname()[1]
        self.messages = []
        self.drawn = []
        self.unmatched = []
        self.models = 100
        self.reply = True
        self.peer = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def status(self, **overrides):
        status = {"v": 1, "t": "status", "plugin": "MeridianATC-XP", "version": "0.1.0",
                  "xpmp2": "3.7.0", "xplane": 120400, "csl_models": self.models,
                  "aircraft": len(self.drawn) + len(self.unmatched),
                  "drawn": list(self.drawn), "unmatched": list(self.unmatched)}
        status.update(overrides)
        if self.peer:
            self.socket.sendto(json.dumps(status).encode(), self.peer)

    def run(self):
        while not self.stop.is_set():
            try:
                wire, self.peer = self.socket.recvfrom(65536)
                packet = json.loads(wire)
                self.messages.append(packet)
                if self.reply and packet["t"] in ("hello", "traffic"):
                    self.status()
            except TimeoutError:
                continue
            except OSError:
                break

    def close(self):
        self.stop.set()
        self.thread.join(1)
        self.socket.close()


@pytest.fixture
def plugin():
    peer = FakePlugin()
    try:
        yield peer
    finally:
        peer.close()


@pytest.fixture
def link(plugin):
    sender = XPlaneTraffic(port=plugin.port)
    try:
        assert sender.attach()
        yield sender
    finally:
        sender.detach()


def position(callsign="DLH4AB", **extra):
    return dict(callsign=callsign, type="A320", operator="DLH", registration="D-AIAB",
                lat=48.1, lon=11.7, altitude_ft=12000, heading=359,
                ground_speed_kt=250, vertical_rate_fpm=600, on_ground=False,
                phase="departed", **extra)


def snapshots(plugin):
    return [p for p in plugin.messages if p["t"] == "traffic"]


def test_snapshot_and_confirmation(plugin, link):
    assert link.available and link.drawing and link.count == 0
    link.sync([position(pitch=2, roll=3, squawk="4321", gear=.2, flaps=.1,
                        lights={"taxi": 1, "land": 0})])
    eventually(lambda: len(snapshots(plugin)) == 1)
    packet = snapshots(plugin)[0]
    assert packet["v"] == 1 and packet["seq"] == 0
    assert packet["ac"] == [{"id": "DLH4AB", "type": "A320", "op": "DLH", "livery": "",
        "reg": "D-AIAB", "lat": 48.1, "lon": 11.7, "alt_ft": 12000, "hdg": 359,
        "pitch": 2, "roll": 3, "gs_kt": 250, "vs_fpm": 600, "gnd": False,
        "gear": .2, "flaps": .1, "lights": {"taxi": 1, "land": 0, "bcn": 1, "strb": 1, "nav": 1},
        "squawk": "4321"}]
    assert not link.has("DLH4AB") and not link.failed("DLH4AB")
    plugin.drawn = ["DLH4AB", "NOTOURS"]
    plugin.status()
    eventually(lambda: link.has("DLH4AB"))
    assert link.count == 1 and not link.has("NOTOURS") and not link.failed("DLH4AB")
    link.sync([])
    eventually(lambda: len(snapshots(plugin)) == 2)
    assert snapshots(plugin)[-1]["ac"] == [] and snapshots(plugin)[-1]["seq"] == 1
    assert not link.has("DLH4AB") and not link.failed("DLH4AB") and link.count == 0


def test_limit_and_datagram_bound(plugin, link):
    link.limit = 2
    link.sync([position("ONE"), position("TWO"), position("THREE")])
    eventually(lambda: len(snapshots(plugin)) == 1)
    assert [a["id"] for a in snapshots(plugin)[0]["ac"]] == ["ONE", "TWO"]
    link.limit = 0
    link.sync([position()])
    eventually(lambda: len(snapshots(plugin)) == 2)
    assert snapshots(plugin)[-1]["ac"] == []
    link.limit = 512
    link.sync([position(str(i), livery="X" * 128) for i in range(512)])
    eventually(lambda: len(snapshots(plugin)) == 3)
    assert 0 < len(snapshots(plugin)[-1]["ac"]) < 512
    assert len(json.dumps(snapshots(plugin)[-1], separators=(",", ":")).encode()) <= 65507


def test_unmatched_timeout_and_recovery(plugin, link):
    link.sync([position("BAD"), position("LATE")])
    eventually(lambda: len(snapshots(plugin)) == 1)
    plugin.unmatched = ["BAD"]
    plugin.status()
    eventually(lambda: link.failed("BAD"))
    assert not link.failed("LATE") and not link.has("BAD")
    link._wanted["LATE"] -= link.ANSWER_WITHIN_S + 1
    assert link.failed("LATE")
    plugin.drawn = ["LATE"]
    plugin.status()
    eventually(lambda: link.has("LATE"))
    assert not link.failed("LATE")
    link._last_status -= link.STATUS_WITHIN_S + 1
    assert not link.available and not link.drawing and not link.has("LATE")
    assert link.refused and link.count == 0
    plugin.status()
    eventually(lambda: link.available)


def test_hello_detach_and_compatibility(plugin, link):
    assert plugin.messages[0] == {"v": 1, "t": "hello"}
    link._last_hello -= link.HELLO_EVERY_S + 1
    link.on_frame()
    eventually(lambda: len([p for p in plugin.messages if p["t"] == "hello"]) == 2)
    assert link.object_id("X") is None and link.objects_of("X") == []
    assert link.read_position("X") is None and link.height_above_ground("X") is None
    assert link.resting_pitch("X") == 0 and link.title_of("X") == link.livery_of("X") == ""
    assert link.level_of("X") == link.target_level("X") == ""
    assert not link.swapping("X") and not link.has_detailed("A320", "DLH")
    assert not link.livery_matches("A320", "DLH", "X")
    assert "CSL models" in link.describe()
    link.detach()
    eventually(lambda: plugin.messages[-1]["t"] == "clear")
    assert not link.available and not link.drawing and not link.refused and link.count == 0
    assert not link.has("X") and not link.failed("X")
    link.detach()


def test_no_plugin_and_disabled(plugin):
    plugin.reply = False
    sender = XPlaneTraffic(port=plugin.port, timeout_s=.02)
    try:
        assert not sender.available and not sender.drawing and not sender.attach()
        assert not sender.refused
        sender._attached_at -= sender.STATUS_WITHIN_S + 1
        assert sender.refused
        sender.sync([position()])
        sender._wanted["DLH4AB"] -= sender.ANSWER_WITHIN_S + 1
        assert sender.failed("DLH4AB") and not sender.has("DLH4AB")
    finally:
        sender.detach()
    disabled = XPlaneTraffic(enabled=False, port=plugin.port)
    assert not disabled.attach() and not disabled.refused and "off" in disabled.describe()
    remote = XPlaneTraffic(host="192.0.2.1", timeout_s=0)
    assert not remote.attach() and not remote.available
    remote.detach()


def test_invalid_status_is_ignored(plugin, link):
    plugin.reply = False
    link.on_frame()
    link._last_status = float("-inf")
    for bad in ({"v": 2}, {"v": True}, {"plugin": "other"}, {"drawn": "X"},
                {"unmatched": [None]}, {"csl_models": -1}, {"aircraft": True}, {"version": 1}):
        plugin.status(**bad)
    plugin.socket.sendto(b"not json", plugin.peer)
    alien = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        alien.sendto(json.dumps({"v": 1, "t": "status", "plugin": "MeridianATC-XP",
            "version": "0.1.0", "xpmp2": "3.7.0", "xplane": 120400,
            "csl_models": 100, "aircraft": 0, "drawn": [], "unmatched": []}).encode(), plugin.peer)
    finally:
        alien.close()
    threading.Event().wait(.02)
    link.on_frame()
    assert not link.available
    plugin.status()
    eventually(lambda: link.available)
    plugin.models = 0
    plugin.status()
    eventually(lambda: link.refused)
    assert link.available and not link.drawing


def test_motion_and_phase_defaults():
    p = position(motion={"lat": 48.12345678, "lon": 11.7654321, "alt": 4500.2,
                         "heading": 361, "gs": 180.4, "vs": -800.2, "on_ground": False})
    p["phase"] = "final"
    a = aircraft_packet(p)
    assert (a["lat"], a["lon"], a["alt_ft"], a["hdg"], a["gs_kt"], a["vs_fpm"]) == (
        48.12345678, 11.7654321, 4500.2, 1, 180.4, -800.2)
    assert a["gear"] == 1 and a["flaps"] == 1 and a["lights"]["land"] == 1
    p = position()
    p.update(phase="parked", on_ground=True, ground_speed_kt=0)
    a = aircraft_packet(p)
    assert a["gnd"] and a["gear"] == 1 and not any(a["lights"].values())
    p.update(phase="taxi", ground_speed_kt=15)
    assert aircraft_packet(p)["lights"]["taxi"] == 1


@pytest.mark.parametrize("extra", [{"lat": float("nan")}, {"type": "TOOLONG"},
    {"squawk": "8888"}, {"lights": {"taxi": .5}}, {"gear": 2}, {"callsign": ""},
    {"lights": "bad"}, {"motion": "bad"}])
def test_invalid_positions(extra, plugin, link):
    p = position()
    p.update(extra)
    with pytest.raises(ValueError):
        aircraft_packet(p)
    link.sync([p])
    eventually(lambda: len(snapshots(plugin)) == 1)
    assert snapshots(plugin)[0]["ac"] == []


def test_engine_backend_selection_and_single_notice(monkeypatch, plugin):
    from meridianatc import engine as module

    engine = module.Engine.__new__(module.Engine)
    engine.config = Config()
    engine.config.traffic.xplane_plugin_port = plugin.port
    engine.config.traffic.in_simulator_limit = 5
    engine.config.traffic.parked = 2
    engine.state = AircraftState(source="xplane", connected=True)
    engine.source = Mock(spec=[])
    engine.simtraffic = old = module.SimTraffic()
    engine.traffic = Mock()
    engine.emit = Mock()
    engine.simwatch = Mock()
    assert engine.attach_simulator_traffic()
    assert isinstance(engine.simtraffic, XPlaneTraffic)
    assert engine.simtraffic.limit == 7 and engine.simtraffic.port == plugin.port
    assert not old.available and not engine.traffic.await_observed
    engine.simwatch.attach.assert_not_called()
    engine._choose_models([position()], time.monotonic())  # No MSFS LOD/config access.
    xp = engine.simtraffic
    xp.detach()
    plugin.reply = False
    xp.timeout_s = .01
    assert not engine.attach_simulator_traffic()
    assert not engine.attach_simulator_traffic()
    engine.emit.assert_called_once()
    assert engine.emit.call_args.kwargs["key"] == "log.xplane_traffic_unavailable"
    assert "xplane/README.md" in engine.emit.call_args.args[1]
    engine.state = AircraftState(source="simconnect", connected=True)
    engine.source = Mock(_sim=object())
    for name in ("simvehicles", "simdoors", "simsystems", "servicepoints"):
        setattr(engine, name, Mock())
    engine.use_fsltl_liveries = Mock()
    engine._read_ai_liveries = Mock()
    monkeypatch.setattr(module.SimTraffic, "attach", Mock(return_value=True))
    assert engine.attach_simulator_traffic()
    assert isinstance(engine.simtraffic, module.SimTraffic)
    assert not xp.available and xp._socket is None
    assert engine.simtraffic.limit == 7
    engine.simwatch.attach.assert_called_once_with(engine.source._sim)
    assert (Config().traffic.xplane_plugin_host, Config().traffic.xplane_plugin_port) == ("127.0.0.1", 49010)
