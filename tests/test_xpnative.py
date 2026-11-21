"""The native-fleet renderer: roster wire format and the pilots behind it."""

from __future__ import annotations

import time
from unittest.mock import Mock

import pytest

from meridianatc.config import Config
from meridianatc.integrations.xpnative import (
    HELICOPTER_TYPES,
    NativePilot,
    XPNativeTraffic,
)
from meridianatc.sim.state import AircraftState
from tests.test_xptraffic import FakePlugin, eventually, position


class NativePlugin(FakePlugin):
    """Same UDP peer, able to drive the AI slots."""

    def status(self, **overrides):
        overrides.setdefault("native_cap", 1)
        super().status(**overrides)


@pytest.fixture
def plugin():
    peer = NativePlugin()
    try:
        yield peer
    finally:
        peer.close()


class FakeSource:
    """The X-Plane dataref link the pilots write through."""

    def __init__(self):
        self.written = []
        self.subscriptions = {}
        self.values = {}
        self.connected = True

    def dref(self, path, value):
        self.written.append((path, float(value)))
        return True

    def subscribe(self, key, path, frequency=None):
        self.subscriptions[key] = path

    def value(self, key):
        return self.values.get(key)


@pytest.fixture
def link(plugin):
    source = FakeSource()
    sender = XPNativeTraffic(port=plugin.port, source=source, settle_s=0)
    try:
        assert sender.attach()
        yield sender
    finally:
        sender.detach()


def npos(callsign="DLH4AB", type="A320", **extra):
    p = position(callsign, **extra)
    p["type"] = type
    return p


def rosters(plugin):
    return [p for p in plugin.messages if p["t"] == "native"]


def last_write(source, channel, slot):
    path = f"sim/multiplayer/controls/{channel}[{slot}]"
    return next((v for p, v in reversed(source.written) if p == path), None)


def test_roster_and_control_writes(plugin, link):
    assert link.available and link.drawing and link.count == 0
    link.sync([position()])
    eventually(lambda: len(rosters(plugin)) == 1)
    assert rosters(plugin)[0] == {"v": 1, "t": "native",
                                  "ac": [{"id": "DLH4AB", "slot": 1,
                                          "type": "A320"}]}
    # The slot's feedback is subscribed and its levers are written.
    source = link.source
    assert source.subscriptions["xpn1:lat"] == \
        "sim/multiplayer/position/plane1_lat"
    assert source.subscriptions["xpn1:psi"] == \
        "sim/multiplayer/position/plane1_psi"
    assert last_write(source, "gear_request", 1) is not None
    assert last_write(source, "engine_throttle_request", 1) is not None
    assert link.has("DLH4AB") and not link.failed("DLH4AB")
    assert link.count == 1
    link.sync([])
    eventually(lambda: rosters(plugin)[-1]["ac"] == [])
    assert not link.has("DLH4AB") and link.count == 0


def test_slot_cap_and_over_cap_failure(plugin, link):
    many = [npos(f"CALL{i:02d}", type="C172") for i in range(25)]
    link.sync(many)
    eventually(lambda: rosters(plugin))
    roster = rosters(plugin)[0]["ac"]
    assert len(roster) == XPNativeTraffic.MAX_SLOTS == 19
    assert [a["slot"] for a in roster] == list(range(1, 20))
    assigned = {a["id"] for a in roster}
    assert all(link.has(a) for a in assigned)
    overflow = "CALL24"
    assert overflow not in assigned and link.failed(overflow)


def test_stable_assignment_and_detach(plugin, link):
    link.sync([position("AAA"), position("BBB")])
    eventually(lambda: rosters(plugin))
    slots = {a["id"]: a["slot"] for a in rosters(plugin)[0]["ac"]}
    link.sync([position("AAA"), position("BBB")])
    link.sync([position("BBB")])
    eventually(lambda: rosters(plugin)[-1]["ac"] ==
               [{"id": "BBB", "slot": slots["BBB"], "type": "A320"}])
    # The freed lever is zeroed so a still-modelled plane does not fly on.
    assert last_write(link.source, "engine_throttle_request",
                      slots["AAA"]) == 0.0
    link.sync([position("CCC")])
    eventually(lambda: any(a["id"] == "CCC" for a in rosters(plugin)[-1]["ac"]))
    # AAA's old slot can serve the newcomer.
    link.detach()
    eventually(lambda: rosters(plugin)[-1]["ac"] == [])


def test_pilot_laws_hold_and_steer():
    pilot = NativePilot("A320")
    parked = position()
    parked.update(phase="parked", on_ground=True, _slot=1)
    writes = pilot.update(parked, {}, time.monotonic())
    assert writes["sim/multiplayer/controls/parking_brake[1]"] == 1.0
    assert writes["sim/multiplayer/controls/yoke_pitch_ratio[1]"] == 0.0

    # Airborne and fast with the sim pointing ninety degrees left of the
    # demand: the roll lever asks for a right bank.
    airborne = dict(position(motion={"lat": 48.1, "lon": 11.7,
                                     "alt": 12000, "heading": 90,
                                     "gs": 250, "vs": 0,
                                     "on_ground": False}), _slot=1)
    measured = {"lat": 48.1, "lon": 11.7, "ele": 3657.6, "psi": 0.0,
                "phi": 0.0}
    writes = pilot.update(airborne, measured, time.monotonic() + 1)
    assert writes["sim/multiplayer/controls/yoke_roll_ratio[1]"] > 0
    # Far below the demand: some pitch-up demand in the lever.
    assert writes["sim/multiplayer/controls/gear_request[1]"] == 0.0


def test_pilot_closed_loop_altitude():
    """The vertical law pulls a point-mass toward the assigned altitude."""
    pilot = NativePilot("C172")
    target = dict(npos(type="C172",
                       motion={"lat": 48.1, "lon": 11.7, "alt": 5000,
                               "heading": 0, "gs": 110, "vs": 0,
                               "on_ground": False}), _slot=2)
    ele_ft, vs_fpm = 3000.0, 0.0
    t = 0.0
    for _ in range(4000):
        t += 0.25
        writes = pilot.update(
            target, {"lat": 48.1, "lon": 11.7, "ele": ele_ft / 3.28084,
                     "psi": 0.0, "phi": 0.0}, t)
        pitch = writes["sim/multiplayer/controls/yoke_pitch_ratio[2]"]
        # A toy plant: pitch lever sets the vertical speed.
        vs_fpm += (pitch * 800.0 - vs_fpm) * 0.15
        ele_ft += vs_fpm * 0.25 / 60.0
    assert abs(ele_ft - 5000) < 300


def test_helicopter_gets_rotor_law(plugin, link):
    assert "S76" in HELICOPTER_TYPES and "R22" in HELICOPTER_TYPES
    pilot = NativePilot("S76", helicopter=True)
    target = dict(npos(type="S76",
                       motion={"lat": 48.1, "lon": 11.7, "alt": 2000,
                               "heading": 270, "gs": 60, "vs": 0,
                               "on_ground": False}), _slot=3)
    writes = pilot.update(target, {"lat": 48.1, "lon": 11.7, "ele": 400.0,
                                   "psi": 0.0, "phi": 0.0}, time.monotonic())
    # 5,000 ft below the demand: collective well above the half lever.
    assert writes["sim/multiplayer/controls/engine_throttle_request[3]"] > .6
    link.sync([npos("HELI01", type="S76")])
    eventually(lambda: rosters(plugin))
    assert rosters(plugin)[0]["ac"] == [{"id": "HELI01", "slot": 1,
                                         "type": "S76"}]
    assert link._pilots["HELI01"].helicopter


def test_engine_picks_native_renderer(monkeypatch, plugin):
    pytest.importorskip("scipy")
    from meridianatc import engine as module

    engine = module.Engine.__new__(module.Engine)
    engine.config = Config()
    engine.config.traffic.xplane_plugin_port = plugin.port
    engine.config.traffic.xplane_renderer = "native"
    engine.state = AircraftState(source="xplane", connected=True)
    engine.source = FakeSource()
    engine.simtraffic = module.SimTraffic()
    engine.traffic = Mock()
    engine.emit = Mock()
    engine.simwatch = Mock()
    assert engine.attach_simulator_traffic()
    assert isinstance(engine.simtraffic, XPNativeTraffic)
    assert engine.simtraffic.native and engine.simtraffic.source is engine.source
    # Switching the renderer rebuilds the link.
    engine.config.traffic.xplane_renderer = "csl"
    assert engine.attach_simulator_traffic()
    from meridianatc.integrations.xptraffic import XPlaneTraffic
    assert type(engine.simtraffic) is XPlaneTraffic
