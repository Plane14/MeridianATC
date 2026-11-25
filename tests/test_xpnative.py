"""The native-fleet renderer: roster wire format and the pilots behind it."""

from __future__ import annotations

import threading
import time
from unittest.mock import Mock

import pytest

from meridianatc import xplane_plugin as xp
from meridianatc.config import Config
from meridianatc.integrations.xpnative import (
    HELICOPTER_TYPES,
    NativePilot,
    XPHybridTraffic,
    XPNativeTraffic,
)
from meridianatc.sim.state import AircraftState
from tests.test_xptraffic import FakePlugin, eventually, position


class NativePlugin(FakePlugin):
    """Same UDP peer, able to drive the AI slots -- and reporting in its
    status which rostered ids actually became planes."""

    def __init__(self):
        super().__init__()
        self.planes = []

    def run(self):
        # Track the ids of the last roster as the planes the sim carries.
        while not self.stop.is_set():
            try:
                wire, self.peer = self.socket.recvfrom(65536)
                packet = __import__("json").loads(wire)
                self.messages.append(packet)
                if packet["t"] == "native":
                    self.planes = [a["id"] for a in packet["ac"]]
                if self.reply and packet["t"] in ("hello", "traffic"):
                    self.status()
            except TimeoutError:
                continue
            except OSError:
                break

    def status(self, **overrides):
        overrides.setdefault("native_cap", 1)
        overrides.setdefault("native", len(self.planes))
        overrides.setdefault("native_ac", list(self.planes))
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
    link.SLOT_KEEP_S = 0.0   # the grace window is covered separately
    assert link.available and link.drawing and link.count == 0
    link.sync([position()])
    eventually(lambda: len(rosters(plugin)) == 1)
    assert rosters(plugin)[0] == {"v": 1, "t": "native",
                                  "ac": [{"id": "DLH4AB", "slot": 1,
                                          "type": "A320", "op": "DLH",
                                          "lat": 48.1,
                                          "lon": 11.7, "alt": 12000}]}
    # The callsign is not confirmed until the plugin reports the plane.
    assert not link.has("DLH4AB")
    plugin.status()
    eventually(lambda: link.has("DLH4AB"))
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
    # Confirmed once the plugin reports the rostered ids as real planes.
    plugin.status()
    link.sync(many)
    assert all(link.has(a) for a in assigned)
    overflow = "CALL24"
    assert overflow not in assigned and link.failed(overflow)


def test_stable_assignment_and_detach(plugin, link):
    link.SLOT_KEEP_S = 0.0   # the grace window is covered separately
    link.sync([position("AAA"), position("BBB")])
    eventually(lambda: rosters(plugin))
    slots = {a["id"]: a["slot"] for a in rosters(plugin)[0]["ac"]}
    link.sync([position("AAA"), position("BBB")])
    link.sync([position("BBB")])
    eventually(lambda: len(rosters(plugin)) >= 2)
    last = rosters(plugin)[-1]["ac"]
    assert [a["id"] for a in last] == ["BBB"]
    assert last[0]["slot"] == slots["BBB"]
    # The freed lever is zeroed so a still-modelled plane does not fly on.
    assert last_write(link.source, "engine_throttle_request",
                      slots["AAA"]) == 0.0
    link.sync([position("CCC")])
    eventually(lambda: any(a["id"] == "CCC" for a in rosters(plugin)[-1]["ac"]))
    # AAA's old slot can serve the newcomer.
    link.detach()
    eventually(lambda: rosters(plugin)[-1]["ac"] == [])


def test_hybrid_draws_the_overflow_while_native_flies(plugin):
    source = FakeSource()
    link = XPHybridTraffic(port=plugin.port, source=source, settle_s=0)
    try:
        assert link.attach()
        many = [npos(f"CALL{i:02d}", type="C172") for i in range(21)]
        link.sync(many)
        eventually(lambda: rosters(plugin))
        rostered = len(rosters(plugin)[-1]["ac"])
        assert rostered == XPNativeTraffic.MAX_SLOTS
        plugin.status(native=rostered)
        time.sleep(0.1)
        link.on_frame()
        assert link._status.get("native") == rostered
        # AI-plane ownership is exclusive only for the multiplayer/TCAS
        # datarefs. XPMP2's visible CSL instances do not need it, so the
        # two overflow callsigns are still sent and drawn.
        traffic = [m for m in plugin.messages if m["t"] == "traffic"]
        assert traffic
        overflow = traffic[-1]["ac"]
        assert {a["id"] for a in overflow} == {"CALL19", "CALL20"}
    finally:
        link.detach()


def test_exclusive_flag_on_roster(plugin, link):
    """traffic.xplane_fill != "sim" leaves a spare slot empty rather than
    letting the simulator's own AI spawn into it."""
    link.exclusive = True
    link.sync([position()])
    eventually(lambda: rosters(plugin))
    assert rosters(plugin)[0]["exclusive"] is True
    # Back to shared: the flag clears on the next roster rather than
    # sticking at the last value sent.
    link.exclusive = False
    link.sync([position(), position("AAA")])
    eventually(lambda: "exclusive" not in rosters(plugin)[-1])
    # An unchanged roster still resends when the flag flips alone.
    link.exclusive = True
    link.sync([position(), position("AAA")])
    eventually(lambda: rosters(plugin)[-1].get("exclusive") is True)


def test_departed_callsign_keeps_its_slot(plugin, link):
    link.sync([position()])
    eventually(lambda: rosters(plugin))
    # A callsign that stops being offered keeps its plane for the grace
    # window, so a feed's quiet spell does not blink it out and back in.
    link.sync([])
    assert any(a["id"] == "DLH4AB" for a in rosters(plugin)[-1]["ac"])
    # Past the window the slot is freed and the levers go neutral.
    link._departed["DLH4AB"] = 0.0
    link.sync([])
    eventually(lambda: rosters(plugin)[-1]["ac"] == [])
    assert last_write(link.source, "engine_throttle_request", 1) == 0.0


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
    entry = rosters(plugin)[0]["ac"][0]
    assert (entry["id"], entry["slot"], entry["type"]) == ("HELI01", 1, "S76")
    assert link._pilots["HELI01"].helicopter


def test_engine_picks_native_renderer(monkeypatch, plugin):
    pytest.importorskip("scipy")
    from meridianatc import engine as module

    monkeypatch.setattr(xp, "plugin_ports", lambda: [])
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


def test_slot_feedback_uses_xplanes_real_elevation_dataref(plugin, link):
    link.sync([position()])
    eventually(lambda: rosters(plugin))
    assert link.source.subscriptions["xpn1:ele"].endswith("plane1_el")
    assert not link.source.subscriptions["xpn1:ele"].endswith("plane1_ele")


def test_native_toe_brakes_are_released():
    pilot = NativePilot("C172")
    target = dict(npos(type="C172", motion={"lat": 48, "lon": 11, "alt": 0,
                       "heading": 0, "gs": 1, "on_ground": True}), _slot=2)
    measured = {"lat": 48, "lon": 11, "ele": 0, "psi": 0,
                "v_x": 5.0, "v_y": 0.0, "v_z": 0.0}
    first = pilot.update(target, measured, 1.0)
    assert first["sim/multiplayer/controls/l_brake_add[2]"] == 1.0
    target["motion"]["gs"] = 20
    second = pilot.update(target, measured, 2.0)
    assert second["sim/multiplayer/controls/l_brake_add[2]"] == 0.0
    assert second["sim/multiplayer/controls/r_brake_add[2]"] == 0.0


def test_malformed_native_position_is_skipped_not_sent(plugin, link):
    broken = position("BROKEN")
    del broken["altitude_ft"]
    link.sync([broken])
    threading.Event().wait(.05)
    assert all(a.get("id") != "BROKEN" for r in rosters(plugin) for a in r["ac"])
