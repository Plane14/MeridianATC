"""The route handed back to the simulator's own flight plan.

``use_route`` is the single place every plan lands -- the OFP, a hand-
loaded .pln, a typed string, the next chained leg -- and the box in the
cockpit should fly the same line. X-Plane takes a ``FPLN`` datagram of
v1100 text; MSFS takes a .pln path through ``SimConnect_FlightPlanLoad``.
"""

from __future__ import annotations

import time

from meridianatc.atc.route import Route, RouteFix, parse_pln, to_fms, to_pln
from meridianatc.sim.source import SimConnectSource
from meridianatc.sim.xplane import XPlaneSource


def _route(**over) -> Route:
    fixes = (
        RouteFix("LPPR", 41.2480, -8.6814, alt_ft=230, kind="apt"),
        RouteFix("TOC", 41.5, -8.5, stage="CLB"),
        RouteFix("SIDFIX", 41.30, -8.70, via="PORTO1A",
                 on_procedure=True),
        RouteFix("INBOM", 41.0, -8.0, alt_ft=35000, via="UN852",
                 kind="wpt"),
        RouteFix("LPPT", 38.7742, -9.1342, kind="apt"),
    )
    args = dict(fixes=fixes, origin="LPPR", destination="LPPT",
                sid="PORTO1A", sid_runway="17", sid_transition="PRT",
                star="XAMTA3B", star_transition="XAMTA", star_runway="03",
                cruise_ft=35000, source="simbrief")
    args.update(over)
    return Route(**args)


def test_to_fms_writes_the_1100_layout():
    text = to_fms(_route())
    lines = text.split("\n")
    assert lines[:3] == ["I", "1100 Version", lines[2]]
    assert lines[2].startswith("CYCLE ")
    assert "ADEP LPPR" in lines
    assert "ADES LPPT" in lines
    # The filed procedure rides in the blocks, not the waypoint rows.
    assert "DEPRWY RW17" in lines
    assert "SID PORTO1A" in lines
    assert "SIDTRANS PRT" in lines
    assert "DESRWY RW03" in lines
    assert "STAR XAMTA3B" in lines
    assert "STARTRANS XAMTA" in lines
    # The en-route rows: both airports, the airway fix -- but not the
    # TOC marker or the SID's own leg, which the sim expands itself.
    assert "NUMENR 3" in lines
    rows = lines[lines.index("NUMENR 3") + 1:]
    assert rows[0] == "1 LPPR ADEP 230.000000 41.248000 -8.681400"
    assert rows[1] == "11 INBOM UN852 35000.000000 41.000000 -8.000000"
    assert rows[2] == "1 LPPT ADES 0.000000 38.774200 -9.134200"
    assert not any("SIDFIX" in row or "TOC" in row for row in rows)


def test_to_pln_round_trips_through_its_own_parser():
    plan = parse_pln(to_pln(_route()))
    assert plan.origin == "LPPR" and plan.destination == "LPPT"
    assert plan.cruise_ft == 35000
    assert plan.sid == "PORTO1A" and plan.departure_runway == "17"
    assert plan.star == "XAMTA3B" and plan.arrival_runway == "03"
    # Every fix except the TOC marker is a waypoint with a position.
    idents = [ident for ident, _r, _k, _w, _a in plan.waypoints]
    assert idents == ["LPPR", "SIDFIX", "INBOM", "LPPT"]
    assert all(where for _i, _r, _k, where, _a in plan.waypoints)


def test_empty_routes_write_no_plan():
    assert to_fms(None) == "" and to_fms(Route()) == ""
    assert to_pln(None) == "" and to_pln(Route()) == ""


def test_xplane_load_fms_plan_sends_an_fpln_datagram():
    link = XPlaneSource()
    # A connected-looking source without a socket: _packet is
    # monkeypatched to catch the wire bytes, so only the flags matter.
    link._socket = True
    link._answered.set()
    link._last_packet = time.monotonic()
    sent = []
    link._packet = sent.append
    assert link.load_fms_plan(_route())
    assert len(sent) == 1
    wire = sent[0]
    assert wire[:5] == b"FPLN\x00"
    # The reference client's 1400-byte pad, and the whole plan inside.
    assert len(wire) == 5 + 1400
    plan = wire[5:].decode().split("\n")
    assert "ADEP LPPR" in plan and "ADES LPPT" in plan
    assert any(line.startswith("NUMENR") for line in plan)


def test_xplane_load_fms_plan_declines_when_disconnected():
    link = XPlaneSource()
    assert link.load_fms_plan(_route()) is False


class _StubDll:
    def __init__(self):
        self.calls = []

    def FlightPlanLoad(self, handle, path):
        self.calls.append((handle, path))
        return 0


class _StubSim:
    def __init__(self):
        self.dll = _StubDll()
        self.hSimConnect = 7

    def IsHR(self, hr, code):
        return hr == 0


def test_msfs_load_fms_plan_writes_a_pln_and_loads_it(tmp_path,
                                                      monkeypatch):
    source = SimConnectSource()
    assert source.load_fms_plan(_route()) is False   # never connected
    source._connected = True
    source._sim = _StubSim()
    monkeypatch.setenv("TEMP", str(tmp_path))
    monkeypatch.setenv("TMP", str(tmp_path))
    assert source.load_fms_plan(_route())
    handle, path = source._sim.dll.calls[0]
    assert handle == 7
    import os
    with open(os.fsdecode(path), encoding="utf-8") as f:
        text = f.read()
    plan = parse_pln(text)
    assert plan.origin == "LPPR" and plan.destination == "LPPT"
    assert plan.cruise_ft == 35000


def test_the_stub_sources_have_no_box_to_load():
    from meridianatc.sim.source import ManualSource, ReplaySource

    assert ManualSource().load_fms_plan(_route()) is False
    assert ReplaySource([]).load_fms_plan(_route()) is False
