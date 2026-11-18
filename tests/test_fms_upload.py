"""The route handed back to the simulator's own flight plan.

``use_route`` is the single place every plan lands -- the OFP, a hand-
loaded .pln, a typed string, the next chained leg -- and the box in the
cockpit should fly the same line. X-Plane takes a ``FPLN`` datagram of
v1100 text; MSFS takes a .pln path through ``SimConnect_FlightPlanLoad``.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

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


def test_the_declared_cycle_is_the_airac_in_force():
    """The 2026-11-17 case: X-Plane's navdata was cycle 2611 while the
    plan it was handed declared 2612, because the day-of-year count is
    not the AIRAC calendar -- each year's ``01`` is effective in late
    January. Anchored on 2601 (2026-01-22) instead."""
    import datetime

    from meridianatc.atc.route import _airac_cycle

    d = datetime.date
    assert _airac_cycle(d(2026, 1, 22)) == "2601"   # 2601 effective
    assert _airac_cycle(d(2026, 1, 21)) == "2513"   # still last year's
    assert _airac_cycle(d(2026, 10, 29)) == "2611"  # 2611 effective
    assert _airac_cycle(d(2026, 11, 17)) == "2611"  # the incident day
    assert _airac_cycle(d(2026, 11, 25)) == "2611"  # 2611's last day
    assert _airac_cycle(d(2026, 11, 26)) == "2612"  # 2612 effective
    assert _airac_cycle(d(2026, 12, 31)) == "2613"
    assert _airac_cycle(d(2027, 1, 21)) == "2701"   # the next year


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


# ------------------------------------------------------------------
# the readback must not re-adopt the upload -- or reset the flight over it
# ------------------------------------------------------------------

from meridianatc.engine.x_the_plan_the import _The_plan_the


def _engine(route=None):
    """A bare plan-mixin: ``use_route``/``begin_session`` are counters."""
    eng = _The_plan_the.__new__(_The_plan_the)
    eng.route = route
    eng.session = object()
    eng.source = object()
    eng.used, eng.begun = [], 0
    eng.use_route = eng.used.append
    eng.begin_session = lambda: setattr(eng, "begun", eng.begun + 1)
    eng.emit = lambda *a, **k: None
    eng.config = SimpleNamespace(
        flight=SimpleNamespace(departure="", destination="",
                               cruise_altitude_ft=0))
    # The navdata's airport lookup: LPPR/LECO/LEVX are fields, the
    # enroute idents are not.
    eng.navdb = SimpleNamespace(
        airport=lambda ident: object()
        if ident in ("LPPR", "LECO", "LEVX", "LEMD", "EGLL") else None)
    return eng


def _fms_entries(route):
    return [{"id": f.ident, "t": {"apt": 1}.get(f.kind, 512),
             "lat": f.lat, "lon": f.lon, "alt": f.alt_ft}
            for f in route.fixes]


def test_the_upload_echo_is_not_a_new_flight():
    route = _route()
    eng = _engine(route)
    # The box answers with the plan this engine uploaded to it.
    eng._use_fms_plan(_fms_entries(route))
    assert eng.used == [] and eng.begun == 0
    # And again every poll after: the fingerprint is settled.
    eng._use_fms_plan(_fms_entries(route))
    assert eng.begun == 0


def test_sequencing_past_fixes_keeps_the_session():
    route = _route()
    eng = _engine(route)
    entries = _fms_entries(route)
    eng._use_fms_plan(entries)
    # The FMS consumes the head of the plan as it is flown.
    eng._use_fms_plan(entries[2:])
    assert eng.begun == 0 and eng.route is route


def test_a_genuinely_new_plan_is_a_new_flight():
    eng = _engine(_route())
    eng._sim_plan_fp = ()
    other = Route(fixes=(RouteFix("LEMD", 40.47, -3.57, kind="apt"),
                       RouteFix("EGLL", 51.47, -0.46, kind="apt")),
                  origin="LEMD", destination="EGLL", source="sim")
    eng._use_fms_plan(_fms_entries(other))
    assert eng.begun == 1
    # The adopted route is a fresh object, not `other`: it is rebuilt
    # from the FMS entries.
    assert len(eng.used) == 1
    assert [f.ident for f in eng.used[0].fixes] == ["LEMD", "EGLL"]


def test_untyped_ends_still_name_the_destination():
    """The Baron flew to LEVX with the box calling both ends waypoints:
    an ident the navdata knows as an airport is the destination,
    whatever the entry type said."""
    eng = _engine(_route())
    eng._sim_plan_fp = ()
    entries = [{"id": "LPPR", "t": 512, "lat": 41.24, "lon": -8.68,
                "alt": 0},
               {"id": "XAPIM", "t": 512, "lat": 41.3, "lon": -8.7,
                "alt": 0},
               {"id": "LEVX", "t": 512, "lat": 42.23, "lon": -8.63,
                "alt": 0}]
    eng._use_fms_plan(entries)
    assert eng.used[0].origin == "LPPR"
    assert eng.used[0].destination == "LEVX"


def test_an_adopted_plan_moves_the_filed_destination():
    """The LEVX case: the box's plan ends at Vigo while the flight was
    filed to A Coruna. The adopted route's destination is the flight's
    word for where it is going -- the clearance's \"cleared to\" is
    built from it, not from the stale setting."""
    eng = _engine(_route())
    eng._sim_plan_fp = ()
    eng.config.flight.destination = "LECO"
    entries = [{"id": "LPPR", "t": 1, "lat": 41.24, "lon": -8.68,
                "alt": 0},
               {"id": "BEKIN", "t": 512, "lat": 42.0, "lon": -8.6,
                "alt": 0},
               {"id": "LEVX", "t": 1, "lat": 42.23, "lon": -8.63,
                "alt": 0}]
    eng._use_fms_plan(entries)
    assert eng.config.flight.destination == "LEVX"
    assert eng.config.flight.departure == "LPPR"


def test_the_msfs_upload_echo_is_skipped(tmp_path):
    import os
    eng = _engine(_route())
    path = str(tmp_path / "meridianatc_plan.pln")
    with open(path, "w", encoding="utf-8") as f:
        f.write(to_pln(_route()))
    eng.source = type("S", (), {"_pln_uploaded": os.path.abspath(path)})()
    eng.load_route_file = lambda p: (_ for _ in ()).throw(
        AssertionError("the echo must not be adopted"))
    eng._use_pln_plan(path)
    # Marked seen so later polls do not re-examine it.
    assert eng._sim_plan_path == path


def test_msfs_load_records_the_upload_path(tmp_path, monkeypatch):
    import os
    source = SimConnectSource()
    source._connected = True
    source._sim = _StubSim()
    monkeypatch.setenv("TEMP", str(tmp_path))
    monkeypatch.setenv("TMP", str(tmp_path))
    assert source.load_fms_plan(_route())
    import tempfile
    assert source._pln_uploaded == os.path.abspath(
        os.path.join(tempfile.gettempdir(), "meridianatc_plan.pln"))
