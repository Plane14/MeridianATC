"""The simulator's own airspace (navdata.simspace).

X-Plane's OpenAir files, the MSFS boundaries the panel feeds back, and
how both reach ``NavDB.airspace_volumes``.
"""

from pathlib import Path
from types import SimpleNamespace

import pytest

from meridianatc.navdata import simspace
from meridianatc.navdata.db import NavDB

OPENAIR = """\
* a comment, like the real files carry
AC D
AN WILLIAM P GWINN
AL GND
AH 2500 MSL
DP 26:50:47 N 080:16:49 W
DP 26:51:07 N 080:23:03 W
DP 26:52:07 N 080:23:59 W

AC CTR
AN PRAGUE CTR
AL 0 MSL
AH 3500 MSL
DP 50:00:00 N 014:00:00 E
DP 50:30:00 N 014:00:00 E
DP 50:30:00 N 015:00:00 E
DP 50:00:00 N 015:00:00 E

AC R
AN RESTRICTED 10
AL 0 MSL
AH FL100
DP 48:00:00 N 012:00:00 E
DP 48:30:00 N 012:00:00 E
DP 48:30:00 N 013:00:00 E
DP 48:00:00 N 013:00:00 E

AC A
AN CIRCULAR
AL 0 MSL
AH 4500 MSL
V X=49:00:00 N 012:30:00 E
DC 5
"""


@pytest.fixture(autouse=True)
def detached():
    """No test leaves a simulator attached for the next one."""
    simspace.detach()
    yield
    simspace.detach()


def test_openair_polygon_classes_and_limits():
    found = {v.name: v for v in simspace.parse_openair(OPENAIR, "xp0")}
    gwinn = found["WILLIAM P GWINN"]
    assert gwinn.cls == "D"
    assert gwinn.shape == "polygon"
    assert gwinn.lower_ref == "SFC"
    assert (gwinn.upper_ft, gwinn.upper_ref) == (2500, "MSL")
    assert gwinn.contains(26.856, -80.355, 1500)
    assert not gwinn.contains(26.856, -80.355, 3000)
    # A control zone counts as the tower's class D.
    assert found["PRAGUE CTR"].cls == "D"
    # Restricted and prohibited keep their own letters for ENTRY_RULES.
    assert found["RESTRICTED 10"].cls == "R"
    assert found["RESTRICTED 10"].upper_ft == 10000  # FL100
    # A V X centre and DC radius make a circle, not a polygon.
    assert found["CIRCULAR"].shape == "circle"
    assert found["CIRCULAR"].radius_nm == pytest.approx(5.0)


def test_openair_bad_record_keeps_the_rest():
    text = "AC D\nAN BROKEN\nDP not-a-point\n\n" + OPENAIR
    names = {v.name for v in simspace.parse_openair(text, "xp0")}
    assert "WILLIAM P GWINN" in names


def _boundary(**kw):
    base = {
        "id": 42, "name": "TEST B", "type": 3,
        "minAlt": 0.0, "maxAlt": 5486.4,      # 18,000 ft in metres
        "minAltType": 1, "maxAltType": 1,
        "vectors": [
            {"type": 1, "lat": 40.0, "lon": -105.0},
            {"type": 2, "lat": 41.0, "lon": -105.0},
            {"type": 2, "lat": 41.0, "lon": -104.0},
            {"type": 2, "lat": 40.0, "lon": -104.0},
        ],
    }
    base.update(kw)
    return base


def test_boundary_polygon():
    v = simspace.boundary_volumes(_boundary())
    assert len(v) == 1 and v[0].cls == "B" and v[0].shape == "polygon"
    assert v[0].lower_ft == 0 and v[0].upper_ft == pytest.approx(18000, abs=2)
    assert v[0].contains(40.5, -104.5, 10000)


def test_boundary_circle_and_unlimited():
    v = simspace.boundary_volumes(_boundary(
        type=5, minAltType=2, minAlt=304.8, maxAltType=3,
        vectors=[{"type": 3, "lat": 50.1, "lon": 14.26, "originId": 1},
                 {"type": 6, "lat": 0, "lon": 0, "radius": 9260.0,
                  "originId": 1}]))
    assert len(v) == 1 and v[0].shape == "circle" and v[0].cls == "D"
    # The circle centres on the origin its originId names, not its own
    # lat/lon (zeroed above to prove it).
    assert (v[0].lat, v[0].lon) == (50.1, 14.26)
    assert v[0].radius_nm == pytest.approx(5.0, abs=0.01)
    assert v[0].lower_ref == "AGL" and v[0].lower_ft == pytest.approx(1000, abs=2)
    assert v[0].upper_ft == simspace.UNLIMITED_FT
    assert v[0].contains(50.1, 14.26, 2000, ground_ft=800)


def test_boundary_two_shapes_become_two_volumes():
    v = simspace.boundary_volumes(_boundary(vectors=[
        {"type": 3, "lat": 40.0, "lon": -104.0, "originId": 1},
        {"type": 6, "lat": 40.0, "lon": -104.0, "radius": 1852.0,
         "originId": 1},
        {"type": 3, "lat": 41.0, "lon": -104.0, "originId": 2},
        {"type": 6, "lat": 41.0, "lon": -104.0, "radius": 1852.0,
         "originId": 2},
    ]))
    assert len(v) == 2
    assert [vol.lat for vol in v] == [40.0, 41.0]


def test_boundary_type_mapping():
    kinds = {15: "R", 16: "P", 19: "Q", 14: "E", 4: "C", 9: "D", 22: "E"}
    for sim_type, cls in kinds.items():
        v = simspace.boundary_volumes(_boundary(type=sim_type))
        assert v[0].cls == cls, sim_type


def test_feed_msfs_diffs():
    simspace.attach("manual")          # not a sim: feeds are ignored
    assert simspace.feed_msfs({"boundaries": [_boundary()]})["volumes"] == 0

    simspace.attach("simconnect")
    # The first feed after a fresh attach asks for the panel's whole
    # set again -- the store was emptied and the panel keeps its own.
    first = simspace.feed_msfs({"boundaries": [_boundary()]})
    assert first["resync"] is True
    assert simspace.feed_msfs({"boundaries": [_boundary()],
                               "reset": True})["volumes"] == 1
    assert len(simspace.volumes()) == 1
    # The same facility again replaces rather than duplicating.
    simspace.feed_msfs({"boundaries": [_boundary(type=5)]})
    assert len(simspace.volumes()) == 1
    assert simspace.volumes()[0].cls == "D"
    simspace.feed_msfs({"remove": [42]})
    assert simspace.volumes() == []
    simspace.feed_msfs({"boundaries": [_boundary()], "reset": True})
    assert len(simspace.volumes()) == 1


def test_feed_msfs_resync_after_reattach():
    simspace.attach("simconnect")
    simspace.feed_msfs({"boundaries": [_boundary()], "reset": True})
    assert len(simspace.volumes()) == 1
    # The link went down and back: the panel's cached boundaries were
    # cleared here but not there, so the next diff asks for them all.
    simspace.attach("simconnect")
    answer = simspace.feed_msfs({"boundaries": [_boundary(id=7)]})
    assert answer["resync"] is True
    simspace.feed_msfs({"boundaries": [_boundary()],
                        "reset": True})
    assert len(simspace.volumes()) == 1
    assert simspace.feed_msfs({"boundaries": [_boundary()]})[
        "resync"] is False


def test_xplane_files_prefers_custom_data(tmp_path):
    root = tmp_path / "X-Plane 12"
    (root / "Custom Data" / "airspaces").mkdir(parents=True)
    custom = root / "Custom Data" / "airspaces" / "airspace.txt"
    custom.write_text("x")
    default = root / "Resources" / "default data" / "airspaces"
    default.mkdir(parents=True)
    (default / "airspace.txt").write_text("x")
    assert simspace.xplane_files([root]) == [custom]

    bare = tmp_path / "bare"
    (bare / "Resources" / "default data" / "airspaces").mkdir(parents=True)
    stock = bare / "Resources" / "default data" / "airspaces" / "a.txt"
    stock.write_text("x")
    assert simspace.xplane_files([bare]) == [stock]


def test_load_xplane_and_attach(tmp_path):
    path = tmp_path / "airspace.txt"
    path.write_text(OPENAIR, encoding="utf-8")
    found = simspace.load_xplane([path])
    assert len(found) == 4
    assert all(v.source == "xp0" for v in found)
    # Distinct ids across files: the second file does not reuse them.
    path2 = tmp_path / "more.txt"
    path2.write_text(OPENAIR, encoding="utf-8")
    more = simspace.load_xplane([path, path2])
    assert len({v.id for v in more}) == 8


def test_navdb_merges_and_attributes(monkeypatch, tmp_path):
    import sqlite3

    # NavDB opens lazily but insists the file is there: an empty one is
    # enough, nothing here queries it.
    sqlite3.connect(tmp_path / "nav.sqlite").close()
    db = NavDB(tmp_path / "nav.sqlite")
    db._airspace = []
    airport = SimpleNamespace(ident="KTST")
    monkeypatch.setattr(db, "nearest_airport", lambda *a, **k: airport)

    simspace.attach("simconnect")
    simspace.feed_msfs({"boundaries": [_boundary()]})
    merged = db.airspace_volumes()
    assert len(merged) == 1
    assert db.volume_airport(merged[0]) == "KTST"
    # ...and cached: a second look does not ask again.
    monkeypatch.setattr(db, "nearest_airport",
                        lambda *a, **k: pytest.fail("not cached"))
    assert db.volume_airport(merged[0]) == "KTST"

    simspace.detach()
    assert db.airspace_volumes() == []


def test_parse_real_xplane_file_if_present(tmp_path):
    """The format claims are checked against a slice of the real file."""
    sample = Path("G:/X-Plane 12/Custom Data/airspaces/airspace.txt")
    if not sample.exists():
        pytest.skip("no X-Plane install here")
    text = sample.read_text(encoding="utf-8", errors="replace")[:400_000]
    found = simspace.parse_openair(text, "xp0")
    assert len(found) > 10
    assert all(v.shape in ("polygon", "circle") for v in found)
