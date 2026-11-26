from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from meridianatc.navdata.build import _clean_facility
from meridianatc.navdata.db import NavDB, Station


def test_cld_is_noise_in_delivery_facility_names():
    assert _clean_facility("CLD", "Lisbon", "DEL") == "Lisbon"


def test_an_unpublished_position_is_named_for_the_controller_not_a_vdf_row():
    """ELLX publishes a VDF "Homer" row on the tower channel. It is a
    direction-finding service sharing the ATS frequency, not the name of
    Clearance Delivery."""
    db = NavDB.__new__(NavDB)
    db.airport = Mock(return_value=SimpleNamespace(
        ident="ELLX", spoken="Luxembourg", dialect="icao"))
    rows = [
        Station("ELLX", "INFO", 118.105, "Homer", "Homer"),
        Station("ELLX", "TWR", 118.105, "Luxembourg", "Luxembourg Tower"),
    ]
    db.stations = Mock(side_effect=lambda ident, position=None: [
        row for row in rows if position is None or row.position == position])

    invented = NavDB.station_on(db, "ELLX", 121.900, "DEL")

    assert invented is not None
    assert invented.facility == "Luxembourg"
    assert invented.callsign == "Luxembourg Clearance Delivery"


def test_msfs_vor_facility_decodes_a_localizer_frequency():
    """The simulator's VOR/ILS facility answer supplies what the static
    database cannot: the localizer frequency behind the runway's ident."""
    import struct

    from meridianatc.integrations.navaids import decode_vor
    raw = struct.pack("<ddIii64s", 50.91, 4.49, 110300000,
                      4, 1, b"Brussels 25L ILS\0")
    row = decode_vor(raw, "IBRU")
    assert row is not None
    assert row["freq_khz"] == pytest.approx(110300.0)
    assert row["type"] == "ILS"


def test_xplane_earth_nav_decodes_a_runway_localizer(tmp_path):
    """Row 4 is MHz x100, with ident, airport and runway in columns 8-10."""
    from meridianatc.integrations.xpnavaids import XPlaneNavaids
    root = tmp_path / "xp"
    (root / "Resources" / "plugins").mkdir(parents=True)
    navdir = root / "Resources" / "default data"
    navdir.mkdir(parents=True)
    (navdir / "earth_nav.dat").write_text(
        "I\n1100 Version\n"
        "4 50.91000000 4.49000000 175 11030 25 249.900 "
        "IBRU EBBR EB 25L ILS-cat-III\n", encoding="utf-8")
    row = XPlaneNavaids(root).navaid("IBRU", airport="EBBR", runway="25L")
    assert row is not None
    assert row["freq_khz"] == pytest.approx(110300.0)
    assert row["type"] == "ILS"


def test_the_recipe_navaid_falls_back_to_the_simulator_lookup():
    """A localizer absent from the static table is still tunable when the
    simulator knows the ident its runway published."""
    from types import SimpleNamespace

    from meridianatc.atc.approach_flight import _navaid

    rw = SimpleNamespace(
        le_ident="25L", he_ident="07R", le_ils="IBRU", he_ils="",
        le_ils_kind="ILS", he_ils_kind="")
    navdb = SimpleNamespace(
        runways=lambda ident: [rw],
        navaid=lambda ident, near=None: None,
        nearest_navaids=lambda *a, **k: [])
    brain = SimpleNamespace(navaid_lookup=lambda ident, **kw: {
        "ident": ident, "type": "ILS", "freq_khz": 110300.0,
        "lat": 50.91, "lon": 4.49})
    nav = _navaid(navdb, "EBBR", "25L", "ils", (50.9, 4.48), brain)
    assert nav is not None
    assert nav.freq_mhz == pytest.approx(110.3)
