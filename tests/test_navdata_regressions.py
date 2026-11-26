from types import SimpleNamespace
from unittest.mock import Mock

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
