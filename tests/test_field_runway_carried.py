"""The departure runway the user left on is not the next field's.

set_field("LIRF", departure_runway="06") used to install "06" -- ELLX's
runway -- as Fiumicino's departure strip and lock it, so the invented
traffic was cleared onto a runway the field does not have (heard as
"Flying Bulgaria four five zero, runway zero six, cleared for takeoff"
on Fiumicino Tower). Runways the field does not publish are dropped;
the wind picks instead.
"""

from __future__ import annotations

from unittest.mock import Mock

from meridianatc.atc.traffic.core import TrafficInjector
from meridianatc.navdata.db import Airport, Runway


def _end(le: str, he: str, length_ft: float = 10000.0) -> Runway:
    return Runway(
        ident="X", le_ident=le, he_ident=he, length_ft=length_ft,
        width_ft=150.0, surface="ASP", lighted=True, closed=False,
        le_lat=41.8, le_lon=12.2, le_heading=70.0,
        he_lat=41.81, he_lon=12.27, he_heading=250.0,
        le_elev_ft=7.0, he_elev_ft=5.0, source="db",
        le_ils="", he_ils="", le_ils_kind="", he_ils_kind="",
        le_displaced_ft=0.0, he_displaced_ft=0.0,
        le_closed=False, he_closed=False)


def _navdb() -> Mock:
    """A navdb with ELLX's 06/24 and LIRF's 07/25-style strips."""
    navdb = Mock()
    airport = Airport(ident="LIRF", icao="LIRF", iata="FCO",
                      name="Fiumicino", spoken="Fiumicino", type="large",
                      lat=41.8, lon=12.25, elev_ft=13, country="IT",
                      region="", municipality="", scheduled=1, size_rank=5)
    navdb.airport.return_value = airport
    navdb.runways.side_effect = lambda ident: (
        [_end("07", "25"), _end("16L", "34R"), _end("16R", "34L")]
        if ident == "LIRF" else [_end("06", "24")])
    navdb.stations.return_value = []
    navdb.station_for.return_value = None
    navdb.best_runway.return_value = ("07", _end("07", "25"))
    return navdb


def test_another_fields_runway_is_not_inherited():
    injector = TrafficInjector(_navdb(), seed=1)
    injector.set_field("ELLX", departure_runway="06", arrival_runway="06")
    assert injector.field.departure_runway == "06"

    injector.set_field("LIRF", departure_runway="06", arrival_runway="06")
    assert injector.field.departure_runway != "06"
    assert injector.field.departure_runway == "07"
    assert injector.field.arrival_runway != "06"


def test_the_fields_own_runway_still_takes():
    injector = TrafficInjector(_navdb(), seed=1)
    injector.set_field("LIRF", departure_runway="07", arrival_runway="25")
    assert injector.field.departure_runway == "07"
    assert injector.field.arrival_runway == "25"
