"""The ambient transits: spare X-Plane slots flown by us, not the sim."""

from __future__ import annotations

from unittest.mock import Mock

from meridianatc.atc.traffic.core import TrafficInjector
from meridianatc.navdata.db import Airport


def _navdb():
    navdb = Mock()
    navdb.airport.return_value = None
    navdb.runways.return_value = []
    navdb.stations.return_value = []
    navdb.nearest_airports.return_value = [
        (Airport(ident="EGKA", icao="EGKA", iata="ESH", name="Shoreham",
                 spoken="Shoreham", type="small", lat=50.835, lon=-0.297,
                 elev_ft=7, country="GB", region="", municipality="",
                 scheduled=0, size_rank=1), 18.0),
        (Airport(ident="EGKB", icao="EGKB", iata="BQH", name="Biggin Hill",
                 spoken="Biggin Hill", type="medium", lat=51.330, lon=0.032,
                 elev_ft=598, country="GB", region="", municipality="",
                 scheduled=1, size_rank=2), 30.0),
        (Airport(ident="EGMD", icao="EGMD", iata="LYX", name="Lydd",
                 spoken="Lydd", type="small", lat=50.956, lon=0.939,
                 elev_ft=13, country="GB", region="", municipality="",
                 scheduled=0, size_rank=1), 42.0),
    ]
    return navdb


def _injector():
    injector = TrafficInjector(_navdb(), seed=7)
    injector.ambient_fill = True
    injector.field.latitude, injector.field.longitude = 51.47, -0.45
    injector.field.elevation_ft = 83.0
    injector.field.ident = "EGLL"
    injector.field.size_rank = 5
    injector.radius_nm = 100.0
    return injector


def test_pool_fills_the_spare_slots():
    injector = _injector()
    positions = injector.ambient_positions(now=1000.0, room=19)
    assert len(positions) == 19
    callsigns = {p["callsign"] for p in positions}
    assert len(callsigns) == len(positions)
    # The snapshot carries what the native renderer asks of a plane:
    # callsign, type and where it is pointed, with the motion ahead of it.
    for p in positions:
        assert p["type"] and p["phase"] == "enroute"
        assert not p["on_ground"] and not p["static"]
        assert p["source"] == "ambient"
        motion = p["motion"]
        assert motion["alt"] > 0 and motion["gs"] > 0
        assert not motion["on_ground"]
    # The pool's callsigns count against the field's book, so a flight on
    # the frequency is never handed a name the sky already wears.
    assert callsigns <= injector._used_callsigns


def test_transits_fly_their_leg():
    injector = _injector()
    first = {p["callsign"]: p for p in
             injector.ambient_positions(now=1000.0, room=19)}
    later = {p["callsign"]: p for p in
             injector.ambient_positions(now=1120.0, room=19)}
    # Same aeroplanes, further on: nobody blinked out or stood still.
    assert set(first) <= set(later)
    assert all(first[c]["lat"] != later[c]["lat"]
               or first[c]["lon"] != later[c]["lon"] for c in first)
    # A mix of legs: some already established, others working a climb
    # or a letdown -- and the IFR/VFR split the mix promises.
    states = {t.state for t in injector._ambient}
    assert states <= {"enroute", "climb", "descend"} and len(states) > 1
    assert any(t.ifr for t in injector._ambient)
    assert any(not t.ifr for t in injector._ambient)


def test_room_is_never_overfilled():
    injector = _injector()
    positions = injector.ambient_positions(now=1000.0, room=5)
    assert len(positions) == 5
    # Less room later: the ones already flying keep flying, on their
    # callsigns, and only what fits is offered.
    positions = injector.ambient_positions(now=1010.0, room=3)
    assert len(positions) == 3


def test_opt_in_only():
    injector = _injector()
    injector.ambient_fill = False
    assert injector.ambient_positions(now=1000.0, room=19) == []
    injector.ambient_fill = True
    assert injector.ambient_positions(now=1000.0, room=0) == []
