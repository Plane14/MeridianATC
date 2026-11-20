"""The FO's taxi follower through sharp apron corners.

Where the real flights bent it: a taxiway turn arrived at with taxi
speed becomes the swing off the pavement -- the wide turn into the
airport's own buildings the logs showed at LPPR. The plant is the same
point-mass the approach tests fly; what is asserted is the ground track
and the speed the corner was made at, not which events left.
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from flight_plant import Plant, PlantDirector

from meridianatc.atc import auto_nav
from meridianatc.atc import flying as fly
from meridianatc.navdata.build import project

BASE_LAT, BASE_LON = 42.23, -8.63
M_TO_NM = 1.0 / 1852.0


class _NavDB:
    """One airport, no runways the taxi follower needs."""

    def __init__(self, airport):
        self.airport = airport

    def runways(self, ident):
        return []

    def nearest_airports(self, *a, **k):
        return []

    def home_airport(self, lat, lon):
        return self.airport

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return lambda *a, **k: None


def _taxi_fo(plant, path, type_code="C172"):
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import AutonomousFlight
    from meridianatc.atc.session import FlightSession
    from meridianatc.config import Config
    from meridianatc.navdata.db import Airport

    airport = Airport(
        ident="LFXX", icao="LFXX", iata="", name="Test", spoken="Test",
        type="medium_airport", lat=BASE_LAT, lon=BASE_LON, elev_ft=0.0,
        country="FR", region="", municipality="", scheduled=0, size_rank=1)
    session = FlightSession(aircraft=SimpleNamespace(
        type_code=type_code, title="", is_heavy=False, spoken="",
        language="en", callsign=SimpleNamespace(spoken="", written="")))
    fo = AutonomousFlight(Config(), _NavDB(airport), None, session,
                          Mock(), Mock(), plant.state, Mock())
    fo._weather_reactions = False
    fo._manage_radio = False
    fo._dir = PlantDirector(plant)
    fo._perf = fly.performance_for(type_code, light=True)
    plant.type_code = type_code
    session.taxi_path = list(path)
    return fo, session


def _l_path(first_m=400.0, turn_deg=90.0, second_m=300.0):
    """An L: east for ``first_m``, a corner, then ``second_m`` more."""
    vertex = project(BASE_LAT, BASE_LON, 90.0, first_m * M_TO_NM)
    end = project(vertex[0], vertex[1], 90.0 + turn_deg, second_m * M_TO_NM)
    return [(BASE_LAT, BASE_LON), vertex, end]


def _u_path():
    """Two right angles, like an apron's out-and-around."""
    a = project(BASE_LAT, BASE_LON, 90.0, 200.0 * M_TO_NM)
    b = project(a[0], a[1], 180.0, 80.0 * M_TO_NM)
    c = project(b[0], b[1], 270.0, 200.0 * M_TO_NM)
    return [(BASE_LAT, BASE_LON), a, b, c]


def _off_path_m(lat, lon, path):
    """How far the aeroplane is from the route, whichever leg is near."""
    best = None
    for a, b in zip(path, path[1:], strict=False):
        _, side = auto_nav._point_segment_distance(lat, lon, a, b)
        if best is None or abs(side) < abs(best):
            best = side
    return abs(best or 0.0)


def _dist(a_lat, a_lon, b):
    from meridianatc.navdata.build import haversine_nm
    return haversine_nm(a_lat, a_lon, b[0], b[1]) * 1852.0


def _taxi_run(fo, plant, monkeypatch, seconds=240.0, dt=0.25):
    """Drive the taxi follower against the plant until the route ends,
    the clock runs out, or the aeroplane has left the pavement for good.
    Returns (per-tick track, path).
    """
    import time

    from meridianatc.atc.autonomous import AutonomousPhase
    clock = [1000.0]
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])
    made = fo._pid

    def on_the_plants_clock(channel):
        pid = made(channel)
        pid._clock = lambda: clock[0]
        return pid
    fo._pid = on_the_plants_clock
    end = clock[0] + seconds
    track = []
    while clock[0] < end:
        state = plant.state()
        fo._taxi_along_path(state, outbound=True)
        plant.step(dt)
        clock[0] += dt
        track.append((clock[0], plant.lat, plant.lon, plant.hdg,
                      plant.ias))
        if fo.state.phase == AutonomousPhase.HOLDING_SHORT:
            break
    return track


def test_turns_ahead_names_the_corner():
    path = _l_path()
    corners = auto_nav.turns_ahead(BASE_LAT, BASE_LON, path, 500.0)
    assert len(corners) == 1
    at_m, deg = corners[0]
    assert at_m == pytest.approx(400.0, abs=1.0)
    assert deg == pytest.approx(90.0, abs=1.0)
    # A straight path has no corner to name, and the window bounds it:
    # a corner outside the lookahead is not the driver's problem yet.
    assert auto_nav.turns_ahead(
        BASE_LAT, BASE_LON, path[:2], 500.0) == []
    assert auto_nav.turns_ahead(BASE_LAT, BASE_LON, path, 150.0) == []


def test_nearest_leg_does_not_leap_to_the_return_lane():
    # A route that passes 20 m beside its own first leg, the way an
    # apron's out-and-back does: a metre off its own line and the
    # nearest pick must stay with the leg the aeroplane is on, not
    # teleport to the nearer later one.
    p0 = (BASE_LAT, BASE_LON)
    p1 = project(BASE_LAT, BASE_LON, 90.0, 300.0 * M_TO_NM)
    p2 = project(p1[0], p1[1], 180.0, 20.0 * M_TO_NM)
    p3 = project(p2[0], p2[1], 270.0, 300.0 * M_TO_NM)
    path = [p0, p1, p2, p3]
    # 100 m down the outbound leg, drifted 15 m off it -- 5 m from the
    # return leg. Unpenalised, that later leg steals the pick.
    me = project(BASE_LAT, BASE_LON, 90.0, 100.0 * M_TO_NM)
    me = project(me[0], me[1], 180.0, 15.0 * M_TO_NM)
    jumped = auto_nav.path_steering(me[0], me[1], path, lookahead_m=60.0)
    assert jumped["nearest"] == 2
    steer = auto_nav.path_steering(me[0], me[1], path,
                                   lookahead_m=60.0, skip_penalty_m=6.0)
    assert steer["nearest"] == 0


def test_a_sharp_corner_is_made_inside_the_pavement(monkeypatch):
    path = _l_path()
    plant = Plant(BASE_LAT, BASE_LON, 0.0, 90.0, 0.0)
    plant.on_ground = True
    fo, _ = _taxi_fo(plant, path)
    track = _taxi_run(fo, plant, monkeypatch)
    assert track, "no taxi happened"
    vertex = path[1]
    max_off = max(_off_path_m(lat, lon, path)
                  for _, lat, lon, *_ in track)
    at_vertex = min(track, key=lambda row: _dist(row[1], row[2], vertex))
    arrived = _dist(track[-1][1], track[-1][2], path[-1]) < 60.0
    assert arrived, "the route was never finished"
    # The corner is taken at walking pace, not taxi speed: both sims
    # shed nosewheel authority with the knots, so the turn radius is
    # the speed's, not the tiller's.
    assert at_vertex[4] <= 8.0, \
        f"corner made at {at_vertex[4]:.1f} kt"
    # And the swing stays on the pavement: ten metres off the line is
    # over the edge of a real apron lane.
    assert max_off <= 10.0, \
        f"swung {max_off:.0f} m off the route"


def test_a_corner_at_taxi_speed_still_turns_on_the_pavement(monkeypatch):
    # The same corner, but the aeroplane is already at taxi speed when
    # the follower takes over: the braking must start before the turn,
    # not in it.
    path = _l_path(first_m=120.0)
    plant = Plant(BASE_LAT, BASE_LON, 0.0, 90.0, 14.0)
    plant.on_ground = True
    fo, _ = _taxi_fo(plant, path)
    track = _taxi_run(fo, plant, monkeypatch, seconds=120.0)
    vertex = path[1]
    max_off = max(_off_path_m(lat, lon, path)
                  for _, lat, lon, *_ in track)
    at_vertex = min(track, key=lambda row: _dist(row[1], row[2], vertex))
    assert at_vertex[4] <= 9.0
    assert max_off <= 12.0
