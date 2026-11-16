"""Taxi routing kept off runways in use, and the borrowed autofly profile.

Covers :meth:`GroundLayout.strip_edges` and the ``closed`` routes it feeds:
a taxi plan never runs down a runway in use while a way round exists, falls
back to the pavement only where the runway is the way, and still crosses
the runway it is bound for. And :func:`meridianatc.atc.autofly.nearest` --
an aeroplane nobody wrote a profile for is flown as the closest one there
is, retuned to its own numbers.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from meridianatc.atc import autofly, flying
from meridianatc.atc.ground import (
    PATH_PARKING,
    PATH_RUNWAY,
    PATH_TAXI,
    GroundLayout,
)
from meridianatc.atc.taxi import (
    RunwayLine,
    closed_runway_edges,
    plan_departure,
)


def _field_layout(*, taxiway: bool = True) -> GroundLayout:
    """A runway and a parallel taxiway, stand at the west end.

    Runway 09/27 runs east along z=0; taxiway A runs along z=60, joined to
    the runway at either end. With ``taxiway`` False the taxiway stops at
    midfield and the pavement is the only way east.
    """
    points = [
        (0, 0, -200.0, 60.0),     # 0 stand connector
        (0, 0, 0.0, 60.0),        # 1 t0
        (0, 0, 400.0, 60.0),      # 2 t1
        (0, 0, 800.0, 60.0),      # 3 t2
        (0, 0, 1200.0, 60.0),     # 4 t3
        (0, 0, 0.0, 0.0),         # 5 r0
        (0, 0, 400.0, 0.0),       # 6 r1
        (0, 0, 800.0, 0.0),       # 7 r2
        (0, 0, 1200.0, 0.0),      # 8 r3
        (0, 0, 800.0, 30.0),      # 9 j2 -- midfield connector onto r2
    ]
    parkings = [(14, 0, "GATE", "", 1, 0, 90.0, 20.0, -200.0, 60.0)]
    paths = [
        (PATH_PARKING, 0, 0, "", 0, 0, -1),
        (PATH_TAXI, 0, 0, "A", 0, 1, -1),
        (PATH_TAXI, 0, 0, "A", 1, 2, -1),
    ]
    if taxiway:
        paths += [
            (PATH_TAXI, 0, 0, "A", 2, 3, -1),
            (PATH_TAXI, 0, 0, "A", 3, 4, -1),
        ]
    paths += [
        (PATH_TAXI, 0, 0, "", 1, 5, -1),   # connector at the 09 end
        (PATH_TAXI, 0, 0, "", 4, 8, -1),   # connector at the 27 end
        (PATH_TAXI, 0, 0, "", 3, 9, 1),   # midfield connector onto r2
        (PATH_TAXI, 0, 0, "B", 9, 7, 1),
        (PATH_RUNWAY, 0, 0, "", 5, 6, -1),
        (PATH_RUNWAY, 0, 0, "", 6, 7, -1),
        (PATH_RUNWAY, 0, 0, "", 7, 8, -1),
    ]
    return GroundLayout("TEST", 51.0, 0.0, points, parkings, paths,
                        ["A", "B"])


def _runway_line() -> RunwayLine:
    return RunwayLine(("09", "27"), (0.0, 0.0), (1200.0, 0.0), 15.0)


def _undirected(edges):
    return {frozenset(pair) for pair in edges}


def test_strip_edges_tells_along_from_crossing():
    layout = _field_layout()
    along, crossing = layout.strip_edges((0.0, 0.0), 90.0, 1200.0)
    assert _undirected(along) == {
        frozenset((("p", 5), ("p", 6))), frozenset((("p", 6), ("p", 7))),
        frozenset((("p", 7), ("p", 8)))}
    # The ways on and off the pavement, not the way down it.
    assert _undirected(crossing) == {
        frozenset((("p", 1), ("p", 5))), frozenset((("p", 4), ("p", 8))),
        frozenset((("p", 3), ("p", 9))), frozenset((("p", 9), ("p", 7)))}


def test_closed_route_keeps_off_the_runway():
    layout = _field_layout()
    along, _crossing = layout.strip_edges((0.0, 0.0), 90.0, 1200.0)
    nodes = layout.route(("p", 0), ("p", 8), closed=along)
    assert ("p", 1) in nodes and ("p", 4) in nodes
    assert not {("p", 6), ("p", 7)} & set(nodes)


def test_closed_route_crosses_on_where_the_clearance_allows():
    layout = _field_layout()
    along, crossing = layout.strip_edges((0.0, 0.0), 90.0, 1200.0)
    # ``along`` alone still lets the midfield connector onto the pavement.
    nodes = layout.route(("p", 0), ("p", 7), closed=along)
    assert nodes[-1] == ("p", 7)
    assert ("p", 9) in nodes
    assert not set(zip(nodes, nodes[1:], strict=False)) & along
    # Closing the crossings too keeps it off the pavement entirely.
    nodes = layout.route(("p", 0), ("p", 7),
                         closed=along | crossing)
    assert nodes == [] or not set(zip(nodes, nodes[1:], strict=False)) \
        & (along | crossing)


def test_closed_route_finds_no_way_where_none_is():
    layout = _field_layout(taxiway=False)
    along, crossing = layout.strip_edges((0.0, 0.0), 90.0, 1200.0)
    # The taxiway stops at midfield: east of it, the pavement is the way.
    assert layout.route(("p", 0), ("p", 8),
                        closed=along | crossing) == []
    assert layout.route(("p", 0), ("p", 8), closed=along) == []
    # Which is why the callers fall back: no route at all is worse than a
    # runway route a clearance would have covered anyway.
    nodes = layout.route(("p", 0), ("p", 8))
    assert nodes[-1] == ("p", 8)


def test_closed_runway_edges_covers_the_named_strips():
    layout = _field_layout()
    line = _runway_line()
    closed = closed_runway_edges(layout, [line], along={"27"})
    assert frozenset((("p", 5), ("p", 6))) in _undirected(closed)
    # The other end's ident names the same strip.
    assert closed == closed_runway_edges(layout, [line], along={"09"})
    with_crossings = closed_runway_edges(layout, [line], along={"09"},
                                         across={"09"})
    assert _undirected(with_crossings - closed) == {
        frozenset((("p", 1), ("p", 5))), frozenset((("p", 4), ("p", 8))),
        frozenset((("p", 3), ("p", 9))), frozenset((("p", 9), ("p", 7)))}


def test_plan_departure_keeps_the_taxiway_round_a_busy_runway():
    layout = _field_layout()
    line = _runway_line()
    lat, lon = layout.to_latlon(-200.0, 60.0)
    plan, why = plan_departure(layout, lat, lon, "27", [line],
                               busy={"09", "27"})
    assert plan is not None, why
    taxi_nodes = set(plan.nodes)
    # The way round is the taxiway: the runway's own nodes are the
    # holding point and the line-up's business, not the taxi's.
    assert ("p", 6) not in taxi_nodes and ("p", 7) not in taxi_nodes


def test_plan_departure_backtracks_when_the_runway_is_the_way():
    layout = _field_layout(taxiway=False)
    line = _runway_line()
    lat, lon = layout.to_latlon(-200.0, 60.0)
    plan, why = plan_departure(layout, lat, lon, "27", [line],
                               busy={"09", "27"})
    assert plan is not None, why
    # With no way round, the runway it is departing from is the route.
    assert plan.nodes


def test_plan_departure_carries_the_paved_way_onto_the_runway():
    """The line-up leg the planner already walked: hold -> connector ->
    the centreline just past the entry. Dropping it is what left the
    FO steering a straight intercept to the centreline across the
    infield at Porto's A3 -- on the grass where a paved way exists."""
    layout = _field_layout()
    line = _runway_line()
    lat, lon = layout.to_latlon(-200.0, 60.0)
    plan, why = plan_departure(layout, lat, lon, "27", [line],
                               busy={"09", "27"})
    assert plan is not None, why
    assert len(plan.lineup) >= 2
    # It picks up where the taxi route leaves off: the holding point.
    assert plan.lineup[0] == pytest.approx(plan.points[-1])
    # And it ends on the centreline just past the entry -- on the
    # pavement, not on the grass beside it.
    end_x, end_z = layout.to_xy(*plan.lineup[-1])
    along, across = layout.runway_frame(end_x, end_z, (0.0, 0.0), 270.0)
    assert across == pytest.approx(0.0, abs=2.0)


def test_prefer_intersection_takes_a_legal_mid_runway_entry():
    """A crew that asks for an intersection is given one: an entry past
    the threshold that still leaves its takeoff distance ahead."""
    layout = _field_layout()
    line = _runway_line()
    lat, lon = layout.to_latlon(-200.0, 60.0)
    plan, why = plan_departure(layout, lat, lon, "09", [line],
                               busy={"09"}, need_m=200.0,
                               prefer_intersection=True)
    assert plan is not None, why
    # The midfield connector, not the threshold it sat nearer to.
    assert plan.intersection == "B"
    assert plan.remaining_m >= 200.0


def test_prefer_intersection_never_spends_the_takeoff_distance():
    """An ask is not a way onto pavement that is not there: where no
    entry leaves the type's distance, the runway is the full length."""
    layout = _field_layout()
    line = _runway_line()
    lat, lon = layout.to_latlon(-200.0, 60.0)
    plan, why = plan_departure(layout, lat, lon, "09", [line],
                               busy={"09"}, need_m=900.0,
                               prefer_intersection=True)
    assert plan is not None, why
    assert not plan.intersection


# -- the borrowed profile -------------------------------------------------


def _profiles():
    """A two-profile catalogue: a fast jet and a slow single."""
    return [
        {
            "name": "Test Single",
            "canAutofly": True,
            "titleMatches": ["testskyhawk"],
            "icao": ["C172"],
            "performance": {
                "rotationSpeedKnots": 55,
                "climbSpeedKnots": 74,
                "cruiseSpeedKnots": 105,
                "approachSpeedKnots": 60,
                "descentRateFpm": 500,
            },
        },
        {
            "name": "Test Jet",
            "canAutofly": True,
            "titleMatches": ["testjet"],
            "icao": ["B738"],
            "performance": {
                "rotationSpeedKnots": 135,
                "climbSpeedKnots": 180,
                "cruiseSpeedKnots": 430,
                "approachSpeedKnots": 140,
                "descentRateFpm": 2500,
            },
        },
    ]


def _with_profiles(profiles, fn):
    import json
    import tempfile
    from pathlib import Path
    old = autofly._profiles, autofly._loaded, autofly.DATA_DIR
    try:
        autofly._profiles = []
        autofly._loaded = False
        with tempfile.TemporaryDirectory() as tmp:
            for index, prof in enumerate(profiles):
                Path(tmp, f"prof{index}.json").write_text(
                    json.dumps(prof), encoding="utf-8")
            autofly.DATA_DIR = Path(tmp)
            try:
                return fn()
            finally:
                autofly.DATA_DIR = old[2]
    finally:
        autofly._profiles, autofly._loaded = old[0], old[1]


def test_nearest_finds_the_same_type_under_another_title():
    def go():
        prof, how = autofly.nearest("C172", "a repaint nobody filed", "",
                                    flying.LIGHT)
        assert prof is not None and how == "icao"
        assert prof["name"] == "Test Single"
    _with_profiles(_profiles(), go)


def test_nearest_finds_the_classmate_with_the_nearest_speeds():
    def go():
        # A made-up piston single: the single's profile, not the jet's.
        prof, how = autofly.nearest("ZZ99", "an unknown aeroplane",
                                    "piston_single", flying.LIGHT)
        assert prof is not None and how == "class"
        assert prof["name"] == "Test Single"
        prof, how = autofly.nearest("ZZ99", "an unknown aeroplane",
                                    "airliner", flying.AIRLINER)
        assert prof is not None and prof["name"] == "Test Jet"
    _with_profiles(_profiles(), go)


def test_nearest_gives_nothing_on_a_guess():
    def go():
        # A type nobody knows is not evidence enough to borrow with.
        prof, how = autofly.nearest("ZZZZZ", "a glider", "", flying.LIGHT)
        assert prof is None and how == ""
    _with_profiles(_profiles(), go)


def test_borrowed_profile_is_retuned_to_the_model():
    def go():
        base = flying.LIGHT
        perf = autofly.override(
            "ZZ99", "an unknown aeroplane", base,
            aircraft_class="piston_single",
            declared={"rotate_kt": 45.0, "cruise_kt": 95.0})
        # Declared speeds are flown as declared.
        assert perf.rotate_kt == 45.0
        assert perf.cruise_kt == 95.0
        # The rest are scaled toward the aeroplane, not flown at the
        # donor's numbers.
        assert perf.climb_kt < 74.0
        assert 0.0 < perf.approach_kt < 60.0
        # And it is not the class table either: the shape is the donor's.
        assert perf != base
    _with_profiles(_profiles(), go)


def test_borrowed_profile_scales_without_declared_figures():
    def go():
        base = flying.LIGHT
        perf = autofly.override("ZZ99", "an unknown aeroplane", base,
                                aircraft_class="piston_single")
        # Scaled by the class table's rotation over the donor's.
        factor = base.rotate_kt / 55.0
        assert abs(perf.rotate_kt - 55.0 * factor) < 0.01
        assert abs(perf.climb_kt - 74.0 * factor) < 0.01
    _with_profiles(_profiles(), go)


def test_title_match_still_wins_outright():
    def go():
        perf = autofly.override("B738", "the testjet itself",
                                flying.AIRLINER)
        assert perf.rotate_kt == 135.0
        assert perf.cruise_kt == 430.0
    _with_profiles(_profiles(), go)


# -- the checklist that waits its turn ------------------------------------


def test_phase_checklists_queue_rather_than_abandon():
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState

    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    started = []

    class _Runner:
        waiting = True

        def start(self, name):
            started.append(name)

    fo.crew = SimpleNamespace(checklists=_Runner())
    fo._run_checklist_for_phase("taxi")
    # A checklist already being read is not cut off for the next one.
    assert not started
    assert "taxi" not in fo.state.checklists_completed

    fo.crew.checklists.waiting = False
    fo._run_checklist_for_phase("taxi")
    assert started == ["taxi"]
    assert "taxi" in fo.state.checklists_completed

    fo._run_checklist_for_phase("taxi")
    assert started == ["taxi"]           # once a flight, not once a pass
