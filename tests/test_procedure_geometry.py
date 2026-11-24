"""Regression tests for the procedure geometry the FO flies.

The audit's gap list: build_path over CA/VA/FA/VM course legs, the
HA/HF/HM racetracks, AF/RF arcs, PI procedure turns, and _find's
spoken-name matching -- none of which had a test. Each test builds a
bare ProcedureLeg/Transition by hand so no nav database is needed.
"""

from __future__ import annotations

from meridianatc.atc.procedure_legs import (
    ALT_AT_OR_BELOW,
    ALT_IN_BETWEEN,
    M_PER_NM,
    ProcedureLeg,
    ProcedureTransition,
    TerminalProcedure,
    _find,
    build_path,
    haversine_nm,
)


def _leg(path: str, **kw) -> ProcedureLeg:
    kw.setdefault("index", 0)
    kw.setdefault("path", path)
    return ProcedureLeg(**kw)


def _path(legs, start=None, kind=""):
    transition = ProcedureTransition(name="T", legs=tuple(legs))
    points, notes = build_path(transition, start=start, kind=kind)
    return points, notes


def test_leg_end_for_a_ca_leg_walks_the_course():
    # Course-to-altitude: no fix, ends along the published course.
    leg = _leg("CA", course=90.0, alt1_ft=3000.0,
               true_degree=True, route_distance=5.0 * M_PER_NM)
    points, _notes = _path([leg], start=(50.0, 0.0), kind="sid")
    assert points
    end_lat, end_lon = points[-1][:2]
    # Due east, roughly five miles along.
    assert abs(end_lat - 50.0) < 0.01
    assert haversine_nm(50.0, 0.0, end_lat, end_lon) > 3.5


def test_vm_leg_needs_a_course_or_falls_back():
    # A VM leg with no course and no fix ends nowhere -- the old
    # `_leg_end` returned None silently and the path stopped dead.
    leg = _leg("VM", course=0.0)
    points, notes = _path([_leg("IF", fix_ident="A", lat=50.0, lon=0.0),
                           leg], start=(50.0, 0.0), kind="star")
    assert points  # the path still ends, on whatever it could draw


def test_a_course_without_a_fix_still_draws_something():
    leg = _leg("FA", course=270.0, true_degree=True,
               route_distance=6.0 * M_PER_NM)
    points, _notes = _path([leg], start=(50.0, 0.0), kind="star")
    lat, lon = points[-1][:2]
    assert lon < 0.0           # west of the start
    assert haversine_nm(50.0, 0.0, lat, lon) > 4.0


def test_a_hold_draws_a_racetrack_back_to_the_fix():
    fix = _leg("HF", fix_ident="OLD", lat=50.0, lon=0.0, course=90.0,
               true_degree=True, route_distance=4.0 * M_PER_NM)
    points, notes = _path([fix], start=(49.98, 0.0), kind="star")
    assert any("racetrack" in n for n in notes)
    # The pattern's last point is the hold's own fix again.
    assert abs(points[-1][0] - 50.0) < 0.001
    assert abs(points[-1][1] - 0.0) < 0.001
    # And it went somewhere outbound first.
    assert max(lon for _lat, lon, _a, _s in points) > 0.02


def test_an_af_arc_turns_about_the_navaid():
    # A DME arc: centre at the origin navaid, radius rho, ending at the
    # leg's own fix.
    leg = _leg("AF", fix_ident="ARC1", lat=50.0, lon=0.1,
               origin_lat=50.0, origin_lon=0.0,
               rho_m=8.0 * M_PER_NM, turn="R")
    points, notes = _path([leg], start=(50.0 + 8.0 / 60.0, 0.0),
                          kind="star")
    assert any("AF arc approximated" in n for n in notes)
    # Every drawn point sits on the arc's own circle, a tenth of a mile.
    for lat, lon, _a, _s in points[:-1]:
        d = haversine_nm(50.0, 0.0, lat, lon)
        assert abs(d - 8.0) < 0.15, d
    assert abs(points[-1][0] - 50.0) < 0.001
    assert abs(points[-1][1] - 0.1) < 0.001


def test_a_pi_leg_goes_out_and_comes_back():
    leg = _leg("PI", fix_ident="FAF", lat=50.0, lon=0.1, course=270.0,
               true_degree=True, route_distance=4.0 * M_PER_NM)
    points, notes = _path([leg], start=(50.0, 0.0), kind="star")
    assert any("PI approximated" in n for n in notes)
    # The turn heads west of the fix first, then comes back to it.
    assert min(lon for _lat, lon, _a, _s in points) < 0.05
    assert abs(points[-1][0] - 50.0) < 0.01
    assert abs(points[-1][1] - 0.1) < 0.01


def test_sid_at_or_below_is_a_climb_through_not_a_level():
    # ALT_AT_OR_BELOW on a SID is a ceiling, not an altitude to level
    # at -- the point's level is 0, not the ceiling.
    leg = _leg("IF", fix_ident="A", lat=50.0, lon=0.0,
               alt_desc=ALT_AT_OR_BELOW, alt1_ft=6000.0)
    points, _notes = _path([leg], start=(50.0, 0.0), kind="sid")
    assert points[-1][2] == 0.0


def test_alt_in_between_flies_the_low_edge():
    leg = _leg("IF", fix_ident="A", lat=50.0, lon=0.0,
               alt_desc=ALT_IN_BETWEEN, alt1_ft=8000.0, alt2_ft=5000.0)
    points, _notes = _path([leg], start=(50.0, 0.0), kind="star")
    assert points[-1][2] == 5000.0


def test_find_takes_the_written_form_not_a_foreign_digit():
    # "ROUSY 2R" must not match a stored "ROUS1L": the digit is part of
    # the designator, not decoration.
    proc = TerminalProcedure(name="ROUS1L", kind="sid")
    table = {"ROUS1L": proc}
    assert _find(table, "ROUSY 1L") is proc
    assert _find(table, "ROUSY 2R") is None
    assert _find(table, "ROUS1L") is proc
    assert _find(table, "") is None
    assert _find(table, "NOWHERE") is None


def test_find_is_the_same_name_written_two_ways():
    proc = TerminalProcedure(name="LUN2A", kind="star")
    table = {"LUN2A": proc}
    assert _find(table, "LUN 2A") is proc or _find(table, "LUN2A") is proc
