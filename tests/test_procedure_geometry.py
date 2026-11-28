"""Regression tests for the procedure geometry the FO flies.

The audit's gap list: build_path over CA/VA/FA/VM course legs, the
HA/HF/HM racetracks, AF/RF arcs, PI procedure turns, and _find's
spoken-name matching -- none of which had a test. Each test builds a
bare ProcedureLeg/Transition by hand so no nav database is needed.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

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
    # Course 090 is the inbound course, so the outbound leg runs west
    # of the fix before turning back onto it.
    assert min(lon for _lat, lon, _a, _s in points) < -0.02


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


def test_an_rf_arc_is_drawn_on_its_own_circle():
    """A radius-to-fix leg -- the shape that makes VQPR's curved final
    possible -- is flown as the arc, not as a straight chord across the
    terrain inside the turn."""
    from meridianatc.navdata.build import project
    centre_lat, centre_lon = 27.40, 89.42
    radius_nm = 4.0
    start = project(centre_lat, centre_lon, 0.0, radius_nm)
    end = project(centre_lat, centre_lon, 90.0, radius_nm)
    first = _leg("IF", fix_ident="PR100", lat=start[0], lon=start[1])
    arc = _leg("RF", fix_ident="PR101", lat=end[0], lon=end[1],
               arc_lat=centre_lat, arc_lon=centre_lon,
               radius_m=radius_nm * M_PER_NM, turn="R")
    points, notes = _path([first, arc], start=start, kind="approach")
    assert any("RF arc approximated" in n for n in notes)
    arc_points = points[1:]
    assert len(arc_points) >= 6
    for lat, lon, _alt, _speed in arc_points[:-1]:
        assert haversine_nm(centre_lat, centre_lon, lat, lon) == \
            pytest.approx(radius_nm, abs=0.12)


def test_track_from_fix_legs_leave_their_fix_instead_of_reversing_to_it():
    """ARINC FA/FC/FD/FM legs name the fix the track starts at. Treating it
    as the terminator makes the path turn back to the point it just left."""
    fc = _leg("FC", fix_ident="SRC", lat=50.0, lon=0.0, course=90.0,
              true_degree=True, route_distance=6.0 * M_PER_NM)
    points, notes = _path([fc], start=(49.95, -0.02), kind="sid")
    assert any("FC track from fix" in n for n in notes)
    assert points[0][:2] == pytest.approx((50.0, 0.0))
    assert points[-1][1] > 0.04       # east of the source fix, not back south-west


def test_fix_to_altitude_leaves_the_fix_on_the_published_track():
    fa = _leg("FA", fix_ident="SRC", lat=50.0, lon=0.0, course=90.0,
              true_degree=True, alt1_ft=3000.0)
    points, notes = _path([fa], start=(50.0, 0.0), kind="sid")
    assert any("FA track from fix" in n for n in notes)
    assert points[-1][1] > 0.04


def test_course_to_fix_joins_the_coded_course_line_before_the_fix():
    """A CF terminates at the fix on the specified course; direct-to-fix
    geometry shortcuts onto the fix from whatever angle the aircraft is on."""
    cf = _leg("CF", fix_ident="CFIX", lat=50.0, lon=0.1, course=90.0,
              true_degree=True)
    points, notes = _path([cf], start=(49.95, -0.02), kind="approach")
    assert any("CF course line" in n for n in notes)
    assert len(points) >= 2
    join_lat, join_lon = points[-2][:2]
    assert join_lat == pytest.approx(50.0, abs=0.01)
    assert join_lon < 0.1
    assert points[-1][:2] == pytest.approx((50.0, 0.1))


def test_fly_over_vertices_are_not_turn_anticipated():
    """Fly-by fixes may be anticipated; a fly-over fix is a hard gate that
    the lookahead may not round before the aircraft passes overhead."""
    from meridianatc.atc import auto_nav
    path = auto_nav.SteeringPath(
        [(50.0, 0.0), (50.0, 0.1), (50.03, 0.1)], barriers={1})
    steer = auto_nav.path_steering(50.0, 0.04, path, lookahead_m=9000.0)
    assert steer["heading"] == pytest.approx(90.0, abs=8.0)
    plain = auto_nav.path_steering(
        50.0, 0.04, [(50.0, 0.0), (50.0, 0.1), (50.03, 0.1)],
        lookahead_m=9000.0)
    assert plain["heading"] < steer["heading"] - 10.0


def test_course_to_dme_distance_ends_on_the_dme_circle():
    """CD/VD terminate where the coded course meets the navaid's DME
    distance, not after a guessed fixed-length course leg."""
    cd = _leg("CD", course=360.0, true_degree=True,
              origin_lat=50.0, origin_lon=0.0, rho_m=5.0 * M_PER_NM)
    start = (50.0 - 10.0 / 60.0, 0.0)
    points, notes = _path([cd], start=start, kind="approach")
    assert any("CD flown on course" in n for n in notes)
    end_lat, end_lon = points[-1][:2]
    assert haversine_nm(50.0, 0.0, end_lat, end_lon) == \
        pytest.approx(5.0, abs=0.15)


def test_course_to_radial_ends_on_the_coded_radial():
    """CR/VR terminate at the intersection with the reference navaid's
    radial, which is a different point for every inbound course."""
    cr = _leg("CR", course=90.0, true_degree=True,
              origin_lat=50.0, origin_lon=0.0, theta=360.0)
    points, _notes = _path([cr], start=(50.1, -0.1), kind="star")
    end_lat, end_lon = points[-1][:2]
    assert end_lat == pytest.approx(50.1, abs=0.01)
    assert end_lon == pytest.approx(0.0, abs=0.01)


def test_course_to_intercept_meets_the_next_course_line():
    """CI/VI end where the coded course intercepts the following leg's
    course line, so the transition is continuous instead of fixed-length."""
    ci = _leg("CI", course=90.0, true_degree=True)
    cf = _leg("CF", fix_ident="NXT", lat=51.0, lon=0.1, course=360.0,
              true_degree=True, index=1)
    points, _notes = _path([ci, cf], start=(50.0, 0.0), kind="approach")
    intercept_lat, intercept_lon = points[0][:2]
    assert intercept_lat == pytest.approx(50.0, abs=0.01)
    assert intercept_lon == pytest.approx(0.1, abs=0.01)


def test_an_approach_navigator_carries_the_published_vpa_angle():
    """Traffic and procedure guidance use the approach's own vertical
    angle. At London City a 5.5-degree path is 585 ft/nm; treating it
    as a generic 318 ft/nm descent leaves the aeroplane far high."""
    from meridianatc.atc.procedure_legs import ProcedureNavigator, TerminalProcedure
    from meridianatc.navdata.build import project
    threshold = (51.504894, 0.066026)
    start = project(*threshold, 93.0, 5.0)
    legs = [
        _leg("IF", fix_ident="LC500", lat=start[0], lon=start[1],
             alt1_ft=3000.0),
        _leg("TF", fix_ident="RW27", lat=threshold[0], lon=threshold[1],
             alt_desc=1, alt1_ft=69.0, vertical_angle=-5.5, is_map=True),
    ]
    transition = ProcedureTransition(name="RNP 27", legs=tuple(legs))
    procedure = TerminalProcedure(name="RNP 27", kind="approach",
                                  transitions={"": transition})
    nav = ProcedureNavigator(procedure, transition, start=start)
    on_path = project(*threshold, 93.0, 3.0)
    guide = nav.guidance(SimpleNamespace(
        latitude=on_path[0], longitude=on_path[1], altitude_ft=1800.0))
    assert guide.altitude_ft == pytest.approx(69.0)
    assert guide.altitude_distance_nm == pytest.approx(3.0, abs=0.15)
    assert guide.vpa_ft_per_nm == pytest.approx(585.0, rel=0.03)
