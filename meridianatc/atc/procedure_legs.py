"""SID/STAR legs as the simulator's facility data gives them, and how the
First Officer flies them.

The navigation database has no procedures; the simulator does. This module
is the canonical model the rest of MeridianATC uses: the decoded departure or
arrival, the transition that was cleared, and a navigator that turns the
leg list into the heading, level and speed the autopilot is given.

The leg record is ARINC 424's path terminator plus the fields MSFS exposes
for it (``SimConnect_AddToFacilityDefinition``, ``APPROACH_LEG``). Not every
terminator is flown as charted -- a DME arc needs the station's DME, a
manual terminator waits for a vector -- but the decoded fields are kept so
the guidance can say what it did rather than guessing.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Iterable
from dataclasses import dataclass, field

from ..navdata.build import bearing_deg, haversine_nm
from ..navdata.magvar import to_true, variation
from . import auto_nav
from .procedures import designator, namesake

log = logging.getLogger(__name__)

# SIMCONNECT_FACILITY_DATA_TYPE members this model is built from.
FACILITY_DEPARTURE = 10
FACILITY_ARRIVAL = 11
FACILITY_RUNWAY_TRANSITION = 12
FACILITY_ENROUTE_TRANSITION = 13
FACILITY_APPROACH_LEG = 7

# APPROACH_LEG.TYPE, the ARINC path terminators MSFS names.
PATH_TYPES = {
    1: "AF", 2: "CA", 3: "CD", 4: "CF", 5: "CI", 6: "CR", 7: "DF",
    8: "FA", 9: "FC", 10: "FD", 11: "FM", 12: "HA", 13: "HF",
    14: "HM", 15: "IF", 16: "PI", 17: "RF", 18: "TF", 19: "VA",
    20: "VD", 21: "VI", 22: "VM", 23: "VR",
}

# APPROACH_ALT_DESC.
ALT_NOT_USED, ALT_AT, ALT_AT_OR_ABOVE, ALT_AT_OR_BELOW, ALT_IN_BETWEEN = \
    range(5)
ALT_DESC = {0: "", 1: "AT", 2: "AT OR ABOVE", 3: "AT OR BELOW",
            4: "IN BETWEEN"}

TURN_NONE, TURN_LEFT, TURN_RIGHT, TURN_EITHER = range(4)
TURN_NAMES = {0: "", 1: "L", 2: "R", 3: "E"}

FT_PER_M = 3.28084
M_PER_NM = 1852.0
# Enroute path steering looks much further ahead than a taxiway does.
PROCEDURE_LOOKAHEAD_M = 9000.0


@dataclass(frozen=True)
class ProcedureLeg:
    """One decoded ``APPROACH_LEG`` record, in the units the FO flies."""

    index: int
    path: str                     # AF, CA, ...; "" when unknown
    path_type: int = 0
    fix_ident: str = ""
    fix_region: str = ""
    fix_kind: str = ""
    lat: float = 0.0
    lon: float = 0.0
    fix_alt_ft: float = 0.0
    fly_over: bool = False
    distance_minute: bool = False
    true_degree: bool = False
    turn: str = ""                 # L, R, E or ""
    origin_ident: str = ""
    origin_region: str = ""
    origin_kind: str = ""
    origin_lat: float = 0.0
    origin_lon: float = 0.0
    origin_alt_ft: float = 0.0
    theta: float = 0.0
    rho_m: float = 0.0
    course: float = 0.0
    route_distance: float = 0.0   # metres, or minutes when distance_minute
    alt_desc: int = 0
    alt1_ft: float = 0.0
    alt2_ft: float = 0.0
    speed_limit_kt: float = 0.0
    vertical_angle: float = 0.0
    arc_ident: str = ""
    arc_region: str = ""
    arc_kind: str = ""
    arc_lat: float = 0.0
    arc_lon: float = 0.0
    arc_alt_ft: float = 0.0
    radius_m: float = 0.0
    rnp_m: float = 0.0
    speed_desc: int = 0
    is_iaf: bool = False
    is_if: bool = False
    is_faf: bool = False
    is_map: bool = False
    is_atc_reporting: bool = False
    is_essential: bool = False
    is_idf: bool = False
    is_stepdown: bool = False

    @property
    def has_fix(self) -> bool:
        return bool(self.fix_ident) and not (self.lat == 0.0 and self.lon == 0.0)

    @property
    def altitude_constraint(self) -> float:
        """The level this leg asks for, feet; 0 when it asks for none."""
        if self.alt_desc == ALT_NOT_USED:
            return 0.0
        return self.alt1_ft or self.alt2_ft


@dataclass
class ProcedureTransition:
    """The cleared part of a SID or STAR: one runway or enroute transition."""

    name: str = ""
    runway: str = ""
    legs: tuple[ProcedureLeg, ...] = ()

    @property
    def first_fix(self) -> ProcedureLeg | None:
        return self.legs[0] if self.legs else None

    @property
    def last_fix(self) -> ProcedureLeg | None:
        return self.legs[-1] if self.legs else None


@dataclass
class TerminalProcedure:
    """A named SID or STAR at one airport."""

    name: str
    kind: str                          # "sid" or "star"
    transitions: list[ProcedureTransition] = field(default_factory=list)
    direct_legs: tuple[ProcedureLeg, ...] = ()


@dataclass
class AirportProcedures:
    """Everything one airport publishes, decoded once."""

    ident: str
    departures: dict[str, TerminalProcedure] = field(default_factory=dict)
    arrivals: dict[str, TerminalProcedure] = field(default_factory=dict)
    lat: float = 0.0
    lon: float = 0.0
    alt_ft: float = 0.0

    def departure(self, name: str) -> TerminalProcedure | None:
        return _find(self.departures, name)

    def arrival(self, name: str) -> TerminalProcedure | None:
        return _find(self.arrivals, name)


def _norm(name: str) -> str:
    return (name or "").strip().upper().replace(" ", "")


def _find(table: dict[str, TerminalProcedure], name: str
          ) -> TerminalProcedure | None:
    wanted = _norm(name)
    if not wanted:
        return None
    if wanted in table:
        return table[wanted]
    parts = designator(wanted)
    for key, proc in table.items():
        if _norm(key) == wanted:
            return proc
        other = designator(key)
        if parts and other and parts == other:
            return proc
        # The spoken form ("ROUSY 1L") against the stored six characters.
        # The digit has to match too: "ROUSY 2R" is not "ROUSY 1L",
        # and `digit in wanted` was always true, the digit coming from
        # `wanted` itself.
        if namesake(key) and namesake(key) in wanted:
            digit = parts[1] if parts else ""
            if digit and other and digit == other[1]:
                return proc
    return None


def _runway_key(runway: str) -> str:
    return (runway or "").strip().upper().replace(" ", "")


def _transition_for(procedure: TerminalProcedure, kind: str,
                    runway: str = "", preferred: str = "",
                    route_fix_idents: Iterable[str] = ()) -> \
        ProcedureTransition | None:
    """The transition to fly: the named one, then -- for a SID the
    runway's own transition first and the route's fixes to break ties;
    for a STAR the enroute entry the route flies first and the runway's
    funnel to break ties."""
    transitions = [t for t in procedure.transitions if t.legs]
    if not transitions:
        if procedure.direct_legs:
            return ProcedureTransition(legs=procedure.direct_legs)
        return None
    key = _runway_key(runway)
    wanted = _norm(preferred)
    if wanted:
        matches = [t for t in transitions
                   if _norm(t.name) == wanted
                   or _norm(t.runway) == wanted]
        if len(matches) > 1 and key:
            same = [t for t in matches if _runway_key(t.runway) == key]
            if same:
                return same[0]
        if matches:
            return matches[0]
    # When the procedure names its runways, another runway's legs are not
    # this one's: an unlabeled transition is anybody's, a labeled one is
    # not.
    if key and any(t.runway for t in transitions):
        eligible = [t for t in transitions
                    if not t.runway or _runway_key(t.runway) == key]
        if not eligible:
            return None
    else:
        eligible = transitions
    # The route's fixes name the transition's fixes better than a guess:
    # the one whose fixes the route reaches first wins.
    order = [i.upper() for i in route_fix_idents if i]
    idents = set(order)

    position = {ident: i for i, ident in enumerate(order)}

    def by_route(candidates) -> ProcedureTransition | None:
        """The candidate the route reaches first. A transition's own
        name in the route is the strongest hint -- it is usually the
        fix the transition is entered or left at; leg hits rank after,
        since the trunk's fixes repeat across transitions."""
        best, best_key = None, None
        for trans in candidates:
            keys = []
            name = _norm(trans.name)
            if name and name in position:
                keys.append((0, position[name]))
            hits = {leg.fix_ident.upper() for leg in trans.legs
                    if leg.fix_ident} & idents
            if hits:
                keys.append((1, min(position[h] for h in hits)))
            if keys:
                score = min(keys)
                if best_key is None or score < best_key:
                    best, best_key = trans, score
        return best

    same_runway = [t for t in eligible
                   if key and _runway_key(t.runway) == key]
    if kind == "star":
        # Where the STAR is entered is the route's business; the runway
        # only funnels the end of it.
        found = by_route(same_runway) or by_route(eligible)
        if found is not None:
            return found
        if same_runway:
            return same_runway[0]
    else:
        # A departure flies the runway it is on; within that the route
        # picks which enroute transition the clearance meant.
        found = by_route(same_runway)
        if found is not None:
            return found
        if same_runway:
            return same_runway[0]
    found = by_route(eligible)
    if found is not None:
        return found
    return eligible[0] if len(eligible) == 1 else None


def idents_order(order: list[str], hits) -> list[int]:
    """Order ``hits`` by where they sit in the route's fix list."""
    position = {ident: i for i, ident in enumerate(order)}
    return sorted(position.get(h, len(order)) for h in hits)


def choose_sid(procedures: AirportProcedures | None, name: str,
               runway: str = "", preferred: str = "",
               route_fix_idents: Iterable[str] = ()) -> \
        tuple[TerminalProcedure | None, ProcedureTransition | None]:
    if procedures is None:
        return None, None
    proc = procedures.departure(name)
    if proc is None:
        return None, None
    return proc, _transition_for(proc, "sid", runway, preferred,
                                 route_fix_idents)


def choose_star(procedures: AirportProcedures | None, name: str,
                runway: str = "", preferred: str = "",
                route_fix_idents: Iterable[str] = ()) -> \
        tuple[TerminalProcedure | None, ProcedureTransition | None]:
    if procedures is None:
        return None, None
    proc = procedures.arrival(name)
    if proc is None:
        return None, None
    return proc, _transition_for(proc, "star", runway, preferred,
                                 route_fix_idents)


# ----------------------------------------------------------------------
# flying it
# ----------------------------------------------------------------------


def _point(lat: float, lon: float, alt_ft: float = 0.0) -> tuple[float, float]:
    return lat, lon


def _offset(lat: float, lon: float, bearing: float, distance_nm: float
            ) -> tuple[float, float]:
    """A point ``distance_nm`` along ``bearing`` from a position."""
    if distance_nm <= 0:
        return lat, lon
    # A nautical mile is a minute of arc: the great-circle formula wants
    # the angle in radians, not the degrees the /60 produces.
    angular = math.radians(distance_nm / 60.0)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    brg = math.radians(bearing)
    lat2 = math.asin(math.sin(lat1) * math.cos(angular)
                     + math.cos(lat1) * math.sin(angular) * math.cos(brg))
    lon2 = lon1 + math.atan2(math.sin(brg) * math.sin(angular)
                             * math.cos(lat1),
                             math.cos(angular) - math.sin(lat1)
                             * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)


def _leg_end(leg: ProcedureLeg, from_lat: float, from_lon: float
             ) -> tuple[float, float] | None:
    """Where a leg's lateral path ends, best available."""
    if leg.has_fix:
        return leg.lat, leg.lon
    # Course/heading terminators carry their own direction and distance.
    course = leg.course
    if course > 0 and not leg.true_degree:
        course = to_true(course, variation(from_lat, from_lon))
    if course <= 0:
        return None
    distance_nm = 0.0
    if leg.route_distance:
        distance_nm = (leg.route_distance * 4.0 if leg.distance_minute
                       else leg.route_distance / M_PER_NM)
    elif leg.rho_m:
        distance_nm = leg.rho_m / M_PER_NM
    if distance_nm <= 0:
        return None
    return _offset(from_lat, from_lon, course, distance_nm)


def _sweep(centre_lat: float, centre_lon: float, radius_nm: float,
           from_lat: float, from_lon: float, to_lat: float, to_lon: float,
           turn: str) -> list[tuple[float, float]] | None:
    """Points along an arc about a centre, in its turn's own direction."""
    start_brg = bearing_deg(centre_lat, centre_lon, from_lat, from_lon)
    end_brg = bearing_deg(centre_lat, centre_lon, to_lat, to_lon)
    # The bearing from the centre runs clockwise around it, so a
    # right-hand arc's sweep counts up and a left-hand's counts down --
    # whatever the size: a 270-degree arc is not the 90 flown the other
    # way.
    if turn == "L":
        sweep = -((start_brg - end_brg) % 360.0)
    elif turn == "R":
        sweep = (end_brg - start_brg) % 360.0
    else:
        sweep = ((end_brg - start_brg + 540.0) % 360.0) - 180.0
    steps = max(2, int(abs(sweep) / 15.0) + 1)
    points = []
    for i in range(1, steps + 1):
        brg = start_brg + sweep * i / steps
        # A point on the circle: offset from the centre by the radius.
        lat, lon = _offset(centre_lat, centre_lon, brg, radius_nm)
        points.append((lat, lon))
    points[-1] = (to_lat, to_lon)
    return points


def _arc_points(leg: ProcedureLeg, from_lat: float, from_lon: float
                ) -> list[tuple[float, float]] | None:
    """Points along an RF arc, when the centre and radius are known."""
    if not leg.has_fix or leg.arc_lat == 0.0 and leg.arc_lon == 0.0:
        return None
    centre_lat, centre_lon = leg.arc_lat, leg.arc_lon
    radius_nm = leg.radius_m / M_PER_NM if leg.radius_m > 0 else \
        haversine_nm(centre_lat, centre_lon, leg.lat, leg.lon)
    if radius_nm < 0.5:
        return None
    return _sweep(centre_lat, centre_lon, radius_nm, from_lat, from_lon,
                  leg.lat, leg.lon, leg.turn)


def _dme_arc_points(leg: ProcedureLeg, from_lat: float, from_lon: float
                    ) -> list[tuple[float, float]] | None:
    """Points along an AF leg: the arc at the navaid's DME distance.

    An AF's centre is the leg's origin (the DME's facility) and its
    radius is rho -- the record names both, so the arc is drawn the
    same way an RF's is.
    """
    if not leg.has_fix or leg.rho_m <= 0 \
            or leg.origin_lat == 0.0 and leg.origin_lon == 0.0:
        return None
    radius_nm = leg.rho_m / M_PER_NM
    if radius_nm < 0.5:
        return None
    return _sweep(leg.origin_lat, leg.origin_lon, radius_nm,
                  from_lat, from_lon, leg.lat, leg.lon, leg.turn)


_COURSE_PATHS = {"CA", "CD", "CI", "CR", "VA", "VD", "VI", "VM", "VR"}
_TRACK_FROM_FIX_PATHS = {"FA", "FC", "FD", "FM"}


def _planar_xy(lat: float, lon: float, origin_lat: float, origin_lon: float
               ) -> tuple[float, float]:
    """Local east/north nautical miles about an origin, adequate for a leg."""
    mean_lat = math.radians((lat + origin_lat) / 2.0)
    return ((lon - origin_lon) * 60.0 * math.cos(mean_lat),
            (lat - origin_lat) * 60.0)


def _true_course(leg: ProcedureLeg, lat: float, lon: float) -> float:
    course = leg.course
    if course > 0 and not leg.true_degree:
        course = to_true(course, variation(lat, lon))
    return course


def _track_from_fix_points(leg: ProcedureLeg, from_lat: float, from_lon: float,
                           prev_alt_ft: float) -> list[tuple[float, float]]:
    """Geometry for FA/FC/FD/FM, whose fix is the *start* of the track.

    Treating that database fix as the terminator reverses the leg back onto
    its own origin. These terminators instead leave that fix on the coded
    track: FC for its distance, FA until the altitude is reached (estimated),
    FD to the DME distance (approximated along track), and FM far enough for
    the manual/vector termination the procedure expects.
    """
    start_lat, start_lon = (leg.lat, leg.lon) if leg.has_fix else (from_lat, from_lon)
    course = _true_course(leg, start_lat, start_lon)
    if not (0 < course <= 360.0):
        return []
    if leg.path == "FC" and leg.route_distance:
        distance_nm = (leg.route_distance * 4.0 if leg.distance_minute
                       else leg.route_distance / M_PER_NM)
    elif leg.path == "FA" and leg.altitude_constraint:
        distance_nm = abs(leg.altitude_constraint - prev_alt_ft) / 300.0
        distance_nm = min(15.0, max(4.0, distance_nm))
    elif leg.rho_m:
        distance_nm = leg.rho_m / M_PER_NM
    elif leg.route_distance:
        distance_nm = (leg.route_distance * 4.0 if leg.distance_minute
                       else leg.route_distance / M_PER_NM)
    else:
        distance_nm = 8.0
    end = _offset(start_lat, start_lon, course, distance_nm)
    points = []
    if leg.has_fix and haversine_nm(from_lat, from_lon, start_lat,
                                    start_lon) > 0.05:
        points.append((start_lat, start_lon))
    points.append(end)
    return points


def _course_to_fix_points(leg: ProcedureLeg, from_lat: float, from_lon: float
                          ) -> list[tuple[float, float]]:
    """A CF's specified course into its fix, rather than a direct shortcut.

    The path terminator requires rolling out on the published course at the
    fix. When the present position is off that final course line, the path
    first joins the line at the position's own projection (or five miles
    behind the fix when the geometry is already abeam/past it), then tracks
    the course to the fix.
    """
    if not leg.has_fix:
        return []
    course = _true_course(leg, leg.lat, leg.lon)
    if not (0 < course <= 360.0):
        return [(leg.lat, leg.lon)]
    rx, ry = _planar_xy(from_lat, from_lon, leg.lat, leg.lon)
    ux, uy = math.sin(math.radians(course)), math.cos(math.radians(course))
    along = rx * ux + ry * uy          # negative before the fix on course
    cross = -rx * uy + ry * ux
    if abs(cross) < 0.25 and along < 0.0:
        return [(leg.lat, leg.lon)]
    if along < -0.2:
        join_x, join_y = ux * along, uy * along
        join_lat = leg.lat + join_y / 60.0
        mean_lat = math.radians(leg.lat)
        join_lon = leg.lon + join_x / (60.0 * max(0.2, math.cos(mean_lat)))
    else:
        join_lat, join_lon = _offset(leg.lat, leg.lon,
                                     (course + 180.0) % 360.0, 5.0)
    points = []
    if haversine_nm(from_lat, from_lon, join_lat, join_lon) > 0.15:
        points.append((join_lat, join_lon))
    points.append((leg.lat, leg.lon))
    return points


def _has_origin_position(leg: ProcedureLeg) -> bool:
    return bool(leg.origin_ident or leg.origin_lat or leg.origin_lon)


def _cross(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * by - ay * bx


def _ray_circle_nm(from_lat: float, from_lon: float, course: float,
                   centre_lat: float, centre_lon: float, radius_nm: float
                   ) -> float | None:
    """Distance along a course ray to a DME circle, in local planar miles."""
    sx, sy = _planar_xy(from_lat, from_lon, centre_lat, centre_lon)
    ux, uy = math.sin(math.radians(course)), math.cos(math.radians(course))
    b = sx * ux + sy * uy
    c = sx * sx + sy * sy - radius_nm * radius_nm
    disc = b * b - c
    if disc < 0.0:
        return None
    root = math.sqrt(disc)
    hits = [t for t in (-b - root, -b + root) if t > 0.1]
    return min(hits) if hits else None


def _ray_radial_nm(from_lat: float, from_lon: float, course: float,
                   centre_lat: float, centre_lon: float, radial: float
                   ) -> float | None:
    """Distance along a course ray to a navaid radial's line."""
    sx, sy = _planar_xy(from_lat, from_lon, centre_lat, centre_lon)
    ux, uy = math.sin(math.radians(course)), math.cos(math.radians(course))
    rx, ry = math.sin(math.radians(radial)), math.cos(math.radians(radial))
    den = _cross(ux, uy, rx, ry)
    if abs(den) < 1e-6:
        return None
    t = _cross(-sx, -sy, rx, ry) / den
    q = _cross(-sx, -sy, ux, uy) / den
    return t if t > 0.1 and q > 0.0 else None


def _ray_course_line_nm(from_lat: float, from_lon: float, course: float,
                        fix_lat: float, fix_lon: float, line_course: float
                        ) -> float | None:
    """Distance along a course ray to another course line through a fix."""
    qx, qy = _planar_xy(fix_lat, fix_lon, from_lat, from_lon)
    ux, uy = math.sin(math.radians(course)), math.cos(math.radians(course))
    vx, vy = math.sin(math.radians(line_course)), math.cos(math.radians(line_course))
    den = _cross(ux, uy, vx, vy)
    if abs(den) < 1e-6:
        return None
    t = _cross(qx, qy, vx, vy) / den
    return t if t > 0.1 else None


def _course_end(leg: ProcedureLeg, from_lat: float, from_lon: float,
                prev_alt_ft: float,
                next_leg: ProcedureLeg | None = None) -> tuple[float, float] | None:
    """A point far enough along the leg's course to fly it by.

    Course-to-altitude and heading legs end on a condition, not a fix --
    the level reached, the DME run, the controller's vector. The path is
    drawn for a plausible distance on the published course: the leg's own
    distance when the record carries one, the miles its climb needs at a
    nominal three hundred feet to the mile, else eight.
    """
    course = leg.course
    if course > 0 and not leg.true_degree:
        course = to_true(course, variation(from_lat, from_lon))
    if not (0 < course <= 360.0):
        return None
    conditional: float | None = None
    if leg.path in ("CD", "VD") and _has_origin_position(leg) \
            and leg.rho_m:
        conditional = _ray_circle_nm(
            from_lat, from_lon, course, leg.origin_lat, leg.origin_lon,
            leg.rho_m / M_PER_NM)
    elif leg.path in ("CR", "VR") and _has_origin_position(leg) \
            and leg.theta:
        radial = leg.theta
        if not leg.true_degree:
            radial = to_true(radial, variation(leg.origin_lat, leg.origin_lon))
        conditional = _ray_radial_nm(
            from_lat, from_lon, course, leg.origin_lat, leg.origin_lon,
            radial)
    elif leg.path in ("CI", "VI") and next_leg is not None \
            and next_leg.has_fix and next_leg.course:
        intercept_course = _true_course(next_leg, next_leg.lat, next_leg.lon)
        conditional = _ray_course_line_nm(
            from_lat, from_lon, course, next_leg.lat, next_leg.lon,
            intercept_course)
    if conditional is not None:
        return _offset(from_lat, from_lon, course,
                       min(30.0, max(0.5, conditional)))
    if leg.route_distance:
        distance_nm = (leg.route_distance * 4.0 if leg.distance_minute
                       else leg.route_distance / M_PER_NM)
    elif leg.rho_m:
        distance_nm = leg.rho_m / M_PER_NM
    elif leg.altitude_constraint:
        distance_nm = abs(leg.altitude_constraint - prev_alt_ft) / 300.0
        distance_nm = min(15.0, max(4.0, distance_nm))
    else:
        distance_nm = 8.0
    return _offset(from_lat, from_lon, course, distance_nm)


def _hold_points(leg: ProcedureLeg, from_lat: float, from_lon: float
                 ) -> list[tuple[float, float]]:
    """One circuit of the holding pattern, drawn as a racetrack.

    The published holding course is inbound to the fix. The hold leaves
    outbound on its reciprocal for the leg's time or distance, makes the
    standard-rate turn, and returns inbound to the fix. One circuit is
    drawn; when the clearance still needs the hold the next pass takes
    the same pattern again.
    """
    fix_lat, fix_lon = (leg.lat, leg.lon) if leg.has_fix \
        else (from_lat, from_lon)
    inbound = leg.course if leg.course > 0 else leg.theta
    if inbound > 0 and not leg.true_degree:
        inbound = to_true(inbound, variation(fix_lat, fix_lon))
    if not (0 < inbound <= 360.0):
        inbound = 0.0
    # ARINC/ICAO holding course is the inbound course to the fix; the
    # racetrack leaves the fix on its reciprocal. Drawing the coded course
    # outbound puts the whole hold on the wrong side of the fix.
    course = (inbound + 180.0) % 360.0
    out = leg.route_distance
    out = out * 4.0 if leg.distance_minute else out / M_PER_NM
    out = min(12.0, max(2.0, out or 4.0))
    turn_sign = -1.0 if leg.turn == "L" else 1.0
    side = (course + 90.0 * turn_sign) % 360.0
    end = _offset(fix_lat, fix_lon, course, out)
    swing = _offset(*end, side, 1.2)
    abeam = _offset(fix_lat, fix_lon, side, 1.2)
    return [end, swing, abeam, (fix_lat, fix_lon)]


def _turn_points(leg: ProcedureLeg, from_lat: float, from_lon: float
                 ) -> list[tuple[float, float]]:
    """A 45/180 course reversal, drawn on the published turn side.

    ARINC's PI does not encode every individual procedure-turn segment;
    the navigation database expects the FMS to construct the standard
    reversal. The path is outbound on the coded course, forty-five degrees
    to the published side, the short cross leg, and the one-eighty back
    toward the fix/inbound course. That is deliberately approximate in
    turn radius, but no longer the impossible direct out-and-back reversal
    the previous polyline drew.
    """
    course = leg.course
    if course > 0 and not leg.true_degree:
        course = to_true(course, variation(from_lat, from_lon))
    outbound = leg.route_distance
    if leg.distance_minute:
        outbound *= 4.0  # ~4 NM a minute, a jet's guess
    else:
        outbound /= M_PER_NM
    outbound = min(12.0, max(2.0, outbound or 4.0))
    if not (0 < course <= 360.0):
        end = _leg_end(leg, from_lat, from_lon)
        return [end] if end is not None else []
    sign = -1.0 if leg.turn == "L" else 1.0
    end_out = _offset(from_lat, from_lon, course, outbound)
    cross_course = (course + 45.0 * sign + 360.0) % 360.0
    cross_nm = min(6.0, max(2.0, outbound * 0.6))
    end_cross = _offset(*end_out, cross_course, cross_nm)
    points = [end_out, end_cross]
    if leg.has_fix:
        points.append((leg.lat, leg.lon))
    else:
        inbound = (course + 180.0) % 360.0
        points.append(_offset(*end_cross, inbound, max(2.0, outbound * 0.8)))
    return points


def build_path(transition: ProcedureTransition,
               start: tuple[float, float] | None = None,
               kind: str = ""
               ) -> tuple[list[tuple[float, float, float, float]], list[str]]:
    """The procedure as a path the autopilot can steer, with each point's
    level and speed limit.

    Returns ``(points, notes)`` where a point is ``(lat, lon, alt_ft,
    speed_kt)``. Terminators whose conditional geometry cannot be drawn
    are noted and flown as a direct leg to the fix the record names where
    that fix really is the terminator; track-from-fix legs are skipped
    rather than reversed back onto their own origin.
    """
    points = auto_nav.SteeringPath()
    notes: list[str] = []
    if not transition.legs:
        return points, ["the procedure has no legs"]
    cursor = start or (transition.legs[0].lat, transition.legs[0].lon)
    for leg_index, leg in enumerate(transition.legs):
        next_leg = (transition.legs[leg_index + 1]
                    if leg_index + 1 < len(transition.legs) else None)
        prev_alt = points[-1][2] if points else 0.0
        # The constraint only: a fix's own elevation is not a level to
        # fly, and on MSFS that is what FIX_ALTITUDE carries.
        alt = leg.altitude_constraint
        if leg.alt_desc == ALT_IN_BETWEEN:
            # A window's low edge is what the constraint is owed.
            alt = min(leg.alt1_ft, leg.alt2_ft) or alt
        if kind == "sid" and leg.alt_desc == ALT_AT_OR_BELOW:
            # A SID's levels are floors the climb makes through; a
            # ceiling is not one -- the climb does not level at it.
            alt = 0.0
        speed = leg.speed_limit_kt
        path = leg.path
        generated: list[tuple[float, float]] = []
        if path == "RF":
            generated = _arc_points(leg, cursor[0], cursor[1]) or []
            if generated:
                notes.append(f"{leg.fix_ident or leg.path}: RF arc approximated")
            else:
                notes.append(f"{leg.fix_ident or leg.path}: RF arc unavailable")
        elif path == "AF":
            generated = _dme_arc_points(leg, cursor[0], cursor[1]) or []
            if generated:
                notes.append(f"{leg.fix_ident or leg.path}: AF arc approximated")
            else:
                notes.append(f"{leg.fix_ident or leg.path}: AF arc unavailable")
        elif path == "PI":
            generated = _turn_points(leg, cursor[0], cursor[1])
            notes.append(f"{leg.fix_ident or leg.path}: {path} approximated")
        elif path in ("HA", "HF", "HM"):
            generated = _hold_points(leg, cursor[0], cursor[1])
            notes.append(f"{leg.fix_ident or leg.path}: {path} racetrack")
        elif path == "CF":
            generated = _course_to_fix_points(leg, cursor[0], cursor[1])
            notes.append(f"{leg.fix_ident or leg.path}: CF course line")
        elif path in _TRACK_FROM_FIX_PATHS:
            generated = _track_from_fix_points(leg, cursor[0], cursor[1],
                                               prev_alt)
            if generated:
                notes.append(f"{leg.fix_ident or leg.path}: {path} track from fix")
            else:
                notes.append(f"{leg.fix_ident or leg.path}: {path} no track")
        elif path in _COURSE_PATHS:
            end = _course_end(leg, cursor[0], cursor[1], prev_alt,
                              next_leg)
            if end is not None:
                generated = [end]
            notes.append(f"{leg.fix_ident or leg.path}: {path} flown on course")
        end = _leg_end(leg, cursor[0], cursor[1])
        if not generated and end is not None \
                and path not in _TRACK_FROM_FIX_PATHS:
            generated = [end]
        if not generated:
            notes.append(f"{leg.fix_ident or leg.path}: no geometry")
            continue
        for lat, lon in generated:
            points.append((lat, lon, alt, speed))
        if leg.fly_over:
            points.barriers.add(len(points) - 1)
        cursor = generated[-1]
    return points, notes


@dataclass
class ProcedureGuidance:
    """What the navigator says this tick."""

    heading: float | None = None       # true degrees
    altitude_ft: float = 0.0
    vs_fpm: float = 0.0
    speed_kt: float = 0.0
    done: bool = False
    note: str = ""
    # Miles out the level named above belongs to -- how far the
    # constrained point is, so a "descend via" can reach the level
    # at the fix on a continuous path rather than miles early.
    altitude_distance_nm: float = 0.0
    # Feet per nautical mile for a published vertical path; zero means
    # the caller's generic three-degree value.
    vpa_ft_per_nm: float = 0.0


class ProcedureNavigator:
    """Flies one chosen transition: lateral path plus leg constraints."""

    def __init__(self, procedure: TerminalProcedure,
                 transition: ProcedureTransition,
                 start: tuple[float, float] | None = None,
                 fallback_altitude_ft: float = 0.0,
                 fallback_speed_kt: float = 0.0):
        self.procedure = procedure
        self.transition = transition
        leg_points, self.notes = build_path(transition, start,
                                            kind=procedure.kind)
        # The path the aeroplane steers starts where the procedure does:
        # the runway end for a SID, the position it was picked up on for a
        # STAR. Without that first point the steering can aim past the
        # first fix while the constraints still say the second leg's.
        barriers = set(getattr(leg_points, "barriers", ()) or ())
        if start is not None and leg_points:
            first = leg_points[0]
            self.points = [(start[0], start[1], first[2], first[3]),
                           *leg_points]
            barriers = {i + 1 for i in barriers}
        else:
            self.points = leg_points
        self.fallback_altitude_ft = fallback_altitude_ft
        self.fallback_speed_kt = fallback_speed_kt
        self._bare = auto_nav.SteeringPath(
            [(lat, lon) for lat, lon, _a, _s in self.points],
            barriers=barriers)

        # A published vertical path angle, where the final leg carries
        # one. The generated path does not retain which RF sample came
        # from which source leg, so the anchor is found by position and
        # the distance left to it is measured along the path itself.
        self._vpa_alt_ft = 0.0
        self._vpa_ft_per_nm = 0.0
        self._vpa_anchor_remaining_m = 0.0
        vpa_leg = next((leg for leg in reversed(transition.legs)
                        if abs(float(getattr(leg, "vertical_angle", 0.0)
                                   or 0.0)) > 0.1
                        and getattr(leg, "has_fix", False)), None)
        if vpa_leg is not None and self.points:
            angle = abs(float(vpa_leg.vertical_angle or 0.0))
            if 1.0 <= angle <= 10.0:
                anchor = min(
                    range(len(self.points)),
                    key=lambda i: haversine_nm(
                        self.points[i][0], self.points[i][1],
                        vpa_leg.lat, vpa_leg.lon))
                if haversine_nm(self.points[anchor][0],
                                self.points[anchor][1],
                                vpa_leg.lat, vpa_leg.lon) <= 0.25:
                    self._vpa_alt_ft = float(
                        vpa_leg.altitude_constraint or 0.0)
                    self._vpa_ft_per_nm = (6076.12 * math.tan(
                        math.radians(angle)))
                    remaining = 0.0
                    for i in range(anchor, len(self._bare) - 1):
                        remaining += haversine_nm(
                            self._bare[i][0], self._bare[i][1],
                            self._bare[i + 1][0], self._bare[i + 1][1]
                        ) * 1852.0
                    self._vpa_anchor_remaining_m = remaining

    @property
    def active(self) -> bool:
        return bool(self._bare)

    def guidance(self, state, climb_fpm: float = 1500.0,
                 descend_fpm: float = 1800.0) -> ProcedureGuidance:
        if not self._bare:
            return ProcedureGuidance(done=True, note="no path")
        lat, lon = state.latitude, state.longitude
        steer = auto_nav.path_steering(
            lat, lon, self._bare, lookahead_m=PROCEDURE_LOOKAHEAD_M)
        if steer is None:
            return ProcedureGuidance(done=True, note="no steering")
        index = min(max(steer.get("nearest", 0) + 1, 0),
                    len(self.points) - 1)
        _lat, _lon, alt, speed = self.points[index]
        current_alt = state.altitude_ft or 0.0
        # A SID's levels are floors the climb makes through; a STAR's are
        # step-downs the descent holds to. The fallback is the clearance's
        # own level for the SID and the arrival floor for the STAR.
        if self.procedure.kind == "sid":
            # A SID's levels are floors: never level below the leg's own
            # altitude, and keep climbing to the clearance's target.
            altitude = max(self.fallback_altitude_ft, alt or 0.0)
            vs = climb_fpm if altitude and current_alt < altitude - 250 \
                else 0.0
        else:
            altitude = alt or self.fallback_altitude_ft
            if altitude and current_alt > altitude + 250:
                vs = -descend_fpm
            elif altitude and current_alt < altitude - 250:
                # Already under the step-down: hold here rather than let
                # the altitude bug climb back to a constraint passed.
                altitude = current_alt
                vs = 0.0
            else:
                vs = 0.0
        target_speed = speed or self.fallback_speed_kt
        # A procedure's speed limit is a limit, not a target: only slow to
        # it when the assigned speed is above it.
        if speed and self.fallback_speed_kt and \
                self.fallback_speed_kt > speed:
            target_speed = speed
        altitude_distance_nm = (haversine_nm(lat, lon, _lat, _lon)
                                if alt else 0.0)
        vpa_per_nm = 0.0
        if self.procedure.kind == "approach" and self._vpa_ft_per_nm:
            distance = max(0.0, (float(steer.get("remaining_m") or 0.0)
                                 - self._vpa_anchor_remaining_m) / 1852.0)
            if self._vpa_alt_ft:
                altitude = self._vpa_alt_ft
                altitude_distance_nm = distance
                vpa_per_nm = self._vpa_ft_per_nm
                if current_alt < altitude - 100.0:
                    vs = 0.0
                elif current_alt > altitude + distance * vpa_per_nm + 100.0:
                    vs = -descend_fpm
                else:
                    vs = 0.0
        return ProcedureGuidance(
            heading=steer.get("heading"), altitude_ft=altitude, vs_fpm=vs,
            speed_kt=target_speed,
            done=bool(steer.get("done")), note=self.notes[0]
            if self.notes else "",
            altitude_distance_nm=altitude_distance_nm,
            vpa_ft_per_nm=vpa_per_nm)


__all__ = [
    "AirportProcedures",
    "ProcedureLeg",
    "ProcedureNavigator",
    "ProcedureTransition",
    "TerminalProcedure",
    "build_path",
    "choose_sid",
    "choose_star",
]
