"""The approach the First Officer actually flies, as geometry.

The controller names an approach -- "ILS Zulu", "RNAV Yankee", "VOR-A" --
and keeps the choice on the session (:attr:`FlightSession.approach_choice`).
What the aeroplane needs is not the name but a recipe: which way the final
course runs, where the final approach fix is, what the minima are, and which
navaid the radios are tuned to. This module turns the choice into that, once,
so :mod:`meridianatc.atc.autonomous` can ask "what am I flying" instead of
re-deriving it every tick.

What the data can and cannot say
--------------------------------

The simulator's facility data names each published procedure -- kind, letter,
runway, final-approach-fix and missed-approach altitudes -- but not its legs.
There is no chart here, so a procedure's *shape* is approximated: the final
course is the landing direction itself for every straight-in type (the one
exception is the localizer back course, which runs the other way), the final
approach fix is that many feet above the field on a three-degree path, and a
VOR or NDB approach tunes the navaid the runway end publishes rather than a
radial read off a chart. Approaches whose published course is deliberately
offset from the runway -- some LOC, LDA and SDF -- are flown on the runway's
course; each place the approximation matters is marked UNVERIFIED.

A circling-only procedure (VOR-A) names no runway of its own: the recipe
keeps the landing runway and carries the circling pattern's own legs and
minima.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..navdata.build import haversine_nm, project
from ..navdata.db import normalize_runway
from .approaches import (
    AIRLINER_LINES,
    CIRCLING,
    GA_LINES,
    MINIMA,
    RNAV_LINES,
    VISUAL,
    legacy_kind,
)
from .final import final_for

# The approach kinds grouped by how they are flown. The catalogue's kinds
# (approaches.KINDS) fold into these: a VOR/DME is a VOR flown with distance
# information, an RNAV's lines are its minima, not its geometry.
FAMILIES = {
    "ILS": "ils",
    "LOC": "loc", "LDA": "loc", "SDF": "loc", "LOC BC": "loc",
    "VOR": "vor", "NDB": "ndb",
    "RNAV": "rnav", "RNP": "rnav",
    VISUAL: "visual",
}

# Feet per nautical mile on a three-degree path -- auto_nav.glide's figure.
FT_PER_NM = 318.0
# The height the path crosses the threshold at: the fifty feet every
# glide path is drawn through, not the pavement itself.
TCH_FT = 50.0
# Where the computed final starts when no published FAF altitude is known:
# five miles is the middle of a non-precision final.
DEFAULT_FAF_NM = 5.0
FAF_MIN_NM, FAF_MAX_NM = 2.5, 15.0
# The intercept gate: a point this far behind the FAF on the same course,
# so path following has a leg to capture rather than starting on the fix.
GATE_NM = 3.0
# How far past the threshold the flown path ends, so "reached the end" is
# unambiguous.
PAST_THRESHOLD_NM = 0.3

# The circling pattern, in the landing runway's frame: the downwind leg's
# end past the threshold, its offset, and where the base leg rolls out on
# final.
CIRCLE_DOWNWIND_NM = -2.5    # past the threshold, along the landing course
CIRCLE_OFFSET_NM = 2.0
CIRCLE_BASE_OUT_NM = 3.5

# The compass words a circle restriction is spoken in, as bearings.
_CIRCLE_SIDES = {"north": 0.0, "east": 90.0, "south": 180.0, "west": 270.0}


@dataclass(frozen=True)
class Navaid:
    """The ground station an approach is tuned to and tracked from."""

    ident: str
    freq_mhz: float
    lat: float
    lon: float
    kind: str = ""          # VOR, NDB, LOC, ILS -- the database's word


@dataclass(frozen=True)
class ApproachRecipe:
    """One cleared approach, resolved for flying."""

    ident: str                    # the field's ICAO
    name: str                     # as charted: "ILS Z", "RNAV Y", "VOR-A"
    family: str                   # ils, loc, vor, ndb, rnav, visual
    approach_runway: str          # the runway the procedure serves
    landing_runway: str           # where the wheels go
    line: str = ""                # the RNAV minima line flown: LPV, LNAV...
    minima_ft: float = 200.0      # height above the touchdown zone
    circle_minima_ft: float = 0.0     # circling MDA, when circling
    final_course: float = 0.0     # true degrees on final
    threshold_lat: float = 0.0
    threshold_lon: float = 0.0
    field_elev_ft: float = 0.0
    faf_lat: float = 0.0
    faf_lon: float = 0.0
    faf_alt_ft: float = 0.0
    missed_alt_ft: float = 0.0
    navaid: Navaid | None = None
    circling: bool = False
    # The published legs where the feed decodes them (the CIFP feed):
    # (lat, lon, alt_ft, speed_kt) points from IAF to the missed-approach
    # point. The synthetic gate-FAF-threshold path stands in where the
    # legs are not known -- a straight-in that does not exist wherever
    # the chart draws something else, like LECO's RNP 03 offset final.
    leg_path: tuple = ()
    map_lat: float = 0.0           # the missed-approach point
    map_lon: float = 0.0
    map_course: float = 0.0        # the course the final segment flies
    # The final segment does not run to the runway: an offset final
    # (RNP 03 at LECO is eighty degrees off the runway's axis) ends at
    # the missed-approach point and a visual manoeuvre lands it --
    # charted there as the VPT, flown here as the circling pattern.
    offset_final: bool = False
    # The clearance's circle restriction as spoken -- "north" etc; the
    # side of the field the manoeuvre has to stay on.
    circle_side: str = ""
    back_course: bool = False   # a localizer back course: needles sense reversed
    why: str = ""

    @property
    def has_navaid(self) -> bool:
        return self.navaid is not None and self.navaid.freq_mhz > 0


def _family(kind: str) -> str:
    return FAMILIES.get(kind, "")


def _threshold_end(navdb, ident: str, runway: str):
    """(lat, lon, true course, elev_ft) of the landing threshold."""
    found = final_for(navdb, ident, runway)
    if found is None:
        return None
    elev = 0.0
    wanted = normalize_runway(runway)
    for rw in navdb.runways(ident):
        for end, prefix in ((rw.le_ident, "le"), (rw.he_ident, "he")):
            if normalize_runway(end or "") != wanted:
                continue
            elev = float(getattr(rw, f"{prefix}_elev_ft", 0.0) or 0.0)
    return found.lat, found.lon, found.course, elev or 0.0


def _as_navaid(row: dict | None) -> Navaid | None:
    if not row:
        return None
    try:
        lat, lon = float(row["lat"]), float(row["lon"])
    except (KeyError, TypeError, ValueError):
        return None
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        return None
    return Navaid(ident=(row.get("ident") or "").strip().upper(),
                  freq_mhz=float(row.get("freq_khz") or 0.0) / 1000.0,
                  lat=lat, lon=lon,
                  kind=(row.get("type") or "").strip().upper())


def _navaid(navdb, ident: str, runway: str, family: str,
            near: tuple[float, float]) -> Navaid | None:
    """The station this approach is tuned to, or None.

    The runway end's own navaid first -- the localizer or the VOR the
    scenery puts at the threshold -- then the nearest navaid of the right
    kind for a field whose runways carry no idents (the database's).
    """
    wanted = normalize_runway(runway)
    end_ident, end_kind = "", ""
    for rw in navdb.runways(ident):
        for end, ils, kind in ((rw.le_ident, rw.le_ils, rw.le_ils_kind),
                               (rw.he_ident, rw.he_ils, rw.he_ils_kind)):
            if normalize_runway(end or "") != wanted:
                continue
            end_ident = (ils or "").strip().upper()
            end_kind = (kind or "")
    # The end's navaid serves the localizer families, and a VOR the scenery
    # marks as one; an NDB approach looks for an NDB.
    if end_ident and (family in ("ils", "loc")
                      or (family == "vor" and end_kind == "VOR")):
        found = _as_navaid(navdb.navaid(end_ident, near=near))
        if found:
            return found
    types = {"vor": ("VOR", "VORTAC", "VOR-DME", "TACAN"),
             "ndb": ("NDB",)}.get(family)
    if not types:
        return None
    best = None
    for row, dist in navdb.nearest_navaids(near[0], near[1], limit=5,
                                           max_nm=60.0, types=types):
        if best is None or dist < best[1]:
            best = (row, dist)
    return _as_navaid(best[0]) if best else None


def _minima(choice, airliner: bool) -> tuple[float, float, str]:
    """(height above the zone, visibility, line) for the flown procedure."""
    approach = getattr(choice, "approach", None)
    if approach is None:
        return 200.0, 0.0, ""
    lines = AIRLINER_LINES if airliner else GA_LINES
    height, vis, line = approach.minima(lines)
    if not line:
        # The aeroplane cannot fly any of an RNAV's lines: the chart's
        # least demanding RNAV figure is still the honest floor.
        height, vis = min((MINIMA.get(l, MINIMA["NDB"]) for l in RNAV_LINES),
                          key=lambda m: m[0])
        line = approach.kind
    return float(height or 200.0), float(vis or 0.0), line


def _published(brain, destination, runway: str, name: str, which: int
               ) -> float:
    """A procedure altitude the simulator publishes, or 0 -- the brain's
    own sanity-banded lookup, so the FO and the clearance agree."""
    if brain is None:
        return 0.0
    try:
        return float(brain._published(destination, runway, name, which))
    except (AttributeError, TypeError, ValueError, KeyError, IndexError):
        return 0.0


def resolve(session, brain, navdb, state) -> ApproachRecipe | None:
    """The cleared approach as a recipe, or None when there is no field.

    ``session.approach_choice`` is the controller's own answer where it has
    been computed; a session that never heard one (a flight begun inside
    the arrival, the catalogue switched off) gets the same answer asked for
    now, which is also what caches it for the radio work. A choice made for
    a different field or a different landing runway is asked again.
    """
    destination = getattr(session, "destination", None)
    if destination is None and getattr(state, "has_position", False):
        destination = navdb.home_airport(state.latitude, state.longitude)
    if destination is None:
        return None
    ident = getattr(destination, "ident", "") or ""

    runway = getattr(session, "arrival_runway", "") or ""
    if not runway:
        runways = list(navdb.runways(ident) or [])
        if runways:
            runway = getattr(max(runways, key=lambda r: r.length_ft or 0),
                             "le_ident", "") or ""
    if not runway:
        return None

    choice = getattr(session, "approach_choice", None)
    key = getattr(session, "approach_key", ()) or ()
    chosen_landing = ""
    if choice is not None:
        chosen_landing = getattr(choice, "circle_to", "") or \
            getattr(choice, "runway", "") or ""
    stale = choice is None or not key or key[0] != ident \
        or (chosen_landing and normalize_runway(chosen_landing)
            != normalize_runway(runway))
    if stale:
        choice = brain.approach_choice(session, destination, runway,
                                       weather=None)
    approach = getattr(choice, "approach", None)
    if approach is None:
        return None

    kind = getattr(approach, "kind", "") or ""
    name = getattr(approach, "name", "") or kind
    family = _family(kind) or _family(legacy_kind(name))
    if not family:
        return None
    landing = getattr(choice, "circle_to", "") or \
        getattr(choice, "runway", "") or runway
    approach_runway = getattr(approach, "runway", "") or landing

    end = _threshold_end(navdb, ident, landing)
    if end is None:
        return None
    t_lat, t_lon, course, elev = end
    # A back course still lands *on* the runway -- the final course is the
    # landing direction; only the needle's sensing runs the other way,
    # which is why the autopilot must not fly it in front-course mode.
    back_course = kind == "LOC BC"

    airliner = bool(getattr(getattr(session, "profile", None),
                            "airliner", True))
    minima_ft, _vis, line = _minima(choice, airliner)
    category = "C" if airliner else "A"
    circle_ft = float(CIRCLING.get(category, CIRCLING["C"])[0]) \
        if getattr(choice, "circle_to", "") else 0.0

    # The final approach fix: the published altitude where the simulator
    # gives one, else a five-mile fix on the path. UNVERIFIED: the FAF
    # *distance* is derived from the altitude on a three-degree path, not
    # read off the procedure.
    faf_ft = _published(brain, destination, approach_runway, name, 0)
    missed_ft = _published(brain, destination, approach_runway, name, 1)
    if not faf_ft:
        faf_ft = float(getattr(approach, "faf_ft", 0.0) or 0.0)
    if not missed_ft:
        missed_ft = float(getattr(approach, "missed_ft", 0.0) or 0.0)
    elev = elev or float(getattr(destination, "elev_ft", 0.0) or 0.0)
    faf_dist = min(FAF_MAX_NM, max(
        FAF_MIN_NM,
        (faf_ft - elev) / FT_PER_NM if faf_ft > elev else DEFAULT_FAF_NM))
    faf_lat, faf_lon = project(t_lat, t_lon, (course + 180.0) % 360.0,
                               faf_dist)

    navaid = _navaid(navdb, ident, approach_runway, family,
                     (t_lat, t_lon)) if family in ("ils", "loc", "vor",
                                                   "ndb") else None

    # The published legs where the feed carries them: the path the chart
    # draws instead of the synthetic final -- RF arcs and all. The last
    # leg's fix is the missed-approach point and its course is the final
    # segment's, which for an offset final is not the runway's at all.
    leg_path: tuple = ()
    map_lat = map_lon = map_course = 0.0
    offset_final = False
    published = tuple(getattr(approach, "legs", ()) or ())
    if published:
        from .procedure_legs import ProcedureTransition, build_path
        try:
            points, _notes = build_path(
                ProcedureTransition(name=name, runway=approach_runway,
                                    legs=published))
        except Exception:
            points = []
        if len(points) >= 2:
            leg_path = tuple(points)
            map_lat, map_lon = points[-1][0], points[-1][1]
            map_course = _segment_course(points)
            offset = abs((map_course - course + 540.0) % 360.0 - 180.0) \
                if map_course > 0 else 0.0
            offset_final = offset > 25.0
            # The published FAF, where a leg carries the mark.
            faf_leg = next((leg for leg in published if leg.is_faf
                            and leg.has_fix), None)
            if faf_leg is not None:
                faf_lat, faf_lon = faf_leg.lat, faf_leg.lon
                faf_alt = faf_leg.altitude_constraint or faf_leg.fix_alt_ft
                if faf_alt:
                    faf_dist = max(FAF_MIN_NM,
                                   min(FAF_MAX_NM,
                                       (faf_alt - elev) / FT_PER_NM))

    return ApproachRecipe(
        ident=ident, name=name, family=family,
        approach_runway=approach_runway, landing_runway=landing, line=line,
        minima_ft=minima_ft or 200.0,
        circle_minima_ft=circle_ft or (minima_ft if offset_final else 0.0),
        final_course=course,
        threshold_lat=t_lat, threshold_lon=t_lon, field_elev_ft=elev,
        faf_lat=faf_lat, faf_lon=faf_lon,
        faf_alt_ft=elev + faf_dist * FT_PER_NM,
        missed_alt_ft=missed_ft or elev + 2500.0,
        navaid=navaid,
        circling=bool(getattr(choice, "circle_to", "") or offset_final),
        circle_side=getattr(choice, "circle_side", "") or "",
        leg_path=leg_path, map_lat=map_lat, map_lon=map_lon,
        map_course=map_course, offset_final=offset_final,
        back_course=back_course,
        why=getattr(choice, "why", "") or "")


def _segment_course(points) -> float:
    """The course the last built segment runs, degrees true."""
    if len(points) < 2:
        return 0.0
    from ..navdata.build import bearing_deg
    a, b = points[-2], points[-1]
    return bearing_deg(a[0], a[1], b[0], b[1])


def legs(recipe: ApproachRecipe) -> list[tuple[float, float]]:
    """The final as a polyline: the published legs where they are known,
    else the computed gate, FAF and past the threshold."""
    if recipe.leg_path:
        path = [(lat, lon) for lat, lon, _alt, _speed in recipe.leg_path]
        # The legs end at the missed-approach point; the flight does not.
        # The threshold and a stretch past it keep the steering forward
        # -- without them a path that ends short of the runway pulls the
        # aeroplane back to the last fix instead of landing it.
        if recipe.threshold_lat or recipe.threshold_lon:
            last = path[-1]
            if haversine_nm(last[0], last[1], recipe.threshold_lat,
                            recipe.threshold_lon) > 0.05:
                path.append((recipe.threshold_lat, recipe.threshold_lon))
            past = project(recipe.threshold_lat, recipe.threshold_lon,
                           recipe.final_course, 0.3)
            path.append(past)
        return path
    faf_nm = max(0.0, recipe.faf_alt_ft - recipe.field_elev_ft) / FT_PER_NM
    gate_lat, gate_lon = project(recipe.threshold_lat, recipe.threshold_lon,
                                 (recipe.final_course + 180.0) % 360.0,
                                 faf_nm + GATE_NM)
    end_lat, end_lon = project(recipe.threshold_lat, recipe.threshold_lon,
                               recipe.final_course, PAST_THRESHOLD_NM)
    return [(gate_lat, gate_lon), (recipe.faf_lat, recipe.faf_lon),
            (end_lat, end_lon)]


def profile_alt(recipe: ApproachRecipe, distance_nm: float) -> float:
    """Where the three-degree path is, that far from the threshold."""
    return (recipe.field_elev_ft + TCH_FT
            + max(0.0, distance_nm) * FT_PER_NM)


def sink_fpm(groundspeed_kt: float) -> float:
    """The descent rate a three-degree path asks for at this speed."""
    return max(250.0, min(900.0,
                          abs(float(groundspeed_kt or 0.0)) * FT_PER_NM / 60.0))


def frame_point(final_lat: float, final_lon: float, course: float,
                out_nm: float, side_nm: float) -> tuple[float, float]:
    """The (lat, lon) ``out``/``side`` miles from a threshold in its frame --
    the inverse of :func:`meridianatc.atc.procedures.frame`. ``out`` is along
    the extended centreline, positive on the approach side; ``side`` is
    across it, positive right of the landing direction."""
    if out_nm >= 0.0:
        lat, lon = project(final_lat, final_lon, (course + 180.0) % 360.0,
                           out_nm)
    else:
        lat, lon = project(final_lat, final_lon, course, -out_nm)
    if side_nm:
        bearing = (course + 90.0) % 360.0 if side_nm > 0 \
            else (course - 90.0) % 360.0
        lat, lon = project(lat, lon, bearing, abs(side_nm))
    return lat, lon


def circle_legs(recipe: ApproachRecipe, navdb, state) -> list | None:
    """The circling pattern as a polyline to the landing runway.

    A rectangle in the landing runway's frame: join on the downwind side,
    fly outbound past the threshold, turn base, and roll out on final.
    UNVERIFIED: real circling procedures are sector-and-altitude specific;
    this is the generic rectangle every simulator circuit uses.
    """
    final = final_for(navdb, recipe.ident, recipe.landing_runway)
    if final is None:
        return None
    from .procedures import frame

    _out, side = frame(final, state.latitude, state.longitude)
    bearing = _CIRCLE_SIDES.get(recipe.circle_side)
    if bearing is not None:
        # The clearance restricted the circle to one side of the field:
        # right of the landing course or left of it, whichever the
        # spoken compass side falls on.
        s = 1.0 if math.sin(math.radians(bearing - final.course)) > 0 \
            else -1.0
    else:
        s = 1.0 if side >= 0 else -1.0
    points = [
        frame_point(final.lat, final.lon, final.course,
                    CIRCLE_DOWNWIND_NM, s * CIRCLE_OFFSET_NM),
        # The base leg does not arrive at final squared onto it: a
        # shallow intercept two miles further out, rolled onto the
        # centreline at under thirty degrees. Coming in steeper was the
        # rollout that reached the flare still turning, too misaligned
        # to ever stabilize inside the gate.
        frame_point(final.lat, final.lon, final.course,
                    CIRCLE_BASE_OUT_NM + 2.0, s * CIRCLE_OFFSET_NM * 0.5),
        frame_point(final.lat, final.lon, final.course,
                    CIRCLE_BASE_OUT_NM, 0.0),
        frame_point(final.lat, final.lon, final.course,
                    PAST_THRESHOLD_NM, 0.0),
    ]
    return [(state.latitude, state.longitude)] + points


__all__ = [
    "FAMILIES",
    "FT_PER_NM",
    "PAST_THRESHOLD_NM",
    "TCH_FT",
    "ApproachRecipe",
    "Navaid",
    "circle_legs",
    "frame_point",
    "legs",
    "profile_alt",
    "resolve",
    "sink_fpm",
]
