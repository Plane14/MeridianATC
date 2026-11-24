"""The filed route as a controller uses it: a line on the chart, not a list
of names.

Before this, the plan was kept as SimBrief's raw JSON and read in two
places for two lists -- (lat, lon) points for the en-route look-ahead and
named fixes for "proceed direct" -- and everything else the controller said
was measured as a great circle to the airport's reference point: when to
start the descent, when to hand to approach, how far there was still to go.
An aeroplane on a STAR that doglegs forty miles round a TMA is not forty
miles nearer for being abeam the field.

So the route is data here: every fix in order with its coordinates, the
altitude the plan crosses it at, whether it is climb, cruise or descent, the
planned time from takeoff, and the airway, SID or STAR it is reached by. On
top of that, the questions a controller asks of a plan (after LocalTC's
``route.py``, read for the design only):

- where the climb ends and the descent starts: the plan's own ``TOC`` and
  ``TOD`` when it names them, else the last climb fix and the fix before the
  first descending one;
- the arrival floor: the last altitude the plan descends to before the
  airport, rounded up to the thousand -- where "descend via" ends;
- the fixes of a SID or a STAR by name;
- how many minutes after departure the cruise is reached (the plan's TOC
  time, else its distance at a climb groundspeed, else an average rate),
  clamped to what "expect FL370 N minutes after departure" can say;
- how far out the descent starts;
- and where the aeroplane is along the route: the active leg, how far off
  it, the distance still to fly *along the route*, the next fix and the time
  to each fix still ahead.

Three loaders, in the order a flight prefers them: SimBrief's navigation log
(every fix with its coordinates), the simulator's flight plan file (``.pln``,
which in MSFS 2024 names each fix and its region but carries no coordinates,
so each is resolved), and a route string typed by hand, resolved the same
way. Resolution is the simulator's facility data first and the navaids and
airports in ``nav.sqlite`` after it; the database has no intersections and
no airways, so offline a string resolves only its navaids and an airway is
flown direct between the two fixes named either side of it -- said in the
route's ``notes`` rather than hidden.
"""

from __future__ import annotations

import logging
import math
import re
import xml.etree.ElementTree as ET
from collections.abc import Callable
from dataclasses import dataclass, field

from ..navdata.build import bearing_deg, haversine_nm

log = logging.getLogger(__name__)

CLB, CRZ, DSC = "CLB", "CRZ", "DSC"

# Without a plan's times: an average climb rate by where the cruise is, jets
# up high and turboprops and pistons lower (after LocalTC: 1,800 fpm to a
# cruise at or above 24,000 ft, 1,000 from 12,000, 600 below).
CLIMB_FPM = ((24000, 1800.0), (12000, 1000.0), (0, 600.0))
# The descent rule of thumb, three miles a thousand feet, and never a descent
# that starts inside thirty miles (after LocalTC).
TOD_NM_PER_1000FT = 3.0
TOD_MIN_NM = 30.0
# What "expect FL370 N minutes after departure" can sensibly say (after
# LocalTC): under five is not worth saying, over thirty is not a climb.
MINUTES_RANGE = (5, 30)


def climb_groundspeed(cruise_ft: float) -> float:
    """A typical average groundspeed over the whole climb, knots (after
    LocalTC: 330 to a cruise in the high levels, 200 to the middle, 110 for
    a light aeroplane)."""
    return 330.0 if cruise_ft >= 24000 else 200.0 if cruise_ft >= 12000 \
        else 110.0


@dataclass(frozen=True)
class RouteFix:
    """One point of the route."""

    ident: str
    lat: float
    lon: float
    alt_ft: int = 0
    stage: str = ""        # CLB, CRZ or DSC; "" when the plan does not say
    time_s: int = 0        # planned seconds from takeoff; 0 when not given
    via: str = ""          # airway, SID or STAR it is reached by, "DCT", ""
    kind: str = ""         # wpt, vor, ndb, apt, ltlg (a plan's TOC/TOD)
    region: str = ""       # the ICAO region the ident is unique in
    on_procedure: bool = False   # SimBrief's is_sid_star

    @property
    def is_marker(self) -> bool:
        """Not a fix anybody is cleared to: the plan's top of climb and
        descent, and the airports at either end."""
        ident = self.ident.upper()
        return ident in ("TOC", "TOD") or self.kind == "apt" \
            or (len(ident) == 4 and ident.isalpha() and self.kind in ("", "apt"))


@dataclass(frozen=True)
class Progress:
    """Where the aeroplane is along the route, at one moment."""

    leg: int                 # index of the fix the active leg runs *to*
    from_fix: RouteFix
    to_fix: RouteFix
    cross_track_nm: float    # positive right of the leg, negative left
    along_leg_nm: float      # flown of the active leg so far
    to_go_nm: float          # still to fly along the route to its last fix
    to_next_nm: float        # to the end of the active leg
    # (ident, seconds) for every fix still ahead, at the groundspeed given
    eta_s: tuple[tuple[str, float], ...] = ()

    @property
    def next_fix(self) -> RouteFix:
        return self.to_fix

    @property
    def leg_name(self) -> str:
        return f"{self.from_fix.ident}-{self.to_fix.ident}"


@dataclass
class Route:
    """The filed route, and what a controller works out from it."""

    fixes: tuple[RouteFix, ...] = ()
    origin: str = ""
    destination: str = ""
    alternate: str = ""
    sid: str = ""
    sid_runway: str = ""
    sid_transition: str = ""
    star: str = ""
    star_transition: str = ""
    star_runway: str = ""
    approach: str = ""        # e.g. "ILS W 16L" as the plan names it
    cruise_ft: int = 0
    source: str = ""          # simbrief, pln, string, sim
    # What the loader could not do, said rather than hidden: a fix nobody
    # could place, an airway flown direct for want of its points.
    notes: tuple[str, ...] = ()

    def __bool__(self) -> bool:
        return len(self.fixes) >= 2

    # ------------------------------------------------------------------
    # the plan's own answers
    # ------------------------------------------------------------------

    @property
    def top_of_climb(self) -> RouteFix | None:
        named = next((f for f in self.fixes if f.ident.upper() == "TOC"), None)
        return named or next(
            (f for f in reversed(self.fixes) if f.stage == CLB), None)

    @property
    def top_of_descent(self) -> RouteFix | None:
        named = next((f for f in self.fixes if f.ident.upper() == "TOD"), None)
        if named is not None:
            return named
        first_down = next((i for i, f in enumerate(self.fixes)
                           if f.stage == DSC), None)
        return self.fixes[first_down - 1] if first_down else None

    @property
    def arrival_floor_ft(self) -> int | None:
        """The last altitude the plan descends to before the airport: where
        "descend via" ends. Not the airport itself, which a SimBrief plan
        lists last at circuit height, but the arrival's last fix."""
        descent = [f.alt_ft for f in self.fixes
                   if f.stage == DSC and f.alt_ft > 0 and not f.is_marker]
        if not descent:
            return None
        return int(math.ceil(descent[-1] / 1000.0) * 1000)

    def procedure(self, name: str | None, stage: str) -> tuple[RouteFix, ...]:
        """The fixes of a SID (``stage`` CLB) or a STAR (DSC), in order.

        A plan that kept each fix's ``via`` names them. One that did not has
        only the stages: a SID is what is climbed through before the top of
        climb, a STAR what is flown after the top of descent.
        """
        if not name:
            return ()
        if any(f.via for f in self.fixes):
            return tuple(f for f in self.fixes
                         if f.via.upper() == name.upper() and not f.is_marker)
        if stage == CLB:
            end = next((i for i, f in enumerate(self.fixes)
                        if f.stage != CLB or f.ident.upper() == "TOC"),
                       len(self.fixes))
            return tuple(f for f in self.fixes[:end] if not f.is_marker)
        top = self.top_of_descent
        start = self.fixes.index(top) + 1 if top is not None \
            else len(self.fixes)
        return tuple(f for f in self.fixes[start:]
                     if f.stage == DSC and not f.is_marker)

    def minutes_to_cruise(self, cruise_ft: float | None = None) -> int:
        """Minutes from takeoff to the top of the climb."""
        cruise = float(cruise_ft or self.cruise_ft or 0)
        toc = self.top_of_climb
        if toc is not None and toc.time_s > 0:
            minutes = toc.time_s / 60.0
        elif toc is not None and len(self.fixes) > 1:
            minutes = (self._along(0, self.fixes.index(toc))
                       / climb_groundspeed(cruise) * 60.0)
        else:
            rate = next((fpm for floor, fpm in CLIMB_FPM if cruise >= floor),
                        CLIMB_FPM[-1][1])
            minutes = cruise / rate
        low, high = MINUTES_RANGE
        return int(min(max(round(minutes), low), high))

    def descent_distance_nm(self, dest_lat: float, dest_lon: float,
                            cruise_ft: float | None = None,
                            elev_ft: float = 0.0) -> float:
        """How far from the destination the descent starts: along the route
        from the plan's top of descent when it has one, else three miles a
        thousand feet, never under thirty."""
        tod = self.top_of_descent
        if tod is not None and tod in self.fixes:
            i = self.fixes.index(tod)
            last = self.fixes[-1]
            along = self._along(i, len(self.fixes) - 1)
            # The plan's last point is the airport, or near enough; if it is
            # not, the rest of the way is flown direct.
            along += haversine_nm(last.lat, last.lon, dest_lat, dest_lon)
            return along
        cruise = float(cruise_ft or self.cruise_ft or 0)
        return max(TOD_MIN_NM, (cruise - elev_ft) / 1000.0 * TOD_NM_PER_1000FT)

    # ------------------------------------------------------------------
    # where the aeroplane is on it
    # ------------------------------------------------------------------

    def progress(self, lat: float, lon: float, groundspeed_kt: float = 0.0,
                 hint: int = 1) -> Progress | None:
        """The active leg and the distance still to fly along the route.

        The leg is the one the aeroplane is nearest to across track, among
        the legs it lies alongside, searched from ``hint`` (the last active
        leg) onwards first: a route that comes back on itself -- a STAR
        that runs downwind past the field it started abeam -- must not snap
        the aeroplane back to the leg it flew twenty minutes ago.
        """
        fixes = self.fixes
        if len(fixes) < 2:
            return None
        hint = max(1, min(int(hint or 1), len(fixes) - 1))
        best = None
        for i in range(1, len(fixes)):
            a, b = fixes[i - 1], fixes[i]
            along, across, length = _leg_position(a, b, lat, lon)
            if length < 0.01:
                continue
            # Off either end of the leg, the distance is to its nearer end.
            if along < 0:
                distance = math.hypot(along, across)
            elif along > length:
                distance = math.hypot(along - length, across)
            else:
                distance = abs(across)
            # A leg already flown counts as further away than it is, so a
            # loop in the route is resolved forwards; the leg just behind the
            # hint is allowed, for a position read a moment before the turn.
            if i < hint - 1:
                distance += 50.0
            key = (distance, i < hint)
            if best is None or key < best[0]:
                best = (key, i, along, across, length)
        if best is None:
            return None
        _, i, along, across, length = best
        a, b = fixes[i - 1], fixes[i]
        flown = min(max(along, 0.0), length)
        to_next = length - flown
        to_go = to_next + self._along(i, len(fixes) - 1)
        etas = []
        if groundspeed_kt > 30:
            running = to_next
            etas.append((b.ident, running / groundspeed_kt * 3600.0))
            for j in range(i + 1, len(fixes)):
                p, q = fixes[j - 1], fixes[j]
                running += haversine_nm(p.lat, p.lon, q.lat, q.lon)
                etas.append((q.ident, running / groundspeed_kt * 3600.0))
        return Progress(leg=i, from_fix=a, to_fix=b,
                        cross_track_nm=across, along_leg_nm=flown,
                        to_go_nm=to_go, to_next_nm=to_next,
                        eta_s=tuple(etas))

    def fix(self, ident: str) -> RouteFix | None:
        ident = (ident or "").strip().upper()
        return next((f for f in self.fixes if f.ident.upper() == ident), None)

    def fix_index(self, ident: str, hint: int = -1, after: int = 0) -> int:
        """The place ``ident`` is at on the route, or -1 when it is not.

        ``fix`` answers the first occurrence, which on a route that visits
        a fix twice -- an out-and-back leg, a STAR that rejoins the airway
        it left -- is not where the flight is going. ``hint`` is a known
        answer (the index kept when the fix was cleared to): honoured while
        it still names the ident. ``after`` scopes the search to the part
        of the route still ahead -- the leg the aeroplane is on or about
        to fly -- before the whole list is tried, so "direct BBBBB" means
        the BBBBB still to come, not the one behind.
        """
        ident = (ident or "").strip().upper()
        fixes = self.fixes
        if ident and 0 <= hint < len(fixes) \
                and fixes[hint].ident.upper() == ident:
            return hint
        for i in range(max(0, after), len(fixes)):
            if fixes[i].ident.upper() == ident:
                return i
        for i in range(min(max(0, after), len(fixes))):
            if fixes[i].ident.upper() == ident:
                return i
        return -1

    def points(self) -> list[tuple[float, float]]:
        return [(f.lat, f.lon) for f in self.fixes]

    def described(self) -> str:
        """One line for the log: where from, by what, to where."""
        bits = [self.origin or "?"]
        if self.sid:
            bits.append(self.sid)
        named = [f.ident for f in self.fixes
                 if not f.is_marker and not f.on_procedure]
        if named:
            bits.append(" ".join(named[:12]) + (" ..." if len(named) > 12
                                                else ""))
        if self.star:
            bits.append(self.star)
        bits.append(self.destination or "?")
        return " ".join(bits)

    def _along(self, first: int, last: int) -> float:
        return sum(haversine_nm(a.lat, a.lon, b.lat, b.lon)
                   for a, b in zip(self.fixes[first:last],
                                   self.fixes[first + 1:last + 1], strict=False))


def _leg_position(a: RouteFix, b: RouteFix, lat: float, lon: float
                  ) -> tuple[float, float, float]:
    """(along, across, length) of a point against the leg a -> b, in nm.

    Flat-earth about the leg's start is plenty: a leg is at most a few
    hundred miles and the question is which leg and roughly how far off,
    not a geodesic to the metre. Across is positive right of the leg.
    """
    length = haversine_nm(a.lat, a.lon, b.lat, b.lon)
    if length < 1e-6:
        return 0.0, 0.0, 0.0
    course = math.radians(bearing_deg(a.lat, a.lon, b.lat, b.lon))
    distance = haversine_nm(a.lat, a.lon, lat, lon)
    if distance < 1e-6:
        return 0.0, 0.0, length
    angle = math.radians(bearing_deg(a.lat, a.lon, lat, lon)) - course
    return distance * math.cos(angle), distance * math.sin(angle), length


# ----------------------------------------------------------------------
# SimBrief
# ----------------------------------------------------------------------


def from_ofp(plan) -> Route:
    """The route of a SimBrief plan (:class:`integrations.simbrief.Ofp`)."""
    navlog = tuple(getattr(plan, "navlog", ()) or ())
    fixes = tuple(
        RouteFix(ident=f.ident, lat=f.lat, lon=f.lon, alt_ft=f.alt_ft,
                 stage=f.stage, time_s=f.time_s, via=_via(f.via),
                 kind=f.kind, region=f.region, on_procedure=f.on_procedure)
        for f in navlog)
    # SimBrief names the SID's last fix's ``via`` after the SID even where
    # the flag says it is no longer on it (ROUSY on the ROUS1L, measured on
    # an EBBR-LIRF plan): the name is the procedure, so it stays.
    #
    # Its log starts at the SID's first fix and ends on the destination, so
    # the origin is put in front: without it the first leg -- the one the
    # aeroplane is on from the moment it lifts off -- does not exist.
    origin = getattr(plan, "origin_position", None)
    if origin and fixes and fixes[0].ident != plan.origin:
        fixes = (RouteFix(ident=plan.origin, lat=origin[0], lon=origin[1],
                          alt_ft=int(origin[2]), stage=CLB, kind="apt"),) \
            + fixes
    return Route(
        fixes=fixes, origin=getattr(plan, "origin", ""),
        destination=getattr(plan, "destination", ""),
        alternate=getattr(plan, "alternate", ""),
        sid=getattr(plan, "sid", ""),
        sid_runway=getattr(plan, "planned_runway", ""),
        star=getattr(plan, "star", ""),
        star_runway=getattr(plan, "arrival_runway", ""),
        approach=getattr(plan, "approach", ""),
        cruise_ft=int(getattr(plan, "cruise_altitude_ft", 0) or 0),
        source="simbrief")


def _via(name: str) -> str:
    name = (name or "").strip().upper()
    return "" if name in ("DCT", "DIRECT") else name


# ----------------------------------------------------------------------
# the simulator's flight plan file
# ----------------------------------------------------------------------

#: (ident, region, kind) -> (lat, lon), or None when nobody knows the fix.
Resolver = Callable[[str, str, str], "tuple[float, float] | None"]

# No flight plan is legitimately near this: even a round-the-world plan is
# tens of kilobytes, so anything bigger is a broken or hostile file.
_PLN_MAX_BYTES = 4 * 1024 * 1024

_PLN_KINDS = {"intersection": "wpt", "vor": "vor", "ndb": "ndb",
              "airport": "apt", "user": "wpt", "runway": "wpt"}

# A runway leg's altitude is the threshold's elevation, never a level.
_PLN_RUNWAY_IDENT = re.compile(r"^(?:RW|RWY)\d")


@dataclass
class PlnPlan:
    """What a ``.pln`` says, before anything is resolved."""

    origin: str = ""
    destination: str = ""
    cruise_ft: int = 0
    ifr: bool | None = None
    departure_runway: str = ""
    sid: str = ""
    sid_transition: str = ""
    star: str = ""
    star_transition: str = ""
    arrival_runway: str = ""
    approach: str = ""
    # (ident, region, kind, (lat, lon, alt_ft) or None, airway)
    waypoints: list[tuple[str, str, str,
                          tuple[float, float, float] | None, str]] = \
        field(default_factory=list)


def parse_pln(text: str) -> PlnPlan:
    """Read a simulator flight plan, MSFS 2020's or 2024's.

    2020 writes every fix with a ``WorldPosition`` and the procedures on the
    waypoints themselves (``DepartureFP``, ``ArrivalFP``); 2024 writes the
    fixes by ident and region only and the procedures in their own
    ``DepartureDetails`` / ``ArrivalDetails`` / ``ApproachDetails`` blocks
    (measured on a SimBrief export for MSFS 2024, EBBR-LIRF, 27 Oct 2026).
    """
    text = text.lstrip("﻿")
    # A plan is a file the pilot downloaded from anywhere: ElementTree
    # expands internal entities, which a crafted document could use to
    # exhaust memory, and a plan has no business being megabytes anyway.
    # Raised as ParseError so the caller treats it like any bad plan.
    if len(text) > _PLN_MAX_BYTES:
        raise ET.ParseError("flight plan file is too large")
    if "<!DOCTYPE" in text or "<!ENTITY" in text:
        raise ET.ParseError("flight plan may not declare a DTD")
    root = ET.fromstring(text)
    fp = root.find("FlightPlan.FlightPlan")
    if fp is None:
        fp = root
    plan = PlnPlan(
        origin=_t(fp, "DepartureID").upper(),
        destination=_t(fp, "DestinationID").upper(),
        cruise_ft=max(_int(_t(fp, "CruisingAlt")), 0))
    rules = _t(fp, "FPType").upper()
    plan.ifr = None if not rules else rules.startswith("IFR")

    dep = fp.find("DepartureDetails")
    if dep is not None:
        plan.departure_runway = _runway(dep)
        plan.sid = _t(dep, "DepartureFP").upper()
        plan.sid_transition = _t(dep, "TransitionFP").upper()
    arr = fp.find("ArrivalDetails")
    if arr is not None:
        plan.arrival_runway = _runway(arr)
        plan.star = _t(arr, "ArrivalFP").upper()
        plan.star_transition = _t(arr, "TransitionFP").upper()
    app = fp.find("ApproachDetails")
    if app is not None:
        runway = _runway(app)
        plan.approach = " ".join(x for x in (
            _t(app, "ApproachTypeFP").upper(), _t(app, "SuffixFP").upper(),
            runway) if x)
        plan.arrival_runway = plan.arrival_runway or runway

    for wp in fp.findall("ATCWaypoint"):
        ident = _t(wp, "ICAO/ICAOIdent").upper() or (wp.get("id") or "").upper()
        region = _t(wp, "ICAO/ICAORegion").upper()
        kind = _PLN_KINDS.get(_t(wp, "ATCWaypointType").lower(), "wpt")
        where = _world_position(_t(wp, "WorldPosition"))
        airway = _t(wp, "ATCAirway").upper()
        # 2020's way of naming the procedures, on the fixes.
        plan.sid = plan.sid or _t(wp, "DepartureFP").upper()
        plan.star = plan.star or _t(wp, "ArrivalFP").upper()
        plan.approach = plan.approach or _t(wp, "ApproachTypeFP").upper()
        plan.departure_runway = plan.departure_runway or _runway(wp, "")
        if ident:
            plan.waypoints.append((ident, region, kind, where, airway))
    return plan


def from_pln(text: str, resolve: Resolver | None = None) -> Route:
    """The route of a ``.pln``, each fix placed by its own position where the
    file has one and by ``resolve`` where it does not."""
    plan = parse_pln(text)
    fixes, notes = [], []
    for ident, region, kind, where, airway in plan.waypoints:
        if where is None and resolve is not None:
            try:
                where = resolve(ident, region, kind)
            except Exception:
                log.debug("resolving %s failed", ident, exc_info=True)
                where = None
        if where is None:
            notes.append(f"{ident} could not be placed")
            continue
        fixes.append(RouteFix(ident=ident, lat=where[0], lon=where[1],
                              alt_ft=int(where[2]), via=_via(airway),
                              kind=kind, region=region))
    fixes = _with_ends(fixes, plan.origin, plan.destination, resolve)
    cruise_ft = plan.cruise_ft
    if cruise_ft <= 0:
        # No filed level -- the waypoints' own altitudes are the best
        # word for it, the highest enroute one standing for the cruise
        # (the same inference _route_from_fms makes of an FMS readback).
        # Airport fixes carry the field's elevation, never a level, and
        # that elevation is the bar: a plan whose legs all sit at or
        # under it simply files nothing.
        origin_ft = next((f.alt_ft for f in fixes
                          if f.kind == "apt" and f.alt_ft > 0), 0.0)
        cruise_ft = max(
            (f.alt_ft for f in fixes
             if f.kind != "apt" and not _PLN_RUNWAY_IDENT.match(f.ident)
             and f.alt_ft > origin_ft),
            default=0)
    return Route(fixes=tuple(fixes), origin=plan.origin,
                 destination=plan.destination, sid=plan.sid,
                 sid_runway=plan.departure_runway,
                 sid_transition=plan.sid_transition, star=plan.star,
                 star_transition=plan.star_transition,
                 star_runway=plan.arrival_runway, approach=plan.approach,
                 cruise_ft=cruise_ft, source="pln", notes=tuple(notes))


def _with_ends(fixes: list[RouteFix], origin: str, destination: str,
               resolve: Resolver | None) -> list[RouteFix]:
    """The airports at either end, so the first and last legs exist."""
    out = list(fixes)
    for ident, at_start in ((origin, True), (destination, False)):
        if not ident or resolve is None:
            continue
        if out and (out[0] if at_start else out[-1]).ident == ident:
            continue
        where = resolve(ident, "", "apt")
        if where is None:
            continue
        end = RouteFix(ident=ident, lat=where[0], lon=where[1], kind="apt")
        out = [end] + out if at_start else out + [end]
    return out


def _t(node, path: str) -> str:
    found = node.find(path)
    return (found.text or "").strip() if found is not None else ""


def _int(raw: str) -> int:
    try:
        return int(float(raw))
    except (TypeError, ValueError):
        return 0


_DESIGNATORS = {"LEFT": "L", "RIGHT": "R", "CENTER": "C", "CENTRE": "C",
                "NONE": "", "WATER": "W"}


def _runway(node, default: str = "") -> str:
    number = _t(node, "RunwayNumberFP")
    if not number:
        return default
    designator = _DESIGNATORS.get(_t(node, "RunwayDesignatorFP").upper(), "")
    return f"{int(number):02d}{designator}" if number.isdigit() \
        else number + designator


def _world_position(raw: str) -> tuple[float, float, float] | None:
    """``N50° 54' 4.91",E4° 29' 3.89",+000175.00`` as (lat, lon, ft)."""
    if not raw:
        return None
    parts = raw.split(",")
    if len(parts) < 2:
        return None
    try:
        alt = float(parts[2]) if len(parts) > 2 and parts[2].strip() \
            else 0.0
        return _dms(parts[0]), _dms(parts[1]), alt
    except ValueError:
        return None


def _dms(text: str) -> float:
    text = text.strip()
    sign = -1.0 if text[:1] in "SW" else 1.0
    numbers = [float(x) for x in re.findall(r"[\d.]+", text)]
    if not numbers:
        raise ValueError(text)
    deg = numbers[0] + (numbers[1] / 60.0 if len(numbers) > 1 else 0.0) \
        + (numbers[2] / 3600.0 if len(numbers) > 2 else 0.0)
    return sign * deg


# ----------------------------------------------------------------------
# a route string
# ----------------------------------------------------------------------

_AIRWAY = re.compile(r"^[A-Z]{1,2}\d{1,4}[A-Z]?$")
_PROCEDURE = re.compile(r"^[A-Z]{2,6}\d[A-Z]?$")
_SPEED_LEVEL = re.compile(r"^[NMK]\d{3,4}[FASM]\d{3,4}$")
_LATLON = re.compile(r"^(\d{2})(\d{2})?([NS])(\d{3})(\d{2})?([EW])$")


def from_string(text: str, origin: str, destination: str,
                resolve: Resolver | None, cruise_ft: int = 0) -> Route:
    """A typed route, ``ROUS1L ROUSY DCT GTQ UN852 XIBIL XIBI3A``.

    The procedures are recognised by shape at either end; an airway is
    flown direct between the fixes either side of it, because neither the
    database nor this function has its points, and the route says so.
    """
    tokens = [t for t in re.split(r"\s+", (text or "").upper().strip()) if t]
    tokens = [t.split("/")[0] for t in tokens]          # ROUSY/N0450F370
    tokens = [t for t in tokens if t and not _SPEED_LEVEL.match(t)]
    sid = star = ""
    if tokens and tokens[0] in (origin.upper(),):
        tokens = tokens[1:]
    if tokens and tokens[-1] in (destination.upper(),):
        tokens = tokens[:-1]
    if tokens and _PROCEDURE.match(tokens[0]) and len(tokens) > 1:
        sid, tokens = tokens[0], tokens[1:]
    if tokens and _PROCEDURE.match(tokens[-1]) and len(tokens) > 1:
        star, tokens = tokens[-1], tokens[:-1]
    fixes, notes, via = [], [], ""
    for token in tokens:
        if token in ("DCT", "DIRECT"):
            via = ""
            continue
        if _AIRWAY.match(token) and fixes:
            via = token
            notes.append(f"{token} flown direct: its points are not known")
            continue
        where = _latlon(token)
        if where is None and resolve is not None:
            where = resolve(token, "", "")
        if where is None:
            notes.append(f"{token} could not be placed")
            via = ""
            continue
        fixes.append(RouteFix(ident=token, lat=where[0], lon=where[1],
                              via=via))
        via = ""
    fixes = _with_ends(fixes, origin.upper(), destination.upper(), resolve)
    return Route(fixes=tuple(fixes), origin=origin.upper(),
                 destination=destination.upper(), sid=sid, star=star,
                 cruise_ft=cruise_ft, source="string", notes=tuple(notes))


def _latlon(token: str) -> tuple[float, float] | None:
    """``4620N00805E`` or ``46N008E``, the ICAO forms of a position."""
    m = _LATLON.match(token)
    if not m:
        return None
    lat = int(m.group(1)) + int(m.group(2) or 0) / 60.0
    lon = int(m.group(4)) + int(m.group(5) or 0) / 60.0
    return (-lat if m.group(3) == "S" else lat,
            -lon if m.group(6) == "W" else lon)


# ----------------------------------------------------------------------
# placing a fix
# ----------------------------------------------------------------------


def resolver(navdb, sim: Resolver | None = None,
             near: tuple[float, float] | None = None) -> Resolver:
    """The simulator's facility data first, then the navaids and airports
    in ``nav.sqlite``.

    The simulator knows every intersection in its AIRAC and the database
    knows none, so offline a ``.pln`` keeps its navaids and its airports and
    loses its intersections -- each listed in the route's notes. A navaid
    ident is not unique worldwide; the one nearest ``near`` (the departure)
    is taken, and the simulator, given the region, is never ambiguous.
    """
    def resolve(ident: str, region: str, kind: str):
        if sim is not None:
            try:
                found = sim(ident, region, kind)
            except Exception:
                log.debug("the simulator could not place %s", ident,
                          exc_info=True)
                found = None
            if found is not None:
                return found
        if kind in ("", "apt") and navdb is not None:
            airport = navdb.airport(ident)
            if airport is not None:
                return airport.lat, airport.lon
        if kind in ("", "vor", "ndb") and navdb is not None:
            row = navdb.navaid(ident, near=near)
            if row is not None:
                return float(row["lat"]), float(row["lon"])
        return None
    return resolve


# ----------------------------------------------------------------------
# the route back out, as simulator plans
# ----------------------------------------------------------------------

# A v1100 .fms row opens with the waypoint's type -- airport, NDB, VOR or
# named fix -- and a .pln ATCWaypoint names the same kinds in words.
# RouteFix.kind uses the loader's names (apt, ndb, vor, wpt, ltlg).
_FMS_KIND = {"apt": 1, "ndb": 2, "vor": 3, "wpt": 11, "ltlg": 11}
_PLN_KIND = {"apt": "Airport", "ndb": "NDB", "vor": "VOR",
             "wpt": "Intersection", "ltlg": "Intersection"}


def _airac_cycle(today=None) -> str:
    """The four-digit AIRAC cycle a .fms declares.

    Cycles are continuous 28-day turns, not calendar buckets: each
    year's ``01`` cycle becomes effective mid-to-late January, so a
    day-of-year count lands a cycle early or late whenever the true
    boundary falls inside it. Counted from a known cycle instead --
    2601, effective 2026-01-22 -- with the year taken from the date
    that run of thirteen began."""
    import datetime

    anchor = datetime.date(2026, 1, 22)          # AIRAC 2601 effective
    cycles = ((today or datetime.date.today()) - anchor).days // 28
    year = (anchor + datetime.timedelta(
        days=(cycles // 13) * 13 * 28)).year
    return f"{year % 100:02d}{cycles % 13 + 1:02d}"


def _fms_rows(route: Route) -> list[tuple[int, str, str, float, float, float]]:
    """The en-route rows a v1100 plan carries: the airports either end
    (via ADEP/ADES) and every fix between that is not a TOC/TOD marker
    and not a procedure leg -- SID and STAR fixes are the simulator's to
    expand from the names in the blocks above, not NUMENR's to draw."""
    fixes = route.fixes
    first, last = fixes[0], fixes[-1]
    rows = [(1, route.origin or first.ident, "ADEP",
             first.lat, first.lon, first.alt_ft)]
    for f in fixes[1:-1]:
        if f.ident.upper() in ("TOC", "TOD") or f.on_procedure:
            continue
        via = f.via if f.via and f.via not in ("DCT", "DIRECT") else "DRCT"
        rows.append((_FMS_KIND.get(f.kind, 11), f.ident, via,
                     f.lat, f.lon, f.alt_ft))
    rows.append((1, route.destination or last.ident, "ADES",
                 last.lat, last.lon, last.alt_ft))
    return rows


def to_fms(route: Route) -> str:
    """The route as an X-Plane 11+ flight plan (the ``1100`` text).

    This is the exact payload a ``FPLN`` datagram and
    ``XPLMLoadFMSFlightPlan`` take: the departure and destination blocks
    with the filed runway, SID and STAR, then the en-route waypoints as
    ``type ident via altitude lat lon`` rows. The approach is left to
    the cockpit: the format's single-letter approach types do not map
    cleanly from the plan's words, and a wrong approach line is worse
    than none.
    """
    if route is None or not route.fixes:
        return ""
    origin = route.origin or route.fixes[0].ident
    destination = route.destination or route.fixes[-1].ident
    lines = ["I", "1100 Version", f"CYCLE {_airac_cycle()}"]
    lines.append(f"ADEP {origin}")
    if route.sid_runway:
        lines.append(f"DEPRWY RW{route.sid_runway}")
    if route.sid:
        lines.append(f"SID {route.sid}")
    if route.sid_transition:
        lines.append(f"SIDTRANS {route.sid_transition}")
    lines.append(f"ADES {destination}")
    if route.star_runway:
        lines.append(f"DESRWY RW{route.star_runway}")
    if route.star:
        lines.append(f"STAR {route.star}")
    if route.star_transition:
        lines.append(f"STARTRANS {route.star_transition}")
    rows = _fms_rows(route)
    lines.append(f"NUMENR {len(rows)}")
    lines += [f"{t} {i} {v} {a:.6f} {la:.6f} {lo:.6f}"
              for t, i, v, la, lo, a in rows]
    return "\n".join(lines)


def _lla(lat: float, lon: float, alt_ft: float) -> str:
    """``N41° 14' 51.50",W8° 40' 41.12",+000748.00`` -- a .pln position."""
    def dms(value: float, pos: str, neg: str, digits: int) -> str:
        hemi = pos if value >= 0 else neg
        value = abs(value)
        deg = int(value)
        minutes = (value - deg) * 60.0
        whole = int(minutes)
        return (f"{hemi}{deg:0{digits}d}\u00b0 {whole:02d}' "
                f"{(minutes - whole) * 60.0:05.2f}\"")
    return f"{dms(lat, 'N', 'S', 2)},{dms(lon, 'E', 'W', 3)},{alt_ft:+010.2f}"


_PLN_DESIGNATOR = {"L": "LEFT", "R": "RIGHT", "C": "CENTER",
                   "W": "WATER", "": "NONE"}


def _pln_runway(name: str) -> str:
    """``36R`` as the ``RunwayNumberFP``/``RunwayDesignatorFP`` pair a
    .pln writes, or "" when the name is not one it knows."""
    match = re.match(r"^0?(\d{1,2})([LRCW]?)$", (name or "").upper())
    if not match:
        return ""
    designator = _PLN_DESIGNATOR.get(match.group(2), "NONE")
    return (f"<RunwayNumberFP>{int(match.group(1))}</RunwayNumberFP>"
            f"<RunwayDesignatorFP>{designator}</RunwayDesignatorFP>")


def to_pln(route: Route, ifr: bool = True) -> str:
    """The route as an MSFS ``.pln``.

    ``SimConnect_FlightPlanLoad`` takes a file path, not a buffer, so
    this is the text a caller writes out before the call. Every fix is
    an ``ATCWaypoint`` with its own ``WorldPosition`` -- nothing on the
    simulator's side is asked to resolve a name. The filed runway, SID
    and STAR ride where both readers look for them: on the endpoint
    waypoints (2020's layout) and in the ``Details`` blocks (2024's).
    """
    if route is None or not route.fixes:
        return ""
    origin = route.origin or route.fixes[0].ident
    destination = route.destination or route.fixes[-1].ident
    first, last = route.fixes[0], route.fixes[-1]
    dep_runway = _pln_runway(route.sid_runway)
    arr_runway = _pln_runway(route.star_runway)
    waypoints = []
    for i, f in enumerate(route.fixes):
        if f.ident.upper() in ("TOC", "TOD"):
            continue
        kind = _PLN_KIND.get(f.kind, "Intersection")
        extra = ""
        if i == 0 and route.sid:
            extra = (f"<DepartureFP>{route.sid}</DepartureFP>" + dep_runway +
                     (f"<TransitionFP>{route.sid_transition}</TransitionFP>"
                      if route.sid_transition else ""))
        elif i == len(route.fixes) - 1 and (route.star or route.approach):
            extra = ((f"<ArrivalFP>{route.star}</ArrivalFP>"
                      if route.star else "") + arr_runway +
                     (f"<ApproachTypeFP>{route.approach}</ApproachTypeFP>"
                      if route.approach else ""))
        waypoints.append(
            f'    <ATCWaypoint id="{f.ident}">\n'
            f'      <ATCWaypointType>{kind}</ATCWaypointType>\n'
            f'      <WorldPosition>{_lla(f.lat, f.lon, f.alt_ft)}'
            f'</WorldPosition>\n'
            + (f'      {extra}\n' if extra else '')
            + f'      <ICAO><ICAOIdent>{f.ident}</ICAOIdent></ICAO>\n'
            f'    </ATCWaypoint>')
    details = ""
    if route.sid or dep_runway:
        details += ("    <DepartureDetails>\n"
                    + (f"      <DepartureFP>{route.sid}</DepartureFP>\n"
                       if route.sid else "")
                    + (f"      {dep_runway}\n" if dep_runway else "")
                    + (f"      <TransitionFP>{route.sid_transition}"
                       f"</TransitionFP>\n" if route.sid_transition else "")
                    + "    </DepartureDetails>\n")
    if route.star or arr_runway or route.approach:
        details += ("    <ArrivalDetails>\n"
                    + (f"      <ArrivalFP>{route.star}</ArrivalFP>\n"
                       if route.star else "")
                    + (f"      {arr_runway}\n" if arr_runway else "")
                    + (f"      <TransitionFP>{route.star_transition}"
                       f"</TransitionFP>\n" if route.star_transition else "")
                    + "    </ArrivalDetails>\n")
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<SimBase.Document Type="FlightPlan" version="1,5">\n'
        '  <Descr>AceXML Document</Descr>\n'
        '  <FlightPlan.FlightPlan>\n'
        f'    <Title>{origin} to {destination}</Title>\n'
        f'    <FPType>{"IFR" if ifr else "VFR"}</FPType>\n'
        '    <RouteType>Direct</RouteType>\n'
        f'    <CruisingAlt>{int(route.cruise_ft or 0)}</CruisingAlt>\n'
        f'    <DepartureID>{origin}</DepartureID>\n'
        f'    <DepartureLLA>{_lla(first.lat, first.lon, first.alt_ft)}'
        f'</DepartureLLA>\n'
        f'    <DestinationID>{destination}</DestinationID>\n'
        f'    <DestinationLLA>{_lla(last.lat, last.lon, last.alt_ft)}'
        f'</DestinationLLA>\n'
        '    <Descr>Route loaded by MeridianATC</Descr>\n'
        + details + "\n".join(waypoints) + "\n"
        '  </FlightPlan.FlightPlan>\n'
        '</SimBase.Document>\n')


__all__ = ["Route", "RouteFix", "Progress", "PlnPlan", "from_ofp",
           "from_pln", "from_string", "parse_pln", "resolver",
           "to_fms", "to_pln", "climb_groundspeed"]
