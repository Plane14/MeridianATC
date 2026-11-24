"""Enroute convective-weather hazards and lateral avoidance geometry.

The model deliberately separates evidence from the avoidance decision.  X-Plane
radar samples are simulator truth but sparse and precipitation-only; Aviation
Weather Center reports are explicit convection but external and therefore only
strategic evidence for what a simulator is rendering.  Neither is treated as a
NEXRAD display, and ordinary cloud or rain is never by itself a thunderstorm.
"""

from __future__ import annotations

import gzip
import json
import logging
import math
import re
import threading
import time
from dataclasses import dataclass, field

import requests

from ..navdata.build import bearing_deg, haversine_nm
from .metar import parse_metar

log = logging.getLogger(__name__)

AWC_BASE = "https://aviationweather.gov/api/data"
METERS_TO_FEET = 3.280839895
SEVERE_CLEARANCE_NM = 20.0
NORMAL_CLEARANCE_NM = 8.0
SCAN_AHEAD_NM = 80.0
SCAN_WIDTH_NM = 25.0
CIRCUMNAVIGATE_COVERAGE = 0.6

# Conservative first-pass interpretation of X-Plane's dimensionless ratios.
# They are precipitation/cloud amounts, not NEXRAD VIP levels; the high tops
# condition is what keeps ordinary rain from becoming a thunderstorm.
RADR_PRECIP_THREAT = 0.45
RADR_STRONG_PRECIP = 0.70
RADR_TALL_TOP_FT = 20000.0
RADR_CLUSTER_DEG = 0.06
RADR_POINT_AGE_S = 90.0


@dataclass(frozen=True)
class HazardCell:
    """One lateral hazardous-weather blob."""

    lat: float
    lon: float
    radius_nm: float
    severity: float                 # 0 benign .. 1 severe
    source: str                     # xplane-radar, awc-metar, awc-sigmet, ...
    reason: str
    base_ft: float = 0.0
    top_ft: float = 0.0
    at: float = field(default_factory=time.time)
    polygon: tuple[tuple[float, float], ...] = ()  # (lat, lon)

    @property
    def severe(self) -> bool:
        return self.severity >= 0.75

    @property
    def clearance_nm(self) -> float:
        return SEVERE_CLEARANCE_NM if self.severe else NORMAL_CLEARANCE_NM

    def contains(self, lat: float, lon: float) -> bool:
        if self.polygon:
            return point_in_polygon(lat, lon, self.polygon)
        return haversine_nm(self.lat, self.lon, lat, lon) <= self.radius_nm

    def distance_nm(self, lat: float, lon: float) -> float:
        if self.polygon and self.contains(lat, lon):
            return 0.0
        if self.polygon:
            return min(haversine_nm(lat, lon, plat, plon)
                       for plat, plon in self.polygon)
        return max(0.0, haversine_nm(self.lat, self.lon, lat, lon)
                   - self.radius_nm)


@dataclass(frozen=True)
class AvoidancePlan:
    """A lateral deviation worth asking for, in executable terms."""

    threatening: bool
    side: str = ""                  # left / right
    degrees: int = 0
    miles_nm: int = 0
    heading_true: float = -1.0
    reason: str = ""
    source: str = ""
    nearest_nm: float = 0.0
    coverage: float = 0.0
    circumnavigate: bool = False


@dataclass(frozen=True)
class RadarPoint:
    """One X-Plane RADR sample, normalised to feet."""

    lat: float
    lon: float
    base_ft: float
    top_ft: float
    clouds_ratio: float
    precip_ratio: float
    at: float


def point_in_polygon(lat: float, lon: float,
                     polygon: tuple[tuple[float, float], ...]) -> bool:
    """Ray-casting containment for a small geographic polygon."""
    inside = False
    count = len(polygon)
    if count < 3:
        return False
    x, y = lon, lat
    j = count - 1
    for i in range(count):
        xi, yi = polygon[i][1], polygon[i][0]
        xj, yj = polygon[j][1], polygon[j][0]
        if (yi > y) != (yj > y):
            crossing = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < crossing:
                inside = not inside
        j = i
    return inside


def _angle_delta(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0


def xplane_cells(points, altitude_ft: float = 0.0,
                 now: float | None = None) -> list[HazardCell]:
    """Cluster current X-Plane radar points into convective candidates."""
    now = time.time() if now is None else now
    fresh = []
    for point in points or ():
        at = float(getattr(point, "at", now) or now)
        if now - at > RADR_POINT_AGE_S:
            continue
        precip = float(getattr(point, "precip_ratio", 0.0) or 0.0)
        top = float(getattr(point, "top_ft", 0.0) or 0.0)
        if precip < RADR_PRECIP_THREAT:
            continue
        # Precipitation well below the aeroplane is not its weather. Tall
        # precipitation is convective evidence; very strong precipitation
        # counts even when the vertical report is coarse or truncated.
        if altitude_ft and top and top < altitude_ft - 3000.0:
            continue
        if top < RADR_TALL_TOP_FT and precip < RADR_STRONG_PRECIP:
            continue
        fresh.append(point)
    buckets: dict[tuple[int, int], list] = {}
    for point in fresh:
        key = (round(float(point.lat) / RADR_CLUSTER_DEG),
               round(float(point.lon) / RADR_CLUSTER_DEG))
        buckets.setdefault(key, []).append(point)
    cells = []
    for cluster in buckets.values():
        lat = sum(p.lat for p in cluster) / len(cluster)
        lon = sum(p.lon for p in cluster) / len(cluster)
        spread = max((haversine_nm(lat, lon, p.lat, p.lon) for p in cluster),
                     default=0.0)
        precip = max(float(p.precip_ratio) for p in cluster)
        top = max(float(p.top_ft) for p in cluster)
        severity = min(1.0, 0.45 + precip * 0.45
                       + (0.15 if top >= 30000.0 else 0.0))
        if precip >= RADR_STRONG_PRECIP and top >= RADR_TALL_TOP_FT:
            severity = max(severity, 0.8)
        cells.append(HazardCell(
            lat, lon, max(2.0, spread + 1.5), severity, "xplane-radar",
            f"precipitation {precip:.2f}, tops {top:,.0f} ft",
            top_ft=top, at=max(float(getattr(p, "at", now) or now)
                               for p in cluster)))
    return cells


def metar_convective_cells(raw_reports, now: float | None = None) -> list[HazardCell]:
    """Explicit thunderstorm reports near the route, as coarse strategic cells."""
    now = time.time() if now is None else now
    cells = []
    for row in raw_reports or ():
        raw = row.get("rawOb") or row.get("raw") or ""
        lat, lon = row.get("lat"), row.get("lon")
        if lat is None or lon is None:
            continue
        weather = parse_metar(raw)
        text = ((weather.present or "") + " " + raw.split(" RMK", 1)[0]).upper()
        explicit = weather.cb or bool(re.search(
            r"(?:^|\s)[-+]?(?:VCTS|TS\w*|GR|SQ|FC)(?:\s|$)|\b(?:FEW|SCT|BKN|OVC|VV)\d{3}CB\b", text))
        if not explicit:
            continue
        severe = any(token in text for token in ("+TS", "GR", "SQ", "FC"))
        cells.append(HazardCell(
            float(lat), float(lon), 10.0, 0.8 if severe else 0.55,
            "awc-metar", f"METAR {row.get('icaoId', '')}: explicit convection",
            at=now))
    return cells


def _rings_from_geometry(value) -> tuple[tuple[float, float], ...]:
    """Decode GeoJSON or compact AWC coordinate geometry to (lat, lon)."""
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return ()
        # Domestic SIGMETs have historically carried WKT-ish polygons.  This
        # parser accepts the coordinate pairs without pretending to be a full
        # WKT implementation.
        nums = [float(x) for x in
                re.findall(r"[-+]?\d+(?:\.\d+)?", text)]
        if len(nums) >= 6:
            # AWC WKT is lon lat pairs.
            return tuple((nums[i + 1], nums[i])
                         for i in range(0, len(nums) - 1, 2))
        return ()
    if isinstance(value, dict):
        kind = value.get("type")
        coords = value.get("coordinates") or ()
        if kind == "Polygon" and coords:
            return tuple((float(p[1]), float(p[0])) for p in coords[0])
        if kind == "MultiPolygon" and coords and coords[0]:
            return tuple((float(p[1]), float(p[0])) for p in coords[0][0])
    if isinstance(value, (list, tuple)):
        if value and isinstance(value[0], dict):
            return tuple((float(p["lat"]), float(p["lon"])) for p in value
                         if "lat" in p and "lon" in p)
        if value and isinstance(value[0], (int, float)) and len(value) >= 2:
            return ((float(value[1]), float(value[0])),)
        if value and isinstance(value[0], (list, tuple)):
            if value[0] and isinstance(value[0][0], (int, float)):
                return tuple((float(p[1]), float(p[0])) for p in value)
            if value[0] and isinstance(value[0][0], (list, tuple)):
                return tuple((float(p[1]), float(p[0])) for p in value[0])
    return ()


def sigmet_cells(rows, altitude_ft: float = 0.0,
                 now: float | None = None) -> list[HazardCell]:
    """Convective SIGMET polygons, filtered to the aircraft's altitude band."""
    now = time.time() if now is None else now
    cells = []
    for row in rows or ():
        hazard = str(row.get("hazard", "")).upper()
        raw = str(row.get("rawAirSigmet") or row.get("rawSigmet") or "").upper()
        if hazard not in ("CONV", "CONVECTIVE") and "CONVECTIVE" not in raw \
                and " TS" not in raw:
            continue
        now_wall = time.time()
        valid_from = row.get("validTimeFrom")
        valid_to = row.get("validTimeTo")
        try:
            if valid_from is not None \
                    and now_wall < float(valid_from) - 300.0:
                continue
            if valid_to is not None \
                    and now_wall > float(valid_to) + 300.0:
                continue
        except (TypeError, ValueError):
            # A service timestamp shape Meridian does not know is not a reason
            # to discard explicit convective geometry.
            pass
        base = row.get("altitudeLow1", row.get("base"))
        top = row.get("altitudeHi1", row.get("top"))
        if altitude_ft and base is not None and top is not None \
                and (altitude_ft < float(base) - 3000.0
                     or altitude_ft > float(top) + 3000.0):
            continue
        polygon = _rings_from_geometry(row.get("coords") or row.get("geometry")
                                       or row.get("geom"))
        if len(polygon) < 3:
            continue
        lat = sum(p[0] for p in polygon) / len(polygon)
        lon = sum(p[1] for p in polygon) / len(polygon)
        radius = max(haversine_nm(lat, lon, p[0], p[1]) for p in polygon)
        severity = 0.9 if "SEV" in str(row.get("severity") or row.get("qualifier") or "").upper() else 0.8
        cells.append(HazardCell(
            lat, lon, min(80.0, max(5.0, radius)), severity, "awc-sigmet",
            "convective SIGMET", float(base or 0.0), float(top or 0.0),
            now, polygon))
    return cells


def pirep_cells(rows, now: float | None = None) -> list[HazardCell]:
    """Recent severe-weather pilot reports as small high-confidence cells."""
    now = time.time() if now is None else now
    cells = []
    for row in rows or ():
        text = " ".join(str(row.get(k, "")) for k in
                        ("rawOb", "raw", "turbulence", "icing", "wxString")).upper()
        if not re.search(r"\b(?:SEV|SEVERE|EXTREME|TS|CB|TSTM|HAIL)\b", text):
            continue
        lat, lon = row.get("lat"), row.get("lon")
        if lat is None or lon is None:
            continue
        cells.append(HazardCell(float(lat), float(lon), 8.0, 0.8,
                                "awc-pirep", "severe-weather PIREP", at=now))
    return cells


def _cross_track_nm(lat, lon, a_lat, a_lon, b_lat, b_lon) -> tuple[float, float]:
    """(signed cross-track, along-track) in nm on a short route leg."""
    course = bearing_deg(a_lat, a_lon, b_lat, b_lon)
    dist = haversine_nm(a_lat, a_lon, lat, lon)
    bearing = bearing_deg(a_lat, a_lon, lat, lon)
    delta = math.radians(_angle_delta(bearing, course))
    return dist * math.sin(delta), dist * math.cos(delta)


def scan_route(cells, lat: float, lon: float, course_true: float,
               altitude_ft: float = 0.0, ahead_nm: float = SCAN_AHEAD_NM,
               width_nm: float = SCAN_WIDTH_NM) -> tuple[list[HazardCell], float]:
    """Hazards in the forward route corridor and their area coverage ratio."""
    hits = []
    area = 0.0
    corridor = max(1.0, ahead_nm * width_nm * 2.0)
    for cell in cells or ():
        references = [(cell.lat, cell.lon)]
        if cell.polygon:
            # A broad SIGMET's centroid can sit outside the corridor while one
            # side of the polygon crosses it. The vertices are sparse, but they
            # are honest geometry; the centre alone is not.
            references.extend(cell.polygon)
        in_corridor = False
        for ref_lat, ref_lon in references:
            dist = haversine_nm(lat, lon, ref_lat, ref_lon)
            if dist > ahead_nm + cell.radius_nm + cell.clearance_nm:
                continue
            bearing = bearing_deg(lat, lon, ref_lat, ref_lon)
            rel = abs(_angle_delta(bearing, course_true))
            forward = math.cos(math.radians(rel)) * dist
            lateral = abs(math.sin(math.radians(rel)) * dist)
            limit = width_nm + cell.radius_nm + cell.clearance_nm
            if forward >= -cell.radius_nm \
                    and forward <= ahead_nm + cell.radius_nm \
                    and lateral <= limit:
                in_corridor = True
                break
        if not in_corridor:
            continue
        hits.append(cell)
        area += math.pi * min(40.0, cell.radius_nm + cell.clearance_nm) ** 2
    return hits, min(1.0, area / corridor)



def _local_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """Small-area east/north nautical-mile coordinates around the aircraft."""
    return ((lon - lon0) * 60.0 * math.cos(math.radians(lat0)),
            (lat - lat0) * 60.0)


def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _cross(a, b) -> float:
    return a[0] * b[1] - a[1] * b[0]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _point_segment_distance(p, a, b) -> float:
    ab = _sub(b, a)
    length2 = _dot(ab, ab)
    if length2 < 1e-12:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, _dot(_sub(p, a), ab) / length2))
    closest = (a[0] + ab[0] * t, a[1] + ab[1] * t)
    return math.hypot(p[0] - closest[0], p[1] - closest[1])


def _ray_edge_intersection(direction, a, b):
    """Positive ray parameter where an infinite ray crosses segment a-b."""
    edge = _sub(b, a)
    denominator = _cross(direction, edge)
    if abs(denominator) < 1e-12:
        return None
    along_ray = _cross(a, edge) / denominator
    along_edge = _cross(a, direction) / denominator
    if along_ray >= 0.0 and -1e-9 <= along_edge <= 1.0 + 1e-9:
        return along_ray
    return None


def _segments_intersect(a, b, c, d) -> bool:
    def orient(p, q, r):
        return _cross(_sub(q, p), _sub(r, p))

    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return (o1 * o2 < -1e-12 and o3 * o4 < -1e-12) or \
        min(_point_segment_distance(c, a, b), _point_segment_distance(d, a, b),
            _point_segment_distance(a, c, d), _point_segment_distance(b, c, d)) < 1e-9


def _segments_distance(a, b, c, d) -> float:
    if _segments_intersect(a, b, c, d):
        return 0.0
    return min(_point_segment_distance(a, c, d), _point_segment_distance(b, c, d),
               _point_segment_distance(c, a, b), _point_segment_distance(d, a, b))


def _circle_course_metrics(center, direction, radius: float, horizon: float):
    """(inside-now, entry-distance, exit-distance) for a ray and circle."""
    along = _dot(center, direction)
    closest2 = max(0.0, _dot(center, center) - along * along)
    if closest2 > radius * radius:
        return False, None, None
    half = math.sqrt(max(0.0, radius * radius - closest2))
    entry, exit_at = along - half, along + half
    inside = entry <= 0.0 <= exit_at and math.hypot(*center) <= radius
    if exit_at < 0.0 or entry > horizon:
        return inside, None, exit_at if inside else None
    return inside, max(0.0, entry), max(0.0, exit_at)


def _course_geometry_cost(cell: HazardCell, lat: float, lon: float,
                          course: float, horizon: float) -> float:
    """Cost of one hazard on one course, with body and clearance separated."""
    direction = (math.sin(math.radians(course)), math.cos(math.radians(course)))
    ray_end = (direction[0] * horizon, direction[1] * horizon)
    if len(cell.polygon) >= 3:
        vertices = [_local_xy(plat, plon, lat, lon) for plat, plon in cell.polygon]
        crossings = []
        clearance_distance = horizon
        for index, a in enumerate(vertices):
            b = vertices[(index + 1) % len(vertices)]
            hit = _ray_edge_intersection(direction, a, b)
            if hit is not None:
                crossings.append(hit)
            clearance_distance = min(clearance_distance,
                                     _segments_distance((0.0, 0.0), ray_end, a, b))
        inside = cell.contains(lat, lon)
        if inside:
            exit_nm = max(crossings, default=0.0)
            return cell.severity * (2.0 + min(20.0, exit_nm) / 10.0)
        body_hits = [hit for hit in crossings if hit <= horizon]
        if body_hits:
            entry = max(5.0, min(body_hits) + 5.0)
            return 1000.0 * cell.severity / entry
        shortfall = cell.clearance_nm - clearance_distance
        if shortfall > 0.0:
            return cell.severity * (shortfall + 1.0) / 10.0
        return 0.0

    center = _local_xy(cell.lat, cell.lon, lat, lon)
    body_inside, body_entry, body_exit = _circle_course_metrics(
        center, direction, cell.radius_nm, horizon)
    if body_inside:
        return cell.severity * (2.0 + min(20.0, body_exit or 0.0) / 10.0)
    if body_entry is not None:
        return 1000.0 * cell.severity / max(5.0, body_entry + 5.0)
    expanded = cell.radius_nm + cell.clearance_nm
    margin_inside, margin_entry, _margin_exit = _circle_course_metrics(
        center, direction, expanded, horizon)
    if margin_entry is not None and not margin_inside:
        closest = math.sqrt(max(0.0, _dot(center, center) - _dot(center, direction) ** 2))
        edge_distance = max(0.0, closest - cell.radius_nm)
        shortfall = cell.clearance_nm - edge_distance
        if shortfall > 0.0:
            return cell.severity * (shortfall + 1.0) / max(5.0, margin_entry + 5.0)
    return 0.0


def avoidance_plan(cells, lat: float, lon: float, course_true: float,
                   altitude_ft: float = 0.0) -> AvoidancePlan:
    """Choose the safer side and magnitude of a lateral weather deviation."""
    hits, coverage = scan_route(cells, lat, lon, course_true, altitude_ft)
    if not hits:
        return AvoidancePlan(False, coverage=coverage)
    nearest = min(h.distance_nm(lat, lon) for h in hits)
    weighted = [h for h in hits
                if h.distance_nm(lat, lon) <= nearest + 10.0] or hits
    scores = {"left": 0.0, "right": 0.0}
    for cell in weighted:
        bearing = bearing_deg(lat, lon, cell.lat, cell.lon)
        rel = _angle_delta(bearing, course_true)
        dist = max(3.0, haversine_nm(lat, lon, cell.lat, cell.lon))
        weight = cell.severity * (cell.clearance_nm + 10.0) / dist
        # A hazard to the right is avoided to the left, and vice versa.
        scores["left" if rel >= 0.0 else "right"] += weight
    side = max(scores, key=scores.get)
    opposite = "right" if side == "left" else "left"
    if scores[opposite] > scores[side] * 1.2:
        side = opposite
    strongest = max(hits, key=lambda h: h.severity)
    severe = strongest.severe
    miles = int(math.ceil((strongest.radius_nm + strongest.clearance_nm
                           + 4.0) / 5.0) * 5)
    miles = max(15 if severe else 10, min(60, miles))
    degrees = max(20, min(90, int(math.ceil(math.degrees(
        math.atan2(miles, max(25.0, nearest + 20.0))) / 10.0) * 10)))
    circumnavigate = coverage >= CIRCUMNAVIGATE_COVERAGE
    if circumnavigate:
        degrees = max(degrees, 60)

    def course_cost(course: float) -> float:
        """Weighted penetration if the proposed deviation heading were flown."""
        return sum(_course_geometry_cost(cell, lat, lon, course, SCAN_AHEAD_NM)
                   for cell in cells or ())

    courses = {
        "left": (course_true - degrees + 360.0) % 360.0,
        "right": (course_true + degrees + 360.0) % 360.0,
    }
    costs = {which: course_cost(course) for which, course in courses.items()}
    opposite = "right" if side == "left" else "left"
    # The first side only answers where the nearest cells sit. Check the
    # heading itself as well: a second cell already inside the scan must not
    # become the steering course merely because it was farther away.
    if costs[opposite] < costs[side] * 0.8:
        side = opposite
    heading = courses[side]
    return AvoidancePlan(
        True, side, degrees, miles, heading,
        strongest.reason, strongest.source, nearest, coverage,
        circumnavigate)


class EnrouteWeatherProvider:
    """Cached external convective-weather evidence for a moving route.

    The service is deliberately asynchronous: radio and flight control never
    wait for aviationweather.gov.  A query begins in the background and the
    last completed evidence is used until it expires.
    """

    REFRESH_S = 300.0
    EXPIRE_S = 900.0

    def __init__(self, enabled: bool = True, session: requests.Session | None = None):
        self.enabled = enabled
        self.session = session or requests.Session()
        self.session.headers.setdefault(
            "User-Agent", "MeridianATC enroute weather avoidance")
        self._lock = threading.Lock()
        self._cells: list[HazardCell] = []
        self._at = 0.0
        self._asked_at = 0.0
        self._key = None
        self._busy = False

    @staticmethod
    def _key_for(lat: float, lon: float) -> tuple[int, int]:
        return (round(lat), round(lon))

    def update(self, lat: float, lon: float, altitude_ft: float = 0.0) -> None:
        """Start a route-area refresh when the aircraft has moved enough."""
        if not self.enabled:
            return
        now = time.monotonic()
        key = self._key_for(lat, lon)
        with self._lock:
            if self._busy or (key == self._key and now - self._asked_at < self.REFRESH_S):
                return
            self._busy = True
            self._asked_at = now
            self._key = key
        thread = threading.Thread(target=self._fetch, args=(lat, lon, altitude_ft),
                                  name="MeridianATC-enroute-weather", daemon=True)
        thread.start()

    def _get(self, product: str, params: dict):
        response = self.session.get(f"{AWC_BASE}/{product}", params=params,
                                    timeout=12.0)
        response.raise_for_status()
        data = response.content
        if not data or response.status_code == 204:
            return []
        if data[:2] == b"\x1f\x8b":
            data = gzip.decompress(data)
        return json.loads(data.decode("utf-8")) if data else []

    def _fetch(self, lat: float, lon: float, altitude_ft: float) -> None:
        # One degree is approximately sixty miles.  The box covers the scan
        # horizon plus a margin in every direction.
        half_lat, half_lon = 1.5, 1.5 / max(0.35, math.cos(math.radians(lat)))
        bbox = (f"{lat - half_lat:.3f},{lon - half_lon:.3f},"
                f"{lat + half_lat:.3f},{lon + half_lon:.3f}")
        cells = []
        answered = False
        try:
            metars = self._get("metar", {"bbox": bbox, "format": "json", "hours": 2})
            cells.extend(metar_convective_cells(metars))
            answered = True
        except Exception:
            log.debug("enroute METAR fetch failed", exc_info=True)
        try:
            domestic = self._get("airsigmet", {"format": "json", "hazard": "conv"})
            cells.extend(sigmet_cells(domestic, altitude_ft))
            answered = True
        except Exception:
            log.debug("domestic SIGMET fetch failed", exc_info=True)
        try:
            international = self._get("isigmet", {"format": "json"})
            cells.extend(sigmet_cells(international, altitude_ft))
            answered = True
        except Exception:
            log.debug("international SIGMET fetch failed", exc_info=True)
        try:
            pireps = self._get("pirep", {"bbox": bbox, "format": "json", "age": 3})
            cells.extend(pirep_cells(pireps))
            answered = True
        except Exception:
            log.debug("PIREP fetch failed", exc_info=True)
        with self._lock:
            # A dead service must not erase the last good evidence early;
            # the normal timestamped expiry does that honestly.
            if answered:
                self._cells = cells
                self._at = time.time()
            self._busy = False

    def cells(self) -> list[HazardCell]:
        with self._lock:
            if not self._cells or time.time() - self._at > self.EXPIRE_S:
                return []
            return list(self._cells)
