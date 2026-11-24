"""Enroute convective-weather hazards and lateral avoidance geometry.

The model deliberately separates evidence from the avoidance decision.  X-Plane
radar samples are simulator truth but sparse and precipitation-only; Aviation
Weather Center reports are explicit convection but external and therefore only
strategic evidence for what a simulator is rendering.  Neither is treated as a
NEXRAD display, and ordinary cloud or rain is never by itself a thunderstorm.
"""

from __future__ import annotations

import gzip
import logging
import math
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
        # Tall precipitation is convective evidence. Very strong precipitation
        # counts even when the vertical report is coarse or truncated.
        if top < max(RADR_TALL_TOP_FT, altitude_ft + 3000.0) \
                and precip < RADR_STRONG_PRECIP:
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
        explicit = weather.cb or any(
            token in text.split() for token in
            ("TS", "+TS", "-TS", "VCTS", "TSRA", "+TSRA", "-TSRA", "GR", "+GR", "SQ", "FC"))
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
                __import__("re").findall(r"[-+]?\d+(?:\.\d+)?", text)]
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
        if hazard != "CONV" and "CONVECTIVE" not in raw and " TS" not in raw:
            continue
        base = row.get("altitudeLow1", row.get("base"))
        top = row.get("altitudeHi1", row.get("top"))
        if altitude_ft and base is not None and top is not None \
                and (altitude_ft < float(base) - 3000.0
                     or altitude_ft > float(top) + 3000.0):
            continue
        polygon = _rings_from_geometry(row.get("geometry") or row.get("geom")
                                       or row.get("coords"))
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
        if not any(word in text for word in
                   ("SEV", "EXTREME", "TS", "CB", "TSTM", "HAIL")):
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
        dist = haversine_nm(lat, lon, cell.lat, cell.lon)
        if dist > ahead_nm + cell.radius_nm + cell.clearance_nm:
            continue
        bearing = bearing_deg(lat, lon, cell.lat, cell.lon)
        rel = abs(_angle_delta(bearing, course_true))
        forward = math.cos(math.radians(rel)) * dist
        lateral = abs(math.sin(math.radians(rel)) * dist)
        limit = width_nm + cell.radius_nm + cell.clearance_nm
        if forward < -cell.radius_nm or forward > ahead_nm + cell.radius_nm \
                or lateral > limit:
            continue
        hits.append(cell)
        area += math.pi * min(40.0, cell.radius_nm + cell.clearance_nm) ** 2
    return hits, min(1.0, area / corridor)


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
    turn = -degrees if side == "left" else degrees
    circumnavigate = coverage >= CIRCUMNAVIGATE_COVERAGE
    if circumnavigate:
        degrees = max(degrees, 60)
        turn = -degrees if side == "left" else degrees
    return AvoidancePlan(
        True, side, degrees, miles, (course_true + turn + 360.0) % 360.0,
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
        if data[:2] == b"\x1f\x8b":
            data = gzip.decompress(data)
        return response.json() if data else []

    def _fetch(self, lat: float, lon: float, altitude_ft: float) -> None:
        # One degree is approximately sixty miles.  The box covers the scan
        # horizon plus a margin in every direction.
        half_lat, half_lon = 1.5, 1.5 / max(0.35, math.cos(math.radians(lat)))
        bbox = (f"{lat - half_lat:.3f},{lon - half_lon:.3f},"
                f"{lat + half_lat:.3f},{lon + half_lon:.3f}")
        cells = []
        try:
            metars = self._get("metar", {"bbox": bbox, "format": "json", "hours": 2})
            cells.extend(metar_convective_cells(metars))
        except Exception:
            log.debug("enroute METAR fetch failed", exc_info=True)
        try:
            domestic = self._get("airsigmet", {"format": "json", "hazard": "conv"})
            cells.extend(sigmet_cells(domestic, altitude_ft))
        except Exception:
            log.debug("domestic SIGMET fetch failed", exc_info=True)
        try:
            international = self._get("isigmet", {"format": "json"})
            cells.extend(sigmet_cells(international, altitude_ft))
        except Exception:
            log.debug("international SIGMET fetch failed", exc_info=True)
        try:
            pireps = self._get("pirep", {"bbox": bbox, "format": "json", "age": 3})
            cells.extend(pirep_cells(pireps))
        except Exception:
            log.debug("PIREP fetch failed", exc_info=True)
        with self._lock:
            self._cells = cells
            self._at = time.time()
            self._busy = False

    def cells(self) -> list[HazardCell]:
        with self._lock:
            if not self._cells or time.time() - self._at > self.EXPIRE_S:
                return []
            return list(self._cells)
