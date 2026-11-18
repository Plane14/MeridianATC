"""The airspace the simulator itself knows about.

The CSV volumes only cover where open data was found and licensed; each
simulator ships the whole world's airspace, the same dataset its own GPS
and map draw.

X-Plane publishes it as text the pilot already owns: OpenAir records at
``Custom Data/airspaces/*.txt`` -- the AIRAC the pilot installed, which
wins -- or the stock ``Resources/default data/airspaces/*.txt``.

MSFS keeps it inside the sim behind the facility listener the stock
avionics use; SimConnect never sees a boundary. The toolbar panel runs
that search -- ``START_NEAREST_SEARCH_SESSION`` for boundaries, then
``SEARCH_NEAREST`` around the aeroplane -- and posts what the sim answers
to ``/api/sim_airspace`` here.

``attach()`` is called when the engine adopts a simulator, ``detach()``
when it loses it, and :func:`volumes` is merged into
``NavDB.airspace_volumes`` so the watch and the maps see the sim's
volumes beside the filed ones.
"""

from __future__ import annotations

import logging
import math
import re
import threading
from pathlib import Path

from .build import bearing_deg, haversine_nm, project
from .vfrdata import Volume

log = logging.getLogger(__name__)

METERS_TO_FT = 3.280839895013
METERS_TO_NM = 1.0 / 1852.0
# "Unlimited" has to be a number for floor/ceiling arithmetic; no flight
# this flies gets near it.
UNLIMITED_FT = 999_999

_lock = threading.Lock()
_mode = ""
_xplane: list[Volume] | None = None
_msfs: dict[str, list[Volume]] = {}
# Set when a fresh sim attach emptied the store: the panel keeps the
# boundaries it learned, so it must be asked to send them all again.
_msfs_resync = False
_version = 0


# ----------------------------------------------------------------------
# shared plumbing
# ----------------------------------------------------------------------

def attach(source: str) -> None:
    """The engine adopted a simulator: bind its airspace channel.

    X-Plane's files are parsed on a worker so the engine is not stalled
    reading a twenty-megabyte file the moment the sim appears; the
    volumes turn up when the parse finishes. MSFS volumes arrive on their
    own over HTTP from the in-sim panel, whenever it learns them.
    """
    global _mode, _msfs_resync
    kind = {"xplane": "xplane", "simconnect": "msfs"}.get(source, "")
    with _lock:
        _mode = kind
        _msfs.clear()
        _msfs_resync = kind == "msfs"
        _bump()
    if kind == "xplane":
        threading.Thread(target=_parse_xplane, daemon=True,
                         name="xplane-airspace").start()


def detach() -> None:
    """The simulator went away: its airspace goes with it."""
    global _mode, _msfs_resync
    with _lock:
        _mode = ""
        _msfs.clear()
        _msfs_resync = False
        _bump()


def version() -> int:
    """Bumped whenever the set can change; callers cache on it."""
    with _lock:
        return _version


def volumes() -> list[Volume]:
    """The volumes the current simulator knows, or nothing."""
    with _lock:
        if _mode == "xplane" and _xplane:
            return list(_xplane)
        if _mode == "msfs":
            return [v for shapes in _msfs.values() for v in shapes]
        return []


def _bump() -> None:
    global _version
    _version += 1


# ----------------------------------------------------------------------
# X-Plane: OpenAir files in the install
# ----------------------------------------------------------------------

# What the class letter asks of the volumes that wear it, mapped onto the
# letters ENTRY_RULES knows. CTR is a control zone: class D in every way
# that matters here. RMZ/TMZ need the radio the way a C does. Glider,
# wave, FIS and other advisory kinds are kept as E so they draw on the
# map without ever warning.
_OPENAIR_CLASSES = {"CTR": "D", "RMZ": "C", "TMZ": "C"}

_DMS = re.compile(
    r"(\d+):(\d+)(?::([\d.]+))?\s*([NS])\s+(\d+):(\d+)(?::([\d.]+))?\s*([EW])")
_LIMIT = re.compile(r"([\d.]+)\s*([A-Z]*)")


def _openair_class(text: str) -> str | None:
    word = (text or "").strip().upper()
    if not word:
        return None
    mapped = _OPENAIR_CLASSES.get(word)
    if mapped:
        return mapped
    if len(word) == 1 and word in "ABCDEPQRFG":
        return word
    return "E"


def _dms(match: re.Match) -> tuple[float, float]:
    d, m, s, ns, dd, mm, ss, ew = match.groups()
    lat = int(d) + int(m) / 60.0 + float(s or 0.0) / 3600.0
    lon = int(dd) + int(mm) / 60.0 + float(ss or 0.0) / 3600.0
    return (-lat if ns == "S" else lat, -lon if ew == "W" else lon)


def _limit(text: str, upper: bool) -> tuple[int, str]:
    """An OpenAir altitude: ``ft, ref`` with ref SFC, AGL or MSL."""
    t = (text or "").strip().upper()
    if t in ("GND", "SFC", "GROUND", "SURFACE"):
        return (0, "MSL") if upper else (0, "SFC")
    if t.startswith("FL"):
        try:
            return (int(float(t[2:].strip()) * 100), "MSL")
        except ValueError:
            return (UNLIMITED_FT, "MSL") if upper else (0, "MSL")
    if t in ("UNL", "UNLIMITED", "ASKNOTAM", "NOTAM"):
        return (UNLIMITED_FT, "MSL") if upper else (0, "MSL")
    m = _LIMIT.match(t)
    if not m:
        return (UNLIMITED_FT, "MSL") if upper else (0, "MSL")
    value = float(m.group(1))
    unit = m.group(2)
    if unit == "M":
        value *= METERS_TO_FT
    return (int(round(value)), "AGL" if unit == "AGL" else "MSL")


def _arc_points(center: tuple[float, float], a: tuple[float, float],
                b: tuple[float, float], radius_nm: float,
                clockwise: bool) -> list[tuple[float, float]]:
    """Points from ``a`` to ``b`` on the circle round ``center``, every
    five degrees of arc so the polygon test stays honest on a curve."""
    start = bearing_deg(center[0], center[1], a[0], a[1])
    end = bearing_deg(center[0], center[1], b[0], b[1])
    span = (end - start) % 360.0 if clockwise else (start - end) % 360.0
    if span <= 0.01:
        span = 360.0
    steps = max(2, int(span / 5.0) + 1)
    out = []
    for i in range(1, steps + 1):
        angle = start + (span * i / steps) * (1 if clockwise else -1)
        out.append(project(center[0], center[1], angle % 360.0, radius_nm))
    return out


def parse_openair(text: str, source: str, first_id: int = 0) -> list[Volume]:
    """The volumes in an OpenAir file: AC/AN/AL/AH/DP/DB/DC/DA and V X/V D.

    Records this cannot read are skipped, like the CSV loader does: one
    bad entry in a file must not lose the rest.
    """
    found: list[Volume] = []
    cls = name = None
    lower = (0, "SFC")
    upper = (UNLIMITED_FT, "MSL")
    ring: list[tuple[float, float]] = []
    center = (0.0, 0.0)
    clockwise = True
    shapes: list[tuple] = []
    number = first_id

    def flush() -> None:
        nonlocal cls, name, lower, upper, ring, center, clockwise, shapes
        nonlocal number
        if cls is not None:
            number += 1
            vid = f"{source}:{number}"
            common = dict(id=vid, name=name or "", cls=cls, airport="",
                          lower_ft=lower[0], lower_ref=lower[1],
                          upper_ft=upper[0], upper_ref=upper[1],
                          hours="", source=source)
            if shapes:
                for _kind, clat, clon, radius in shapes:
                    dlat = radius / 60.0
                    dlon = radius / (60.0 * max(
                        0.01, math.cos(math.radians(clat))))
                    found.append(Volume(
                        shape="circle", lat=clat, lon=clon, radius_nm=radius,
                        bbox=(clat - dlat, clon - dlon,
                              clat + dlat, clon + dlon), **common))
            elif len(ring) >= 3:
                lats = [p[0] for p in ring]
                lons = [p[1] for p in ring]
                found.append(Volume(shape="polygon", ring=tuple(ring),
                                    bbox=(min(lats), min(lons),
                                          max(lats), max(lons)), **common))
        cls = name = None
        lower, upper = (0, "SFC"), (UNLIMITED_FT, "MSL")
        ring, shapes = [], []
        center = (0.0, 0.0)
        clockwise = True

    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("*"):
            continue
        tag, _, rest = line.partition(" ")
        tag = tag.upper()
        rest = rest.strip()
        if tag == "AC":
            flush()
            cls = _openair_class(rest)
        elif cls is None:
            continue
        elif tag == "AN":
            name = rest
        elif tag == "AL":
            lower = _limit(rest, upper=False)
        elif tag == "AH":
            upper = _limit(rest, upper=True)
        elif tag == "DP":
            m = _DMS.search(rest)
            if m:
                ring.append(_dms(m))
        elif tag == "V":
            key, _, value = rest.partition("=")
            key = key.strip().upper()
            if key == "X":
                m = _DMS.search(value)
                if m:
                    center = _dms(m)
            elif key == "D":
                clockwise = value.strip() != "-"
        elif tag == "DC":
            try:
                radius = float(rest.split()[0])
                if center != (0.0, 0.0) and radius > 0:
                    shapes.append(("circle", center[0], center[1], radius))
            except (ValueError, IndexError):
                pass
        elif tag in ("DB", "DA"):
            try:
                if tag == "DB":
                    a_raw, _, b_raw = rest.partition(",")
                    ma, mb = _DMS.search(a_raw), _DMS.search(b_raw)
                    if not (ma and mb):
                        continue
                    a, b = _dms(ma), _dms(mb)
                    radius = haversine_nm(center[0], center[1], a[0], a[1])
                else:
                    parts = rest.replace(",", " ").split()
                    radius, a1, a2 = (float(parts[0]), float(parts[1]),
                                      float(parts[2]))
                    a = project(center[0], center[1], a1, radius)
                    b = project(center[0], center[1], a2, radius)
                if radius <= 0 or center == (0.0, 0.0):
                    continue
                # The arc starts at ``a``; when the outline is somewhere
                # else there is an implied straight leg to it first.
                if not ring or haversine_nm(ring[-1][0], ring[-1][1],
                                            a[0], a[1]) > 0.02:
                    ring.append(a)
                ring.extend(_arc_points(center, a, b, radius, clockwise))
            except (ValueError, IndexError):
                pass
    flush()
    return found


def xplane_files(roots=None) -> list[Path]:
    """The OpenAir files in each X-Plane install, the pilot's own first.

    ``Custom Data/airspaces`` holds the AIRAC the pilot installed and
    wins over the stock copy under ``Resources/default data`` -- an
    install with the former does not also read the latter, like the sim.
    """
    if roots is None:
        from ..xplane_plugin import install_roots

        roots = install_roots()
    found: list[Path] = []
    for root in roots:
        root = Path(root)
        custom = sorted((root / "Custom Data" / "airspaces").glob("*.txt"))
        if custom:
            found.extend(custom)
            continue
        default = sorted(
            (root / "Resources" / "default data" / "airspaces").glob("*.txt"))
        found.extend(default)
    return found


def load_xplane(paths=None) -> list[Volume]:
    """Every volume in the files, ids unique across them."""
    out: list[Volume] = []
    for i, path in enumerate(paths if paths is not None else xplane_files()):
        path = Path(path)
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            log.warning("could not read X-Plane airspace %s: %s", path, exc)
            continue
        parsed = parse_openair(text, f"xp{i}", first_id=len(out))
        log.info("%s: %d airspace volumes", path.name, len(parsed))
        out.extend(parsed)
    return out


def _parse_xplane() -> None:
    global _xplane
    parsed = load_xplane()
    with _lock:
        if _mode == "xplane":
            _xplane = parsed
            _bump()
            log.info("X-Plane airspace: %d volumes", len(parsed))


# ----------------------------------------------------------------------
# MSFS: boundaries the in-sim panel finds and posts back
# ----------------------------------------------------------------------

# BoundaryType -> the letter ENTRY_RULES knows. Tower, ground and
# clearance volumes are the field's zone; departure and approach areas
# act like a C. MOA, warning, alert, Mode-C, radar and training areas
# advise rather than forbid: kept as E so they draw without warning.
_MSFS_CLASSES = {
    1: "E",   # Center
    2: "A", 3: "B", 4: "C", 5: "D", 6: "E", 7: "F", 8: "G",
    9: "D",   # Tower
    10: "D",  # Clearance
    11: "D",  # Ground
    12: "C",  # Departure
    13: "C",  # Approach
    14: "E",  # MOA
    15: "R",  # Restricted
    16: "P",  # Prohibited
    17: "E",  # Warning
    18: "E",  # Alert
    19: "Q",  # Danger
    20: "E",  # National park
    21: "E",  # Mode-C veil
    22: "E",  # Radar
    23: "E",  # Training
}


def _msfs_limit(alt_m, alt_type: int, upper: bool) -> tuple[int, str]:
    """BoundaryAltitudeType: 0 unknown, 1 MSL, 2 AGL, 3 unlimited."""
    if alt_type == 3:
        return (UNLIMITED_FT, "MSL") if upper else (0, "MSL")
    if alt_type == 2:
        return (int(round((alt_m or 0.0) * METERS_TO_FT)), "AGL")
    if alt_type == 1:
        return (int(round((alt_m or 0.0) * METERS_TO_FT)), "MSL")
    return (UNLIMITED_FT, "MSL") if upper else (0, "MSL")


def boundary_volumes(raw: dict) -> list[Volume]:
    """One sim boundary facility as its Volumes.

    A facility can be several shapes -- disjoint outlines and circles,
    each delimited by its own Start vector -- and every shape becomes a
    zone of its own. Arcs and circles are centred on the origin vector
    their ``originId`` names, never on the last origin seen; an arc's
    radius is the origin's distance to the arc's end point, the way the
    stock avionics flatten the same data.
    """
    cls = _MSFS_CLASSES.get(raw.get("type"), "E")
    fid = raw.get("id")
    lower = _msfs_limit(raw.get("minAlt"), raw.get("minAltType", 0),
                        upper=False)
    upper = _msfs_limit(raw.get("maxAlt"), raw.get("maxAltType", 0),
                        upper=True)
    common = dict(
        name=str(raw.get("name") or ""), cls=cls, airport="",
        lower_ft=lower[0], lower_ref=lower[1],
        upper_ft=upper[0], upper_ref=upper[1],
        hours="", source="msfs")
    rings: list[list[tuple[float, float]]] = []
    ring: list[tuple[float, float]] = []
    origins: dict[int, tuple[float, float]] = {}
    last_origin: tuple[float, float] | None = None
    circles: list[tuple[float, float, float]] = []
    for vec in raw.get("vectors") or []:
        kind = vec.get("type")
        lat, lon = vec.get("lat"), vec.get("lon")
        if lat is None or lon is None:
            continue
        if kind == 3:      # Origin: registers its own position by id
            origins[vec.get("originId")] = (lat, lon)
            last_origin = (lat, lon)
        elif kind == 6:    # Circle round the origin its originId names
            radius = (vec.get("radius") or 0.0) * METERS_TO_NM
            center = origins.get(vec.get("originId"), last_origin)
            if radius > 0:
                clat, clon = center if center is not None else (lat, lon)
                circles.append((clat, clon, radius))
        elif kind in (4, 5):  # Arc to this point round its named origin
            center = origins.get(vec.get("originId"), last_origin)
            if center is None or not ring:
                ring.append((lat, lon))
                continue
            radius = haversine_nm(center[0], center[1], lat, lon)
            if radius <= 0:
                continue
            ring.extend(_arc_points(center, ring[-1], (lat, lon), radius,
                                    clockwise=(kind == 4)))
        elif kind == 1:    # Start: another outline begins
            if ring:
                rings.append(ring)
            ring = [(lat, lon)]
        else:              # Line
            ring.append((lat, lon))
    if ring:
        rings.append(ring)
    found: list[Volume] = []
    for clat, clon, radius in circles:
        dlat = radius / 60.0
        dlon = radius / (60.0 * max(0.01, math.cos(math.radians(clat))))
        found.append(Volume(
            id=f"msfs:{fid}:{len(found)}", shape="circle",
            lat=clat, lon=clon, radius_nm=radius,
            bbox=(clat - dlat, clon - dlon,
                  clat + dlat, clon + dlon), **common))
    for ring in rings:
        if len(ring) < 3:
            continue
        lats = [p[0] for p in ring]
        lons = [p[1] for p in ring]
        found.append(Volume(
            id=f"msfs:{fid}:{len(found)}", shape="polygon",
            ring=tuple(ring),
            bbox=(min(lats), min(lons), max(lats), max(lons)), **common))
    return found


def feed_msfs(payload: dict) -> dict:
    """Boundaries the panel's search answered with.

    ``boundaries`` are added or replaced by facility id, ``remove`` drops
    ids the sim stopped reporting, ``reset`` clears first -- the search
    answers in diffs against the previous query, and the panel forwards
    them verbatim. The answer carries ``resync`` when the store was
    rebuilt (the app restarted or the sim was re-adopted): the panel
    then reposts everything it knows with ``reset`` set.
    """
    global _msfs_resync
    added = removed = 0
    resync = False
    with _lock:
        if _mode != "msfs":
            return {"volumes": 0, "resync": False}
        if payload.get("reset"):
            _msfs.clear()
            _msfs_resync = False
        elif _msfs_resync:
            resync = True
            _msfs_resync = False
        for raw in payload.get("boundaries") or []:
            shapes = boundary_volumes(raw)
            if shapes:
                _msfs[f"msfs:{raw.get('id')}"] = shapes
                added += 1
        for vid in payload.get("remove") or []:
            if _msfs.pop(f"msfs:{vid}", None) is not None:
                removed += 1
        if added or removed or payload.get("reset"):
            _bump()
        total = sum(len(shapes) for shapes in _msfs.values())
    return {"volumes": total, "resync": resync}


__all__ = ["attach", "boundary_volumes", "detach", "feed_msfs",
           "load_xplane", "parse_openair", "version", "volumes",
           "xplane_files"]
