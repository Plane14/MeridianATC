"""Asking X-Plane for an airport's stands and taxiways: its apt.dat files.

X-Plane has no facility API a UDP client can ask for the taxi graph with, so
this reads what the simulator itself reads: the ``apt.dat`` of the scenery
pack that owns the airport, in ``scenery_packs.ini`` priority order -- the
first pack containing the airport is the one X-Plane loaded. The global
airports file is one of those packs (``*GLOBAL_AIRPORTS*``) and is where
most fields live; it is twelve million lines, so each file's airports are
indexed once by byte offset and the index is cached beside the data root.

An airport block gives the graph the same things the SimConnect facility
API gives :mod:`meridianatc.integrations.taxiways`: taxi nodes (1201) and the
named segments between them (1202, where ``runway`` is the runway itself and
``taxiway_<X>`` names the taxiway), the segments that count as runway zone
(1204), the service-road edges (1206), and the stands (1300). Stands are
not on the graph in apt.dat, so each is linked to its nearest taxi node --
the same thing the MSFS ``PATH_PARKING`` segments say.

The answer is a :class:`meridianatc.atc.ground.GroundLayout`, identical in
shape to the simulator's, so the taxi planner, the pilot's taxi and the
driven traffic use it unchanged. A field whose block has no taxi network
(LPBR has a runway, a helipad and pavement only) decodes to a falsy layout
and the caller falls back to the invented route, exactly as an empty
SimConnect answer does.
"""

from __future__ import annotations

import hashlib
import json
import logging
import math
import re
from pathlib import Path

from ..atc.ground import (
    EARTH_M,
    GroundLayout,
    PATH_PARKING,
    PATH_ROAD,
    PATH_RUNWAY,
    PATH_TAXI,
)
from ..paths import data_root
from ..xplane_plugin import install_roots

log = logging.getLogger(__name__)

# How far a stand may sit from its taxi node and still be joined to it:
# beyond this it is scenery detail, not a place an aeroplane is taxied to.
STAND_LINK_M = 250.0

# Margins around a runway's pavement when deciding which nodes are on it:
# past the edge for the shoulder and safety area, past each end for the
# run-up pads. Wider than a node's own truth, so the links from the
# holding bars in stay taxiway until the pavement itself.
_RUNWAY_EDGE_PAD_M = 15.0
_RUNWAY_END_PAD_M = 60.0

# Stand kinds, in the SimConnect numbering GroundLayout and the planner
# already read: 8 is inside the gate set, 1/2/4 inside the ramp set.
_KIND = {"gate": 8, "tie_down": 1, "hangar": 2, "misc": 4}
# The radius a stand is given from the aircraft classes its apt.dat row
# allows -- the biggest of them, or the type's own default when the row
# says nothing.
_CLASS_RADIUS = {"heavy": 20.0, "jets": 16.0, "fighters": 14.0,
                 "turboprops": 12.0, "props": 8.0, "helos": 6.0}
_TYPE_RADIUS = {"gate": 16.0, "hangar": 12.0, "tie_down": 8.0, "misc": 10.0}

# A stand's number is what the controller says ("stand 8"); it is taken
# from the name only when the name is a stand designator, not a sentence.
_DESIGNATOR = re.compile(r"^[A-Za-z]{0,3}\s?(\d{1,4})[A-Za-z]?$")


def _index_path(path: Path) -> Path:
    """Where one apt.dat's airport index is cached."""
    key = hashlib.sha1(str(path).encode("utf-8", "replace")).hexdigest()[:16]
    return data_root() / "cache" / "xplane-aptdat" / f"{key}.json"


def _scan_index(path: Path) -> dict[str, tuple[int, int]]:
    """Every airport in the file, ICAO -> (start byte, end byte).

    One pass, whatever the file's size: a header row is ``1 <elev> <flags>
    <flags> <ICAO> <name>`` at the start of a line, and the airport ends
    where the next one begins.
    """
    found: dict[str, int] = {}
    order: list[str] = []
    with open(path, "rb") as handle:
        offset = handle.tell()
        for raw in handle:
            if raw.startswith(b"1 ") or raw == b"1\n" or raw == b"1\r\n":
                fields = raw.split(None, 5)
                if len(fields) >= 5 and fields[1].isdigit() \
                        and 3 <= len(fields[4]) <= 6:
                    icao = fields[4].decode("ascii", "ignore").upper()
                    if icao not in found:
                        found[icao] = offset
                        order.append(icao)
            offset = handle.tell()
    end = offset
    return {icao: (found[icao],
                   found[order[i + 1]] if i + 1 < len(order) else end)
            for i, icao in enumerate(order)}


def _on_strip(x: float, z: float, ax: float, az: float, bx: float,
              bz: float, half: float) -> bool:
    """Whether the point lies on the runway's pavement -- within the
    centreline's half width plus margin, and not beyond either end's."""
    dx, dz = bx - ax, bz - az
    length = math.hypot(dx, dz)
    if length == 0.0:
        return False
    along = ((x - ax) * dx + (z - az) * dz) / length
    across = abs((x - ax) * dz - (z - az) * dx) / length
    return -_RUNWAY_END_PAD_M <= along <= length + _RUNWAY_END_PAD_M \
        and across <= half


def _stand_number(name: str) -> int:
    """The number a controller could say for a designator, else 0."""
    first = (name or "").strip().split(" ", 1)[0].rstrip("(")
    match = _DESIGNATOR.match(first)
    return int(match.group(1)) if match else 0


def _stand_radius(aircraft: str, kind: str) -> float:
    """The stand's size from the aeroplanes its apt.dat row allows."""
    allowed = [cls for cls in (aircraft or "").split("|")
               if cls in _CLASS_RADIUS]
    if allowed:
        return max(_CLASS_RADIUS[cls] for cls in allowed)
    return _TYPE_RADIUS.get(kind, 10.0)


class _AirportBlock:
    """One airport's rows, parsed into the pieces GroundLayout wants."""

    def __init__(self):
        self.datum: tuple[float, float] | None = None
        # id -> (lat, lon); edges -> (a_id, b_id, taxiway-name-or-"runway")
        self.nodes: dict[int, tuple[float, float]] = {}
        self.edges: list[tuple[int, int, bool, str]] = []
        self.roads: list[tuple[int, int]] = []
        # (lat, lon, heading, kind, aircraft, name)
        self.stands: list[tuple[float, float, float, str, str, str]] = []
        # (lat, lon, lat, lon, width m) of each paved runway
        self.runways: list[tuple[float, float, float, float, float]] = []


def _parse_block(text: str) -> _AirportBlock:
    """The taxi network of one airport's apt.dat block.

    A 1204 after a 1202 says that segment is inside a runway's zone --
    for routing it is runway pavement, not taxiway. A 1206 is a service
    road; a 1301 only describes the stand before it.
    """
    block = _AirportBlock()
    datum: dict[str, float] = {}
    last_edge = -1
    for raw in text.splitlines():
        fields = raw.split()
        if not fields:
            continue
        code = fields[0]
        try:
            if code == "1302" and len(fields) >= 3:
                if fields[1] in ("datum_lat", "datum_lon"):
                    datum[fields[1][6:]] = float(fields[2])
                if "lat" in datum and "lon" in datum:
                    block.datum = (datum["lat"], datum["lon"])
            elif code == "1201" and len(fields) >= 5:
                block.nodes[int(fields[4])] = (float(fields[1]),
                                               float(fields[2]))
            elif code == "1202" and len(fields) >= 5:
                kind = fields[4].split(" ", 1)[0]
                if kind.startswith("runway"):
                    name = "runway"
                elif kind == "taxiway":
                    name = ""
                elif kind.startswith("taxiway_"):
                    name = kind[8:]
                else:
                    continue
                block.edges.append((int(fields[1]), int(fields[2]),
                                    fields[3] == "twoway", name))
                last_edge = len(block.edges) - 1
            elif code == "1204" and last_edge >= 0:
                a, b, _two, _name = block.edges[last_edge]
                block.edges[last_edge] = (a, b, _two, "runway")
            elif code == "1206" and len(fields) >= 3:
                block.roads.append((int(fields[1]), int(fields[2])))
            elif code == "100" and len(fields) >= 20:
                block.runways.append((
                    float(fields[9]), float(fields[10]),
                    float(fields[18]), float(fields[19]),
                    float(fields[1])))
            elif code == "1300" and len(fields) >= 6:
                block.stands.append((
                    float(fields[1]), float(fields[2]),
                    float(fields[3]) % 360.0,
                    fields[4].lower(),
                    fields[5] if len(fields) > 5 else "",
                    " ".join(fields[6:]) if len(fields) > 6 else ""))
            elif code in ("1", "99") and block.nodes:
                break
        except (ValueError, IndexError):
            continue
    return block


class XPlaneGround:
    """The simulator's taxi network, one airport at a time, from apt.dat."""

    def __init__(self, root: str | Path | None = None):
        try:
            self._roots = install_roots(root)
        except Exception:
            log.debug("no X-Plane install found", exc_info=True)
            self._roots = []
        self._cache: dict[str, GroundLayout | None] = {}
        self._packs: list[Path] | None = None
        self._indices: dict[Path, dict[str, tuple[int, int]]] = {}

    # -- which file answers ----------------------------------------------

    def _files(self) -> list[Path]:
        """Every pack's apt.dat, in scenery_packs.ini priority order.

        First containing the airport wins, which is X-Plane's own rule.
        Without the ini (or an install at all) the answer is whatever
        packs are lying there, global airports last -- the airport the
        simulator loaded is still the likeliest one found.
        """
        if self._packs is not None:
            return self._packs
        files: list[Path] = []
        seen: set[Path] = set()
        for root in self._roots:
            ini = root / "Custom Scenery" / "scenery_packs.ini"
            ordered: list[Path] = []
            try:
                for raw in ini.read_text(encoding="utf-8",
                                         errors="replace").splitlines():
                    line = raw.strip()
                    if not line.startswith("SCENERY_PACK "):
                        continue
                    entry = line[len("SCENERY_PACK "):].strip()
                    if not entry or entry.startswith("DISABLED"):
                        continue
                    if entry == "*GLOBAL_AIRPORTS*":
                        folder = root / "Global Scenery" / "Global Airports"
                    else:
                        folder = Path(entry)
                        if not folder.is_absolute():
                            folder = root / folder
                    ordered.append(folder / "Earth nav data" / "apt.dat")
            except OSError:
                pass
            if not ordered:
                custom = root / "Custom Scenery"
                try:
                    ordered = sorted(custom.glob("*/Earth nav data/apt.dat"))
                    ordered.append(root / "Global Scenery" /
                                   "Global Airports" /
                                   "Earth nav data" / "apt.dat")
                except OSError:
                    pass
            for path in ordered:
                if path not in seen and path.is_file():
                    seen.add(path)
                    files.append(path)
        self._packs = files
        return files

    def _index_for(self, path: Path) -> dict[str, tuple[int, int]]:
        """The file's airports by byte offset, from cache or one scan."""
        if path in self._indices:
            return self._indices[path]
        index: dict[str, tuple[int, int]] = {}
        try:
            stat = path.stat()
            stamp = (stat.st_mtime_ns, stat.st_size)
            disk = _index_path(path)
            try:
                cached = json.loads(disk.read_text("utf-8"))
                if cached.get("stamp") == stamp and \
                        isinstance(cached.get("airports"), dict):
                    index = {key.upper(): tuple(pair)
                             for key, pair in cached["airports"].items()}
            except (OSError, ValueError, TypeError, AttributeError):
                pass
            if not index:
                index = _scan_index(path)
                try:
                    disk.parent.mkdir(parents=True, exist_ok=True)
                    disk.write_text(json.dumps(
                        {"stamp": stamp,
                         "airports": {key: list(pair)
                                      for key, pair in index.items()}}),
                        "utf-8")
                except OSError:
                    pass
        except OSError:
            pass
        self._indices[path] = index
        return index

    # -- the layout -------------------------------------------------------

    def layout(self, ident: str) -> GroundLayout | None:
        """The airport's stands and taxiways, or None where no pack
        describes them -- the same contract as :class:`SimGround`."""
        ident = (ident or "").strip().upper()
        if not ident:
            return None
        if ident in self._cache:
            return self._cache[ident]
        found = self._find(ident)
        self._cache[ident] = found
        if found is not None:
            log.info("%s: %d stands and %d taxi nodes from apt.dat",
                     ident, len(found.stands), len(found.xy))
        return found

    def _find(self, ident: str) -> GroundLayout | None:
        for path in self._files():
            index = self._index_for(path)
            span = index.get(ident)
            if span is None:
                continue
            try:
                with open(path, "rb") as handle:
                    handle.seek(span[0])
                    raw = handle.read(max(0, span[1] - span[0]))
            except OSError:
                continue
            block = _parse_block(raw.decode("utf-8", "replace"))
            return self._build(ident, block)
        return None

    @staticmethod
    def _build(ident: str, block: _AirportBlock) -> GroundLayout | None:
        """The parsed rows as the same points/parkings/paths/names the
        SimConnect reader produces."""
        if not block.nodes and not block.stands:
            return None
        if block.datum is not None:
            lat0, lon0 = block.datum
        elif block.nodes:
            lat0 = sum(lat for lat, _ in block.nodes.values()) \
                / len(block.nodes)
            lon0 = sum(lon for _, lon in block.nodes.values()) \
                / len(block.nodes)
        elif block.stands:
            lat0 = sum(s[0] for s in block.stands) / len(block.stands)
            lon0 = sum(s[1] for s in block.stands) / len(block.stands)
        else:
            return None
        cos = math.cos(math.radians(lat0))

        def xy(lat: float, lon: float) -> tuple[float, float]:
            return (math.radians(lon - lon0) * EARTH_M * cos,
                    math.radians(lat - lat0) * EARTH_M)

        order = sorted(block.nodes)
        index = {node_id: i for i, node_id in enumerate(order)}
        points = [(0, 0, *xy(*block.nodes[node_id])) for node_id in order]

        names: list[str] = []
        named: dict[str, int] = {}

        def name_index(name: str) -> int:
            if name not in named:
                named[name] = len(names)
                names.append(name)
            return named[name]

        paths = []
        taxi_nodes: set[int] = set()
        for a_id, b_id, _two, name in block.edges:
            if a_id not in index or b_id not in index:
                continue
            kind = PATH_RUNWAY if name == "runway" else PATH_TAXI
            if kind == PATH_TAXI:
                taxi_nodes.update((index[a_id], index[b_id]))
            paths.append((kind, 0.0, 0, 0, index[a_id], index[b_id],
                          name_index(name)))
        for a_id, b_id in block.roads:
            if a_id not in index or b_id not in index:
                continue
            paths.append((PATH_ROAD, 0.0, 0, 0, index[a_id], index[b_id],
                          0))

        parkings = []
        stand_xy: list[tuple[float, float]] = []
        for lat, lon, heading, kind, aircraft, name in block.stands:
            x, z = xy(lat, lon)
            stand_xy.append((x, z))
            parkings.append((_KIND.get(kind, 4), 0, "", "",
                             _stand_number(name), 0, heading,
                             _stand_radius(aircraft, kind), x, z))
        # Each stand is joined to its nearest node that carries taxiway
        # segments -- the same thing the MSFS PATH_PARKING segments say,
        # which apt.dat does not give. Nearest-anything would do: a hold
        # marker or a runway-surface node is a dead end or a gate opening
        # onto the runway itself.
        node_xy = [(row[2], row[3]) for row in points]
        for i, (x, z) in enumerate(stand_xy):
            best, nearest = STAND_LINK_M, -1
            for j in taxi_nodes:
                nx, nz = node_xy[j]
                distance = math.hypot(nx - x, nz - z)
                if distance < best:
                    best, nearest = distance, j
            if nearest >= 0:
                paths.append((PATH_PARKING, 0.0, 0, 0, nearest, i, 0))

        layout = GroundLayout(ident, lat0, lon0, points, parkings, paths,
                              names)
        if not layout:
            return None
        # apt.dat's ``runway`` segments are not just the pavement: they are
        # the whole runway environment, each holding position linked to the
        # centreline by one. On the pavement is what the planner's
        # ``runway_nodes`` means -- where holding is incursion and the first
        # non-runway node is where you are clear -- so mark by the runway
        # rows' own geometry instead of by segment kind. Then the hold
        # nodes are ordinary taxi nodes: a departure's hold-short lands at
        # the painted bars and a vacating arrival waits past them.
        if block.runways:
            strips = [(*xy(lat1, lon1), *xy(lat2, lon2),
                       width * 0.5 + _RUNWAY_EDGE_PAD_M)
                      for lat1, lon1, lat2, lon2, width in block.runways]
            layout.runway_nodes = {
                node for node, (x, z) in layout.xy.items()
                if node[0] == "p" and any(
                    _on_strip(x, z, ax, az, bx, bz, half)
                    for ax, az, bx, bz, half in strips)}
        return layout


__all__ = ["XPlaneGround"]
