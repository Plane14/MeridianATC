"""Where the aeroplane is on the ground -- StormATC's TaxiSurfaceAdvisor.

It only interprets the airport taxi graph the simulator already provided:
nearest stand, the taxiway edge the aeroplane is on, how far along it, and
how far to the clearance limit. It does not read sign objects; "next turn"
is what the issued route's graph says the pilot should expect at the next
junction. Everything a training pilot sees on the surface page comes from
here, so the words "stand 14" or "taxiway B" are the simulator's own data.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

STAND_CAPTURE_M = 45.0
ON_ROUTE_M = 80.0


@dataclass(frozen=True)
class SurfacePosition:
    """Where the aeroplane is on the airport's graph."""
    found: bool = False
    stand: str = ""            # the stand it is on or nearest, if any
    edge: tuple = ()           # (a, b) node pair it sits on
    taxiway: str = ""          # the edge's name, if it has one
    along: float = 0.0         # 0..1 along the edge
    offset_m: float = 0.0      # how far off the named thing


def _from_latlon(layout, lat: float, lon: float) -> tuple[float, float]:
    """The inverse of ``GroundLayout.to_latlon``: degrees to metres."""
    from .ground import EARTH_M

    z = math.radians(lat - layout.latitude) * EARTH_M
    x = math.radians(lon - layout.longitude) * EARTH_M * layout._cos
    return x, z


def locate(layout, lat: float, lon: float) -> SurfacePosition:
    """The aeroplane's position on ``layout``: a stand, or an edge, or not."""
    if layout is None:
        return SurfacePosition()
    x, z = _from_latlon(layout, lat, lon)

    # Nearest stand inside its radius wins over an edge: parked is parked.
    best_stand = None
    best_d = STAND_CAPTURE_M
    for stand in getattr(layout, "stands", ()):
        d = math.hypot(stand.x - x, stand.z - z)
        if d < best_d:
            best_stand, best_d = stand, d
    if best_stand is not None:
        name = getattr(best_stand, "number", "") \
            or getattr(best_stand, "name", "")
        return SurfacePosition(found=True, stand=str(name),
                               offset_m=round(best_d, 1))

    # Else the nearest edge: the taxiway the aeroplane is on.
    best_edge = None
    best_frac = 0.0
    best_off = ON_ROUTE_M
    edges = getattr(layout, "edges", {})
    for a, links in edges.items():
        for b, _length, is_runway, name in links:
            if is_runway:
                continue
            ax, az = layout.xy[a]
            bx, bz = layout.xy[b]
            dx, dz = bx - ax, bz - az
            length2 = dx * dx + dz * dz
            if length2 <= 0:
                continue
            t = max(0.0, min(1.0,
                             ((x - ax) * dx + (z - az) * dz) / length2))
            px, pz = ax + t * dx, az + t * dz
            off = math.hypot(px - x, pz - z)
            if off < best_off:
                best_edge, best_frac, best_off = (a, b), t, off
                best_name = name
    if best_edge is not None:
        return SurfacePosition(found=True, edge=best_edge,
                               taxiway=best_name or "",
                               along=round(best_frac, 2),
                               offset_m=round(best_off, 1))
    return SurfacePosition()


def progress(layout, lat: float, lon: float, route: list[str]) -> dict:
    """Surface training: how far along the issued taxi route the flight is.

    ``route`` is the taxi clearance's taxiway names in order ("B", "A",
    "A5") -- the same list the controller hands the pilot and
    ``session.taxi_route`` keeps. Each name is resolved to the layout's
    edges that carry it; the answer is where on that chain the aeroplane
    is, the next named taxiway ahead, and the metres still to run to the
    clearance limit -- the last named taxiway's end.
    """
    if layout is None or not route:
        return {}
    x, z = _from_latlon(layout, lat, lon)
    edges = getattr(layout, "edges", {})
    # All edges carrying each named taxiway, in route order.
    chains: list[list[tuple]] = []
    for name in route:
        links = [(a, b) for a, outs in edges.items()
                 for b, _l, rw, n in outs
                 if not rw and (n or "").strip().lower()
                 == str(name).strip().lower()]
        chains.append(links)
    pos = locate(layout, lat, lon)
    next_name = ""
    remaining = 0.0
    # Everything after the taxiway the aeroplane is on counts; the one it is
    # on counts by what of it is left.
    found_chain = False
    for i, links in enumerate(chains):
        on_this = pos.found and pos.taxiway and \
            pos.taxiway.strip().lower() == str(route[i]).strip().lower()
        for a, b in links:
            ax, az = layout.xy[a]
            bx, bz = layout.xy[b]
            seg = math.hypot(bx - ax, bz - az)
            if on_this:
                dx, dz = bx - ax, bz - az
                length2 = dx * dx + dz * dz
                t = max(0.0, min(1.0, ((x - ax) * dx + (z - az) * dz)
                                 / length2)) if length2 > 0 else 0.0
                remaining += seg * (1.0 - t)
                found_chain = True
            elif found_chain or not (pos.found and pos.taxiway):
                remaining += seg
        if found_chain and i + 1 < len(route) and not next_name:
            next_name = str(route[i + 1])
    if not found_chain:
        next_name = str(route[0])
        remaining = sum(
            math.hypot(layout.xy[b][0] - layout.xy[a][0],
                       layout.xy[b][1] - layout.xy[a][1])
            for links in chains for a, b in links)
    return {"next_taxiway": next_name,
            "remaining_m": round(remaining, 0),
            "clearance_limit": str(route[-1])}


__all__ = ["SurfacePosition", "locate", "progress",
           "STAND_CAPTURE_M", "ON_ROUTE_M"]
