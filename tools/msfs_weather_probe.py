"""Feasibility probe for the MSFS MapView weather-radar readback idea.

The important question is not whether WASM can draw the radar -- the documented
MapView API plainly can -- but whether a module can get numeric pixels back.
This probe audits an installed MSFS SDK for exactly that missing half.
"""

from __future__ import annotations

import argparse
import json
import os
import re
from pathlib import Path

import requests

READBACK_RE = re.compile(
    r"\b(?:fs\w*(?:read|copy|lock|download|get)\w*(?:pixel|texture|surface|frame)|"
    r"(?:read|copy|map|lock|download)_pixels?)\w*\s*\(",
    re.IGNORECASE)
MAPVIEW_RE = re.compile(r"\bfsMapView[A-Za-z0-9_]*\s*\(")
RENDER_RE = re.compile(r"\bfsRender[A-Za-z0-9_]*\s*\(")


def sdk_roots() -> list[Path]:
    roots = []
    for value in (os.environ.get("MSFS_SDK"), os.environ.get("MSFS_SDK_PATH")):
        if value:
            roots.append(Path(value))
    roots += [
        Path(r"C:\MSFS SDK"), Path(r"C:\Program Files\MSFS SDK"),
        Path(r"C:\Program Files (x86)\MSFS SDK"),
    ]
    return list(dict.fromkeys(roots))


def scan(root: Path) -> dict:
    headers = list(root.rglob("*.h")) + list(root.rglob("*.hpp")) \
        if root.exists() else []
    text = "\n".join(p.read_text(errors="ignore")[:2_000_000]
                     for p in headers)
    mapview = sorted(set(m.group(0)[:-1] for m in MAPVIEW_RE.finditer(text)))
    render = sorted(set(m.group(0)[:-1] for m in RENDER_RE.finditer(text)))
    readback = sorted(set(m.group(0)[:-1] for m in READBACK_RE.finditer(text)))
    radar = [name for name in mapview if "WeatherRadar" in name]
    documented_path = bool(radar and "fsMapViewCreate" in mapview)
    # fsRenderUpdateTexture is host-to-texture and fsRenderGetTextureSize only
    # dimensions; neither is a readback. Any genuinely promising function would
    # have to expose pixels, a mapped texture or a framebuffer copy.
    return {
        "sdk_root": str(root), "exists": root.exists(),
        "headers": len(headers), "mapview_functions": mapview,
        "weather_radar_functions": radar, "render_functions": render,
        "readback_candidates": readback,
        "documented_rendering_path": documented_path,
        "documented_numeric_readback": bool(readback),
    }


def scan_online() -> dict:
    """Audit the published MSFS 2024 API documentation itself."""
    pages = [
        "https://docs.flightsimulator.com/msfs2024/retail/programming-apis/wasm/mapview-api/mapview-api/",
        "https://docs.flightsimulator.com/msfs2024/retail/programming-apis/wasm/gauge-api/gauge-api/",
    ]
    text = "\n".join(requests.get(page, timeout=15).text for page in pages)
    mapview = sorted(set(re.findall(r"\bfsMapView[A-Za-z0-9_]*", text)))
    render = sorted(set(re.findall(r"\bfsRender[A-Za-z0-9_]*", text)))
    radar = [name for name in mapview if "WeatherRadar" in name]
    readback = sorted(set(
        name for name in mapview + render
        if re.search(r"read|copy|lock|download|pixel|framebuffer", name,
                     re.IGNORECASE)
        and name not in {"fsRenderUpdateTexture", "fsRenderGetTextureSize"}))
    return {
        "sdk_root": "MSFS 2024 online documentation", "exists": True,
        "headers": 0, "mapview_functions": mapview,
        "weather_radar_functions": radar, "render_functions": render,
        "readback_candidates": readback,
        "documented_rendering_path": bool(radar and "fsMapViewCreate" in mapview),
        "documented_numeric_readback": bool(readback),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk", type=Path, help="MSFS SDK root")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--online", action="store_true",
                        help="audit the live public SDK documentation")
    args = parser.parse_args()
    if args.online:
        found = scan_online()
    else:
        roots = [args.sdk] if args.sdk else sdk_roots()
        reports = [scan(root) for root in roots]
        found = next((r for r in reports if r["headers"]), reports[0])
    found["verdict"] = (
        "documented numeric radar readback available"
        if found["documented_numeric_readback"] else
        "not viable through the documented public API: MapView can render "
        "weather radar, but the SDK exposes no pixel/texture/framebuffer readback")
    if args.json:
        print(json.dumps(found, indent=2))
    else:
        print(found["verdict"])
        print(f"SDK root: {found['sdk_root']} ({found['headers']} headers)")
        print(f"weather-radar functions: {len(found['weather_radar_functions'])}")
        print(f"readback candidates: {found['readback_candidates'] or 'none'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
