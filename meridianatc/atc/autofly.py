"""Per-add-on autofly profiles.

StormATC's autofly-profiles/*.json give a matched-on-title Performance
override for the aircraft they know: an A320's numbers, a C172's. The
engine's own class table (flying.performance_for) stays the baseline;
these files refine it when the pilot is in one of the covered add-ons.
"""

from __future__ import annotations

import json
import logging
import math
import threading
from dataclasses import replace
from pathlib import Path

from .flying import Performance

log = logging.getLogger(__name__)

DATA_DIR = Path(__file__).parent / "autofly"

# StormATC's event names are MSFS's own; the autopilot director already
# sends the same events, so only the performance block is consumed here.
_FIELD_MAP = {
    "rotationSpeedKnots": "rotate_kt",
    "climbSpeedKnots": "climb_kt",
    "cruiseSpeedKnots": "cruise_kt",
    "approachSpeedKnots": "approach_kt",
    "climbRateFpm": "climb_fpm",
    "descentRateFpm": "descend_fpm",
}

_profiles: list[dict] = []
_loaded = False
_lock = threading.Lock()


def _load() -> None:
    global _loaded
    with _lock:
        if _loaded:
            return
        for path in sorted(DATA_DIR.glob("*.json")):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                if not isinstance(data, dict):
                    raise ValueError("expected an object")
                if not isinstance(data.get("canAutofly", False), bool):
                    raise ValueError("canAutofly must be a boolean")
                if not data.get("canAutofly"):
                    continue
                matches = data.get("titleMatches")
                if not isinstance(matches, list) or not matches or not all(isinstance(m, str) and m.strip() for m in matches):
                    raise ValueError("titleMatches must contain nonempty strings")
                perf = data.get("performance")
                if not isinstance(perf, dict):
                    raise ValueError("performance must be an object")
                for key in _FIELD_MAP:
                    if key in perf and (isinstance(perf[key], bool) or not isinstance(perf[key], (int, float)) or not math.isfinite(perf[key]) or perf[key] <= 0):
                        raise ValueError(f"performance.{key} must be a positive finite number")
            except (OSError, ValueError) as exc:
                log.warning("autofly profile %s unreadable: %s", path.name, exc)
                continue
            _profiles.append(data)
        _loaded = True
        if _profiles:
            log.info("autofly: %d profile(s) loaded", len(_profiles))


def override(type_code: str, title: str, base: Performance) -> Performance:
    """Refine ``base`` when a shipped profile matches this aircraft title."""
    _load()
    hay = " ".join(filter(None, (type_code, title))).lower()
    for prof in _profiles:
        if any(str(m).lower() in hay for m in prof.get("titleMatches", ())):
            perf = prof.get("performance") or {}
            patch = {v: float(perf[k]) for k, v in _FIELD_MAP.items()
                     if isinstance(perf.get(k), (int, float))}
            if patch:
                return replace(base, **patch)
    return base
