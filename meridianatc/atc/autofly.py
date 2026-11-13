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

from .control_gains import FixedWingGains
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
    "touchdownSpeedKnots": "touchdown_kt",
    "goAroundSpeedKnots": "goaround_kt",
    "maxBankDegrees": "max_bank_deg",
    "climbRateFpm": "climb_fpm",
    "descentRateFpm": "descend_fpm",
}

# Flap handles count detents; the lever event takes a fraction of travel.
# A four-detent handle is the common case: detent 2 on an Airbus is half
# travel, detent 3 on a Cessna is three quarters.
_FLAP_MAP = {
    "takeoffFlaps": "takeoff_flaps",
    "landingFlaps": "landing_flaps",
}

# The structural limits a profile can declare, where it knows its own:
# a model's positive and negative G. Signed values, which is why they
# are their own map -- _FIELD_MAP's every-number-positive validation
# would reject the negative limit. Bounds are loose on purpose: an
# aerobatic mount is a legal profile entry, a typo is not.
_G_MAP = {
    "positiveGLimit": ("pos_g", 1.0, 12.0),
    "negativeGLimit": ("neg_g", -8.0, 0.0),
}

# The handling block's channels and terms: the control-law gains the class
# table seeds and a profile may retune. Unknown channels and terms are
# rejected at load, the way unknown performance fields would be -- a typo
# in a gain file is a misbehaving aeroplane, not a harmless omission.
_GAIN_CHANNELS = {
    "throttle", "steer", "taxi", "rotation", "flare", "vertical",
}
_GAIN_TERMS = {
    "kp", "ki", "kd", "deadband", "integral_cap", "low", "high", "slew",
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
                for key in _FLAP_MAP:
                    if key in perf and (isinstance(perf[key], bool) or not isinstance(perf[key], (int, float)) or not math.isfinite(perf[key]) or not 0 <= perf[key] <= 8):
                        raise ValueError(f"performance.{key} must be a detent number 0..8")
                for key, (_field, lo, hi) in _G_MAP.items():
                    if key in perf and (isinstance(perf[key], bool) or not isinstance(perf[key], (int, float)) or not math.isfinite(perf[key]) or not lo <= perf[key] <= hi):
                        raise ValueError(f"performance.{key} must be a G limit {lo}..{hi}")
                hand = data.get("handling")
                if hand is not None:
                    if not isinstance(hand, dict):
                        raise ValueError("handling must be an object")
                    for channel, terms in hand.items():
                        if channel not in _GAIN_CHANNELS:
                            raise ValueError(f"handling.{channel} is not a channel")
                        if not isinstance(terms, dict):
                            raise ValueError(f"handling.{channel} must be an object")
                        for key, val in terms.items():
                            if key not in _GAIN_TERMS or isinstance(val, bool) \
                                    or not isinstance(val, (int, float)) \
                                    or not math.isfinite(val):
                                raise ValueError(
                                    f"handling.{channel}.{key} must be a finite number")
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
            for k, v in _FLAP_MAP.items():
                detent = perf.get(k)
                if isinstance(detent, (int, float)) and detent > 0:
                    patch[v] = min(1.0, float(detent) / 4.0)
            for k, (v, lo, hi) in _G_MAP.items():
                limit = perf.get(k)
                if isinstance(limit, (int, float)) and lo <= limit <= hi:
                    patch[v] = float(limit)
            if patch:
                return replace(base, **patch)
    return base


def handling(type_code: str, title: str,
             base: FixedWingGains) -> FixedWingGains:
    """Refine ``base``'s control gains where a matched profile says to.

    The same matching and merging discipline as :func:`override`: a
    profile that says nothing about a channel leaves the class's gains
    alone, and one that does is merged term by term.
    """
    _load()
    hay = " ".join(filter(None, (type_code, title))).lower()
    for prof in _profiles:
        if not any(str(m).lower() in hay
                   for m in prof.get("titleMatches", ())):
            continue
        hand = prof.get("handling")
        if not isinstance(hand, dict):
            continue
        patch = {}
        for channel, terms in hand.items():
            current = getattr(base, channel, None)
            if current is None or not isinstance(terms, dict):
                continue
            allowed = {k: float(v) for k, v in terms.items()
                       if k in _GAIN_TERMS
                       and isinstance(v, (int, float))
                       and not isinstance(v, bool)
                       and math.isfinite(v)}
            if allowed:
                patch[channel] = replace(current, **allowed)
        if patch:
            return replace(base, **patch)
    return base
