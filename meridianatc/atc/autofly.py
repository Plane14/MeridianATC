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
                icao = data.get("icao")
                if icao is not None and (not isinstance(icao, list)
                                         or not all(isinstance(c, str)
                                                    and c.strip()
                                                    for c in icao)):
                    raise ValueError("icao must be nonempty strings")
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


def _performance_patch(prof: dict) -> dict:
    """A profile's performance block as a ``Performance`` field patch."""
    perf = prof.get("performance") or {}
    patch = {v: float(perf[k]) for k, v in _FIELD_MAP.items()
             if isinstance(perf.get(k), (int, float))}
    for k, v in _FLAP_MAP.items():
        detent = perf.get(k)
        if isinstance(detent, (int, float)) and detent >= 0:
            # Detent zero is a real answer too -- the flaps-up takeoff
            # a King Air's POH wants -- not a missing one.
            patch[v] = min(1.0, float(detent) / 4.0)
    for k, (v, lo, hi) in _G_MAP.items():
        limit = perf.get(k)
        if isinstance(limit, (int, float)) and lo <= limit <= hi:
            patch[v] = float(limit)
    return patch


def _title_match(prof: dict, hay: str) -> bool:
    return any(str(m).lower() in hay for m in prof.get("titleMatches", ()))


def _profile_class(prof: dict) -> str:
    """The aircraft class a profile's own type belongs to."""
    from . import aircraft_class
    rep = next((str(c) for c in prof.get("icao", ()) if str(c).strip()), "")
    if not rep:
        rep = str((prof.get("titleMatches") or [""])[0])
    return aircraft_class.classify(rep).name


def nearest(type_code: str, title: str, aircraft_class_name: str = "",
            base: Performance | None = None):
    """The closest shipped profile for this aeroplane, and how it matched.

    ``("title"|"icao"|"class", profile)``: the add-on the file was written
    for, the same type under a title it was not filed under, and the class
    it flies like when nothing nearer exists -- a C152 is a C172 flown a
    little slower, and an aeroplane nobody has heard of is still somebody's
    classmate. Within a class the nearest is the profile whose rotation
    speed is closest to what this one would do: a light twin does not want
    the airliner's numbers.
    """
    _load()
    hay = " ".join(filter(None, (type_code, title))).lower()
    for prof in _profiles:
        if _title_match(prof, hay):
            return prof, "title"
    code = (type_code or "").strip().upper()
    if code:
        for prof in _profiles:
            if code in {str(c).strip().upper()
                        for c in prof.get("icao", ())}:
                return prof, "icao"
    want = (aircraft_class_name or "").strip()
    if not want:
        if not code:
            return None, ""
        from . import aircraft_class
        classified = aircraft_class.classify(code)
        if classified.guessed:
            # The commonest-class default is no evidence of class: an
            # aeroplane nobody knows is not borrowed somebody's numbers.
            return None, ""
        want = classified.name
    candidates = [prof for prof in _profiles
                  if _profile_class(prof) == want]
    if not candidates:
        return None, ""
    ref = getattr(base, "rotate_kt", 0.0) or 0.0
    def _distance(prof) -> float:
        theirs = ((prof.get("performance") or {})
                  .get("rotationSpeedKnots"))
        return abs(float(theirs) - ref) \
            if isinstance(theirs, (int, float)) and ref else 0.0
    return min(candidates, key=_distance), "class"


# A borrowed profile's speeds are scaled to the aeroplane being flown by
# at most this far either way: an estimate, not a different aeroplane.
_ADAPT_MIN, _ADAPT_MAX = 0.6, 1.6


def _adapted(patch: dict, prof: dict, base: Performance,
             declared: dict | None) -> dict:
    """Retune a borrowed profile's numbers for the aeroplane in hand.

    What the simulator declares -- its own rotation, cruise and stall
    speeds -- is used directly; what it does not is scaled by how much
    faster or slower this aeroplane rotates than the profile's did, so a
    slower single keeps the shape of a faster one's profile without being
    flown at speeds it cannot reach.
    """
    declared = declared or {}
    perf = prof.get("performance") or {}
    own = perf.get("rotationSpeedKnots")
    ref = float(declared.get("rotate_kt") or 0.0) \
        or float(getattr(base, "rotate_kt", 0.0) or 0.0)
    if isinstance(own, (int, float)) and own > 0 and ref > 0:
        factor = max(_ADAPT_MIN, min(_ADAPT_MAX, ref / float(own)))
    else:
        factor = 1.0
    knots = {"rotate_kt", "climb_kt", "cruise_kt", "approach_kt",
             "touchdown_kt", "goaround_kt"}
    for name, value in declared.items():
        if (name in knots or name == "descend_fpm") \
                and isinstance(value, (int, float)) and value > 0:
            patch[name] = float(value)
    for name in knots & set(patch):
        if name not in declared:
            patch[name] = patch[name] * factor
    return patch


def override(type_code: str, title: str, base: Performance,
             *, aircraft_class: str = "",
             declared: dict | None = None) -> Performance:
    """Refine ``base`` when a shipped profile matches this aircraft --
    its title, its type code, or the class of aeroplane it is.

    A profile matched on anything but the title is borrowed: its numbers
    are retuned to what this aeroplane declares for itself
    (:func:`_adapted`), and the borrow is logged, so a log line says what
    the aeroplane was actually flown as.
    """
    prof, how = nearest(type_code, title, aircraft_class, base)
    if prof is None:
        return base
    patch = _performance_patch(prof)
    if not patch:
        return base
    if how != "title":
        patch = _adapted(patch, prof, base, declared)
        log.info("autofly: no profile for %r (%s); flying it as %s "
                 "(%s match)", title or type_code or "?", type_code or "?",
                 prof.get("name", "?"), how)
    return replace(base, **patch)


def handling(type_code: str, title: str,
             base: FixedWingGains,
             *, aircraft_class: str = "") -> FixedWingGains:
    """Refine ``base``'s control gains where a matched profile says to.

    The same matching and merging discipline as :func:`override`: a
    profile that says nothing about a channel leaves the class's gains
    alone, and one that does is merged term by term.
    """
    prof, how = nearest(type_code, title, aircraft_class, None)
    if prof is None or how == "class":
        # A classmate's gains are no nearer than the class table's own:
        # borrowed numbers are retuned, borrowed loops are not.
        return base
    hand = prof.get("handling")
    if not isinstance(hand, dict):
        return base
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


__all__ = ["handling", "nearest", "override"]
