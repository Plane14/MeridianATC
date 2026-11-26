"""Complete structured parameter tracing for the runtime logs.

The ordinary telemetry line is a pilot-sized summary. Debugging the FO,
the phase machine and the simulator adapters needs the rest of the truth:
every aircraft, session, autonomous and non-secret configuration parameter,
but without repeating thousands of unchanged values every five seconds.
This module serializes the complete snapshot and turns later samples into
a change feed from which the full state can be reconstructed.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import fields, is_dataclass
from enum import Enum
from typing import Any

from .logs import _CREDENTIAL_NAME

_PRIMITIVES = (str, int, float, bool, type(None))
# Complex object representations must not contain memory addresses: the
# address changes between processes and makes an unchanged object look like
# a perpetual parameter change. The type name is the stable part the trace
# can honestly promise for objects outside the dataclass state trees.
_MAX_TEXT = 500


def _redacted_key(key: object) -> bool:
    text = str(key)
    return bool(_CREDENTIAL_NAME.fullmatch(text)) or any(
        part in text.lower()
        for part in ("api_key", "apikey", "token", "password", "secret"))


def _short(text: str) -> str:
    return text if len(text) <= _MAX_TEXT else text[:_MAX_TEXT] + "…"


def safe_value(value: Any, *, _depth: int = 0, _seen: set[int] | None = None,
               _key: str = "") -> Any:
    """A JSON-compatible form of a runtime parameter, credentials excepted."""
    if _redacted_key(_key):
        return "<redacted>"
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, os.PathLike):
        return _short(os.fspath(value))
    if isinstance(value, bytes):
        try:
            return _short(value.decode("utf-8"))
        except UnicodeDecodeError:
            return _short(value.hex())
    if hasattr(value, "isoformat") and callable(value.isoformat):
        try:
            return _short(value.isoformat())
        except (TypeError, ValueError):
            pass
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, _PRIMITIVES):
        return _short(value) if isinstance(value, str) else value
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        try:
            return safe_value(tolist(), _depth=_depth, _seen=_seen)
        except (TypeError, ValueError):
            pass
    if _seen is None:
        _seen = set()
    ident = id(value)
    if ident in _seen:
        return f"<{type(value).__name__} cycle>"
    _seen.add(ident)
    try:
        if is_dataclass(value) and not isinstance(value, type):
            if _depth >= 8:
                return f"<{type(value).__name__}>"
            settings = getattr(type(value), "__settings__", {}) or {}
            out = {}
            for item in fields(value):
                setting = settings.get(item.name)
                if getattr(setting, "kind", "") == "secret":
                    out[item.name] = "<redacted>"
                else:
                    out[item.name] = safe_value(
                        getattr(value, item.name), _depth=_depth + 1,
                        _seen=_seen, _key=item.name)
            return out
        if isinstance(value, dict):
            if _depth >= 8:
                return f"<dict[{len(value)}]>"
            return {
                str(key): "<redacted>" if _redacted_key(key) else safe_value(
                    item, _depth=_depth + 1, _seen=_seen, _key=str(key))
                for key, item in value.items()
            }
        if isinstance(value, (list, tuple, set, frozenset)):
            if _depth >= 8:
                return f"<{type(value).__name__}[{len(value)}]>"
            items = sorted(value, key=repr) if isinstance(value, (set, frozenset)) else value
            return [safe_value(item, _depth=_depth + 1, _seen=_seen)
                    for item in items]
        return f"<{type(value).__name__}>"
    finally:
        _seen.discard(ident)


def runtime_snapshot(aircraft: Any, autonomous: Any, session: Any,
                     config: Any = None) -> dict[str, Any]:
    """The complete serializable runtime parameter tree."""
    return {
        "aircraft": safe_value(aircraft),
        "autonomous": safe_value(autonomous),
        "session": safe_value(session),
        "config": safe_value(config),
    }


def flatten(tree: Any, prefix: str = "") -> dict[str, Any]:
    """A snapshot tree as dotted leaf paths, for exact change detection."""
    if isinstance(tree, dict):
        if not tree:
            return {prefix: {}} if prefix else {}
        out: dict[str, Any] = {}
        for key, value in tree.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            out.update(flatten(value, path))
        return out
    if isinstance(tree, list):
        if not tree:
            return {prefix: []}
        out = {}
        for index, value in enumerate(tree):
            out.update(flatten(value, f"{prefix}[{index}]"))
        return out
    return {prefix: tree}


class ParameterTracer:
    """Turn repeated complete snapshots into full snapshots plus changes."""

    FULL_SNAPSHOT_S = 300.0

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._flat: dict[str, Any] | None = None
        self._full_at = 0.0

    def observe(self, snapshot: dict[str, Any],
                now: float | None = None) -> tuple[bool, dict[str, Any]]:
        """Whether this is a full snapshot, and the leaf changes since last."""
        if now is None:
            now = self._clock()
        current = flatten(snapshot)
        full = self._flat is None or now - self._full_at >= self.FULL_SNAPSHOT_S
        changes: dict[str, Any] = {}
        if not full:
            assert self._flat is not None
            for key, value in current.items():
                if key not in self._flat:
                    changes[key] = {"from": None, "to": value}
                elif self._flat[key] != value:
                    changes[key] = {"from": self._flat[key], "to": value}
            for key, value in self._flat.items():
                if key not in current:
                    changes[key] = {"from": value, "to": None}
        self._flat = current
        if full:
            self._full_at = now
        return full, changes


__all__ = ["ParameterTracer", "flatten", "runtime_snapshot", "safe_value"]
