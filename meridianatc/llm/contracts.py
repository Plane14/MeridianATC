"""Typed contracts for tactical decisions a model may propose.

The older advisory layer passes strings and option indices directly. This
module names the boundary more precisely: the engine enumerates legal
candidates, supplies the state they were legal in, and names the deterministic
baseline; the model may only choose one candidate or decline them all. The
objects are deliberately small and serialisable so every decision can be
logged and replayed.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any

#: The pseudo-option a model may choose when the request allows no action.
NONE_ID = "none"


class ContractError(ValueError):
    """A decision request or candidate that is not safe to submit."""


def _jsonable(value: Any) -> bool:
    try:
        json.dumps(value, default=str)
    except (TypeError, ValueError):
        return False
    return True


def _clean_mapping(value: Any, name: str) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ContractError(f"{name} must be a mapping")
    out = dict(value)
    if not all(isinstance(k, str) and k for k in out):
        raise ContractError(f"{name} keys must be non-empty strings")
    if not _jsonable(out):
        raise ContractError(f"{name} must be serialisable")
    return out


@dataclass(frozen=True)
class DecisionOption:
    """One engine-generated action the model may choose.

    ``facts`` is explanatory state for the model and the log. It is not an
    execution channel: the caller must look the chosen option up in its own
    candidate table and execute only the deterministic action attached there.
    """

    id: str
    label: str
    facts: dict[str, Any] = field(default_factory=dict)
    safety_critical: bool = False
    expires_s: float = 5.0

    def __post_init__(self) -> None:
        if not isinstance(self.id, str) or not self.id.strip():
            raise ContractError("option id must be a non-empty string")
        if not isinstance(self.label, str) or not self.label.strip():
            raise ContractError("option label must be a non-empty string")
        if self.id == NONE_ID:
            raise ContractError("'none' is reserved")
        object.__setattr__(self, "id", self.id.strip())
        object.__setattr__(self, "label", self.label.strip())
        object.__setattr__(self, "facts", _clean_mapping(self.facts, "facts"))
        if not 0.1 <= float(self.expires_s) <= 600.0:
            raise ContractError("expires_s must be between 0.1 and 600")


@dataclass(frozen=True)
class DecisionRequest:
    """One bounded tactical question, generated from deterministic state."""

    scope: str
    question: str
    state_digest: dict[str, Any]
    state_version: int
    options: tuple[DecisionOption, ...]
    baseline_id: str
    allow_none: bool = True
    timeout_s: float = 0.8
    cache_key: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.scope, str) or not self.scope.strip():
            raise ContractError("scope must be a non-empty string")
        object.__setattr__(self, "scope", self.scope.strip())
        if not isinstance(self.question, str) or not self.question.strip():
            raise ContractError("question must be a non-empty string")
        object.__setattr__(
            self, "state_digest",
            _clean_mapping(self.state_digest, "state_digest"))
        if self.state_version < 0:
            raise ContractError("state_version must be non-negative")
        object.__setattr__(self, "options", tuple(self.options or ()))
        if not self.options:
            raise ContractError("at least one option is required")
        if not all(isinstance(o, DecisionOption) for o in self.options):
            raise ContractError("options must be DecisionOption objects")
        ids = [o.id for o in self.options]
        if len(ids) != len(set(ids)):
            raise ContractError("option ids must be unique")
        if self.baseline_id not in ids:
            raise ContractError("baseline_id must name an option")
        if not 0.05 <= float(self.timeout_s) <= 30.0:
            raise ContractError("timeout_s must be between 0.05 and 30")
        if not isinstance(self.cache_key, str):
            raise ContractError("cache_key must be a string")

    @property
    def key(self) -> tuple[str, str]:
        return self.scope, self.cache_key

    def option(self, option_id: str) -> DecisionOption | None:
        for option in self.options:
            if option.id == option_id:
                return option
        return None

    @property
    def baseline(self) -> DecisionOption:
        found = self.option(self.baseline_id)
        if found is None:  # impossible after validation; keeps type checkers honest
            raise ContractError("baseline missing")
        return found


@dataclass(frozen=True)
class DecisionResult:
    """What the gateway decided, and why.

    ``selected_id`` is the action the caller may execute. In shadow mode it is
    always the deterministic baseline even when ``model_id`` holds a valid
    model pick. ``valid`` means the model answer itself survived structural
    and operational validation; a fallback result can therefore be both safe
    to execute and ``valid=False`` as a model result.
    """

    scope: str
    selected_id: str
    source: str          # model | shadow | fallback | veto | none | off
    baseline_id: str
    model_id: str | None = None
    confidence: float = 0.0
    valid: bool = False
    reason: str = ""
    latency_s: float = 0.0
    stale: bool = False
    state_version: int = 0
    cache_key: str = ""
    completed_at: float = field(default_factory=time.monotonic)

    def as_log(self) -> dict[str, Any]:
        return {
            "scope": self.scope,
            "selected": self.selected_id,
            "source": self.source,
            "baseline": self.baseline_id,
            "model": self.model_id,
            "valid": self.valid,
            "reason": self.reason,
            "latency_ms": round(self.latency_s * 1000.0, 1),
            "stale": self.stale,
            "state_version": self.state_version,
        }


__all__ = [
    "ContractError", "DecisionOption", "DecisionRequest", "DecisionResult",
    "NONE_ID",
]
