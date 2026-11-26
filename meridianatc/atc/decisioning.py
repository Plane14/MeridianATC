"""Helpers for wiring tactical decision scopes into deterministic systems."""

from __future__ import annotations

import time

from ..llm.contracts import NONE_ID, DecisionOption, DecisionRequest
from .assurance import validate_candidate_state


def candidate_validator(request: DecisionRequest, option_id: str) -> str:
    """Validate one selected option from its producer-supplied facts."""
    option = request.option(option_id)
    return validate_candidate_state(option.facts if option is not None else {})


def select_candidate(gateway, scope: str, question: str,
                     state_digest: dict, candidates: list[dict],
                     baseline_id: str, cache_key: str,
                     state_version: int | None = None,
                     timeout_s: float = 0.8,
                     allow_none: bool = False) -> str:
    """Synchronously select one engine-generated candidate.

    ``candidates`` is a list of dicts carrying at least ``id`` and ``label``;
    all other keys become validation/model facts. The return is the selected
    candidate id, or the baseline when the gateway is absent/off/failing. This
    helper never executes anything itself: callers map the id back to their
    own deterministic behavior.
    """
    if gateway is None or not candidates:
        return baseline_id
    try:
        options = tuple(
            DecisionOption(
                str(c["id"]), str(c.get("label") or c["id"]),
                facts={k: v for k, v in c.items()
                       if k not in ("id", "label", "safety_critical", "expires_s")},
                safety_critical=bool(c.get("safety_critical", False)),
                expires_s=float(c.get("expires_s", 5.0)))
            for c in candidates)
        request = DecisionRequest(
            scope=scope, question=question, state_digest=state_digest,
            state_version=int(state_version if state_version is not None
                              else time.monotonic() * 10),
            options=options, baseline_id=baseline_id,
            allow_none=allow_none, timeout_s=timeout_s,
            cache_key=cache_key)
        result = gateway.select_sync(request)
    except Exception:
        return baseline_id
    if result.stale:
        return baseline_id
    if result.selected_id == NONE_ID:
        return NONE_ID if allow_none else baseline_id
    ids = {str(c["id"]) for c in candidates}
    return result.selected_id if result.selected_id in ids else baseline_id


__all__ = ["candidate_validator", "select_candidate"]
