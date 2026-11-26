"""Asynchronous tactical decision gateway.

``Decisions`` is the synchronous advisory layer used on paths that can afford
to wait a bounded moment. This gateway is for decisions made inside moving
simulation state: callers submit the legal candidate set, continue on the
deterministic baseline, and inspect a completed result on a later tick. The
model is therefore incapable of blocking or directly moving anything.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from collections import Counter
from collections.abc import Callable
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import replace

from ..logs import decision
from .contracts import (
    NONE_ID,
    DecisionRequest,
    DecisionResult,
)
from .decisions import _parse_choice

log = logging.getLogger(__name__)

ModeFor = Callable[[str], str]
Validator = Callable[[DecisionRequest, str], str | None]


def _normal_mode(mode: str | None) -> str:
    mode = (mode or "off").strip().lower()
    return mode if mode in ("off", "shadow", "active") else "off"


class DecisionGateway:
    """Bounded model selection with deterministic fallback.

    ``mode_for(scope)`` returns ``off``, ``shadow`` or ``active``. ``off`` is
    not merely a paused model: no provider availability check or network call
    is made. In ``shadow`` the model is asked but the baseline is returned. In
    ``active`` a valid model choice may be returned; a timeout, malformed
    answer, unavailable provider or operational veto still returns baseline.
    """

    _SYSTEM = (
        "You advise an air traffic control simulation. You are given one "
        "operational situation, the deterministic baseline, and numbered "
        "legal options. Answer only with a JSON object {\"choice\": N}, or "
        "{\"choice\": \"none\"} when no optional action should be taken. "
        "Never invent an option, instruction, clearance or movement."
    )

    def __init__(self, provider=None, *,
                 mode_for: ModeFor | dict[str, str] | None = None,
                 validators: dict[str, Validator | list[Validator]] | None = None,
                 max_workers: int = 2,
                 default_timeout_s: float = 0.8,
                 clock: Callable[[], float] = time.monotonic):
        self.provider = provider
        if callable(mode_for):
            self._mode_for = mode_for
        else:
            modes = dict(mode_for or {})
            self._mode_for = lambda scope: modes.get(scope, "off")
        self._validators: dict[str, list[Validator]] = {}
        for scope, validator in (validators or {}).items():
            values = validator if isinstance(validator, list) else [validator]
            self._validators[scope] = list(values)
        self._clock = clock
        self.default_timeout_s = max(0.05, float(default_timeout_s or 0.8))
        self._executor = ThreadPoolExecutor(
            max_workers=max(1, max_workers), thread_name_prefix="ai-decision")
        self._lock = threading.RLock()
        self._pending: dict[tuple[str, str], Future] = {}
        self._results: dict[tuple[str, str], DecisionResult] = {}
        self._scopes: dict[str, Counter] = {}
        self._closed = False

    @property
    def scopes(self) -> dict[str, Counter]:
        return self._scopes

    def mode(self, scope: str) -> str:
        try:
            return _normal_mode(self._mode_for(scope))
        except Exception:
            return "off"

    def register_validator(self, scope: str, validator: Validator) -> None:
        """Add an operational validator for one scope, or ``*`` for all.

        The validator receives the request and the model-selected option id;
        it returns ``None`` when the choice is legal or a short reason when it
        must be vetoed.
        """
        self._validators.setdefault(scope, []).append(validator)

    # ------------------------------------------------------------------
    # submission/result handling
    # ------------------------------------------------------------------

    def _cap_request(self, request: DecisionRequest) -> DecisionRequest:
        """Apply the installation's maximum tactical wait to a request."""
        if request.timeout_s <= self.default_timeout_s:
            return request
        return replace(request, timeout_s=self.default_timeout_s)

    def submit(self, request: DecisionRequest) -> None:
        """Ask the model asynchronously unless the scope is off.

        Repeated submission of the same scope/cache key is deduplicated while
        a call is pending. Invalid requests produce an immediate fallback
        result rather than raising into the simulation loop.
        """
        if not isinstance(request, DecisionRequest):
            return
        request = self._cap_request(request)
        mode = self.mode(request.scope)
        if mode == "off":
            self._count(request.scope, "off")
            self._record_immediate(request, self._fallback(
                request, "off", "scope off", latency=0.0, valid=True))
            return
        key = request.key
        with self._lock:
            if self._closed:
                self._record_immediate(request, self._fallback(
                    request, "fallback", "gateway closed", latency=0.0))
                return
            old = self._pending.get(key)
            if old is not None and not old.done():
                self._count(request.scope, "deduplicated")
                return
            self._count(request.scope, "asked")
            future = self._executor.submit(self._resolve, request, mode)
            self._pending[key] = future
            future.add_done_callback(
                lambda done, req=request: self._finished(req, done))

    def current(self, scope: str, cache_key: str,
                state_version: int) -> DecisionResult | None:
        """The latest result for a decision key, marked stale when needed."""
        key = (scope, cache_key)
        with self._lock:
            result = self._results.get(key)
        if result is None:
            return None
        stale = result.stale or result.state_version != state_version
        # Candidate expiry is deliberately conservative: no result may be
        # trusted forever, even where the caller forgets to version state.
        if time.monotonic() - result.completed_at > 600.0:
            stale = True
        return replace(result, stale=stale) if stale != result.stale else result

    def select_sync(self, request: DecisionRequest) -> DecisionResult:
        """A bounded synchronous selection for infrequent decision points."""
        request = self._cap_request(request)
        mode = self.mode(request.scope)
        if mode == "off":
            self._count(request.scope, "off")
            return self._fallback(request, "off", "scope off", 0.0, valid=True)
        self._count(request.scope, "asked")
        future = self._executor.submit(self._resolve, request, mode)
        started = self._clock()
        try:
            return future.result(timeout=request.timeout_s)
        except Exception:
            self._count(request.scope, "overtime")
            return self._fallback(
                request, "fallback", "timeout",
                latency=max(0.0, self._clock() - started))

    # ------------------------------------------------------------------
    # model resolution and validation
    # ------------------------------------------------------------------

    def _prompt(self, request: DecisionRequest) -> str:
        lines = [
            f"Scope: {request.scope}",
            f"Question: {request.question}",
            "State: " + json.dumps(request.state_digest, ensure_ascii=False,
                                   default=str, sort_keys=True),
            f"Deterministic baseline: {request.baseline.label}",
            "Options:",
        ]
        for index, option in enumerate(request.options, 1):
            facts = json.dumps(option.facts, ensure_ascii=False, default=str,
                               sort_keys=True)
            baseline = " (baseline)" if option.id == request.baseline_id else ""
            lines.append(f"{index}. {option.id}: {option.label}{baseline}; {facts}")
        if request.allow_none:
            lines.append("0. none -- take no optional action.")
        return "\n".join(lines)

    def _call_provider(self, prompt: str) -> str | None:
        if self.provider is None:
            return None
        try:
            available = self.provider.available()
        except Exception:
            return None
        if not available:
            return None
        call = getattr(self.provider, "complete", None) \
            or getattr(self.provider, "classify", None)
        if call is None:
            return None
        try:
            return call(prompt, self._SYSTEM)
        except Exception as exc:
            log.info("AI decision provider failed (%s)", type(exc).__name__)
            return None

    def _resolve(self, request: DecisionRequest, mode: str) -> DecisionResult:
        started = self._clock()
        raw = self._call_provider(self._prompt(request))
        latency = max(0.0, self._clock() - started)
        if latency > request.timeout_s:
            self._count(request.scope, "overtime")
            return self._fallback(request, "fallback", "timeout", latency)
        if raw is None:
            self._count(request.scope, "failed")
            return self._fallback(request, "fallback", "provider unavailable", latency)
        choice = _parse_choice(raw)
        if choice == 0:
            choice = "none"
        if choice == "none":
            if not request.allow_none:
                self._count(request.scope, "rejected")
                return self._fallback(request, "fallback", "none not allowed", latency)
            self._count(request.scope, "declined")
            return self._outcome(request, mode, NONE_ID, latency)
        if not isinstance(choice, int) or not 1 <= choice <= len(request.options):
            self._count(request.scope, "rejected")
            return self._fallback(request, "fallback", "invalid choice", latency)
        return self._outcome(request, mode, request.options[choice - 1].id,
                             latency)

    def _outcome(self, request: DecisionRequest, mode: str, model_id: str,
                 latency: float) -> DecisionResult:
        veto = self._validate(request, model_id)
        if veto:
            self._count(request.scope, "vetoed")
            return self._fallback(request, "veto", veto, latency,
                                  model_id=model_id)
        self._count(request.scope, "proposed")
        if mode == "shadow":
            return DecisionResult(
                scope=request.scope, selected_id=request.baseline_id,
                source="shadow", baseline_id=request.baseline_id,
                model_id=model_id, confidence=1.0, valid=True,
                latency_s=latency, state_version=request.state_version,
                cache_key=request.cache_key)
        if model_id == NONE_ID:
            return DecisionResult(
                scope=request.scope, selected_id=NONE_ID, source="none",
                baseline_id=request.baseline_id, model_id=model_id,
                confidence=1.0, valid=True, latency_s=latency,
                state_version=request.state_version,
                cache_key=request.cache_key)
        return DecisionResult(
            scope=request.scope, selected_id=model_id, source="model",
            baseline_id=request.baseline_id, model_id=model_id,
            confidence=1.0, valid=True, latency_s=latency,
            state_version=request.state_version, cache_key=request.cache_key)

    def _validate(self, request: DecisionRequest, option_id: str) -> str:
        if option_id == NONE_ID:
            return ""
        option = request.option(option_id)
        if option is None:
            return "unknown option"
        if option.safety_critical:
            return "safety-critical option is deterministic only"
        validators = list(self._validators.get("*", ())) \
            + list(self._validators.get(request.scope, ()))
        for validator in validators:
            try:
                reason = validator(request, option_id)
            except Exception as exc:
                return f"validator error: {type(exc).__name__}"
            if reason:
                return str(reason)[:160]
        return ""

    def _fallback(self, request: DecisionRequest, source: str, reason: str,
                  latency: float, *, model_id: str | None = None,
                  valid: bool = False) -> DecisionResult:
        return DecisionResult(
            scope=request.scope, selected_id=request.baseline_id,
            source=source, baseline_id=request.baseline_id,
            model_id=model_id, confidence=0.0, valid=valid, reason=reason,
            latency_s=latency, state_version=request.state_version,
            cache_key=request.cache_key)

    # ------------------------------------------------------------------
    # bookkeeping
    # ------------------------------------------------------------------

    def _finished(self, request: DecisionRequest, future: Future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            result = self._fallback(request, "fallback",
                                    f"gateway error: {type(exc).__name__}", 0.0)
        self._record_immediate(request, result)
        with self._lock:
            if self._pending.get(request.key) is future:
                self._pending.pop(request.key, None)

    def _record_immediate(self, request: DecisionRequest,
                          result: DecisionResult) -> None:
        with self._lock:
            self._results[request.key] = result
        try:
            decision("ai_decision", **result.as_log())
        except Exception:
            log.debug("could not write AI decision telemetry", exc_info=True)

    def _count(self, scope: str, key: str) -> None:
        self._scopes.setdefault(scope, Counter())[key] += 1

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        self._executor.shutdown(wait=False, cancel_futures=True)
        if self._scopes:
            summary = {scope: dict(counts)
                       for scope, counts in sorted(self._scopes.items())}
            log.info("AI decision gateway telemetry: %s", summary)
            try:
                decision("ai_decision_stats", scopes=summary)
            except Exception:
                log.debug("could not write AI decision summary", exc_info=True)


__all__ = ["DecisionGateway"]
