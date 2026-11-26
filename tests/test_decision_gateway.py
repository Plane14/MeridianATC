from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import pytest

from meridianatc.llm.contracts import (
    NONE_ID,
    ContractError,
    DecisionOption,
    DecisionRequest,
)
from meridianatc.llm.gateway import DecisionGateway


def provider(raw='{"choice": 2}', *, available=True):
    return SimpleNamespace(
        complete=lambda *a, **k: raw,
        classify=lambda *a, **k: raw,
        available=lambda: available,
        name="stub",
    )


def request(**kw):
    options = kw.pop("options", (
        DecisionOption("baseline", "continue as cleared"),
        DecisionOption("hold", "hold position"),
        DecisionOption("avoid", "take the weather deviation"),
    ))
    values = dict(
        scope="traffic.circuit_spacing",
        question="What should the traffic do?",
        state_digest={"airport": "EGLL", "airborne": True},
        state_version=7,
        options=options,
        baseline_id="baseline",
        cache_key="flight-1",
        timeout_s=0.5,
    )
    values.update(kw)
    return DecisionRequest(**values)


def test_contract_rejects_duplicate_ids():
    with pytest.raises(ContractError):
        request(options=(
            DecisionOption("same", "one"),
            DecisionOption("same", "two")))


def test_contract_rejects_a_missing_baseline():
    with pytest.raises(ContractError):
        request(baseline_id="missing")


def test_active_valid_model_choice_is_selected():
    gateway = DecisionGateway(provider(), mode_for={"traffic.circuit_spacing": "active"})
    result = gateway.select_sync(request())
    assert result.source == "model"
    assert result.selected_id == "hold"
    assert result.model_id == "hold"
    assert result.valid
    gateway.close()


def test_shadow_logs_model_choice_but_selects_baseline():
    gateway = DecisionGateway(provider(), mode_for={"traffic.circuit_spacing": "shadow"})
    req = request()
    result = gateway.select_sync(req)
    assert result.source == "shadow"
    assert result.selected_id == "baseline"
    assert result.valid

    completed = None
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and completed is None:
        current = gateway.current(req.scope, req.cache_key, req.state_version)
        if current is not None and current.reason != "shadow pending":
            completed = current
        else:
            time.sleep(0.01)
    assert completed is not None
    assert completed.source == "shadow"
    assert completed.selected_id == "baseline"
    assert completed.model_id == "hold"
    gateway.close()


def test_shadow_does_not_wait_for_a_slow_provider():
    def slow_complete(*_args, **_kwargs):
        time.sleep(0.4)
        return '{"choice": 2}'

    slow = SimpleNamespace(complete=slow_complete, available=lambda: True)
    gateway = DecisionGateway(slow, mode_for={"traffic.circuit_spacing": "shadow"})
    started = time.monotonic()
    result = gateway.select_sync(request())
    elapsed = time.monotonic() - started
    assert result.selected_id == "baseline"
    assert elapsed < 0.2
    gateway.close()


def test_malformed_choice_falls_back():
    gateway = DecisionGateway(provider("I choose the second one"),
                              mode_for={"traffic.circuit_spacing": "active"})
    result = gateway.select_sync(request())
    assert result.source == "fallback"
    assert result.selected_id == "baseline"
    assert not result.valid
    assert result.reason == "invalid choice"
    gateway.close()


def test_none_is_an_explicit_active_outcome():
    gateway = DecisionGateway(provider('{"choice": "none"}'),
                              mode_for={"traffic.circuit_spacing": "active"})
    result = gateway.select_sync(request())
    assert result.source == "none"
    assert result.selected_id == NONE_ID
    gateway.close()


def test_none_rejected_when_not_allowed():
    gateway = DecisionGateway(provider('{"choice": "none"}'),
                              mode_for={"traffic.circuit_spacing": "active"})
    result = gateway.select_sync(request(allow_none=False))
    assert result.selected_id == "baseline"
    assert result.reason == "none not allowed"
    gateway.close()


def test_unavailable_provider_never_throws_and_falls_back():
    gateway = DecisionGateway(provider(available=False),
                              mode_for={"traffic.circuit_spacing": "active"})
    result = gateway.select_sync(request())
    assert result.selected_id == "baseline"
    assert result.reason == "provider unavailable"
    gateway.close()


def test_off_scope_makes_no_provider_call():
    called = []
    p = SimpleNamespace(
        complete=lambda *a, **k: called.append(a) or '{"choice": 2}',
        available=lambda: True,
    )
    gateway = DecisionGateway(p, mode_for={"traffic.circuit_spacing": "off"})
    result = gateway.select_sync(request())
    assert result.source == "off"
    assert result.selected_id == "baseline"
    assert called == []
    gateway.close()


def test_operational_validator_vetoes_a_model_pick():
    gateway = DecisionGateway(
        provider(), mode_for={"traffic.circuit_spacing": "active"},
        validators={"traffic.circuit_spacing": lambda req, pick: "runway protected"})
    result = gateway.select_sync(request())
    assert result.source == "veto"
    assert result.selected_id == "baseline"
    assert result.model_id == "hold"
    assert result.reason == "runway protected"
    gateway.close()


def test_safety_critical_candidate_is_never_model_selected():
    gateway = DecisionGateway(provider(), mode_for={"traffic.circuit_spacing": "active"})
    req = request(options=(
        DecisionOption("baseline", "continue"),
        DecisionOption("hold", "hold", safety_critical=True),
    ))
    result = gateway.select_sync(req)
    assert result.source == "veto"
    assert result.selected_id == "baseline"
    gateway.close()


def test_sync_timeout_falls_back():
    wait = threading.Event()

    def slow(*a, **k):
        wait.wait(2.0)
        return '{"choice": 2}'

    p = SimpleNamespace(complete=slow, available=lambda: True)
    gateway = DecisionGateway(p, mode_for={"traffic.circuit_spacing": "active"})
    started = time.monotonic()
    result = gateway.select_sync(request(timeout_s=0.1))
    assert result.selected_id == "baseline"
    assert time.monotonic() - started < 1.0
    wait.set()
    gateway.close()


def test_submit_is_async_deduplicates_and_current_marks_state_stale():
    release = threading.Event()

    def slow(*a, **k):
        release.wait(2.0)
        return '{"choice": 2}'

    p = SimpleNamespace(complete=slow, available=lambda: True)
    gateway = DecisionGateway(p, mode_for={"traffic.circuit_spacing": "active"})
    req = request()
    gateway.submit(req)
    gateway.submit(req)
    assert gateway.scopes[req.scope]["deduplicated"] == 1
    assert gateway.current(req.scope, req.cache_key, req.state_version) is None
    release.set()
    deadline = time.monotonic() + 2.0
    result = None
    while time.monotonic() < deadline:
        result = gateway.current(req.scope, req.cache_key, req.state_version)
        if result is not None:
            break
        time.sleep(0.01)
    assert result is not None
    assert result.selected_id == "hold"
    assert not result.stale
    stale = gateway.current(req.scope, req.cache_key, req.state_version + 1)
    assert stale is not None and stale.stale
    gateway.close()


def test_config_modes_are_mastered_and_shadow_is_a_demotion():
    from meridianatc.config import AiConfig

    ai = AiConfig(decisions=True, traffic_decisions="active",
                  fo_decisions="shadow", atc_decisions="active")
    assert ai.decision_mode("traffic.circuit_spacing") == "active"
    assert ai.decision_mode("fo.goaround") == "shadow"
    assert ai.decision_mode("crew.priority") == "off"
    ai.decisions_shadow = True
    assert ai.decision_mode("traffic.circuit_spacing") == "shadow"
    ai.decisions = False
    assert ai.decision_mode("traffic.circuit_spacing") == "off"



def test_shadow_workers_do_not_starve_an_active_decision():
    def complete(prompt, *_args, **_kwargs):
        if "active.separation" in prompt:
            return '{"choice": 2}'
        time.sleep(0.5)
        return '{"choice": 2}'

    busy = SimpleNamespace(complete=complete, available=lambda: True)
    gateway = DecisionGateway(
        busy,
        mode_for={"traffic.circuit_spacing": "shadow",
                  "active.separation": "active"},
        max_workers=2)
    gateway.select_sync(request(cache_key="shadow-1"))
    gateway.select_sync(request(cache_key="shadow-2"))

    active = request(scope="active.separation", cache_key="active-1",
                     timeout_s=0.2)
    result = gateway.select_sync(active)
    assert result.source == "model"
    assert result.selected_id == "hold"
    gateway.close()



def test_saturated_active_workers_fall_back_without_queueing():
    entered = []
    entered_lock = threading.Lock()

    def complete(prompt, *_args, **_kwargs):
        if "Scope: slow." in prompt:
            with entered_lock:
                entered.append(prompt)
            time.sleep(0.5)
        return '{"choice": 2}'

    provider = SimpleNamespace(complete=complete, available=lambda: True)
    modes = {"slow.one": "active", "slow.two": "active",
             "fast.active": "active"}
    gateway = DecisionGateway(provider, mode_for=modes, max_workers=2)
    outcomes = []

    def slow(scope):
        outcomes.append(gateway.select_sync(
            request(scope=scope, cache_key=scope, timeout_s=0.7)))

    threads = [threading.Thread(target=slow, args=(scope,))
               for scope in ("slow.one", "slow.two")]
    for thread in threads:
        thread.start()
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline and len(entered) < 2:
        time.sleep(0.005)

    started = time.monotonic()
    result = gateway.select_sync(
        request(scope="fast.active", cache_key="fast", timeout_s=0.2))
    elapsed = time.monotonic() - started
    assert result.selected_id == "baseline"
    assert result.reason == "active decision workers busy"
    assert elapsed < 0.1
    for thread in threads:
        thread.join()
    gateway.close()
