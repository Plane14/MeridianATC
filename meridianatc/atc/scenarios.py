"""Small deterministic scenario/fault-injection harness.

This is deliberately simulator-independent. A scenario mutates a plain state
mapping at scripted times and repeatedly invokes the controller under test.
It gives regression tests the same fault-injection shape a SITL suite has,
without needing MSFS, X-Plane, networking or wall-clock sleeps.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class FaultInjection:
    """Set ``field`` to ``value`` once the scenario reaches ``at_s``."""

    at_s: float
    field: str
    value: Any


@dataclass(frozen=True)
class ScenarioResult:
    """The final state and every controller output, in time order."""

    state: dict
    outputs: tuple[Any, ...]
    faults_applied: tuple[FaultInjection, ...]


Controller = Callable[[dict, float, float], Any]


def run_scenario(initial_state: dict, controller: Controller, *,
                 faults: Iterable[FaultInjection] = (), duration_s: float,
                 step_s: float = 0.1) -> ScenarioResult:
    """Run a deterministic scripted scenario.

    The same state object is passed to the controller each step so tests can
    model persistence. Faults are applied before the controller invocation at
    their timestamp. ``controller`` receives ``(state, now_s, dt)``.
    """
    if step_s <= 0.0 or duration_s < 0.0:
        raise ValueError("duration must be non-negative and step positive")
    state = dict(initial_state)
    ordered = sorted(faults, key=lambda f: f.at_s)
    applied = []
    outputs = []
    now = 0.0
    # Include the endpoint so assertions can inspect the exact moment a
    # persistence timer expires, rather than depending on arange semantics.
    steps = int(round(duration_s / step_s)) + 1
    for index in range(steps):
        now = index * step_s
        while ordered and ordered[0].at_s <= now + step_s * 1e-6:
            fault = ordered.pop(0)
            state[fault.field] = fault.value
            applied.append(fault)
        outputs.append(controller(state, now, step_s))
    return ScenarioResult(state=state, outputs=tuple(outputs),
                          faults_applied=tuple(applied))


__all__ = ["Controller", "FaultInjection", "ScenarioResult", "run_scenario"]
