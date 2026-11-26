import time

from meridianatc.atc.assurance import choose_fallback
from meridianatc.atc.decisioning import candidate_validator, select_candidate
from meridianatc.atc.rotor import RotorSpeedGovernor
from meridianatc.atc.scenarios import FaultInjection, run_scenario
from meridianatc.llm.gateway import DecisionGateway


def test_rotor_droop_fault_is_corrected_then_reported():
    governor = RotorSpeedGovernor(target_pct=100.0, min_pct=95.0,
                                  max_pct=104.0, deadband_pct=1.0,
                                  proportional=0.01, trim_rate=0.0,
                                  max_corr=0.2, fault_after_s=0.5)

    def controller(state, _now, dt):
        return governor.update(0.70, state["rotor_rpm_pct"], dt,
                               collective_pct=state["collective_pct"])

    result = run_scenario(
        {"rotor_rpm_pct": 100.0, "collective_pct": 50.0}, controller,
        faults=[FaultInjection(0.2, "rotor_rpm_pct", 94.0)],
        duration_s=0.8, step_s=0.1)

    assert len(result.faults_applied) == 1
    assert result.outputs[2].fault == ""
    assert result.outputs[2].throttle > 0.70
    persistent = [out for out in result.outputs if out.fault == "underspeed"]
    assert persistent
    assert persistent[-1].throttle > 0.80


def test_engine_failure_scenario_selects_landing_not_route_continuation():
    def controller(state, _now, _dt):
        return choose_fallback(airborne=True,
                               engine_failed=state["engine_failed"]).name

    result = run_scenario(
        {"engine_failed": False}, controller,
        faults=[FaultInjection(0.3, "engine_failed", True)],
        duration_s=0.5, step_s=0.1)
    assert result.outputs[0] == "maintain_safe_target"
    assert result.outputs[-1] == "controlled_landing"


class SlowProvider:
    def available(self):
        return True

    def complete(self, *_args, **_kwargs):
        time.sleep(0.25)
        return '{"choice": 1}'


def test_slow_model_times_out_to_the_deterministic_baseline():
    gateway = DecisionGateway(SlowProvider(), mode_for={"traffic.x": "active"},
                              validators={"*": candidate_validator})
    started = time.monotonic()
    picked = select_candidate(
        gateway, "traffic.x", "pick", {},
        [{"id": "turn", "label": "turn"}, {"id": "extend", "label": "extend"}],
        "extend", "timeout", timeout_s=0.05)
    elapsed = time.monotonic() - started
    gateway.close()
    assert picked == "extend"
    assert elapsed < 0.2
    assert gateway.scopes["traffic.x"]["overtime"] == 1
