"""Complete runtime-parameter trace regressions."""

from __future__ import annotations

import json
from types import SimpleNamespace
from unittest.mock import patch

from meridianatc.atc.autonomous._shared import AutonomousState
from meridianatc.atc.phraseology import Aircraft
from meridianatc.atc.session import FlightSession
from meridianatc.atc.speech import parse_callsign
from meridianatc.logs import _plain
from meridianatc.sim.state import AircraftState
from meridianatc.state_trace import ParameterTracer, runtime_snapshot


def _session() -> FlightSession:
    return FlightSession(Aircraft(parse_callsign("ECX3S")))


def test_the_snapshot_covers_aircraft_autonomous_session_and_config():
    aircraft = AircraftState(
        latitude=43.1, eng_torque_nm=1784.0, eng2_torque_nm=1779.0,
        unavailable=("eng_torque_pct",))
    autonomous = AutonomousState(assigned_altitude_ft=17000.0,
                                 direct_fix="FORNO")
    session = _session()
    session.assigned_altitude_ft = 17000.0
    session.squawk = "7260"
    config = SimpleNamespace(api_key="must-not-appear", notes="ok")

    snap = runtime_snapshot(aircraft, autonomous, session, config)
    assert snap["aircraft"]["eng_torque_nm"] == 1784.0
    assert snap["aircraft"]["eng2_torque_nm"] == 1779.0
    assert snap["aircraft"]["unavailable"] == ["eng_torque_pct"]
    assert snap["autonomous"]["assigned_altitude_ft"] == 17000.0
    assert snap["autonomous"]["direct_fix"] == "FORNO"
    assert snap["session"]["assigned_altitude_ft"] == 17000.0
    assert snap["session"]["squawk"] == "7260"
    assert snap["config"] == "<SimpleNamespace>"
    # The complete tree is valid structured-log JSON after the normal
    # formatter's object conversion, object fields and all.
    json.dumps(_plain(snap), ensure_ascii=False)


def test_the_tracer_logs_every_change_after_one_full_snapshot():
    tracer = ParameterTracer(clock=lambda: 100.0)
    aircraft = AircraftState(eng_torque_nm=1700.0, throttle_pct=85.0)
    snap = runtime_snapshot(aircraft, AutonomousState(), _session())
    full, changes = tracer.observe(snap, 100.0)
    assert full and not changes

    aircraft.eng_torque_nm = 1712.5
    aircraft.throttle_pct = 84.0
    full, changes = tracer.observe(
        runtime_snapshot(aircraft, AutonomousState(), _session()), 105.0)
    assert not full
    assert changes["aircraft.eng_torque_nm"] == {"from": 1700.0, "to": 1712.5}
    assert changes["aircraft.throttle_pct"] == {"from": 85.0, "to": 84.0}
    assert "aircraft.latitude" not in changes


def test_the_engine_trace_writes_the_full_snapshot_then_state_changes():
    import meridianatc.engine.x_situation_loop as loop_module
    from meridianatc.engine.x_situation_loop import _Situation_loop

    owner = SimpleNamespace(
        config=None,
        autonomous_flight=SimpleNamespace(state=AutonomousState()),
        source=SimpleNamespace(_values={"sim/test/raw_value": 12.5}),
        _parameter_trace_at=-10.0,
        PARAMETER_TRACE_S=5.0)
    state = AircraftState(latitude=43.0, longitude=-8.0, on_ground=False,
                          eng_torque_nm=1700.0)
    with patch.object(loop_module.log, "info") as info:
        _Situation_loop._trace_state_parameters(owner, state, _session())
        first = info.call_args.kwargs["extra"]
        assert "state_snapshot" in first
        assert first["state_snapshot"]["aircraft"]["eng_torque_nm"] == 1700.0
        assert first["state_snapshot"]["sim_values"]["sim/test/raw_value"] == 12.5

        owner._parameter_trace_at = -10.0
        state.eng_torque_nm = 1710.0
        owner.source._values["sim/test/raw_value"] = 13.0
        _Situation_loop._trace_state_parameters(owner, state, _session())
        second = info.call_args.kwargs["extra"]
        assert "state_snapshot" not in second
        assert second["state_changes"]["aircraft.eng_torque_nm"] == {
            "from": 1700.0, "to": 1710.0}
        assert second["state_changes"]["sim_values.sim/test/raw_value"] == {
            "from": 12.5, "to": 13.0}


def test_config_secrets_are_traced_as_redacted_not_written():
    from meridianatc.config import Config
    config = Config()
    config.ai.api_key = "sk-test-secret-value"
    snap = runtime_snapshot(AircraftState(), AutonomousState(), _session(), config)
    assert snap["config"]["ai"]["api_key"] == "<redacted>"
    assert "sk-test-secret-value" not in json.dumps(_plain(snap))
