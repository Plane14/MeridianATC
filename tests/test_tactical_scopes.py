from meridianatc.atc.decisioning import candidate_validator, select_candidate
from meridianatc.llm.gateway import DecisionGateway


class Provider:
    def __init__(self, raw):
        self.raw = raw
        self.calls = 0

    def available(self):
        return True

    def complete(self, *_args, **_kwargs):
        self.calls += 1
        return self.raw


def traffic_request_candidates():
    return [
        {"id": "turn_base", "label": "turn base",
         "runway_occupied": True, "separation_nm": 0.0,
         "min_separation_nm": 0.5},
        {"id": "extend_downwind", "label": "extend downwind"},
    ]


def test_active_model_cannot_turn_base_into_a_protected_runway():
    gateway = DecisionGateway(Provider('{"choice": 1}'),
                              mode_for={"traffic.circuit_spacing": "active"},
                              validators={"*": candidate_validator})
    picked = select_candidate(
        gateway, "traffic.circuit_spacing", "turn or extend?", {},
        traffic_request_candidates(), "extend_downwind", "k", timeout_s=0.2)
    gateway.close()
    assert picked == "extend_downwind"
    assert gateway.scopes["traffic.circuit_spacing"]["vetoed"] == 1


def test_shadow_logs_but_returns_the_deterministic_baseline():
    gateway = DecisionGateway(Provider('{"choice": 1}'),
                              mode_for={"traffic.circuit_spacing": "shadow"})
    candidates = traffic_request_candidates()
    candidates[0]["runway_occupied"] = False
    candidates[0]["separation_nm"] = 99.0
    picked = select_candidate(
        gateway, "traffic.circuit_spacing", "turn or extend?", {},
        candidates, "extend_downwind", "k", timeout_s=0.2)
    gateway.close()
    assert picked == "extend_downwind"
    assert gateway.scopes["traffic.circuit_spacing"]["proposed"] == 1


def test_active_valid_traffic_choice_is_returned():
    gateway = DecisionGateway(Provider('{"choice": 1}'),
                              mode_for={"traffic.circuit_spacing": "active"})
    candidates = traffic_request_candidates()
    candidates[0]["runway_occupied"] = False
    candidates[0]["separation_nm"] = 99.0
    picked = select_candidate(
        gateway, "traffic.circuit_spacing", "turn or extend?", {},
        candidates, "extend_downwind", "k", timeout_s=0.2)
    gateway.close()
    assert picked == "turn_base"


def test_off_scope_never_calls_the_provider():
    provider = Provider('{"choice": 1}')
    gateway = DecisionGateway(provider, mode_for={"crew.notice_priority": "off"})
    picked = select_candidate(
        gateway, "crew.notice_priority", "mention?", {},
        [{"id": "all_due", "label": "all"}, {"id": "fuel", "label": "fuel"}],
        "all_due", "crew", timeout_s=0.2)
    gateway.close()
    assert picked == "all_due"
    assert provider.calls == 0


def test_active_fo_weather_choice_uses_the_validated_opposite_side():
    from types import SimpleNamespace

    from meridianatc.atc.autonomous._shared import AutonomousPhase
    from meridianatc.atc.autonomous.x_weather import _Weather
    from meridianatc.weather.enroute import AvoidancePlan

    class Provider:
        def available(self):
            return True

        def complete(self, *_args, **_kwargs):
            return '{"choice": 2}'

    weather = _Weather()
    weather._is_helicopter = False
    weather._heli = None
    weather._weather_deviation_requested_at = 80.0
    weather.decision_gateway = DecisionGateway(
        Provider(), mode_for={"fo.weather_deviation": "active"})
    weather.session = SimpleNamespace(deviating="", weather_deviation_at=0.0)
    weather.state = SimpleNamespace(phase=AutonomousPhase.CRUISE)
    weather._enroute_weather_plan = lambda state: AvoidancePlan(
        True, "left", 30, 15, nearest_nm=5.0, source="test")
    weather._enroute_weather_cells = lambda state: []
    weather._weather_source_label = lambda plan: "test"
    weather._fo_say = lambda *_args, **_kwargs: True
    weather._emit = lambda *_args, **_kwargs: None
    state = SimpleNamespace(on_ground=False, track_true=90.0, heading_true=90.0,
                            altitude_ft=8000.0, latitude=50.0, longitude=4.0)

    weather._watch_enroute_convection(state, 100.0)
    weather.decision_gateway.close()

    assert weather.session.weather_deviation_side == "right"
    assert weather._urgent_deviation_side == "right"
