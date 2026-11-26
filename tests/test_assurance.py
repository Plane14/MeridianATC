from meridianatc.atc.assurance import choose_fallback, validate_candidate_state


def test_fixed_wing_normal_fallback_maintains_safe_target():
    assert choose_fallback(airborne=True).name == "maintain_safe_target"


def test_fixed_wing_engine_failure_lands():
    assert choose_fallback(airborne=True, engine_failed=True).name == "controlled_landing"


def test_rotorcraft_rotor_failure_autorotates():
    assert choose_fallback(airborne=True, rotorcraft=True,
                           rotor_failed=True).name == "autorotate"


def test_ground_fallback_stops_first():
    assert choose_fallback(airborne=False).name == "stop"


def test_candidate_validator_vetoes_conflicts():
    assert validate_candidate_state({"runway_occupied": True}) == "runway protected"
    assert validate_candidate_state({"separation_nm": 2.0,
                                     "min_separation_nm": 3.0}) == "separation"
    assert validate_candidate_state({"terrain_clearance_ft": 100,
                                     "min_terrain_ft": 500}) == "terrain clearance"
    assert validate_candidate_state({"speed_kt": 150, "max_speed_kt": 120}) == "speed envelope"
    assert validate_candidate_state({"speed_kt": 100, "max_speed_kt": 120}) == ""
