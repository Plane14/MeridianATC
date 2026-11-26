"""Propeller rotation and the takeoff roll's standing rudder."""

from types import SimpleNamespace

from meridianatc.atc.autonomous.x_the_checklist_answered import (
    _The_checklist_answered,
)
from meridianatc.sim.msfsconfig import parse_rotation, propeller_rotation


def _balance(**state):
    flight = _The_checklist_answered.__new__(_The_checklist_answered)
    flight._aircraft_class = state.pop("aircraft_class", "")
    return flight._prop_rotation_balance(SimpleNamespace(**state))


def test_opposite_rotating_twin_props_cancel_their_p_factor():
    """FAA-H-8083-3C: counter-rotating light twins have no critical engine
    because either thrust line is the same distance from the centreline."""
    assert _balance(engine_type=0, engine_count=2,
                    prop_rotation_dirs=(1.0, -1.0)) == 0.0


def test_same_direction_and_reverse_rotation_keep_their_correct_foot():
    """Clockwise propellers want right rudder (+); anticlockwise want left."""
    assert _balance(engine_type=0, engine_count=2,
                    prop_rotation_dirs=(1.0, 1.0)) == 1.0
    assert _balance(engine_type=0, engine_count=2,
                    prop_rotation_dirs=(-1.0, -1.0)) == -1.0


def test_jets_get_no_propeller_right_foot_at_all():
    """The old feedforward gave every airliner a single-engine prop's right
    rudder even though none of the turning moments exist."""
    assert _balance(engine_type=1, engine_count=2,
                    prop_rotation_dirs=(1.0, 1.0)) == 0.0


def test_missing_engine_slots_default_clockwise_but_do_not_invent_balance():
    """Both simulators document clockwise as the default rotation."""
    assert _balance(engine_type=0, engine_count=2,
                    prop_rotation_dirs=(-1.0,)) == 0.0
    assert _balance(engine_type=0, engine_count=1,
                    prop_rotation_dirs=()) == 1.0


def test_msfs_engines_cfg_rotation_is_read_only_from_the_propeller_section():
    text = """
[ENGINE]
rotation = 99, 99
[PROPELLER]
rotation = 1, -1 ; left clockwise, right anti-clockwise
"""
    assert parse_rotation(text, 2) == (1.0, -1.0)
    assert parse_rotation("[PROPELLER]\nrotation = -1\n", 2) == (-1.0, 1.0)
    assert parse_rotation("[PROPELLER]\n", 2) == ()


def test_msfs_aircraft_title_is_matched_to_its_package_engines_cfg(tmp_path):
    package = (tmp_path / "Packages" / "Community" / "test-twin" /
               "SimObjects" / "Airplanes" / "TestTwin")
    package.mkdir(parents=True)
    (package / "aircraft.cfg").write_text(
        "[fltsim.0]\ntitle = Test Twin Analog\n", encoding="utf-8")
    (package / "engines.cfg").write_text(
        "[PROPELLER]\nrotation = 1, -1\n", encoding="utf-8")
    assert propeller_rotation("Test Twin Analog", 2,
                              folders=[tmp_path / "Packages"]) == (1.0, -1.0)


def test_msfs2024_modular_preset_reads_preset_then_common_rotation(tmp_path):
    root = (tmp_path / "Packages" / "Community" / "test-modular" /
            "SimObjects" / "Airplanes" / "TestTwin")
    preset = root / "Presets" / "standard" / "config"
    preset.mkdir(parents=True)
    newline = chr(10)
    (preset / "aircraft.cfg").write_text(
        newline.join(["[fltsim.0]", "title = Modular Test Twin", ""]),
        encoding="utf-8")
    (preset / "engines.cfg").write_text(
        newline.join(["[PROPELLER]", "rotation = 1, -1", ""]),
        encoding="utf-8")
    assert propeller_rotation("Modular Test Twin", 2,
                              folders=[tmp_path / "Packages"]) == (1.0, -1.0)

    # A modular preset that does not override rotation inherits Common's.
    (preset / "engines.cfg").write_text("[PROPELLER]" + newline,
                                        encoding="utf-8")
    common = root / "Common" / "config"
    common.mkdir(parents=True)
    (common / "engines.cfg").write_text(
        newline.join(["[PROPELLER]", "rotation = -1, 1", ""]),
        encoding="utf-8")
    assert propeller_rotation("Modular Test Twin", 2,
                              folders=[tmp_path / "Packages"]) == (-1.0, 1.0)


def test_msfs2024_modern_props_use_physical_reverse_switches_not_rotation():
    newline = chr(10)
    text = newline.join([
        "[GENERALENGINEDATA]",
        "Engine.0 = 10, -8.4, 0.5",
        "Engine.1 = 10, 8.4, 0.5",
        "[PROPELLER]",
        "rotation = 1, 1 ; ignored by the modern physics",
        "prop_mod_version = 2",
        "prop_mod_reverse_left = 0",
        "prop_mod_reverse_right = 1",
        "",
    ])
    assert parse_rotation(text, 2) == (1.0, -1.0)


def test_msfs_legacy_rotation_still_rules_without_the_modern_model():
    newline = chr(10)
    text = newline.join([
        "[GENERALENGINEDATA]",
        "Engine.0 = 10, -8.4, 0.5",
        "Engine.1 = 10, 8.4, 0.5",
        "[PROPELLER]",
        "rotation = 1, 1",
        "prop_mod_reverse_left = 0",
        "prop_mod_reverse_right = 1",
        "",
    ])
    assert parse_rotation(text, 2) == (1.0, 1.0)
