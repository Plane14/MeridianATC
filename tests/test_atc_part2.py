from __future__ import annotations

import ast
import json
import string
from pathlib import Path

import pytest

from meridianatc.atc import checklist_files, intents, intents_fr
from meridianatc.atc.session import Instruction
from meridianatc.atc.trainer import VoiceTrainer


@pytest.mark.parametrize("text", ["squawk 1200", "heading 300", "speed 200", "contact 121.500", "QNH 1000", "code seven thousand"])
def test_other_numeric_fields_are_not_altitudes(text):
    assert "altitude_ft" not in intents.IntentParser().parse(text).values


@pytest.mark.parametrize("text,value", [("maintain 3000 expect flight level 350", 3000), ("climb one two thousand", 12000), ("flight level three five zero", 35000), ("5000", 5000), ("squawk 1200 maintain 5000", 5000)])
def test_cleared_altitude_is_not_the_expected_level(text, value):
    assert intents.extract_altitude(text) == value


@pytest.mark.parametrize("text", ["N1234 ready to taxi", "heading 123", "heading one two three"])
def test_callsign_is_not_a_prefix_or_numeric_clearance(text):
    assert not intents.IntentParser(["N123"]).parse(text).callsign_matched


@pytest.mark.parametrize("text", ["N123 ready to taxi", "ready to taxi N123", "November one two three ready to taxi"])
def test_full_written_and_spoken_callsigns(text):
    parser = intents.IntentParser(["N123", "November one two three"])
    assert parser.parse(text).callsign_matched


@pytest.mark.parametrize("text", ["FL350", "fl 350", "F.L.350", "niveau de vol trois cinq zéro"])
def test_french_flight_level_abbreviations(text):
    assert intents_fr.extract_altitude_fr(text) == 35000


@pytest.mark.parametrize("suffix", ["L", "C", "R"])
def test_french_written_parallel_runways(suffix):
    assert intents_fr.extract_runway_fr("piste 27" + suffix) == "27" + suffix
    assert intents_fr.values_fr("maintenons avant la piste 27" + suffix)["hold_short"] == "27" + suffix


@pytest.mark.parametrize("language,word", [("es", "pista"), ("de", "piste"), ("it", "pista"), ("pt", "pista")])
@pytest.mark.parametrize("suffix", ["L", "C", "R"])
def test_local_written_parallel_runways(language, word, suffix):
    parser = intents.IntentParser()
    assert parser.parse(f"{word} 27{suffix}", language=language).values["runway"] == "27" + suffix


@pytest.mark.parametrize("expected,heard,wrong", [(121.610, 121.615, True), (121.600, 121.605, False), (121.610, 121.610, False), (121.610, "bad", True)])
def test_readback_frequency_matches_carriers_not_adjacent_channels(expected, heard, wrong):
    instruction = Instruction("handoff", "test", values={"frequency": expected})
    assert bool(instruction.mismatches({"frequency": heard})) is wrong


def test_empty_and_restricted_readback_requirements():
    instruction = Instruction("test", "", values={"heading": 90}, required_items=set())
    assert instruction.owed_items == []
    assert instruction.mismatches({}) == []
    instruction.required_items = None
    instruction.values["altitude_ft"] = 5000
    instruction.outstanding = {"altitude_ft"}
    assert instruction.owed_items == ["altitude_ft"]
    assert instruction.mismatches({"altitude_ft": 5000}) == []


def test_north_headings_match_and_invalid_numbers_do_not():
    instruction = Instruction("heading", "", values={"heading": 360})
    assert instruction.mismatches({"heading": 0}) == []
    assert instruction.mismatches({"heading": float("nan")}) == ["heading"]


@pytest.mark.parametrize("accept", [3, [3], {"bad": 1}])
def test_malformed_checklist_answers_are_reported_not_fatal(accept):
    problems = []
    item = checklist_files._item({"challenge": "Brakes", "accept": accept}, "test item", problems)
    assert item.accept == ("check",)
    assert problems


@pytest.mark.parametrize("raw", [{"var": "altitude_ft", "op": "near", "value": 100, "tolerance": -1}, {"var": "altitude_ft", "op": "min", "value": "bad"}, {"var": "altitude_ft", "op": "in", "value": 3}, {"var": "altitude_ft", "tolerance": float("nan")}])
def test_invalid_checks_are_disabled_with_diagnostics(raw):
    problems = []
    assert checklist_files._check(raw, "test item", problems) is None
    assert problems


def test_shipped_checklists_load_without_problems(tmp_path):
    for aircraft_class in checklist_files.CLASSES:
        book = checklist_files.load(aircraft_class, own=tmp_path)
        assert book.checklists
        assert not book.problems


def test_invalid_addon_root_does_not_break_discovery(tmp_path, monkeypatch):
    (tmp_path / "broken.json").write_text("[]", encoding="utf-8")
    monkeypatch.setattr(checklist_files, "TYPES", tmp_path)
    paths = checklist_files._add_ons("some aircraft")
    problems = []
    assert checklist_files._read(paths[0], problems) is None
    assert problems


def test_trainer_failed_atomic_replace_preserves_file_and_memory(tmp_path, monkeypatch):
    trainer = VoiceTrainer(tmp_path)
    trainer.add_keyterm("Alpha")
    original = (tmp_path / "keyterms.json").read_bytes()
    def fail(*args):
        raise PermissionError("locked")
    monkeypatch.setattr("meridianatc.atc.trainer.os.replace", fail)
    with pytest.raises(PermissionError):
        trainer.add_keyterm("Bravo")
    assert trainer.keyterms == ["Alpha"]
    assert (tmp_path / "keyterms.json").read_bytes() == original
    assert list(tmp_path.glob("*.tmp")) == []


def test_trainer_corrections_are_literal_and_persist(tmp_path):
    trainer = VoiceTrainer(tmp_path)
    trainer.correct("stair", r"stand \g<1>")
    assert VoiceTrainer(tmp_path).apply("stair") == r"stand \g<1>"
    trainer.add_keyterm("Alpha")
    trainer.remove_keyterm("Alpha")
    assert VoiceTrainer(tmp_path).keyterms == []


@pytest.mark.parametrize("data", [[], {"canAutofly": "false"}, {"canAutofly": True, "titleMatches": "a320"}, {"canAutofly": True, "titleMatches": ["a320"], "performance": {"climbRateFpm": -1}}])
def test_invalid_autofly_profiles_do_not_override(tmp_path, monkeypatch, data):
    from meridianatc.atc import autofly
    from meridianatc.atc.flying import AIRLINER
    (tmp_path / "profile.json").write_text(json.dumps(data), encoding="utf-8")
    monkeypatch.setattr(autofly, "DATA_DIR", tmp_path)
    monkeypatch.setattr(autofly, "_profiles", [])
    monkeypatch.setattr(autofly, "_loaded", False)
    assert autofly.override("A320", "a320", AIRLINER) is AIRLINER
    assert autofly._loaded


def test_phrasebook_keys_and_format_arguments_match_all_languages():
    from meridianatc.atc import phrasebook
    from meridianatc.atc.phrasebooks import BOOKS
    tree = ast.parse(Path(phrasebook.__file__).read_text(encoding="utf-8"))
    formatter = string.Formatter()
    for book in BOOKS.values():
        assert not set(phrasebook.PHRASE_KEYS) - book.say.keys()
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "_" and node.args and isinstance(node.args[0], ast.Constant):
                assert node.args[0].value in book.say
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute) or node.func.attr != "format":
                continue
            source = node.func.value
            if not isinstance(source, ast.Call) or not isinstance(source.func, ast.Attribute) or source.func.attr != "_" or not source.args or not isinstance(source.args[0], ast.Constant):
                continue
            if any(k.arg is None for k in node.keywords):
                continue
            fields = {field for _, field, _, _ in formatter.parse(book.say[source.args[0].value]) if field}
            assert fields <= {k.arg for k in node.keywords}, (book.language, source.args[0].value)


@pytest.mark.parametrize("content", ["not json", "[]", '{"roles": []}', '{"roles": {"stairs": {"standoff_m": "bad"}}}', '{"roles": {"stairs": {"facing": []}}}', '{"stairs_models": {"bad": {}}}'])
def test_invalid_service_layout_falls_back_with_warning(tmp_path, caplog, content):
    from meridianatc.atc.servicelayout import load_layout
    path = tmp_path / "layout.json"
    path.write_text(content, encoding="utf-8")
    assert load_layout(path) == {}
    assert "layout.json" in caplog.text


@pytest.mark.parametrize("content", ["not json", "[]", '{"profiles": []}', '{"profiles": {"bad": {"deboard_s": -1}}}'])
def test_invalid_turnaround_table_disables_services(tmp_path, caplog, content):
    from meridianatc.atc.handling import load_timelines, profile_for
    path = tmp_path / "turnaround.json"
    path.write_text(content, encoding="utf-8")
    table = load_timelines(path)
    profile = profile_for("A320", table=table)
    assert profile.turnaround == ()
    assert "turnaround.json" in caplog.text


def test_shipped_ground_service_tables_validate():
    from meridianatc.atc.handling import load_timelines
    from meridianatc.atc.servicelayout import load_layout
    assert load_timelines()["profiles"]["narrowbody"].turnaround
    assert load_layout()["stairs_models"]


@pytest.mark.parametrize("seen", [None, "bad", -1, float("nan"), float("inf")])
def test_invalid_visibility_cannot_allow_special_vfr(seen):
    from datetime import UTC, datetime
    from types import SimpleNamespace

    from meridianatc.atc.svfr import conditions
    result = conditions(SimpleNamespace(visibility_sm=seen, clouds=[]), "icao", 0, 0, datetime(2026, 1, 1, 12, tzinfo=UTC))
    assert not result.vmc
    assert not result.svfr


@pytest.mark.parametrize("height", [None, "bad", -1, float("nan")])
def test_invalid_reported_ceiling_is_not_clear_sky(height):
    from types import SimpleNamespace

    from meridianatc.atc.svfr import ceiling_ft
    assert ceiling_ft(SimpleNamespace(clouds=[("BKN", height)])) == 0


@pytest.mark.parametrize("content", ["[]", "not json", '{"bad": [1]}'])
def test_invalid_training_data_is_reported(tmp_path, caplog, content):
    (tmp_path / "corrections.json").write_text(content, encoding="utf-8")
    assert VoiceTrainer(tmp_path).corrections == {}
    assert "corrections.json" in caplog.text


@pytest.mark.parametrize("text,language", [("121 decimale six un zero cinq", "fr"), ("121 decimal seis uno cero cinco", "es"), ("121 Komma sechs eins null fünf", "de"), ("121 decimale sei uno zero cinque", "it"), ("121 decimal seis um zero cinco", "pt")])
def test_overlong_spoken_frequency_is_not_rounded_to_a_clearance(text, language):
    assert "frequency" not in intents.IntentParser().parse(text, language=language).values


def test_valid_autofly_profile_loads_once_across_threads(tmp_path, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor

    from meridianatc.atc import autofly
    from meridianatc.atc.flying import AIRLINER
    (tmp_path / "profile.json").write_text(json.dumps({"canAutofly": True, "titleMatches": ["test"], "performance": {"climbRateFpm": 1234}}), encoding="utf-8")
    monkeypatch.setattr(autofly, "DATA_DIR", tmp_path)
    monkeypatch.setattr(autofly, "_profiles", [])
    monkeypatch.setattr(autofly, "_loaded", False)
    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(lambda _: autofly.override("TEST", "", AIRLINER), range(12)))
    assert all(p.climb_fpm == 1234 for p in results)
    assert len(autofly._profiles) == 1


def test_empty_autofly_directory_uses_baseline(tmp_path, monkeypatch):
    from meridianatc.atc import autofly
    from meridianatc.atc.flying import AIRLINER
    monkeypatch.setattr(autofly, "DATA_DIR", tmp_path)
    monkeypatch.setattr(autofly, "_profiles", [])
    monkeypatch.setattr(autofly, "_loaded", False)
    assert autofly.override("TEST", "", AIRLINER) is AIRLINER
    assert autofly._loaded


@pytest.mark.parametrize("data", [[], {"match": "bad", "no_check": ["battery_master"]}, {"match": ["bad"], "no_check": [{}]}])
def test_invalid_addon_cannot_change_radio_power_policy(tmp_path, monkeypatch, data):
    from meridianatc.atc import radio_power
    path = tmp_path / "addon.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    monkeypatch.setattr(checklist_files, "TYPES", tmp_path)
    radio_power.own_electrics.cache_clear()
    assert not radio_power.own_electrics("bad fixture addon")
    radio_power.own_electrics.cache_clear()


def test_checklist_file_removed_during_reload_is_harmless(tmp_path, monkeypatch):
    path = tmp_path / "gone.json"
    monkeypatch.setattr(checklist_files, "_files", lambda *args: [path])
    book = checklist_files.load("airliner", own=tmp_path)
    assert book.checklists == {}


def test_north_heading_extraction():
    assert intents.extract_heading("heading zero zero zero") == 360
    assert intents.extract_heading("heading three six zero") == 360
    assert intents.extract_heading("heading 361") is None
