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


# ------------------------------------------------------------------
# the transponder watch
# ------------------------------------------------------------------


def _transponder_fixture(*, expected="4621", observed="1200",
                         assigned=True, established=True):
    from unittest.mock import Mock

    from meridianatc.atc.controller import ControllerBrain
    from meridianatc.atc.phraseology import Aircraft, Phraseology
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.atc.watch import TransponderWatch
    from meridianatc.navdata.db import Station
    from meridianatc.sim.state import AircraftState

    brain = ControllerBrain(Mock(), Phraseology())
    brain.staffed = lambda station: True
    watch = TransponderWatch(Mock(), brain)
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.squawk = expected
    session.squawk_assigned = assigned
    station = Station("TEST", "APP", 124.5, "Approach")
    if established:
        session.establish(station)
    state = AircraftState(transponder_code=observed)
    return watch, session, station, state


def test_transponder_watch_asks_about_a_wrong_code():
    watch, session, station, state = _transponder_fixture()
    assert watch.look(session, station, state) is None  # settling
    watch._since -= watch.SETTLE_S
    reply = watch.look(session, station, state)
    assert reply is not None
    assert "squawk" in reply.text.lower()
    assert "4621" in reply.text.replace(" ", "") or \
        "four six two one" in reply.text.lower()


def test_transponder_watch_leaves_matching_emergency_and_unset_codes():
    watch, session, station, state = _transponder_fixture(observed="4621")
    assert watch.look(session, station, state) is None
    for code in ("7500", "7600", "7700"):
        w, s, st, _ = _transponder_fixture(observed=code)
        state = type("S", (), {"transponder_code": code})()
        w.look(s, st, state)
        w._since -= w.SETTLE_S
        assert w.look(s, st, state) is None
    w, s, st, state = _transponder_fixture(assigned=False)
    assert w.look(s, st, state) is None


def test_transponder_watch_waits_for_the_pilot_to_set_it():
    watch, session, station, state = _transponder_fixture()
    # Just issued: within the settle window, nothing is said.
    assert watch.look(session, station, state) is None
    # Past it, asked; right after, not asked again.
    watch._since -= watch.SETTLE_S + 1.0
    assert watch.look(session, station, state) is not None
    assert watch.look(session, station, state) is None


def test_transponder_watch_ignores_non_code_observations():
    for observed in ("", "12", "12000", "ABCD"):
        w, s, st, state = _transponder_fixture(observed=observed)
        w.look(s, st, state)
        w._since -= w.SETTLE_S
        assert w.look(s, st, state) is None


# ------------------------------------------------------------------
# asking for the pressure setting
# ------------------------------------------------------------------


@pytest.mark.parametrize("text", [
    "N123 request altimeter",
    "N123 what is the QNH",
    "N123 say the altimeter",
    "N123 requesting the QNH",
])
def test_altimeter_request_parses(text):
    parsed = intents.IntentParser(["N123"]).parse(text, "en")
    assert parsed.intent is intents.Intent.REQUEST_ALTIMETER


@pytest.mark.parametrize("language,text", [
    ("fr", "N123 demande le qnh"),
    ("es", "N123 solicito el qnh"),
    ("de", "N123 erbitte das qnh"),
    ("it", "N123 chiedo il qnh"),
    ("pt", "N123 peco o qnh"),
])
def test_altimeter_request_parses_local(language, text):
    parsed = intents.IntentParser(["N123"]).parse(text, language)
    assert parsed.intent is intents.Intent.REQUEST_ALTIMETER


def test_altimeter_request_is_answered_by_any_station():
    from unittest.mock import Mock

    from meridianatc.atc.controller import ControllerBrain
    from meridianatc.atc.phraseology import Aircraft, Phraseology, Weather
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.navdata.db import Station
    from meridianatc.sim.state import AircraftState

    brain = ControllerBrain(Mock(), Phraseology())
    brain.staffed = lambda station: True
    session = FlightSession(Aircraft(parse_callsign("N123")))
    weather = Weather(qnh_hpa=1020.0)
    parsed = intents.IntentParser(["N123"]).parse(
        "N123 request altimeter", "en")
    for position in ("DEL", "GND", "TWR", "APP", "DEP", "CTR", "INFO"):
        station = Station("TEST", position, 120.0, position)
        reply = brain.respond(parsed, session, station, weather,
                              AircraftState())
        assert reply.speak, position
        assert "one zero two zero" in reply.text, position


# ------------------------------------------------------------------
# stands by aircraft size class
# ------------------------------------------------------------------


def _layout_with_stands():
    """A ground layout with a light stand and an airliner stand."""
    from meridianatc.atc.ground import GroundLayout, Stand

    layout = GroundLayout.__new__(GroundLayout)
    layout.stands = [
        Stand(index=1, x=0.0, z=0.0, heading=0.0, radius=6.0, kind=8,
              number=1),
        Stand(index=2, x=100.0, z=0.0, heading=0.0, radius=20.0, kind=8,
              number=2),
    ]
    # ``stands_for`` wants the stand nodes on the taxi graph.
    layout.edges = {("k", 1): [], ("k", 2): []}
    return layout


def test_stand_size_class_filters_small_stands():
    layout = _layout_with_stands()
    # A business jet (9.0 m) cannot take the 6 m stand.
    big = [s.index for s in layout.stands_for(
        airliner=False, min_radius_m=9.0)]
    assert big == [2]
    # Without the class figure the old behaviour is unchanged.
    light = [s.index for s in layout.stands_for(airliner=False)]
    assert light == [1, 2]


def test_stand_radius_per_class():
    from meridianatc.atc import aircraft_class

    assert aircraft_class.stand_radius_m(aircraft_class.PISTON_SINGLE) < \
        aircraft_class.stand_radius_m(aircraft_class.BUSINESS_JET) < \
        aircraft_class.stand_radius_m(aircraft_class.AIRLINER)
    # Unknown classes get the light figure, not silence.
    assert aircraft_class.stand_radius_m("spaceship") == \
        aircraft_class.stand_radius_m(aircraft_class.PISTON_SINGLE)


def test_free_stands_honours_class_radius():
    layout = _layout_with_stands()
    free = [s.index for s in layout.free_stands(
        airliner=False, occupied=[], taken=set(), min_radius_m=9.0)]
    assert free == [2]


# ------------------------------------------------------------------
# off the taxi route, and one deviation too many
# ------------------------------------------------------------------


def _taxi_watch_fixture(path=None):
    from unittest.mock import Mock

    from meridianatc.atc.controller import ControllerBrain
    from meridianatc.atc.phraseology import Aircraft, Phraseology
    from meridianatc.atc.session import FlightSession, Phase
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.atc.watch import TaxiWatch
    from meridianatc.navdata.db import Station
    from meridianatc.sim.state import AircraftState

    brain = ControllerBrain(Mock(), Phraseology())
    brain.staffed = lambda station: True
    watch = TaxiWatch(Mock(), brain)
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.phase = Phase.TAXI_OUT
    session.taxi_path = path or [(40.0, -75.0), (40.01, -75.0)]
    session.establish(Station("TEST", "GND", 121.7, "Ground"))
    station = Station("TEST", "GND", 121.7, "Ground")
    return watch, session, station, AircraftState


def test_taxi_watch_says_nothing_on_the_route():
    watch, session, station, state_cls = _taxi_watch_fixture()
    state = state_cls(latitude=40.005, longitude=-75.0,
                      ground_speed_kt=15.0, on_ground=True)
    watch._off_since = None
    assert watch.look(session, station, state) is None


def test_taxi_watch_offers_help_off_the_route():
    watch, session, station, state_cls = _taxi_watch_fixture()
    # 2 nm north of the line, still moving: well off it.
    state = state_cls(latitude=40.04, longitude=-75.0,
                      ground_speed_kt=15.0, on_ground=True)
    assert watch.look(session, station, state) is None  # settle
    watch._off_since -= watch.SETTLE_S + 1.0
    reply = watch.look(session, station, state)
    assert reply is not None
    assert "assistance" in reply.text.lower()
    assert session.assist_offered == "taxi"
    # Offered once: no nagging while the crew thinks.
    watch._off_since = -watch.SETTLE_S
    assert watch.look(session, station, state) is None


def test_taxi_watch_ignores_a_stopped_aeroplane():
    watch, session, station, state_cls = _taxi_watch_fixture()
    state = state_cls(latitude=40.04, longitude=-75.0,
                      ground_speed_kt=0.0, on_ground=True)
    watch._off_since = -watch.SETTLE_S
    assert watch.look(session, station, state) is None


def test_deviation_watch_escalates_to_an_offer():
    from unittest.mock import Mock

    from meridianatc.atc.controller import ControllerBrain
    from meridianatc.atc.phraseology import Aircraft, Phraseology
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.atc.watch import DeviationAssistWatch
    from meridianatc.navdata.db import Station
    from meridianatc.sim.state import AircraftState

    brain = ControllerBrain(Mock(), Phraseology())
    brain.staffed = lambda station: True
    watch = DeviationAssistWatch(Mock(), brain)
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.assigned_altitude_ft = 5000.0
    station = Station("TEST", "APP", 124.5, "Approach")
    session.establish(station)
    # 1000 ft low, airborne.
    state = AircraftState(altitude_ft=4000.0, on_ground=False,
                          latitude=40.0, longitude=-75.0)

    first = watch.look(session, station, state)
    assert first is None          # still inside the settle window
    watch._seen["altitude"][0] -= watch.SETTLE_S + 1.0
    first = watch.look(session, station, state)
    assert first is not None and "verify" in first.text.lower()
    assert not session.assistance_offered()
    # The same call a minute later, then the offer.
    watch._seen["altitude"][1] -= watch.REMIND_S + 1.0
    second = watch.look(session, station, state)
    assert second is not None and "verify" in second.text.lower()
    watch._seen["altitude"][1] -= watch.REMIND_S + 1.0
    third = watch.look(session, station, state)
    assert third is not None
    assert "intentions" in third.text.lower()
    assert session.assist_offered == "flight"


def test_deviation_watch_clears_on_the_ground():
    from unittest.mock import Mock

    from meridianatc.atc.controller import ControllerBrain
    from meridianatc.atc.phraseology import Aircraft, Phraseology
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.atc.watch import DeviationAssistWatch
    from meridianatc.navdata.db import Station
    from meridianatc.sim.state import AircraftState

    brain = ControllerBrain(Mock(), Phraseology())
    brain.staffed = lambda station: True
    watch = DeviationAssistWatch(Mock(), brain)
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.assigned_altitude_ft = 5000.0
    session.establish(Station("TEST", "TWR", 118.1, "Tower"))
    state = AircraftState(altitude_ft=100.0, on_ground=True)
    assert watch.look(session, Station("TEST", "TWR", 118.1, "Tower"),
                      state) is None


def test_assistance_offer_lifecycle():
    import time

    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign

    session = FlightSession(Aircraft(parse_callsign("N123")))
    now = time.monotonic()
    assert session.offer_assistance("taxi", now)
    assert not session.offer_assistance("taxi", now + 10)
    assert session.assistance_offered("taxi")
    assert not session.assistance_offered("flight")
    assert session.answer_assistance() == "taxi"
    assert not session.assistance_offered()


class TestTrafficChatter:
    """Crews answer calls addressed to them, and call the pilot on CTAF."""

    def _injector(self):
        from meridianatc.atc.traffic.x_pilots_talking import PilotsTalking
        return PilotsTalking()

    def _flight(self, spoken="delta four five", **kw):
        from unittest.mock import MagicMock
        f = MagicMock()
        f.done = False
        f.phase = kw.pop("phase", "final")
        f.runway = kw.pop("runway", "24")
        f.arriving = kw.pop("arriving", True)
        f.destination = kw.pop("destination", "Frankfurt")
        f.language = "en"
        f.written = "DAL45"
        f.distance_nm = 6.0
        ac = MagicMock()
        cs = MagicMock()
        cs.spoken_full = spoken
        cs.spoken_short = spoken
        ac.for_language.return_value = cs
        f.aircraft = ac
        return f

    def _session(self):
        from unittest.mock import MagicMock
        s = MagicMock()
        s.aircraft.for_language.return_value.spoken_short = \
            "november one two three"
        return s

    def _parsed(self, text, matched=False):
        from unittest.mock import MagicMock

        from meridianatc.atc.intents import normalize
        return MagicMock(text=text, normalized=normalize(text),
                         callsign_matched=matched)

    def test_flight_called_matches_spoken_callsign(self):
        inj = self._injector()
        f = self._flight()
        inj.flights = [f]
        assert inj.flight_called("delta four five say position") is f

    def test_flight_called_ignores_partial_and_done(self):
        inj = self._injector()
        f = self._flight()
        inj.flights = [f]
        assert inj.flight_called("delta tower hello") is None
        f.done = True
        assert inj.flight_called("delta four five say position") is None

    def test_answer_position_report(self):
        from unittest.mock import MagicMock
        inj = self._injector()
        f = self._flight()
        inj.flights = [f]
        inj._say = MagicMock()
        inj.field = MagicMock()
        ok = inj.answer_the_user(
            self._session(),
            self._parsed("delta four five say your position"), "CTAF")
        assert ok
        make = inj._say.call_args.kwargs["make"]
        assert "on final runway 24" in make(None, None, None)

    def test_answer_weather(self):
        from unittest.mock import MagicMock
        inj = self._injector()
        f = self._flight()
        inj.flights = [f]
        inj._say = MagicMock()
        inj.field = MagicMock()
        inj.field.weather.wind_dir = 240
        inj.field.weather.wind_kt = 12
        ok = inj.answer_the_user(
            self._session(),
            self._parsed("delta four five how is the wind"), "CTAF")
        assert ok
        make = inj._say.call_args.kwargs["make"]
        assert "wind 240" in make(None, None, None)

    def test_own_callsign_goes_to_atc(self):
        inj = self._injector()
        f = self._flight()
        inj.flights = [f]
        assert not inj.answer_the_user(
            self._session(),
            self._parsed("delta four five climb", matched=True), "CTAF")

    def test_mayday_only_on_ctaf(self):
        from unittest.mock import MagicMock
        inj = self._injector()
        f = self._flight(phase="takeoff")
        inj.flights = [f]
        inj._say = MagicMock()
        inj._user_called_at = 0.0
        assert not inj.maybe_call_the_user(self._session(), "TWR", 9999.0)
        assert inj.maybe_call_the_user(self._session(), "CTAF", 9999.0)
        assert "intentions" in inj._say.call_args.kwargs["make"](
            None, None, None)
        # throttled: no second call inside the window
        assert not inj.maybe_call_the_user(self._session(), "CTAF", 9999.5)


class TestFollowMeCar:
    """The follow-me car's spot on the cleared taxi route."""

    def _path(self):
        # ~500 m north then ~500 m east, from the origin
        return [(0.0, 0.0), (0.0045, 0.0), (0.0045, 0.0072)]

    def test_car_leads_along_route(self):
        from meridianatc.atc.taxi import car_ahead
        lat, lon, hdg = car_ahead(0.0, 0.0, self._path(), 35.0)
        assert hdg < 1.0 and 0.0002 < lat < 0.0004 and lon == 0.0

    def test_car_turns_the_corner(self):
        from meridianatc.atc.taxi import car_ahead
        _, _, hdg = car_ahead(0.0044, 0.0, self._path(), 80.0)
        assert hdg > 45.0

    def test_car_pulls_off_at_the_limit(self):
        from meridianatc.atc.taxi import car_ahead
        assert car_ahead(0.0045, 0.0070, self._path(), 35.0,
                         done_m=30.0) is None

    def test_no_car_off_route_or_no_route(self):
        from meridianatc.atc.taxi import car_ahead
        assert car_ahead(0.02, 0.0, self._path(), 35.0) is None
        assert car_ahead(0.0, 0.0, [(0.0, 0.0)], 35.0) is None
        assert car_ahead(0.0, 0.0, [], 35.0) is None

    def test_car_clamps_at_route_end(self):
        from meridianatc.atc.taxi import car_ahead
        lat, lon, hdg = car_ahead(0.0045, 0.0068, self._path(), 200.0,
                                  done_m=5.0)
        end = self._path()[-1]
        assert abs(lat - end[0]) < 1e-9 and abs(lon - end[1]) < 1e-9
        assert hdg > 45.0

    def test_followme_wire(self):
        import json

        from meridianatc.integrations.xptraffic import XPlaneTraffic
        xp = XPlaneTraffic()
        assert xp.followme(True, 48.0, 11.0, 270.0) is False
        sent = []

        class Sock:
            def sendto(self, w, e):
                sent.append(w)
        xp._socket = Sock()
        xp._endpoint = ("127.0.0.1", 49010)
        assert xp.followme(True, 48.0, 11.0, 270.0)
        msg = json.loads(sent[0])
        assert msg["t"] == "followme" and msg["on"] == 1
        assert msg["lat"] == 48.0 and msg["hdg"] == 270.0
