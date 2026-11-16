"""Behaviour audit, second pass: the VFR code and the settings applied live."""
from types import SimpleNamespace

from meridianatc import config as cfg
from meridianatc.atc import profile as flight_profile
from meridianatc.atc.crew import CrewDirector
from meridianatc.atc.language import phraseology_for
from meridianatc.atc.phraseology import Aircraft
from meridianatc.atc.speech import parse_callsign


def _crew(ident, squawk, profile=None):
    crew = CrewDirector.__new__(CrewDirector)
    crew.here = lambda: SimpleNamespace(ident=ident)
    crew._session = SimpleNamespace(
        ifr=False, squawk=squawk, clearance_issued=False,
        profile=profile or flight_profile.DEFAULT)
    return crew


def test_the_checklist_wants_the_vfr_code_the_controller_says():
    """Vancouver's controller said 1200 and the checklist wanted 7000:
    only FAA *wording* was taken to mean 1200."""
    assert _crew("CYVR", "1200")._reference("$squawk") == "1200"
    assert _crew("YSSY", "1200")._reference("$squawk") == "1200"
    assert _crew("RJTT", "1400")._reference("$squawk") == "1400"
    assert _crew("EGLL", "7000")._reference("$squawk") == "7000"
    assert _crew("KJFK", "1200")._reference("$squawk") == "1200"
    # A discrete code is still the one to set.
    assert _crew("CYVR", "4521")._reference("$squawk") == "4521"


def test_every_language_ends_radar_service_on_the_fields_vfr_code():
    """"Squawk VFR" set 1200 on the session at a Canadian field while
    the French and the phrasebook controllers said 7000."""
    for language in ("fr", "es", "de", "it", "pt"):
        p = phraseology_for(language, "CYUL")
        a = Aircraft(parse_callsign("N123"))
        a.language = language
        assert p.vfr_squawk == "1200"
        there = phraseology_for(language, "LFPG")
        for text, other in (
                (p.radar_service_terminated(a),
                 there.radar_service_terminated(a)),
                (p.ifr_cancelled(a), there.ifr_cancelled(a))):
            assert text != other, language


def test_tone_and_gsx_settings_apply_without_a_restart():
    from meridianatc.web import server

    config = cfg.Config()
    forgot = []
    engine = SimpleNamespace(
        config=config,
        synth=SimpleNamespace(tone=None, prosody=None, imperfections=None,
                              forget=lambda: forgot.append(True)),
        gsx=SimpleNamespace(enabled=config.gsx.enabled,
                            control=config.gsx.control))
    was = config.delivery.tone.enabled
    changed = server._apply_settings(engine, {
        "delivery.tone.enabled": not was,
        "gsx.enabled": not config.gsx.enabled,
        "gsx.control": not config.gsx.control})
    assert {"delivery.tone.enabled", "gsx.enabled", "gsx.control"} <= changed
    assert engine.synth.tone.enabled == (not was) and forgot
    assert engine.gsx.enabled == config.gsx.enabled
    assert engine.gsx.control == config.gsx.control


def test_switching_traffic_off_is_said_to_need_a_restart():
    assert cfg.SETTINGS["traffic.enabled"].restart
