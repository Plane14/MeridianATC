"""The first officer's transmissions now have a voice.

The call used to be written to the log and handed to the controller and
heard by nobody: the aeroplane keyed nothing. Now the same render lane
that carries the controller and the traffic carries the seat that made
the call -- the intercom's voice, on the radio, before the answer.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock


def _engine():
    from meridianatc.config import Config
    from meridianatc.engine import Engine

    engine = Engine.__new__(Engine)
    engine.config = Config()
    # The radio has power without needing an aircraft to prove it.
    engine.config.atc.radio_needs_power = False
    engine.station = None
    engine.station2 = None
    engine.last_language = "en"
    engine._current_station = Mock(return_value=None)
    engine._language_for = Mock(return_value="en")
    engine.emit = Mock()
    engine.handle_text = Mock()
    engine.player = Mock()
    engine.synth = Mock()
    engine._radio_renders = 0
    engine._voices_reported = False
    engine.session = SimpleNamespace(
        aircraft=SimpleNamespace(written="D-EABC"))
    engine.crew = Mock()
    engine.crew._language = "en"
    engine.crew._callsign = "D-EABC"
    engine.crew._cast = {"first_officer": "voice_from_intercom"}
    engine.crew._voice_for = Mock(return_value="")
    engine.crew._gender_for = Mock(return_value="m")
    engine.crew._accent = Mock(return_value="us")
    return engine


def test_the_call_is_keyed_in_the_first_officers_voice():
    engine = _engine()
    renders = []
    engine.synth.render_soon = lambda job, on_done, priority=1: \
        renders.append((job, on_done))
    engine.first_officer_transmits("D-EABC, cleared to Leirovis, roger")

    assert renders, "the transmission was never sent to the voice"
    job, on_done = renders[0]
    job()
    args, kwargs = engine.synth.speak.call_args
    # A pilot on a frequency, keyed on the callsign -- and the same
    # person the intercom already cast in the seat.
    assert kwargs["position"] == "PILOT"
    assert kwargs["facility"] == "D-EABC"
    assert kwargs["voice_key"] == "voice_from_intercom"
    assert kwargs["signal_quality"] == 1.0
    assert kwargs["radio"] == engine.config.audio.radio_effects


def test_the_voice_goes_out_before_the_answer_is_asked():
    engine = _engine()
    order = []
    engine.synth.render_soon = lambda job, on_done, priority=1: \
        order.append("render")
    engine.handle_text = Mock(
        side_effect=lambda *a, **k: order.append("handle"))
    engine.first_officer_transmits("D-EABC, roger")
    assert order == ["render", "handle"]
    assert engine.handle_text.call_args[1]["speaker"] == "first_officer"


def test_the_render_lands_on_the_player_and_keeps_the_cast():
    engine = _engine()
    engine.crew._cast = {}
    engine.crew._voice_for = Mock(return_value="")
    transmission = SimpleNamespace(
        audio=b"audio", sample_rate=16000, voice_key="drawn_voice")
    engine.synth.render_soon = lambda job, on_done, priority=1: \
        on_done(transmission, None)
    engine._speak_first_officer("D-EABC, roger", "en")

    engine.player.play.assert_called_once()
    assert engine.player.play.call_args[1]["label"] == "D-EABC"
    assert engine.crew._cast["first_officer"] == "drawn_voice"
    assert engine._radio_renders == 0


def test_a_dead_radio_carries_nobodys_words():
    engine = _engine()
    engine.config.atc.radio_needs_power = True
    engine.state = SimpleNamespace(
        connected=True, source="simconnect", title="",
        engines_running=False, com1_status=2, main_bus_volts=0.0)
    engine.first_officer_transmits("D-EABC, roger")
    assert not engine.synth.render_soon.called
