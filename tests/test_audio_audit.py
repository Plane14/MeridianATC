from __future__ import annotations

import itertools
import queue
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from meridianatc.audio.io import (
    CabinPlayer,
    InputMeter,
    MicrophoneCapture,
    PushToTalk,
    RadioPlayer,
    list_audio_devices,
)
from meridianatc.audio.samples import pcm, rate
from meridianatc.audio.stt import Recognizer, Transcript, resample_to


class Stream:
    def __init__(self, **options):
        self.options = options
        self.active = False
        self.closed = False
        self.fail_start = False
        self.fail_stop = False

    def start(self):
        if self.fail_start:
            raise RuntimeError("device lost")
        self.active = True

    def stop(self):
        if self.fail_stop:
            raise RuntimeError("device lost")
        self.active = False

    def close(self):
        self.closed = True

    def send(self, samples):
        self.options["callback"](samples, len(samples), None, None)


@pytest.fixture
def device(monkeypatch):
    streams = []
    def create(**options):
        stream = Stream(**options)
        streams.append(stream)
        return stream
    sd = SimpleNamespace(InputStream=create)
    monkeypatch.setitem(sys.modules, "sounddevice", sd)
    return sd, streams


def test_capture_downmixes_and_bounds_memory_before_release(device):
    _, streams = device
    capture = MicrophoneCapture(sample_rate=100, max_seconds=1, channels=2)
    capture.start()
    stream = streams[0]
    for _ in range(20):
        stream.send(np.column_stack([np.ones(30), np.zeros(30)]))
    assert sum(len(a) for a in capture._frames) == 100
    result = capture.stop()
    assert result.duration_s == 1
    assert result.truncated
    assert np.all(result.audio == .5)
    stream.send(np.ones((100, 2)))
    assert capture._frames == []
    assert stream.closed


def test_native_input_rate_is_reported(device):
    sd, streams = device
    def check(**options):
        if options["samplerate"] != 48000:
            raise RuntimeError("unsupported rate")
    sd.check_input_settings = check
    sd.query_devices = lambda *args: {"default_samplerate": 48000}
    capture = MicrophoneCapture()
    capture.start()
    streams[0].send(np.ones((4800, 1)))
    result = capture.stop()
    assert result.sample_rate == 48000
    assert result.duration_s == .1


@pytest.mark.parametrize("stage", ["start", "stop"])
def test_capture_closes_partial_or_disconnected_stream(device, stage):
    sd, streams = device
    create = sd.InputStream
    def fail(**options):
        stream = create(**options)
        stream.fail_start = stage == "start"
        stream.fail_stop = stage == "stop"
        return stream
    sd.InputStream = fail
    capture = MicrophoneCapture()
    if stage == "start":
        with pytest.raises(RuntimeError):
            capture.start()
    else:
        capture.start()
        assert not capture.stop().audio.size
    assert not capture.recording
    assert streams[0].closed


def test_meter_reopens_lost_device_and_cleans_partial_start(device):
    _, streams = device
    meter = InputMeter()
    assert meter.read()["ok"]
    streams[0].active = False
    assert meter.read()["ok"]
    assert streams[0].closed
    assert len(streams) == 2
    streams[1].send(np.array([[np.nan], [2.0]]))
    assert meter.read()["peak"] == 1
    meter.close()


def test_enumeration_loss_is_recoverable(device):
    sd, _ = device
    def fail():
        raise RuntimeError("device missing")
    sd.query_devices = fail
    assert list_audio_devices() == ([], [])
    sd.query_devices = lambda: [{"name": "headset", "max_input_channels": 1, "max_output_channels": 2}]
    assert list_audio_devices() == ([(0, "headset")], [(0, "headset")])


def test_keyboard_repeat_and_shutdown_release(monkeypatch):
    listeners, pressed, released = [], [], []
    class Listener:
        def __init__(self, **callbacks):
            self.callbacks = callbacks
            listeners.append(self)
        def start(self):
            pass
        def stop(self):
            pass
    monkeypatch.setitem(sys.modules, "pynput", SimpleNamespace(keyboard=SimpleNamespace(Listener=Listener)))
    ptt = PushToTalk(on_press=lambda: pressed.append(1), on_release=lambda: released.append(1))
    ptt.start()
    key = SimpleNamespace(char="`", vk=192)
    listener = listeners[0]
    listener.callbacks["on_press"](key)
    listener.callbacks["on_press"](key)
    ptt.stop()
    listener.callbacks["on_press"](key)
    assert pressed == [1]
    assert released == [1]
    assert not ptt._down


@pytest.mark.parametrize("bad", [0, -1, float("nan"), float("inf"), True, 1.5])
def test_invalid_sample_rates_are_rejected(bad):
    with pytest.raises(ValueError):
        rate(bad)


def test_pcm_sanitizes_and_downmixes():
    out = pcm([[np.nan, 2], [np.inf, -np.inf]], mono=True)
    assert np.allclose(out, [.5, 0])
    assert pcm([]).size == 0
    with pytest.raises(ValueError):
        pcm(np.zeros((2, 2, 2)))


def test_stt_does_not_flatten_stereo_duration():
    recognizer = Recognizer()
    assert recognizer.transcribe(np.zeros((1600, 2)), 16000).duration_s == .1


def test_stereo_resampling_uses_frame_axis():
    pytest.importorskip("scipy")
    out = resample_to(np.ones((4800, 2), dtype=np.float32), 48000, 16000)
    assert out.shape == (1600, 2)
    assert np.isfinite(out).all()


def test_radio_effect_cannot_output_nan_or_clipped_audio():
    pytest.importorskip("scipy")
    from meridianatc.audio.radio_fx import RadioEffect
    effect = RadioEffect()
    out = effect.process(np.tile([np.nan, np.inf, -np.inf, .5], 2000), signal_quality=float("nan"), seed=1)
    assert np.isfinite(out).all()
    assert np.max(np.abs(out)) <= 1
    assert effect.process(np.array([])).size == 0


@pytest.mark.parametrize("length", [1, 10, 1024, 1500])
def test_short_music_beds_wrap_as_many_times_as_needed(length):
    player = CabinPlayer()
    bed = np.arange(length, dtype=np.float32).reshape(-1, 1)
    first = player._next_bed_block(bed)
    second = player._next_bed_block(bed)
    assert first.shape == second.shape == (1024, 1)
    assert np.array_equal(second[:, 0], np.arange(1024, 2048) % length)


def test_player_stop_does_not_poison_restart(device):
    sd, _ = device
    sd.play = lambda *args, **kwargs: None
    sd.stop = lambda: None
    player = RadioPlayer()
    player.start()
    player.stop()
    completed = threading.Event()
    player.play(np.zeros(10), 16000, on_done=completed.set)
    assert completed.wait(2)
    player.stop()


def test_failed_playback_still_takes_a_transmissions_time(device):
    """No speaker is silent, not instant: the frequency stays occupied for
    the transmission's length instead of letting callers pile up."""
    sd, _ = device
    def broken(*args, **kwargs):
        raise RuntimeError("no output device")
    sd.play = broken
    sd.stop = lambda: None
    player = RadioPlayer()
    completed = threading.Event()
    player.play(np.zeros(3200), 16000, on_done=completed.set)
    time.sleep(0.05)
    assert not completed.is_set()
    assert completed.wait(1.0)
    player.stop()


def test_player_queue_is_bounded_and_rejection_finishes_callback(monkeypatch):
    player = RadioPlayer()
    monkeypatch.setattr(player, "start", lambda: None)
    player._queue = queue.Queue(maxsize=1)
    completed = []
    player.play(np.zeros(10), 16000)
    player.play(np.zeros(10), 16000, on_done=lambda: completed.append(1))
    assert player._queue.qsize() == 1
    assert completed == [1]
    player.clear()


def test_render_worker_stop_and_backpressure():
    pytest.importorskip("scipy")
    from meridianatc.audio.tts import Synthesizer
    synth = Synthesizer.__new__(Synthesizer)
    synth._render_jobs = queue.PriorityQueue(maxsize=1)
    synth._render_done = queue.Queue(maxsize=1)
    synth._render_stop = threading.Event()
    synth._render_seq = itertools.count()
    errors = []
    synth.render_soon(lambda: 1, lambda *args: None)
    synth.render_soon(lambda: 2, lambda result, error: errors.append(error))
    assert isinstance(errors[0], RuntimeError)
    synth._render_thread = threading.Thread(target=synth._render_lane)
    synth._render_thread.start()
    synth.stop()
    assert not synth._render_thread.is_alive()


def test_cloud_stt_failure_falls_back_without_retry_storm():
    from meridianatc.integrations.deepgram import DeepgramError, DeepgramRecognizer
    calls = []
    fallback = SimpleNamespace(transcribe=lambda *args, **kwargs: Transcript("offline"))
    recognizer = DeepgramRecognizer("test-key", fallback=fallback)
    def fail(*args, **kwargs):
        calls.append(1)
        raise DeepgramError("down")
    recognizer._transcribe_cloud = fail
    assert recognizer.transcribe(np.zeros(16000)).text == "offline"
    assert recognizer.transcribe(np.zeros(16000)).text == "offline"
    assert calls == [1]
    assert not fallback.allow_download
    recognizer.close()


def test_output_rate_and_channel_fallback_preserves_duration():
    pytest.importorskip("scipy")
    from meridianatc.audio.io import _output_audio
    def check(**options):
        if options["samplerate"] != 48000 or options["channels"] != 2:
            raise RuntimeError("unsupported format")
    sd = SimpleNamespace(check_output_settings=check, query_devices=lambda *args: {"default_samplerate": 48000, "max_output_channels": 2})
    samples, sample_rate = _output_audio(sd, None, np.ones(1600, dtype=np.float32), 16000)
    assert sample_rate == 48000
    assert samples.shape == (4800, 2)


def test_cabin_stop_aborts_wedged_device():
    player = CabinPlayer()
    released = threading.Event()
    player._stream = SimpleNamespace(abort=released.set)
    player._thread = threading.Thread(target=lambda: released.wait(5))
    player._thread.start()
    thread = player._thread
    player.stop()
    assert released.is_set()
    assert not thread.is_alive()


def test_joystick_stop_releases_a_held_button():
    from meridianatc.audio.controllers import ButtonWatch
    released = []
    hub = SimpleNamespace(unwatch=lambda *args: None)
    watch = ButtonWatch(hub, on_release=lambda: released.append(1))
    watch._down = True
    watch.controller = SimpleNamespace(id="test")
    watch.stop()
    watch.stop()
    assert released == [1]


def test_engine_phrasing_runs_on_render_lane():
    pytest.importorskip("scipy")
    pytest.importorskip("rich")
    from meridianatc.engine import Engine
    jobs, calls = [], []
    fake = SimpleNamespace(
        radio_powered=True, state=SimpleNamespace(altitude_ft=10000),
        config=SimpleNamespace(audio=SimpleNamespace(radio_effects=True)),
        phrasing=SimpleNamespace(render=lambda text: calls.append(text) or text),
        spoken_form=lambda text: text, _radio_renders=0, _spoken=lambda *args: None,
        synth=SimpleNamespace(render_soon=lambda job, **kwargs: jobs.append(job), speak=lambda *args, **kwargs: None))
    station = SimpleNamespace(facility="Tower", ident="TEST", position="TOWER", distance_nm=1)
    Engine.speak(fake, "N123 climb 5000", station)
    assert calls == []
    jobs[0]()
    assert calls == ["N123 climb 5000"]


def test_missing_cloud_key_uses_offline_model_without_network():
    from meridianatc.integrations.deepgram import DeepgramRecognizer
    fallback = SimpleNamespace(transcribe=lambda *args, **kwargs: Transcript("offline"))
    recognizer = DeepgramRecognizer("", fallback=fallback)
    assert recognizer.transcribe(np.zeros(16000)).text == "offline"
    recognizer.close()
