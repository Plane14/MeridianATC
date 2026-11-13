"""Deepgram integration: keyterm budget, endpoint pinning, and the
streaming listen path (integrations/deepgram._StreamSession) exercised
against a local websocket server that plays the Deepgram side.

The server is real websockets plumbing on 127.0.0.1, so the client's
threading, ordering and close behaviour are all genuine; only the answers
are canned.
"""

from __future__ import annotations

import asyncio
import json
import threading
import time

import numpy as np
import pytest

import meridianatc.integrations.deepgram as deepgram_mod
from meridianatc.audio.stt import Transcript
from meridianatc.integrations.deepgram import (
    DeepgramRecognizer,
    build_keyterms,
)

websockets = pytest.importorskip("websockets")
from websockets.asyncio.server import serve  # noqa: E402

# --------------------------------------------------------------------------
# a pretend api.deepgram.com
# --------------------------------------------------------------------------

class FakeListen:
    """The server's half of /v1/listen, recorded and scripted.

    ``replies`` are sent once ``reply_after`` audio frames have arrived --
    enough for the client to have proven it is streaming, not buffering.
    ``mute`` keeps it silent and never closes, which is how a network that
    dies mid-transmission looks from the socket.
    """

    def __init__(self, replies=(), reply_after=1, mute=False):
        self.replies = list(replies)
        self.reply_after = reply_after
        self.mute = mute
        self.audio: list[bytes] = []
        self.requests = []
        self.close_stream_seen = threading.Event()
        self.port = 0
        self._loop = None
        self._server = None
        self._thread = None

    async def _handler(self, ws):
        self.requests.append(ws.request)
        heard = 0
        if self.mute:
            try:
                await asyncio.Future()
            except Exception:
                return
        try:
            async for raw in ws:
                if isinstance(raw, (bytes, bytearray)):
                    self.audio.append(bytes(raw))
                    heard += 1
                    if heard == self.reply_after:
                        for reply in self.replies:
                            await ws.send(json.dumps(reply))
                else:
                    try:
                        message = json.loads(raw)
                    except ValueError:
                        continue
                    if message.get("type") == "CloseStream":
                        self.close_stream_seen.set()
                        await ws.close()
                        return
        except Exception:
            return

    def __enter__(self):
        started = threading.Event()

        def run():
            loop = asyncio.new_event_loop()
            self._loop = loop
            asyncio.set_event_loop(loop)

            async def main():
                self._server = await serve(self._handler, "127.0.0.1", 0)
                self.port = self._server.sockets[0].getsockname()[1]
                started.set()

            loop.run_until_complete(main())
            loop.run_forever()
            loop.run_until_complete(self._server.wait_closed())
            loop.close()

        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()
        assert started.wait(5.0)
        return self

    def __exit__(self, *exc):
        if self._server is not None:
            self._loop.call_soon_threadsafe(self._server.close)
        time.sleep(0.1)
        self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    @property
    def url(self) -> str:
        return f"ws://127.0.0.1:{self.port}/v1/listen"


@pytest.fixture
def open_policy(monkeypatch):
    """Let the pinned-endpoint check pass for the local test server."""
    monkeypatch.setattr(deepgram_mod, "is_allowed",
                        lambda provider, url: True)


def _result(text: str, confidence: float = 0.95) -> dict:
    return {"type": "Results", "is_final": True,
            "channel": {"alternatives": [
                {"transcript": text, "confidence": confidence}]}}


# --------------------------------------------------------------------------
# streaming
# --------------------------------------------------------------------------

class TestStreaming:
    def test_streamed_transcript_is_returned(self, open_policy):
        fake = FakeListen(replies=[_result("kilo tower good day")])
        with fake:
            rec = DeepgramRecognizer("test-key",
                                     stream_url=fake.url,
                                     stream_finalize_s=5.0)
            feed = rec.start_stream(16000)
            assert feed is not None
            feed(np.zeros(1600, dtype=np.float32))
            out = rec.transcribe(np.zeros(16000, dtype=np.float32), 16000)
            assert out.text == "kilo tower good day"
            assert out.confidence == 0.95
            assert out.duration_s == pytest.approx(1.0)
            assert fake.close_stream_seen.is_set()
            assert fake.audio            # the frame really went out

    def test_finals_are_joined_in_order(self, open_policy):
        fake = FakeListen(replies=[_result("climbing"),
                                   _result("flight level three five zero")])
        with fake:
            rec = DeepgramRecognizer("test-key", stream_url=fake.url)
            feed = rec.start_stream(16000)
            feed(np.zeros(1600, dtype=np.float32))
            out = rec.transcribe(np.zeros(16000, dtype=np.float32), 16000)
            assert out.text == "climbing flight level three five zero"

    def test_every_frame_is_streamed(self, open_policy):
        fake = FakeListen(replies=[_result("roger")])
        with fake:
            rec = DeepgramRecognizer("test-key", stream_url=fake.url)
            feed = rec.start_stream(16000)
            for _ in range(5):
                feed(np.zeros(1600, dtype=np.float32))
            rec.transcribe(np.zeros(16000, dtype=np.float32), 16000)
            deadline = time.monotonic() + 3.0
            while len(fake.audio) < 5 and time.monotonic() < deadline:
                time.sleep(0.02)
            assert len(fake.audio) == 5

    def test_auth_header_and_query_reach_the_socket(self, open_policy):
        fake = FakeListen(replies=[_result("roger")])
        with fake:
            rec = DeepgramRecognizer("test-key", stream_url=fake.url)
            feed = rec.start_stream(16000)
            feed(np.zeros(1600, dtype=np.float32))
            rec.transcribe(np.zeros(16000, dtype=np.float32), 16000)
            deadline = time.monotonic() + 3.0
            while not fake.requests and time.monotonic() < deadline:
                time.sleep(0.02)
            request = fake.requests[0]
            assert request.headers.get("Authorization") == "Token test-key"
            assert "encoding=linear16" in request.path
            assert "sample_rate=16000" in request.path
            assert "model=nova-3" in request.path
            assert "keyterm=" in request.path

    def test_stream_failure_falls_back_to_rest(self, open_policy):
        # A port that refuses the connect: the stream errors, transcribe
        # hands the same audio to the REST path it always used.
        calls = []
        rec = DeepgramRecognizer(
            "test-key",
            stream_url="ws://127.0.0.1:1/v1/listen",
            stream_finalize_s=5.0)

        def rest(*args, **kwargs):
            calls.append(1)
            return Transcript(text="via rest")

        rec._transcribe_cloud = rest
        feed = rec.start_stream(16000)
        assert feed is not None
        feed(np.zeros(1600, dtype=np.float32))
        out = rec.transcribe(np.zeros(16000, dtype=np.float32), 16000)
        assert out.text == "via rest"
        assert calls == [1]

    def test_mute_server_times_out_into_rest(self, open_policy):
        fake = FakeListen(mute=True)
        with fake:
            rec = DeepgramRecognizer("test-key", stream_url=fake.url,
                                     stream_finalize_s=0.5)
            calls = []
            rec._transcribe_cloud = (
                lambda *a, **k: calls.append(1) or Transcript("rest"))
            feed = rec.start_stream(16000)
            feed(np.zeros(1600, dtype=np.float32))
            out = rec.transcribe(np.zeros(16000, dtype=np.float32), 16000)
            assert out.text == "rest"
            assert calls == [1]

    def test_abort_ends_the_session(self, open_policy):
        fake = FakeListen(replies=[_result("roger")])
        with fake:
            rec = DeepgramRecognizer("test-key", stream_url=fake.url)
            rec._transcribe_cloud = lambda *a, **k: Transcript("rest")
            feed = rec.start_stream(16000)
            feed(np.zeros(1600, dtype=np.float32))
            rec.abort_stream()
            assert rec._stream is None
            out = rec.transcribe(np.zeros(16000, dtype=np.float32), 16000)
            assert out.text == "rest"


class TestStreamGates:
    def test_no_key_means_no_stream(self, open_policy):
        rec = DeepgramRecognizer("", stream_url="wss://x.test/v1/listen")
        assert rec.start_stream(16000) is None

    def test_disabled_means_no_stream(self, open_policy):
        rec = DeepgramRecognizer("key", stream=False,
                                 stream_url="wss://x.test/v1/listen")
        assert rec.start_stream(16000) is None

    def test_blocked_endpoint_means_no_stream(self, monkeypatch):
        monkeypatch.setattr(deepgram_mod, "is_allowed", lambda p, u: False)
        rec = DeepgramRecognizer("key", stream_url="wss://x.test/v1/listen")
        assert rec.start_stream(16000) is None

    def test_no_websockets_means_no_stream(self, monkeypatch):
        import sys
        # A None in sys.modules is what a failed import leaves behind.
        monkeypatch.setitem(sys.modules, "websockets", None)
        rec = DeepgramRecognizer("key", stream_url="wss://x.test/v1/listen")
        assert rec.start_stream(16000) is None


# --------------------------------------------------------------------------
# the REST path still does what it did
# --------------------------------------------------------------------------

def test_transcribe_without_stream_is_unchanged(monkeypatch):
    rec = DeepgramRecognizer("test-key")
    rec._transcribe_cloud = (
        lambda *a, **k: Transcript(text="over rest"))
    out = rec.transcribe(np.zeros(16000, dtype=np.float32), 16000)
    assert out.text == "over rest"


def test_close_aborts_a_live_stream(open_policy):
    fake = FakeListen(replies=[_result("roger")])
    with fake:
        rec = DeepgramRecognizer("test-key", stream_url=fake.url)
        feed = rec.start_stream(16000)
        feed(np.zeros(1600, dtype=np.float32))
        rec.close()
        assert rec._stream is None


# --------------------------------------------------------------------------
# keyterm budget -- the parts of build_keyterms nobody else covers
# --------------------------------------------------------------------------

def test_keyterm_token_budget_breaks_cleanly():
    words = [" ".join(["w"] * 8)] * 70        # 560 tokens over the cap
    terms = build_keyterms(user_terms=words)
    assert all(isinstance(t, str) and t for t in terms)
    assert len(terms) <= 100
    assert sum(len(t.split()) for t in terms) <= 500


def test_keyterm_list_is_capped_at_a_hundred():
    terms = build_keyterms(user_terms=[f"term{i}" for i in range(200)])
    assert len(terms) == 100


def test_stream_session_is_daemon():
    session = deepgram_mod._StreamSession("k", "ws://127.0.0.1:1/l", 16000)
    assert session._thread.daemon
    session.abort()
