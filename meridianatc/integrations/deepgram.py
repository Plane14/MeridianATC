"""Deepgram speech, for pilots who bring their own key.

A port of StormATC's Deepgram integration onto this program's contracts:

* :class:`DeepgramRecognizer` answers the same question
  :class:`~meridianatc.audio.stt.Recognizer` does -- a block of recorded
  audio in, a :class:`~meridianatc.audio.stt.Transcript` out -- through
  Deepgram's REST ``/v1/listen`` instead of the local Whisper model. The
  aviation keyterms StormATC ships, plus the pilot's own from the voice
  trainer, plus the callsign and stations the engine knows, are sent as
  ``keyterm`` parameters -- the same vocabulary-biasing the Whisper prompt
  does locally.

* :class:`DeepgramTTS` answers the backend contract in
  :mod:`meridianatc.audio.backends` through ``/v1/speak`` -- for a pilot who
  would rather the controller's voice be a hosted one than a local model.
  Its capabilities honestly declare English and Spanish, which is what Aura
  voices speak; every other language stays local.

The key comes from :func:`~meridianatc.security.secrets.resolve_secret`,
never from a settings file, and the endpoints are pinned by
:mod:`meridianatc.security.endpoints`: a request that does not pass the
policy is never made, and redirects on credentialed calls are not followed.
"""

from __future__ import annotations

import contextlib
import io
import json
import logging
import queue
import threading
import time

import numpy as np
import requests

from ..audio.backends.base import Capabilities
from ..audio.samples import pcm, rate
from ..audio.stt import SAMPLE_RATE, Transcript
from ..security.endpoints import is_allowed

log = logging.getLogger(__name__)

BASE_URL = "https://api.deepgram.com"
STREAM_URL = "wss://api.deepgram.com/v1/listen"
SPEAK_URL = "https://api.deepgram.com/v1/speak"

DEFAULT_STT_MODEL = "nova-3"
DEFAULT_VOICE = "aura-2-orpheus-en"

#: Deepgram's own caps on keyterm lists (per the StormATC profile).
MAX_KEYTERMS = 100
MAX_KEYTERM_TOKENS = 500


class DeepgramError(RuntimeError):
    """The Deepgram endpoint refused or could not be reached."""


# --------------------------------------------------------------------------
# keyterms -- StormATC's DeepgramRecognitionProfile, ported
# --------------------------------------------------------------------------

#: The words a frequency always carries and a recogniser always needs help
#: with. Deepgram biases decoding toward these rather than relying on its
#: general model having heard "squawk".
AVIATION_KEYTERMS: tuple[str, ...] = (
    "IFR clearance", "pushback", "ready for taxi", "holding short",
    "ready for departure", "cleared for takeoff", "request climb",
    "request descent", "flight level", "squawk", "runway", "taxiway",
    "localizer", "request approach", "established", "cleared to land",
    "runway vacated", "taxi to stand", "go around", "mayday", "pan pan",
    "say again", "Delivery", "Ground", "Tower", "Departure", "Approach",
    "Center", "contact Ground", "remain this frequency",
    "request pushback and start", "what is the frequency",
)


def build_keyterms(user_terms=(), callsign: str = "", stations=()) -> list[str]:
    """The keyterm list one request carries, within Deepgram's own caps.

    Aviation terms first -- they are always right on a frequency -- then the
    pilot's own from the voice trainer, then the flight's callsign and the
    stations in reach. Duplicates drop, and the list stops at 100 terms or
    500 tokens, whichever bites first.
    """
    terms: list[str] = []
    seen: set[str] = set()
    tokens = 0
    for term in list(AVIATION_KEYTERMS) + [str(t) for t in user_terms] \
            + ([callsign] if callsign else []) \
            + [str(s) for s in stations]:
        term = " ".join(str(term).split()).strip()
        if not term or term.lower() in seen:
            continue
        cost = len(term.split())
        if len(terms) >= MAX_KEYTERMS or tokens + cost > MAX_KEYTERM_TOKENS:
            break
        seen.add(term.lower())
        terms.append(term)
        tokens += cost
    return terms


# --------------------------------------------------------------------------
# speech to text
# --------------------------------------------------------------------------


def _wav_bytes(samples: np.ndarray, sample_rate: int) -> bytes:
    """The recording as a WAV body Deepgram accepts."""
    import soundfile as sf

    buffer = io.BytesIO()
    sf.write(buffer, np.asarray(samples, dtype=np.float32), sample_rate,
             format="WAV", subtype="PCM_16")
    return buffer.getvalue()


class _StreamSession:
    """One Deepgram listen websocket, fed from the audio callback.

    A ``websockets`` client lives on its own thread with its own event loop;
    :meth:`feed` is the only call the capture thread makes, and it does no
    more than hand a queue a PCM frame. :meth:`finish` tells the sender to
    end the stream (Deepgram answers the ``CloseStream`` message with its
    last transcripts and then closes), and waits for that close. ``done`` is
    set however the pump ends -- clean close, refused connect, lost network.

    The session bounds itself: a transmission whose transcript is never asked
    for -- a second key-press while the first is still being worked on --
    closes itself rather than holding a socket open for the rest of the
    flight.
    """

    #: How long a stream may live un-finished. Well past the microphone's
    #: own transmission limit.
    LIFETIME_S = 120.0

    def __init__(self, api_key: str, url: str, sample_rate: int,
                 lifetime_s: float = LIFETIME_S):
        self.url = url
        self.rate = int(sample_rate)
        self._queue: queue.Queue = queue.Queue()
        self._closed = False
        self.done = threading.Event()
        self.finals: list[str] = []
        self.confidence = 0.0
        self.detected = ""
        self.error: Exception | None = None
        self._loop = None
        self._ws = None
        self._deadline = time.monotonic() + max(5.0, lifetime_s)
        self._thread = threading.Thread(
            target=self._run, args=(api_key,), daemon=True,
            name="meridianatc-dg-stream")
        self._thread.start()

    # -- the capture thread's side -------------------------------------------

    def feed(self, samples) -> None:
        """Queue one captured float32 mono block; called on the audio thread."""
        if self._closed:
            return
        pcm16 = (np.clip(np.asarray(samples, dtype=np.float32), -1.0, 1.0)
                 * 32767.0).astype(np.int16).tobytes()
        self._queue.put(pcm16)

    # -- the consumer's side ---------------------------------------------------

    def finish(self, timeout_s: float) -> bool:
        """End the stream and wait for the server's last transcripts.

        False on timeout: the session is still winding down in the
        background and its partial finals are not trusted, so the caller
        falls back to sending the same audio over REST.
        """
        if not self._closed:
            self._closed = True
            self._queue.put(None)
        return self.done.wait(timeout_s)

    def abort(self) -> None:
        """Drop a stream nobody will transcribe (watchdog, key-up noise).

        The socket is closed outright, not just stopped being fed: a server
        that never answers CloseStream would otherwise hold the pump's
        receive loop open for the rest of the flight.
        """
        self._closed = True
        self._queue.put(None)
        loop, ws, asyncio = self._loop, self._ws, getattr(self, "_asyncio", None)
        if loop is not None and ws is not None and asyncio is not None:
            with contextlib.suppress(Exception):
                asyncio.run_coroutine_threadsafe(ws.close(), loop)
        self.done.wait(2.0)

    # -- the pump ------------------------------------------------------------

    def _note(self, raw) -> None:
        # The pump calls this for every websocket message; a frame the
        # service shaped differently must not kill the pump, so the whole
        # parse sits inside the guard, not just the JSON decode.
        try:
            message = json.loads(raw)
            if not isinstance(message, dict):
                return
            if message.get("type") != "Results" or not message.get("is_final"):
                return
            channel = message.get("channel") or {}
            alternatives = channel.get("alternatives") or [{}]
            alternative = alternatives[0] if alternatives else {}
            if not isinstance(alternative, dict):
                return
            text = (alternative.get("transcript") or "").strip()
            if text:
                self.finals.append(text)
            confidence = float(alternative.get("confidence") or 0.0)
            if confidence:
                self.confidence = confidence
            detected = (channel.get("detected_language")
                        or alternative.get("language"))
            if detected:
                self.detected = str(detected)[:2]
        except (TypeError, ValueError, AttributeError, IndexError):
            return

    async def _send(self, ws, asyncio) -> None:
        while True:
            remaining = self._deadline - time.monotonic()
            if remaining <= 0.0:
                break
            try:
                chunk = await asyncio.to_thread(
                    self._queue.get, True, min(0.5, remaining))
            except queue.Empty:
                continue
            if chunk is None:
                break
            await ws.send(chunk)
        with contextlib.suppress(Exception):
            await ws.send(json.dumps({"type": "CloseStream"}))

    async def _pump(self, api_key: str, asyncio, websockets) -> None:
        self._asyncio = asyncio
        self._loop = asyncio.get_running_loop()
        async with websockets.connect(
                self.url, open_timeout=10.0,
                additional_headers={
                    "Authorization": f"Token {api_key}"}) as ws:
            self._ws = ws
            sender = asyncio.create_task(self._send(ws, asyncio))
            try:
                async for raw in ws:
                    self._note(raw)
            finally:
                sender.cancel()
                self._ws = None

    def _run(self, api_key: str) -> None:
        try:
            import asyncio

            import websockets
        except ImportError as exc:
            self.error = exc
            self.done.set()
            return
        try:
            asyncio.run(self._pump(api_key, asyncio, websockets))
        except Exception as exc:
            self.error = exc
            log.debug("deepgram stream ended early: %s", exc)
        finally:
            self._loop = None
            self.done.set()


class DeepgramRecognizer:
    """Pilot transcription through Deepgram's listen endpoints.

    Implements the contract the engine uses on :class:`Recognizer` -- the
    methods it actually calls: ``transcribe``, ``set_languages``,
    ``set_context``, ``detect``, ``loaded``, ``warm`` and the ``multilingual``
    property. There is no model to load: ``loaded`` is always true and
    ``allow_download`` is meaningless here.

    With ``stream`` on and ``websockets`` installed, a transmission is also
    offered to ``/v1/listen`` over a websocket while it is being captured
    (:meth:`start_stream`), so the transcript is already back when the button
    comes up instead of starting its journey then. Whatever the stream does
    not produce -- a refused connect, a socket that dies mid-flight -- sends
    the same captured audio to the REST endpoint it always used; the stream
    only ever saves the wait, it never changes what was heard.
    """

    allow_download = False

    def __init__(self, api_key: str, model: str = DEFAULT_STT_MODEL,
                 base_url: str = BASE_URL, languages: list[str] | None = None,
                 keyterms=(), timeout_s: float = 20.0, fallback=None,
                 stream: bool = True, stream_url: str = STREAM_URL,
                 stream_finalize_s: float = 4.0):
        self.api_key = api_key
        self.model = model or DEFAULT_STT_MODEL
        self.base_url = (base_url or BASE_URL).rstrip("/")
        self.languages = [lang.lower()[:2] for lang in (languages or ["en"])]
        self.language = self.languages[0] if len(self.languages) == 1 else None
        self.user_keyterms = list(keyterms)
        self.callsign = ""
        self.stations: list[str] = []
        self.timeout_s = timeout_s
        self._session = requests.Session()
        self._session.max_redirects = 0
        self.fallback = fallback
        if fallback is not None:
            fallback.allow_download = False
        self._retry_at = 0.0
        self._failures = 0
        self.stream_enabled = stream
        self.stream_url = (stream_url or STREAM_URL).rstrip("/")
        self.stream_finalize_s = max(0.5, stream_finalize_s)
        self._stream: _StreamSession | None = None
        self._stream_lock = threading.Lock()

    @property
    def multilingual(self) -> bool:
        return len(self.languages) > 1

    @property
    def loaded(self) -> bool:
        return True

    def set_languages(self, languages: list[str]) -> bool:
        if self.fallback is not None:
            self.fallback.set_languages(languages)
        wanted = [lang.lower()[:2] for lang in languages if lang] or ["en"]
        if wanted == self.languages:
            return False
        self.languages = wanted
        self.language = wanted[0] if len(wanted) == 1 else None
        return True

    def set_context(self, callsign: str = "", stations=()) -> None:
        self.callsign = callsign or ""
        self.stations = list(dict.fromkeys(s for s in stations if s))
        if self.fallback is not None:
            self.fallback.set_context(callsign, stations)

    def detect(self, samples: np.ndarray) -> tuple[str, float]:
        """Deepgram does its own detection; the first language is nominal."""
        return self.languages[0], 0.0

    def warm(self) -> None:
        pass

    # ------------------------------------------------------------------
    # streaming -- the same transcript, heard while it is spoken
    # ------------------------------------------------------------------

    def start_stream(self, sample_rate: int):
        """Open a listen websocket for the transmission about to begin.

        Returns the sink the microphone feeds each captured block to, or None
        when streaming is not on offer -- switched off, no key, a blocked
        endpoint, no ``websockets`` -- in which case nothing changes: the
        audio is still recorded and still goes to REST when the button comes
        up.
        """
        if not self.stream_enabled or not self.api_key:
            return None
        if not is_allowed("deepgram_stream", self.stream_url):
            log.debug("deepgram stream blocked by policy: %r", self.stream_url)
            return None
        try:
            import websockets  # noqa: F401
        except ImportError:
            log.debug("websockets not installed; streamed recognition off")
            return None
        with self._stream_lock:
            if self._stream is not None:
                # A session still waiting on its transcribe belongs to the
                # last transmission: mixing a new one into it would corrupt
                # both, so it is closed and its audio goes over REST instead.
                old, self._stream = self._stream, None
                old.abort()
            session = _StreamSession(
                self.api_key, self._stream_url(sample_rate), sample_rate)
            self._stream = session
            return session.feed

    def abort_stream(self) -> None:
        """Drop a stream whose audio will never be transcribed."""
        with self._stream_lock:
            session, self._stream = self._stream, None
        if session is not None:
            session.abort()

    def _stream_url(self, sample_rate: int) -> str:
        params = self._listen_params(language=None)
        params.update({
            "encoding": "linear16",
            "sample_rate": str(int(sample_rate)),
            "channels": "1",
            # Only finals: interim guesses repaint the transcript for no
            # benefit -- the pilot sees only the finished call anyway.
            "interim_results": "false",
        })
        terms = build_keyterms(self.user_keyterms, self.callsign,
                               self.stations)
        query = "&".join(
            [f"{k}={v}" for k, v in params.items()]
            + [f"keyterm={requests.utils.quote(t)}" for t in terms])
        return f"{self.stream_url}?{query}"

    def _listen_params(self, language: str | None) -> dict:
        """The query both transports share; the socket adds its encoding."""
        params = {
            "model": self.model,
            "smart_format": "true",
            "punctuate": "true",
            "filler_words": "false",
        }
        chosen = (language or "").lower()[:2] or self.language
        if chosen is not None:
            params["language"] = chosen
        else:
            # The one case "multi" is honest: the flight crossed a border and
            # two languages are genuinely open.
            params["language"] = "multi"
        return params

    def transcribe(self, audio: np.ndarray, sample_rate: int = SAMPLE_RATE,
                   *, prompt: str | None = None,
                   language: str | None = None) -> Transcript:
        session = self._take_stream()
        if session is not None:
            t0 = time.perf_counter()
            if session.finish(self.stream_finalize_s) and not session.error:
                text = " ".join(t for t in session.finals if t).strip()
                samples = pcm(audio, mono=True)
                duration = len(samples) / float(rate(sample_rate)
                                                or SAMPLE_RATE)
                return Transcript(
                    text=text, confidence=session.confidence,
                    duration_s=duration,
                    decode_s=time.perf_counter() - t0,
                    language=session.detected or self.language or "en",
                    language_confidence=session.confidence,
                    segments=[text] if text else [])
            session.abort()
            # The recording is identical either way: whatever the socket
            # failed to deliver, the REST endpoint is still asked for it.
            log.info("streamed recognition failed (%s); sending the "
                     "recording over REST instead",
                     session.error or "timed out")
        if self.fallback is None:
            return self._transcribe_cloud(audio, sample_rate, prompt=prompt, language=language)
        if self.api_key and time.monotonic() >= self._retry_at:
            try:
                result = self._transcribe_cloud(audio, sample_rate, prompt=prompt, language=language)
                self._failures = 0
                return result
            except (DeepgramError, ValueError, TypeError, IndexError, KeyError) as exc:
                self._failures += 1
                self._retry_at = time.monotonic() + min(60.0, 2.0 ** min(self._failures, 6))
                log.warning("cloud recognition failed (%s); using the installed offline model during backoff", type(exc).__name__)
        elif not self.api_key and not self._failures:
            self._failures = 1
            log.warning("cloud recognition key missing; using the installed offline model")
        return self.fallback.transcribe(audio, sample_rate, prompt=prompt, language=language)

    def _take_stream(self) -> _StreamSession | None:
        with self._stream_lock:
            session, self._stream = self._stream, None
        return session

    def close(self) -> None:
        self.abort_stream()
        self._session.close()

    def _transcribe_cloud(self, audio, sample_rate, *, prompt=None, language=None) -> Transcript:
        samples = pcm(audio, mono=True)
        sample_rate = rate(sample_rate)
        duration = len(samples) / float(sample_rate or SAMPLE_RATE)
        if duration < 0.25:
            return Transcript(text="", duration_s=duration)
        if not is_allowed("deepgram", self.base_url):
            raise DeepgramError(
                f"endpoint blocked by policy: {self.base_url!r}")

        params = self._listen_params(language)
        terms = build_keyterms(self.user_keyterms, self.callsign,
                               self.stations)
        query = "&".join(
            [f"{k}={v}" for k, v in params.items()]
            + [f"keyterm={requests.utils.quote(t)}" for t in terms])
        url = f"{self.base_url}/v1/listen?{query}"

        t0 = time.perf_counter()
        try:
            response = self._session.post(
                url, data=_wav_bytes(samples, int(sample_rate)),
                headers={"Authorization": f"Token {self.api_key}",
                         "Content-Type": "audio/wav"},
                timeout=self.timeout_s, allow_redirects=False)
        except requests.RequestException as exc:
            raise DeepgramError(f"listen failed: {type(exc).__name__}") from exc
        decode_s = time.perf_counter() - t0

        if response.status_code != 200:
            raise DeepgramError(f"listen returned {response.status_code}")
        body = response.json()
        channel = (body.get("results") or {}).get("channels", [{}])[0]
        alt = (channel.get("alternatives") or [{}])[0]
        text = (alt.get("transcript") or "").strip()
        confidence = float(alt.get("confidence") or 0.0)
        requested = params.get("language") or "en"
        detected = (channel.get("detected_language") or alt.get("language")
                    or (requested if requested != "multi" else "en"))
        return Transcript(
            text=text, confidence=confidence, duration_s=duration,
            decode_s=decode_s, language=str(detected)[:2],
            language_confidence=confidence,
            segments=[text] if text else [])


# --------------------------------------------------------------------------
# text to speech
# --------------------------------------------------------------------------


class DeepgramTTS:
    """A controller voice hosted at Deepgram Aura (``/v1/speak``).

    The radio chain in :mod:`meridianatc.audio.radio_fx` still does the
    band-limiting and the transmitter noise -- this returns clean speech and
    nothing else, as the backend contract requires.
    """

    capabilities = Capabilities(
        name="deepgram",
        # Aura voices speak English and Spanish; the registry keeps every
        # other language on the local engines.
        languages=frozenset({"en", "es"}),
        needs_gpu=False, can_clone=False, sample_rate=24000, tier="cpu")

    def __init__(self, api_key: str = "", voice: str = DEFAULT_VOICE,
                 base_url: str = SPEAK_URL, timeout_s: float = 20.0):
        self.api_key = api_key
        self.voice = voice or DEFAULT_VOICE
        self.url = base_url or SPEAK_URL
        self.timeout_s = timeout_s
        self._session = requests.Session()
        self._session.max_redirects = 0

    def available(self) -> bool:
        return bool(self.api_key) and is_allowed("deepgram", self.url)

    def warm(self) -> None:
        pass

    def synthesize(self, text: str, voice, lang: str, *,
                   length_scale: float | None = None) -> tuple[np.ndarray, int]:
        if not self.api_key:
            raise DeepgramError("no Deepgram key")
        if not is_allowed("deepgram", self.url):
            raise DeepgramError(f"endpoint blocked by policy: {self.url!r}")
        voice_id = getattr(voice, "voice_id", "") or getattr(
            voice, "id", "") or self.voice
        params = (f"model={requests.utils.quote(str(voice_id))}"
                  "&encoding=linear16&sample_rate="
                  f"{self.capabilities.sample_rate}&container=none")
        try:
            response = self._session.post(
                f"{self.url}?{params}", json={"text": text},
                headers={"Authorization": f"Token {self.api_key}"},
                timeout=self.timeout_s, allow_redirects=False)
        except requests.RequestException as exc:
            raise DeepgramError(f"speak failed: {type(exc).__name__}") from exc
        if response.status_code != 200:
            raise DeepgramError(f"speak returned {response.status_code}")
        if not response.content or len(response.content) % 2:
            raise DeepgramError("speak returned a body that is not PCM16")
        pcm = np.frombuffer(response.content, dtype=np.int16)
        audio = (pcm.astype(np.float32) / 32768.0).clip(-1.0, 1.0)
        return audio, self.capabilities.sample_rate


__all__ = ["AVIATION_KEYTERMS", "BASE_URL", "DEFAULT_STT_MODEL",
           "DEFAULT_VOICE", "STREAM_URL", "DeepgramError", "DeepgramRecognizer",
           "DeepgramTTS", "build_keyterms"]
