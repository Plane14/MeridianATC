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

import io
import logging
import time
from dataclasses import dataclass

import numpy as np
import requests

from ..audio.backends.base import Capabilities
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


class DeepgramRecognizer:
    """Pilot transcription through Deepgram's REST listen endpoint.

    Implements the contract the engine uses on :class:`Recognizer` -- the
    methods it actually calls: ``transcribe``, ``set_languages``,
    ``set_context``, ``detect``, ``loaded``, ``warm`` and the ``multilingual``
    property. There is no model to load: ``loaded`` is always true and
    ``allow_download`` is meaningless here.
    """

    allow_download = False

    def __init__(self, api_key: str, model: str = DEFAULT_STT_MODEL,
                 base_url: str = BASE_URL, languages: list[str] | None = None,
                 keyterms=(), timeout_s: float = 20.0):
        self.api_key = api_key
        self.model = model or DEFAULT_STT_MODEL
        self.base_url = (base_url or BASE_URL).rstrip("/")
        self.languages = [l.lower()[:2] for l in (languages or ["en"])]
        self.language = self.languages[0] if len(self.languages) == 1 else None
        self.user_keyterms = list(keyterms)
        self.callsign = ""
        self.stations: list[str] = []
        self.timeout_s = timeout_s
        self._session = requests.Session()
        self._session.max_redirects = 0

    @property
    def multilingual(self) -> bool:
        return len(self.languages) > 1

    @property
    def loaded(self) -> bool:
        return True

    def set_languages(self, languages: list[str]) -> bool:
        wanted = [l.lower()[:2] for l in languages if l] or ["en"]
        if wanted == self.languages:
            return False
        self.languages = wanted
        self.language = wanted[0] if len(wanted) == 1 else None
        return True

    def set_context(self, callsign: str = "", stations=()) -> None:
        self.callsign = callsign or ""
        self.stations = list(dict.fromkeys(s for s in stations if s))

    def detect(self, samples: np.ndarray) -> tuple[str, float]:
        """Deepgram does its own detection; the first language is nominal."""
        return self.languages[0], 0.0

    def warm(self) -> None:
        pass

    def transcribe(self, audio: np.ndarray, sample_rate: int = SAMPLE_RATE,
                   *, prompt: str | None = None,
                   language: str | None = None) -> Transcript:
        samples = np.asarray(audio, dtype=np.float32).flatten()
        duration = len(samples) / float(sample_rate or SAMPLE_RATE)
        if duration < 0.25:
            return Transcript(text="", duration_s=duration)
        if not is_allowed("deepgram", self.base_url):
            raise DeepgramError(
                f"endpoint blocked by policy: {self.base_url!r}")

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
        detected = (channel.get("detected_language")
                    or alt.get("language") or chosen or "en")
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
        pcm = np.frombuffer(response.content, dtype=np.int16)
        audio = (pcm.astype(np.float32) / 32768.0).clip(-1.0, 1.0)
        return audio, self.capabilities.sample_rate


__all__ = ["AVIATION_KEYTERMS", "BASE_URL", "DEFAULT_STT_MODEL",
           "DEFAULT_VOICE", "DeepgramError", "DeepgramRecognizer",
           "DeepgramTTS", "build_keyterms"]
