"""The voice trainer, ported from StormATC's recognition profile handling.

Two small stores in the data root, both plain JSON:

* **keyterms** -- words the recogniser should listen harder for (tail
  numbers a Whisper model mishears, an airline name the cloud service does
  not know). The Deepgram recogniser sends them as ``keyterm`` parameters;
  the local one puts them in the decode prompt. No audio is ever stored --
  a correction is words, not a recording.
* **corrections** -- "when you heard X, I meant Y" pairs. Applied to the
  transcript text after recognition, so "say again" becomes "say again"
  rather than "say again" misheard as "stay again".
"""

from __future__ import annotations

import json
import logging
import re
import threading
from pathlib import Path

log = logging.getLogger(__name__)


class VoiceTrainer:
    """Pilot-taught vocabulary and corrections, persisted as JSON."""

    def __init__(self, folder: Path):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        # RLock, not Lock: add_keyterm takes it and then reads ``keyterms``,
        # which takes it again -- a plain Lock would deadlock there.
        self._lock = threading.RLock()
        self._keyterms_path = self.folder / "keyterms.json"
        self._corrections_path = self.folder / "corrections.json"
        self._keyterms: list[str] | None = None
        self._corrections: dict[str, str] | None = None

    def _read_list(self, path: Path) -> list[str]:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return []
        return [str(x).strip() for x in data if str(x).strip()] \
            if isinstance(data, list) else []

    def _read_map(self, path: Path) -> dict[str, str]:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}
        return {str(k).strip().lower(): str(v).strip()
                for k, v in data.items() if str(k).strip() and str(v).strip()} \
            if isinstance(data, dict) else {}

    # ------------------------------------------------------------------
    # keyterms
    # ------------------------------------------------------------------

    @property
    def keyterms(self) -> list[str]:
        with self._lock:
            if self._keyterms is None:
                self._keyterms = self._read_list(self._keyterms_path)
            return list(self._keyterms)

    def add_keyterm(self, term: str) -> None:
        term = " ".join(str(term).split()).strip()
        if not term:
            return
        with self._lock:
            terms = self.keyterms
            if term.lower() not in {t.lower() for t in terms}:
                terms.append(term)
                self._keyterms = terms
                self._keyterms_path.write_text(
                    json.dumps(terms, indent=1) + "\n", encoding="utf-8")

    def remove_keyterm(self, term: str) -> None:
        with self._lock:
            terms = [t for t in self.keyterms
                     if t.lower() != str(term).strip().lower()]
            self._keyterms = terms
            self._keyterms_path.write_text(
                json.dumps(terms, indent=1) + "\n", encoding="utf-8")

    # ------------------------------------------------------------------
    # corrections
    # ------------------------------------------------------------------

    @property
    def corrections(self) -> dict[str, str]:
        with self._lock:
            if self._corrections is None:
                self._corrections = self._read_map(self._corrections_path)
            return dict(self._corrections)

    def correct(self, heard: str, meant: str) -> None:
        heard, meant = str(heard).strip().lower(), str(meant).strip()
        if not heard or not meant:
            return
        with self._lock:
            mapping = self.corrections
            mapping[heard] = meant
            self._corrections = mapping
            self._corrections_path.write_text(
                json.dumps(mapping, indent=1) + "\n", encoding="utf-8")

    def apply(self, text: str) -> str:
        """Every correction applied to a transcript line."""
        out = text
        for heard, meant in self.corrections.items():
            if heard and meant:
                out = re.sub(rf"\b{re.escape(heard)}\b", meant, out,
                             flags=re.I)
        return out


__all__ = ["VoiceTrainer"]
