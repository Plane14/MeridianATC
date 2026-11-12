"""The opt-in phrasing renderer.

With ``ai.phrasing`` on and a cloud provider configured, an approved
transmission -- the words the deterministic phraseology composed, nothing
more -- is offered to the model for one natural rephrase. What comes back is
checked against what went in before it is ever spoken:

* every fact token in the approved text -- callsign, runway, frequency,
  squawk, heading, altitude, speed, every numeral -- must appear in the
  render, digit-for-digit;
* no fact token may appear in the render that was not in the approved text.

A render that fails either check is discarded and the deterministic wording
is used, which is what is said anyway with the switch off or the provider
unreachable. The model never sees the situation, only the sentence it is
asked to rephrase: it cannot grant, extend, or withhold a clearance because
it never learns there is one.
"""

from __future__ import annotations

import logging
import re
import threading
from collections import Counter
from functools import lru_cache

log = logging.getLogger(__name__)

# A fact is a number (with an optional decimal tail), a runway designator,
# a squawk, a frequency, or the callsign -- anything whose letters are
# figures or capitals a listener could copy down. Connective words ("on",
# "the", "contact") are not facts and may move freely.
_FACT = re.compile(
    r"\b(?:\d{1,3}\.\d{1,3}|[0-9A-Z]+[0-9]+[0-9A-Z]*|\d+[LRW]?)\b")
_ACTIONS = frozenset(("climb", "descend", "cleared", "cancel", "hold", "short",
                      "stop", "not", "never", "no", "cannot", "can't", "don't",
                      "ne", "pas", "non", "nicht", "nao", "montez", "descendez"))


def _facts(text: str) -> list[str]:
    """The tokens in ``text`` a render is not allowed to change."""
    from ..atc.numbers import fold

    words = _spoken_words()
    facts, spoken = [], []
    for token in re.findall(r"\d+(?:\.\d+)?|[^\W\d_]+\d+[\w]*|[^\W\d_]+(?:['’][^\W\d_]+)*", text):
        folded = fold(token)
        if folded in words:
            spoken.append(folded)
            continue
        if spoken:
            facts.append("~" + " ".join(spoken))
            spoken = []
        if any(c.isdigit() for c in token) or token[0].isupper():
            facts.append(token.upper())
    if spoken:
        facts.append("~" + " ".join(spoken))
    return facts


@lru_cache(maxsize=1)
def _spoken_words() -> frozenset[str]:
    from ..atc.intents import _NUMBER_WORDS
    from ..atc.intents_fr import _NUMBER_TOKENS
    from ..atc.numbers import DIGITS, fold, word_values
    from ..atc.speech import _ALPHABET_REVERSE

    words = set(_NUMBER_WORDS) | set(_NUMBER_TOKENS) | set(_ALPHABET_REVERSE)
    words.update(("hundred", "thousand", "left", "right", "centre", "center",
                  "gauche", "droite", "links", "rechts", "sinistra", "destra",
                  "izquierda", "derecha", "esquerda", "direita"))
    for language, digits in DIGITS.items():
        words.update(word_values(language))
        words.update(digits.values())
    return frozenset(fold(w) for w in words) - {"to", "too", "for", "a", "an", "and"}


class PhrasingRenderer:
    """Rephrases approved transmissions through the configured provider."""

    def __init__(self, provider, prompt_dir=None):
        self.provider = provider
        self._system = self._load_prompt(prompt_dir)
        self._closed = threading.Event()

    @staticmethod
    def _load_prompt(prompt_dir) -> str:
        """StormATC's renderer prompt, shipped in ``prompts/`` at the root."""
        from ..paths import bundle_root

        path = (prompt_dir or bundle_root() / "prompts").joinpath(
            "90-phraseology-renderer.md")
        try:
            return path.read_text(encoding="utf-8").strip()
        except OSError:
            return ("Rephrase the air traffic control instruction as one "
                    "natural radio transmission. Keep every number, name, "
                    "runway, frequency and callsign exactly. Add nothing. "
                    "Remove nothing. Reply with the transmission only.")

    def render(self, approved: str) -> str:
        """A rephrase of ``approved``, or ``approved`` itself unchanged.

        Returns the deterministic wording on any failure: unreachable
        provider, empty answer, a fact changed, or a fact invented.
        """
        if self._closed.is_set() or self.provider is None or not approved.strip():
            return approved
        # ``complete`` is the free-form answer a rephrase needs; providers
        # without it only know how to classify, whose schema would hand back
        # a JSON intent blob instead of a sentence.
        call = getattr(self.provider, "complete", None) \
            or self.provider.classify
        try:
            render = call(approved, self._system).strip()
        except Exception as exc:
            log.debug("phrasing render failed (%s)", type(exc).__name__)
            return approved
        if self._closed.is_set():
            return approved
        if not render or render == approved:
            return approved
        # Strip the quotes a model sometimes wraps the answer in.
        render = render.strip('"').strip()
        if not render:
            return approved
        wanted = Counter(_facts(approved))
        got = Counter(_facts(render))
        missing = list((wanted - got).elements())
        invented = list((got - wanted).elements())
        def actions(text):
            return Counter(w for w in re.findall(r"[\w'’]+", text.lower()) if w in _ACTIONS)
        if missing or invented or actions(approved) != actions(render):
            log.info("phrasing render rejected (missing=%s invented=%s)",
                     missing, invented)
            return approved
        return render

    def close(self) -> None:
        self._closed.set()


__all__ = ["PhrasingRenderer"]
