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

log = logging.getLogger(__name__)

# A fact is a number (with an optional decimal tail), a runway designator,
# a squawk, a frequency, or the callsign -- anything whose letters are
# figures or capitals a listener could copy down. Connective words ("on",
# "the", "contact") are not facts and may move freely.
_FACT = re.compile(
    r"\b(?:\d{1,3}\.\d{1,3}|[0-9A-Z]+[0-9]+[0-9A-Z]*|\d+[LRW]?)\b")


def _facts(text: str) -> list[str]:
    """The tokens in ``text`` a render is not allowed to change."""
    return [m.group(0).upper() for m in _FACT.finditer(text)]


class PhrasingRenderer:
    """Rephrases approved transmissions through the configured provider."""

    def __init__(self, provider, prompt_dir=None):
        self.provider = provider
        self._system = self._load_prompt(prompt_dir)

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
        if self.provider is None or not approved.strip():
            return approved
        try:
            render = self.provider.classify(approved, self._system).strip()
        except Exception as exc:
            log.debug("phrasing render failed: %s", exc)
            return approved
        if not render or render == approved:
            return approved
        # Strip the quotes a model sometimes wraps the answer in.
        render = render.strip('"').strip()
        if not render:
            return approved
        wanted = _facts(approved)
        got = set(_facts(render))
        missing = [f for f in wanted if f not in got]
        invented = [f for f in got if f not in set(wanted)]
        if missing or invented:
            log.info("phrasing render rejected (missing=%s invented=%s)",
                     missing, invented)
            return approved
        return render


__all__ = ["PhrasingRenderer"]
