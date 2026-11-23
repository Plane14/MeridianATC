"""Model proposes, engine disposes.

The understanding layer lets the model label what the pilot said; this layer
lets it choose among things the engine could already do, and word the reply
whose facts the engine already fixed. It decides nothing itself:

* :meth:`Decisions.pick` -- the engine enumerates the options; the model
  returns one index. A pick outside the list is discarded and the engine's
  own choice stands. It cannot invent an option, extend a clearance or move
  an aeroplane, because the only thing that crosses the boundary is a number.

* :meth:`Decisions.draft` -- the engine supplies the reply it was about to
  make and the facts it was made from; the model may word it. The draft is
  kept only when every fact token it carries -- a number, a callsign, a
  name -- was already in the supplied facts and it dropped none of the
  reply's own: the same check the phrasing renderer makes. It cannot grant,
  extend or withhold anything, because every new fact discards the draft.

Either call failing, for any reason, leaves the deterministic answer, which
is also what stands with the layer switched off or the model unreachable.

With ``shadow`` on, the model is still asked and its answer is logged beside
the one the engine used (``decision`` rows, kind ``ai_advisor``), but the
engine's answer is the one acted on -- a way to measure agreement before the
pick is trusted.
"""

from __future__ import annotations

import json
import logging
import re
import threading
import time
from collections import Counter
from dataclasses import dataclass

from ..logs import decision
from .rendering import _ACTIONS, _facts

log = logging.getLogger(__name__)

# The JSON object a small model wraps its pick in, wherever it puts it.
_OBJECT = re.compile(r"\{.*\}", re.DOTALL)

_WORDS = re.compile(r"[\w'’]+")

# The operational words a draft is not allowed to change. The renderer's own
# verb set plus the rest of the vocabulary that would alter meaning without
# carrying a fact token -- a draft may not turn "downwind" into "final" or
# a chat into a clearance, because every word in this set must survive
# unchanged, count included.
_ADVISORY_ACTIONS = _ACTIONS | frozenset((
    "land", "landing", "takeoff", "taxi", "departure", "arrival",
    "final", "downwind", "base", "upwind", "crosswind", "touchdown",
    "lineup", "backtrack", "vacate", "clearance", "mayday", "pan",
    "wilco", "unable", "affirm", "negative", "roger", "squawk",
))


@dataclass
class DecisionsStats:
    """How the advisory layer is performing, for the console and the log."""

    calls: int = 0
    failures: int = 0
    accepted: int = 0
    rejected: int = 0
    shadowed: int = 0
    model_time_s: float = 0.0


# What is counted per scope, for the telemetry a shadow run produces.
_SCOPE_KEYS = ("asked", "proposed", "declined", "rejected", "failed",
               "overtime")


def _parse_choice(raw: str):
    """The pick out of the model's reply: an option number, "none", or None."""
    found = _OBJECT.search(raw or "")
    if not found:
        return None
    try:
        value = json.loads(found.group(0)).get("choice")
    except (ValueError, AttributeError):
        return None
    if isinstance(value, str):
        value = value.strip().lower()
        if value == "none":
            return "none"
        if value.isdigit():
            return int(value)
        return None
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    return None


class Decisions:
    """One provider, two advisory calls, a deterministic fallback."""

    def __init__(self, provider=None, *, enabled: bool = True,
                 shadow: bool = False):
        self.provider = provider
        self.enabled = enabled
        self.shadow = shadow
        self.stats = DecisionsStats()
        # Per-scope counters: the veto/miss rates a ``decisions_shadow`` run
        # is run to produce, summed into the log when the layer is closed.
        self.scopes: dict[str, Counter] = {}
        self._closed = threading.Event()
        self._retry_at = 0.0
        self._failures = 0

    def _count(self, scope: str, key: str) -> None:
        self.scopes.setdefault(scope, Counter())[key] += 1

    @property
    def model_available(self) -> bool:
        if self._closed.is_set() or time.monotonic() < self._retry_at \
                or not (self.enabled and self.provider is not None):
            return False
        try:
            return bool(self.provider.available())
        except Exception as exc:
            log.debug("advisor availability check failed (%s)",
                      type(exc).__name__)
            return False

    # ------------------------------------------------------------------
    # pick: one index out of an enumerated list
    # ------------------------------------------------------------------

    _PICK_SYSTEM = (
        "You advise an air traffic control simulation. You are given a "
        "situation and a numbered list of options. Answer with a JSON object "
        "{\"choice\": N} holding the option number, or {\"choice\": \"none\"} "
        "when none should be taken. Answer with the JSON object only; never "
        "invent an option.")

    def pick(self, scope: str, question: str, options: list[str], *,
             allow_none: bool = True, default: int | None = None,
             wait_s: float | None = None):
        """The index into ``options`` the model chose, or ``default``.

        ``-1`` means the model declined every option, which a caller only
        sees when it passed ``allow_none``. An answer that does not parse,
        or a number outside the list, leaves ``default`` -- the engine's
        own pick. ``wait_s`` bounds how long the call may stall the caller,
        for picks made on a clock.
        """
        if not options or not self.model_available:
            return default
        numbered = "\n".join(
            f"{i + 1}. {text}" for i, text in enumerate(options))
        prompt = f"{question}\n\nOptions:\n{numbered}"
        if allow_none:
            prompt += "\n0. None -- take no action."
        raw = (self._ask(prompt, self._PICK_SYSTEM, scope)
               if wait_s is None
               else self._ask_bounded(prompt, self._PICK_SYSTEM, scope,
                                      wait_s))
        if raw is None:
            return default
        choice = _parse_choice(raw)
        # The prompt numbers "none" as 0; a model that answers 0 means the
        # same thing as "none".
        if choice == 0:
            choice = "none"
        if choice == "none":
            if not allow_none:
                self._rejected(scope)
                return default
            self._count(scope, "declined")
            self._took(scope, picked=-1)
            return default if self.shadow else -1
        if isinstance(choice, int) and 1 <= choice <= len(options):
            self._took(scope, picked=choice - 1)
            return default if self.shadow else choice - 1
        self._rejected(scope)
        log.info("AI advisor %s pick rejected: %r", scope, raw[:120])
        return default

    # ------------------------------------------------------------------
    # draft: word a reply whose facts are already fixed
    # ------------------------------------------------------------------

    _DRAFT_SYSTEM = (
        "You are {persona} in a flight simulation. Word the reply below "
        "naturally, as spoken words only -- one or two short sentences. "
        "Every number, name, callsign and place in your reply must come from "
        "the facts given; invent none, and drop none of the reply's own. No "
        "clearances, instructions or permissions: this is conversation. "
        "Answer with the spoken words only.")

    def draft(self, scope: str, question: str, facts: str, baseline: str, *,
              persona: str = "a crew member", max_len: int = 240,
              wait_s: float | None = None) -> str:
        """A model-worded version of ``baseline``, or ``baseline`` unchanged.

        The draft survives only while every fact token in it was already in
        ``facts`` or the baseline, and while it keeps every fact the
        baseline carried. ``wait_s`` bounds how long the call may stall the
        caller, for drafts made on a clock -- otherwise a provider timeout
        sits on the reply path.
        """
        if not (baseline or "").strip() or not self.model_available:
            return baseline
        system = self._DRAFT_SYSTEM.format(persona=persona)
        prompt = (f"The pilot said: {question}\n"
                  f"Facts: {facts}\n"
                  f"Reply to word: {baseline}")
        raw = (self._ask(prompt, system, scope)
               if wait_s is None
               else self._ask_bounded(prompt, system, scope, wait_s))
        if raw is None:
            return baseline
        candidate = raw.strip().strip("\"'").strip()
        if not candidate or len(candidate) > max_len:
            self._rejected(scope)
            return baseline
        allowed = Counter(_facts(f"{baseline} {facts}"))
        wanted = Counter(_facts(baseline))
        got = Counter(_facts(candidate))
        invented = list((got - allowed).elements())
        missing = list((wanted - got).elements())
        # And the operational vocabulary: identical multiset against the
        # baseline, so a draft may not add an instruction the facts check
        # cannot see -- "cleared to land", or "final" for "downwind".
        def actions(text):
            return Counter(w for w in _WORDS.findall(text.lower())
                           if w in _ADVISORY_ACTIONS)
        if invented or missing or actions(candidate) != actions(baseline):
            self._rejected(scope)
            log.info("AI %s draft rejected (invented=%s missing=%s)",
                     scope, invented, missing)
            return baseline
        self._took(scope, drafted=candidate)
        return baseline if self.shadow else candidate

    # ------------------------------------------------------------------

    def _ask_bounded(self, prompt: str, system: str, scope: str,
                     wait_s: float) -> str | None:
        """``_ask`` on a scratch thread, waited out for ``wait_s`` seconds.

        For callers on a clock -- the situation loop cannot sit out a
        provider timeout. The abandoned thread finishes on its own and only
        moves the counters; its late answer is discarded.
        """
        box: list[str | None] = []

        def run() -> None:
            box.append(self._ask(prompt, system, scope))

        worker = threading.Thread(target=run, daemon=True)
        worker.start()
        worker.join(wait_s)
        if worker.is_alive():
            self._count(scope, "overtime")
            log.info("AI advisor answer over %.1fs; deterministic answer",
                     wait_s)
            return None
        return box[0] if box else None

    def _ask(self, prompt: str, system: str, scope: str) -> str | None:
        """One free-form model call, with the same backoff the understanding
        layer uses: a failed provider is retried after a pause, not on every
        line of a flight."""
        call = getattr(self.provider, "complete", None) \
            or self.provider.classify
        self._count(scope, "asked")
        started = time.perf_counter()
        try:
            raw = call(prompt, system)
        except Exception as exc:
            self.stats.failures += 1
            self._count(scope, "failed")
            self._failures += 1
            self._retry_at = (time.monotonic()
                              + min(60.0, 2.0 ** min(self._failures, 6)))
            log.warning("AI advisor call failed (%s); deterministic answer "
                        "during retry backoff", type(exc).__name__)
            return None
        finally:
            self.stats.calls += 1
            self.stats.model_time_s += time.perf_counter() - started
        if self._closed.is_set():
            return None
        self._failures = 0
        self._retry_at = 0.0
        return raw

    def _took(self, scope: str, **fields) -> None:
        self.stats.accepted += 1
        self._count(scope, "proposed")
        if self.shadow:
            self.stats.shadowed += 1
        decision("ai_advisor", scope=scope, shadow=self.shadow, **fields)

    def _rejected(self, scope: str) -> None:
        self.stats.rejected += 1
        self._count(scope, "rejected")

    def close(self) -> None:
        self._closed.set()
        if self.scopes:
            summary = {scope: {k: counts.get(k, 0) for k in _SCOPE_KEYS}
                       for scope, counts in sorted(self.scopes.items())}
            log.info("AI advisor telemetry: %s", summary)
            try:
                decision("ai_advisor_stats", scopes=summary)
            except Exception:
                log.debug("could not write advisor telemetry",
                          exc_info=True)

    def describe(self) -> str:
        """One line for the console about the advisory layer."""
        if not self.model_available:
            return "Advisor: off."
        mode = "shadow" if self.shadow else "advisory"
        return (f"Advisor: {self.provider.name} "
                f"({getattr(self.provider, 'model', '?')}), {mode} scope.")


__all__ = ["Decisions", "DecisionsStats"]
