"""Controller logic: what each position says in reply to the pilot.

This is the decision layer. It never composes words -- every reply comes from
:mod:`meridianatc.atc.phraseology` -- it only decides *which* instruction is
appropriate given the position being talked to, the phase of flight, and what
the pilot just said.

Readbacks are checked item by item. A pilot who reads twelve thousand back as
two thousand gets corrected, which is both realistic and the main safety value
of a readback in the first place.

Two limits are worth stating plainly, because both come from data that no open
dataset publishes:

* **Taxi routes come from the simulator where it is there.** There is no free
  worldwide taxiway graph, but the simulator describes every airport it draws,
  and :mod:`meridianatc.atc.taxi` plans the pilot's taxi on that graph -- the
  same one the AI traffic drives on -- with the hold-short taken from the
  runways the route actually crosses. Without it, or where the graph has no
  path, the route is :func:`taxi_route`: a deterministic, plausible sequence
  of taxiway identifiers that does not match the airport diagram, with a
  hold-short picked from the other runways. The phraseology is correct
  either way, and ``logs radio`` says which one was given and why.

* **SIDs and STARs are not used.** Departure and arrival procedure names are
  likewise unpublished in open data, so clearances are issued with radar
  vectors instead. That is real phraseology and is what smaller fields use
  anyway; it simply is not what you would get at a major hub.
"""

from __future__ import annotations

import hashlib
import logging
import math
import re
import time
from dataclasses import dataclass
from typing import Any

from ...logs import decision
from ...navdata import terrain
from ...navdata.channels import same_channel
from ...navdata.db import (
    DEFAULT_TRANSITION_FT,
    NavDB,
    Station,
    bearing_deg,
    haversine_nm,
    normalize_runway,
    transition_altitude,
    transition_level,
    vfr_region,
)
from .. import speech as sp
from ..approaches import (
    VISUAL,
    Approach,
    Choice,
    FieldApproaches,
    asked_for,
    choose,
    legacy_kind,
)
from ..approaches import plan as plan_approach
from ..circuit import agrees, choose_join, circuit_for, runway_length_nm
from ..climb import (
    CLIMBING_FPM,
    DEPARTURE_REACHING_FT,
    LAST_STEP_FT,
    centre_step,
    departure_step,
    departure_top,
    initial_altitude,
)
from ..enroute import is_oceanic
from ..final import final_for
from ..intents import _REPORT_WORD, Intent, ParsedIntent
from ..phraseology import Phraseology, Weather, _join, language_of
from ..procedures import heard as procedure_heard
from ..procedures import near as procedure_near
from ..procedures import on_course_fix, sid_first_fix
from ..procedures import written as written_procedure
from ..session import (
    ATTEMPTS_BEFORE_REISSUE,
    PROMPTS_BEFORE_REISSUE,
    FlightSession,
    Instruction,
    Phase,
    nearly_matches,
    station_key,
)
from ..taxi import say_words

log = logging.getLogger(__name__)


@dataclass
class Reply:
    """One controller transmission to be spoken."""

    text: str
    station: Station
    instruction: Instruction | None = None
    handoff_to: Station | None = None
    silent: bool = False           # nothing to say; do not key the transmitter
    # A ground-handling action this reply authorises: "pushback", or nothing.
    # Named here rather than inferred by the caller from the words, because
    # the words are phraseology in six languages and the tug is a switch.
    ground: str = ""

    @property
    def speak(self) -> bool:
        return bool(self.text) and not self.silent


# A transmission asking for a runway rather than reading one back. Only
# consulted where nothing is pending, so it cannot mistake a readback for a
# request; what it separates is "request runway three zero" from a pilot
# repeating a runway nobody gave them.
_ASKS_FOR_RUNWAY = re.compile(
    r"\b(request(ing)?|like|want|prefer|need|ready for)\b[^.]{0,24}\brunway\b")


# The side and the size of a weather deviation, in every pilot language.
_SIDES = (("left", r"\b(left|gauche|izquierda|links|sinistra|esquerda)\b"),
          ("right", r"\b(right|droite|derecha|rechts|destra|direita)\b"))
_DEGREE_WORDS = r"(degrees?|degres?|grados?|grad|gradi|graus?)"
# English and French tens, which numbers.parse_number does not read.
_TENS = {"five": 5, "ten": 10, "fifteen": 15, "twenty": 20, "thirty": 30,
         "forty": 40, "fifty": 50, "sixty": 60, "cinq": 5, "dix": 10,
         "quinze": 15, "vingt": 20, "trente": 30, "quarante": 40,
         "cinquante": 50, "soixante": 60}


def _side_said(text: str) -> str:
    for side, pattern in _SIDES:
        if re.search(pattern, text):
            return side
    return ""


def _degrees_said(text: str, language: str = "en") -> int:
    """The number in front of "degrees", in figures or words, or 0."""
    from ..numbers import parse_number

    found = re.search(r"\b([\w-]+(?: [\w-]+)?) " + _DEGREE_WORDS + r"\b", text)
    if not found:
        return 0
    words = found.group(1).split()
    for candidate in (words[-1:], words):
        phrase = " ".join(candidate)
        if phrase.isdigit():
            return int(phrase)
        if phrase in _TENS:
            return _TENS[phrase]
        value = parse_number(phrase, language)
        if value:
            return int(value)
    return 0


# Taxiway identifiers used to build the invented route, for a field the
# simulator has not described. Real airports use these letters; the sequence
# is invented, the alphabet is not.
_TAXIWAYS = ("A", "B", "C", "D", "E", "F", "G", "H", "J", "K", "L", "M",
             "N", "P", "Q", "R", "S", "T", "U", "V", "W", "Y", "Z")


def _stable(*parts: str) -> int:
    return int.from_bytes(
        hashlib.sha256("|".join(parts).encode("utf-8")).digest()[:6], "big"
    )


def taxi_route(airport_ident: str, runway: str, inbound: bool = False) -> list[str]:
    """A deterministic, plausible taxi route, for where there is no real one.

    Invented -- see the module docstring. It is what the pilot is given when
    the simulator has not described the airport, or its taxiways have no path
    from where the aeroplane is (:mod:`meridianatc.atc.taxi` plans the real one).
    Determinism is what matters here:
    the same airport and runway always produce the same route, so a pilot who
    flies from one field repeatedly learns "their" taxi route and a readback
    can be checked against it.
    """
    seed = _stable(airport_ident, normalize_runway(runway), "in" if inbound else "out")
    count = 2 + (seed % 2)
    route: list[str] = []
    for i in range(count):
        letter = _TAXIWAYS[(seed >> (8 * (i + 1))) % len(_TAXIWAYS)]
        if letter not in route:
            route.append(letter)
    return route or ["A"]


# The words a pilot reads a way out of the zone back with, in the
# languages the controller works.
_EXIT_WORDS = r"(?:via|by|par|uber|über|por|tramite|pelo|pela)"


def squawk_for(session: FlightSession, airport_ident: str) -> str:
    """Assign a discrete beacon code.

    Real codes come from a block allocated to the facility. This picks a stable
    pseudo-random code in the ordinary discrete range, avoiding the reserved
    ones: 1200 (US VFR), 7000 (European VFR), 7500 hijack, 7600 radio failure,
    7700 emergency, and anything ending in 00 which is normally a block header.
    """
    seed = _stable(session.aircraft.callsign.written, airport_ident)
    reserved = {"1200", "7000", "7500", "7600", "7700", "2000", "0000"}
    for attempt in range(64):
        digits = []
        value = seed + attempt * 7919
        for _ in range(4):
            digits.append(str(value % 8))
            value //= 8
        code = "".join(reversed(digits))
        if code in reserved or code.endswith("00") or code.startswith("0"):
            continue
        return code
    return "4321"


DEPARTING_WITHIN_NM = 40.0


def _airborne(state) -> bool:
    """Whether the aircraft is flying.

    ``None`` means no simulator, and an aircraft nobody can see is treated as
    being on the ground: that is where a flight starts, and refusing a taxi
    clearance to somebody testing the radio without a sim would be worse than
    the mistake this exists to prevent.
    """
    return state is not None and not getattr(state, "on_ground", True)


def _similar(a: str, b: str) -> float:
    """Cheap similarity for matching a mis-heard identifier."""
    if not a or not b:
        return 0.0
    from difflib import SequenceMatcher

    return SequenceMatcher(None, a, b).ratio()

__all__ = [
    "log",
    "Reply",
    "_ASKS_FOR_RUNWAY",
    "_SIDES",
    "_DEGREE_WORDS",
    "_TENS",
    "_side_said",
    "_degrees_said",
    "_TAXIWAYS",
    "_stable",
    "taxi_route",
    "_EXIT_WORDS",
    "squawk_for",
    "DEPARTING_WITHIN_NM",
    "_airborne",
    "_similar",
    "hashlib",
    "logging",
    "math",
    "re",
    "time",
    "dataclass",
    "Any",
    "decision",
    "terrain",
    "same_channel",
    "DEFAULT_TRANSITION_FT",
    "NavDB",
    "Station",
    "bearing_deg",
    "haversine_nm",
    "normalize_runway",
    "transition_altitude",
    "transition_level",
    "vfr_region",
    "sp",
    "VISUAL",
    "Approach",
    "Choice",
    "FieldApproaches",
    "asked_for",
    "choose",
    "legacy_kind",
    "plan_approach",
    "agrees",
    "choose_join",
    "circuit_for",
    "runway_length_nm",
    "CLIMBING_FPM",
    "DEPARTURE_REACHING_FT",
    "LAST_STEP_FT",
    "centre_step",
    "departure_step",
    "departure_top",
    "initial_altitude",
    "is_oceanic",
    "final_for",
    "_REPORT_WORD",
    "Intent",
    "ParsedIntent",
    "Phraseology",
    "Weather",
    "_join",
    "language_of",
    "procedure_heard",
    "procedure_near",
    "on_course_fix",
    "sid_first_fix",
    "written_procedure",
    "ATTEMPTS_BEFORE_REISSUE",
    "PROMPTS_BEFORE_REISSUE",
    "FlightSession",
    "Instruction",
    "Phase",
    "nearly_matches",
    "station_key",
    "say_words",
]
