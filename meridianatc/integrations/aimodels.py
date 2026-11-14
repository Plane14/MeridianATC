"""Every aircraft model the simulator has installed, by ICAO type.

The table the traffic used to pick a drawn model was written down: each
ICAO type, a handful of titles -- the simulator's own AI models, a flyable
one or two -- and anything not in it drew the generic stand-in of its
size. That table only ever named the models one installation happened to
have. A pilot with FSLTL has two thousand more, a pilot with AIG another
set, and a helicopter flight drew an A320 because the table named no
rotorcraft at all.

The simulator itself can say what it has:
``SimConnect_EnumerateSimObjectsAndLiveries`` answers with every aircraft
title it can create -- the stock AI models, the flyable ones, every
FSLTL, AIG or third-party pack's -- each with the liveries it was
installed with. This is that answer as a catalogue: every listed title,
the ICAO type it is read as where the name gives one, whether it is a
helicopter, and the airline a pack model is painted for where the title
says so.

What it is for: :meth:`meridianatc.integrations.aitraffic.SimTraffic._titles_at`
asks the catalogue first, so an invented or live aeroplane is drawn with
a model that is actually installed -- its own type where the simulator
has one, rotorcraft included, and the nearest of the same kind where it
does not. The written-down table stays as the fallback for a simulator
that never answered, and a title the simulator then refuses is still
never asked for twice.

Nothing is asked of the disk, and nothing is written: the answer comes
from the simulator, once a session, on the worker that lists the AI
liveries -- :meth:`meridianatc.engine.core._read_ai_liveries`.
"""

from __future__ import annotations

import logging
import re

log = logging.getLogger(__name__)

#: ICAO designators that are rotorcraft, so a helicopter flight is drawn
#: with a helicopter -- any helicopter -- rather than the airliner
#: stand-in of its weight class.
_HELICOPTER_TYPES = frozenset({
    "R22", "R44", "R66", "G2CA",
    "B06", "B47G", "B407", "B412", "B427", "B429", "B430", "B505", "UH1",
    "EC20", "EC25", "EC30", "EC35", "EC45", "EC55", "H160", "H500", "H64",
    "AS32", "AS3B", "AS50", "AS55", "AS65",
    "S61", "S76", "S92", "SK76",
    "A109", "A119", "A129", "A139", "A149", "A169", "A189", "A6",
    "MD52", "MD60", "EXPL", "GUMA", "LAMA", "NH90", "TIGR",
})

#: How a title that says so is a helicopter, where the type guess does
#: not: every one of these in a title means rotorcraft.
_HELICOPTER_WORDS = re.compile(
    r"helicopter|heli|robinson|cabri|guimbal|r22|r44|r66|h125|h145|h160|"
    r"bell ?407|bell ?429|bell ?505|jet ?ranger|ec1[34]5|as3[0-9]0|s-?76|"
    r"aw ?139|md ?5[0-9]0", re.I)

#: A pack title's airline is its last word: ``FSLTL_B738_EZY`` is
#: easyJet's. Where the last word is the pack's unpainted model of the
#: type there is no airline -- FSLTL names those with a run of Zs.
_GENERIC_PAINT = re.compile(r"^(Z+|GEN|WHITE|BLANK|UNPAINTED|DEFAULT|"
                            r"NO[ _]?LIVERY)", re.I)

#: A pack prefix: the titles that carry "<pack>_<type>_<airline>".
_AI_PACKS = {"FSLTL", "AIG", "AIGAIM", "AIM", "FAIB", "FSP", "IVAO",
             "PAI", "UTT", "BVAI", "JCAI", "AI"}

#: Names that mean a model built for AI rather than a flyable one:
#: cheaper to have a dozen of, which is what a busy apron is. A flyable
#: add-on draws the type too, at the cost a cockpit model carries, so it
#: sorts after them.
_AI_WORDS = ("passiveaircraft", "fsltl_", "aig", "_ai_", "aicraft",
             "traffic", "ivao", "vatsim")

#: What a named model is, where the type guess finds no designator:
#: ordered, first match wins -- "A320 NEO" is an A20N and only after it
#: is an A320, and the longer variants always precede their prefixes
#: ("737-800" before "737-8", or every -800 would come out a MAX).
_NAME_TYPES: tuple[tuple[str, str], ...] = (
    ("A380", "A388"), ("A350", "A359"), ("A340", "A346"),
    ("A330-900", "A339"), ("A330NEO", "A339"), ("A330 NEO", "A339"),
    ("A330", "A333"), ("A310", "A306"),
    ("A321NEO", "A21N"), ("A321 NEO", "A21N"),
    ("A320NEO", "A20N"), ("A320 NEO", "A20N"),
    ("A319NEO", "A19N"), ("A319 NEO", "A19N"),
    ("A321", "A321"), ("A320", "A320"), ("A319", "A319"), ("A318", "A318"),
    ("787-10", "B78X"), ("787-9", "B789"), ("787-8", "B788"),
    ("787", "B789"),
    ("777-300", "B77W"), ("777-200", "B77L"), ("777", "B77W"),
    ("767", "B763"), ("757-300", "B753"), ("757-200", "B752"),
    ("757", "B752"),
    ("747-8", "B748"), ("747", "B744"),
    ("737 MAX 9", "B39M"), ("737 MAX9", "B39M"),
    ("737 MAX 8", "B38M"), ("737 MAX8", "B38M"),
    ("737-900", "B739"), ("737-800", "B738"), ("737-700", "B737"),
    ("737-600", "B736"), ("737-9", "B39M"), ("737-8", "B38M"),
    ("737", "B738"), ("717", "B712"),
    ("E195", "E195"), ("E190", "E190"), ("E175", "E75L"),
    ("ERJ", "E145"), ("CRJ", "CRJ9"),
    ("ATR 72", "AT76"), ("ATR72", "AT76"), ("ATR 42", "AT46"),
    ("ATR42", "AT46"),
    ("DHC-8", "DH8D"), ("DASH8", "DH8D"), ("DASH 8", "DH8D"),
    ("Q400", "DH8D"), ("DHC6", "DHC6"), ("TWIN OTTER", "DHC6"),
    ("SKYHAWK", "C172"), ("C172", "C172"), ("C152", "C152"),
    ("C182", "C182"), ("C208", "C208"), ("CARAVAN", "C208"),
    ("CITATION CJ2", "C25A"), ("CITATION", "C25A"),
    ("BONANZA", "BE36"), ("BARON", "BE58"), ("KING AIR", "B350"),
    ("DA62", "DA42"), ("DA42", "DA42"), ("DA40", "DA40"),
    ("SR22", "SR22"), ("VISION", "SF50"), ("SF50", "SF50"),
    ("TBM", "TBM9"), ("PC-12", "PC12"), ("PC12", "PC12"),
    ("PC-24", "PC24"), ("PC24", "PC24"), ("CONCORDE", "CONC"),
    # Rotorcraft: names where the type guess finds nothing.
    ("BELL 407", "B407"), ("B407", "B407"), ("BELL 429", "B429"),
    ("BELL 505", "B505"), ("JET RANGER", "B06"), ("BELL 206", "B06"),
    ("H160", "H160"), ("H125", "H125"), ("H145", "EC45"),
    ("AS350", "AS50"), ("AS355", "AS55"), ("EC135", "EC35"),
    ("EC145", "EC45"), ("R66", "R66"), ("R44", "R44"), ("R22", "R22"),
    ("CABRI", "G2CA"), ("S-76", "S76"), ("S76", "S76"),
    ("AW139", "A139"), ("A139", "A139"), ("MD500", "H500"),
)


def is_helicopter_type(type_code: str) -> bool:
    """Whether an ICAO designator is a rotorcraft's."""
    return (type_code or "").upper() in _HELICOPTER_TYPES


def type_of_title(title: str) -> str:
    """The ICAO designator a model title is read as, or "".

    Three passes, each weaker than the last: the whole title the way an
    injector writes its model field (``AC_MODEL_B738``, ``B738``), then
    the named-model table on the whole title (``A320 NEO`` is an A20N,
    which the word scan could never tell from an A320), then each
    ``_``-separated word (``FSLTL_B738_EZY``). A title none of them names
    keeps no type -- better than a wrong one.
    """
    from ..atc.traffic import observed_type

    text = (title or "").strip()
    if not text:
        return ""
    found = observed_type(text)
    if found:
        return found
    upper = text.upper()
    for name, code in _NAME_TYPES:
        if name in upper:
            return code
    for token in re.split(r"[_\s]+", text):
        found = observed_type(token)
        if found:
            return found
    return ""


class ModelInfo:
    """One installed title: its type, its liveries, what it is for."""

    __slots__ = ("title", "type_code", "liveries", "heli", "ai", "airline")

    def __init__(self, title: str, type_code: str = "",
                 liveries: tuple = (), heli: bool = False,
                 ai: bool = True, airline: str = "") -> None:
        self.title = title
        self.type_code = type_code
        self.liveries = liveries
        self.heli = heli
        self.ai = ai
        self.airline = airline


class Catalogue:
    """The installed aircraft models, by ICAO type and by title."""

    def __init__(self) -> None:
        self._by_title: dict[str, ModelInfo] = {}
        self._by_type: dict[str, list[ModelInfo]] = {}

    def __len__(self) -> int:
        return len(self._by_title)

    def __bool__(self) -> bool:
        return bool(self._by_title)

    def __contains__(self, title: str) -> bool:
        return title in self._by_title

    def info(self, title: str) -> ModelInfo | None:
        return self._by_title.get(title)

    @classmethod
    def from_pairs(cls, pairs) -> Catalogue:
        """Build the catalogue from the simulator's own answer: the
        (title, livery) list of ``enumerate_simobjects``."""
        found = cls()
        liveries: dict[str, list[str]] = {}
        for title, livery in pairs or ():
            if not title:
                continue
            liveries.setdefault(title, [])
            if livery:
                liveries[title].append(livery)
        for title, paint in liveries.items():
            type_code = type_of_title(title)
            lower = title.lower()
            heli = is_helicopter_type(type_code) \
                or bool(_HELICOPTER_WORDS.search(title))
            ai = any(word in lower for word in _AI_WORDS)
            found.add(ModelInfo(
                title, type_code=type_code,
                liveries=tuple(dict.fromkeys(paint)),
                heli=heli, ai=ai, airline=_airline_of(title, type_code)))
        return found

    def add(self, info: ModelInfo) -> None:
        if info.title in self._by_title:
            return
        self._by_title[info.title] = info
        if info.type_code:
            self._by_type.setdefault(info.type_code, []).append(info)

    def titles_for(self, type_code: str, airline: str = "") -> list[str]:
        """Installed titles for a type, best first: the airline's own
        painted model, then models that name no airline (the packs'
        generics and the simulator's own), AI models before flyable ones
        within each. Another airline's painted model is not here -- see
        :meth:`other_titles_for`."""
        found = self._by_type.get((type_code or "").upper(), [])
        airline = (airline or "").upper()
        painted = [m for m in found if m.airline and m.airline == airline]
        plain = [m for m in found if not m.airline]
        return ([m.title for m in _sorted(painted)]
                + [m.title for m in _sorted(plain)])

    def other_titles_for(self, type_code: str, airline: str = "") -> list[str]:
        """This type painted for somebody else: the last resort, after
        everything unbranded, because a wrong airline's paint is still a
        model of the type."""
        airline = (airline or "").upper()
        return [m.title for m in _sorted(
            m for m in self._by_type.get((type_code or "").upper(), [])
            if m.airline and m.airline != airline)]

    def heli_titles(self, airline: str = "") -> list[str]:
        """Every installed rotorcraft title, AI ones first: the stand-in
        for a helicopter type the simulator has no model of."""
        found = [m for m in self._by_title.values() if m.heli]
        airline = (airline or "").upper()
        painted = [m for m in found if m.airline and m.airline == airline]
        plain = [m for m in found if not m.airline]
        return ([m.title for m in _sorted(painted)]
                + [m.title for m in _sorted(plain)])

    def has_type(self, type_code: str) -> bool:
        return bool(self._by_type.get((type_code or "").upper()))


def _sorted(models: list[ModelInfo]) -> list[ModelInfo]:
    """AI models first, then shorter titles -- the pack's own model of a
    type before a skinned variant of somebody else's."""
    return sorted(models, key=lambda m: (not m.ai, len(m.title), m.title))


def _airline_of(title: str, type_code: str) -> str:
    """The airline a pack title is painted for, or "".

    Only titles built the way the AI packs name theirs --
    ``PACK_TYPE_AIRLINE`` -- carry one; a flyable model's title is its
    name, not a livery, and a livery is the enumeration's second field.
    """
    parts = title.split("_")
    if len(parts) < 3 or parts[0].upper() not in _AI_PACKS:
        return ""
    tail = parts[-1].upper()
    if not tail.isalpha() or not 3 <= len(tail) <= 5 \
            or _GENERIC_PAINT.match(tail):
        return ""
    return tail


__all__ = ["Catalogue", "ModelInfo", "is_helicopter_type", "type_of_title"]
