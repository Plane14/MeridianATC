"""The first officer's checklists, as files a pilot can write.

The flows for each class of aeroplane (``aircraft_class.CLASSES``) ship in
``meridianatc/atc/checklists/<class>.json``. A pilot's own go in the data root,
``data/checklists/<ICAO type>.json`` for one aeroplane ("C172.json") or
``data/checklists/<class>.json`` for every aeroplane of a class. The format
is documented in ``meridianatc/atc/checklists/README.md``.

What wins, from the bottom up: the shipped class file, the pilot's class
file, the pilot's type file. A file replaces a checklist it names, whole,
and leaves the others alone, so a type file can hold one checklist and still
fly with the class's other ten; a checklist set to ``null`` is removed.
Replacing item by item was the alternative, and a pilot reading their own
file could not then tell what the first officer would actually say.

A study-level add-on drives its own switches rather than the simulator's,
so on a Fenix A320 the landing light can be on in the cockpit and off in
LIGHT LANDING. Those add-ons ship a file in ``checklists/types/``, matched
on the simulator's title (``"match"``, any of the fragments), whose
``"no_check"`` takes the checks off the shipped items for the variables
the add-on does not publish. A pilot's own files come after and may check
them again. Between the shipped class file and the pilot's, so: shipped
class, matched add-on, own class, own type.

The files are read again whenever one has changed (by modification time),
so a file saved in the middle of a session counts at the next checklist,
without a restart. A mistake in a file is reported, never fatal: an item
whose check names a variable the program does not read keeps its words and
loses only its check, and a file that is not JSON is skipped whole -- in
both cases a ``checklist_file`` row in ``logs crew`` says what and where.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import math
import threading
from dataclasses import dataclass, field
from pathlib import Path

from ..paths import data_root
from ..sim.state import AircraftState
from .aircraft_class import CLASSES

log = logging.getLogger(__name__)

SHIPPED = Path(__file__).with_name("checklists")
# The study-level add-ons' files, matched on the simulator's title.
TYPES = SHIPPED / "types"

#: The checklists every class with checklists ships, in the order a flight
#: meets them. A file may add others, which are started by name.
STANDARD_CHECKLISTS = ("preflight", "before_start", "after_start", "taxi",
                       "before_takeoff", "after_takeoff", "cruise", "descent",
                       "approach", "before_landing", "after_landing",
                       "shutdown")

#: Which flight a checklist or an item belongs to.
RULES = ("any", "vfr", "ifr")

#: How an item's check compares the variable with what it expects.
#:   eq / ne   equal / not equal to ``value``
#:   in        one of the list ``value``
#:   min / max at least / at most ``value``
#:   near      within ``tolerance`` of ``value``
OPERATORS = ("eq", "ne", "in", "min", "max", "near")

#: What ``value`` may name instead of a number: what the controller gave,
#: or what the pilot just answered.
REFERENCES = ("$qnh", "$squawk", "$altitude", "$heading", "$frequency",
              "$runway", "$said")

#: The kinds of answer an item may accept (Phase 1 parses them).
ANSWERS = ("check", "set", "on", "off", "up", "down", "green", "complete",
           "value")

#: The variables a check may name: every field of the aircraft state.
VARIABLES = frozenset(f.name for f in dataclasses.fields(AircraftState))


def own_folder() -> Path:
    return data_root() / "checklists"


@dataclass(frozen=True)
class Check:
    var: str
    op: str = "eq"
    value: object = True
    tolerance: float = 0.0


@dataclass(frozen=True)
class Item:
    #: What the first officer reads, by language; "en" is the fallback.
    challenge: dict
    #: What a pilot says back, by language: the words the first officer
    #: expects, and what it says itself when the pilot asks for it.
    response: dict
    accept: tuple[str, ...] = ("check",)
    rules: str = "any"
    #: The conditions the aeroplane must agree with; one challenge can
    #: name several switches ("APU generator and bleed"), so the check
    #: is a conjunction. An empty tuple is an answer-only item.
    checks: tuple[Check, ...] = ()
    #: Why the item is there, or why it is not sure it belongs.
    note: str = ""

    @property
    def check(self) -> Check | None:
        """The first condition, or None -- for the callers that only
        need to name a variable (notes, disagreement phrasing)."""
        return self.checks[0] if self.checks else None

    def challenge_in(self, language: str) -> str:
        return _in(self.challenge, language)

    def response_in(self, language: str) -> str:
        return _in(self.response, language)


@dataclass(frozen=True)
class Checklist:
    name: str
    title: dict
    items: tuple[Item, ...]
    rules: str = "any"
    #: The file it came from.
    origin: str = ""

    def title_in(self, language: str) -> str:
        return _in(self.title, language) or self.name.replace("_", " ")

    def for_rules(self, rules: str) -> tuple[Item, ...]:
        """The items for a VFR or an IFR flight."""
        return tuple(i for i in self.items if i.rules in ("any", rules))


@dataclass
class Book:
    """Every checklist for one aeroplane, after the files are merged."""

    aircraft_class: str
    type_code: str
    checklists: dict[str, Checklist] = field(default_factory=dict)
    #: The files that were read, lowest first.
    files: list[str] = field(default_factory=list)
    #: What was wrong in them, one sentence each.
    problems: list[str] = field(default_factory=list)
    #: The variables an add-on's file took the checks off.
    unchecked: frozenset = frozenset()

    def for_rules(self, rules: str) -> dict[str, Checklist]:
        return {name: c for name, c in self.checklists.items()
                if c.rules in ("any", rules)}


def _in(words, language: str) -> str:
    if isinstance(words, str):
        return words
    words = words or {}
    return words.get((language or "en")[:2]) or words.get("en") or ""


def _words(value) -> dict:
    if isinstance(value, str):
        return {"en": value}
    if isinstance(value, dict):
        return {k[:2]: v for k, v in value.items() if isinstance(k, str) and isinstance(v, str) and v.strip()}
    return {}


def _checks(raw, where: str, problems: list[str]) -> tuple[Check, ...]:
    """One check dict, or a list of them for a step that names several
    switches -- every condition must hold for the item to pass."""
    if raw is None:
        return ()
    if isinstance(raw, list):
        return tuple(c for c in (
            _check(e, f"{where} check {i + 1}", problems)
            for i, e in enumerate(raw)) if c is not None)
    check = _check(raw, where, problems)
    return (check,) if check is not None else ()


def _check(raw, where: str, problems: list[str]) -> Check | None:
    if not isinstance(raw, dict) or "var" not in raw:
        problems.append(f"{where}: a check needs a \"var\"")
        return None
    var, op = str(raw["var"]), str(raw.get("op", "eq"))
    if var not in VARIABLES:
        problems.append(f"{where}: \"{var}\" is not a variable the program "
                        "reads; the item is kept without its check")
        return None
    if op not in OPERATORS:
        problems.append(f"{where}: \"{op}\" is not one of {', '.join(OPERATORS)}")
        return None
    value = raw.get("value", True)
    if isinstance(value, str) and value.startswith("$") \
            and value not in REFERENCES:
        problems.append(f"{where}: \"{value}\" is not one of "
                        f"{', '.join(REFERENCES)}")
        return None
    try:
        tolerance = float(raw.get("tolerance", 0.0))
        if not math.isfinite(tolerance) or tolerance < 0:
            raise ValueError
    except (TypeError, ValueError):
        problems.append(f"{where}: the tolerance must be finite and nonnegative; the check is skipped")
        return None
    if op == "in" and not isinstance(value, list):
        problems.append(f"{where}: an in check needs a list of values")
        return None
    if (op in ("min", "max", "near")
            and not (isinstance(value, str) and value in REFERENCES)
            and (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value))):
        problems.append(f"{where}: {op} needs a finite numeric value or reference")
        return None
    return Check(var=var, op=op, value=value, tolerance=tolerance)


def _rules(value, where: str, problems: list[str]) -> str:
    rules = str(value or "any").lower()
    if rules not in RULES:
        problems.append(f"{where}: rules \"{rules}\" is not any, vfr or ifr")
        return "any"
    return rules


def _item(raw, where: str, problems: list[str]) -> Item | None:
    if not isinstance(raw, dict) or not raw.get("challenge"):
        problems.append(f"{where}: an item needs a \"challenge\"")
        return None
    accept = raw.get("accept") or ["check"]
    if isinstance(accept, str):
        accept = [accept]
    if not isinstance(accept, list) or not all(isinstance(a, str) for a in accept):
        problems.append(f"{where}: accept must be a list of answer names")
        accept = ["check"]
    unknown = [a for a in accept if a not in ANSWERS]
    if unknown:
        problems.append(f"{where}: {', '.join(unknown)} is not an answer "
                        f"({', '.join(ANSWERS)})")
        accept = [a for a in accept if a in ANSWERS] or ["check"]
    challenge, response = _words(raw["challenge"]), _words(raw.get("response", "check"))
    if not challenge.get("en") or not response.get("en"):
        problems.append(f"{where}: challenge and response need English fallback text")
        return None
    return Item(challenge=challenge,
                response=response,
                accept=tuple(accept),
                rules=_rules(raw.get("rules"), where, problems),
                checks=_checks(raw.get("check"), where, problems),
                note=str(raw.get("note", "")))


def _checklist(name: str, raw, origin: str,
               problems: list[str]) -> Checklist | None:
    where = f"{Path(origin).name}, {name}"
    if not isinstance(raw, dict) or not isinstance(raw.get("items"), list):
        problems.append(f"{where}: a checklist needs a list of \"items\"")
        return None
    items = []
    for n, one in enumerate(raw["items"], 1):
        item = _item(one, f"{where}, item {n}", problems)
        if item is not None:
            items.append(item)
    return Checklist(name=name, title=_words(raw.get("title", "")),
                     items=tuple(items),
                     rules=_rules(raw.get("rules"), where, problems),
                     origin=origin)


def _read(path: Path, problems: list[str]) -> dict | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None
    except OSError:
        problems.append(f"{path.name}: cannot read file; check permissions")
        return None
    except ValueError as exc:
        problems.append(f"{path.name}: not JSON ({exc}); the file is skipped")
        return None
    if path.parent == TYPES and isinstance(data, dict):
        # An add-on's file need not carry checklists of its own.
        data.setdefault("checklists", {})
    if not isinstance(data, dict) or not isinstance(data.get("checklists"), dict):
        problems.append(f"{path.name}: no \"checklists\" in it; the file is skipped")
        return None
    matches = data.get("match", [])
    if not isinstance(matches, list) or not all(isinstance(m, str) and m.strip() for m in matches):
        problems.append(f"{path.name}: match must be a list of nonempty title fragments; the file is skipped")
        return None
    unchecked = data.get("no_check", [])
    if not isinstance(unchecked, list) or not all(isinstance(v, str) for v in unchecked):
        problems.append(f"{path.name}: no_check must be a list of variable names")
        data["no_check"] = []
    return data


def _compact_key(text: str) -> str:
    """Aircraft title text with punctuation and spacing removed."""
    return "".join(ch for ch in (text or "").lower() if ch.isalnum())


# Fragments too broad to identify one aircraft or add-on. The generated
# type files carry every token of a package title; manufacturer names and
# words such as "flight" otherwise make a Citation, a Caravan and a C172
# all wear the same POH card.
_GENERIC_MATCHES = frozenset({
    "a2a", "aerosoft", "air", "airbus", "asobo", "beechcraft", "bell",
    "black", "boeing", "cessna", "cirrus", "diamond", "douglas", "embraer",
    "fenix", "flight", "flightsim", "flybywire", "fsreborn", "grumman",
    "ifly", "inibuilds", "jets", "just", "king", "leonardo", "mcdonnell",
    "microsoft", "nemeth", "piper", "pmdg", "robinson", "square", "studio",
    "taog", "toliss", "xplane", "super",
})

# Vendor names are evidence that a vendor edition is present, never evidence
# of which aircraft that edition is. Model-family words shared by distinct
# aircraft are likewise too broad on their own.
_VENDOR_MATCHES = frozenset({
    "blacksquare", "flightfx", "pmdg", "fenix", "flybywire", "ifly",
    "a2a", "fsreborn", "inibuilds", "toliss", "aerosoft", "nemeth",
    "taog", "justflight", "flightsimstudio",
})
_FAMILY_MATCHES = frozenset({"citation"})


def _match_fragment(fragment: str, title_key: str, code_key: str) -> bool:
    """Whether one declared fragment is specific evidence for this aircraft."""
    key = _compact_key(fragment)
    if not key or key in _GENERIC_MATCHES \
            or key in _VENDOR_MATCHES or key in _FAMILY_MATCHES:
        return False
    if code_key and key == code_key:
        # The ICAO type is exact evidence even when it is short -- no
        # substring matching, so C172 does not pull in every "172" title.
        return True
    # Title fragments need enough of the model name to mean one aircraft:
    # bare numbers such as 172/737/600 and one-letter abbreviations were
    # matching across whole manufacturers. Four alphanumeric characters
    # keeps SR22, H125 and C172 while discarding those collisions.
    if len(key) < 4:
        return False
    at = title_key.find(key)
    while at >= 0:
        end = at + len(key)
        # A family fragment is not a variant: ``a330`` inside
        # ``a330-900neo`` must not put the A330-300's POH on the -900.
        # Compact text loses the hyphen, so a digit immediately after the
        # fragment is the variant boundary.
        if end >= len(title_key) or not key[-1].isdigit() \
                or not title_key[end].isdigit():
            return True
        at = title_key.find(key, at + 1)
    return False


def _vendor_required(matches: list[str]) -> str:
    """The add-on vendor a file's own match list declares, normalized.

    Vendor editions share the aircraft type and much of the model name with
    the stock aircraft. A Black Square Baron is still "BE58", so the type
    code alone can never distinguish it from the X-Plane Baron; applying
    its preflight card to the stock aeroplane is exactly the wrong POH.
    Where the file declares a vendor, that vendor must appear in the
    simulator's title before any of its model fragments count.
    """
    keys = {_compact_key(fragment) for fragment in matches}
    vendors = ("blacksquare", "flightfx", "pmdg", "fenix", "flybywire",
               "ifly", "a2a", "fsreborn", "inibuilds", "toliss",
               "aerosoft", "nemeth", "taog")
    for vendor in vendors:
        if any(key == vendor or key.startswith(vendor + "-")
               for key in keys):
            return vendor
    if {"just", "flight"} <= keys:
        return "justflight"
    if {"flightsim", "studio"} <= keys:
        return "flightsimstudio"
    return ""


def _add_ons(title: str, type_code: str = "") -> list[Path]:
    """The add-on files whose specific fragments name this aircraft.

    Matching is deliberately stricter than a raw substring of the title:
    manufacturer names, package vendors and bare numeric tokens are not a
    POH. A file must name a model-specific fragment (normalised so
    ``Cessna 172`` and ``cessna-172`` are the same evidence) or the exact
    ICAO type code, which is what no-sim aircraft often carry when no rich
    title is reported.
    """
    title_key = _compact_key(title)
    code_key = _compact_key(type_code)
    if not TYPES.is_dir():
        return []
    found = []
    for path in sorted(TYPES.glob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            found.append(path)        # read again, and reported, by _merge
            continue
        if not isinstance(data, dict):
            found.append(path)
            continue
        matches = data.get("match", [])
        if not isinstance(matches, list) or not all(isinstance(m, str) and m.strip() for m in matches):
            found.append(path)
            continue
        vendor = _vendor_required(matches)
        if vendor and vendor not in title_key:
            continue
        if any(_match_fragment(m, title_key, code_key) for m in matches):
            found.append(path)
    return found


def _files(aircraft_class: str, type_code: str,
           own: Path | None = None, title: str = "") -> list[Path]:
    own = own if own is not None else own_folder()
    files = [SHIPPED / f"{aircraft_class}.json",
             *_add_ons(title, type_code), own / f"{aircraft_class}.json"]
    code = (type_code or "").strip().upper()
    if code and code.lower() not in CLASSES:
        files.append(own / f"{code}.json")
    return [p for p in files if p.is_file()]


def _merge(aircraft_class: str, type_code: str, files: list[Path]) -> Book:
    book = Book(aircraft_class=aircraft_class,
                type_code=(type_code or "").upper())
    unchecked: set[str] = set()
    shipped_done = False
    for path in files:
        if not shipped_done and path.parent not in (TYPES, SHIPPED):
            # The pilot's own files from here. What an add-on's file takes
            # off is what shipped; a pilot's file may check it again.
            _uncheck(book, unchecked)
            shipped_done = True
        data = _read(path, book.problems)
        if data is None:
            continue
        book.files.append(str(path))
        if path.parent == TYPES:
            for var in data.get("no_check") or ():
                if var in VARIABLES:
                    unchecked.add(var)
                else:
                    book.problems.append(
                        f"{path.name}: \"{var}\" in no_check is not a "
                        "variable the program reads")
        for name, raw in data["checklists"].items():
            if name.startswith("_"):
                continue
            if raw is None:
                book.checklists.pop(name, None)
                continue
            checklist = _checklist(name, raw, str(path), book.problems)
            if checklist is not None:
                book.checklists[name] = checklist
    if not shipped_done:
        _uncheck(book, unchecked)
    book.unchecked = frozenset(unchecked)
    return book


def _uncheck(book: Book, variables: set[str]) -> None:
    """Take the checks on ``variables`` off every checklist in ``book``."""
    for name, checklist in list(book.checklists.items()):
        items = []
        for item in checklist.items:
            remaining = tuple(c for c in item.checks
                              if c.var not in variables)
            if len(remaining) == len(item.checks):
                items.append(item)
                continue
            items.append(dataclasses.replace(
                item, checks=remaining,
                note=(item.note + "; " if item.note else "")
                + "unchecked: the add-on does not publish it"))
        book.checklists[name] = dataclasses.replace(
            checklist, items=tuple(items))


_cache: dict[tuple, tuple[tuple, Book]] = {}
_lock = threading.Lock()


def load(aircraft_class: str, type_code: str = "",
         own: Path | None = None, title: str = "") -> Book:
    """Every checklist for this aeroplane, read again if a file changed.

    ``title`` is the simulator's title for the model, which an add-on's
    file is matched on."""
    if aircraft_class not in CLASSES:
        raise ValueError(f"{aircraft_class!r} is not a class of aeroplane")
    files = _files(aircraft_class, type_code, own, title)
    stamp = []
    for path in files:
        try:
            stat = path.stat()
            stamp.append((str(path), stat.st_mtime_ns, stat.st_size))
        except OSError:
            stamp.append((str(path), None, None))
    stamp = tuple(stamp)
    key = (aircraft_class, (type_code or "").upper(), str(own),
           tuple(str(p) for p in files))
    with _lock:
        cached = _cache.get(key)
        if cached is not None and cached[0] == stamp:
            return cached[1]
    book = _merge(aircraft_class, type_code, files)
    with _lock:
        _cache[key] = (stamp, book)
    if book.problems:
        from ..logs import CREW

        for problem in book.problems:
            CREW.write("checklist_file", problem=problem,
                       aircraft_class=aircraft_class, type=book.type_code)
        log.warning("checklist files for %s %s: %s", aircraft_class,
                    book.type_code, "; ".join(book.problems))
    return book


__all__ = ["ANSWERS", "Book", "Check", "Checklist", "Item", "OPERATORS",
           "STANDARD_CHECKLISTS",
           "REFERENCES", "RULES", "SHIPPED", "VARIABLES", "load", "own_folder"]
