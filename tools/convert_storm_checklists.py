#!/usr/bin/env python
"""Convert StormATC flightchecklists into MeridianATC types/ files.

StormATC ships one file per aircraft (``data/flightchecklists/<id>.json``)
with PascalCase fields and a flat list of steps under a single Category.
MeridianATC's format is a ``checklists`` map of named challenge-response
lists; a file in ``checklists/types/`` is matched on the simulator title by
``match`` fragments and its checklists replace the class ones for that
aeroplane.

Mapping:

* ``Title`` -> ``challenge``; ``Description`` -> the ``note`` carries what
  does not fit the response line.
* ``ExpectedResponse`` -> ``response`` + the matching ``accept`` kinds;
  StormATC left it null everywhere, so the kind comes from ``Type``:
  Verification -> check, Configuration -> set/on/off, Action -> complete,
  Calculation -> value.
* ``Category`` ("Preflight" for every shipped file) -> a checklist named
  after it, lowercased; the pilot starts it by name or the class file may.
* ``match`` is built from the file's Id and Name tokens so the simulator
  title ("FlyByWire A32NX", "PMDG 737-800") finds it.
* ``SimVarBinding``/``EventBinding`` are null in every shipped file; where a
  file does bind one, the binding is dropped into the note rather than
  guessed into a variable the program does not read.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ACCEPTS = {
    "verification": ("check",),
    "configuration": ("set", "on", "off", "check"),
    "action": ("complete", "check", "on", "off"),
    "calculation": ("value", "check"),
}


def _accepts(step_type: str, expected: str | None) -> tuple[str, ...]:
    kinds = ACCEPTS.get((step_type or "").lower(), ("check",))
    words = set(re.findall(r"[a-z]+", (expected or "").lower()))
    if "value" in words or any(w.isdigit() for w in words):
        kinds = kinds + ("value",) if "value" not in kinds else kinds
    return kinds


def convert(path: Path) -> dict | None:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    steps = raw.get("Steps") or []
    if not steps:
        return None
    items = []
    for step in steps:
        title = str(step.get("Title") or "").strip()
        if not title:
            continue
        response = str(step.get("ExpectedResponse") or "").strip()
        description = str(step.get("Description") or "").strip()
        if not response:
            # The spoken answer is the description's tail where there is
            # one ("NAV", "ON", "SET"); else a bare "check" suffices.
            tail = description.rsplit(" ", 1)[-1] if description else ""
            response = tail.lower() if 0 < len(tail) <= 12 else "check"
        item = {"challenge": title, "response": response,
                "accept": list(_accepts(step.get("Type"),
                                        step.get("ExpectedResponse")))}
        if description and description.lower() != response.lower():
            item["note"] = description
        if step.get("IsOptional"):
            item["note"] = (item.get("note", "") + " Optional.").strip()
        if step.get("SimVarBinding") or step.get("EventBinding"):
            binding = step.get("SimVarBinding") or step.get("EventBinding")
            item["note"] = (item.get("note", "") +
                            f" Sim var bound in the source: {binding}."
                            ).strip()
        items.append(item)
    if not items:
        return None
    # StormATC's shipped files use the root Category "Preflight" for a
    # list that nevertheless continues through engine start, takeoff,
    # landing and shutdown. Those phases cannot be reconstructed safely
    # from the flat list, and keeping them in preflight made the FO read
    # airborne/landing items before departure. Keep the pre-start part;
    # MeridianATC's class cards own the later phases.
    cut = len(items)
    magnetos_seen = False
    start_words = ("engine master", "starter", "start engine", "engine start",
                   "ignition start")
    for index, item in enumerate(items):
        challenge = str(item.get("challenge") or "").lower()
        if any(word in challenge for word in start_words):
            cut = index
            break
        if "magnetos switch" in challenge or "magneto switch" in challenge:
            if magnetos_seen:
                cut = index
                break
            magnetos_seen = True
    items = items[:cut]
    if not items:
        return None
    category = str(raw.get("Category") or "preflight").strip().lower()
    name = str(raw.get("Name") or raw.get("Id") or path.stem).strip()
    match = {path.stem.lower()}
    match.update(re.findall(r"[a-z0-9]+", name.lower()))
    match.update(re.findall(r"[a-z0-9]+",
                            str(raw.get("Id") or "").lower()))
    # One-letter and numeric tokens match too much ("a", "250").
    match = sorted(m for m in match if len(m) >= 3)
    return {
        "_comment": ("Converted from StormATC data/flightchecklists/"
                     f"{path.name} ({name}). Verification steps keep their "
                     "challenge and note rather than a guessed simvar check. "
                     "Only the pre-start part is kept from StormATC's "
                     "flattened phase list."),
        "source": f"StormATC {name} checklist set",
        "match": match,
        "checklists": {
            category: {
                "title": {"en": category.replace("_", " ")},
                "items": items,
            },
        },
    }


def main(source: Path, out_dir: Path) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    written = 0
    for path in sorted(source.glob("*.json")):
        data = convert(path)
        if data is None:
            print(f"skip {path.name}: not convertible", file=sys.stderr)
            continue
        (out_dir / path.name).write_text(
            json.dumps(data, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        written += 1
    print(f"{written} files -> {out_dir}")
    return 0


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[1]
    source = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        root.parent / "StormATC" / "data" / "flightchecklists"
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else \
        root / "meridianatc" / "atc" / "checklists" / "types"
    raise SystemExit(main(source, out))
