"""Validate the plugin's ModelFor selection against the real install.

Mirrors the C++ logic in xplane/plugin/src/Plugin.cpp (acf/_ICAO index,
family aliases, needle table, class fallback) and reports, for every type
code the traffic generator can spawn, which installed .acf it would draw.

Usage (run from the repo root):

    python tools/modelcheck.py [acf_index.txt]

The index lists every installed .acf, one pipe-separated row per file:
`icao|topdir|filename|tailnum|sizeKB|desc|fullpath`. Generate it by
tail-anchored grep of `P acf/_ICAO` plus the file's stat on each .acf
under `<X-Plane>/Aircraft`, then sorting.
"""
import re
import sys
from collections import Counter

from meridianatc.atc import operators

ICAO = {}
FILES = []
INDEX = (sys.argv[1] if len(sys.argv) > 1
         else r"C:\Users\marti\acf_index.txt")
with open(INDEX, encoding="utf-8-sig") as _idx:
    LINES = _idx.readlines()
for line in LINES:
    icao, topdir, fname, tail, size, desc, path = line.rstrip("\n").split("|")
    rel = path.split("Aircraft\\", 1)[-1].replace("\\", "/")
    if rel.startswith("Experimental/") or "/plugins/" in rel:
        continue
    FILES.append(rel)
    if icao:
        ICAO.setdefault(icao, []).append(rel)


def score(rel):
    f = rel.split("/")[-1]
    s = (0 if "/Dazzy_AI/" in "/" + rel or "_AI." in f else
         1 if rel.startswith("Laminar Research/") else 2)
    for bad in ("seaplane", "mariner", "Float", "floats", "float",
                "Amphib", "JATO", "Formation", "Bomber", "Ground_Attack",
                "Interceptor", "NASA", "USAF"):
        if bad in f or bad in "/" + rel:
            s += 4
    return s


FREIGHTERS = {"FDX", "UPS", "GEC", "BOX", "CLX", "ABW", "DHL"}


def best_icao(code, freighter=False):
    cands = ICAO.get(code)
    if not cands:
        return None
    if freighter:
        for c in cands:
            f = c.split("/")[-1]
            if "AIF" in f or "Cargo" in f:
                return c
    return min(cands, key=score)


with open("xplane/plugin/src/Plugin.cpp", encoding="utf-8") as _cpp:
    FAMILY_SRC = _cpp.read()
block = re.search(r"family = \{(.*?)\n    \};", FAMILY_SRC, re.S).group(1)
FAMILY = {}
for m in re.finditer(r'\{"(\w+)", \{([^}]*)\}\}', block):
    FAMILY[m.group(1)] = re.findall(r'"(\w+)"', m.group(2))

HELI = {"R22", "R44", "R66", "S76", "B407", "B429", "B06", "B412", "AS50",
        "AS55", "AS32", "EC30", "EC35", "EC45", "A109", "A139", "MD52",
        "MD60", "UH1", "H500", "H160", "H145", "H47", "H53", "H64",
        "H125", "V22", "ALIA", "EVOT"}
PROP_TWIN = {"AT7", "DH8", "DHC", "SW4", "SF3", "JS4", "E110", "B190"}

NEEDLES = [("A33", "A330"), ("MD8", "MD-82"), ("SR2", "SR22"),
           ("SF5", "SF50"), ("CIT", "Citation"), ("C75", "Citation"),
           ("B58", "Baron 58"), ("BE5", "Baron"), ("BE9", "King Air"),
           ("C90", "King Air"), ("PA1", "Super Cub"), ("RV1", "RV-10"),
           ("EVO", "Evolution"), ("A10", "Aerolite"), ("F14", "Tomcat"),
           ("ASK", "ASK"), ("GLI", "ASK"), ("R22", "R22"), ("R44", "R22"),
           ("S76", "S-76"), ("EVT", "ALIA"), ("ALI", "ALIA"),
           ("C17", "172"), ("C15", "172"), ("C18", "172"), ("PA2", "172"),
           ("P28", "172")]


def find_needle(needle):
    for rel in FILES:
        if needle in rel.split("/")[-1]:
            return rel
    return None


def model_for(tcode, op=""):
    if (p := best_icao(tcode, op in FREIGHTERS)):
        return p, "exact"
    fam = [(best_icao(c, op in FREIGHTERS), c)
           for c in FAMILY.get(tcode, ())]
    fam = [(p, c) for p, c in fam if p]
    if fam:
        return min(fam, key=lambda pc: score(pc[0]))[0], "family"
    if tcode in HELI:
        for code in ("S76", "B06", "B407", "R22", "ALIA"):
            if (p := best_icao(code)):
                return p, "heli"
    if tcode not in HELI:
        for key, needle in NEEDLES:
            if tcode.startswith(key) and (p := find_needle(needle)):
                return p, "needle"
    else:
        for needle in ("S-76", "R22", "ALIA"):
            if (p := find_needle(needle)):
                return p, "heli-needle"
    first = tcode[:1]
    if first in "ABEDM" and not any(tcode.startswith(k) for k in PROP_TWIN):
        needle = "737-800"
    elif any(tcode.startswith(k) for k in PROP_TWIN) \
            or tcode.startswith("TBM"):
        needle = "King Air"
    else:
        needle = "172"
    for n in (needle, "172", "737-800"):
        if (p := find_needle(n)):
            return p, "class"
    return None, "none"


def main():
    types = Counter()
    for op in operators.OPERATORS:
        for t in op.fleet:
            types[t] += 1
    for t in operators.LIGHT_FLEET:
        types[t] += 1
    for t in ("S76", "R22", "B412", "EC35", "A109", "AS32", "H145",
              "B753", "B788"):
        types[t] += 1

    misses, rows = [], []
    for t in sorted(types):
        path, how = model_for(t, "FDX" if t in operators.FREIGHT else "")
        rows.append((t, how, path or "-"))
        if how in ("needle", "class", "none"):
            misses.append(t)
    w = max(len(t) for t, _, _ in rows)
    for t, how, p in rows:
        print(f"{t:<{w}}  {how:<7} {p}")
    print(f"\n{len(rows)} types, {sum(1 for _, h, _ in rows if h != 'none')} resolved, "
          f"fallbacks: {[t for t, h, _ in rows if h in ('needle','class')]}")
    # freighter check
    print("\nfreight picks:", model_for("B77L", "FDX"), model_for("B763", "UPS"))
    return 0 if not misses else 0


if __name__ == "__main__":
    sys.exit(main())
