"""X-Plane's terminal procedures: the CIFP files the sim itself reads.

The simulator's facility data is a SimConnect thing; X-Plane's equivalent
lives on disk. X-Plane 12 loads every airport's departures, arrivals and
approaches from ``CIFP/<ICAO>.dat`` -- a simplified comma-separated
rendering of the ARINC 424 PD/PE/PF records -- under ``Custom Data`` when a
data subscription is installed and under ``Resources/default data``
otherwise. The legs in those files name their fixes; the coordinates come
from ``earth_fix.dat`` and ``earth_nav.dat`` beside them, the runway ends
from the file's own ``RWY:`` records.

The decoded shape is :mod:`meridianatc.atc.procedure_legs`' -- the same
model the simulator's facility data decodes into -- so the First Officer,
the controller and the traffic all consume one structure whichever
simulator is flying. What a leg cannot say (there is no lat/lon for a fix
the navdata does not hold) is left at zero coordinates, which
:class:`ProcedureNavigator` already flies as "no geometry" rather than
guess.

Row layout (``SID``/``STAR``/``APPCH``), the positions the ARINC 424.20
field numbers land on after ``convert424toxp`` reorders them::

    seq,route_type,procedure,transition,fix,icao,section,subsection,
    desc,turn,rnp,path,?,navaid,nav_icao,nav_sect,nav_subsect,
    arc_radius,theta,rho,course,distance,alt_desc,alt1,alt2,trans_alt,
    speed_desc,speed,vertical_angle,tba,center_fix,cf_icao,cf_sect,
    cf_subsect,multi,gps,?,?

``RWY:`` rows carry the threshold coordinates in an optional ``;`` group:
``RWY:RW09L,<magvar>,<gap>,<tdze>,,<ils ident>,<type>, ;N51283900,W000290597,1014;``
and ``PRDAT:`` rows carry an RNAV approach's minima lines
(``LPV``/``LNAV/VNAV``/``LNAV``) inside the approach they sit inside --
they have no sequence field, so they are matched by file position.
"""

from __future__ import annotations

import logging
from pathlib import Path

from ..atc.approaches import Approach, FieldApproaches
from ..atc.procedure_legs import (
    ALT_AT,
    ALT_AT_OR_ABOVE,
    ALT_AT_OR_BELOW,
    ALT_IN_BETWEEN,
    ALT_NOT_USED,
    AirportProcedures,
    ProcedureLeg,
    ProcedureTransition,
    TerminalProcedure,
)

log = logging.getLogger(__name__)

_M_PER_NM = 1852.0

# Route types the CIFP rows use (ARINC 424 5.7): for a SID the low digits
# are the runway transition, the common portion and the enroute
# transition; for a STAR enroute first, common, then the runway
# transition. 4-6 are the RNAV set of the same three. In practice a data
# house may encode the whole runway variant under the "common" type with
# the transition field naming the runway (Navigraph does), so the common
# types are split by their transition field instead: "RWxx" is a
# runway's path, "ALL" or blank is shared, anything else an enroute
# transition.
_SID_RUNWAY = {1, 4}
_SID_COMMON = {2, 5}
_SID_ENROUTE = {3, 6}
_STAR_ENROUTE = {1, 4}
_STAR_COMMON = {2, 5}
_STAR_RUNWAY = {3, 6}
# RNAV procedures need an RNAV-capable box; a Cessna's ADF does not fly
# one. The traffic draws that line too.
RNAV_TYPES = {4, 5, 6}

# The approach final segment's route type letter names the approach kind;
# "A" is a transition feeding the final.
_APPROACH_KIND = {
    "B": "LOC BC", "D": "VOR/DME", "E": "VOR", "F": "NDB/DME",
    "G": "NDB", "H": "RNP", "I": "ILS", "J": "GLS", "K": "LDA",
    "L": "LOC", "N": "NDB", "P": "RNAV", "Q": "RNAV", "R": "RNAV",
    "S": "RNAV", "T": "RNAV", "U": "SDF", "V": "VOR",
    "W": "SDF", "X": "LOC", "Y": "SDF", "Z": "MLS",
}

# Waypoint description code (4.1.9.1 cols 40-43): the fourth character is
# the fix's role -- the IAF a transition opens with, the FAF, the missed
# approach point, a hold.
_ROLE_IAF = {"A", "C"}
_ROLE_IF = {"B", "I"}
_ROLE_FAF = {"F"}
_ROLE_MAP = {"M"}

# Section/subsection pairs a fix is resolved by: enroute and terminal
# waypoints in earth_fix.dat, navaids in earth_nav.dat, runways in the
# file's own RWY: records, localizers in earth_nav.dat.
_TERMINAL_FIX = {("P", "C")}
_NAVAID_FIX = {"D"}
_RUNWAY_FIX = {("P", "G")}
_LOCALIZER_FIX = {("P", "I")}

# The words a PRDAT row uses for its minima lines.
_PRDAT_LINES = {"LPV", "LNAV/VNAV", "LNAV", "LP", "LNAV+V", "RNP",
                "CIRCLE", "CIRCLING", "GLS"}


def _tenths(text: str) -> float:
    """A 0.1-unit field ("0880" -> 88.0); empty is zero."""
    text = (text or "").strip()
    if not text:
        return 0.0
    try:
        return float(text) / 10.0
    except ValueError:
        return 0.0


def _number(text: str) -> float:
    text = (text or "").strip()
    if not text:
        return 0.0
    try:
        return float(text)
    except ValueError:
        return 0.0


def _altitude(text: str) -> float:
    """A CIFP altitude field: plain feet, or a flight level "FLxxx"."""
    text = (text or "").strip().upper()
    if not text:
        return 0.0
    if text.startswith("FL"):
        text = text[2:]
    try:
        return float(text) * 100.0 if len(text) <= 3 else float(text)
    except ValueError:
        return 0.0


def _alt_desc(code: str) -> int:
    """4.1.9.1 col 83 altitude description -> the model's constants."""
    return {
        " ": ALT_AT, "+": ALT_AT_OR_ABOVE, "-": ALT_AT_OR_BELOW,
        "B": ALT_IN_BETWEEN, "C": ALT_AT_OR_BELOW,
        # Glide-slope-related codes: "at", the second altitude being the
        # intercept's. Read as AT rather than dropped.
        "G": ALT_AT, "H": ALT_AT, "I": ALT_AT, "J": ALT_AT,
    }.get((code or " ").strip()[:1] or " ", ALT_NOT_USED)


def _lat(text: str) -> float:
    """"N51283900" -> 51.4775: hemisphere plus DDMMSScc."""
    text = (text or "").strip().upper()
    if len(text) < 9 or text[0] not in "NS":
        return 0.0
    try:
        deg = int(text[1:3]) + int(text[3:5]) / 60.0 \
            + float(text[5:]) / 360000.0
    except ValueError:
        return 0.0
    return deg if text[0] == "N" else -deg


def _lon(text: str) -> float:
    """"W000290597" -> -0.4916."""
    text = (text or "").strip().upper()
    if len(text) < 10 or text[0] not in "EW":
        return 0.0
    try:
        deg = int(text[1:4]) + int(text[4:6]) / 60.0 \
            + float(text[6:]) / 360000.0
    except ValueError:
        return 0.0
    return deg if text[0] == "E" else -deg


def _runway_name(trans: str) -> str:
    """"RW09L" -> "09L"; anything else is a named transition."""
    trans = (trans or "").strip().upper()
    return trans[2:] if trans.startswith("RW") else ""


def _row(line: str) -> tuple[str, list[str], list[list[str]]]:
    """"CODE:a,b,c;opt1,opt2;" -> ("CODE", [a..c], [optional groups])."""
    line = line.strip()
    if not line:
        return "", [], []
    head, _, rest = line.partition(":")
    groups = rest.split(";")
    common = groups[0].split(",") if groups and groups[0] else []
    optional = [g.split(",") for g in groups[1:] if g.strip()]
    return head.strip().upper(), common, optional


def _distance(text: str) -> tuple[float, bool]:
    """The leg-length field: metres, or minutes when it is a time.

    "0020" is 2.0 NM; "T010" is 1.0 minute -- the T marks the hold and
    procedure-turn legs that carry a time rather than a distance.
    """
    text = (text or "").strip().upper()
    if text.startswith("T"):
        try:
            return float(text[1:]) / 10.0, True
        except ValueError:
            return 0.0, True
    return _tenths(text) * _M_PER_NM, False


class _AirportFile:
    """One ``CIFP/<ICAO>.dat``'s rows, grouped for decoding."""

    def __init__(self, path: Path):
        self.path = path
        self.legs: dict[str, list[list[str]]] = {"SID": [], "STAR": [],
                                                 "APPCH": []}
        self.runways: dict[str, tuple[float, float]] = {}
        # PRDAT rows have no sequence number of their own; what records
        # where they sit is how many APPCH rows came before them.
        self.prdats: list[tuple[int, list[str]]] = []
        try:
            with open(path, encoding="utf-8", errors="replace") as fh:
                for raw in fh:
                    code, fields, optional = _row(raw)
                    if not fields:
                        continue
                    if code in self.legs:
                        self.legs[code].append(fields)
                    elif code == "RWY" and optional:
                        self._runway(fields, optional)
                    elif code == "PRDAT":
                        self.prdats.append(
                            (len(self.legs["APPCH"]), fields))
        except OSError:
            log.debug("could not read %s", self.path, exc_info=True)

    def _runway(self, fields: list[str], optional: list[list[str]]) -> None:
        name = _runway_name(fields[0])
        if not name:
            return
        for group in optional:
            if len(group) >= 2 and group[0][:1].upper() in "NS":
                lat, lon = _lat(group[0]), _lon(group[1])
                if lat or lon:
                    self.runways.setdefault(name, (lat, lon))


class _FixIndex:
    """ident -> [(region, terminal, lat, lon)], built from a nav file on
    first need and kept for the session."""

    def __init__(self):
        self.by_ident: dict[str, list[tuple[str, str, float, float]]] = {}

    def resolve(self, ident: str, region: str = "", terminal: str = "",
                near: tuple[float, float] | None = None
                ) -> tuple[float, float]:
        """The fix's coordinates: the exact (ident, region, terminal) row,
        else (ident, terminal), else (ident, region), else the same-ident
        row nearest ``near``."""
        entries = self.by_ident.get((ident or "").strip().upper())
        if not entries:
            return 0.0, 0.0
        for want_region, want_term in ((region, terminal), ("", terminal),
                                       (region, "")):
            pool = [e for e in entries
                    if (not want_region or e[0] == want_region)
                    and (not want_term or e[1] == want_term)]
            if pool:
                return pool[0][2], pool[0][3]
        if near is not None and (near[0] or near[1]):
            from ..navdata.build import haversine_nm
            best = min(entries,
                       key=lambda e: haversine_nm(near[0], near[1],
                                                  e[2], e[3]))
            return best[2], best[3]
        return entries[0][2], entries[0][3]


def _index_fixes(index: _FixIndex, path: Path) -> None:
    """earth_fix.dat: "lat lon IDENT terminal region numeric-id [name]"."""
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                parts = line.split()
                if len(parts) < 6:
                    continue
                try:
                    lat, lon = float(parts[0]), float(parts[1])
                except ValueError:
                    continue
                index.by_ident.setdefault(parts[2].upper(), []).append(
                    (parts[4].upper(), parts[3].upper(), lat, lon))
    except OSError:
        log.debug("could not read fixes %s", path, exc_info=True)


def _index_navaids(index: _FixIndex, path: Path) -> None:
    """earth_nav.dat: "code lat lon elev freq range magvar IDENT terminal
    region name..."; the navaid rows and the localizers."""
    wanted = {"2", "3", "4", "5", "6", "12", "13"}
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                parts = line.split()
                if len(parts) < 10 or parts[0] not in wanted:
                    continue
                try:
                    lat, lon = float(parts[1]), float(parts[2])
                except ValueError:
                    continue
                index.by_ident.setdefault(parts[7].upper(), []).append(
                    (parts[9].upper(), parts[8].upper(), lat, lon))
    except OSError:
        log.debug("could not read navaids %s", path, exc_info=True)


class _LegReader:
    """Turns one airport file's leg rows into ProcedureLegs."""

    def __init__(self, af: _AirportFile, fixes: _FixIndex,
                 navaids: _FixIndex, ident: str):
        self.af = af
        self.fixes = fixes
        self.navaids = navaids
        self.ident = ident

    def _section(self, fields: list[str], base: int) -> tuple[str, str]:
        """(section, subsection) of the ident at ``base``."""
        sect = fields[base + 2].strip().upper() if len(fields) > base + 2 \
            else ""
        sub = fields[base + 3].strip().upper() if len(fields) > base + 3 \
            else ""
        return sect, sub

    def _coords(self, ident: str, region: str, sect: str, sub: str,
                ) -> tuple[float, float]:
        """The coordinates an ident resolves to, (0, 0) when unknown."""
        if not ident:
            return 0.0, 0.0
        if ident.startswith("RW") or (sect, sub) in _RUNWAY_FIX:
            return self.af.runways.get(
                ident[2:] if ident.startswith("RW") else ident, (0.0, 0.0))
        if sect in _NAVAID_FIX or (sect, sub) in _LOCALIZER_FIX:
            return self.navaids.resolve(ident, region, self.ident)
        if (sect, sub) in _TERMINAL_FIX:
            return self.fixes.resolve(ident, region, self.ident)
        # Enroute or unmarked: the waypoint file first, then the navaid
        # file where the waypoint file knows nothing.
        found = self.fixes.resolve(ident, region, self.ident)
        if found == (0.0, 0.0):
            found = self.navaids.resolve(ident, region, self.ident)
        return found

    def leg(self, index: int, fields: list[str]) -> ProcedureLeg:
        """One CIFP leg row as a ProcedureLeg, coordinates resolved."""
        def at(i: int) -> str:
            return fields[i] if len(fields) > i else ""

        desc = at(8).strip().upper()
        path = at(11).strip().upper()
        ident, region = at(4).strip().upper(), at(5).strip().upper()
        sect, sub = self._section(fields, 4)
        origin, origin_region = at(13).strip().upper(), \
            at(14).strip().upper()
        origin_sect, origin_sub = self._section(fields, 13)
        distance, is_time = _distance(at(21))
        alt1 = _altitude(at(23))
        arc_ident = at(30).strip().upper()
        arc_sect, arc_sub = self._section(fields, 30)
        role = desc[3:4]
        turn = at(9).strip().upper()[:1]
        return ProcedureLeg(
            index=index,
            path=path,
            fix_ident=ident,
            fix_region=region,
            fix_kind=("vor" if sect in _NAVAID_FIX
                      or (sect, sub) in _LOCALIZER_FIX
                      else "apt" if (sect, sub) in _RUNWAY_FIX
                      or ident.startswith("RW") else "wpt"),
            lat=self._coords(ident, region, sect, sub)[0],
            lon=self._coords(ident, region, sect, sub)[1],
            fix_alt_ft=alt1,
            fly_over=desc[1:2] == "Y",
            distance_minute=is_time,
            turn=turn if turn in "LRE" else "",
            origin_ident=origin,
            origin_region=origin_region,
            origin_kind=("vor" if origin_sect in _NAVAID_FIX
                         or (origin_sect, origin_sub) in _LOCALIZER_FIX
                         else "wpt"),
            origin_lat=self._coords(origin, origin_region, origin_sect,
                                    origin_sub)[0] if origin else 0.0,
            origin_lon=self._coords(origin, origin_region, origin_sect,
                                    origin_sub)[1] if origin else 0.0,
            theta=_tenths(at(18)),
            rho_m=_tenths(at(19)) * _M_PER_NM,
            course=_tenths(at(20)),
            route_distance=distance,
            alt_desc=_alt_desc(at(22)),
            alt1_ft=alt1,
            alt2_ft=_altitude(at(24)),
            speed_limit_kt=_number(at(27)),
            vertical_angle=_number(at(28)) / 100.0,
            arc_ident=arc_ident,
            arc_region=at(31).strip().upper(),
            arc_kind=("vor" if arc_sect in _NAVAID_FIX else "wpt"),
            arc_lat=self._coords(arc_ident, at(31).strip().upper(),
                                 arc_sect, arc_sub)[0]
            if arc_ident else 0.0,
            arc_lon=self._coords(arc_ident, at(31).strip().upper(),
                                 arc_sect, arc_sub)[1]
            if arc_ident else 0.0,
            radius_m=_number(at(17)) / 1000.0 * _M_PER_NM,
            rnp_m=_number(at(10)),
            is_iaf=role in _ROLE_IAF,
            is_if=role in _ROLE_IF,
            is_faf=role in _ROLE_FAF,
            is_map=role in _ROLE_MAP,
        )


def _expand_runway(name: str, af: _AirportFile) -> list[str]:
    """The runways a transition serves: "16B" is the whole parallel set
    (16L/16C/16R at Seattle), spelled out against the file's own RWY:
    records; L/R is the guess when there are none."""
    if not name.endswith("B"):
        return [name]
    prefix = name[:-1]
    matches = sorted(r for r in af.runways if r.startswith(prefix))
    if matches:
        return matches
    return [prefix + side for side in "LR"]


def _leg_groups(rows: list[list[str]]) -> \
        dict[tuple[str, str, str], list[tuple[int, list[str]]]]:
    """Leg rows by (route_type, procedure, transition), file order kept
    as each row's position in the section."""
    groups: dict[tuple[str, str, str], list[tuple[int, list[str]]]] = {}
    for pos, fields in enumerate(rows):
        key = (fields[1].strip().upper() if len(fields) > 1 else "",
               fields[2].strip().upper() if len(fields) > 2 else "",
               fields[3].strip().upper() if len(fields) > 3 else "")
        groups.setdefault(key, []).append((pos, fields))
    for fields in groups.values():
        fields.sort(key=lambda pf: _number(pf[1][0]))
    return groups


def _compose(reader: _LegReader,
             runway_groups: dict[str, list[list[str]]],
             common_rows: list[list[str]],
             all_runway_rows: list[list[str]],
             enroute_groups: dict[str, list[list[str]]],
             runway_first: bool) -> list[ProcedureTransition]:
    """Each transition as a complete flyable path.

    A ProcedureNavigator flies one transition's leg list, so the decoded
    parts are composed the way the clearance reads them: a SID's runway
    legs, then the common portion, then the enroute transition it was
    filed to; a STAR the other way round -- its enroute transition, the
    common portion, the runway funnel. Rows whose transition field is
    "ALL" are common legs that belong to every composition.
    """
    def decode(rows: list[list[str]]) -> list[ProcedureLeg]:
        return [reader.leg(i, f) for i, f in enumerate(rows)]

    # Rows tagged "ALL" sit with the runway legs: the all-runways climb
    # after a SID's runway legs, the all-runways funnel before a STAR's.
    common = decode(common_rows)
    all_runway = decode(all_runway_rows)
    shared = all_runway + common if runway_first else common + all_runway
    out: list[ProcedureTransition] = []
    enroutes = enroute_groups or {"": []}
    runways = runway_groups or {"": []}
    for en_name, en_rows in enroutes.items():
        for rw_name, rw_rows in runways.items():
            en_legs, rw_legs = decode(en_rows), decode(rw_rows)
            legs = [*rw_legs, *shared, *en_legs] if runway_first \
                else [*en_legs, *shared, *rw_legs]
            if not legs:
                continue
            out.append(ProcedureTransition(
                name=en_name, runway=rw_name, legs=tuple(legs)))
    return out


def _terminal_procedures(af: _AirportFile, fixes: _FixIndex,
                         navaids: _FixIndex, ident: str
                         ) -> AirportProcedures | None:
    """The SID/STAR rows as AirportProcedures."""
    reader = _LegReader(af, fixes, navaids, ident)
    airport = AirportProcedures(ident)
    for code, kind in (("SID", "sid"), ("STAR", "star")):
        table = airport.departures if kind == "sid" else airport.arrivals
        by_proc: dict[str, dict] = {}
        for (rtype, proc_name, trans), rows in _leg_groups(
                af.legs[code]).items():
            if not proc_name or not rtype.isdigit():
                continue
            number = int(rtype)
            parts = by_proc.setdefault(proc_name, {"runway": {},
                                                   "enroute": {},
                                                   "common": [],
                                                   "all": []})
            runway_set, enroute_set, common_set = (
                (_SID_RUNWAY, _SID_ENROUTE, _SID_COMMON)
                if kind == "sid"
                else (_STAR_RUNWAY, _STAR_ENROUTE, _STAR_COMMON))
            raw = [f for _p, f in rows]
            rw_name = _runway_name(trans)
            if number in runway_set or (number in common_set and rw_name):
                if rw_name:
                    for one in _expand_runway(rw_name, af):
                        parts["runway"].setdefault(one, []).extend(raw)
                else:
                    parts["all"].extend(raw)
            elif number in enroute_set:
                name = "" if trans in ("", "ALL") else trans
                parts["enroute"].setdefault(name, []).extend(raw)
            else:
                # Common legs ("ALL"/blank), named enroute transitions
                # encoded under the common type, and unknown types.
                if trans in ("", "ALL"):
                    parts["common"].extend(raw)
                else:
                    parts["enroute"].setdefault(trans, []).extend(raw)
        for proc_name, parts in by_proc.items():
            proc = table.setdefault(proc_name, TerminalProcedure(
                name=proc_name, kind=kind))
            transitions = _compose(
                reader, parts["runway"], parts["common"], parts["all"],
                parts["enroute"], runway_first=(kind == "sid"))
            proc.transitions.extend(transitions)
        for proc in table.values():
            proc.transitions.sort(key=lambda t: (t.runway, t.name))
    if not airport.departures and not airport.arrivals:
        return None
    return airport


def _approach_runway(proc_ident: str) -> str:
    """"I09L"/"R22LZ" -> "09L"/"22L": the runway the approach serves."""
    ident = (proc_ident or "").strip().upper()
    digits = "".join(ch for ch in ident if ch.isdigit())
    side = "".join(ch for ch in ident[1:] if ch in "LRC")
    if len(digits) < 2:
        return ""
    return f"{int(digits[:2]):02d}{side[:1]}"


def _approach_suffix(proc_ident: str) -> str:
    """"I09LY"/"R22LZ" -> "Y"/"Z": the duplicate-procedure letter."""
    ident = (proc_ident or "").strip().upper()
    extra = [ch for ch in ident[1:]
             if ch.isalpha() and ch not in "LRC"]
    return extra[-1] if extra else ""


def _approaches(af: _AirportFile, fixes: _FixIndex,
                navaids: _FixIndex, ident: str) -> FieldApproaches | None:
    """The APPCH rows as FieldApproaches.

    Route type "A" rows are transitions feeding the final; the first
    other-typed row opens the final and its letter names the kind
    (I=ILS, R=RNAV, ...). PRDAT rows carry an RNAV's minima lines and
    belong to the final their file position sits inside. The FAF and
    missed altitudes come off the legs' own constraints.
    """
    reader = _LegReader(af, fixes, navaids, ident)
    groups = _leg_groups(af.legs["APPCH"])
    finals: list[tuple[str, str, list[int], list[list[str]]]] = []
    for (rtype, proc_ident, _trans), rows in groups.items():
        if rtype == "A" or not proc_ident or not rows:
            continue
        finals.append((rtype, proc_ident, [p for p, _f in rows],
                       [f for _p, f in rows]))
    finals.sort(key=lambda f: min(f[2]))
    found = FieldApproaches(ident)
    for rtype, proc_ident, positions, rows in finals:
        kind = _APPROACH_KIND.get(rtype, "RNAV")
        runway = _approach_runway(proc_ident)
        legs = [reader.leg(i, f) for i, f in enumerate(rows)]
        faf_ft = next((leg.altitude_constraint or leg.fix_alt_ft
                       for leg in legs if leg.is_faf), 0.0)
        map_index = next((leg.index for leg in legs if leg.is_map),
                         len(legs) - 1)
        missed_ft = next(
            (leg.altitude_constraint or leg.fix_alt_ft
             for leg in legs
             if leg.index > map_index
             and (leg.altitude_constraint or leg.fix_alt_ft)),
            0.0)
        # The PRDAT inside this final (it sits between the final's rows
        # in the file) carries the RNAV minima lines.
        lines = set()
        lo, hi = min(positions), max(positions)
        for pos, prdat in af.prdats:
            if lo <= pos <= hi:
                for value in prdat:
                    word = value.strip().upper()
                    if word in _PRDAT_LINES:
                        lines.add({"LNAV+V": "LNAV/VNAV",
                                   "CIRCLE": "CIRCLING",
                                   "RNP": "RNAV"}.get(word, word))
        found.add_procedure(Approach(
            kind, runway, _approach_suffix(proc_ident), frozenset(lines),
            float(faf_ft), float(missed_ft),
            missed_legs=tuple(leg for leg in legs
                              if leg.index > map_index),
            legs=tuple(leg for leg in legs if leg.index <= map_index)))
    return found if found.procedures else None


class CifpSource:
    """Terminal procedures and approaches from a CIFP data tree.

    ``roots`` are directories that may hold a ``CIFP`` subdirectory plus
    ``earth_fix.dat``/``earth_nav.dat`` -- the X-Plane install's
    ``Custom Data`` first (a data subscription wins, exactly as the
    simulator's own layering does), then ``Resources/default data``.
    Everything is read lazily and cached per airport; a missing file is
    cached as None so a procedure-less airport is asked for once.
    """

    def __init__(self, roots=()):
        self._dirs = [Path(root) for root in roots if root]
        self._procedures: dict[str, AirportProcedures | None] = {}
        self._approaches: dict[str, FieldApproaches | None] = {}
        self._fixes: _FixIndex | None = None
        self._navaids: _FixIndex | None = None
        self._files: dict[str, _AirportFile | None] = {}

    def _file(self, ident: str) -> _AirportFile | None:
        if ident in self._files:
            return self._files[ident]
        found = None
        for root in self._dirs:
            path = root / "CIFP" / f"{ident}.dat"
            if path.is_file():
                found = _AirportFile(path)
                break
        self._files[ident] = found
        return found

    def _navfiles(self) -> tuple[_FixIndex, _FixIndex]:
        """The fix and navaid indexes, built from the first root that has
        each file -- the subscription data wins, the default fills gaps."""
        if self._fixes is not None and self._navaids is not None:
            return self._fixes, self._navaids
        fixes, navaids = _FixIndex(), _FixIndex()
        for root in self._dirs:
            if not fixes.by_ident:
                candidate = root / "earth_fix.dat"
                if candidate.is_file():
                    _index_fixes(fixes, candidate)
            if not navaids.by_ident:
                candidate = root / "earth_nav.dat"
                if candidate.is_file():
                    _index_navaids(navaids, candidate)
            if fixes.by_ident and navaids.by_ident:
                break
        self._fixes, self._navaids = fixes, navaids
        return fixes, navaids

    def procedures(self, ident: str) -> AirportProcedures | None:
        ident = (ident or "").strip().upper()
        if not ident:
            return None
        if ident in self._procedures:
            return self._procedures[ident]
        found = None
        af = self._file(ident)
        if af is not None:
            fixes, navaids = self._navfiles()
            try:
                found = _terminal_procedures(af, fixes, navaids, ident)
            except Exception:
                log.info("%s: could not decode CIFP procedures", ident,
                         exc_info=True)
        self._procedures[ident] = found
        return found

    def approaches(self, ident: str) -> FieldApproaches | None:
        ident = (ident or "").strip().upper()
        if not ident:
            return None
        if ident in self._approaches:
            return self._approaches[ident]
        found = None
        af = self._file(ident)
        if af is not None:
            fixes, navaids = self._navfiles()
            try:
                found = _approaches(af, fixes, navaids, ident)
            except Exception:
                log.info("%s: could not decode CIFP approaches", ident,
                         exc_info=True)
        self._approaches[ident] = found
        return found


__all__ = ["CifpSource", "RNAV_TYPES"]
