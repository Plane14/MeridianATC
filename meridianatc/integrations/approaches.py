"""Asking the simulator which approaches an airport publishes.

The navigation data has runways and frequencies and no procedures, so every
approach was "the ILS" (see :mod:`meridianatc.atc.approaches`). The simulator's
facility data has the airport's approaches, each with a type and the runway
it serves, and this reads them the same way
:mod:`meridianatc.integrations.taxiways` reads the taxiways: one session of
its own, one request, the answer decoded against a fixed block shape and
thrown away whole if the shape is wrong.

The fields and their values are the SDK's (AddToFacilityDefinition, APPROACH:
TYPE, SUFFIX, RUNWAY_NUMBER and RUNWAY_DESIGNATOR, each an INT32; FAF_ALTITUDE
in metres and MISSED_ALTITUDE in feet, each a FLOAT32 -- the units are the
documentation's, odd as the pair is, and each altitude is only used inside a
sane band above the field, see ``ControllerBrain.intercept_altitude``). The
minima flags (HAS_LNAV, HAS_LNAVVNAV, HAS_LP, HAS_LPV) name the RNAV lines the
approach carries, and the FINAL_APPROACH_LEG and MISSED_APPROACH_LEG groups
under the approach are its path -- the same APPROACH_LEG fields
:mod:`meridianatc.integrations.terminal_procedures` decodes under the SIDs
and STARs, which is where the leg layout and parser come from. A final
segment is what traffic flies when the cleared approach has one; the missed
segment is what a go-around flies. The block size has not been checked
against a running simulator the way the taxiway sizes were; a block of any
other size leaves the airport unknown -- which is the ILS, exactly as before
-- rather than read wrongly. A leg block of another size is additive data
this build does not read: it is skipped, and the approach keeps the header's
name and altitudes without the legs.
"""

from __future__ import annotations

import logging
import struct

from ..atc.approaches import KINDS, Approach, FieldApproaches, runway_from
from ..logs import decision
from .taxiways import ask_facility_records
from .terminal_procedures import _LEG_REQUEST, _leg

log = logging.getLogger(__name__)

_APPROACH = 5          # SIMCONNECT_FACILITY_DATA_TYPE
_FINAL_LEG = 8
_MISSED_LEG = 9


def _leg_group(kind: bytes) -> tuple[bytes, ...]:
    """One leg group under the approach: the same APPROACH_LEG fields
    the terminal-procedure feed asks for."""
    return (b"OPEN " + kind,) + tuple(
        name.encode("ascii") for name in _LEG_REQUEST) + \
        (b"CLOSE " + kind,)


_FIELDS = (
    b"OPEN AIRPORT",
    b"OPEN APPROACH", b"TYPE", b"SUFFIX", b"RUNWAY_NUMBER",
    b"RUNWAY_DESIGNATOR", b"FAF_ALTITUDE", b"MISSED_ALTITUDE",
    b"HAS_LNAV", b"HAS_LNAVVNAV", b"HAS_LP", b"HAS_LPV",
    b"N_TRANSITIONS", b"N_FINAL_APPROACH_LEGS",
    b"N_MISSED_APPROACH_LEGS",
    *_leg_group(b"FINAL_APPROACH_LEG"),
    *_leg_group(b"MISSED_APPROACH_LEG"),
    b"CLOSE APPROACH",
    b"CLOSE AIRPORT",
)
_SHAPE = "<iiiiffiiiiiii"
FT_PER_M = 3.28084

# The RNAV minima lines each HAS flag names, in the order the request asks
# for them. A flag of one means the approach publishes that line.
_LINE_FLAGS = ("LNAV", "LNAV/VNAV", "LP", "LPV")


def decode(records, ident: str) -> FieldApproaches | None:
    """The approaches out of the raw records, or None if they are not the
    shape this was written against. Legs that do not match the layout are
    skipped, not fatal: they are data this read adds, not data it needs.
    """
    found = FieldApproaches(ident)
    size = struct.calcsize(_SHAPE)
    legs_by_owner: dict[tuple[int, int], list] = {}
    for record in records:
        if record.type not in (_FINAL_LEG, _MISSED_LEG):
            continue
        leg = _leg(record.item_index, record.raw)
        if leg is None:
            log.info("%s: a %d-byte leg block under an approach; "
                     "keeping the headers, dropping the legs", ident,
                     len(record.raw))
            continue
        index = (record.item_index if record.is_list_item
                 else len(legs_by_owner.get(
                     (record.type, record.parent_id), [])))
        legs_by_owner.setdefault(
            (record.type, record.parent_id), []).append((index, leg))

    def legs(kind: int, parent: int) -> tuple:
        return tuple(leg for _i, leg in sorted(
            legs_by_owner.get((kind, parent), []),
            key=lambda item: item[0]))

    for record in records:
        if record.type != _APPROACH:
            continue
        if len(record.raw) != size:
            log.info("%s: a %d-byte approach block where %d were expected; "
                     "not using the simulator's approaches", ident,
                     len(record.raw), size)
            return None
        unpacked = struct.unpack(_SHAPE, record.raw)
        kind, suffix, number, designator, faf_m, missed_ft = unpacked[:6]
        flags = unpacked[6:10]
        name = KINDS.get(kind)
        runway = runway_from(number, designator)
        # The chart's letter (ILS Z, VOR-A), an ASCII code. A procedure to
        # no runway is a circling-only one, kept now that a circle can be
        # flown (the IFR plan, Phase 4); before, it was dropped.
        letter = chr(suffix) if 65 <= suffix <= 90 else ""
        kept = bool(name and (runway or letter))
        lines = frozenset(
            line for line, flag in zip(_LINE_FLAGS, flags, strict=True)
            if flag)
        final = legs(_FINAL_LEG, record.unique_id)
        missed = legs(_MISSED_LEG, record.unique_id)
        decision("approach_data", airport=ident, runway=runway or None,
                 type=name or kind, suffix=letter or None,
                 faf_raw_m=round(faf_m, 1), missed_raw=round(missed_ft, 1),
                 lines=sorted(lines) or None,
                 legs=len(final) or None,
                 missed_legs=len(missed) or None,
                 kept=kept)
        if kept:
            found.add_procedure(Approach(
                name, runway, letter, lines,
                faf_ft=max(0.0, faf_m) * FT_PER_M,
                missed_ft=max(0.0, missed_ft),
                missed_legs=missed, legs=final))
    return found


class SimApproaches:
    """The simulator's published approaches, one airport at a time."""

    TIMEOUT_S = 10.0

    def __init__(self, dll_path: str = ""):
        self.dll_path = dll_path
        self._cache: dict[str, FieldApproaches | None] = {}

    def approaches(self, ident: str) -> FieldApproaches | None:
        ident = (ident or "").strip().upper()
        if not ident:
            return None
        if ident in self._cache:
            return self._cache[ident]
        try:
            records = ask_facility_records(
                self.dll_path, b"MeridianATC approaches",
                _FIELDS, ident, self.TIMEOUT_S)
            found = None if records is None else decode(records, ident)
        except Exception:
            log.info("asking the simulator for %s's approaches raised", ident,
                     exc_info=True)
            found = None
        self._cache[ident] = found
        return found


__all__ = ["SimApproaches", "decode"]
