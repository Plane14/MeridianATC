"""Which SID, STAR or approach a flight gets: the one policy the
controller, the First Officer and the traffic all use.

The names a clearance, a checklist or a spawned aeroplane speaks come
through here, so all three hear the same answer for the same field and
runway. Two entry points per kind:

- ``select_*`` validates a name somebody filed or asked for. The answer
  is a :class:`Selected`: ``ok`` carries the procedure and its
  transition; anything else says why not, and is what "unable, that
  procedure is not available" is said from -- a name that is not in the
  data is never silently swapped for another.
- ``pick_*`` chooses when nobody named one: the traffic's invented
  IFR flights, which file only a destination. Scored by runway, then by
  how well the procedure's own fixes line up with where the flight is
  going, and deterministic -- the same field and runway always pick the
  same one.

Approach naming wraps :func:`meridianatc.atc.approaches.plan`, which is
already the controller's whole policy (runway, weather, minima, what the
aeroplane's navigator flies).
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import NamedTuple

from ..navdata.build import bearing_deg
from ..navdata.db import normalize_runway
from .approaches import Choice
from .approaches import plan as _approach_plan
from .procedure_legs import (
    AirportProcedures,
    ProcedureTransition,
    TerminalProcedure,
    choose_sid,
    choose_star,
)


class Selected(NamedTuple):
    """A chosen procedure and its transition, or why there is none."""

    procedure: TerminalProcedure | None
    transition: ProcedureTransition | None
    status: str                       # "ok" | "not_found" | "no_transition"
    name: str = ""                    # the procedure's name, for speech
    transition_name: str = ""         # the transition's, when it has one

    @property
    def ok(self) -> bool:
        return self.procedure is not None and self.transition is not None


def select_sid(procedures: AirportProcedures | None, name: str,
               runway: str = "", preferred: str = "",
               route_fix_idents: Iterable[str] = ()) -> Selected:
    """The SID ``name`` to fly, validated against the data."""
    if procedures is None:
        return Selected(None, None, "not_found", name)
    proc, trans = choose_sid(procedures, name, runway=runway,
                             preferred=preferred,
                             route_fix_idents=route_fix_idents)
    if proc is None:
        return Selected(None, None, "not_found", name)
    if trans is None:
        return Selected(proc, None, "no_transition", proc.name)
    return Selected(proc, trans, "ok", proc.name,
                    trans.name or trans.runway)


def select_star(procedures: AirportProcedures | None, name: str,
                runway: str = "", preferred: str = "",
                route_fix_idents: Iterable[str] = ()) -> Selected:
    """The STAR ``name`` to fly, validated against the data."""
    if procedures is None:
        return Selected(None, None, "not_found", name)
    proc, trans = choose_star(procedures, name, runway=runway,
                              preferred=preferred,
                              route_fix_idents=route_fix_idents)
    if proc is None:
        return Selected(None, None, "not_found", name)
    if trans is None:
        return Selected(proc, None, "no_transition", proc.name)
    return Selected(proc, trans, "ok", proc.name,
                    trans.name or trans.runway)


def _direction_score(at: tuple[float, float],
                     toward: tuple[float, float] | None,
                     fix_lat: float, fix_lon: float) -> float:
    """0-4 for how well the fix's bearing from the field lines up with
    the bearing the flight is going; 0 when no direction is known."""
    if toward is None or not (fix_lat or fix_lon):
        return 0.0
    wanted = bearing_deg(at[0], at[1], toward[0], toward[1])
    offered = bearing_deg(at[0], at[1], fix_lat, fix_lon)
    diff = abs(((wanted - offered + 540.0) % 360.0) - 180.0)
    return max(0.0, 4.0 * (1.0 - diff / 180.0))


def pick_sid(procedures: AirportProcedures | None, runway: str = "",
             route_fix_idents: Iterable[str] = (),
             at: tuple[float, float] | None = None,
             toward: tuple[float, float] | None = None) -> Selected:
    """The most appropriate departure when nobody filed one.

    The runway's own transitions are the only ones a clearance can name;
    among the SIDs that have one, the winner is the one whose last fix
    points the way the flight is going. ``at``/``toward`` are the field
    and the destination, so "going" has a bearing.
    """
    if procedures is None:
        return Selected(None, None, "not_found")
    best: Selected | None = None
    best_score = -1.0
    for name in sorted(procedures.departures):
        chosen = select_sid(procedures, name, runway=runway,
                            route_fix_idents=route_fix_idents)
        if chosen.transition is None:
            continue
        trans = chosen.transition
        if (runway and trans.runway
                and normalize_runway(trans.runway) != normalize_runway(runway)):
            continue
        score = (8.0 if runway and trans.runway
                 and normalize_runway(trans.runway) == normalize_runway(runway)
                 else 0.0)
        last = trans.last_fix
        if at is not None and last is not None:
            score += _direction_score(at, toward, last.lat, last.lon)
        if score > best_score:
            best, best_score = chosen, score
    return best or Selected(None, None, "no_transition")


def pick_star(procedures: AirportProcedures | None, runway: str = "",
              route_fix_idents: Iterable[str] = (),
              at: tuple[float, float] | None = None,
              toward: tuple[float, float] | None = None) -> Selected:
    """The most appropriate arrival when nobody filed one.

    The STAR whose entry fix is on the side the flight is coming from:
    ``toward`` is the origin, so a CHINS5 from the east picks the
    transition whose first fix is east of the field rather than the
    one whose name sorts first.
    """
    if procedures is None:
        return Selected(None, None, "not_found")
    best: Selected | None = None
    best_score = -1.0
    for name in sorted(procedures.arrivals):
        chosen = select_star(procedures, name, runway=runway,
                             route_fix_idents=route_fix_idents)
        if chosen.transition is None:
            continue
        trans = chosen.transition
        score = 4.0 if runway and trans.runway == runway else 0.0
        first = trans.first_fix
        if at is not None and first is not None:
            score += _direction_score(at, toward, first.lat, first.lon)
        if score > best_score:
            best, best_score = chosen, score
    return best or Selected(None, None, "no_transition")


def select_approach(known, runway: str, weather=None, *,
                    requested: str = "", airliner: bool = True,
                    region: str = "icao",
                    runway_headings: dict | None = None,
                    ils: bool | None = None) -> Choice:
    """The approach a controller names, the same answer for the user,
    the traffic and the FO: ``approaches.plan`` is the policy already,
    this is the seam everything goes through."""
    return _approach_plan(known, runway, weather, requested=requested,
                          airliner=airliner, region=region,
                          runway_headings=runway_headings, ils=ils)


__all__ = [
    "Selected",
    "pick_sid",
    "pick_star",
    "select_approach",
    "select_sid",
    "select_star",
]
