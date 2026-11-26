"""Runtime-assurance primitives for autonomous flight.

The AI decision gateway answers *which legal candidate* was selected. This
module answers the question that remains when no candidate is acceptable:
what deterministic action is preferred now. Fallbacks are ordered, named and
logged rather than improvised at the point of failure.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FallbackAction:
    """One deterministic fallback action in priority order."""

    name: str
    intent: str
    immediate: bool = False


# The first feasible action wins. These are intentionally behavioural
# ladders, not simulator commands: callers map the selected action to their
# existing phase/target machinery.
FIXED_WING_LADDER = (
    FallbackAction("maintain_safe_target", "maintain the current safe target", True),
    FallbackAction("rejoin_route", "rejoin the cleared route"),
    FallbackAction("hold", "hold at a safe fix or present position"),
    FallbackAction("divert", "divert to the planned alternate"),
    FallbackAction("controlled_landing", "land at the nearest suitable airfield"),
)

ROTORCRAFT_LADDER = (
    FallbackAction("hold_hover", "hold a stable hover", True),
    FallbackAction("land_immediately", "land immediately at the present safe spot", True),
    FallbackAction("reposition", "reposition to a safe landing spot"),
    FallbackAction("autorotate", "enter autorotation", True),
)

GROUND_LADDER = (
    FallbackAction("stop", "stop and hold position", True),
    FallbackAction("hold_short", "hold short of the runway", True),
    FallbackAction("vacate", "vacate the runway or conflict area"),
    FallbackAction("return_to_stand", "return to a safe stand"),
)


def fallback_ladder(*, rotorcraft: bool = False,
                    airborne: bool = True) -> tuple[FallbackAction, ...]:
    """The deterministic fallback ladder for this aircraft state."""
    if not airborne:
        return GROUND_LADDER
    return ROTORCRAFT_LADDER if rotorcraft else FIXED_WING_LADDER


def choose_fallback(*, rotorcraft: bool = False, airborne: bool = True,
                    rotor_failed: bool = False,
                    engine_failed: bool = False,
                    landing_spot_available: bool = True) -> FallbackAction:
    """Choose the first applicable deterministic fallback.

    Rotor failure in a helicopter forces autorotation. Engine failure in any
    aircraft rejects holding/rejoining in favour of landing. A missing safe
    rotorcraft spot favours autorotation over repositioning only when the
    aircraft cannot remain in a powered hover.
    """
    ladder = fallback_ladder(rotorcraft=rotorcraft, airborne=airborne)
    if rotorcraft and airborne and rotor_failed:
        return next(a for a in ladder if a.name == "autorotate")
    if not airborne:
        return ladder[0]
    if engine_failed:
        name = "autorotate" if rotorcraft else "controlled_landing"
        return next(a for a in ladder if a.name == name)
    if rotorcraft and not landing_spot_available:
        return next(a for a in ladder if a.name == "hold_hover")
    return ladder[0]


def validate_candidate_state(facts: dict) -> str:
    """Common operational checks for an AI movement candidate.

    Returns a veto reason, or empty when valid. Candidate producers put their
    own deterministic facts into the option; this shared monitor rejects
    obvious envelope, separation and runway-protection violations before an
    active model selection is ever executed.
    """
    if not facts:
        return ""
    if facts.get("safety_critical"):
        return "safety-critical action"
    if facts.get("runway_occupied"):
        return "runway protected"
    if facts.get("separation_nm", 99.0) < facts.get("min_separation_nm", 0.0):
        return "separation"
    if facts.get("terrain_clearance_ft", 9999.0) < facts.get("min_terrain_ft", 0.0):
        return "terrain clearance"
    if facts.get("speed_kt", 0.0) > facts.get("max_speed_kt", 999.0):
        return "speed envelope"
    if facts.get("altitude_ft", 0.0) > facts.get("ceiling_ft", 99999.0):
        return "ceiling"
    return ""


__all__ = [
    "FallbackAction", "FIXED_WING_LADDER", "GROUND_LADDER",
    "ROTORCRAFT_LADDER", "choose_fallback", "fallback_ladder",
    "validate_candidate_state",
]
