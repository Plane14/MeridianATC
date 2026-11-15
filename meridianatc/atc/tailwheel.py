"""Conventional-gear handling: the taildragger rules the ground keeps.

What the sim calls a taildragger and what the Airplane Flying Handbook
(AFH, chapter 14) says to do with one:

- the tailwheel steers through springs to about 30 degrees and then
  casters; below that it is free, and the aeroplane is held straight on
  rudder and differential brake alone;
- the centre of gravity sits behind the mains, so the aeroplane
  weathervanes harder and every skid is a ground loop beginning --
  corrections want to be early and small;
- the stick positions for a quartering wind are the tricycle's
  reversed: "climb into" a quartering headwind (elevator back, aileron
  up on the windward wing), "dive away" from a quartering tailwind
  (elevator forward, aileron down on the windward wing);
- the nose hides the taxiway ahead, so the pilot zigzags gently -- the
  S-turns every tailwheel checkout teaches.

Detection is layered: the sim's own flag first (``IS TAIL DRAGGER`` on
SimConnect, ``sim/aircraft/gear/acf_is_taildragger`` on X-Plane), then
the ICAO type and the title for sources that do not report it.
"""

from __future__ import annotations

import math

# ICAO type designators that are conventional gear as built. Not every
# tailwheel machine has a code (most warbirds do not), and not every
# airframe under a code has kept its tailwheel (C170 conversions exist),
# so the sim's own flag wins whenever it reports one.
TAILWHEEL_TYPES = frozenset({
    "J3", "PA11", "PA12", "PA14", "PA15", "PA16", "PA18", "PA20",
    "PA25",
    "C120", "C140", "C170", "C180", "C185", "C195",
    # The DC-4 onward is nosewheel; only the DC-3 sits on its tail.
    "DC3",
    "AN2", "DHC2", "DHC3",
    # P-38 and B-24 are tricycle; the rest of the fighters and both
    # other bombers are conventional.
    "T6", "P40", "P47", "P51", "B17", "B25",
    "BL8", "CH7A", "CH7B", "PTS1", "PTS2", "EXTR", "EDGE", "MX2",
    "SU26", "SU29", "SU31", "YAK52", "YAK55",
    "ST75", "DH82", "STIL", "L5", "L6", "L8", "L13", "L19",
    # Van's "A"-suffixed kits and the RV-10/12 are nosewheel; the
    # RV-7/8/14 tailwheel models are the conventional ones.
    "RV7", "RV8", "RV14",
    # PA-22 Tri-Pacer, DR40, Z-42/43 and SYMP are all nosewheel
    # despite sitting next to tailwheel families.
})

# Title words a type code does not cover: restored warbirds, kitplanes,
# and the Cub family's endless renames.
TAILWHEEL_TITLE_FRAGMENTS = (
    "super cub", "carbon cub", "top cub", "sport cub", "cubcrafters",
    "taildragger", "tail dragger", "conventional gear",
    "spitfire", "hurricane", "mustang", "p-51", "warhawk", "p-40",
    "stearman", "kaydet", "citabria", "decathlon", "scout",
    "pitts", "chipmunk", "texan", "t-6", "harvard",
    "dc-3", "c-47", "dakota", "beech 18", "staggerwing",
    "husky", "christen eagle", "extra 300", "extra 330", "edge 540",
    "sukhoi", "yak-52", "yak 52", "yak-55",
    "stinson", "luscombe", "aeronca", "champ", "taylorcraft",
    "maule", "bearhawk", "aviat", "waco", "jungmann", "stampe",
    "tiger moth", "bf 109", "me-109", "fw 190", "corsair",
)


def is_tailwheel(state) -> bool:
    """Whether this aeroplane sits on its tail.

    The sim's flag wins: it knows the gear it loaded. Where it does not
    report one (``None``), the type code and then the title answer, so
    an online feed that only sends "PA18" still flies like a Cub.
    """
    seen = getattr(state, "tail_dragger", None)
    if seen is not None:
        return bool(seen)
    code = (getattr(state, "type_code", "") or "").strip().upper()
    if code in TAILWHEEL_TYPES:
        return True
    title = (getattr(state, "title", "") or "").lower()
    return any(fragment in title for fragment in TAILWHEEL_TITLE_FRAGMENTS)


def _relative(wind_from: float, heading: float) -> float:
    """Wind direction relative to the nose, + = blowing from the right."""
    return ((wind_from - heading + 540.0) % 360.0) - 180.0


def taxi_controls(wind_from: float, wind_kt: float, heading: float,
                  tailwheel: bool) -> tuple[float, float]:
    """The stick position a taxi holds in a wind: (elevator, aileron).

    Elevator is +1 stick back (nose up), aileron +1 turns the yoke right
    (right wing down). The AFH's figure 14-4, in numbers:

    - quartering headwind: aileron up on the windward wing -- the yoke
      turns *into* the wind; the elevator stays neutral on a tricycle
      and comes back on a tailwheel, pinning the tail down ("climb
      into the wind").
    - quartering tailwind: aileron down on the windward wing and the
      elevator forward on either gear -- "dive away from the wind".
    - calm: nothing but the tailwheel's habit of keeping the stick
      back, which keeps the tail planted over rough pavement.
    """
    authority = min(1.0, wind_kt / 15.0)
    if wind_kt < 4.0:
        return (0.4, 0.0) if tailwheel else (0.0, 0.0)
    rel = _relative(wind_from, heading)
    side = 1.0 if rel > 0.0 else -1.0
    if abs(rel) <= 90.0:
        # Quartering headwind: aileron up on the windward wing, which
        # is the yoke turned toward the wind. The tailwheel also wants
        # the elevator back; a tricycle keeps it neutral.
        elevator = (0.2 + 0.6 * authority) if tailwheel else 0.0
        return elevator, side * authority
    # Quartering tailwind: "dive away" -- aileron down on the windward
    # wing, elevator forward on both gear arrangements.
    return -0.8 * authority, -side * authority


# Differential braking on the ground: used below this speed, blended out
# above it, where the rudder has the authority the brakes no longer need
# to add to. UNVERIFIED: the numbers are handling tuning, not SOP.
DIFF_BRAKE_FULL_KT = 10.0
DIFF_BRAKE_MAX_KT = 28.0
# Pedal demand below this steers on rudder and nosewheel alone; past it
# the inside wheel starts to help. High enough that straight tracking
# never drags a brake.
DIFF_BRAKE_AFTER = 0.35
DIFF_BRAKE_GAIN = 1.0
DIFF_BRAKE_MAX = 0.45


def brake_assist(demand: float, gs_kt: float) -> tuple[float, float]:
    """The (left, right) brake a steering demand wants added.

    ``demand`` is the law's signed steering ask, -1..+1 (+ = right) --
    the value before ``_pedal_dir``, so an inverted-axis model still
    brakes the wheel the turn wants. A taildragger's answer to a turn
    it cannot steer into is the inside wheel braked: at walking pace
    the castered tailwheel gives nothing and the rudder has no
    airflow, so the tight apron turn is flown on the pedals' tops.
    The assist fades with groundspeed and is gone by
    DIFF_BRAKE_MAX_KT, where standing on a brake would lift the tail.
    """
    if gs_kt >= DIFF_BRAKE_MAX_KT:
        return 0.0, 0.0
    excess = abs(demand) - DIFF_BRAKE_AFTER
    if excess <= 0.0:
        return 0.0, 0.0
    amount = min(DIFF_BRAKE_MAX, excess * DIFF_BRAKE_GAIN)
    if gs_kt > DIFF_BRAKE_FULL_KT:
        span = DIFF_BRAKE_MAX_KT - DIFF_BRAKE_FULL_KT
        amount *= max(0.0, 1.0 - (gs_kt - DIFF_BRAKE_FULL_KT) / span)
    return (0.0, amount) if demand > 0.0 else (amount, 0.0)


# The weave a taildragger taxis: a slow S so the pilot can see past the
# nose. Small enough that the corridor logic never reads it as leaving
# the path; the period is a deliberate aeroplane, not a wobble.
S_TURN_DEG = 9.0
S_TURN_PERIOD_S = 9.0


def s_turn_offset(now_s: float) -> float:
    """Degrees to add to the taxi heading for the see-over-the-nose S."""
    return S_TURN_DEG * math.sin(2.0 * math.pi * now_s / S_TURN_PERIOD_S)
