"""Per-aircraft control-law gains for the autonomous pilots.

One bundle per class, resolved the same place the performance figures
are: the class table is the baseline, an autofly profile's ``handling``
block refines it for a matched add-on. The numbers below are seeded
from the open-loop laws they replace -- a gain that used to be "a
degree of error over eighteen" is written here as 1/18, so the closed
loop starts where the hand did -- and each carries the clamps that keep
a bad model or a bad gain inside the envelope.

The fixed-wing channels, and what they move:

- ``throttle``: knots of speed error to a lever position (0..1).
- ``steer``: degrees of heading error to rudder/nosewheel (-1..1).
- ``taxi``: knots of groundspeed error to a lever position (0..1).
- ``rotation``: fpm of vertical-speed error to elevator (-1..1) --
  the model publishes no pitch attitude to this layer, so the loop
  closes on the rate coming alive, which is the cue a pilot uses too.
- ``flare``: feet-per-minute of sink error to elevator (-1..1).
- ``vertical``: feet of path error to a vertical-speed demand (fpm).

The rotorcraft channels are per type code, since a Robinson and a
Super Puma share nothing but the rotor. ``collective`` is feet of
height error to per-cent of collective; ``cyclic`` is knots of airspeed
error to axis units; ``pedals`` is degrees of heading error to axis
units; ``rotor`` is per-cent rotor RPM error to collective delta, the
autorotation's Nr governor.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .control import PidTerms
from .flying import RATE_LIGHT, Performance


@dataclass(frozen=True)
class FixedWingGains:
    """The channels a fixed-wing flight director is closed over."""

    throttle: PidTerms = field(default_factory=PidTerms)
    steer: PidTerms = field(default_factory=PidTerms)
    taxi: PidTerms = field(default_factory=PidTerms)
    rotation: PidTerms = field(default_factory=PidTerms)
    flare: PidTerms = field(default_factory=PidTerms)
    vertical: PidTerms = field(default_factory=PidTerms)


#: The channels a FixedWingGains bundle carries, in declaration order.
FIXED_WING_KEYS = ("throttle", "steer", "taxi", "rotation", "flare",
                   "vertical")


@dataclass(frozen=True)
class RotorGains:
    """The channels a rotorcraft's three axes are closed over."""

    collective: PidTerms = field(default_factory=PidTerms)
    cyclic: PidTerms = field(default_factory=PidTerms)
    pedals: PidTerms = field(default_factory=PidTerms)
    rotor: PidTerms = field(default_factory=PidTerms)


# --- fixed wing ------------------------------------------------------------
# Seeded from the laws they replace (autonomous._steer divided error by 18 or
# 28; _manage_throttle stepped ~0.006/kt; the flare was (sink-150)/1800) and
# given the damping and integration those laws never had.

AIRLINER_GAINS = FixedWingGains(
    throttle=PidTerms(kp=0.008, ki=0.004, deadband=3.0,
                      integral_cap=1.0, low=0.05, high=0.97, slew=0.5),
    steer=PidTerms(kp=1.0 / 18.0, ki=0.002, kd=1.0 / 9.0,
                   low=-0.5, high=0.5, wrap=360.0),
    taxi=PidTerms(kp=0.010, ki=0.003, deadband=1.0,
                  integral_cap=0.7, low=0.0, high=0.6, slew=0.4),
    rotation=PidTerms(kp=0.0010, ki=0.0005, kd=0.0004,
                      low=-0.05, high=0.7, slew=0.8),
    flare=PidTerms(kp=1.0 / 1800.0, kd=1.0 / 9000.0,
                   low=0.0, high=0.5, slew=0.6),
    vertical=PidTerms(kp=0.6, ki=0.02, deadband=15.0,
                      integral_cap=400.0, low=-400.0, high=350.0,
                      slew=400.0),
)

TURBOPROP_GAINS = FixedWingGains(
    throttle=PidTerms(kp=0.010, ki=0.005, deadband=3.0,
                      integral_cap=1.0, low=0.05, high=0.97, slew=0.6),
    steer=PidTerms(kp=1.0 / 18.0, ki=0.003, kd=1.0 / 10.0,
                   low=-0.5, high=0.5, wrap=360.0),
    taxi=PidTerms(kp=0.010, ki=0.004, deadband=1.0,
                  integral_cap=0.7, low=0.0, high=0.6, slew=0.5),
    rotation=PidTerms(kp=0.0012, ki=0.0007, kd=0.0005,
                      low=-0.05, high=0.7, slew=0.9),
    flare=PidTerms(kp=1.0 / 1500.0, kd=1.0 / 7500.0,
                   low=0.0, high=0.5, slew=0.7),
    vertical=PidTerms(kp=0.7, ki=0.02, deadband=15.0,
                      integral_cap=400.0, low=-400.0, high=350.0,
                      slew=450.0),
)

LIGHT_GAINS = FixedWingGains(
    throttle=PidTerms(kp=0.012, ki=0.006, deadband=3.0,
                      integral_cap=1.0, low=0.05, high=0.97, slew=0.7),
    steer=PidTerms(kp=1.0 / 28.0, ki=0.004, kd=1.0 / 14.0,
                   low=-0.5, high=0.5, wrap=360.0),
    taxi=PidTerms(kp=0.012, ki=0.005, deadband=1.0,
                  integral_cap=0.7, low=0.0, high=0.6, slew=0.6),
    rotation=PidTerms(kp=0.0015, ki=0.0008, kd=0.0006,
                      low=-0.05, high=0.6, slew=1.0),
    flare=PidTerms(kp=1.0 / 1200.0, kd=1.0 / 6000.0,
                   low=0.0, high=0.45, slew=0.8),
    vertical=PidTerms(kp=0.8, ki=0.03, deadband=15.0,
                      integral_cap=350.0, low=-350.0, high=300.0,
                      slew=500.0),
)


def class_key(performance: Performance) -> str:
    """Which bundle a Performance belongs to.

    The turn rate is the class's own discriminator -- the same figure
    ``_steer`` used to pick its divisor -- and the approach speed parts
    the two heavy classes, which share it.
    """
    if performance.turn_rate >= RATE_LIGHT:
        return "light"
    if performance.approach_kt <= 125.0:
        return "turboprop"
    return "airliner"


def fixed_wing_gains(performance: Performance) -> FixedWingGains:
    return {"light": LIGHT_GAINS, "turboprop": TURBOPROP_GAINS,
            "airliner": AIRLINER_GAINS}[class_key(performance)]


# --- rotorcraft --------------------------------------------------------------
# Keyed by the type code the session carries, as _HELI_PROFILES is. The
# DEFAULT row is the light single's: collective at the UNVERIFIED 0.12/ft
# the hand-flying module already flies, given integration (a slow one --
# the adaptive trim does the fast learning), and derivative damping on
# the cyclic and pedals those axes never had.

DEFAULT_ROTOR_GAINS = RotorGains(
    collective=PidTerms(kp=0.12, ki=0.01, kd=0.04, deadband=2.0,
                        integral_cap=15.0, low=-30.0, high=45.0,
                        slew=40.0),
    cyclic=PidTerms(kp=30.0, ki=2.0, kd=12.0, deadband=1.0,
                    integral_cap=2000.0, low=-6000.0, high=6000.0,
                    slew=20000.0),
    pedals=PidTerms(kp=8.0, ki=0.4, kd=3.0, deadband=0.5,
                    integral_cap=1500.0, low=-5000.0, high=5000.0,
                    slew=20000.0, wrap=360.0),
    rotor=PidTerms(kp=2.0, ki=0.3, kd=0.5, deadband=1.5,
                   integral_cap=20.0, low=-10.0, high=35.0, slew=30.0),
)

# Twins carry more inertia in the pitch and yaw axes; the figures are the
# default's, softened. UNVERIFIED like the defaults they descend from.
_TWIN_ROTOR_GAINS = RotorGains(
    collective=PidTerms(kp=0.10, ki=0.008, kd=0.05, deadband=2.0,
                        integral_cap=15.0, low=-30.0, high=45.0,
                        slew=35.0),
    cyclic=PidTerms(kp=24.0, ki=1.6, kd=14.0, deadband=1.0,
                    integral_cap=2000.0, low=-6000.0, high=6000.0,
                    slew=18000.0),
    pedals=PidTerms(kp=6.5, ki=0.3, kd=4.0, deadband=0.5,
                    integral_cap=1500.0, low=-5000.0, high=5000.0,
                    slew=18000.0, wrap=360.0),
    rotor=PidTerms(kp=1.8, ki=0.25, kd=0.6, deadband=1.5,
                   integral_cap=20.0, low=-10.0, high=35.0, slew=28.0),
)

_ROTOR_GAINS: dict[str, RotorGains] = {
    # Light singles get the default hand; the twins and mediums get the
    # softer set. Everything unlisted keeps the default, the same rule
    # the envelope profiles follow.
    "A109": _TWIN_ROTOR_GAINS,
    "A139": _TWIN_ROTOR_GAINS,
    "AS32": _TWIN_ROTOR_GAINS,
    "AS50": _TWIN_ROTOR_GAINS,
    "B412": _TWIN_ROTOR_GAINS,
    "EC30": _TWIN_ROTOR_GAINS,
    "EC35": _TWIN_ROTOR_GAINS,
    "EC45": _TWIN_ROTOR_GAINS,
    "H145": _TWIN_ROTOR_GAINS,
    "H160": _TWIN_ROTOR_GAINS,
    "S76": _TWIN_ROTOR_GAINS,
}


def rotor_gains(type_code: str) -> RotorGains:
    """The channels for this rotorcraft, or the conservative default."""
    return _ROTOR_GAINS.get((type_code or "").strip().upper(),
                            DEFAULT_ROTOR_GAINS)


__all__ = ["FIXED_WING_KEYS", "FixedWingGains", "RotorGains", "class_key",
           "fixed_wing_gains", "rotor_gains"]
