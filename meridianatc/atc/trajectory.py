"""Smooth deterministic movement primitives.

The ArduPilot control stack separates navigation from control: waypoint
navigation produces shaped position/velocity targets, position control turns
those into acceleration-limited motion, and fixed-wing speed/height control
manages total energy. MeridianATC does not need ArduPilot's real-hardware
implementation, but the same decomposition is useful here. These helpers are
small Python implementations of the *concepts*, with simulator-friendly units,
for the deterministic executor that AI tactical decisions are never allowed
to bypass.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Earth/gravity conversions used by the guidance math. Speeds in these
# functions are knots and distances nautical miles unless the name says
# otherwise; controller internals elsewhere already use those units.
KT_TO_MPS = 0.514444
MPS_TO_KT = 1.0 / KT_TO_MPS
G_MSS = 9.80665


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class TrajectoryLimits:
    """Kinematic limits for one shaped axis.

    ``max_velocity`` and ``max_acceleration`` bound the commanded motion;
    ``max_jerk`` bounds how quickly acceleration may change. Positive values
    only. These are target-shaping limits, not proof of aircraft capability.
    """

    max_velocity: float
    max_acceleration: float
    max_jerk: float
    position_gain: float = 1.0

    def __post_init__(self) -> None:
        for name in ("max_velocity", "max_acceleration", "max_jerk",
                     "position_gain"):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive")


@dataclass
class AxisState:
    """The shaped position/velocity/acceleration of one axis."""

    position: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0


class AxisShaper:
    """A small jerk-limited position-to-velocity-to-acceleration cascade.

    This is intentionally simple rather than a time-optimal S-curve solver:
    position error proposes a velocity, velocity error proposes an
    acceleration, and acceleration is slew-limited by jerk. That is enough to
    remove target steps without hiding the deterministic limits inside an
    opaque planner.
    """

    def __init__(self, limits: TrajectoryLimits, position: float = 0.0,
                 velocity: float = 0.0):
        self.limits = limits
        self.state = AxisState(position, velocity, 0.0)

    def reset(self, position: float, velocity: float = 0.0,
              acceleration: float = 0.0) -> None:
        self.state = AxisState(position, velocity, acceleration)

    def update(self, target_position: float, dt: float) -> AxisState:
        """Advance the shaped axis towards ``target_position``.

        A nonpositive or very large ``dt`` is ignored as a paused/discontinuous
        simulation sample rather than allowed to inject a control step.
        """
        if dt <= 0.0 or dt > 5.0:
            return self.state
        lim = self.limits
        state = self.state
        error = target_position - state.position
        desired_velocity = clamp(error * lim.position_gain,
                                 -lim.max_velocity, lim.max_velocity)
        # Do not command a velocity that cannot be removed inside the gap:
        # sqrt(2 a x), with a floor so numerical noise does not stop motion.
        braking_velocity = math.sqrt(max(0.0, 2.0 * lim.max_acceleration
                                         * abs(error)))
        sign = 1.0 if error >= 0.0 else -1.0
        desired_velocity = sign * min(abs(desired_velocity),
                                      max(braking_velocity, 0.05))
        desired_acceleration = clamp(
            (desired_velocity - state.velocity) / max(dt, 0.05),
            -lim.max_acceleration, lim.max_acceleration)
        accel_step = lim.max_jerk * dt
        state.acceleration += clamp(desired_acceleration - state.acceleration,
                                    -accel_step, accel_step)
        state.velocity += state.acceleration * dt
        state.velocity = clamp(state.velocity,
                               -lim.max_velocity, lim.max_velocity)
        # Clamp the update so shaping never crosses the target in one sample.
        movement = state.velocity * dt
        if error and abs(movement) > abs(error):
            movement = error
            state.velocity = movement / dt
            state.acceleration = 0.0
        state.position += movement
        return state


@dataclass(frozen=True)
class PathGuidance:
    """L1-style guidance for one horizontal path leg."""

    desired_track_deg: float
    cross_track_m: float
    along_track_m: float
    lateral_accel_mss: float
    bank_deg: float


def _local_xy_m(lat: float, lon: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """East/north metres relative to a nearby reference point."""
    m_per_deg_lat = 111320.0
    m_per_deg_lon = m_per_deg_lat * math.cos(math.radians(ref_lat))
    dlon = (lon - ref_lon + 180.0) % 360.0 - 180.0
    return dlon * m_per_deg_lon, (lat - ref_lat) * m_per_deg_lat


def l1_guidance(lat: float, lon: float,
                origin_lat: float, origin_lon: float,
                dest_lat: float, dest_lon: float,
                speed_kt: float, *, period_s: float = 20.0,
                damping: float = 0.8,
                max_bank_deg: float = 25.0) -> PathGuidance | None:
    """Track guidance for a straight leg, in the spirit of ArduPilot's L1.

    Pure pursuit alone aims at a point and can keep re-cutting corners. L1
    combines cross-track error with the angle to a lookahead point and turns
    the result into a bounded lateral acceleration/bank demand. The values are
    advisory outputs for Meridian's deterministic controllers; nothing writes
    a simulator control here.
    """
    if speed_kt <= 1.0 or period_s <= 1.0 or damping <= 0.0:
        return None
    ax, ay = _local_xy_m(origin_lat, origin_lon, origin_lat, origin_lon)
    bx, by = _local_xy_m(dest_lat, dest_lon, origin_lat, origin_lon)
    px, py = _local_xy_m(lat, lon, origin_lat, origin_lon)
    dx, dy = bx - ax, by - ay
    leg_m = math.hypot(dx, dy)
    if leg_m < 1.0:
        return None
    ux, uy = dx / leg_m, dy / leg_m
    rx, ry = px - ax, py - ay
    along = rx * ux + ry * uy
    # Positive means right of the leg in the aviation heading frame: for a
    # northbound leg, an aircraft east of the line is to its right.
    cross = rx * uy - ry * ux
    path_angle = math.atan2(ux, uy)  # aviation bearing: clockwise from north
    speed_mps = max(1.0, speed_kt * KT_TO_MPS)
    l1_m = max(30.0, speed_mps * period_s / math.pi)
    # Cross-track correction becomes shallower with damping. The sign turns a
    # right-of-path error back towards the path.
    correction = math.atan2(damping * cross, l1_m)
    desired = math.degrees(path_angle - correction) % 360.0
    angle_error = math.radians((desired - math.degrees(path_angle) + 540.0) % 360.0 - 180.0)
    lateral_accel = clamp(2.0 * speed_mps * speed_mps / l1_m
                          * math.sin(angle_error), -G_MSS, G_MSS)
    bank = math.degrees(math.atan2(lateral_accel, G_MSS))
    bank = clamp(bank, -abs(max_bank_deg), abs(max_bank_deg))
    return PathGuidance(desired_track_deg=desired,
                        cross_track_m=cross,
                        along_track_m=along,
                        lateral_accel_mss=lateral_accel,
                        bank_deg=bank)


def stopping_speed_kt(distance_nm: float, final_kt: float,
                      decel_kt_s: float, cap_kt: float) -> float:
    """The speed that can still be removed before a point.

    A constant-deceleration kinematic schedule: ``sqrt(v_f^2 + 2 a x)``. It
    is deliberately conservative in low speed/short distance cases and never
    exceeds ``cap_kt``. Rotorcraft spot approaches and traffic spacing both
    need this shape of answer.
    """
    if distance_nm <= 0.0:
        return max(0.0, final_kt)
    if decel_kt_s <= 0.0 or cap_kt <= 0.0:
        return max(0.0, min(final_kt, cap_kt))
    distance_kt_s = distance_nm * 3600.0  # knots times seconds, matching v²+2ax
    speed = math.sqrt(max(0.0, final_kt * final_kt
                          + 2.0 * decel_kt_s * distance_kt_s))
    return clamp(speed, max(0.0, final_kt), cap_kt)


@dataclass(frozen=True)
class EnergyCommand:
    """A simplified fixed-wing total-energy speed/height allocation.

    ``throttle`` is the total-energy correction, positive for more energy;
    ``pitch`` is the distribution correction, positive for nose-up/height.
    ``priority`` is ``speed`` whenever underspeed protection must dominate.
    Values are normalised suggestions for deterministic controllers, not raw
    control outputs.
    """

    throttle: float
    pitch: float
    priority: str
    energy_error_ft: float


def speed_energy_ft(speed_error_kt: float, speed_kt: float) -> float:
    """Approximate height-equivalent energy of a speed error, in feet."""
    speed_kt = max(40.0, speed_kt)
    # E = v²/2g. Linearised for a small error about the current speed,
    # converted metres to feet.
    return (speed_error_kt * KT_TO_MPS * speed_kt * KT_TO_MPS / G_MSS) * 3.28084


def energy_speed_height(height_error_ft: float, speed_error_kt: float,
                        speed_kt: float, *, underspeed_margin_kt: float = 5.0,
                        max_correction_ft: float = 500.0) -> EnergyCommand:
    """Allocate a height/speed error between throttle and pitch.

    This is the control *concept* behind TECS, not a port: throttle corrects
    total energy while pitch distributes energy between speed and height. A
    real underspeed condition is always speed-priority.
    """
    energy_ft = height_error_ft + speed_energy_ft(speed_error_kt, speed_kt)
    energy_ft = clamp(energy_ft, -max_correction_ft, max_correction_ft)
    height = clamp(height_error_ft, -max_correction_ft, max_correction_ft)
    throttle = clamp(energy_ft / max_correction_ft, -1.0, 1.0)
    pitch = clamp(height / max_correction_ft, -1.0, 1.0)
    priority = "speed" if speed_error_kt < -abs(underspeed_margin_kt) \
        else "balanced"
    if priority == "speed":
        throttle = max(throttle, 0.5)
        pitch = min(pitch, -0.15)
    return EnergyCommand(throttle=throttle, pitch=pitch, priority=priority,
                         energy_error_ft=energy_ft)


__all__ = [
    "AxisShaper", "AxisState", "EnergyCommand", "PathGuidance",
    "TrajectoryLimits", "energy_speed_height", "l1_guidance",
    "speed_energy_ft", "stopping_speed_kt",
]
