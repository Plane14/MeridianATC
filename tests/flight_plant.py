"""A small aeroplane the First Officer can actually fly in a test.

Not a flight model: a point mass with first-order answers to the
autopilot bugs and the hand-flown axes, enough for the FO's guidance to
be run closed-loop -- so "does it reach the runway" is asserted against
motion, not against which events were sent.
"""
from __future__ import annotations

import math

from meridianatc.navdata.build import project
from meridianatc.navdata.magvar import to_magnetic, variation
from meridianatc.sim.state import AircraftState

FT_PER_NM = 6076.12


class Plant:
    def __init__(self, lat, lon, alt_ft, heading, ias_kt, field_elev_ft=0.0,
                 wind_from=0.0, wind_kt=0.0, stall_kt=50.0,
                 autothrottle=False):
        self.lat, self.lon, self.alt = lat, lon, alt_ft
        self.hdg, self.ias, self.vs = heading, ias_kt, 0.0
        self.elev = field_elev_ft
        self.wind_from, self.wind_kt = wind_from, wind_kt
        self.stall_kt = stall_kt
        self.vref = ias_kt
        self.alt_bug = None
        self.autothrottle = autothrottle
        self.on_ground = False
        self.ap = False
        self.hdg_bug = heading
        self.vs_bug = 0.0
        self.vs_mode = False
        self.spd_bug = ias_kt
        self.lever = 0.4
        self.elev_cmd = 0.0
        self.rudder_cmd = 0.0
        self.brake = 0.0
        self.flaps = 0
        self.gear = True
        self.bank = 0.0
        self.yaw_rate = 0.0
        self.touchdown_fpm = None
        self.touchdown_offset_m = None
        self.min_agl_airborne = 1e9
        self.min_ias_airborne = 1e9
        self.t = 0.0
        self.var = variation(lat, lon)

    # -- the state the FO reads ------------------------------------------

    def ground_vector(self):
        """(north kt, east kt) over the ground."""
        h = math.radians(self.hdg)
        if self.on_ground:
            # Wheels on the ground: it goes where it points.
            return (self.ias * math.cos(h), self.ias * math.sin(h))
        w = math.radians(self.wind_from)
        return (self.ias * math.cos(h) - self.wind_kt * math.cos(w),
                self.ias * math.sin(h) - self.wind_kt * math.sin(w))

    def state(self) -> AircraftState:
        n, e = self.ground_vector()
        gs = math.hypot(n, e)
        return AircraftState(
            latitude=self.lat, longitude=self.lon, altitude_ft=self.alt,
            indicated_altitude_ft=self.alt,
            altitude_agl_ft=max(0.0, self.alt - self.elev),
            heading_true=self.hdg % 360.0,
            heading_magnetic=to_magnetic(self.hdg, self.var) % 360.0,
            track_true=math.degrees(math.atan2(e, n)) % 360.0,
            ground_speed_kt=gs, indicated_airspeed_kt=self.ias,
            true_airspeed_kt=self.ias, vertical_speed_fpm=self.vs,
            on_ground=self.on_ground, gear_down=self.gear,
            flaps_index=self.flaps, autopilot=self.ap,
            engines_running=True, engine_count=1,
            type_code=getattr(self, "type_code", ""), source="simconnect")

    # -- physics ---------------------------------------------------------

    def step(self, dt: float) -> None:
        self.t += dt
        if self.on_ground:
            self.yaw_rate += (self.rudder_cmd * 30.0 - self.yaw_rate) \
                * min(1.0, dt * 3.0)
            self.hdg += self.yaw_rate * dt
            decel = 1.0 + self.brake * 8.0 - self.lever * 6.0
            self.ias = max(0.0, self.ias - decel * dt)
            self.vs = 0.0
        else:
            if self.ap:
                err = (self.hdg_bug - to_magnetic(self.hdg, self.var)
                       + 540.0) % 360.0 - 180.0
                want_bank = max(-25.0, min(25.0, err * 1.5))
                self.bank += (want_bank - self.bank) * min(1.0, dt * 1.0)
                tas_ms = max(20.0, self.ias * 0.5144)
                self.hdg += math.degrees(
                    9.81 * math.tan(math.radians(self.bank)) / tas_ms) * dt
                want_vs = self.vs_bug if self.vs_mode else 0.0
                if self.alt_bug is not None and want_vs:
                    # Altitude capture: the rate washes out into the
                    # selected altitude instead of flying through it.
                    to_go = self.alt_bug - self.alt
                    if to_go * want_vs > 0.0:
                        want_vs = math.copysign(
                            min(abs(want_vs), abs(to_go) * 6.0), want_vs)
                self.vs += (want_vs - self.vs) * min(1.0, dt * 0.5)
            else:
                self.bank *= max(0.0, 1.0 - dt)
                self.hdg += self.rudder_cmd * 10.0 * dt
                # Back pressure arrests the sink; released, the nose
                # drops toward the approach's descent again.
                self.vs += (self.elev_cmd * 2500.0 - 250.0) * dt
                self.vs = max(-1500.0, min(self.vs, 1500.0))
            # Thrust against drag, and the energy the path trades.
            lever = self.lever
            if self.autothrottle and self.ap:
                lever = max(0.0, min(1.0, 0.45 + (self.spd_bug - self.ias)
                                     * 0.05))
            drag = 1.5 * (self.ias / self.vref) ** 2 * (
                1.0 + 0.25 * self.flaps + (0.1 if self.gear else 0.0))
            accel = 4.0 * lever - drag - self.vs / 600.0
            self.ias = max(0.0, self.ias + accel * dt)
            self.alt += self.vs / 60.0 * dt
            agl = self.alt - self.elev
            if agl <= 0.0:
                self.alt = self.elev
                self.on_ground = True
                self.touchdown_fpm = self.vs
                self.vs = 0.0
            else:
                self.min_agl_airborne = min(self.min_agl_airborne, agl)
                self.min_ias_airborne = min(self.min_ias_airborne, self.ias)
        n, e = self.ground_vector()
        gs = math.hypot(n, e)
        if gs > 0.01:
            track = math.degrees(math.atan2(e, n)) % 360.0
            self.lat, self.lon = project(self.lat, self.lon, track,
                                         gs * dt / 3600.0)


class PlantDirector:
    """The FlightDirector's surface, wired to the plant. Anything the
    plant has no use for is accepted and ignored."""

    mode = "direct"
    sim = None

    def __init__(self, plant: Plant):
        self.p = plant

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return lambda *a, **k: True

    def autopilot(self, on=True):
        self.p.ap = bool(on)
        return True

    def heading(self, degrees):
        self.p.hdg_bug = float(degrees)
        return True

    def altitude(self, feet):
        self.p.alt_bug = float(feet)
        return True

    def vertical_speed(self, fpm):
        self.p.vs_bug = float(fpm)
        return True

    def vs_hold(self, on=True):
        self.p.vs_mode = bool(on)
        return True

    def speed(self, knots):
        self.p.spd_bug = float(knots)
        return True

    def throttle(self, position):
        self.p.lever = max(0.0, min(1.0, float(position)))
        return True

    def throttle_idle(self):
        return self.throttle(0.0)

    def elevator(self, position):
        self.p.elev_cmd = float(position)
        return True

    def rudder(self, position):
        self.p.rudder_cmd = float(position)
        return True

    def brakes(self, position):
        self.p.brake = float(position)
        return True

    def flaps(self, position):
        self.p.flaps = int(round(float(position) * 3))
        return True

    def gear(self, down):
        self.p.gear = bool(down)
        return True
