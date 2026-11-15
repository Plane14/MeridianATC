"""A small helicopter the rotorcraft controller can actually fly in a test.

Not a flight model: a point mass whose climb answers the collective,
whose speed along and across the nose answers the cyclic, and whose
heading answers the pedals -- enough to ask where the helicopter ends up
rather than which events were sent. Positive longitudinal cyclic is aft,
positive lateral cyclic is right, positive pedal is right: the yoke
conventions the X-Plane bridge lands these axes on.
"""
from __future__ import annotations

import math

from meridianatc.navdata.build import project
from meridianatc.sim.state import AircraftState

HOVER_COLLECTIVE_PCT = 55.0


class HeliPlant:
    def __init__(self, lat, lon, agl_ft, heading, speed_kt=0.0,
                 field_elev_ft=0.0, engine=True):
        self.lat, self.lon = lat, lon
        self.elev = field_elev_ft
        self.alt = field_elev_ft + agl_ft
        self.hdg = heading
        self.u, self.v, self.vs = speed_kt, 0.0, 0.0   # along, right, fpm
        self.coll = HOVER_COLLECTIVE_PCT
        self.cyc_long = self.cyc_lat = self.pedal = 0
        self.engine = engine
        self.rpm = 100.0
        self.on_ground = agl_ft <= 0.0
        self.touchdown = None
        self.t = 0.0
        self.sent: list = []

    # -- what the controller writes ---------------------------------------

    def send(self, name: bytes, value: int = 0) -> bool:
        self.sent.append((name, value))
        if name == b"AXIS_COLLECTIVE_SET":
            self.coll = (value + 16383.0) / 32766.0 * 100.0
        elif name == b"AXIS_CYCLIC_LONGITUDINAL_SET":
            self.cyc_long = value
        elif name == b"AXIS_CYCLIC_LATERAL_SET":
            self.cyc_lat = value
        elif name in (b"AXIS_RUDDER_SET", b"ROTOR_AXIS_TAIL_ROTOR_SET",
                      b"AXIS_TAIL_ROTOR_SET"):
            self.pedal = value
        return True

    # -- what it reads -----------------------------------------------------

    def ground_vector(self):
        h = math.radians(self.hdg)
        return (self.u * math.cos(h) - self.v * math.sin(h),
                self.u * math.sin(h) + self.v * math.cos(h))

    def state(self) -> AircraftState:
        n, e = self.ground_vector()
        gs = math.hypot(n, e)
        return AircraftState(
            latitude=self.lat, longitude=self.lon, altitude_ft=self.alt,
            indicated_altitude_ft=self.alt,
            altitude_agl_ft=max(0.0, self.alt - self.elev),
            heading_true=self.hdg % 360.0, heading_magnetic=self.hdg % 360.0,
            track_true=(math.degrees(math.atan2(e, n)) % 360.0
                        if gs > 0.5 else self.hdg % 360.0),
            ground_speed_kt=gs, indicated_airspeed_kt=max(0.0, self.u),
            true_airspeed_kt=max(0.0, self.u), vertical_speed_fpm=self.vs,
            on_ground=self.on_ground, engines_running=self.engine,
            engine_count=1, collective_position=self.coll,
            rotor_rpm_pct=self.rpm, num_main_rotors=1,
            eng_torque_pct=self.coll, connected=True,
            source="simconnect")

    # -- physics -----------------------------------------------------------

    def step(self, dt: float) -> None:
        self.t += dt
        if self.on_ground:
            self.vs = 0.0
            self.u *= max(0.0, 1.0 - 2.0 * dt)
            self.v *= max(0.0, 1.0 - 2.0 * dt)
            if self.engine and self.coll > HOVER_COLLECTIVE_PCT + 2.0:
                self.on_ground = False
            else:
                self._move(dt)
                return
        if self.engine:
            self.rpm += (100.0 - self.rpm) * min(1.0, dt)
            want_vs = (self.coll - HOVER_COLLECTIVE_PCT) * 60.0
        else:
            # Power off: the lever is the rotor's brake. Down, the rotor
            # is driven by the descent; up, it spends what it has.
            self.rpm += ((30.0 - self.coll) * 0.5
                         + max(0.0, -self.vs) / 600.0) * dt
            self.rpm = max(0.0, min(110.0, self.rpm))
            lift = self.rpm / 100.0
            want_vs = -1800.0 + (self.coll * 30.0) * lift * lift
        self.vs += (want_vs - self.vs) * min(1.0, dt / 1.2)
        self.u += (-self.cyc_long / 6000.0 * 6.0 - 0.03 * self.u) * dt
        self.v += (self.cyc_lat / 6000.0 * 6.0 - 0.25 * self.v) * dt
        self.hdg = (self.hdg + self.pedal / 5000.0 * 20.0 * dt) % 360.0
        self._move(dt)
        self.alt += self.vs / 60.0 * dt
        if self.alt <= self.elev:
            self.alt = self.elev
            if self.touchdown is None:
                self.touchdown = dict(
                    vs=self.vs, u=self.u, v=self.v, lat=self.lat,
                    lon=self.lon, t=self.t)
            self.on_ground = True
            self.vs = 0.0

    def _move(self, dt: float) -> None:
        n, e = self.ground_vector()
        gs = math.hypot(n, e)
        if gs > 0.01:
            self.lat, self.lon = project(
                self.lat, self.lon, math.degrees(math.atan2(e, n)) % 360.0,
                gs * dt / 3600.0)
