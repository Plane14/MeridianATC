"""X-Plane's own aircraft as the traffic renderer.

Instead of XPMP2/CSL instances the plugin acquires the simulator's AI-plane
slots, loads a default-fleet ``.acf`` into each and holds the AI-autopilot
override down; this module is the pilot for them. Each slot is steered
through the ``sim/multiplayer/controls`` datarefs -- the same raw channels
the FO writes for the pilot's own aircraft -- by the same PID laws, reading
the slot's real position back over ``sim/multiplayer/position/planeN_*``.
The flight model stays the simulator's: weight, engines, gear, rotors.

The plugin owns what no dataref can do (acquiring planes, picking .acf,
latching the override); this class owns the roster that tells it which
callsign each slot is. An empty roster hands the planes back.
"""

from __future__ import annotations

import logging
import math
import time

from ..atc.control import Pid, PidTerms
from ..atc.control_gains import fixed_wing_gains, rotor_gains
from ..atc.flying import RATE_LIGHT, Performance
from .xptraffic import XPlaneTraffic, _number, _text

log = logging.getLogger(__name__)

_CONTROLS = "sim/multiplayer/controls"
_POSITION = "sim/multiplayer/position"

# ICAO type designators that are rotorcraft: the pilots for these get the
# rotor law, and the plugin maps them onto the S-76/R22 default models.
HELICOPTER_TYPES = frozenset({
    "R22", "R44", "R66", "S76", "B407", "B429", "B06", "B412",
    "AS50", "AS55", "AS32", "EC30", "EC35", "EC45", "A109", "A139",
    "MD52", "MD60", "UH1", "H500", "H160", "H145", "H47", "H53", "H64",
})

# The channels one slot is fed back over RREF. ``ele`` arrives in metres.
_FEEDBACK = ("lat", "lon", "ele", "psi", "phi")

_MPS_TO_KT = 1.94384
_M_TO_FT = 3.28084


def _clamp(value, low, high):
    return max(low, min(high, value))


class NativePilot:
    """Flies one AI slot toward its target motion.

    The laws are the FO's: a vertical channel that turns a height error
    into a vertical-speed demand and a rotation channel that flies the
    demand, a throttle channel for speed, a steer channel for ground
    heading -- each with the gain bundle the First Officer uses for the
    class. Lateral flight is bank through ``yoke_roll_ratio``, the piece
    the FO never needed on the pilot's own aeroplane.
    """

    # A grounded target asks for parking brake under this speed.
    HOLD_KT = 0.5

    def __init__(self, type_code, helicopter=False):
        self.type_code = type_code
        self.helicopter = helicopter
        # The loops advance on the caller's clock -- the engine's tick --
        # not on wall time, so a paused or slow sim does not poison the
        # integrals either.
        self._now = time.monotonic()

        def clock():
            return self._now
        if helicopter:
            gains = rotor_gains(type_code)
            # Collective is percent of lever per collective-gain output;
            # the lever itself is written as a 0..1 request.
            self._collective = Pid(gains.collective, clock=clock)
            self._pedals = Pid(gains.pedals, clock=clock)
        else:
            # The class guess from the type's own family: heavies fly the
            # airliner set, light singles the light one, the rest the
            # turboprop's. ``approach_kt`` decides airliner vs turboprop.
            heavy = type_code[:1] in ("A", "B", "E", "D", "M") \
                and type_code not in ("EVOT", "A10")
            perf = Performance(
                turn_rate=RATE_LIGHT if not heavy else 3.0,
                approach_kt=140.0 if heavy else 95.0)
            gains = fixed_wing_gains(perf)
            self._throttle = Pid(gains.throttle, clock=clock)
            self._steer = Pid(gains.steer, clock=clock)
            self._taxi = Pid(gains.taxi, clock=clock)
            self._pitch = Pid(gains.rotation, clock=clock)
            self._vert = Pid(gains.vertical, clock=clock)
        # The bank loop: degrees of heading error to degrees of bank, then
        # degrees of bank error to the roll lever. Gentle, like the rest.
        self._roll = Pid(PidTerms(kp=1.0 / 25.0, ki=0.002, kd=1.0 / 60.0,
                                  low=-1.0, high=1.0, slew=0.8),
                         clock=clock)
        self._prev_geo = None      # (lat, lon, ele_m, t) for gs/vs estimates
        self._est_gs_kt = 0.0
        self._est_vs_fpm = 0.0

    def _estimate(self, m, now):
        """GS (kt) and VS (fpm) from position samples; the slots carry no
        speed datarefs."""
        lat, lon, ele = m.get("lat"), m.get("lon"), m.get("ele")
        if lat is None or lon is None or ele is None:
            return self._est_gs_kt, self._est_vs_fpm
        prev = self._prev_geo
        self._prev_geo = (lat, lon, ele, now)
        if prev is None or now - prev[3] <= 0:
            return self._est_gs_kt, self._est_vs_fpm
        dt = now - prev[3]
        # Local flat-earth metres around the sample.
        north = (lat - prev[0]) * 111320.0
        east = (lon - prev[1]) * 111320.0 * math.cos(math.radians(lat))
        gs = math.hypot(north, east) / dt * _MPS_TO_KT
        vs = (ele - prev[2]) * _M_TO_FT / dt * 60.0
        # New samples are halved in, so one jump does not jerk the levers.
        self._est_gs_kt += (gs - self._est_gs_kt) * 0.5
        self._est_vs_fpm += (_clamp(vs, -6000.0, 6000.0) - self._est_vs_fpm) * 0.5
        return self._est_gs_kt, self._est_vs_fpm

    def update(self, target, m, now):
        """The control writes for this slot, as dataref-path -> value.

        ``target`` is the injector's position dict (motion under
        ``"motion"``); ``m`` the slot's measured lat/lon/ele(m)/psi/phi,
        any of which may be None until the first samples arrive.
        """
        self._now = now
        motion = target.get("motion") or {}
        ground = bool(motion.get("on_ground", target.get("on_ground")))
        phase = target.get("phase", "")
        parked = phase == "parked" or bool(target.get("static"))
        t_alt = _number(motion.get("alt", target.get("altitude_ft", 0)),
                        -2000, 100000)
        t_hdg = _number(motion.get("heading", target.get("heading", 0)),
                        -1e9, 1e9) % 360
        t_gs = _number(motion.get("gs", target.get("ground_speed_kt", 0)),
                       0, 2000)
        t_vs = _number(motion.get("vs", target.get("vertical_rate_fpm", 0)),
                       -30000, 30000)

        slot = target["_slot"]
        yoke_pitch = f"{_CONTROLS}/yoke_pitch_ratio[{slot}]"
        yoke_roll = f"{_CONTROLS}/yoke_roll_ratio[{slot}]"
        yoke_hdg = f"{_CONTROLS}/yoke_heading_ratio[{slot}]"
        throttle = f"{_CONTROLS}/engine_throttle_request[{slot}]"
        out = {}

        # Gear, flaps, brakes and lights come straight off the target
        # state -- the same table aircraft_packet uses for CSL.
        runway = phase in ("final", "takeoff", "lineup", "landed",
                           "go_around")
        out[f"{_CONTROLS}/gear_request[{slot}]"] = \
            1.0 if ground or phase in ("final", "landed") else 0.0
        out[f"{_CONTROLS}/flap_request[{slot}]"] = \
            1.0 if phase == "final" else 0.3 if runway else 0.0
        lights_on = 0.0 if parked else 1.0
        for light in ("beacon_lights_on", "nav_lights_on"):
            out[f"{_POSITION}/plane{slot}_{light}"] = lights_on
        out[f"{_POSITION}/plane{slot}_landing_lights_on"] = \
            0.0 if parked else (1.0 if runway or not ground and t_alt < 10000 else 0.0)

        if parked or (ground and t_gs <= self.HOLD_KT):
            out[f"{_CONTROLS}/parking_brake[{slot}]"] = 1.0
            out[throttle] = 0.0
            out[yoke_pitch] = out[yoke_roll] = out[yoke_hdg] = 0.0
            return out
        out[f"{_CONTROLS}/parking_brake[{slot}]"] = 0.0

        gs, vs = self._estimate(m, now)
        psi = m.get("psi")
        phi = m.get("phi") or 0.0
        ele_ft = (m.get("ele") or 0.0) * _M_TO_FT if m.get("ele") is not None else None
        have_heading = psi is not None
        hdg_err = ((t_hdg - psi) + 180.0) % 360 - 180.0 if have_heading else 0.0
        alt_err = 0.0 if ele_ft is None else t_alt - ele_ft

        if self.helicopter:
            return self._heli_writes(out, slot, ground, parked,
                                     t_hdg, t_gs, t_vs, hdg_err,
                                     alt_err, gs, have_heading,
                                     throttle, yoke_pitch, yoke_roll,
                                     yoke_hdg)

        if ground:
            # Taxi: heading error to the nosewheel/rudder channel, speed
            # to the throttle. The FO's steer channel already caps and
            # fades; the lever is modest because an AI slot at full taxi
            # thrust is a hazard to itself.
            out[yoke_hdg] = self._steer.update(hdg_err) if have_heading else 0.0
            out[throttle] = _clamp(self._taxi.update(t_gs - gs), 0.0, 0.6)
            out[yoke_pitch] = out[yoke_roll] = 0.0
            if t_gs <= self.HOLD_KT + 1.0 and gs > t_gs + 1.0:
                out[f"{_CONTROLS}/l_brake_add[{slot}]"] = 1.0
                out[f"{_CONTROLS}/r_brake_add[{slot}]"] = 1.0
            return out

        # Airborne: heading error to a bank demand, bank error to the
        # roll lever; height error to a VS demand, VS error to the pitch
        # lever; speed to the throttle.
        bank = _clamp(hdg_err * 1.5, -25.0, 25.0)
        out[yoke_roll] = self._roll.update(bank - phi) \
            if have_heading else 0.0
        vs_demand = self._vert.update(alt_err) if ele_ft is not None else t_vs
        out[yoke_pitch] = self._pitch.update(vs_demand - vs)
        out[throttle] = self._throttle.update(t_gs - gs)
        return out

    def _heli_writes(self, out, slot, ground, parked, t_hdg, t_gs, t_vs,
                     hdg_err, alt_err, gs, have_heading,
                     throttle, yoke_pitch, yoke_roll, yoke_hdg):
        """Rotorcraft law: altitude to the collective (the throttle
        channel, percent of lever), forward speed to longitudinal cyclic,
        heading to the pedals with a little cyclic roll to help the turn."""
        collective = self._collective.update(alt_err) / 100.0
        out[throttle] = _clamp(0.5 + collective, 0.0, 1.0) \
            if not ground else _clamp(collective, 0.0, 0.4)
        out[yoke_pitch] = -_clamp((t_gs - gs) * 0.008, -0.25, 0.25)
        out[yoke_hdg] = _clamp(self._pedals.update(hdg_err) / 5000.0,
                               -0.5, 0.5) if have_heading else 0.0
        out[yoke_roll] = _clamp(hdg_err * 0.004, -0.15, 0.15) \
            if have_heading else 0.0
        if ground and t_gs <= self.HOLD_KT:
            out[f"{_CONTROLS}/parking_brake[{slot}]"] = 1.0
        return out


class XPNativeTraffic(XPlaneTraffic):
    """The UDP peer unchanged; the payload a roster instead of drawings.

    Nothing is sent on the ``traffic`` channel: the plugin reads the
    ``native`` roster, loads the default-fleet models and hands the slots
    to the flight model; this class writes each slot's controls and
    watches its position come back.
    """

    # X-Plane has 19 usable AI slots (index 0 is the pilot's own plane).
    MAX_SLOTS = 19

    def __init__(self, *args, source=None, **kwargs):
        super().__init__(*args, **kwargs)
        # The XPlaneSource: dref() writes control channels, subscribe()
        # adds each slot's feedback datarefs, value() reads them back.
        self.source = source
        self._slots = {}          # id -> slot index (1..MAX_SLOTS)
        self._pilots = {}         # id -> NativePilot
        self._roster = ()         # the (slot, id, type) tuples last sent
        self._roster_at = 0.0
        self._assigned_at = {}    # id -> when its slot was claimed

    @property
    def native(self):
        return True

    @property
    def drawing(self):
        # A CSL renderer is "drawing" when it has models; the native one
        # when the plugin says it can drive the AI slots.
        return self.available and bool(self._status.get("native_cap"))

    @property
    def count(self):
        return len([i for i in self._wanted
                    if i in self._drawn]) if self.available else 0

    def has(self, callsign):
        return (self.available and callsign in self._wanted
                and callsign in self._drawn)

    def failed(self, callsign):
        self.on_frame()
        asked = self._wanted.get(callsign)
        if asked is None:
            return False
        if not self.available:
            return time.monotonic() - asked >= self.ANSWER_WITHIN_S
        # Over the slot cap is decided at roster time, not waited out.
        if callsign not in self._slots and len(self._slots) >= self.MAX_SLOTS:
            return True
        return (callsign not in self._drawn
                and time.monotonic() - asked >= self.ANSWER_WITHIN_S)

    def _subscribe_slot(self, slot):
        if self.source is None:
            return
        for key in _FEEDBACK:
            self.source.subscribe(
                f"xpn{slot}:{key}",
                f"{_POSITION}/plane{slot}_{key}")

    def _send_roster(self, roster, now):
        if roster == self._roster and now - self._roster_at < self.HELLO_EVERY_S:
            return
        if self._send({"v": 1, "t": "native",
                       "ac": [{"id": ident, "slot": slot, "type": tcode}
                              for slot, ident, tcode in roster]}):
            self._roster = roster
            self._roster_at = now

    def sync(self, positions, now=None):
        with self._lock:
            self.on_frame()
            if self._socket is None:
                return
            now = time.monotonic()

            # Keep the slots already flying somebody, then fill the rest
            # in the order the injector offered -- nearest first.
            taken = {slot for slot, _ident, _tc in self._roster}
            wanted = {}
            roster = []
            for position in positions:
                try:
                    ident = _text(position["callsign"], 64, True)
                    tcode = _text(position.get("type") or "C172", 4, True)
                except (ValueError, TypeError, KeyError):
                    continue
                if ident in wanted:
                    continue
                wanted[ident] = self._wanted.get(ident, now)
                slot = self._slots.get(ident)
                if slot is None and len(taken) < self.MAX_SLOTS:
                    slot = next(s for s in range(1, self.MAX_SLOTS + 1)
                                if s not in taken)
                    self._slots[ident] = slot
                    self._assigned_at[ident] = now
                    self._subscribe_slot(slot)
                if slot is None:
                    continue
                taken.add(slot)
                roster.append((slot, ident, tcode))
                position["_slot"] = slot
            roster.sort()

            # Free the slots nobody claimed this tick, and let their
            # levers go neutral so a still-modelled plane does not fly
            # off on the last demand.
            for ident in [i for i in self._slots
                          if not any(r[1] == i for r in roster)]:
                slot = self._slots.pop(ident)
                self._pilots.pop(ident, None)
                self._assigned_at.pop(ident, None)
                if self.source is not None:
                    for chan in ("yoke_pitch_ratio", "yoke_roll_ratio",
                                 "yoke_heading_ratio",
                                 "engine_throttle_request",
                                 "parking_brake"):
                        self.source.dref(f"{_CONTROLS}/{chan}[{slot}]", 0.0)

            self._send_roster(tuple(roster), now)

            # Pilot each assigned slot.
            by_id = {position["callsign"]: position for position in positions}
            for slot, ident, tcode in roster:
                position = by_id.get(ident)
                if position is None:
                    continue
                pilot = self._pilots.get(ident)
                if pilot is None or pilot.type_code != tcode:
                    pilot = self._pilots[ident] = NativePilot(
                        tcode, helicopter=tcode in HELICOPTER_TYPES)
                measured = {}
                if self.source is not None:
                    for key in _FEEDBACK:
                        measured[key] = self.source.value(f"xpn{slot}:{key}")
                writes = pilot.update(position, measured, now)
                if self.source is not None:
                    for path, value in writes.items():
                        self.source.dref(path, value)

            # A slot is "drawn" once the model has had time to load and
            # the first samples come back -- the settle the CSL renderer
            # waits through before admitting a callsign to the frequency.
            self._drawn = {ident for ident in wanted
                           if ident in self._slots
                           and now - self._assigned_at.get(ident, now)
                           >= self.settle_s}
            self._wanted = wanted

    def detach(self):
        # An empty roster releases the planes before the socket goes.
        with self._lock:
            if self._socket is not None:
                self._send({"v": 1, "t": "native", "ac": []})
            self._slots.clear()
            self._pilots.clear()
            self._assigned_at.clear()
            self._roster = ()
        super().detach()
