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

import json
import logging
import math
import time

from ..atc import perftable
from ..atc.control import Pid, PidTerms
from ..atc.control_gains import fixed_wing_gains, rotor_gains
from .xptraffic import XPlaneTraffic, _number, _text, aircraft_packet

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

# Rotorcraft types whose gear is wheels rather than skids. Everything
# else in HELICOPTER_TYPES hover-taxis because it cannot roll.
WHEELED_HELICOPTERS = frozenset({
    "S76", "A109", "A139", "H160", "H47", "H53", "H64", "AS32",
})

# The channels one slot is fed back over RREF. ``ele`` arrives in metres,
# ``v_*`` the slot's cartesian velocities in m/s (x east, y up, z south),
# ``the``/``phi`` its pitch and roll in degrees.
_FEEDBACK = ("lat", "lon", "ele", "psi", "phi", "the", "v_x", "v_y", "v_z")

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
    # Skid-equipped rotorcraft taxi on a low hover: this much altitude
    # above the surface keeps the skids clear of it.
    HOVER_TAXI_FT = 5.0
    # Rotorcraft climb away vertically this high before the forward
    # speed demand is allowed to open up.
    TRANSLATE_FT = 40.0
    TRANSLATE_KT = 25.0

    def __init__(self, type_code, helicopter=False):
        self.type_code = type_code
        self.helicopter = helicopter
        self._skids = helicopter and type_code not in WHEELED_HELICOPTERS
        self._surface_ft = None  # last ele seen on the ground, for AGL
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
            # The type's own published figures, the same ones the motion
            # model flies: the first letter is a weak guess next to what
            # the table knows (a CRJ9 is no 737, a King Air no A380).
            perf = perftable.traffic_performance(
                type_code, light=type_code[:1] not in ("A", "B", "E", "M"))
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
        self._est_track = None     # deg, from the cartesian velocities

    def _estimate(self, m, now):
        """GS (kt) and VS (fpm). The ``v_x/v_y/v_z`` datarefs report the
        slot's motion exactly; when they have not arrived yet the
        position samples are differentiated as before."""
        vx, vy, vz = m.get("v_x"), m.get("v_y"), m.get("v_z")
        if vx is not None and vy is not None and vz is not None:
            gs = math.hypot(vx, vz) * _MPS_TO_KT
            vs = vy * _M_TO_FT * 60.0
            if gs > 2.0:
                # +x is east, +z is south: north is -z.
                self._est_track = \
                    math.degrees(math.atan2(vx, -vz)) % 360
            # New samples are halved in, so one jump does not jerk the
            # levers.
            self._est_gs_kt += (gs - self._est_gs_kt) * 0.5
            self._est_vs_fpm += \
                (_clamp(vs, -6000.0, 6000.0) - self._est_vs_fpm) * 0.5
            return self._est_gs_kt, self._est_vs_fpm
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

        # Position recovery: what the tower sequenced is the target
        # point, not the target heading. A plane that fell behind on an
        # acceleration or drifted on a turn steers back toward the point
        # -- gradually blended in over ~2.8 km of separation.
        t_lat = motion.get("lat", target.get("lat"))
        t_lon = motion.get("lon", target.get("lon"))
        if (m.get("lat") is not None and m.get("lon") is not None
                and t_lat is not None and t_lon is not None):
            dlat = math.radians(t_lat - m["lat"])
            # The antimeridian turns a tiny eastward gap into nearly a
            # full circle west -- wrap before measuring.
            dlon = math.radians(
                (t_lon - m["lon"] + 180.0) % 360.0 - 180.0
            ) * math.cos(math.radians(t_lat))
            dnm = math.hypot(dlat, dlon) * 3440.065
            if dnm > 0.02:
                bearing = math.degrees(math.atan2(dlon, dlat)) % 360
                mix = _clamp(dnm / 1.5, 0.0, 0.8)
                t_hdg = (t_hdg + ((bearing - t_hdg + 180.0) % 360 - 180.0)
                         * mix) % 360

        gs, vs = self._estimate(m, now)
        psi = m.get("psi")
        phi = m.get("phi") or 0.0
        pitch_deg = m.get("the")
        ele_ft = (m.get("ele") or 0.0) * _M_TO_FT if m.get("ele") is not None else None
        if ground and ele_ft is not None:
            self._surface_ft = ele_ft
        agl_ft = (ele_ft - self._surface_ft
                  if ele_ft is not None and self._surface_ft is not None
                  else None)
        have_heading = psi is not None
        hdg_err = ((t_hdg - psi) + 180.0) % 360 - 180.0 if have_heading else 0.0
        alt_err = 0.0 if ele_ft is None else t_alt - ele_ft

        if self.helicopter:
            return self._heli_writes(out, slot, ground, parked,
                                     t_hdg, t_gs, t_vs, hdg_err,
                                     alt_err, gs, have_heading,
                                     throttle, yoke_pitch, yoke_roll,
                                     yoke_hdg, pitch_deg, agl_ft, phase)

        if ground:
            # Taxi: heading error to the nosewheel/rudder channel, speed
            # to the throttle. The FO's steer channel already caps and
            # fades; the lever is modest because an AI slot at full taxi
            # thrust is a hazard to itself.
            out[yoke_hdg] = self._steer.update(hdg_err) if have_heading else 0.0
            out[throttle] = _clamp(self._taxi.update(t_gs - gs), 0.0, 0.6)
            out[yoke_pitch] = out[yoke_roll] = 0.0
            braking = 1.0 if t_gs <= self.HOLD_KT + 1.0 and gs > t_gs + 1.0 else 0.0
            out[f"{_CONTROLS}/l_brake_add[{slot}]"] = braking
            out[f"{_CONTROLS}/r_brake_add[{slot}]"] = braking
            return out

        # Airborne: heading error to a bank demand, bank error to the
        # roll lever; height error to a VS demand, VS error to the pitch
        # lever; speed to the throttle. With the velocity datarefs live
        # the bank loop flies the ground track -- the course that leaves
        # no wind-drift error -- rather than the nose's heading.
        steer_ref = psi
        if self._est_track is not None and gs > 20.0:
            steer_ref = self._est_track
        trk_err = ((t_hdg - steer_ref) + 180.0) % 360 - 180.0 \
            if steer_ref is not None else hdg_err
        bank = _clamp(trk_err * 1.5, -25.0, 25.0)
        out[yoke_roll] = self._roll.update(bank - phi) \
            if have_heading else 0.0
        correction = self._vert.update(alt_err) if ele_ft is not None else 0.0
        vs_demand = t_vs + correction
        out[yoke_pitch] = _clamp(
            self._pitch.update(vs_demand - vs)
            - (pitch_deg or 0.0) * self.PITCH_DAMP,
            -1.0, 1.0)
        out[throttle] = self._throttle.update(t_gs - gs)
        return out

    # A degree of unwanted pitch is worth this much lever: the damper
    # that keeps the VS loop from porpoising.
    PITCH_DAMP = 0.01

    def _heli_writes(self, out, slot, ground, parked, t_hdg, t_gs, t_vs,
                     hdg_err, alt_err, gs, have_heading,
                     throttle, yoke_pitch, yoke_roll, yoke_hdg, pitch_deg,
                     agl_ft, phase):
        """Rotorcraft law: altitude to the collective (the throttle
        channel, percent of lever), forward speed to a pitch-attitude
        demand flown by longitudinal cyclic, heading to the pedals with
        a little cyclic roll to help the turn.

        Skid-equipped types hover-taxi -- a few feet over the surface,
        pedals steering -- because they cannot roll. After liftoff the
        forward-speed demand stays capped until the climb clears
        TRANSLATE_FT, so a departure climbs away vertically before it
        translates."""
        hover_taxi = (ground and self._skids
                      and t_gs > self.HOLD_KT and agl_ft is not None)
        if hover_taxi:
            alt_err += self.HOVER_TAXI_FT
        collective = self._collective.update(alt_err) / 100.0
        out[throttle] = _clamp(0.5 + collective, 0.0, 1.0) \
            if not ground else _clamp(collective, 0.0, 0.9 if hover_taxi
                                     else 0.4)
        if not ground and phase == "takeoff" and agl_ft is not None \
                and agl_ft < self.TRANSLATE_FT:
            t_gs = min(t_gs, self.TRANSLATE_KT)
        if pitch_deg is not None:
            # Forward cyclic is nose-down: a speed deficit wants a lower
            # attitude, and the lever chases the attitude error.
            want_pitch = -_clamp((t_gs - gs) * 0.3, -12.0, 12.0)
            out[yoke_pitch] = _clamp((want_pitch - pitch_deg) * 0.05,
                                     -0.4, 0.4)
        else:
            out[yoke_pitch] = -_clamp((t_gs - gs) * 0.008, -0.25, 0.25)
        out[yoke_hdg] = _clamp(self._pedals.update(hdg_err) / 5000.0,
                               -0.5, 0.5) if have_heading else 0.0
        out[yoke_roll] = _clamp(hdg_err * 0.004, -0.15, 0.15) \
            if have_heading else 0.0
        if ground and t_gs <= self.HOLD_KT and not hover_taxi:
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
    # A callsign that drops out of the positions stream keeps its plane
    # this long before the slot is freed: a live feed's quiet spell
    # should not blink a drawn aeroplane out and back in. The slot still
    # answers the roster, so the plane holds its last state in the sim.
    SLOT_KEEP_S = 45.0

    def __init__(self, *args, source=None, **kwargs):
        super().__init__(*args, **kwargs)
        # The XPlaneSource: dref() writes control channels, subscribe()
        # adds each slot's feedback datarefs, value() reads them back.
        self.source = source
        self._slots = {}          # id -> slot index (1..MAX_SLOTS)
        self._pilots = {}         # id -> NativePilot
        self._roster = ()         # the (slot, id, type, pos) tuples last sent
        self._roster_at = 0.0
        self._assigned_at = {}    # id -> when its slot was claimed
        self._departed = {}       # id -> when it stopped being offered
        self._confirmed = set()   # ids the plugin's status reports as planes
        # traffic.xplane_fill != "sim": an unclaimed slot stays empty
        # rather than spawning one of the simulator's own AI planes.
        self.exclusive = False
        self._exclusive_sent = False

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
                    if self.has(i)]) if self.available else 0

    def _refresh_confirmed(self):
        confirmed = self._status.get("native_ac") or []
        self._confirmed = set(confirmed) & self._slots.keys()

    def has(self, callsign):
        # "Drawn" here means the plugin's status carries the callsign in
        # native_ac -- proof the plane exists in the sim, not just that
        # the roster asked for it -- and the settle has passed.
        if not self.available or callsign not in self._wanted:
            return False
        self._refresh_confirmed()
        return (callsign in self._confirmed
                and time.monotonic()
                - self._assigned_at.get(callsign, 0.0) >= self.settle_s)

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
        return (not self.has(callsign)
                and time.monotonic() - asked >= self.ANSWER_WITHIN_S)

    def _subscribe_slot(self, slot):
        if self.source is None:
            return
        for key in _FEEDBACK:
            # X-Plane's elevation dataref is planeN_el, while the rest of
            # the table carries the spelled-out axis names.
            ref = "el" if key == "ele" else key
            self.source.subscribe(
                f"xpn{slot}:{key}",
                f"{_POSITION}/plane{slot}_{ref}")

    def _send_roster(self, roster, now):
        exclusive = bool(self.exclusive)
        if (roster == self._roster and exclusive == self._exclusive_sent
                and now - self._roster_at < self.HELLO_EVERY_S):
            return
        packet = {"v": 1, "t": "native",
                  "ac": [{"id": ident, "slot": slot, "type": tcode,
                          "op": op,
                          "lat": pos[0], "lon": pos[1],
                          "alt": pos[2]}
                         for slot, ident, tcode, op, pos in roster]}
        if exclusive:
            packet["exclusive"] = True
        if self._send(packet):
            self._roster = roster
            self._roster_at = now
            self._exclusive_sent = exclusive

    def sync(self, positions, now=None):
        with self._lock:
            self.on_frame()
            if self._socket is None:
                return
            now = time.monotonic()

            # Keep the slots already flying somebody, then fill the rest
            # in the order the injector offered -- nearest first.
            taken = {slot for slot, _ident, _tc, _op, _pos in self._roster}
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
                self._departed.pop(ident, None)
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
                try:
                    pos = (_number(position["lat"], -90, 90),
                           _number(position["lon"], -180, 180),
                           _number(position["altitude_ft"], -2000, 100000))
                except (ValueError, TypeError, KeyError):
                    pos = None
                if pos is None:
                    continue
                op = _text(position.get("operator") or "", 3)
                roster.append((slot, ident, tcode, op, pos))
                position["_slot"] = slot
            roster.sort()

            # Free the slots nobody claimed this tick -- but not at
            # once: one that merely stopped being offered keeps its
            # plane for SLOT_KEEP_S, so a feed's quiet spell does not
            # blink it out and back in. Past the grace, let the levers
            # go neutral so a still-modelled plane does not fly off on
            # the last demand.
            self._refresh_confirmed()
            previous = {r[1]: r for r in self._roster}
            for ident in [i for i in self._slots
                          if not any(r[1] == i for r in roster)]:
                gone = self._departed.setdefault(ident, now)
                if now - gone < self.SLOT_KEEP_S and ident in previous:
                    # Keep the slot on the roster with its last position;
                    # the plugin only re-seeds a changed assignment, so
                    # the plane simply holds where it was left.
                    keep = previous[ident]
                    roster.append(keep)
                    taken.add(keep[0])
                    continue
                slot = self._slots.pop(ident)
                self._pilots.pop(ident, None)
                self._assigned_at.pop(ident, None)
                self._departed.pop(ident, None)
                if self.source is not None:
                    for chan in ("yoke_pitch_ratio", "yoke_roll_ratio",
                                 "yoke_heading_ratio",
                                 "engine_throttle_request",
                                 "parking_brake"):
                        self.source.dref(f"{_CONTROLS}/{chan}[{slot}]", 0.0)

            self._send_roster(tuple(roster), now)

            # Pilot each assigned slot.
            by_id = {position["callsign"]: position for position in positions}
            for slot, ident, tcode, _op, _pos in roster:
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


class XPHybridTraffic(XPNativeTraffic):
    """Native physics for the planes that can get a slot, CSL drawings
    for the rest.

    The positions arrive nearest-first, so the nineteen slots seat the
    traffic that matters up close -- departures, arrivals, the stand next
    door -- and everything beyond the cap is drawn by XPMP2 instead of
    being dropped. The plugin takes both payloads over the same channel:
    ``native`` rosters are not session packets, so no protocol change is
    needed.
    """

    @property
    def drawing(self):
        # Either fleet standing is enough to draw: native slots when the
        # plugin can drive them, CSL instances when it has models.
        return self.available and (
            bool(self._status.get("native_cap"))
            or self._status.get("csl_models", 0) > 0)

    def _is_native(self, callsign):
        """Whether the callsign rides a native slot rather than a drawing.

        A slot is claimed locally before the plugin can prove the plane
        exists; when it never materializes -- slots held by another
        plugin, an .acf the load refused -- the callsign is a CSL
        drawing's, not a slot's, or it never shows at all.
        """
        if callsign not in self._slots:
            return False
        if callsign in self._confirmed:
            return True
        return (time.monotonic()
                - self._assigned_at.get(callsign, 0.0)
                < self.ANSWER_WITHIN_S)

    def has(self, callsign):
        if self._is_native(callsign):
            return super().has(callsign)
        return (self.available and callsign in self._wanted
                and callsign in self._drawn
                and callsign not in self._unmatched)

    def failed(self, callsign):
        # Over the slot cap is not a failure here -- the callsign falls
        # through to the CSL drawing instead.
        if self._is_native(callsign):
            return super().failed(callsign)
        self.on_frame()
        asked = self._wanted.get(callsign)
        if asked is None:
            return False
        if not self.available:
            return time.monotonic() - asked >= self.ANSWER_WITHIN_S
        if callsign in self._unmatched:
            return True
        return (not self.has(callsign)
                and time.monotonic() - asked >= self.ANSWER_WITHIN_S)

    def _sync_csl(self, positions, now):
        """The XPlaneTraffic drawing path for the unslotted callsigns,
        sharing this object's socket, sequence and status."""
        aircraft = []
        seen = set()
        live_room, room = self.live_limit, self.limit
        for position in positions:
            try:
                packet = aircraft_packet(position)
            except (ValueError, TypeError, KeyError, OverflowError):
                continue
            if packet["id"] in seen:
                continue
            if position.get("source") == "live":
                if not live_room:
                    continue
                live_room -= 1
            elif room:
                room -= 1
            else:
                continue
            aircraft.append(packet)
            seen.add(packet["id"])
        message = {"v": 1, "t": "traffic", "seq": self._sequence,
                   "ac": aircraft}
        while len(json.dumps(message, separators=(",", ":")).encode()) > 65507:
            aircraft.pop()
        if self._send(message):
            self._sequence += 1
            wanted = {a["id"]: self._wanted.get(a["id"], now)
                      for a in aircraft}
            self._drawn.intersection_update(wanted)
            self._unmatched.intersection_update(wanted)
            self._wanted.update(wanted)

    def sync(self, positions, now=None):
        # The native path claims the slots first; whatever it could not
        # seat goes to the drawings.
        super().sync(positions, now)
        with self._lock:
            if self._socket is None:
                return
            # Slots hold a callsign for the window it has to prove it
            # made the plane; past it, the drawing path gets it instead
            # so a blocked slot does not turn the aeroplane invisible.
            native_ids = {c for c in self._slots if self._is_native(c)}
            csl = [p for p in positions
                   if p.get("callsign") not in native_ids]
            # Native AI ownership and CSL rendering coexist: ownership is
            # exclusive only for the multiplayer/TCAS datarefs, not for
            # XPMP2's visible instances. Overflow traffic is still drawn,
            # albeit without TCAS/dataref visibility while native owns the
            # slots.
            self._sync_csl(csl, time.monotonic())
