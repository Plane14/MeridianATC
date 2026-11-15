"""X-Plane control translations: AP modes, switches, systems and raw axes.

Paths are from the DataRefs.txt and Commands.txt mirrors linked in xplane.py.
Mode commands are guarded by RREF status: AP buttons can toggle, rather than
behave like SimConnect's explicit ON/OFF events. See also Laminar's
developer.x-plane.com/article/accessing-the-x-plane-autopilot-from-datarefs/.

The hand-flown phases (taxi, rotation, flare, go-around, rollout) run on
the same joystick datarefs a plugin would write: each axis is pushed
through ``sim/joystick/yoke_*_ratio`` behind its matching
``sim/operation/override/*`` flag, which is how X-Plane says "the pilot's
stick is not on this axis any more". The overrides are released by
CENTER_AILER_RUDDER -- the handoff's last write -- so a manual handback
leaves a working yoke, not a dead one.
"""

from __future__ import annotations

import math
import time

AP = "sim/cockpit2/autopilot/"
HSI = "sim/cockpit2/radios/actuators/HSI_source_select_pilot"
FLAPS = "sim/cockpit2/controls/flap_handle_request_ratio"
THROTTLE = "sim/cockpit2/engine/actuators/throttle_ratio"
ACT = "sim/cockpit2/engine/actuators/"
ICE = "sim/cockpit2/ice/"
JSTICK = "sim/joystick/"
OVR = "sim/operation/override/"

# key: (status dataref suffix, engage command, disengage command, axis, threshold)
MODES = {
    "AP_HDG_HOLD": ("heading_status", "heading", "wing_leveler", "lateral", 1),
    "AP_NAV1_HOLD": ("nav_status", "NAV", "NAV", "lateral", 1),
    "AP_LOC_HOLD": ("nav_status", "NAV", "NAV", "lateral", 1),
    "AP_APR_HOLD": ("approach_status", "approach", "approach", "approach", 1),
    "AP_ALT_HOLD": ("altitude_hold_status", "altitude_hold", "pitch_sync", "vertical", 2),
    "AP_VS": ("vvi_status", "vertical_speed_pre_sel", "pitch_sync", "vertical", 1),
    "FLIGHT_LEVEL_CHANGE": ("speed_status", "level_change", "pitch_sync", "vertical", 1),
    "AP_BANK_HOLD": ("roll_status", "wing_leveler", "heading", "lateral", 1),
    "AP_WING_LEVELER": ("roll_status", "wing_leveler", "heading", "lateral", 1),
}
READS = {"control:" + mode[0]: AP + mode[0] for mode in MODES.values()}
READS["control:autothrottle"] = AP + "autothrottle_enabled"
READS["control:ap_on"] = AP + "autopilot_on"

# name: (path, minimum, maximum, scale)
VARIABLES = {
    "AUTOPILOT HEADING LOCK DIR": (AP + "heading_dial_deg_mag_pilot", 0, 360, 1),
    "AUTOPILOT ALTITUDE LOCK VAR": (AP + "altitude_dial_ft", -2000, 100000, 1),
    "AUTOPILOT VERTICAL HOLD VAR": (AP + "vvi_dial_fpm", -20000, 20000, 1),
    "AUTOPILOT AIRSPEED HOLD VAR": (AP + "airspeed_dial_kts", 0, 2000, 1),
    "AUTOPILOT MACH HOLD VAR": (AP + "airspeed_dial_kts_mach", 0, 5, 1),
    "AUTOPILOT FLIGHT DIRECTOR ACTIVE": (AP + "flight_director_command_bars_pilot", 0, 1, 1),
}
BUGS = {
    "AP_HEADING_BUG_SET_EX1": ("AUTOPILOT HEADING LOCK DIR", 360 / 16384),
    "AP_HDG_VAR_SET": ("AUTOPILOT HEADING LOCK DIR", 1),
    "HEADING_BUG_SET": ("AUTOPILOT HEADING LOCK DIR", 1),
    "AP_ALT_VAR_SET_ENGLISH": ("AUTOPILOT ALTITUDE LOCK VAR", 1),
    "AP_VS_VAR_SET_ENGLISH": ("AUTOPILOT VERTICAL HOLD VAR", 1),
    "AP_SPD_VAR_SET": ("AUTOPILOT AIRSPEED HOLD VAR", 1),
    "AP_SPD_VAR_SET_EX1": ("AUTOPILOT AIRSPEED HOLD VAR", .01),
}
COMMANDS = {
    "AUTOPILOT_ON": "sim/autopilot/servos_on",
    "AUTOPILOT_OFF": "sim/autopilot/servos_off_any",
    "AP_MASTER": "sim/autopilot/servos_toggle",
    "TOGGLE_FLIGHT_DIRECTOR": "sim/autopilot/fdir_toggle",
    "ENGINE_AUTO_START": "sim/operation/auto_start",
    "ENGINE_AUTO_SHUTDOWN": "sim/starters/shut_down",
    "GEAR_DOWN": "sim/flight_controls/landing_gear_down",
    "GEAR_UP": "sim/flight_controls/landing_gear_up",
    "YAW_DAMPER_ON": "sim/systems/yaw_damper_on",
    "YAW_DAMPER_OFF": "sim/systems/yaw_damper_off",
    "PITOT_HEAT_ON": "sim/ice/pitot_heat0_on",
    "PITOT_HEAT_OFF": "sim/ice/pitot_heat0_off",
    "AUTO_THROTTLE_TO_GA": "sim/autopilot/take_off_go_around",
    "THROTTLE_REVERSE_THRUST_TOGGLE": "sim/engines/thrust_reverse_toggle",
    "TOGGLE_PUSHBACK": "sim/ground_ops/pushback_straight",
    # X-Plane's tug pushes until it is told to let go -- this is the
    # "let go". SimConnect's own push completes itself, so a source that
    # does not know the event answers False and nothing happens.
    "PUSHBACK_STOP": "sim/ground_ops/pushback_stop",
    "CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE": "sim/systems/seatbelt_sign_toggle",
    "AP_ATT_HOLD_ON": "sim/autopilot/pitch_sync",
    "AP_PITCH_LEVELER_ON": "sim/autopilot/pitch_sync",
}
# The joystick-axis events (-16383..16384) onto X-Plane's -1..1 yoke
# ratios. The rotorcraft controller's cyclics and pedals are the same
# three axes on a helicopter, so its event names land here too.
AXES = {
    "AILERON_SET": ("yoke_roll_ratio", "override_joystick_roll"),
    "ELEVATOR_SET": ("yoke_pitch_ratio", "override_joystick_pitch"),
    "AXIS_RUDDER_SET": ("yoke_heading_ratio", "override_joystick_heading"),
    "RUDDER_SET": ("yoke_heading_ratio", "override_joystick_heading"),
    "AXIS_CYCLIC_LATERAL_SET": ("yoke_roll_ratio", "override_joystick_roll"),
    "AXIS_CYCLIC_LONGITUDINAL_SET": ("yoke_pitch_ratio", "override_joystick_pitch"),
    "ROTOR_AXIS_TAIL_ROTOR_SET": ("yoke_heading_ratio", "override_joystick_heading"),
    "AXIS_TAIL_ROTOR_SET": ("yoke_heading_ratio", "override_joystick_heading"),
}
# Every override flag an axis or ground write can latch: released
# together on close, so a dead stick is never what the handoff leaves
# behind. The engine levers latch theirs for the same reason the stick
# does: without it X-Plane re-reads the pilot's throttle quadrant into
# the actuator the frame a lever write stops -- the takeoff's "power to
# idle at liftoff" was exactly that. They are two different releases,
# though: CENTER_AILER_RUDDER hands back only the stick -- a stop or a
# hold is still the FO's flight, and releasing the quadrant every time
# the yoke centres is how the parked aeroplane's throttle lever ended up
# back on the pilot's idle hardware between taxi probes. The levers
# leave only on the handoff itself.
STICK_OVERRIDES = ("override_joystick_pitch", "override_joystick_roll",
                   "override_joystick_heading", "override_wheel_steer",
                   "override_toe_brakes")
LEVER_OVERRIDES = ("override_throttles", "override_prop_pitch",
                   "override_mixture", "override_flap_handle",
                   "override_speedbrakes", "override_cowl_flaps")
OVERRIDES = STICK_OVERRIDES + LEVER_OVERRIDES
# What each lever override feeds the engine through, per DataRefs.txt:
# override_throttles -> "use ENGN_thro_use to control them",
# override_prop_pitch -> "use POINT_pitch_deg_use to edit",
# override_mixture -> "use ENGN_mixt to edit". The lever ratios are the
# cockpit's handle; with the flag latched these are what the engines
# actually run against -- the parked Baron had levers drawn at 66%
# and engines idling on a use value nobody had written.
THRO_USE = "sim/flightmodel/engine/ENGN_thro_use"
MIX_USE = "sim/flightmodel/engine/ENGN_mixt"
PITCH_USE = "sim/flightmodel/engine/POINT_pitch_deg_use"
BRAKES = {
    "AXIS_LEFT_BRAKE_SET": "sim/cockpit2/controls/left_brake_ratio",
    "AXIS_RIGHT_BRAKE_SET": "sim/cockpit2/controls/right_brake_ratio",
}
# The nosewheel's steer command is degrees, both namespaces written so a
# build on the older API still turns: the gear's own authority (read as
# "steer_deg", acf_nw_steerdeg1) scales the -1..1 the caller means.
STEER_DREFS = ("sim/flightmodel2/gear/tire_steer_command_deg[0]",
               "sim/flightmodel/parts/tire_steer_cmd[0]")
SWITCHES = {
    "PARKING_BRAKE_SET": "BRAKE PARKING POSITION",
    "GEAR_SET": "GEAR HANDLE POSITION",
    "AVIONICS_MASTER_SET": "AVIONICS MASTER SWITCH",
}
LIGHTS = {"BEACON_LIGHTS": "LIGHT BEACON", "NAV_LIGHTS": "LIGHT NAV",
          "STROBES": "LIGHT STROBE", "TAXI_LIGHTS": "LIGHT TAXI",
          "LANDING_LIGHTS": "LIGHT LANDING"}


class XPlaneControls:
    def __init__(self, source):
        self.source = source
        self.pending = {}
        self.autothrottle = None
        # The prop lever's commanded ratio and the blade angle the
        # software governor is currently holding per engine -- the
        # loop in ``govern_props`` owns them while the prop override
        # is latched.
        self._prop_lever = {}
        self._prop_deg = {}

    def clear(self):
        self.pending.clear()
        self.autothrottle = None
        self._release_overrides()

    def _release_overrides(self):
        """Hand every overridden channel back to the pilot's hardware."""
        for flag in OVERRIDES:
            self.source._dref(OVR + flag, 0)
        self._prop_lever.clear()
        self._prop_deg.clear()

    def _release_stick_overrides(self):
        """Hand back the yoke, pedals, tiller and toe brakes only.

        The engine levers are not the stick's business: a stop centres
        the yoke without the quadrant leaving the FO's hands, so the
        levers' latches stay down until ``_release_overrides`` at the
        handoff.
        """
        for flag in STICK_OVERRIDES:
            self.source._dref(OVR + flag, 0)

    def _throttle_use(self, ratio, index=None):
        """The engine's throttle under its override: every engine where
        no index is named -- the _all lever writes' match."""
        if index is None:
            return all(self.source._dref(f"{THRO_USE}[{i}]", ratio)
                       for i in range(16))
        return self.source._dref(f"{THRO_USE}[{index}]", ratio)

    def _pitch_range(self, index):
        """The governor's pitch stops for a prop, as the .acf publishes."""
        with self.source._lock:
            lo = self.source._values.get(f"acf_min_pitch{index}")
            hi = self.source._values.get(f"acf_max_pitch{index}")
        if lo is None or hi is None or not lo < hi:
            # A constant-speed prop's usual governed range.
            return 8.0, 45.0
        return float(lo), float(hi)

    def _pitch_seed(self, index, ratio):
        """The blade angle the governor starts a lever's demand from:
        coarse side of fine, so a demand the loop has not yet wound
        under-revs rather than over-revs."""
        lo, hi = self._pitch_range(index)
        return lo + (hi - lo) * (0.15 + 0.55 * max(0.0, 1.0 - ratio))

    def _pitch_use(self, index, ratio, lever=True):
        """The blade angle the engine reads under the prop override.

        A collective is a blade-angle lever -- up asks more pitch, and
        the write lands raw. A prop lever is an RPM lever, and the
        override carries none of the governor that normally answers
        it: ``govern_props`` walks this angle toward the lever's
        commanded RPM every read, so what is written here is only the
        seed the loop takes over from -- writing the governor's
        minimum stop directly was the parked Baron's 3000-plus RPM.
        """
        if not lever:
            lo, hi = self._pitch_range(index)
            deg = lo + ratio * (hi - lo)
            return self.source._dref(f"{PITCH_USE}[{index}]", deg)
        self._prop_lever[index] = ratio
        if index not in self._prop_deg:
            self._prop_deg[index] = self._pitch_seed(index, ratio)
        return self.source._dref(f"{PITCH_USE}[{index}]",
                                 self._prop_deg[index])

    # The governor's slew limits: fast enough to take a lever from
    # flight-idle to takeoff inside a couple of seconds, slow enough
    # that one tick's RPM readback does not ring through the next.
    _GOV_GAIN = 0.004          # degrees of pitch per RPM of error per step
    _GOV_STEP = 0.6            # degrees per step at most
    _GOV_DEADBAND = 25.0       # RPM close enough to commanded
    _GOV_MIN_RPM = 200.0       # below this the engine is not turning yet
    _GOV_RATED_RPM = 2700.0    # prop design point when acf does not publish one

    def govern_props(self):
        """One step of the prop governor the override replaced.

        ``override_prop_pitch`` hands the engines a raw blade angle
        with nothing behind it: a fixed angle overspeeds at power and
        lugs in the descent. The lever asks an RPM -- full lever is
        the prop's design point -- and this walks the angle the way
        the real governor does until the prop is there. It runs from
        the source's ``read``, which is the flight model's own tick,
        and only on engines a prop lever has ever been written for:
        a collective's blade-angle writes are already direct control
        and keep them.
        """
        if not self._prop_lever:
            return
        with self.source._lock:
            values = dict(self.source._values)
        for index, ratio in list(self._prop_lever.items()):
            deg = self._prop_deg.get(index)
            if deg is None:
                deg = self._prop_deg[index] = self._pitch_seed(index, ratio)
            rpm = values.get(f"prop_rpm{index}")
            if rpm is None or rpm < self._GOV_MIN_RPM:
                # Not turning: a stopped or starting engine wants the
                # seed's fine pitch, not a governor winding to the stop.
                deg = self._prop_deg[index] = self._pitch_seed(index, ratio)
                self.source._dref(f"{PITCH_USE}[{index}]", deg)
                continue
            design = values.get(f"acf_des_rpm{index}") \
                or self._GOV_RATED_RPM
            err = ratio * design - rpm
            if abs(err) <= self._GOV_DEADBAND:
                continue
            lo, hi = self._pitch_range(index)
            step = max(-self._GOV_STEP, min(self._GOV_STEP,
                                          err * self._GOV_GAIN))
            # Under speed the angle fines, over speed it coarsens.
            deg = max(lo, min(hi, deg - step))
            if deg != self._prop_deg[index]:
                self._prop_deg[index] = deg
                self.source._dref(f"{PITCH_USE}[{index}]", deg)

    def _pitch_use_all(self, ratio, lever=False):
        return all(self._pitch_use(i, ratio, lever) for i in range(16))

    def _clear_axis_pending(self, axis, keep=None):
        for other, _engage, _disengage, other_axis, _threshold in MODES.values():
            if axis == other_axis and other != keep:
                self.pending.pop(other, None)

    def _mode(self, mode, on):
        status, engage, disengage, axis, threshold = mode
        source = self.source
        with source._lock:
            observed = source._values.get("control:" + status)
            if observed is None:
                return False
            current = observed >= threshold
            now = time.monotonic()
            pending = self.pending.get(status)
            if pending is not None:
                wanted, deadline = pending
                if current == wanted or now >= deadline:
                    self.pending.pop(status, None)
                else:
                    current = wanted
            if current == on:
                if on:
                    self._clear_axis_pending(axis, keep=status)
                return True
            command = engage if on else disengage
            if not source._command("sim/autopilot/" + command):
                return False
            self._clear_axis_pending(axis)
            self.pending[status] = on, now + 1.0
            return True

    def _autothrottle(self, mode):
        if self.source._dref(AP + "autothrottle_enabled", mode):
            self.autothrottle = mode, time.monotonic() + 1.0
            return True
        return False

    def _arm_autothrottle(self):
        with self.source._lock:
            mode = self.source._values.get("control:autothrottle")
        if self.autothrottle is not None and time.monotonic() < self.autothrottle[1]:
            mode = self.autothrottle[0]
        if mode is None:
            return False
        return True if mode >= 0 else self._autothrottle(0)

    def set_variable(self, name, value, unit=None):
        if unit is not None:
            return False
        if name == "GPS DRIVES NAV1":
            return value in (0, 1) and self.source._dref(HSI, 2 if value else 0)
        if name == "AUTOPILOT MASTER":
            return value in (0, 1) and self.send("AUTOPILOT_ON" if value else "AUTOPILOT_OFF", 0)
        if name == "AUTOPILOT THROTTLE ARM":
            return value in (0, 1) and (self._arm_autothrottle() if value else self._autothrottle(-1))
        if name.startswith("GENERAL ENG THROTTLE LEVER POSITION:"):
            try:
                index = int(name.rsplit(":", 1)[1]) - 1
            except ValueError:
                return False
            if not (0 <= index < 16 and 0 <= value <= 100):
                return False
            self.source._dref(OVR + "override_throttles", 1)
            ok = self.source._dref(f"{THROTTLE}[{index}]", value / 100)
            # The override moves the engine's input from the lever to
            # ENGN_thro_use: write both or the handle moves and the
            # engine never hears it.
            return self._throttle_use(value / 100, index) and ok
        # The levers that are not the throttle, same 0..100 caller scale.
        # Each latches its own joystick override the way the axes do: a
        # quadrant lever left partway would otherwise re-take the
        # actuator the moment a write is not repeated every frame.
        lever = {"GENERAL ENG MIXTURE LEVER POSITION:":
                 (ACT + "mixture_ratio", "override_mixture"),
                 "GENERAL ENG PROPELLER LEVER POSITION:":
                 (ACT + "prop_ratio", "override_prop_pitch"),
                 "RECIP ENG COWL FLAP POSITION:":
                 (ACT + "cowl_flap_ratio", "override_cowl_flaps")}
        for prefix, (path, flag) in lever.items():
            if name.startswith(prefix):
                try:
                    index = int(name.rsplit(":", 1)[1]) - 1
                except ValueError:
                    return False
                if not (0 <= index < 16 and 0 <= value <= 100):
                    return False
                if flag:
                    self.source._dref(OVR + flag, 1)
                ok = self.source._dref(f"{path}[{index}]", value / 100)
                # Under each flag the engine's ear is the family's
                # "use" dataref, the way ENGN_thro_use is the
                # throttle's: the lever is what the cockpit draws.
                if path == ACT + "mixture_ratio":
                    ok = self.source._dref(
                        f"{MIX_USE}[{index}]", value / 100) and ok
                elif path == ACT + "prop_ratio":
                    ok = self._pitch_use(index, value / 100) and ok
                return ok
        if name.startswith("GENERAL ENG ANTI ICE POSITION:"):
            try:
                index = int(name.rsplit(":", 1)[1]) - 1
            except ValueError:
                return False
            if not (0 <= index < 16 and 0 <= value <= 1):
                return False
            # The SimConnect variable serves carburettor heat on a piston
            # and inlet/cowl anti-ice on a turbine; the writes a model
            # does not carry are ignored, so all three go out and the
            # one that is wired answers.
            on = int(bool(value))
            return all([
                self.source._dref(f"{ACT}carb_heat_ratio[{index}]", value),
                self.source._dref(f"{ICE}ice_inlet_heat_on_per_engine[{index}]", on),
                self.source._dref(f"{ICE}cowling_thermal_anti_ice_per_engine[{index}]", on)])
        if name.startswith("PROP DEICE SWITCH:"):
            try:
                index = int(name.rsplit(":", 1)[1]) - 1
            except ValueError:
                return False
            if not (0 <= index < 16 and 0 <= value <= 1):
                return False
            on = int(bool(value))
            ok = self.source._dref(f"{ICE}ice_prop_heat_on_per_engine[{index}]", on)
            return self.source._dref(ICE + "ice_prop_heat_on", on) and ok \
                if index == 0 else ok
        if name == "STRUCTURAL DEICE SWITCH":
            on = int(bool(value))
            # Boots, bleed and TKS answer the same SimConnect switch; a
            # model carries at most one of them.
            return all([
                self.source._dref(ICE + "ice_surface_boot_on", on),
                self.source._dref(ICE + "ice_surface_hot_bleed_air_on", on),
                self.source._dref(ICE + "ice_surface_tks_on", on)])
        if name.startswith("WINDSHIELD DEICE SWITCH:"):
            try:
                index = int(name.rsplit(":", 1)[1]) - 1
            except ValueError:
                return False
            if not 0 <= index < 4:
                return False
            on = int(bool(value))
            ok = self.source._dref(f"{ICE}ice_window_heat_on_window[{index}]", on)
            return self.source._dref(ICE + "ice_window_heat_on", on) and ok \
                if index == 0 else ok
        if name == "TRANSPONDER STATE:1":
            # The SimConnect enum (0 off, 1 standby, 2 test, 3 on, 4 alt,
            # 5 ground) onto X-Plane's transponder_mode knob, whose order
            # differs (0 off, 1 stby, 2 on, 3 alt, 4 test, 5 gnd).
            mode = {0: 0, 1: 1, 2: 4, 3: 2, 4: 3, 5: 5}.get(int(value), 7)
            return self.source._dref(
                "sim/cockpit2/radios/actuators/transponder_mode", mode)
        if name == "FLAPS HANDLE INDEX":
            with self.source._lock:
                detents = self.source._values.get("flap_detents", 0)
            return (detents > 1 and 0 <= value < detents and value == int(value)
                    and self.source._dref(OVR + "override_flap_handle", 1)
                    and self.source._dref(FLAPS, value / (detents - 1)))
        mapping = VARIABLES.get(name)
        if mapping is None:
            return False
        path, minimum, maximum, factor = mapping
        if not minimum <= value <= maximum:
            return False
        if name == "AUTOPILOT MACH HOLD VAR" and not self.source._dref(AP + "airspeed_is_mach", 1):
            return False
        if name == "AUTOPILOT AIRSPEED HOLD VAR" and not self.source._dref(AP + "airspeed_is_mach", 0):
            return False
        return self.source._dref(path, value * factor)

    def send(self, name, value):
        source = self.source
        if not source.connected:
            return False
        try:
            name = name.decode("ascii") if isinstance(name, bytes) else str(name)
            value = float(value)
        except (UnicodeError, TypeError, ValueError, OverflowError):
            return False
        if not math.isfinite(value):
            return False
        if name in BUGS:
            variable, scale = BUGS[name]
            value *= scale
            if "HEADING" in variable:
                value %= 360
            return self.set_variable(variable, value)
        if name in COMMANDS:
            return source._command(COMMANDS[name])
        # -- the hand-flown channels ----------------------------------------
        if name in AXES:
            axis, flag = AXES[name]
            ratio = max(-1.0, min(1.0, value / 16383.0))
            # Latch the override first: a packet that writes the ratio
            # without it loses to the pilot's hardware the same frame.
            source._dref(OVR + flag, 1)
            return source._dref(JSTICK + axis, ratio)
        if name in BRAKES:
            if not -16384 <= value <= 16384:
                return False
            source._dref(OVR + "override_toe_brakes", 1)
            return source._dref(BRAKES[name],
                                min(1.0, (value + 16383.0) / 32766.0))
        if name == "BRAKES":
            # The plain event presses both pedals together.
            source._dref(OVR + "override_toe_brakes", 1)
            ok = source._dref(BRAKES["AXIS_LEFT_BRAKE_SET"], 1.0)
            return source._dref(BRAKES["AXIS_RIGHT_BRAKE_SET"], 1.0) and ok
        if name in ("STEERING_SET", "AXIS_STEERING_SET"):
            if not -16384 <= value <= 16384:
                return False
            with source._lock:
                limit = source._values.get("steer_deg") or 60.0
            degrees = value / 16383.0 * max(0.0, min(75.0, limit))
            source._dref(OVR + "override_wheel_steer", 1)
            source._dref("sim/cockpit2/controls/nosewheel_steer_on", 1)
            ok = False
            for path in STEER_DREFS:
                ok = source._dref(path, degrees) or ok
            return ok
        if name == "CENTER_AILER_RUDDER":
            # Centre the stick, then hand just those axes back: the
            # engine levers stay the FO's until the handoff itself, so
            # a stop or a hold does not return the quadrant to wherever
            # the pilot's hardware happens to be sitting.
            for axis in {axis for axis, _flag in AXES.values()}:
                source._dref(JSTICK + axis, 0.0)
            for path in STEER_DREFS:
                source._dref(path, 0.0)
            for path in BRAKES.values():
                source._dref(path, 0.0)
            self._release_stick_overrides()
            return True
        if name == "RELEASE_OVERRIDES":
            # The handoff's release: every latched channel back to the
            # pilot's hardware, engine levers included -- a quadrant the
            # sim keeps ignoring after the FO lets go is the worst
            # souvenir there is.
            self._release_overrides()
            return True
        # -- the rotorcraft's extra channels ---------------------------------
        if name == "AXIS_COLLECTIVE_SET":
            if not -16384 <= value <= 16384:
                return False
            # X-Plane carries the collective on the prop lever. The axis
            # is signed like a joystick's; -16383 is the floor.
            source._dref(OVR + "override_prop_pitch", 1)
            ratio = min(1.0, (value + 16383.0) / 32766.0)
            ok = source._dref(ACT + "prop_ratio_all", ratio)
            # Up on a collective is more blade angle, the prop lever's
            # other way round -- lo..hi of the rotor's own range.
            return self._pitch_use_all(ratio, lever=False) and ok
        if name in ("HELICOPTER_COLLECTIVE_SET", "COLLECTIVE_SET"):
            # The named forms are unipolar, 0..16384.
            if not 0 <= value <= 16384:
                return False
            source._dref(OVR + "override_prop_pitch", 1)
            ratio = value / 16383.0
            ok = source._dref(ACT + "prop_ratio_all", ratio)
            return self._pitch_use_all(ratio, lever=False) and ok
        if name == "AXIS_HELICOPTER_THROTTLE_SET":
            if not -16384 <= value <= 16384:
                return False
            source._dref(OVR + "override_throttles", 1)
            ratio = min(1.0, (value + 16383.0) / 32766.0)
            ok = source._dref(THROTTLE + "_all", ratio)
            return self._throttle_use(ratio) and ok
        if name == "HELICOPTER_THROTTLE_SET":
            if not 0 <= value <= 16384:
                return False
            source._dref(OVR + "override_throttles", 1)
            ratio = value / 16383.0
            ok = source._dref(THROTTLE + "_all", ratio)
            return self._throttle_use(ratio) and ok
        if name in ("HELICOPTER_THROTTLE_FULL", "HELICOPTER_THROTTLE_CUT"):
            source._dref(OVR + "override_throttles", 1)
            ratio = float(name.endswith("FULL"))
            ok = source._dref(THROTTLE + "_all", ratio)
            return self._throttle_use(ratio) and ok
        if name == "ROTOR_GOV_SWITCH_SET":
            return value in (0, 1) and all(
                source._dref(f"{ACT}governor_on[{i}]", value)
                for i in range(8))
        if name in ("ROTOR_BRAKE", "ROTOR_BRAKE_ON", "ROTOR_BRAKE_OFF"):
            on = value if name == "ROTOR_BRAKE" else int(name.endswith("_ON"))
            return source._dref("sim/cockpit2/switches/rotor_brake",
                                1 if on else 0)
        # -- engine start -----------------------------------------------------
        if name in ("STARTER_SET", "TOGGLE_ALL_STARTERS"):
            if name == "TOGGLE_ALL_STARTERS" or value:
                # ignition_key sits spring-loaded at 4=starting: the write
                # buys a tick of crank, the command covers the rest.
                ok = source._command("sim/engines/engage_starters")
                return all(source._dref(f"{ACT}ignition_key[{i}]", 4)
                           for i in range(8)) and ok
            return all(source._dref(f"{ACT}ignition_key[{i}]", 3)
                       for i in range(8))
        if name.startswith("STARTER") and name.endswith("_SET"):
            try:
                index = int(name[7:-4]) - 1
            except ValueError:
                return False
            if not 0 <= index < 8:
                return False
            if value:
                ok = source._command(f"sim/starters/engage_starter_{index + 1}")
                return source._dref(f"{ACT}ignition_key[{index}]", 4) and ok
            return source._dref(f"{ACT}ignition_key[{index}]", 3)
        # -- the rest ----------------------------------------------------------
        if name == "AP_MAX_BANK_ANGLE_SET":
            # X-Plane's bank limit is a 1..6 mode index over the model's
            # own degree steps, not an arbitrary angle: the nearest step
            # to what was asked for is what it gets.
            with source._lock:
                angles = [source._values.get(f"bank_angle{i}", 0)
                          for i in range(6)]
            angles = [a for a in angles if a > 0] or [5, 10, 15, 20, 25, 30]
            mode = min(range(len(angles)),
                       key=lambda i: abs(angles[i] - value)) + 1
            return source._dref(AP + "bank_angle_mode", mode)
        if name in SWITCHES:
            return value in (0, 1) and source.set_variable(SWITCHES[name], value)
        for root, mode in MODES.items():
            if name in (root + "_ON", root + "_OFF"):
                return self._mode(mode, name.endswith("_ON"))
        for root, variable in LIGHTS.items():
            if name in (root + "_ON", root + "_OFF"):
                return source.set_variable(variable, int(name.endswith("_ON")))
        if name in ("MASTER_BATTERY_SET", "BATTERY1_SET"):
            if value not in (0, 1):
                return False
            count = 8 if name == "MASTER_BATTERY_SET" else 1
            return all([source._dref(f"sim/cockpit2/electrical/battery_on[{i}]", value)
                        for i in range(count)])
        if name == "AP_NAV_SELECT_SET":
            return value in (1, 2) and source._dref(HSI, value - 1)
        if name == "AUTO_THROTTLE_ARM":
            return self._arm_autothrottle()
        if name in ("AP_AIRSPEED_ON", "AUTO_THROTTLE_DISCONNECT", "AP_AIRSPEED_OFF"):
            return self._autothrottle(1 if name == "AP_AIRSPEED_ON" else -1)
        if name in ("AP_MACH_ON", "AP_MACH_OFF"):
            on = name.endswith("_ON")
            return source._dref(AP + "airspeed_is_mach", int(on)) and (not on or self._autothrottle(1))
        if name in ("FLAPS_SET", "AXIS_FLAPS_SET", "SPOILERS_SET"):
            minimum = -16383 if name == "AXIS_FLAPS_SET" else 0
            if not minimum <= value <= 16383:
                return False
            ratio = (value - minimum) / (16383 - minimum)
            # A quadrant flap or spoiler axis left at a different detent
            # fights the write the same way a throttle quadrant does.
            source._dref(OVR + ("override_flap_handle" if "FLAPS" in name
                                else "override_speedbrakes"), 1)
            return source._dref(FLAPS if "FLAPS" in name else "sim/cockpit2/controls/speedbrake_ratio", ratio)
        if name in ("SPOILERS_ARM_ON", "SPOILERS_ARM_OFF"):
            source._dref(OVR + "override_speedbrakes", 1)
            return source._dref("sim/cockpit2/controls/speedbrake_ratio", -.5 if name.endswith("_ON") else 0)
        if name == "KOHLSMAN_SET":
            return source.set_variable("KOHLSMAN SETTING MB", value / 16)
        if name in ("THROTTLE_FULL", "THROTTLE_CUT", "THROTTLE_IDLE"):
            source._dref(OVR + "override_throttles", 1)
            ratio = float(name == "THROTTLE_FULL")
            ok = source._dref(THROTTLE + "_all", ratio)
            return self._throttle_use(ratio) and ok
        if name == "THROTTLE_SET":
            if not 0 <= value <= 16383:
                return False
            source._dref(OVR + "override_throttles", 1)
            ratio = value / 16383.0
            ok = source._dref(THROTTLE + "_all", ratio)
            return self._throttle_use(ratio) and ok
        if name.startswith("AXIS_THROTTLE") and name.endswith("_SET"):
            try:
                index = int(name[13:-4]) - 1
            except ValueError:
                return False
            if not (0 <= index < 16 and -16384 <= value <= 16384):
                return False
            source._dref(OVR + "override_throttles", 1)
            ratio = max(0.0, min(1.0, (value + 16383.0) / 32766.0))
            ok = source._dref(f"{THROTTLE}[{index}]", ratio)
            return self._throttle_use(ratio, index) and ok
        return False
