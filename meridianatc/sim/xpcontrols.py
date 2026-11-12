"""Standard X-Plane AP/switch translations; no flight-model overrides or axes.

Paths are from the DataRefs.txt and Commands.txt mirrors linked in xplane.py.
Mode commands are guarded by RREF status: AP buttons can toggle, rather than
behave like SimConnect's explicit ON/OFF events. See also Laminar's
developer.x-plane.com/article/accessing-the-x-plane-autopilot-from-datarefs/.
"""

from __future__ import annotations

import math
import time

AP = "sim/cockpit2/autopilot/"
HSI = "sim/cockpit2/radios/actuators/HSI_source_select_pilot"
FLAPS = "sim/cockpit2/controls/flap_handle_request_ratio"
THROTTLE = "sim/cockpit2/engine/actuators/throttle_ratio"

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
    "ENGINE_AUTO_START": "sim/operation/auto_start",
    "ENGINE_AUTO_SHUTDOWN": "sim/starters/shut_down",
    "GEAR_DOWN": "sim/flight_controls/landing_gear_down",
    "GEAR_UP": "sim/flight_controls/landing_gear_up",
    "YAW_DAMPER_ON": "sim/systems/yaw_damper_on",
    "YAW_DAMPER_OFF": "sim/systems/yaw_damper_off",
    "PITOT_HEAT_ON": "sim/ice/pitot_heat0_on",
    "PITOT_HEAT_OFF": "sim/ice/pitot_heat0_off",
    "AUTO_THROTTLE_TO_GA": "sim/autopilot/take_off_go_around",
}
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

    def clear(self):
        self.pending.clear()
        self.autothrottle = None

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
            return self.source._dref(f"{THROTTLE}[{index}]", value / 100)
        if name == "FLAPS HANDLE INDEX":
            with self.source._lock:
                detents = self.source._values.get("flap_detents", 0)
            return (detents > 1 and 0 <= value < detents and value == int(value)
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
            return source._dref(FLAPS if "FLAPS" in name else "sim/cockpit2/controls/speedbrake_ratio", ratio)
        if name in ("SPOILERS_ARM_ON", "SPOILERS_ARM_OFF"):
            return source._dref("sim/cockpit2/controls/speedbrake_ratio", -.5 if name.endswith("_ON") else 0)
        if name == "KOHLSMAN_SET":
            return source.set_variable("KOHLSMAN SETTING MB", value / 16)
        if name in ("THROTTLE_FULL", "THROTTLE_CUT", "THROTTLE_IDLE"):
            return source._dref(THROTTLE + "_all", int(name == "THROTTLE_FULL"))
        if name == "THROTTLE_SET":
            return 0 <= value <= 16383 and source._dref(THROTTLE + "_all", value / 16383)
        return False
