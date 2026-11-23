"""X-Plane 12's built-in UDP aircraft link (no plugin required).

Names and units checked against X-Plane's 1208 DataRefs.txt, mirrored at
https://github.com/X-Plane/XPlane2Blender/blob/master/io_xplane2blender/resources/DataRefs.txt
Commands checked against Resources/plugins/Commands.txt mirrored at
https://github.com/Rororo098/Xplane-Dataref-Bridge/blob/main/Commands.txt
The 12.1.1+ Web API (8086) is deliberately not needed by this link.
"""

from __future__ import annotations

import contextlib
import logging
import math
import socket
import struct
import threading
import time
from dataclasses import fields

from .source import SimConnectSource
from .state import AircraftState, floats_said
from .xpcontrols import OVERRIDES, OVR, READS, XPlaneControls

FT = 3.280839895
KT = 1.943844492
LB = 2.204622622
RADIOS = "sim/cockpit2/radios/actuators/"

log = logging.getLogger(__name__)

# attribute: (dataref, multiplier); booleans retain their dataclass type.
DATAREFS = {
    "latitude": ("sim/flightmodel/position/latitude", 1),
    "longitude": ("sim/flightmodel/position/longitude", 1),
    "altitude_ft": ("sim/flightmodel/position/elevation", FT),
    "altitude_agl_ft": ("sim/flightmodel/position/y_agl", FT),
    "indicated_altitude_ft": ("sim/cockpit2/gauges/indicators/altitude_ft_pilot", 1),
    "heading_true": ("sim/flightmodel/position/true_psi", 1),
    "heading_magnetic": ("sim/flightmodel/position/mag_psi", 1),
    "track_true": ("sim/cockpit2/gauges/indicators/ground_track_true_pilot", 1),
    "ground_speed_kt": ("sim/flightmodel/position/groundspeed", KT),
    "indicated_airspeed_kt": ("sim/flightmodel/position/indicated_airspeed", 1),
    "true_airspeed_kt": ("sim/flightmodel/position/true_airspeed", KT),
    "vertical_speed_fpm": ("sim/flightmodel/position/vh_ind_fpm", 1),
    "on_ground": ("sim/flightmodel/failures/onground_any", 1),
    "bank_deg": ("sim/flightmodel/position/phi", 1),
    # The envelope protection's reads (atc/autonomous/x_the_load): the
    # normal load factor and the pitch attitude. flightmodel2/misc is
    # the XP11+ namespace, fine on 12; a build without it lists the
    # field in state.unavailable and the protections quietly stand down.
    "g_force": ("sim/flightmodel2/misc/gforce_normal", 1),
    "pitch_deg": ("sim/flightmodel/position/theta", 1),
    # The stall watch's (atc/autonomous/x_the_stall): the horn itself,
    # the angle of attack, and the model's own warning angle -- a model
    # that does not wire them lands in state.unavailable, which the
    # watch reads as unpublished rather than never stalling.
    "stall_warning": ("sim/flightmodel/failures/stallwarning", 1),
    "aoa_deg": ("sim/flightmodel/position/alpha", 1),
    "stall_aoa_deg": ("sim/aircraft/overflow/acf_stall_warn_alpha", 1),
    "parking_brake": ("sim/cockpit2/controls/parking_brake_ratio", 1),
    "gear_down": ("sim/cockpit2/controls/gear_handle_down", 1),
    "seatbelt_sign": ("sim/cockpit2/switches/fasten_seat_belts", 1),
    "beacon_light": ("sim/cockpit2/switches/beacon_on", 1),
    "landing_light": ("sim/cockpit2/switches/landing_lights_on", 1),
    "taxi_light": ("sim/cockpit2/switches/taxi_light_on", 1),
    "nav_light": ("sim/cockpit2/switches/navigation_lights_on", 1),
    "strobe_light": ("sim/cockpit2/switches/strobe_lights_on", 1),
    "engine_count": ("sim/aircraft/engine/acf_num_engines", 1),
    "kohlsman_hpa": ("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot", 33.86389),
    "mixture_pct": ("sim/cockpit2/engine/actuators/mixture_ratio[0]", 100),
    "pitot_heat": ("sim/cockpit2/ice/ice_pitot_heat_on_pilot", 1),
    "avionics_master": ("sim/cockpit2/switches/avionics_power_on", 1),
    "battery_master": ("sim/cockpit2/electrical/battery_on[0]", 1),
    "main_bus_volts": ("sim/cockpit2/electrical/bus_volts[0]", 1),
    "nav1_freq": (RADIOS + "nav1_frequency_hz", .01),
    "ap_airspeed_kt": ("sim/cockpit2/autopilot/airspeed_dial_kts", 1),
    "ap_altitude_ft": ("sim/cockpit2/autopilot/altitude_dial_ft", 1),
    "ap_heading_deg": ("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot", 1),
    "throttle_pct": ("sim/cockpit2/engine/actuators/throttle_ratio[0]", 100),
    # The trim actuators, ratio of their travel to per cent -- the
    # figures a trim write reads back as (xpcontrols.TRIMS writes the
    # same datarefs).
    "elevator_trim_pct": ("sim/cockpit2/controls/elevator_trim", 100),
    "aileron_trim_pct": ("sim/cockpit2/controls/aileron_trim", 100),
    "rudder_trim_pct": ("sim/cockpit2/controls/rudder_trim", 100),
    "airspeed_mach": ("sim/flightmodel/misc/machno", 1),
    "total_weight_lb": ("sim/flightmodel/weight/m_total", LB),
    "max_gross_weight_lb": ("sim/aircraft/weight/acf_m_max", LB),
    "fuel_total_lb": ("sim/flightmodel/weight/m_fuel_total", LB),
    "paused": ("sim/time/paused", 1),
    "ambient_wind_direction": ("sim/weather/aircraft/wind_now_direction_degt", 1),
    "ambient_wind_velocity": ("sim/weather/aircraft/wind_now_speed_msc", KT),
    "ambient_temperature_c": ("sim/weather/aircraft/temperature_ambient_deg_c", 1),
    "total_air_temperature_c": ("sim/weather/aircraft/temperature_leadingedge_deg_c", 1),
    "ambient_visibility_m": ("sim/weather/aircraft/visibility_reported_sm", 1609.344),
    "ambient_humidity_pct": ("sim/weather/aircraft/relative_humidity_sealevel_percent", 1),
    "sea_level_pressure_hpa": ("sim/weather/aircraft/qnh_pas", .01),
    "com1_receive": (RADIOS + "audio_selection_com1", 1),
    "com2_receive": (RADIOS + "audio_selection_com2", 1),
    "structural_ice_pct": ("sim/flightmodel/failures/frm_ice", 100),
    "pressure_altitude_ft": ("sim/flightmodel2/position/pressure_altitude", 1),
    "autothrottle": ("sim/cockpit2/autopilot/autothrottle_on", 1),
    "design_vs0_kt": ("sim/aircraft/view/acf_Vso", 1),
    "design_vs1_kt": ("sim/aircraft/view/acf_Vs", 1),
    "design_vc_kt": ("sim/aircraft/view/acf_Vno", 1),
    "barber_pole_kt": ("sim/aircraft/view/acf_Vne", 1),
    "mach_max_operate": ("sim/aircraft/view/acf_Mmo", 1),
    "nav1_cdi": ("sim/cockpit2/radios/indicators/nav1_hdef_dots_pilot", 127 / 2.5),
    "nav1_gsi": ("sim/cockpit2/radios/indicators/nav1_vdef_dots_pilot", 119 / 2.5),
    "nav1_has_glideslope": ("sim/cockpit2/radios/indicators/nav1_display_vertical", 1),
    "rotor_brake_active": ("sim/cockpit2/switches/rotor_brake", 1),
    "rotor_clutch_active": ("sim/cockpit2/switches/clutch_engage", 1),
    # The tug's own readback: on the nosewheel while the push runs, off
    # when it has let go -- what an open-ended pushback command knows it
    # is finished by. XP's ground services publish nothing for "a tug is
    # on its way", so pushback_wait is never set here.
    "pushback_attached": ("sim/aircraft/overflow/pushback_attached", 1),
    "collective_position": ("sim/cockpit2/engine/actuators/prop_ratio[0]", 100),
    "fuel_selector": ("sim/cockpit2/fuel/fuel_tank_selector", 1),
    # The engine watch's instruments (atc.powerplant): gas-producer/fan
    # and power-turbine per cent, the turbine temperature (X-Plane's
    # EGT is what its turbine models carry), the piston crankshaft RPM.
    # A dataref a model does not publish lands in state.unavailable and
    # the protection stands down, which is the wanted answer.
    "eng_n1_pct": ("sim/flightmodel/engine/ENGN_N1_[0]", 1),
    "eng_n2_pct": ("sim/flightmodel/engine/ENGN_N2_[0]", 1),
    "eng_itt_c": ("sim/flightmodel/engine/ENGN_EGT_c[0]", 1),
    "eng_rpm": ("sim/cockpit2/engine/indicators/engine_speed_rpm[0]", 1),
    # The rest of the cockpit the checklists name (the same fields the
    # SimConnect source fills): starter, gauges, brakes, de-ice, the
    # APU, packs, hydraulics, the cabin and the minimums bug. A dataref
    # a model does not publish leaves its field None and the check
    # reads it as agreement, never an objection.
    "starter_active": ("sim/flightmodel2/engines/starter_is_running[0]", 1),
    "oil_pressure_psi": ("sim/cockpit2/engine/indicators/oil_pressure_psi[0]", 1),
    "eng2_oil_pressure_psi": ("sim/cockpit2/engine/indicators/oil_pressure_psi[1]", 1),
    "suction_inhg": ("sim/cockpit/misc/vacuum", 1),
    "battery_amps": ("sim/cockpit2/electrical/battery_amps[0]", 1),
    "brake_left_pct": ("sim/cockpit2/controls/left_brake_ratio", 100),
    "brake_right_pct": ("sim/cockpit2/controls/right_brake_ratio", 100),
    "carb_heat_pct": ("sim/cockpit2/engine/actuators/carb_heat_ratio[0]", 100),
    "yaw_damper_on": ("sim/cockpit2/switches/yaw_damper_on", 1),
    "autobrake_pos": ("sim/cockpit2/switches/auto_brake_level", 1),
    "apu_on": ("sim/cockpit/engine/APU_switch", 1),
    "apu_pct": ("sim/cockpit2/electrical/APU_N1_percent", 1),
    "apu_gen_on": ("sim/cockpit2/electrical/APU_generator_on", 1),
    "apu_bleed_on": ("sim/cockpit2/bleedair/actuators/apu_bleed", 1),
    "pack_left_on": ("sim/cockpit2/bleedair/actuators/pack_left", 1),
    "pack_right_on": ("sim/cockpit2/bleedair/actuators/pack_right", 1),
    "engine_anti_ice_on": ("sim/cockpit2/ice/ice_inlet_heat_on", 1),
    "window_heat_on": ("sim/cockpit2/ice/ice_window_heat_on", 1),
    "prop_deice_on": ("sim/cockpit2/ice/ice_prop_heat_on", 1),
    "aoa_heat_on": ("sim/cockpit2/ice/ice_AOA_heat_on", 1),
    "cabin_altitude_ft": ("sim/cockpit2/pressurization/indicators/cabin_altitude_ft", 1),
    "cabin_altitude_goal_ft": ("sim/cockpit2/pressurization/actuators/cabin_altitude_ft", 1),
    "pressurization_diff_psi": ("sim/cockpit2/pressurization/indicators/pressure_diffential_psi", 1),
    "minimums_ft": ("sim/cockpit2/gauges/actuators/radio_altimeter_bug_ft_pilot", 1),
    "no_smoking_sign": ("sim/cockpit2/switches/no_smoking", 1),
    "wiper_pos": ("sim/cockpit2/switches/wiper_speed", 1),
    "oxygen_on": ("sim/cockpit2/oxygen/actuators/o2_valve_on", 1),
    "hydraulic_pressure": ("sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1", 1),
    "hydraulic_pump_on": ("sim/cockpit2/switches/electric_hydraulic_pump_on", 1),
    "rotor_brake_handle_pos": ("sim/cockpit2/switches/rotor_brake_ratio", 100),
    "rotor_lateral_trim_pct": ("sim/cockpit2/controls/rotor_trim", 100),
}
for _radio in (1, 2):
    for _standby in (False, True):
        _suffix = "standby" if _standby else "active"
        _path = f"com{_radio}_" + ("standby_" if _standby else "") + "frequency_hz_833"
        # Despite the historic 'hz' spelling, 833 channel numbers are kHz.
        DATAREFS[f"com{_radio}_{_suffix}"] = (RADIOS + _path, .001)

EXTRA = {
    "xp_engine_type": "sim/aircraft/prop/acf_en_type[0]",
    "helicopter": "sim/aircraft2/metadata/is_helicopter",
    # Conventional gear, for the taxi law: a taildragger taxis and rolls
    # out differently (meridianatc.atc.tailwheel).
    "tail_dragger": "sim/aircraft/gear/acf_is_taildragger",
    "xp_transponder": RADIOS + "transponder_mode",
    "ident": "sim/cockpit2/radios/indicators/transponder_id",
    "squawk": RADIOS + "transponder_code",
    "flaps": "sim/cockpit2/controls/flap_ratio",
    "flap_detents": "sim/aircraft/controls/acf_flap_detents",
    "speedbrake": "sim/cockpit2/controls/speedbrake_ratio",
    "transmit": RADIOS + "audio_com_selection",
    "autopilot_mode": "sim/cockpit2/autopilot/servos_on",
    # DataRefs.txt says 'appears to be in lbs'; in practice this is kg,
    # matching m_fuel_total and the per-tank quantities.
    "fuel_capacity_kg": "sim/aircraft/weight/acf_m_fuel_tot",
    "hsi_source": RADIOS + "HSI_source_select_pilot",
    "nav_type": "sim/cockpit2/radios/indicators/nav1_type",
    "rain": "sim/weather/aircraft/precipitation_on_aircraft_ratio",
    # Snow's share of what is falling (XP12.1+ region weather); absent on
    # older builds, which simply keep reporting rain-or-nothing.
    "snow": "sim/weather/region/snow_percent",
    # The nosewheel's own authority in degrees -- what a -1..1 steering
    # write scales by; 0 or absent leaves the controls' default.
    "steer_deg": "sim/aircraft/gear/acf_nw_steerdeg1",
    # The AP bank selector's six steps in degrees, for
    # AP_MAX_BANK_ANGLE_SET picking the nearest one.
    "cht_is_c": "sim/aircraft/engine/acf_CHT_is_C",
    "oil_is_c": "sim/aircraft/engine/acf_oilT_is_C",
    # The model's own CHT red line where it publishes one -- the ceiling
    # the cooling watch works against before the class figure is used.
    "cht_red_hi": "sim/aircraft/limits/red_hi_CHT",
    # The same story for the torque the rotorcraft collective limit
    # watches: shaft torque in N-m against the book's red line in ft-lbs.
    "trq_nm": "sim/flightmodel/engine/ENGN_TRQ[0]",
    "trq_red_hi_ftpds": "sim/aircraft/limits/red_hi_TRQ",
    "trq_grn_hi_ftpds": "sim/aircraft/limits/green_hi_TRQ",
    "acf_max_trq_nm": "sim/aircraft/engine/acf_max_TRQ",
    "itt_red_hi_c": "sim/aircraft/limits/red_hi_ITT",
    "itt_grn_hi_c": "sim/aircraft/limits/green_hi_ITT",
    # COM1's power switch, the honest half of the COM STATUS enum; and
    # the two tank gauges, kilograms like X-Plane's other fuel figures.
    "com1_powered": "sim/cockpit2/radios/actuators/com1_power",
    "fuel_left_kg": "sim/cockpit2/fuel/fuel_level_indicated_left",
    "fuel_right_kg": "sim/cockpit2/fuel/fuel_level_indicated_right",
    # The wind vector at the aeroplane, metres a second in the world
    # axes -- read() turns it into the body-axes knots the weather watch
    # reads.
    "wind_x_ms": "sim/weather/aircraft/wind_now_x_msc",
    "wind_y_ms": "sim/weather/aircraft/wind_now_y_msc",
    "wind_z_ms": "sim/weather/aircraft/wind_now_z_msc",
}
for _layer in range(3):
    # The cloud layers the aeroplane's own weather knows: base and tops
    # in metres MSL and the coverage, so read() can say "in cloud" the
    # way SimConnect's AMBIENT IN CLOUD does.
    EXTRA[f"cloud_base_m{_layer}"] = (
        f"sim/weather/aircraft/cloud_base_msl_m[{_layer}]")
    EXTRA[f"cloud_top_m{_layer}"] = (
        f"sim/weather/aircraft/cloud_tops_msl_m[{_layer}]")
    EXTRA[f"cloud_cov{_layer}"] = (
        f"sim/weather/aircraft/cloud_coverage_percent[{_layer}]")
for _engine in range(8):
    EXTRA[f"cht{_engine}"] = f"sim/flightmodel/engine/ENGN_CHT_c[{_engine}]"
    EXTRA[f"oil{_engine}"] = f"sim/flightmodel/engine/ENGN_oil_temp_c[{_engine}]"
    EXTRA[f"cowl{_engine}"] = f"sim/cockpit2/engine/actuators/cowl_flap_ratio[{_engine}]"
for _step in range(6):
    EXTRA[f"bank_angle{_step}"] = f"sim/cockpit2/autopilot/bank_angles_values[{_step}]"
# ...and which of those steps the selector currently stands on.
EXTRA["bank_mode"] = "sim/cockpit2/autopilot/bank_angle_mode"
for _engine in range(16):
    EXTRA[f"running{_engine}"] = f"sim/flightmodel/engine/ENGN_running[{_engine}]"
    EXTRA[f"flow{_engine}"] = f"sim/flightmodel/engine/ENGN_FF_[{_engine}]"
    # The governor's pitch stops per prop: the scale the override's
    # POINT_pitch_deg_use writes are mapped through (props forward is
    # fine pitch -- the min -- on an RPM lever, more blade on a
    # collective).
    EXTRA[f"acf_min_pitch{_engine}"] = \
        f"sim/aircraft/prop/acf_min_pitch[{_engine}]"
    EXTRA[f"acf_max_pitch{_engine}"] = \
        f"sim/aircraft/prop/acf_max_pitch[{_engine}]"
    # What the software governor in xpcontrols runs against while the
    # prop override is latched: the prop's own speed and the design
    # point a full lever asks for.
    EXTRA[f"prop_rpm{_engine}"] = \
        f"sim/cockpit2/engine/indicators/prop_speed_rpm[{_engine}]"
    EXTRA[f"acf_des_rpm{_engine}"] = \
        f"sim/aircraft/prop/acf_des_rpm_prp[{_engine}]"
for _engine in range(1, 4):
    # Engines two through four's own gauges and levers: engine one's
    # figures keep the scalar names in DATAREFS, these land under the
    # state's eng2_/eng3_/eng4_ fields. acf_num_engines is the roster
    # read() believes -- the arrays answer zeros past it, and a zero
    # is not an engine.
    EXTRA[f"trq_nm{_engine}"] = \
        f"sim/flightmodel/engine/ENGN_TRQ[{_engine}]"
    EXTRA[f"itt{_engine}"] = \
        f"sim/flightmodel/engine/ENGN_EGT_c[{_engine}]"
    EXTRA[f"n1_{_engine}"] = \
        f"sim/flightmodel/engine/ENGN_N1_[{_engine}]"
    EXTRA[f"n2_{_engine}"] = \
        f"sim/flightmodel/engine/ENGN_N2_[{_engine}]"
    EXTRA[f"rpm{_engine}"] = \
        f"sim/cockpit2/engine/indicators/engine_speed_rpm[{_engine}]"
    EXTRA[f"thr{_engine}"] = \
        f"sim/cockpit2/engine/actuators/throttle_ratio[{_engine}]"
    EXTRA[f"mix{_engine}"] = \
        f"sim/cockpit2/engine/actuators/mixture_ratio[{_engine}]"
    EXTRA[f"prop{_engine}"] = \
        f"sim/cockpit2/engine/actuators/prop_ratio[{_engine}]"
for _tank in range(9):
    EXTRA[f"tank{_tank}"] = f"sim/flightmodel/weight/m_fuel[{_tank}]"
    EXTRA[f"tank_ratio{_tank}"] = f"sim/aircraft/overflow/acf_tank_rat[{_tank}]"
# The checklist systems' per-engine switches: fuel pumps, generators,
# magneto keys, turbine ignition and bleed valves.
for _engine in range(4):
    EXTRA[f"fuelpump{_engine}"] = \
        f"sim/cockpit2/engine/actuators/fuel_pump_on[{_engine}]"
    EXTRA[f"gen{_engine}"] = \
        f"sim/cockpit2/electrical/generator_on[{_engine}]"
    EXTRA[f"bleed{_engine}"] = \
        f"sim/cockpit2/bleedair/actuators/engine_bleed_sov[{_engine}]"
for _engine in range(2):
    EXTRA[f"ign_key{_engine}"] = \
        f"sim/cockpit2/engine/actuators/ignition_key[{_engine}]"
    EXTRA[f"ignition_on{_engine}"] = \
        f"sim/cockpit2/engine/actuators/ignition_on[{_engine}]"
# The doors' open ratios, the rotor's free-turbine percent and
# governor switch (a helicopter's Nr and its governor's presence),
# the yaw pedals (its anti-torque channel), and the wing de-ice's
# two spellings (boots and the hot surface -- XP's "surfce" typo).
for _door in range(10):
    EXTRA[f"door{_door}"] = f"sim/flightmodel2/misc/door_open_ratio[{_door}]"
EXTRA["n2_0"] = "sim/cockpit2/engine/indicators/N2_percent[0]"
EXTRA["gov_0"] = "sim/cockpit2/engine/actuators/governor_on[0]"
EXTRA["pedal_yaw"] = "sim/cockpit2/controls/yoke_heading_ratio"
EXTRA["wing_boot"] = "sim/cockpit2/ice/ice_surface_boot_on"
EXTRA["wing_heat"] = "sim/cockpit2/ice/ice_surfce_heat_on"

# The gear table's ten slots: Plane Maker's types are 0 none, 1 wheel,
# 2 skid, 3 float -- a float anywhere is a float-fitted airframe, which
# a landplane's own code never says (a C172 on floats is "C172").
for _gear in range(10):
    EXTRA[f"gear_type{_gear}"] = f"sim/aircraft/parts/acf_gear_type[{_gear}]"

EXTRA.update(READS)

STRINGS = {
    "type_code": ("sim/aircraft/view/acf_ICAO", 40),
    "tail_number": ("sim/aircraft/view/acf_tailnum", 40),
    "title": ("sim/aircraft/view/acf_ui_name", 250),
}

DERIVED = {
    "flaps_index": ("flaps", "flap_detents"),
    "transponder_code": ("squawk",),
    "transponder_state": ("xp_transponder",),
    "transponder_mode": ("xp_transponder", "ident"),
    "autopilot": ("autopilot_mode",),
    "spoilers_armed": ("speedbrake",),
    "engines_running": tuple(f"running{i}" for i in range(16)),
    "speedbrake_pct": ("speedbrake",),
    "fuel_pump_on": ("fuelpump0",),
    "eng2_fuel_pump_on": ("fuelpump1",),
    "eng3_fuel_pump_on": ("fuelpump2",),
    "eng4_fuel_pump_on": ("fuelpump3",),
    "generator_on": ("gen0",),
    "eng2_generator_on": ("gen1",),
    "eng3_generator_on": ("gen2",),
    "eng4_generator_on": ("gen3",),
    "alternator_on": ("gen0",),
    "eng2_alternator_on": ("gen1",),
    "bleed_air_on": ("bleed0",),
    "eng2_bleed_air_on": ("bleed1",),
    "eng3_bleed_air_on": ("bleed2",),
    "eng4_bleed_air_on": ("bleed3",),
    "magneto_pos": ("ign_key0",),
    "eng2_magneto_pos": ("ign_key1",),
    "magneto1_left": ("ign_key0",),
    "magneto1_right": ("ign_key0",),
    "magneto2_left": ("ign_key1",),
    "magneto2_right": ("ign_key1",),
    "ignition_on": ("ignition_on0",),
    "eng2_ignition_on": ("ignition_on1",),
    "exits_open_pct": ("door0",),
    "exit1_open_pct": ("door0",),
    "exit2_open_pct": ("door1",),
    "exit3_open_pct": ("door2",),
    "exit4_open_pct": ("door3",),
    "eng_fuel_flow_pph": ("flow0",),
}

WRITABLE = {
    "BRAKE PARKING POSITION": ("parking_brake", 1),
    "GEAR HANDLE POSITION": ("gear_down", 1),
    "ELECTRICAL MASTER BATTERY": ("battery_master", 1),
    "AVIONICS MASTER SWITCH": ("avionics_master", 1),
    "LIGHT BEACON": ("beacon_light", 1),
    "LIGHT LANDING": ("landing_light", 1),
    "LIGHT TAXI": ("taxi_light", 1),
    "LIGHT NAV": ("nav_light", 1),
    "LIGHT STROBE": ("strobe_light", 1),
    "KOHLSMAN SETTING MB": ("kohlsman_hpa", 1 / 33.86389),
    # The checklist systems' switches, under the SimConnect names the
    # FO's switch table asks for on both sims. The first four slots
    # write a percent over the dataref's ratio; the rest are booleans
    # and enumerations at their own scale.
    "GENERAL ENG ANTI ICE POSITION:1": ("carb_heat_pct", 0.01),
    "BRAKE LEFT POSITION": ("brake_left_pct", 0.01),
    "BRAKE RIGHT POSITION": ("brake_right_pct", 0.01),
    "SPOILERS HANDLE POSITION": ("speedbrake", 0.01),
    "EXIT OPEN:1": ("door0", 0.01),
    "AUTOPILOT YAW DAMPER": ("yaw_damper_on", 1),
    "AUTO BRAKE SWITCH CB": ("autobrake_pos", 1),
    "APU SWITCH": ("apu_on", 1),
    "APU GENERATOR SWITCH": ("apu_gen_on", 1),
    "BLEED AIR APU": ("apu_bleed_on", 1),
    "APU PACKS": ("pack_left_on", 1),
    "ENG ANTI ICE:1": ("engine_anti_ice_on", 1),
    "STRUCTURAL DEICE SWITCH": ("wing_heat", 1),
    "WINDSHIELD DEICE SWITCH": ("window_heat_on", 1),
    "PROP DEICE SWITCH:1": ("prop_deice_on", 1),
    "PANEL ANTI ICE SWITCH": ("aoa_heat_on", 1),
    "HYDRAULIC SWITCH:1": ("hydraulic_pump_on", 1),
    "PRESSURIZATION CABIN ALTITUDE GOAL": ("cabin_altitude_goal_ft", 1),
    "DECISION HEIGHT": ("minimums_ft", 1),
    "CABIN NO SMOKING ALERT SWITCH": ("no_smoking_sign", 1),
    "OXYGEN SWITCH": ("oxygen_on", 1),
    "WIPER SWITCH": ("wiper_pos", 1),
    "IGNITION_KEY:1": ("ign_key0", 1),
    "IGNITION_KEY:2": ("ign_key1", 1),
    "TURB ENG IGNITION SWITCH:1": ("ignition_on0", 1),
    "TURB ENG IGNITION SWITCH:2": ("ignition_on1", 1),
}
for _slot in range(4):
    WRITABLE[f"GENERAL ENG FUEL PUMP SWITCH:{_slot + 1}"] = (
        f"fuelpump{_slot}", 1)
    WRITABLE[f"GENERAL ENG GENERATOR SWITCH:{_slot + 1}"] = (
        f"gen{_slot}", 1)
    WRITABLE[f"BLEED AIR ENGINE:{_slot + 1}"] = (f"bleed{_slot}", 1)
for _slot in range(2):
    WRITABLE[f"GENERAL ENG MASTER ALTERNATOR:{_slot + 1}"] = (
        f"gen{_slot}", 1)


class XPlaneSource:
    # The simulator's level-change is pitch-for-speed: the throttle
    # manager chases the same airspeed, and two controllers on one
    # variable is a phugoid -- the descent it owned porpoised the
    # aeroplane into terrain. VS owns the axis on it outright, so it
    # is also the flag the FLC probe reads to leave the mode alone;
    # and its autopilot mode-status reads run armed at one -- only
    # two means engaged. MSFS needs neither: its readback is a
    # boolean where one already means engaged, and its FLC manages
    # the levers itself.
    flc_pitch_only = True
    ap_status_engaged = 2.0
    # The sim's own NAV cannot be trusted to steer the route on it: a
    # plan the FMS cannot load is dropped silently, and one it can is
    # followed on an active leg the box picks itself and will not
    # always sequence -- watched following a course back at the
    # departure end, circling the fix it would not pass. The FO steers
    # enroute by its own bearing to the active fix on the heading bug
    # instead, the way the procedure legs are already flown. MSFS's
    # GPS drives NAV1 properly, so it keeps the flag.
    box_nav = False

    def __init__(self, host="127.0.0.1", port=49000, poll_interval=.25,
                 timeout_s=1.0, disconnect_s=5.0, gone_s=30.0,
                 discovery_s=1.1):
        self.host, self.port = host, int(port)
        self.version = None
        self.timeout_s, self.disconnect_s = timeout_s, disconnect_s
        self.gone_s = max(gone_s, disconnect_s)
        self.discovery_s = discovery_s
        self.frequency = max(1, round(1 / max(.05, poll_interval)))
        self._socket = None
        self._peer = None
        self._thread = None
        self._stop = threading.Event()
        self._answered = threading.Event()
        self._lock = threading.Lock()
        self._values = {}
        # The endpoint that answers the subscriptions: the first packet
        # pins it, and only it is heard afterwards. The simulator's own
        # send port is not the one the requests went to, so it is learned
        # rather than assumed.
        self._sender = None
        # Set by the engine once the traffic plugin is up: a callable
        # returning the FMS entries the plugin reads, for the flight
        # plan the simulator itself is flying.
        self.fms_reader = None
        # Teleport/slew detection: the position a read ago and until when
        # a jump it could not have flown marks the state relocated.
        self._last_geo = None
        self._relocated_until = 0.0
        self._controls = XPlaneControls(self)
        self._last_packet = 0.0
        self._subscriptions = [(key, path) for key, (path, _) in DATAREFS.items()]
        self._subscriptions += list(EXTRA.items())
        self._subscriptions += [(f"{key}:{i}", f"{path}[{i}]")
                                for key, (path, size) in STRINGS.items()
                                for i in range(size)]

    @staticmethod
    def parse_beacon(packet, sender):
        if len(packet) < 21 or packet[:5] != b"BECN\0":
            return None
        major, _minor, app, version, role, port = struct.unpack_from("<BBiiIH", packet, 5)
        if major != 1 or app != 1 or role != 1 or version < 120000 or not port:
            return None
        return sender[0], port, version

    def _discover(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as beacon:
            beacon.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            beacon.bind(("", 49707))
            beacon.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                              socket.inet_aton("239.255.1.1") + socket.inet_aton("0.0.0.0"))
            deadline = time.monotonic() + self.discovery_s
            while time.monotonic() < deadline:
                beacon.settimeout(max(.001, deadline - time.monotonic()))
                packet, sender = beacon.recvfrom(2048)
                found = self.parse_beacon(packet, sender)
                if found:
                    return found
        return None

    @property
    def connected(self):
        return (self._socket is not None and self._answered.is_set()
                and time.monotonic() - self._last_packet < self.disconnect_s)

    @property
    def alive(self):
        # Whether X-Plane is plausibly still there: answered once, and not
        # silent past ``gone_s``. A long frame or a scenery load stops the
        # RREF stream exactly like a crash does, so silence alone is not
        # yet gone -- ``connected`` asks "fresh", this asks "still here".
        return (self._socket is not None and self._answered.is_set()
                and time.monotonic() - self._last_packet < self.gone_s)

    def connect(self):
        if self.connected:
            return True
        self.close()
        self._values.clear()
        self._answered.clear()
        self._stop.clear()
        found = None
        with contextlib.suppress(OSError):
            found = self._discover()
        candidates = [(self.host, self.port)]
        if found:
            self.version = found[2]
            candidates.insert(0, found[:2])
        for endpoint in dict.fromkeys(candidates):
            try:
                with self._lock:
                    self._values.clear()
                    self._last_packet = 0.0
                    self._sender = None
                    self._last_geo = None
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._socket.bind(("", 0))
                # The answers leave from the simulator's send port (49001 by
                # default), not the port the requests went to: a connected
                # socket would silently drop every one of them.
                self._peer = (socket.gethostbyname(endpoint[0]), endpoint[1])
                self._socket.settimeout(.1)
                self._stop.clear()
                self._answered.clear()
                self._thread = threading.Thread(target=self._receive,
                                                name="XPlane-RREF", daemon=True)
                self._thread.start()
                for index, (_key, path) in enumerate(self._subscriptions):
                    self._subscribe(index, path, self.frequency)
                if self._answered.wait(self.timeout_s) and self.connected:
                    return True
            except OSError:
                pass
            self.close()
        return False

    def _packet(self, payload):
        if self._peer is not None:
            self._socket.sendto(payload, self._peer)

    def _subscribe(self, index, path, frequency):
        self._packet(b"RREF\0" + struct.pack("<ii400s", frequency, index, path.encode("ascii")))

    def _receive(self):
        while not self._stop.is_set():
            try:
                packet, sender = self._socket.recvfrom(65535)
            except TimeoutError:
                continue
            except OSError:
                return
            if self._peer is None or sender[0] != self._peer[0]:
                continue
            if len(packet) < 13 or packet[:4] != b"RREF" or (len(packet) - 5) % 8:
                continue
            valid = False
            with self._lock:
                if self._sender is not None and sender != self._sender:
                    continue
                for index, value in struct.iter_unpack("<if", packet[5:]):
                    if 0 <= index < len(self._subscriptions) and math.isfinite(value):
                        self._values[self._subscriptions[index][0]] = value
                        valid = True
                if valid:
                    self._sender = self._sender or sender
                    self._last_packet = time.monotonic()
                    self._answered.set()

    def read(self):
        with self._lock:
            values = self._values.copy()
        state = AircraftState(source="xplane", connected=self.connected)
        state.stalled = self.alive and not self.connected
        for key, (_path, factor) in DATAREFS.items():
            if key in values:
                raw = values[key] * factor
                default = getattr(state, key)
                setattr(state, key, bool(raw) if isinstance(default, bool)
                        else int(raw) if isinstance(default, int) else raw)
        for key, (_path, size) in STRINGS.items():
            raw = bytes(max(0, min(255, int(values.get(f"{key}:{i}", 0)))) for i in range(size))
            setattr(state, key, raw.split(b"\0", 1)[0].decode("utf-8", errors="replace").strip())
        state.callsign = state.tail_number
        state.engines_running = any(values.get(f"running{i}", 0) for i in range(16))
        for i in range(1, 4):
            if f"running{i}" in values and state.engine_count > i:
                setattr(state, f"eng{i + 1}_combustion", bool(values[f"running{i}"]))
        state.engine_type = {0: 0, 1: 0, 2: 5, 3: 4, 4: 1, 5: 1, 6: 4,
                             7: 1, 8: 5, 9: 5, 10: 5}.get(
            int(values.get("xp_engine_type", -1)), -1)
        state.tail_dragger = (bool(values["tail_dragger"])
                              if "tail_dragger" in values else None)
        # acf_nw_steerdeg1 is the deflection the gear steers by; a model
        # that reports ~0 has a free-castoring wheel the brakes steer.
        state.castoring_wheel = (float(values["steer_deg"]) < 0.5
                                 if "steer_deg" in values else None)
        if any(f"gear_type{i}" in values for i in range(10)):
            state.on_floats = any(
                int(values.get(f"gear_type{i}", 0)) == 3 for i in range(10))
        else:
            # The table did not answer; the aircraft's own name is the
            # remaining evidence ("Cessna Skyhawk Floats", and the like).
            state.on_floats = floats_said(state.title)
        state.category = "Helicopter" if values.get("helicopter") else "Airplane"
        if state.category == "Helicopter":
            state.engine_type = 3 if state.engine_type == 5 else state.engine_type
        # X-Plane has no condition-lever dataref: on a turboprop the
        # model wires that lever to the mixture channel outright -- a
        # parked C90 reads props forward, mixture zero, which is the
        # condition levers at cut-off. The single-lever scheme the 12.1
        # joystick curves added ("Prop Lever with Feather and Shutoff")
        # is Plane-Maker opt-in and would instead read prop at zero here.
        if state.engine_type == 5 and "mixture_pct" in values:
            state.condition_pct = state.mixture_pct
        # The blue lever reads the same actuator the collective does on
        # a helicopter: on a fixed-wing piston or turboprop prop_ratio
        # is the prop control; jets have neither lever and a helicopter
        # keeps the collective's name for it.
        if state.engine_type in (1, 5) and "collective_position" in values:
            state.prop_pct = state.collective_position
        # The checklists' remaining cockpit, per engine first: fuel
        # pumps, generators (the one electrical switch answers the
        # piston alternator's name too), bleeds, the magneto keys'
        # five stops, and the turbine ignition switches.
        for i, attr in enumerate(("fuel_pump_on", "eng2_fuel_pump_on",
                                  "eng3_fuel_pump_on", "eng4_fuel_pump_on")):
            if f"fuelpump{i}" in values:
                setattr(state, attr, bool(values[f"fuelpump{i}"]))
        for i, attr in enumerate(("generator_on", "eng2_generator_on",
                                  "eng3_generator_on", "eng4_generator_on")):
            if f"gen{i}" in values:
                on = bool(values[f"gen{i}"])
                setattr(state, attr, on)
                if i == 0:
                    state.alternator_on = on
                elif i == 1:
                    state.eng2_alternator_on = on
        for i, attr in enumerate(("bleed_air_on", "eng2_bleed_air_on",
                                  "eng3_bleed_air_on", "eng4_bleed_air_on")):
            if f"bleed{i}" in values:
                setattr(state, attr, bool(values[f"bleed{i}"]))
        for i, attr in enumerate(("magneto_pos", "eng2_magneto_pos")):
            if f"ign_key{i}" in values:
                pos = int(values[f"ign_key{i}"])
                setattr(state, attr, pos)
                # The two switches the position is the sum of, for the
                # files that check a magneto on its own.
                setattr(state, "magneto1_left" if i == 0 else "magneto2_left",
                        pos in (1, 3, 4))
                setattr(state, "magneto1_right" if i == 0 else "magneto2_right",
                        pos in (2, 3, 4))
        for i, attr in enumerate(("ignition_on", "eng2_ignition_on")):
            if f"ignition_on{i}" in values:
                setattr(state, attr, bool(values[f"ignition_on{i}"]))
        # The doors: per-exit reads plus the most-open figure a
        # "doors ... closed" item checks.
        doors = [values[f"door{i}"] for i in range(10) if f"door{i}" in values]
        if doors:
            state.exits_open_pct = max(doors) * 100.0
        for i in range(4):
            if f"door{i}" in values:
                setattr(state, f"exit{i + 1}_open_pct",
                        values[f"door{i}"] * 100.0)
        state.speedbrake_pct = max(0.0, values.get("speedbrake", 0.0)) * 100.0
        if "flow0" in values:
            state.eng_fuel_flow_pph = values["flow0"] * LB * 3600
        wing = [bool(values[k]) for k in ("wing_boot", "wing_heat")
                if k in values]
        if wing:
            state.wing_deice_on = any(wing)
        # The helicopter's rotor instruments XP publishes no dedicated
        # names for: the free-turbine percent is its Nr, the governor's
        # switch is its target's presence, and the yaw channel is the
        # anti-torque pedals.
        if state.category == "Helicopter":
            if "n2_0" in values:
                state.rotor_rpm_pct = values["n2_0"]
            if "gov_0" in values:
                state.rotor_gov_target_pct = \
                    100.0 if values["gov_0"] else 0.0
            if "pedal_yaw" in values:
                state.tail_rotor_pedal_position = values["pedal_yaw"] * 100.0
        state.flaps_index = round(values.get("flaps", 0) * max(0, values.get("flap_detents", 1) - 1))
        state.transponder_state = {0: 0, 1: 1, 2: 3, 3: 4, 4: 2, 5: 4, 6: 4, 7: 4}.get(
            int(values.get("xp_transponder", 0)), 0)
        state.transponder_mode = 5 if values.get("ident") else (
            1 if state.transponder_state == 2 else state.transponder_state)
        state.transponder_code = f"{int(values.get('squawk', 1200)):04d}"
        # audio_com_selection is the mic selector: 0 COM1, 1 COM2. Absent
        # means "assume the radio the pilot was last on" -- COM1.
        state.com1_transmit = values.get("transmit", 0) == 0
        # servos_on is a boolean, not the autopilot_mode enum of
        # sim/cockpit/autopilot: the AP is engaged where it reads non-zero.
        state.autopilot = bool(values.get("control:ap_on",
                                          values.get("autopilot_mode")))
        # X-Plane's selector is its own enum (0 none, 1 left, 2 centre,
        # 3 right, 4 all, 5 aft); the state carries the SimConnect one.
        state.fuel_selector = {0: 0, 1: 2, 2: 6, 3: 3, 4: 1, 5: 8}.get(
            int(values.get("fuel_selector", -1)), -1)
        state.gps_drives_nav1 = values.get("hsi_source", 0) in (2, 3)
        state.nav1_has_localizer = bool(int(values.get("nav_type", 0)) & (8 | 16))
        state.ambient_precip_state = (4 if values.get("rain", 0) > 0
                                      else 2 if values.get("snow", 0) > 0
                                      else 0)
        # The bank selector's active step. The mode dataref is an enum:
        # 0 is automatic and 1-6 pick bank_angles_values[0..5] -- the same
        # indexing AP_MAX_BANK writes use (xpcontrols.py sends index+1).
        bank_mode = int(values.get("bank_mode", -1))
        state.ap_max_bank_deg = (
            values[f"bank_angle{bank_mode - 1}"]
            if 1 <= bank_mode <= 6 and f"bank_angle{bank_mode - 1}" in values
            else 0.0)
        state.spoilers_armed = values.get("speedbrake", 0) < 0
        # X-Plane reports fuel mass, not volume; use the same nominal 6 lb/gal
        # as ManualSource where the model does not declare a fuel density.
        state.fuel_total_gal = state.fuel_total_lb / 6
        state.fuel_capacity_gal = values.get("fuel_capacity_kg", 0) * LB / 6
        state.est_fuel_flow_pph = sum(values.get(f"flow{i}", 0) for i in range(16)) * LB * 3600
        if "fuel_left_kg" in values:
            state.fuel_left_gal = values["fuel_left_kg"] * LB / 6
        if "fuel_right_kg" in values:
            state.fuel_right_gal = values["fuel_right_kg"] * LB / 6
        # The SimConnect COM STATUS's two honest answers onto the power
        # switch X-Plane publishes: on means working, off means no
        # electricity. Not fitted and failed it cannot say.
        if "com1_powered" in values:
            state.com1_status = 0 if values["com1_powered"] else 2
        # The published Mmo is also the barber pole's Mach where a red
        # line of its own is not separately published.
        if "mach_max_operate" in values and "barber_pole_mach" not in values:
            state.barber_pole_mach = state.mach_max_operate
        trq, trq_red = values.get("trq_nm"), values.get("trq_red_hi_ftpds", 0.0)
        if trq_red < 50.0:
            # A 1 ft-lb red line is a field the model did not fill in, not
            # a limit any engine has -- reading it as real made a King
            # Air's healthy torque report seventeen hundred times its
            # ceiling and flapped the collective limiter every tick.
            trq_red = 0.0
        if trq is not None and trq_red > 0.0:
            # N-m against the book's ft-lbs red line, the per cent of
            # rated the collective limit reads.
            state.eng_torque_pct = trq / (trq_red * 1.35582) * 100.0
        if trq is not None:
            state.eng_torque_nm = trq
        # The model's own red lines, where it publishes them as numbers:
        # Plane Maker's limit fields all default to paint (red_hi_TRQ
        # reading 1.0 is a King Air's way of saying the author never set
        # it), so every figure gets a plausibility gate before it can
        # rule the lever cap.
        tq_lim = tq_toff = 0.0
        if values.get("acf_max_trq_nm", 0.0) >= 100.0:
            tq_toff = values["acf_max_trq_nm"]
        if values.get("trq_red_hi_ftpds", 0.0) >= 50.0:
            tq_toff = tq_toff or values["trq_red_hi_ftpds"] * 1.35582
        if values.get("trq_grn_hi_ftpds", 0.0) >= 50.0:
            tq_lim = values["trq_grn_hi_ftpds"] * 1.35582
        if not tq_toff:
            tq_toff = tq_lim
        if not tq_lim:
            tq_lim = tq_toff
        state.eng_tq_limit_nm, state.eng_tq_takeoff_nm = tq_lim, tq_toff
        itt_red, itt_grn = (values.get("itt_red_hi_c", 0.0),
                            values.get("itt_grn_hi_c", 0.0))
        if 300.0 <= itt_red <= 1400.0:
            state.eng_itt_takeoff_c = itt_red
            state.eng_itt_limit_c = (itt_grn
                                     if 300.0 <= itt_grn <= itt_red
                                     else itt_red)
        elif 300.0 <= itt_grn <= 1400.0:
            state.eng_itt_takeoff_c = state.eng_itt_limit_c = itt_grn
        # Engines two through four, where the airframe has them: the
        # same gauges and levers engine one's scalars report, under the
        # eng2_/eng3_/eng4_ names. acf_num_engines is the roster -- the
        # dataref arrays answer zeros past it, and a zero is not an
        # engine -- so an absent count stands the whole row down rather
        # than inventing three more engines' readings.
        for i in range(1, 4):
            if state.engine_count <= i:
                break
            eng = f"eng{i + 1}"
            if f"thr{i}" in values:
                setattr(state, f"{eng}_throttle_pct",
                        values[f"thr{i}"] * 100.0)
            if f"mix{i}" in values:
                setattr(state, f"{eng}_mixture_pct",
                        values[f"mix{i}"] * 100.0)
                # On a turboprop the mixture channel is the condition
                # lever, the same wiring engine one's condition_pct
                # derives.
                if state.engine_type == 5:
                    setattr(state, f"{eng}_condition_pct",
                            values[f"mix{i}"] * 100.0)
            if f"prop{i}" in values and state.engine_type in (1, 5):
                setattr(state, f"{eng}_prop_pct",
                        values[f"prop{i}"] * 100.0)
            if f"trq_nm{i}" in values:
                setattr(state, f"{eng}_torque_nm", values[f"trq_nm{i}"])
                if trq_red > 0.0:
                    setattr(state, f"{eng}_torque_pct",
                            values[f"trq_nm{i}"]
                            / (trq_red * 1.35582) * 100.0)
            if f"itt{i}" in values:
                setattr(state, f"{eng}_itt_c", values[f"itt{i}"])
            if f"n1_{i}" in values:
                setattr(state, f"{eng}_n1_pct", values[f"n1_{i}"])
            if f"n2_{i}" in values:
                setattr(state, f"{eng}_n2_pct", values[f"n2_{i}"])
            if f"rpm{i}" in values:
                setattr(state, f"{eng}_rpm", values[f"rpm{i}"])
        # The world-axes wind onto the body axes the weather watch
        # reads: along the nose and across the wing, knots.
        if "wind_x_ms" in values or "wind_z_ms" in values:
            psi = math.radians(float(state.heading_true or 0.0))
            wx, wz = values.get("wind_x_ms", 0.0), values.get("wind_z_ms", 0.0)
            state.aircraft_wind_x_kt = (wx * math.cos(psi)
                                        + wz * math.sin(psi)) * KT
            state.aircraft_wind_z_kt = (wx * math.sin(psi)
                                        - wz * math.cos(psi)) * KT
        if "wind_y_ms" in values:
            state.aircraft_wind_y_kt = values["wind_y_ms"] * KT
        # The FAA heavy line on the model's own maximum weight.
        if state.max_gross_weight_lb > 0:
            state.atc_heavy = state.max_gross_weight_lb >= 255000.0
        # Inside a broken-or-thicker layer's base and tops is what "in
        # cloud" means; scattered wisps below that coverage are not.
        altitude_m = float(state.altitude_ft or 0.0) / FT
        state.ambient_in_cloud = any(
            values.get(f"cloud_cov{i}", 0.0) >= 0.5
            and values.get(f"cloud_base_m{i}", -1.0)
            <= altitude_m <= values.get(f"cloud_top_m{i}", -1.0)
            for i in range(3))
        # The cooling watch's instruments (atc.powerplant.cooling_for):
        # the hottest cylinder-head and oil temperatures the engines
        # report -- degc_or_f datarefs, converted by their acf flags --
        # the model's own CHT red line, and where the cowl-flap levers
        # sit. Missing engines contribute nothing to the max, so a
        # single reads engine one's gauges and a twin the hotter of two.
        count = max(1, min(8, int(state.engine_count or 0)))
        if not bool(values.get("cht_is_c", 1)):
            def _to_c(t):
                return (t - 32.0) * 5.0 / 9.0
        else:
            def _to_c(t):
                return t
        if "cht0" in values:
            state.eng_cht_c = max(_to_c(values.get(f"cht{i}", 0.0))
                                  for i in range(count))
        red = values.get("cht_red_hi", 0.0)
        if red > 0.0:
            state.eng_cht_limit_c = _to_c(red)
        if not bool(values.get("oil_is_c", 1)):
            def _to_c(t):
                return (t - 32.0) * 5.0 / 9.0
        else:
            def _to_c(t):
                return t
        if "oil0" in values:
            state.eng_oil_temp_c = max(_to_c(values.get(f"oil{i}", 0.0))
                                     for i in range(count))
        if "cowl0" in values:
            state.eng_cowl_pct = max(values.get(f"cowl{i}", 0.0)
                                     for i in range(count)) * 100.0
        known = set(DATAREFS).intersection(values)
        known.update(key for key, dependencies in DERIVED.items()
                     if all(dependency in values for dependency in dependencies))
        for derived, source_key in (("eng_cht_c", "cht0"),
                                    ("eng_oil_temp_c", "oil0"),
                                    ("eng_cowl_pct", "cowl0"),
                                    ("eng_cht_limit_c", "cht_red_hi"),
                                    ("fuel_left_gal", "fuel_left_kg"),
                                    ("fuel_right_gal", "fuel_right_kg"),
                                    ("com1_status", "com1_powered"),
                                    ("barber_pole_mach", "mach_max_operate"),
                                    ("aircraft_wind_x_kt", "wind_x_ms"),
                                    ("aircraft_wind_y_kt", "wind_y_ms"),
                                    ("aircraft_wind_z_kt", "wind_z_ms"),
                                    ("atc_heavy", "max_gross_weight_lb"),
                                    ("ambient_in_cloud", "cloud_cov0")):
            if source_key in values:
                known.add(derived)
        if trq is not None and trq_red > 0.0:
            known.add("eng_torque_pct")
        if trq is not None:
            known.add("eng_torque_nm")
        if state.eng_tq_limit_nm:
            known.add("eng_tq_limit_nm")
            known.add("eng_tq_takeoff_nm")
        if state.eng_itt_limit_c:
            known.add("eng_itt_limit_c")
            known.add("eng_itt_takeoff_c")
        if state.engine_type == 5 and "mixture_pct" in values:
            known.add("condition_pct")
        if state.engine_type in (1, 5) and "collective_position" in values:
            known.add("prop_pct")
        for i in range(1, 4):
            if state.engine_count <= i:
                break
            eng = f"eng{i + 1}"
            for suffix, src in (("throttle_pct", f"thr{i}"),
                                ("mixture_pct", f"mix{i}"),
                                ("torque_nm", f"trq_nm{i}"),
                                ("itt_c", f"itt{i}"),
                                ("n1_pct", f"n1_{i}"),
                                ("n2_pct", f"n2_{i}"),
                                ("rpm", f"rpm{i}")):
                if src in values:
                    known.add(f"{eng}_{suffix}")
            if f"prop{i}" in values and state.engine_type in (1, 5):
                known.add(f"{eng}_prop_pct")
            if f"mix{i}" in values and state.engine_type == 5:
                known.add(f"{eng}_condition_pct")
            if f"trq_nm{i}" in values and trq_red > 0.0:
                known.add(f"{eng}_torque_pct")
        if "wing_boot" in values or "wing_heat" in values:
            known.add("wing_deice_on")
        if state.category == "Helicopter":
            for attr, src in (("rotor_rpm_pct", "n2_0"),
                              ("rotor_gov_target_pct", "gov_0"),
                              ("tail_rotor_pedal_position", "pedal_yaw")):
                if src in values:
                    known.add(attr)
        if state.ap_max_bank_deg:
            known.add("ap_max_bank_deg")
        known.add("relocated")
        state.unavailable = tuple(f.name for f in fields(state) if f.name not in known)
        # A menu move or a slew is a position the aeroplane could not have
        # flown to -- further than its groundspeed could carry it since
        # the last read. Marked for a couple of seconds so the change-
        # watchers hold rather than fire on the delta.
        now = time.monotonic()
        if self.connected and state.has_position:
            if self._last_geo is not None:
                lat0, lon0, t0 = self._last_geo
                dt = max(0.05, now - t0)
                dlat = (state.latitude - lat0) * 60.0
                dlon = ((state.longitude - lon0)
                        * math.cos(math.radians((state.latitude + lat0) / 2))
                        * 60.0)
                reach_nm = (state.ground_speed_kt or 0.0) / 3600.0 * dt * 2.5 + 0.25
                if math.hypot(dlat, dlon) > reach_nm:
                    self._relocated_until = now + 2.0
            self._last_geo = (state.latitude, state.longitude, now)
        state.relocated = now < self._relocated_until
        # The prop governor's tick: the override the lever writes latch
        # carries no RPM limiting, so the controls walk the blade angle
        # toward the lever's commanded RPM every read.
        self._controls.govern_props()
        return state

    def read_now(self, attribute):
        if not self.connected:
            return None
        normalized = str(attribute).upper().replace("_", " ")
        attribute = next((attr for name, attr, _kind, _tier in SimConnectSource.REQUESTS
                          if name.replace("_", " ") == normalized), attribute)
        state = self.read()
        return None if attribute in state.unavailable else getattr(state, attribute, None)

    def _dref(self, path, value):
        try:
            if not self.connected or not math.isfinite(float(value)):
                return False
            self._packet(b"DREF\0" + struct.pack("<f500s", float(value), path.encode("ascii")))
            return True
        except (OSError, ValueError, TypeError, OverflowError):
            return False

    def dref(self, path, value):
        # Public write for channels outside the FO's named set, e.g. the
        # multiplayer control arrays driving AI slots.
        return self._dref(path, value)

    def value(self, key):
        with self._lock:
            return self._values.get(key)

    def subscribe(self, key, path, frequency=None):
        # Late subscriptions (native-traffic plane feedback): appended at
        # connect-time order otherwise, subscribed at once when live. The
        # index stays bound to the subscription list position, so removals
        # are not possible -- callers keep their keys.
        with self._lock:
            if any(existing == key for existing, _ in self._subscriptions):
                return
            index = len(self._subscriptions)
            self._subscriptions.append((key, path))
        if self.connected:
            self._subscribe(index, path, frequency or self.frequency)

    def _command(self, command):
        if not self.connected:
            return False
        try:
            self._packet(b"CMND\0" + command.encode("ascii") + b"\0")
            return True
        except OSError:
            return False

    # How long a write is given to show in the subscribed readback before
    # it is called a refusal -- the same pattern SimConnect's set_com uses.
    CONFIRM_S = 0.8
    CONFIRM_STEP_S = 0.05

    def _await(self, key, predicate):
        """Wait for the subscribed ``key`` to satisfy ``predicate``.

        DREF and CMND writes are fire-and-forget datagrams: this is the
        read-back half SimConnect's set_com already does. True when the
        key confirmed; False when it read back something else. A key the
        subscription never answers returns None -- the write went out and
        cannot be verified, which is an honest "sent", not a failure.
        """
        deadline = time.monotonic() + self.CONFIRM_S
        seen = False
        while time.monotonic() < deadline:
            with self._lock:
                found = self._values.get(key)
            if found is not None:
                seen = True
                if predicate(found):
                    return True
            time.sleep(self.CONFIRM_STEP_S)
        return False if seen else None

    def set_com(self, mhz, radio=1, standby=False):
        try:
            value = float(mhz)
            if radio not in (1, 2) or not 118 <= value <= 136.999:
                return False
            path = f"com{radio}_" + ("standby_" if standby else "") + "frequency_hz_833"
            if not self._dref(RADIOS + path, round(value * 1000)):
                return False
            # kHz channels: the subscribed key is the same dataref just
            # written, so the readback is the honest answer.
            key = f"com{radio}_{'standby' if standby else 'active'}"
            confirmed = self._await(key, lambda khz: abs(khz - value * 1000) <= 2.0)
            return confirmed is not False
        except (ValueError, TypeError):
            return False

    def swap_com(self, radio=1):
        return radio in (1, 2) and self._command(f"sim/radios/com{radio}_standy_flip")

    def set_nav(self, mhz, radio=1):
        try:
            value = float(mhz)
            if radio not in (1, 2) or not 108 <= value <= 117.95:
                return False
            if not self._dref(RADIOS + f"nav{radio}_frequency_hz", round(value * 100)):
                return False
            confirmed = self._await(f"nav{radio}_freq",
                                    lambda khz: abs(khz - value * 100) < 2.0)
            return confirmed is not False
        except (ValueError, TypeError):
            return False

    def set_squawk(self, code):
        code = str(code)
        if not (len(code) == 4 and all(c in "01234567" for c in code)):
            return False
        if not self._dref(RADIOS + "transponder_code", int(code)):
            return False
        confirmed = self._await("squawk", lambda seen: int(seen) == int(code))
        return confirmed is not False

    def pushback(self):
        return self._command("sim/ground_ops/pushback_straight")

    def set_variable(self, name, value, unit=None):
        try:
            name = str(name).upper().replace("_", " ")
            value = float(value)
            if not math.isfinite(value):
                return False
            if self._controls.set_variable(name, value, unit):
                return True
            if name == "FUEL TANK SELECTOR:1":
                # SimConnect's selector enum to X-Plane's: positions the
                # model does not have (aux, tips, externals, crossfeeds)
                # are asked for as all -- fuel flowing beats none.
                translated = {0: 0, 1: 4, 2: 1, 3: 3, 4: 1, 5: 3, 6: 2,
                              7: 2, 8: 5, 9: 4, 10: 4, 11: 3, 12: 1,
                              13: 4, 14: 4, 15: 4, 16: 4, 17: 4, 18: 0,
                              19: 1, 20: 3}.get(int(value), 4)
                return self._dref(DATAREFS["fuel_selector"][0], translated)
            mapping = WRITABLE.get(name)
            if mapping is None:
                return False
            attribute, scale = mapping
            # DATAREFS entries are (path, factor) pairs, EXTRA entries
            # the path alone -- the two shapes, not one.
            if attribute in DATAREFS:
                path = DATAREFS[attribute][0]
            else:
                path = EXTRA.get(attribute)
            if not path:
                return False
            return self._dref(path, float(value) * scale)
        except (TypeError, ValueError, OverflowError):
            return False

    def _send(self, name, value):
        return self._controls.send(name, value)

    def loaded_flight_plan(self):
        # There is no plan file on this link; the engine's _poll_sim_plan
        # drives request_flight_plan/flight_plan_reply instead, which the
        # plugin's FMS readback answers.
        return None

    def request_flight_plan(self):
        """Whether the FMS entries can be asked for right now."""
        try:
            return bool(self.fms_reader and self.fms_reader())
        except Exception:
            return False

    def flight_plan_reply(self):
        """The plan in the FMS, as the plugin last read it -- a list of
        entry dicts, not a path; the engine turns it into a route."""
        try:
            entries = self.fms_reader() if self.fms_reader else None
        except Exception:
            return None
        return list(entries) if entries else None

    def load_fms_plan(self, route) -> bool:
        """The route into the simulator's own FMS, over the UDP channel.

        X-Plane 12.06 and later take an ``FPLN`` datagram carrying a
        v1100-format flight plan -- the exact text
        ``XPLMLoadFMSFlightPlan`` would take, procedures and all -- so
        the box ends up flying what the engine thinks was filed. Older
        builds silently drop the packet, which is a polite no. The
        plugin's next status read carries the new entries back, which
        is the verification this write cannot otherwise have.
        """
        try:
            from ..atc.route import to_fms

            text = to_fms(route)
        except Exception:
            log.debug("the .fms text could not be built", exc_info=True)
            return False
        if not text or not self.connected:
            return False
        payload = text.encode("utf-8", "replace")
        # The reference client pads the plan to 1400 bytes; a longer one
        # is sent whole and its own datagram length ends it.
        try:
            self._packet(b"FPLN\0" + payload.ljust(max(1400, len(payload))))
            return True
        except OSError:
            return False

    def fuel_tanks(self):
        if not self.connected:
            return []
        with self._lock:
            values = self._values.copy()
        capacity = values.get("fuel_capacity_kg", 0)
        return [(f"sim/flightmodel/weight/m_fuel[{i}]",
                 capacity * values[f"tank_ratio{i}"] * LB / 6,
                 values.get(f"tank{i}", 0) * LB / 6)
                for i in range(9) if values.get(f"tank_ratio{i}", 0) > 0]

    def set_fuel(self, gallons):
        """Fill every tank to the same fraction of its capacity.

        A tank that refuses the write does not void the others: fuel is
        set when at least one took it, which is what the caller's
        "did the fuel go in" is asking. A model that does not publish its
        total capacity is filled by even share of the tanks it does show.
        """
        try:
            gallons = float(gallons)
            if not math.isfinite(gallons) or gallons < 0:
                return False
            tanks = self.fuel_tanks()
            capacity = sum(cap for _, cap, _ in tanks)
            if capacity > 0:
                fraction = min(1, gallons / capacity)
                wanted = [(path, cap * fraction) for path, cap, _ in tanks]
            else:
                with self._lock:
                    values = self._values.copy()
                present = [f"sim/flightmodel/weight/m_fuel[{i}]"
                           for i in range(9)
                           if values.get(f"tank{i}", 0) > 0
                           or values.get(f"tank_ratio{i}", 0) > 0]
                if not present:
                    return False
                wanted = [(path, gallons / len(present)) for path in present]
            writes = [self._dref(path, gal * 6 / LB) for path, gal in wanted]
            return any(writes)
        except (ValueError, TypeError):
            return False

    def close(self):
        self._stop.set()
        if self._socket is not None:
            # The overrides first, while there is still a socket to say
            # it on, and whether or not the link is still answering:
            # released after the socket was shut, the write went nowhere
            # and X-Plane was left ignoring the pilot's own stick, pedals
            # and brakes for the rest of the session.
            for flag in OVERRIDES:
                with contextlib.suppress(OSError, ValueError):
                    self._packet(b"DREF\0" + struct.pack(
                        "<f500s", 0.0, (OVR + flag).encode("ascii")))
            for index, (_key, path) in enumerate(self._subscriptions):
                with contextlib.suppress(OSError):
                    self._subscribe(index, path, 0)
            if self._thread is not None:
                self._thread.join(timeout=.5)
            self._socket.close()
            self._socket = None
        self._peer = None
        self._sender = None
        self._answered.clear()
        self._controls.clear()
