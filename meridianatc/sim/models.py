"""Which add-on flies, and what its systems are actually called.

Study-level aircraft do not always answer the simulator's stock
variables: FlyByWire's Airbuses keep their overhead pushbuttons in
``L:`` variables, the devinci Concorde and Zibo's 737 keep theirs in
custom datarefs. The state reader's "nothing heard is nothing broken"
rule would then wave every checklist item through, so each known
add-on gets a table that points the same field at the name the model
really publishes. An add-on not in the tables reads the stock names,
exactly as before.

Detection needs no new sim channel: MSFS already reads ``TITLE`` into
``state.title`` and X-Plane reads ``acf_ui_name`` plus
``acf_relative_path`` (``STRINGS``), so a fragment match on those
picks the table -- "flybywire a380x" keys the A380X, the acf path's
``B737-800X`` keys Zibo.

MSFS aliases are keyed by the SimConnect wire name the readers and the
first officer already use ("GENERAL ENG FUEL PUMP SWITCH:1"); the
value is the ``L:`` variable the add-on answers instead. X-Plane read
aliases are keyed by the ``AircraftState`` attribute, write aliases by
the same wire name MSFS uses -- ``XPlaneSource.set_variable`` accepts
those names today. Aliases only ever change where a value comes from:
an add-on that leaves the stock variable live keeps using it, and a
name an add-on does not publish is left out of its table rather than
guessed.
"""

from __future__ import annotations

# Fragments of the sim's own aircraft identity that name the add-on,
# not just the type: the title's "FlyByWire A380X" and the acf path's
# "Devinci_B2707_300/b2707_300.acf". Longest-phrase first where one
# name contains another (fenix a321 before fenix).
MSFS_MODELS: tuple[tuple[str, str], ...] = (
    ("flybywire a380", "fbw_a380x"), ("a380x", "fbw_a380x"),
    ("flybywire", "fbw_a32nx"), ("a32nx", "fbw_a32nx"),
    ("headwind", "fbw_a32nx"),          # A330-900 on the A32NX codebase
    ("fenix a321", "fenix"), ("fenix", "fenix"),
    ("pmdg 737", "pmdg_737"), ("pmdg 747", "pmdg_747"),
    ("pmdg 777", "pmdg_777"),
    ("ifly", "ifly_737"),
    ("inibuilds a310", "ini_a310"), ("inibuilds a350", "ini_a350"),
    ("inibuilds a380", "ini_a380"), ("inibuilds", "inibuilds"),
    ("aerosoft crj", "aerosoft_crj"), ("crj 5", "aerosoft_crj"),
    ("crj 7", "aerosoft_crj"), ("crj 9", "aerosoft_crj"),
    ("maddog", "leonardo_md82"),
    ("a2a", "a2a"), ("blacksquare", "blacksquare"),
    ("devinci", "devinci_b2707"), ("2707", "devinci_b2707"),
)

XP_MODELS: tuple[tuple[str, str], ...] = (
    ("devinci_b2707", "devinci_b2707"), ("b2707", "devinci_b2707"),
    ("2707", "devinci_b2707"),
    ("mad_a320", "mad_a320"),
    ("737-800x", "zibo_738"), ("zibo", "zibo_738"),
    ("toliss", "toliss"), ("airbusfbw", "toliss"),
    ("flightfactor", "flightfactor"), ("1-sim", "flightfactor"),
    ("ixeg", "ixeg_733"),
    ("rotate", "rotate_md80"), ("md-80", "rotate_md80"),
    ("cl650", "hotstart_cl650"), ("hotstart", "hotstart"),
    ("taog", "taog"), ("nemeth", "nemeth"), ("cowansim", "cowansim"),
)


def msfs_model_key(title: str, model: str = "") -> str:
    """The add-on the title names, or "" for a stock-aircraft model."""
    text = f"{title} {model}".lower()
    for fragment, key in MSFS_MODELS:
        if fragment in text:
            return key
    return ""


def xp_model_key(acf_path: str, title: str = "") -> str:
    """The add-on the loaded .acf names, or "" for a stock model.

    The relative path ("Aircraft/Extra Aircraft/Devinci_B2707_300/
    b2707_300.acf") identifies an add-on whose title does not -- the
    MAD A320 calls itself "Airbus A320 Neo" like a stock model would.
    """
    text = f"{acf_path} {title}".lower()
    for fragment, key in XP_MODELS:
        if fragment in text:
            return key
    return ""


# ---------------------------------------------------------------------
# MSFS: SimConnect wire name -> the L: variable the add-on answers.
# ---------------------------------------------------------------------

# FlyByWire A32NX/A380X -- the Flight Deck API's pushbutton variables,
# confirmed against the installed A380X package's own var list. FBW
# still drives the stock simvars for generators, starters, ignition,
# lights, gear and flaps, so only the overhead pushbuttons and signs
# need remapping.
# Wire names as the request table and the first officer's write calls
# spell them: the battery is asked without an index, the signs by
# their CABIN ... ALERT names. A32NX_APU_N is an ARINC 429 word, not a
# number -- the RAW variant is the plain percent. The transponder is
# deliberately not aliased: the FBW mode knob's enum (0 STBY, 1 AUTO,
# 2 ON, 3 ALT RPTG, 4 TA, 5 TA/RA) does not line up with the
# SimConnect one a check compares against, and a translated read
# would still leave the FO's writes wrong.
_FBW: dict[str, str] = {
    "ELECTRICAL MASTER BATTERY": "L:A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO",
    "CABIN SEATBELTS ALERT SWITCH":
        "L:XMLVAR_SWITCH_OVHD_INTLT_SEATBELT_Position",
    "CABIN NO SMOKING ALERT SWITCH":
        "L:XMLVAR_SWITCH_OVHD_INTLT_NOSMOKING_POSITION",
    "BLEED AIR APU": "L:A32NX_OVHD_PNEU_APU_BLEED_PB_IS_ON",
    "BLEED AIR ENGINE:1": "L:A32NX_OVHD_PNEU_ENG_1_BLEED_PB_IS_AUTO",
    "BLEED AIR ENGINE:2": "L:A32NX_OVHD_PNEU_ENG_2_BLEED_PB_IS_AUTO",
    "APU SWITCH": "L:A32NX_OVHD_APU_MASTER_SW_PB_IS_ON",
    "APU PCT RPM": "L:A32NX_APU_N_RAW",
}

_FBW_A380X: dict[str, str] = {
    **_FBW,
    # Four engines on the 380.
    "BLEED AIR ENGINE:3": "L:A32NX_OVHD_PNEU_ENG_3_BLEED_PB_IS_AUTO",
    "BLEED AIR ENGINE:4": "L:A32NX_OVHD_PNEU_ENG_4_BLEED_PB_IS_AUTO",
}

MSFS_ALIASES: dict[str, dict[str, str]] = {
    "fbw_a32nx": _FBW,
    "fbw_a380x": _FBW_A380X,
}


# ---------------------------------------------------------------------
# X-Plane: AircraftState attribute -> (custom dataref, factor), and the
# SimConnect-style write name -> the same dataref.
# ---------------------------------------------------------------------

# MAD_A320 -- merspieler/FG-port A320 running the Laminar A333 xlua
# system code. Ref names verified in the installed package's
# plugins/xlua sources (create_dataref calls).
_MAD_A320_READ: dict[str, tuple[str, float]] = {
    "battery_master": ("laminar/A333/buttons/battery1_pos", 1),
    "generator_on": ("laminar/A333/buttons/gen1_pos", 1),
    "eng2_generator_on": ("laminar/A333/buttons/gen2_pos", 1),
    "apu_gen_on": ("laminar/A333/buttons/gen_apu_pos", 1),
    "apu_on": ("laminar/A333/buttons/APU_master", 1),
    "apu_pct": ("laminar/A333/annun/apu_avail", 100),
    "apu_bleed_on": ("laminar/A333/buttons/apu_bleed_pos", 1),
    "bleed_air_on": ("laminar/A333/buttons/eng_bleed_1_pos", 1),
    "eng2_bleed_air_on": ("laminar/A333/buttons/eng_bleed_2_pos", 1),
    "pack_left_on": ("laminar/A333/buttons/pack_1_pos", 1),
    "pack_right_on": ("laminar/A333/buttons/pack_2_pos", 1),
    "fuel_pump_on": ("laminar/A333/fuel/buttons/left1_pump_pos", 1),
    "eng2_fuel_pump_on": ("laminar/A333/fuel/buttons/right1_pump_pos", 1),
    "hydraulic_pump_on":
        ("laminar/A330/buttons/hyd/eng1_pump_green_pos", 1),
    "eng2_hydraulic_pump_on":
        ("laminar/A330/buttons/hyd/eng2_pump_yellow_pos", 1),
    "engine_anti_ice_on": ("laminar/A333/buttons/engine_anti_ice1", 1),
    "eng2_engine_anti_ice_on": ("laminar/A333/buttons/engine_anti_ice2", 1),
    "window_heat_on": ("laminar/A333/buttons/window_probe_ice", 1),
    "pitot_heat": ("laminar/A333/buttons/window_probe_ice", 1),
    "aoa_heat_on": ("laminar/A333/buttons/window_probe_ice", 1),
    "seatbelt_sign": ("laminar/A333/switches/fasten_seatbelts", 1),
    "no_smoking_sign": ("laminar/A333/switches/no_smoking", 1),
    "parking_brake": ("laminar/A333/switches/park_brake_pos", 1),
    "oxygen_on": ("laminar/A333/buttons/oxy/crew_valve_pos", 1),
    # The FMA's AP1+2 annunciation, nonzero while either channel flies.
    "autopilot": ("laminar/A333/PFD/FMAs/autopilot_12_status", 1),
}

_MAD_A320_WRITE: dict[str, str] = {
    "GENERAL ENG FUEL PUMP SWITCH:1":
        "laminar/A333/fuel/buttons/left1_pump_pos",
    "GENERAL ENG FUEL PUMP SWITCH:2":
        "laminar/A333/fuel/buttons/right1_pump_pos",
    "GENERAL ENG GENERATOR SWITCH:1": "laminar/A333/buttons/gen1_pos",
    "GENERAL ENG GENERATOR SWITCH:2": "laminar/A333/buttons/gen2_pos",
    "APU GENERATOR SWITCH": "laminar/A333/buttons/gen_apu_pos",
    "BLEED AIR APU": "laminar/A333/buttons/apu_bleed_pos",
    "BLEED AIR ENGINE:1": "laminar/A333/buttons/eng_bleed_1_pos",
    "BLEED AIR ENGINE:2": "laminar/A333/buttons/eng_bleed_2_pos",
    "APU PACKS": "laminar/A333/buttons/pack_1_pos",
    "CABIN SEATBELTS ALERT SWITCH":
        "laminar/A333/switches/fasten_seatbelts",
    "CABIN NO SMOKING ALERT SWITCH": "laminar/A333/switches/no_smoking",
    "BRAKE PARKING POSITION": "laminar/A333/switches/park_brake_pos",
}

# Zibo 737-800X -- names from the package's own B738_Datarefs.txt.
_ZIBO_READ: dict[str, tuple[str, float]] = {
    "apu_on": ("laminar/B738/switches/apu_start", 1),
    "apu_bleed_on": ("laminar/B738/toggle_switch/bleed_air_apu_pos", 1),
    "bleed_air_on": ("laminar/B738/toggle_switch/bleed_air_1_pos", 1),
    "eng2_bleed_air_on": ("laminar/B738/toggle_switch/bleed_air_2_pos", 1),
    "pack_left_on": ("laminar/B738/air/l_pack_pos", 1),
    "pack_right_on": ("laminar/B738/air/r_pack_pos", 1),
    "fuel_pump_on": ("laminar/B738/fuel/fuel_tank_pos_lft1", 1),
    "eng2_fuel_pump_on": ("laminar/B738/fuel/fuel_tank_pos_rgt1", 1),
    "hydraulic_pump_on":
        ("laminar/B738/toggle_switch/hydro_pumps1_pos", 1),
    "eng2_hydraulic_pump_on":
        ("laminar/B738/toggle_switch/hydro_pumps2_pos", 1),
    "engine_anti_ice_on": ("laminar/B738/ice/eng1_heat_pos", 1),
    "eng2_engine_anti_ice_on": ("laminar/B738/ice/eng2_heat_pos", 1),
    "window_heat_on": ("laminar/B738/ice/window_heat_l_fwd_pos", 1),
    "pitot_heat": ("laminar/B738/toggle_switch/capt_probes_pos", 1),
    "spoilers_armed": ("laminar/B738/annunciator/speedbrake_armed", 1),
    "parking_brake": ("laminar/B738/annunciator/parking_brake", 1),
    "autopilot": ("laminar/B738/autopilot/cmd_a_status", 1),
    # A flight with only CMD B engaged answers autopilot off through
    # the first channel alone; the second is a separate slot the
    # reader ORs in.
    "autopilot2": ("laminar/B738/autopilot/cmd_b_status", 1),
    "autothrottle": ("laminar/B738/autopilot/autothrottle_status", 1),
    "kohlsman_hpa": ("laminar/B738/EFIS/baro_sel_in_hg_pilot", 33.8639),
    "minimums_ft": ("laminar/B738/fmod/dh_minimum_pilot", 1),
    # Deliberately not aliased: the starter-detent and crossfeed enums
    # do not line up with the SimConnect positions a check compares
    # against, and wing_heat is only an extra write target, not a
    # state attribute the readers fill.
}

_ZIBO_WRITE: dict[str, str] = {
    "GENERAL ENG FUEL PUMP SWITCH:1":
        "laminar/B738/fuel/fuel_tank_pos_lft1",
    "GENERAL ENG FUEL PUMP SWITCH:2":
        "laminar/B738/fuel/fuel_tank_pos_rgt1",
    "BLEED AIR APU": "laminar/B738/toggle_switch/bleed_air_apu_pos",
    "BLEED AIR ENGINE:1": "laminar/B738/toggle_switch/bleed_air_1_pos",
    "BLEED AIR ENGINE:2": "laminar/B738/toggle_switch/bleed_air_2_pos",
    "APU PACKS": "laminar/B738/air/l_pack_pos",
}

# Devinci B2707-300 Concorde -- names from the installed plugin's
# dataref table (b2707/*).
_B2707_READ: dict[str, tuple[str, float]] = {
    "battery_master": ("b2707/electrical/panel/battery_master", 1),
    "generator_on": ("b2707/electrical/panel/generator_control_1", 1),
    "eng2_generator_on": ("b2707/electrical/panel/generator_control_2", 1),
    "eng3_generator_on": ("b2707/electrical/panel/generator_control_3", 1),
    "eng4_generator_on": ("b2707/electrical/panel/generator_control_4", 1),
    "parking_brake": ("b2707/brakes/parking_brake_engaged", 1),
    "hydraulic_pressure": ("b2707/brakes/accumulator_pressure_psi", 1),
    "hydraulic_pump_on": ("b2707/hydraulic/all_circuits_pressurised", 1),
    # Concorde feeds its four engines from nineteen boost pumps; the
    # forward four are the ones a "pumps on" call would light up first.
    "fuel_pump_on": ("b2707/fuel/panel/boost_pump_1", 1),
    "eng2_fuel_pump_on": ("b2707/fuel/panel/boost_pump_2", 1),
    "eng3_fuel_pump_on": ("b2707/fuel/panel/boost_pump_3", 1),
    "eng4_fuel_pump_on": ("b2707/fuel/panel/boost_pump_4", 1),
    # doors/position is kept out: what it counts was not verified, and
    # an exits check with the wrong scale fails doors that are shut.
}

_B2707_WRITE: dict[str, str] = {
    "GENERAL ENG GENERATOR SWITCH:1":
        "b2707/electrical/panel/generator_control_1",
    "GENERAL ENG GENERATOR SWITCH:2":
        "b2707/electrical/panel/generator_control_2",
    "GENERAL ENG GENERATOR SWITCH:3":
        "b2707/electrical/panel/generator_control_3",
    "GENERAL ENG GENERATOR SWITCH:4":
        "b2707/electrical/panel/generator_control_4",
    "GENERAL ENG FUEL PUMP SWITCH:1": "b2707/fuel/panel/boost_pump_1",
    "GENERAL ENG FUEL PUMP SWITCH:2": "b2707/fuel/panel/boost_pump_2",
    "GENERAL ENG FUEL PUMP SWITCH:3": "b2707/fuel/panel/boost_pump_3",
    "GENERAL ENG FUEL PUMP SWITCH:4": "b2707/fuel/panel/boost_pump_4",
}

# ToLiss A319/A320/A321/A330/A340 -- names confirmed against the
# published package's own smartcopilot.cfg and the cockpit-builders
# threads on the official forum: the individually named switches are
# safe; the OHP array indices are not, so fuel pumps, bleeds, packs and
# generators stay on the stock names rather than a guessed index (the
# generator pbs are not exposed at all, per the vendor). The engine
# masters (AirbusFBW/ENG1MasterSwitch) deliberately map nothing: an
# ENG MASTER on an Airbus is not the ignition switch a check asks
# about, and a wrong alias fails items that were right.
_TOLISS_READ: dict[str, tuple[str, float]] = {
    # ElecOHPArray[5] is BAT1's switch -- the one battery_master checks.
    "battery_master": ("AirbusFBW/ElecOHPArray[5]", 1),
    "apu_on": ("AirbusFBW/APUMaster", 1),
    "pitot_heat": ("AirbusFBW/ProbeHeatSwitch", 1),
    "aoa_heat_on": ("AirbusFBW/ProbeHeatSwitch", 1),
    "window_heat_on": ("AirbusFBW/ProbeHeatSwitch", 1),
    "oxygen_on": ("AirbusFBW/CrewOxySwitch", 1),
}

_TOLISS_WRITE: dict[str, str] = {
    "APU SWITCH": "AirbusFBW/APUMaster",
}

XP_READ_ALIASES: dict[str, dict[str, tuple[str, float]]] = {
    "mad_a320": _MAD_A320_READ,
    "zibo_738": _ZIBO_READ,
    "devinci_b2707": _B2707_READ,
    "toliss": _TOLISS_READ,
}

XP_WRITE_ALIASES: dict[str, dict[str, str]] = {
    "mad_a320": _MAD_A320_WRITE,
    "zibo_738": _ZIBO_WRITE,
    "devinci_b2707": _B2707_WRITE,
    "toliss": _TOLISS_WRITE,
}

# The model keys left to the stock fallback, on purpose: flightfactor,
# hotstart_cl650, rotate_md80, ixeg_733 and the helicopter makers
# (taog, nemeth, cowansim) publish their cockpit datarefs only in
# their own SDKs or not at all, and pmdg_737/fenix/ifly on MSFS expose
# no documented L: names for the switches the checks read. A guessed
# name fails an item the stock value answered correctly, so they stay
# unmapped until a published list confirms one.
