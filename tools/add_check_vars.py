"""Give every checklist item that names a readable state its check.

Second pass after commit a0adcb5: the items it left answer-only because
no AircraftState field existed for them -- magnetos, fuel pumps,
generators, doors, brakes, de-ice, the APU, bleeds, hydraulics, the
cabin, minimums -- now have fields read on both sims. This walks every
shipped checklist file, and for each unchecked item whose challenge and
response name a state the sim can now prove, writes the check that
proves it. Items the sim still cannot prove (briefings, inspections,
FMC pages, transient tests like a mag-drop) keep their answer-only form.

Usage: python tools/add_check_vars.py [--write]
"""
import glob
import json
import re
import sys

FILES = "meridianatc/atc/checklists/**/*.json"


def _en(words):
    if isinstance(words, dict):
        return (words.get("en") or "").lower()
    return (words or "").lower()


_ON = re.compile(r"^(on|up|armed|set|closed|latched|engage|extend|deploy|"
                 r"charged?|rich|full( forward)?|forward|high|in|closed "
                 r"and (latched|armed)|both|auto|normal|green)\b\.?", )
_OFF = re.compile(r"^(off|down|retract|disarm|open|out|idle|cut.?off|"
                  r"feather|low|released?|stowed?)\b\.?")


def _onoff(response):
    """Whether the response names on or off, or None for neither."""
    r = _en(response)
    if _ON.match(r):
        return True
    if _OFF.match(r):
        return False
    return None


def _engine(challenge):
    """Which engine a per-engine item names: 1 left/first, 2 right/second."""
    c = _en(challenge)
    if re.search(r"\bright\b|\bnumber two\b|\bno\.? ?2\b|two\b$", c):
        return 2
    if re.search(r"one through four|one to four|all four|through four", c):
        return 4
    return 1


def _switch(challenge, response, base, per_engine=True):
    """An on/off switch item -> (var, eq, bool), or None."""
    want = _onoff(response)
    if want is None:
        return None
    eng = _engine(challenge) if per_engine else 1
    var = base if eng == 1 else f"eng{eng}_{base}"
    return {"var": var, "op": "eq", "value": want}


# (challenge regex, maker) -- first match wins. A maker answers the check
# dict, or None to leave the item answer-only (transients, aggregates
# the sim cannot prove, procedural states with no variable).
RULES = (
    # magnetos: the five-stop key. "both"->3, "off"->0, "start"->4,
    # a mag check ends at both.
    (r"magneto|mag\.? switch|ignition switch",
     lambda c, r: _magneto(c, r)),
    (r"ignition|igniter|auto.?ignition",
     lambda c, r: _switch(c, r, "ignition_on")),
    (r"starter|starter switch|starter key|starter button|start valve",
     lambda c, r: _starter(c, r)),
    (r"fuel pump|boost pump|fuel.?pump",
     lambda c, r: None if re.search(r"two seconds|then off", _en(r))
     else _switch(c, r, "fuel_pump_on")),
    (r"\balternator",
     lambda c, r: _switch(c, r, "alternator_on")),
    (r"generator",
     lambda c, r: _switch(c, r, "generator_on")),
    (r"oil press",
     lambda c, r: {"var": "eng2_oil_pressure_psi" if _engine(c) == 2
                   else "oil_pressure_psi", "op": "min", "value": 10}),
    (r"ammeter|loadmeter|load meter",
     lambda c, r: {"var": "battery_amps", "op": "min", "value": 0}
     if re.search(r"charg|positive|check", _en(r)) else None),
    (r"battery voltage|bus volt|voltage",
     lambda c, r: {"var": "main_bus_volts", "op": "min",
                   "value": _number(r) or 20}),
    (r"vacuum|suction",
     lambda c, r: {"var": "suction_inhg", "op": "min", "value": 3.5}),
    (r"\bdoor|\bexits?\b|windows and doors|seats, belts",
     lambda c, r: _doors(c, r)),
    (r"brake (accumulator|pressure|hydraulic)|accumulator",
     lambda c, r: {"var": "hydraulic_pressure", "op": "min",
                   "value": 500}),
    (r"\bbrakes?\b|wheel brake|toe brake",
     lambda c, r: _brakes(c, r)),
    (r"carb(uretor)? ?heat",
     lambda c, r: {"var": "carb_heat_pct", "op": "max", "value": 5}
     if _onoff(r) is False or re.search(r"check off|off", _en(r))
     else ({"var": "carb_heat_pct", "op": "min", "value": 50}
           if _onoff(r) is True else None)),
    (r"yaw ?damp",
     lambda c, r: _switch(c, r, "yaw_damper_on", per_engine=False)),
    (r"anti.?skid|antiskid",
     lambda c, r: _switch(c, r, "antiskid_on", per_engine=False)),
    (r"auto.?brake|autobrake",
     lambda c, r: _autobrake(r)),
    (r"speed.?brake|speedbrake|spoilers|lift dump|air brake",
     lambda c, r: _speedbrake(r)),
    (r"apu (gen|bleed|air)", lambda c, r: None),
    (r"apu generator|apu gen",
     lambda c, r: _switch(c, r, "apu_gen_on", per_engine=False)),
    (r"apu bleed|apu air",
     lambda c, r: _switch(c, r, "apu_bleed_on", per_engine=False)),
    (r"apu",
     lambda c, r: _apu(c, r)),
    (r"packs?\b",
     lambda c, r: _switch(c, r, "pack_left_on", per_engine=False)),
    (r"engine bleed|bleed air|eng.?bleed",
     lambda c, r: _switch(c, r, "bleed_air_on")),
    (r"engine anti.?ice|eng anti.?ice|inlet|nacelle anti|cowling anti",
     lambda c, r: _switch(c, r, "engine_anti_ice_on", per_engine=False)),
    (r"wing (anti.?ice|de.?ice)|boots|surface (de|anti)",
     lambda c, r: _switch(c, r, "wing_deice_on", per_engine=False)),
    (r"prop(eller)? (de.?ice|heat|anti)",
     lambda c, r: _switch(c, r, "prop_deice_on", per_engine=False)),
    (r"windshield (heat|de.?ice|anti)|window heat|windscreen",
     lambda c, r: _switch(c, r, "window_heat_on", per_engine=False)),
    (r"aoa|probe heat|angle of attack",
     lambda c, r: _switch(c, r, "aoa_heat_on", per_engine=False)),
    (r"anti.?ice|ice protection|de.?ice",
     lambda c, r: None),  # aggregates: "as required" is the weather's call
    (r"hydraulic pump|elec.*hydraulic|hyd pumps|utility pumps",
     lambda c, r: _switch(c, r, "hydraulic_pump_on", per_engine=False)),
    (r"hydraulic",
     lambda c, r: {"var": "hydraulic_pressure", "op": "min", "value": 1}
     if re.search(r"press|normal|arc|check|availab|green", _en(r))
     else _switch(c, r, "hydraulic_pump_on", per_engine=False)),
    (r"cabin alt",
     lambda c, r: _cabin(c, r)),
    (r"pressuri",
     lambda c, r: {"var": "cabin_altitude_goal_ft", "op": "min",
                   "value": 100}
     if re.search(r"set|landing alt|auto|check", _en(r)) else None),
    (r"minimums|decision height|decision altitude|dh\b",
     lambda c, r: {"var": "minimums_ft", "op": "min", "value": 1}
     if re.search(r"set|check|cross", _en(r)) else None),
    (r"oxygen|o2",
     lambda c, r: _switch(c, r, "oxygen_on", per_engine=False)),
    (r"wiper",
     lambda c, r: {"var": "wiper_pos", "op": "eq", "value": 0}
     if _onoff(r) is False else
     ({"var": "wiper_pos", "op": "min", "value": 1}
      if _onoff(r) is True else None)),
    (r"no.?smoking",
     lambda c, r: _switch(c, r, "no_smoking_sign", per_engine=False)),
    (r"logo ?light",
     lambda c, r: _switch(c, r, "logo_light", per_engine=False)),
    (r"wing (inspection )?light",
     lambda c, r: _switch(c, r, "wing_light", per_engine=False)),
    (r"recog(nition)? ?light",
     lambda c, r: _switch(c, r, "recog_light", per_engine=False)),
    (r"fuel flow",
     lambda c, r: {"var": "eng_fuel_flow_pph", "op": "min", "value": 10}
     if re.search(r"check|positive|normal|green|rising|verify", _en(r))
     else None),
    (r"rotor (rpm|speed)|\bnr\b|rotor tach",
     lambda c, r: {"var": "rotor_rpm_pct", "op": "min", "value": 90}),
    (r"rotor governor|governor",
     lambda c, r: {"var": "rotor_gov_target_pct", "op": "min",
                   "value": 80}),
    (r"throttles?|thrust (lever|control)",
     lambda c, r: {"var": "throttle_pct", "op": "max", "value": 10}
     if re.search(r"idle|closed?|cut", _en(r)) else None),
    (r"condition lever",
     lambda c, r: _condition(r)),
    (r"prop(eller)? lever|rpm lever",
     lambda c, r: {"var": "prop_pct", "op": "min", "value": 90}
     if re.search(r"full|forward|high|increase|max", _en(r))
     else ({"var": "prop_pct", "op": "max", "value": 10}
           if re.search(r"feather|low|back|pull", _en(r)) else None)),
    (r"mixture",
     lambda c, r: _mixture(r)),
)


def _magneto(challenge, response):
    c, r = _en(challenge), _en(response)
    eng = _engine(c)
    var = "magneto_pos" if eng == 1 else "eng2_magneto_pos"
    if re.search(r"rpm drop|drop ok|test|check left|check right|"
                 r"each side|right then left|left only|right only", c + " " + r):
        # The mag check ends where it began, on BOTH.
        return {"var": var, "op": "eq", "value": 3}
    if "start" in r or "start" in c:
        return {"var": var, "op": "in", "value": [3, 4]}
    if "both" in r or "both" in c:
        return {"var": var, "op": "eq", "value": 3}
    if _onoff(response) is False:
        return {"var": var, "op": "eq", "value": 0}
    if _onoff(response) is True:
        return {"var": var, "op": "min", "value": 1}
    return None


def _starter(challenge, response):
    r = _en(response)
    if re.search(r"diseng|off|released?|light.?out|motor off|stops? cranking", r):
        return {"var": "starter_active", "op": "eq", "value": False}
    if re.search(r"engage|push|hold|crank|start", r):
        return {"var": "starter_active", "op": "eq", "value": True}
    return None


def _doors(challenge, response):
    r = _en(response)
    if re.search(r"closed|latched|secure|check closed|armed", r):
        return {"var": "exits_open_pct", "op": "max", "value": 5}
    if re.search(r"\bopen\b", r):
        return {"var": "exits_open_pct", "op": "min", "value": 50}
    return None


def _brakes(challenge, response):
    c, r = _en(challenge), _en(response)
    if "parking" in c or "park" in c:
        return None  # parking_brake already checked by the first pass
    if re.search(r"as required|respond|responsive|steering", r + " " + c):
        return None
    if re.search(r"off|release", r):
        return {"var": "brake_left_pct", "op": "max", "value": 10}
    if re.search(r"check|hold|on|set|apply|test|pressure", r):
        return {"var": "brake_left_pct", "op": "min", "value": 10}
    return None


def _autobrake(response):
    r = _en(response)
    if _onoff(response) is False or re.search(r"\boff\b", r):
        return {"var": "autobrake_pos", "op": "eq", "value": 0}
    # Armed, set, RTO and the numbered settings are all a non-zero dial.
    if re.search(r"arm|set|max|rto|[1-5]|as required|med", r):
        return {"var": "autobrake_pos", "op": "min", "value": 1}
    return None


def _speedbrake(response):
    r = _en(response)
    if re.search(r"retract|stow|down|off|in\b", r):
        return {"var": "speedbrake_pct", "op": "max", "value": 10}
    if re.search(r"deploy|extend|up|arm|out", r):
        return {"var": "speedbrake_pct", "op": "min", "value": 50}
    return None


def _apu(challenge, response):
    c, r = _en(challenge), _en(response)
    if re.search(r"rpm|one hundred percent|100%|avail|wait for|running|"
                 r"started", r):
        return {"var": "apu_pct", "op": "min", "value": 80}
    want = _onoff(response)
    if want is None:
        want = True if re.search(r"start", r + " " + c) else None
    if want is None:
        return None
    if not want:
        return {"var": "apu_pct", "op": "max", "value": 5}
    return {"var": "apu_on", "op": "eq", "value": True}


_NUM_WORDS = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
              "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10}


def _number(text):
    """The digits or small number-words a response names, or None."""
    m = re.search(r"(\d[\d,.]*)", _en(text))
    if m:
        return float(m.group(1).replace(",", "").replace(".", ""))
    words = _en(text).split()
    if "thousand" in words:
        i = words.index("thousand")
        if i > 0 and words[i - 1] in _NUM_WORDS:
            return _NUM_WORDS[words[i - 1]] * 1000.0
    if words and words[0] in _NUM_WORDS:
        return float(_NUM_WORDS[words[0]])
    return None


def _cabin(challenge, response):
    r = _en(response)
    if re.search(r"below|under|less than|max", r):
        n = _number(r)
        return ({"var": "cabin_altitude_ft", "op": "max", "value": n}
                if n else {"var": "cabin_altitude_ft", "op": "max",
                           "value": 8000})
    if re.search(r"set|check|landing", r):
        return {"var": "cabin_altitude_goal_ft", "op": "min", "value": 100}
    return None


def _condition(response):
    r = _en(response)
    if re.search(r"high|full|forward", r):
        return {"var": "condition_pct", "op": "min", "value": 90}
    if re.search(r"low|idle", r):
        return {"var": "condition_pct", "op": "min", "value": 40}
    if re.search(r"cut|off|feather", r):
        return {"var": "condition_pct", "op": "max", "value": 10}
    return None


def _mixture(response):
    r = _en(response)
    if re.search(r"rich|full|forward|in\b", r):
        return {"var": "mixture_pct", "op": "min", "value": 90}
    if re.search(r"lean", r):
        # Leaned is just shy of full rich; the sim cannot prove the EGT
        # the pilot leaned to, only that the lever came back some.
        return {"var": "mixture_pct", "op": "max", "value": 95}
    if re.search(r"cut|off|back|pull|idle cut", r):
        return {"var": "mixture_pct", "op": "max", "value": 10}
    return None


def main(write=False):
    touched, added = 0, 0
    for path in sorted(glob.glob(FILES, recursive=True)):
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
        changed = False
        for cl in (data.get("checklists") or {}).values():
            if not isinstance(cl, dict):
                continue
            for item in cl.get("items") or []:
                if not isinstance(item, dict) or item.get("check"):
                    continue
                ch, rs = item.get("challenge"), item.get("response")
                text = _en(ch)
                for pat, maker in RULES:
                    if re.search(pat, text):
                        check = maker(ch, rs)
                        if check:
                            item["check"] = check
                            changed = True
                            added += 1
                        break
        if changed:
            touched += 1
            if write:
                with open(path, "w", encoding="utf-8") as fh:
                    json.dump(data, fh, ensure_ascii=False, indent=2)
                    fh.write("\n")
    print(f"{added} checks added across {touched} files")


if __name__ == "__main__":
    main("--write" in sys.argv)
