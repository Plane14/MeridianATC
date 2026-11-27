"""X-Plane 12's native fleet: the POH profiles, checklists and speeds.

Each native aircraft's POH gives its rotation, climb, cruise, approach
and touchdown figures -- written down in ``autofly/*.json`` profiles that
match the simulator's title, its checklist set in
``checklists/types/xplane-*.json``, and its type's own taxi speed for
placed traffic.
"""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from meridianatc.atc import autofly, checklist_files
from meridianatc.atc.flying import AIRLINER, LIGHT
from meridianatc.atc.vspeeds import rotate_kt

# The X-Plane 12 hangar, a title fragment the simulator reports, the
# checklist class it files as, and the rotation the POH gives it.
NATIVE = [
    ("A330", "Airbus A330-300", "airliner", 150),
    ("BE58", "Beechcraft Baron 58", "piston_twin", 85),
    ("BE9L", "King Air C90B", "turboprop_twin", 90),
    ("B738", "Boeing 737-800", "airliner", 140),
    ("B744", "Boeing 747-400", "airliner", 170),
    ("C172", "Cessna 172 Skyhawk", "piston_single", 55),
    ("C750", "Cessna Citation X", "business_jet", 145),
    ("SR22", "Cirrus SR22", "piston_single", 73),
    ("SF50", "Cirrus Vision Jet SF50", "business_jet", 90),
    ("F14", "Grumman F-14 Tomcat", "airliner", 150),
    ("EVOT", "Lancair Evolution", "turboprop_single", 77),
    ("MD82", "McDonnell Douglas MD-82", "airliner", 130),
    ("PA18", "Piper PA-18 Super Cub", "piston_single", 40),
    ("RV10", "Van's RV-10", "piston_single", 65),
]


@pytest.mark.parametrize("code,title,klass,rotate", NATIVE)
def test_poh_profile_overrides_the_class_figures(code, title, klass, rotate):
    perf = autofly.override(code, title,
                            LIGHT if klass != "airliner" else AIRLINER)
    assert perf.rotate_kt == pytest.approx(rotate)


@pytest.mark.parametrize("code,title,klass,rotate", NATIVE)
def test_poh_checklists_load_for_the_native_title(code, title, klass, rotate):
    book = checklist_files.load(klass, code, title=title)
    assert book.checklists, f"{title}: no checklists after the merge"
    assert not book.problems, f"{title}: {book.problems}"
    xplane = [f for f in book.files if "xplane-" in Path(f).name]
    if code == "F14":
        # The Tomcat has no class file that fits; the type file stands alone.
        assert xplane
    assert xplane, f"{title}: the POH checklist file was not matched"


def test_r22_poh_checklists_load():
    book = checklist_files.load("helicopter", "R22",
                                title="Robinson R22 Beta II")
    assert book.checklists
    assert not book.problems
    assert any("xplane-" in Path(f).name for f in book.files)


# The vspeeds table's own entries for the native fleet -- the codes
# without an older sourced entry, whose figures come from the X-Plane
# manuals themselves. (B738/SR22/A330 keep their older sourced table
# values; the autofly profile overrides them in the performance stack.)
VSPEEDS_NATIVE = [
    ("BE58", 85), ("PA18", 40), ("RV10", 65), ("EVOT", 77),
    ("BE9L", 90), ("SF50", 90), ("C750", 145), ("MD82", 130),
    ("B744", 170), ("F14", 150),
]


@pytest.mark.parametrize("code,rotate", VSPEEDS_NATIVE)
def test_native_rotation_figures_in_the_vspeed_table(code, rotate):
    assert rotate_kt(code) == pytest.approx(rotate)


def test_placed_traffic_taxis_at_the_types_own_speed():
    from meridianatc.atc.traffic._shared import Flight
    from meridianatc.atc.traffic.x_the_field import _The_field

    field = _The_field.__new__(_The_field)
    field.field = SimpleNamespace(latitude=51.15, longitude=-0.19,
                                  elevation_ft=200.0)
    field.flights = []
    field._threshold = lambda runway: None

    def placed(performance):
        flight = Flight("TEST123", "C172", SimpleNamespace(), "en", None,
                        False, runway="09", phase="taxi")
        flight.performance = performance
        field._place(flight)
        return flight.ground_speed_kt

    # A Cessna creeps where a heavy rolls: the placed speed is the
    # type's own figure, not one number for the whole aerodrome.
    assert placed(LIGHT) == pytest.approx(LIGHT.taxi_kt)
    assert placed(AIRLINER) == pytest.approx(AIRLINER.taxi_kt)
    assert LIGHT.taxi_kt < AIRLINER.taxi_kt


def test_every_autofly_profile_names_a_title():
    # A profile that matches no title is a profile that never applies.
    for path in autofly.DATA_DIR.glob("*.json"):
        data = json.loads(path.read_text(encoding="utf-8"))
        assert data.get("titleMatches"), path.name
        assert all(m.strip() for m in data["titleMatches"]), path.name


def test_stock_aircraft_do_not_inherit_unrelated_type_checklists():
    """The title/type matcher must not mix whole manufacturers or vendors:
    a stock Caravan is not a Citation, and an H125 is not an Airbus A320."""
    cases = [
        ("Cessna 208 Caravan", "C208", "cessna-208-caravan.json",
         {"citation-x.json", "cessna-152.json", "cessna-172.json",
          "blacksquare-caravan.json"}),
        ("Airbus H125", "AS50", "h125.json",
         {"a320-neo.json", "a330-900neo.json", "king-air-350i.json"}),
        ("Beechcraft Baron 58", "BE58", "xplane-baron-58.json",
         {"blacksquare-baron.json"}),
        ("Cessna Citation X", "C750", "xplane-citation-x.json",
         {"citation-longitude.json"}),
    ]
    for title, code, wanted, unwanted in cases:
        names = {p.name for p in checklist_files._add_ons(title, code)}
        assert wanted in names, (title, names)
        assert not names & unwanted, (title, names & unwanted)


def test_vendor_editions_still_match_their_own_titles():
    names = {p.name for p in checklist_files._add_ons(
        "Black Square Baron 58", "BE58")}
    assert "blacksquare-baron.json" in names
    names = {p.name for p in checklist_files._add_ons(
        "FlightFX Citation X", "C750")}
    assert "citation-x.json" in names
    assert "hawker-800xp.json" not in names
