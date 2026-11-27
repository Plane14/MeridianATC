"""MSFS 2024 stock-aircraft checklist matching.

The stock titles are the Microsoft Flight Simulator 2024 edition aircraft
list. A type file is evidence only when it names the aircraft, not an
avionics suite, a manufacturer, another variant, or a vendor edition the
pilot is not flying.
"""

from meridianatc.atc import checklist_files
from meridianatc.sim.source import type_from_title


def names(title, code="", simulator="simconnect"):
    return {p.name for p in checklist_files._add_ons(title, code, simulator)}


def test_msfs_stock_type_files_match_the_aircraft_they_document():
    cases = {
        ("Airbus A310-300", "A310"): "airbus-a310.json",
        ("Airbus A320neo", "A20N"): "a320-neo.json",
        ("Airbus A400M Atlas", "A400"): "a400m.json",
        ("Airbus Helicopters H125", "AS50"): "asobo_h125.json",
        ("Beechcraft Bonanza G36", "BE36"): "bonanza-g36.json",
        ("Beechcraft King Air 350i", "B350"): "king-air-350i.json",
        ("Bell 407", "B407"): "bell-407.json",
        ("Boeing 737 MAX 8", "B38M"): "boeing-737-max.json",
        ("Boeing 747-8I", "B748"): "boeing-747-8i.json",
        ("Cessna 152", "C152"): "cessna-152.json",
        ("Cessna 172 Skyhawk G1000", "C172"): "cessna-172.json",
        ("Cessna 208B Grand Caravan EX", "C208"): "cessna-208-caravan.json",
        ("Daher TBM 930", "TBM9"): "tbm-930.json",
        ("De Havilland Canada DHC-2 Beaver", ""): "dhc-2-beaver.json",
        ("De Havilland Canada DHC-6 Twin Otter", ""): "dhc-6-twin-otter.json",
        ("Diamond DA40 NG", "DA40"): "da40-ng.json",
        ("Douglas DC-3", ""): "douglas-dc-3.json",
        ("Pilatus PC-12 NGX", "PC12"): "pilatus-pc-12.json",
    }
    for (title, code), wanted in cases.items():
        assert wanted in names(title, code), (title, names(title, code))


def test_msfs_stock_aircraft_do_not_inherit_cross_type_or_vendor_cards():
    cases = [
        ("Cessna 172 Skyhawk G1000", "C172",
         {"da40-ng.json", "xplane-cessna-172.json"}),
        ("Pilatus PC-6 Porter", "PC6T", {"pilatus-pc-12.json"}),
        ("Bell 407", "B407", {"nemeth_bell407.json"}),
        ("Airbus A330-300", "A333", {"xplane-airbus-a330.json"}),
        ("Cirrus Vision SF50", "SF50", {"xplane-vision-sf50.json"}),
    ]
    for title, code, unwanted in cases:
        assert not names(title, code) & unwanted, (
            title, names(title, code) & unwanted)


def test_simulator_specific_files_do_not_cross_between_the_sims():
    assert "asobo_h125.json" in names(
        "Airbus Helicopters H125", "AS50", "simconnect")
    assert "asobo_h125.json" not in names(
        "Airbus Helicopters H125", "AS50", "xplane")
    assert "xplane-cessna-172.json" in names(
        "Cessna 172 Skyhawk", "C172", "xplane")
    assert "xplane-cessna-172.json" not in names(
        "Cessna 172 Skyhawk G1000", "C172", "simconnect")


def test_msfs_vendor_rotorcraft_requires_the_vendor_title():
    assert "nemeth_bell407.json" not in names("Bell 407", "B407")
    assert "nemeth_bell407.json" in names(
        "Nemeth Designs Bell 407", "B407")


def test_msfs_stock_titles_infer_their_own_type_designators():
    cases = {
        "CubCrafters NXCub": "CC19",
        "CubCrafters XCub": "CC19",
        "Zlin Savage Cub": "SAVG",
        "Bell 407": "B407",
        "Robinson R66": "R66",
        "Airbus A330-743L Beluga XL": "A337",
        "Boeing F/A-18E": "F18S",
        "Archer Midnight": "MDNT",
        "Joby Aviation S4": "JAS4",
        "Airbus Helicopters H225": "EC25",
        "Boeing CH47D Chinook": "H47",
        "Bell 47J Ranger": "B47J",
        "Westland Scout": "SCOU",
        "Westland Wasp": "WASP",
        "Beechcraft C90 GTX King Air": "BE9L",
    }
    for title, code in cases.items():
        assert type_from_title(title) == code

    # The CC19/SAVG correction also prevents the X-Plane PA-18 POH from
    # being pulled onto MSFS's modern cub types through the inferred code.
    assert "xplane-piper-pa18.json" not in names(
        "CubCrafters NXCub", type_from_title("CubCrafters NXCub"))
    assert "xplane-piper-pa18.json" not in names(
        "Zlin Savage Cub", type_from_title("Zlin Savage Cub"))


def test_msfs_stock_aircraft_do_not_inherit_study_addon_no_check_cards():
    """A stock 737/CRJ/A310 is not PMDG, Aerosoft or iniBuilds merely
    because the model number agrees; those files also disable checks."""
    assert "pmdg-737-800.json" not in names("Boeing 737-800", "B738")
    assert "pmdg_737.json" not in names("Boeing 737-800", "B738")
    assert "aerosoft_crj.json" not in names("CRJ 700", "CRJ7")
    assert "inibuilds_a310.json" not in names("Airbus A310-300", "A310")

    assert "pmdg-737-800.json" in names("PMDG 737-800", "B738")
    assert "aerosoft_crj.json" in names("Aerosoft CRJ 700", "CRJ7")
    assert "inibuilds_a310.json" in names("iniBuilds A310-300", "A310")


def test_vendor_files_documenting_multiple_vendors_accept_either_title():
    assert "a340-600.json" in names("ToLiss A340-600", "A346")
    assert "a340-600.json" in names("Aerosoft A340-600", "A346")
    assert "a340-600.json" not in names("Airbus A340-600", "A346")


def test_msfs_new_types_classify_before_checklist_selection():
    """The inferred designator feeds the class file as well as the type
    match: modern cub types are piston singles, while the MSFS 2024 eVTOL
    types are powered lift rather than imaginary piston singles."""
    from meridianatc.atc import aircraft_class as ac

    assert ac.classify("CC19").name == ac.PISTON_SINGLE
    assert ac.classify("SAVG").name == ac.PISTON_SINGLE
    assert ac.classify("B407").name == ac.HELICOPTER
    assert ac.classify("R66").name == ac.HELICOPTER
    assert ac.classify("S64").name == ac.HELICOPTER
    assert ac.classify("EC25").name == ac.HELICOPTER
    assert ac.classify("H47").name == ac.HELICOPTER
    assert ac.classify("B47J").name == ac.HELICOPTER
    assert ac.classify("SCOU").name == ac.HELICOPTER
    assert ac.classify("WASP").name == ac.HELICOPTER
    assert ac.classify("MDNT").name == ac.POWERED_LIFT
    assert ac.classify("JAS4").name == ac.POWERED_LIFT
    assert ac.classify(title="Volocopter VoloCity").name == ac.POWERED_LIFT
    assert ac.classify(title="Jetson One").name == ac.POWERED_LIFT


def test_msfs_deluxe_and_premium_titles_do_not_cross_variants_or_sims():
    assert "king-air-350i.json" not in names(
        "Beechcraft C90 GTX King Air", type_from_title(
            "Beechcraft C90 GTX King Air"))
    assert "xplane-kingair-c90.json" not in names(
        "Beechcraft C90 GTX King Air", type_from_title(
            "Beechcraft C90 GTX King Air"))

    assert "cirrus-sr22.json" in names("Cirrus SR22", "SR22")
    assert "xplane-cirrus-sr22.json" not in names("Cirrus SR22", "SR22")
    assert "citation-longitude.json" in names(
        "Cessna Citation Longitude", "C700")
    assert "boeing-787-9.json" not in names("Boeing 787-10", "B78X")
    assert "pmdg-747-400.json" not in names("Boeing 747-400", "B744")
    assert "xplane-boeing-747-400.json" not in names(
        "Boeing 747-400", "B744")
    assert "atr-72-600.json" not in names("ATR 42-600", "AT46")


def test_load_uses_the_simulator_when_merging_type_books():
    msfs = checklist_files.load(
        "piston_single", "C172", title="Cessna 172 Skyhawk G1000",
        simulator="simconnect")
    xplane = checklist_files.load(
        "piston_single", "C172", title="Cessna 172 Skyhawk",
        simulator="xplane")
    assert not any("xplane-cessna-172.json" in path
                   for path in msfs.files)
    assert any("xplane-cessna-172.json" in path for path in xplane.files)
