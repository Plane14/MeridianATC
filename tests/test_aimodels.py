"""The installed-model catalogue: every aircraft title the simulator
can create, by type, airline and rotorcraft."""

from __future__ import annotations

from meridianatc.integrations import fsltl
from meridianatc.integrations.aimodels import (
    Catalogue,
    is_helicopter_type,
    type_of_title,
)


def pairs(*titles):
    return [(t, "") for t in titles]


def test_type_of_title_reads_injector_and_named_models():
    assert type_of_title("TT:ATCCOM.AC_MODEL_B738.0.text") == "B738"
    assert type_of_title("FSLTL_B738_EZY") == "B738"
    assert type_of_title("Asobo PassiveAircraft A320 NEO") == "A20N"
    assert type_of_title("Boeing 777-300ER Asobo") == "B77W"
    assert type_of_title("Cessna Skyhawk G1000 Asobo") == "C172"
    assert type_of_title("") == ""
    assert type_of_title("FSDT_Pushback_03") == ""


def test_catalogue_finds_every_installed_type_not_just_the_table():
    cat = Catalogue.from_pairs(pairs(
        "Asobo PassiveAircraft A320 NEO",
        "FSLTL_B738_EZY",
        "FSLTL_A339_DLH",
        "Fenix A320 CFM",
    ))
    # Types the written-down table never named are answerable now.
    assert cat.has_type("A339")
    assert cat.has_type("B738")
    assert cat.has_type("A320")


def test_airlines_model_comes_first_then_plain_then_no_other_airline():
    cat = Catalogue.from_pairs(pairs(
        "FSLTL_B738_EZY",
        "FSLTL_B738_KLM",
        "FSLTL_B738_ZZZZ",
        "Asobo PassiveAircraft B737-800",
    ))
    titles = cat.titles_for("B738", "KLM")
    assert titles[0] == "FSLTL_B738_KLM"
    assert "FSLTL_B738_EZY" not in titles
    assert cat.other_titles_for("B738", "KLM") == ["FSLTL_B738_EZY"]
    # No airline asked: painted models answer to nobody's livery call.
    plain = cat.titles_for("B738")
    assert "FSLTL_B738_KLM" not in plain


def test_ai_models_sort_before_flyable():
    cat = Catalogue.from_pairs(pairs(
        "Fenix A320 CFM",                       # flyable, expensive
        "Asobo PassiveAircraft A320 CEO",       # the AI one
    ))
    assert cat.titles_for("A320")[0] == "Asobo PassiveAircraft A320 CEO"


def test_rotorcraft_are_their_own_kind():
    cat = Catalogue.from_pairs(pairs(
        "FSLTL_B738_EZY",
        "Asobo PassiveAircraft H160",
        "Bell 407 Helicopter",
    ))
    assert is_helicopter_type("B407")
    assert is_helicopter_type("H160")
    assert not is_helicopter_type("B738")
    helis = cat.heli_titles()
    assert len(helis) == 2
    # And a rotorcraft type is answerable by a real model, not an A320.
    assert cat.titles_for("B407")


# -- every installed .vmr, not only FSLTL's -----------------------------------

VMR = ('<?xml version="1.0"?><ModelMatchRules>'
       '<ModelMatchRule CallsignPrefix="{airline}" TypeCode="{type}" '
       'ModelName="{model}"/></ModelMatchRules>')


def _usercfg(tmp_path, packages):
    appdata = tmp_path / "appdata"
    cfg = appdata / "Microsoft Flight Simulator 2024"
    cfg.mkdir(parents=True)
    (cfg / "UserCfg.opt").write_text(
        f'InstalledPackagesPath "{packages}"', encoding="utf-8")
    return {"APPDATA": str(appdata), "LOCALAPPDATA": ""}


def test_every_pack_vmr_is_read(tmp_path):
    packages = tmp_path / "packages"
    fsltl_dir = packages / "Community" / "fsltl-traffic-base"
    aig_dir = packages / "Community" / "aig-aitraffic-oci"
    fsltl_dir.mkdir(parents=True)
    aig_dir.mkdir(parents=True)
    (fsltl_dir / "FSLTL_Rules.vmr").write_text(
        VMR.format(airline="EZY", type="B738", model="FSLTL_B738_EZY"),
        encoding="utf-8")
    (aig_dir / "AIGAIM.vmr").write_text(
        VMR.format(airline="DLH", type="A339", model="AIG_A339_DLH"),
        encoding="utf-8")
    environ = _usercfg(tmp_path, packages)

    found = fsltl.find_rule_files(environ)
    assert len(found) == 2
    assert found[0].name == "FSLTL_Rules.vmr"   # FSLTL's is read first

    liveries = fsltl.load_installed(environ)
    assert liveries.models("EZY", "B738") == ("FSLTL_B738_EZY",)
    # AIG's table answers for its airlines too, not only FSLTL's.
    assert liveries.models("DLH", "A339") == ("AIG_A339_DLH",)
    assert fsltl.traffic_source(
        type("T", (), {"source": "auto", "fsltl_rules": ""})(),
        environ) == "fsltl"


def test_no_pack_means_no_rules_and_meridianatc_source(tmp_path):
    packages = tmp_path / "packages"
    (packages / "Community" / "scenery").mkdir(parents=True)
    environ = _usercfg(tmp_path, packages)
    assert fsltl.find_rule_files(environ) == []
    assert fsltl.load_installed(environ) is None
    assert fsltl.traffic_source(
        type("T", (), {"source": "auto", "fsltl_rules": ""})(),
        environ) == "meridianatc"
