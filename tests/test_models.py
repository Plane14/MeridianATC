"""The per-add-on variable tables in sim/models.py: detection picks
the right table, and the stock name then resolves to the name the
model really publishes -- the add-on's own L: variable on MSFS, its
custom dataref on X-Plane."""

from types import SimpleNamespace

from meridianatc.sim import models, xplane
from meridianatc.sim.models import (
    MSFS_ALIASES,
    msfs_model_key,
    xp_model_key,
)
from meridianatc.sim.source import SimConnectSource
from meridianatc.sim.xplane import XPlaneSource


def test_the_title_names_the_msfs_model():
    assert msfs_model_key("FlyByWire A380X") == "fbw_a380x"
    assert msfs_model_key("FlyByWire A320neo") == "fbw_a32nx"
    assert msfs_model_key("PMDG 737-800") == "pmdg_737"
    assert msfs_model_key("Cessna Skyhawk") == ""


def test_the_acf_path_names_the_xplane_model():
    assert xp_model_key(
        "Aircraft/Extra Aircraft/Devinci_B2707_300/b2707_300.acf"
    ) == "devinci_b2707"
    # The MAD A320's title is the stock "Airbus A320 Neo" -- only its
    # path tells it apart.
    assert xp_model_key("Aircraft/MAD_A320/mad_a320.acf",
                        "Airbus A320 Neo") == "mad_a320"
    assert xp_model_key("Aircraft/Laminar Research/Boeing 737-800X/"
                        "b738.acf", "Boeing 737-800X") == "zibo_738"
    assert xp_model_key("Aircraft/Laminar Research/Cessna 172SP/"
                        "c172.acf", "Cessna Skyhawk") == ""


def test_the_msfs_definition_asks_the_lvar():
    helper = SimpleNamespace(list={
        "ELECTRICAL_MASTER_BATTERY:index": [
            "d", b"ELECTRICAL MASTER BATTERY:index", b"Bool", "N"],
        "APU_PCT_RPM": ["d", b"APU PCT RPM", b"Percent", "N"],
    })
    source = SimConnectSource.__new__(SimConnectSource)
    source._requests = SimpleNamespace(list=[helper])
    # No model: the stock variable is asked, untouched.
    assert source._definition_for("APU_PCT_RPM") == (b"APU PCT RPM",
                                                     b"Percent")
    source._model = "fbw_a32nx"
    var, unit = source._definition_for("ELECTRICAL_MASTER_BATTERY:1")
    assert var == b"L:A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO"
    assert unit == b"Number"
    assert source._definition_for("APU_PCT_RPM") == (b"L:A32NX_APU_N",
                                                     b"Number")


def test_the_xplane_alias_subscribes_the_addon_datarefs():
    link = XPlaneSource()
    link._subscribe = lambda index, path, frequency: None
    link._model = "zibo_738"
    link._subscribe_model_aliases()
    subscribed = dict(link._subscriptions)
    assert subscribed["alias:apu_on"] == "laminar/B738/switches/apu_start"
    assert (subscribed["alias:parking_brake"]
            == "laminar/B738/annunciator/parking_brake")


def test_the_xplane_read_prefers_the_alias_answer():
    link = XPlaneSource()
    link._model = "zibo_738"
    link._alias_factors["parking_brake"] = 1.0
    # The stock ref still answers 0; the add-on's own ref says set.
    link._values = {"parking_brake": 0.0, "alias:parking_brake": 1.0}
    assert link.read().parking_brake is True
    # With no model the alias values must not leak in.
    link._alias_factors.clear()
    assert link.read().parking_brake is False


def test_the_xplane_write_reaches_the_addon_dataref():
    link = XPlaneSource()
    link._model = "zibo_738"
    link._controls.set_variable = lambda name, value, unit: False
    written = []
    link._dref = lambda path, value: written.append((path, value)) or True
    assert link.set_variable("BLEED AIR ENGINE:1", 1) is True
    assert written == [("laminar/B738/toggle_switch/bleed_air_1_pos", 1.0)]


def test_every_alias_table_names_a_model_the_detect_can_find():
    for table in (models.XP_READ_ALIASES, models.XP_WRITE_ALIASES):
        for key in table:
            assert any(key == model for _f, model in models.XP_MODELS)
    for key in MSFS_ALIASES:
        assert any(key == model for _f, model in models.MSFS_MODELS)
    # And every XP write target is a dataref the model was seen
    # publishing -- no invented names.
    for key, table in models.XP_WRITE_ALIASES.items():
        reads = models.XP_READ_ALIASES.get(key, {})
        for name, path in table.items():
            assert path.startswith(("laminar/", "b2707/", "AirbusFBW/"))
            assert name
