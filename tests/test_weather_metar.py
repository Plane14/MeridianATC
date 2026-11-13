"""Raw METAR present-weather parsing, incl. groups the scanner used to miss
or invent (station idents are legal weather tokens by shape alone)."""

from __future__ import annotations

import pytest

from meridianatc.atc.trafficinfo import runway_wet
from meridianatc.weather.metar import parse_metar, weather_tokens


@pytest.mark.parametrize("group", [
    "-SHRA", "+TSRA", "FZFG", "BR", "RA", "SHSN", "BLSN", "MIFG", "-DZ",
    "VCSH", "VCTS", "VCFG", "VCBLSN", "VCSHRA", "RESHRA", "RETS", "REFZRA",
    "-RASN", "+RAGR", "DZRA", "UP", "TS",
])
def test_present_weather_group_kept(group):
    assert parse_metar(f"KJFK 121851Z 18005KT 10SM {group} "
                       "SCT020 12/08 A2992").present == group


def test_station_ident_is_not_weather():
    for ident in ("SABE", "RABW", "DURA", "FURA", "SADF"):
        wx = parse_metar(f"{ident} 121851Z 18005KT 10SM SCT020 12/08 A2992")
        assert wx.present == "", ident


def test_compound_phenomena_not_truncated():
    wx = parse_metar("KJFK 121851Z 18005KT 10SM -RASN SCT020 12/08 A2992")
    assert wx.present == "-RASN"


def test_several_groups_in_order():
    wx = parse_metar("SADF 121850Z 23015G25KT 3000 -RA BR BKN010 17/16 Q1013")
    assert wx.present == "-RA BR"


def test_nsw_and_non_weather_tokens_ignored():
    wx = parse_metar("KJFK 121851Z 18005KT 10SM NSW SKC 12/08 A2992")
    assert wx.present == ""


def test_weather_only_after_wind():
    # A weather-shaped word ahead of the wind group is not observed weather.
    assert weather_tokens("FURA 121851Z 18005KT CAVOK") == []
    assert weather_tokens("121851Z 18005KT CAVOK") == []


def test_runway_wet():
    def wet(metar):
        return runway_wet(parse_metar(metar))
    assert wet("KJFK 121851Z 18005KT 10SM -SHRA SCT020 12/08 A2992")
    assert wet("KJFK 121851Z 18005KT 10SM FZRA SCT020 12/08 A2992")
    # Recent or vicinity-only precipitation does not make the runway wet.
    assert not wet("KJFK 121851Z 18005KT 10SM RESHRA SCT020 12/08 A2992")
    assert not wet("KJFK 121851Z 18005KT 10SM VCSHRA SCT020 12/08 A2992")
    # A precipitation-shaped station ident is not falling weather.
    assert not wet("RAGR 121851Z 18005KT 10SM SCT020 12/08 A2992")


def test_wind_units_and_variable():
    wx = parse_metar("KJFK 121851Z VRB03MPS 9999 FEW020 12/08 Q1013")
    assert wx.wind_kt == pytest.approx(3 * 1.94384)
    assert wx.variable
    wx = parse_metar("KJFK 121851Z 18010G22KMH 9999 FEW020 12/08 Q1013")
    assert wx.wind_kt == pytest.approx(10 * 0.539957)
    assert wx.gust_kt == pytest.approx(22 * 0.539957)


def test_merges_keep_present():
    from meridianatc.weather.metar import WeatherSample, merge_weather

    real = WeatherSample(parse_metar(
        "KJFK 121851Z 18005KT 10SM -SHRA BKN020 12/08 A2992"), "metar")
    merged = merge_weather(None, real)
    assert merged.weather.present == "-SHRA"
