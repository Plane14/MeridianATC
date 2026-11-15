

def test_icao_wind_carries_its_units_and_faa_keeps_at():
    from meridianatc.atc.phraseology import Phraseology, Weather

    wx = Weather(wind_dir=270, wind_kt=12, gust_kt=22)
    assert Phraseology("icao").wind(wx) == (
        "wind two seven zero degrees one two knots gusting two two")
    assert Phraseology("faa").wind(wx) == (
        "wind two seven zero at one two gusting two two")
    assert Phraseology("icao").wind(Weather(wind_kt=0)) == "wind calm"
