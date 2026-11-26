from meridianatc.navdata.build import _clean_facility


def test_cld_is_noise_in_delivery_facility_names():
    assert _clean_facility("CLD", "Lisbon", "DEL") == "Lisbon"
