"""The unusual add-on types, and the water runways only they can use."""
from types import SimpleNamespace

from meridianatc.atc import aircraft_class as ac
from meridianatc.atc import vspeeds
from meridianatc.sim.state import floats_said


class TestUnusualTypes:
    def test_the_spyplanes_are_jets(self):
        assert ac.classify("U2").name == ac.BUSINESS_JET
        assert ac.classify("B57").name == ac.BUSINESS_JET

    def test_the_ssts_are_airliners(self):
        assert ac.classify("CONC").name == ac.AIRLINER
        assert ac.classify("B2707").name == ac.AIRLINER

    def test_the_flying_boats_are_amphibian_turboprops(self):
        for code in ("US1", "US2"):
            found = ac.classify(code)
            assert found.name == ac.TURBOPROP_TWIN
            assert ac.amphibious(code)
        # And the rest of the amphibian list.
        for code in ("U16", "PBY", "PBYA", "G73", "G73T", "LAKE", "SEAB"):
            assert ac.amphibious(code), code

    def test_a_landplane_is_not_amphibious(self):
        assert not ac.amphibious("C172")
        assert not ac.amphibious("B738")

    def test_the_types_have_speeds(self):
        for code in ("U2", "B57", "CONC", "B2707", "US1", "US2", "LAKE",
                     "V22", "A609", "HAR", "AV8B", "F35"):
            speeds = vspeeds.table_speeds(code)
            assert speeds.vr > 0, code


def _water_runway(length_ft=4000):
    return SimpleNamespace(length_ft=length_ft, surface="WAT",
                           is_hard=False)


def _paved_runway(length_ft=4000):
    return SimpleNamespace(length_ft=length_ft, surface="ASP",
                           is_hard=True)


class TestWaterRunways:
    def test_water_serves_only_the_amphibious(self):
        assert ac.runway_usable(_water_runway(), "US2")
        assert ac.runway_usable(_water_runway(), "LAKE")
        assert not ac.runway_usable(_water_runway(), "C172")
        assert not ac.runway_usable(_water_runway(), "B738")

    def test_a_seaplane_base_serves_only_the_amphibious(self):
        base = SimpleNamespace(type="seaplane_base")
        water = [_water_runway()]
        assert ac.airport_usable(base, water, "US2")
        assert ac.airport_usable(base, water, "A5")
        assert not ac.airport_usable(base, water, "C172")
        assert not ac.airport_usable(base, water, "PA28")

    def test_an_amphibian_still_uses_a_paved_runway(self):
        assert ac.runway_usable(_paved_runway(), "US2")

    def test_a_short_water_lane_limits_the_boat(self):
        # A US-2 plans on 3,000 ft; a 2,000 ft water lane is too small.
        assert not ac.runway_usable(_water_runway(2000), "US2")
        assert ac.runway_usable(_water_runway(2000), "LAKE")

    def test_the_ssts_want_a_long_runway(self):
        assert ac.min_runway_ft("CONC") >= 9000
        assert ac.min_runway_ft("B2707") >= 10000
        airport = SimpleNamespace(type="small_airport")
        short = [_paved_runway(8000)]
        assert not ac.airport_usable(airport, short, "CONC")


class TestFloatVariants:
    """A landplane on floats reports the landplane's code; the sim's own
    word for the floats -- on_floats -- is what lets it take the water."""

    def test_the_title_gives_the_floats_away(self):
        assert floats_said("Cessna 172 Skyhawk Floats")
        assert floats_said("de Havilland DHC-2 Beaver Amphibian")
        assert floats_said("Searey LSX Seaplane")
        assert not floats_said("Cessna 172 Skyhawk")
        assert not floats_said("")

    def test_floats_make_a_landplane_water_capable(self):
        assert ac.amphibious("C172", on_floats=True)
        assert not ac.amphibious("C172")
        assert ac.runway_usable(_water_runway(), "C172", on_floats=True)
        assert not ac.runway_usable(_water_runway(), "C172")

    def test_a_seaplane_base_serves_the_float_variant(self):
        base = SimpleNamespace(type="seaplane_base")
        water = [_water_runway()]
        assert ac.airport_usable(base, water, "C172", on_floats=True)
        assert not ac.airport_usable(base, water, "C172")

    def test_a_float_variant_still_uses_a_paved_runway(self):
        # The flag says the airframe can take water; it never takes its
        # pavement away -- the floats-on-wheels amphibian goes both ways.
        assert ac.runway_usable(_paved_runway(), "C172", on_floats=True)


class TestPoweredLift:
    def test_tiltrotors_and_stovl_jets_are_powered_lift(self):
        for code in ("V22", "A609", "HAR", "AV8B"):
            assert ac.classify(code).name == ac.POWERED_LIFT, code
        assert ac.classify("F35", title="F-35B Lightning II").name == ac.POWERED_LIFT
        assert ac.classify("F35", title="F-35A Lightning II").name != ac.POWERED_LIFT

    def test_the_simulator_type_does_not_hide_powered_lift(self):
        # A tiltrotor's engines may be reported as turboprops and a Harrier's
        # as jets; the type evidence is more specific than either category.
        assert ac.classify("V22", engine_type=5, engines=2).name == ac.POWERED_LIFT
        assert ac.classify("HAR", engine_type=1, engines=1).name == ac.POWERED_LIFT

    def test_powered_lift_can_plan_on_vertical_ports(self):
        pad = SimpleNamespace(type="heliport")
        assert ac.airport_usable(pad, [], "V22")
        assert ac.airport_usable(pad, [], "HAR")
        # F35 alone is ambiguous: the A and C are conventional aircraft, so
        # the shared ICAO code is not enough to plan a pad without the B title.
        assert not ac.airport_usable(pad, [], "F35")

    def test_powered_lift_runway_planning_is_short_but_not_zero(self):
        assert ac.min_runway_ft("V22") <= 1500
        assert ac.min_runway_ft("HAR") <= 1500
        assert ac.min_runway_ft("F35") >= 5000


def test_powered_lift_checklists_are_not_copied_business_jet_flows():
    """The powered-lift class is conversion/nacelle centric and must not
    brief business-jet-only pressurisation/yaw-damper items as a POH."""
    from meridianatc.atc import checklist_files

    book = checklist_files.load("powered_lift")
    text = " ".join(
        str(item.challenge) + " " + str(item.response)
        for checklist in book.checklists.values()
        for item in checklist.items).lower()
    assert "conversion" in text or "nacelle" in text
    assert "pressurisation" not in text
    assert "yaw damper" not in text
