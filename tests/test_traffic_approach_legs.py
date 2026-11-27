"""Traffic following the published approach legs.

An IFR arrival used to fly the STAR's legs and then a straight-in the
charts do not have -- LECO's RNP 03 ends on a final eighty degrees off
the runway's heading at a missed-approach point that is not the
threshold, and the traffic stream drew the same line over it as over
every runway. The cleared approach's own legs now take over when the
ones before them run out: these regressions pin that the model flies
them, that a cleared approach with no STAR in front of it is still
flown, and that the simulator's own facility read decodes the legs.
"""

import struct
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


def _injector():
    from meridianatc.atc.traffic.core import TrafficInjector
    injector = TrafficInjector.__new__(TrafficInjector)
    injector.field = SimpleNamespace(elevation_ft=0.0)
    return injector


def _performance():
    return SimpleNamespace(approach_kt=90.0, climb_kt=100.0,
                           cruise_kt=140.0, descend_fpm=1800.0,
                           rotate_kt=80.0)


# An offset final the way LECO's RNP 03 is drawn: the last leg comes in
# off the runway's axis toward a missed-approach point that is not the
# threshold.
_THRESHOLD = (43.295, -8.377)     # LECO runway 03
_LEGS = (
    ("CO418", 43.370, -8.300, 4000.0, False, False),
    ("CO424", 43.320, -8.420, 2000.0, False, True),
    ("RW03", 43.302, -8.407, 900.0, True, False),
)


def _approach_legs():
    from meridianatc.atc.procedure_legs import ProcedureLeg
    legs = []
    for index, (ident, lat, lon, alt, is_map, is_faf) in enumerate(_LEGS):
        legs.append(ProcedureLeg(
            index=index, path="TF" if index else "IF",
            path_type=18 if index else 15,
            fix_ident=ident, lat=lat, lon=lon, alt1_ft=alt,
            is_map=is_map, is_faf=is_faf))
    return tuple(legs)


def _arrival(**over):
    flight = SimpleNamespace(
        phase="inbound", latitude=43.40, longitude=-8.28, heading=210.0,
        altitude_ft=6000.0, distance_nm=20.0, on_ground=False,
        airborne_model=True, ground_speed_kt=150.0,
        runway="03", arriving=True, ifr=True,
        sid="", star="", approach="RNP",
        approach_legs=_approach_legs(),
        procedure_pair=None, procedure_nav=None, hold=None,
        missed_nav=None, missed_alt_ft=0.0,
        performance=_performance(),
        target=SimpleNamespace(heading=None, altitude_ft=None,
                               speed_kt=None, intent=""),
        assigned_heading=None, assigned_altitude_ft=None,
        assigned_speed_kt=None, expedite_until=0.0)
    for key, value in over.items():
        setattr(flight, key, value)
    return flight


def test_an_arrival_with_no_star_takes_the_approach_legs():
    """Cleared for the approach from the start: the published final is
    the path, offset course and all, not a straight-in invented at the
    threshold."""
    injector = _injector()
    flight = _arrival()
    injector._steer_procedure(flight, flight.target)
    assert flight.procedure_nav is not None
    assert flight.procedure_nav.procedure.kind == "approach"
    assert flight.target.heading is not None
    assert flight.target.intent == "on the RNP approach"
    # The legs were taken over, not left to promote again.
    assert flight.approach_legs == ()


def test_the_legs_come_off_the_runways_axis():
    """The path the navigator hands the model is the charted one: it
    ends at the published missed-approach point, offset from the
    runway's centreline -- a straight-in would put the last point on
    the axis instead."""
    injector = _injector()
    flight = _arrival()
    injector._steer_procedure(flight, flight.target)
    last = flight.procedure_nav.points[-1]
    assert last[0] == pytest.approx(43.302, abs=0.01)
    assert last[1] == pytest.approx(-8.407, abs=0.01)


def test_a_done_star_hands_over_to_the_approach():
    """The STAR's last leg is not where the published flying stops: the
    cleared approach's own legs take the pair over the tick the STAR
    runs out."""
    injector = _injector()
    flight = _arrival(procedure_pair=(Mock(), Mock()),
                      procedure_nav=SimpleNamespace(
                          guidance=lambda f: SimpleNamespace(done=True)))
    injector._steer_procedure(flight, flight.target)
    proc, _trans = flight.procedure_pair
    assert proc.kind == "approach"
    assert flight.target.heading is not None


def test_a_done_approach_goes_back_to_the_phase():
    """Once the published legs themselves run out the model is not
    restarted on them: the pair is let go and the phase's own answer --
    the intercept and the glide -- lands it."""
    injector = _injector()
    flight = _arrival(approach_legs=(),
                      procedure_pair=(Mock(), Mock()),
                      procedure_nav=SimpleNamespace(
                          guidance=lambda f: SimpleNamespace(done=True)))
    injector._steer_procedure(flight, flight.target)
    assert flight.procedure_pair is None
    assert flight.procedure_nav is None


def test_a_departure_is_never_handled_the_approach():
    """The promotion is an arrival's: a departure's pair running out is
    simply the end of the procedure steering."""
    injector = _injector()
    flight = _arrival(arriving=False)
    injector._steer_procedure(flight, flight.target)
    assert flight.procedure_pair is None
    assert flight.procedure_nav is None


def test_an_approach_arrival_descends_on_the_path():
    """The continuous-descent smoothing a STAR got is the approach's
    too: three degrees down to the level the fix owns, not the book
    rate and a level wait."""
    injector = _injector()
    nav = SimpleNamespace(
        procedure=SimpleNamespace(kind="approach"),
        guidance=lambda f: SimpleNamespace(
            heading=210.0, altitude_ft=2000.0, vs_fpm=-1800.0,
            speed_kt=0.0, done=False, altitude_distance_nm=10.0))
    flight = _arrival(procedure_pair=(Mock(), Mock()),
                      procedure_nav=nav)
    injector._steer_procedure(flight, flight.target)
    assert flight.target.altitude_ft == pytest.approx(2000.0 + 3180.0)


# -- the simulator's own read ------------------------------------------------

def _record(type_, raw, unique_id=0, parent_id=0, item_index=0,
            is_list_item=1):
    from meridianatc.integrations.taxiways import FacilityRecord
    return FacilityRecord(type=type_, unique_id=unique_id,
                          parent_id=parent_id, item_index=item_index,
                          list_size=1, is_list_item=is_list_item,
                          raw=raw)


def _leg_raw(fix_icao: bytes, lat: float, lon: float, alt_m: float,
             path_type: int = 18, is_map: int = 0) -> bytes:
    from meridianatc.integrations.terminal_procedures import _LEG_SHAPE
    values = (path_type, fix_icao, b"LE", 86, lat, lon, alt_m, 0,
              0, 1, 0, b"LECO", b"LE", 65, 43.295, -8.377, 100.0,
              0.0, 0.0, 114.0, 0.0, 1, alt_m, alt_m, 0.0, 3.0,
              b"CO424", b"LE", 86, 43.32, -8.42, 500.0, 0.0,
              0, 0, 0, is_map, 0, 1, 0, 0, 0.5, 1)
    return struct.pack(_LEG_SHAPE, *values)


def test_the_simulators_approach_read_decodes_the_legs():
    """What the simulator reports has the path in it now: the final's
    legs on the approach, the missed's after it, and the RNAV lines
    the HAS flags name."""
    from meridianatc.integrations.approaches import _SHAPE, decode
    header = struct.pack(_SHAPE, 10, ord("Z"), 3, 0, 1200.0, 1500.0,
                         1, 1, 0, 1, 0, 3, 2)
    records = [
        _record(5, header, unique_id=100),
        _record(8, _leg_raw(b"CO418", 43.370, -8.300, 1200.0,
                            path_type=15),
                parent_id=100, item_index=0),
        _record(8, _leg_raw(b"CO424", 43.320, -8.420, 600.0),
                parent_id=100, item_index=1),
        _record(8, _leg_raw(b"RW03", 43.302, -8.407, 270.0, is_map=1),
                parent_id=100, item_index=2),
        _record(9, _leg_raw(b"CO550", 43.350, -8.500, 900.0),
                parent_id=100, item_index=0),
    ]
    found = decode(records, "LECO")
    assert found is not None
    approach = found.procedures[0]
    assert approach.kind == "RNAV" and approach.suffix == "Z"
    assert approach.runway == "03"
    assert approach.lines == frozenset({"LNAV", "LNAV/VNAV", "LPV"})
    assert len(approach.legs) == 3
    assert approach.legs[-1].fix_ident == "RW03"
    assert approach.legs[-1].is_map
    assert approach.legs[1].alt1_ft == pytest.approx(600.0 * 3.28084)
    assert len(approach.missed_legs) == 1
    assert approach.missed_legs[0].fix_ident == "CO550"


def test_a_leg_block_of_another_size_keeps_the_header():
    """Legs this build does not read are dropped on their own: the
    approach's name and altitudes still stand, the way the header-only
    read always had them."""
    from meridianatc.integrations.approaches import _SHAPE, decode
    header = struct.pack(_SHAPE, 4, 0, 3, 0, 1200.0, 1500.0,
                         0, 0, 0, 0, 0, 0, 0)
    records = [
        _record(5, header, unique_id=100),
        _record(8, b"\x00" * 12, parent_id=100, item_index=0),
    ]
    found = decode(records, "LECO")
    assert found is not None
    approach = found.procedures[0]
    assert approach.kind == "ILS" and approach.legs == ()


def test_traffic_smoothing_uses_the_approachs_own_vpa_angle():
    """A steep RNP-AR final is not smoothed on a generic three-degree
    slope: London City's 5.5-degree path puts three miles at roughly
    1,820 ft over the threshold, not 1,020 ft."""
    injector = _injector()
    nav = SimpleNamespace(
        procedure=SimpleNamespace(kind="approach"),
        guidance=lambda f: SimpleNamespace(
            heading=273.0, altitude_ft=69.0, vs_fpm=-1300.0,
            speed_kt=0.0, done=False, altitude_distance_nm=3.0,
            vpa_ft_per_nm=585.0))
    flight = _arrival(procedure_pair=(Mock(), Mock()),
                      procedure_nav=nav, ground_speed_kt=140.0)
    injector._steer_procedure(flight, flight.target)
    assert flight.target.altitude_ft == pytest.approx(
        69.0 + 3.0 * 585.0, abs=20.0)
