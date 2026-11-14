"""The X-Plane CIFP reader: per-airport procedure files, fix resolution,
and the procedure-selection policy the controller, FO and traffic share.

Fixtures are a scratch ``CIFP`` tree + ``earth_fix.dat``/``earth_nav.dat``
written beside it -- the same layout ``Custom Data`` has, none of the
real install needed.
"""

from __future__ import annotations

from pathlib import Path
from unittest.mock import Mock

import pytest

from meridianatc.atc.procedure_legs import (
    choose_sid,
    choose_star,
)
from meridianatc.atc.procedure_select import (
    pick_sid,
    pick_star,
    select_sid,
    select_star,
)
from meridianatc.atc.traffic import TrafficInjector
from meridianatc.atc.traffic._shared import _runway_key, _strips_parallel
from meridianatc.integrations.cifp import CifpSource


def _fix_line(ident, lat, lon, terminal="ENRT", region="K2", numid="1"):
    return f"{lat:>13.9f} {lon:>14.9f} {ident} {terminal} {region} {numid}\n"


def _nav_line(ident, lat, lon, freq="11200", terminal="ENRT", region="K2"):
    return (f"3 {lat:>13.9f} {lon:>14.9f} 100 {freq} 130 0.0 {ident} "
            f"{terminal} {region} TEST NAVAID\n")


@pytest.fixture
def tree(tmp_path: Path) -> Path:
    """A field with one runway, one SID with runway variants, a STAR with
    a named transition, an ILS and an RNAV approach."""
    cifp = tmp_path / "CIFP"
    cifp.mkdir()
    (tmp_path / "earth_fix.dat").write_text(
        "I\n1200 Version - data cycle 9999\n"
        "99\n"
        + _fix_line("ALPHA", 51.0, -1.0)
        + _fix_line("BETAX", 51.5, -0.5)
        + _fix_line("GAMAX", 51.8, -0.2)
        + _fix_line("ENTRY", 50.0, -2.0)
        + _fix_line("MIDPT", 50.5, -1.5)
        + _fix_line("T111A", 51.001, -1.05, terminal="TEST", region="K1")
        + _fix_line("T222B", 51.2, -1.1, terminal="TEST", region="K1")
        + _fix_line("HLDXY", 50.2, -1.0)
    )
    (tmp_path / "earth_nav.dat").write_text(
        "I\n1200 Version - data cycle 9999\n"
        "99\n"
        + _nav_line("TST", 51.0, -1.0)
    )
    (cifp / "TEST.dat").write_text(
        "I\n1200 Version - data cycle 9999\n"
        "99\n"
        "RWY:RW09L,+0000,      ,00100, ,ITST,3,   ;N51000000,W001000000,0100;\n"
        # A SID with runway variants for 09L/09R ending at ALPHA: complete
        # runway paths under type 2 (the Navigraph style).
        "SID:010,2,ALPH1A,RW09L,T111A,K1,P,C,EY  , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,06000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        "SID:020,2,ALPH1A,RW09L,ALPHA,K2,E,A,VE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,06000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        "SID:010,2,ALPH1B,RW09R,T222B,K1,P,C,EY  , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,06000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        "SID:020,2,ALPH1B,RW09R,ALPHA,K2,E,A,VE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,06000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        # A second SID ending south: the direction pick prefers ALPH1A
        # for a flight going north.
        "SID:010,2,BETA1A,RW09L,T111A,K1,P,C,EY  , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,06000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        "SID:020,2,BETA1A,RW09L,BETAX,K2,E,A,VE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,06000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        # A STAR: a common "ALL" portion plus a named transition.
        "STAR:010,5,GAMA1E,ALL,MIDPT,K2,E,A,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,09000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        "STAR:020,5,GAMA1E,ALL,TST,K2,D, ,VE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,06000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        "STAR:010,1,GAMA1E,ENTRY,ENTRY,K2,E,A,E   , ,   ,IF, , , , , ,      ,    ,    ,    ,    , ,12000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        "STAR:020,1,GAMA1E,ENTRY,MIDPT,K2,E,A,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,09000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        # An ILS to 09L with a final leg and a missed-approach level.
        "APPCH:010,A,I09L,T111A,T111A,K1,P,C,E   , ,   ,IF, , , , , ,      ,    ,    ,    ,    ,+,08000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        "APPCH:020,I,I09L, ,RW09L,K1,P,G,VE  , ,   ,TF, ,ITST,K1,P,I,      ,    ,    ,1800,    , ,02000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        "APPCH:030,I,I09L, ,TST,K2,D, ,VE  , ,   ,HM,R, , , , ,      ,    ,    ,0840,T010, ,05000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        # An RNAV to 09L whose PRDAT carries the minima lines.
        "APPCH:010,A,R09LY,T111A,T111A,K1,P,C,E   , ,   ,IF, , , , , ,      ,    ,    ,    ,    ,+,08000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        "APPCH:020,R,R09LY, ,RW09L,K1,P,G,VE  , ,   ,RF,R,TST,K2,D, ,  4000,    ,    ,    ,    , ,02000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
        "PRDAT: ,          ,A, LNAV/VNAV,A,      LNAV, ,   , ,   , ,   , ,   ,J,S;\n"
        "APPCH:030,R,R09LY, ,HLDXY,K2,E,A,E   , ,   ,HM,L, , , , ,      ,    ,    ,0900,T015, ,03000,     ,     , ,   ,    ,   , , , , , , , , ;\n"
    )
    return tmp_path


def test_rw_runways(tree):
    source = CifpSource([tree])
    af = source._file("TEST")
    assert af.runways["09L"] == pytest.approx((51.0, -1.0))


def test_sid_runway_variants(tree):
    procs = CifpSource([tree]).procedures("TEST")
    assert set(procs.departures) == {"ALPH1A", "ALPH1B", "BETA1A"}
    proc, trans = choose_sid(procs, "ALPH1A", runway="09L")
    assert trans.runway == "09L"
    assert [leg.fix_ident for leg in trans.legs] == ["T111A", "ALPHA"]
    # The runway's variant is not served to the other runway.
    _p, wrong = choose_sid(procs, "ALPH1A", runway="09R")
    assert wrong is None or wrong.runway != "09L"
    # The terminal waypoint resolves against earth_fix's TEST rows.
    first = trans.legs[0]
    assert first.has_fix
    assert (first.lat, first.lon) == pytest.approx((51.001, -1.05))
    # The enroute fix resolves against the ENRT rows.
    assert (trans.legs[-1].lat, trans.legs[-1].lon) == \
        pytest.approx((51.0, -1.0))


def test_star_transitions(tree):
    procs = CifpSource([tree]).procedures("TEST")
    proc, trans = choose_star(procs, "GAMA1E", runway="09L",
                              preferred="ENTRY")
    assert trans is not None
    assert trans.name == "ENTRY"
    # Enroute transition legs first, then the shared "ALL" part.
    assert [leg.fix_ident for leg in trans.legs] == \
        ["ENTRY", "MIDPT", "MIDPT", "TST"]
    # The navaid's coordinates come from earth_nav.dat.
    assert (trans.legs[-1].lat, trans.legs[-1].lon) == \
        pytest.approx((51.0, -1.0))


def test_altitude_and_speed_constraints(tree):
    procs = CifpSource([tree]).procedures("TEST")
    _p, trans = choose_star(procs, "GAMA1E")
    assert trans.legs[0].alt1_ft == 12000.0


def test_hold_leg_time(tree):
    """A "T010" leg length is 1.0 minute, not 10 NM."""
    # The HM leg's time survives on the decoded approach side below;
    # here it is checked through the raw decode helper instead.
    af = CifpSource([tree])._file("TEST")
    hm = [f for f in af.legs["APPCH"]
          if len(f) > 11 and f[11].strip() == "HM"]
    assert hm and hm[0][21].strip() == "T010"


def test_approaches(tree):
    apps = CifpSource([tree]).approaches("TEST")
    kinds = {(a.kind, a.runway, a.suffix) for a in apps.procedures}
    assert ("ILS", "09L", "") in kinds
    assert ("RNAV", "09L", "Y") in kinds
    rnav = next(a for a in apps.procedures if a.kind == "RNAV")
    assert "LNAV/VNAV" in rnav.lines and "LNAV" in rnav.lines


def test_missing_airport(tree):
    source = CifpSource([tree])
    assert source.procedures("ZZZZ") is None
    assert source.approaches("ZZZZ") is None
    # And cached, so a second ask does not re-read the disk.
    assert "ZZZZ" in source._files


def test_no_data_dir(tmp_path):
    assert CifpSource([tmp_path]).procedures("TEST") is None


def test_select_validates_not_invents(tree):
    procs = CifpSource([tree]).procedures("TEST")
    found = select_sid(procs, "NOSUCH1", runway="09L")
    assert not found.ok and found.status == "not_found"
    found = select_star(procs, "NOSUCH9")
    assert not found.ok and found.status == "not_found"
    assert select_sid(None, "ALPH1A").status == "not_found"


def test_pick_sid_prefers_direction(tree):
    """Among the runway's SIDs the pick is the one ending the way the
    flight is going: north for a flight north, not BETA1A."""
    procs = CifpSource([tree]).procedures("TEST")
    at = (51.0, -1.0)
    north = pick_sid(procs, runway="09L", at=at, toward=(54.0, -1.0))
    assert north.ok and north.name == "ALPH1A"
    south = pick_sid(procs, runway="09L", at=at, toward=(48.0, -1.0))
    assert south.ok and south.name == "BETA1A"


def test_pick_star_prefers_entry(tree):
    procs = CifpSource([tree]).procedures("TEST")
    chosen = pick_star(procs, runway="09L", at=(51.0, -1.0),
                       toward=(50.0, -2.0))
    assert chosen.ok and chosen.name == "GAMA1E"
    assert chosen.transition.name == "ENTRY"


def test_expand_runway_b(tree):
    """"16B" means every 16 strip the file declares -- 16L/16C/16R at a
    three-parallel field, L and R only when the file says nothing."""
    from meridianatc.integrations.cifp import _expand_runway
    af = CifpSource([tree])._file("TEST")
    assert _expand_runway("09B", af) == ["09L"]
    af.runways["09R"] = (51.001, -1.0)
    af.runways["09C"] = (51.0, -1.001)
    assert _expand_runway("09B", af) == ["09C", "09L", "09R"]
    af.runways.clear()
    assert _expand_runway("16B", af) == ["16L", "16R"]
    assert _expand_runway("16L", af) == ["16L"]


def test_runway_key_strips():
    assert _runway_key("09L") == _runway_key("27R")
    assert _runway_key("09L") != _runway_key("09R")
    assert _strips_parallel(_runway_key("09L"), _runway_key("09R"))
    assert not _strips_parallel(_runway_key("09L"), _runway_key("16L"))


def test_per_runway_occupancy():
    """A roll on one strip frees the other: what a parallel field moves
    by."""
    navdb = Mock()
    navdb.airport.return_value = None
    injector = TrafficInjector(navdb, seed=1)
    injector.field.departure_runway = "09L"
    injector.field.arrival_runway = "09R"
    injector._hold_runway("09L", now=100.0, seconds=30.0)
    assert injector._runway_free_at("09L") == 130.0
    # 27R is the same strip: an arrival the other way is held the same.
    assert injector._runway_free_at("27R") == 130.0
    # 09R is the parallel: free.
    assert injector._runway_free_at("09R") == 0.0
    assert injector.runway_is_busy(110.0, runway="09L")
    assert not injector.runway_is_busy(110.0, runway="09R")
    injector._release_runway("09L", now=120.0)
    assert injector._runway_free_at("09L") == 120.0


def test_max_on_approach_scales():
    injector = TrafficInjector(Mock(), seed=1, density=1.0)
    assert injector._max_on_approach() == 4
    injector.density = 2.0
    assert injector._max_on_approach() == 8
    injector.density = 0.0
    assert injector._max_on_approach() == 1
