"""The rotorcraft controller flown closed-loop against a small helicopter.

Asserted on where the helicopter ends up -- on the spot, stopped, at a
sink the skids survive -- not on which events left.
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from heli_plant import HeliPlant

from meridianatc.navdata.build import project

SPOT_LAT, SPOT_LON = 45.0, 5.0


class _Session:
    """A flight nobody has cleared for anything: every ask is None."""

    ifr = False

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return


def _heli(plant, stage, monkeypatch, pids=False):
    pytest.importorskip("scipy")
    from meridianatc.atc import autonomous_heli as mod
    from meridianatc.atc.autonomous import AutonomousState

    clock = [1000.0]
    monkeypatch.setattr(mod.time, "monotonic", lambda: clock[0])
    owner = SimpleNamespace(
        config=SimpleNamespace(
            immersion=SimpleNamespace(autonomous_pid=pids)),
        session=_Session(),
        sim=None, get_state=plant.state, crew=Mock(), on_event=Mock(),
        state=AutonomousState(), _send_event=plant.send, _emit=Mock(),
        _engines_out=lambda state: (
            (0, 1) if state.engines_running else (1, 1)),
        _use_pids=pids, _envelope_protection=True,
        _declare_emergency=Mock(), _set_error=Mock(), _transition=Mock())
    heli = mod.RotorcraftFlight(owner)
    heli._checklist = lambda *a, **k: None
    heli.state = mod.HeliState(
        stage=stage, stage_entered_at=clock[0], dest_lat=SPOT_LAT,
        dest_lon=SPOT_LON, dest_elev_ft=0.0, dest_ident="LFXX",
        cruise_alt_ft=1500.0, pattern_alt_ft=mod.PATTERN_AGL_FT)
    made = heli._pid

    def on_the_plants_clock(channel):
        pid = made(channel)
        pid._clock = lambda: clock[0]
        return pid

    heli._pid = on_the_plants_clock
    return heli, owner, clock


def _run(plant, heli, clock, seconds, until=lambda: False, dt=0.05):
    for i in range(int(seconds / dt)):
        if i % 5 == 0:
            heli.tick(plant.state())
        plant.step(dt)
        clock[0] += dt
        if until():
            break


def _spot_m(plant):
    return plant.state().distance_to(SPOT_LAT, SPOT_LON) * 1852.0


def _inbound(distance_nm, agl_ft, speed_kt, bearing_from=270.0):
    lat, lon = project(SPOT_LAT, SPOT_LON, bearing_from, distance_nm)
    return HeliPlant(lat, lon, agl_ft, (bearing_from + 180.0) % 360.0,
                     speed_kt)


@pytest.mark.parametrize("pids", [False, True])
def test_the_approach_ends_on_the_spot_not_short_of_it(monkeypatch, pids):
    from meridianatc.atc.autonomous_heli import PATTERN_AGL_FT
    plant = _inbound(2.5, PATTERN_AGL_FT, 60.0)
    heli, owner, clock = _heli(plant, "approach", monkeypatch, pids)
    _run(plant, heli, clock, 900.0, until=lambda: plant.on_ground)
    assert plant.touchdown is not None, heli.state.stage
    owner._set_error.assert_not_called()
    assert _spot_m(plant) < 120.0
    # Put down from a hover: no run-on, no drop.
    assert plant.touchdown["vs"] > -400.0
    assert abs(plant.touchdown["u"]) < 12.0
    assert abs(plant.touchdown["v"]) < 6.0


def test_the_approach_holds_its_height_until_the_spot_is_in_reach(
        monkeypatch):
    from meridianatc.atc.autonomous_heli import PATTERN_AGL_FT
    plant = _inbound(2.5, PATTERN_AGL_FT, 60.0)
    heli, owner, clock = _heli(plant, "approach", monkeypatch)
    _run(plant, heli, clock, 40.0)
    # Two miles out the old approach was already on its way down.
    assert _spot_m(plant) > 2500.0
    assert plant.alt > PATTERN_AGL_FT - 120.0


def test_a_landing_from_the_hover_keeps_its_heading_and_its_place(
        monkeypatch):
    plant = HeliPlant(SPOT_LAT, SPOT_LON, 30.0, 135.0, 4.0)
    heli, owner, clock = _heli(plant, "landing", monkeypatch)
    _run(plant, heli, clock, 240.0, until=lambda: plant.on_ground)
    assert plant.touchdown is not None
    assert plant.touchdown["vs"] > -300.0
    assert _spot_m(plant) < 40.0
    assert abs(((plant.hdg - 135.0 + 180.0) % 360.0) - 180.0) < 25.0


def test_a_lift_off_is_not_dropped_through_fifteen_feet(monkeypatch):
    from meridianatc.atc.autonomous_heli import HOVER_AGL_FT
    plant = HeliPlant(SPOT_LAT, SPOT_LON, 0.0, 90.0)
    heli, owner, clock = _heli(plant, "hover", monkeypatch)
    heli.state.dest_lat, heli.state.dest_lon = project(
        SPOT_LAT, SPOT_LON, 90.0, 20.0)
    _run(plant, heli, clock, 60.0,
         until=lambda: plant.alt > HOVER_AGL_FT * 0.6)
    assert plant.alt > HOVER_AGL_FT * 0.6


def test_too_slow_is_answered_with_forward_cyclic(monkeypatch):
    plant = HeliPlant(SPOT_LAT + 1.0, SPOT_LON, 1500.0, 180.0, 50.0)
    heli, owner, clock = _heli(plant, "cruise", monkeypatch)
    heli.state.ias_cruise_kt = 90.0
    _run(plant, heli, clock, 120.0)
    assert plant.u > 75.0, plant.u
    owner._set_error.assert_not_called()


def test_an_overdue_stage_in_the_air_keeps_flying(monkeypatch):
    plant = HeliPlant(SPOT_LAT + 1.0, SPOT_LON, 1500.0, 180.0, 80.0)
    heli, owner, clock = _heli(plant, "climb", monkeypatch)
    heli.state.cruise_alt_ft = 30000.0        # never reached
    heli.state.stage_entered_at = clock[0] - 901.0
    _run(plant, heli, clock, 2.0)
    owner._set_error.assert_not_called()
    assert heli.state.stage == "cruise"


def test_an_engine_failure_is_flown_to_the_ground_not_abandoned(monkeypatch):
    from meridianatc.atc.autonomous_heli import COLLECTIVE_SET
    plant = HeliPlant(SPOT_LAT + 0.1, SPOT_LON, 1200.0, 180.0, 80.0)
    heli, owner, clock = _heli(plant, "cruise", monkeypatch)
    _run(plant, heli, clock, 2.0)
    plant.engine = False
    _run(plant, heli, clock, 3.0)
    assert heli.state.stage == "autorotate"
    owner._declare_emergency.assert_called_once()
    # The lever goes down at once: the rotor is kept turning.
    assert plant.coll < 20.0
    steady = []
    _run(plant, heli, clock, 300.0,
         until=lambda: steady.append(plant.vs) or plant.on_ground)
    assert plant.touchdown is not None
    owner._set_error.assert_not_called()
    # The flare and the pull took most of the descent rate off, and the
    # rotor was still turning when they were asked for.
    assert plant.touchdown["vs"] > 0.4 * min(steady)
    assert any(name == COLLECTIVE_SET and value > 8000
               for name, value in plant.sent[-40:])
