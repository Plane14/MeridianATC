"""Regressions from the second audit pass: chain-leg state, altimeter
units, checklist toggles and held live traffic."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest


def _flight():
    from meridianatc.atc.autonomous import (
        AutonomousFlight, AutonomousState)
    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    return flight


def test_an_inches_altimeter_is_dialled_in_hectopascals():
    """A "2992" from the controller is not written as 29.92 hPa."""
    from meridianatc.atc.autonomous import AutonomousFlight
    convert = AutonomousFlight._altimeter_hpa
    assert convert(29.92) == pytest.approx(1013.2, abs=0.2)
    assert convert(30.00) == pytest.approx(1015.9, abs=0.2)
    assert convert(1013) == 1013
    assert convert(0) == 0


def test_the_radio_dial_compares_like_units():
    """The dialled check cannot call 29.92 inHg 'already at 29.92 hPa'."""
    flight = _flight()
    flight._dialled_at = 0.0
    flight.state.squawk_dialled = "1200"
    flight.state.altimeter_dialled = 0.0
    flight.sim = SimpleNamespace(set_squawk=lambda code: True)
    flight._dir = SimpleNamespace(kohlsman=Mock(), sim=flight.sim)
    sim_state = SimpleNamespace(
        kohlsman_hpa=1013.0, transponder_code="1200")
    flight.session = SimpleNamespace(squawk="1200", altimeter=29.92)
    # 1013.25 hPa showing, 29.92 given: the two match within half a
    # hectopascal -- nothing is dialled, the figure is remembered.
    flight._dial_radios(sim_state, 100.0)
    flight._dir.kohlsman.assert_not_called()
    assert flight.state.altimeter_dialled == pytest.approx(1013.2, abs=0.2)


def test_a_chained_leg_forgets_the_last_legs_session():
    """The readback owed on leg one is not owed again on leg two."""
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign

    flight = _flight()
    session = FlightSession(Aircraft(parse_callsign("N123")))
    # Leg one's leavings.
    session.squawk = "4567"
    session.altimeter = 29.92
    session.assigned_altitude_ft = 11000.0
    session.assigned_heading = 270.0
    session.cleared_landing = True
    session.going_around = True
    session.emergency = "MAYDAY"
    session.pending = object()
    session.taxi_path = ["A", "B"]
    session.log.append((0.0, "tower", "cleared to land"))
    flight.session = session
    flight.navdb = SimpleNamespace(
        airport=lambda ident: SimpleNamespace(ident=ident))

    flight._reset_session_for_leg({
        "origin": "EGLL", "destination": "LFPG",
        "cruise_altitude_ft": 35000, "ifr": True})

    assert session.squawk == "1200"
    assert session.altimeter == 0.0
    assert session.assigned_altitude_ft == 0.0
    assert session.assigned_heading is None
    assert session.cleared_landing is False
    assert session.going_around is False
    assert session.emergency == ""
    assert session.pending is None
    assert session.taxi_path == []
    # What is kept: the aeroplane, the transcript, the new leg's fields.
    assert session.aircraft.callsign.written == "N123"
    assert session.log == [(0.0, "tower", "cleared to land")]
    assert session.departure.ident == "EGLL"
    assert session.destination.ident == "LFPG"
    assert session.destination_input == "LFPG"
    assert session.cruise_altitude_ft == 35000


def test_a_toggle_switch_is_asked_once_per_checklist_item():
    """A seatbelt toggle re-asked would turn the sign back off."""
    flight = _flight()
    flight._checklist_sent = set()
    flight._switch_for = Mock(return_value=True)
    flight.crew = SimpleNamespace(
        checklists=SimpleNamespace(_expected=lambda c: c.value))
    state = SimpleNamespace(seatbelt_sign=False)
    check = SimpleNamespace(
        var="seatbelt_sign", value=1, op="eq", tolerance=0.0)
    item = SimpleNamespace(check=check)

    assert flight._perform_check_item(state, item) is True
    assert flight._perform_check_item(state, item) is True
    assert flight._switch_for.call_count == 1
    # The next item gets its own ask.
    flight._checklist_sent.clear()
    assert flight._perform_check_item(state, item) is True
    assert flight._switch_for.call_count == 2


def test_a_held_live_aeroplane_is_let_go_when_its_strip_goes():
    """An unreleased hold draws a statue on the feed forever."""
    from meridianatc.atc.traffic.core import TrafficInjector

    injector = TrafficInjector.__new__(TrafficInjector)
    injector.hold_live = Mock()
    held = SimpleNamespace(live_held="runway", hex="abc123")
    injector._release_held(held)
    injector.hold_live.assert_called_once_with("abc123", False)
    assert held.live_held == ""
    # And one never held asks nothing of the feed.
    injector.hold_live.reset_mock()
    injector._release_held(SimpleNamespace(live_held="", hex="def456"))
    injector.hold_live.assert_not_called()
