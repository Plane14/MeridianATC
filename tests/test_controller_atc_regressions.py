"""Controller-side ATC regressions found in the feature audit."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

from meridianatc.atc.controller import ControllerBrain
from meridianatc.atc.intents import Intent, ParsedIntent
from meridianatc.atc.phraseology import Aircraft, Phraseology, Weather
from meridianatc.atc.session import FlightSession, Phase
from meridianatc.atc.speech import parse_callsign
from meridianatc.navdata.db import Station


def _brain():
    brain = ControllerBrain(Mock(), Phraseology())
    brain.staffed = lambda station: True
    return brain


def test_uncalled_traffic_in_sight_is_acknowledged_not_a_separation_instruction():
    brain = _brain()
    session = FlightSession(Aircraft(parse_callsign("N123")))
    station = Station("TEST", "APP", 124.5, "Approach")
    parsed = ParsedIntent(intent=Intent.REPORT_TRAFFIC_IN_SIGHT,
                          normalized="traffic in sight")
    state = SimpleNamespace(on_ground=False)

    reply = brain._radar(parsed, session, station, Weather(), Phraseology(), state)

    assert "roger" in reply.text.lower()
    assert "visual separation" not in reply.text.lower()


def test_called_traffic_in_sight_still_carries_visual_separation():
    brain = _brain()
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.traffic_called["ABC"] = "called"
    station = Station("TEST", "APP", 124.5, "Approach")
    parsed = ParsedIntent(intent=Intent.REPORT_TRAFFIC_IN_SIGHT,
                          normalized="traffic in sight")
    state = SimpleNamespace(on_ground=False)

    reply = brain._radar(parsed, session, station, Weather(), Phraseology(), state)

    assert "visual separation" in reply.text.lower()


def test_the_taxi_instruction_made_when_takeoff_is_asked_early_can_be_repeated():
    brain = _brain()
    airport = SimpleNamespace(ident="TEST")
    brain.navdb.airport.return_value = airport
    brain.navdb.runway_at.return_value = "27"
    brain.navdb.at_the_runway.return_value = False
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.departure_runway = "09"
    session.phase = Phase.TAXI_OUT
    station = Station("TEST", "TWR", 118.1, "Tower")
    parsed = ParsedIntent(intent=Intent.REQUEST_TAKEOFF,
                          normalized="ready for departure")
    parsed.values["runway"] = None
    state = SimpleNamespace(has_position=True, latitude=40.0,
                            longitude=-74.0, heading_true=90.0)

    reply = brain._issue_takeoff(session, station, Weather(), Phraseology(),
                                 parsed, state)

    assert reply.instruction is not None
    assert reply.instruction.text == reply.text
    assert reply.instruction.text
