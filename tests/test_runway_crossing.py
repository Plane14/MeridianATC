"""The runway-crossing clearance: asked for, granted, refused and done.

Every runway a real taxi route meets is a clearance of its own. These cover
the lifecycle -- the pilot asking, ground granting or holding, the watch
spending the grant once the runway is behind, and the entry nobody cleared.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from meridianatc.atc.intents import Intent, IntentParser
from meridianatc.atc.phraseology import Aircraft, Phraseology
from meridianatc.atc.session import FlightSession
from meridianatc.atc.speech import parse_callsign
from meridianatc.navdata.db import Station
from meridianatc.sim.state import AircraftState


def _layout():
    """A flat plane where longitude is x and latitude is z, metres each."""
    return SimpleNamespace(
        to_xy=lambda lat, lon: (lon * 1000.0, lat * 1000.0),
        to_latlon=lambda x, z: (z / 1000.0, x / 1000.0),
    )


def _navdb():
    """One field, one runway: 09/27 along x at z=50000, 20 m long."""
    navdb = Mock()
    navdb.airport.return_value = SimpleNamespace(ident="TEST", lat=50.0,
                                                 lon=3.0)
    navdb.runways.return_value = [
        SimpleNamespace(le_lat=50.0, le_lon=3.0, he_lat=50.0, he_lon=3.02,
                        width_ft=150.0, le_ident="09", he_ident="27")]
    navdb.runway_strip.side_effect = lambda ident, runway: (
        {"09": "09/27", "27": "09/27"}.get(runway, runway))
    navdb.runway_at.side_effect = (
        lambda ident, lat, lon, heading=None:
            "09" if abs(lat - 50.0) < 0.005 and 3.0 <= lon <= 3.02 else "")
    return navdb


def _brain(navdb, layout, *, occupied=False, arrival_s=1e9, on_strip=()):
    from meridianatc.atc.controller import ControllerBrain

    brain = ControllerBrain(navdb, Phraseology())
    brain.staffed = lambda station: True
    brain.ground_layout = lambda ident: layout
    brain.traffic = Mock()
    brain.traffic.runway_occupied = (
        lambda now, ignore=None, runway="": occupied)
    brain.traffic._nearest_arrival_s = (
        lambda flight, runway="": arrival_s)
    brain.traffic.ground_occupancy = lambda lay: (list(on_strip), set())
    return brain


def _session(crossings=("09",)):
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.taxi_crossings = list(crossings)
    return session


GROUND = Station("TEST", "GND", 121.7, "Ground")

# 100 m short of the runway centreline, inside the decision distance.
HOLD = AircraftState(latitude=50.10, longitude=3.010, on_ground=True)
# Far enough out that nothing is decided yet.
AWAY = AircraftState(latitude=50.50, longitude=3.010, on_ground=True)
# On the strip itself.
ON = AircraftState(latitude=50.001, longitude=3.010, on_ground=True)
# Just off the far side of it, on the field.
OFF = AircraftState(latitude=49.90, longitude=3.010, on_ground=True)


def test_cross_runway_intent():
    parser = IntentParser()
    assert parser.parse("request cross runway").intent \
        is Intent.REQUEST_CROSS_RUNWAY
    assert parser.parse("crossing the runway").intent \
        is Intent.REQUEST_CROSS_RUNWAY
    assert parser.parse("request permission to cross").intent \
        is Intent.REQUEST_CROSS_RUNWAY


@pytest.mark.parametrize("language,text", [
    ("es", "solicito cruzar la pista"),
    ("de", "erbitte Bahnquerung"),
    ("it", "chiedo di attraversare la pista"),
    ("pt", "peco para atravessar a pista"),
])
def test_cross_runway_intent_local(language, text):
    assert IntentParser().parse(text, language=language).intent \
        is Intent.REQUEST_CROSS_RUNWAY


def test_cross_runway_intent_fr():
    parsed = IntentParser().parse("demande a traverser la piste",
                                  language="fr")
    assert parsed.intent is Intent.REQUEST_CROSS_RUNWAY


def test_crossing_session_state():
    session = _session(("09", "27"))
    assert session.pending_crossing() == "09"
    session.grant_crossing("09")
    assert session.crossing_granted == "09"
    assert session.cleared_runway == "09"
    crossed = session.spend_crossing()
    assert crossed == "09"
    assert session.crossing_granted == ""
    assert session.cleared_runway == ""
    assert session.pending_crossing() == "27"
    # The next runway is what the pilot now has to hold short of.
    assert session.hold_short_of == "27"
    session.spend_crossing()
    assert session.pending_crossing() == ""
    assert session.hold_short_of == ""


def test_crossing_clearance_grants_when_clear():
    navdb, layout = _navdb(), _layout()
    brain = _brain(navdb, layout)
    session = _session()
    p = brain.for_station(GROUND)
    reply = brain.crossing_clearance(session, GROUND, p, HOLD)
    assert reply is not None
    assert "cross" in reply.text.lower()
    assert "vacat" in reply.text.lower()
    assert session.crossing_granted == "09"
    assert session.cleared_runway == "09"
    # A crossing is a runway clearance: a readback of it is owed.
    assert session.pending is not None
    assert session.pending.kind == "cross_runway"
    assert "runway" in session.pending.owed_items
    # And the grant is not said again every look.
    assert brain.crossing_clearance(session, GROUND, p, HOLD) is None
    # Asked again, the clearance is the answer again.
    repeat = brain.crossing_clearance(session, GROUND, p, HOLD, asked=True)
    assert repeat is not None and "cross" in repeat.text.lower()


def test_crossing_clearance_waits_for_the_hold():
    navdb, layout = _navdb(), _layout()
    brain = _brain(navdb, layout)
    session = _session()
    p = brain.for_station(GROUND)
    assert brain.crossing_clearance(session, GROUND, p, AWAY) is None
    # Asked, it answers wherever the aircraft is.
    reply = brain.crossing_clearance(session, GROUND, p, AWAY, asked=True)
    assert reply is not None and "cross" in reply.text.lower()


def test_crossing_clearance_refuses_for_arrival_inside_window():
    navdb, layout = _navdb(), _layout()
    brain = _brain(navdb, layout, arrival_s=10.0)
    session = _session()
    session.departure_runway = "09"
    p = brain.for_station(GROUND)
    reply = brain.crossing_clearance(session, GROUND, p, HOLD)
    assert reply is not None
    assert "hold short" in reply.text.lower()
    assert session.crossing_granted == ""


def test_crossing_clearance_refuses_while_occupied():
    navdb, layout = _navdb(), _layout()
    brain = _brain(navdb, layout, occupied=True)
    session = _session()
    session.departure_runway = "09"
    reply = brain.crossing_clearance(
        session, GROUND, brain.for_station(GROUND), HOLD)
    assert "hold short" in reply.text.lower()


def test_crossing_clearance_refuses_with_somebody_on_another_runway():
    navdb, layout = _navdb(), _layout()
    # A crossing of a runway that is not in use: somebody on the strip is
    # still somebody on the strip.
    brain = _brain(navdb, layout, on_strip=[(3010.0, 50000.0, 30.0)])
    session = _session()
    session.departure_runway = "24"
    reply = brain.crossing_clearance(
        session, GROUND, brain.for_station(GROUND), HOLD)
    assert "hold short" in reply.text.lower()


def test_crossing_clearance_refuses_when_the_picture_is_unknown():
    navdb, layout = _navdb(), _layout()
    brain = _brain(navdb, layout)
    brain.traffic = None
    session = _session()
    reply = brain.crossing_clearance(
        session, GROUND, brain.for_station(GROUND), HOLD)
    assert "hold short" in reply.text.lower()


def test_crossing_watch_lifecycle():
    from meridianatc.atc.watch import CrossingWatch

    navdb, layout = _navdb(), _layout()
    brain = _brain(navdb, layout)
    watch = CrossingWatch(navdb, brain)
    session = _session()
    # Approaching the hold: granted.
    reply = watch.look(session, GROUND, HOLD, AWAY)
    assert reply is not None and "cross" in reply.text.lower()
    assert session.crossing_granted == "09"
    # On it with the clearance: nothing to say.
    session.occupying_runway = "09"
    assert watch.look(session, GROUND, ON, HOLD) is None
    # Off the far side: spent, and the limit is gone with the last crossing.
    session.occupying_runway = ""
    assert watch.look(session, GROUND, OFF, ON) is None
    assert session.pending_crossing() == ""
    assert session.cleared_runway == ""


def test_crossing_watch_does_not_spend_a_crossing_when_the_aircraft_turns_back():
    from meridianatc.atc.watch import CrossingWatch

    navdb, layout = _navdb(), _layout()
    brain = _brain(navdb, layout, occupied=True)
    watch = CrossingWatch(navdb, brain)
    session = _session()
    # The approach side is seen before the crossing, then traffic reopens.
    assert watch.look(session, GROUND, HOLD, AWAY) is not None
    session.grant_crossing("09")
    session.occupying_runway = "09"
    assert watch.look(session, GROUND, ON, HOLD) is None
    session.occupying_runway = ""

    # Back off the hold side: the runway is still ahead, not behind.
    assert watch.look(session, GROUND, HOLD, ON) is None
    assert session.pending_crossing() == "09"
    assert session.crossing_granted == ""
    assert session.cleared_runway == ""


def test_crossing_watch_calls_an_uncleared_entry_once():
    from meridianatc.atc.watch import CrossingWatch

    navdb, layout = _navdb(), _layout()
    brain = _brain(navdb, layout)
    watch = CrossingWatch(navdb, brain)
    session = _session()
    session.occupying_runway = "09"
    reply = watch.look(session, GROUND, ON, HOLD)
    assert reply is not None and "hold position" in reply.text.lower()
    # Once, not every poll.
    assert watch.look(session, GROUND, ON, ON) is None


@pytest.mark.parametrize("language", ["en", "fr", "es", "de", "it", "pt"])
def test_report_vacated_in_every_book(language):
    from meridianatc.atc.language import phraseology_for

    assert phraseology_for(language, "TEST").report_vacated().strip()


def test_invented_taxi_never_holds_short_of_the_same_strip():
    """At a one-runway field "hold short of the reciprocal" was really the
    departure runway itself, so the FO was bound to incur on it."""
    from types import SimpleNamespace

    from meridianatc.atc.controller import ControllerBrain

    strip = SimpleNamespace()
    parallel = SimpleNamespace()
    navdb = SimpleNamespace(runway_ends=lambda ident: [
        ("03", 28.0, strip), ("21", 208.0, strip)])
    brain = SimpleNamespace(navdb=navdb)
    assert ControllerBrain._hold_short_candidate(
        brain, SimpleNamespace(ident="X"), "03") == ""

    navdb.runway_ends = lambda ident: [
        ("08R", 78.0, strip), ("26L", 258.0, strip),
        ("08L", 78.0, parallel), ("26R", 258.0, parallel)]
    assert ControllerBrain._hold_short_candidate(
        brain, SimpleNamespace(ident="X"), "08R") == "08L"


def test_first_officer_crossing_request_is_parsed_as_one():
    from meridianatc.atc.pilot import PilotSpeech

    aircraft = Aircraft(parse_callsign("N123"))
    said = PilotSpeech("en", Phraseology()).request_cross_runway(
        aircraft, "Ground", "09")
    assert IntentParser().parse(said, "en").intent \
        is Intent.REQUEST_CROSS_RUNWAY


def test_invented_hold_short_can_be_crossed_when_asked_without_a_layout():
    """No-sim has no taxiway graph. The controller still has the published
    strip, so a pilot held short on an invented route can be cleared across."""
    navdb, layout = _navdb(), _layout()
    brain = _brain(navdb, layout)
    brain.ground_layout = None
    session = _session(())
    session.hold_short_of = "09"
    reply = brain.crossing_clearance(
        session, GROUND, brain.for_station(GROUND), AWAY, asked=True)
    assert reply is not None and "cross" in reply.text.lower()
    assert session.crossing_granted == "09"
