"""The tower's airborne conflict watch: one turn, held, released once."""

from types import SimpleNamespace

from meridianatc.atc.traffic.x_instructions_and_being import (
    _Instructions_and_being,
)
from meridianatc.atc.traffic.x_the_tower_watching import _The_tower_watching


class _Tower(_The_tower_watching):
    def __init__(self, flights):
        self.flights = flights
        self.user = None
        self.said = []

    def _already_told(self, flight, kind):
        return False

    def _exchange(self, flight, now, position, *, controller, pilot_back,
                  command):
        self.said.append(command)
        kind, values = command
        if kind == "heading":
            flight.assigned_heading = values["heading"]
            flight.heading = values["heading"]
        elif kind == "resume":
            flight.assigned_heading = None


def _flight(written, lon, heading, lat=40.0):
    return SimpleNamespace(
        written=written, latitude=lat, longitude=lon, altitude_ft=3000.0,
        heading=heading, ground_speed_kt=200.0, done=False, on_ground=False,
        phase="downwind", position="TWR", assigned_heading=None,
        assigned_speed_kt=None)


def test_the_avoiding_vector_is_held_until_the_meeting_is_past():
    """The turn used to be answered with "resume own navigation" on the
    next look -- the new heading met nobody -- which pointed the flight
    back at the conflict, to be turned again."""
    mine = _flight("CS-AAA", -8.0, 90.0)
    other = _flight("CS-BBB", -7.92, 270.0)
    tower = _Tower([mine, other])
    tower._watch_conflict(mine, 100.0)
    assert [kind for kind, _ in tower.said] == ["heading"]
    assert tower.said[0][1]["turn"] == "right"
    assert mine._avoid_from == 90.0

    # On the vector, clear of them -- but the track left still meets.
    tower._watch_conflict(mine, 101.0)
    tower._watch_conflict(mine, 102.0)
    assert [kind for kind, _ in tower.said] == ["heading"]

    # Behind and opening: released, once, heading only.
    other.longitude = -8.1
    tower._watch_conflict(mine, 103.0)
    tower._watch_conflict(mine, 104.0)
    assert tower.said[1:] == [("resume", {"only": "heading"})]
    assert mine._avoid_key == ""


def test_head_on_a_little_right_of_the_nose_is_still_turned_right():
    mine = _flight("CS-AAA", -8.0, 90.0)
    other = _flight("CS-BBB", -7.92, 270.0, lat=39.9995)
    tower = _Tower([mine, other])
    tower._watch_conflict(mine, 100.0)
    assert tower.said[0][1]["turn"] == "right"


def test_resume_gives_back_only_what_it_names():
    """"Resume normal speed" took an avoiding heading away with it, and
    "resume own navigation" a spacing speed."""
    apply = _Instructions_and_being._carry_out
    flight = SimpleNamespace(assigned_heading=130.0, assigned_speed_kt=160.0)
    apply(SimpleNamespace(), flight,
          SimpleNamespace(kind="resume", values={"only": "speed"}), 0.0)
    assert flight.assigned_heading == 130.0
    assert flight.assigned_speed_kt is None

    flight = SimpleNamespace(assigned_heading=130.0, assigned_speed_kt=160.0)
    apply(SimpleNamespace(), flight,
          SimpleNamespace(kind="resume", values={"only": "heading"}), 0.0)
    assert flight.assigned_heading is None
    assert flight.assigned_speed_kt == 160.0


def test_a_slow_climb_towards_the_level_is_not_a_deviation():
    """Converging at two hundred and fifty a minute -- a light single
    high up, or anybody's last few hundred feet -- was challenged as
    "verify altitude": only more than three hundred counted."""
    from unittest.mock import Mock

    from meridianatc.atc.watch import DeviationAssistWatch
    watch = DeviationAssistWatch(Mock(), Mock())
    watch._magnetic = lambda state: 0.0
    session = SimpleNamespace(assigned_altitude_ft=9000.0,
                              assigned_heading=None, assigned_speed_kt=None)

    def off(altitude, vs):
        return "altitude" in watch._off_now(session, SimpleNamespace(
            altitude_ft=altitude, vertical_speed_fpm=vs, on_ground=False,
            indicated_airspeed_kt=0.0))

    assert not off(8000.0, 250.0)
    assert not off(9240.0, -300.0)
    assert off(8000.0, 0.0)
    assert off(8000.0, -250.0)
    assert not off(8900.0, 0.0)


def test_a_gusty_slow_climb_is_still_converging():
    """The 2026-11-20 climb: a Baron averaging three hundred a minute to
    its cleared nine thousand, dipping under a hundred in the gusts, was
    called for "verify altitude" every minute of the way. The trend
    counts, not the gust."""
    from unittest.mock import Mock

    from meridianatc.atc.controller import ControllerBrain
    from meridianatc.atc.phraseology import Aircraft, Phraseology
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.atc.watch import DeviationAssistWatch
    from meridianatc.navdata.db import Station
    from meridianatc.sim.state import AircraftState

    brain = ControllerBrain(Mock(), Phraseology())
    brain.staffed = lambda station: True
    watch = DeviationAssistWatch(Mock(), brain)
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.assigned_altitude_ft = 9000.0
    station = Station("TEST", "APP", 124.5, "Approach")
    session.establish(station)
    alt = 6000.0
    for _ in range(120):
        alt += 10.0
        state = AircraftState(altitude_ft=alt, on_ground=False,
                              latitude=42.0, longitude=-8.0,
                              vertical_speed_fpm=600.0)
        watch._vs_at -= 0.5
        assert watch.look(session, station, state) is None
        state.vertical_speed_fpm = 30.0
        watch._vs_at -= 0.5
        assert watch.look(session, station, state) is None
    assert "altitude" not in watch._seen


def test_a_short_approach_read_back_tightens_the_base_turn():
    """"Make short approach" was said, read back, and the circuit flown
    exactly as wide as before: no command of that kind was carried out."""
    from meridianatc.atc.flying import Command

    tower = _Tower([])
    tower.random = SimpleNamespace(uniform=lambda a, b: a)
    flight = SimpleNamespace(
        performance=SimpleNamespace(approach_kt=65.0), held_since=0.0,
        arriving=True, extending=False)
    normal = tower._base_nm(flight)
    _Instructions_and_being._carry_out(
        tower, flight, Command("short_approach", {}), 100.0)
    assert flight.tight_base
    assert tower._base_nm(flight) < normal - 0.5
    assert tower._base_nm(flight) > tower._stable_by_nm(flight)


def test_a_speed_resume_in_progress_does_not_swallow_the_headings_release():
    """With "resume normal speed" still being read back the conflict's
    books were closed and the avoiding vector never lifted."""
    mine = _flight("AAA1", -8.0, 90.0)
    mine.assigned_heading = 120.0
    mine._avoid_key = "BBB2"
    mine._avoid_from = None
    tower = _Tower([mine])
    busy = [True]
    tower._already_told = lambda flight, kind: kind == "resume" and busy[0]
    tower._watch_conflict(mine, 100.0)
    assert mine._avoid_key == "BBB2" and not tower.said
    busy[0] = False
    tower._watch_conflict(mine, 101.0)
    assert tower.said == [("resume", {"only": "heading"})]
    assert mine._avoid_key == ""
