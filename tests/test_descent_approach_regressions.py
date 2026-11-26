"""Regressions for the descent/approach defects read off a live log:

- the check-in named the *cleared* level, not the one flown;
- the throttle servo never brought the power back in the descent, so a
  Cessna rode ninety-odd percent the whole way down and dove at the end;
- a level held on the altimeter was judged against true MSL -- a
  hundred-plus feet apart on a QNH-1018 day;
- a vector into the cone was abandoned for the intercept while the
  watch still expected it flown;
- the approach flap and the gear went down three thousand feet up and
  ten miles out, far ahead of need.
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import pytest


def _engine_for_check_in():
    from meridianatc.atc.phraseology import Phraseology
    from meridianatc.engine.core import Engine
    eng = Engine.__new__(Engine)
    eng.session = SimpleNamespace(
        assigned_altitude_ft=9000.0, pending=None, handoff_pending=None,
        aircraft=SimpleNamespace(
            spoken=lambda abbreviated=False: "x-ray golf sierra"),
        phase=None)
    eng.station = SimpleNamespace(
        ident="LEST", callsign_in=lambda spoken: "Santiago")
    eng._current_station = lambda: eng.station
    eng._language_for = lambda ident, last: "en"
    eng.brain = SimpleNamespace(
        for_station=lambda station, spoken: Phraseology())
    eng.last_language = "en"
    return eng


def test_check_in_reports_the_level_flown_not_the_cleared_one():
    """Through one thousand for nine, the call is "climbing one
    thousand" -- not "flight level niner zero", which invited four
    verify-altitude calls for a level never claimed."""
    eng = _engine_for_check_in()
    eng.state = SimpleNamespace(
        vertical_speed_fpm=1200.0, indicated_altitude_ft=1100.0,
        altitude_ft=970.0)
    text, why = eng.first_officer_call("check_in")
    assert why == ""
    assert "one thousand" in text
    assert "niner" not in text


def test_check_in_level_says_the_altimeters_level():
    """Nearly level, the report is the level the altimeter shows."""
    eng = _engine_for_check_in()
    eng.session.assigned_altitude_ft = 6000.0
    eng.state = SimpleNamespace(
        vertical_speed_fpm=80.0, indicated_altitude_ft=6020.0,
        altitude_ft=5890.0)
    text, why = eng.first_officer_call("check_in")
    assert why == ""
    assert "six thousand" in text


def _fo_descent():
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.state.phase = AutonomousPhase.DESCENT
    fo.session = SimpleNamespace(assigned_altitude_ft=6000.0)
    fo.throttle_writes = []
    fo.sim = None
    fo._dir = SimpleNamespace(
        sim=None,
        throttle=lambda lever: fo.throttle_writes.append(lever))
    fo._last_lever = 0.92
    fo.config = SimpleNamespace(
        immersion=SimpleNamespace(autonomous_pid=False))
    fo._altitude_target = lambda state: 6000.0
    fo._engine_limit_cap = lambda state: 1.0
    return fo


def _airspeed_state(**kw):
    base = dict(
        on_ground=False, engines_running=True, autothrottle=False,
        indicated_airspeed_kt=105.0, throttle_pct=92.0,
        altitude_ft=7900.0, indicated_altitude_ft=7900.0,
        barber_pole_kt=0.0, mach_max_operate=0.0, barber_pole_mach=0.0,
        airspeed_mach=0.0)
    base.update(kw)
    return SimpleNamespace(**base)


def test_a_descent_above_its_level_brings_the_lever_back():
    """Ninety-two percent all the way down was the sawtooth: above the
    cleared level with the speed on target, descent power is idle-ish,
    and the pitch-vs owns the rate. The lever walks back at a hand's
    speed rather than jumping the travel -- each pass moves it at most
    the down rate, so the pull is measured over the ramp."""
    import time as _time
    fo = _fo_descent()
    for _ in range(4):
        fo._lever_at = _time.monotonic() - 1.0
        # The sim's lever follows the last write the way it does live.
        fo._manage_throttle(_airspeed_state(
            throttle_pct=(fo.throttle_writes[-1] * 100.0
                          if fo.throttle_writes else 92.0)), 105.0)
    assert fo.throttle_writes and fo.throttle_writes[-1] <= 0.20
    # And the walk was monotonic down, never an up-swing on the way.
    writes = fo.throttle_writes
    assert all(b <= a for a, b in zip(writes, writes[1:], strict=False))


def test_a_level_leg_in_the_descent_keeps_normal_power():
    """The cap is for losing height, not holding a step: at the cleared
    level the servo behaves as before -- a fast leg gets a normal trim,
    not descent power."""
    fo = _fo_descent()
    fo._manage_throttle(
        _airspeed_state(indicated_airspeed_kt=115.0, altitude_ft=6000.0,
                        indicated_altitude_ft=6000.0), 105.0)
    assert fo.throttle_writes[-1] > 0.50


def test_a_descent_running_slow_gets_its_power_back():
    """Speed sinking under the target lifts the cap -- the lever is the
    speed control while the pitch holds the rate."""
    fo = _fo_descent()
    fo._manage_throttle(
        _airspeed_state(indicated_airspeed_kt=88.0), 105.0)
    assert fo.throttle_writes[-1] > 0.20


def test_the_flown_level_is_the_altimeters_not_true_msl():
    """indicated over true: on a QNH-1018 day they sit a hundred feet
    apart, and the bug's level is the one the needle reads."""
    from meridianatc.atc.autonomous import AutonomousFlight
    state = SimpleNamespace(indicated_altitude_ft=6030.0,
                            altitude_ft=5900.0)
    assert AutonomousFlight._flown_altitude_ft(state) == 6030.0
    state.indicated_altitude_ft = 0.0
    assert AutonomousFlight._flown_altitude_ft(state) == 5900.0


def test_the_watch_judges_what_the_scope_shows():
    """True 5,900 with QNH 1018 reads six thousand on the scope -- level
    as cleared, no verify call earned."""
    from meridianatc.atc.watch import DeviationAssistWatch
    watch = DeviationAssistWatch.__new__(DeviationAssistWatch)
    watch._vs_avg = 0.0
    watch._vs_at = 0.0
    session = SimpleNamespace(
        assigned_altitude_ft=6000.0, assigned_heading=None,
        assigned_speed_kt=None)
    state = SimpleNamespace(
        indicated_altitude_ft=6030.0, altitude_ft=5900.0,
        vertical_speed_fpm=0.0)
    assert "altitude" not in watch._off_now(session, state)
    state.indicated_altitude_ft = 5500.0
    assert "altitude" in watch._off_now(session, state)


def _fo_for_join(assigned, out_nm, course, track, side_nm=0.1):
    """An approach's flight, parked out_nm up the final's course."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    from meridianatc.navdata.build import project
    t_lat, t_lon = 43.295, -8.377          # LECO 03
    lat, lon = project(t_lat, t_lon, course + 180.0, out_nm)
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.state.runway_length_ft = 7900.0
    fo._rejoin_path = None
    fo.session = SimpleNamespace(
        assigned_heading=assigned, assigned_altitude_ft=3000.0,
        assigned_speed_kt=None, ifr=True, pattern_direction="left")
    fo._perf = SimpleNamespace(cruise_kt=110.0)
    fo._final_frame = Mock(
        return_value=(t_lat, t_lon, course, 310.0))
    fo._approach_speed = Mock(return_value=70.0)
    fo._hold_level = Mock()
    fo._emit = Mock()
    # Shift abeam the centreline where a nonzero side is wanted.
    if side_nm:
        lat, lon = project(lat, lon, course + 90.0, side_nm)
    state = SimpleNamespace(
        latitude=lat, longitude=lon, has_position=True,
        track_true=track, heading_true=track,
        altitude_ft=3200.0, indicated_altitude_ft=3200.0,
        altitude_agl_ft=2900.0)
    return fo, state


def test_a_vector_inside_the_cone_is_still_flown():
    """Inside the final's cone but not yet joined, "right heading zero
    zero two" is still the heading -- the intercept is not jumped
    early while the controller owns the vector."""
    fo, state = _fo_for_join(assigned=2.0, out_nm=8.0, course=29.0,
                             track=2.0, side_nm=1.5)
    assert fo._join_final(state, None) is True
    fo._hold_level.assert_called_once()
    assert fo._hold_level.call_args.args[2] == 2.0


def test_the_vector_is_spent_once_the_final_is_joined():
    """On the centreline and roughly on course the vector has done its
    job -- it is released so the approach, and the watch, both move on.
    """
    fo, state = _fo_for_join(assigned=2.0, out_nm=8.0, course=29.0,
                             track=29.0, side_nm=0.0)
    assert fo._join_final(state, None) is False
    assert fo.session.assigned_heading is None


def test_a_vector_that_cannot_intercept_is_released():
    """An assigned heading past sixty off the final's course never
    joins -- the final is flown instead of a vector flown forever."""
    fo, state = _fo_for_join(assigned=150.0, out_nm=8.0, course=29.0,
                             track=90.0, side_nm=1.5)
    assert fo._join_final(state, None) is False
    assert fo.session.assigned_heading is None


def _engine_for_reports():
    """An engine answered the tower's verify calls: same bones as the
    check-in fixture, with the session carrying the flags "say your
    intentions" and "verify ..." read off."""
    from meridianatc.atc.phraseology import Phraseology
    from meridianatc.engine.core import Engine
    eng = Engine.__new__(Engine)
    eng.session = SimpleNamespace(
        assigned_altitude_ft=4000.0, assigned_heading=2.0,
        assigned_speed_kt=90.0, pending=None, handoff_pending=None,
        going_around=False, cleared_landing=False, arrival_runway="03",
        aircraft=SimpleNamespace(
            spoken=lambda abbreviated=False: "x-ray golf sierra",
            kept_short=False),
        phase=None)
    eng.station = SimpleNamespace(
        ident="LECO", callsign_in=lambda spoken: "Tower")
    eng._current_station = lambda: eng.station
    eng._language_for = lambda ident, last: "en"
    eng.brain = SimpleNamespace(
        for_station=lambda station, spoken: Phraseology())
    eng.last_language = "en"
    return eng


def test_verify_altitude_reports_the_level_flown_and_the_fix():
    """1000 low and still coming down, the answer is not "four thousand"
    -- it is what the needle shows plus the descent toward the cleared
    level."""
    eng = _engine_for_reports()
    eng.state = SimpleNamespace(
        vertical_speed_fpm=-800.0, indicated_altitude_ft=3200.0,
        altitude_ft=3100.0)
    text, why = eng.first_officer_call("verify_altitude")
    assert why == ""
    assert "three thousand" in text and "four thousand" in text


def test_verify_altitude_uses_the_clearances_direction_not_the_gusts():
    """The live LEVX-LECO scan: momentarily descending while still
    cleared up to FL170 produced the impossible answer "descend FL170".
    The report names the clearance's direction from the present level."""
    eng = _engine_for_reports()
    eng.session.assigned_altitude_ft = 17000.0
    eng.state = SimpleNamespace(
        vertical_speed_fpm=-800.0, indicated_altitude_ft=13500.0,
        altitude_ft=13400.0)
    text, why = eng.first_officer_call("verify_altitude")
    assert why == ""
    assert "one three thousand five hundred" in text
    assert "climb" in text and "one seven" in text
    assert "descend" not in text


def test_verify_altitude_level_reports_the_level():
    eng = _engine_for_reports()
    eng.state = SimpleNamespace(
        vertical_speed_fpm=0.0, indicated_altitude_ft=2900.0,
        altitude_ft=2800.0)
    text, why = eng.first_officer_call("verify_altitude")
    assert why == ""
    assert "two thousand nine hundred" in text.replace("niner", "nine")


def test_verify_heading_and_speed_report_the_assigned_values():
    eng = _engine_for_reports()
    eng.state = SimpleNamespace(heading_magnetic=40.0,
                                indicated_airspeed_kt=110.0)
    text, _ = eng.first_officer_call("verify_heading")
    assert "zero zero two" in text or "heading" in text.lower()
    text, _ = eng.first_officer_call("verify_speed")
    assert "niner zero" in text or "90" in text


def test_intentions_on_a_cleared_landing_names_the_runway():
    eng = _engine_for_reports()
    eng.session.cleared_landing = True
    eng.state = SimpleNamespace(vertical_speed_fpm=-500.0)
    text, why = eng.first_officer_call("intentions")
    assert why == ""
    assert "land" in text.lower() and "three" in text


def test_intentions_mid_vector_names_the_vector():
    eng = _engine_for_reports()
    eng.state = SimpleNamespace(vertical_speed_fpm=-600.0)
    text, why = eng.first_officer_call("intentions")
    assert why == ""
    assert "zero zero two" in text
    assert "four thousand" in text


def test_the_deviation_watch_stamps_the_query_for_the_crew():
    """A "verify altitude" that nobody heard was a tree falling in the
    forest: the call is stamped on the session so the crew's ear --
    _answer_the_queries -- has something to answer."""
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
    session.assigned_altitude_ft = 5000.0
    station = Station("TEST", "APP", 124.5, "Approach")
    session.establish(station)
    state = AircraftState(altitude_ft=4000.0, on_ground=False,
                          latitude=40.0, longitude=-75.0)
    watch.look(session, station, state)
    watch._seen["altitude"][0] -= watch.SETTLE_S + 1.0
    reply = watch.look(session, station, state)
    assert reply is not None and "verify" in reply.text.lower()
    assert session.deviation_query == "altitude"
    assert session.deviation_query_at > 0.0


def _fo_for_answers():
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.state.last_radio_at = 0.0
    fo._fo_say = Mock(return_value=True)
    return fo


def test_the_crew_answers_the_deviation_query():
    """The stamp the watch left is heard, answered once, and spent -- a
    second call of the same kind is a new stamp, not a repeat."""
    import time as _t
    fo = _fo_for_answers()
    now = _t.monotonic()
    session = SimpleNamespace(
        deviation_query="altitude", deviation_query_at=now,
        assist_offered_at=0.0,
        assistance_offered=lambda what="": False)
    state = SimpleNamespace(on_ground=False)
    fo._answer_the_queries(state, session, now)
    fo._fo_say.assert_called_once_with("verify_altitude")
    assert session.deviation_query == ""
    # The same stamp back on the session is the call already answered,
    # not a new question.
    session.deviation_query = "altitude"
    fo._fo_say.reset_mock()
    fo._answer_the_queries(state, session, now + 5.0)
    fo._fo_say.assert_not_called()
    # A fresh stamp -- the watch called it again -- is a new question.
    session.deviation_query_at = now + 5.0
    fo._answer_the_queries(state, session, now + 6.0)
    fo._fo_say.assert_called_once_with("verify_altitude")


def test_the_crew_answers_say_your_intentions():
    import time as _t
    fo = _fo_for_answers()
    now = _t.monotonic()
    session = SimpleNamespace(
        deviation_query="", deviation_query_at=0.0,
        assist_offered="flight", assist_offered_at=now,
        assistance_offered=lambda what="": what == "flight",
        answer_assistance=Mock(return_value="flight"))
    state = SimpleNamespace(on_ground=False)
    fo._answer_the_queries(state, session, now)
    fo._fo_say.assert_called_once_with("intentions")
    session.answer_assistance.assert_called_once()


def _runway_watch_fixture():
    """A tower mid-change 03 -> 21, and a flight cleared to land 03."""
    from meridianatc.atc.controller import Reply
    from meridianatc.atc.phraseology import Aircraft, Phraseology
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.atc.watch import RunwayWatch
    from meridianatc.navdata.db import Station

    def go_around(session, station, weather, p, airport, told=False):
        session.cleared_landing = False
        session.going_around = True
        return Reply("go around, fly runway heading, climb to "
                     "four thousand", station)

    brain = SimpleNamespace(
        realism=SimpleNamespace(runway_change=True),
        staffed=lambda station: True,
        arrival_runway=lambda airport, weather: "21",
        departure_runway=lambda airport, weather: "21",
        runway_changes={"TEST": ("03", "21")},
        for_station=lambda station, language: Phraseology(),
        station_name=lambda station, p: "Tower",
        go_around=go_around)
    airport = SimpleNamespace(ident="TEST")
    navdb = SimpleNamespace(airport=lambda ident: airport)
    weather = SimpleNamespace(qnh_hpa=1013.0, altimeter_inhg=29.92)
    watch = RunwayWatch(navdb, brain,
                        weather_for=lambda ident: weather)
    station = Station("TEST", "TWR", 118.1, "Tower")
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.establish(station)
    return watch, session, station


def test_a_cleared_landing_is_sent_round_to_the_new_runway():
    """Cleared to land 03 when the tower's broadcast makes 21 the runway
    in use: the strip the clearance was written for no longer passes the
    wind, so the flight is sent around and worked back for 21 -- the
    call the tower owes the last arrival of the old direction."""
    watch, session, station = _runway_watch_fixture()
    session.arrival_runway = "03"
    session.cleared_landing = True
    state = SimpleNamespace(on_ground=False)
    reply = watch.look(session, station, state)
    assert reply is not None
    assert "runway in use two one" in reply.text.lower() \
        or "runway two one" in reply.text.lower()
    assert "go around" in reply.text.lower()
    assert not session.cleared_landing
    assert session.going_around
    assert session.arrival_runway == "21"


def test_a_lined_up_departure_keeps_the_runway_it_was_given():
    watch, session, station = _runway_watch_fixture()
    session.departure_runway = "03"
    session.lined_up = True
    state = SimpleNamespace(on_ground=True)
    reply = watch.look(session, station, state)
    assert reply is not None
    assert "go around" not in reply.text.lower()
    assert session.departure_runway == "03"


def _legpath_recipe(offset_course=114.0, runway_course=34.0):
    """An RNP-style recipe: published legs ending at a MAPt off the
    runway's axis, the way LECO's RNP 03 is drawn."""
    from meridianatc.atc import approach_flight
    from meridianatc.navdata.build import project
    t_lat, t_lon = 43.295, -8.377                    # LECO 03
    map_lat, map_lon = project(t_lat, t_lon, offset_course + 180.0, 2.0)
    mid_lat, mid_lon = project(map_lat, map_lon, offset_course + 180.0, 4.0)
    leg_path = ((mid_lat, mid_lon, 2000.0, 0.0),
                (map_lat, map_lon, 1500.0, 0.0))
    return approach_flight.ApproachRecipe(
        ident="LECO", name="RNP 03", family="rnav",
        approach_runway="03", landing_runway="03", line="LNAV",
        minima_ft=450.0, circle_minima_ft=450.0,
        final_course=runway_course,
        threshold_lat=t_lat, threshold_lon=t_lon, field_elev_ft=310.0,
        faf_lat=mid_lat, faf_lon=mid_lon, faf_alt_ft=2000.0,
        missed_alt_ft=2800.0, leg_path=leg_path,
        map_lat=map_lat, map_lon=map_lon, map_course=offset_course,
        offset_final=abs((offset_course - runway_course + 540.0) % 360.0
                         - 180.0) > 25.0)


def test_the_published_legs_are_the_flown_path():
    """Where the feed carries the legs, the path the FO steers is the
    chart's -- not a synthetic gate-to-threshold straight-in. The legs
    end at the missed-approach point; the runway is appended so the
    steering keeps going forward instead of pulling back to the fix."""
    from meridianatc.atc import approach_flight
    recipe = _legpath_recipe()
    path = approach_flight.legs(recipe)
    assert path[0] == (recipe.leg_path[0][0], recipe.leg_path[0][1])
    assert (recipe.map_lat, recipe.map_lon) in path
    assert (recipe.threshold_lat, recipe.threshold_lon) in path
    assert path[-1] != (recipe.map_lat, recipe.map_lon)


def test_an_offset_final_says_it_is_one():
    """The eighty degrees between LECO's final and its runway is what
    marks the approach as ending in a visual manoeuvre."""
    recipe = _legpath_recipe()
    assert recipe.offset_final is True
    aligned = _legpath_recipe(offset_course=40.0, runway_course=34.0)
    assert aligned.offset_final is False


def test_the_final_frame_is_the_missed_approach_point():
    """Established on a published final means established on its own
    course -- the runway's extended centreline is never flown."""
    from meridianatc.atc.autonomous import AutonomousFlight
    fo = AutonomousFlight.__new__(AutonomousFlight)
    recipe = _legpath_recipe()
    frame = fo._final_frame(SimpleNamespace(has_position=False), recipe)
    assert frame == (recipe.map_lat, recipe.map_lon,
                     recipe.map_course, recipe.field_elev_ft)


def test_the_leg_profile_descends_to_the_next_constraint():
    """Above the fix ahead's constraint the profile sinks; level with it,
    it holds -- the chart's steps, not a three-degree guess."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    recipe = _legpath_recipe()
    from meridianatc.navdata.build import project
    lat, lon = project(recipe.leg_path[0][0], recipe.leg_path[0][1],
                       recipe.map_course + 180.0, 0.5)
    high = SimpleNamespace(latitude=lat, longitude=lon, altitude_ft=2200.0,
                           ground_speed_kt=90.0)
    on_it = SimpleNamespace(latitude=lat, longitude=lon, altitude_ft=1510.0,
                            ground_speed_kt=90.0)
    assert fo._recipe_vertical(high, recipe) < -200.0
    assert abs(fo._recipe_vertical(on_it, recipe)) < 100.0


def test_an_offset_final_circles_from_the_map():
    """The instrument segment ends at the MAPt: the visual manoeuvre
    begins there even though the aeroplane is still above the MDA."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.session = SimpleNamespace(aircraft=SimpleNamespace(type="C172"))
    fo.navdb = None
    fo._emit = Mock()
    recipe = _legpath_recipe()
    # At the MAPt, still at the OCA -- the threshold triggers cannot see
    # it (eighty degrees off and past the point) but the MAPt can.
    state = SimpleNamespace(
        latitude=recipe.map_lat, longitude=recipe.map_lon,
        altitude_agl_ft=1200.0, has_position=True,
        ground_speed_kt=90.0, heading_true=recipe.map_course,
        track_true=recipe.map_course,
        distance_to=lambda a, b: 0.1 if (a, b) == (
            recipe.map_lat, recipe.map_lon) else 2.5)
    from unittest.mock import patch

    from meridianatc.atc import approach_flight
    with patch.object(approach_flight, "circle_legs",
                      return_value=[(1.0, 2.0)]):
        assert fo._circling(state, recipe) is True
    assert fo.state.circling is True


def test_past_the_map_the_profile_descends_to_the_runway():
    """Once the published legs run out the profile is the runway's
    three degrees again -- the missed-approach point's level is not
    flown all the way over the threshold."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    recipe = _legpath_recipe()
    # A step past the MAPt on the stretch to the threshold, still high.
    from meridianatc.navdata.build import bearing_deg, project
    toward = bearing_deg(recipe.map_lat, recipe.map_lon,
                         recipe.threshold_lat, recipe.threshold_lon)
    lat, lon = project(recipe.map_lat, recipe.map_lon, toward, 0.2)
    from meridianatc.navdata.build import haversine_nm
    state = SimpleNamespace(
        latitude=lat, longitude=lon,
        altitude_ft=1500.0, ground_speed_kt=90.0, has_position=True,
        distance_to=lambda a, b: haversine_nm(lat, lon, a, b))
    vs = fo._recipe_vertical(state, recipe)
    assert vs < -100.0


def test_the_circle_steering_can_only_move_on():
    """Nearest-segment steering flips between the downwind and base
    legs on the teardrop; the kept index makes the pattern's progress
    one-way, so the orbit the report caught cannot repeat."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.sim = None
    fo._dir = SimpleNamespace(localizer=Mock(), approach=Mock(), sim=None)
    fo.state.ap_apr_mode = False
    fo._apply_flight_director = Mock()
    fo._magnetic = Mock(side_effect=lambda h, s: h)
    fo._vapp = Mock(return_value=70.0)
    fo._emit = Mock()
    fo.state.circle_path = [(42.0, -8.0), (42.0, -7.9), (42.05, -7.9),
                            (42.05, -8.0)]
    fo.state.circle_index = 2      # on the base leg already
    recipe = _legpath_recipe()
    # Physically back near the downwind start -- without the index the
    # nearest segment is behind and the pattern would re-fly itself.
    state = SimpleNamespace(latitude=42.0, longitude=-8.0,
                            altitude_ft=900.0, has_position=True)
    assert fo._fly_circle(state, recipe) is True
    assert fo.state.circle_index >= 2


def test_a_thirty_degree_cut_inside_the_gate_is_not_an_intercept():
    """The live LECO runway-21 scan: base at 3.0 nm out and 2.8 nm
    beside the centreline was cleared straight onto the ILS. That cut
    joined well inside the slow pattern's 5 nm gate, left the Baron high
    on final, and caused the second go-around. Geometry that cannot
    reach the gate is re-sequenced instead."""
    from meridianatc.atc import vectors as vec
    from meridianatc.atc.final import Final
    from meridianatc.navdata.build import project

    final = Final("21", 43.308586, -8.3719, 211.0)
    lat, lon = project(final.lat, final.lon, (final.course + 180.0) % 360.0, 3.0)
    lat, lon = project(lat, lon, (final.course + 90.0) % 360.0, 2.8)
    leg = vec.vector(final, lat, lon, speed_kt=126.0,
                     heading_true=122.0, after="base", side=1.0)
    assert leg.leg == "downwind"
    assert leg.heading_true == (final.course + 180.0) % 360.0

    lat, lon = project(final.lat, final.lon, (final.course + 180.0) % 360.0, 7.0)
    lat, lon = project(lat, lon, (final.course + 90.0) % 360.0, 1.0)
    leg = vec.vector(final, lat, lon, speed_kt=126.0,
                     heading_true=122.0, after="base", side=1.0)
    assert leg.leg == "intercept"
    assert leg.join_nm >= vec.SLOW.gate_nm - 0.5


def _fo_for_computed_floor(out_nm=7.0, side_nm=1.0, altitude=3150.0):
    """An FO on a straight final, with the moving parts spied."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    from meridianatc.navdata.build import project
    from meridianatc.sim.state import AircraftState
    recipe = _legpath_recipe(offset_course=34.0, runway_course=34.0)
    lat, lon = project(recipe.threshold_lat, recipe.threshold_lon,
                       recipe.final_course + 180.0, out_nm)
    if side_nm:
        lat, lon = project(lat, lon, recipe.final_course + 90.0, side_nm)
    state = AircraftState(
        latitude=lat, longitude=lon, altitude_ft=altitude,
        indicated_altitude_ft=altitude, track_true=recipe.final_course,
        heading_true=recipe.final_course, ground_speed_kt=110.0,
        connected=True, source="test")
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.session = SimpleNamespace(assigned_altitude_ft=3000.0)
    sim = object()
    fo.sim = sim
    fo._dir = Mock()
    fo._dir.sim = sim
    fo._apply_flight_director = Mock()
    fo._recipe_vertical = Mock(return_value=-800.0)
    fo._approach_speed = Mock(return_value=90.0)
    fo._into_wind = lambda _state, heading: heading
    fo._magnetic = lambda heading, _state: heading
    fo._final_alt_bug = lambda elev: elev
    fo._flown_altitude_ft = lambda _state: altitude
    return fo, state, recipe


def test_maintain_until_established_is_not_descended_through_off_final():
    """"Maintain three thousand until established" still binds after the
    vector itself is spent: the computed final may be joined laterally,
    but it may not take the aeroplane below the platform level early."""
    fo, state, recipe = _fo_for_computed_floor(side_nm=1.0)
    assert not fo._established_on_final(state, recipe)
    fo._fly_computed(state, recipe)
    kwargs = fo._apply_flight_director.call_args.kwargs
    assert kwargs["altitude"] == 3000.0
    # 150 ft above the floor: no more than about 300 fpm down, so the
    # level is captured rather than swept through at the profile's rate.
    assert kwargs["vs_fpm"] == pytest.approx(-300.0)
    assert fo.session.assigned_altitude_ft == 3000.0


def test_the_until_established_level_is_spent_once_established():
    fo, state, recipe = _fo_for_computed_floor(side_nm=0.0, altitude=3000.0)
    assert fo._established_on_final(state, recipe)
    fo._fly_computed(state, recipe)
    kwargs = fo._apply_flight_director.call_args.kwargs
    assert kwargs["altitude"] == recipe.field_elev_ft
    assert kwargs["vs_fpm"] == -800.0
    assert fo.session.assigned_altitude_ft is None
