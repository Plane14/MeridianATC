"""Surface traffic awareness: the FO yields, the controller calls it.

Covers the corridor math (:func:`meridianatc.atc.auto_nav.traffic_ahead`),
the give-way call in ``TaxiWatch``, the autonomous flight's own yielding
and runway-occupancy checks, the holding-point stop, the backtrack, and
the gear assertions the traffic writer owes every object it draws.
"""

from __future__ import annotations

import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from meridianatc.atc import auto_nav
from meridianatc.navdata.build import haversine_nm


def _nm(a_lat, a_lon, b_lat, b_lon) -> float:
    return haversine_nm(a_lat, a_lon, b_lat, b_lon)


def _state(lat, lon, **kw):
    base = dict(latitude=lat, longitude=lon, altitude_ft=500.0,
                heading_true=0.0, ground_speed_kt=12.0, on_ground=True,
                has_position=True,
                indicated_airspeed_kt=0.0, vertical_speed_fpm=0.0)
    base.update(kw)
    state = SimpleNamespace(**base)
    state.distance_to = lambda a, b: _nm(
        state.latitude, state.longitude, a, b)
    return state


def _contact(lat, lon, **kw):
    base = dict(written="SHT123", type_code="A320", latitude=lat,
                longitude=lon, altitude_ft=500.0, heading=0.0,
                ground_speed_kt=10.0, vertical_rate_fpm=0.0,
                on_ground=True, observed=False, runway="")
    base.update(kw)
    c = SimpleNamespace(**base)
    c.describe = lambda language="en": "the Airbus"
    return c


# A straight taxi path east-west at a mid latitude, ~1 nm long.
def _path(lat=51.15, lon=-0.19, legs=6, step=0.005):
    return [(lat, lon + i * step) for i in range(legs)]


# -- traffic_ahead ------------------------------------------------------


def test_traffic_ahead_finds_head_on():
    path = _path()
    # On the route ahead, coming the other way.
    ahead = _contact(path[3][0], path[3][1], heading=270.0)
    found = auto_nav.traffic_ahead(path[0][0], path[0][1], path, [ahead],
                                   corridor_m=30.0, look_m=1500.0)
    assert found and found[0][0] is ahead


def test_traffic_ahead_ignores_behind_and_off_route():
    path = _path()
    behind = _contact(path[0][0] - 0.001, path[0][1], written="AFT1")
    offside = _contact(path[3][0] + 0.001, path[3][1], written="OFF1")
    # 0.001 deg lat ~ 110 m off the corridor; 0.0002 ~ 22 m is inside.
    near = _contact(path[3][0] + 0.0002, path[3][1], written="NEAR1")
    found = auto_nav.traffic_ahead(path[0][0], path[0][1], path,
                                   [behind, offside, near],
                                   corridor_m=30.0, look_m=1500.0)
    assert [c.written for c, _ in found] == ["NEAR1"]


def test_traffic_ahead_orders_nearest_first_and_skips_airborne():
    path = _path()
    far = _contact(path[4][0], path[4][1], written="FAR1")
    close = _contact(path[1][0], path[1][1], written="CLOSE1")
    air = _contact(path[2][0], path[2][1], written="AIR1", on_ground=False,
                   altitude_ft=3000.0)
    found = auto_nav.traffic_ahead(path[0][0], path[0][1], path,
                                   [far, close, air],
                                   corridor_m=30.0, look_m=1500.0)
    assert [c.written for c, _ in found] == ["CLOSE1", "FAR1"]
    assert found[0][1] < found[1][1]


def test_traffic_ahead_respects_look_distance():
    path = _path()
    far = _contact(path[5][0], path[5][1])
    assert auto_nav.traffic_ahead(
        path[0][0], path[0][1], path, [far],
        corridor_m=30.0, look_m=100.0) == []


# -- TaxiWatch give-way ---------------------------------------------------


def _phraseology():
    return SimpleNamespace(
        language="en",
        give_way=lambda aircraft, traffic: f"give way to {traffic}",
        continue_taxi=lambda aircraft, route=(): "continue taxi",
        taxi_off_route=lambda aircraft: "are you off route")


def _watch(brain):
    from meridianatc.atc.watch import TaxiWatch
    return TaxiWatch(Mock(), brain)


def _session(path):
    return SimpleNamespace(
        phase=__import__("meridianatc.atc.session", fromlist=["Phase"]).Phase
        .TAXI_OUT,
        taxi_path=path,
        aircraft=SimpleNamespace(language="en"))


def _station(position="GND"):
    return SimpleNamespace(position=position, callsign="Test Ground")


def test_taxi_watch_says_give_way_once_then_continue():
    path = _path()
    # On the route ~125 m ahead (0.0018 deg lon at lat 51), coming the
    # other way -- inside the 150 m call window.
    brain = Mock()
    brain.contacts = lambda: [_contact(path[0][0], path[0][1] + 0.0018,
                                     heading=270.0)]
    brain.for_station = Mock(return_value=_phraseology())
    watch = _watch(brain)
    session = _session(path)
    state = _state(*path[0])

    reply = watch.look(session, _station(), state)
    assert reply is not None and "give way" in reply.text
    # Said once while the conflict stands.
    assert watch.look(session, _station(), state) is None
    # And "continue taxi" once it is clear.
    brain.contacts = lambda: []
    reply = watch.look(session, _station(), state)
    assert reply is not None and "continue taxi" in reply.text
    assert watch.look(session, _station(), state) is None


def test_taxi_watch_quiet_for_parked_traffic_off_the_stop():
    path = _path()
    # Standing on the line ~120 m out -- in the look window but beyond
    # the ~95 m point where a parked aeroplane is genuinely in the way.
    parked = _contact(path[0][0], path[0][1] + 0.0017,
                      ground_speed_kt=0.0)
    brain = Mock()
    brain.contacts = lambda: [parked]
    brain.for_station = Mock(return_value=_phraseology())
    watch = _watch(brain)
    session = _session(path)
    assert watch.look(session, _station(), _state(*path[0])) is None


def test_taxi_watch_calls_for_parked_traffic_at_the_stop():
    path = _path()
    parked = _contact(path[0][0], path[0][1] + 0.0008,  # ~56 m
                      ground_speed_kt=0.0)
    brain = Mock()
    brain.contacts = lambda: [parked]
    brain.for_station = Mock(return_value=_phraseology())
    watch = _watch(brain)
    session = _session(path)
    reply = watch.look(session, _station(), _state(*path[0]))
    assert reply is not None and "give way" in reply.text


def test_taxi_watch_calls_for_moving_traffic_in_window():
    path = _path()
    moving = _contact(path[0][0], path[0][1] + 0.0018, heading=270.0)
    brain = Mock()
    brain.contacts = lambda: [moving]
    brain.for_station = Mock(return_value=_phraseology())
    watch = _watch(brain)
    session = _session(path)
    state = _state(*path[0])
    reply = watch.look(session, _station(), state)
    assert reply is not None and "give way" in reply.text


# -- the FO's own yielding -------------------------------------------------


def _fo(**attrs):
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.state.phase = AutonomousPhase.TAXI_OUT
    fo.state.sub = "taxi"
    fo.state.sub_entered_at = 0.0
    fo.on_event = Mock()
    fo.sim = Mock()
    fo._dir = Mock()
    fo._dir.sim = fo.sim
    fo._stop = Mock()
    fo._steer = Mock()
    fo._ground_still = Mock(return_value=False)
    fo._find_runway = Mock()
    fo._configure_for_takeoff = Mock()
    fo._run_checklist_for_phase = Mock()
    fo._held_at_split = False
    fo._perf = SimpleNamespace(taxi_kt=12.0, rotate_kt=140.0,
                               takeoff_kt_s=4.0, turn_rate=5.0,
                               cruise_kt=180.0)
    fo.TAKEOFF_TIMEOUT = 240.0
    fo.brain = SimpleNamespace(contacts=lambda: [])
    for key, value in attrs.items():
        setattr(fo, key, value)
    return fo


def _fo_session(path, **kw):
    base = dict(taxi_path=path, taxi_split_at=None, cleared_takeoff=False,
                lined_up=False, phase="",
                departure_runway="27", arrival_runway="")
    base.update(kw)
    return SimpleNamespace(**base)


def test_fo_stops_for_traffic_on_the_route():
    path = _path()
    fo = _fo()
    fo.session = _fo_session(path)
    # ~84 m ahead on the route -- inside the FO's 90 m give-way window.
    conflict = _contact(path[0][0], path[0][1] + 0.0012, heading=270.0)
    fo.brain = SimpleNamespace(contacts=lambda: [conflict])
    state = _state(path[0][0], path[0][1])
    fo._taxi_along_path(state, outbound=True)
    fo._stop.assert_called()
    fo._steer.assert_not_called()


def test_fo_continues_when_the_route_is_clear():
    path = _path()
    fo = _fo()
    fo.session = _fo_session(path)
    state = _state(path[0][0], path[0][1])
    fo._taxi_along_path(state, outbound=True)
    fo._steer.assert_called()


def test_fo_holds_at_the_bars_not_fifty_metres_short():
    path = _path()
    fo = _fo()
    fo.session = _fo_session(path)
    from meridianatc.atc.autonomous import AutonomousPhase
    # 7 m short of the last point -- a light aeroplane's nose is at the
    # bars (the datum stop is the class's nose radius plus the margin).
    end_lat, end_lon = path[-1]
    short = _state(end_lat, end_lon - 0.0001, ground_speed_kt=3.0)
    fo._taxi_along_path(short, outbound=True)
    assert fo.state.phase == AutonomousPhase.HOLDING_SHORT
    assert fo.state.sub == "" or fo.state.sub == "align"


# -- runway occupancy and the roll ----------------------------------------


def _fo_for_lineup(**attrs):
    fo = _fo(**attrs)
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.LINE_UP
    fo.state.sub = "align"
    fo.state.runway_heading = 90.0
    fo.state.runway_lat = 51.15
    fo.state.runway_lon = -0.19
    fo.state.runway_key = ("TEST", "09")
    fo.state.runway_length_ft = 8000.0
    return fo


def test_fo_waits_off_an_occupied_runway():
    fo = _fo_for_lineup()
    fo.session = _fo_session(_path())
    blocked = _contact(51.1502, -0.189, heading=90.0)
    fo.brain = SimpleNamespace(contacts=lambda: [blocked])
    fo._runway_blocked = Mock(return_value=True)
    state = _state(51.15, -0.20)
    fo._phase_line_up(state, 100.0)
    fo._steer.assert_not_called()
    fo._stop.assert_called()


def test_fo_holds_lined_up_past_five_minutes_without_dying():
    fo = _fo_for_lineup()
    fo.session = _fo_session(_path())
    fo.state.sub = "set"
    fo.state.sub_entered_at = 0.0
    fo.state.aligned = True
    fo._runway_blocked = Mock(return_value=False)
    # On the numbers, pointed down them: the wait is the clearance's.
    state = _state(51.15, -0.19, heading_true=90.0)
    fo._phase_line_up(state, 400.0)
    from meridianatc.atc.autonomous import AutonomousPhase
    assert fo.state.phase != AutonomousPhase.ERROR
    assert fo.state.sub == "set"


def test_fo_realigns_when_nudged_off_the_centreline():
    fo = _fo_for_lineup()
    fo.session = _fo_session(_path())
    fo.state.sub = "set"
    fo.state.aligned = True
    fo._runway_blocked = Mock(return_value=False)
    # Sixty metres off the centreline pointing across it: not a lineup.
    state = _state(51.1507, -0.19, heading_true=160.0)
    fo._phase_line_up(state, 100.0)
    assert fo.state.sub == "align"
    assert fo.state.aligned is False


def test_fo_does_not_call_aligned_at_the_runway_edge():
    """A 45 m strip's old gate -- inside its half-width -- called the
    Baron "aligned" ~15 m off the paint, near the edge, which is where
    the roll then started. Fifteen metres out is still steering, not a
    lineup."""
    fo = _fo_for_lineup()
    fo.state.runway_width_ft = 148.0    # LPPR's 45 m
    fo.session = _fo_session(_path())
    fo._runway_blocked = Mock(return_value=False)
    # ~15 m north of the centreline at the numbers, pointed down them.
    state = _state(51.150135, -0.19, heading_true=90.0)
    fo._phase_line_up(state, 100.0)
    assert fo.state.aligned is False
    assert fo.state.sub == "align"
    # And on the paint it does line up.
    state = _state(51.15002, -0.19, heading_true=90.0)
    fo._phase_line_up(state, 101.0)
    assert fo.state.aligned is True
    assert fo.state.sub == "set"


def test_fo_does_not_call_aligned_short_of_the_numbers():
    """On the extended centreline a hundred metres before the threshold
    is the taxiway, not a lineup -- the roll would start in the grass."""
    fo = _fo_for_lineup()
    fo.state.runway_width_ft = 148.0
    fo.session = _fo_session(_path())
    fo._runway_blocked = Mock(return_value=False)
    # ~100 m behind the numbers, dead on the centreline, runway heading.
    state = _state(51.15, -0.19144, heading_true=90.0)
    fo._phase_line_up(state, 100.0)
    assert fo.state.aligned is False


def test_fo_realigns_on_a_small_drift_in_the_hold():
    """Lined up and waiting, a nudge thirteen metres sideways used to
    keep the "set" -- the old slack was nearly half a wide runway. Now
    it is a re-align."""
    fo = _fo_for_lineup()
    fo.state.runway_width_ft = 148.0
    fo.session = _fo_session(_path())
    fo.state.sub = "set"
    fo.state.aligned = True
    fo._runway_blocked = Mock(return_value=False)
    state = _state(51.150117, -0.19, heading_true=90.0)   # ~13 m off
    fo._phase_line_up(state, 100.0)
    assert fo.state.sub == "align"
    assert fo.state.aligned is False


def test_fo_lines_up_on_the_connector_not_across_the_infield():
    """LPPR's A3: the hold sits well past the threshold, so a straight
    intercept to the centreline is the backtrack's reciprocal through
    the grass. The planner's paved connector path is the way on; the FO
    follows it instead of entering the backtrack off the pavement."""
    fo = _fo_for_lineup()
    fo._runway_blocked = Mock(return_value=False)
    # A connector from a hold south of the strip, bending north onto it,
    # ~280 m along from the threshold.
    lineup = [(51.1496, -0.1862), (51.1499, -0.1858), (51.1501, -0.1855)]
    fo.session = _fo_session(_path(), lineup_path=lineup)
    fo.state.sub_entered_at = time.monotonic()
    state = _state(*lineup[0], heading_true=90.0)
    fo._phase_line_up(state, 100.0)
    # Not the backtrack: the paved way on is still under the wheels.
    assert fo.state.sub == "align"
    fo._steer.assert_called()
    paved = auto_nav.path_steering(state.latitude, state.longitude,
                                   lineup, lookahead_m=30.0)
    assert fo._steer.call_args[0][1] == pytest.approx(paved["heading"],
                                                    abs=1.0)


def test_fo_backtracks_without_a_paved_lineup_path():
    """Same geometry with no connector route to offer: the far-along
    entry onto an eastbound strip is a backtrack, as before."""
    fo = _fo_for_lineup()
    fo._runway_blocked = Mock(return_value=False)
    fo.session = _fo_session(_path())
    fo.state.sub_entered_at = time.monotonic()
    state = _state(51.1496, -0.1862, heading_true=90.0)
    fo._phase_line_up(state, 100.0)
    assert fo.state.sub == "backtrack"
    fo._steer.assert_not_called()


def test_fo_on_the_strip_never_bounces_to_holding_short():
    """LPPR's Echo: no align clock bounces the aeroplane to a taxiway
    state once it is on the pavement -- however long the line-up takes
    it keeps steering to the line, and 'hold short' is never a stop
    mid-runway."""
    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _fo_for_lineup()
    fo._runway_blocked = Mock(return_value=False)
    fo.session = _fo_session(_path())
    fo.state.sub_entered_at = 0.0        # the align is long past any clock
    # On the pavement near the threshold, unaligned: keep driving.
    state = _state(51.1501, -0.1899, heading_true=140.0)
    fo._phase_line_up(state, 100.0)
    assert fo.state.phase == AutonomousPhase.LINE_UP
    assert fo.state.sub == "align"
    fo._steer.assert_called()


def test_fo_keeps_the_taxi_route_until_the_bars():
    """LPPR: cleared to line up while still taxiing the parallel taxiway
    ~100 m off the strip. The way to the runway is the rest of the taxi
    route to the bars -- not the diagonal over the infield that the
    intercept and the distant connector both draw."""
    fo = _fo_for_lineup()
    fo._runway_blocked = Mock(return_value=False)
    # A parallel taxiway ~100 m south of the centreline, heading east.
    taxi = [(51.1491, -0.186), (51.1491, -0.184), (51.1491, -0.182)]
    fo.session = _fo_session(taxi)
    fo.state.sub_entered_at = time.monotonic()
    state = _state(51.1491, -0.185, heading_true=90.0)
    fo._phase_line_up(state, 100.0)
    assert fo.state.sub == "align"
    fo._steer.assert_called()
    on_taxi = auto_nav.path_steering(state.latitude, state.longitude,
                                   taxi, lookahead_m=30.0)
    assert fo._steer.call_args[0][1] == pytest.approx(on_taxi["heading"],
                                                    abs=1.0)


def test_fo_does_not_enter_the_backtrack_from_the_taxiway():
    """The same place with nothing left to drive to: the far-along
    check that would normally mean "backtrack" is only a backtrack when
    the aeroplane is near the pavement -- on the parallel taxiway it is
    a runway it has not reached."""
    fo = _fo_for_lineup()
    fo._runway_blocked = Mock(return_value=False)
    fo.session = _fo_session([(51.1491, -0.185)])   # taxi path: already done
    fo.state.sub_entered_at = time.monotonic()
    state = _state(51.1491, -0.185, heading_true=90.0)
    fo._phase_line_up(state, 100.0)
    assert fo.state.sub == "align"


def test_fo_does_not_follow_a_lineup_path_it_is_nowhere_near():
    """A connector leg hundreds of metres away is not "the paved way
    under the wheels": steering at it is the same diagonal through the
    infield the path exists to prevent."""
    fo = _fo_for_lineup()
    fo._runway_blocked = Mock(return_value=False)
    # The connector is far south-east -- nowhere near the aircraft.
    lineup = [(51.1480, -0.1820), (51.1490, -0.1810), (51.1499, -0.1805)]
    fo.session = _fo_session([(51.1491, -0.185)], lineup_path=lineup)
    fo.state.sub_entered_at = time.monotonic()
    state = _state(51.1491, -0.185, heading_true=90.0)
    fo._phase_line_up(state, 100.0)
    assert fo._steer.call_args[0][1] != pytest.approx(
        auto_nav.path_steering(state.latitude, state.longitude,
                               lineup, lookahead_m=30.0)["heading"],
        abs=1.0)


def test_fo_rejected_takeoff_recovers_to_a_fresh_lineup():
    """The await after a reject is not a parking spot mid-strip: once
    the aeroplane has been held a breath the reject is worked off and
    the ready call goes out again -- the next roll is a fresh
    clearance's, not a latch on the rejected one."""
    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _fo()
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "await"
    fo.state.sub_entered_at = 0.0          # long past the hold
    fo.state.rejected = True
    fo.state.abort_announced = True
    fo.state.aligned = True
    fo.session = _fo_session([], cleared_takeoff=True)
    fo._compute_speeds = Mock()
    state = _state(51.15, -0.19, ground_speed_kt=0.0)
    fo._phase_takeoff(state, 100.0)
    assert fo.state.phase == AutonomousPhase.LINE_UP
    assert fo.state.sub == "align"
    assert fo.state.rejected is False
    assert fo.state.aligned is False
    assert fo.session.cleared_takeoff is False


def test_fo_wait_clearance_holds_and_never_errors():
    fo = _fo()
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "wait_clearance"
    fo.state.sub_entered_at = 0.0
    fo.session = _fo_session(_path())
    fo._compute_speeds = Mock()
    fo._runway_blocked = Mock(return_value=False)
    state = _state(51.15, -0.19)
    fo._phase_takeoff(state, 400.0)
    assert fo.state.phase != AutonomousPhase.ERROR
    assert fo.state.sub == "wait_clearance"


def test_fo_rejected_takeoff_stays_powered_on_the_runway():
    fo = _fo()
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "abort"
    fo.state.rejected = True
    fo.session = _fo_session(_path())
    fo._compute_speeds = Mock()
    state = _state(51.15, -0.19)
    fo._phase_takeoff(state, 100.0)
    # Holding powered where the roll stopped -- not a shutdown.
    assert fo.state.phase == AutonomousPhase.TAKEOFF
    assert fo.state.sub == "await"
    fo._phase_takeoff(state, 101.0)
    fo._stop.assert_called()


def test_fo_never_rolls_unaligned():
    fo = _fo()
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "wait_clearance"
    fo.state.runway_heading = 90.0
    fo.state.aligned = False
    fo.session = _fo_session(_path(), cleared_takeoff=True)
    fo._compute_speeds = Mock()
    fo._runway_blocked = Mock(return_value=False)
    fo._phase_takeoff(_state(51.15, -0.19), 100.0)
    # Not on the centreline: back through the lineup, not a roll.
    assert fo.state.phase == AutonomousPhase.LINE_UP


def test_fo_rolls_with_engines_running():
    fo = _fo()
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "roll"
    fo.state.runway_heading = 90.0
    fo.state.sub_entered_at = time.monotonic()
    fo.session = _fo_session(_path(), cleared_takeoff=True)
    fo._compute_speeds = Mock()
    fo._set_takeoff_power = Mock()
    fo._steer_lead = Mock(return_value=0.0)
    fo._turn_rate = Mock(return_value=0.0)
    fo._pedal_dir = Mock(return_value=1.0)
    fo._runway_blocked = Mock(return_value=False)
    # Engines combusting, twenty knots in: the roll continues -- a
    # truthy (out, seen) tuple was how a healthy roll aborted.
    state = _state(51.15, -0.19, heading_true=90.0,
                   indicated_airspeed_kt=20.0, engines_running=1)
    fo._phase_takeoff(state, 101.0)
    assert fo.state.sub == "roll"
    assert fo.state.rejected is False
    fo._set_takeoff_power.assert_called()


def test_fo_rejects_below_v1_on_an_engine_out():
    fo = _fo()
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "roll"
    fo.state.runway_heading = 90.0
    fo.state.sub_entered_at = time.monotonic()
    fo.session = _fo_session(_path(), cleared_takeoff=True)
    fo._compute_speeds = Mock()
    fo._set_takeoff_power = Mock()
    fo._steer_lead = Mock(return_value=0.0)
    fo._turn_rate = Mock(return_value=0.0)
    fo._pedal_dir = Mock(return_value=1.0)
    fo._runway_blocked = Mock(return_value=False)
    state = _state(51.15, -0.19, heading_true=90.0,
                   indicated_airspeed_kt=60.0, engines_running=0)
    fo._phase_takeoff(state, 101.0)
    assert fo.state.rejected is True
    assert fo.state.sub == "abort"


def _steerable_fo(**attrs):
    """An FO whose ``_steer`` is real -- the fixture mocks the law
    itself away, which is what these tests exercise."""
    fo = _fo()
    del fo._steer
    fo._prop_levers = Mock()
    fo._last_lever = 0.0
    fo._stuck_probe = Mock(return_value=0)
    for key, value in attrs.items():
        setattr(fo, key, value)
    return fo


def test_steer_takes_a_small_correction_whole():
    """The eased demand exists for pursuit teleports: a real
    correction of a few degrees arriving at less than half per tick
    was the trail the pursuit wove behind the turn. A jump inside a
    tick of turning goes in whole."""
    fo = _steerable_fo()
    state = _state(51.15, -0.19, heading_true=90.0, ground_speed_kt=8.0)
    fo._steer_want = 100.0
    fo._steer(state, 108.0, 12.0)
    assert fo._steer_want == pytest.approx(108.0, abs=1e-6)


def test_steer_still_eases_a_pursuit_teleport():
    """A hundred-degree re-pick is the segment flip, not a turn order
    -- it still arrives a share at a time."""
    fo = _steerable_fo()
    state = _state(51.15, -0.19, heading_true=90.0, ground_speed_kt=8.0)
    fo._steer_want = 100.0
    fo._steer(state, 200.0, 12.0)
    # 45% of +100.
    assert fo._steer_want == pytest.approx(145.0, abs=1e-6)


def test_steer_brakes_the_swing_before_the_line():
    """The rollout law already brakes early; the taxi law had the rate
    term but no lead, so the counter-pedal only came once the nose was
    through the mark -- the overshoot the lineup hunts with. Nose
    already swinging for the mark: the led demand brakes hard, not
    the gentle bias the un-led error asked for."""
    fo = _steerable_fo()
    state = _state(51.15, -0.19, heading_true=90.0, ground_speed_kt=8.0)
    fo._steer_dt = 2.0
    fo._turn_rate = Mock(return_value=8.0)
    fo._steer(state, 110.0, 12.0)
    # err +20: the un-led ask was ~-0.1; the led one counters, walked
    # to the first slew step on this call.
    assert fo._dir.rudder.call_args[0][0] <= -0.15 + 1e-9


def test_steer_writes_nothing_on_the_heading():
    """A degree of error and no swing is not a turn order: the
    alternating two-per-cent pedal the lineup sat on for its last
    half minute was twitch, not steering."""
    fo = _steerable_fo()
    state = _state(51.15, -0.19, heading_true=95.0, ground_speed_kt=8.0)
    fo._steer(state, 95.0, 12.0)
    assert fo._dir.rudder.call_args[0][0] == 0.0


def test_steer_orders_only_what_the_tiller_has():
    """A weak axis asked for the class's brisk turn is saturated
    pedal with nothing left to brake the swing: the ask is capped at
    the rate this model's measured authority still delivers at the
    limit the ground allows."""
    fo = _steerable_fo()
    fo._yaw_authority_b = {1: 5.0}
    state = _state(51.15, -0.19, heading_true=90.0, ground_speed_kt=8.0)
    fo._steer(state, 130.0, 12.0)
    # Authority 5, limit ~0.40: deliverable ~1.8, floored at 2 deg/s.
    # err +40 wants 10; capped to 2 -> rudder 2/5 = 0.4, reached at the
    # wheel's walked pace a few calls in.
    for _ in range(4):
        fo._steer(state, 130.0, 12.0)
    assert fo._dir.rudder.call_args[0][0] == pytest.approx(0.4, abs=0.05)


def test_lineup_pursuit_never_walks_backwards():
    """On the connector the pursuit re-seated itself to the nearer of
    two legs each tick -- the +140 degree teleport that swung the
    aeroplane through the numbers and made it orbit. The walk only
    moves forward, and a re-planned path re-seeds it."""
    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _fo()
    fo.state.phase = AutonomousPhase.LINE_UP
    fo.state.sub = "align"
    fo.state.runway_heading = 90.0
    fo.state.runway_lat = 51.1505
    fo.state.runway_lon = -0.1780
    fo._runway_blocked = Mock(return_value=False)
    fo._runway_half_m = Mock(return_value=20.0)
    fo._clearance_strobes = Mock()
    # An L of a connector: the north leg, then east onto the runway.
    lineup = [(51.1500, -0.1800), (51.1505, -0.1800), (51.1505, -0.1780)]
    fo.session = _fo_session([], lineup_path=lineup,
                             cleared_takeoff=True)
    # Beside the east leg: it is the nearer, so the walk seats there.
    state = _state(51.1506, -0.1795, heading_true=90.0,
                   ground_speed_kt=5.0)
    fo._phase_line_up(state, 100.0)
    assert fo._lineup_progress == 1
    # Back beside the north leg -- a fresh whole-path pick would
    # re-seat the pursuit onto the leg already passed. The walk holds,
    # and the demand keeps pointing down the connector.
    state2 = _state(51.1502, -0.1805, heading_true=90.0,
                    ground_speed_kt=5.0)
    fo._phase_line_up(state2, 102.0)
    assert fo._lineup_progress == 1
    desired = fo._steer.call_args[0][1]
    assert 30.0 < desired < 90.0
    # A re-planned connector re-seeds the walk instead of inheriting.
    fo.session.lineup_path = [(51.1500, -0.1800), (51.1501, -0.1800),
                              (51.1504, -0.1800)]
    state3 = _state(51.15005, -0.1805, heading_true=90.0,
                    ground_speed_kt=5.0)
    fo._phase_line_up(state3, 104.0)
    assert fo._lineup_progress == 0


def test_fo_rolls_past_ghost_combustion_flags():
    """The C172's first SimConnect takeoff: the combustion indexes
    past its single engine answer False, and the roll was rejected
    for a triple engine failure that never happened. Slots past the
    reported count are not engines."""
    fo = _fo()
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "roll"
    fo.state.runway_heading = 90.0
    fo.state.sub_entered_at = time.monotonic()
    fo.session = _fo_session(_path(), cleared_takeoff=True)
    fo._compute_speeds = Mock()
    fo._set_takeoff_power = Mock()
    fo._steer_lead = Mock(return_value=0.0)
    fo._turn_rate = Mock(return_value=0.0)
    fo._pedal_dir = Mock(return_value=1.0)
    fo._runway_blocked = Mock(return_value=False)
    state = _state(51.15, -0.19, heading_true=90.0,
                   indicated_airspeed_kt=20.0, engines_running=1,
                   engine_count=1, eng2_combustion=0,
                   eng3_combustion=0, eng4_combustion=0)
    fo._phase_takeoff(state, 101.0)
    assert fo.state.sub == "roll"
    assert fo.state.rejected is False
    fo._set_takeoff_power.assert_called()


def test_prop_levers_forward_on_a_prop_aeroplane():
    # The Baron's stand was this: props left at low pitch governed the
    # engines down to propellers that could not move the aeroplane, and
    # the hardware axis held them there -- the levers are the FO's the
    # way the throttle is.
    from meridianatc.atc import aircraft_class as ac
    fo = _fo()
    fo._aircraft_class = ac.PISTON_TWIN
    fo._prop_levers()
    fo._dir.prop.assert_called_once_with(1.0)
    fo._dir.mixture.assert_called_once_with(1.0)


@pytest.mark.parametrize("cls", ["helicopter", "business_jet",
                                 "airliner", ""])
def test_prop_levers_untouched_where_there_are_none(cls):
    # On a helicopter the prop channel is the collective: a full-
    # forward write there is a climb nobody asked for, and a jet has
    # neither lever.
    fo = _fo()
    fo._aircraft_class = cls
    fo._prop_levers()
    fo._dir.prop.assert_not_called()
    fo._dir.mixture.assert_not_called()


def test_takeoff_power_puts_props_forward_first():
    fo = _fo()
    from meridianatc.atc import aircraft_class as ac
    fo._aircraft_class = ac.PISTON_TWIN
    fo._set_takeoff_power()
    fo._dir.prop.assert_called_with(1.0)
    fo._dir.mixture.assert_called_with(1.0)
    fo._dir.throttle.assert_called()
    fo._dir.toga.assert_called_with(True)


def test_go_around_puts_props_forward():
    fo = _fo()
    from meridianatc.atc import aircraft_class as ac
    from meridianatc.atc.autonomous import AutonomousPhase
    fo._aircraft_class = ac.PISTON_TWIN
    fo.state.phase = AutonomousPhase.LANDING
    fo.state.go_arounds = 0
    fo.state.go_around_at = 0.0
    fo.state.circling = False
    fo.state.flare_at = 0.0
    fo.state.approach_recipe = None
    fo.state.missed_alt_ft = 0.0
    fo.state.flaps_index = 0
    fo.state.pitch_deg = 0.0
    fo.session = _fo_session(_path(), assigned_altitude_ft=0.0)
    fo._manage_radio = False
    fo._fo_say = Mock()
    fo._emit = Mock()
    fo._divert_after_go_arounds = Mock()
    fo._manual_control_required = Mock()
    fo._field_elevation = Mock(return_value=0.0)
    fo._pitch_target_deg = Mock(return_value=12.0)
    fo._g_pull_cap = Mock(return_value=0.3)
    state = _state(51.15, -0.19, altitude_ft=100.0, source="xplane",
                   flaps_index=0)
    fo._start_go_around(state)
    fo._dir.prop.assert_called_with(1.0)
    fo._dir.mixture.assert_called_with(1.0)
    fo._dir.throttle.assert_called_with(1.0)


def _fo_in_initial_climb(**state_kw):
    """A FO off the ground ten seconds into the initial climb."""
    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _fo()
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "initial_climb"
    fo.state.airborne_at = time.monotonic() - 3.0
    fo.state.vr_kt = 85.0
    fo.state.v2_kt = 90.0
    fo._perf.cruise_kt = 180.0
    # The lever the roll left at the stop: the slew continues from it,
    # not from idle.
    fo._last_lever = 1.0
    fo._lever_at = time.monotonic() - 0.5
    fo.session = _fo_session(_path())
    fo._compute_speeds = Mock()
    base = dict(on_ground=False, altitude_agl_ft=150.0,
                indicated_airspeed_kt=92.0, vertical_speed_fpm=700.0,
                pitch_deg=8.0, gear_down=True, flaps_index=0,
                engines_running=1)
    base.update(state_kw)
    return fo, _state(51.15, -0.19, **base)


def test_fo_rotate_stamps_the_liftoff_clock():
    # The clock the initial climb is timed from: left at 0 on the
    # rotate -> initial_climb path, elapsed never grew and the 25 s
    # fallback to the AP was dead code.
    fo = _fo()
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "rotate"
    fo.state.rotate_at = time.monotonic()
    fo.state.airborne_at = 0.0
    fo.session = _fo_session(_path())
    fo._compute_speeds = Mock()
    # The flag already held a second: a liftoff, not a flicker.
    fo._off_ground_since = time.monotonic() - 1.0
    state = _state(51.15, -0.19, on_ground=False, pitch_deg=8.0)
    fo._phase_takeoff(state, time.monotonic())
    assert fo.state.sub == "initial_climb"
    assert fo.state.airborne_at > 0.0


def test_airborne_needs_the_flag_held_not_a_flicker():
    """The C172's overrun: SIM ON GROUND blipped off for one read as
    the aeroplane ran off the end, and the roll walked itself into a
    "climb" at ground level. A flicker is not a liftoff -- the flag
    has to stay off the better part of a second."""
    fo = _fo()
    state = _state(51.15, -0.19, on_ground=False)
    assert fo._airborne_confirmed(state, 100.0) is False
    # Still off inside the window: believed, not yet.
    assert fo._airborne_confirmed(state, 100.5) is False
    # Held: the flag means it.
    assert fo._airborne_confirmed(state, 101.0) is True
    # A bounce put the wheels back: the clock starts over.
    ground = _state(51.15, -0.19, on_ground=True)
    assert fo._airborne_confirmed(ground, 102.0) is False
    assert fo._airborne_confirmed(state, 102.5) is False
    assert fo._airborne_confirmed(state, 103.2) is False
    assert fo._airborne_confirmed(state, 103.4) is True


def test_roll_flicker_does_not_jump_to_rotate():
    """The same blip inside the roll: one off read must not walk the
    aeroplane past the speed gate into rotate."""
    fo = _fo()
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "roll"
    fo.state.runway_heading = 90.0
    fo.state.sub_entered_at = time.monotonic()
    fo.session = _fo_session(_path(), cleared_takeoff=True)
    fo._compute_speeds = Mock()
    fo._set_takeoff_power = Mock()
    fo._steer_lead = Mock(return_value=0.0)
    fo._turn_rate = Mock(return_value=0.0)
    fo._pedal_dir = Mock(return_value=1.0)
    fo._runway_blocked = Mock(return_value=False)
    state = _state(51.15, -0.19, heading_true=90.0,
                   indicated_airspeed_kt=20.0, engines_running=1,
                   on_ground=False)
    fo._phase_takeoff(state, time.monotonic())
    assert fo.state.sub == "roll"


def _roller():
    """An FO mid-roll on a 2400 ft strip whose threshold is the
    fixture's origin: the distance checks answer the pavement itself."""
    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _fo()
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "roll"
    fo.state.runway_heading = 90.0
    fo.state.runway_lat = 51.15
    fo.state.runway_lon = -0.19
    fo.state.runway_length_ft = 2400.0
    fo.state.sub_entered_at = time.monotonic()
    fo.session = _fo_session(_path(), cleared_takeoff=True)
    fo._compute_speeds = Mock()
    fo._set_takeoff_power = Mock()
    fo._steer_lead = Mock(return_value=0.0)
    fo._turn_rate = Mock(return_value=0.0)
    fo._pedal_dir = Mock(return_value=1.0)
    fo._runway_blocked = Mock(return_value=False)
    return fo


def test_roll_aborts_past_the_far_threshold():
    """The C172's overrun: still on the wheels beyond the far end is
    an overrun whatever the clock or the speeds said."""
    fo = _roller()
    # ~800 m down a 731 m strip, and fast -- the crash's own numbers.
    state = _state(51.15, -0.19 + 800.0 / 69900.0, heading_true=90.0,
                   indicated_airspeed_kt=78.0, engines_running=1)
    fo._phase_takeoff(state, time.monotonic())
    assert fo.state.rejected is True
    assert fo.state.sub == "abort"


def test_roll_aborts_under_seventy_percent_at_the_midpoint():
    """Half the pavement gone without seventy per cent of the rotate
    speed is the takeoff the books stop, not continue -- the 70/50
    rule a crew keeps for the day the numbers do not feel right."""
    fo = _roller()
    # The first pass only stamps where the roll began.
    state = _state(51.15, -0.19 + 60.0 / 69900.0, heading_true=90.0,
                   indicated_airspeed_kt=10.0, engines_running=1)
    fo._phase_takeoff(state, time.monotonic())
    assert fo.state.sub == "roll"
    # Over halfway through the run-out and still under 0.7 * 140.
    state.latitude, state.longitude = 51.15, -0.19 + 430.0 / 69900.0
    fo._phase_takeoff(state, time.monotonic())
    assert fo.state.rejected is True
    assert fo.state.sub == "abort"


def test_roll_continues_when_the_midpoint_speed_is_made():
    """Same pavement, speed where it should be: the roll goes on --
    the yardstick watches for a slow roll, not a healthy one."""
    fo = _roller()
    state = _state(51.15, -0.19 + 60.0 / 69900.0, heading_true=90.0,
                   indicated_airspeed_kt=10.0, engines_running=1)
    fo._phase_takeoff(state, time.monotonic())
    state.latitude, state.longitude = 51.15, -0.19 + 430.0 / 69900.0
    state.indicated_airspeed_kt = 110.0   # past 0.7 * 140
    fo._phase_takeoff(state, time.monotonic())
    assert fo.state.rejected is False
    assert fo.state.sub == "roll"


def test_roll_past_the_end_below_v1_rejects_above_v1_continues():
    """The same overrun, two speeds: below the decision speed the
    overrun is a stop; past it the stop no longer fits and the books
    say fly it off -- the reject into the fence is the accident V1
    exists to prevent."""
    fo = _roller()
    fo.state.v1_kt = 45.0
    fo.state.vr_kt = 55.0
    # Stamp the roll's start, then run past the far threshold still
    # under the decision speed: a stop.
    state = _state(51.15, -0.19 + 60.0 / 69900.0, heading_true=90.0,
                   indicated_airspeed_kt=20.0, engines_running=1)
    fo._phase_takeoff(state, time.monotonic())
    state.longitude = -0.19 + 800.0 / 69900.0
    state.indicated_airspeed_kt = 40.0
    fo._phase_takeoff(state, time.monotonic())
    assert fo.state.rejected is True
    assert fo.state.sub == "abort"
    # The same pavement, the speed made: committed -- the sub rotates,
    # the levers stay in.
    fo = _roller()
    fo.state.v1_kt = 45.0
    fo.state.vr_kt = 55.0
    state = _state(51.15, -0.19 + 60.0 / 69900.0, heading_true=90.0,
                   indicated_airspeed_kt=20.0, engines_running=1)
    fo._phase_takeoff(state, time.monotonic())
    state.longitude = -0.19 + 800.0 / 69900.0
    state.indicated_airspeed_kt = 78.0
    fo._phase_takeoff(state, time.monotonic())
    assert fo.state.rejected is False
    assert fo.state.sub == "rotate"


def test_roll_intersection_measures_the_cleared_distance():
    """A C1 join is not the chart's full length: the tower's "eight
    hundred metres remaining" is the strip this roll may use, and
    reaching its end still on the wheels is the overrun -- even with
    the runway's own pavement continuing beyond it."""
    fo = _roller()
    fo.state.v1_kt = 45.0
    fo.state.vr_kt = 55.0
    # 731 m of runway, a 400 m intersection clearance: half of the
    # cleared run is the midpoint, its end the reject line.
    fo.session.departure_remaining_m = 400.0
    state = _state(51.15, -0.19 + 60.0 / 69900.0, heading_true=90.0,
                   indicated_airspeed_kt=10.0, engines_running=1)
    fo._phase_takeoff(state, time.monotonic())
    assert fo.state.sub == "roll"
    # 460 m from the join -- inside the runway, outside the clearance.
    state.longitude = -0.19 + 520.0 / 69900.0
    state.indicated_airspeed_kt = 40.0
    fo._phase_takeoff(state, time.monotonic())
    assert fo.state.rejected is True
    assert fo.state.sub == "abort"


def test_rotate_past_the_end_committed_flies_the_pull():
    """Rotating into the overrun at speed is a continued takeoff, not
    a reject: the nose stays coming up. Tonight's crash was the
    inverse -- a reject's calm elevator where a fly-off pull was
    owed."""
    fo = _roller()
    fo.state.sub = "rotate"
    fo.state.rotate_at = time.monotonic()
    fo.state.rotate_speed_kt = 55.0
    fo.state.v1_kt = 45.0
    fo.state.vr_kt = 55.0
    # The roll's stamp, as the roll sub left it.
    fo._roll_stamp = fo.state.sub_entered_at
    fo._roll_along0 = 60.0
    fo._roll_run0 = 671.0          # the 731 m strip, joined 60 m in
    state = _state(51.15, -0.19 + 800.0 / 69900.0, heading_true=90.0,
                   indicated_airspeed_kt=70.0, engines_running=1,
                   on_ground=True, pitch_deg=-2.0)
    fo._phase_takeoff(state, time.monotonic())
    assert fo.state.rejected is False
    assert fo.state.sub == "rotate"
    pull = fo._dir.elevator.call_args.args[0]
    assert pull > 0.5


def test_rotate_pull_answers_the_attitude_gap():
    """The overrun's own picture: the fixed schedule fed the model a
    polite third of a yoke and the C172 crawled to 4.6 degrees at
    78 kt while the pavement ran out. The pull answers how far the
    nose still has to come -- full early, tapering at the mark."""
    fo = _fo()
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "rotate"
    fo.state.rotate_at = time.monotonic()
    fo.state.rotate_speed_kt = 55.0
    fo.state.v1_kt = 45.0
    fo.session = _fo_session(_path())
    fo._compute_speeds = Mock()
    # On the wheels, nose down, at Vr: the ask is nearly all of it.
    state = _state(51.15, -0.19, on_ground=True, pitch_deg=-2.0,
                   indicated_airspeed_kt=56.0)
    fo._phase_takeoff(state, time.monotonic())
    early = fo._dir.elevator.call_args.args[0]
    assert early > 0.7
    # Near the attitude it eases itself off, the way a hand does.
    state.pitch_deg = 8.0
    fo._phase_takeoff(state, time.monotonic())
    late = fo._dir.elevator.call_args.args[0]
    assert 0.0 < late < early


def test_sim_rotation_speed_skips_the_negative_sentinel():
    """MSFS answers DESIGN SPEED MIN ROTATION with -1 when the cfg
    lacks it -- a truthy sentinel that used to stand in front of the
    takeoff fallback. A negative read is no figure at all."""
    from meridianatc.atc import vspeeds
    got = vspeeds.speeds_for("C172", "light",
                             min_rotation_kt=-1.0,
                             design_takeoff_kt=60.0)
    # The takeoff figure, pulled back to the unstick the way the
    # declared block retunes it -- not the sentinel, not the climb-away.
    assert got.vr == pytest.approx(54.0)
    got = vspeeds.speeds_for("C172", "light",
                             min_rotation_kt=-1.0,
                             design_takeoff_kt=0.0)
    # Nothing declared either way: the type's own table stands.
    assert got.vr == pytest.approx(55.0)


def test_a_re_identified_aircraft_recomputes_its_speeds():
    """The sim names the ramp BE58 before it settles on the C172
    parked in it -- every session tonight did exactly that. Speeds
    resolved under the first guess would fly a Baron's Vr on the
    Skyhawk's wing."""
    from meridianatc.atc.autonomous import AutonomousFlight
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = SimpleNamespace(v1_kt=0.0, vr_kt=0.0, v2_kt=0.0,
                               vapp_kt=0.0, climb_kt=0.0)
    fo.session = SimpleNamespace(
        aircraft=SimpleNamespace(type_code="BE58", title=""),
        profile=SimpleNamespace(kind=""))
    fo._perf = SimpleNamespace(climb_kt=180.0)
    fo._perf_code = "BE58"
    state = SimpleNamespace(min_rotation_kt=0.0, design_takeoff_kt=0.0,
                            design_vs1_kt=0.0)
    fo.get_state = lambda: state
    fo._aircraft_class = "piston_twin"
    speeds = fo._compute_speeds(state)
    assert speeds.vr == pytest.approx(85.0)   # the Baron's figure
    fo.session.aircraft.type_code = "C172"
    fo._perf_code = "C172"     # the perf cache followed too
    fo._perf = SimpleNamespace(climb_kt=80.0)
    speeds = fo._compute_speeds(state)
    # The aeroplane's own number, not the first guess's.
    assert speeds.vr == pytest.approx(55.0)
    assert fo.state.climb_kt == 80.0


def test_fo_initial_climb_holds_power_and_flies_the_attitude():
    # The crash in the log: the sub issued nothing at all, so the
    # rotation's pull stayed latched on the stick and nothing held the
    # levers -- the aeroplane bled to 67 kt and stalled at 133 ft.
    fo, state = _fo_in_initial_climb()
    fo._phase_takeoff(state, time.monotonic())
    # The levers are commanded every pass -- and commanded to power,
    # never to idle while the aeroplane is this low.
    throttle = [c.args[0] for c in fo._dir.throttle.call_args_list]
    assert throttle and all(w > 0.5 for w in throttle)
    fo._dir.throttle_idle.assert_not_called()
    fo._dir.toga.assert_called_with(True)
    # The roll's steering demands are the runway's: released at liftoff
    # rather than left pulling.
    fo._dir.rudder.assert_called_with(0.0)
    fo._dir.steering.assert_called_with(0.0)
    # And the pitch is flown, eased rather than left where the rotation
    # put it.
    pull = fo._dir.elevator.call_args.args[0]
    assert 0.0 < pull < 0.4
    from meridianatc.atc.autonomous import AutonomousPhase
    assert fo.state.phase == AutonomousPhase.TAKEOFF


def test_fo_initial_climb_levels_the_wings_off_the_departure():
    """The 2026-11-20 case: the yoke went neutral at liftoff and nothing
    held a wing -- the bank read -25 degrees at forty feet and the
    banked climb folded into the trees off the end. The sub levels the
    wings toward the runway heading now."""
    fo, state = _fo_in_initial_climb(bank_deg=-25.0, heading_true=157.0)
    fo.state.runway_heading = 190.0
    fo._phase_takeoff(state, time.monotonic())
    aileron = fo._dir.aileron.call_args.args[0]
    # A left bank and a heading left of the runway's both ask right.
    assert aileron > 0.0
    # A right wing already down asks the other way.
    fo2, state2 = _fo_in_initial_climb(bank_deg=20.0, heading_true=190.0)
    fo2.state.runway_heading = 190.0
    fo2._phase_takeoff(state2, time.monotonic())
    assert fo2._dir.aileron.call_args.args[0] < 0.0


def test_fo_initial_climb_lowers_the_nose_below_the_safe_speed():
    # The exact picture the log drew: 67 kt against a 90 kt safe speed
    # and sinking a thousand feet a minute -- the nose is lowered
    # outright, because a climb the wing cannot pay for is a stall.
    fo, state = _fo_in_initial_climb(altitude_agl_ft=130.0,
                                     indicated_airspeed_kt=67.0,
                                     vertical_speed_fpm=-1100.0,
                                     pitch_deg=14.0)
    fo._phase_takeoff(state, time.monotonic())
    assert fo._dir.elevator.call_args.args[0] < 0.0
    fo._dir.throttle_idle.assert_not_called()


def test_fo_initial_climb_engages_the_ap_after_the_wait():
    fo, state = _fo_in_initial_climb(altitude_agl_ft=300.0,
                                     indicated_airspeed_kt=95.0,
                                     vertical_speed_fpm=800.0,
                                     pitch_deg=10.0, gear_down=False)
    fo.state.airborne_at = time.monotonic() - 30.0
    fo._engage_climb = Mock()
    fo._phase_takeoff(state, time.monotonic())
    # The yoke is centred for the handoff: an AP handed a stick still
    # pulled from the rotation fights the climb it is asked to fly.
    assert fo._dir.elevator.call_args.args[0] == 0.0
    fo._engage_climb.assert_called_once()
    from meridianatc.atc.autonomous import AutonomousPhase
    assert fo.state.phase == AutonomousPhase.CLIMB


def test_fo_rollout_centres_the_surfaces_at_touchdown():
    # The flare's pull and into-wind wing are flare demands: left
    # latched on the stick they are a nosewheel held light through the
    # rollout and a surface sitting displaced into the taxi-in.
    fo = _fo()
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.LANDING
    fo.state.sub = "flare"
    fo.session = _fo_session(_path())
    state = _state(51.15, -0.19, on_ground=True, pitch_deg=4.0)
    fo._phase_landing(state, time.monotonic())
    assert fo.state.sub == "rollout"
    fo._dir.throttle_idle.assert_called()
    fo._dir.elevator.assert_called_with(0.0)
    fo._dir.aileron.assert_called_with(0.0)


def test_fo_go_around_releases_the_ground_axes():
    # A go-around started from the flare inherits its de-crab pedal
    # and its into-wind wing: latched into the climb they are a skid
    # and a bank at fifty feet.
    fo = _fo()
    fo.state.sub = "go_around"
    fo.state.go_around_at = time.monotonic()
    fo._perf.cruise_kt = 180.0
    state = _state(51.15, -0.19, on_ground=False, pitch_deg=6.0,
                   indicated_airspeed_kt=90.0, vertical_speed_fpm=300.0)
    fo._fly_go_around(state, time.monotonic())
    fo._dir.rudder.assert_called_with(0.0)
    fo._dir.steering.assert_called_with(0.0)
    fo._dir.aileron.assert_called_with(0.0)
    # The hand-flown pass still flies pitch and power.
    assert fo._dir.elevator.call_args.args[0] > 0.0
    assert fo._dir.throttle.call_args.args[0] >= 1.0


def test_stop_centres_the_stick_while_still_rolling():
    # A rejected takeoff rolling out at eighty knots: the stop is
    # brakes, pedals and a neutral stick -- a rotation pull latched
    # from the roll is a nosewheel held light through the braking.
    fo = _fo()
    fo.session = _fo_session(_path())
    state = _state(51.15, -0.19, ground_speed_kt=80.0)
    from meridianatc.atc.autonomous import AutonomousFlight
    AutonomousFlight._stop(fo, state)
    fo._dir.elevator.assert_called_with(0.0)
    fo._dir.aileron.assert_called_with(0.0)
    fo._dir.rudder.assert_called_with(0.0)
    fo._dir.steering.assert_called_with(0.0)


def test_fo_vacates_the_runway_on_a_taxi_instruction():
    fo = _fo()
    from meridianatc.atc.autonomous import AutonomousPhase
    from meridianatc.atc.session import Phase
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "await"
    fo.state.rejected = True
    fo.session = _fo_session(_path(), phase=Phase.TAXI_IN)
    fo._compute_speeds = Mock()
    fo._phase_takeoff(_state(51.15, -0.19), 100.0)
    assert fo.state.phase == AutonomousPhase.TAXI_IN


def test_fo_does_not_reroll_after_a_rejection():
    fo = _fo()
    from meridianatc.atc.autonomous import AutonomousPhase
    fo.state.phase = AutonomousPhase.TAKEOFF
    fo.state.sub = "wait_clearance"
    fo.state.rejected = True
    fo.session = _fo_session(_path(), cleared_takeoff=True)
    fo._compute_speeds = Mock()
    fo._runway_blocked = Mock(return_value=False)
    fo._phase_takeoff(_state(51.15, -0.19), 100.0)
    assert fo.state.sub == "wait_clearance"


def test_fo_backtracks_a_far_entry_then_lines_up():
    fo = _fo_for_lineup()
    fo.session = _fo_session(_path())
    fo._runway_blocked = Mock(return_value=False)
    # Threshold at 51.15,-0.19 heading 90; the aeroplane joined 600 m
    # down the runway -> backtrack toward the threshold.
    state = _state(51.15, -0.19 + (600.0 / 111320.0)
                   / max(math.cos(math.radians(51.15)), 0.01))
    fo._phase_line_up(state, 100.0)
    assert fo.state.sub == "backtrack"
    # Drive back: inside the turn gate, it goes for the 180.
    state = _state(51.15, -0.19 + (30.0 / 111320.0)
                   / max(math.cos(math.radians(51.15)), 0.01))
    fo._phase_line_up(state, 101.0)
    assert fo.state.sub == "turn_out"
    # Pointed the departure way again, it returns to align.
    state = _state(51.15, -0.19, heading_true=88.0)
    fo._phase_line_up(state, 102.0)
    assert fo.state.sub == "align"


# -- gear ------------------------------------------------------------------


def _sim_traffic():
    from meridianatc.integrations.aitraffic import SimTraffic
    st = SimTraffic.__new__(SimTraffic)
    st._dll = Mock()
    st._handle = 0
    st._objects = {"CS1": 42}
    st._drawn = {}
    st._move_gear = {}
    st._grounded = set()
    st._velocity_ok = False
    st.pose_writes = 0
    st._titles = {}
    st._static_pitch = {}
    st._standing_ft = {}
    st._settled = set()
    return st


def _pose(**kw):
    base = dict(lat=51.15, lon=-0.19, alt_ft=200.0, heading=90.0,
                pitch=0.0, bank=0.0, gs_kt=0.0, vs_fpm=0.0,
                on_ground=True, gear_down=True, surface_ft=200.0)
    base.update(kw)
    return SimpleNamespace(**base)


def _gear_events(dll):
    from meridianatc.integrations.aitraffic import _EV_GEAR_DOWN, _EV_GEAR_UP
    return [c.args[2] for c in dll.TransmitClientEvent.call_args_list
            if c.args[2] in (_EV_GEAR_DOWN, _EV_GEAR_UP)]


def test_draw_asserts_gear_down_on_first_write():
    st = _sim_traffic()
    drawn = {}
    st._draw("CS1", 42, _pose(), drawn)
    from meridianatc.integrations.aitraffic import _EV_GEAR_DOWN as down
    assert _gear_events(st._dll) == [down]
    # Once per object: a repeated pose does not send it again.
    st._draw("CS1", 42, _pose(), drawn)
    assert _gear_events(st._dll) == [down]


def test_draw_reasserts_gear_after_a_model_swap():
    st = _sim_traffic()
    drawn = {}
    st._draw("CS1", 42, _pose(), drawn)
    drawn.clear()                           # the swap's reset
    st._draw("CS1", 43, _pose(), drawn)
    from meridianatc.integrations.aitraffic import _EV_GEAR_DOWN as down
    assert _gear_events(st._dll) == [down, down]


def test_draw_sends_gear_up_for_airborne_departure():
    st = _sim_traffic()
    st._draw("CS1", 42, _pose(on_ground=False, gear_down=False,
                              alt_ft=5000.0), {})
    from meridianatc.integrations.aitraffic import _EV_GEAR_UP as up
    assert _gear_events(st._dll) == [up]


def test_move_asserts_gear_on_first_write_and_tracks_changes():
    from meridianatc.integrations.aitraffic import _EV_GEAR_DOWN, _EV_GEAR_UP
    st = _sim_traffic()
    st._objects = {"CS1": 7}
    st._grounded = {"CS1"}
    pos = {"lat": 51.15, "lon": -0.19, "altitude_ft": 200.0,
           "heading": 90.0, "ground_speed_kt": 0.0, "on_ground": True,
           "phase": "taxi", "motion": {"surface_ft": 200.0}}
    st._move("CS1", pos)
    assert _gear_events(st._dll) == [_EV_GEAR_DOWN]
    st._move("CS1", pos)                    # same pose: asserted once
    assert _gear_events(st._dll) == [_EV_GEAR_DOWN]
    st._grounded.discard("CS1")
    st._move("CS1", {**pos, "on_ground": False, "altitude_ft": 3000.0,
                     "phase": "departed"})
    assert _gear_events(st._dll) == [_EV_GEAR_DOWN, _EV_GEAR_UP]


def test_fo_backtracks_down_the_centreline():
    fo = _fo_for_lineup()
    fo.session = _fo_session(_path())
    fo._runway_blocked = Mock(return_value=False)
    # 600 m down the runway and ~5 m off the centreline: entering the
    # backtrack and then driving it.
    cos_lat = math.cos(math.radians(51.15))
    lon = -0.19 + (600.0 / 111320.0) / cos_lat
    lat = 51.15 - (5.0 / 111320.0)
    fo._phase_line_up(_state(lat, lon), 100.0)
    assert fo.state.sub == "backtrack"
    fo._steer.assert_not_called()          # the entry tick only turns to it
    fo._phase_line_up(_state(lat, lon), 101.0)
    hdg, kt = fo._steer.call_args[0][1:]
    # The steer carries a correction back toward the line, not a
    # parallel track at the reciprocal heading.
    assert hdg != 270.0
    assert abs((hdg - 270.0 + 540.0) % 360.0 - 180.0) <= 60.0
    # The turn is a long way off, so it rolls at the type's taxi speed.
    assert kt == pytest.approx(fo.performance.taxi_kt)


def test_fo_backtrack_slowing_for_the_turn():
    fo = _fo_for_lineup()
    fo.session = _fo_session(_path())
    fo._runway_blocked = Mock(return_value=False)
    fo.state.sub = "backtrack"
    fo.state.sub_entered_at = time.monotonic()
    cos_lat = math.cos(math.radians(51.15))
    # 120 m from the numbers: the walk-up-to-the-180 speed, not taxi.
    lon = -0.19 + (120.0 / 111320.0) / cos_lat
    fo._phase_line_up(_state(51.15, lon), 100.0)
    assert fo.state.sub == "backtrack"
    _, kt = fo._steer.call_args[0][1:]
    assert 3.5 <= kt < fo.performance.taxi_kt


# -- the light a moving aeroplane shows -------------------------------------


def test_fo_puts_the_taxi_light_on_for_the_taxi():
    path = _path()
    fo = _fo()
    fo.session = _fo_session(path)
    state = _state(path[0][0], path[0][1], taxi_light=False)
    fo._taxi_along_path(state, outbound=True)
    fo._dir.taxi_light.assert_called_once_with(True)


def test_fo_leaves_a_lit_taxi_light_alone():
    path = _path()
    fo = _fo()
    fo.session = _fo_session(path)
    state = _state(path[0][0], path[0][1], taxi_light=True)
    fo._taxi_along_path(state, outbound=True)
    fo._dir.taxi_light.assert_not_called()


def test_fo_lines_up_with_the_light_on():
    fo = _fo_for_lineup()
    fo.session = _fo_session(_path())
    fo._runway_blocked = Mock(return_value=False)
    fo.state.sub_entered_at = time.monotonic()
    fo._phase_line_up(_state(51.15, -0.19), 100.0)
    fo._dir.taxi_light.assert_called_once_with(True)
