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
        AutonomousFlight, AutonomousPhase, AutonomousState)
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
                               takeoff_kt_s=4.0)
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
    # 10 m short of the last point -- the nose is at the bars.
    end_lat, end_lon = path[-1]
    short = _state(end_lat, end_lon - 0.00016, ground_speed_kt=3.0)
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
    from meridianatc.integrations.aitraffic import (
        _EV_GEAR_DOWN, _EV_GEAR_UP)
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
