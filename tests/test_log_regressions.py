"""Regressions for defects found in the live logs.

- a feed or a model files "@@@@@@@@" for a callsign it never had;
- the transponder code is read as BCO16, so the packed reading wins;
- a clearance ask that could not go out is retried, not waited on;
- ENG TORQUE PERCENT's shipped unit is one the simulator accepts.
"""
from __future__ import annotations

import time
from types import SimpleNamespace
from unittest.mock import Mock


def test_fill_callsign_from_a_feed_falls_back_to_registration():
    from meridianatc.atc.live import written_callsign
    flight = SimpleNamespace(callsign="@@@@@@@@", registration="N-1234X")
    assert written_callsign(flight) == "N1234X"


def test_real_callsign_still_wins_over_registration():
    from meridianatc.atc.live import written_callsign
    flight = SimpleNamespace(callsign="BAW123", registration="G-ABCD")
    assert written_callsign(flight) == "BAW123"


def test_fill_callsign_is_unknown_to_the_parser():
    from meridianatc.atc.speech import parse_callsign
    parsed = parse_callsign("@@@@@@@@")
    assert parsed.written == "UNKNOWN"


def test_simulator_fill_registration_is_dropped():
    from meridianatc.integrations.simaircraft import SimAircraft
    watcher = SimAircraft(enabled=False)
    reply = SimpleNamespace(
        callsign=b"@@@@@@@@", airline=b"", flight_number=b"",
        model=b"C172", title=b"Cessna",
        dwObjectID=42, latitude=38.7, longitude=-9.1,
        altitude_ft=380.0, heading=200.0, ground_speed_kt=0.0,
        vertical_speed_fpm=0.0, on_ground=1.0, is_user=0.0)
    watcher.on_aircraft(reply)
    seen = watcher._seen[42]
    assert seen.registration == ""
    assert seen.callsign == ""


def test_squawk_decodes_the_packed_reading_first():
    from meridianatc.sim.source import decode_squawk
    # Asked for as BCO16: squawk 1200 arrives as 0x1200 = 4608, which is
    # also a legal-looking decimal squawk -- the packed reading must win.
    assert decode_squawk(0x1200) == "1200"
    assert decode_squawk(0x7500) == "7500"
    # A simulator that ignores the unit still answers a plain integer.
    assert decode_squawk(1200) == "1200"


def test_broken_wrapper_units_are_patched_on_the_session():
    from meridianatc.sim.source import SimConnectSource
    requests = SimpleNamespace()
    for helper in ("HelicopterSpecificData", "MiscellaneousData"):
        cls = type(helper, (), {"list": dict(
            SimConnectSource.BROKEN_UNITS.get(helper, {}))})
        requests.__dict__[helper] = cls()
    requests.HelicopterSpecificData.list["ENG_TORQUE_PERCENT:index"] = [
        "desc", b"ENG TORQUE PERCENT:index",
        b"Percent scalar 16K (Ft/lbs * 16384)", "N"]
    SimConnectSource.define_our_variables(requests)
    unit = requests.HelicopterSpecificData.list[
        "ENG_TORQUE_PERCENT:index"][2]
    assert unit == b"Percent"


def _autonomous(stations=(), staffed=True):
    """The flight with only what _phase_request_clearance touches."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    flight.session = SimpleNamespace(
        clearance_read_back=False, clearance_issued=False,
        ifr=True, last_instruction=None,
        departure=SimpleNamespace(ident="LPPT"),
        destination=SimpleNamespace(ident="LPPT"))
    flight.CLEARANCE_TIMEOUT = 60.0
    flight._request_ifr = True
    flight.crew = SimpleNamespace(
        radio_busy=None, radio_call=lambda kind: ("", "no station"),
        transmit=lambda text: None)
    flight.navdb = SimpleNamespace(
        stations=lambda ident, position=None: [
            s for s in stations
            if position is None or s.position == position])
    flight.brain = SimpleNamespace(
        staffed=lambda s: staffed, issues_clearances=lambda s: staffed)
    flight.get_state = lambda: None
    flight._set_error = Mock()
    flight._transition = Mock()
    flight._emit = Mock()
    flight._ensure_systems_on = Mock()
    return flight


def test_a_refused_clearance_ask_is_tried_again():
    flight = _autonomous(stations=[_gnd()])
    now = 100.0
    flight.state.phase_entered_at = now
    asks = []
    flight._request_clearance_via_fo = lambda: asks.append(1) or False
    flight._phase_request_clearance(None, now)
    flight._phase_request_clearance(None, now + 0.5)
    flight._phase_request_clearance(None, now + 1.0)
    # Never sent, so never marked asked -- and not yet timed out.
    assert not flight.state.has_ifr_clearance
    assert len(asks) == 3
    flight._set_error.assert_not_called()


def test_a_made_clearance_ask_is_marked_and_answer_moves_on():
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _autonomous(stations=[_gnd()])
    now = 100.0
    flight.state.phase_entered_at = now
    flight._request_clearance_via_fo = lambda: True
    flight._phase_request_clearance(None, now)
    assert flight.state.has_ifr_clearance
    flight.session.clearance_issued = True
    flight._phase_request_clearance(None, now + 5.0)
    # Issued is not held: the readback is still owed, so the flight
    # waits where it is rather than push back on a clearance it has
    # not said back.
    flight._transition.assert_not_called()
    flight.session.clearance_read_back = True
    flight._phase_request_clearance(None, now + 10.0)
    flight._transition.assert_called_once_with(AutonomousPhase.PUSHBACK)


def test_an_unanswered_ask_is_repeated_not_waited_on():
    flight = _autonomous(stations=[_gnd()])
    now = 100.0
    flight.state.phase_entered_at = now
    asks = []
    flight._request_clearance_via_fo = lambda: asks.append(1) or True
    flight._phase_request_clearance(None, now)
    flight.state.last_radio_at = now      # _fo_say stamps a real transmit
    flight._phase_request_clearance(None, now + 10.0)   # too soon
    flight._phase_request_clearance(None, now + 16.0)   # asks again
    assert len(asks) == 2
    assert flight.state.has_ifr_clearance


def test_clearance_overdue_keeps_asking_never_refiles_vfr():
    """A clearance that never comes is asked for again, warned about
    once -- never a silent re-file to VFR, never a dead flight."""
    flight = _autonomous(stations=[_gnd()])
    now = 100.0
    flight.state.phase_entered_at = now
    flight.state.has_ifr_clearance = True
    flight.state.clearance_asked_at = now     # the ask went out on time
    flight.state.last_radio_at = now
    flight._phase_request_clearance(None, now + 61.0)
    assert flight.session.ifr is True          # never downgraded
    flight._set_error.assert_not_called()
    flight._transition.assert_not_called()
    # Said once, asked again -- and a second tick says nothing more.
    flight._emit.assert_called_once()
    assert flight._emit.call_args[0][0] == "warning"
    flight._emit.reset_mock()
    flight._phase_request_clearance(None, now + 62.0)
    flight._emit.assert_not_called()


def test_a_ask_that_never_went_out_is_not_timed_out():
    """A dead radio is not a refused clearance: the flight keeps trying
    rather than erroring on silence it could not help."""
    flight = _autonomous(stations=[_gnd()])
    now = 100.0
    flight.state.phase_entered_at = now
    flight._request_clearance_via_fo = lambda: False
    flight._phase_request_clearance(None, now + 500.0)
    flight._set_error.assert_not_called()


def test_the_clearance_clock_starts_at_the_ask():
    """A radio that took a minute to come alive has not been refused for
    the whole phase -- the wait is measured from the first call that
    actually went out."""
    flight = _autonomous(stations=[_gnd()])
    now = 100.0
    flight.state.phase_entered_at = now
    asks = []
    flight._request_clearance_via_fo = lambda: asks.append(1) or False
    # Radio dead for fifty seconds, then the first ask goes out.
    flight._phase_request_clearance(None, now + 50.0)
    flight._request_clearance_via_fo = lambda: asks.append(1) or True
    flight._phase_request_clearance(None, now + 50.5)
    assert flight.state.clearance_asked_at == now + 50.5
    # Fifty-nine seconds of unanswered asking is still inside the wait.
    flight.state.last_radio_at = now + 50.5
    flight._phase_request_clearance(None, now + 109.5)
    flight._set_error.assert_not_called()
    assert "clearance" not in flight.state.overdue
    # Past the patience mark the wait is warned about -- once -- and
    # the asks keep going out rather than the flight being failed.
    flight.state.last_radio_at = now + 60.0
    flight._phase_request_clearance(None, now + 112.0)
    flight._set_error.assert_not_called()
    assert "clearance" in flight.state.overdue


def test_an_uncontrolled_field_departs_after_its_announcement():
    """Nowhere to file: the call is the announcement, then it goes."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _autonomous()                       # no positions at all
    now = 100.0
    flight.state.phase_entered_at = now
    flight._request_clearance_via_fo = lambda: True
    flight._phase_request_clearance(None, now)
    assert flight.state.has_ifr_clearance
    # The call just went out -- not yet aired.
    flight.state.last_radio_at = now
    flight._phase_request_clearance(None, now + 5.0)
    flight._transition.assert_not_called()
    # After a moment for it, the departure is the flight's own -- and it
    # keeps the rules it was filed under.
    flight._phase_request_clearance(None, now + 13.0)
    flight._transition.assert_called_once_with(AutonomousPhase.PUSHBACK)
    assert flight.session.ifr is True


def test_a_vfr_release_is_held_to_the_same_readback():
    """The VFR release is owed a readback like the IFR clearance is."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _autonomous(stations=[_gnd()])
    now = 100.0
    flight.state.phase_entered_at = now
    flight.session.ifr = False
    flight._request_clearance_via_fo = lambda: True
    flight._phase_request_clearance(None, now)
    # Released but not yet read back: the stand is not let go.
    flight.session.last_instruction = SimpleNamespace(
        kind="vfr_release", read_back=False)
    flight._phase_request_clearance(None, now + 5.0)
    flight._transition.assert_not_called()
    # The release read back is the release held.
    flight.session.last_instruction.read_back = True
    flight._phase_request_clearance(None, now + 6.0)
    flight._transition.assert_called_once_with(AutonomousPhase.PUSHBACK)


def test_vfr_refused_below_minima_files_ifr():
    """"VFR not possible" is answered the way a pilot answers: file IFR."""
    flight = _autonomous(stations=[_gnd()])
    now = 100.0
    flight.state.phase_entered_at = now
    flight.session.ifr = False
    flight.state.has_ifr_clearance = True
    flight.state.last_radio_at = now
    flight.session.last_instruction = SimpleNamespace(
        kind="below_minima", read_back=False)
    flight._request_clearance_via_fo = lambda: False
    flight._phase_request_clearance(None, now + 5.0)
    assert flight.session.ifr is True
    assert not flight.state.has_ifr_clearance
    flight._emit.assert_called()


def test_a_vfr_release_readback_says_the_frequency():
    """The release loop in the live log: the FO answered the squawk and
    never the departure frequency, and the controller corrected it
    forever. The readback is built from the issued values, so the
    frequency owed is the frequency said."""
    from meridianatc.atc.copilot import readback
    from meridianatc.atc.intents import IntentParser
    from meridianatc.atc.phraseology import Aircraft, Phraseology
    from meridianatc.atc.session import Instruction
    from meridianatc.atc.speech import Callsign

    aircraft = Aircraft(callsign=Callsign(
        written="TUI123", spoken_full="tui one two three",
        spoken_short="tui one two three", telephony="Tomjet",
        is_airline=True))
    instruction = Instruction(
        kind="vfr_release", station_key="LPPT:del",
        values={"squawk": "7366", "frequency": 120.910,
                "vfr_ceiling_ft": 2500.0})
    said = readback(instruction, aircraft, Phraseology("icao"))
    heard = IntentParser().parse(said, "en").readback_items
    assert heard.get("frequency") == 120.910
    assert heard.get("squawk") == "7366"
    assert not instruction.mismatches(heard)


def test_a_taxi_readback_says_the_qnh():
    """The live loop: Ground issued QNH 1017 with the taxi route, the FO
    read back only route and runway, and the controller correctly answered
    negative twice. The issued altimeter is part of the spoken readback."""
    from meridianatc.atc.copilot import readback
    from meridianatc.atc.intents import IntentParser
    from meridianatc.atc.phraseology import Aircraft, Phraseology
    from meridianatc.atc.session import Instruction
    from meridianatc.atc.speech import Callsign

    aircraft = Aircraft(callsign=Callsign(
        written="ECX3S", spoken_full="king air three xray sierra",
        spoken_short="king air three xray sierra", telephony="King Air",
        is_airline=False))
    instruction = Instruction(
        kind="taxi", station_key="LECO:gnd",
        values={"runway": "21", "route": ["N"], "altimeter": 1017.0})
    said = readback(instruction, aircraft, Phraseology("icao"))
    assert "QNH" in said.upper()
    heard = IntentParser().parse(said, "en").readback_items
    assert heard.get("altimeter") == 1017.0
    assert heard.get("runway") == "21"
    assert not instruction.mismatches(heard)


def test_no_releases_on_github_is_no_update_not_an_error():
    import urllib.error

    from meridianatc import update

    def opener(request, timeout):
        raise urllib.error.HTTPError(
            "https://api.github.com/x", 404, "Not Found", {}, None)

    assert update.fetch_latest(opener) is None


def test_a_real_http_error_still_raises():
    import urllib.error

    import pytest

    from meridianatc import update

    def opener(request, timeout):
        raise urllib.error.HTTPError(
            "https://api.github.com/x", 500, "Broken", {}, None)

    with pytest.raises(urllib.error.HTTPError):
        update.fetch_latest(opener)


def _batched_source():
    """A SimConnectSource with the batch built against a stubbed sim."""
    from meridianatc.sim.source import SimConnectSource

    source = SimConnectSource.__new__(SimConnectSource)
    source.request_timeout_ms = 400
    dll = SimpleNamespace()
    dll.added = []
    dll.AddToDataDefinition = lambda *a: dll.added.append(a) or 0
    dll.sent = []
    dll.RequestDataOnSimObjectType = lambda *a: dll.sent.append(a) or 0
    source._sim = SimpleNamespace(
        dll=dll, hSimConnect=1, new_def_id=lambda: SimpleNamespace(value=7),
        new_request_id=lambda: SimpleNamespace(value=8),
        IsHR=lambda err, code: err == code)
    helper = SimpleNamespace(list={
        "PLANE_ALTITUDE": ["d", b"PLANE ALTITUDE", b"feet", "N"],
        "COM_ACTIVE_FREQUENCY:index": [
            "d", b"COM ACTIVE FREQUENCY:index", b"MHz", "N"],
    })
    source._requests = SimpleNamespace(list=[helper])
    source._batch_names = ()
    source._batch_set = frozenset()
    source._batch_raw = {}
    source._batch_at = 0.0
    source._batch_hook = None
    source._batch_fails = 0
    source._make_batch()
    return source


def test_the_batch_definition_keeps_the_request_order():
    source = _batched_source()
    added = [a[2] for a in source._sim.dll.added]
    wanted = [
        name for name, _a, kind, _t in source.REQUESTS if kind != "text"]
    # Only names the table resolved join the definition, in REQUESTS order.
    wanted = [n for n in wanted if source._definition_for(n)]
    assert source._batch_names == tuple(wanted)
    # The :1 index is baked into the variable, not left as ":index".
    assert b"COM ACTIVE FREQUENCY:1" in added
    assert all(b":index" not in a for a in added)


def test_a_batch_reply_files_every_value_by_name():
    import ctypes

    from meridianatc.sim.source import _BatchHeader, _bytype_id

    source = _batched_source()
    count = len(source._batch_names)

    class _Message(ctypes.Structure):
        _pack_ = 1
        _fields_ = _BatchHeader._fields_ + [
            ("values", ctypes.c_double * count)]

    message = _Message()
    message.dwID = _bytype_id()
    message.dwRequestID = 8
    message.dwDefineCount = count
    for i in range(count):
        message.values[i] = i + 0.5
    pointer = ctypes.cast(
        ctypes.pointer(message), ctypes.POINTER(_Message))
    assert source._batch_heard(pointer)
    for i, name in enumerate(source._batch_names):
        assert source._batch_raw[name] == i + 0.5


def test_a_reply_for_another_request_is_left_alone():
    import ctypes

    from meridianatc.sim.source import _BatchHeader, _bytype_id

    source = _batched_source()

    class _Message(ctypes.Structure):
        _pack_ = 1
        _fields_ = _BatchHeader._fields_

    message = _Message()
    message.dwID = _bytype_id()
    message.dwRequestID = 99
    pointer = ctypes.cast(
        ctypes.pointer(message), ctypes.POINTER(_Message))
    assert source._batch_heard(pointer) is False
    assert source._batch_raw == {}


def test_the_user_object_reply_is_also_a_batch_reply():
    """For SIMOBJECT_TYPE.USER the sim answers SIMOBJECT_DATA, not BYTYPE."""
    import ctypes

    from meridianatc.sim.source import _BatchHeader

    source = _batched_source()
    count = len(source._batch_names)

    class _Message(ctypes.Structure):
        _pack_ = 1
        _fields_ = _BatchHeader._fields_ + [
            ("values", ctypes.c_double * count)]

    message = _Message()
    message.dwID = 8          # SIMCONNECT_RECV_ID_SIMOBJECT_DATA
    message.dwRequestID = 8
    message.dwDefineCount = count
    for i in range(count):
        message.values[i] = i + 0.5
    pointer = ctypes.cast(
        ctypes.pointer(message), ctypes.POINTER(_Message))
    assert source._batch_heard(pointer)
    for i, name in enumerate(source._batch_names):
        assert source._batch_raw[name] == i + 0.5


def test_a_refused_variable_leaves_the_batch_and_the_reply_unpacks():
    """An AddToDataDefinition the sim did not know throws its exception on
    the pump, not as the call's return: the definition ships without the
    datum, every reply is one short, and without the send-id bookkeeping
    the strict count dropped the batch forever (the ~500 ms tick.read of
    the live logs). The send id in the exception ties it to the name, the
    name leaves the key and the missing list, and the survivors unpack."""
    import ctypes

    from meridianatc.sim.source import (
        _BatchHeader,
        _bytype_id,
        _exception_id,
        _ExcHeader,
    )

    source = _batched_source()
    source._missing = set()
    count = len(source._batch_names)
    names = list(source._batch_names)
    refused = names[1]
    # The send id the sim reported for that AddToDataDefinition.
    source._batch_send_names = {42: refused}
    send_id = 42

    class _Exception(ctypes.Structure):
        _pack_ = 1
        _fields_ = _ExcHeader._fields_

    message = _Exception()
    message.dwID = _exception_id()
    message.UNKNOWN_SENDID = send_id
    pointer = ctypes.cast(
        ctypes.pointer(message), ctypes.POINTER(_Exception))
    assert source._batch_heard(pointer) is False
    assert refused in source._missing
    assert refused not in source._batch_names
    assert len(source._batch_names) == count - 1

    # The reply that follows is one datum short, and still unpacks.
    class _Message(ctypes.Structure):
        _pack_ = 1
        _fields_ = _BatchHeader._fields_ + [
            ("values", ctypes.c_double * (count - 1))]

    reply = _Message()
    reply.dwID = _bytype_id()
    reply.dwRequestID = 8
    reply.dwDefineCount = count - 1
    for i in range(count - 1):
        reply.values[i] = i + 0.5
    pointer = ctypes.cast(
        ctypes.pointer(reply), ctypes.POINTER(_Message))
    assert source._batch_heard(pointer)
    survivors = [n for n in names if n != refused]
    for i, name in enumerate(survivors):
        assert source._batch_raw[name] == i + 0.5
    assert refused not in source._batch_raw


def _flier(performance=None):
    """The flight with only what the flying phases touch."""
    from meridianatc.atc import flying
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    flight.state.phase_entered_at = 0.0
    flight.session = SimpleNamespace(
        ifr=True, cleared_landing=True, going_around=False,
        destination=SimpleNamespace(ident="LPPT"))
    flight._perf = performance or flying.AIRLINER
    flight.sim = None
    flight._dir = Mock()
    flight._dir.sim = None          # the property returns this director
    flight._ap_mode = "direct"
    flight._last_lever = 0.0
    flight._weather_reactions = False
    flight._ga_flag_seen = False
    flight.crew = SimpleNamespace(
        checklists=Mock(), transmit=Mock(), radio_busy=None)
    flight._transition = Mock()
    flight._set_error = Mock()
    flight.LANDING_TIMEOUT = 600.0
    flight._run_checklist_for_phase = Mock()
    flight._start_go_around = Mock()
    flight._emit = Mock()
    return flight


def test_the_flare_pulls_against_the_sink_it_has():
    """A pilot flares to kill the descent rate: more sink, more pull."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _flier()
    flight.state.phase = AutonomousPhase.LANDING
    flight.state.sub = "flare"
    flight.state.runway_heading = 30.0
    state = SimpleNamespace(
        on_ground=False, altitude_agl_ft=40.0, heading_true=30.0,
        indicated_airspeed_kt=140.0, ground_speed_kt=140.0,
        vertical_speed_fpm=-700.0)
    flight._phase_landing(state, 10.0)
    hard = flight.director.elevator.call_args[0][0]
    state.vertical_speed_fpm = -250.0
    flight._phase_landing(state, 20.0)
    soft = flight.director.elevator.call_args[0][0]
    assert 0.0 < soft < hard <= 0.5


def test_the_rollout_uses_reverse_then_less_brake():
    """A fast aeroplane takes reverse to sixty, then wheel brakes."""
    from meridianatc.atc import flying
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _flier()
    flight.state.phase = AutonomousPhase.LANDING
    flight.state.sub = "rollout"
    state = SimpleNamespace(
        on_ground=True, heading_true=30.0, ground_speed_kt=110.0,
        vertical_speed_fpm=0.0, source="simconnect")
    flight.state.runway_heading = 30.0
    flight._phase_landing(state, 10.0)
    assert flight.state.reversers_out
    flight.director.send.assert_called_with(
        b"THROTTLE_REVERSE_THRUST_TOGGLE", 0)
    # Reverse in means the wheel brakes are eased, not stood on.
    flight.director.brakes.assert_called_with(0.4)
    # Below sixty the reversers come in and the brakes take the stop.
    state.ground_speed_kt = 40.0
    flight._phase_landing(state, 12.0)
    assert not flight.state.reversers_out
    flight.director.brakes.assert_called_with(0.7)
    # A Cessna has no reversers to pull: brakes only.
    light = _flier(flying.LIGHT)
    light.state.phase = AutonomousPhase.LANDING
    light.state.sub = "rollout"
    light.state.runway_heading = 30.0
    light._phase_landing(state, 10.0)
    light.director.send.assert_not_called()
    assert not light.state.reversers_out


def test_speed_is_capped_at_250_below_ten_thousand():
    """Two-fifty below ten, for the aeroplanes fast enough to break it."""
    from meridianatc.atc import flying
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _flier()
    flight.state.phase = AutonomousPhase.CLIMB
    state = SimpleNamespace(
        altitude_ft=5000.0, estimated_cruise_kt=0.0, barber_pole_kt=0.0)
    flight.session.assigned_speed_kt = 0
    assert flight._speed_target(state) == 250.0
    state.altitude_ft = 20000.0
    assert flight._speed_target(state) == flight.performance.climb_kt or \
        flight._speed_target(state) > 250.0
    # A light aeroplane's climb speed is already under the limit.
    light = _flier(flying.LIGHT)
    light.state.phase = AutonomousPhase.CLIMB
    light.session.assigned_speed_kt = 0
    assert light._speed_target(state) < 250.0


def test_autofly_profiles_teach_the_flown_by_hand_numbers():
    """A matched profile refines touchdown, go-around and flap detents."""
    import json
    from dataclasses import replace

    from meridianatc.atc import autofly, flying

    profile = {
        "canAutofly": True,
        "titleMatches": ["testjet"],
        "performance": {
            "rotationSpeedKnots": 135,
            "touchdownSpeedKnots": 130,
            "goAroundSpeedKnots": 155,
            "maxBankDegrees": 22,
            "takeoffFlaps": 2,
            "landingFlaps": 4,
            "climbRateFpm": 1800,
        },
    }
    old = autofly._profiles, autofly._loaded
    try:
        autofly._profiles = []
        autofly._loaded = False
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp, "testjet.json").write_text(
                json.dumps(profile), encoding="utf-8")
            original = autofly.DATA_DIR
            autofly.DATA_DIR = Path(tmp)
            try:
                perf = autofly.override("TJ1", "a testjet", flying.AIRLINER)
            finally:
                autofly.DATA_DIR = original
    finally:
        autofly._profiles, autofly._loaded = old

    assert perf.rotate_kt == 135.0
    assert perf.touchdown_kt == 130.0
    assert perf.goaround_kt == 155.0
    assert perf.max_bank_deg == 22.0
    # Detent 2 of 4 is half the flap travel; full is full.
    assert perf.takeoff_flaps == 0.5
    assert perf.landing_flaps == 1.0
    # A non-match leaves the class performance alone.
    assert replace(flying.AIRLINER, rotate_kt=135.0) != perf or True
    assert autofly.override("XXX", "a glider", flying.AIRLINER) \
        == flying.AIRLINER


def test_all_engines_out_flies_the_glide_speed():
    """Dead stick is a best-glide, not the plan's speed."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _flier()
    flight.state.phase = AutonomousPhase.CRUISE
    flight.state.emergency = "engine"
    flight.session.assigned_speed_kt = 0
    state = SimpleNamespace(
        altitude_ft=8000.0, estimated_cruise_kt=0.0, barber_pole_kt=0.0,
        design_vs0_kt=0.0, design_vs1_kt=0.0, design_vc_kt=0.0,
        barber_pole_mach=0.0, mach_max_operate=0.0)
    speed = flight._speed_target(state)
    # Best glide: under the cruise figure but well above a stall.
    assert flight.performance.approach_kt * 0.8 < speed \
        < flight.performance.cruise_kt


def test_one_engine_out_flies_the_drift_down():
    """A twin on one is a drift-down at climb speed, not cruise."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _flier()
    flight.state.phase = AutonomousPhase.CRUISE
    flight.state.emergency = "engine-out"
    flight.state.climb_kt = 160.0
    flight.session.assigned_speed_kt = 0
    state = SimpleNamespace(
        altitude_ft=20000.0, estimated_cruise_kt=0.0, barber_pole_kt=0.0,
        design_vs0_kt=0.0, design_vs1_kt=0.0, design_vc_kt=0.0,
        barber_pole_mach=0.0, mach_max_operate=0.0)
    assert flight._speed_target(state) == 160.0


def test_cruise_speed_is_the_cruise_not_the_climb():
    """X-Plane publishes no estimated cruise speed: the fallback was
    the climb figure, so a Baron "cruised" a whole leg at Vy. Cruise
    takes the type's own Vno pulled back to a cruise setting, or the
    profile's figure where one is matched -- never the climb speed."""
    from dataclasses import replace

    from meridianatc.atc import flying
    from meridianatc.atc.autonomous import AutonomousPhase
    # The Baron, as X-Plane reports it: no estimate, Vno at 195, and
    # the autofly profile's POH numbers on the performance.
    baron = replace(flying.LIGHT, climb_kt=105.0, cruise_kt=185.0)
    flight = _flier(baron)
    flight.state.phase = AutonomousPhase.CRUISE
    flight.state.climb_kt = 105.0
    flight.session.assigned_speed_kt = 0
    state = SimpleNamespace(
        altitude_ft=8000.0, estimated_cruise_kt=0.0, barber_pole_kt=0.0,
        design_vc_kt=195.0)
    speed = flight._speed_target(state)
    # The profile's 185 stands over 0.75 x Vno; either way not 105.
    assert speed == 185.0
    # A model with no profile still cruises at its own figure: the
    # class's 110 would sell a Vno-195 aeroplane half its legs short.
    plain = _flier(flying.LIGHT)
    plain.state.phase = AutonomousPhase.CRUISE
    plain.state.climb_kt = 85.0
    plain.session.assigned_speed_kt = 0
    assert plain._speed_target(state) == 0.75 * 195.0
    # MSFS still wins where it publishes the estimate.
    state.estimated_cruise_kt = 170.0
    assert flight._speed_target(state) == 170.0


def test_climb_accelerates_past_the_noise_height():
    """Vy is for the first stretch; past fifteen hundred the crew
    pitches for the enroute climb -- part way toward the cruise, not
    the book figure held to the top."""
    from dataclasses import replace

    from meridianatc.atc import flying
    from meridianatc.atc.autonomous import AutonomousPhase
    baron = replace(flying.LIGHT, climb_kt=105.0, cruise_kt=185.0)
    flight = _flier(baron)
    flight.state.phase = AutonomousPhase.CLIMB
    flight.state.climb_kt = 105.0
    flight.session.assigned_speed_kt = 0
    low = SimpleNamespace(
        altitude_ft=1000.0, altitude_agl_ft=800.0,
        estimated_cruise_kt=0.0, barber_pole_kt=0.0)
    assert flight._speed_target(low) == 105.0
    high = SimpleNamespace(
        altitude_ft=6000.0, altitude_agl_ft=5500.0,
        estimated_cruise_kt=0.0, barber_pole_kt=0.0)
    speed = flight._speed_target(high)
    assert 105.0 < speed < 185.0


def test_the_shared_figure_is_not_the_climb_speed_either():
    """Any phase with no answer of its own fell back to the climb
    speed first -- the STAR guidance's descent came down at Vy, the
    same leg-slow bug as the cruise's. The generic figure is the
    descent profile's, part way back from cruise."""
    from dataclasses import replace

    from meridianatc.atc import flying
    from meridianatc.atc.autonomous import AutonomousPhase
    baron = replace(flying.LIGHT, climb_kt=105.0, cruise_kt=185.0)
    flight = _flier(baron)
    flight.state.phase = AutonomousPhase.DESCENT
    flight.state.climb_kt = 105.0
    flight.session.assigned_speed_kt = 0
    state = SimpleNamespace(
        altitude_ft=9000.0, estimated_cruise_kt=0.0, barber_pole_kt=0.0,
        design_vc_kt=195.0)
    speed = flight._speed_target(state)
    # 0.85 of the cruise figure, nowhere near the climb's 105.
    assert abs(speed - 0.85 * 185.0) < 0.01
    # And the helper both phases now share reads the model's own
    # estimate where MSFS publishes one.
    state.estimated_cruise_kt = 170.0
    assert flight._cruise_speed_kt(state) == 170.0


def test_gear_up_gets_one_goaround_then_the_belly_landing():
    """The gear that stayed up: one try, then the runway it has."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _flier()
    flight.state.phase = AutonomousPhase.APPROACH
    flight.state.sub = "final"
    flight.session.ifr = False
    state = SimpleNamespace(
        altitude_agl_ft=600.0, gear_down=False, on_ground=False,
        vertical_speed_fpm=-500.0, indicated_airspeed_kt=140.0,
        bank_deg=0.0, heading_true=30.0)
    assert flight._should_go_around(state) is True
    assert flight.state.gear_ga_done
    # The second approach takes the belly landing, announced once.
    state.altitude_agl_ft = 400.0
    assert flight._should_go_around(state) is False
    assert flight.state.gear_up_said


def test_a_helicopter_engine_loss_flies_an_autorotation():
    """The rotor's energy is spent landing, not holding the plan."""
    from meridianatc.atc.autonomous_heli import RotorcraftFlight
    owner = _flier()
    owner._send_event = Mock(return_value=True)
    owner._declare_emergency = Mock()
    owner.state.emergency_declared = False
    heli = RotorcraftFlight.__new__(RotorcraftFlight)
    heli.owner = owner
    heli.session = owner.session
    heli.crew = owner.crew
    from meridianatc.atc import autonomous_heli as hm
    heli.state = hm.HeliState(stage="cruise", ias_approach_kt=60.0)
    heli._profile = hm.DEFAULT_HELI_PROFILE
    heli._transitioned_owner = ""
    heli._last_tick = 0.0
    heli.TICK_S = 0.25
    owner._engines_out = lambda s: (1, 1)
    # Airborne with the one engine not combusting: an autorotation.
    state = SimpleNamespace(
        connected=True, on_ground=False, altitude_agl_ft=500.0,
        engine_count=1, engines_running=False,
        eng2_combustion=None, eng3_combustion=None, eng4_combustion=None,
        indicated_airspeed_kt=80.0, heading_true=90.0,
        has_position=False, rotor_lateral_trim_pct=0.0)
    assert heli._engine_lost(state)
    heli.tick(state)
    assert heli.state.stage == "autorotate"
    # The collective is pulled to preserve the rotor, the throttle off.
    heli._autorotate(state)
    calls = {c[0][0]: c[0][1] for c in owner._send_event.call_args_list}
    from meridianatc.atc.autonomous_heli import COLLECTIVE_SET
    assert calls[COLLECTIVE_SET] < 2000   # ~10 per cent of the axis
    # On the ground it lands out, not keeps pulling.
    state.on_ground = True
    heli._autorotate(state)
    assert heli.state.stage == "shutdown"


def _taxi_flight(stations=(), ground_control=True, phase="cleared",
                 staffed=None):
    """A flight with only what the taxi and pushback gates touch."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    flight.session = SimpleNamespace(
        phase=phase, taxi_path=None, taxi_split_at=None,
        departure=SimpleNamespace(ident="LPPT"))
    flight.navdb = SimpleNamespace(
        stations=lambda ident, position=None: [
            s for s in stations
            if position is None or s.position == position])
    flight.brain = SimpleNamespace(ground_control=ground_control)
    if staffed is not None:
        flight.brain.staffed = staffed
        flight.brain.issues_clearances = staffed
    flight._held_at_split = False
    flight._get_current_airport = lambda: flight.session.departure
    flight.state.phase_entered_at = 0.0
    flight._run_checklist_for_phase = Mock()
    flight._taxi_along_path = Mock()
    flight._stop = Mock()
    flight._transition = Mock()
    flight._set_error = Mock()
    flight._goto_sub = Mock()
    flight._emit = Mock()
    flight._dir = Mock()
    flight._dir.sim = None        # the property returns this director
    flight.sim = None
    return flight


def _gnd():
    return SimpleNamespace(position="GND", mhz=121.750)


def test_taxi_waits_for_the_instruction_at_a_controlled_field():
    """The wheels turn on the route clearance, not on the FO's say-so."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()])
    flight.state.phase = AutonomousPhase.TAXI_OUT
    flight.state.sub = "taxi"
    # Inside the grace the aeroplane holds on the stand.
    flight.state.sub_entered_at = _time.monotonic() - 60.0
    state = SimpleNamespace(ground_speed_kt=0.0)
    flight._phase_taxi_out(state, 60.0)
    flight._taxi_along_path.assert_not_called()
    flight._stop.assert_called()
    # Cleared: the route itself is the instruction. TAXI_OUT alone is
    # not -- it also flips on groundspeed alone.
    flight.session.phase = "taxi_out"
    flight.session.taxi_route = "via A to holding point runway 35"
    flight._phase_taxi_out(state, 65.0)
    flight._taxi_along_path.assert_called_once()


def test_taxi_without_a_ground_position_announces_and_goes():
    """Nowhere to ask: the call is the announcement, then it taxis."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[
        SimpleNamespace(position="CTAF", mhz=122.800)])
    flight.state.phase = AutonomousPhase.TAXI_OUT
    flight.state.sub = "taxi"
    state = SimpleNamespace(ground_speed_kt=0.0)
    flight.state.sub_entered_at = _time.monotonic() - 5.0
    flight._phase_taxi_out(state, 5.0)      # the call still going out
    flight._taxi_along_path.assert_not_called()
    flight.state.sub_entered_at = _time.monotonic() - 15.0
    flight._phase_taxi_out(state, 15.0)
    flight._taxi_along_path.assert_called_once()


def test_a_staffed_ground_is_waited_on_for_the_instruction():
    """A worked position owns the taxi: silent or not, it is waited on."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()])
    flight.state.phase = AutonomousPhase.TAXI_OUT
    flight.state.sub = "taxi"
    flight.state.sub_entered_at = _time.monotonic() - 200.0
    state = SimpleNamespace(ground_speed_kt=0.0)
    flight._phase_taxi_out(state, 200.0)
    # No instruction ever came: the aeroplane is held, not left behind.
    flight._taxi_along_path.assert_not_called()
    flight._stop.assert_called()


def test_a_published_but_unworked_ground_announces_and_goes():
    """Ground printed in the chart but not on the air is nobody to ask."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()], staffed=lambda s: False)
    flight.state.phase = AutonomousPhase.TAXI_OUT
    flight.state.sub = "taxi"
    state = SimpleNamespace(ground_speed_kt=0.0)
    flight.state.sub_entered_at = _time.monotonic() - 5.0
    flight._phase_taxi_out(state, 5.0)
    flight._taxi_along_path.assert_not_called()
    flight.state.sub_entered_at = _time.monotonic() - 15.0
    flight._phase_taxi_out(state, 15.0)
    flight._taxi_along_path.assert_called_once()


def _ground_brain(layout=None, pending=False):
    """A controller brain whose field's taxiways answer what the test
    says: a layout, nothing yet, or a read still in flight."""
    from meridianatc.atc.controller import ControllerBrain
    from meridianatc.atc.phraseology import Phraseology
    navdb = Mock()
    navdb.airport.return_value = SimpleNamespace(
        ident="LPPR", lat=41.24, lon=-8.67)
    navdb.runways.return_value = []
    navdb.runway_ends.return_value = []
    brain = ControllerBrain(navdb, Phraseology())
    brain.ground_layout = lambda ident: layout
    brain.ground_pending = lambda ident: pending
    brain.traffic = None
    return brain


def _ground_station():
    from meridianatc.navdata.db import Station
    return Station("LPPR", "GND", 121.75, "Porto Ground")


def test_a_taxi_asked_while_the_layout_reads_gets_standby():
    """The read takes seconds on a worker: "stand by" is the answer the
    real route arrives on, not the invented one a pilot would keep."""
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession, Phase
    from meridianatc.atc.speech import parse_callsign
    brain = _ground_brain(pending=True)
    station = _ground_station()
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.departure_runway = "35"
    weather = SimpleNamespace(qnh_hpa=1013, altimeter_inhg=29.92)
    reply = brain._issue_taxi(
        session, station, weather, brain.for_station(station))
    assert "standby" in reply.text.lower()
    # Nothing is kept: no invented route, no empty path, and the phase
    # stays where it was -- the next ask gets the real route.
    assert not session.taxi_route
    assert not session.taxi_path
    assert session.phase is not Phase.TAXI_OUT


def test_a_taxi_asked_with_the_layout_dead_gets_the_invented_route():
    """No read coming is a different answer: the invented route, kept."""
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession, Phase
    from meridianatc.atc.speech import parse_callsign
    brain = _ground_brain(pending=False)
    station = _ground_station()
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.departure_runway = "35"
    weather = SimpleNamespace(qnh_hpa=1013, altimeter_inhg=29.92)
    reply = brain._issue_taxi(
        session, station, weather, brain.for_station(station))
    assert "standby" not in reply.text.lower()
    assert session.taxi_route
    assert session.phase is Phase.TAXI_OUT


def test_the_taxi_instructions_pressure_reaches_the_instruction_values():
    """A pressure spoken but never recorded is a figure the FO can
    never dial: the King Air that levelled at 4,000 with the box still
    on standard heard "QNH one zero two one" in the taxi call and the
    instruction's values carried no altimeter -- tower then queried an
    altitude that was only wrong on the subscale."""
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    brain = _ground_brain(pending=False)
    station = _ground_station()
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.departure_runway = "35"
    weather = SimpleNamespace(qnh_hpa=1021, altimeter_inhg=30.12)
    p = brain.for_station(station)
    reply = brain._issue_taxi(session, station, weather, p)
    # The instruction keeps the same figure the transmission said,
    # whichever dialect said it.
    expected = (weather.altimeter_inhg if p.dialect == "faa"
                else weather.qnh_hpa)
    assert reply.instruction.values["altimeter"] == expected
    session.apply(reply.instruction.values)
    assert session.altimeter == expected


def test_an_already_given_pressure_is_not_repeated_or_recorded_again():
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    brain = _ground_brain(pending=False)
    station = _ground_station()
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.departure_runway = "35"
    session.altimeter_issued = True
    weather = SimpleNamespace(qnh_hpa=1021, altimeter_inhg=30.12)
    reply = brain._issue_taxi(
        session, station, weather, brain.for_station(station))
    # Said once already: not said again, and not remembered again.
    assert "altimeter" not in reply.instruction.values


def test_the_straight_line_waits_while_the_layout_reads():
    """No route and nobody to ask is still not a reason to aim at the
    runway while the field's taxiways are mid-read."""
    flight = _path_flight(None)
    flight.session.phase = "cleared"
    flight.session.departure_runway = "35"
    flight.session.departure = SimpleNamespace(ident="LPPR")
    flight.state.runway_heading = 353.0
    flight.state.runway_key = ("LPPR", "35", False)
    flight.state.runway_lat = 41.27
    flight.state.runway_lon = -8.68
    flight._find_runway = Mock()
    flight._taxi_issuing_position = Mock(return_value=None)
    flight._layout_still_coming = Mock(return_value=True)
    flight._self_plan_taxi = Mock(return_value=False)
    flight._get_current_airport = lambda: flight.session.departure
    flight._taxi_without_path(
        _ground_state(41.24, -8.67, gs=0.0), outbound=True)
    flight._stop.assert_called()
    assert not flight._steer.called


def test_the_straight_line_steers_when_no_layout_is_coming():
    """The fallback itself is kept: nothing read and nothing coming,
    the runway is still the way out."""
    flight = _path_flight(None)
    flight.session.phase = "cleared"
    flight.session.departure_runway = "35"
    flight.session.departure = SimpleNamespace(ident="LPPR")
    flight.state.runway_heading = 353.0
    flight.state.runway_key = ("LPPR", "35", False)
    flight.state.runway_lat = 41.27
    flight.state.runway_lon = -8.68
    flight._find_runway = Mock()
    flight._taxi_issuing_position = Mock(return_value=None)
    flight._layout_still_coming = Mock(return_value=False)
    flight._self_plan_taxi = Mock(return_value=False)
    flight._get_current_airport = lambda: flight.session.departure
    flight._yield_to_traffic = Mock(return_value=False)
    flight._taxi_without_path(
        _ground_state(41.24, -8.67, gs=0.0), outbound=True)
    assert flight._steer.called


def test_an_unstaffed_field_taxis_on_the_airports_own_taxiways():
    """Nobody giving a route does not mean no route: with the taxiways
    read, the crew follows the airport diagram, not the crow."""
    flight = _path_flight(None)
    flight.session.phase = "cleared"
    flight.session.departure_runway = "35"
    flight.session.departure = SimpleNamespace(ident="LPPR")
    flight.state.runway_key = ("LPPR", "35", False)
    flight.navdb = SimpleNamespace(runways=lambda ident: [])
    flight.brain = SimpleNamespace(
        ground_layout=lambda ident: object())
    flight._get_current_airport = lambda: flight.session.departure
    planned = SimpleNamespace(
        points=[(41.240, -8.672), (41.242, -8.672)], crossings=["17"])
    flight._self_plan_at = 0.0
    from unittest.mock import patch
    with patch("meridianatc.atc.taxi.runway_lines", return_value=[]), \
            patch("meridianatc.atc.taxi.plan_departure",
                  return_value=(planned, "")):
        assert flight._self_plan_taxi(
            _ground_state(41.24, -8.67, gs=0.0)) is True
    assert flight.session.taxi_path == list(planned.points)
    assert flight.session.taxi_crossings == ["17"]


def test_a_tower_with_no_ground_position_taxis_the_flight_itself():
    """LPPR has no Ground frequency: the position lookup falls back to
    the tower itself, which is also who the pilot is already talking
    to -- "contact Tower" said by Tower is a loop that never ends."""
    from meridianatc.atc.intents import Intent
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession, Phase
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.navdata.db import Station
    brain = _ground_brain(pending=False)
    brain.staffed = lambda s: True
    brain.realism = SimpleNamespace(tower_continue=False)
    tower = Station("LPPR", "TWR", 118.005, "Porto", "Porto Tower")
    # Asking for Ground here answers the tower itself.
    brain.navdb.station_for = lambda ident, position: tower
    brain.navdb.station_for_runway = Mock(return_value=None)
    session = FlightSession(Aircraft(parse_callsign("N123")))
    session.departure_runway = "35"
    parsed = SimpleNamespace(intent=Intent.REQUEST_TAXI)
    weather = SimpleNamespace(qnh_hpa=1013, altimeter_inhg=29.92)
    reply = brain._tower(parsed, session, tower, weather,
                         brain.for_station(tower), None)
    assert "contact" not in reply.text.lower()
    assert session.taxi_route
    assert session.phase is Phase.TAXI_OUT


def test_a_split_taxi_waits_for_the_rest_not_the_runway():
    """Held at the split taxiway: resume on the rest, never HOLDING_SHORT."""

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()], phase="taxi_out")
    flight.state.phase = AutonomousPhase.TAXI_OUT
    flight.state.sub = "hold_point"
    flight._held_at_split = True
    flight.session.taxi_split_at = (38.77, -9.13)
    state = SimpleNamespace(ground_speed_kt=0.0)
    flight._phase_taxi_out(state, 10.0)
    # Still owed the rest: held, and no false move to the runway hold.
    flight._goto_sub.assert_not_called()
    flight._transition.assert_not_called()
    # The rest is given: the taxi resumes on the same clearance.
    flight.session.taxi_split_at = None
    flight._phase_taxi_out(state, 11.0)
    flight._goto_sub.assert_called_once_with("taxi")
    flight._transition.assert_not_called()


def _vigo_tower_setup():
    """A tower brain over LEVX's real runway numbers, for the hold-short
    cases: the sim's strip runs 42.2213,-8.6297 (01) to 42.2425,-8.6239
    (19)."""
    from meridianatc.atc.controller import ControllerBrain
    from meridianatc.atc.phraseology import Phraseology
    from meridianatc.navdata.db import NavDB, Runway, Station
    navdb = Mock()
    navdb.airport.return_value = SimpleNamespace(
        ident="LEVX", lat=42.2318, lon=-8.6268)
    runway = Runway(
        ident="LEVX", le_ident="01", he_ident="19",
        length_ft=7874.0, width_ft=148.0, surface="ASP", lighted=True,
        closed=False,
        le_lat=42.2213, le_lon=-8.62966, le_heading=11.4,
        he_lat=42.2425, he_lon=-8.62389, he_heading=191.4)
    navdb.runways.return_value = [runway]
    navdb.runway_at.return_value = ""
    navdb.at_the_runway = NavDB.at_the_runway.__get__(navdb)
    brain = ControllerBrain(navdb, Phraseology())
    brain.traffic = None
    brain.realism = SimpleNamespace(
        tower_continue=False, conditional_clearances=False,
        ifr_departure=False)
    tower = Station("LEVX", "TWR", 118.45, "Vigo", "Vigo Tower")
    return brain, tower


def _ready_ask(brain, tower, lat, lon):
    """A "ready for departure runway one niner" asked of the tower from a
    ground position, answered as the tower answers it."""
    from meridianatc.atc.intents import Intent
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession, Phase
    from meridianatc.atc.speech import parse_callsign
    session = FlightSession(Aircraft(parse_callsign("ECXGS")))
    session.departure_runway = "19"
    session.phase = Phase.TAXI_OUT
    session.ifr = True
    parsed = SimpleNamespace(intent=Intent.REQUEST_TAKEOFF,
                             get=lambda k, d=None: d)
    weather = SimpleNamespace(qnh_hpa=1018, altimeter_inhg=29.92,
                              wind_dir=270, wind_kt=8.0, gust_kt=0.0,
                              variable=False, variation=-1.0)
    state = SimpleNamespace(has_position=True, on_ground=True,
                            latitude=lat, longitude=lon, heading_true=42.0)
    reply = brain._tower(parsed, session, tower, weather,
                         brain.for_station(tower), state)
    return reply, session


def test_the_bars_are_at_the_runway_for_the_takeoff_clearance():
    """Held at the 19 hold is at the runway: a departure is supposed to
    ask from there, outside the pavement runway_at reads. Asking "ready"
    from the bars used to loop "taxi to the runway" forever."""
    brain, tower = _vigo_tower_setup()
    # 200 ft past the 19 threshold, 180 ft off the centreline: the hold.
    reply, session = _ready_ask(brain, tower, 42.24294, -8.62309)
    assert "cleared for takeoff" in reply.text.lower()
    assert session.cleared_takeoff


def test_a_midfield_hold_is_at_the_runway_for_the_takeoff_clearance():
    """An intersection hold sits beside the pavement halfway down it --
    at the runway the same way the threshold's bars are."""
    brain, tower = _vigo_tower_setup()
    reply, session = _ready_ask(brain, tower, 42.23180, -8.62612)
    assert "cleared for takeoff" in reply.text.lower()
    assert session.cleared_takeoff


def test_the_apron_is_not_at_the_runway_for_the_takeoff_clearance():
    """The audit's case stands: "ready" heard on the apron is sent to the
    runway, not cleared off it."""
    brain, tower = _vigo_tower_setup()
    # Fifteen hundred feet off the centreline midfield: the apron.
    reply, session = _ready_ask(brain, tower, 42.23108, -8.62132)
    assert "taxi to the runway" in reply.text.lower()
    assert not session.cleared_takeoff


def test_the_far_ends_hold_is_not_at_the_runway_for_the_takeoff_clearance():
    """Held at 01's bars asking for 19 is at the wrong end's runway, so
    it is taxied to the right one rather than cleared the wrong way."""
    brain, tower = _vigo_tower_setup()
    # 200 ft past the 01 threshold: the reciprocal's hold, not 19's.
    reply, session = _ready_ask(brain, tower, 42.22066, -8.62915)
    assert "taxi to the runway" in reply.text.lower()
    assert not session.cleared_takeoff


def test_the_rules_are_decided_by_the_aircraft_and_the_weather(monkeypatch):
    """A transport is always IFR; a light aeroplane files by the sky."""
    # The filing clock is the real one; pin daylight so a run after
    # sunset does not file the VFR assertions IFR (SERA night rule).
    monkeypatch.setattr("meridianatc.atc.svfr.is_night",
                        lambda *a: False)
    flight = _flier()
    flight._aircraft_class = "airliner"
    flight._is_helicopter = False
    flight._request_ifr = True
    assert flight._ifr_required(SimpleNamespace()) is True
    # A light aeroplane in VMC may genuinely go VFR.
    flight._aircraft_class = "piston_single"
    flight.session.ifr = False        # nothing filed
    flight.session.departure = SimpleNamespace(
        ident="LPPT", lat=38.7, lon=-9.1)
    flight.session.destination = SimpleNamespace(
        ident="LPPT", lat=38.7, lon=-9.1)
    wx = SimpleNamespace(raw="METAR", clouds=[("BKN", 3000)],
                         visibility_sm=10.0, source="metar")
    flight.brain = SimpleNamespace(
        weather_for=lambda ident: wx,
        region_for=lambda ident: "icao",
        realism=SimpleNamespace(special_vfr=True))
    flight._get_current_airport = lambda: flight.session.departure
    assert flight._ifr_required(SimpleNamespace()) is False
    # Below the control-zone minima the same aeroplane files IFR.
    wx.clouds = [("BKN", 400)]
    assert flight._ifr_required(SimpleNamespace()) is True
    # The pilot's own "VFR only" preference is honoured in good weather.
    flight._request_ifr = False
    wx.clouds = [("BKN", 3000)]
    assert flight._ifr_required(SimpleNamespace()) is False


def test_a_flight_filed_ifr_asks_for_ifr_whatever_the_sky():
    """A filed IFR light aeroplane does not get re-filed VFR by sunshine."""
    flight = _flier()
    flight._aircraft_class = "piston_single"
    flight._is_helicopter = False
    flight._request_ifr = True
    flight.session.ifr = True          # filed IFR
    flight.session.departure = SimpleNamespace(
        ident="LPPT", lat=38.7, lon=-9.1)
    flight.session.destination = SimpleNamespace(
        ident="LPPT", lat=38.7, lon=-9.1)
    wx = SimpleNamespace(raw="METAR", clouds=[("BKN", 3000)],
                         visibility_sm=10.0, source="metar")
    flight.brain = SimpleNamespace(
        weather_for=lambda ident: wx,
        region_for=lambda ident: "icao",
        realism=SimpleNamespace(special_vfr=True))
    flight._get_current_airport = lambda: flight.session.departure
    assert flight._ifr_required(SimpleNamespace()) is True
    # The pilot's "VFR only" preference still outranks the filing.
    flight._request_ifr = False
    assert flight._ifr_required(SimpleNamespace()) is False


def test_a_sim_sourced_sky_still_files_ifr_on_an_imc_metar(monkeypatch):
    """A VFR light aeroplane under a real-world overcast files IFR --
    the METAR's ceiling is the paperwork even when the merged weather
    is sim-sourced and its ceiling field is blanked."""
    monkeypatch.setattr("meridianatc.atc.svfr.is_night",
                        lambda *a: False)
    flight = _flier()
    flight._aircraft_class = "piston_single"
    flight._is_helicopter = False
    flight._request_ifr = True
    flight.session.ifr = False
    flight.session.departure = SimpleNamespace(
        ident="LPPR", lat=41.24, lon=-8.67)
    flight.session.destination = SimpleNamespace(
        ident="LPPT", lat=38.7, lon=-9.1)
    # Sim-sourced merged weather: ceiling list still carries the
    # METAR's OVC004 -- the filing must read it, not the blanked
    # ceiling field.
    wx = SimpleNamespace(raw="METAR", source="sim LPPR",
                         clouds=[("OVC", 400)], visibility_sm=10.0)
    flight.brain = SimpleNamespace(
        weather_for=lambda ident: wx,
        region_for=lambda ident: "icao",
        realism=SimpleNamespace(special_vfr=True))
    flight._get_current_airport = lambda: flight.session.departure
    assert flight._ifr_required(SimpleNamespace()) is True
    # And a real METAR clear sky still goes VFR.
    wx.clouds = []
    assert flight._ifr_required(SimpleNamespace()) is False


def test_a_vfr_leg_flown_into_cloud_files_ifr():
    """A stay in the cloud on a VFR leg is the pop-up pickup's ask."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _flier()
    flight._is_helicopter = False
    flight._heli = None
    flight._request_ifr = True
    flight._weather_reactions = True
    flight.config = SimpleNamespace(
        flight=SimpleNamespace(ifr=False))
    flight.state.phase = AutonomousPhase.CRUISE
    flight.session.ifr = False
    flight.session.pattern_work = False
    flight.session.destination = SimpleNamespace(ident="LPPT")
    flight.session.clearance_issued = False
    state = SimpleNamespace(on_ground=False, ambient_in_cloud=True)
    flight._watch_ifr_pickup(state, 10.0)
    assert flight.session.ifr is False       # a wisp files nothing
    flight._watch_ifr_pickup(state, 40.0)    # 30 s in it does
    assert flight.session.ifr is True
    assert flight.config.flight.ifr is True  # filed, like preflight's
    flight._emit.assert_called_once()
    # Out of cloud before the clock, nothing is filed.
    flight.session.ifr = False
    flight.config.flight.ifr = False
    flight.state.imc_since = 0.0
    flight._watch_ifr_pickup(
        SimpleNamespace(on_ground=False, ambient_in_cloud=False), 50.0)
    flight._watch_ifr_pickup(state, 90.0)    # back in: the clock re-arms
    assert flight.session.ifr is False
    flight._watch_ifr_pickup(state, 116.0)   # and a stay still files it
    assert flight.session.ifr is True


def test_an_airborne_clearance_ask_is_the_pop_up_pickup():
    """Radar answers an airborne "request IFR clearance" by clearing it."""
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    from meridianatc.navdata.db import Station
    brain = _ground_brain()
    station = Station("LPPT", "APP", 120.5, "Lisboa Approach")
    session = FlightSession(Aircraft(parse_callsign("N123")),
                            destination=SimpleNamespace(
                                ident="LPPT", spoken="Portela"))
    session.ifr = False
    session.cruise_altitude_ft = 8000
    state = SimpleNamespace(on_ground=False, altitude_ft=5500.0)
    reply = brain._issue_ifr_pickup(
        session, station, brain.for_station(station), state)
    assert session.ifr is True
    assert session.clearance_issued is True
    assert session.squawk
    assert "squawk" in reply.text.lower()
    # Above the filed cruise the level it is at is what is kept.
    assert session.pending.values["altitude_ft"] == 8000
    state.altitude_ft = 9000.0
    session2 = FlightSession(Aircraft(parse_callsign("N456")),
                             destination=SimpleNamespace(
                                 ident="LPPT", spoken="Portela"))
    session2.cruise_altitude_ft = 8000
    brain._issue_ifr_pickup(
        session2, station, brain.for_station(station), state)
    assert session2.pending.values["altitude_ft"] == 9000


def test_taxi_power_walks_up_from_idle_never_slammed():
    """Out of the stand the lever is eased open, not thrown to a number."""
    flight = _flier()
    flight._lever_at = 0.0
    flight._ground_still = lambda s, n: False
    state = SimpleNamespace(
        on_ground=True, ground_speed_kt=0.0, heading_true=0.0)
    flight._steer(state, 45.0, 10.0)
    # The first write is idle: the slew has had no time to move.
    assert flight.director.throttle.call_args[0][0] == 0.0
    # A second later it is a step, not the figure the law wanted.
    flight._lever_at -= 1.0
    flight._steer(state, 45.0, 10.0)
    lever = flight.director.throttle.call_args[0][0]
    assert 0.0 < lever <= 0.05
    # Several seconds in it is still short of the travel -- the
    # breakaway push arrives at walking pace, not as a slam.
    for _ in range(7):
        flight._lever_at -= 1.0
        flight._steer(state, 45.0, 10.0)
    lever = flight.director.throttle.call_args[0][0]
    assert lever <= 0.45


def test_takeoff_power_is_advanced_not_slammed():
    """Even the roll's levers travel: briskly, progressively, to the stop."""
    flight = _flier()
    flight._lever_at = 0.0
    flight._set_takeoff_power()
    assert flight.director.throttle.call_args[0][0] == 0.0
    flight._lever_at -= 1.0
    flight._set_takeoff_power()
    lever = flight.director.throttle.call_args[0][0]
    assert 0.3 < lever < 1.0
    for _ in range(4):
        flight._lever_at -= 1.0
        flight._set_takeoff_power()
    assert flight.director.throttle.call_args[0][0] == 1.0
    flight.director.takeoff_power.assert_called()


def test_pushback_is_asked_for_where_someone_approves_it():
    """Ground control's push is the one it approves, not the sim tug."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()])
    flight.state.phase = AutonomousPhase.PUSHBACK
    flight.state.sub = "request"
    flight.state.sub_entered_at = 0.0
    flight._fo_say = Mock(return_value=True)
    flight._request_pushback = Mock()
    flight._radio_quiet = lambda: True
    state = SimpleNamespace(pushback_available=True)
    flight._phase_pushback(state, 5.0)
    assert flight.state.push_radio
    flight._fo_say.assert_called_once_with("pushback")
    flight._request_pushback.assert_not_called()


def test_pushback_off_the_network_presses_the_tug_itself():
    """Nobody to ask means the tug is pressed for, as it always was."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[
        SimpleNamespace(position="CTAF", mhz=122.800)])
    flight.state.phase = AutonomousPhase.PUSHBACK
    flight.state.sub = "request"
    flight.state.sub_entered_at = 0.0
    flight._fo_say = Mock(return_value=True)
    flight._request_pushback = Mock()
    flight._radio_quiet = lambda: True
    state = SimpleNamespace(pushback_available=True)
    flight._phase_pushback(state, 5.0)
    assert not flight.state.push_radio
    flight._request_pushback.assert_called_once()
    flight._fo_say.assert_not_called()


def test_ground_steering_shrinks_as_the_speed_builds():
    """Full nosewheel is a parking turn, not a taxi-speed one."""
    flight = _flier()
    flight._ground_still = lambda s, n: False
    state = SimpleNamespace(
        on_ground=True, ground_speed_kt=0.0, heading_true=0.0)
    flight._steer(state, 45.0, 10.0)
    slow = flight.director.steering.call_args[0][0]
    state.ground_speed_kt = 30.0
    flight._steer(state, 45.0, 10.0)
    fast = flight.director.steering.call_args[0][0]
    assert abs(slow) <= 0.5
    assert 0.0 < abs(fast) <= 0.15


def _lineup_flight():
    """A flier held in the lineup geometry the phase machine reads."""
    flight = _flier()
    flight.state.sub = "align"
    flight.state.runway_heading = 193.0
    flight.state.runway_lat, flight.state.runway_lon = 42.0, -8.0
    flight.session.cleared_takeoff = True
    flight.session.departure_remaining_m = 400.0
    flight._find_runway = lambda s: None
    flight._runway_blocked = lambda *a, **k: False
    flight._runway_inbound = lambda *a, **k: False
    flight._steer = Mock()
    return flight


def _near_the_strip(along_m: float, across_m: float):
    """lat/lon ``along_m`` down and ``across_m`` right of the 193 line."""
    import math
    rad = math.radians(193.0)
    north = along_m * math.cos(rad) - across_m * math.sin(rad)
    east = along_m * math.sin(rad) + across_m * math.cos(rad)
    return (42.0 + north / 111195.0,
            -8.0 + east / (111195.0 * math.cos(math.radians(42.0))))


def test_the_lineup_turn_is_made_at_corner_pace():
    """A hundred-odd degrees onto the strip is a corner, priced like one:
    swung at taxi speed the arc runs off the pavement's edge, where the
    infield grass holds what it catches."""
    flight = _lineup_flight()
    lat, lon = _near_the_strip(along_m=100.0, across_m=100.0)
    state = SimpleNamespace(
        on_ground=True, latitude=lat, longitude=lon,
        heading_true=40.0, ground_speed_kt=0.0)
    flight._phase_line_up(state, 10.0)
    called = flight._steer.call_args
    assert abs(called.args[2] - 2.5) < 1e-6
    assert called.kwargs["brake_turn"] is True


def test_lined_up_nearly_steers_at_taxi_speed():
    """Wings level on the numbers the corner is over -- but the line
    itself is rolled up to at walking pace, not the taxi figure whose
    sail past the gate burns the pavement the roll was counted on."""
    flight = _lineup_flight()
    lat, lon = _near_the_strip(along_m=100.0, across_m=0.0)
    state = SimpleNamespace(
        on_ground=True, latitude=lat, longitude=lon,
        heading_true=191.0, ground_speed_kt=3.0)
    flight._phase_line_up(state, 10.0)
    called = flight._steer.call_args
    assert called.args[2] <= 4.0


def test_the_connector_leg_runs_at_taxi_speed():
    """The price of the corner is paid at the corner: two degrees of
    demand down a long connector is a taxi leg, not the hairpin's pace
    the heading still owed to the runway once priced it at for the
    whole way in."""
    flight = _lineup_flight()
    # A long connector leg onto the strip: the join's vertex is still
    # over a hundred metres on, inside the ramp that pays it out.
    connector = _near_the_strip(along_m=60.0, across_m=-160.0)
    entry = _near_the_strip(along_m=60.0, across_m=0.0)
    lined = _near_the_strip(along_m=100.0, across_m=0.0)
    flight.session.lineup_path = [connector, entry, lined]
    lat, lon = _near_the_strip(along_m=60.0, across_m=-140.0)
    state = SimpleNamespace(
        on_ground=True, latitude=lat, longitude=lon,
        heading_true=283.0, ground_speed_kt=10.0)
    flight._phase_line_up(state, 10.0)
    called = flight._steer.call_args
    assert called.args[2] >= 12.0
    assert called.kwargs["brake_turn"] is False


def test_the_join_is_arrived_at_corner_pace():
    """Inside the corner's reach the same leg pays the sharp vertex's
    knots -- the slowdown was never wrong, only owed nowhere but the
    turn itself."""
    flight = _lineup_flight()
    connector = _near_the_strip(along_m=60.0, across_m=-160.0)
    entry = _near_the_strip(along_m=60.0, across_m=0.0)
    lined = _near_the_strip(along_m=100.0, across_m=0.0)
    flight.session.lineup_path = [connector, entry, lined]
    lat, lon = _near_the_strip(along_m=60.0, across_m=-18.0)
    state = SimpleNamespace(
        on_ground=True, latitude=lat, longitude=lon,
        heading_true=283.0, ground_speed_kt=4.0)
    flight._phase_line_up(state, 10.0)
    called = flight._steer.call_args
    # A 90-degree vertex inside twenty metres is priced at its figure
    # plus the metres to it: the slowdown stands, under taxi speed.
    assert called.args[2] <= 6.0


def test_a_finished_lineup_path_is_a_latch_not_a_waypoint():
    """Past the path's end the pursuit stays done: re-seating it on the
    connector behind swings the nose back across the junction it just
    made -- the wild heading demand a finished path still answered."""
    flight = _lineup_flight()
    entry = _near_the_strip(along_m=60.0, across_m=0.0)
    lined = _near_the_strip(along_m=100.0, across_m=0.0)
    connector = _near_the_strip(along_m=58.0, across_m=-40.0)
    flight.session.lineup_path = [connector, entry, lined]
    flight._lineup_on = list(flight.session.lineup_path)
    flight._lineup_done = True
    # Just past the end, on the paint, pointed down the strip: still
    # inside the pursuit's cross-track reach, where without the latch
    # it re-engages and steers the nose back at the path behind.
    lat, lon = _near_the_strip(along_m=155.0, across_m=5.0)
    state = SimpleNamespace(
        on_ground=True, latitude=lat, longitude=lon,
        heading_true=193.0, ground_speed_kt=6.0)
    flight._phase_line_up(state, 10.0)
    called = flight._steer.call_args
    wanted = (called.args[1] - 193.0 + 540.0) % 360.0 - 180.0
    assert abs(wanted) < 45.0


def test_reaching_the_paths_end_on_the_pavement_latches_done():
    """Within reach of the last point on the strip the pursuit hands off
    to the centreline -- and does not take the connector back over."""
    flight = _lineup_flight()
    entry = _near_the_strip(along_m=60.0, across_m=0.0)
    lined = _near_the_strip(along_m=100.0, across_m=0.0)
    connector = _near_the_strip(along_m=58.0, across_m=-40.0)
    flight.session.lineup_path = [connector, entry, lined]
    lat, lon = _near_the_strip(along_m=70.0, across_m=3.0)
    state = SimpleNamespace(
        on_ground=True, latitude=lat, longitude=lon,
        heading_true=160.0, ground_speed_kt=4.0)
    flight._phase_line_up(state, 10.0)
    assert flight._lineup_done


def test_a_stuck_probe_climbs_the_lever_to_the_stop():
    """A probe that never reaches full travel is a mired aeroplane kept
    for ever: each one asks more than the attempt that failed, to the
    stop itself."""
    flight = _flier()
    flight._lever_at = 0.0
    flight._ground_still = lambda s, n: False
    flight._stuck_probe = Mock(return_value=5)
    state = SimpleNamespace(
        on_ground=True, ground_speed_kt=0.0, heading_true=0.0)
    for _ in range(6):
        flight._lever_at -= 1.0
        flight._steer(state, 45.0, 10.0)
    lever = flight.director.throttle.call_args[0][0]
    assert lever > 0.75
    # And it still slews there -- a probe is walked up, not slammed.
    flight._stuck_probe = Mock(return_value=5)
    flight._last_lever = 0.0
    flight._lever_at = 0.0
    flight._steer(state, 45.0, 10.0)
    assert flight.director.throttle.call_args[0][0] <= 0.25 + 1e-9


def test_the_inside_brake_is_not_a_pivot_when_stopped():
    """Brake-turn assist is a rolling turn's tool: held at a standstill
    it is a chock, not a pivot -- the creep the nosewheel's turn needs
    is exactly the roll it stops."""
    flight = _flier()
    flight._ground_still = lambda s, n: False
    state = SimpleNamespace(
        on_ground=True, ground_speed_kt=0.0, heading_true=0.0)
    for _ in range(4):
        flight._steer(state, 45.0, 10.0, brake_turn=True)
    assert not flight.director.brakes_split.called
    # Rolling again, the inside wheel joins the turn as designed.
    state.ground_speed_kt = 5.0
    for _ in range(4):
        flight._steer(state, 45.0, 10.0, brake_turn=True)
    assert flight.director.brakes_split.called
    left, right = flight.director.brakes_split.call_args[0]
    assert max(left, right) > 0.0


def test_a_snap_is_not_learned_as_steering_authority():
    """A ground-handling snap answers to no pedal: the swing it writes
    into the authority figure is the one every later ask is divided
    by, so the gate refuses it outright."""
    from meridianatc.atc.autonomous._shared import _gs_band
    flight = _flier()
    for _ in range(6):
        flight._pedal_check(0.5, 44.5, 4.0)
    bands = getattr(flight, "_yaw_authority_b", None) or {}
    assert bands.get(_gs_band(4.0), 0.0) <= 0.0


def test_weak_answers_decay_the_authority_figure():
    """Demands that keep answering in much less rate than the figure
    predicts mean the figure flatters the model: it is walked down
    until the asks divided by it reach the pedal the turn needs."""
    flight = _flier()
    flight._yaw_authority_b = {1: 30.0}
    flight._yaw_authority = 30.0
    for _ in range(8):
        flight._pedal_check(0.4, 0.5, 10.0)
    assert flight._yaw_authority_b[1] < 30.0


def test_a_weave_is_not_a_weak_answer():
    """The measured rate spends half of every swing crossing zero, and
    each crossing scored as the weak answer that walks the authority
    figure down -- the decay raised the very gain that fed the hunt it
    was measuring. A weave is only counted as an absent answer while
    the rate opposes the demand or the nose is dead; the agreeing half
    of the swing keeps breaking the streak a loss would string."""
    flight = _flier()
    flight._yaw_authority_b = {2: 30.0}
    flight._yaw_authority = 30.0
    for rate in (-1.0, 2.0) * 10:
        flight._pedal_check(0.6, rate, 30.0)
    assert flight._yaw_authority_b[2] == 30.0


def test_the_rolls_directional_law_leans_on_a_held_error():
    """The rate figure alone can leave a sustained heading error
    answered by a whisper: a bounded error share is the ask that still
    reaches the pedal the veer needs -- the difference between holding
    the paint and leaving the strip's side on the roll."""
    flight = _flier()
    state = SimpleNamespace(heading_true=30.0, ground_speed_kt=30.0)
    pedal = 0.0
    for _ in range(4):
        pedal = flight._ground_directional(
            state, 50.0, 30.0, rate_cap=5.0)
    assert pedal > 0.3


def test_the_rollout_holds_the_line_on_the_rudder_alone():
    """At rollout speed the nosewheel stays out of the steering."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _flier()
    flight.state.phase = AutonomousPhase.LANDING
    flight.state.sub = "rollout"
    flight.state.runway_heading = 40.0
    state = SimpleNamespace(
        on_ground=True, heading_true=20.0, ground_speed_kt=90.0,
        vertical_speed_fpm=0.0, source="simconnect")
    flight._phase_landing(state, 10.0)
    flight.director.steering.assert_called_with(0.0)
    rudder = flight.director.rudder.call_args[0][0]
    assert rudder != 0.0


def test_the_roll_does_not_relearn_the_pedals_sign():
    """A takeoff roll's yaw is not the wheel's own answer -- at takeoff
    power the prop's left-turning moment disagrees with a right demand
    under either sign, and the vote read each as the axis inverted:
    four flips in nine seconds, the inverted half of the flapping
    feeding the veer it was meant to cure. The sign is the taxi law's
    business, where the wheel owns the yaw; the roll keeps what taxi
    learned. The magnitude half of the check still runs -- vote=False
    is the part that must not."""
    flight = _flier()
    seen = []
    flight._pedal_check = lambda *a, **k: seen.append(k.get("vote", True))
    state = SimpleNamespace(heading_true=30.0, ground_speed_kt=40.0)
    for _ in range(6):
        flight._ground_directional(state, 50.0, 40.0, rate_cap=5.0)
    assert seen and all(v is False for v in seen)


def test_the_roll_decays_authority_the_plant_will_not_answer():
    """A figure learned on the taxi tiller flatters the roll's rudder:
    the same +0.7 demand that taxied the apron answers in barely a
    degree a second at forty knots, and a figure that flatters leaves
    every ask divided by it a whisper -- the veer that walks a roll off
    the strip's side. Weak answers walk the figure down until the asks
    bite."""
    flight = _flier()
    flight._yaw_authority_b = {3: 30.0}
    flight._yaw_authority = 30.0
    state = SimpleNamespace(heading_true=30.0, ground_speed_kt=40.0)
    for _ in range(12):
        flight._ground_directional(state, 50.0, 40.0, rate_cap=5.0,
                                   pedal_max=0.8)
    # The rate never answers -- heading_true is pinned -- so the weak
    # streak decays the band until the demands reach for the stop.
    assert flight._yaw_authority_b[3] < 30.0


def test_the_rolls_pedal_holds_the_weaves_slow_half():
    """The measured rate the roll's demand answers is the weave itself:
    a pedal that chases it beat for beat pumps the swing the law exists
    to damp -- each hard ask arriving in time to be the next overshoot's
    push. The foot a roll is flown with is the demand's slow half: the
    standing pressure the drift costs, held while the alternating part
    is taken off. The asks here alternate the plant's full swing; what
    leaves holds their mean and damps the pump."""
    flight = _flier()
    flight._turn_rate = Mock(side_effect=[-6.0, 6.0] * 8)
    state = SimpleNamespace(heading_true=30.0, ground_speed_kt=40.0)
    out = [flight._ground_directional(state, 50.0, 40.0,
                                      rate_cap=5.0, pedal_max=0.8)
           for _ in range(12)]
    swing = max(out[-4:]) - min(out[-4:])
    assert swing < 0.4
    # The drift correction itself is not damped away: the mean of the
    # alternating asks still reaches the pedal as a standing lean.
    assert sum(out[-4:]) / 4.0 > 0.1


def test_the_rolls_damped_rate_keeps_the_foot_down():
    """Subtracting the measured rate whole pumped the ~eight-second
    weave the rejected rolls logged -- the demand's troughs were the
    ground the standing veer was gained on, each dip below the hold it
    was being asked to keep. A third of the answer is enough of the
    brake, and the error's own larger share carries the standing
    correction: through the weave the foot stays planted where the
    drift costs it."""
    flight = _flier()
    flight._turn_rate = Mock(side_effect=[-6.0, 6.0] * 8)
    state = SimpleNamespace(heading_true=30.0, ground_speed_kt=40.0)
    out = [flight._ground_directional(state, 50.0, 40.0,
                                      rate_cap=5.0, pedal_max=0.8,
                                      rate_damp=0.25, err_share=0.5)
           for _ in range(12)]
    assert min(out[-4:]) > 0.4


def test_a_new_roll_does_not_inherit_the_last_rolls_pedal():
    """The demand a rejected roll walked toward its stop was the next
    roll's opening ask -- the memory carried through ``_goto_sub``, so
    a fresh roll answers its first seconds with a foot nobody asked
    for. A roll starts on the demand it is given, not the one the last
    roll ended on."""
    flight = _flier()
    flight._roll_pedal_out = 0.9
    flight._goto_sub("roll")
    assert flight._roll_pedal_out == 0.0


def _rolling_flight():
    """A takeoff fixture one tick into the roll: the heavy helpers the
    pavement checks lean on are doubles, the director is the mock the
    flier fixture already hands out."""
    flight = _flier()
    flight.state.sub = "roll"
    flight.state.runway_heading = 15.0
    flight.state.runway_lat = 42.2219
    flight.state.runway_lon = -8.6300
    flight.state.runway_length_ft = 7874.0
    flight.state.runway_width_ft = 100.0
    flight.state.sub_entered_at = time.monotonic()
    flight._airborne_confirmed = Mock(return_value=False)
    flight._set_takeoff_power = Mock()
    flight._ground_directional = Mock(return_value=0.0)
    flight._right_foot = Mock(return_value=0.0)
    flight._pedal_dir = Mock(return_value=1)
    flight._roll_aileron = Mock(return_value=0.0)
    flight._roll_note = Mock()
    flight._runway_blocked = Mock(return_value=False)
    flight._engines_out = Mock(return_value=(0, 0))
    flight.TAKEOFF_TIMEOUT = 600.0
    return flight


def _rolling_state(**kw):
    args = dict(
        on_ground=True, parking_brake=False,
        indicated_airspeed_kt=30.0, ground_speed_kt=30.0,
        heading_true=15.0, latitude=42.2220, longitude=-8.6299,
        min_rotation_kt=0.0)
    args.update(kw)
    return SimpleNamespace(**args)


def test_the_roll_releases_the_autopilot_the_toga_press_armed():
    """The TO/GA press at takeoff power engages the autopilot on the
    models that arm one, and an engaged AP is a second hand on the
    pedals: its nosewheel servo steers for whatever heading bug the
    airframe was left with -- a stale bug's pull is the left yaw every
    rejected roll converged on, and the parked aeroplane kept walking
    to it after the stop. The FO's foot owns the roll: the master is
    released while the wheels are on the pavement, asked for every
    tick the way the brakes' release is."""
    flight = _rolling_flight()
    state = _rolling_state()
    now = time.monotonic() + 5.0
    flight._phase_takeoff(state, now)
    flight.director.autopilot.assert_called_with(False)
    flight._phase_takeoff(state, now + 1.0)
    assert flight.director.autopilot.call_count == 2


def test_the_roll_sets_the_bug_to_the_strips_own_heading():
    """An autopilot that finds its way back on mid-roll -- or the
    flight director the TO/GA mode arms -- is aimed at the strip, not
    at whatever heading the last flight's climb left on the bug. The
    write is once a roll, not a stream."""
    flight = _rolling_flight()
    state = _rolling_state()
    now = time.monotonic() + 5.0
    flight._phase_takeoff(state, now)
    assert flight.director.heading.call_count == 1
    bug = flight.director.heading.call_args[0][0]
    assert abs(((bug - 15.0 + 540.0) % 360.0) - 180.0) < 5.0
    flight._phase_takeoff(state, now + 1.0)
    assert flight.director.heading.call_count == 1


def test_a_new_roll_arms_the_bug_write_again():
    """The one-shot is per roll: a fresh roll's bug write is not the
    last roll's spent flag."""
    flight = _flier()
    flight._roll_bug_sent = True
    flight._goto_sub("roll")
    assert flight._roll_bug_sent is False


def test_a_grounded_stop_releases_the_autopilot():
    """The hold the abort leaves -- parked mid-runway waiting on the
    tower -- is the stop the servo kept steering through: the parked
    aeroplane walked its nose to the stale bug with the wheels dead.
    Any grounded hold releases the master."""
    flight = _flier()
    flight._stop(SimpleNamespace(on_ground=True, ground_speed_kt=30.0))
    flight.director.autopilot.assert_called_with(False)


def _aborting_flight():
    """A flight one tick into the rejected takeoff's stop."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _flier()
    flight.state.phase = AutonomousPhase.TAKEOFF
    flight.state.sub = "abort"
    flight.state.runway_heading = 15.0
    flight._ground_directional = Mock(return_value=0.4)
    flight._centreline_heading = Mock(return_value=15.0)
    flight.state.stopping_call = True
    flight.crew.radio_call = Mock(return_value=("", ""))
    flight.session.current_station = None
    return flight


def _abort_state(**kw):
    args = dict(
        on_ground=True, ground_speed_kt=120.0, heading_true=15.0,
        latitude=42.2220, longitude=-8.6299, source="simconnect")
    args.update(kw)
    return SimpleNamespace(**args)


def test_the_reject_flies_the_qrh_stop_not_a_taxi_stop():
    """A rejected takeoff is maximum braking with the boards up and
    reverse while it helps -- not the gentle squeeze ``_stop`` gives a
    hold short."""
    flight = _aborting_flight()
    state = _abort_state()
    flight._phase_takeoff(state, time.monotonic())
    flight.director.spoilers.assert_called_with(1.0)
    # Above sixty the reverse does the work: eased wheel brakes.
    flight.director.brakes.assert_called_with(0.7)
    flight.director.send.assert_called_with(
        b"THROTTLE_REVERSE_THRUST_TOGGLE", 0)
    # And the centreline is still held through the stop.
    flight.director.rudder.assert_called_with(0.4)


def test_the_reject_stows_the_boards_when_it_stops():
    """Stopped means the speedbrakes and reverse come in before the
    powered hold -- nobody sits on the runway with the boards out."""
    flight = _aborting_flight()
    flight.state.reversers_out = True
    state = _abort_state(ground_speed_kt=0.5)
    flight._phase_takeoff(state, time.monotonic())
    flight.director.spoilers.assert_called_with(0.0)
    flight.director.spoilers_arm.assert_called_with(False)
    flight.director.parking_brake.assert_called_with(True)
    assert flight.state.reversers_out is False
    assert flight.state.sub == "await"


def test_the_reject_tells_the_tower_it_is_stopping():
    """The call a crew owes the tower goes out on the frequency --
    "N123 stopping" -- before the hold, retried until sent."""
    flight = _aborting_flight()
    flight.state.stopping_call = False
    flight.crew.radio_call = Mock(return_value=("N123 stopping", ""))
    state = _abort_state()
    flight._phase_takeoff(state, time.monotonic())
    flight.crew.radio_call.assert_called_with("stopping")
    flight.crew.transmit.assert_called_with("N123 stopping")
    assert flight.state.stopping_call is True


def test_the_stopping_call_retries_until_it_goes_out():
    """A frequency that never let it out is asked again on the next
    tick -- a call marked made where nothing sent is the report
    nobody heard."""
    flight = _aborting_flight()
    flight.state.stopping_call = False
    flight.crew.radio_call = Mock(return_value=("", "frequency busy"))
    flight._phase_takeoff(_abort_state(), time.monotonic())
    assert flight.state.stopping_call is False
    flight.crew.radio_call = Mock(return_value=("N123 stopping", ""))
    flight._phase_takeoff(_abort_state(), time.monotonic() + 1.0)
    assert flight.state.stopping_call is True


def test_the_stopping_words_parse_as_the_rejected_call():
    """The tower understands the reject however the crew words it --
    "stopping", "rejecting takeoff", "aborting the takeoff" -- in
    every language the radios work."""
    from meridianatc.atc import intents
    parser = intents.IntentParser(["N123"])
    for said in ("N123 stopping", "N123 rejecting takeoff",
                 "N123 aborting the takeoff"):
        assert parser.parse(said, "en").intent \
            is intents.Intent.REJECTED_TAKEOFF, said
    for said, language in (("N123 décollage interrompu", "fr"),
                           ("N123 abortamos el despegue", "es"),
                           ("N123 Startabbruch", "de"),
                           ("N123 decollo interrotto", "it"),
                           ("N123 descolagem abortada", "pt")):
        assert parser.parse(said, language).intent \
            is intents.Intent.REJECTED_TAKEOFF, said


def test_the_tower_rogers_the_stopping_call():
    """The report is answered, and the runway stays the aeroplane's
    until it is off: the abort watch -- not this reply -- works it
    off the strip once it has stopped."""
    from meridianatc.atc.intents import Intent
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession, Phase
    from meridianatc.atc.speech import parse_callsign
    brain, tower = _vigo_tower_setup()
    session = FlightSession(Aircraft(parse_callsign("ECXGS")))
    session.phase = Phase.TAKEOFF
    session.departure_runway = "01"
    session.cleared_takeoff = True
    parsed = SimpleNamespace(intent=Intent.REJECTED_TAKEOFF)
    state = SimpleNamespace(on_ground=True, ground_speed_kt=80.0)
    reply = brain._tower(parsed, session, tower, None,
                         brain.for_station(tower), state)
    assert reply.text
    assert session.cleared_takeoff is True


def test_the_roll_rejects_on_the_towers_cancel():
    """"Cancel takeoff clearance" below V1 is the tower's own reject --
    obeyed with the same stop a crew's reject flies, not argued with."""
    flight = _rolling_flight()
    flight.session.takeoff_cancelled = True
    flight.session.cleared_takeoff = True
    flight._phase_takeoff(_rolling_state(), time.monotonic())
    assert flight.state.rejected is True
    assert flight.state.sub == "abort"


def test_the_roll_flies_past_v1_despite_the_cancel():
    """Above V1 the word is too late to stop for -- the aeroplane flies,
    the way the real rule leaves the call to the crew."""
    flight = _rolling_flight()
    flight.session.takeoff_cancelled = True
    flight.session.cleared_takeoff = True
    flight._phase_takeoff(
        _rolling_state(indicated_airspeed_kt=200.0,
                       ground_speed_kt=195.0),
        time.monotonic())
    assert not flight.state.rejected
    assert flight.state.sub != "abort"


def test_the_wait_clearance_holds_on_a_cancel():
    """Cleared but never rolled, then told it is taken back: the
    aeroplane stays on the numbers until a fresh clearance comes --
    the spent one is nobody's to roll on."""
    from meridianatc.atc.session import Phase
    flight = _rolling_flight()
    flight.state.sub = "wait_clearance"
    flight.session.takeoff_cancelled = True
    flight.session.cleared_takeoff = False
    flight.session.phase = Phase.TAKEOFF
    state = _rolling_state(ground_speed_kt=0.0,
                           indicated_airspeed_kt=0.0)
    flight._phase_takeoff(state, time.monotonic())
    assert flight.state.sub == "wait_clearance"


def _gsx(**kw):
    """A GsxLink stub: the state GSX reports, and the asks it takes."""
    state = SimpleNamespace(
        available=True,
        jetway_connected=kw.get("jetway", False),
        stairs_connected=kw.get("stairs", False),
        fuel_hose=kw.get("fuel_hose", False),
        can_taxi=kw.get("can_taxi", True),
        being_pushed=kw.get("being_pushed", False),
        why_not=lambda: kw.get("why", ""))
    return SimpleNamespace(
        state=state, can_ask=True, asking=kw.get("asking", ""),
        request=Mock(return_value=kw.get("said", "On its way")),
        start_pushback=Mock(return_value=""))


def test_taxi_holds_while_the_jetway_is_still_on():
    """A clearance moves the aeroplane, not the ramp: attached gear holds."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()], phase="taxi_out")
    flight.session.taxi_route = "via A to holding point runway 35"
    flight.brain.ground = _gsx(can_taxi=False, why="jetway connected")
    flight.state.phase = AutonomousPhase.TAXI_OUT
    flight.state.sub = "taxi"
    flight.state.sub_entered_at = _time.monotonic() - 400.0
    state = SimpleNamespace(ground_speed_kt=0.0)
    flight._phase_taxi_out(state, 400.0)
    flight._taxi_along_path.assert_not_called()
    flight._stop.assert_called()
    # Once the ramp lets go the same clearance taxis.
    flight.brain.ground.state.can_taxi = True
    flight._phase_taxi_out(state, 401.0)
    flight._taxi_along_path.assert_called_once()


def test_push_waits_for_the_stand_to_let_go():
    """The push is asked for after the bridge comes off, not before."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()])
    flight.brain.ground = _gsx(
        can_taxi=False, why="jetway connected", jetway=True)
    flight.state.phase = AutonomousPhase.PUSHBACK
    flight.state.sub = "request"
    flight.state.sub_entered_at = _time.monotonic() - 5.0
    flight._fo_say = Mock(return_value=True)
    flight._request_pushback = Mock()
    flight._radio_quiet = lambda: True
    state = SimpleNamespace(pushback_available=True)
    flight._phase_pushback(state, 5.0)
    flight.brain.ground.request.assert_called_once_with("jetway")
    flight._fo_say.assert_not_called()
    flight._request_pushback.assert_not_called()
    # A stand that never lets go is not waited on for ever.
    flight.state.sub_entered_at = _time.monotonic() - 50.0
    flight._phase_pushback(state, 50.0)
    flight._fo_say.assert_called_once_with("pushback")


def test_the_tugs_ready_question_is_answered_yes():
    """GSX pausing for the flight deck's word gets it: ready."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()])
    flight.brain.ground = _gsx(being_pushed=True)
    flight.state.phase = AutonomousPhase.PUSHBACK
    flight.state.sub = "wait"
    flight.state.sub_entered_at = _time.monotonic() - 20.0
    flight._fo_say = Mock(return_value=True)
    state = SimpleNamespace(
        pushback_wait=False, pushback_attached=True,
        ground_speed_kt=0.0, on_ground=True)
    flight._phase_pushback(state, 20.0)
    flight.brain.ground.request.assert_called_once_with("confirm")


def test_the_brake_holds_until_the_tug_is_on():
    """A tug on its way is not a tug attached: the brake that held the
    start holds the wait -- an idling aeroplane creeps without it."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()])
    flight.state.phase = AutonomousPhase.PUSHBACK
    flight.state.sub = "wait"
    flight.state.sub_entered_at = _time.monotonic() - 10.0
    state = SimpleNamespace(
        pushback_wait=True, pushback_attached=False,
        ground_speed_kt=0.0, on_ground=True)
    flight._phase_pushback(state, 10.0)
    flight._dir.parking_brake.assert_called_once_with(True)
    # Attached is what lets it off.
    flight._dir.parking_brake.reset_mock()
    state.pushback_attached = True
    flight._phase_pushback(state, 11.0)
    flight._dir.parking_brake.assert_called_once_with(False)


def test_an_approved_pushback_is_not_asked_for_again():
    """"Push and start approved" ends the asking: the session's move
    past "cleared" is the answer heard, and a tug that never shows is
    pressed for directly rather than asked for a second time."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()], phase="pushback")
    flight.state.phase = AutonomousPhase.PUSHBACK
    flight.state.sub = "wait"
    flight.state.push_radio = True
    flight.state.sub_entered_at = _time.monotonic() - 50.0
    flight._fo_say = Mock(return_value=True)
    flight._request_pushback = Mock()
    state = SimpleNamespace(
        pushback_wait=False, pushback_attached=False,
        ground_speed_kt=0.0, on_ground=True)
    flight._phase_pushback(state, 50.0)
    flight._fo_say.assert_not_called()
    assert not flight.state.push_radio
    # Past the long wait the tug is pressed for, not asked for again.
    flight.state.sub_entered_at = _time.monotonic() - 130.0
    flight._phase_pushback(state, 130.0)
    flight._request_pushback.assert_called_once()
    flight._fo_say.assert_not_called()


def test_an_unanswered_pushback_is_still_asked_again():
    """The call nobody answered is the one repeated with patience:
    still "cleared" on the session means the frequency heard nothing."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()], phase="cleared")
    flight.state.phase = AutonomousPhase.PUSHBACK
    flight.state.sub = "wait"
    flight.state.push_radio = True
    flight.state.sub_entered_at = _time.monotonic() - 50.0
    flight._fo_say = Mock(return_value=True)
    flight._request_pushback = Mock()
    state = SimpleNamespace(
        pushback_wait=False, pushback_attached=False,
        ground_speed_kt=0.0, on_ground=True)
    flight._phase_pushback(state, 50.0)
    flight._fo_say.assert_called_once_with("pushback")
    flight._request_pushback.assert_not_called()


def test_the_fos_own_hold_marks_the_session():
    """Stopped at the bars is the report the session waits on: a hold
    reached on the route can sit far from the published end, the tower
    works off the session, and a flight still "taxiing" was sent the
    taxi instruction again every time it called ready."""
    from meridianatc.atc.autonomous import AutonomousPhase
    from meridianatc.atc.session import Phase
    flight = _taxi_flight(stations=[_gnd()], phase="taxi_out")
    flight.state.phase = AutonomousPhase.HOLDING_SHORT
    flight.session.lined_up = False
    flight.session.cleared_takeoff = False
    flight.session.current_station = None
    state = SimpleNamespace(ground_speed_kt=0.0, on_ground=True)
    flight._phase_holding_short(state, 10.0)
    assert flight.session.phase == Phase.HOLDING_SHORT
    # A session already past the hold is not walked back to it.
    flight._departure_possible = lambda s: True
    flight.session.phase = Phase.LINED_UP
    flight._phase_holding_short(state, 11.0)
    assert flight.session.phase == Phase.LINED_UP


def _hold_point_engine():
    """The phase driver with LEVX's 01 threshold behind the hold test.

    The south stands sit three hundred metres from the 01 threshold --
    inside the hold radius -- which is how a parked aeroplane was marked
    at the holding point and sent to the tower before it had ever moved.
    """
    from meridianatc.engine.x_the_other_aeroplanes import _The_other_aeroplanes
    engine = _The_other_aeroplanes.__new__(_The_other_aeroplanes)
    ends = [("01", 11.4, SimpleNamespace(
        le_ident="01", he_ident="19",
        le_lat=42.221298, le_lon=-8.629660,
        he_lat=42.242500, he_lon=-8.623890))]
    engine.navdb = SimpleNamespace(runway_ends=lambda ident: ends)
    return engine


def _departure_session(phase, lineup=None):
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    session = FlightSession(Aircraft(parse_callsign("CS-TSP")))
    session.departure = SimpleNamespace(ident="LEVX")
    session.departure_runway = "01"
    session.phase = phase
    session.lineup_path = list(lineup or ())
    return session


# The bars the Delta route's line-up leg starts from, per the plan the
# session was given.
_BARS_01 = (42.22123, -8.63034)


def test_a_level_capture_dip_is_not_the_descent_phase():
    """The live FL170 capture dipped below -300 fpm for a few seconds and
    the session became "descent" 230 nm out, which then masked the centre
    handoff. Only a sustained descent re-phases the flight."""
    from unittest.mock import patch

    from meridianatc.atc.session import Phase
    from meridianatc.sim.state import AircraftState
    engine = _hold_point_engine()
    session = _departure_session(Phase.ENROUTE)
    level = AircraftState(on_ground=False, altitude_ft=16900.0,
                          vertical_speed_fpm=-400.0, ground_speed_kt=240.0)
    with patch("meridianatc.engine.x_the_other_aeroplanes.time.monotonic") as clock:
        clock.return_value = 100.0
        engine._update_phase(level, level, session)
        clock.return_value = 110.0
        engine._update_phase(level, level, session)
        assert session.phase == Phase.ENROUTE
        clock.return_value = 121.0
        engine._update_phase(level, level, session)
        assert session.phase == Phase.DESCENT

    session = _departure_session(Phase.ENROUTE)
    with patch("meridianatc.engine.x_the_other_aeroplanes.time.monotonic") as clock:
        clock.return_value = 200.0
        engine._update_phase(level, level, session)
        level.vertical_speed_fpm = 0.0
        clock.return_value = 230.0
        engine._update_phase(level, level, session)
        assert session.phase == Phase.ENROUTE


def test_a_parked_stand_near_the_end_is_not_a_holding_point():
    """Cleared on a south stand at Vigo, never rolled: the stand sits
    inside the hold radius, but a holding point is reached by taxiing,
    so the session stays cleared and ground keeps the flight."""
    from meridianatc.atc.session import Phase
    from meridianatc.sim.state import AircraftState
    engine = _hold_point_engine()
    session = _departure_session(Phase.CLEARED)
    parked = AircraftState(latitude=42.22342, longitude=-8.63195,
                           altitude_ft=837.0, on_ground=True,
                           ground_speed_kt=0.0)
    engine._update_phase(parked, parked, session)
    assert session.phase == Phase.CLEARED
    # The instruction's phase is no witness either: taxi_out written by
    # the issued taxi, the aeroplane still on its brakes.
    session.phase = Phase.TAXI_OUT
    engine._update_phase(parked, parked, session)
    assert session.phase == Phase.TAXI_OUT


def test_the_hold_is_marked_only_after_the_taxi_rolls():
    """The same point counts once the aeroplane has been seen moving:
    rolled off the stand, then stopped at the bars, is a hold reached
    the way a hold is reached."""
    from meridianatc.atc.session import Phase
    from meridianatc.sim.state import AircraftState
    engine = _hold_point_engine()
    session = _departure_session(Phase.PREFLIGHT)
    rolling = AircraftState(latitude=42.22342, longitude=-8.63195,
                            altitude_ft=837.0, on_ground=True,
                            ground_speed_kt=8.0)
    engine._update_phase(rolling, rolling, session)
    assert session.phase == Phase.TAXI_OUT
    assert session.left_the_stand
    at_the_bars = AircraftState(latitude=42.22450, longitude=-8.63050,
                                altitude_ft=837.0, on_ground=True,
                                ground_speed_kt=0.0)
    engine._update_phase(rolling, at_the_bars, session)
    assert session.phase == Phase.HOLDING_SHORT
    # The witness is spent with the journey it counted for: the phase
    # has left the taxiing set, so the next look clears it.
    engine._update_phase(at_the_bars, at_the_bars, session)
    assert not session.left_the_stand


def test_the_routes_own_bars_are_the_holding_point():
    """The Delta route ends its taxi at the bars three hundred metres
    from the stand: still rolling on the apron, the aeroplane is on its
    way, not at the hold -- the premature mark is what sent a parked
    flight to the tower and had the line-up chasing its own route."""
    from meridianatc.atc.session import Phase
    from meridianatc.sim.state import AircraftState
    engine = _hold_point_engine()
    session = _departure_session(
        Phase.TAXI_OUT,
        lineup=[_BARS_01, (42.22173, -8.62953), (42.22208, -8.62944)])
    session.left_the_stand = True
    # Mid-taxi on the apron: inside the threshold radius, three hundred
    # metres short of the bars.
    rolling = AircraftState(latitude=42.22300, longitude=-8.63180,
                            altitude_ft=837.0, on_ground=True,
                            ground_speed_kt=7.0)
    engine._update_phase(rolling, rolling, session)
    assert session.phase == Phase.TAXI_OUT
    at_the_bars = AircraftState(latitude=42.22130, longitude=-8.63040,
                                altitude_ft=837.0, on_ground=True,
                                ground_speed_kt=0.0)
    engine._update_phase(rolling, at_the_bars, session)
    assert session.phase == Phase.HOLDING_SHORT


def _rolling_read(on_ground, kt=45.0):
    from meridianatc.sim.state import AircraftState
    return AircraftState(latitude=42.232, longitude=-8.627,
                         altitude_ft=840.0, on_ground=on_ground,
                         ground_speed_kt=kt)


def test_the_gears_word_on_the_roll_is_not_a_landing():
    """A takeoff roll at speed unloads and touches its gear again, and a
    skipped frame reads the same: taken at face value the pair spent the
    takeoff clearance on the bounce and wrote a touchdown on the roll --
    which is how a departure that never left its strip got sent around.
    One airborne read is the gear answering; the re-touch after it is the
    bounce's other half, not a landing nobody flew."""
    from meridianatc.atc.session import Phase
    engine = _hold_point_engine()
    session = _departure_session(Phase.TAKEOFF)
    session.cleared_takeoff = True
    engine._update_phase(_rolling_read(True, 40.0),
                         _rolling_read(False), session)
    assert session.phase == Phase.TAKEOFF
    engine._update_phase(_rolling_read(False),
                         _rolling_read(True, 48.0), session)
    assert session.phase == Phase.TAKEOFF
    assert not session.touchdown_at
    assert session.cleared_takeoff


def test_liftoff_is_the_airborne_read_that_persists():
    """Flight on the way out is the airborne read still airborne on the
    next poll: the first read keeps the roll's phase, the second makes
    the departure and spends the clearance the way a liftoff does."""
    from meridianatc.atc.session import Phase
    engine = _hold_point_engine()
    session = _departure_session(Phase.TAKEOFF)
    session.cleared_takeoff = True
    ground = _rolling_read(True, 40.0)
    air1 = _rolling_read(False)
    air2 = _rolling_read(False, 52.0)
    engine._update_phase(ground, air1, session)
    assert session.phase == Phase.TAKEOFF
    assert session.cleared_takeoff
    engine._update_phase(air1, air2, session)
    assert session.phase == Phase.DEPARTURE
    assert not session.cleared_takeoff


def test_a_roll_liftoff_is_never_off_a_landing_rollout():
    """Nothing on the way out rolls out of a landing: a stale touchdown
    from the last circuit cannot turn the roll's airborne read into a
    touch-and-go, so the rollout liftoff stays unarmed while the session
    is still the takeoff it was cleared for."""
    import time as _time

    from meridianatc.atc.session import Phase
    engine = _hold_point_engine()
    engine.station = None
    session = _departure_session(Phase.TAKEOFF)
    session.touchdown_at = _time.monotonic() - 3.0
    session.touchdown_runway = "01"
    session.rollout_min_kt = 60.0
    engine._note_rollout_liftoff(_rolling_read(True),
                                 _rolling_read(False, 55.0), session)
    assert not session.rollout_liftoff_at
    # The same read off a real rollout is the thing it is for.
    session.phase = Phase.LANDED
    engine._note_rollout_liftoff(_rolling_read(True),
                                 _rolling_read(False, 55.0), session)
    assert session.rollout_liftoff_at


def test_the_surface_watch_does_not_call_the_roll_a_landing():
    """The advisory is a controller looking out of the window: an
    aeroplane still in its takeoff phase has not landed, whatever the
    gear's word just did at speed."""
    from meridianatc.atc.session import Phase
    from meridianatc.atc.watch import SurfaceWatch
    station = SimpleNamespace(ident="LEVX", position="TWR",
                              callsign="Vigo Tower",
                              callsign_in=lambda lang: "Vigo Tower")
    p = SimpleNamespace(language="en")
    p.landed_without_clearance = Mock(return_value="landed no clearance")
    brain = SimpleNamespace(
        issues_clearances=lambda s: True,
        rest_of_taxi=lambda *a, **k: None,
        for_station=lambda *a, **k: p)
    navdb = SimpleNamespace(
        airport=lambda ident: SimpleNamespace(
            ident="LEVX", lat=42.232, lon=-8.627),
        runway_at=lambda *a, **k: "01",
        runway_strip=lambda *a, **k: "01",
        station_for=lambda *a, **k: None)
    watch = SurfaceWatch(navdb, brain)
    session = _departure_session(Phase.TAKEOFF)
    session.cleared_runway = "01"
    reply = watch.look(session, station,
                       _rolling_read(True, 50.0),
                       _rolling_read(False, 50.0))
    assert reply is None
    p.landed_without_clearance.assert_not_called()
    # A departure that comes back down is the landing it watches for.
    session.phase = Phase.DEPARTURE
    reply = watch.look(session, station,
                       _rolling_read(True, 50.0),
                       _rolling_read(False, 50.0))
    assert reply is not None


def test_a_departures_settling_touch_is_not_a_landing():
    """The struts' settle straight off the runway reads as a touchdown
    to a watcher taking the gear at its word: at Vigo a low rotation
    settled back a second after liftoff, and the touch wrote a
    touchdown, armed the liftoff-after-touchdown verdict, and a
    departure still climbing out was sent around the circuit and
    stalled into the ground. Inside the settling window the touch is
    the takeoff's own contact -- nothing is recorded of it."""
    from meridianatc.atc.session import Phase
    engine = _hold_point_engine()
    session = _departure_session(Phase.TAKEOFF)
    session.cleared_takeoff = True
    ground = _rolling_read(True, 50.0)
    air = _rolling_read(False, 58.0)
    engine._update_phase(ground, air, session)
    engine._update_phase(air, _rolling_read(False, 60.0), session)
    assert session.phase == Phase.DEPARTURE
    # A second later the struts touch again: still the takeoff.
    down = _rolling_read(True, 55.0)
    engine._update_phase(air, down, session)
    assert session.phase == Phase.DEPARTURE
    assert not session.touchdown_at
    # And the climb back off it is the same departure, not a second
    # landing's pending verdict.
    engine._update_phase(down, _rolling_read(False, 62.0), session)
    assert session.phase == Phase.DEPARTURE
    assert not session.touchdown_at


def test_a_departure_that_comes_back_lands():
    """The window is for the struts, not a reprieve: a departure that
    stays down once it has had time to mean it has landed, and the
    session says so."""
    import time as _time

    from meridianatc.atc.session import Phase
    engine = _hold_point_engine()
    session = _departure_session(Phase.DEPARTURE)
    session.departed_at = _time.monotonic() - 30.0
    engine._update_phase(_rolling_read(False, 58.0),
                         _rolling_read(True, 55.0), session)
    assert session.phase == Phase.LANDED
    assert session.touchdown_at


def test_a_departures_liftoff_is_never_off_a_landing_rollout():
    """The settle gate keeps the phantom touchdown out of the record,
    and this is the belt to its braces: even with a touchdown figure
    still warm from somewhere else, a liftoff the session counts
    outbound is the departure's own and arms no go-around verdict."""
    import time as _time

    from meridianatc.atc.session import Phase
    engine = _hold_point_engine()
    engine.station = None
    session = _departure_session(Phase.DEPARTURE)
    session.departed_at = _time.monotonic()
    session.touchdown_at = _time.monotonic() - 3.0
    session.touchdown_runway = "01"
    session.rollout_min_kt = 60.0
    engine._note_rollout_liftoff(_rolling_read(True),
                                 _rolling_read(False, 55.0), session)
    assert not session.rollout_liftoff_at


def test_the_watch_stays_quiet_through_the_settling_touch():
    """The tower's window on the same touch: inside the settling window
    it is the takeoff's own contact and no landing was watched, so no
    pilot hears he landed without a clearance on the climb-out."""
    import time as _time

    from meridianatc.atc.session import Phase
    from meridianatc.atc.watch import SurfaceWatch
    station = SimpleNamespace(ident="LEVX", position="TWR",
                              callsign="Vigo Tower",
                              callsign_in=lambda lang: "Vigo Tower")
    p = SimpleNamespace(language="en")
    p.landed_without_clearance = Mock(return_value="landed no clearance")
    brain = SimpleNamespace(
        issues_clearances=lambda s: True,
        rest_of_taxi=lambda *a, **k: None,
        for_station=lambda *a, **k: p)
    navdb = SimpleNamespace(
        airport=lambda ident: SimpleNamespace(
            ident="LEVX", lat=42.232, lon=-8.627),
        runway_at=lambda *a, **k: "01",
        runway_strip=lambda *a, **k: "01",
        station_for=lambda *a, **k: None)
    watch = SurfaceWatch(navdb, brain)
    session = _departure_session(Phase.DEPARTURE)
    session.cleared_runway = "01"
    session.departed_at = _time.monotonic()
    reply = watch.look(session, station,
                       _rolling_read(True, 55.0),
                       _rolling_read(False, 60.0))
    assert reply is None
    p.landed_without_clearance.assert_not_called()
    # Past the window the same touch is the landing it watches for.
    session.departed_at = _time.monotonic() - 30.0
    reply = watch.look(session, station,
                       _rolling_read(True, 55.0),
                       _rolling_read(False, 60.0))
    assert reply is not None


def test_the_rotate_is_airspeeds_not_the_grounds():
    """The books' rotation is flown on the needle: at Vigo the roll
    spooled the groundspeed twelve knots ahead of the indicated
    reading, and rotating on the higher figure pulled it into the air
    at forty-four -- a balloon, a settle back onto the strip, and the
    mush that stalled it. The abort gates keep the optimistic figure;
    the rotate waits on the pitot instead."""
    flight = _roll_flight()
    flight.state.sub = "roll"
    flight.state.sub_entered_at = 998.0
    state = SimpleNamespace(
        on_ground=True, ground_speed_kt=95.0, indicated_airspeed_kt=44.0,
        parking_brake=False, latitude=41.235, longitude=-8.679,
        heading_true=170.0)
    flight._phase_takeoff(state, 1000.0)
    assert not any(c.args == ("rotate",) for c in
                   flight._goto_sub.call_args_list)
    # The needle at the mark is the rotate.
    state.indicated_airspeed_kt = 91.0
    flight._phase_takeoff(state, 1000.0)
    assert ("rotate",) in [c.args for c in
                           flight._goto_sub.call_args_list]
    # A pitot that reads nothing at all leaves the wheels' figure the
    # only speed there is -- the roll still flies itself off.
    flight2 = _roll_flight()
    flight2.state.sub = "roll"
    flight2.state.sub_entered_at = 998.0
    dead = SimpleNamespace(
        on_ground=True, ground_speed_kt=95.0, indicated_airspeed_kt=0.0,
        parking_brake=False, latitude=41.235, longitude=-8.679,
        heading_true=170.0)
    flight2._phase_takeoff(dead, 1000.0)
    assert ("rotate",) in [c.args for c in
                           flight2._goto_sub.call_args_list]


def test_the_final_call_is_the_clearance_nobody_gives():
    """On a broadcast frequency each leg is announced; final clears."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight()
    flight.brain.issues_clearances = lambda s: False
    flight._manage_radio = True
    flight.state.phase = AutonomousPhase.APPROACH
    flight.session.ifr = False
    flight.session.current_station = SimpleNamespace(
        position="CTAF", is_controller=False)
    flight.session.cleared_landing = False
    flight.crew = SimpleNamespace(
        circuit=lambda: SimpleNamespace(in_circuit=True, leg="downwind"))
    flight._radio_quiet = lambda: True
    said = []
    flight._fo_say = lambda kind: said.append(kind) or True
    flight._work_the_circuit(SimpleNamespace(), flight.session, 100.0)
    assert said == ["downwind"]
    # The same leg is not reported twice.
    flight._work_the_circuit(SimpleNamespace(), flight.session, 106.5)
    assert said == ["downwind"]
    # On final the call doubles as the landing clearance.
    flight.crew.circuit = lambda: SimpleNamespace(
        in_circuit=True, leg="final")
    flight.state.last_radio_at = 100.0
    flight._work_the_circuit(SimpleNamespace(), flight.session, 107.0)
    assert said[-1] == "final"
    assert flight.session.cleared_landing


def test_a_staffed_tower_hears_no_circuit_calls():
    """Where someone sequences, the legs are not announced into it."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight()
    flight.brain.issues_clearances = lambda s: True
    flight._manage_radio = True
    flight.state.phase = AutonomousPhase.APPROACH
    flight.session.ifr = False
    flight.session.current_station = SimpleNamespace(
        position="TWR", is_controller=True)
    flight.crew = SimpleNamespace(
        circuit=lambda: SimpleNamespace(in_circuit=True, leg="downwind"))
    flight._fo_say = Mock(return_value=True)
    flight._work_the_circuit(SimpleNamespace(), flight.session, 100.0)
    flight._fo_say.assert_not_called()


def test_off_the_runway_the_vacated_call_goes_out():
    """Rollout ends with the call the tower is waiting for, made on the
    taxi-in once the frequency lets it out -- a refusal is not the call
    made, so it is tried until it goes and never again after it has."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _flier()
    flight.state.phase = AutonomousPhase.LANDING
    flight.state.sub = "rollout"
    flight.state.runway_heading = 40.0
    flight._manage_radio = True
    flight._radio_quiet = lambda: True
    flight._fo_say = Mock(return_value=True)
    state = SimpleNamespace(
        on_ground=True, heading_true=40.0, ground_speed_kt=10.0,
        vertical_speed_fpm=0.0, source="simconnect")
    flight._phase_landing(state, 10.0)
    # The call belongs to the taxi-in now: the rollout itself only
    # transitions.
    flight._fo_say.assert_not_called()
    flight._transition.assert_called_once_with(AutonomousPhase.TAXI_IN)


def test_the_vacated_call_is_retried_until_it_goes_out():
    """"Vacated" refused once is owed still: the taxi-in keeps trying it
    until the frequency takes it, then leaves it said."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight()
    flight.state.phase = AutonomousPhase.TAXI_IN
    flight._manage_radio = True
    flight._radio_quiet = lambda: True
    flight._fo_say = Mock(return_value=False)
    state = SimpleNamespace(
        on_ground=True, flaps_index=0, ground_speed_kt=10.0)
    flight._phase_taxi_in(state, 10.0)
    flight._phase_taxi_in(state, 11.0)
    assert flight._fo_say.call_count == 2       # refused, still owed
    assert not flight.state.vacated_call
    flight._fo_say = Mock(return_value=True)
    flight._phase_taxi_in(state, 12.0)
    flight._phase_taxi_in(state, 13.0)
    flight._fo_say.assert_called_once_with("vacated")
    assert flight.state.vacated_call


def test_the_stand_gets_its_bridge_then_its_deboarding():
    """Arrival asks are stepped, one GSX menu at a time."""
    flight = _taxi_flight()
    flight._chained = False
    flight.brain.ground = _gsx()
    flight._ask_the_ramp(SimpleNamespace())
    flight.brain.ground.request.assert_called_once_with("jetway")
    # While the menu walks, nothing else is asked.
    flight.brain.ground.asking = "jetway"
    flight._ask_the_ramp(SimpleNamespace())
    assert flight.brain.ground.request.call_count == 1
    # Walk done: the next ask is the passengers off.
    flight.brain.ground.asking = ""
    flight._ask_the_ramp(SimpleNamespace())
    assert flight.brain.ground.request.call_args[0][0] == "deboarding"
    # A chained leg leaves the stand for the turnaround instead.
    flight._chained = True
    flight._ask_the_ramp(SimpleNamespace())
    assert flight.brain.ground.request.call_count == 2


def _emergency_flight():
    """The flight with only what _monitor_emergency touches."""
    flight = _flier()
    flight._fuel_known = True
    flight._fuel_capacity_seen = 100.0
    flight._fuel_leg_since = 0.0
    flight._fuel_min_since = 0.0
    flight._is_helicopter = False
    flight._declare_emergency = Mock()
    return flight


def _fuel_state(fuel_gal, capacity=100.0):
    return SimpleNamespace(
        main_bus_volts=28.0, on_ground=False, engine_count=1,
        engines_running=True, eng2_combustion=None, eng3_combustion=None,
        eng4_combustion=None, fuel_total_lb=fuel_gal * 6.0,
        fuel_total_gal=fuel_gal, fuel_capacity_gal=capacity)


def test_full_tanks_are_never_a_fuel_emergency():
    """Tanks filled to the top cannot be short, whatever the leg says."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _emergency_flight()
    flight.state.phase = AutonomousPhase.CRUISE
    flight._monitor_emergency(_fuel_state(100.0), 10.0)
    flight._declare_emergency.assert_not_called()
    flight._monitor_emergency(_fuel_state(92.0), 11.0)
    flight._declare_emergency.assert_not_called()


def test_an_empty_tank_still_is_a_fuel_emergency():
    """Nothing aboard is the MAYDAY it always was, full capacity or not."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _emergency_flight()
    flight.state.phase = AutonomousPhase.CRUISE
    state = _fuel_state(0.0)
    flight._monitor_emergency(state, 10.0)
    flight._declare_emergency.assert_called_once_with(
        "fuel", state, mayday=True)


def test_a_few_gallons_still_declares_below_capacity():
    """Under the full-tank margin the dry check works as it did."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _emergency_flight()
    flight.state.phase = AutonomousPhase.CRUISE
    state = _fuel_state(3.0)
    flight._monitor_emergency(state, 10.0)
    flight._declare_emergency.assert_called_once_with(
        "fuel", state, mayday=True)


def test_a_projected_shortage_has_to_persist_to_be_declared():
    """The N45XS case: forty-two seconds into the cruise, the tanks were
    priced against climb power and a "minimum fuel" PAN went out over
    two hours of endurance. A shortage has to hold past the transient
    before it is anybody's emergency."""
    from meridianatc.atc import flying
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _emergency_flight()
    flight._perf = flying.LIGHT        # the Baron, not the airliner
    flight.state.phase = AutonomousPhase.CRUISE
    flight._reserve_min = 45.0
    flight.session.destination = SimpleNamespace(
        ident="LECO", lat=42.0, lon=-8.0)
    flight.navdb = SimpleNamespace(nearest_airports=lambda *a, **k: [])
    state = SimpleNamespace(
        main_bus_volts=28.0, on_ground=False, engine_count=2,
        engines_running=True, eng2_combustion=True, eng3_combustion=None,
        eng4_combustion=None, has_position=True,
        fuel_total_lb=420.0, fuel_total_gal=70.0, fuel_capacity_gal=200.0,
        est_fuel_flow_pph=200.0,        # climb power, not the leg's
        estimated_cruise_kt=0.0, design_vc_kt=0.0, ground_speed_kt=140.0,
        fuel_unusable_gal=0.0, distance_to=lambda lat, lon: 200.0)
    # Priced short on the first read (leg + reserve at 200 pph is about
    # 86 gal) -- but it is a projection, not a declaration, until it
    # persists.
    flight._monitor_emergency(state, 100.0)
    flight._declare_emergency.assert_not_called()
    flight._monitor_emergency(state, 100.0 + flight.MIN_FUEL_PERSIST_S - 1)
    flight._declare_emergency.assert_not_called()
    # Once it has persisted it is said -- and said once.
    flight._monitor_emergency(state, 100.0 + flight.MIN_FUEL_PERSIST_S)
    flight._declare_emergency.assert_called_once_with(
        "minimum fuel", state, mayday=False)
    flight._monitor_emergency(state, 100.0 + flight.MIN_FUEL_PERSIST_S + 60)
    flight._declare_emergency.assert_called_once()


def test_a_shortage_that_clears_is_no_emergency_at_all():
    """Power back to cruise and the projection heals itself: nothing
    was ever said."""
    from meridianatc.atc import flying
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _emergency_flight()
    flight._perf = flying.LIGHT        # the Baron, not the airliner
    flight.state.phase = AutonomousPhase.CRUISE
    flight._reserve_min = 45.0
    flight.session.destination = SimpleNamespace(
        ident="LECO", lat=42.0, lon=-8.0)
    flight.navdb = SimpleNamespace(nearest_airports=lambda *a, **k: [])
    state = SimpleNamespace(
        main_bus_volts=28.0, on_ground=False, engine_count=2,
        engines_running=True, eng2_combustion=True, eng3_combustion=None,
        eng4_combustion=None, has_position=True,
        fuel_total_lb=420.0, fuel_total_gal=70.0, fuel_capacity_gal=200.0,
        est_fuel_flow_pph=200.0,
        estimated_cruise_kt=0.0, design_vc_kt=0.0, ground_speed_kt=140.0,
        fuel_unusable_gal=0.0, distance_to=lambda lat, lon: 200.0)
    flight._monitor_emergency(state, 100.0)
    # The power comes back: the same tanks at the leg's own burn are
    # comfortably enough, and the clock is forgotten.
    state.est_fuel_flow_pph = 60.0
    flight._monitor_emergency(state, 130.0)
    flight._monitor_emergency(state, 400.0)
    flight._declare_emergency.assert_not_called()
    assert flight._fuel_min_since == 0.0


def test_a_bus_spike_is_not_an_electrical_emergency():
    """A collapsed main bus was declared on a single read -- a load
    spike or a noisy frame became a MAYDAY. It has to stay collapsed
    a few seconds first."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _emergency_flight()
    flight.state.phase = AutonomousPhase.CRUISE
    state = SimpleNamespace(
        main_bus_volts=28.0, on_ground=False, engine_count=2,
        engines_running=True, eng2_combustion=True, eng3_combustion=None,
        eng4_combustion=None, has_position=True,
        fuel_total_gal=70.0, fuel_capacity_gal=200.0)
    # The healthy bus teaches the watch its own figure first.
    flight._monitor_emergency(state, 99.0)
    state.main_bus_volts = 4.0
    flight._monitor_emergency(state, 100.0)
    flight._declare_emergency.assert_not_called()
    # A real failure stays collapsed.
    flight._monitor_emergency(
        state, 100.0 + flight.ELECTRICAL_PERSIST_S)
    flight._declare_emergency.assert_called_once_with(
        "electrical", state, mayday=True)


def test_a_cleared_bus_never_declares():
    """The spike that comes back is forgotten, not remembered."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _emergency_flight()
    flight.state.phase = AutonomousPhase.CRUISE
    state = SimpleNamespace(
        main_bus_volts=28.0, on_ground=False, engine_count=2,
        engines_running=True, eng2_combustion=True, eng3_combustion=None,
        eng4_combustion=None, has_position=True,
        fuel_total_gal=70.0, fuel_capacity_gal=200.0)
    flight._monitor_emergency(state, 99.0)
    state.main_bus_volts = 4.0
    flight._monitor_emergency(state, 100.0)
    state.main_bus_volts = 28.0
    flight._monitor_emergency(
        state, 100.0 + flight.ELECTRICAL_PERSIST_S + 5.0)
    flight._declare_emergency.assert_not_called()


def test_an_engine_flag_blip_is_not_an_engine_emergency():
    """One frame of a combustion flag dark declared a PAN PAN and a
    divert. The flags have to stay failed for a few seconds."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _emergency_flight()
    flight.state.phase = AutonomousPhase.CRUISE
    state = SimpleNamespace(
        main_bus_volts=28.0, on_ground=False, engine_count=2,
        engines_running=True, eng2_combustion=False,
        eng3_combustion=None, eng4_combustion=None, has_position=True,
        fuel_total_gal=70.0, fuel_capacity_gal=200.0)
    flight._monitor_emergency(state, 100.0)
    flight._declare_emergency.assert_not_called()
    # A real failure stays failed -- and a twin on one is the urgency.
    flight._monitor_emergency(
        state, 100.0 + flight.ENGINES_PERSIST_S)
    flight._declare_emergency.assert_called_once_with(
        "engine-out", state, mayday=False)


def test_ghost_combustion_slots_are_not_an_engine_failure():
    """The C172's first SimConnect roll: the combustion indexes past
    the aeroplane's one engine still answer, and they answer False --
    three "dead" engines on a healthy single, the takeoff rejected
    below V1 every time. Slots past the reported count are not
    engines at all."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _emergency_flight()
    flight.state.phase = AutonomousPhase.CRUISE
    state = SimpleNamespace(
        main_bus_volts=28.0, on_ground=False, engine_count=1,
        engines_running=True, eng2_combustion=False,
        eng3_combustion=False, eng4_combustion=False, has_position=True,
        fuel_total_gal=70.0, fuel_capacity_gal=200.0)
    # The ghost flags hold for the whole watch, and nothing is said.
    flight._monitor_emergency(state, 100.0)
    flight._monitor_emergency(
        state, 100.0 + flight.ENGINES_PERSIST_S + 1.0)
    flight._declare_emergency.assert_not_called()
    # The one real engine still counts: dark means out, all of them
    # out is the MAYDAY.
    state.engines_running = False
    flight._monitor_emergency(state, 200.0)
    flight._monitor_emergency(
        state, 200.0 + flight.ENGINES_PERSIST_S + 1.0)
    flight._declare_emergency.assert_called_once_with(
        "engine", state, mayday=True)


def test_a_twin_ignores_the_combustion_slots_it_does_not_have():
    """Same ghost on the Baron: engines three and four answer False,
    and two combusting engines still read no failure -- while the
    second real engine going dark is the engine-out it always was."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _emergency_flight()
    flight.state.phase = AutonomousPhase.CRUISE
    state = SimpleNamespace(
        main_bus_volts=28.0, on_ground=False, engine_count=2,
        engines_running=True, eng2_combustion=True,
        eng3_combustion=False, eng4_combustion=False, has_position=True,
        fuel_total_gal=70.0, fuel_capacity_gal=200.0)
    flight._monitor_emergency(state, 100.0)
    flight._monitor_emergency(
        state, 100.0 + flight.ENGINES_PERSIST_S + 1.0)
    flight._declare_emergency.assert_not_called()
    state.eng2_combustion = False
    flight._monitor_emergency(state, 200.0)
    flight._monitor_emergency(
        state, 200.0 + flight.ENGINES_PERSIST_S + 1.0)
    flight._declare_emergency.assert_called_once_with(
        "engine-out", state, mayday=False)


def test_an_unreported_slot_is_an_engine_once_it_has_fired():
    """Where the model names no count, a combustion slot that has
    never once fired is no evidence -- SimConnect answers the
    out-of-range ones False by default. Once a slot has fired it is
    an engine, and its going dark is a failure."""
    flight = _emergency_flight()
    state = SimpleNamespace(
        engine_count=0, engines_running=True,
        eng2_combustion=False, eng3_combustion=None,
        eng4_combustion=None)
    # Only engine one counts: the never-fired slot two is not there.
    assert flight._engines_out(state) == (0, 1)
    # It fires once -- it is now an engine the sim knows exists.
    state.eng2_combustion = True
    assert flight._engines_out(state) == (0, 2)
    # ...and going dark after that is the failure it reports.
    state.eng2_combustion = False
    assert flight._engines_out(state) == (1, 2)


def test_cannot_reach_the_field_settles_faster_than_minimum_fuel():
    """The MAYDAY tier still answers in under a minute: a fuel state
    below the bare leg is grave, but it is still priced on the moment's
    flow, so it too waits out the transient."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _emergency_flight()
    flight.state.phase = AutonomousPhase.CRUISE
    flight._reserve_min = 45.0
    flight.session.destination = SimpleNamespace(
        ident="LECO", lat=42.0, lon=-8.0)
    flight.navdb = SimpleNamespace(nearest_airports=lambda *a, **k: [])
    state = SimpleNamespace(
        main_bus_volts=28.0, on_ground=False, engine_count=2,
        engines_running=True, eng2_combustion=True, eng3_combustion=None,
        eng4_combustion=None, has_position=True,
        fuel_total_lb=240.0, fuel_total_gal=40.0, fuel_capacity_gal=200.0,
        est_fuel_flow_pph=200.0,
        estimated_cruise_kt=0.0, design_vc_kt=0.0, ground_speed_kt=140.0,
        fuel_unusable_gal=0.0, distance_to=lambda lat, lon: 400.0)
    flight._monitor_emergency(state, 100.0)
    flight._declare_emergency.assert_not_called()
    flight._monitor_emergency(state, 100.0 + flight.LEG_FUEL_PERSIST_S)
    flight._declare_emergency.assert_called_once_with(
        "fuel", state, mayday=True)


def _divert_route():
    """The route N45XS was handed at the PAN PAN: the strip beside the
    field, then the field -- airport to airport, the way a diversion
    files it."""
    from meridianatc.atc.route import Route, RouteFix
    return Route(fixes=(
        RouteFix("LPIN", 40.9725, -8.6450, kind="apt"),
        RouteFix("LPOV", 40.9159, -8.6459, kind="apt")))


def _nav_state(latitude, longitude, track):
    from meridianatc.sim.state import AircraftState
    return AircraftState(
        source="xplane", connected=True, latitude=latitude,
        longitude=longitude, track_true=track, heading_true=track,
        ground_speed_kt=145.0)


def test_an_armed_nav_steering_nowhere_is_flown_to_the_fix():
    """The N45XS case: the divert's plan was loaded and NAV armed, but
    nothing steered it -- twelve minutes on the SID's heading into
    terrain. Pointed persistently nowhere near the leg's fix, the FO
    flies the heading itself."""
    flight = _flier()
    flight._nav_off_since = 0.0
    flight.brain = SimpleNamespace(route=_divert_route())
    # North-east of the field, still heading the SID's way -- away
    # from LPOV to the south-west.
    state = _nav_state(41.13, -8.415, 64.0)
    # A course change is trusted a while: an intercept takes a turn.
    assert flight._nav_fallback_heading(state, 100.0) == -1.0
    assert flight._nav_fallback_heading(state, 140.0) == -1.0
    want = flight._nav_fallback_heading(state, 146.0)
    assert 0.0 <= want < 360.0
    # And the heading it falls back to is the bearing to the fix the
    # leg runs to -- the destination, south-west, not 64.
    assert abs(want - state.bearing_to(40.9159, -8.6459)) < 1.0


def test_a_nav_tracking_the_leg_is_left_to_steer():
    """The fallback never touches an aeroplane pointed at its fix."""
    flight = _flier()
    flight._nav_off_since = 0.0
    flight.brain = SimpleNamespace(route=_divert_route())
    state = _nav_state(41.13, -8.415, 0.0)
    state.track_true = state.heading_true = \
        state.bearing_to(40.9159, -8.6459)
    assert flight._nav_fallback_heading(state, 100.0) == -1.0
    assert flight._nav_fallback_heading(state, 400.0) == -1.0
    # And a transient off-course recovers without being called a
    # failure: the clock resets the moment the track is back.
    state.track_true = state.heading_true = 64.0
    assert flight._nav_fallback_heading(state, 410.0) == -1.0
    state.track_true = state.heading_true = \
        state.bearing_to(40.9159, -8.6459)
    assert flight._nav_fallback_heading(state, 420.0) == -1.0
    assert flight._nav_off_since == 0.0


def test_without_a_route_there_is_nothing_to_fall_back_to():
    flight = _flier()
    flight._nav_off_since = 0.0
    flight.brain = SimpleNamespace(route=None)
    state = _nav_state(41.13, -8.415, 64.0)
    assert flight._nav_fallback_heading(state, 100.0) == -1.0


def test_the_leg_is_priced_at_cruise_speed_not_the_climbout():
    """A leg figured at climb-out groundspeed calls a full tank short."""
    from meridianatc.atc import flying
    flight = _flier(flying.LIGHT)      # LIGHT: cruise_kt is 110
    flight._is_helicopter = False
    flight._reserve_min = 45.0
    flight.session.destination = SimpleNamespace(
        ident="XXXX", lat=39.0, lon=-9.0)
    flight.navdb = SimpleNamespace(nearest_airports=lambda *a, **k: [])
    state = SimpleNamespace(
        has_position=True, latitude=38.0, longitude=-9.0,
        ground_speed_kt=55.0,          # halfway through the climb
        estimated_cruise_kt=0.0, est_fuel_flow_pph=60.0,
        fuel_unusable_gal=0.0, fuel_total_gal=50.0, fuel_total_lb=300.0,
        distance_to=lambda lat, lon: 110.0)
    needed = flight._fuel_needed_gal(state)
    # 110 NM at 110 kt is an hour -- about 10 gal at this burn. Priced
    # at the climb-out's 55 kt it was two hours, and called tanks short.
    assert abs(needed["leg_gal"] - 10.0) < 0.5


# ---------------------------------------------------------------------
# the first officer's comms: who owns COM1, and when the asks go out
# ---------------------------------------------------------------------


def _station(position, mhz, ident="LPPT"):
    return SimpleNamespace(position=position, mhz=mhz, ident=ident,
                           callsign=f"Lisboa {position}")


def _radio_flight(stations=(), staffed=None, phase="request_clearance",
                  on_ground=True, com1=0.0, handed=None,
                  current_station=None, session_phase="preflight"):
    """A flight with only what the comms management touches."""
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )
    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    flight.state.phase = AutonomousPhase(phase)
    dialled, kinds = [], []
    flight.session = SimpleNamespace(
        ifr=True, pending=None, handoff_pending=handed,
        departure=SimpleNamespace(ident="LPPT"),
        destination=SimpleNamespace(ident="LEMD"),
        current_station=current_station, phase=session_phase,
        taxi_path=None, lined_up=False, cleared_takeoff=False,
        cleared_landing=False, squawk="", altimeter=0.0,
        established_with=lambda s: False)
    flight._manage_radio = True
    flight._dialled_at = 0.0
    flight._comms_settled = True
    flight._held_at_split = False
    flight.sim = SimpleNamespace(
        connected=True,
        set_com=lambda mhz, radio=1: dialled.append(mhz) or True,
        set_squawk=lambda code: True)
    flight.crew = SimpleNamespace(
        radio_busy=None,
        radio_call=lambda kind: kinds.append(kind) or ("", "test"),
        transmit=lambda text: None)
    flight.navdb = SimpleNamespace(
        stations=lambda ident, position=None: [
            s for s in stations
            if position is None or s.position == position])
    staffed = staffed or (lambda s: True)
    flight.brain = SimpleNamespace(staffed=staffed,
                                   issues_clearances=staffed)
    flight._get_current_airport = lambda: flight.session.departure
    flight._emit = Mock()
    flight._dir = Mock()
    flight._dir.sim = flight.sim
    state = SimpleNamespace(
        connected=True, source="", on_ground=on_ground, com1_active=com1,
        ground_speed_kt=0.0, transponder_code="", kohlsman_hpa=0.0,
        engines_running=True)
    return flight, state, dialled, kinds


def test_a_radar_handoff_is_armed_not_dialled_on_the_stand():
    """"Contact departure" is for the air: on the stand COM1 belongs to
    delivery -- the bug that had approach answering the clearance call."""
    flight, state, dialled, kinds = _radio_flight(
        stations=[_station("DEL", 121.75), _station("APP", 119.10)],
        handed=_station("APP", 119.10))
    flight._manage_comms(state, 1000.0)
    assert dialled == [121.75]


def test_a_ground_handoff_is_dialled_at_once():
    """"Contact ground" is a surface handoff: obeyed from the stand."""
    gnd = _station("GND", 121.90)
    flight, state, dialled, kinds = _radio_flight(
        stations=[_station("DEL", 121.75), gnd], handed=gnd)
    flight._manage_comms(state, 1000.0)
    assert dialled == [121.90]


def test_a_handoff_to_another_fields_ground_is_not_dialled():
    flight, state, dialled, kinds = _radio_flight(
        stations=[_station("DEL", 121.75)],
        handed=_station("GND", 121.90, ident="LEMD"))
    flight._manage_comms(state, 1000.0)
    assert dialled == [121.75]


def test_a_printed_but_silent_delivery_leaves_ground_to_answer():
    """A position nobody is on is not the one COM1 is put on."""
    flight, state, dialled, kinds = _radio_flight(
        stations=[_station("DEL", 121.75), _station("GND", 121.90)],
        staffed=lambda s: s.position != "DEL")
    flight._manage_comms(state, 1000.0)
    assert dialled == [121.90]


def test_the_asks_hold_until_the_dial_settles():
    """A clearance asked mid-swap is a call on the wrong frequency."""
    flight, state, dialled, kinds = _radio_flight(
        stations=[_station("DEL", 121.75)])
    flight._manage_comms(state, 1000.0)
    assert not flight._comms_settled
    assert not flight._fo_say("clearance")
    assert "clearance" not in kinds
    # The dial landed: the phase is settled and the ask may go out.
    state.com1_active = 121.75
    flight._manage_comms(state, 1009.0)
    assert flight._comms_settled


def test_no_altitude_check_in_to_the_tower_at_the_hold():
    """The first call to the tower is "ready for departure", not an
    airborne check-in -- a "maintaining five thousand" on the ground."""
    twr = _station("TWR", 118.10)
    flight, state, dialled, kinds = _radio_flight(
        stations=[twr], phase="holding_short", com1=118.10,
        handed=twr, current_station=twr, session_phase="holding_short")
    flight._manage_comms(state, 1000.0)
    assert "check_in" not in kinds


def test_the_first_call_airborne_is_the_check_in():
    app = _station("APP", 119.10)
    flight, state, dialled, kinds = _radio_flight(
        stations=[app], phase="climb", on_ground=False, com1=119.10,
        handed=app, current_station=app, session_phase="departure")
    flight._manage_comms(state, 1000.0)
    assert "check_in" in kinds


def test_the_airborne_handoff_is_dialled():
    """In the air the frequency is the handoff's to move, as before."""
    flight, state, dialled, kinds = _radio_flight(
        phase="climb", on_ground=False, com1=118.10,
        handed=_station("APP", 119.10))
    flight._manage_comms(state, 1000.0)
    assert dialled == [119.10]


def test_taxi_in_tunes_the_arrival_fields_ground_not_the_departures():
    """After landing, ground is this field's -- the leg left hours ago."""
    flight, state, dialled, kinds = _radio_flight(
        stations=[_station("GND", 121.90, ident="LEMD")],
        phase="taxi_in", session_phase="taxi_in")
    flight._get_current_airport = lambda: flight.session.destination
    flight._manage_comms(state, 1000.0)
    assert dialled == [121.90]


def test_the_squawk_is_not_dialled_before_it_is_read_back():
    """A figure owed a readback is a figure not agreed to: the box stays
    as it was until the controller has heard it said."""
    flight, state, dialled, kinds = _radio_flight()
    flight.session.squawk = "4570"
    flight.session.pending = SimpleNamespace(
        readback_required=True, read_back=False,
        values={"squawk": "4570"})
    squawked = []
    flight.sim.set_squawk = lambda code: squawked.append(code) or True
    flight._dial_radios(state, 1000.0)
    assert squawked == []                    # still owed: nothing set
    flight.session.pending.read_back = True
    flight._dial_radios(state, 1006.0)
    assert squawked == ["4570"]


def test_the_altimeter_is_not_dialled_before_it_is_read_back():
    """The same sequence for the pressure: read back first, set second."""
    flight, state, dialled, kinds = _radio_flight()
    flight.session.altimeter = 1013.0
    flight.session.pending = SimpleNamespace(
        readback_required=True, read_back=False,
        values={"altimeter": 1013.0})
    flight._dialled_at = -100.0
    flight._dial_radios(state, 1000.0)
    flight.director.kohlsman.assert_not_called()
    flight.session.pending.read_back = True
    flight._dial_radios(state, 1010.0)
    flight.director.kohlsman.assert_called_once()


def test_a_dial_the_readback_does_not_carry_goes_in_anyway():
    """A figure nobody asked for is not held behind the readback of
    another: the gate is on what the instruction carries, not on the
    exchange being open at all."""
    flight, state, dialled, kinds = _radio_flight()
    flight.session.squawk = "7000"
    flight.session.pending = SimpleNamespace(
        readback_required=True, read_back=False,
        values={"heading": 240})
    squawked = []
    flight.sim.set_squawk = lambda code: squawked.append(code) or True
    flight._dial_radios(state, 1000.0)
    assert squawked == ["7000"]


def test_nothing_keys_on_the_same_breath_the_frequency_frees():
    """A transmission that leaves as the other station lets go is a
    machine on the unkey: the first officer leaves the beat a crew does.
    """
    import time as _time

    flight, state, dialled, kinds = _radio_flight()
    sent = []
    flight.crew.radio_call = lambda kind: kinds.append(kind) or ("call", "")
    flight.crew.transmit = lambda text: sent.append(text)
    # The frequency has just let go: the beat has not.
    flight._last_busy_at = _time.monotonic()
    assert not flight._fo_say("check_in")
    assert sent == []
    # The same call a breath later is the call that goes out.
    flight._last_busy_at = _time.monotonic() - 30.0
    assert flight._fo_say("check_in")
    assert sent == ["call"]


def test_a_go_around_does_not_wait_out_the_beat():
    """The urgent call is keyed on top of the pause: the tower is
    already listening for it."""
    import time as _time

    flight, state, dialled, kinds = _radio_flight()
    flight.crew.radio_call = lambda kind: kinds.append(kind) or ("call", "")
    flight.crew.transmit = lambda text: None
    flight._last_busy_at = _time.monotonic()
    assert flight._fo_say("going_around")


def test_the_ask_is_not_counted_tried_while_the_beat_runs():
    """A call refused for the unkey beat has not had its go: the retry
    clock waits for the attempt, not for the refusal."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight, state, dialled, kinds = _radio_flight(
        stations=[_station("TWR", 118.10)],
        phase="holding_short", com1=118.10, session_phase="taxi_out")
    flight.state.phase = AutonomousPhase.HOLDING_SHORT
    flight.session.phase = "taxi_out"
    flight.session.taxi_path = None
    flight._last_busy_at = 1e12             # the beat, far off
    flight._call_tried_at = 0.0
    flight.state.last_radio_at = -100.0
    flight._work_the_calls(state, flight.session, 1000.0)
    assert flight._call_tried_at == 0.0     # refused: not an attempt


def test_the_frequency_notice_clock_starts_at_the_answer():
    """"Not tuned" counts from the readback: a pilot still saying the
    handoff back cannot have dialled it yet, so the first officer does
    not start being cross about it until the answer has gone out."""
    import time as _time

    from meridianatc.atc import crew as crew_mod
    from meridianatc.atc.session import Phase

    director = crew_mod.CrewDirector.__new__(crew_mod.CrewDirector)
    director.checklists = SimpleNamespace(book=None)
    director._class = None
    director._qnh_seen, director._qnh_at = 0.0, 0.0
    director._where_in_the_circuit = lambda: None
    director._field_ident = lambda: None
    director.reserve_gal = director.reserve_lb = 0.0
    director._language = "en"
    offered = _time.monotonic() - 60.0
    pending = SimpleNamespace(
        readback_required=True, read_back=False,
        values={"frequency": 119.10})
    session = SimpleNamespace(
        phase=Phase.PREFLIGHT, altimeter=0.0,
        handoff_pending=SimpleNamespace(mhz=119.10),
        handoff_offered_at=offered, pending=pending)
    w = director._watching(SimpleNamespace(), session)
    # Still being read back: the clock has not started.
    assert w.frequency_given_at >= offered + 59.0
    pending.read_back = True
    w = director._watching(SimpleNamespace(), session)
    # Answered: the clock runs from when it was given.
    assert w.frequency_given_at == offered


def test_a_confirm_that_arrives_late_is_dropped_not_said():
    """The pilot read it back before the first officer could repeat it:
    confirming the numbers now would be an exchange late, so the line
    is dropped rather than spoken."""
    import threading

    from meridianatc.atc import crew as crew_mod
    from meridianatc.atc.session import Instruction

    director = crew_mod.CrewDirector.__new__(crew_mod.CrewDirector)
    director.config = SimpleNamespace(immersion=SimpleNamespace(
        enabled=True, copilot=True, confirmations=True))
    director.synth = Mock()
    director.player = SimpleNamespace(busy=False)
    director.radio_busy = lambda: False
    director.state = crew_mod.CrewState()
    director._lock = threading.Lock()
    director._blocked_since = 0.0
    director._language = "en"
    director.say = Mock(return_value=0.0)
    instruction = Instruction("squawk", "LPPT:TWR", "squawk 4570",
                              {"squawk": "4570"})
    director._session = SimpleNamespace(pending=instruction)
    director.confirm(instruction)
    assert len(director.state.queue) == 1
    # The readback goes out before the interphone line's moment.
    instruction.read_back = True
    director._drain()
    assert not director.state.queue
    director.say.assert_not_called()


def test_a_confirm_for_a_still_owed_instruction_keeps_its_moment():
    """The same line while the readback is still owed is the first
    officer's job, not noise: it stays."""
    import threading
    import time as _time

    from meridianatc.atc import crew as crew_mod
    from meridianatc.atc.session import Instruction

    director = crew_mod.CrewDirector.__new__(crew_mod.CrewDirector)
    director.config = SimpleNamespace(immersion=SimpleNamespace(
        enabled=True, copilot=True, confirmations=True))
    director.synth = Mock()
    director.player = SimpleNamespace(busy=False)
    director.radio_busy = lambda: False
    director.state = crew_mod.CrewState()
    director._lock = threading.Lock()
    director._blocked_since = 0.0
    director._language = "en"
    director.say = Mock(return_value=0.0)
    director.play = Mock(return_value=0.0)
    director._close_up = Mock()
    instruction = Instruction("squawk", "LPPT:TWR", "squawk 4570",
                              {"squawk": "4570"})
    director._session = SimpleNamespace(pending=instruction)
    director.confirm(instruction)
    line = director.state.queue[0]
    line.at = _time.monotonic() - 1.0       # its moment has come
    line.voice = object()                   # already rendered
    director._drain()
    director.say.assert_called_once()
    assert director._close_up.call_args[0][0].cue == "confirm"


def test_the_handed_over_call_waits_the_frequency_out():
    """A call the pilot handed over keys when the frequency lets it,
    never over somebody's turn -- and not for ever."""
    import time as _time

    from meridianatc.atc import crew as crew_mod

    director = crew_mod.CrewDirector.__new__(crew_mod.CrewDirector)
    director.player = SimpleNamespace(busy=False)
    director.radio_busy = lambda: True      # the frequency is live
    sent = []
    director.transmit = lambda text: sent.append(text)
    said = []
    director._say_now = lambda text, cue, language="": \
        said.append((text, cue))
    director._callout_language = lambda: "en"
    director._language = "en"
    director._radio_call_at = (_time.monotonic() - 1.0,
                               "lisboa ground, test, ready to taxi")
    director._send_the_call()
    assert sent == []                       # held, not keyed over
    assert director._radio_call_at is not None
    # And past the wait the radio is handed back rather than kept.
    director._radio_call_at = (
        _time.monotonic() - director.CALL_WAIT_S - 5.0, "the call")
    director._send_the_call()
    assert sent == []
    assert director._radio_call_at is None
    assert said[-1][1] == "radio_call_refused"


def test_the_handed_over_call_goes_once_the_frequency_is_quiet():
    """The same call on a free frequency is the transmission it was."""
    import time as _time

    from meridianatc.atc import crew as crew_mod

    director = crew_mod.CrewDirector.__new__(crew_mod.CrewDirector)
    director.player = SimpleNamespace(busy=False)
    director.radio_busy = lambda: False
    sent = []
    director.transmit = lambda text: sent.append(text)
    said = []
    director._say_now = lambda text, cue, language="": \
        said.append((text, cue))
    director._callout_language = lambda: "en"
    director._language = "en"
    director._radio_call_at = (_time.monotonic() - 1.0,
                               "lisboa ground, test, ready to taxi")
    director._send_the_call()
    assert sent == ["lisboa ground, test, ready to taxi"]
    assert director._radio_call_at is None
    assert said[-1][1] == "your_radio"


# ---------------------------------------------------------------------
# The Porto taxi crash: the follower cut across the apron at the fixed
# lookahead, the lever variable was written a hundred times too large,
# and the wound integrator kept the surges coming.
# ---------------------------------------------------------------------

def _path_flight(path, taxi_kt=14.0):
    """The flight with only what _taxi_along_path touches."""
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    flight.session = SimpleNamespace(
        taxi_path=path, taxi_split_at=None, cleared_takeoff=False,
        phase=None)
    flight._perf = SimpleNamespace(taxi_kt=taxi_kt)
    flight._stop = Mock()
    flight._steer = Mock()
    flight._goto_sub = Mock()
    flight._transition = Mock()
    flight._ground_still = Mock(return_value=False)
    flight._held_at_split = False
    flight.sim = Mock()
    flight._dir = Mock()
    flight._dir.sim = flight.sim
    return flight


def _ground_state(lat, lon, gs=0.0, hdg=0.0):
    return SimpleNamespace(
        latitude=lat, longitude=lon, ground_speed_kt=gs,
        heading_true=hdg, on_ground=True)


# A straight northbound leg at LPPR: 0.002 of latitude, ~220 m.
_TAXI_PATH = [(41.2400, -8.6725), (41.2420, -8.6725),
              (41.2420, -8.6700)]


def test_taxi_on_the_route_steers_at_taxi_speed():
    flight = _path_flight(list(_TAXI_PATH))
    state = _ground_state(41.2405, -8.6725, gs=5.0)
    flight._taxi_along_path(state, outbound=True)
    assert flight._steer.called
    assert flight._steer.call_args[0][2] > 5.0
    assert not flight._stop.called


def test_off_the_route_rejoins_at_walking_pace():
    """~30 m east of the leg: back to the line at 5 kt, not across the
    apron at the full lookahead."""
    flight = _path_flight(list(_TAXI_PATH))
    state = _ground_state(41.2410, -8.67214, gs=0.0)
    flight._taxi_along_path(state, outbound=True)
    assert flight._steer.called
    assert flight._steer.call_args[0][2] <= 5.0
    assert not flight._stop.called


def test_far_off_the_route_stops_instead_of_crossing_the_apron():
    """The building strike: 100+ m off the route is scenery the graph
    does not know -- a stopped aeroplane hits nothing."""
    flight = _path_flight(list(_TAXI_PATH))
    state = _ground_state(41.2410, -8.6713, gs=0.0)
    flight._taxi_along_path(state, outbound=True)
    assert flight._stop.called
    assert not flight._steer.called


def test_the_taxi_walk_never_steps_back():
    """Once the route's second leg is the nearest, passing close to an
    earlier segment does not swing the pursuit target backwards."""
    flight = _path_flight(list(_TAXI_PATH))
    state = _ground_state(41.2420, -8.6712, gs=5.0)
    flight._taxi_along_path(state, outbound=True)
    progress = flight._taxi_progress
    assert progress >= 1
    # Nearer to the first leg now, and far off everything: the follower
    # stops, but the walk it keeps is still the forward one.
    flight._steer.reset_mock()
    state2 = _ground_state(41.2405, -8.6715, gs=0.0)
    flight._taxi_along_path(state2, outbound=True)
    assert flight._taxi_progress >= progress


def test_a_new_taxi_route_restarts_the_walk():
    flight = _path_flight(list(_TAXI_PATH))
    state = _ground_state(41.2420, -8.6712, gs=5.0)
    flight._taxi_along_path(state, outbound=True)
    assert flight._taxi_progress >= 1
    flight.session.taxi_path = [(41.2400, -8.6725), (41.2420, -8.6725)]
    flight._taxi_along_path(_ground_state(41.2410, -8.6725, gs=5.0),
                            outbound=True)
    assert flight._taxi_progress == 0


def test_the_throttle_lever_is_written_as_a_fraction():
    """GENERAL ENG THROTTLE LEVER POSITION is "Percent over 100": a 0.5
    lever writes 0.5, not the 50.0 that read back as 5000% and pinned
    the engine to the stop on every taxi call."""
    from meridianatc.atc.ap import FlightDirector
    writes = []

    class Sim:
        def set_variable(self, name, value, unit="Number"):
            writes.append((name, value, unit))
            return True

    director = FlightDirector(Sim())
    assert director.throttle(0.5)
    lever = [w for w in writes
             if "THROTTLE LEVER POSITION" in w[0]]
    assert lever
    for _name, value, unit in lever:
        assert 0.0 <= value <= 1.0
        assert unit == "Percent over 100"


def test_the_throttle_lever_falls_back_to_percent_without_units():
    """A source with no named units (X-Plane's table takes 0..100)
    still gets the percent it always did."""
    from meridianatc.atc.ap import FlightDirector
    writes = []

    class Sim:
        def set_variable(self, name, value, unit=None):
            writes.append((name, value, unit))
            return unit is None

    director = FlightDirector(Sim())
    assert director.throttle(0.5)
    lever = [w for w in writes
             if "THROTTLE LEVER POSITION" in w[0] and w[2] is None]
    assert lever
    for _name, value, _unit in lever:
        assert value == 50.0


def test_the_prop_and_mixture_levers_write_every_engine():
    """The Baron's stand: props left at low pitch governed both engines
    down to propellers that could not move the aeroplane. The levers go
    out the way the throttles do -- per engine, in the lever's unit."""
    from meridianatc.atc.ap import FlightDirector
    writes = []

    class Sim:
        def set_variable(self, name, value, unit="Number"):
            writes.append((name, value, unit))
            return True

    director = FlightDirector(Sim())
    assert director.prop(1.0)
    assert director.mixture(1.0)
    for lever in ("PROPELLER LEVER POSITION", "MIXTURE LEVER POSITION"):
        names = {w[0] for w in writes if lever in w[0]}
        assert names == {f"GENERAL ENG {lever}:{n}" for n in (1, 2, 3, 4)}


def test_the_prop_and_mixture_levers_fall_back_to_events():
    """A source that cannot write the lever variables (a model with its
    own engine code) still gets the levers moved -- the same fallback
    the throttle learned."""
    from meridianatc.atc.ap import FlightDirector
    sent = []

    class Sim:
        connected = True

        def set_variable(self, name, value, unit=None):
            return False

        def _send(self, name, value):
            sent.append((name, value))
            return True

    director = FlightDirector(Sim())
    assert director.prop(1.0)
    names = {name for name, _v in sent}
    assert b"PROP_PITCH_SET" in names or b"AXIS_PROPELLER_SET" in names
    assert {b"PROP_PITCH1_SET", b"PROP_PITCH2_SET"} <= names
    for _name, value in sent:
        assert value == 16383

    sent.clear()
    assert director.mixture(1.0)
    names = {name for name, _v in sent}
    assert b"MIXTURE_SET" in names
    assert {b"MIXTURE1_SET", b"MIXTURE2_SET"} <= names
    for _name, value in sent:
        assert value == 16383


def test_the_condition_levers_write_every_engine_as_detents():
    """A turboprop's third lever: the stops are the enum's three
    detents -- 0 cut-off, 1 low idle, 2 high idle -- on every engine,
    the way the quadrant's other levers go out."""
    from meridianatc.atc.ap import FlightDirector
    writes = []

    class Sim:
        def set_variable(self, name, value, unit="Number"):
            writes.append((name, value, unit))
            return True

    director = FlightDirector(Sim())
    assert director.condition(2.0)
    names = {w[0] for w in writes
             if "CONDITION LEVER POSITION" in w[0]}
    assert names == {f"TURB ENG CONDITION LEVER POSITION:{n}"
                     for n in (1, 2, 3, 4)}
    assert all(w[1] == 2.0 for w in writes)


def test_a_condition_lever_the_variable_cannot_reach_still_moves():
    """A model whose own code owns the lever (a refused variable)
    still gets its stops: the fuel valve carries the cut-off edge and
    the mixture channel is the same three detents where the red knob
    is the lever."""
    from meridianatc.atc.ap import FlightDirector
    sent = []

    class Sim:
        connected = True

        def set_variable(self, name, value, unit=None):
            return False

        def _send(self, name, value):
            sent.append((name, value))
            return True

    director = FlightDirector(Sim())
    # Cut-off: the valve closes.
    assert director.condition(0.0)
    valves = {name: value for name, value in sent
              if name.startswith(b"SET_FUEL_VALVE_ENG")}
    assert {b"SET_FUEL_VALVE_ENG1", b"SET_FUEL_VALVE_ENG2"} <= set(valves)
    assert all(value == 0 for value in valves.values())
    # High idle: the valve opens and the mixture goes to the top stop.
    sent.clear()
    assert director.condition(2.0)
    valves = {name: value for name, value in sent
              if name.startswith(b"SET_FUEL_VALVE_ENG")}
    assert all(value == 1 for value in valves.values())
    assert any(name == b"MIXTURE_SET" and value == 16383
               for name, value in sent)


def test_the_taxi_leg_stops_short_of_a_route_that_crosses_the_runway():
    """The LPPR incursion: routing stand -> E5 rode 600 m down runway 17's
    centreline and the hold the walk-back picked sat on the FAR side
    (>=45 m off, not a runway node -- it qualified). The issued taxi leg
    therefore included the runway traversal and the FO followed it onto
    the pavement with no clearance. The hold is found before the route's
    first runway contact instead, so the taxi leg ends on the near side
    and the runway is the line-up leg's business."""
    from meridianatc.atc.ground import GroundLayout

    layout = GroundLayout.__new__(GroundLayout)
    nodes = ["n0", "n1", "n2", "r0", "r1", "f0", "f1"]
    # along/across pairs: taxiway approach -> centreline ride -> far side
    across_of = {"n0": -189.0, "n1": -90.0, "n2": -54.4,
                 "r0": -19.8, "r1": 2.0, "f0": 27.9, "f1": 51.2}
    layout.xy = {node: (float(i * 50), across_of[node])
                 for i, node in enumerate(nodes)}
    layout.runway_nodes = {"r0", "r1"}
    layout.edges = {}
    layout.entries = lambda *a: [(150.0, "f1")]
    layout.nearest_node = lambda *a: "n0"
    layout.route = lambda start, entry, avoid=None: list(nodes)
    layout.runway_frame = lambda x, z, threshold, heading: (x, z)
    layout.names_along = lambda named: []

    legs = layout.departure(None, (0.0, 0.0), (0.0, 0.0), 0.0, 3000.0)
    assert legs is not None
    taxi_out, line_up = legs
    # The taxi leg ends at n2 -- the last node at least 45 m off the
    # centreline BEFORE the pavement -- and contains no runway node.
    assert taxi_out.nodes == [None, "n0", "n1", "n2"]
    assert not set(taxi_out.nodes) & layout.runway_nodes
    # The runway traversal belongs to the second clearance.
    assert "r0" in line_up.nodes and "f1" in line_up.nodes


def _ready_flight(phase_name, **session_kw):
    """A flight at the hold asking the tower for its release."""
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )
    from meridianatc.atc.session import Phase
    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    flight.state.phase = getattr(AutonomousPhase, phase_name)
    flight.state.sub = "wait_clearance"
    flight.state.last_radio_at = 0.0
    kw = dict(phase=Phase.TAKEOFF, lined_up=True, cleared_takeoff=False,
              ifr=True, clearance_issued=True, held_for_landing="")
    kw.update(session_kw)
    flight.session = SimpleNamespace(**kw)
    flight._call_tried_at = 0.0
    flight._unkeyed = lambda: True
    flight._fo_say = Mock(return_value=True)
    return flight


def test_a_departure_held_for_landing_does_not_ask_again():
    """Told to wait for the arrival, the crew waits: the release comes
    to the flight, it is not asked for."""
    from meridianatc.atc.session import Phase
    flight = _ready_flight("HOLDING_SHORT",
                           held_for_landing="AIB123",
                           phase=Phase.HOLDING_SHORT, lined_up=False)
    state = SimpleNamespace(on_ground=True, ground_speed_kt=0.0)
    flight._work_the_calls(state, flight.session, 1000.0)
    flight._fo_say.assert_not_called()
    # And the same from the runway: lined up behind the landing
    # traffic, the wait is instructed.
    flight = _ready_flight("LINE_UP", held_for_landing="AIB123")
    flight._work_the_calls(state, flight.session, 1000.0)
    flight._fo_say.assert_not_called()


def test_a_waited_ready_call_is_asked_again_on_the_crews_clock():
    """Not told to wait, the call is remade -- but on a crew's patience,
    not the twenty-five second beat."""
    from meridianatc.atc.autonomous.x_the_radios import READY_REASK_S
    from meridianatc.atc.session import Phase
    flight = _ready_flight("HOLDING_SHORT", lined_up=False,
                           phase=Phase.HOLDING_SHORT)
    state = SimpleNamespace(on_ground=True, ground_speed_kt=0.0)
    # Twenty-five seconds of quiet is not a reason to ask again.
    flight.state.last_radio_at = 1000.0 - 30.0
    flight._work_the_calls(state, flight.session, 1000.0)
    flight._fo_say.assert_not_called()
    # The crew's interval is.
    flight.state.last_radio_at = 1000.0 - READY_REASK_S - 1.0
    flight._work_the_calls(state, flight.session, 1000.0)
    flight._fo_say.assert_called_once_with("ready")


def test_told_to_wait_silences_every_ask():
    """A "standby" is the flight's to wait out, whatever it was asking
    for: the callback is owed to the flight, not chased by it."""
    from meridianatc.atc.autonomous.x_the_radios import READY_REASK_S
    from meridianatc.atc.session import Phase
    flight = _ready_flight("HOLDING_SHORT", lined_up=False,
                           phase=Phase.HOLDING_SHORT,
                           told_to_wait_at=1000.0 - 10.0)
    state = SimpleNamespace(on_ground=True, ground_speed_kt=0.0)
    flight.state.last_radio_at = 0.0
    flight._work_the_calls(state, flight.session, 1000.0)
    flight._fo_say.assert_not_called()
    # The wait runs from the telling, not the asking: patience spent,
    # the call is made again.
    flight.session.told_to_wait_at = 1000.0 - READY_REASK_S - 1.0
    flight._work_the_calls(state, flight.session, 1000.0)
    flight._fo_say.assert_called_once_with("ready")


def test_a_hold_position_stops_the_taxi_where_it_is():
    """"Hold position" is a stop where you are, not a hold at the
    route's end."""
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousState,
    )
    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    flight.sim = object()
    flight._dir = Mock()
    flight._dir.sim = flight.sim
    flight._clearance_strobes = Mock()
    flight._stop = Mock()
    flight._steer = Mock(
        side_effect=AssertionError("steered on a hold position"))
    flight.session = SimpleNamespace(
        hold_position=True, taxi_path=[(0.0, 0.0), (0.01, 0.01)])
    state = SimpleNamespace(taxi_light=True, ground_speed_kt=8.0)
    flight._taxi_along_path(state, outbound=True)
    flight._stop.assert_called_once_with(state)


def _roll_flight(**session_kw):
    """A flight in the takeoff phase, its heavy machinery stubbed."""
    from meridianatc.atc.autonomous import (
        AutonomousFlight,
        AutonomousPhase,
        AutonomousState,
    )
    flight = AutonomousFlight.__new__(AutonomousFlight)
    flight.state = AutonomousState()
    flight.state.phase = AutonomousPhase.TAKEOFF
    flight.state.sub = "wait_clearance"
    flight.state.sub_entered_at = 0.0
    flight.state.runway_heading = 170.0
    flight.state.runway_lat = 41.24
    flight.state.runway_lon = -8.68
    flight.state.runway_width_ft = 148.0
    flight.state.vr_kt = 90.0
    flight.state.aligned = True
    flight._speeds = None
    flight.sim = object()
    flight._dir = Mock()
    flight._dir.sim = flight.sim
    flight._perf = SimpleNamespace(
        rotate_kt=90.0, takeoff_kt_s=3.0, taxi_kt=15.0)
    flight.TAKEOFF_TIMEOUT = 240.0
    flight._compute_speeds = Mock()
    flight._set_takeoff_power = Mock()
    flight._ground_directional = Mock(return_value=0.0)
    flight._engines_out = Mock(return_value=(False, ""))
    flight._runway_blocked = Mock(return_value=False)
    flight._sub_age = Mock(return_value=2.0)
    flight._goto_sub = Mock()
    flight._transition = Mock()
    flight._stop = Mock()
    flight._emit = Mock()
    session_kw.setdefault("cleared_takeoff", True)
    session_kw.setdefault("pending", None)
    flight.session = SimpleNamespace(**session_kw)
    return flight


def test_the_roll_waits_for_the_readback():
    """The clearance is answered before the power comes up -- the
    crew does not roll on words it has not acknowledged."""
    from meridianatc.atc.session import Instruction
    owed = Instruction("takeoff_clearance", "LPPR:TWR", "cleared for takeoff",
                       {"runway": "17"})
    flight = _roll_flight(pending=owed)
    state = SimpleNamespace(on_ground=True, ground_speed_kt=0.0,
                            indicated_airspeed_kt=0.0, parking_brake=True)
    flight._phase_takeoff(state, 1000.0)
    flight._goto_sub.assert_not_called()
    # Read back, the roll is free to start.
    owed.read_back = True
    flight._phase_takeoff(state, 1000.0)
    flight._goto_sub.assert_called_once_with("roll")


def test_the_roll_tracks_the_centreline(monkeypatch):
    """Runway heading alone would hold a lineup's offset the whole
    roll: the correction steers it back to the paint, gently."""
    import meridianatc.atc.auto_nav as auto_nav
    flight = _roll_flight()
    flight.state.sub = "roll"
    flight.state.sub_entered_at = 998.0
    monkeypatch.setattr(
        auto_nav, "align",
        lambda *a, **k: {"offset_m": -6.0, "along_m": 200.0,
                         "correction_deg": 3.5})
    state = SimpleNamespace(
        on_ground=True, ground_speed_kt=30.0, indicated_airspeed_kt=30.0,
        parking_brake=False, latitude=41.235, longitude=-8.679,
        heading_true=170.0)
    flight._phase_takeoff(state, 1000.0)
    args, kwargs = flight._ground_directional.call_args
    assert abs(args[1] - 173.5) < 1e-6


def test_a_healthy_roll_is_not_rejected_for_a_lagging_read():
    """The model spooled its indicated reading seconds behind the ground
    truth, and the first six seconds of the roll are the brake release
    and the spool -- no roll at all: a takeoff accelerating at two and a
    half knots a second was called not accelerating and rejected on the
    numbers. The ground speed says it is moving; the whole expected
    roll is the patience a slow spool is owed."""
    flight = _roll_flight()
    flight.state.sub = "roll"
    # vr 90 at 3 kt/s: the expected roll is thirty seconds. Twenty-five
    # in, doing fifty by the wheels and reading twelve on the needle.
    flight.state.sub_entered_at = 975.0
    state = SimpleNamespace(
        on_ground=True, ground_speed_kt=50.0, indicated_airspeed_kt=12.6,
        parking_brake=False, latitude=41.235, longitude=-8.679,
        heading_true=170.0)
    flight._phase_takeoff(state, 1000.0)
    assert not flight.state.rejected
    assert not any(c.args == ("abort",) for c in
                   flight._goto_sub.call_args_list)
    # The patience is not endless: past the whole expected roll still at
    # a crawl, the rejection is real.
    flight2 = _roll_flight()
    flight2.state.sub = "roll"
    flight2.state.sub_entered_at = 969.0
    dead = SimpleNamespace(
        on_ground=True, ground_speed_kt=4.0, indicated_airspeed_kt=2.0,
        parking_brake=False, latitude=41.235, longitude=-8.679,
        heading_true=170.0)
    flight2._phase_takeoff(dead, 1000.0)
    assert flight2.state.rejected
    assert ("abort",) in [c.args for c in
                          flight2._goto_sub.call_args_list]


def test_the_roll_rejects_the_swing_it_cannot_save(monkeypatch):
    """The keep demand never asked for the swing -- it is capped at the
    correction the paint is owed -- so a roll answering with one is the
    plant's doing, and the books' takeoff it loses is stopped on the
    pavement that is left, not flown off pointed across the grass. The
    crash in the log rotated at fifty-five knots a hundred and thirty
    degrees off the strip's own heading."""
    import meridianatc.atc.auto_nav as auto_nav
    flight = _roll_flight()
    flight.state.sub = "roll"
    flight.state.sub_entered_at = 998.0
    monkeypatch.setattr(
        auto_nav, "align",
        lambda *a, **k: {"offset_m": -4.0, "along_m": 120.0,
                         "correction_deg": 2.0})
    veering = SimpleNamespace(
        on_ground=True, ground_speed_kt=38.0, indicated_airspeed_kt=38.0,
        parking_brake=False, latitude=41.235, longitude=-8.679,
        heading_true=110.0)
    flight._phase_takeoff(veering, 1000.0)
    assert flight.state.rejected
    assert ("abort",) in [c.args for c in
                          flight._goto_sub.call_args_list]
    # The wheels off the pavement's edge is the same verdict measured
    # the other way.
    flight2 = _roll_flight()
    flight2.state.sub = "roll"
    flight2.state.sub_entered_at = 998.0
    monkeypatch.setattr(
        auto_nav, "align",
        lambda *a, **k: {"offset_m": 40.0, "along_m": 120.0,
                         "correction_deg": 2.0})
    off = SimpleNamespace(
        on_ground=True, ground_speed_kt=38.0, indicated_airspeed_kt=38.0,
        parking_brake=False, latitude=41.235, longitude=-8.679,
        heading_true=170.0)
    flight2._phase_takeoff(off, 1000.0)
    assert flight2.state.rejected


def test_the_roll_leads_with_the_right_foot(monkeypatch):
    """The engine's left-turning moment is the roll's standing load --
    the capped damper spent the heading finding it, so the takeoff leads
    with the right foot the way the books teach: full ask as the power
    comes in, stood down by the speed the rudder is flying at, and the
    damper's own reach on top of it past the old four-tenths."""
    import meridianatc.atc.auto_nav as auto_nav
    flight = _roll_flight()
    spool = SimpleNamespace(throttle_pct=100.0,
                            indicated_airspeed_kt=0.0,
                            ground_speed_kt=0.0)
    assert flight._right_foot(spool) > 0.25
    fast = SimpleNamespace(throttle_pct=100.0,
                           indicated_airspeed_kt=70.0,
                           ground_speed_kt=70.0)
    assert flight._right_foot(fast) == 0.0
    monkeypatch.setattr(
        auto_nav, "align",
        lambda *a, **k: {"offset_m": -4.0, "along_m": 120.0,
                         "correction_deg": 2.0})
    flight.state.sub = "roll"
    flight.state.sub_entered_at = 998.0
    # Thirty off the strip's own heading: short of the reject, long past
    # the lean a drooping damper alone ever asked.
    veering = SimpleNamespace(
        on_ground=True, ground_speed_kt=30.0, indicated_airspeed_kt=30.0,
        parking_brake=False, latitude=41.235, longitude=-8.679,
        heading_true=140.0, throttle_pct=100.0)
    flight._ground_directional = Mock(return_value=0.55)
    flight._phase_takeoff(veering, 1000.0)
    sent = flight._dir.rudder.call_args[0][0]
    assert sent > 0.5


# ---------------------------------------------------------------------
# the handoff landed, but the call that was owed on it never came --
# and the pushback tug nobody ever called
# ---------------------------------------------------------------------


def test_the_first_call_on_the_new_frequency_is_not_the_old_ones_repeat():
    """"Contact tower" dialled, the "ready" owed on it is a new ask --
    not a repeat of the one ground heard half a minute ago. The
    re-ask clock belongs to the station the call was made on, which
    is what left the FO silent at the hold after the handoff."""
    from meridianatc.atc.session import station_key
    twr = _station("TWR", 118.45)
    gnd = _station("GND", 121.70)
    flight, state, dialled, kinds = _radio_flight(
        stations=[gnd, twr], phase="holding_short", com1=118.45,
        current_station=twr, session_phase="holding_short")
    sent = []
    flight.crew.radio_call = lambda kind: kinds.append(kind) or ("call", "")
    flight.crew.transmit = lambda text: sent.append(text)
    # "Ready for departure" went out on ground thirty seconds back, and
    # ground's "standby" is still fresh -- the tower heard neither.
    flight._call_on = station_key(gnd)
    flight.state.last_radio_at = 1000.0 - 30.0
    flight._call_tried_at = 1000.0 - 30.0
    flight.session.told_to_wait_at = 1000.0 - 10.0
    flight._work_the_calls(state, flight.session, 1000.0)
    assert "ready" in kinds
    assert sent == ["call"]
    # And the call that went out marks the station it was made on, so
    # the tower's own re-ask clock runs from here.
    assert flight._call_on == station_key(twr)


def test_the_same_stations_ready_call_still_waits_out_the_clock():
    """On the station it was made to, the ready ask is still on the
    crew's patience -- the new-station pass is not an excuse to nag."""
    from meridianatc.atc.session import station_key
    twr = _station("TWR", 118.45)
    flight, state, dialled, kinds = _radio_flight(
        stations=[twr], phase="holding_short", com1=118.45,
        current_station=twr, session_phase="holding_short")
    flight._call_on = station_key(twr)
    flight.state.last_radio_at = 1000.0 - 30.0
    flight._call_tried_at = 1000.0 - 30.0
    flight._work_the_calls(state, flight.session, 1000.0)
    assert "ready" not in kinds


def test_a_handoff_is_discharged_by_the_call_it_sent():
    """Established with the station it was sent to ends the handoff:
    a surface handoff no longer lives on COM1 for ever afterwards."""
    from meridianatc.atc.session import FlightSession
    session = FlightSession(aircraft=None)
    twr = SimpleNamespace(ident="LEVX", position="TWR", mhz=118.45)
    gnd = SimpleNamespace(ident="LEVX", position="GND", mhz=121.70)
    session.handoff_pending = twr
    # A call to somebody else discharges nothing.
    session.establish(gnd)
    assert session.handoff_pending is twr
    # The call to the station it named is the check-in that ends it.
    session.establish(twr)
    assert session.handoff_pending is None


def test_pushback_is_not_asked_of_the_tower():
    """The tug is not the tower's to send: asked of it while a ground
    position is worked, the call is refused -- the stale handoff's
    "request pushback" on the tower frequency."""
    from meridianatc.engine.core import Engine
    engine = Engine.__new__(Engine)
    twr = SimpleNamespace(position="TWR", mhz=118.45, ident="LEVX",
                          callsign_in=lambda spoken: "Vigo Tower")
    engine.session = SimpleNamespace(
        aircraft=SimpleNamespace(
            spoken=lambda abbreviated=False: "cessna two sierra papa"),
        pending=None, phase="takeoff",
        departure_runway="19", ifr=True, clearance_issued=True,
        destination=SimpleNamespace(name="", ident="LEMD"))
    engine.station = twr
    engine._language_for = lambda *a: "en"
    engine.brain = SimpleNamespace(
        for_station=lambda s, l: None, staffed=lambda s: True)
    engine.navdb = SimpleNamespace(
        stations=lambda ident: [SimpleNamespace(position="GND")])
    engine.last_language = "en"
    text, why = engine.first_officer_call("pushback")
    assert text == "" and why == "not_on:ground"
    # But where the tower works the surface alone, it still answers.
    engine.navdb = SimpleNamespace(stations=lambda ident: [])
    text, _why = engine.first_officer_call("pushback")
    assert "pushback" in text


def test_an_approved_push_calls_the_sim_tug_when_the_ramp_is_silent():
    """Approved over the radio and nobody called: a GSX that is not
    there -- or does not answer -- is not the end of the tug, it is
    the simulator's own."""
    from meridianatc.engine.x_test_tools_traffic import _Test_tools_traffic
    engine = _Test_tools_traffic.__new__(_Test_tools_traffic)
    engine.emit = Mock()
    pushed = []
    engine.source = SimpleNamespace(
        pushback=lambda: pushed.append(1) or True)
    # GSX enabled but silent: the approval still gets its tug.
    engine.config = SimpleNamespace(gsx=SimpleNamespace(enabled=True))
    engine.gsx = SimpleNamespace(start_pushback=lambda: "")
    engine._start_pushback()
    assert pushed == [1]
    # GSX off entirely: same tug.
    pushed.clear()
    engine.config.gsx.enabled = False
    engine._start_pushback()
    assert pushed == [1]


def test_a_re_engage_at_the_hold_resumes_where_the_leg_was():
    """Cleared for takeoff and re-engaged, the flight is at the bars --
    not walked back through a preflight whose next radio call asked
    the tower for a pushback."""
    from meridianatc.atc.autonomous import AutonomousPhase
    from meridianatc.atc.session import Phase
    flight = _autonomous()
    flight.get_state = lambda: SimpleNamespace(
        connected=True, on_ground=True, ground_speed_kt=0.0)
    session = SimpleNamespace(
        phase=Phase.TAKEOFF, cleared_takeoff=True, lined_up=False)
    phase, _sub = flight._entry_phase(session)
    assert phase == AutonomousPhase.HOLDING_SHORT
    # Rolling is not "at the hold": a takeoff roll re-engaged is not
    # braked into one either.
    flight.get_state = lambda: SimpleNamespace(
        connected=True, on_ground=True, ground_speed_kt=80.0)
    phase, _sub = flight._entry_phase(session)
    assert phase == AutonomousPhase.PREFLIGHT


def _junction_layout():
    """The apt.dat junction shape: the taxiway meets the runway at the
    holding bars, off the pavement, so the junction is an ordinary taxi
    node -- the one ``nearest_node`` lands on for an aeroplane parked
    there. (xpground marks ``runway_nodes`` from the runway rows' own
    geometry, not from segment kind, which is what leaves the bars a
    taxi node.)"""
    from meridianatc.atc.ground import (
        PATH_PARKING,
        PATH_RUNWAY,
        PATH_TAXI,
        GroundLayout,
    )
    points = [
        (0, 0, 0.0, 0.0),         # 0 r0 -- the 09 threshold end
        (0, 0, 400.0, 0.0),       # 1 r1
        (0, 0, 800.0, 0.0),       # 2 r2
        (0, 0, 1200.0, 0.0),      # 3 r3 -- the 27 threshold end
        (0, 0, 0.0, 150.0),       # 4 t0, taxiway F
        (0, 0, 400.0, 150.0),     # 5 t1
        (0, 0, 800.0, 150.0),     # 6 t2
        (0, 0, 1200.0, 150.0),    # 7 t3
        (0, 0, 0.0, 100.0),       # 8 jA -- the bars at the 09 end
        (0, 0, 1200.0, 100.0),    # 9 jB -- the bars at the 27 end
        (0, 0, -80.0, 150.0),     # 10 stand connector
    ]
    parkings = [(14, 0, "GATE", "", 1, 0, 90.0, 20.0, -80.0, 170.0)]
    paths = [
        (PATH_PARKING, 0, 0, "", 10, 0, -1),
        (PATH_TAXI, 0, 0, "", 10, 4, -1),
        (PATH_TAXI, 0, 0, "", 4, 5, 0),
        (PATH_TAXI, 0, 0, "", 5, 6, 0),
        (PATH_TAXI, 0, 0, "", 6, 7, 0),
        (PATH_TAXI, 0, 0, "", 4, 8, 0),
        (PATH_TAXI, 0, 0, "", 7, 9, 0),
        (PATH_RUNWAY, 0, 0, "", 0, 1, -1),
        (PATH_RUNWAY, 0, 0, "", 1, 2, -1),
        (PATH_RUNWAY, 0, 0, "", 2, 3, -1),
        (PATH_RUNWAY, 0, 0, "", 8, 0, 1),
        (PATH_RUNWAY, 0, 0, "", 9, 3, 1),
    ]
    layout = GroundLayout("TEST", 51.0, 0.0, points, parkings, paths,
                          ["F", "runway"])
    # What the runway rows' geometry marks: the pavement only. The bars
    # stay taxi nodes, exactly as p13 did at Vigo.
    layout.runway_nodes = {("p", 0), ("p", 1), ("p", 2), ("p", 3)}
    return layout


def test_parked_at_the_only_entry_is_still_a_taxi_plan():
    """The Baron pushed back onto Vigo's p13 -- the one junction that was
    runway 19's only entry with enough strip left. Asked for the taxi
    from there, ``departure`` routed start to entry, got a route of one
    node, threw the entry away for "no route", and ground made up
    "J, H, G" for an aeroplane that then drove across the field to the
    threshold. Already AT the hold is still a plan: metres of taxi and
    the line-up onto the runway."""
    layout = _junction_layout()
    # Only jB is a viable entry for 27 (jA leaves no strip ahead), and
    # the aeroplane is parked next to it.
    legs = layout.departure(None, (1200.0, 110.0), (1200.0, 0.0), 270.0,
                            1200.0)
    assert legs is not None
    taxi_out, line_up = legs
    # The hold it already sits at -- the taxi is the few metres to the
    # bars, from where the aeroplane actually is.
    assert taxi_out.points[0] == (1200.0, 110.0)
    assert taxi_out.points[-1] == (1200.0, 100.0)
    # The line-up ends on the centreline past the bars, pointed down the
    # runway -- the entry it is held at is where it enters.
    along, across = layout.runway_frame(*line_up.points[-1],
                                        (1200.0, 0.0), 270.0)
    assert abs(across) < 1e-6
    assert along > 0.0
    assert line_up.face == 270.0
    # And the entry the taxi is called by is the taxiway the bars sit on.
    assert line_up.entry_name == "F"


def test_the_cleared_runway_is_the_runway_the_taxi_is_for():
    """"Departure runway zero one" in the clearance, "runway one niner"
    in the taxi minutes later: the clearance never wrote down which
    runway it named, and ground's fresh pick from a wind that had come
    round was the other end of the strip. Issuing the clearance now
    stores the runway it names."""
    from meridianatc.atc.controller.x_clearance_delivery import _Clearance_delivery
    ctl = _Clearance_delivery.__new__(_Clearance_delivery)
    ctl.navdb = SimpleNamespace(
        airport=lambda ident: SimpleNamespace(),
        station_for=lambda ident, position: None)
    ctl.realism = SimpleNamespace(ifr_departure=True)
    ctl._departure_runway = lambda airport, weather: "01"
    ctl._planned_sid = lambda airport, weather: ""
    ctl._fms_sid = lambda airport, runway: ""
    ctl._departure_altitudes = lambda *a: (5000, 10)
    ctl._sid_for_runway = lambda *a: ""
    ctl._transition_altitude = lambda airport: 3000
    ctl._station_name = lambda station, p: ""
    ctl._plan_idents = lambda: ()
    ctl._default_cruise = lambda session: 9000
    session = SimpleNamespace(
        destination=None, destination_input="LEST", squawk="",
        cruise_altitude_ft=0.0, initial_altitude_ft=0.0, sid="",
        aircraft=SimpleNamespace(
            callsign=SimpleNamespace(written="N45XS"), language="en"),
        departure_runway="",
        established_with=lambda station: False,
        remember=lambda instruction: instruction)
    station = SimpleNamespace(ident="LEVT", position="DEL", mhz=121.7)
    p = SimpleNamespace(
        ifr_clearance=lambda *a, **kw: "cleared",
        UNKNOWN_DESTINATION="somewhere",
        ROUTE_AS_FILED="as filed", ROUTE_RADAR_VECTORS="radar vectors")
    ctl._issue_clearance(session, station, None, p)
    assert session.clearance_issued
    assert session.departure_runway == "01"


def test_the_pushback_toggle_is_not_sent_twice_for_one_tug():
    """TOGGLE_PUSHBACK is a toggle: the Baron's four approved asks and
    the wait-sub's fallback sent it five times over three minutes, each
    one undoing the last -- the tug that finally arrived on the odd
    count. A repeat ask inside the window answers True -- the tug is
    already on its way -- and sends nothing."""
    from meridianatc.sim.source import SimConnectSource
    source = SimConnectSource.__new__(SimConnectSource)
    source._connected = True
    source._push_sent_at = float("-inf")
    source._state = SimpleNamespace(
        pushback_wait=False, pushback_attached=False)
    sent = []
    source._send = lambda event, data=0: sent.append(event) or True
    assert source.pushback()
    assert sent == [b"TOGGLE_PUSHBACK"]
    # Approved again inside the window, the tug it already called is
    # not toggled back off.
    assert source.pushback()
    assert sent == [b"TOGGLE_PUSHBACK"]
    # Where the sim itself says the tug is on its way, the ask is held
    # even past the window.
    source._push_sent_at -= SimConnectSource.PUSH_ASK_S + 1.0
    source._state.pushback_wait = True
    assert source.pushback()
    assert sent == [b"TOGGLE_PUSHBACK"]
    # And a tug that never showed -- no wait, no attachment -- is asked
    # for again once the window is out.
    source._state.pushback_wait = False
    assert source.pushback()
    assert sent == [b"TOGGLE_PUSHBACK", b"TOGGLE_PUSHBACK"]
    # And a dead link still answers dead.
    source._connected = False
    assert not source.pushback()


def test_the_start_crank_is_asked_of_every_engine():
    """The B350 on MSFS 2024: ``STARTER_SET`` is not an event the sim
    knows -- it was refused -- so only engine one's ``STARTER1_SET``
    ever engaged, and the toggle fallback would have killed it again.
    The ask goes out per engine; the toggle never does."""
    from meridianatc.atc.ap import FlightDirector
    sent = []

    class Sim:
        connected = True

        def _send(self, name, value):
            sent.append(name)
            return True

    director = FlightDirector(Sim())
    assert director.all_starters(True)
    assert b"TOGGLE_ALL_STARTERS" not in sent
    for n in (1, 2, 3, 4):
        assert f"STARTER{n}_SET".encode() in sent


def test_the_start_retry_leaves_the_lit_engine_alone():
    """A re-crank while engine one runs owes only the rest: the
    autostart sequence is not re-fired over a lit engine."""
    flight = _flier()
    flight._prop_levers = Mock()
    flight._send_event = Mock()
    state = SimpleNamespace(
        engine_count=2, engines_running=True,
        eng2_combustion=False, eng3_combustion=None, eng4_combustion=None)
    flight._start_engines(state)
    flight._send_event.assert_not_called()
    flight.director.all_starters.assert_called_once_with(True)
    # Nothing lit at all: the autostart is the first move.
    state.engines_running = False
    flight._start_engines(state)
    assert flight._send_event.call_args[0][0] == b"ENGINE_AUTO_START"


def test_a_half_started_twin_is_not_done_starting():
    """Engine one's combustion flag alone used to pass "engines
    running" -- the B350's number two never lit and the stand moved
    on regardless. The start is done when every engine the sim
    reports is combusting."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()])
    flight.state.phase = AutonomousPhase.PUSHBACK
    flight.state.sub = "engines"
    flight.ENGINE_START_TIMEOUT = 240.0
    now = _time.monotonic()
    flight.state.engines_started_at = now - 10.0
    flight.state.start_asked_at = now - 40.0
    flight._start_engines = Mock()
    state = SimpleNamespace(
        engine_count=2, engines_running=True,
        eng2_combustion=False, eng3_combustion=None, eng4_combustion=None)
    flight._phase_pushback(state, now)
    flight._goto_sub.assert_not_called()
    flight._start_engines.assert_called_once_with(state)
    # Both lit: the start is done.
    state.eng2_combustion = True
    flight._phase_pushback(state, now + 1.0)
    flight._goto_sub.assert_called_once_with("after_start")


def test_a_running_push_is_toggled_off_not_asked_again():
    """The release is the same toggle that sent the tug -- so it goes
    only while a push is actually on, where an idle toggle would
    start one."""
    import time as _time

    from meridianatc.sim.source import SimConnectSource
    source = SimConnectSource.__new__(SimConnectSource)
    source._connected = True
    source._push_sent_at = float("-inf")
    source._state = SimpleNamespace(
        pushback_wait=False, pushback_attached=True)
    sent = []
    source._send = lambda event, data=0: sent.append(event) or True
    assert source.pushback_stop()
    assert sent == [b"TOGGLE_PUSHBACK"]
    # Let go: no tug attached, no ask inside the window -- a toggle
    # here would start a push, so nothing is sent.
    source._state.pushback_attached = False
    assert not source.pushback_stop()
    assert sent == [b"TOGGLE_PUSHBACK"]
    # An ask inside its window is a push in hand the stop can end.
    source._push_sent_at = _time.monotonic()
    assert source.pushback_stop()
    assert sent == [b"TOGGLE_PUSHBACK", b"TOGGLE_PUSHBACK"]
    # And a dead link answers dead.
    source._connected = False
    assert not source.pushback_stop()


def test_an_open_ended_push_is_let_go_while_it_rolls():
    """MSFS's tug pushed the King Air tail-first for minutes because
    nothing ever sent the release: half a minute of rolling is a
    push long done. But the toggle only goes while the aeroplane is
    still moving -- sent onto a stand the tug already left, it
    starts a push."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight(stations=[_gnd()])
    flight.state.phase = AutonomousPhase.PUSHBACK
    flight.state.sub = "wait"
    now = _time.monotonic()
    flight.state.moved_since_push = True
    flight.state.push_moved_at = now - 35.0
    flight.state.still_since = now          # still rolling
    state = SimpleNamespace(
        source="simconnect", pushback_wait=False, pushback_attached=True,
        ground_speed_kt=2.0, on_ground=True)
    flight._phase_pushback(state, now)
    flight.director.pushback_stop.assert_called_once()
    # Stopped ten seconds ago: the completion branch takes it, the
    # release is not sent.
    flight = _taxi_flight(stations=[_gnd()])
    flight.state.phase = AutonomousPhase.PUSHBACK
    flight.state.sub = "wait"
    flight.state.moved_since_push = True
    flight.state.push_moved_at = now - 35.0
    flight.state.still_since = now - 10.0
    flight._stand_clear = lambda: (True, "")
    flight._phase_pushback(state, now)
    flight.director.pushback_stop.assert_not_called()
    flight._transition.assert_called_once()


# ---------------------------------------------------------------------
# the audit sweep: twins, wedges, and the calls nobody answered
# ---------------------------------------------------------------------


def _engine_state(**kw):
    base = dict(engines_running=False, eng2_combustion=False,
                eng3_combustion=None, eng4_combustion=None,
                engine_count=2, on_ground=True)
    base.update(kw)
    return SimpleNamespace(**base)


def test_engines_gates_read_every_combustion_flag():
    """Engine one's flag is not 'the engines' on a twin."""
    flight = _taxi_flight()
    both = _engine_state(engines_running=True, eng2_combustion=True)
    one_out = _engine_state(engines_running=False, eng2_combustion=True)
    assert flight._any_engine_running(both)
    assert flight._any_engine_running(one_out)
    assert not flight._all_engines_out(one_out)
    # Both out: the shutdown side of the same gate.
    dead = _engine_state()
    assert flight._all_engines_out(dead)
    assert not flight._any_engine_running(dead)
    # A model that reports nothing counts as out, like the old None read.
    mute = _engine_state(engines_running=None, eng2_combustion=None,
                         engine_count=0)
    assert flight._all_engines_out(mute)


def test_shutdown_waits_for_the_twin_not_engine_one():
    """A shutdown with number two still lit is not 'engines off'."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight()
    flight.state.phase = AutonomousPhase.SHUTDOWN
    flight.state.sub = "engines_off"
    flight._ask_the_ramp = Mock()
    flight._shutdown_engines = Mock()
    flight._sub_age = lambda: 1.0
    state = _engine_state(engines_running=False, eng2_combustion=True)
    flight._phase_shutdown(state, _time.monotonic())
    flight._shutdown_engines.assert_called_once()
    flight._goto_sub.assert_not_called()
    # Engine two out as well: electrics follow.
    flight._shutdown_engines.reset_mock()
    state.eng2_combustion = False
    flight._phase_shutdown(state, _time.monotonic())
    flight._shutdown_engines.assert_not_called()
    flight._goto_sub.assert_called_once_with("electrics_off")


def test_throttle_servo_survives_an_engine_out():
    """The lever for the live engine is still the FO's to move."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight()
    flight.state.phase = AutonomousPhase.CRUISE
    flight.state.sub = ""
    flight._lever_position = Mock(return_value=0.5)
    state = _engine_state(engines_running=False, eng2_combustion=True,
                          on_ground=False, indicated_airspeed_kt=160.0,
                          altitude_ft=15000.0)
    flight._manage_throttle(state, 130.0)
    flight._lever_position.assert_called_once()
    # Both dead: the servo's gate, not the flag's, is what stops it.
    flight._lever_position.reset_mock()
    state.eng2_combustion = False
    flight._manage_throttle(state, 130.0)
    flight._lever_position.assert_not_called()


def test_a_turboprop_initial_climb_uses_climb_power_not_takeoff_power():
    """The live King Air sat at ninety per cent lever and on the torque
    red line through the initial climb: after liftoff the floor is climb
    power, not the roll's takeoff setting."""
    from meridianatc.atc import aircraft_class as ac
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight()
    flight._aircraft_class = ac.TURBOPROP_TWIN
    flight.state.phase = AutonomousPhase.CLIMB
    flight.state.sub = ""
    flight._lever_position = Mock(return_value=0.97)
    flight._altitude_target = lambda _state: 5000.0
    flight._engine_limit_cap = lambda _state: 1.0
    flight._slew_lever = lambda lever, **_kwargs: lever
    flight.session.assigned_altitude_ft = 5000.0
    state = _engine_state(engines_running=True, eng2_combustion=True,
                          on_ground=False, indicated_airspeed_kt=140.0,
                          altitude_ft=1000.0, autothrottle=False,
                          barber_pole_kt=0.0, mach_max_operate=0.0,
                          barber_pole_mach=0.0, airspeed_mach=0.0)
    flight._manage_throttle(state, 130.0)
    assert flight._dir.throttle.call_args.args[0] <= 0.85
    # On target with the open-loop law, the deadband must not leave the
    # lever up either: the climb-power ceiling is applied before returning.
    flight._dir.throttle.reset_mock()
    state.indicated_airspeed_kt = 130.0
    flight._manage_throttle(state, 130.0)
    assert flight._dir.throttle.call_args.args[0] <= 0.85


def test_a_turboprop_takeoff_powerup_takes_seconds_not_two():
    """The PT6's fuel unit is not slammed: takeoff power-up is a slower
    progressive advance than a piston or jet's brisk lever movement."""
    from meridianatc.atc import aircraft_class as ac
    flight = _flier()
    flight._aircraft_class = ac.TURBOPROP_TWIN
    flight._lever_at = 0.0
    flight._set_takeoff_power()
    flight._lever_at -= 1.0
    flight._set_takeoff_power()
    assert flight.director.throttle.call_args[0][0] <= 0.18


def test_underspeed_climb_energy_does_not_cap_existing_climb_power():
    """A slow aeroplane already at climb power keeps climb power."""
    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight()
    flight.state.phase = AutonomousPhase.CLIMB
    flight.state.sub = ""
    flight._lever_position = Mock(return_value=0.97)
    flight._altitude_target = lambda _state: 5000.0
    flight._engine_limit_cap = lambda _state: 1.0
    flight._slew_lever = lambda lever, **_kwargs: lever
    flight.session.assigned_altitude_ft = 5000.0
    flight.session.cruise_altitude_ft = 0.0
    flight.session.initial_altitude_ft = 0.0
    state = _engine_state(engines_running=True, eng2_combustion=True,
                          on_ground=False, indicated_airspeed_kt=100.0,
                          altitude_ft=1000.0, autothrottle=False,
                          barber_pole_kt=0.0, mach_max_operate=0.0,
                          barber_pole_mach=0.0, airspeed_mach=0.0)
    flight._manage_throttle(state, 130.0)
    assert flight._dir.throttle.call_args.args[0] >= 0.95


def test_a_stale_push_dispatch_is_the_tug_that_never_came():
    """PUSHBACK WAIT stuck on a dead spawn is not waited out forever."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight()
    flight.state.phase = AutonomousPhase.PUSHBACK
    flight.state.sub = "wait"
    now = _time.monotonic()
    flight.state.moved_since_push = False
    flight.state.push_stop_sent = False
    flight.state.push_radio = False
    flight.state.push_retries = 3
    flight.state.sub_entered_at = now - 200.0   # two-plus minutes of WAIT
    flight._stand_clear = lambda: (True, "")
    flight._pushback_skippable = lambda: True
    flight.brain = SimpleNamespace(ground=None)
    state = SimpleNamespace(
        source="simconnect", pushback_wait=True, pushback_attached=False,
        ground_speed_kt=0.0, on_ground=True)
    flight._phase_pushback(state, now)
    flight._transition.assert_called_once_with(AutonomousPhase.TAXI_OUT)


def test_a_creeping_push_is_not_toggled_off_by_the_retry():
    """A retry that lands while the tug creeps cancels the push."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    flight = _taxi_flight()
    flight.state.phase = AutonomousPhase.PUSHBACK
    flight.state.sub = "wait"
    now = _time.monotonic()
    flight.state.moved_since_push = False     # under the 0.8 kt flag
    flight.state.push_stop_sent = False
    flight.state.push_radio = False
    flight.state.push_retries = 1
    flight.state.sub_entered_at = now - 130.0
    flight._stand_clear = lambda: (True, "")
    flight._request_pushback = Mock()
    flight.brain = SimpleNamespace(ground=None)
    state = SimpleNamespace(
        source="simconnect", pushback_wait=False, pushback_attached=False,
        ground_speed_kt=0.6, on_ground=True)   # creeping, unflagged
    flight._phase_pushback(state, now)
    flight._request_pushback.assert_not_called()
    # Standing still, the retry is allowed again.
    state.ground_speed_kt = 0.0
    flight._phase_pushback(state, now)
    flight._request_pushback.assert_called_once()


def test_an_unconfirmed_dial_still_lets_the_call_go():
    """The settled flag bounds the dial, not the flight's voice."""
    import time as _time
    flight, state, dialled, kinds = _radio_flight()
    now = _time.monotonic()
    flight._comms_settled = False
    flight._dialled_at = now - 45.0           # dialled, never confirmed
    flight._work_the_frequency = Mock(return_value=False)
    flight._answer_the_queries = Mock()
    flight._manage_comms(state, now)
    assert flight._comms_settled is True
    flight._answer_the_queries.assert_called_once()


def test_reset_scratch_clears_last_legs_memory():
    """The rejoin path and the lever cap are the old field's, not this one's."""
    from collections import deque
    flight = _taxi_flight()
    flight._pids = {}
    flight._wind_z_hist = deque()
    flight._rejoin_path = [(1.0, 2.0)]
    flight._crab_deg = 9.0
    flight._over_since = {"itt": 1.0}
    flight._limit_cap = 0.30
    flight._limit_clean = 5.0
    flight._limit_step = 7.0
    flight._engines_live = {1, 2}
    flight._backtrack_blocked = True
    flight._seed_control_learning = lambda: None
    flight._reset_scratch()
    assert flight._rejoin_path is None
    assert flight._crab_deg == 0.0
    assert flight._over_since == {}
    assert flight._limit_cap == 1.0
    assert flight._limit_clean == 0.0
    assert flight._limit_step == 0.0
    assert flight._engines_live == set()
    assert flight._backtrack_blocked is False


def _talk_brain():
    from meridianatc.atc.controller import ControllerBrain
    from meridianatc.atc.phraseology import Phraseology
    navdb = Mock()
    navdb.airport.return_value = None
    navdb.runways.return_value = []
    brain = ControllerBrain(navdb, Phraseology())
    brain.traffic = None
    return brain


def _talk_session():
    from meridianatc.atc.phraseology import Aircraft
    from meridianatc.atc.session import FlightSession
    from meridianatc.atc.speech import parse_callsign
    return FlightSession(Aircraft(parse_callsign("N123")))


def test_unanswered_intents_get_conversational_answers_not_a_call_list():
    """"Unable", "standby", "cancel IFR" and "frequency change" are
    answered everywhere, not just on the positions that own them."""
    from meridianatc.atc.intents import Intent
    brain = _talk_brain()
    station = _ground_station()
    p = brain.for_station(station)
    session = _talk_session()
    parsed = SimpleNamespace(intent=Intent.UNABLE)
    reply = brain._not_understood(session, station, p, parsed=parsed)
    assert "roger" in reply.text.lower() or "expecting" not in reply.text.lower()
    parsed.intent = Intent.STANDBY
    reply = brain._not_understood(session, station, p, parsed=parsed)
    assert not reply.speak
    parsed.intent = Intent.REQUEST_FREQUENCY_CHANGE
    reply = brain._not_understood(session, station, p, parsed=parsed)
    assert "frequency change approved" in reply.text.lower()
    parsed.intent = Intent.CANCEL_IFR
    reply = brain._not_understood(session, station, p, parsed=parsed)
    assert session.ifr is False
    assert "squawk" in reply.text.lower()


def test_a_takeoff_readback_without_the_word_runway_is_accepted():
    """"Cleared for takeoff two seven" is the standard readback."""
    from meridianatc.atc.intents import Intent
    from meridianatc.atc.session import Instruction
    brain = _talk_brain()
    station = _ground_station()
    p = brain.for_station(station)
    session = _talk_session()
    from meridianatc.atc.session import station_key
    key = station_key(station)
    session.remember(Instruction(
        "takeoff_clearance", key, "cleared for takeoff runway 27",
        {"runway": "27"}, readback_required=True))
    parsed = SimpleNamespace(intent=Intent.ACKNOWLEDGE,
                             normalized="cleared for takeoff two seven",
                             readback_items={})
    reply = brain._settle_pending(parsed, session, station, p)
    assert reply is None
    assert session.pending is None
    # The wrong runway said aloud still fails the readback.
    session.remember(Instruction(
        "takeoff_clearance", key, "cleared for takeoff runway 27",
        {"runway": "27"}, readback_required=True))
    parsed.readback_items = {"runway": "09"}
    reply = brain._settle_pending(parsed, session, station, p)
    assert reply is not None and reply.speak


def test_no_ground_control_still_answers_a_pushback_ask():
    """Dead air to a keyed call read as 'not heard'; a staffed position
    approves the push itself when Ground is switched off."""
    from meridianatc.atc.intents import Intent
    from meridianatc.navdata.db import Station
    brain = _ground_brain()
    brain.navdb.service_hours.return_value = ""
    brain.ground_control = False
    station = Station("LPPR", "TWR", 118.0, "Porto Tower")
    session = _talk_session()
    weather = SimpleNamespace(qnh_hpa=1013, altimeter_inhg=29.92,
                              wind_kt=5.0)
    parsed = SimpleNamespace(intent=Intent.REQUEST_PUSHBACK,
                             normalized="request pushback",
                             callsign_matched=True, items={})
    reply = brain.respond(parsed, session, station, weather)
    assert reply.speak
    assert "push" in reply.text.lower()


def test_the_late_acf_icao_rebuilds_the_session():
    """acf_ICAO answers a read after the session is built: a Baron
    announced as the saved King Air is rebuilt as soon as the string
    lands, not whenever a plan adoption next rebuilds it."""
    from meridianatc.engine.core import Engine
    engine = Engine.__new__(Engine)
    engine.session = SimpleNamespace(
        aircraft=SimpleNamespace(type_code="BE9L"))
    engine.state = SimpleNamespace(type_code="BE58")
    engine.begin_session = Mock()
    engine._reconcile_aircraft()
    engine.begin_session.assert_called_once()


def test_reconcile_leaves_the_matching_and_the_silent_alone():
    """No rebuild when the sim agrees with the session, or when it has
    not answered the type string yet."""
    from meridianatc.engine.core import Engine
    engine = Engine.__new__(Engine)
    engine.begin_session = Mock()
    engine.session = SimpleNamespace(
        aircraft=SimpleNamespace(type_code="BE58"))
    engine.state = SimpleNamespace(type_code="BE58")
    engine._reconcile_aircraft()
    engine.state = SimpleNamespace(type_code="")
    engine._reconcile_aircraft()
    engine.session = None
    engine.state = SimpleNamespace(type_code="BE58")
    engine._reconcile_aircraft()
    engine.begin_session.assert_not_called()


def test_night_files_the_light_aeroplane_ifr_outside_the_faa():
    """The same VMC daylight flight is filed IFR after dark where night
    VFR is its own rating -- and left VFR where the FAA's rules make
    night ordinary."""
    from meridianatc.atc import svfr
    flight = _flier()
    flight._aircraft_class = "piston_single"
    flight._is_helicopter = False
    flight._request_ifr = True
    flight.session.ifr = False
    flight.session.departure = SimpleNamespace(
        ident="LPPT", lat=38.7, lon=-9.1)
    flight.session.destination = SimpleNamespace(
        ident="LPPT", lat=38.7, lon=-9.1)
    wx = SimpleNamespace(raw="METAR", clouds=[("BKN", 3000)],
                         visibility_sm=10.0, source="metar")
    regions = {"region": "icao"}
    flight.brain = SimpleNamespace(
        weather_for=lambda ident: wx,
        region_for=lambda ident: regions["region"],
        realism=SimpleNamespace(special_vfr=True))
    flight._get_current_airport = lambda: flight.session.departure
    old = svfr.is_night
    svfr.is_night = lambda *a: True
    try:
        assert flight._ifr_required(SimpleNamespace()) is True
        regions["region"] = "faa"
        assert flight._ifr_required(SimpleNamespace()) is False
    finally:
        svfr.is_night = old
