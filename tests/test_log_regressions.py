"""Regressions for defects found in the live logs.

- a feed or a model files "@@@@@@@@" for a callsign it never had;
- the transponder code is read as BCO16, so the packed reading wins;
- a clearance ask that could not go out is retried, not waited on;
- ENG TORQUE PERCENT's shipped unit is one the simulator accepts.
"""
from __future__ import annotations

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


def test_clearance_timeout_errors_never_refiles_vfr():
    """A clearance that never comes is a failure, not a VFR departure."""
    flight = _autonomous(stations=[_gnd()])
    now = 100.0
    flight.state.phase_entered_at = now
    flight.state.has_ifr_clearance = True
    flight.state.clearance_asked_at = now     # the ask went out on time
    flight.state.last_radio_at = now
    flight._phase_request_clearance(None, now + 61.0)
    assert flight.session.ifr is True          # never downgraded
    flight._set_error.assert_called_once()
    flight._transition.assert_not_called()


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
    # Past the timeout it is the failure it always was.
    flight.state.last_radio_at = now + 60.0
    flight._phase_request_clearance(None, now + 112.0)
    flight._set_error.assert_called_once()


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
    # Cleared: the session's own phase is the instruction.
    flight.session.phase = "taxi_out"
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


def test_the_rules_are_decided_by_the_aircraft_and_the_weather():
    """A transport is always IFR; a light aeroplane files by the sky."""
    flight = _flier()
    flight._aircraft_class = "airliner"
    flight._is_helicopter = False
    flight._request_ifr = True
    assert flight._ifr_required(SimpleNamespace()) is True
    # A light aeroplane in VMC may genuinely go VFR.
    flight._aircraft_class = "piston_single"
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
    for _ in range(4):
        flight._lever_at -= 1.0
        flight._set_takeoff_power()
    lever = flight.director.throttle.call_args[0][0]
    assert 0.5 < lever < 1.0
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
    """Rollout ends with the call the tower is waiting for."""
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
    flight._fo_say.assert_called_once_with("vacated")
    flight._transition.assert_called_once_with(AutonomousPhase.TAXI_IN)


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
