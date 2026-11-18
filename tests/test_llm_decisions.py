"""The advisory layer (meridianatc.llm.decisions): the model may pick among
options the engine enumerated and word replies whose facts are fixed, and
either way the deterministic answer is what stands on a failure.
"""

from __future__ import annotations

import logging
from types import SimpleNamespace
from unittest.mock import Mock

from meridianatc.atc.callouts import Call
from meridianatc.atc.crew import CrewDirector
from meridianatc.atc.notices import Notices
from meridianatc.atc.session import Phase
from meridianatc.llm.decisions import Decisions, _parse_choice


def _provider(raw, *, available=True):
    return SimpleNamespace(
        complete=lambda *a, **k: raw,
        classify=lambda *a, **k: raw,
        available=lambda: available,
        name="stub",
    )


def _decisions(raw, **kw):
    return Decisions(_provider(raw), **kw)


class TestPick:
    OPTIONS = ["ABC123: 2 o'clock, 3.0 miles", "XYZ9: 10 o'clock, 5.0 miles"]

    def test_model_pick_is_returned(self):
        d = _decisions('{"choice": 2}')
        assert d.pick("t", "q", self.OPTIONS) == 1
        assert d.stats.accepted == 1

    def test_pick_inside_other_json_is_found(self):
        d = _decisions('thinking... {"choice": 1} done')
        assert d.pick("t", "q", self.OPTIONS) == 0

    def test_out_of_range_pick_is_the_default(self):
        d = _decisions('{"choice": 9}')
        assert d.pick("t", "q", self.OPTIONS, default=0) == 0
        assert d.stats.rejected == 1

    def test_zero_is_not_an_option_number(self):
        d = _decisions('{"choice": 0}')
        assert d.pick("t", "q", self.OPTIONS, allow_none=False,
                      default=1) == 1

    def test_zero_means_none_when_the_prompt_advertises_it(self):
        # The prompt numbers "none" as option 0; a model that answers 0
        # has declined, not picked the default.
        d = _decisions('{"choice": 0}')
        assert d.pick("t", "q", self.OPTIONS, allow_none=True) == -1

    def test_garbage_is_the_default(self):
        d = _decisions("the second one is closest I think")
        assert d.pick("t", "q", self.OPTIONS, default=0) == 0

    def test_none_declines_when_allowed(self):
        d = _decisions('{"choice": "none"}')
        assert d.pick("t", "q", self.OPTIONS, allow_none=True) == -1

    def test_none_rejected_when_not_allowed(self):
        d = _decisions('{"choice": "none"}')
        assert d.pick("t", "q", self.OPTIONS, allow_none=False,
                      default=1) == 1

    def test_no_options_short_circuits(self):
        d = _decisions('{"choice": 1}')
        assert d.pick("t", "q", [], default=7) == 7
        assert d.stats.calls == 0

    def test_unavailable_model_gives_default(self):
        d = Decisions(_provider('{"choice": 2}', available=False))
        assert d.pick("t", "q", self.OPTIONS, default=0) == 0
        assert d.stats.calls == 0

    def test_disabled_gives_default(self):
        d = _decisions('{"choice": 2}', enabled=False)
        assert d.pick("t", "q", self.OPTIONS, default=0) == 0

    def test_failure_backs_off(self):
        provider = SimpleNamespace(
            complete=lambda *a, **k: (_ for _ in ()).throw(ValueError("x")),
            classify=lambda *a, **k: "",
            available=lambda: True,
        )
        d = Decisions(provider)
        assert d.pick("t", "q", self.OPTIONS, default=0) == 0
        assert d.stats.failures == 1
        # Second call is inside the backoff window: no provider call at all.
        assert d.pick("t", "q", self.OPTIONS, default=0) == 0
        assert d.stats.calls == 1

    def test_shadow_logs_but_keeps_default(self):
        d = _decisions('{"choice": 2}', shadow=True)
        assert d.pick("t", "q", self.OPTIONS, default=0) == 0
        assert d.stats.accepted == 1 and d.stats.shadowed == 1

    def test_closed_gives_default(self):
        d = _decisions('{"choice": 2}')
        d.close()
        assert d.pick("t", "q", self.OPTIONS, default=0) == 0
        assert d.stats.calls == 0


class TestParseChoice:
    def test_number(self):
        assert _parse_choice('{"choice": 3}') == 3

    def test_string_number(self):
        assert _parse_choice('{"choice": "2"}') == 2

    def test_none_word(self):
        assert _parse_choice('{"choice": "none"}') == "none"

    def test_bool_is_not_a_choice(self):
        assert _parse_choice('{"choice": true}') is None

    def test_no_object(self):
        assert _parse_choice("two") is None

    def test_not_json(self):
        assert _parse_choice("{choice: two}") is None


class TestDraft:
    BASE = "G-BX, DAL45: on downwind runway 27L"
    KEPT = "DAL45 here, G-BX: on downwind for runway 27L"

    def test_clean_draft_is_kept(self):
        d = _decisions(self.KEPT)
        got = d.draft("traffic_pilot", "where are you?",
                      "DAL45 runway 27L", self.BASE)
        assert got == self.KEPT
        assert d.stats.accepted == 1

    def test_invented_fact_rejects(self):
        d = _decisions("DAL45: on final runway 09R")
        assert d.draft("t", "q", "DAL45 runway 27L", self.BASE) == self.BASE
        assert d.stats.rejected == 1

    def test_missing_fact_rejects(self):
        d = _decisions("DAL45: hi there")
        assert d.draft("t", "q", "DAL45 runway 27L", self.BASE) == self.BASE

    def test_added_instruction_rejects(self):
        # The facts all survive -- but a clearance has been spoken into
        # being, which the fact check alone cannot see.
        d = _decisions("DAL45 G-BX, cleared to land runway 27L")
        assert d.draft("t", "q", "DAL45 runway 27L", self.BASE) == self.BASE

    def test_swapped_leg_rejects(self):
        d = _decisions("DAL45 here G-BX, on final for runway 27L")
        assert d.draft("t", "q", "DAL45 runway 27L", self.BASE) == self.BASE

    def test_too_long_rejects(self):
        d = _decisions("x" * 500)
        assert d.draft("t", "q", "", "hello") == "hello"

    def test_empty_baseline_never_calls(self):
        d = _decisions("anything")
        assert d.draft("t", "q", "facts", "") == ""
        assert d.stats.calls == 0

    def test_unavailable_keeps_baseline(self):
        d = Decisions(_provider("DAL45", available=False))
        assert d.draft("t", "q", "facts", self.BASE) == self.BASE
        assert d.stats.calls == 0

    def test_shadow_logs_but_keeps_baseline(self):
        d = _decisions(self.KEPT, shadow=True)
        got = d.draft("t", "q", "G-BX DAL45 runway 27L", self.BASE)
        assert got == self.BASE
        assert d.stats.shadowed == 1

    def test_quoted_output_is_unwrapped(self):
        d = _decisions(f'"{self.KEPT}"')
        got = d.draft("t", "q", "G-BX DAL45 runway 27L", self.BASE)
        assert got == self.KEPT


# ----------------------------------------------------------------------
# TrafficWatch with an advisor
# ----------------------------------------------------------------------

class _Flight:
    def __init__(self, written, lat):
        self.written = written
        self.on_ground = False
        self.latitude = lat
        self.longitude = 0.0
        self.altitude_ft = 3000.0
        self.heading = 90.0
        self.ground_speed_kt = 120.0
        self.observed = "invented"
        self.indefinite = lambda language: f"a {written}"


def _session():
    aircraft = SimpleNamespace(language="en")
    return SimpleNamespace(
        ifr=False,
        flight_following=True,
        traffic_called={},
        aircraft=aircraft,
        established_with=lambda station: True,
        remember=lambda instruction: instruction,
    )


def _state():
    return SimpleNamespace(
        has_position=True,
        on_ground=False,
        latitude=0.0,
        longitude=0.0,
        altitude_ft=2500.0,
        ground_speed_kt=110.0,
        track_true=90.0,
        heading_true=90.0,
    )


def _phraseology():
    return SimpleNamespace(
        language="en",
        traffic_call=lambda aircraft, **kw: (
            f"traffic {kw.get('oclock')} o'clock"),
        traffic_no_factor=lambda aircraft: "traffic no factor",
    )


def _watch(advisor=None):
    from meridianatc.atc.trafficinfo import TrafficWatch

    # Both ahead of the ownship's 090 track, inside two miles so they stay
    # factors: ABC about a mile, DEF under two.
    brain = SimpleNamespace(
        contacts=lambda: [_Flight("ABC", 0.015), _Flight("DEF", 0.03)],
        staffed=lambda station: True,
        for_station=lambda station, language: _phraseology(),
    )
    watch = TrafficWatch(brain, advisor=advisor)
    station = SimpleNamespace(position="APP", ident="XAPP", mhz=120.500)
    return watch, station


class TestBoundedAsk:
    def test_wait_s_bounds_the_call(self):
        import threading
        import time

        done = threading.Event()

        def slow(*a, **k):
            done.wait(5)
            return '{"choice": 1}'

        provider = SimpleNamespace(complete=slow,
                                   available=lambda: True)
        d = Decisions(provider)
        started = time.monotonic()
        got = d.pick("t", "q", ["one", "two"], default=0, wait_s=0.2)
        assert got == 0
        assert time.monotonic() - started < 2.0
        done.set()

    def test_wait_s_passes_a_fast_answer_through(self):
        d = _decisions('{"choice": 2}')
        assert d.pick("t", "q", ["one", "two"], wait_s=5.0) == 1


class TestTrafficWatchAdvisor:
    def test_closest_picked_without_advisor(self):
        watch, station = _watch()
        reply = watch.look(_session(), station, _state())
        assert reply is not None

    def test_advisor_pick_changes_the_contact(self):
        advisor = SimpleNamespace(
            pick=lambda scope, q, options, **kw: 1)
        watch, station = _watch(advisor)
        reply = watch.look(_session(), station, _state())
        assert reply is not None

    def test_advisor_declining_suppresses_the_call(self):
        advisor = SimpleNamespace(
            pick=lambda scope, q, options, **kw: -1)
        watch, station = _watch(advisor)
        assert watch.look(_session(), station, _state()) is None

    def test_pick_is_asked_once_per_contact_set(self):
        calls = []
        advisor = SimpleNamespace(
            pick=lambda scope, q, options, **kw: calls.append(options) or -1)
        watch, station = _watch(advisor)
        session = _session()
        # Clear the called flags between looks so both passes are eligible.
        watch.look(session, station, _state())
        session.traffic_called.clear()
        watch.look(session, station, _state())
        assert len(calls) == 1

    def test_cached_pick_survives_a_reordered_contact_list(self):
        # The same contacts arriving in a different order must not move the
        # cached index onto a different aeroplane.
        picks = iter((1,))
        advisor = SimpleNamespace(
            pick=lambda scope, q, options, **kw: next(picks, 1))
        watch, station = _watch(advisor)
        session = _session()
        first = watch.look(session, station, _state())
        session.traffic_called.clear()
        # Reverse the contact order: the signature is the same set.
        flights = watch.brain.contacts()
        watch.brain.contacts = lambda: list(reversed(flights))
        second = watch.look(session, station, _state())
        assert first is not None and second is not None
        assert "ABC" not in session.traffic_called
        assert session.traffic_called == {"DEF": "called"}


# ----------------------------------------------------------------------
# Per-scope telemetry
# ----------------------------------------------------------------------

class TestTelemetry:
    def test_scopes_count_independently(self):
        d = _decisions('{"choice": 2}')
        d.pick("traffic_pick", "q", ["a", "b"])
        d.pick("fo_notice", "q", ["a", "b"])
        assert d.scopes["traffic_pick"]["asked"] == 1
        assert d.scopes["traffic_pick"]["proposed"] == 1
        assert d.scopes["fo_notice"]["asked"] == 1
        assert d.scopes["traffic_pick"]["failed"] == 0

    def test_declines_rejects_and_failures_count(self):
        d = _decisions('{"choice": "none"}')
        d.pick("t", "q", ["a"], allow_none=True)
        assert d.scopes["t"]["declined"] == 1
        assert d.scopes["t"]["proposed"] == 1

        d = _decisions('{"choice": 9}')
        d.pick("t", "q", ["a"], default=0)
        assert d.scopes["t"]["rejected"] == 1

        provider = SimpleNamespace(
            complete=lambda *a, **k: (_ for _ in ()).throw(ValueError("x")),
            available=lambda: True)
        d = Decisions(provider)
        d.pick("t", "q", ["a"], default=0)
        assert d.scopes["t"]["failed"] == 1

    def test_close_writes_the_summary(self, caplog):
        d = _decisions('{"choice": 1}')
        d.pick("traffic_pick", "q", ["a"])
        with caplog.at_level(logging.INFO):
            d.close()
        assert "fo" not in d.scopes
        assert "traffic_pick" in caplog.text

    def test_close_with_nothing_asked_is_quiet(self):
        d = _decisions('{"choice": 1}')
        d.close()
        assert d.scopes == {}


# ----------------------------------------------------------------------
# The first officer choosing which notice to surface
# ----------------------------------------------------------------------


def _notice_crew(advisor):
    crew = CrewDirector.__new__(CrewDirector)
    crew.advisor = advisor
    crew.notices = Notices()
    return crew


_notice_session = SimpleNamespace(phase=Phase.ENROUTE)
_notice_state = SimpleNamespace(altitude_ft=2500.0, on_ground=False)


def _notice_calls():
    return [Call("gear_up_final", {}, "gear up at 400 ft"),
            Call("altimeter_not_set", {}, "set 1013, given 1021"),
            Call("wrong_tank", {}, "0.5 gal against 3.0")]


class TestAdvisedNotices:
    def test_no_advisor_says_everything(self):
        crew = _notice_crew(None)
        assert crew._advised_notices(_notice_calls(), _notice_session,
                                     _notice_state) == _notice_calls()

    def test_pick_chooses_one_and_defers_the_rest(self):
        crew = _notice_crew(SimpleNamespace(
            pick=lambda *a, **k: 1))  # second option: wrong_tank
        crew.notices = Mock(wraps=Notices())
        calls = _notice_calls()
        out = crew._advised_notices(calls, _notice_session, _notice_state)
        assert [c.cue for c in out] == ["gear_up_final", "wrong_tank"]
        crew.notices.defer.assert_called_once_with("altimeter_not_set")

    def test_declining_leaves_only_the_safety_calls(self):
        crew = _notice_crew(SimpleNamespace(pick=lambda *a, **k: -1))
        crew.notices = Mock(wraps=Notices())
        out = crew._advised_notices(_notice_calls(), _notice_session,
                                    _notice_state)
        assert [c.cue for c in out] == ["gear_up_final"]
        assert crew.notices.defer.call_count == 2

    def test_an_absent_answer_says_everything(self):
        # Model unavailable, slow, or shadowing: unchanged behavior.
        crew = _notice_crew(SimpleNamespace(pick=lambda *a, **k: None))
        out = crew._advised_notices(_notice_calls(), _notice_session,
                                    _notice_state)
        assert [c.cue for c in out] == [c.cue for c in _notice_calls()]

    def test_all_safety_means_the_model_is_not_asked(self):
        asked = []
        advisor = SimpleNamespace(
            pick=lambda *a, **k: asked.append(a) or 0)
        crew = _notice_crew(advisor)
        calls = [Call("gear_up_final", {}, ""),
                 Call("fuel_reserve", {}, "")]
        out = crew._advised_notices(calls, _notice_session, _notice_state)
        assert out == calls and asked == []

    def test_deferred_notice_stays_due(self):
        notices = Notices()
        first = notices._once("k1", "altimeter_not_set")
        assert first and notices._once("k1", "altimeter_not_set") == []
        notices.defer("altimeter_not_set")
        assert notices._once("k1", "altimeter_not_set")
