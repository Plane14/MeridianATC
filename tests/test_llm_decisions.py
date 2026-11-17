"""The advisory layer (meridianatc.llm.decisions): the model may pick among
options the engine enumerated and word replies whose facts are fixed, and
either way the deterministic answer is what stands on a failure.
"""

from __future__ import annotations

from types import SimpleNamespace

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
        assert d.pick("t", "q", self.OPTIONS, default=1) == 1

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
