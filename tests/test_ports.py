"""Tests for the StormATC ports: providers, deepgram, renderer, trainer,
continuous, guidance, surface, history, secrets, endpoints."""

from __future__ import annotations

import numpy as np
import pytest

from meridianatc.atc.continuous import ContinuousFlights, MissionRequest
from meridianatc.atc.guidance import for_session
from meridianatc.atc.session import FlightSession, Phase
from meridianatc.atc.trainer import VoiceTrainer
from meridianatc.history import History
from meridianatc.integrations.deepgram import (
    AVIATION_KEYTERMS,
    DeepgramError,
    DeepgramRecognizer,
    DeepgramTTS,
    build_keyterms,
)
from meridianatc.llm.providers import OpenAICompatibleProvider, build_provider
from meridianatc.llm.rendering import PhrasingRenderer, _facts
from meridianatc.security.endpoints import is_allowed

# --------------------------------------------------------------------------
# endpoints
# --------------------------------------------------------------------------

class TestEndpoints:
    @pytest.mark.parametrize("provider,url", [
        ("openrouter", "https://openrouter.ai/api/v1/chat/completions"),
        ("openai", "https://api.openai.com/v1/chat/completions"),
        ("groq", "https://api.groq.com/openai/v1/chat/completions"),
        ("deepseek", "https://api.deepseek.com/chat/completions"),
        ("deepgram", "https://api.deepgram.com/v1/speak"),
        ("deepgram", "https://api.deepgram.com/v1/listen"),
        ("simbrief", "https://www.simbrief.com/api/xml.fetcher.php"),
    ])
    def test_official_endpoints_pass(self, provider, url):
        assert is_allowed(provider, url)

    @pytest.mark.parametrize("provider,url", [
        ("openrouter", "https://openrouter.ai.evil.com/api/v1/chat"),
        ("openrouter", "http://openrouter.ai/api/v1/chat/completions"),
        ("openrouter", "https://openrouter.ai/api/v1beta/chat"),
        ("openai", "https://openrouter.ai/api/v1/chat/completions"),
        ("deepgram", "https://api.deepgram.com.evil.test/v1/listen"),
        ("openrouter", "https://openrouter.ai:8443/api/v1/x"),
        ("openrouter", "https://user:pw@openrouter.ai/api/v1/x"),
        ("openrouter", "https://openrouter.ai/api/v1/x?key=k"),
        ("unknown", "https://anything.test/"),
        ("openrouter", ""),
    ])
    def test_everything_else_refused(self, provider, url):
        assert not is_allowed(provider, url)


# --------------------------------------------------------------------------
# providers
# --------------------------------------------------------------------------

class TestProviders:
    def test_none_is_none(self):
        assert build_provider("none") is None
        assert build_provider("") is None
        assert build_provider("off") is None

    def test_ollama(self):
        p = build_provider("ollama")
        assert p.name == "ollama"

    @pytest.mark.parametrize("kind", ["openrouter", "openai", "groq",
                                      "deepseek"])
    def test_openai_compatible_kinds(self, kind):
        p = build_provider(kind, api_key="key")
        assert isinstance(p, OpenAICompatibleProvider)
        assert p.name == kind
        assert p.available()

    def test_no_key_not_available(self):
        assert not build_provider("openrouter", api_key="").available()

    def test_custom_requires_allowlisted_host(self):
        with pytest.raises(ValueError):
            build_provider("custom", api_key="k", host="https://evil.test")

    def test_unknown_provider(self):
        with pytest.raises(ValueError):
            build_provider("watson")


# --------------------------------------------------------------------------
# deepgram
# --------------------------------------------------------------------------

class TestKeyterms:
    def test_aviation_terms_first(self):
        terms = build_keyterms()
        assert terms[:3] == list(AVIATION_KEYTERMS[:3])

    def test_dedup_and_order(self):
        terms = build_keyterms(["pushback", "x-plane-12"], "N123AB",
                               ["Ground", "Pushback"])
        lowered = [t.lower() for t in terms]
        assert len(lowered) == len(set(lowered))
        assert "x-plane-12" in terms and "N123AB" in terms

    def test_caps(self):
        terms = build_keyterms([f"term{i}" for i in range(200)])
        assert len(terms) <= 100

    def test_token_cap(self):
        terms = build_keyterms(["one two three four five six seven"] * 200)
        total = sum(len(t.split()) for t in terms)
        assert total <= 500


class TestDeepgramRecognizer:
    def _recognizer(self):
        return DeepgramRecognizer(api_key="k", languages=["en", "fr"])

    def test_loaded_and_multilingual(self):
        r = self._recognizer()
        assert r.loaded and r.multilingual

    def test_set_languages(self):
        r = self._recognizer()
        assert r.set_languages(["es"])
        assert not r.multilingual
        assert not r.set_languages(["es"])

    def test_set_context(self):
        r = self._recognizer()
        r.set_context("BAW123", ["Ground", "Tower", "Ground"])
        assert r.callsign == "BAW123"
        assert r.stations == ["Ground", "Tower"]

    def test_blocked_endpoint(self):
        r = DeepgramRecognizer(api_key="k", base_url="https://evil.test")
        with pytest.raises(DeepgramError):
            r.transcribe(np.ones(16000, dtype=np.float32), 16000)


class TestDeepgramTTS:
    def test_not_available_without_key(self):
        assert not DeepgramTTS().available()

    def test_available_with_key(self):
        assert DeepgramTTS(api_key="k").available()

    def test_blocked_url(self):
        assert not DeepgramTTS(api_key="k",
                               base_url="https://evil.test/v1/speak").available()


# --------------------------------------------------------------------------
# renderer
# --------------------------------------------------------------------------

class _FakeProvider:
    name = "fake"

    def __init__(self, reply: str):
        self.reply = reply
        self.seen = []

    def classify(self, prompt, system):
        self.seen.append(prompt)
        return self.reply


class TestRenderer:
    def test_rephrase_used_when_facts_match(self):
        approved = "BAW123, descend FL180, contact 118.310, squawk 4471"
        render = "BAW123, descend to FL180 and squawk 4471, contact 118.310"
        r = PhrasingRenderer(_FakeProvider(render))
        assert r.render(approved) == render

    def test_missing_fact_rejected(self):
        approved = "BAW123, descend FL180, contact 118.310, squawk 4471"
        r = PhrasingRenderer(_FakeProvider("BAW123, descend FL180, squawk 4471"))
        assert r.render(approved) == approved

    def test_invented_fact_rejected(self):
        approved = "BAW123, cleared takeoff runway 31"
        r = PhrasingRenderer(_FakeProvider("BAW123, cleared takeoff runway 32"))
        assert r.render(approved) == approved

    def test_failure_falls_back(self):
        class Broken:
            name = "broken"
            def classify(self, *a):
                raise RuntimeError("down")
        assert PhrasingRenderer(Broken()).render("abc") == "abc"

    def test_none_provider_passthrough(self):
        assert PhrasingRenderer(None).render("abc") == "abc"

    def test_facts_extraction(self):
        assert _facts("BAW123 descend FL180 contact 118.310") == \
            ["BAW123", "FL180", "118.310"]


# --------------------------------------------------------------------------
# trainer
# --------------------------------------------------------------------------

class TestTrainer:
    def test_keyterms_round_trip(self, tmp_path):
        t = VoiceTrainer(tmp_path)
        t.add_keyterm("N123AB")
        t.add_keyterm("n123ab")  # dup case-insensitive
        t.add_keyterm("  spaced   out  ")
        assert t.keyterms == ["N123AB", "spaced out"]
        t.remove_keyterm("n123AB")
        assert t.keyterms == ["spaced out"]
        # Persisted across instances
        assert VoiceTrainer(tmp_path).keyterms == ["spaced out"]

    def test_corrections_apply(self, tmp_path):
        t = VoiceTrainer(tmp_path)
        t.correct("stay again", "say again")
        t.correct("foxtrot", "foxtrot")
        assert t.apply("stay again please") == "say again please"
        # word-boundary: "staying" is not "stay again"
        assert t.apply("staying on course") == "staying on course"

    def test_empty_ignored(self, tmp_path):
        t = VoiceTrainer(tmp_path)
        t.add_keyterm("   ")
        t.correct("", "x")
        assert t.keyterms == [] and t.corrections == {}


# --------------------------------------------------------------------------
# continuous flights
# --------------------------------------------------------------------------

class _Airport:
    def __init__(self, ident, lat, lon):
        self.ident, self.lat, self.lon = ident, lat, lon


class _DB:
    def nearest_airports(self, lat, lon, limit=5, max_nm=60,
                         min_rank=0, require_runway=False):
        return [(_Airport("KBBB", lat + 0.5, lon), 30.0),
                (_Airport("KCCC", lat + 1.0, lon), 60.0),
                (_Airport("KAAA", lat, lon), 0.0)]


class TestContinuous:
    def test_validation(self):
        assert MissionRequest(min_nm=0).validate()
        assert MissionRequest(min_nm=100, max_nm=50).validate()
        assert MissionRequest(service_minutes=1).validate()
        assert MissionRequest(reserve_factor=0.5).validate()
        assert not MissionRequest().validate()

    def test_not_running_plans_nothing(self):
        cf = ContinuousFlights(_DB())
        assert cf.plan_next_leg(40.0, -74.0, "KAAA", "N123") is None

    def test_leg_planned_inside_band(self):
        cf = ContinuousFlights(_DB(), seed=1)
        cf.start(MissionRequest(min_nm=10, max_nm=120))
        leg = cf.plan_next_leg(40.0, -74.0, "KAAA", "N123")
        assert leg is not None and leg.number == 1
        assert leg.departure == "KAAA"
        assert leg.destination in ("KBBB", "KCCC")

    def test_reserve_shrinks_band(self):
        cf = ContinuousFlights(_DB(), seed=1)
        # reserve 2x on a 120 max -> 60 usable; KCCC at 60 nm is the edge,
        # KBBB at 30 qualifies, KAAA excluded as current.
        cf.start(MissionRequest(min_nm=10, max_nm=120, reserve_factor=2.0))
        leg = cf.plan_next_leg(40.0, -74.0, "KAAA", "N123")
        assert leg.destination == "KBBB"

    def test_stop(self):
        cf = ContinuousFlights(_DB())
        cf.start(MissionRequest())
        assert cf.running
        cf.stop()
        assert not cf.running


# --------------------------------------------------------------------------
# guidance
# --------------------------------------------------------------------------

class TestGuidance:
    def _session(self, phase):
        s = FlightSession.__new__(FlightSession)
        s.phase = phase
        s.flight_rules = "ifr"
        return s

    def test_no_session(self):
        g = for_session(None, plan_known=False, spoken_callsign="X")
        assert "simulator" in g.title.lower()

    def test_no_plan(self):
        g = for_session(self._session(Phase.PREFLIGHT), plan_known=False,
                        spoken_callsign="X")
        assert "flight plan" in g.detail.lower()

    def test_preflight_ifr(self):
        g = for_session(self._session(Phase.PREFLIGHT), plan_known=True,
                        spoken_callsign="BAW123")
        assert "IFR" in g.title and "BAW123" in g.example

    def test_preflight_vfr(self):
        s = self._session(Phase.PREFLIGHT)
        s.flight_rules = "vfr"
        g = for_session(s, plan_known=True, spoken_callsign="BAW123")
        assert "VFR" in g.title

    def test_taxi_out(self):
        g = for_session(self._session(Phase.TAXI_OUT), plan_known=True,
                        spoken_callsign="BAW123")
        assert "holding short" in g.example

    def test_readback_due(self):
        g = for_session(self._session(Phase.CLEARED), plan_known=True,
                        spoken_callsign="BAW123",
                        readback_due=["runway 31", "squawk 4471"])
        assert "runway 31" in g.detail


# --------------------------------------------------------------------------
# history
# --------------------------------------------------------------------------

class TestHistory:
    def test_round_trip(self, tmp_path):
        h = History(tmp_path / "h.db")
        sid = h.open_session("KJFK", "KBOS", "BAW123", "A320", "ifr")
        h.record("pilot", "request clearance", station="DEL")
        h.record("controller", "cleared to KBOS", station="DEL",
                 intent="clearance")
        h.close_session()
        sessions = h.sessions()
        assert len(sessions) == 1 and sessions[0].callsign == "BAW123"
        lines = h.transcript(sid)
        assert [t.who for t in lines] == ["pilot", "controller"]
        assert lines[1].intent == "clearance"
        h.close()

    def test_no_open_session_no_write(self, tmp_path):
        h = History(tmp_path / "h.db")
        h.record("pilot", "hello")
        assert h.transcript(1) == []
        h.close()

    def test_schema_is_wal(self, tmp_path):
        h = History(tmp_path / "h.db")
        mode = h._conn.execute("PRAGMA journal_mode").fetchone()[0]
        assert mode == "wal"
        h.close()


# --------------------------------------------------------------------------
# surface advisor
# --------------------------------------------------------------------------

class _Layout:
    latitude = 40.0
    longitude = -74.0
    _cos = 0.766

    class _Stand:
        def __init__(self, x, z, number):
            self.x, self.z, self.number = x, z, number
            self.gate = True
            self.kind = "ramp"
            self.heading = 90.0
            self.radius = 20

    def __init__(self):
        self.xy = {("p", 0): (0.0, 0.0), ("p", 1): (100.0, 0.0),
                   ("p", 2): (100.0, 100.0)}
        self.edges = {
            ("p", 0): [(("p", 1), 100.0, False, "B")],
            ("p", 1): [(("p", 0), 100.0, False, "B"),
                       (("p", 2), 100.0, False, "A")],
            ("p", 2): [(("p", 1), 100.0, False, "A")],
        }
        self.stands = [self._Stand(200.0, 0.0, "14")]


class TestSurface:
    def test_on_stand(self):
        from meridianatc.atc.surface import locate
        layout = _Layout()
        lat = layout.latitude
        lon = layout.longitude + 200.0 / (111194.9 * 0.766)
        pos = locate(layout, lat, lon)
        assert pos.found and pos.stand == "14"

    def test_on_taxiway(self):
        from meridianatc.atc.surface import locate
        layout = _Layout()
        lat = layout.latitude
        lon = layout.longitude + 50.0 / (111194.9 * 0.766)
        pos = locate(layout, lat, lon)
        assert pos.found and pos.taxiway == "B" and pos.along == 0.5

    def test_nowhere(self):
        from meridianatc.atc.surface import locate
        pos = locate(_Layout(), 41.0, -75.0)
        assert not pos.found

    def test_none_layout(self):
        from meridianatc.atc.surface import locate, progress
        assert not locate(None, 0, 0).found
        assert progress(None, 0, 0, ["B"]) == {}
