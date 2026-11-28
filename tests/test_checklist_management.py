"""Checklist runner management: pause clocks and detent-aware execution."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from meridianatc.atc.autonomous.x_the_checklist_answered import (
    _The_checklist_answered,
)
from meridianatc.atc.checklist import ChecklistRunner


def test_a_pause_does_not_age_offers_level_time_or_cached_picks():
    clock = [100.0]
    runner = ChecklistRunner(Mock(), clock=lambda: clock[0])
    runner._offer = ("cruise", 95.0)
    runner._last_offer = 90.0
    runner._talked_at = 98.0
    runner._begun_at = 20.0
    runner._level_since = 40.0
    runner._checklist_pick = (("cruise",), 0, 92.0)

    runner.pause()
    clock[0] = 700.0
    runner.tick(SimpleNamespace())

    assert runner._offer == ("cruise", 695.0)
    assert runner._last_offer == pytest.approx(690.0)
    assert runner._talked_at == pytest.approx(698.0)
    assert runner._begun_at == pytest.approx(620.0)
    assert runner._level_since == pytest.approx(640.0)
    assert runner._checklist_pick == (("cruise",), 0, 692.0)


def test_checklist_flap_fallback_uses_the_aircrafts_detent_count():
    fo = object.__new__(_The_checklist_answered)
    fo.director = Mock()
    fo.director.write.return_value = False
    fo.director.flaps.return_value = True
    fo._flaps_asked = 0.0
    fo._flaps_allowed = lambda state: True
    state = SimpleNamespace(flaps_detents=7)

    assert fo._checklist_flaps(state, 3)
    # Seven detents are indices 0..6, so index 3 is half travel rather
    # than the three-quarters the fixed four-index fallback produced.
    fo.director.flaps.assert_called_with(0.5)
