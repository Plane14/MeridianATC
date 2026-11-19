"""The airframe learning: trims, scales and polarities, kept per model."""

from __future__ import annotations

import pytest

from meridianatc.atc import adaptation
from meridianatc.atc.adaptation import Adaptation, AdaptivePid, for_aircraft
from meridianatc.atc.control import Pid, PidTerms


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def step(self, dt):
        self.t += dt


@pytest.fixture(autouse=True)
def clean_store():
    adaptation.reset()
    yield
    adaptation.reset()


def test_trim_learned_then_preloaded():
    # What a loop converges to is written down on reset, and the next
    # loop on the same channel opens there rather than at zero.
    adapt = Adaptation()
    pid = AdaptivePid("throttle", adapt,
                      PidTerms(kp=0.1, ki=0.5, low=0.0, high=1.0),
                      clock=Clock())
    pid.reset(output=0.4)
    for _ in range(40):
        pid.update(0.0, dt=0.25)
    pid.reset()                      # converged output is learned
    assert adapt.trim("throttle") > 0.0
    second = AdaptivePid("throttle", adapt,
                         PidTerms(kp=0.1, ki=0.5, low=0.0, high=1.0),
                         clock=Clock())
    # The learned trim preloads the integrator, not the output: a
    # caller's bumpless transfer still wins, and the first update opens
    # near the trim rather than at zero.
    assert second.output is None
    assert second.update(0.0, dt=0.25) == pytest.approx(
        adapt.trim("throttle"), abs=0.05)


def test_gain_grows_when_saturated_and_unanswered():
    adapt = Adaptation()
    pid = AdaptivePid("collective", adapt,
                      PidTerms(kp=1.0, deadband=1.0, low=0.0, high=1.0),
                      clock=Clock())
    # Pinned on the clamp with a big error for longer than the wait.
    for _ in range(40):
        pid.update(50.0, dt=0.25)
    assert adapt.scale("collective") > 1.0
    assert pid.terms.kp == pytest.approx(1.0 * adapt.scale("collective"))


def test_gain_shrinks_when_the_loop_chases_itself():
    adapt = Adaptation()
    pid = AdaptivePid("throttle", adapt,
                      PidTerms(kp=0.1, deadband=0.5, low=-1.0, high=1.0),
                      clock=Clock())
    # An error that flips sign under a working lever, four times a second,
    # without touching the clamps: the loop is chasing itself.
    for i in range(80):
        pid.update(8.0 if i % 2 else -8.0, dt=0.25)
    assert adapt.scale("throttle") < 1.0


def test_disabled_store_is_a_plain_pid():
    adapt = Adaptation(enabled=False)
    adapt.learn_trim("throttle", 0.9)
    adapt.learn_scale("throttle", 2.0)
    assert adapt.trim("throttle") == 0.0
    assert adapt.scale("throttle") == 1.0
    pid = AdaptivePid("throttle", adapt,
                      PidTerms(kp=0.5, ki=0.4, low=0.0, high=1.0),
                      clock=Clock())
    plain = Pid(PidTerms(kp=0.5, ki=0.4, low=0.0, high=1.0), clock=Clock())
    assert pid.update(10.0, dt=0.5) == plain.update(10.0, dt=0.5)


def test_sign_votes_are_kept():
    adapt = Adaptation()
    assert adapt.sign("steer") == 0
    for _ in range(5):
        adapt.vote("steer", -1)
    assert adapt.sign("steer") == -1
    adapt.learn_votes("steer", 3)    # the flip's reseed
    assert adapt.sign("steer") == 1


def test_store_round_trip(tmp_path, monkeypatch):
    monkeypatch.setattr(adaptation, "_store_path",
                        lambda: tmp_path / "control_adaptations.json")
    found = for_aircraft("Add-on A320", "A320")
    found.learn_trim("throttle", 0.33)
    found.learn("steer.authority", 42.0)
    adaptation.save(force=True)
    adaptation.reset()
    again = for_aircraft("Add-on A320", "A320")
    assert again.trim("throttle") == pytest.approx(0.33)
    assert again.value("steer.authority") == pytest.approx(42.0)


def test_same_type_different_titles_learn_apart():
    one = for_aircraft("Add-on A320 X", "A320")
    two = for_aircraft("Add-on A320 Y", "A320")
    one.learn_trim("throttle", 0.9)
    assert two.trim("throttle") == 0.0
    # And a model with no title still learns, by its type.
    typed = for_aircraft("", "B738")
    typed.learn_trim("throttle", 0.7)
    assert for_aircraft("", "B738").trim("throttle") == pytest.approx(0.7)


def test_oscillation_verdict_beats_saturation():
    adapt = Adaptation()
    pid = AdaptivePid("throttle", adapt,
                      PidTerms(kp=2.0, deadband=0.5, low=-1.0, high=1.0),
                      clock=Clock())
    # An error swinging fast enough to pin the lever on the rails both
    # ways is oscillating, not under-gained: it must not grow the gain.
    for i in range(80):
        pid.update(8.0 if i % 2 else -8.0, dt=0.25)
    assert adapt.scale("throttle") < 1.0


def test_bump_gap_leaves_the_loop_alone():
    adapt = Adaptation()
    pid = AdaptivePid("collective", adapt,
                      PidTerms(kp=1.0, deadband=1.0, low=0.0, high=1.0),
                      clock=Clock())
    clock = pid._clock
    pid._bump(adaptation.SCALE_UP, clock())
    pid._bump_t = clock()              # just bumped: the gap is open
    after = adapt.scale("collective")
    # Saturated evidence inside the gap does not bump again.
    for _ in range(20):
        clock.step(0.25)
        pid.update(50.0, dt=0.25)
    assert adapt.scale("collective") == pytest.approx(after)


def test_bump_rescales_the_integrator_not_the_lever():
    adapt = Adaptation()
    pid = AdaptivePid("throttle", adapt,
                      PidTerms(kp=0.1, ki=0.5, low=0.0, high=1.0),
                      clock=Clock())
    for _ in range(40):
        pid.update(1.0, dt=0.25)
    ki_i = pid._i * pid.terms.ki
    pid._bump(adaptation.SCALE_DOWN, pid._clock())
    assert pid._i * pid.terms.ki == pytest.approx(ki_i)


def test_reset_clears_oscillation_evidence():
    adapt = Adaptation()
    pid = AdaptivePid("throttle", adapt,
                      PidTerms(kp=0.1, deadband=0.5, low=-1.0, high=1.0),
                      clock=Clock())
    # Four flips: short of the oscillation verdict -- evidence on the
    # tally that has not yet acted.
    for i in range(5):
        pid.update(8.0 if i % 2 else -8.0, dt=0.25)
    assert pid._flips
    assert adapt.scale("throttle") == pytest.approx(1.0)
    pid.reset(output=0.5)
    assert not pid._flips
    assert pid._sat_t == 0.0
    # The old phase's flips cannot trigger a bump in the new one.
    for _ in range(6):
        pid.update(5.0, dt=0.25)
    assert adapt.scale("throttle") == pytest.approx(1.0)


def test_effort_firms_or_softens_a_pass():
    terms = PidTerms(kp=0.1, low=-2.0, high=2.0)
    firm = Pid(terms, clock=Clock())
    soft = Pid(terms, clock=Clock())
    assert firm.update(5.0, dt=0.25, effort=1.5) == pytest.approx(0.75)
    assert soft.update(5.0, dt=0.25, effort=0.5) == pytest.approx(0.25)
    # The ask is bounded: absurd effort is clamped, not trusted.
    assert firm.update(5.0, dt=0.25, effort=9.0) == pytest.approx(1.25)
    # The integrator never carries the press: the trim a firmer pass
    # winds is the same one a neutral pass would.
    integ = Pid(PidTerms(ki=0.5, low=-2.0, high=2.0), clock=Clock())
    assert integ.update(5.0, dt=0.25, effort=2.0) == pytest.approx(0.625)


def test_assist_presses_while_the_error_is_not_coming_in():
    adapt = Adaptation()
    pid = AdaptivePid("throttle", adapt,
                      PidTerms(kp=0.1, deadband=0.5, low=-2.0, high=2.0),
                      clock=Clock())
    # An error holding its own for seconds: the pass presses harder
    # each tick, leaning in long before the saturation verdict could.
    first = pid.update(8.0, dt=0.25)
    last = first
    for _ in range(24):
        last = pid.update(8.0, dt=0.25)
    assert pid._assist > 1.0
    assert last > first


def test_assist_softens_as_the_error_collapses():
    adapt = Adaptation()
    pid = AdaptivePid("throttle", adapt,
                      PidTerms(kp=0.1, deadband=0.5, low=-2.0, high=2.0),
                      clock=Clock())
    for _ in range(12):
        pid.update(8.0, dt=0.25)           # build the trailing reference
    outs = [pid.update(1.0, dt=0.25) for _ in range(6)]
    assert pid._assist < 1.0
    assert outs[0] > outs[-1]              # easing off, not stepping off
    assert outs[-1] < 0.1                  # softer than the plain pass


def test_assist_comes_home_when_there_is_nothing_to_press():
    adapt = Adaptation()
    pid = AdaptivePid("throttle", adapt,
                      PidTerms(kp=0.1, deadband=0.5, low=-2.0, high=2.0),
                      clock=Clock())
    for _ in range(12):
        pid.update(8.0, dt=0.25)
    for _ in range(40):
        pid.update(0.1, dt=0.25)           # inside the deadband: settled
    assert pid._assist == pytest.approx(1.0)


def test_assist_leaves_a_dead_tick_alone():
    adapt = Adaptation()
    pid = AdaptivePid("throttle", adapt,
                      PidTerms(kp=0.1, deadband=0.5, low=-2.0, high=2.0),
                      clock=Clock())
    pid.update(8.0, dt=0.25)
    before = pid._assist
    pid.update(8.0, dt=60.0)               # a resumed tick, not a held one
    assert pid._assist == before
