"""Holding the AI arrival queue.

When more arrivals are inbound than the runway takes, approach holds the
back of the queue -- the same ``atc.holds`` model the pilot gets -- and
lets it go when the queue has shrunk to where it fits. These cover the
queue counting, the hold being issued and released, and the racetrack
the flight model steers while it lasts.
"""

from __future__ import annotations

from unittest.mock import Mock

import pytest

from meridianatc.atc import holds as hold_rules
from meridianatc.atc.traffic import TrafficInjector
from meridianatc.atc.traffic._shared import Flight


def _injector(ident="KXXX"):
    navdb = Mock()
    navdb.airport.return_value = None
    injector = TrafficInjector(navdb, seed=1, density=1.0)
    injector.field.ident = ident
    injector.field.latitude = 51.0
    injector.field.longitude = -1.0
    injector.field.arrival_runway = "09L"
    injector.field.departure_runway = "09L"
    injector._exchange = Mock()
    return injector


def _inbound(distance_nm: float, runway: str = "09L",
             **kw) -> Flight:
    flight = Flight(written="TST123", type_code="B738", aircraft=Mock(),
                    language="en", operator=None, arriving=True)
    flight.ifr = True
    flight.phase = "inbound"
    flight.step = 1
    flight.runway = runway
    flight.distance_nm = distance_nm
    flight.latitude = 51.0
    flight.longitude = -1.0 - distance_nm / 60.0
    flight.altitude_ft = 9000.0
    flight.on_ground = False
    flight.airborne_model = True
    for key, value in kw.items():
        setattr(flight, key, value)
    return flight


def test_arrivals_ahead_counts_the_queue():
    injector = _injector()
    closer = [_inbound(d) for d in (10.0, 20.0, 30.0)]
    back = _inbound(45.0)
    # A further arrival and a parallel-strip arrival are nobody's queue.
    injector.flights = closer + [back, _inbound(50.0),
                                 _inbound(8.0, runway="09R")]
    assert injector._arrivals_ahead(back) == 3
    assert injector._arrivals_ahead(closer[0]) == 0


def test_hold_issued_past_the_threshold():
    injector = _injector()                      # faa field: six ahead
    queue = [_inbound(d) for d in (10, 18, 26, 34, 42, 50)]
    tail = _inbound(60.0)
    injector.flights = queue + [tail]
    assert not injector._needs_the_hold(queue[-1])     # five ahead: fits
    assert injector._needs_the_hold(tail)              # six ahead: held


def test_hold_given_and_released():
    injector = _injector()
    queue = [_inbound(d) for d in (10, 18, 26, 34, 42, 50)]
    tail = _inbound(60.0)
    injector.flights = queue + [tail]

    injector._give_the_hold(tail, now=100.0)
    hold = tail.hold
    assert hold is not None
    assert hold.reason == "sequence" and hold.number == 7
    assert tail.hold_leg == "in"
    injector._exchange.assert_called_once()

    # Five ahead still: it stays -- the gap between "hold at six" and
    # "released at four" is what keeps it out of a hold/leave/hold flap.
    injector.flights = queue[:5] + [tail]
    assert injector._needs_the_hold(tail)              # five ahead: stays
    injector.flights = queue[:4] + [tail]
    assert not injector._needs_the_hold(tail)          # four: released
    injector._leave_the_hold(tail, now=200.0)
    assert tail.hold is None and tail.hold_leg == ""


def test_held_arrival_keeps_its_place():
    """A held aeroplane's slot does not move while it orbits: a later
    arrival farther out still counts it as ahead."""
    injector = _injector()
    holder = _inbound(40.0)
    holder.hold = hold_rules.Hold(
        fix="TSTFX", lat=51.5, lon=-0.5, inbound_true=90.0,
        side="north-west", leg_minutes=1.0, altitude_ft=9000.0,
        reason="sequence", efc_hhmm="0000", number=2)
    late = _inbound(55.0)
    injector.flights = [holder, _inbound(10.0), late]
    assert injector._arrivals_ahead(late) == 2
    # And the holder does not count the one behind it.
    assert injector._arrivals_ahead(holder) == 1


def test_racetrack_steering():
    injector = _injector()
    flight = _inbound(30.0)
    flight.hold = hold_rules.Hold(
        fix="TSTFX", lat=51.0, lon=-0.5, inbound_true=90.0,
        side="north", leg_minutes=1.0, altitude_ft=8000.0,
        reason="sequence", efc_hhmm="0000")
    flight.hold_leg = "in"
    flight.hold_leg_at = injector._now()

    target = flight.target
    injector._steer_hold(flight, target)
    # Inbound leg: at the fix, slowed and levelled to the hold's numbers.
    assert target.heading == pytest.approx(90.0, abs=1.0)
    assert target.altitude_ft == 8000.0
    assert target.speed_kt <= injector.HOLD_SPEED_KT
    assert "holding" in target.intent

    # Over the fix the outbound leg begins, on the reciprocal.
    flight.latitude, flight.longitude = 51.0, -0.5
    injector._steer_hold(flight, target)
    assert flight.hold_leg == "out"
    injector._steer_hold(flight, target)
    assert target.heading == pytest.approx(270.0)

    # A minute later, at the end of the outbound leg, it turns back in
    # -- the aeroplane is a few miles out by then, off the fix itself.
    flight.hold_leg_at -= 61.0
    flight.longitude = -0.55
    injector._steer_hold(flight, target)
    assert flight.hold_leg == "in"


def test_hold_deferred_when_no_room_to_say_it():
    """Before the check-in is done approach does not hold it -- the
    instruction needs a radio that has been called."""
    injector = _injector()
    tail = _inbound(60.0)
    tail.step = 0
    injector.flights = [_inbound(d) for d in (10, 18, 26, 34, 42, 50)] \
        + [tail]
    # _needs_the_hold is checked from the inbound block behind step >= 1,
    # so the raw predicate says hold -- the caller's step gate says wait.
    assert injector._needs_the_hold(tail)
