"""The powerplant tables, the fixed-wing engine watch, the kit-driven
icing logic, and the rotorcraft's torque/rotor protections -- without a
simulator."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from meridianatc.atc.powerplant import ice_kit_for, limits_for
from meridianatc.sim.state import AircraftState

# -- the tables ---------------------------------------------------------------


def test_rotorcraft_limits_carry_the_researched_figures():
    r66 = limits_for("R66", "helicopter")
    assert (r66.tq_takeoff_pct, r66.tq_cont_pct) == (100.0, 83.0)
    assert r66.itt_max_c == 706.0 and r66.itt_takeoff_c == 782.0
    assert (r66.nr_min_pct, r66.nr_max_pct) == (99.0, 101.0)
    b407 = limits_for("B407", "helicopter")
    assert b407.tq_cont_pct == 93.5 and b407.top_max_ias_kt == 100.0
    ec35 = limits_for("EC35", "helicopter")
    assert ec35.tq_takeoff_pct == 75.0 and ec35.tq_cont_pct == 69.0
    b412 = limits_for("B412", "helicopter")
    assert b412.tq_cont_pct == 81.0 and b412.itt_start_c == 1090.0
    # Every rotorcraft type in the fleet table resolves.
    for code in ("R44", "R66", "H125", "AS50", "EC30", "B06", "B407",
                 "EC35", "EC45", "H145", "H160", "AS32", "S76", "A109",
                 "A139", "B412", "UH1", "CBRI"):
        assert limits_for(code, "helicopter") is not None


def test_fixed_wing_limits_and_the_class_ladder():
    tbm = limits_for("TBM9", "turboprop_single")
    assert (tbm.itt_takeoff_c, tbm.itt_max_c) == (850.0, 840.0)
    assert tbm.ng_max_pct == 104.1
    b738 = limits_for("B738", "airliner")
    assert b738.itt_takeoff_c == 950.0
    # An unknown type resolves the class's figures, marked unverified.
    assert limits_for("ZZ99", "airliner").unverified
    assert limits_for("ZZ99", "piston_single").rpm_max == 2700.0
    # No class at all: a blank figure set, nothing invented.
    lim = limits_for("ZZ99", "")
    assert lim.tq_takeoff_pct == 0.0 and lim.itt_max_c == 0.0


def test_the_king_air_torque_is_a_number_not_the_gauges_paint():
    """The BE9L's red_hi_TRQ reads 1.0 ft-lb live -- every limit dataref
    left at paint -- so the type carries the POH's own figures: 1,315
    ft-lb is 1,783 N-m, the ceiling either way the sim reports it."""
    be9l = limits_for("BE9L", "turboprop_twin")
    assert be9l.tq_takeoff_nm == be9l.tq_max_nm == 1783.0
    assert be9l.itt_takeoff_c == 700.0 and be9l.itt_max_c == 695.0
    assert be9l.ng_max_pct == 101.5


def test_ice_kits_match_the_types_systems():
    assert ice_kit_for("C152", "piston_single").carb_heat
    assert not ice_kit_for("C172", "piston_single").carb_heat
    assert not ice_kit_for("C172", "piston_single").engine_inlet
    tbm = ice_kit_for("TBM9", "turboprop_single")
    assert tbm.inertial_separator and tbm.known_ice and tbm.airframe_deice
    assert ice_kit_for("R44", "helicopter").carb_heat
    assert not ice_kit_for("R44", "helicopter").known_ice
    assert ice_kit_for("B407", "helicopter").engine_inlet
    assert not ice_kit_for("A139", "helicopter").known_ice
    assert ice_kit_for("B738", "airliner").known_ice


# -- the fixed-wing engine watch ----------------------------------------------


def _fo(**kw):
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import AutonomousFlight, AutonomousState
    fo = AutonomousFlight.__new__(AutonomousFlight)
    fo.state = AutonomousState()
    fo.sim = None
    fo._dir = Mock()
    fo._dir.sim = None
    fo._heli = None
    fo._is_helicopter = False
    fo._aircraft_class = "airliner"
    fo._ap_mode = ""
    fo.session = SimpleNamespace(
        aircraft=SimpleNamespace(type_code="B738", title=""),
        assigned_heading=None, assigned_altitude_ft=0,
        assigned_speed_kt=0, cruise_altitude_ft=0, initial_altitude_ft=0)
    fo.on_event = Mock()
    fo._last_lever = 0.0
    for key, value in kw.items():
        setattr(fo, key, value)
    return fo


def _emit_keys(fo):
    return [call.kwargs.get("key") for call in fo.on_event.call_args_list]


def test_limit_cap_eases_the_lever_over_the_red_line():
    fo = _fo()
    fo._OVER_PERSIST_S = 0.0
    # A B738 over its continuous EGT in cruise: the cap comes off.
    hot = AircraftState(on_ground=False, engines_running=True,
                        eng_itt_c=960.0)
    assert fo._engine_limit_cap(hot) == pytest.approx(0.85)
    assert "autonomous.engines.limit" in _emit_keys(fo)
    # Inside the line from the start: no cap at all. (A gauge that just
    # cleared keeps its ceiling a few clean seconds -- that is the
    # ratchet test below, not this one's never-over aeroplane.)
    fo2 = _fo()
    fo2._OVER_PERSIST_S = 0.0
    warm = AircraftState(on_ground=False, engines_running=True,
                         eng_itt_c=900.0)
    assert fo2._engine_limit_cap(warm) == pytest.approx(1.0)


def test_the_cap_walks_down_until_the_needle_is_inside():
    """A King Air still over its line at the reflex pull: the ceiling
    keeps coming down a step at a time until the reading is in, floored
    where power stays on the aeroplane -- and it is only let back out
    once the gauges have held clean."""
    import time as _time

    fo = _king_air()
    fo._OVER_PERSIST_S = 0.0
    fo._LIMIT_STEP_S = 0.0
    fo._LIMIT_CLEAR_S = 60.0
    hot = AircraftState(on_ground=False, engines_running=True,
                        eng_itt_c=710.0)
    cool = AircraftState(on_ground=False, engines_running=True,
                         eng_itt_c=600.0)
    assert fo._engine_limit_cap(hot) == pytest.approx(0.85)
    walked = [fo._engine_limit_cap(hot) for _ in range(3)]
    assert walked == pytest.approx([0.78, 0.71, 0.64])
    for _ in range(20):
        cap = fo._engine_limit_cap(hot)
    assert cap == pytest.approx(0.30)
    # Just cleared: the ceiling holds where the walk left it.
    assert fo._engine_limit_cap(cool) == pytest.approx(0.30)
    # Held clean long enough, it is let back out.
    fo._limit_clean = _time.monotonic() - 61.0
    assert fo._engine_limit_cap(cool) == pytest.approx(1.0)


# -- the autothrottle watchdog ----------------------------------------------


def _cruising(ias: float, at: bool = True, lever: float = 0.9) -> AircraftState:
    return AircraftState(on_ground=False, engines_running=True,
                         autothrottle=at, indicated_airspeed_kt=ias,
                         throttle_pct=lever * 100.0)


def test_a_working_autothrottle_keeps_the_levers():
    """On the bug, a live autothrottle is trusted with the levers: the
    servo does not touch them and the doubt clock stays down."""
    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _king_air()
    fo.state.phase = AutonomousPhase.CRUISE
    fo._manage_throttle(_cruising(210.0), 210.0)
    fo._dir.throttle.assert_not_called()
    assert getattr(fo, "_at_bad_since", 0.0) == 0.0


def test_a_gust_over_the_bug_does_not_disconnect():
    """Over the figure but not for long: an honest autothrottle is
    given the window to pull it back, so the servo still stands down."""
    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _king_air()
    fo.state.phase = AutonomousPhase.CRUISE
    fo._AT_ERR_S = 999.0
    fo._manage_throttle(_cruising(255.0), 210.0)
    fo._dir.throttle.assert_not_called()
    fo._dir.autothrottle_disconnect.assert_not_called()
    assert fo._at_bad_since > 0.0


def test_an_autothrottle_off_the_speed_is_taken_off_the_levers():
    """Fifteen knots over for as long as a change or a gust takes: the
    thing on the engines is the model's own logic, not an autothrottle
    -- it is disconnected once, flagged untrusted, and the FO's servo
    runs the levers itself from then on."""
    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _king_air()
    fo.state.phase = AutonomousPhase.CRUISE
    fo._AT_ERR_S = 0.0
    fo._manage_throttle(_cruising(255.0), 210.0)
    fo._dir.autothrottle_disconnect.assert_called_once()
    assert "autonomous.at" in _emit_keys(fo)
    assert fo._at_distrusted
    # The servo itself now runs the speed: the lever comes back from
    # where it was, a notch at a time (a hand's second between calls --
    # the slew, not the target, holds the rate).
    import time as _time
    fo._dir.reset_mock()
    fo._lever_at = _time.monotonic() - 1.0
    fo._manage_throttle(_cruising(255.0, at=True, lever=0.9), 210.0)
    assert fo._dir.throttle.called
    assert fo._dir.throttle.call_args[0][0] == pytest.approx(0.84, abs=0.02)
    # The flag dropping (disconnect landed, or the AT was switched)
    # re-earns the benefit of the doubt for the next engagement.
    fo._manage_throttle(_cruising(210.0, at=False, lever=0.5), 210.0)
    assert not fo._at_distrusted


def test_takeoff_rating_governs_the_roll():
    fo = _fo()
    fo._OVER_PERSIST_S = 0.0
    # 940 C is over the 925 continuous figure, under the 950 takeoff one.
    roll = AircraftState(on_ground=True, engines_running=True,
                         eng_itt_c=940.0)
    assert fo._engine_limit_cap(roll, takeoff=True) == pytest.approx(1.0)
    assert fo._engine_limit_cap(roll) == pytest.approx(0.85)


def test_a_missing_gauge_is_never_a_limit():
    fo = _fo()
    quiet = AircraftState(on_ground=False, engines_running=True)
    assert fo._engine_limit_cap(quiet) == pytest.approx(1.0)
    assert fo._engines_over_limit(quiet) == []


def test_ng_and_piston_rpm_are_watched_too():
    fo = _fo()
    surge = AircraftState(on_ground=False, engines_running=True,
                          eng_n1_pct=105.5)
    assert "ng" in fo._engines_over_limit(surge)
    fo._aircraft_class = "piston_single"
    fo.session.aircraft = SimpleNamespace(type_code="C172", title="")
    fo._eng_limit_key = None
    over = AircraftState(on_ground=False, engines_running=True,
                         eng_rpm=2850.0)
    assert "rpm" in fo._engines_over_limit(over)


def test_the_watch_emits_once_and_rearms():
    fo = _fo()
    fo._OVER_PERSIST_S = 0.0
    hot = AircraftState(on_ground=False, engines_running=True,
                        eng_itt_c=960.0)
    fo._watch_engines(hot, 10.0)
    fo._watch_engines(hot, 11.0)
    assert _emit_keys(fo).count("autonomous.engines.overlimit") == 1
    fo._watch_engines(AircraftState(on_ground=False,
                                    engines_running=True,
                                    eng_itt_c=800.0), 12.0)
    assert not fo.state.eng_warned


def test_a_spike_across_the_line_is_not_an_over_limit():
    """A King Air's EGT feed swung 150 C between quarter-second reads --
    enough to dip under the line and cross it again inside a second.
    The cap and the call wait for a reading that stays there, the way a
    hand on the lever does."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _fo()
    fo.state.phase = AutonomousPhase.CRUISE
    fo.state.sub = "level"
    hot = AircraftState(on_ground=False, engines_running=True,
                        eng_itt_c=960.0)
    cool = AircraftState(on_ground=False, engines_running=True,
                         eng_itt_c=800.0)
    now = _time.monotonic()
    # A first read over the line starts the clock, not the cap.
    fo._watch_engines(hot, now)
    assert fo._engine_limit_cap(hot) == pytest.approx(1.0)
    assert "autonomous.engines.overlimit" not in _emit_keys(fo)
    # The gauge dipping back under restarts the clock, so the spike can
    # never accumulate -- and the sustained read two seconds on counts.
    fo._watch_engines(cool, now + 0.5)
    fo._watch_engines(hot, now + 0.7)
    assert "autonomous.engines.overlimit" not in _emit_keys(fo)
    fo._watch_engines(hot, now + 2.8)
    assert "autonomous.engines.overlimit" in _emit_keys(fo)


def test_engines_stopping_forgets_the_over_clock():
    fo = _fo()
    fo._over_since = {"itt": 5.0}
    quiet = AircraftState(on_ground=False, engines_running=False)
    fo._watch_engines(quiet, 10.0)
    assert fo._over_since == {}


def _king_air(**kw):
    fo = _fo(_aircraft_class="turboprop_twin")
    fo.session.aircraft = SimpleNamespace(type_code="BE9L", title="")
    fo._eng_limit_key = None
    return fo


def test_real_torque_is_limited_when_the_red_line_is_paint():
    """The C90 reports honest N-m against a red_hi_TRQ of 1.0: the
    per-cent path has nothing to divide by, so the type's 1,315 ft-lb
    figure is the ceiling the shaft torque answers to."""
    fo = _king_air()
    over = AircraftState(on_ground=False, engines_running=True,
                         eng_torque_nm=1900.0)
    assert "tq" in fo._engines_over_limit(over)
    fine = AircraftState(on_ground=False, engines_running=True,
                         eng_torque_nm=1700.0)
    assert "tq" not in fo._engines_over_limit(fine)


def test_the_models_own_red_line_beats_the_class_table():
    """A model that publishes green_hi_ITT gets its continuous ceiling
    from that number, not the type's or the class's looser figure --
    here the B738's own 925 line would wave 705 through."""
    fo = _fo()
    hot = AircraftState(on_ground=False, engines_running=True,
                        eng_itt_c=705.0, eng_itt_limit_c=695.0,
                        eng_itt_takeoff_c=700.0)
    assert "itt" in fo._engines_over_limit(hot)
    same_read_without_the_model_line = AircraftState(
        on_ground=False, engines_running=True, eng_itt_c=705.0)
    assert "itt" not in fo._engines_over_limit(
        same_read_without_the_model_line)


def test_a_piston_model_does_not_inherit_xplanes_generic_itt_line():
    """X-Plane publishes ENGN_ITT and generic red/green ITT limits even
    on a piston Baron. Neither number is a piston instrument: the twin
    IO-550 watch keeps RPM and CHT, and must not invent a turbine
    temperature exceedance."""
    fo = _fo(_aircraft_class="piston_twin")
    fo.session.aircraft = SimpleNamespace(type_code="BE58", title="")
    hot_itt = AircraftState(on_ground=False, engines_running=True,
                            engine_type=0, eng_itt_c=705.0,
                            eng_itt_limit_c=695.0,
                            eng_itt_takeoff_c=700.0)
    assert "itt" not in fo._engines_over_limit(hot_itt)
    # Even if the class evidence were wrong, the simulator's explicit
    # piston engine answer wins over it.
    explicit = _fo()
    explicit.session.aircraft = SimpleNamespace(type_code="BE58", title="")
    assert "itt" not in explicit._engines_over_limit(hot_itt)
    over_revved = AircraftState(on_ground=False, engines_running=True,
                                engine_type=0, eng_rpm=2800.0,
                                eng_itt_c=705.0, eng_itt_limit_c=695.0)
    assert fo._engines_over_limit(over_revved) == ["rpm"]


def test_percent_torque_still_governs_where_it_is_published():
    """A model that does report a sane red_hi_TRQ keeps the per-cent
    path; the N-m fallback does not double-derive its own ceiling."""
    fo = _fo(_aircraft_class="helicopter")
    fo.session.aircraft = SimpleNamespace(type_code="R66", title="")
    fo._eng_limit_key = None
    over = AircraftState(on_ground=False, engines_running=True,
                         eng_torque_pct=102.0, eng_torque_nm=10.0)
    assert "tq" in fo._engines_over_limit(over)
    # Under the per-cent line, the same N-m reading is not a limit.
    under = AircraftState(on_ground=False, engines_running=True,
                          eng_torque_pct=80.0, eng_torque_nm=9000.0)
    assert "tq" not in fo._engines_over_limit(under)


def test_the_second_engines_limits_are_its_own():
    """A King Air with the right engine running hot: engine one's
    scalar reads fine, but the watch answers the slot that is over --
    "itt #2" with its own persistence clock, not a false all-clear
    from the engine that happens to be first."""
    fo = _king_air()
    hot_right = AircraftState(
        on_ground=False, engines_running=True,
        eng_itt_c=650.0, eng2_itt_c=760.0,
        eng_itt_limit_c=695.0, eng_itt_takeoff_c=700.0)
    over = fo._engines_over_limit(hot_right)
    assert "itt #2" in over and "itt" not in over
    # And the other way around: engine one's own scalar is #1.
    hot_left = AircraftState(
        on_ground=False, engines_running=True,
        eng_itt_c=760.0, eng2_itt_c=650.0,
        eng_itt_limit_c=695.0, eng_itt_takeoff_c=700.0)
    over = fo._engines_over_limit(hot_left)
    assert "itt" in over and "itt #2" not in over


def test_the_second_engines_torque_is_capped_too():
    fo = _king_air()
    over = AircraftState(on_ground=False, engines_running=True,
                         eng_torque_nm=1200.0, eng2_torque_nm=1900.0)
    assert "tq #2" in fo._engines_over_limit(over)


def test_an_unreported_slot_is_no_engine():
    """A single has no eng2_ fields at all: the per-engine pass must
    not read a second engine out of the defaults."""
    fo = _fo()
    state = AircraftState(on_ground=False, engines_running=True)
    assert fo._engines_over_limit(state) == []
    assert all(not name.endswith(("#2", "#3", "#4"))
               for name in fo._engines_over_limit(state))


# -- the fuel selector --------------------------------------------------------


def _airborne(**kw):
    return AircraftState(on_ground=False, engines_running=True, **kw)


def test_a_dry_selected_tank_is_switched_to_the_full_side():
    """The PA-28 case: still on the left tank, right wing full."""
    fo = _fo()
    fo._manage_fuel_tanks(_airborne(fuel_selector=2,
                                    fuel_left_gal=0.4,
                                    fuel_right_gal=14.0))
    fo._dir.write.assert_called_once_with("FUEL TANK SELECTOR:1", 3.0)
    assert "autonomous.fuel.tank_switch" in _emit_keys(fo)


def test_a_healthy_selected_tank_is_left_alone():
    fo = _fo()
    fo._manage_fuel_tanks(_airborne(fuel_selector=2,
                                    fuel_left_gal=12.0,
                                    fuel_right_gal=14.0))
    fo._dir.write.assert_not_called()


def test_nothing_with_fuel_is_a_tank_change_with_nowhere_to_go():
    fo = _fo()
    fo._manage_fuel_tanks(_airborne(fuel_selector=2,
                                    fuel_left_gal=0.4,
                                    fuel_right_gal=0.6))
    fo._dir.write.assert_not_called()


def test_all_and_off_are_the_crews_own_positions():
    """ALL already feeds from every tank aboard; OFF is for the pilot
    to answer -- the selector is never moved to either by this watch."""
    fo = _fo()
    for position in (0, 1, 16):
        fo._manage_fuel_tanks(_airborne(fuel_selector=position,
                                        fuel_left_gal=0.4,
                                        fuel_right_gal=14.0))
    fo._dir.write.assert_not_called()


def test_the_right_tank_mirrors_to_the_left():
    fo = _fo()
    fo._manage_fuel_tanks(_airborne(fuel_selector=20,
                                    fuel_left_gal=18.0,
                                    fuel_right_gal=0.3))
    fo._dir.write.assert_called_once_with("FUEL TANK SELECTOR:1", 19.0)


def _jet_tanks(center_qty: float, wing_qty: float = 200.0):
    return [("FUEL TANK CENTER QUANTITY", 300.0, center_qty),
            ("FUEL TANK LEFT MAIN QUANTITY", 250.0, wing_qty),
            ("FUEL TANK RIGHT MAIN QUANTITY", 250.0, wing_qty)]


def test_a_dry_centre_tank_feeds_from_all():
    """A jet whose centre has burnt out still has the wings -- ALL is
    the position that reaches them without naming one."""
    fo = _fo()
    fo.sim = Mock()
    fo.sim.fuel_tanks.return_value = _jet_tanks(0.2)
    fo._dir.sim = fo.sim
    fo._manage_fuel_tanks(_airborne(fuel_selector=6,
                                    fuel_left_gal=200.0,
                                    fuel_right_gal=200.0))
    fo._dir.write.assert_called_once_with("FUEL TANK SELECTOR:1", 1.0)


def test_a_centre_tank_with_fuel_is_left_alone():
    fo = _fo()
    fo.sim = Mock()
    fo.sim.fuel_tanks.return_value = _jet_tanks(50.0)
    fo._dir.sim = fo.sim
    fo._manage_fuel_tanks(_airborne(fuel_selector=6))
    fo._dir.write.assert_not_called()


def test_xplane_tank_indexes_answer_the_same_list():
    """m_fuel[2] is the centre in the Plane-Maker order."""
    fo = _fo()
    fo.sim = Mock()
    fo.sim.fuel_tanks.return_value = [
        ("sim/flightmodel/weight/m_fuel[0]", 50.0, 10.0),
        ("sim/flightmodel/weight/m_fuel[1]", 50.0, 30.0),
        ("sim/flightmodel/weight/m_fuel[2]", 40.0, 0.1)]
    fo._dir.sim = fo.sim
    fo._manage_fuel_tanks(_airborne(fuel_selector=6))
    fo._dir.write.assert_called_once_with("FUEL TANK SELECTOR:1", 1.0)


def test_an_aux_position_is_judged_by_the_list_not_the_gauges():
    """The side gauges read the mains: a dry left aux under full mains
    would look healthy to the gauges -- the list is what knows."""
    fo = _fo()
    fo.sim = Mock()
    fo.sim.fuel_tanks.return_value = _jet_tanks(200.0) + [
        ("FUEL TANK LEFT AUX QUANTITY", 30.0, 0.2)]
    fo._dir.sim = fo.sim
    fo._manage_fuel_tanks(_airborne(fuel_selector=4,
                                    fuel_left_gal=200.0,
                                    fuel_right_gal=200.0))
    fo._dir.write.assert_called_once_with("FUEL TANK SELECTOR:1", 1.0)


def test_the_selector_is_not_written_twice_in_the_same_breath():
    fo = _fo()
    state = _airborne(fuel_selector=2, fuel_left_gal=0.4,
                      fuel_right_gal=14.0)
    fo._manage_fuel_tanks(state)
    fo._manage_fuel_tanks(state)
    fo._dir.write.assert_called_once()


def test_a_refused_write_is_not_hammered_every_tick():
    fo = _fo()
    fo._dir.write.return_value = False
    state = _airborne(fuel_selector=2, fuel_left_gal=0.4,
                      fuel_right_gal=14.0)
    fo._manage_fuel_tanks(state)
    # The attempt still latches the cooldown: a model that cannot take
    # the position is asked again on the gauge's cadence, not every
    # pass.
    fo._manage_fuel_tanks(state)
    fo._dir.write.assert_called_once()


def test_an_engine_not_running_is_not_a_tank_problem():
    fo = _fo()
    fo._manage_fuel_tanks(AircraftState(on_ground=False,
                                        engines_running=False,
                                        fuel_selector=2,
                                        fuel_left_gal=0.0,
                                        fuel_right_gal=14.0))
    fo._dir.write.assert_not_called()


def test_an_unpublished_selector_is_never_moved():
    fo = _fo()
    fo._manage_fuel_tanks(AircraftState(
        on_ground=False, engines_running=True, fuel_selector=-1,
        fuel_left_gal=0.0, fuel_right_gal=14.0,
        unavailable=("fuel_selector",)))
    fo._dir.write.assert_not_called()


# -- the condition lever ------------------------------------------------


def _running(**kw):
    return AircraftState(engines_running=True, **kw)


def test_a_condition_lever_at_cutoff_on_the_ground_is_lifted_to_low_idle():
    """The parked King Air's lever: a shut fuel cock on a running
    engine is the low-idle detent's business, not a warning's."""
    fo = _fo(_aircraft_class="turboprop_twin")
    fo._manage_condition(_running(on_ground=True, condition_pct=0.0))
    fo._dir.condition.assert_called_once_with(1.0)
    assert "autonomous.condition.lever" in _emit_keys(fo)


def test_high_idle_is_the_flight_detent():
    """Low idle in the air is the prop governor's floor: a go-around
    from it is seconds of nothing before the engine answers."""
    fo = _fo(_aircraft_class="turboprop_twin")
    fo._manage_condition(_running(on_ground=False, condition_pct=50.0))
    fo._dir.condition.assert_called_once_with(2.0)


def test_the_line_up_wants_high_idle_on_the_ground():
    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _fo(_aircraft_class="turboprop_twin")
    fo.state.phase = AutonomousPhase.LINE_UP
    fo._manage_condition(_running(on_ground=True, condition_pct=50.0))
    fo._dir.condition.assert_called_once_with(2.0)


def test_low_idle_on_the_taxi_is_the_detent_the_phase_asks():
    """The ground's own stop: a running engine at low idle is normal
    ground ops, and a lever at high idle there is the pilot's to keep."""
    fo = _fo(_aircraft_class="turboprop_twin")
    for lever in (50.0, 100.0):
        fo._manage_condition(_running(on_ground=True,
                                      condition_pct=lever))
    fo._dir.condition.assert_not_called()


def test_a_pistons_red_knob_is_never_a_condition_lever():
    """A piston's lever is a mixture and a jet has neither handle: the
    field reads zero there and must not move either."""
    fo = _fo(_aircraft_class="piston_single")
    fo._manage_condition(_running(on_ground=True, condition_pct=0.0))
    fo._dir.condition.assert_not_called()


def test_an_unpublished_condition_lever_is_never_moved():
    fo = _fo(_aircraft_class="turboprop_twin")
    fo._manage_condition(_running(on_ground=True, condition_pct=0.0,
                                  unavailable=("condition_pct",)))
    fo._dir.condition.assert_not_called()


def test_engines_not_running_leaves_the_lever_at_the_checklists_stop():
    """The shutdown's own item puts the levers at cut-off -- the watch
    is not there to argue with it."""
    fo = _fo(_aircraft_class="turboprop_twin")
    fo._manage_condition(AircraftState(on_ground=True,
                                       engines_running=False,
                                       condition_pct=0.0))
    fo._dir.condition.assert_not_called()


def test_the_second_levers_detent_counts_on_a_twin():
    """A King Air whose number-two lever stayed at cut-off: the first
    lever at the detent does not excuse it -- the floor is the lowest
    lever, and the write moves them both."""
    fo = _fo(_aircraft_class="turboprop_twin")
    fo._manage_condition(_running(
        on_ground=True, condition_pct=50.0, eng2_condition_pct=0.0))
    fo._dir.condition.assert_called_once_with(1.0)
    # And in the air: low idle on the second engine still wants high.
    fo2 = _fo(_aircraft_class="turboprop_twin")
    fo2._manage_condition(_running(
        on_ground=False, condition_pct=100.0, eng2_condition_pct=50.0))
    fo2._dir.condition.assert_called_once_with(2.0)


def test_the_condition_lever_is_not_written_twice_in_the_same_breath():
    fo = _fo(_aircraft_class="turboprop_twin")
    state = _running(on_ground=True, condition_pct=0.0)
    fo._manage_condition(state)
    fo._manage_condition(state)
    fo._dir.condition.assert_called_once()


def test_deadband_does_not_hide_an_engine_over_limit():
    """On-target speed is the deadband the throttle servo leaves alone --
    but a gauge drifting past its red line at that speed still gets the
    power pulled; an exceedance is not a deadband case."""
    import time as _time

    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _fo()
    fo._OVER_PERSIST_S = 0.0
    fo.state.phase = AutonomousPhase.CRUISE
    # A slew clock a second old: the lever has room to reach the cap.
    fo._lever_at = _time.monotonic() - 1.0
    fo._manage_throttle(AircraftState(
        on_ground=False, engines_running=True,
        indicated_airspeed_kt=140.0, throttle_pct=95.0,
        eng_itt_c=960.0), 140.0)
    fo._dir.throttle.assert_called_with(pytest.approx(0.85))
    # A quiet engine on the same on-target speed leaves the lever alone.
    fo2 = _fo()
    fo2.state.phase = AutonomousPhase.CRUISE
    fo2._manage_throttle(AircraftState(
        on_ground=False, engines_running=True,
        indicated_airspeed_kt=140.0, throttle_pct=95.0,
        eng_itt_c=800.0), 140.0)
    fo2._dir.throttle.assert_not_called()


def test_a_climb_owes_height_keeps_the_lever_up():
    """The 2026-11-20 mush: ten knots fast in a climb that still owed
    six thousand feet walked the lever to single digits at six hundred
    AGL and the aeroplane sank. A climb is flown at climb power -- the
    pitch bug answers the speed, not the throttle."""
    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _fo()
    fo.state.phase = AutonomousPhase.CLIMB
    fo.session.assigned_altitude_ft = 7000
    fo._manage_throttle(AircraftState(
        on_ground=False, engines_running=True,
        indicated_airspeed_kt=98.0, altitude_ft=1100.0,
        altitude_agl_ft=600.0, throttle_pct=95.0), 85.0)
    lever = fo._dir.throttle.call_args.args[0]
    assert lever >= 0.75


def test_heli_watch_judges_departure_on_the_takeoff_rating():
    """The rotorcraft's sub-phase is never "roll": without its own stage
    the watch would hold a legal takeoff power to the continuous line --
    the H125's 800 C is over 775 continuous and under 846 takeoff."""
    fo = _fo(_aircraft_class="helicopter", _is_helicopter=True)
    fo._OVER_PERSIST_S = 0.0
    fo.session.aircraft = SimpleNamespace(type_code="H125", title="")
    fo._heli = SimpleNamespace(state=SimpleNamespace(stage="departure"))
    legal = AircraftState(on_ground=False, engines_running=True,
                          eng_itt_c=800.0)
    fo._watch_engines(legal, 10.0)
    assert "autonomous.engines.overlimit" not in _emit_keys(fo)
    # The same figure in cruise is over the continuous ceiling.
    fo._heli.state.stage = "cruise"
    fo._watch_engines(legal, 11.0)
    assert "autonomous.engines.overlimit" in _emit_keys(fo)


# -- the icing watch ------------------------------------------------------------


def test_carb_heat_on_a_carburetted_piston():
    fo = _fo(_aircraft_class="piston_single")
    fo.session.aircraft = SimpleNamespace(type_code="C152", title="")
    icing = AircraftState(on_ground=False, engines_running=True,
                          engine_count=1, ambient_in_cloud=True,
                          ambient_temperature_c=2.0,
                          total_air_temperature_c=2.0,
                          throttle_pct=80.0)
    fo._watch_icing(icing, 10.0)
    fo._dir.pitot_heat.assert_called_with(True)
    # Carb heat is the anti-ice position for a piston -- on.
    assert fo._dir.write.call_args_list
    calls = [c.args for c in fo._dir.write.call_args_list]
    assert ("GENERAL ENG ANTI ICE POSITION:1", 1.0) in calls
    # No boots, no prop, no windshield on a C152.
    assert not any("STRUCTURAL" in str(c[0]) for c in calls)
    assert fo.state.carb_set


def test_carb_heat_comes_back_off_when_the_air_dries():
    fo = _fo(_aircraft_class="piston_single")
    fo.session.aircraft = SimpleNamespace(type_code="C152", title="")
    fo.state.carb_set = True
    dry = AircraftState(on_ground=False, engines_running=True,
                        engine_count=1, ambient_temperature_c=15.0,
                        ambient_humidity_pct=40.0,
                        throttle_pct=80.0)
    fo._watch_icing(dry, 10.0)
    calls = [c.args for c in fo._dir.write.call_args_list]
    assert ("GENERAL ENG ANTI ICE POSITION:1", 0.0) in calls
    assert not fo.state.carb_set


def test_an_injected_piston_gets_pitot_only():
    fo = _fo(_aircraft_class="piston_single")
    fo.session.aircraft = SimpleNamespace(type_code="C172", title="")
    icing = AircraftState(on_ground=False, engines_running=True,
                          engine_count=1, ambient_in_cloud=True,
                          ambient_temperature_c=2.0,
                          total_air_temperature_c=2.0,
                          throttle_pct=80.0)
    fo._watch_icing(icing, 10.0)
    fo._dir.pitot_heat.assert_called_with(True)
    calls = [c.args for c in fo._dir.write.call_args_list]
    assert not any("ANTI ICE" in str(c[0]) for c in calls)
    assert not fo.state.carb_set


def test_a_turboprop_gets_the_full_kit():
    fo = _fo(_aircraft_class="turboprop_single")
    fo.session.aircraft = SimpleNamespace(type_code="TBM9", title="")
    icing = AircraftState(on_ground=False, engines_running=True,
                          engine_count=1, ambient_in_cloud=True,
                          ambient_temperature_c=-5.0,
                          total_air_temperature_c=-2.0)
    fo._watch_icing(icing, 10.0)
    calls = [c.args[0] for c in fo._dir.write.call_args_list]
    assert "GENERAL ENG ANTI ICE POSITION:1" in calls
    assert "PROP DEICE SWITCH:1" in calls
    assert "STRUCTURAL DEICE SWITCH" in calls
    assert "WINDSHIELD DEICE SWITCH:1" in calls


def test_a_turbine_helicopter_gets_engine_anti_ice():
    fo = _fo(_aircraft_class="helicopter", _is_helicopter=True)
    fo.session.aircraft = SimpleNamespace(type_code="B407", title="")
    icing = AircraftState(on_ground=False, engines_running=True,
                          engine_count=1, ambient_in_cloud=True,
                          ambient_temperature_c=2.0,
                          total_air_temperature_c=2.0)
    fo._watch_icing(icing, 10.0)
    calls = [c.args for c in fo._dir.write.call_args_list]
    assert ("GENERAL ENG ANTI ICE POSITION:1", 1.0) in calls
    # And no structural/windshield switches on a rotorcraft.
    assert not any("STRUCTURAL" in str(c[0]) for c in calls)


def test_collective_is_the_low_power_proxy_for_a_rotorcraft():
    fo = _fo(_aircraft_class="helicopter", _is_helicopter=True)
    fo.session.aircraft = SimpleNamespace(type_code="R44", title="")
    descent = AircraftState(on_ground=False, engines_running=True,
                            collective_position=20.0,
                            ambient_temperature_c=15.0,
                            ambient_humidity_pct=40.0)
    assert fo._carb_heat_wanted(descent)
    climb = AircraftState(on_ground=False, engines_running=True,
                          collective_position=80.0,
                          ambient_temperature_c=15.0,
                          ambient_humidity_pct=40.0)
    assert not fo._carb_heat_wanted(climb)


def test_carb_heat_comes_off_for_the_takeoff_roll():
    """Low power is the heat case -- idle, descent, autorotation -- but
    the POHs want the heat OFF at takeoff power, which the roll is.
    A blanket on_ground term took off with the heat on."""
    fo = _fo(_aircraft_class="piston_single")
    fo.session.aircraft = SimpleNamespace(type_code="C152", title="")
    idle = AircraftState(on_ground=True, engines_running=True,
                         throttle_pct=5.0, ambient_temperature_c=15.0,
                         ambient_humidity_pct=40.0)
    assert fo._carb_heat_wanted(idle)
    roll = AircraftState(on_ground=True, engines_running=True,
                         throttle_pct=95.0, ambient_temperature_c=15.0,
                         ambient_humidity_pct=40.0)
    assert not fo._carb_heat_wanted(roll)


def test_accretion_warns_earlier_on_an_uncertified_airframe():
    fo = _fo(_aircraft_class="piston_single")
    fo.session.aircraft = SimpleNamespace(type_code="C172", title="")
    icing = AircraftState(on_ground=False, engines_running=True,
                          structural_ice_pct=30.0,
                          ambient_temperature_c=-3.0)
    fo._watch_icing(icing, 10.0)
    keys = _emit_keys(fo)
    assert "autonomous.weather.ice" in keys
    # A known-ice-certified type stays quiet until 40%.
    fo2 = _fo(_aircraft_class="turboprop_single")
    fo2.session.aircraft = SimpleNamespace(type_code="TBM9", title="")
    fo2._watch_icing(AircraftState(on_ground=False, engines_running=True,
                                   structural_ice_pct=30.0), 10.0)
    assert "autonomous.weather.ice" not in _emit_keys(fo2)


# -- the rotorcraft protections ----------------------------------------------


def _heli(stage="cruise", type_code=""):
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import AutonomousState
    from meridianatc.atc.autonomous_heli import (
        DEFAULT_HELI_PROFILE,
        HeliState,
        RotorcraftFlight,
    )
    owner = SimpleNamespace(
        config=SimpleNamespace(
            immersion=SimpleNamespace(autonomous_pid=False)),
        session=SimpleNamespace(
            aircraft=SimpleNamespace(type_code=type_code)),
        sim=None, get_state=lambda: None, crew=Mock(), on_event=Mock(),
        state=AutonomousState(),
        _send_event=Mock(return_value=True),
        _emit=Mock(), _engines_out=lambda state: (0, 0),
        _use_pids=False,
        _envelope_protection=True,
        _set_error=Mock())
    heli = RotorcraftFlight(owner)
    import time as _time
    heli.state = HeliState(stage=stage, coll_trim=50.0,
                           stage_entered_at=_time.monotonic())
    heli._profile = DEFAULT_HELI_PROFILE
    heli._limits = limits_for(type_code, "helicopter")
    return heli


def _sent(heli):
    return heli.owner._send_event.call_args_list


def _emit_keys_heli(heli):
    return [c.kwargs.get("key") for c in heli.owner._emit.call_args_list]


def test_heli_eases_collective_at_the_types_torque_line():
    from meridianatc.atc.autonomous_heli import COLLECTIVE_SET
    # A Bell 407 at 105% torque: over its 100% takeoff figure.
    heli = _heli(type_code="B407")
    heli._protect(AircraftState(on_ground=False, eng_torque_pct=105.0),
                  vs=0.0, ias=80.0, agl=1000.0)
    sent = _sent(heli)
    assert any(c.args[0] == COLLECTIVE_SET and c.args[1] < 8192
               for c in sent)
    assert "autonomous.rotorcraft.torque" in _emit_keys_heli(heli)


def test_heli_torque_window_is_per_type_not_a_flat_95():
    from meridianatc.atc.autonomous_heli import COLLECTIVE_SET
    # 90% torque: over the EC135's 75% takeoff figure...
    heli = _heli(type_code="EC35")
    heli._protect(AircraftState(on_ground=False, eng_torque_pct=90.0),
                  vs=0.0, ias=80.0, agl=1000.0)
    sent = _sent(heli)
    assert any(c.args[0] == COLLECTIVE_SET and c.args[1] < 0
               for c in sent)
    # ...but inside the Bell 412's window (red line 100%).
    heli2 = _heli(type_code="B412")
    heli2._protect(AircraftState(on_ground=False, eng_torque_pct=90.0),
                   vs=0.0, ias=80.0, agl=1000.0)
    sent2 = _sent(heli2)
    assert not any(c.args[0] == COLLECTIVE_SET for c in sent2)


def test_heli_ignores_a_torque_reading_that_is_not_a_percent():
    from meridianatc.atc.autonomous_heli import COLLECTIVE_SET
    heli = _heli(type_code="B407")
    heli._protect(AircraftState(on_ground=False, eng_torque_pct=900.0),
                  vs=0.0, ias=80.0, agl=1000.0)
    sent = _sent(heli)
    assert not any(c.args[0] == COLLECTIVE_SET for c in sent)


def test_rotor_droop_unloads_and_overspeed_loads():
    from meridianatc.atc.autonomous_heli import COLLECTIVE_SET
    # S-76 drooping to 95%: under its 100% power-on floor.
    heli = _heli(type_code="S76")
    heli._protect(AircraftState(on_ground=False, rotor_rpm_pct=95.0),
                  vs=0.0, ias=80.0, agl=1000.0)
    sent = _sent(heli)
    assert any(c.args[0] == COLLECTIVE_SET and c.args[1] < 0
               for c in sent)
    assert "autonomous.rotorcraft.droop" in _emit_keys_heli(heli)
    # B407 winding to 106%: over its 104% ceiling -- collective up. The
    # collective axis is signed: under the floor is below zero, over the
    # ceiling above it.
    heli2 = _heli(type_code="B407")
    heli2._protect(AircraftState(on_ground=False, rotor_rpm_pct=106.0),
                   vs=0.0, ias=80.0, agl=1000.0)
    sent2 = _sent(heli2)
    assert any(c.args[0] == COLLECTIVE_SET and c.args[1] > 0
               for c in sent2)
    assert "autonomous.rotorcraft.overspeed" in _emit_keys_heli(heli2)


def test_rotor_watch_stands_down_in_the_autorotation():
    from meridianatc.atc.autonomous_heli import COLLECTIVE_SET
    heli = _heli(stage="autorotate", type_code="B407")
    heli._protect(AircraftState(on_ground=False, rotor_rpm_pct=95.0),
                  vs=-1500.0, ias=65.0, agl=500.0)
    sent = _sent(heli)
    # The autorotation unloads the rotor on purpose: no droop call.
    assert "autonomous.rotorcraft.droop" not in _emit_keys_heli(heli)
    assert not any(c.args[0] == COLLECTIVE_SET for c in sent)


def test_departure_accelerates_level_before_it_climbs():
    heli = _heli()
    heli._fly = Mock()
    heli._departure(AircraftState(on_ground=False, altitude_agl_ft=30.0,
                                  indicated_airspeed_kt=10.0))
    assert heli._fly.call_args.kwargs["vs_target"] == 0.0
    heli._departure(AircraftState(on_ground=False, altitude_agl_ft=60.0,
                                  indicated_airspeed_kt=45.0))
    assert heli._fly.call_args.kwargs["vs_target"] == \
        heli._profile.climb_vs_fpm


def test_departure_respects_the_takeoff_power_airspeed_ceiling():
    heli = _heli(type_code="H125")
    heli._fly = Mock()
    heli._departure(AircraftState(on_ground=False, altitude_agl_ft=30.0,
                                  indicated_airspeed_kt=10.0))
    # The H125's takeoff power is published to 40 KIAS.
    assert heli._fly.call_args.kwargs["ias_target"] == pytest.approx(40.0)


def test_departure_climbs_once_the_capped_speed_is_reached():
    """The H125's 40 kt ceiling sits under the transition figure's 0.9
    gate: a gate on the transition speed would hold the accel level for
    ever -- the climb begins when the *ceiling* is reached."""
    heli = _heli(type_code="H125")
    heli._fly = Mock()
    heli._departure(AircraftState(on_ground=False, altitude_agl_ft=30.0,
                                  indicated_airspeed_kt=42.0))
    assert heli._fly.call_args.kwargs["vs_target"] == \
        heli._profile.climb_vs_fpm


def test_approach_walks_the_speed_down_with_the_height():
    heli = _heli()
    heli.state.ias_approach_kt = 60.0
    heli._fly = Mock()
    # 150 ft AGL inside the ramp: halfway between the approach figure
    # and the 20 kt walk speed.
    heli._approach(AircraftState(on_ground=False, altitude_agl_ft=150.0,
                                 indicated_airspeed_kt=55.0))
    kw = heli._fly.call_args.kwargs
    assert kw["ias_target"] == pytest.approx(40.0)


def test_autorotation_flies_the_types_auto_speed_and_flare():
    from meridianatc.atc.autonomous_heli import COLLECTIVE_SET, HELI_THROTTLE_SET
    heli = _heli(stage="autorotate", type_code="R44")
    heli._autorotate(AircraftState(on_ground=False, altitude_agl_ft=500.0,
                                   indicated_airspeed_kt=65.0,
                                   rotor_rpm_pct=0.0))
    sent = _sent(heli)
    # Throttle chopped, collective near the bottom (~10%) -- deep into
    # the bottom half of a signed axis.
    assert any(c.args[0] == HELI_THROTTLE_SET for c in sent)
    coll = [c.args[1] for c in sent if c.args[0] == COLLECTIVE_SET]
    assert coll and coll[-1] < -8000


def test_shutdown_idles_the_throttle_for_the_cooldown():
    """A cool-down run at cruise throttle cools nothing: the window
    opens with the lever at idle, which is also what lets the ITT
    figure below ever be reached."""
    from meridianatc.atc.autonomous_heli import HELI_THROTTLE_SET
    heli = _heli(stage="shutdown", type_code="B06")
    # shutdown_at untouched: the first pass opens the window.
    heli._shutdown(AircraftState(on_ground=True, engines_running=True,
                                 rotor_rpm_pct=60.0))
    sent = _sent(heli)
    # Idle on a signed throttle axis is the bottom of its travel.
    assert any(c.args[0] == HELI_THROTTLE_SET and c.args[1] == -16383
               for c in sent)


def test_r22_governor_card_matches_the_poh_powered_states():
    """The R22 POH starts with the governor ON and the shutdown flow
    leaves it OFF; neither item is the governor's RPM target."""
    import json
    from pathlib import Path
    data = json.loads(Path(
        "meridianatc/atc/checklists/types/xplane-robinson-r22.json"
    ).read_text(encoding="utf-8"))
    states = {}
    for phase in ("before_start", "after_start", "shutdown"):
        for item in data["checklists"][phase]["items"]:
            if item.get("challenge") == "Governor":
                states[phase] = (
                    item["response"], item["check"]["var"],
                    item["check"]["value"])
    assert states == {
        "before_start": ("on", "rotor_governor_on", True),
        "after_start": ("on", "rotor_governor_on", True),
        "shutdown": ("off", "rotor_governor_on", False),
    }


def test_shutdown_starts_the_type_shutdown_checklist():
    """The rotorcraft state machine used to cool and cut the engine
    without ever reading the type's shutdown card; the card is owed at
    the start of shutdown, chained leg or last leg alike."""
    heli = _heli(stage="shutdown", type_code="R22")
    runner = Mock()
    runner.waiting = False
    heli.crew.checklists = runner
    heli._shutdown(AircraftState(on_ground=True, engines_running=True,
                                 rotor_rpm_pct=60.0))
    runner.start.assert_called_with("shutdown")
    assert "shutdown" in heli.owner.state.checklists_completed


def test_shutdown_cools_down_on_itt_where_published():
    import time

    from meridianatc.atc.autonomous_heli import ENGINE_STOP
    heli = _heli(stage="shutdown", type_code="B06")
    heli.state.shutdown_at = time.monotonic() - 5.0   # young cooldown
    # MGT under the 540 C cool-down figure: out of the wait early.
    heli._shutdown(AircraftState(on_ground=True, engines_running=True,
                                 rotor_rpm_pct=60.0, eng_itt_c=500.0))
    sent = _sent(heli)
    assert any(c.args[0] == ENGINE_STOP for c in sent)
    # A hot engine on the same young clock is not shut down yet.
    heli2 = _heli(stage="shutdown", type_code="B06")
    heli2.state.shutdown_at = time.monotonic() - 5.0
    heli2._shutdown(AircraftState(on_ground=True, engines_running=True,
                                  rotor_rpm_pct=60.0, eng_itt_c=700.0))
    sent2 = _sent(heli2)
    assert not any(c.args[0] == ENGINE_STOP for c in sent2)


def test_pressurisation_is_inferred_from_type_when_no_sim_engine_data():
    """A manual B739 is still a pressurised jet even though the no-sim
    source cannot publish ENGINE TYPE; otherwise the oxygen physiology cap
    wrongly reduces a filed FL350 to 10,000 ft."""
    from meridianatc.atc.autonomous.core import AutonomousFlight
    fo = AutonomousFlight.__new__(AutonomousFlight)
    assert fo._pressurized_airframe(SimpleNamespace(
        type_code="B739", engine_type=-1, pressurization_capable=False,
        pressurization_diff_psi=0.0, altitude_ft=0.0,
        pressure_altitude_ft=0.0, cabin_altitude_ft=0.0))
    assert not fo._pressurized_airframe(SimpleNamespace(
        type_code="BE58", engine_type=-1, pressurization_capable=False,
        pressurization_diff_psi=0.0, altitude_ft=0.0,
        pressure_altitude_ft=0.0, cabin_altitude_ft=0.0))
    assert not fo._pressurized_airframe(SimpleNamespace(
        type_code="B739", engine_type=0, pressurization_capable=False,
        pressurization_diff_psi=0.0, altitude_ft=0.0,
        pressure_altitude_ft=0.0, cabin_altitude_ft=0.0))


def test_physiology_cap_uses_filed_type_when_manual_state_has_none():
    """`/flight aircraft_type=B739` is enough evidence that the manual
    aeroplane is pressurised; an empty manual TYPE CODE must not silently
    mean piston."""
    from meridianatc.atc.autonomous.core import AutonomousFlight
    from meridianatc.sim.state import AircraftState
    fo = AutonomousFlight.__new__(AutonomousFlight)
    session = SimpleNamespace(
        aircraft=SimpleNamespace(type_code="B739"),
        cruise_altitude_ft=35000.0)
    fo._cap_cruise_for_physiology(session, AircraftState())
    assert session.cruise_altitude_ft == 35000.0


def test_cowl_flaps_stay_open_at_high_power_and_low_airspeed():
    """2026-11-28 LECO: full power at about ninety knots cooled only
    because the flaps were open. As soon as the hysteresis let them
    close, the same CHT warning came back; the cooling demand belongs
    to power/airflow as well as phase."""
    pytest.importorskip("scipy")
    from meridianatc.atc.autonomous import AutonomousPhase
    fo = _fo()
    fo.session.aircraft.type_code = "BE58"
    fo._aircraft_class = "piston_twin"
    fo.state.phase = AutonomousPhase.DESCENT
    state = AircraftState(on_ground=False, indicated_airspeed_kt=95.0,
                          throttle_pct=100.0, eng_cht_c=200.0)
    assert fo._cowl_flap_wanted(state) == pytest.approx(1.0)
    state.throttle_pct = 20.0
    assert fo._cowl_flap_wanted(state) == pytest.approx(0.0)
