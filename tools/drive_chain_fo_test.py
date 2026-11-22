"""Chained First Officer flights, driven end-to-end with no simulator.

TEST-ONLY driver derived from tools/drive_full_fo.py: same real Engine
+ scripted Plant, but with chained flights enabled and one planned
second leg EBBR -> EBCI. Leg one flies ELLX -> EBBR; on the stand the
FO turns the aeroplane around and flies the chained leg itself, then
runs out of plans and shuts down.

The descent lines print dist / 3-degree path / vs so the continuous
descent is visible in the transcript.
"""
import logging
import math
import sys
import time
from pathlib import Path

logging.basicConfig(level=logging.INFO,
                    format="LOG %(name)s: %(message)s")

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from flight_plant import Plant  # noqa: E402

from meridianatc.atc import approach_flight  # noqa: E402
from meridianatc.atc.autonomous._shared import AutonomousPhase  # noqa: E402
from meridianatc.config import Config  # noqa: E402
from meridianatc.engine import Engine  # noqa: E402
from meridianatc.navdata.db import NavDB  # noqa: E402

LOG = []


def note(*a):
    line = " ".join(str(x) for x in a)
    LOG.append(line)
    print(line, flush=True)


def sink(kind, text, **kw):
    if kind in ("atc", "pilot", "info", "callout", "cue", "line",
                "checklist", "notice", "error"):
        note(f"  [{kind}] {text}")


class SimAdapter:
    """The transport boundary: answers _send/_send-like calls by moving
    the plant, and passes radio calls to the ManualSource so the tuned
    frequency and squawk read true on the next state refresh."""

    connected = True

    def __init__(self, plant, source):
        self.p = plant
        self._source = source
        self.writes = []

    def _send(self, name, value):
        p = self.p
        n = name.decode() if isinstance(name, bytes) else str(name)
        self.writes.append((n, value))
        v = float(value)
        if n in ("AP_MASTER_ON", "AUTOPILOT_ON"):
            p.ap = True
        elif n in ("AP_MASTER_OFF", "AUTOPILOT_OFF"):
            p.ap = False
        elif n == "AP_HDG_VAR_SET":
            p.hdg_bug = v
        elif n == "AP_HEADING_BUG_SET_EX1":
            p.hdg_bug = v * 360.0 / 16384.0
        elif n == "HEADING_BUG_SET":
            p.hdg_bug = v
        elif n == "AP_ALT_VAR_SET_ENGLISH":
            p.alt_bug = v
        elif n == "AP_VS_VAR_SET_ENGLISH":
            p.vs_bug = v
        elif n == "AP_VS_ON":
            p.vs_mode = True
        elif n == "AP_VS_OFF":
            p.vs_mode = False
        elif n == "AP_SPD_VAR_SET":
            p.spd_bug = v
        elif n == "AP_SPD_VAR_SET_EX1":
            p.spd_bug = v / 100.0
        elif n == "ELEVATOR_SET":
            p.elev_cmd = v / 16384.0
        elif n == "AILERON_SET":
            p.aileron_cmd = v / 16384.0
        elif n == "AXIS_RUDDER_SET":
            p.rudder_cmd = v / 16384.0
        elif n in ("STEERING_SET", "AXIS_STEERING_SET"):
            p.steer_cmd = v / 16384.0
        elif n == "THROTTLE_SET":
            p.lever = max(0.0, v / 16383.0)
        elif n.startswith("AXIS_THROTTLE"):
            p.lever = max(0.0, (v + 16383.0) / 32766.0)
        elif n == "THROTTLE_FULL":
            p.lever = 1.0
        elif n in ("AXIS_LEFT_BRAKE_SET", "AXIS_RIGHT_BRAKE_SET"):
            p.brake = max(p.brake, (v + 16383.0) / 32766.0)
        elif n == "BRAKES":
            p.brake = 1.0
        elif n == "PARKING_BRAKE_SET":
            p.parking = v >= 0.5
        elif n == "FLAPS_SET":
            p.flaps = int(round(v / 16383.0 * 3.0))
        elif n == "AXIS_FLAPS_SET":
            p.flaps = int(round((v + 16383.0) / 32766.0 * 3.0))
        elif n in ("GEAR_DOWN",):
            p.gear = True
        elif n in ("GEAR_UP",):
            p.gear = False
        elif n == "GEAR_SET":
            p.gear = v >= 0.5
        elif n == "TOGGLE_PUSHBACK":
            p.pushing = True
        elif n == "ENGINE_AUTO_SHUTDOWN":
            p.engines_running = False
        elif n == "ENGINE_AUTO_START":
            p.engines_running = True
        return True

    def set_variable(self, name, value, *a, **k):
        self.writes.append(("var:" + str(name), value))
        try:
            v = float(value)
            if "THROTTLE LEVER" in str(name):
                self.p.lever = max(0.0, min(1.0, v))
        except (TypeError, ValueError):
            pass
        return True

    def set_com(self, mhz, radio=1, standby=False):
        return self._source.set_com(mhz, radio=radio, standby=standby)

    def swap_com(self, radio=1):
        return self._source.swap_com(radio)

    def set_squawk(self, code):
        return self._source.set_squawk(code)

    def set_nav(self, mhz, radio=1):
        return self._source.set_nav(mhz, radio=radio)

    def set_fuel(self, gallons):
        return True

    def pushback(self):
        self.p.pushing = True
        return True

    def load_fms_plan(self, plan):
        self.writes.append(("fms", str(plan)[:60]))
        return True

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        def _log(*a, **k):
            self.writes.append(("call:" + name, (a, k)))
            return True
        return _log


def sync_state(source, p):
    st = p.state()
    source.update(
        latitude=st.latitude, longitude=st.longitude,
        altitude_ft=st.altitude_ft, indicated_altitude_ft=st.altitude_ft,
        altitude_agl_ft=st.altitude_agl_ft,
        heading_true=st.heading_true, heading_magnetic=st.heading_magnetic,
        track_true=st.track_true, ground_speed_kt=st.ground_speed_kt,
        indicated_airspeed_kt=st.indicated_airspeed_kt,
        true_airspeed_kt=st.true_airspeed_kt,
        vertical_speed_fpm=st.vertical_speed_fpm,
        on_ground=st.on_ground, gear_down=st.gear_down,
        flaps_index=st.flaps_index, autopilot=st.autopilot,
        engines_running=p.engines_running, engine_count=2,
        # Tanks read full so the turnaround's fuel check covers the leg.
        fuel_total_gal=6400.0, fuel_capacity_gal=6400.0,
    )


def dt_for(phase):
    if phase in (AutonomousPhase.APPROACH, AutonomousPhase.LANDING,
                 AutonomousPhase.LINE_UP, AutonomousPhase.TAKEOFF):
        return 0.35
    if phase in (AutonomousPhase.TAXI_OUT, AutonomousPhase.TAXI_IN,
                 AutonomousPhase.HOLDING_SHORT, AutonomousPhase.PUSHBACK):
        return 0.6
    return 1.5


def main():
    config = Config.load(str(ROOT / "config.test.yaml"))
    # the scripted plant answers explicit bug writes (hdg/vs/alt/spd),
    # not managed AP modes like NAV/FLCH it cannot emulate.
    config.immersion.autonomous_ap_mode = "direct"
    config.immersion.autonomous_chained_flights = True
    config.immersion.autonomous_chain_delay_s = 30.0
    config.flight.callsign = "BEL8KA"
    config.flight.aircraft_type = "A320"
    config.flight.departure = "ELLX"
    config.flight.destination = "EBBR"
    config.flight.cruise_altitude_ft = 9000.0
    engine = Engine(config, NavDB(), sink)
    engine.warm()
    engine.start(connect_sim=False)
    time.sleep(1.5)

    p = Plant(49.6268, 6.2121, 1234.0, 0.0, 0.0,
              field_elev_ft=1234.0, stall_kt=115.0, autothrottle=True)
    p.vref = 145.0
    p.on_ground = True
    p.type_code = "A320"
    p.pushing = False
    p.parking = True
    p.aileron_cmd = 0.0
    p.engines_running = True

    engine.source.update(source="simconnect", engines_running=True,
                         engine_count=2, type_code="A320")
    sync_state(engine.source, p)
    engine.refresh_state()
    engine.begin_session()

    fo = engine.autonomous_flight
    # the live ADS-B feed put a real aircraft on the departure runway,
    # and the FO correctly held for it. for the scripted arc emulate a
    # quiet airport: nobody in the surface picture to yield to.
    fo._surface_contacts = lambda: []
    # the tower keeps its own picture: live ADS-B arrivals at ELLX hold
    # our departure for the gaps between them, and a quiet circuit is
    # what the scripted arc wants to fly.
    engine.brain.contacts = lambda: []
    adapter = SimAdapter(p, engine.source)
    fo.sim = adapter

    ok, msg = engine.engage_autonomous()
    note("engage:", ok, msg or "(ok)")
    if not ok:
        return 2

    # Leg two is planned, not picked: EBBR -> EBCI. After it the chain
    # runs out -- the picker is stubbed so "no destination" ends the
    # day rather than launching a random third leg.
    fo.set_chain_flights([{
        "origin": "EBBR", "destination": "EBCI",
        "origin_name": "Brussels Airport",
        "destination_name": "Brussels South Charleroi Airport",
        "distance_nm": 26, "cruise_altitude_ft": 6000,
        "ifr": False, "aircraft_type": "A320",
    }])
    fo._select_next_destination = lambda: None

    last_phase = None
    last_leg = 0
    t0 = time.monotonic()
    timeout = 3600.0
    try:
        while time.monotonic() - t0 < timeout:
            phase = fo.state.phase
            leg = fo.state.chain_flight_index
            if leg != last_leg:
                note(f"=== CHAIN leg index {leg} "
                     f"t={time.monotonic()-t0:.0f}s")
                last_leg = leg
                # Per-leg plant fixtures: the tug walk and the field
                # elevation the AGL is measured against belong to the
                # aerodrome the leg is at.
                p.pushing = False
                p.push_delay = 0
                p.push_count = 0
                p.pushed_done = False
            if phase != last_phase:
                note(f"=== phase {phase} sub={fo.state.sub} "
                     f"t={time.monotonic()-t0:.0f}s pos={p.lat:.4f},{p.lon:.4f} "
                     f"alt={p.alt:.0f} ias={p.ias:.0f} hdg={p.hdg:.0f}")
                last_phase = phase
            if int(time.monotonic() - t0) % 15 == 0:
                s = fo.state
                try:
                    blk = fo._runway_blocked()
                    inb = fo._runway_inbound()
                    ncon = len(fo._surface_contacts())
                    ses = fo.session
                    extra = (f"cto={ses.cleared_takeoff} rej={fo.state.rejected} "
                             f"sph={ses.phase} aligned={fo.state.aligned} "
                             f"eng={p.engines_running} "
                             f"pend={getattr(getattr(ses,'pending',None),'readback_required',None)}/"
                             f"{getattr(getattr(ses,'pending',None),'read_back',None)}")
                    if phase in (AutonomousPhase.DESCENT,):
                        dist = fo._distance_to_destination_nm(p.state())
                        elev = fo._field_elevation(s)
                        path = (elev + approach_flight.TCH_FT
                                + (dist or 0.0) * approach_flight.FT_PER_NM)
                        extra += (f" CDA dist={dist and f'{dist:.1f}'}"
                                  f" path={path:.0f}")
                    if phase in (AutonomousPhase.APPROACH,
                                 AutonomousPhase.LANDING):
                        state = fo.state
                        try:
                            off = fo._centreline_offset_m(state)
                        except Exception:
                            off = "?"
                        try:
                            vapp = f"{fo._vapp(state):.0f}"
                        except Exception:
                            vapp = "?"
                        try:
                            to_go = math.hypot(
                                (state.runway_lat - p.lat) * 111320.0,
                                (state.runway_lon - p.lon) * 111320.0
                                * math.cos(math.radians(p.lat)))
                        except Exception:
                            to_go = -1.0
                        print(
                            f"    STAB ias={p.ias:.0f} "
                            f"vapp={vapp} vs={p.vs:.0f} "
                            f"agl={(p.alt - p.elev):.0f} off={off} "
                            f"to_go={to_go:.0f} stab={state.stabilized} "
                            f"hdg={getattr(state, 'heading_true', '?')} "
                            f"rhdg={state.runway_heading} "
                            f"dest={getattr(getattr(ses, 'destination', None), 'ident', ses.destination)} "
                            f"arr={getattr(ses, 'arrival_runway', '?')}", flush=True)
                except Exception as e:
                    blk = inb = ncon = extra = f"?{e}"
                note(f"  .. {phase}/{s.sub} pos={p.lat:.4f},{p.lon:.4f} "
                     f"ias={p.ias:.0f} alt={p.alt:.0f} vs={p.vs:.0f} "
                     f"hdg={p.hdg:.0f} "
                     f"lever={p.lever:.2f} brake={p.brake:.2f} park={p.parking} "
                     f"blk={blk} inb={inb} contacts={ncon} {extra} "
                     f"writes={len(adapter.writes)}")
            if phase in (AutonomousPhase.COMPLETE, AutonomousPhase.ERROR):
                break
            # the plant's AGL is measured against the field it left:
            # once the descent starts, the ground it must land on is
            # the destination's. field elevation comes from the nav data
            # and follows the leg the session is on.
            if phase in (AutonomousPhase.DESCENT, AutonomousPhase.APPROACH,
                         AutonomousPhase.LANDING):
                dest = getattr(fo.session, "destination", None)
                ident = getattr(dest, "ident", None) or "EBBR"
                if getattr(p, "elev_dest", None) != ident:
                    p.elev = float(getattr(
                        engine.navdb.airport(ident), "elev_ft", 175.0)
                        or 175.0)
                    p.elev_dest = ident
            if getattr(fo.session, "held_for_landing", ""):
                fo.session.held_for_landing = ""
            # the ramp approved the push but in no-sim nothing moves us:
            # emulate the tug ourselves a beat after the request lands
            if fo.state.push_requested and not p.pushing \
                    and not getattr(p, "pushed_done", False):
                p.push_delay = getattr(p, "push_delay", 0) + 1
                if p.push_delay > 20:          # ~2.4 s for the tug to attach
                    p.pushing = True
            if p.pushing:
                # a tug's 2 kt backward walk, ~10 s of it. the plant's
                # ground speed is ias on wheels -- the FO reads >0.8 kt
                # as "the tug is walking us".
                p.ias = 2.0
                from meridianatc.navdata.build import project
                p.lat, p.lon = project(p.lat, p.lon,
                                       (p.hdg + 180.0) % 360.0,
                                       2.0 * 0.12 / 3600.0)
                p.push_count = getattr(p, "push_count", 0) + 1
                if p.push_count > 60:
                    p.pushing = False
                    p.pushed_done = True
                    p.ias = 0.0
            elif p.on_ground:
                # the plant's ground model is a bare integrator (no
                # drag): the FO's speed loop surges then stomps the
                # brakes and creeps. give it car-like dynamics instead:
                # lever pulls, drag bites, brakes bite harder.
                dt = dt_for(phase)
                accel = p.lever * 10.0 - 0.05 * p.ias - p.brake * 8.0 \
                    - (2.0 if p.parking else 0.0)
                p.ias = max(0.0, p.ias + accel * dt)
                # track the heading the FO's steering law is asking
                # for -- a synthetic nosewheel servo kept hunting into
                # limit cycles; the FO still picks the target. On the
                # takeoff and landing rolls the FO steers with the pedal
                # -- _steer_want holds its last taxi value there -- so
                # past taxi speed the rudder command is what moves the
                # nose, at an authority that decays with groundspeed
                # like a real nosewheel's.
                want_hdg = getattr(fo, "_steer_want", None)
                rolling = phase in (AutonomousPhase.TAKEOFF,
                                    AutonomousPhase.LANDING)
                if want_hdg is not None and p.ias > 0.5 and not rolling:
                    herr = (want_hdg - p.hdg + 540.0) % 360.0 - 180.0
                    rate = min(12.0, abs(herr) * 0.5)
                    p.hdg += max(-rate, min(rate, herr)) * dt
                else:
                    # Nosewheel steering dominant at taxi speed, rudder
                    # authority fading in with airflow the way the FO's
                    # split writes mean it.
                    nws = getattr(p, "steer_cmd", 0.0) * 18.0 \
                        * min(1.0, p.ias / 10.0)
                    rud = p.rudder_cmd * 4.0 \
                        * min(1.0, p.ias / 60.0)
                    p.hdg += (nws + rud) * dt
                p.vs = 0.0
                import math as _m
                trk = _m.degrees(_m.atan2(p.ias * _m.sin(_m.radians(p.hdg)),
                                        p.ias * _m.cos(_m.radians(p.hdg)))) % 360.0
                if p.ias > 0.01:
                    from meridianatc.navdata.build import project
                    p.lat, p.lon = project(p.lat, p.lon, trk,
                                           p.ias * dt / 3600.0)
                # the wheels come unstuck the way they do: back pressure
                # at flying speed. the FO's rotate writes the elevator;
                # past Vr it answers.
                vr = getattr(fo.state, "rotate_speed_kt", None) \
                    or getattr(fo.state, "vr_kt", None) or 135.0
                if p.ias >= vr * 0.95 and p.elev_cmd > 0.02:
                    p.on_ground = False
                    p.vs = 500.0
            else:
                p.step(dt_for(phase))
            p.brake = max(0.0, p.brake - 0.5)   # brakes bleed off
            if not p.pushing and p.parking:
                p.ias = 0.0
            sync_state(engine.source, p)
            time.sleep(0.12)
    finally:
        engine.disengage_autonomous()
        engine.stop()

    note(f"\nDONE t={time.monotonic()-t0:.0f}s phase={fo.state.phase} "
         f"leg={fo.state.chain_flight_index} "
         f"pos={p.lat:.4f},{p.lon:.4f} alt={p.alt:.0f} "
         f"td_fpm={p.touchdown_fpm} writes={len(adapter.writes)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
