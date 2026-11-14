"""Live ground watch: position, speed, controls, and how far the
aircraft is from the nearest taxiway segment of its own airport.

Second SimConnect client -- runs beside the app, changes nothing.
"""
import math
import sys
import time

sys.path.insert(0, __file__.rsplit("\\tools\\", 1)[0])

from SimConnect import AircraftRequests, SimConnect

IDENT = sys.argv[1] if len(sys.argv) > 1 else "LPPR"

from meridianatc.integrations.taxiways import SimGround  # noqa: E402

lay = SimGround().layout(IDENT)
if lay is None:
    print("no layout for", IDENT)
    sys.exit(1)
links = [(a, b, rwy) for a, outs in lay.edges.items()
         for b, _l, rwy, _n in outs]
print(f"layout {IDENT}: {len(lay.xy)} nodes, {len(links)} links", flush=True)

sm = SimConnect()
aq = AircraftRequests(sm, _time=400)
KEYS = [("lat", "PLANE_LATITUDE"), ("lon", "PLANE_LONGITUDE"),
        ("gs", "GROUND_VELOCITY"), ("hdg", "PLANE_HEADING_DEGREES_TRUE"),
        ("rud", "RUDDER_POSITION"), ("thr", "GENERAL_ENG_THROTTLE_LEVER_POSITION:1"),
        ("park", "BRAKE_PARKING_POSITION"), ("ong", "SIM_ON_GROUND")]
reqs = {n: aq.find(k) for n, k in KEYS}


def seg_dist_m(lat, lon):
    x, z = lay.to_xy(lat, lon)
    best = math.inf
    for a, b, _rwy in links:
        if a not in lay.xy or b not in lay.xy:
            continue
        ax, az = lay.xy[a]
        bx, bz = lay.xy[b]
        dx, dz = bx - ax, bz - az
        L2 = dx * dx + dz * dz
        t = 0.0 if L2 == 0 else max(
            0.0, min(1.0, ((x - ax) * dx + (z - az) * dz) / L2))
        d = math.hypot(x - (ax + t * dx), z - (az + t * dz))
        best = min(best, d)
    return best


t0 = time.monotonic()
while True:
    v = {}
    for n, r in reqs.items():
        v[n] = r.get() if r else None
        v[n] = v[n].get() if hasattr(v[n], "get") else v[n]
    d = seg_dist_m(v["lat"], v["lon"])
    print(f"{time.monotonic()-t0:7.1f} lat={v['lat']:.6f} lon={v['lon']:.6f} "
          f"gs={v['gs'] or 0:5.1f} hdg={v['hdg'] or 0:6.1f} "
          f"rud={v['rud']:+.3f} thr={v['thr']} park={v['park']} "
          f"off_twy={d:.1f}m", flush=True)
    time.sleep(0.5)
