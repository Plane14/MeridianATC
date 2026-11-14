"""Watch the user aircraft on the ground from a second SimConnect client.

Prints one line per poll: time, lat, lon, groundspeed, heading, rudder,
throttle lever, toe brakes, parking brake. Stdout, so it can be piped
into a file and read afterwards.
"""
import time

from SimConnect import AircraftRequests, SimConnect

sm = SimConnect()
aq = AircraftRequests(sm, _time=400)

KEYS = [
    ("lat", "PLANE_LATITUDE"),
    ("lon", "PLANE_LONGITUDE"),
    ("gs", "GROUND_VELOCITY"),
    ("hdg", "PLANE_HEADING_DEGREES_TRUE"),
    ("rud", "RUDDER_POSITION"),
    ("ped", "RUDDER_PEDAL_POSITION"),
    ("thr", "GENERAL_ENG_THROTTLE_LEVER_POSITION:1"),
    ("brkL", "BRAKE_LEFT_POSITION"),
    ("brkR", "BRAKE_RIGHT_POSITION"),
    ("park", "BRAKE_PARKING_POSITION"),
    ("ong", "SIM_ON_GROUND"),
]

reqs = {name: aq.find(key) for name, key in KEYS}
missing = [name for name, r in reqs.items() if r is None]
if missing:
    print("missing keys:", {n: k for n, k in KEYS.items() if n in missing})

t0 = time.monotonic()
while True:
    vals = {}
    for name, r in reqs.items():
        if r is None:
            vals[name] = None
        else:
            v = r.get()
            vals[name] = v.get() if hasattr(v, "get") else v
    t = time.monotonic() - t0
    print(
        f"{t:7.1f} lat={vals['lat']:.6f} lon={vals['lon']:.6f} "
        f"gs={vals['gs'] or 0:5.1f} hdg={vals['hdg'] or 0:6.1f} "
        f"rud={vals['rud']} ped={vals['ped']} "
        f"thr={vals['thr']} brkL={vals['brkL']} brkR={vals['brkR']} "
        f"park={vals['park']} ong={vals['ong']}",
        flush=True)
    time.sleep(0.5)
