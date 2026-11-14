"""Live-verify the new X-Plane reads/writes against the running sim.

Read-only first: the cooling/steering/bank-angle subscriptions plus what
the aircraft currently is (engines, on ground). Then the writes that do
not move the aeroplane: the joystick overrides latched and released, cowl
flap ratio exercised and restored, and pushback_stop sent as a no-op
(nothing is pushing). The pushback_start command is NOT sent -- a tug
actually moves the aeroplane.
"""
import socket
import struct
import sys
import time

HOST, PORT = "127.0.0.1", 49000

READS = [
    "sim/flightmodel/position/onground_any",
    "sim/flightmodel/engine/ENGN_running[0]",
    "sim/aircraft/engine/acf_CHT_is_C",
    "sim/aircraft/engine/acf_oilT_is_C",
    "sim/aircraft/limits/red_hi_CHT",
    "sim/aircraft/gear/acf_nw_steerdeg1",
    "sim/cockpit2/autopilot/bank_angle_mode",
] + [f"sim/flightmodel/engine/ENGN_CHT_c[{i}]" for i in range(4)] \
  + [f"sim/flightmodel/engine/ENGN_oil_temp_c[{i}]" for i in range(4)] \
  + [f"sim/cockpit2/engine/actuators/cowl_flap_ratio[{i}]" for i in range(4)] \
  + [f"sim/cockpit2/autopilot/bank_angles_values[{i}]" for i in range(6)]


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    values = {}

    def packet(data):
        sock.sendto(data, (HOST, PORT))

    def drain(seconds):
        end = time.time() + seconds
        while time.time() < end:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            if len(data) < 13 or data[:4] != b"RREF":
                continue
            for off in range(5, len(data) - 7, 8):
                index, value = struct.unpack_from("<if", data, off)
                if index in subs:
                    values[subs[index]] = value

    subs = {}
    for i, path in enumerate(READS):
        subs[i] = path
        packet(b"RREF\0" + struct.pack("<ii400s", 4, i, path.encode("ascii")))
    drain(1.5)

    print("== reads ==")
    for path in READS:
        print(f"  {path} = {values.get(path, 'NO ANSWER')}")

    def dref(path, value):
        packet(b"DREF\0" + struct.pack("<f500s", float(value), path.encode("ascii")))
        time.sleep(0.05)

    def command(text):
        packet(b"CMND\0" + text.encode("ascii") + b"\0")
        time.sleep(0.05)

    print("== override latch + release ==")
    for flag in ("override_joystick_pitch", "override_joystick_roll",
                 "override_joystick_heading", "override_wheel_steer",
                 "override_toe_brakes"):
        dref(f"sim/operation/override/{flag}", 1)
    # re-read the flags
    check = [f"sim/operation/override/{f}" for f in (
        "override_joystick_pitch", "override_joystick_roll",
        "override_joystick_heading", "override_wheel_steer",
        "override_toe_brakes")]
    subs2 = {}
    for i, path in enumerate(check):
        idx = 100 + i
        subs2[idx] = path
        packet(b"RREF\0" + struct.pack("<ii400s", 4, idx, path.encode("ascii")))
    subs.update(subs2)
    drain(1.0)
    for path in check:
        print(f"  {path} = {values.get(path, 'NO ANSWER')} (want 1)")
    # release them again
    for flag in ("override_joystick_pitch", "override_joystick_roll",
                 "override_joystick_heading", "override_wheel_steer",
                 "override_toe_brakes"):
        dref(f"sim/operation/override/{flag}", 0)
    drain(1.0)
    for path in check:
        print(f"  {path} = {values.get(path, 'NO ANSWER')} (want 0)")

    print("== cowl flap exercise + restore ==")
    before = values.get("sim/cockpit2/engine/actuators/cowl_flap_ratio[0]", 0.0)
    dref("sim/cockpit2/engine/actuators/cowl_flap_ratio[0]", 1.0)
    drain(1.0)
    print("  cowl[0] after open :",
          values.get("sim/cockpit2/engine/actuators/cowl_flap_ratio[0]"))
    dref("sim/cockpit2/engine/actuators/cowl_flap_ratio[0]", before)
    drain(1.0)
    print("  cowl[0] restored to:",
          values.get("sim/cockpit2/engine/actuators/cowl_flap_ratio[0]"))

    print("== pushback_stop no-op (nothing pushing) ==")
    command("sim/ground_ops/pushback_stop")
    print("  sent")

    print("== unsubscribe ==")
    for i, path in enumerate(list(subs.items())):
        idx, p = path
        packet(b"RREF\0" + struct.pack("<ii400s", 0, idx, p.encode("ascii")))


if __name__ == "__main__":
    main()
