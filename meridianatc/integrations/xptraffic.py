"""Local UDP snapshot writer for the MeridianATC-XP renderer.

The plugin owns models and motion. A sent position is not confirmation: only
the plugin's ``drawn`` list admits an invented aircraft to the frequency.
"""

from __future__ import annotations

import json
import logging
import math
import select
import socket
import threading
import time
from collections import deque

log = logging.getLogger(__name__)


def _text(value, size, required=False):
    value = str(value or "")
    if ((required and not value) or len(value.encode("utf-8")) > size
            or any(ord(c) < 32 or ord(c) == 127 for c in value)):
        raise ValueError("invalid aircraft text")
    return value


def _number(value, low, high):
    result = float(value)
    if not math.isfinite(result) or not low <= result <= high:
        raise ValueError("invalid aircraft number")
    return result


def aircraft_packet(position):
    """Translate TrafficInjector.positions(), preferring its unrounded motion."""
    p = position
    motion = p.get("motion") or {}
    if not isinstance(motion, dict):
        raise ValueError("invalid aircraft motion")
    ground = bool(motion.get("on_ground", p.get("on_ground", False)))
    phase = p.get("phase", "")
    altitude = _number(motion.get("alt", p["altitude_ft"]), -2000, 100000)
    speed = _number(motion.get("gs", p.get("ground_speed_kt", 0)), 0, 2000)
    parked = phase == "parked" or bool(p.get("static"))
    runway = phase in ("final", "takeoff", "lineup", "landed", "go_around")
    gear = ground or phase in ("final", "landed")
    if not ground and phase == "takeoff" and "surface_ft" in motion:
        gear = altitude - float(motion["surface_ft"]) < 1000
    flaps = 1.0 if phase == "final" else .3 if runway else 0.0
    defaults = {"taxi": ground and not parked and speed > 0,
                "land": not parked and (runway or not ground and altitude < 10000),
                "bcn": not parked, "strb": runway or not ground, "nav": not parked}
    supplied = p.get("lights") or {}
    if not isinstance(supplied, dict):
        raise ValueError("invalid aircraft lights")
    lights = {}
    for key, default in defaults.items():
        value = supplied.get(key, default)
        if value not in (0, 1):
            raise ValueError("invalid aircraft light")
        lights[key] = int(value)
    squawk = str(p.get("squawk", "2000")).zfill(4)
    if len(squawk) != 4 or any(c not in "01234567" for c in squawk):
        raise ValueError("invalid squawk")
    return {
        "id": _text(p["callsign"], 64, True), "type": _text(p.get("type") or "C172", 4, True),
        "op": _text(p.get("operator"), 3), "livery": _text(p.get("livery"), 128),
        "reg": _text(p.get("registration", p.get("tail_number")), 32),
        "lat": _number(motion.get("lat", p["lat"]), -90, 90),
        "lon": _number(motion.get("lon", p["lon"]), -180, 180), "alt_ft": altitude,
        "hdg": _number(motion.get("heading", p["heading"]), -1e9, 1e9) % 360,
        "pitch": _number(p.get("pitch", motion.get("pitch", 0)), -90, 90),
        "roll": _number(p.get("roll", motion.get("roll", 0)), -180, 180),
        "gs_kt": speed,
        "vs_fpm": _number(motion.get("vs", p.get("vertical_rate_fpm", 0)), -30000, 30000),
        "gnd": ground, "gear": _number(p.get("gear", float(gear)), 0, 1),
        "flaps": _number(p.get("flaps", flaps), 0, 1), "lights": lights, "squawk": squawk,
    }


class XPlaneTraffic:
    HELLO_EVERY_S = 2.0
    STATUS_WITHIN_S = 3.0
    ANSWER_WITHIN_S = 10.0

    def __init__(self, enabled=True, limit=12, settle_s=3.0, smooth=True,
                 host="127.0.0.1", port=49010, timeout_s=.4,
                 live_limit=300):
        self.enabled = enabled
        self.limit = max(0, min(512, int(limit)))
        # The live aeroplanes' own budget (traffic.live.limit), on top of
        # ``limit`` rather than inside it.
        self.live_limit = max(0, min(300, int(live_limit)))
        self.smooth, self.settle_s = smooth, settle_s
        self.host, self.port = host, int(port)
        self.timeout_s = max(0, timeout_s)
        self.viewer = None
        self.lod_models = {}
        self.ai_liveries = None
        self.liveries = None
        self.on_swapped = None
        self.on_removed = None
        self._lock = threading.RLock()
        self._socket = None
        self._endpoint = None
        self._attached_at = None
        self._last_status = float("-inf")
        self._last_hello = float("-inf")
        self._status = {}
        self._wanted = {}
        self._drawn = set()
        self._unmatched = set()
        self._sequence = 0
        # Commands the plugin's key bindings send back up the same
        # channel, drained by the engine each tick.
        self._commands = deque(maxlen=64)
        # METARs the plugin answers for: station -> (asked_at, text or
        # None). A None text can mean "not asked yet" or "the sim is not
        # in real-weather mode" -- asked_at distinguishes them, and an
        # empty answer is not re-asked until the station's clock runs.
        self._metars = {}

    def _send(self, message):
        try:
            wire = json.dumps(message, separators=(",", ":"), allow_nan=False).encode("utf-8")
            self._socket.sendto(wire, self._endpoint)
            return True
        except (OSError, AttributeError, ValueError):
            return False

    def _receive(self, now):
        for _ in range(64):
            try:
                wire, sender = self._socket.recvfrom(65536)
            except (OSError, AttributeError):
                break
            if sender != self._endpoint or len(wire) > 65507:
                continue
            try:
                status = json.loads(wire)
                if (isinstance(status, dict) and status.get("v") == 1
                        and status.get("t") == "command"
                        and isinstance(status.get("cmd"), str)):
                    self._commands.append(status)
                    continue
                if (isinstance(status, dict) and status.get("v") == 1
                        and status.get("t") == "metar"
                        and isinstance(status.get("icao"), str)
                        and len(status["icao"]) <= 8
                        and isinstance(status.get("metar"), str)
                        and len(status["metar"]) <= 512):
                    # An empty report is a real answer: the sim is not in
                    # real-weather mode, which metar() reports as no text.
                    self._metars[status["icao"]] = (now, status["metar"] or None)
                    continue
                if (not isinstance(status, dict) or type(status.get("v")) is not int
                        or status["v"] != 1 or status.get("t") != "status"
                        or status.get("plugin") != "MeridianATC-XP"):
                    continue
                if any(type(status.get(k)) is not int or status[k] < 0
                       for k in ("xplane", "csl_models", "aircraft")):
                    continue
                if any(not isinstance(status.get(k), str) for k in ("version", "xpmp2")):
                    continue
                if any(not isinstance(status.get(k), list) or len(status[k]) > 512
                       or any(not isinstance(s, str) or not s or len(s.encode()) > 64
                              for s in status[k]) for k in ("drawn", "unmatched")):
                    continue
                self._status = status
                self._last_status = now
                self._drawn = set(status["drawn"]) & self._wanted.keys()
                self._unmatched = set(status["unmatched"]) & self._wanted.keys()
            except (ValueError, TypeError, UnicodeError, RecursionError):
                continue

    def on_frame(self, now=None):
        """Poll bounded, nonblocking status reads; no simulator frame hook needed."""
        with self._lock:
            if self._socket is None:
                return
            now = time.monotonic()
            self._receive(now)
            if now - self._last_hello >= self.HELLO_EVERY_S:
                self._send({"v": 1, "t": "hello"})
                self._last_hello = now

    def command(self):
        """The next command the plugin sent, or None when there is none."""
        with self._lock:
            self.on_frame()
            return self._commands.popleft() if self._commands else None

    @property
    def available(self):
        self.on_frame()
        return (self.enabled and self._socket is not None
                and time.monotonic() - self._last_status < self.STATUS_WITHIN_S)

    @property
    def drawing(self):
        return self.available and self._status.get("csl_models", 0) > 0

    @property
    def refused(self):
        return (self.enabled and self._attached_at is not None and not self.drawing
                and (bool(self._status) or time.monotonic() - self._attached_at >= self.STATUS_WITHIN_S))

    @property
    def count(self):
        return len(self._drawn & self._wanted.keys()) if self.available else 0

    def has(self, callsign):
        return self.available and callsign in self._wanted and callsign in self._drawn and callsign not in self._unmatched

    def failed(self, callsign):
        self.on_frame()
        asked = self._wanted.get(callsign)
        if asked is None:
            return False
        if self.available and callsign in self._unmatched:
            return True
        return not self.has(callsign) and time.monotonic() - asked >= self.ANSWER_WITHIN_S

    def attach(self, simconnect=None):
        with self._lock:
            if not self.enabled:
                return False
            if self._socket is not None:
                return self.drawing
            self._attached_at = time.monotonic()
            try:
                address = socket.gethostbyname(self.host)
                if address != "127.0.0.1":
                    raise ValueError("MeridianATC-XP only accepts localhost traffic")
                self._endpoint = (address, self.port)
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._socket.bind(("127.0.0.1", 0))
                self._socket.setblocking(False)
                self.on_frame()
                deadline = time.monotonic() + self.timeout_s
                while not self.available and time.monotonic() < deadline:
                    select.select([self._socket], [], [], max(0, deadline - time.monotonic()))
                return self.drawing
            except (OSError, ValueError, OverflowError):
                if self._socket is not None:
                    self._socket.close()
                self._socket = None
                log.debug("could not attach X-Plane traffic", exc_info=True)
                return False

    def followme(self, on, lat=0.0, lon=0.0, hdg=0.0):
        """Draw or withdraw the follow-me car the plugin renders.

        ``on`` keeps it on the taxiway at (lat, lon) facing ``hdg``; off it
        goes back to the depot. Latest wins -- a car has no sequence worth
        replaying.
        """
        with self._lock:
            if self._socket is None:
                return False
            try:
                return self._send({
                    "v": 1, "t": "followme", "on": int(bool(on)),
                    "lat": _number(lat, -90, 90),
                    "lon": _number(lon, -180, 180),
                    "hdg": _number(hdg, -1e9, 1e9) % 360})
            except (ValueError, TypeError, OverflowError):
                return False

    def route(self, points, on=True):
        """The cleared taxi route for the plugin's map layer.

        ``points`` is a list of (lat, lon) along the taxi path; an empty
        list or ``on`` False withdraws the line. Like the follow-me car,
        the latest send wins and the plugin drops a route that stops
        arriving.
        """
        with self._lock:
            if self._socket is None:
                return False
            try:
                pts = [[_number(lat, -90, 90), _number(lon, -180, 180)]
                       for lat, lon in points][:512]
                return self._send({"v": 1, "t": "route", "on": int(bool(on)),
                                   "pts": pts})
            except (ValueError, TypeError, OverflowError):
                return False

    def metar(self, station, ask_older_s=300.0):
        """The METAR the sim itself downloaded for ``station``, or None.

        Asks the plugin once per ``ask_older_s`` per station; the reply
        arrives on the same channel. An empty answer -- the sim is not
        in real-weather mode -- reads as None until the clock re-asks.
        """
        station = str(station or "")[:8]
        if not station:
            return None
        with self._lock:
            self.on_frame()
            if self._socket is None:
                return None
            now = time.monotonic()
            asked, text = self._metars.get(station, (float("-inf"), None))
            if now - asked >= ask_older_s:
                try:
                    self._send({"v": 1, "t": "metar", "icao": station})
                except (ValueError, TypeError, OverflowError):
                    pass
                self._metars[station] = (now, text)
            return text

    def sync(self, positions, now=None):
        with self._lock:
            self.on_frame()
            if self._socket is None:
                return
            aircraft = []
            seen = set()
            # Live aeroplanes spend their own budget (live_limit); the rest
            # spend ``limit`` -- in order, so relevance still decides.
            live_room, room = self.live_limit, self.limit
            for position in positions:
                # The budget is spent on a packet that will actually be
                # sent: validating and deduping first keeps an entry the
                # model rejects from costing a real aeroplane its place.
                try:
                    packet = aircraft_packet(position)
                except (ValueError, TypeError, KeyError, OverflowError):
                    log.debug("invalid X-Plane traffic position", exc_info=True)
                    continue
                if packet["id"] in seen:
                    continue
                if position.get("source") == "live":
                    if not live_room:
                        continue
                    live_room -= 1
                elif room:
                    room -= 1
                else:
                    continue
                aircraft.append(packet)
                seen.add(packet["id"])
            message = {"v": 1, "t": "traffic", "seq": self._sequence, "ac": aircraft}
            while len(json.dumps(message, separators=(",", ":")).encode()) > 65507:
                aircraft.pop()
            if self._send(message):
                self._sequence += 1
                current = time.monotonic()
                wanted = {a["id"]: self._wanted.get(a["id"], current) for a in aircraft}
                self._drawn.intersection_update(wanted)
                self._unmatched.intersection_update(wanted)
                self._wanted = wanted

    def detach(self):
        with self._lock:
            if self._socket is not None:
                self._send({"v": 1, "t": "clear"})
                self._socket.close()
            self._socket = None
            self._attached_at = None
            self._last_status = self._last_hello = float("-inf")
            self._status.clear()
            self._wanted.clear()
            self._drawn.clear()
            self._unmatched.clear()
            self._metars.clear()
            self._sequence = 0

    def describe(self):
        if not self.enabled:
            return "X-Plane traffic: off"
        if not self.available:
            return "X-Plane traffic: plugin not connected"
        return (f"X-Plane traffic: {self.count} drawn, {len(self._unmatched)} unmatched, "
                f"{self._status['csl_models']} CSL models")

    def object_id(self, callsign):
        return None

    def objects_of(self, callsign):
        return []

    def read_position(self, callsign):
        return None

    def height_above_ground(self, callsign):
        return None

    def resting_pitch(self, callsign):
        return 0.0

    def title_of(self, callsign):
        return ""

    def livery_of(self, callsign):
        return ""

    def level_of(self, callsign):
        return ""

    def target_level(self, callsign):
        return ""

    def swapping(self, callsign):
        return False

    def has_detailed(self, type_code, operator=""):
        return False

    def livery_matches(self, type_code, operator, callsign=""):
        return False
