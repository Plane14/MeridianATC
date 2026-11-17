"""Traffic information: who else is out there, from the pilot's seat.

The phrases for it existed and nothing called them. Flight following
assigned a code and never mentioned another aeroplane again, and the
controller's whole picture of the sky was the traffic this program invents:
a pilot flying with FSLTL, or with the simulator's own traffic, or with the
invented traffic switched off, had a controller blind to everything out of
the window.

So there are three things here.

:class:`Contact` is one aeroplane in the controller's picture, whatever put
it there -- an invented :class:`~meridianatc.atc.traffic.Flight`, or a
:class:`~meridianatc.integrations.simaircraft.Seen` the simulator reported --
with the few things a controller reads off a radar label.
``ControllerBrain.contacts`` builds the picture out of both.

:func:`relative` is where a contact is from the pilot: the clock position off
the pilot's own track, the distance, the height above or below, which way it
is moving across the windscreen, and whether it is still a factor or has
gone behind and is opening.

:class:`TrafficWatch` is what a radar controller giving a VFR flight
advisories does without being asked: a traffic call for anything converging
inside five miles and 1,500 ft, once, and "no factor" once it has passed --
unless the pilot has said they have it in sight. A centre does the same for
an IFR flight in cruise inside five miles and 1,000 ft; it used to say
nothing to one, because the watch was for advisories only.
"""

from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass

from ..logs import decision
from ..navdata.build import bearing_deg, haversine_nm
from .session import Instruction, station_key

# A traffic call is made for anything converging inside these (FAA JO
# 7110.65 2-1-21 leaves it to judgement; these are the figures commonly
# used for VFR advisories).
CALL_WITHIN_NM = 5.0
CALL_WITHIN_FT = 1500.0

# How long a model pick for the same contact set stands before it is asked
# again: a declined call may not suppress the advisories forever, because
# the contacts it declined are still closing.
_PICK_REASK_S = 30.0
# An IFR flight in a centre's cruise is told about traffic inside five miles
# and a thousand feet: the RVSM level next to its own, which is the only
# traffic a controller separating it would mention.
IFR_CALL_WITHIN_FT = 1000.0
# Past these, a called contact is no longer a factor whatever its geometry.
NO_FACTOR_NM = 6.0
NO_FACTOR_FT = 2000.0


@dataclass(frozen=True)
class Contact:
    """One aeroplane in the controller's picture."""

    written: str
    type_code: str
    latitude: float
    longitude: float
    altitude_ft: float
    heading: float
    ground_speed_kt: float = 0.0
    vertical_rate_fpm: float = 0.0
    on_ground: bool = False
    observed: bool = False          # the simulator's, not ours
    runway: str = ""

    def describe(self, language: str = "en") -> str:
        """"the Piper", "le Cessna": by make, as it is pointed out."""
        from .traffic import article, make_of

        return article(make_of(self.type_code), language) or {
            "fr": "le trafic", "es": "el tráfico", "de": "der Verkehr",
            "it": "il traffico", "pt": "o tráfego"}.get(language, "the traffic")

    def indefinite(self, language: str = "en") -> str:
        """"a Piper", for a traffic call, which introduces it."""
        from .traffic import make_of

        make = make_of(self.type_code)
        if not make:
            return ""
        if language == "en":
            return f"{'an' if make[0] in 'AEIOU' else 'a'} {make}"
        articles = {"fr": "un", "es": "un", "de": "eine", "it": "un", "pt": "um"}
        return f"{articles.get(language, '')} {make}".strip()


def from_flight(flight) -> Contact:
    return Contact(
        written=flight.written, type_code=flight.type_code,
        latitude=flight.latitude, longitude=flight.longitude,
        altitude_ft=float(flight.altitude_ft or 0.0),
        heading=float(getattr(flight, "heading", 0.0) or 0.0),
        ground_speed_kt=float(getattr(flight, "ground_speed_kt", 0.0) or 0.0),
        vertical_rate_fpm=float(getattr(flight, "vertical_rate_fpm", 0.0) or 0.0),
        on_ground=bool(getattr(flight, "on_ground", False)),
        observed=bool(getattr(flight, "observed", False)),
        runway=getattr(flight, "runway", "") or "")


def from_seen(one) -> Contact:
    return Contact(
        written=(one.callsign or one.registration or "").strip().upper(),
        type_code=one.type_code, latitude=one.latitude,
        longitude=one.longitude, altitude_ft=float(one.altitude_ft or 0.0),
        heading=float(one.heading or 0.0),
        ground_speed_kt=float(one.ground_speed_kt or 0.0),
        vertical_rate_fpm=float(one.vertical_speed_fpm or 0.0),
        on_ground=bool(one.on_ground), observed=True)


# ----------------------------------------------------------------------
# where it is
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class Relative:
    oclock: int
    distance_nm: float
    relative_ft: int            # its height minus the pilot's, to 100 ft
    motion: str                 # same, opposite, left_to_right, right_to_left
    factor: bool                # converging, or close enough to matter


def _angle(a: float) -> float:
    return ((a + 540.0) % 360.0) - 180.0


def relative(own_lat: float, own_lon: float, own_alt: float, own_track: float,
             lat: float, lon: float, alt: float, heading: float, *,
             own_speed: float = 100.0, speed: float = 100.0) -> Relative:
    """A contact as the pilot sees it."""
    distance = haversine_nm(own_lat, own_lon, lat, lon)
    bearing = bearing_deg(own_lat, own_lon, lat, lon)
    off = (bearing - own_track) % 360.0
    oclock = int(round(off / 30.0)) % 12 or 12
    turn = _angle(heading - own_track)
    if abs(turn) < 30.0:
        motion = "same"
    elif abs(turn) > 150.0:
        motion = "opposite"
    else:
        motion = "left_to_right" if turn > 0 else "right_to_left"
    # Closing or opening: the rate of change of the distance, from the two
    # velocities, in a flat frame centred on the pilot.
    rx = distance * math.sin(math.radians(bearing))
    ry = distance * math.cos(math.radians(bearing))
    vx = speed * math.sin(math.radians(heading)) - own_speed * math.sin(math.radians(own_track))
    vy = speed * math.cos(math.radians(heading)) - own_speed * math.cos(math.radians(own_track))
    opening = rx * vx + ry * vy > 0.0
    height = int(round((alt - own_alt) / 100.0) * 100)
    behind = oclock in (4, 5, 6, 7, 8)
    factor = (distance <= NO_FACTOR_NM and abs(height) <= NO_FACTOR_FT
              and not (opening and (behind or distance > 2.0)))
    return Relative(oclock, distance, height, motion, factor)


def closest_approach(own_lat: float, own_lon: float, own_alt_ft: float,
                     own_track: float, own_speed_kt: float,
                     lat: float, lon: float, alt_ft: float,
                     heading: float, speed_kt: float, *,
                     own_vs_fpm: float = 0.0, vs_fpm: float = 0.0,
                     horizon_s: float = 120.0) -> tuple[float, float, float]:
    """Where two aeroplanes come closest, if both keep flying as they are.

    Returns ``(miss_nm, seconds_to_it, vertical_gap_ft)``: the horizontal
    distance at the closest point of approach, how far ahead that point
    is, and how far apart they are in height when they reach it --
    straight-line extrapolation, the same arithmetic a conflict probe
    uses for the near term. Over the minute or two it is asked to look
    ahead, real tracks change little enough that the flat answer is the
    honest one.

    ``seconds_to_it`` of zero means the closest point is now or behind:
    the pair is already opening, whatever the distance reads.
    """
    distance = haversine_nm(own_lat, own_lon, lat, lon)
    bearing = bearing_deg(own_lat, own_lon, lat, lon)
    rx = distance * math.sin(math.radians(bearing))
    ry = distance * math.cos(math.radians(bearing))
    # Relative velocity in nautical miles a second: their track minus
    # ours, in the same flat frame as the position.
    vx = (speed_kt * math.sin(math.radians(heading))
          - own_speed_kt * math.sin(math.radians(own_track))) / 3600.0
    vy = (speed_kt * math.cos(math.radians(heading))
          - own_speed_kt * math.cos(math.radians(own_track))) / 3600.0
    speed2 = vx * vx + vy * vy
    if speed2 <= 1e-12:
        # Stationary relative motion: the distance is what it is.
        t = 0.0
    else:
        t = max(0.0, min(horizon_s, -(rx * vx + ry * vy) / speed2))
    miss = math.hypot(rx + vx * t, ry + vy * t)
    vertical = abs((alt_ft + vs_fpm * t / 60.0)
                   - (own_alt_ft + own_vs_fpm * t / 60.0))
    return miss, t, vertical


# ----------------------------------------------------------------------
# cautions
# ----------------------------------------------------------------------

# Precipitation in a METAR's present weather, not in the vicinity (VC) and
# not a recent (RE) one. UNVERIFIED as a basis for "runway wet": no runway
# condition report is available, so falling rain or snow at the field is
# taken as the runway being wet.
_PRECIPITATION = re.compile(
    r"[-+]?(?:TS|SH|FZ)?(?:DZ|RA|SN|SG|PL|GR|GS|UP)+")


def runway_wet(weather) -> bool:
    raw = getattr(weather, "raw", "") or ""
    if not raw:
        return False
    body = raw.upper().split(" RMK", 1)[0]
    from ..weather.metar import weather_tokens
    return any(not token.startswith(("VC", "RE"))
               and _PRECIPITATION.fullmatch(token)
               for token in weather_tokens(body))


# What the wake caution names the generating aeroplane as doing (FAA JO
# 7110.65 2-1-20: "CAUTION WAKE TURBULENCE (traffic information)", e.g.
# "heavy Boeing 747 departing runway two seven right").
_WAKE_ROLE: dict[str, dict[str, str]] = {
    "departing": {"en": "departing", "fr": "au décollage", "es": "en despegue",
                  "de": "beim Start", "it": "in decollo", "pt": "em decolagem"},
    "landing": {"en": "on final", "fr": "en finale", "es": "en final",
                "de": "im Endanflug", "it": "in finale", "pt": "em final"},
}


def wake_traffic(type_code: str, role: str = "", language: str = "en") -> str:
    """Who a wake caution names: "heavy Boeing departing".

    JO 7110.65 2-1-20 wants the generating aircraft described in the caution,
    with "heavy" or "super" in the description when it is one -- the same
    words the callsign carries, which is why they stay in English.
    """
    from .traffic import make_of

    make = make_of(type_code)
    if not make:
        return ""
    from .phraseology import wake_suffix

    words = []
    category = wake_suffix(type_code)
    if category:
        words.append(category)
    words.append(make)
    said = _WAKE_ROLE.get(role, {}).get(language, _WAKE_ROLE.get(role, {}).get("en", ""))
    if said:
        words.append(said)
    return " ".join(words)


def caution(p, *, wet: bool = False, wake: bool = False,
            wake_traffic: str = "") -> str:
    """The cautions for a clearance, in the phrasebook's words."""
    parts = []
    if wake:
        parts.append(p.caution_wake(wake_traffic))
    if wet:
        parts.append(p.runway_wet())
    return ", ".join(parts)


def needs_wake_caution(ahead_type: str, own_type: str,
                       light: bool | None = None) -> bool:
    """Who the wake caution goes to.

    A light aircraft behind a jet is the case that matters most (ICAO Doc
    4444 chapter 5 wake turbulence categories; FAA JO 7110.65 2-1-20 for the
    caution), and the same paragraph has it issued to *any* aircraft it may
    affect -- so an airliner hears it behind a heavy or a super as well
    ("caution wake turbulence, heavy Boeing seven seven"), just not behind
    one of its own category.

    ``light`` is the pilot's profile saying whether their own aeroplane is
    not an airliner; None asks the type, which did not know a C25C."""
    from . import speech as sp
    from .phraseology import wake_suffix

    if light is None:
        light = not sp.is_airliner(own_type or "")
    if not ahead_type:
        return False
    if sp.is_airliner(ahead_type) and light:
        return True
    # A bigger category ahead: the same advisory goes to a 737 behind a
    # 777, and to anything at all behind a super.
    rank = {"super": 2, "heavy": 1, "": 0}
    return rank.get(wake_suffix(ahead_type), 0) > rank.get(
        wake_suffix(own_type), 0)


# ----------------------------------------------------------------------
# the watch
# ----------------------------------------------------------------------

class TrafficWatch:
    """Traffic calls for a VFR flight receiving radar advisories, and for
    an IFR flight with a radar centre."""

    def __init__(self, brain, advisor=None):
        self.brain = brain
        # An optional llm.decisions.Decisions. With one, the model picks
        # which of the eligible contacts is called first -- or that none is
        # -- but only among contacts that already qualified for a call.
        # The call itself, and whether one is owed at all, stay here.
        self.advisor = advisor
        # One model pick per eligible set, remembered: ``look`` runs on the
        # situation loop, which may not stall behind a provider timeout.
        self._picks: dict = {}

    def look(self, session, station, state, language: str = "en"):
        from .controller import Reply
        from .final import course_over_ground

        if station is None or station.position not in ("APP", "DEP", "CTR"):
            return None
        within_ft = CALL_WITHIN_FT
        if session.ifr:
            # A centre with radar, and only a centre: departure and approach
            # sequence their IFR traffic, and an HF station over the ocean
            # cannot see any.
            from .enroute import is_oceanic

            if station.position != "CTR" or is_oceanic(station):
                return None
            within_ft = IFR_CALL_WITHIN_FT
        elif not session.flight_following:
            return None
        if not session.established_with(station) or not self.brain.staffed(station):
            return None
        if state is None or not state.has_position or state.on_ground:
            return None
        track = course_over_ground(state)
        if track is None:
            return None
        p = self.brain.for_station(station, language)
        own = (state.latitude, state.longitude, float(state.altitude_ft or 0.0), track)
        own_speed = float(getattr(state, "ground_speed_kt", 0.0) or 100.0)
        called = session.traffic_called
        eligible = []
        for contact in self.brain.contacts():
            if contact.on_ground or not contact.written:
                continue
            where = relative(*own, contact.latitude, contact.longitude,
                             contact.altitude_ft, contact.heading,
                             own_speed=own_speed,
                             speed=contact.ground_speed_kt or 100.0)
            said = called.get(contact.written, "")
            if said in ("called", "looking", "negative") and not where.factor:
                called[contact.written] = "done"
                decision("traffic", contact=contact.written, said="no_factor")
                return Reply(p.traffic_no_factor(session.aircraft), station)
            if said or not where.factor:
                continue
            if where.distance_nm > CALL_WITHIN_NM or abs(where.relative_ft) > within_ft:
                continue
            eligible.append((where, contact))
        if not eligible:
            return None
        where, contact = self._pick(eligible) or (None, None)
        if where is None:
            return None
        called[contact.written] = "called"
        session.aircraft.language = getattr(p, "language", "en")
        text = p.traffic_call(
            session.aircraft, oclock=where.oclock, distance_nm=where.distance_nm,
            motion=where.motion, heading=contact.heading,
            traffic=contact.indefinite(getattr(p, "language", "en")),
            relative_ft=where.relative_ft)
        decision("traffic", contact=contact.written, said="call",
                 oclock=where.oclock, miles=round(where.distance_nm, 1),
                 relative_ft=where.relative_ft, motion=where.motion,
                 observed=contact.observed)
        instruction = session.remember(Instruction(
            "traffic", station_key(station), text, {}, readback_required=False))
        return Reply(text, station, instruction=instruction)

    def _pick(self, eligible):
        """Which eligible contact gets the call: the closest, or the one the
        advisor names.

        The model only ever chooses among contacts that already qualified,
        and the wording comes from the phrasebook either way -- the pick is
        ordering, never whether a call may exist. ``-1`` from the advisor
        declines the call this pass.
        """
        # A canonical order -- distance, then callsign -- so the same set
        # always presents the same option list and a cached pick cannot
        # silently point at a different contact.
        eligible = sorted(
            eligible, key=lambda wc: (wc[0].distance_nm, wc[1].written))
        closest = 0
        if self.advisor is None or len(eligible) < 2:
            return eligible[closest]
        options = []
        for where, contact in eligible:
            options.append(
                f"{contact.written}: {where.oclock} o'clock, "
                f"{where.distance_nm:.1f} miles, "
                f"{abs(where.relative_ft)} feet "
                f"{'above' if where.relative_ft > 0 else 'below'}, "
                f"moving {where.motion.replace('_', ' ')}")
        signature = tuple(c.written for _w, c in eligible)
        entry = self._picks.get(signature)
        if entry is not None and time.monotonic() - entry[1] < _PICK_REASK_S:
            picked = entry[0]
        else:
            # The situation loop polls telemetry, cautions and the autopilot;
            # the model gets a second to advise, never a provider timeout.
            picked = self.advisor.pick(
                "traffic_call",
                "Several contacts qualify for a traffic advisory to this "
                "flight. Which is most worth calling out first?",
                options, allow_none=True, default=closest, wait_s=1.0)
            if len(self._picks) > 64:
                self._picks.clear()
            self._picks[signature] = (picked, time.monotonic())
        if picked == -1:
            decision("traffic", said="none", advisor="model")
            return None
        if picked is not None:
            where, contact = eligible[picked]
            decision("traffic", contact=contact.written, said="advisor_pick",
                     miles=round(where.distance_nm, 1))
            return where, contact
        return eligible[closest]


__all__ = ["Contact", "Relative", "TrafficWatch", "caution",
           "closest_approach", "from_flight", "from_seen",
           "needs_wake_caution", "relative", "runway_wet"]
