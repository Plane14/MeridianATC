"""The other aeroplanes.

An empty frequency is the least realistic thing about a single-player ATC
simulation. Real radio is mostly other people: you hold short listening to two
departures and an arrival, and your own clearance arrives in a gap. This module
manufactures that, as complete two-sided exchanges -- a controller instruction
and the readback that follows it -- on the frequency the user is actually
tuned to.

Four things make it worth listening to rather than noise.

*The traffic belongs where it is.* The roster comes from
:mod:`meridianatc.atc.operators`, which is weighted by where each operator is
based, so Orly is full of Air France and Frankfurt is full of Lufthansa.

*Each crew speaks the right language.* An operator's crew uses its own language
where the state below works it and English where it does not, by the same rule
that decides what the user is answered in. Air France speaks French at Orly and
English at Frankfurt, in the middle of the same flight.

*A callsign belongs to one aeroplane.* A flight number is drawn once, from the
operator's own range, and stays with that aircraft from clearance delivery to
the gate. It is never handed to a second aeroplane while the first is still on
frequency, and the aircraft type never changes underneath it.

*The runway is a resource.* Only one aeroplane is on it at a time, arrivals
have priority over departures, and a departure released in front of an arrival
gets the gap it needs. That is what makes a takeoff clearance arrive when it
plausibly would rather than on a timer.
"""

from __future__ import annotations

import logging
import math
import random
import time
from dataclasses import dataclass, field

from ...logs import decision
from ...navdata.build import project
from ...navdata.channels import same_channel
from ...navdata.db import (
    NavDB,
    Station,
    bearing_deg,
    haversine_nm,
    normalize_runway,
)
from .. import circuit as circuitry
from .. import fleet as fleets
from .. import flows as flowmod  # noqa: F401  (re-exported)
from .. import ground as groundwork
from .. import live as livemod
from .. import perftable
from ..callsigns import every_language
from ..circuit import (
    CIRCUIT_BASE_NM,
    CIRCUIT_CENTRELINE_NM,
    CIRCUIT_UPWIND_NM,
    CIRCUIT_WIDTH_NM,
)
from ..flying import (
    AIRLINER,
    FLARE_FPM,
    FLARE_FT,
    LIFTOFF_AFTER_S,
    OWN_DECISIONS,
    Command,
    Performance,
    Target,
    glide_altitude,
    offset_from_track,
)
from ..flying import LIGHT as LIGHT_PERFORMANCE
from ..flying import step as fly_step
from ..language import phraseology_for, reply_language
from ..local_time import field_hour, host_hour
from ..operators import (
    FREIGHT,
    LIGHT_FLEET,
    REGIONAL,
    WIDEBODY,
    Operator,
    by_icao,
    registration_prefix,
    roster_for,
)
from ..phraseology import Aircraft, Weather, wake_suffix
from ..pilot import PilotSpeech

log = logging.getLogger(__name__)

# How long a departure occupies the runway, and the gap behind a landing.
RUNWAY_ROLL_S = 45.0
LANDING_ROLL_S = 55.0

# The pause between an instruction and its readback. Long enough to hear as two
# transmissions rather than one, short enough to sound like a working pilot.
READBACK_GAP_S = 2.6

# Live flights per density unit, by airport size rank. A rank-5 field runs a
# lot of aeroplanes; a rank-2 grass strip runs one Cessna in the circuit.
CONCURRENT_BY_RANK: dict[int, int] = {0: 1, 1: 1, 2: 2, 3: 3, 4: 5, 5: 7}

# Below this rank the traffic is light aircraft rather than airliners, and --
# because that is what a field of that size actually works -- VFR rather than
# IFR. An airfield does not run an IFR departure queue.
AIRLINE_RANK = 3

# How many of the VFR aircraft at an airfield are doing circuits rather than
# going somewhere. Roughly what a fine afternoon at a training field looks
# like: most of the movements are the same four aeroplanes going round.
CIRCUIT_SHARE = 0.45


def _reciprocal_of(runway: str) -> str:
    """"09L" -> "27R": the other end of the same strip."""
    runway = (runway or "").strip().upper()
    if len(runway) < 2 or not runway[:2].isdigit():
        return ""
    num = int(runway[:2])
    other = ((num + 17) % 36) + 1
    side = runway[2:]
    other_side = {"L": "R", "R": "L"}.get(side, side)
    return f"{other:02d}{other_side}"


def _runway_key(runway: str) -> str:
    """The strip a runway end is on: "09L" and "27R" are the same pavement,
    "09L" and "09R" are not."""
    a = (runway or "").strip().upper()
    b = _reciprocal_of(a)
    return "/".join(sorted(x for x in (a, b) if x))


def _strips_parallel(a: str, b: str) -> bool:
    """Whether two strip keys are parallel runways: "09L/27R" and
    "09R/27L" share a heading pair, "09L/27R" and "16L/34R" do not.
    Crossing strips are left to the tower's spacing, which treats them
    as the same pavement anyway."""
    if not a or not b:
        # An unnamed runway is not parallel to anything: the spacing
        # checks it would skip are the conservative thing to keep.
        return False
    numbers = {key[:2] for key in (a, b) for key in key.split("/")}
    # Two strips share a heading when a number appears in both; with the
    # reciprocal pair inside each key that is "09" against "09".
    return len(numbers) < len([key for key in (a, b)
                               for key in key.split("/")])


@dataclass
class Flight:
    """One AI aeroplane, from first call to last."""

    written: str                 # AFR1234 or F-GXYZ
    type_code: str
    aircraft: Aircraft
    language: str                # what this crew speaks at this field
    operator: Operator | None
    arriving: bool
    # Which rules this aeroplane is flying under. A Cessna in the circuit at a
    # grass strip does not file IFR to Nice, and having it ask for an airways
    # clearance was the single most obviously wrong thing on the frequency at
    # a small field.
    ifr: bool = True
    # Staying in the circuit rather than leaving it.
    pattern_work: bool = False
    pattern_direction: str = "left"
    runway: str = ""
    destination: str = ""
    origin: str = ""
    # The aerodromes those names are, where the route was picked from the
    # navdata rather than the global destination list: the map draws the
    # route line to these, and the spoken names stay for the radio.
    destination_icao: str = ""
    origin_icao: str = ""
    squawk: str = "2000"
    altitude_ft: float = 0.0
    distance_nm: float = 0.0     # arrivals: distance still to run
    phase: str = "clearance"
    step: int = 0
    next_at: float = 0.0
    sequence: int = 0            # position in the landing or departure queue
    done: bool = False
    # The positions this crew has already greeted, so they say good morning
    # once per frequency rather than on every transmission.
    greeted: set[str] = field(default_factory=set)

    # Where it actually is. Flown there by the model in :mod:`.flying` once
    # the aeroplane is on the runway or in the air, so the map, the strip and
    # the simulator all see the same thing -- and see it arrive at the rate an
    # aeroplane arrives at it.
    latitude: float = 0.0
    longitude: float = 0.0
    heading: float = 0.0
    ground_speed_kt: float = 0.0
    vertical_rate_fpm: float = 0.0
    on_ground: bool = True
    # The track the wind makes of the heading, and the wind doing it --
    # seeded from the field's weather at spawn, and what the energy model
    # writes back each step. Zero is "no wind reported".
    track: float = 0.0
    wind_deg: float = 0.0
    wind_kt: float = 0.0
    # Whether the flight model owns this aeroplane's position. It does from
    # the moment it lines up until it has slowed down after landing; before
    # and after that it is on a taxiway, and there is no taxiway data to fly
    # it along, so it is placed rather than flown.
    airborne_model: bool = False
    # When the nose came up on the takeoff roll (the model's clock), or None:
    # the wheels leave flying.LIFTOFF_AFTER_S later, and the injector's
    # follower pitches the drawn aeroplane from it.
    rotate_at: float | None = None

    # What it can do, and what it is currently trying to do.
    performance: Performance = AIRLINER
    target: Target = field(default_factory=Target)

    # What the controller has told it, and when each takes effect -- which is
    # when the readback finishes, not when the instruction goes out.
    commands: list[Command] = field(default_factory=list)
    # The last one to have taken effect, which is what the strip and the map
    # show: "cleared takeoff", "go around", "HDG 270".
    doing: Command | None = None

    # What it has been assigned and is still holding. These outrank whatever
    # the phase would otherwise fly, which is what makes an instruction an
    # instruction rather than a suggestion.
    assigned_heading: float | None = None
    assigned_altitude_ft: float | None = None
    assigned_speed_kt: float | None = None
    # A downwind being extended: the crew flies past the point they would
    # normally turn until they are told to turn base.
    extending: bool = False
    # How many times this aeroplane has been sent around. It is worth knowing
    # because a second one at the same field is a controller getting it wrong.
    go_arounds: int = 0
    # The circuit it has decided will be its last: it goes round once more and
    # lands off that one rather than asking for the option again.
    last_circuit: bool = False
    # The wind the last takeoff/landing clearance carried, as
    # (direction, kt, gust kt): the baseline a wind check is issued from.
    wind_at_clearance: tuple = ()

    # Somebody else's aeroplane: FSLTL, AIG, the simulator's own live traffic.
    # It is read rather than flown and it cannot be instructed, so the flight
    # model never touches it and the tower never tells it to do anything --
    # see :meth:`TrafficInjector.set_observed`.
    observed: bool = False
    # Which object in the simulator it is, which is what it is recognised by
    # between polls. A callsign is not enough: FSLTL reuses one the moment a
    # flight number turns round.
    observed_id: int = 0
    # Where an observed one comes from: "simulator" (FSLTL, AIG, the
    # simulator's own) or "live" (the ADS-B feed, atc/live.py).
    source: str = ""
    # airline, cargo or ga (atc/live.py): what it counts against in the
    # field's density. Set when invented; worked out for an observed one.
    category: str = ""
    # An invented flight whose callsign a real one has turned up with: it
    # finishes what it is doing and goes at its next gate.
    retire: bool = False
    # A live one (atc/live.py): its address, how far its lead (where the
    # real one is now) has to run, and whether our tower is holding it.
    # A long taxi cut in two (atc.taxi.split_route): the taxiways still to
    # be given, and where (layout metres) it was told to hold short.
    taxi_rest: list = field(default_factory=list)
    taxi_split_xy: tuple | None = None
    # An intersection departure (ground.INTERSECTION_M): the taxiway the
    # entry joins the runway at and the runway still ahead of it, set by
    # the departure plan. The tower names the first in the clearance.
    intersection: str = ""
    remaining_m: float = 0.0
    hex: str = ""
    lead_distance_nm: float = 0.0
    lead_at: tuple | None = None   # (lat, lon) of the real one now
    live_held: str = ""          # "" | "runway" | "give way"
    cleared: bool = False        # delivery has given it its clearance

    # The procedures it is flying, picked the way the controller picks
    # them (atc.procedure_select): the names for the radio, and the
    # (procedure, transition) pair the navigator is built from. Empty
    # means the field had none to give, or the flight is VFR -- it is
    # cleared and flown the generic way, which is what a field with no
    # published procedures sounds like.
    sid: str = ""
    sid_transition: str = ""
    star: str = ""
    star_transition: str = ""
    approach: str = ""
    procedure_pair: tuple | None = None
    # The navigator flying those legs (ProcedureNavigator), built when
    # the model takes over; None again once the legs are done.
    procedure_nav: object | None = None
    # The published missed-approach navigator, built when an IFR arrival
    # is sent around (integrations.cifp keeps the legs after the MAP on
    # the Approach record); None for VFR or a field with no missed coded,
    # which keeps the old circuit -- and None again once it is flown,
    # when the flight comes back for another approach.
    missed_nav: object | None = None
    # A missed that ends in a coded hold is held, not flown through: the
    # fix's racetrack is orbited until this clock runs out, at the
    # procedure's altitude -- then the stream takes it back in.
    missed_hold: bool = False
    missed_hold_until: float = 0.0
    missed_hold_alt_ft: float = 0.0
    # The level a missed is climbed to even where the data codes no
    # legs for it: the plate's missed altitude, else two thousand feet
    # over the field. An IFR go-around holds runway heading up to it
    # and comes back for another approach; the circuit is the VFR
    # aeroplane's answer, not this one's. 0 while there is none.
    missed_alt_ft: float = 0.0
    # Told to hurry and by when that expires: a higher climb/roll speed
    # and shorter pauses until then, 0 if it was never told.
    expedite_until: float = 0.0
    # The hold approach gave it when the queue ahead was longer than the
    # runway takes (atc.holds.Hold): the fix and the pattern, plus which
    # half of the racetrack the model is on and when that half began.
    # None while it is simply inbound, and gone again once it is let out.
    hold: object | None = None
    hold_leg: str = ""
    hold_leg_at: float = 0.0

    # On the ground, at an airport whose taxiways the simulator has given us
    # (see meridianatc.atc.ground). ``stand`` is the one it is on or going to;
    # ``taxi`` the route it is on now; ``line_up`` the way from the holding
    # point onto the runway, kept until the tower says to use it.
    on_layout: bool = False
    stand: int | None = None
    taxi: object | None = None
    line_up: object | None = None
    planned: object | None = None
    parked_at: float = 0.0
    # The schedule (traffic.lod.mode: schedule). An arrival's stand, held
    # from the moment it is invented so its model can be chosen for where
    # it will turn round; and a parked aeroplane booked as a departure:
    # when its boarding starts, and whether it has.
    planned_stand: int | None = None
    boarding_at: float = 0.0
    boarding: bool = False
    # Boarded with the test button (traffic.debug.triggers): its model is
    # changed in front of the pilot if need be, because seeing its doors is
    # what the button was pressed for.
    by_hand: bool = False
    # A departure that has been handed on and is flying away out of sight,
    # rather than vanishing a mile off the end of the runway. On any strip,
    # that its next-frequency handoff was said -- an observed one handed
    # back to the feed is not sent a second time.
    handed_off: bool = False
    # Since when it has been waiting for the runway, or 0 if it is not. How
    # the tower knows to leave a gap in the arrivals for it.
    held_since: float = 0.0
    # Parked at a stand and going nowhere yet: out of the window and on the
    # map, never on the radio. See TrafficInjector._fill_stands.
    static: bool = False
    # Whether the simulator has drawn it, for a static aeroplane, which does
    # not wait to be drawn before it exists the way a moving one does.
    drawn: bool = False
    # The watchdog's (TrafficInjector._watchdog): where it last moved from
    # and since when it has stood there, whether it has been given its one
    # recovery, and whether it is stuck -- out of the controller's picture
    # from then on, and taken away if the recovery does not move it.
    still_at: tuple | None = None
    still_since: float = 0.0
    recovered: bool = False
    stuck: bool = False
    # Asked to be drawn again from scratch, at the same spot, as the
    # recovery for one the simulator may have lost: left out of the next
    # positions() so the object is removed and then created again.
    redraw: bool = False

    @property
    def positioned(self) -> bool:
        return self.latitude != 0.0 or self.longitude != 0.0

    @property
    def order(self) -> dict:
        """What it was last told, as a kind and a number.

        Empty until somebody has said something to it, which for a flight
        still on the stand is most of the time.
        """
        return self.doing.as_dict() if self.doing else {"kind": "", "detail": ""}

    def snapshot(self) -> dict:
        """What this aeroplane looks like from outside.

        The shape the map, the traffic list and the simulator injector all
        read, so there is one answer to where an aeroplane is rather than
        three.
        """
        return {
            "callsign": self.written,
            "spoken": self.aircraft.for_language(self.language).spoken_short,
            "type": self.type_code,
            "ifr": self.ifr,
            "operator": (self.operator.icao if self.operator
                         else self.written[:3]
                         if self.source == "live"
                         and self.category in (livemod.AIRLINE, livemod.CARGO)
                         else ""),
            "language": self.language,
            "phase": self.phase,
            "arriving": self.arriving,
            "runway": self.runway,
            "sequence": self.sequence,
            "position": self.position,
            "lat": round(self.latitude, 6),
            "lon": round(self.longitude, 6),
            "altitude_ft": round(self.altitude_ft),
            "heading": round(self.heading),
            "ground_speed_kt": round(self.ground_speed_kt),
            "on_ground": self.on_ground,
            "observed": self.observed,
            "source": self.source or ("simulator" if self.observed
                                      else "invented"),
            "static": self.static,
            "vertical_rate_fpm": round(self.vertical_rate_fpm),
            "distance_nm": round(self.distance_nm, 1),
            # What it is doing and why, which is the whole point of being able
            # to see it: an aeroplane going around on the map is one that was
            # told to go around a moment ago, and the strip says so.
            "order": self.order,
            "intent": self.target.intent,
            "assigned_heading": (round(self.assigned_heading)
                                 if self.assigned_heading is not None else None),
            "assigned_altitude_ft": (round(self.assigned_altitude_ft)
                                     if self.assigned_altitude_ft is not None
                                     else None),
            "assigned_speed_kt": (round(self.assigned_speed_kt)
                                  if self.assigned_speed_kt is not None else None),
        }

    @property
    def position(self) -> str:
        """The controller position this flight is working right now."""
        return _POSITION_OF_PHASE.get(self.phase, "TWR")

    @property
    def is_heavy(self) -> bool:
        return bool(self.aircraft.wake)

    def describe(self, language: str = "en") -> str:
        """How the controller points this aeroplane out to somebody else.

        By make, which is what is actually said: "follow the Airbus on short
        final", never "follow the A320". The manufacturer names are proper
        nouns and survive translation; the article in front of one does not,
        so it comes from a table.
        """
        return article(make_of(self.type_code), language)


# What a controller calls each type on the radio. Grouped by make, because
# that is the level of detail a sequencing instruction actually carries: you
# are told to follow the Airbus, not the A320.
_MAKE: tuple[tuple[tuple[str, ...], str], ...] = (
    (("A319", "A320", "A321", "A20N", "A21N", "A306", "A333", "A359", "A388"),
     "Airbus"),
    (("B737", "B738", "B38M", "B37M", "B752", "B753", "B763", "B77W", "B77L",
      "B788", "B789", "B78X"), "Boeing"),
    (("E190", "E195", "E75L", "CRJ9"), "Embraer"),
    (("AT76", "DH8D"), "ATR"),
    (("C152", "C172", "C182", "C25A"), "Cessna"),
    (("PA28", "PA34", "P28A"), "Piper"),
    (("DA40", "DA42"), "Diamond"),
    (("SR22",), "Cirrus"),
    (("BE36",), "Beechcraft"),
    (("TBM9",), "TBM"),
    (("PC12",), "Pilatus"),
    (("R44",), "Robinson"),
    (("TB20", "AT3"), "Socata"),
)

# "the Airbus", in each language the controller may be speaking. French and
# Italian elide before a vowel and do not before a consonant, so each has two
# forms and the make decides which one: "l'Airbus", but "le Boeing".
_ARTICLE: dict[str, tuple[str, str]] = {
    "en": ("the {}", "the {}"),
    "fr": ("l'{}", "le {}"),
    "es": ("el {}", "el {}"),
    "de": ("die {}", "die {}"),
    "it": ("l'{}", "il {}"),
    "pt": ("o {}", "o {}"),
}

# "traffic on the runway", which is why you are going around.
_RUNWAY_TRAFFIC: dict[str, str] = {
    "en": "traffic on the runway",
    "fr": "trafic sur la piste",
    "es": "tráfico en la pista",
    "de": "Verkehr auf der Piste",
    "it": "traffico in pista",
    "pt": "tráfego na pista",
}

# "traffic landing", which is why you are being held on the runway.
_LANDING_TRAFFIC: dict[str, str] = {
    "en": "traffic landing, {}",
    "fr": "trafic à l'atterrissage, {}",
    "es": "tráfico aterrizando, {}",
    "de": "landender Verkehr, {}",
    "it": "traffico in atterraggio, {}",
    "pt": "tráfego a aterrar, {}",
}


# "windshear reported", which is why you are going around.
_WINDSHEAR: dict[str, str] = {
    "en": "windshear reported",
    "fr": "cisaillement de vent signalé",
    "es": "cizalladura reportada",
    "de": "Windscherung gemeldet",
    "it": "segnalato windshear",
    "pt": "windshear reportado",
}


def article(make: str, language: str = "en") -> str:
    """``"Airbus"`` -> ``"the Airbus"``, ``"l'Airbus"``, ``"il Boeing"``."""
    if not make:
        return ""
    vowel, consonant = _ARTICLE.get(language, _ARTICLE["en"])
    frame = vowel if make[0].upper() in "AEIOU" else consonant
    return frame.format(make)


def landing_traffic(make_phrase: str, language: str = "en") -> str:
    """Why an aircraft is being held on the runway rather than released."""
    if not make_phrase:
        return ""
    return _LANDING_TRAFFIC.get(language, _LANDING_TRAFFIC["en"]).format(
        make_phrase)


def runway_traffic(language: str = "en") -> str:
    """Why an aircraft on short final is being sent around.

    Not which aeroplane: a crew at two hundred feet is told to go around and
    told why in three words, and whether the thing on the runway is an Airbus
    is a conversation for afterwards.
    """
    return _RUNWAY_TRAFFIC.get(language, _RUNWAY_TRAFFIC["en"])


def windshear(language: str = "en") -> str:
    """Why an aircraft is going around on a gusty day."""
    return _WINDSHEAR.get(language, _WINDSHEAR["en"])


def wind_moved(was: tuple, weather) -> bool:
    """Whether the field's wind has moved enough since ``was`` that the
    tower would say so -- the same materiality the ATIS applies before
    it rolls a new letter: five knots or thirty degrees, or a gust
    spread appearing that was not there."""
    if not was or weather is None:
        return False
    was_dir, was_kt, was_gust = was
    kt = float(getattr(weather, "wind_kt", 0.0) or 0.0)
    deg = float(getattr(weather, "wind_dir", 0.0) or 0.0)
    gust = float(getattr(weather, "gust_kt", 0.0) or 0.0)
    return (abs(was_kt - kt) >= 5.0
            or (abs((was_dir - deg + 540.0) % 360.0 - 180.0) >= 30.0
                and kt >= 5.0)
            or (gust - kt >= 10.0 and was_gust - was_kt < 10.0))


def wind_snapshot(weather) -> tuple:
    """The (direction, kt, gust kt) a clearance carried, kept so a wind
    check can be issued when it has moved since."""
    if weather is None:
        return ()
    return (float(getattr(weather, "wind_dir", 0.0) or 0.0),
            float(getattr(weather, "wind_kt", 0.0) or 0.0),
            float(getattr(weather, "gust_kt", 0.0) or 0.0))


def make_of(type_code: str) -> str:
    """The manufacturer a type belongs to, for pointing it out on frequency."""
    code = (type_code or "").upper()
    for codes, make in _MAKE:
        if code in codes:
            return make
    return ""


# Which frequency a phase happens on. A pilot hears an exchange only if they
# are tuned to the same one, which is what keeps the ground frequency full of
# taxi instructions and the tower frequency full of takeoff clearances.
_POSITION_OF_PHASE: dict[str, str] = {
    "clearance": "DEL",
    "pushback": "GND",
    "taxi": "GND",
    # At the holding point: asking for the runway, and waiting to hear.
    "holding": "TWR",
    "ready": "TWR",
    "waiting": "TWR",
    "joining": "TWR",
    "lineup": "TWR",
    "takeoff": "TWR",
    "departed": "TWR",
    "inbound": "APP",
    "final": "TWR",
    "landed": "TWR",
    "taxi_in": "GND",
    # The circuit, and the aeroplane that has just been sent round it again.
    # All of it belongs to the tower, which is the position that put it there.
    "upwind": "TWR",
    "crosswind": "TWR",
    "downwind": "TWR",
    "base": "TWR",
    "go_around": "TWR",
    # And the hold the published missed ends in -- still the tower's
    # aeroplane, orbiting until it is worked back in.
    "missed_hold": "TWR",
}

# The legs of a circuit, which are flown rather than timed.
_CIRCUIT_PHASES = frozenset({"upwind", "crosswind", "downwind", "base"})

# The shape of that circuit lives in :mod:`meridianatc.atc.circuit`, where the
# tower reads the user's position against the same numbers these aeroplanes
# fly: how far out before the crosswind turn, how wide the downwind, how far
# past the threshold before base, how close to the centreline is final.

def _turn_radius_nm(speed_kt: float, bank_deg: float = 25.0) -> float:
    """How wide a rate-limited turn is at this speed, in nautical miles."""
    return (speed_kt * speed_kt
            / (11.26 * math.tan(math.radians(bank_deg)))) / 6076.0


# How close to the start of the downwind counts as having got there.
JOIN_REACHED_NM = 0.5

# How an aeroplane gets back onto the centreline: it aims at a point this far
# ahead of itself along it, and turns no more than this to do it. Aiming at a
# point rather than correcting by an angle is what stops it weaving -- and
# aiming at all is what was missing, so an aeroplane rolling out of a base
# turn a little wide flew a final parallel to the runway rather than to it.
INTERCEPT_LEAD_NM = 1.0
INTERCEPT_LIMIT_DEG = 30.0

# How far before the turning point the tower decides whether this aeroplane is
# turning base. A controller watching an aeroplane run down the downwind says
# "extend downwind" before it turns, not after: an instruction that arrives
# while the crew is already in the turn is an instruction that arrived late.
BASE_LOOKAHEAD_NM = 0.8

# Where a departure is heading before anybody has said otherwise: the initial
# climb in the clearance, and the level it settles at on the way out.
INITIAL_CLIMB_FT = 3000.0
# Climbing out, the speed is held at V2 plus a little to this height above the
# runway, and only then allowed to build to the climb speed.
ACCELERATE_AT_FT = 1000.0
CLIMB_TO_FT = 5000.0

# --------------------------------------------------------------------------
# aeroplanes somebody else is flying
# --------------------------------------------------------------------------
#
# How much of what the simulator reports is this airport's business. A busy
# region has hundreds of aircraft in it and almost none of them are working the
# field the user is at; putting one on the strip because it happened to be
# thirty thousand feet over the top is the fastest way to make the traffic list
# useless.

# The live ones on the scenery (TrafficInjector._live_pose). The centreline
# is the pilot's figure (30 Oct 2026); the rest UNVERIFIED, chosen: a
# heading this close to the runway's is lined up with it, the final is this
# long, a runway with no published length this long, and a taxiway this
# close is the one it is on.
LIVE_CENTRELINE_M = 40.0
LIVE_ALIGNED_DEG = 15.0
LIVE_FINAL_M = 3.0 * 1852.0
LIVE_RUNWAY_M = 3000.0
LIVE_TAXIWAY_M = 40.0
# Rolled onto a stand (after LocalTC): within 150 m, ahead, at taxi speed,
# at a walking pace, for 45 minutes at most.
LIVE_STAND_M = 150.0
LIVE_AHEAD_DEG = 70.0
LIVE_TAXI_IN_KT = 30.0
LIVE_ROLL_MS = 1.5
LIVE_PARKED_S = 45 * 60.0
M_PER_DEG = 111_320.0
# A hectopascal of QNH near the ground, in feet (ISA: 8.3 m).
FT_PER_HPA = 27.3
# The pilot's runway against the live ones (after LocalTC): the pilot on a
# final inside 4 nm counts as using it; a live arrival this close to the
# threshold with the pilot on the runway, or this little behind the pilot
# on the same final, is sent round; held departures go 90 s apart; a hold
# short is said once a minute at most; on the ground one taxiing gives way
# to the pilot this close in front. UNVERIFIED: the 2 and 2.5 nm and the
# 150 m / 45 degrees, chosen.
PILOT_FINAL_NM = 4.0
LIVE_GO_AROUND_NM = 2.0
LIVE_BEHIND_NM = 2.5
RELEASE_GAP_S = 90.0
HOLD_SHORT_EVERY_S = 60.0
GIVE_WAY_M = 150.0
GIVE_WAY_DEG = 45.0
#: Nose-to-nose on the same lane is a conflict long before the noses are
#: there: a head-on pilot is yielded to from well ahead of the meeting
#: point, not once the noses have met.
GIVE_WAY_HEAD_ON_M = 450.0
GIVE_WAY_HEAD_ON_DEG = 120.0
#: And where the stop lands too late -- already nose to nose, neither
#: moving -- the yield can only deadlock them: after a wait long enough
#: to be sure nobody is passing, it is let go to keep right and pass.
GIVE_WAY_STUCK_M = 60.0
GIVE_WAY_STUCK_S = 20.0

#: How far from the field an observed aeroplane still counts as working it.
OBSERVED_RANGE_NM = 30.0
#: And how far above it. An airliner in the cruise over the top is not an
#: arrival, however close to the field it is.
OBSERVED_CEILING_FT = 12_000.0
#: Rolling rather than taxiing, and moving rather than parked.
ROLLING_KT = 40.0
TAXIING_KT = 3.0
# Pushed back: slow, and moving the other way from the nose (atc/live.py).
PUSHBACK_KT = 8.0
PUSHBACK_DEG = 120.0
# How far from the field's reference point somebody else's parked aeroplane
# can be and still be standing on one of its stands.
PARKED_RANGE_NM = 3.0
#: How far out an observed arrival is cleared to land. Far enough to be a
#: landing clearance rather than a commentary, near enough that it is
#: certainly landing off this approach.
LANDING_CLEARANCE_NM = 8.0
#: How many of the simulator's aeroplanes are worked at once. A big airport
#: under FSLTL has a hundred aircraft parked on it, and a hundred strips is
#: not a traffic list -- it is a wall. The ones that matter are the ones near
#: the runway, so that is what the limit keeps.
OBSERVED_LIMIT = 20

# How far above the field an aeroplane has to be before it counts as having
# left it. On the tick the wheels come off it is still at field elevation, and
# calling that "climbing out" is how a departure reaches the map at zero feet.
AIRBORNE_BY_FT = 60.0


# How fast an aeroplane of each kind is going, by what it is doing. Rounded
# numbers rather than a performance model: the point is that a landing aircraft
# is slower than a departing one and that neither of them is stationary.
_SPEED = {
    "clearance": 0.0, "taxi": 12.0, "holding": 0.0, "ready": 0.0,
    "waiting": 0.0, "taxi_in": 12.0,
}

# The phases an aeroplane is flown through rather than placed in. Anything in
# here is handed to the flight model the moment it enters it, wherever it came
# from, so a flight put together by hand behaves like one the field made.
_FLYING_PHASES = frozenset({
    "lineup", "takeoff", "departed", "inbound", "joining", "final", "landed",
    "go_around",
}) | _CIRCUIT_PHASES

# Every phase a flight can be in, which is what the panel needs a label for.
# "parked" is the apron: this program's aeroplanes at the gates and FSLTL's,
# which are on the map and never on a strip.
PHASES: frozenset[str] = frozenset(_SPEED) | _FLYING_PHASES | {"parked"}

# Three degrees, in feet per nautical mile.
GLIDE_FT_PER_NM = 318.0

# What an arrival is doing at the threshold and what it is doing before it
# starts slowing down, with the deceleration spread over the miles between.
APPROACH_SPEED_KT = 140.0
ARRIVAL_SPEED_KT = 250.0
SLOWING_FROM_NM = 15.0
STABLE_BY_NM = 4.0


def _squawk(code: str) -> str:
    """The real one's code where it is a code, the conspicuity one else."""
    code = (code or "").strip()
    if len(code) == 4 and all(ch in "01234567" for ch in code):
        return code
    return "2000"


def observed_type(raw: str) -> str:
    """The ICAO type designator out of whatever the simulator calls a model.

    Injectors differ. FSLTL writes the designator straight into ``ATC MODEL``
    and the answer is the string itself; the simulator's own aircraft write a
    localisation key -- ``TT:ATCCOM.AC_MODEL_B738.0.text`` -- with the
    designator buried in the middle of it. Anything that does not come out as
    a plausible designator comes out empty, and an aeroplane with no type is
    still an aeroplane on the runway.
    """
    text = (raw or "").strip().upper()
    if not text:
        return ""
    if "AC_MODEL" in text:
        text = text.split("AC_MODEL", 1)[1]
    text = text.strip("_. ").split(".", 1)[0].strip("_ ")
    # "A350-900" and "A320 neo" are a designator with something after it, and
    # the something is not part of it. A name with no designator in it at all
    # -- "Cherokee" -- comes back empty rather than truncated.
    head = text.replace("-", " ").split()
    text = "".join(c for c in (head[0] if head else "") if c.isalnum())
    # FSLTL's models say what they are the way a ramp agent would: "737",
    # "ATR76". Read off a running 2024 at Madeira, and each is one type.
    text = _MODEL_NAMES.get(text, text)
    if 3 <= len(text) <= 4 and text[0].isalpha() and any(
            c.isdigit() for c in text):
        return text
    return ""


#: What FSLTL writes in ATC MODEL for the types it does not give a designator
#: for, and the designator. A bare Boeing number is the variant FSLTL's base
#: model of it is.
_MODEL_NAMES: dict[str, str] = {
    "737": "B738", "738": "B738", "739": "B739", "747": "B744", "748": "B748",
    "757": "B752", "767": "B763", "777": "B77W", "787": "B789",
    "ATR72": "AT76", "ATR75": "AT75", "ATR76": "AT76", "ATR42": "AT46",
    "ATR45": "AT45", "ATR46": "AT46", "A320NEO": "A20N", "A321NEO": "A21N",
    "A319NEO": "A19N", "A330NEO": "A339",
}


def approach_speed(distance_nm: float,
                   performance: Performance | None = None) -> float:
    """How fast an arrival that far from the threshold is going.

    Threshold speed close in, arrival speed far out, and the deceleration
    spread over the miles between. With a performance it is that type's own
    two numbers, which is what keeps a Dash 8 off an airliner's profile and a
    Cessna off both.
    """
    stable = APPROACH_SPEED_KT if performance is None else performance.approach_kt
    arrival = ARRIVAL_SPEED_KT if performance is None else performance.cruise_kt
    if distance_nm <= STABLE_BY_NM:
        return stable
    if distance_nm >= SLOWING_FROM_NM:
        return arrival
    across = ((distance_nm - STABLE_BY_NM)
              / (SLOWING_FROM_NM - STABLE_BY_NM))
    return stable + across * (arrival - stable)


@dataclass
class Call:
    """One transmission, ready to be spoken."""

    text: str
    language: str
    from_pilot: bool
    position: str
    speaker: str
    flight: Flight
    at: float = 0.0
    # Something that cannot wait its turn behind two taxi instructions. A
    # go-around is the whole of this category.
    urgent: bool = False
    # Which exchange this is a half of, so that the two halves live or die
    # together. A call, its clearance and its readback are one thing to
    # listen to, and hearing the middle of it is worse than hearing none of
    # it: a controller answering an aeroplane that never called sounds like
    # the frequency is missing transmissions, which it is. Zero for a call
    # that is not part of an exchange at all.
    exchange: int = 0
    # The channel it is said on, where known: what decides who hears it.
    mhz: float = 0.0


#: How big a parked aeroplane is taken to be when it is standing nowhere the
#: layout calls a stand: the radius the simulator gives an airliner's gate,
#: and a light aircraft's tie-down.
AIRLINER_FOOTPRINT_M = 18.0
LIGHT_FOOTPRINT_M = 6.0


@dataclass
class ParkedObserved:
    """Somebody else's aeroplane standing on this airport.

    Not a :class:`Flight`: it is on no frequency and in no sequence. It is
    on the map, and the ground handling calls on it.
    """

    object_id: int
    callsign: str
    registration: str = ""
    type_code: str = ""
    # The simulator title, for the door table.
    title: str = ""
    since: float = 0.0
    # A live flight (atc/live.py): it holds its stand and counts against the
    # parked ones, but nothing in the simulator is it yet, so nothing is
    # sent to it -- no vehicles, no doors.
    live: bool = False
    category: str = ""
    # A live one's airline, for its model and livery; and when it was put
    # on the stand, for how long it stays when the feed no longer hears it.
    operator: str = ""
    parked_at: float = 0.0
    latitude: float = 0.0
    longitude: float = 0.0
    heading: float = 0.0
    altitude_ft: float = 0.0

    @property
    def key(self) -> str:
        """What the handling knows it by. FSLTL's parked aeroplanes carry
        made-up callsigns that can repeat; the object id cannot."""
        return f"{self.callsign or 'AI'}#{self.object_id}"

    def snapshot(self, field) -> dict:
        """The shape :meth:`Flight.snapshot` has, as far as the map reads it."""
        return {
            "callsign": self.callsign or self.registration or "AI",
            "spoken": "",
            "type": self.type_code,
            "ifr": True,
            "operator": self.operator,
            "language": "",
            "phase": "parked",
            "arriving": False,
            "runway": "",
            "sequence": 0,
            "position": "",
            "lat": round(self.latitude, 6),
            "lon": round(self.longitude, 6),
            "altitude_ft": round(self.altitude_ft),
            "heading": round(self.heading),
            "ground_speed_kt": 0,
            "on_ground": True,
            "observed": True,
            "source": "live" if self.live else "simulator",
            "static": True,
            "vertical_rate_fpm": 0,
            "distance_nm": round(haversine_nm(
                field.latitude, field.longitude,
                self.latitude, self.longitude), 1) if field else 0.0,
            "order": {"kind": "", "detail": ""},
            "intent": "parked",
            "assigned_heading": None,
            "assigned_altitude_ft": None,
            "assigned_speed_kt": None,
        }


@dataclass
class UserAircraft:
    """The aeroplane the person is flying, as the traffic needs to see it.

    The AI used to work a field with one aeroplane missing from it: the one
    that mattered. A departure was held for an arrival that was another AI and
    never for the user, and an AI landed on a runway the user was sitting on.

    This is the fix, and it is deliberately thin. The traffic does not need to
    know what the user was cleared for or what they read back -- only where
    they are, which way they are pointing and whether they are on the ground.
    Everything a controller would decide from looking out of the window, and
    nothing that would need the two halves of the program to agree about
    state.
    """

    callsign: str = ""
    type_code: str = ""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude_ft: float = 0.0
    heading: float = 0.0
    ground_speed_kt: float = 0.0
    on_ground: bool = True

    @property
    def has_position(self) -> bool:
        return not (abs(self.latitude) < 1e-6 and abs(self.longitude) < 1e-6)

    @property
    def make(self) -> str:
        """What a controller would call it when pointing it out to somebody."""
        return make_of(self.type_code)


# How wide a runway is, and how long, in miles -- for deciding whether an
# aeroplane is on one. Generous on both counts: half a mile of centreline
# either side of the threshold catches an aeroplane lined up on a displaced
# one, and two miles catches the far end of the longest runway there is.
RUNWAY_WIDTH_NM = 0.05
RUNWAY_LENGTH_NM = 2.2

# How far off the runway heading an aeroplane on the pavement can be and still
# be lined up on it rather than crossing it. The same figure the surface watch
# uses on the user, and for the same reason.
LINED_UP_DEG = 35.0

#: Positions that belong to the airspace rather than to the aerodrome.
#:
#: The distinction only matters when one is standing in for another. A field
#: that publishes its own Approach has an approach controller; a field that
#: publishes the enroute Center frequency because that is who you call in the
#: overlying airspace does not, and reading the second as the first is what
#: made an uncontrolled strip look staffed.
_ENROUTE_POSITIONS = frozenset({"CTR"})


def _is_enroute_standin(station, position: str) -> bool:
    """Whether this station is the airspace above the field, not the field.

    True only when the lookup substituted one position for another *and* what
    came back is enroute. A Ground standing in for a Delivery is left alone:
    plenty of towered fields really do issue clearances on the ground
    frequency, and the traffic sounds right doing the same.
    """
    return (station.position != position
            and station.position in _ENROUTE_POSITIONS)


@dataclass
class _Field:
    """The airport the injector is working."""

    ident: str = ""
    name: str = ""
    size_rank: int = 3
    # Airline field or airfield, and why (meridianatc.atc.fleet). Logged as a
    # "fleet" row when the field is set.
    kind: fleets.FieldKind | None = None
    departure_runway: str = ""
    arrival_runway: str = ""
    # Every end the field is working in each direction, primaries first.
    # A big field lands two parallels at once; the single-runway fields
    # these default to keep the list at one. Occupancy, closure and
    # assignment all read these, never the primaries alone.
    departure_runways: list[str] = field(default_factory=list)
    arrival_runways: list[str] = field(default_factory=list)
    # Somebody chose a runway for the user -- the session's assignment,
    # not the field's own wind pick -- so the primaries are not the
    # flow's to rewrite.
    runway_locked: bool = False
    # The scenery's own traffic flow in force (atc.flows.Flow), where the
    # airport's apt.dat carries flows -- X-Plane's authored runway-in-use
    # rules. None under MSFS or where no flow passes.
    flow: object | None = None
    weather: Weather | None = None
    # The traffic frequency, where a field has one. Set when the field has no
    # controllers at all, which is what makes it an uncontrolled one.
    advisory: Station | None = None
    # The field's own aerodrome information service (an AFIS, or a UK air/
    # ground "Radio"), where it has one and no controller. Kortrijk has
    # nothing else, and with neither this nor a traffic frequency the
    # injector worked it as a towered field with no tower.
    afis: Station | None = None
    stations: dict[str, Station] = field(default_factory=dict)
    # Where the field is, and where its runway thresholds are, so an aeroplane
    # can be put on a centreline rather than merely near an airport.
    latitude: float = 0.0
    longitude: float = 0.0
    elevation_ft: float = 0.0
    thresholds: dict[str, tuple[float, float, float]] = field(
        default_factory=dict)          # runway -> (lat, lon, heading)
    # How the runway itself lies: runway -> (threshold elevation, elevation
    # at the far end, length in nm). Only where both ends are published;
    # everything else is taken to be at the field's elevation.
    surfaces: dict[str, tuple[float, float, float]] = field(
        default_factory=dict)


def _said_route(layout, taxi, field: str, *, inbound: bool):
    """A route as a controller gives it (atc.taxi): ``(said now, held short
    of, the rest, where the rest is given)`` -- each taxiway once, its
    number only at the holding point or the exit, five at most before the
    cut, field words said as words -- from a ground Taxiing."""
    from .. import taxi as taxiplan

    if layout is None or taxi is None:
        return [], "", [], None
    nodes = [node for node in getattr(taxi, "nodes", []) if node is not None]
    said = taxiplan.spoken_route(layout, nodes, inbound=inbound)
    names = [name for name, _ in said]
    first, hold_at, rest = taxiplan.split_route(names)
    where = None
    if rest:
        where = layout.xy[nodes[said[len(first)][1]]]
    return (taxiplan.say_words(field, first),
            taxiplan.say_words(field, [hold_at])[0] if hold_at else "",
            taxiplan.say_words(field, rest), where)


# How close one of ours gets to where it was told to hold short before the
# rest of a cut taxi is given. Chosen: a junction's width.
REST_OF_TAXI_M = 60.0


def _spell_route(route: list[str]) -> str:
    """``["A", "B"]`` -> ``"Alpha Bravo"``, for a pilot reading it back."""
    from ..speech import taxiway

    return " ".join(taxiway(letter) for letter in route)


def _local_or_english(ident: str) -> str:
    """What a local light aircraft speaks: the field's own working language
    if it has one -- not one it merely accepts (Kortrijk's French)."""
    from ..language import traffic_language

    return traffic_language(ident) or "en"


# Somewhere to be going. Spoken names, because they are read out on the radio.
_DESTINATIONS: tuple[str, ...] = (
    "Nice", "Lyon", "Toulouse", "Marseille", "Barcelona", "Madrid", "Lisbon",
    "Rome", "Milan", "Naples", "Munich", "Berlin", "Hamburg", "Vienna",
    "Zurich", "Geneva", "Brussels", "Amsterdam", "London Heathrow",
    "Manchester", "Dublin", "Copenhagen", "Stockholm", "Oslo", "Helsinki",
    "Warsaw", "Prague", "Budapest", "Athens", "Istanbul", "Casablanca",
    "Algiers", "Tunis", "New York", "Chicago", "Atlanta", "Dallas", "Denver",
    "Los Angeles", "Miami", "Toronto", "Montreal", "Dubai", "Doha",
    "Singapore", "Tokyo", "São Paulo", "Buenos Aires", "Mexico City",
)


# Where each of those is, so a departure's path on the map points the right
# way out of the circuit. The airport a city means to an airline.
DESTINATION_AIRPORTS: dict[str, str] = {
    "Nice": "LFMN", "Lyon": "LFLL", "Toulouse": "LFBO", "Marseille": "LFML",
    "Barcelona": "LEBL", "Madrid": "LEMD", "Lisbon": "LPPT", "Rome": "LIRF",
    "Milan": "LIMC", "Naples": "LIRN", "Munich": "EDDM", "Berlin": "EDDB",
    "Hamburg": "EDDH", "Vienna": "LOWW", "Zurich": "LSZH", "Geneva": "LSGG",
    "Brussels": "EBBR", "Amsterdam": "EHAM", "London Heathrow": "EGLL",
    "Manchester": "EGCC", "Dublin": "EIDW", "Copenhagen": "EKCH",
    "Stockholm": "ESSA", "Oslo": "ENGM", "Helsinki": "EFHK",
    "Warsaw": "EPWA", "Prague": "LKPR", "Budapest": "LHBP", "Athens": "LGAV",
    "Istanbul": "LTFM", "Casablanca": "GMMN", "Algiers": "DAAG",
    "Tunis": "DTTA", "New York": "KJFK", "Chicago": "KORD", "Atlanta": "KATL",
    "Dallas": "KDFW", "Denver": "KDEN", "Los Angeles": "KLAX",
    "Miami": "KMIA", "Toronto": "CYYZ", "Montreal": "CYUL", "Dubai": "OMDB",
    "Doha": "OTHH", "Singapore": "WSSS", "Tokyo": "RJTT",
    "São Paulo": "SBGR", "Buenos Aires": "SAEZ", "Mexico City": "MMMX",
}



__all__ = [
    "log",
    "RUNWAY_ROLL_S",
    "LANDING_ROLL_S",
    "READBACK_GAP_S",
    "CONCURRENT_BY_RANK",
    "AIRLINE_RANK",
    "CIRCUIT_SHARE",
    "Flight",
    "_MAKE",
    "_ARTICLE",
    "_RUNWAY_TRAFFIC",
    "_LANDING_TRAFFIC",
    "_WINDSHEAR",
    "article",
    "landing_traffic",
    "runway_traffic",
    "windshear",
    "make_of",
    "_POSITION_OF_PHASE",
    "_CIRCUIT_PHASES",
    "_turn_radius_nm",
    "JOIN_REACHED_NM",
    "INTERCEPT_LEAD_NM",
    "INTERCEPT_LIMIT_DEG",
    "BASE_LOOKAHEAD_NM",
    "INITIAL_CLIMB_FT",
    "ACCELERATE_AT_FT",
    "CLIMB_TO_FT",
    "LIVE_CENTRELINE_M",
    "LIVE_ALIGNED_DEG",
    "LIVE_FINAL_M",
    "LIVE_RUNWAY_M",
    "LIVE_TAXIWAY_M",
    "LIVE_STAND_M",
    "LIVE_AHEAD_DEG",
    "LIVE_TAXI_IN_KT",
    "LIVE_ROLL_MS",
    "LIVE_PARKED_S",
    "M_PER_DEG",
    "FT_PER_HPA",
    "PILOT_FINAL_NM",
    "LIVE_GO_AROUND_NM",
    "LIVE_BEHIND_NM",
    "RELEASE_GAP_S",
    "HOLD_SHORT_EVERY_S",
    "GIVE_WAY_M",
    "GIVE_WAY_DEG",
    "OBSERVED_RANGE_NM",
    "OBSERVED_CEILING_FT",
    "ROLLING_KT",
    "TAXIING_KT",
    "PUSHBACK_KT",
    "PUSHBACK_DEG",
    "PARKED_RANGE_NM",
    "LANDING_CLEARANCE_NM",
    "OBSERVED_LIMIT",
    "AIRBORNE_BY_FT",
    "_SPEED",
    "_FLYING_PHASES",
    "PHASES",
    "GLIDE_FT_PER_NM",
    "APPROACH_SPEED_KT",
    "ARRIVAL_SPEED_KT",
    "SLOWING_FROM_NM",
    "STABLE_BY_NM",
    "_squawk",
    "observed_type",
    "_MODEL_NAMES",
    "approach_speed",
    "Call",
    "AIRLINER_FOOTPRINT_M",
    "LIGHT_FOOTPRINT_M",
    "ParkedObserved",
    "UserAircraft",
    "RUNWAY_WIDTH_NM",
    "RUNWAY_LENGTH_NM",
    "LINED_UP_DEG",
    "_ENROUTE_POSITIONS",
    "_is_enroute_standin",
    "_Field",
    "_said_route",
    "REST_OF_TAXI_M",
    "_spell_route",
    "_local_or_english",
    "_DESTINATIONS",
    "DESTINATION_AIRPORTS",
    "logging",
    "math",
    "random",
    "time",
    "dataclass",
    "field",
    "decision",
    "project",
    "same_channel",
    "NavDB",
    "Station",
    "bearing_deg",
    "haversine_nm",
    "normalize_runway",
    "circuitry",
    "fleets",
    "groundwork",
    "livemod",
    "perftable",
    "every_language",
    "CIRCUIT_BASE_NM",
    "CIRCUIT_CENTRELINE_NM",
    "CIRCUIT_UPWIND_NM",
    "CIRCUIT_WIDTH_NM",
    "AIRLINER",
    "FLARE_FPM",
    "FLARE_FT",
    "LIFTOFF_AFTER_S",
    "OWN_DECISIONS",
    "Command",
    "Performance",
    "Target",
    "glide_altitude",
    "offset_from_track",
    "LIGHT_PERFORMANCE",
    "fly_step",
    "phraseology_for",
    "reply_language",
    "field_hour",
    "host_hour",
    "FREIGHT",
    "LIGHT_FLEET",
    "REGIONAL",
    "WIDEBODY",
    "Operator",
    "by_icao",
    "registration_prefix",
    "roster_for",
    "Aircraft",
    "Weather",
    "wake_suffix",
    "PilotSpeech",
]
