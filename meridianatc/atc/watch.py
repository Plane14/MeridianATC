"""What the controller notices without being told.

Everything else in this package answers the pilot. This watches the aircraft.

The two are not the same job, and leaving the second one out is why a flight
could take a runway, depart and land again without a single clearance and never
hear a word about it. A controller sitting in the cab is not waiting to be
addressed; they are looking out of the window, and an aircraft that rolls
without a clearance gets called on it before it is airborne.

This module is the surface half of it. The other two are
:mod:`meridianatc.atc.departure`, for a flight the tower has put on the runway
and told to wait, and :mod:`meridianatc.atc.arrival`, for one nobody has said
anything to on the way in.

Three things are worth calling, and they are exactly the three where being
somewhere without a clearance is an event in its own right rather than a matter
of taste:

* **taking a runway** you have not been cleared onto;
* **departing** without a takeoff clearance;
* **landing** without a landing clearance.

Each is said once. A controller does not repeat an advisory four times a second
because the simulator is being polled that often.

What is deliberately *not* called is crossing a runway the route never
mentioned. Real ATC issues a crossing clearance for each runway a taxi
route meets, and with the simulator's taxiways this project does too:
:class:`CrossingWatch` grants the crossing as the aircraft reaches the
hold, repeats the hold-short while the runway cannot be crossed, and calls
an entry nobody cleared. Without the simulator the route is invented and
the aircraft is bound to cross runways it never mentioned, so invented
routes still run the old way -- saying nothing about crossings at all. The
discriminator for those is alignment: an aircraft lined up on a runway is
pointing along it, and an aircraft crossing one is pointing across it.
"""

from __future__ import annotations

import logging
import math
import time

from ..logs import decision
from ..navdata.build import bearing_deg, project
from ..navdata.db import NavDB, Station
from .controller import ControllerBrain, Reply
from .phraseology import language_of
from .session import FlightSession, Phase

log = logging.getLogger(__name__)

# How far from the field the surface watch still applies. Beyond this the
# aircraft is flying, not manoeuvring on an airport.
FIELD_RADIUS_NM = 8.0

# An aircraft pointing within this many degrees of the runway heading is lined
# up on it. Anything further off is crossing it, which is not called.
ALIGNED_DEG = 35.0

# A takeoff roll and a landing rollout both pass through this speed. Requiring
# it stops a simulator that has just connected, or an aircraft placed in the
# air by the user, from reading as a departure.
ROLLING_KT = 20.0


class SurfaceWatch:
    """Watches the aircraft and speaks when it does something uncleared."""

    def __init__(self, navdb: NavDB, brain: ControllerBrain):
        self.navdb = navdb
        self.brain = brain

    # ------------------------------------------------------------------

    def look(
        self,
        session: FlightSession,
        station: Station | None,
        state,
        previous,
        language: str = "en",
    ) -> Reply | None:
        """One look at the aircraft. Returns something to say, or nothing.

        ``previous`` is the state at the last poll, which is what makes a
        takeoff or a touchdown visible: both are transitions, and neither can
        be read off a single snapshot.
        """
        if not self.brain.issues_clearances(station):
            # An uncontrolled field has nobody in the cab, and a broadcast
            # frequency cannot transmit to one aircraft. Silence is correct.
            # Nor does an information service: at Kortrijk's AFIS, after
            # "take off at your discretion", this called the line-up an
            # incursion and the takeoff a pilot deviation.
            return None
        if state is None or previous is None or not state.has_position:
            return None

        airport = self.navdb.airport(station.ident)
        if airport is None:
            return None
        if state.distance_to(airport.lat, airport.lon) > FIELD_RADIUS_NM:
            return None

        # A long taxi cut in two: at the taxiway it was told to hold short
        # of, the rest of the route.
        if station.position in ("GND", "TWR", "DEL"):
            rest = self.brain.rest_of_taxi(
                session, station, self.brain.for_station(station, language),
                state)
            if rest is not None:
                return rest

        runway = self._runway_under(airport.ident, state)
        if state.on_ground:
            session.occupying_runway = runway
            if not runway:
                # Off the pavement again. Entering later is a fresh event, so
                # the advisory for each runway is armed once more. A runway
                # is the strip, both its ends (``NavDB.runway_strip``): one
                # entry backtracked along 06 to the 24 threshold was two.
                session.advisories -= {
                    key for key in session.advisories if key.startswith("incursion:")
                }

        p = self.brain.for_station(station, language)
        aircraft = session.aircraft
        # The callsign is spoken differently in each language, and an advisory
        # is a transmission like any other, so the aircraft is told which one
        # this is in before a template runs.
        aircraft.language = language_of(p)

        # --- departed without a takeoff clearance ---------------------
        # Not a liftoff off a landing rollout: a bounce, a touch-and-go or a
        # go-around after touchdown is classified by the engine, and none of
        # them is a takeoff (``session.rollout_liftoff_at``).
        if (previous.on_ground and not state.on_ground
                and previous.ground_speed_kt > ROLLING_KT
                and not session.cleared_takeoff
                and not session.rollout_liftoff_at
                and session.advise_once("departed_without_clearance")):
            used = session.occupying_runway or session.departure_runway or runway
            decision("unauthorised_departure", runway=used,
                     speed_kt=round(float(previous.ground_speed_kt or 0.0)),
                     phase=session.phase.value)
            return Reply(
                p.departed_without_clearance(
                    aircraft, used, self._tower_name(station, p)
                ),
                station,
            )

        # --- landed without a landing clearance -----------------------
        if (not previous.on_ground and state.on_ground
                and state.ground_speed_kt > ROLLING_KT
                and not session.cleared_landing
                and not session.cleared_option
                and session.advise_once("landed_without_clearance")):
            used = runway or session.occupying_runway or session.arrival_runway
            return Reply(
                p.landed_without_clearance(
                    aircraft, used, self._tower_name(station, p)
                ),
                station,
            )

        # --- lined up on a runway without a clearance -----------------
        if (state.on_ground and runway and not self._allowed(
                airport.ident, session, runway)
                and self._aligned(runway, self.magnetic_heading(state))):
            key = f"incursion:{self.navdb.runway_strip(airport.ident, runway)}"
            if session.advise_once(key):
                session.tell_to_wait(hold=True)
                return Reply(p.hold_position_incursion(aircraft, runway), station)

        return None

    # ------------------------------------------------------------------

    def _runway_under(self, ident: str, state) -> str:
        # True, because the runway ends it is compared with are true.
        heading = getattr(state, "heading_true", None)
        try:
            return self.navdb.runway_at(
                ident, state.latitude, state.longitude, heading
            )
        except Exception:
            log.debug("runway lookup failed at %s", ident, exc_info=True)
            return ""

    def _allowed(self, ident: str, session: FlightSession, runway: str) -> bool:
        """Whether a clearance the aircraft holds covers this runway -- the
        whole strip, so an aeroplane cleared onto 24 may backtrack along it
        pointing at 06."""
        cleared = session.cleared_runway
        if not cleared:
            return False
        return (self.navdb.runway_strip(ident, cleared)
                == self.navdb.runway_strip(ident, runway))

    @staticmethod
    def magnetic_heading(state) -> float | None:
        """The aeroplane's heading from magnetic north, which is what a
        runway's number is.

        Worked out from the true heading and the model rather than read off
        the simulator's magnetic one, which a stand-in source leaves at
        zero -- and ``heading_true or heading_magnetic`` took a real heading
        of zero for no heading at all.
        """
        from ..navdata.magvar import to_magnetic, variation

        heading = getattr(state, "heading_true", None)
        if heading is None or not getattr(state, "has_position", False):
            return heading
        return to_magnetic(heading, variation(state.latitude, state.longitude))

    @staticmethod
    def _aligned(runway: str, heading: float | None) -> bool:
        """Lined up along the runway rather than crossing it.

        ``heading`` is magnetic: the runway's number is. Compared with a
        true heading it was out by the variation, which at Kennedy turned a
        forty-degree crossing into a line-up.
        """
        if heading is None:
            return False
        from ..navdata.db import _heading_from_ident

        runway_heading = _heading_from_ident(runway)
        if runway_heading is None:
            return False
        apart = abs(((float(heading) - runway_heading) + 180.0) % 360.0 - 180.0)
        return apart <= ALIGNED_DEG

    def _tower_name(self, station: Station, p) -> str:
        """Who to call about it afterwards: the tower, whoever you are on."""
        tower = self.navdb.station_for(station.ident, "TWR") or station
        return tower.callsign_in(language_of(p))


# A hold-short refused for traffic is not said again for this long: the
# runway is the picture to watch, and the aeroplane is stopped at it.
CROSSING_HOLD_S = 60.0


class CrossingWatch:
    """The runways a real taxi route holds the aircraft short of.

    Every runway the simulator's own taxiways take the aircraft across is a
    clearance of its own (:meth:`FlightSession.pending_crossing`): the taxi
    clearance names the first, and each is then granted or refused as the
    aeroplane gets to it, the way a ground controller works them. Granted,
    the runway becomes the one the aircraft may be on until it is off the
    far side; refused -- somebody on the strip, an arrival inside the
    window -- the hold-short is repeated, at most once a minute. Entered
    without the clearance it is an incursion like any other, called once.
    """

    def __init__(self, navdb: NavDB, brain: ControllerBrain):
        self.navdb = navdb
        self.brain = brain
        # When a hold was last said for a crossing, by runway strip.
        self._held: dict[str, float] = {}

    def look(self, session: FlightSession, station: Station | None, state,
             previous, language: str = "en") -> Reply | None:
        if station is None or not self.brain.issues_clearances(station):
            return None
        if station.position not in ("GND", "DEL", "TWR"):
            return None
        if state is None or previous is None or not state.has_position:
            return None
        if not session.taxi_crossings:
            return None
        airport = self.navdb.airport(station.ident)
        if airport is None or not state.on_ground:
            return None
        if state.distance_to(airport.lat, airport.lon) > FIELD_RADIUS_NM:
            return None

        ident = airport.ident
        pending = session.pending_crossing()
        if not pending:
            return None
        strip = self.navdb.runway_strip(ident, pending)
        before = self._runway_under(ident, previous)
        under = session.occupying_runway or self._runway_under(ident, state)
        on_pending = bool(under) and \
            self.navdb.runway_strip(ident, under) == strip
        was_pending = bool(before) and \
            self.navdb.runway_strip(ident, before) == strip

        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)

        # Off the far side, or back to the hold side: the crossing is done
        # either way and the next runway on the route is the new limit.
        if was_pending and not on_pending:
            crossed = session.spend_crossing()
            decision("runway_crossing", airport=ident, event="vacated",
                     runway=crossed)
            pending = session.pending_crossing()
            if not pending:
                return None
            strip = self.navdb.runway_strip(ident, pending)
            on_pending = bool(under) and \
                self.navdb.runway_strip(ident, under) == strip

        # On the runway without a crossing clearance, granted or spent --
        # the same call the surface watch makes for a line-up, with the
        # alignment test it cannot make for a crossing.
        if on_pending and not session.crossing_granted:
            key = f"incursion:cross:{strip}"
            if session.advise_once(key):
                decision("runway_crossing", airport=ident,
                         event="incursion", runway=under)
                session.tell_to_wait(hold=True)
                return Reply(
                    p.hold_position_incursion(session.aircraft,
                                              under or pending),
                    station)
            return None

        if on_pending:
            # On the strip with the clearance: nothing to decide.
            return None

        # Held short of it, or still on the way: the grant -- or the
        # hold-short again, no oftener than once a minute.
        last = self._held.get(strip, 0.0)
        remind = time.monotonic() - last >= CROSSING_HOLD_S
        reply = self.brain.crossing_clearance(
            session, station, p, state, remind=remind)
        if reply is not None and not session.crossing_granted:
            self._held[strip] = time.monotonic()
        return reply

    def _runway_under(self, ident: str, state) -> str:
        heading = getattr(state, "heading_true", None)
        try:
            return self.navdb.runway_at(
                ident, state.latitude, state.longitude, heading
            )
        except Exception:
            log.debug("runway lookup failed at %s", ident, exc_info=True)
            return ""


class TransponderWatch:
    """The code the transponder shows against the one the flight was given.

    A squawk that is never set is a target the radar room does not have.
    The watch gives the pilot thirty seconds from the assigned code's being
    issued to set it, then asks once a minute while it stays wrong. Codes
    nobody can set -- emergency squawks, which are the pilot's to choose --
    and observations that are not four digits are left alone.
    """

    # From the clearance to the box: how long setting a code takes before
    # it is worth asking about.
    SETTLE_S = 30.0
    # And how often it is worth asking again.
    REMIND_S = 60.0
    EMERGENCY = {"7500", "7600", "7700"}

    def __init__(self, navdb: NavDB, brain: ControllerBrain):
        self.navdb = navdb
        self.brain = brain
        # The assigned code as last seen, so the settle clock starts when
        # the code changes, not when the watch starts.
        self._expected = ""
        self._since = 0.0
        self._reminded = 0.0

    def look(self, session: FlightSession, station: Station | None, state,
             language: str = "en") -> Reply | None:
        if station is None or not self.brain.issues_clearances(station):
            return None
        if state is None or not session.established_with(station):
            return None
        if not session.squawk_assigned:
            return None
        expected = (session.squawk or "").strip()
        observed = (getattr(state, "transponder_code", "") or "").strip()
        now = time.monotonic()
        if expected != self._expected:
            self._expected, self._since = expected, now
            self._reminded = 0.0
        if not expected or not observed:
            return None
        if observed in self.EMERGENCY or expected in self.EMERGENCY:
            return None
        if not (observed.isdigit() and len(observed) == 4):
            return None
        if observed == expected:
            return None
        if now - self._since < self.SETTLE_S \
                or now - self._reminded < self.REMIND_S:
            return None
        self._reminded = now
        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        decision("squawk_mismatch", station=station.callsign,
                 expected=expected, observed=observed)
        return Reply(
            p.verify_squawk(session.aircraft, expected, observed), station)


class TaxiWatch:
    """Whether the taxiing aeroplane is still on the route it was given.

    A route drawn from the simulator's own taxiways
    (``FlightSession.taxi_path``) is checkable: the aeroplane's distance
    from the line is knowable, and when it has sat off it, moving, for a
    while the ground controller's real next call is to ask whether the
    crew wants help. Answered yes, the route is planned again from where
    the aeroplane actually is (:meth:`FlightSession.offer_assistance`).
    """

    # Far enough from the cleared line that no taxiway width explains it:
    # a wide apron connector is some 60-70 m, so 90 is "off the route".
    OFF_ROUTE_NM = 90.0 / 1852.0
    # Off the line, moving, this long before it is said -- turns and
    # stopped traffic are not a deviation.
    SETTLE_S = 20.0
    MOVING_KT = 6.0
    # The give-way call: on the corridor the route uses, this far ahead.
    # Narrower than a parallel taxiway's spacing and far enough to be
    # said before it is met, whatever either aeroplane is doing.
    CONFLICT_CORRIDOR_M = 30.0
    GIVE_WAY_M = 150.0

    def __init__(self, navdb: NavDB, brain: ControllerBrain):
        self.navdb = navdb
        self.brain = brain
        self._off_since: float | None = None
        # The aeroplanes a give-way has been said for on this taxi --
        # cleared when the corridor is, so a fresh conflict is a fresh
        # call rather than a silent second one.
        self._advised: set[str] = set()

    def look(self, session: FlightSession, station: Station | None, state,
             language: str = "en") -> Reply | None:
        if station is None or station.position != "GND":
            return None
        if session.phase not in (Phase.TAXI_OUT, Phase.TAXI_IN):
            self._off_since = None
            self._advised.clear()
            return None
        path = session.taxi_path
        if len(path) < 2 or state is None or not state.has_position \
                or not state.on_ground:
            return None
        # On the route, ahead of the off-route question, is the collision
        # one: whatever the rest of the line is about to meet is given
        # way to, and its passing answered with "continue taxi".
        if self._off_route_nm(state, path) <= self.OFF_ROUTE_NM:
            self._off_since = None
            return self._give_way(session, station, state, path, language)
        if (getattr(state, "ground_speed_kt", 0.0) or 0.0) < self.MOVING_KT:
            return None
        now = time.monotonic()
        if self._off_since is None:
            self._off_since = now
            return None
        if now - self._off_since < self.SETTLE_S:
            return None
        self._off_since = None
        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        if session.offer_assistance("taxi", now):
            decision("taxi_off_route", station=station.callsign,
                     distance_nm=round(self._off_route_nm(state, path), 3))
            return Reply(p.taxi_off_route(session.aircraft), station)
        return None

    def _give_way(self, session: FlightSession, station: Station, state,
                  path, language: str) -> Reply | None:
        """"Give way to the ..." for whatever the cleared route meets.

        The controller's whole picture answers -- the invented traffic,
        the live aeroplanes this program flies, and the simulator's own,
        which obey nobody: a give-way is exactly the call for the one the
        ground cannot instruct. Said once per aeroplane per conflict, and
        "continue taxi" once the corridor is clear again.
        """
        from .auto_nav import traffic_ahead

        contacts = getattr(self.brain, "contacts", None)
        if contacts is None:
            return None
        try:
            found = traffic_ahead(state.latitude, state.longitude,
                                  list(path), contacts(),
                                  corridor_m=self.CONFLICT_CORRIDOR_M,
                                  look_m=self.GIVE_WAY_M)
        except Exception:
            log.debug("the give-way check raised", exc_info=True)
            return None
        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        # Moving traffic is a conflict whatever its distance; standing
        # traffic only once it is genuinely in the way -- a parked
        # aeroplane thirty metres off the line is not a call to make.
        relevant = [one for one in found
                    if (one[0].ground_speed_kt or 0.0) >= 3.0
                    or one[1] <= 95.0]
        if relevant:
            contact = relevant[0][0]
            key = getattr(contact, "written", "") or repr(id(contact))
            if key in self._advised:
                return None
            self._advised.add(key)
            decision("taxi_give_way", station=station.callsign,
                     traffic=key,
                     ahead_m=round(relevant[0][1]))
            return Reply(p.give_way(session.aircraft,
                                    contact.describe(language)), station)
        if self._advised:
            self._advised.clear()
            return Reply(p.continue_taxi(session.aircraft), station)
        return None

    @staticmethod
    def _off_route_nm(state, path) -> float:
        """Distance to the cleared line, in nm -- equirectangular about the
        aircraft, close enough at taxi distances."""
        kx = 60.0 * math.cos(math.radians(state.latitude))
        xy = [((lon - state.longitude) * kx,
               (lat - state.latitude) * 60.0) for lat, lon in path]
        best = math.inf
        for (ax, ay), (bx, by) in zip(xy, xy[1:], strict=False):
            dx, dy = bx - ax, by - ay
            if dx or dy:
                t = max(0.0, min(1.0,
                                 -(ax * dx + ay * dy) / (dx * dx + dy * dy)))
                ax, ay = ax + t * dx, ay + t * dy
            best = min(best, math.hypot(ax, ay))
        return best


class DeviationAssistWatch:
    """One deviation call too many, when it becomes an offer of help.

    A crew that is off its assigned altitude, heading or speed once is
    told so; told a third time, the real call is not the same call again
    but "say your intentions" -- the controller works out whether the
    aeroplane can fly what it was cleared for. Each kind of check gets
    its own count; the offer is made once through
    :meth:`FlightSession.offer_assistance`, and whatever the pilot then
    asks for is answered as a fresh clearance.
    """

    ALT_FT = 200.0
    HDG_DEG = 20.0
    SPD_KT = 15.0
    # Off this long before it counts: a turn through the assigned heading
    # or a slow climb through the level is not a deviation.
    SETTLE_S = 15.0
    # A reminder per kind, until the third becomes the offer.
    REMIND_S = 60.0
    OFFER_AFTER = 2
    # A return to profile this brief is still the same excursion: a
    # crew weaving in and out in bursts would otherwise start a fresh
    # settle on every one and never be warned at all.
    HOLD_S = 60.0

    def __init__(self, navdb: NavDB, brain: ControllerBrain):
        self.navdb = navdb
        self.brain = brain
        # kind -> (since, last said, said so far)
        self._seen: dict[str, list] = {}

    def look(self, session: FlightSession, station: Station | None, state,
             language: str = "en") -> Reply | None:
        if station is None or not self.brain.issues_clearances(station):
            return None
        if station.position == "GND" or state is None or state.on_ground:
            self._seen.clear()
            return None
        now = time.monotonic()
        off = self._off_now(session, state)
        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        for kind in off:
            since, last, said = self._seen.get(kind, [now, 0.0, 0])[:3]
            # On the excursion again: whatever the hold window left is
            # the same episode, and its settle clock keeps running.
            self._seen[kind] = [since, last, said, 0.0]
            if now - since < self.SETTLE_S:
                continue
            if now - last < self.REMIND_S:
                continue
            if said < self.OFFER_AFTER:
                self._seen[kind] = [since, now, said + 1, 0.0]
                decision("deviation", check=kind,
                         station=station.callsign, count=said + 1)
                return Reply(
                    self._verify(kind, p, session, state), station)
            if session.offer_assistance("flight", now):
                self._seen.clear()
                decision("assist", check=kind,
                         station=station.callsign)
                return Reply(p.offer_assistance(session.aircraft), station)
            # The offer was made recently and stands -- the deviation is
            # still there, so the call is still made. Going quiet for
            # the hour was a controller who had stopped watching.
            self._seen[kind] = [since, now, said + 1, 0.0]
            decision("deviation", check=kind,
                     station=station.callsign, count=said + 1)
            return Reply(
                self._verify(kind, p, session, state), station)
        # Back within limits is not the end of the episode until it has
        # lasted: a kind that comes off again inside HOLD_S resumes the
        # clock it left, and one that stays inside is a new episode.
        for kind in list(self._seen):
            if kind in off:
                continue
            entry = self._seen[kind][:3] + [self._seen[kind][3]
                                          if len(self._seen[kind]) > 3
                                          and self._seen[kind][3] else now]
            self._seen[kind] = entry
            if now - entry[3] >= self.HOLD_S:
                del self._seen[kind]
        return None

    def _off_now(self, session: FlightSession, state) -> list[str]:
        out = []
        # Mode C is QNH-corrected by the radar, so what the controller sees
        # is true altitude whatever the Kollsman window says -- a mis-set
        # altimeter is a deviation here exactly as it would be on a scope.
        if session.assigned_altitude_ft:
            delta = (session.assigned_altitude_ft
                     - (getattr(state, "altitude_ft", 0.0) or 0.0))
            if abs(delta) > self.ALT_FT:
                # A leg climbing or descending *toward* the assigned
                # level is doing exactly what it was cleared to do --
                # below 9,000 on the way to 9,000 is not a deviation,
                # the way it is when the aeroplane is level or moving
                # away. Only the ones not converging are challenged.
                # "Converging" is any rate a crew would call a climb or
                # a descent: a light single near its ceiling makes two
                # hundred a minute, and the last few hundred feet onto a
                # level are flown at three hundred or less by anybody.
                vs = getattr(state, "vertical_speed_fpm", 0.0) or 0.0
                closing = (delta > 0.0 and vs >= 100.0) \
                    or (delta < 0.0 and vs <= -100.0)
                if not closing:
                    out.append("altitude")
        if session.assigned_heading \
                and _heading_off(self._magnetic(state),
                                 session.assigned_heading) > self.HDG_DEG:
            out.append("heading")
        if session.assigned_speed_kt \
                and abs((getattr(state, "indicated_airspeed_kt", 0.0)
                         or getattr(state, "true_airspeed_kt", 0.0) or 0.0)
                        - session.assigned_speed_kt) > self.SPD_KT:
            out.append("speed")
        return out

    @staticmethod
    def _magnetic(state) -> float:
        """The aeroplane's magnetic heading: assigned headings are magnetic.

        The simulator's own magnetic value where it gives one; the true
        heading corrected by the local variation else -- a stand-in source
        leaves the magnetic at zero, and comparing an assigned heading with
        a true one deviates by the variation itself.
        """
        heading = float(getattr(state, "heading_magnetic", 0.0) or 0.0)
        if heading or not getattr(state, "has_position", False):
            return heading
        from ..navdata.magvar import to_magnetic, variation
        true = float(getattr(state, "heading_true", 0.0) or 0.0)
        return to_magnetic(true, variation(state.latitude, state.longitude))

    @staticmethod
    def _verify(kind: str, p, session: FlightSession, state) -> str:
        aircraft = session.aircraft
        if kind == "altitude":
            return p.verify(aircraft, "altitude")
        if kind == "heading":
            return p.verify(aircraft, "heading")
        return p.verify(aircraft, "speed")


def _heading_off(a: float, b: float) -> float:
    """Degrees the actual heading is off the assigned, 0-180."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


# ----------------------------------------------------------------------
# the airspace
# ----------------------------------------------------------------------

# What each class asks of a VFR flight before it enters (ICAO Annex 11
# 2.6; FAA AIM 3-2): A is closed to VFR and so is a prohibited area, B
# needs a clearance and so do restricted and danger areas, C and D need
# two-way contact (and, outside the US, the tower's leave to enter), E,
# F and G need nothing. The simulator's files carry R, P and Q of their
# own (navdata.simspace), which the CSV classes never did.
ENTRY_RULES = {"A": "no_vfr", "P": "no_vfr", "B": "clearance",
               "R": "clearance", "Q": "clearance",
               "C": "contact", "D": "contact"}
# How far ahead the watch looks for the edge: a minute at the aeroplane's
# ground speed, which is the warning a controller tries to give.
LOOKAHEAD_S = 60.0
# Below this an aeroplane is not going anywhere fast enough to warn.
MOVING_KT = 40.0
# A climb or descent this quick is looked along too, a minute's worth.
CLIMBING_FPM = 300.0
# The positions that give a VFR flight a radar service.
RADAR = ("DEP", "APP", "CTR")


class AirspaceWatch:
    """A VFR aeroplane about to enter airspace it needs something for.

    The infringement half of a controller's job, which nothing did: a
    Cessna could fly through Denver's Class B at 9,000 ft and hear nothing.
    Looking a minute ahead along the track, with the volumes from
    ``NavDB.airspace_at``:

    * an aeroplane that has not called the unit that owns the volume, and
      is listening to it, is told it is entering without clearance, given a
      heading out of it and told to remain outside -- firmly, once;
    * one that is on frequency is let in where two-way contact is not
      enough on its own: outside the US by "enter the zone via (point)",
      and anywhere a Class B by being told to remain clear until it is
      cleared into it.

    Only the volume's own unit speaks -- the field the volume belongs to --
    because nobody else is listening for that aeroplane. Each volume is
    said once.
    """

    def __init__(self, navdb: NavDB, brain: ControllerBrain):
        self.navdb = navdb
        self.brain = brain

    def _speaks_for(self, volume, station: Station) -> bool:
        """Whether the unit on this frequency owns the volume.

        The field it is filed under, and for a Class B or C the radar room
        that works it, whichever field that room is filed at: Denver's Class
        B belongs to KDEN in the data and is Denver Departure's on
        Centennial's 132.750. A Class D is the tower's own.
        """
        if self.navdb.volume_airport(volume) == station.ident:
            return True
        if (station.position not in RADAR
                or volume.cls not in ("B", "C", "P", "Q", "R")):
            return False
        room = (station.facility or "").strip().lower()
        if not room:
            return False
        if volume.name.lower().startswith(room + " "):
            return True
        return any(other.facility.strip().lower() == room
                   for other in self.navdb.stations(
                       self.navdb.volume_airport(volume))
                   if other.position in RADAR)

    def look(self, session: FlightSession, station: Station | None, state,
             language: str = "en") -> Reply | None:
        # Not on an information service: it clears nobody into anything, so
        # it has no "without a clearance" to say.
        if station is None or not self.brain.issues_clearances(station):
            return None
        if session.ifr or session.emergency or state is None:
            return None
        if not state.has_position or state.on_ground:
            return None
        speed = float(getattr(state, "ground_speed_kt", 0.0) or 0.0)
        if speed < MOVING_KT:
            return None
        from .final import course_over_ground

        track = course_over_ground(state)
        if track is None:
            return None
        altitude = float(state.altitude_ft or 0.0)
        here = {v.id for v in self.navdb.airspace_at(
            state.latitude, state.longitude, altitude)}
        lat, lon = project(state.latitude, state.longitude, track,
                           speed * LOOKAHEAD_S / 3600.0)
        ahead = [(lat, lon, altitude)]
        # And through a floor or a ceiling: climbing at 8,200 ft into the
        # shelf over Centennial was never across an edge at the same height,
        # so it was never seen.
        climb = float(getattr(state, "vertical_speed_fpm", 0.0) or 0.0)
        if abs(climb) >= CLIMBING_FPM:
            level = altitude + climb * LOOKAHEAD_S / 60.0
            ahead += [(state.latitude, state.longitude, level),
                      (lat, lon, level)]
        entering, seen = [], set(here)
        for at_lat, at_lon, at_ft in ahead:
            for v in self.navdb.airspace_at(at_lat, at_lon, at_ft):
                if v.id not in seen and v.cls in ENTRY_RULES:
                    seen.add(v.id)
                    entering.append(v)
        severity = {"no_vfr": 0, "clearance": 1, "contact": 2}
        entering = [v for v in sorted(
            entering, key=lambda v: severity[ENTRY_RULES[v.cls]])
            if self._speaks_for(v, station)]
        if not entering:
            return None
        volume = entering[0]
        key = f"airspace:{volume.id}"
        if key in session.advisories:
            return None
        rule = ENTRY_RULES[volume.cls]
        established = session.established_with(station)
        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        airspace = p.airspace(volume.cls)
        if established and rule == "contact":
            if p.region == "faa":
                return None
            if (session.transition_approved or session.expected_vrp
                    or session.pattern_leg or session.phase.name in (
                        "APPROACH", "LANDING")):
                return None
            session.advise_once(key)
            airport = self.navdb.airport(self.navdb.volume_airport(volume))
            point = self.brain.vrp_ahead(airport, state) if airport else None
            text = p.enter_zone(session.aircraft, airspace,
                                p.vrp(point.name) if point else "")
            decision("airspace", volume=volume.name, cls=volume.cls,
                     said="enter", via=point.name if point else None)
            return Reply(text, station)
        if established and rule == "clearance":
            # A landing or Bravo clearance covers the field it names, not
            # the restricted and danger areas along the way.
            if volume.cls == "B" and (
                    session.cleared_bravo == self.navdb.volume_airport(volume)
                    or session.cleared_landing):
                return None
            session.advise_once(key)
            decision("airspace", volume=volume.name, cls=volume.cls,
                     said="remain_clear")
            return Reply(p.remain_clear_of_airspace(session.aircraft, airspace),
                         station)
        if established and rule != "no_vfr":
            return None
        # Not on frequency, or Class A: the way out is straight away from
        # the middle of the volume, as a magnetic heading.
        session.advise_once(key)
        south, west, north, east = volume.bbox
        away = bearing_deg((south + north) / 2.0, (west + east) / 2.0,
                           state.latitude, state.longitude)
        from ..navdata.magvar import to_magnetic, variation

        heading = round(to_magnetic(away, variation(state.latitude,
                                                    state.longitude)) / 10.0) * 10.0
        heading = heading % 360.0 or 360.0
        turn = "right" if ((away - track) % 360.0) < 180.0 else "left"
        name = self.brain.station_name(station, p) if not established else ""
        decision("airspace", volume=volume.name, cls=volume.cls,
                 said="infringement", heading=heading, turn=turn,
                 established=established)
        return Reply(p.infringement(session.aircraft, name, airspace,
                                    p.side(turn), heading), station)


class ReportingPointWatch:
    """A reporting point a VFR flight was told to report and flew past
    without a word: "say position", once.

    The arrival's entry point (``session.expected_vrp``) is past when the
    aeroplane is nearer the field than the point is; the departure's exit
    point (``session.expected_exit``) when it is further out than the point.
    Either way it is half a mile past it and still more than half a mile
    from it, so an aeroplane flying over the point is not asked where it is.
    """

    PAST_NM = 0.5

    def __init__(self, navdb: NavDB, brain: ControllerBrain):
        self.navdb = navdb
        self.brain = brain

    def look(self, session: FlightSession, station: Station | None, state,
             language: str = "en") -> Reply | None:
        from ..navdata.build import haversine_nm
        from .session import Instruction, station_key

        if not self.brain.realism.reporting_points:
            return None
        if station is None or station.position != "TWR":
            return None
        if not self.brain.staffed(station) or not session.established_with(station):
            return None
        if session.ifr or state is None or not state.has_position or state.on_ground:
            return None
        field = self.navdb.airport(station.ident)
        if field is None:
            return None
        for name, outbound in ((session.expected_vrp, False),
                               (session.expected_exit, True)):
            if not name:
                continue
            point = next((v for v in self.navdb.vrps(field.ident)
                          if v.name == name), None)
            if point is None:
                continue
            point_out = haversine_nm(field.lat, field.lon, point.lat, point.lon)
            out = state.distance_to(field.lat, field.lon)
            there = state.distance_to(point.lat, point.lon)
            past = (out > point_out + self.PAST_NM if outbound
                    else out < point_out - self.PAST_NM)
            if not past or there <= self.PAST_NM:
                continue
            if not session.advise_once(f"vrp_chase:{field.ident}:{name}"):
                continue
            p = self.brain.for_station(station, language)
            session.aircraft.language = language_of(p)
            decision("vrp", airport=field.ident, use="chased", point=name,
                     outbound=outbound, out_nm=round(out, 2),
                     point_nm=round(point_out, 2), from_point_nm=round(there, 2))
            text = p.say_position(session.aircraft)
            instruction = session.remember(Instruction(
                "say_position", station_key(station), text, {},
                readback_required=False))
            return Reply(text, station, instruction=instruction)
        return None


class RunwayWatch:
    """The tower telling the frequency the runway in use has changed.

    The runway is kept until the wind takes it away
    (``ControllerBrain._wind_runway``); when it does, a pilot on the tower
    frequency is told -- everybody at once, "all stations, runway in use now
    two four", with the new information letter where the field has one --
    and a VFR flight still in the circuit for the old runway is told which
    way round to join for the new one. A flight cleared to land, lined up or
    cleared for takeoff keeps the runway it was given; one waiting on the
    ground is moved to the new one for its takeoff clearance.

    Said once per change and only to a flight that was given the old runway:
    the first look at a field, before any METAR has come, chooses from calm,
    and the real wind arriving is not a change anybody was told about.
    """

    def __init__(self, navdb: NavDB, brain: ControllerBrain, weather_for,
                 atis_for=None):
        self.navdb = navdb
        self.brain = brain
        self.weather_for = weather_for
        self.atis_for = atis_for

    def look(self, session: FlightSession, station: Station | None, state,
             language: str = "en") -> Reply | None:
        from .controller import _airborne
        from .session import Instruction, station_key

        if not self.brain.realism.runway_change:
            return None
        if station is None or station.position != "TWR":
            return None
        if not self.brain.staffed(station) or not session.established_with(station):
            return None
        airport = self.navdb.airport(station.ident)
        if airport is None:
            return None
        weather = self.weather_for(station.ident)
        # Asking is what notices the wind has taken the runway away.
        self.brain.arrival_runway(airport, weather)
        self.brain.departure_runway(airport, weather)
        change = self.brain.runway_changes.get(station.ident)
        if change is None:
            return None
        was, now = change
        heard = f"{station.ident}:{now}"
        if session.runway_change_heard == heard:
            return None
        session.runway_change_heard = heard
        if was not in (session.arrival_runway, session.departure_runway):
            decision("runway_change", airport=station.ident, said="nothing",
                     was=was, now=now, why="never given the old runway")
            return None

        p = self.brain.for_station(station, language)
        name = self.brain.station_name(station, p)
        def capital(sentence: str) -> str:
            return sentence[:1].upper() + sentence[1:]

        text = capital(p.runway_change(name, now))
        atis = self.atis_for(station.ident, language) if self.atis_for else None
        letter = getattr(atis, "letter", "") if atis is not None else ""
        if letter:
            pressure = (weather.altimeter_inhg if getattr(p, "dialect", "") == "faa"
                        else weather.qnh_hpa)
            text = f"{text} {capital(p.information_current(letter, pressure))}"

        airborne = _airborne(state)
        flight = "untouched"
        instruction = None
        if (airborne and session.visual and not session.cleared_landing
                and session.arrival_runway == was):
            session.arrival_runway = now
            session.allow_runway("")
            session.pattern_leg = ""
            session.expected_report = "downwind"
            session.expected_report_chased = False
            session.base_owed_behind = ""
            direction = self.brain._pattern_direction(session, None, p, airport, now)
            session.aircraft.language = language_of(p)
            rejoin = p.enter_pattern(session.aircraft, "", now, leg="",
                                     direction=direction, report=p.join_report())
            text = f"{text} {rejoin}"
            flight = "rejoin"
            instruction = session.remember(Instruction(
                "pattern_entry", station_key(station), rejoin,
                {"runway": now, "pattern_direction": session.pattern_direction}))
        elif (not airborne and session.departure_runway == was
                and not session.lined_up and not session.cleared_takeoff):
            session.departure_runway = now
            flight = "departure_moved"
        elif session.cleared_landing or session.cleared_takeoff or session.lined_up:
            flight = "kept"
        decision("runway_change", airport=station.ident, said="broadcast",
                 was=was, now=now, letter=letter or None, flight=flight)
        return Reply(text, station, instruction=instruction)


__all__ = ["AirspaceWatch", "ENTRY_RULES", "ReportingPointWatch",
           "RunwayWatch", "SurfaceWatch",
           "FIELD_RADIUS_NM", "ALIGNED_DEG", "ROLLING_KT"]


class RadarServiceWatch:
    """The end of a VFR flight's radar service, said by the controller.

    "Radar service terminated, squawk VFR, frequency change approved" was
    only ever said to flight following, at the edge of its range. A VFR
    departure the tower handed to Denver with a code got "radar contact"
    and then nothing, all the way north out of the area (Centennial, 26
    October). The service it has is the same, and ends the same way: once
    the aeroplane is plainly leaving -- ``LEAVING_NM`` out from the field
    the room is filed at, flying away from it, with no destination near
    it. Flight following is handed on by range instead (Engine
    ._offer_handoff), because the flight asked to be watched the whole way.
    """

    LEAVING_NM = 15.0
    # Within this of straight away from the field.
    AWAY_DEG = 70.0

    def __init__(self, navdb: NavDB, brain: ControllerBrain):
        self.navdb = navdb
        self.brain = brain

    def look(self, session: FlightSession, station: Station | None, state,
             language: str = "en") -> Reply | None:
        from ..navdata.build import haversine_nm
        from .final import course_over_ground
        from .session import Instruction, station_key

        if station is None or station.position not in RADAR:
            return None
        if session.ifr or session.emergency or session.flight_following:
            return None
        key = station_key(station)
        if session.radar_service != key:
            return None
        if state is None or not state.has_position or state.on_ground:
            return None
        airport = self.navdb.airport(station.ident)
        if airport is None:
            return None
        distance = haversine_nm(airport.lat, airport.lon,
                                state.latitude, state.longitude)
        if distance < self.LEAVING_NM:
            return None
        track = course_over_ground(state)
        away = bearing_deg(airport.lat, airport.lon,
                           state.latitude, state.longitude)
        if track is None or abs((track - away + 540.0) % 360.0 - 180.0) \
                > self.AWAY_DEG:
            return None
        destination = session.destination
        if destination is not None and haversine_nm(
                airport.lat, airport.lon, destination.lat,
                destination.lon) <= 2 * self.LEAVING_NM:
            # Going somewhere in this room's own area: it is kept.
            return None
        if not session.advise_once(f"radar:end:{key}"):
            return None
        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        session.radar_service = ""
        session.vfr_ceiling_ft = 0.0
        session.squawk = p.vfr_squawk
        text = p.radar_service_terminated(session.aircraft)
        decision("radar_service", station=station.callsign, said="terminated",
                 distance_nm=round(distance, 1), track=round(track))
        instruction = session.remember(Instruction(
            "frequency_change", key, text, {}, readback_required=False))
        return Reply(text, station, instruction=instruction)


class RestrictionWatch:
    """A VFR flight above the altitude a radar controller holds it under.

    At 8,200 ft into Denver's Class B shelf with "maintain VFR at or below
    seven thousand five hundred", nobody said a word: the restriction was
    given and never looked at again. ``ABOVE_FT`` over it the controller
    asks first -- an encoder can be wrong, and "say altitude" is how one is
    checked -- and if the answer leaves it up there, ``ANSWER_S`` later,
    tells it to descend back under. Back within ``BACK_UNDER_FT`` of the
    restriction, the next climb through it is asked about again.
    """

    ABOVE_FT = 300.0
    BACK_UNDER_FT = 100.0
    ANSWER_S = 15.0

    def __init__(self, navdb: NavDB, brain: ControllerBrain):
        self.navdb = navdb
        self.brain = brain

    def look(self, session: FlightSession, station: Station | None, state,
             now: float, language: str = "en") -> Reply | None:
        from .session import Instruction, station_key

        if station is None or station.position not in RADAR:
            return None
        if session.ifr or session.emergency:
            return None
        ceiling = float(session.vfr_ceiling_ft or 0.0)
        key = station_key(station)
        if not ceiling or session.radar_service != key:
            return None
        if state is None or not state.has_position or state.on_ground:
            return None
        altitude = float(state.altitude_ft or 0.0)
        if altitude < ceiling + self.BACK_UNDER_FT:
            session.restriction_asked_at = 0.0
            session.advisories.discard(f"restriction:descend:{key}")
            return None
        if altitude < ceiling + self.ABOVE_FT:
            return None
        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        if not session.restriction_asked_at:
            session.restriction_asked_at = now
            decision("vfr_restriction", station=station.callsign,
                     said="say_altitude", altitude_ft=round(altitude),
                     ceiling_ft=round(ceiling))
            return Reply(p.say_altitude(session.aircraft), station)
        if now - session.restriction_asked_at < self.ANSWER_S:
            return None
        if not session.advise_once(f"restriction:descend:{key}"):
            return None
        transition = self.brain._transition_altitude(
            self.navdb.airport(station.ident))
        text = p.descend_at_or_below(session.aircraft, ceiling,
                                     transition_altitude=transition)
        decision("vfr_restriction", station=station.callsign, said="descend",
                 altitude_ft=round(altitude), ceiling_ft=round(ceiling))
        instruction = session.remember(Instruction(
            "vfr_ceiling", key, text, {"vfr_ceiling_ft": ceiling}))
        return Reply(text, station, instruction=instruction)
