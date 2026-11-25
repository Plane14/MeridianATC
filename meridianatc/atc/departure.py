"""What the controller does about a departure it has already held.

The third of the watches, after :mod:`meridianatc.atc.watch` for the surface and
:mod:`meridianatc.atc.arrival` for the way in. All three exist for the same
reason: a controller is not a thing that answers, it is a thing that works a
flight, and everything here happens because of what the aeroplane is doing
rather than because somebody keyed a microphone.

Three watches. The first is for the moment the program used to lose the
flight entirely, below. The other two are the IFR plan's (Phase 1): the
climb given in steps before each is reached (:class:`ClimbWatch`), and a
flight off the SID it was cleared on (:class:`SidWatch`).

An aeroplane asks for takeoff with somebody landing behind it. The tower does
the right thing: it puts it on the runway and tells it to line up and wait.
Then the arrival lands, the runway clears, and nothing happens. There was no
path anywhere in the controller that went back to a departure it had held, so
the only way out was for the pilot to ask again -- and a pilot who has been
told to wait does not ask again, because they have been told to wait. The
aeroplane sat on the runway for the rest of the flight.

So: when the runway is free, the clearance the aeroplane is waiting for is
issued. That is the whole of this module.

What it does not do
-------------------

It does not un-hold anything else. A pilot told to hold short, hold position
or give way is being held for a reason that is about the ground rather than
about the runway, and the taxi routes here are synthetic -- the program does
not know enough about a taxiway to know when one is clear. The runway is the
one piece of an airport this program does model, and it is the one piece
where being left waiting is the difference between a flight and no flight.
"""

from __future__ import annotations

import logging

from ..logs import decision
from ..navdata.db import NavDB, Station
from . import climb
from .controller import ControllerBrain, Reply
from .phraseology import Weather, language_of
from .route import CLB
from .session import FlightSession, Instruction, Phase, station_key

log = logging.getLogger(__name__)

# How long to leave an aeroplane sitting there before the clearance goes out.
#
# Not for realism -- a real tower clears you the moment the wheels of the
# landing traffic are on the ground. It is for the arrival that has touched
# down and not yet slowed: the traffic model frees the runway on a timer, and
# a couple of seconds either side of that is the difference between a
# clearance and a clearance issued over the top of somebody's rollout.
SETTLE_S = 4.0


class DepartureWatch:
    """Releases a departure the tower has put on the runway and held."""

    def __init__(self, navdb: NavDB, brain: ControllerBrain):
        self.navdb = navdb
        self.brain = brain
        # When the runway was last seen to be occupied, so a clearance is not
        # issued in the same instant it comes free.
        self._busy_at = 0.0
        # Since when the cleared departure has not moved -- the abort watch.
        self._stopped_since = 0.0
        # The clearance timestamp the departure was seen rolling under --
        # the evidence that a stop is a rejection and not a lineup that
        # has never moved.
        self._rolled_for = 0.0

    # ------------------------------------------------------------------

    def look(
        self,
        session: FlightSession,
        station: Station | None,
        state,
        now: float,
        weather_for=None,
        language: str = "en",
    ) -> Reply | None:
        """One look at the departure. Returns the clearance, or nothing."""
        if station is None or station.position != "TWR":
            return None
        if state is None or not state.on_ground:
            return None
        if session.emergency:
            return None
        if session.phase in (Phase.LANDED, Phase.TAXI_IN):
            # Down and rolling out -- or on the taxiway out still inside
            # the runway strip: the runway wants it moving when somebody
            # is on final behind it, and "expedite" is the call for that.
            return self._expedite_the_rollout(
                session, station, state, now, language)
        if session.cleared_takeoff:
            if float(getattr(state, "ground_speed_kt", 0.0) or 0.0) \
                    >= self.ROLLING_KT:
                self._rolled_for = session.cleared_takeoff_at
            reply = self._cancel_for_incursion(
                session, station, state, now, language)
            if reply is not None:
                return reply
            reply = self._vacate_the_stopped(
                session, station, state, now, weather_for, language)
            if reply is not None:
                return reply
            if self.brain.realism.delays:
                return self._expedite(session, station, state, now, language)
            return None
        # Lined up and not yet cleared. Both halves matter: an aeroplane still
        # at the holding point has not been put anywhere it cannot leave, and
        # one already cleared is not waiting for anything. Held short for an
        # arrival counts as lined up: in the US nothing is lined up behind one.
        if not (session.lined_up or session.held_for_landing) \
                or session.cleared_takeoff:
            return None
        # And on its way out. An aeroplane that has landed and is still on the
        # runway is not waiting for a takeoff clearance.
        if not session.phase.outbound:
            return None

        field = self.navdb.airport(station.ident)
        if not self.brain.runway_available(field, session.departure_runway):
            self._busy_at = now
            return None
        if now - self._busy_at < SETTLE_S:
            return None

        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        weather = self._weather(weather_for, field, station)

        reply = self.brain.release_for_takeoff(session, station, weather, p,
                                               state)
        if reply is None or not reply.speak:
            return None
        # Only the clearance. Anything else the tower might have answered --
        # "hold position", "you are already cleared" -- is an answer to a
        # question, and nobody asked one.
        if not session.cleared_takeoff:
            return None
        # A fresh clearance after a cancel settles the flag it left: the
        # words that took the last one back are spent.
        session.takeoff_cancelled = False
        return reply

    # ------------------------------------------------------------------

    # Cleared and still standing this long, with an arrival this close on
    # the final: "expedite". UNVERIFIED: both figures.
    EXPEDITE_AFTER_S = 12.0
    EXPEDITE_WITHIN_NM = 3.0
    # Stopped this long on the runway after the clearance before the tower
    # works the aeroplane off it. An abort, a failure or a pilot who has
    # changed their mind all read the same way from the tower: stopped,
    # where the roll should be. Longer than the expedite watch above, which
    # is what answers a roll that is merely slow to start.
    ABORT_AFTER_S = 20.0
    # Faster than any lineup or backtrack drive, so only a takeoff roll
    # reads as rolling: a stop that follows one is the abort the watch is
    # for, where a lineup that has not moved yet is line-up-and-wait.
    ROLLING_KT = 20.0

    def _cancel_for_incursion(self, session, station, state, now,
                              language) -> Reply | None:
        """"Cancel takeoff clearance" (JO 7110.65 3-9-11): somebody on
        the strip a cleared departure is pointed at.

        Cancelled for safety and only for safety -- the rule's own
        words. While the roll has not started the call takes the
        clearance back and the release above re-gives a fresh one once
        the strip clears; once the aeroplane is rolling the clearance
        stays on the session so the stop is worked off exactly like a
        crew's own reject -- the abort watch's taxi-in, or the
        aeroplane's own re-lineup.
        """
        if session.takeoff_cancelled or not session.departure_runway:
            return None
        field = self.navdb.airport(station.ident)
        occupants = self.brain.runway_occupants(
            field, session.departure_runway)
        if not occupants:
            return None
        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        described = (occupants[0].describe(language_of(p))
                     if occupants else "")
        session.takeoff_cancelled = True
        if self._rolled_for != session.cleared_takeoff_at \
                or not session.cleared_takeoff_at:
            # Never rolled under this clearance: it is taken back now,
            # so a fresh one is re-issued once the runway is free --
            # not re-asked for by a crew already told it had one.
            session.cleared_takeoff = False
            session.cleared_takeoff_at = 0.0
        decision("takeoff_cancelled", airport=station.ident,
                 runway=session.departure_runway,
                 who=occupants[0].written or None)
        return Reply(
            p.cancel_takeoff(
                session.aircraft, p.traffic_on_runway(described)),
            station)

    def _vacate_the_stopped(self, session, station, state, now,
                          weather_for, language) -> Reply | None:
        """A departure that has stopped on the runway is worked off it.

        The clearance it was rolling on is gone and what it is now is an
        aeroplane that has to get off the runway -- given the same taxi to
        a stand an arrival gets, because the alternative was the one the
        log showed: an aeroplane holding mid-runway for the rest of the
        session while every arrival was sent around it.
        """
        if not session.phase.outbound:
            return None
        if self._rolled_for != session.cleared_takeoff_at \
                or not session.cleared_takeoff_at:
            # Never rolled under this clearance: holding on the numbers
            # for a departure ahead or a gap in the arrivals is the
            # clearance doing what "line up and wait" is for, not an
            # abort. The expedite watch above answers a roll that is
            # merely slow to start.
            return None
        if not getattr(state, "on_ground", True) \
                or float(getattr(state, "ground_speed_kt", 0.0) or 0.0) >= 3.0:
            self._stopped_since = 0.0
            return None
        if not self._stopped_since:
            self._stopped_since = now
            return None
        if now - self._stopped_since < self.ABORT_AFTER_S:
            return None
        field = self.navdb.airport(station.ident)
        runway = session.departure_runway or session.arrival_runway
        if not self._still_on_it(field, runway, state):
            # Already off the strip and stopped somewhere else: its next
            # call, not the tower's instruction.
            self._stopped_since = 0.0
            return None
        if not session.advise_once("abort:vacate"):
            return None
        standing = now - self._stopped_since
        self._stopped_since = 0.0
        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        weather = self._weather(weather_for, field, station)
        session.cleared_takeoff = False
        session.cleared_takeoff_at = 0.0
        session.takeoff_cancelled = False
        session.lined_up = False
        if not session.arrival_runway:
            session.arrival_runway = runway
        decision("abort_vacate", airport=station.ident, runway=runway,
                 standing_s=round(standing))
        return self.brain._issue_taxi_in(session, station, weather, p, state)

    def _still_on_it(self, field, runway: str, state) -> bool:
        """Whether the aeroplane is still on the strip -- the check that a
        stop on the taxiway out is not answered with the runway's taxi in."""
        if field is None or not runway \
                or not getattr(state, "has_position", True):
            return False
        from types import SimpleNamespace

        from .occupancy import on_the_runway

        me = SimpleNamespace(
            latitude=state.latitude, longitude=state.longitude,
            altitude_ft=float(getattr(state, "altitude_ft", 0.0) or 0.0),
            on_ground=True,
            heading=float(getattr(state, "heading_true", 0.0)
                          or getattr(state, "heading_magnetic", 0.0) or 0.0),
            ground_speed_kt=0.0)
        return bool(on_the_runway(self.navdb, field, runway, [me]))

    # How long "expedite" stands: the window the crew's brisk taxi is
    # bought for. The same figure the traffic side gives its aeroplanes
    # (x_instructions_and_being).
    EXPEDITE_WINDOW_S = 90.0

    def _expedite_the_rollout(self, session, station, state, now,
                              language) -> Reply | None:
        """An arrival still on its runway while the next one is on final.

        The traffic side's aeroplanes have been told "expedite" for
        years; the flight being flown never was, and it rolled out at
        taxi speed in front of whatever was on final behind it. Brisk
        enough already -- thirty knots on the strip -- and the word is
        not spent on it.
        """
        if float(getattr(state, "ground_speed_kt", 0.0) or 0.0) >= 30.0:
            return None
        field = self.navdb.airport(station.ident)
        runway = session.arrival_runway or session.departure_runway
        if not self._still_on_it(field, runway, state):
            return None
        coming = self.brain.arrivals_on_final(
            field, runway, self.EXPEDITE_WITHIN_NM)
        if not coming or not session.advise_once("expedite:landing"):
            return None
        session.expedite_until = now + self.EXPEDITE_WINDOW_S
        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        described = (coming[0].indefinite(language_of(p))
                     or coming[0].describe(language_of(p)))
        decision("landing", airport=station.ident, runway=runway,
                 said="expedite", arrival=coming[0].written or None,
                 arrival_nm=round(coming[0].distance_nm, 1))
        return Reply(p.expedite(session.aircraft, p.traffic_on_final(
            described, coming[0].distance_nm)), station)

    def _expedite(self, session, station, state, now, language) -> Reply | None:
        """A departure cleared in front of an arrival that has not moved."""
        if not session.cleared_takeoff_at:
            return None
        if now - session.cleared_takeoff_at < self.EXPEDITE_AFTER_S:
            return None
        if float(getattr(state, "ground_speed_kt", 0.0) or 0.0) >= 5.0:
            return None
        field = self.navdb.airport(station.ident)
        coming = self.brain.arrivals_on_final(field, session.departure_runway,
                                              self.EXPEDITE_WITHIN_NM)
        if not coming or not session.advise_once("expedite:takeoff"):
            return None
        session.expedite_until = now + self.EXPEDITE_WINDOW_S
        p = self.brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        described = (coming[0].indefinite(language_of(p))
                     or coming[0].describe(language_of(p)))
        decision("takeoff", airport=station.ident,
                 runway=session.departure_runway, said="expedite",
                 arrival=coming[0].written or None,
                 arrival_nm=round(coming[0].distance_nm, 1),
                 standing_s=round(now - session.cleared_takeoff_at))
        return Reply(p.expedite(session.aircraft, p.traffic_on_final(
            described, coming[0].distance_nm)), station)

    def began(self) -> None:
        """A new flight, so forget which runway was busy on the last one."""
        self._busy_at = 0.0
        self._stopped_since = 0.0
        self._rolled_for = 0.0

    @staticmethod
    def _weather(weather_for, field, station) -> Weather:
        """The weather at the field, calm and clear where there is none."""
        ident = (field.ident if field is not None else "") or station.ident
        if weather_for is not None:
            try:
                return weather_for(ident)
            except Exception:
                log.debug("no weather for %s", ident, exc_info=True)
        return Weather()


class ClimbWatch:
    """The next step of an IFR climb, given before the last one is reached.

    Departure's steps up to the top of its airspace, then each centre's own
    level, then the cruise (atc/climb.py). Departure gives the next step to
    a flight still climbing within 1,500 ft of the last, so the climb never
    stops; a centre lets the flight come up to its level and gives the next
    after a pause that is its own. Only to a flight that has checked in and
    owes nothing: a step on top of an unread clearance is two instructions
    where the pilot has acknowledged neither.
    """

    def __init__(self, brain: ControllerBrain):
        self.brain = brain

    def look(self, session: FlightSession, station: Station | None, state,
             now: float, language: str = "en") -> Reply | None:
        brain = self.brain
        if not (brain.realism.climb_steps and session.ifr) or station is None:
            return None
        if station.position not in ("DEP", "APP", "CTR"):
            return None
        if state is None or state.on_ground or session.emergency:
            return None
        if not session.established_with(station):
            return None
        pending = session.pending
        if pending is not None and not pending.read_back:
            return None
        cruise = float(session.cruise_altitude_ft or 0)
        if session.phase in (Phase.DESCENT, Phase.APPROACH, Phase.LANDING):
            return None
        here = float(state.altitude_ft or 0.0)
        if cruise and here >= cruise - 500.0:
            return None
        vs = float(state.vertical_speed_fpm or 0.0)
        assigned = float(session.assigned_altitude_ft or 0.0)
        if not assigned or assigned >= cruise:
            return None
        callsign = session.aircraft.callsign.written
        if station.position in ("DEP", "APP"):
            if not climb.departure_step_due(here, vs, assigned, cruise):
                return None
            step = climb.departure_step(station.callsign, callsign, assigned,
                                        here, cruise)
            why = "departure's step"
        else:
            if here < assigned - climb.CENTRE_REACHING_FT:
                session.level_reached_with = ""
                return None
            if session.level_reached_with != station_key(station):
                session.level_reached_with = station_key(station)
                session.level_reached_at = now
                return None
            if now - session.level_reached_at < climb.centre_wait_s(
                    station.callsign):
                return None
            step = climb.centre_step(station.callsign, here, assigned, cruise)
            why = "the centre's step" if step < cruise else "the cruise"
        if step <= assigned:
            return None
        session.level_reached_with = ""
        p = brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        field = self.brain.navdb.airport(station.ident)
        text = p.climb_maintain(
            session.aircraft, step,
            transition_altitude=brain._transition_altitude(field))
        decision("climb", event="step", station=station.callsign,
                 here_ft=round(here), assigned_ft=round(assigned),
                 step_ft=int(step), why=why)
        instruction = session.remember(Instruction(
            "altitude", station_key(station), text, {"altitude_ft": int(step)}))
        return Reply(text, station, instruction=instruction)


class SidWatch:
    """A flight off the SID it was cleared on: "verify you are on the ...
    departure", once.

    More than 2 nm across the SID's path, measured on the route's legs
    between the SID's first fix and its last -- not on the leg from the
    runway to the first fix, which every departure turns onto from runway
    heading and which a plan draws as a straight line it is not. A
    constraint busted is not watched: a SimBrief plan carries the altitude
    the aeroplane is planned to cross each fix at, not the SID's own
    restriction, and calling a deviation from a plan is not a controller's
    business. Nor is "resume own navigation", which follows a vector, and
    departure gives none before the vectors of Phase 5.
    """

    OFF_PATH_NM = 2.0
    # Off for this long, so the turn at a fix is not a deviation.
    OFF_FOR_S = 30.0

    def __init__(self, brain: ControllerBrain):
        self.brain = brain
        self._off_since = 0.0

    def look(self, session: FlightSession, station: Station | None, state,
             now: float, language: str = "en") -> Reply | None:
        brain = self.brain
        route = brain.route
        if not (brain.realism.sid_watch and session.ifr and session.sid
                and route) or station is None:
            return None
        if station.position not in ("DEP", "APP", "TWR", "CTR"):
            return None
        if state is None or state.on_ground or not session.phase.outbound:
            return None
        fixes = route.procedure(route.sid, CLB)
        if len(fixes) < 2:
            return None
        first = route.fixes.index(fixes[0])
        last = route.fixes.index(fixes[-1])
        found = route.progress(state.latitude, state.longitude)
        if found is None or not (first < found.leg <= last):
            self._off_since = 0.0
            return None
        if abs(found.cross_track_nm) <= self.OFF_PATH_NM:
            self._off_since = 0.0
            return None
        if not self._off_since:
            self._off_since = now
            return None
        if now - self._off_since < self.OFF_FOR_S:
            return None
        if not session.advise_once("verify_sid"):
            return None
        p = brain.for_station(station, language)
        session.aircraft.language = language_of(p)
        decision("sid_watch", sid=session.sid, leg=found.leg_name,
                 cross_track_nm=round(found.cross_track_nm, 1),
                 station=station.callsign)
        return Reply(p.verify_on_departure(session.aircraft, session.sid),
                     station)


__all__ = ["DepartureWatch", "ClimbWatch", "SidWatch", "SETTLE_S"]
