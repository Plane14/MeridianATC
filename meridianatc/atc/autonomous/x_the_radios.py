from __future__ import annotations

from ._shared import (
    AutonomousPhase,
    Phase,
    log,
    station_key,
    time,
)

# The positions a handoff may move COM1 to while the aeroplane is on the
# ground: the surface progression, and nothing else. "Contact departure"
# is flown before it is dialled -- a radar handoff that arrives on the
# stand is armed for liftoff, not obeyed from it, and a handoff to another
# field's surface positions is nonsense on the stand either way.
_GROUND_HANDOFF_POSITIONS = frozenset({"DEL", "GND", "TWR", "RMP"})

# The positions that can own COM1 on the ground, per phase, best first.
# The position the phase is worked through leads; the broadcast channels
# are appended by ``_ground_station`` itself, not listed here.
_PHASE_POSITIONS = {
    AutonomousPhase.PREFLIGHT: ("DEL", "GND", "TWR"),
    AutonomousPhase.REQUEST_CLEARANCE: ("DEL", "GND", "TWR"),
    AutonomousPhase.PUSHBACK: ("GND", "RMP", "TWR"),
    AutonomousPhase.TAXI_OUT: ("GND", "TWR"),
    AutonomousPhase.HOLDING_SHORT: ("TWR",),
    AutonomousPhase.LINE_UP: ("TWR",),
    AutonomousPhase.TAKEOFF: ("TWR",),
    AutonomousPhase.TAXI_IN: ("GND", "TWR"),
    AutonomousPhase.PARKED: ("GND", "TWR"),
}

# Where the announcements go at a field with nobody worked: the traffic
# channel, then an information frequency (an open AFIS is somebody to
# talk to -- ``staffed`` says which -- and a closed one is still the
# field's published channel).
_BROADCAST_POSITIONS = ("CTAF", "INFO")

# The calls that volunteer something rather than answer something, held
# while COM1 is mid-retune: a request made on the frequency being left
# is one nobody was asked to hear. A readback is the opposite -- it is
# owed on the frequency it is leaving, before the switch -- and a
# go-around or a "runway vacated" cannot wait for a dial to settle.
_VOLUNTEERED = frozenset({
    "clearance", "vfr", "taxi", "ready", "pushback", "check_in",
    "downwind", "base", "final",
})

# Seconds of quiet on the frequency before the first officer keys in. A
# transmission that leaves on the same breath the other station lets go
# is a machine waiting on the unkey, not a crew that was listening -- the
# same beat the cabin's own answers leave (crew.ANSWER_DELAY_S).
UNKEY_BEAT_S = 1.2

# How long a crew waits before asking "ready for departure" again. The
# tower has the flight on its list the moment the first call lands, and
# the answer -- "standby", "hold position", the clearance itself -- is
# what comes back, not a second ask. A minute and a half is the interval
# a patient crew leaves before reminding the tower it is still there.
READY_REASK_S = 90.0

# The calls keyed on top of that beat rather than behind it: a go-around
# is what the tower is already listening for, and a call that urgent is
# late exactly the length of the pause a slower one would leave.
_CANNOT_WAIT = frozenset({"going_around"})


class _The_radios:
    # -- the radios ---------------------------------------------------------

    def _comms_ready(self) -> bool:
        """Whether COM1 is settled on the frequency it should be on --
        the FO holds an ask across a retune the way a pilot does, and
        only volunteers calls once the dial shows the right number.
        With the radios unmanaged there is nothing the hold defers to.
        """
        return not getattr(self, "_manage_radio", False) \
            or getattr(self, "_comms_settled", True)

    def _fo_say(self, kind: str) -> bool:
        """One transmission by the first officer, through the same speech
        and the same parsing as the pilot's. False where the call was not
        made -- nothing to say, or nobody to say it on, a volunteered
        call the frequency change is not ready for, or a frequency that
        has not been quiet for the beat a crew leaves before keying."""
        if kind in _VOLUNTEERED and not self._comms_ready():
            return False
        if kind not in _CANNOT_WAIT and not self._unkeyed():
            return False
        if self.crew.radio_call is None or self.crew.transmit is None:
            return False
        try:
            text, why = self.crew.radio_call(kind)
        except Exception:
            log.debug("the first officer's %s call failed", kind,
                      exc_info=True)
            return False
        if not text:
            log.debug("first officer %s call not made: %s", kind, why)
            return False
        self.crew.transmit(text)
        self.state.last_radio_at = time.monotonic()
        return True

    def _radio_quiet(self) -> bool:
        try:
            busy = bool(self.crew.radio_busy and self.crew.radio_busy())
        except Exception:
            log.debug("could not tell whether the frequency is busy",
                      exc_info=True)
            return True
        if busy:
            # Remembered on the way through, so the unkey beat measures
            # the quiet from when the frequency actually let go.
            self._last_busy_at = time.monotonic()
        return not busy

    def _unkeyed(self) -> bool:
        """Whether the frequency has been quiet for the beat a crew
        leaves before keying. Nobody transmits on the same breath the
        last voice lets go on; a call that does is a machine that was
        waiting on the edge of it, and it reads exactly that."""
        return (time.monotonic() - getattr(self, "_last_busy_at", 0.0)
                >= UNKEY_BEAT_S)

    def _manage_comms(self, state, now: float) -> None:
        """Work the radios the way the first officer flying would.

        On the ground COM1 belongs to the surface progression -- the
        delivery, ramp, ground and tower positions the phase is worked
        through, each staffed position ahead of a printed one nobody is
        on, and the field's broadcast channel where nobody is worked at
        all. A controller's handoff outranks that default while it is
        still to the surface; a handoff to a radar position is armed on
        the ground and obeyed once the wheels are up, which is how it
        is flown for real. In the air the frequency is the handoff's to
        move and nobody else's.

        The squawk and the altimeter are dialled to the figures the
        controller gave. A readback owed goes out before anything
        volunteered, nothing is keyed while the frequency is busy or
        while a retune is still settling, and nothing at all while the
        radio has no power.
        """
        if not self._manage_radio:
            return
        sim = self.sim
        session = self.session
        if sim is None or not getattr(sim, "connected", False) \
                or session is None:
            return
        if self.state.phase in (AutonomousPhase.INIT,
                                AutonomousPhase.COMPLETE,
                                AutonomousPhase.ERROR):
            return
        from ..radio_power import decide

        powered, _why = decide(state)
        if not powered or not self._radio_quiet():
            return

        # The dials are worked whatever else the frequency is doing: they
        # are knob-turns, not transmissions.
        self._dial_radios(state, now)

        # A readback owed goes before anything the FO would volunteer --
        # a controller who has spoken is answered first.
        pending = getattr(session, "pending", None)
        if pending is not None and pending.readback_required \
                and not pending.read_back:
            if now - self.state.last_radio_at >= 4.0:
                self._fo_say("readback")
            return

        # Who owns COM1, and whether it is already showing the number --
        # a retune still settling is a frequency not to talk on, and the
        # flag is what the phase's own asks (the clearance, the pushback)
        # defer to as well.
        handed = getattr(session, "handoff_pending", None)
        self._comms_settled = self._work_the_frequency(
            state, session, handed, now)
        if not self._comms_settled:
            return

        # On a broadcast frequency the circuit legs are the calls the
        # traffic is sequenced by -- run before the asks.
        self._work_the_circuit(state, session, now)
        self._work_the_calls(state, session, now)
        self._check_in_once(state, session, now)

    # -- who owns COM1 -------------------------------------------------------

    def _work_the_frequency(self, state, session, handed, now: float) -> bool:
        """Settle COM1 on the station that should have it.

        True when the dial shows the frequency it should -- the asks and
        the check-in are safe to make. False while a retune is owed: the
        FO does not talk across a frequency change it is still making,
        and a ``set_com`` the sim has not landed yet is one of those.

        On the ground a handoff owns the dial only while it points at a
        surface position at this field -- "contact ground" is obeyed
        from the stand; "contact departure" is armed for the air. The
        phase's own position is the answer otherwise, which is what
        keeps a stray radar handoff from leaving COM1 on approach while
        the flight is asking delivery for its clearance.
        """
        if state.on_ground:
            if self._ground_handoff(state, session, handed, now):
                return self._on_channel(state, handed.mhz)
            return self._tune_for_phase(state, session, now)
        if handed is not None and getattr(handed, "mhz", 0.0):
            self._dial_com1(state, float(handed.mhz), now, 6.0,
                            getattr(handed, "callsign", ""))
            return self._on_channel(state, handed.mhz)
        # In the air with nobody sending the flight anywhere, COM1 is
        # where the last handoff left it -- there is nothing to settle.
        return True

    def _ground_handoff(self, state, session, handed, now: float) -> bool:
        """Whether the pending handoff owns COM1 on the ground.

        Only a surface position at the field the aeroplane is standing
        at. A handoff to a radar position -- however it got written onto
        the session, an early "contact departure" or a nudge meant for
        an aeroplane that was flying -- is for the air and is left armed
        for liftoff; one to another field's surface is never dialled
        either. Either way the dial is attempted here, so the handoff
        claim is what the phase tuning defers to.
        """
        if handed is None or not getattr(handed, "mhz", 0.0):
            return False
        if getattr(handed, "position", "") not in _GROUND_HANDOFF_POSITIONS:
            return False
        airport = self._field_under(session)
        if airport is not None \
                and getattr(handed, "ident", "") != airport.ident:
            return False
        self._dial_com1(state, float(handed.mhz), now, 6.0,
                        getattr(handed, "callsign", ""))
        return True

    def _tune_for_phase(self, state, session, now: float) -> bool:
        """COM1 on the position the ground phase is worked through.

        True when it already is (or when there is simply nowhere better
        to put it -- calls are still owed on whatever is in the radio).
        """
        positions = _PHASE_POSITIONS.get(self.state.phase)
        if not positions:
            return True
        airport = self._field_under(session)
        if airport is None:
            return True
        station = self._ground_station(airport, positions)
        if station is None:
            return True
        self._dial_com1(state, float(station.mhz), now, 8.0,
                        getattr(station, "callsign", ""))
        return self._on_channel(state, station.mhz)

    def _ground_station(self, airport, positions):
        """The best of the phase's candidates that exists at the field.

        A staffed position is dialled ahead of a senior one that is
        printed but silent -- a delivery nobody is on leaves ground to
        answer, and a tower shut for the night leaves the field on its
        broadcast channel. With nobody worked at all the broadcast
        channel is where the announcements go; and with not even that,
        the first printed candidate is still a better home for COM1
        than whatever the last leg left in the radio.
        """
        staffed = getattr(getattr(self, "brain", None), "staffed", None)
        try:
            rows = [(position, station)
                    for position in tuple(positions) + _BROADCAST_POSITIONS
                    for station in
                    (self.navdb.stations(airport.ident, position) or ())
                    if getattr(station, "mhz", 0.0)]
        except Exception:
            return None
        if staffed is not None:
            for position in positions:
                for row_position, station in rows:
                    if row_position != position:
                        continue
                    try:
                        if staffed(station):
                            return station
                    except Exception:
                        continue
        for position in _BROADCAST_POSITIONS:
            for row_position, station in rows:
                if row_position == position:
                    return station
        for position in positions:
            for row_position, station in rows:
                if row_position == position:
                    return station
        return None

    def _field_under(self, session):
        """The airport the aeroplane is standing at: where it is first,
        then the filed ends -- the departure on the way out and the
        destination on the way in, so taxi-in tunes this field's ground,
        not the one it left hours ago.
        """
        field = self._get_current_airport()
        if field is not None:
            return field
        if self.state.phase in (AutonomousPhase.TAXI_IN,
                                AutonomousPhase.PARKED):
            return getattr(session, "destination", None) \
                or getattr(session, "departure", None)
        return getattr(session, "departure", None) \
            or getattr(session, "destination", None)

    def _dial_com1(self, state, mhz: float, now: float, gap_s: float,
                   callsign: str = "") -> bool:
        """Put COM1 on a frequency, throttled so a dial that does not
        take is retried on a clock rather than every tick. True when the
        set went out or was already there."""
        active = float(getattr(state, "com1_active", 0.0) or 0.0)
        if abs(active - mhz) <= 0.004:
            return True
        if now - self._dialled_at < gap_s:
            return False
        if not self.sim.set_com(mhz, 1):
            return False
        self._dialled_at = now
        name = f"{callsign} " if callsign else ""
        self._emit("info", f"COM1 to {name}{mhz:.3f}",
                   "autonomous.comms.tune")
        return True

    @staticmethod
    def _on_channel(state, mhz) -> bool:
        """Whether COM1's active already shows the frequency wanted."""
        active = float(getattr(state, "com1_active", 0.0) or 0.0)
        return abs(active - float(mhz)) <= 0.004

    # -- what gets said on it ------------------------------------------------

    def _work_the_circuit(self, state, session, now: float) -> None:
        """Position reports on a broadcast frequency, in the circuit.

        Where nobody clears anybody -- a CTAF, a unicom, an AFIS that
        only advises -- the calls are the traffic: each leg reported
        once as it is flown. The final call doubles as the landing
        clearance nobody gives, so after it has had its moment on the
        frequency the field is the pilot's to land on. A staffed
        controller sequences instead and hears none of this.
        """
        if not self._manage_radio or getattr(state, "on_ground", False):
            return
        if self.state.phase not in (AutonomousPhase.DESCENT,
                                    AutonomousPhase.APPROACH,
                                    AutonomousPhase.LANDING):
            return
        # The pattern the traffic engine itself uses: a flight at a
        # field where nobody clears announces itself around the circuit,
        # IFR or not. An uncontrolled frequency has nobody to clear an
        # IFR arrival either -- a pilot announces intentions and lands,
        # he does not go around for a clearance nobody gives.
        station = getattr(session, "current_station", None)
        if station is not None and self.brain.issues_clearances(station):
            return
        try:
            circuit = getattr(self.crew, "circuit", None)
            where = circuit() if circuit else None
        except Exception:
            where = None
        if where is None or not getattr(where, "in_circuit", False):
            return
        leg = getattr(where, "leg", "") or ""
        if leg not in ("downwind", "base", "final") \
                or leg == self.state.circuit_said:
            return
        if now - self.state.last_radio_at < 6.0 \
                or not self._radio_quiet() or not self._unkeyed():
            return
        if not self._fo_say(leg):
            return
        self.state.circuit_said = leg
        if leg == "final" and not session.cleared_landing:
            # The broadcast nobody answers: the call is the clearance.
            session.cleared_landing = True
            self._emit("info", "No landing clearance on this frequency -- "
                       "landing after the final call",
                       "autonomous.circuit.selfclear")

    def _work_the_calls(self, state, session, now: float) -> None:
        """Ask for the thing the phase is waiting on.

        A controller only answers a call that was made: taxi is asked of
        ground once the aeroplane is off the stand, and ready-for-
        departure is asked of the tower at the hold. On a broadcast
        frequency the call is itself the announcement and nobody is
        expected to answer, but the asking is still how the flight's
        intentions get onto the frequency. A call is repeated while it
        goes unanswered -- the frequency can be too busy to have heard
        the first one -- and ``last_radio_at`` is what keeps that a
        request and not a stream.
        """
        phase = self.state.phase
        if not getattr(state, "on_ground", False) \
                and getattr(session, "ifr", False) \
                and not getattr(session, "clearance_issued", False):
            # An IFR leg airborne that never collected a clearance is
            # owed the pop-up pickup -- the leg that filed IFR into
            # cloud on the way, or the one engaged in the air before
            # its clearance ever was. Asked for until somebody issues
            # it; on a frequency that clears nobody the call is the
            # pilot's announcement, made once and not repeated.
            station = getattr(session, "current_station", None)
            if getattr(self, "_pickup_announced", False) and (
                    station is None
                    or not self.brain.issues_clearances(station)):
                return
            kind = "clearance"
        elif phase == AutonomousPhase.TAXI_OUT:
            reask = getattr(self, "_taxi_reask", False)
            if not reask and not getattr(self, "_held_at_split", False) and (
                    session.phase in (Phase.HOLDING_SHORT,
                                      Phase.LINED_UP, Phase.TAKEOFF)
                    or getattr(session, "taxi_path", None)
                    or getattr(session, "taxi_route", None)):
                # Under a clearance already -- except held at a cut
                # taxi's split, where the call is the "ready to
                # continue" that brings the rest of the route, and
                # except the re-ask a stop off the route wants.
                # TAXI_OUT alone does not count: the phase flips on
                # groundspeed without an instruction ever having
                # landed, and trusting it is how the aeroplane went
                # off down the apron on no route at all.
                return
            if not reask and getattr(self, "_taxi_announced", False) \
                    and self._taxi_issuing_position() is None:
                # Where nobody issues a route the call is the
                # announcement itself: made once, not repeated to an
                # empty room.
                return
            kind = "taxi"
        elif phase == AutonomousPhase.TAXI_IN:
            # Inbound the stand is the ground controller's to name: the
            # aeroplane asks where to park until the instruction or its
            # route arrives, the same ask the outbound one makes.
            if not getattr(self, "_taxi_reask", False) and (
                    session.phase not in (Phase.LANDED,)
                    or getattr(session, "taxi_path", None)):
                return
            kind = "taxi"
        elif phase == AutonomousPhase.HOLDING_SHORT:
            if session.lined_up or session.cleared_takeoff \
                    or session.phase in (Phase.LINED_UP, Phase.TAKEOFF):
                return
            if getattr(session, "held_for_landing", ""):
                # Held at the bars for the arrival on final: told to
                # wait, and a pilot told to wait does not ask again --
                # the release comes to the flight, not the other way
                # round.
                return
            if (state.ground_speed_kt or 0.0) > 1.0:
                # Ready-for-departure is called from a stopped aeroplane.
                return
            kind = "ready"
        elif phase in (AutonomousPhase.LINE_UP, AutonomousPhase.TAKEOFF):
            if session.cleared_takeoff or self.state.rejected \
                    or self.state.sub not in ("set", "wait_clearance"):
                return
            if getattr(session, "held_for_landing", ""):
                # Lined up behind the landing traffic -- the wait is
                # already instructed, so the call is not repeated; the
                # tower's release watch clears the flight when the
                # runway frees.
                return
            if (state.ground_speed_kt or 0.0) > 1.0:
                return
            # Lined up and still not released: the call that says so.
            kind = "ready"
        else:
            return
        tried = getattr(self, "_call_tried_at", 0.0)
        # A ready call answered with "standby"/"hold position" is asked
        # again a minute and a half on, not on the next quiet patch: the
        # tower knows the aeroplane is there, and the crew that waits a
        # stretch is the one that does not get a second telling-off.
        interval = READY_REASK_S if kind == "ready" else 25.0
        if now - max(self.state.last_radio_at, tried) < interval \
                or not self._unkeyed():
            return
        self._call_tried_at = now
        if not self._fo_say(kind):
            return
        if kind == "taxi":
            # The taxi call went out: on an unanswered frequency it is
            # the announcement, not something to keep asking for.
            self._taxi_announced = True
            self._taxi_reask = False
        elif kind == "clearance":
            self._pickup_announced = True

    def _check_in_once(self, state, session, now: float) -> None:
        """The first call on a new airborne station: the level, and
        whether it is being climbed or descended to.

        Never on the ground -- there the calls that ask for things are
        the establishment, and an altitude check-in to the tower at the
        holding point is not a call a tower is waiting for. A station
        the flight is already established with needs nothing either.
        """
        if state.on_ground:
            return
        current = getattr(session, "current_station", None)
        if current is None \
                or getattr(current, "position", "") in ("DEL", "GND"):
            return
        if session.established_with(current):
            return
        key = station_key(current)
        fresh = key != self.state.checked_in_with
        if (fresh or now - self.state.last_radio_at >= 60.0) \
                and now - self.state.last_radio_at >= 4.0 \
                and self._fo_say("check_in"):
            self.state.checked_in_with = key

    def _dial_radios(self, state, now: float) -> None:
        """Squawk and altimeter as the controller gave them, dialled once --
        and only once the readback that carries them has gone out. A figure
        the controller has not heard back is a figure the flight has not
        agreed to: dialling it anyway is acting on a clearance nobody
        acknowledged, which is the order a crew does not work in.
        """
        session = self.session
        pending = getattr(session, "pending", None)
        owed = (getattr(pending, "values", None) or {}
                if pending is not None
                and getattr(pending, "readback_required", True)
                and not getattr(pending, "read_back", False)
                else {})
        want = "".join(c for c in str(getattr(session, "squawk", "") or "")
                       if c.isdigit())[:4]
        if "squawk" not in owed and want and want != self.state.squawk_dialled:
            have = "".join(
                c for c in str(getattr(state, "transponder_code", ""))
                if c.isdigit())[:4]
            if have == want:
                self.state.squawk_dialled = want
            elif now - self._dialled_at >= 5.0 \
                    and self.sim.set_squawk(want):
                self._dialled_at = now
        alt = self._altimeter_hpa(getattr(session, "altimeter", 0.0))
        if "altimeter" not in owed and alt > 0.0 \
                and abs(alt - self.state.altimeter_dialled) > 0.2:
            have = float(getattr(state, "kohlsman_hpa", 0.0) or 0.0)
            if have > 0.0 and abs(have - alt) <= 0.5:
                self.state.altimeter_dialled = alt
            elif now - self._dialled_at >= 5.0:
                self.director.kohlsman(alt)
                self._dialled_at = now
