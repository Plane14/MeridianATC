from __future__ import annotations

from ._shared import (
    _EXIT_WORDS,
    ATTEMPTS_BEFORE_REISSUE,
    PROMPTS_BEFORE_REISSUE,
    FlightSession,
    Intent,
    ParsedIntent,
    Phase,
    Phraseology,
    Reply,
    Station,
    decision,
    nearly_matches,
    re,
    station_key,
)


class _Readback_checking:
    # ------------------------------------------------------------------
    # readback checking
    # ------------------------------------------------------------------

    # What gets through an outstanding readback. An emergency does not wait to
    # be collationed, a go-around is flown before it is discussed, and a pilot
    # who says "say again" or "unable" is answering the instruction, not
    # ignoring it. Everything else waits.
    _READBACK_EXEMPT = frozenset({
        Intent.EMERGENCY, Intent.URGENCY, Intent.MINIMUM_FUEL,
        Intent.EMERGENCY_DETAILS,
        Intent.GOING_AROUND, Intent.MISSED_APPROACH,
        Intent.SAY_AGAIN, Intent.UNABLE, Intent.NEGATIVE, Intent.STANDBY,
        Intent.RADIO_CHECK,
    })

    def _settle_pending(
        self, parsed: ParsedIntent, session: FlightSession,
        station: Station, p: Phraseology, state=None,
    ) -> Reply | None:
        """Deal with an instruction that has not been read back yet.

        Returns the controller's reply when the readback has to be insisted on
        (or when the pilot in fact gave one), and ``None`` when the flight may
        get on with whatever it asked for.
        """
        pending = session.pending
        if pending is None or not pending.readback_required:
            return None

        # They are talking to somebody else now. The controller who issued it
        # cannot chase a readback on a frequency the aircraft has left, so the
        # instruction stops being outstanding rather than blocking the new
        # position for the rest of the flight.
        if station_key(station) != pending.station_key:
            session.clear_pending()
            return None

        # "Affirm" to a "confirm the ... departure" settles what was asked.
        if pending.confirming and parsed.intent is Intent.AFFIRMATIVE:
            for key in pending.confirming:
                parsed.readback_items.setdefault(key, pending.values[key])
            return self._check_readback(parsed, session, station, p, state)

        heard = parsed.readback_items
        owed = pending.owed_items

        # A readback by any other name. "Runway two seven, hold short of
        # runway three three" reads as a taxi request to a rule table and as a
        # readback to a controller, and the controller is right.
        if owed and heard and not pending.mismatches(heard):
            return self._check_readback(parsed, session, station, p, state)

        # An instruction with nothing to check back -- a taxi to the ramp, a
        # clearance direct to a fix -- is satisfied by being acknowledged.
        # That is what "roger" and "wilco" are for, and demanding a readback of
        # something with no numbers in it is pedantry rather than safety.
        if not owed and parsed.intent in (Intent.ACKNOWLEDGE,
                                          Intent.AFFIRMATIVE):
            pending.read_back = True
            session.apply(pending.values)
            session.clear_pending()
            return None

        if parsed.intent in self._READBACK_EXEMPT:
            return None

        # "Cleared to land, Delta twelve thirty four": the clearance restated
        # with nothing contradicting it. A runway that is said has to be the
        # right one (that is a readback, checked above); one that is not said
        # is accepted, as most pilots read a landing clearance back.
        if (pending.kind == "landing_clearance"
                and parsed.intent is Intent.ACKNOWLEDGE
                and "cleared to land" in parsed.normalized
                and "runway" not in heard):
            pending.read_back = True
            session.apply(pending.values)
            session.clear_pending()
            return None

        # Asking again for the thing that produced the instruction is not
        # ignoring it, it is not having heard it. That gets the answer -- which
        # may well be "you are already cleared for takeoff" or "hold position"
        # -- rather than a demand for a readback of something unheard. The
        # readback stays owed, and the situation loop goes on chasing it.
        if pending.from_intent and parsed.intent.value == pending.from_intent:
            return None

        return self._insist(session, station, p)

    def _insist(self, session: FlightSession, station: Station,
                p: Phraseology) -> Reply:
        """Ask again for a readback, or re-read the instruction.

        Twice the controller names what it is waiting for. After that it reads
        the whole instruction again, because a pilot who has not read back
        twice has usually not heard it, and asking a third time for a readback
        of something they never received helps nobody.
        """
        pending = session.pending
        assert pending is not None
        pending.prompted()
        if pending.prompts > PROMPTS_BEFORE_REISSUE:
            pending.prompts = 0
            return Reply(
                self._repeat(pending, station, p, session.aircraft),
                station, instruction=pending,
            )
        return Reply(
            p.read_back_instruction(session.aircraft, pending.owed_items),
            station, instruction=pending,
        )

    def chase_readback(self, session: FlightSession, station: Station | None,
                       p: Phraseology) -> Reply | None:
        """What the controller says when the pilot simply goes quiet.

        Called by the situation loop rather than by a transmission. A readback
        that is never given is the case the old code could not see at all: the
        instruction sat pending forever and nothing was ever said about it.
        """
        pending = session.pending
        if pending is None or not pending.readback_required:
            return None
        if station is None or station_key(station) != pending.station_key:
            return None
        if not pending.overdue:
            return None
        if pending.kind == "handoff" and pending.prompts >= 1:
            # Asked for once. What follows, if the flight has not switched,
            # is the handoff said again (Engine._reoffer_handoff), not the
            # same request on repeat.
            return None
        return self._insist(session, station, p)

    def _check_readback(
        self, parsed: ParsedIntent, session: FlightSession,
        station: Station, p: Phraseology, state=None,
    ) -> Reply:
        pending = session.pending
        assert pending is not None
        wrong = pending.mismatches(parsed.readback_items)
        wrong += [key for key in self._vfr_items_wrong(parsed, session, pending, station, p)
                  if key not in wrong]

        # Only what the recogniser nearly caught, and nothing else wrong --
        # a SID's name heard close, a figure one off: asked about, the way a
        # controller who did not quite hear it asks, rather than "negative"
        # to a pilot who may well have said it right.
        nearly = set(getattr(parsed, "nearly", set()))
        if self._unsure(parsed):
            heard = parsed.readback_items or {}
            nearly |= {key for key in wrong
                       if nearly_matches(key, pending.values.get(key),
                                         heard.get(key))}
        if wrong and set(wrong) <= nearly:
            pending.outstanding = set(wrong)
            pending.confirming = set(wrong)
            decision("readback", station=station.callsign, instruction=pending.kind,
                     said="confirm", items=sorted(wrong),
                     heard_confidence=round(parsed.heard_confidence, 2),
                     confirm_below=self._confirm_below(), heard=parsed.text)
            return Reply(
                p.correction(
                    session.aircraft, wrong, pending.values,
                    current_altitude_ft=getattr(state, "altitude_ft", 0.0) or 0.0,
                    transition_altitude=self._transition_altitude(
                        self.navdb.airport(station.ident)),
                    confirm=True),
                station, instruction=pending)

        if not wrong:
            pending.read_back = True
            session.apply(pending.values)
            session.clear_pending()

            # Delivery hands off to Ground once the clearance is read back;
            # every other position simply accepts it.
            if pending.kind == "ifr_clearance":
                session.clearance_read_back = True
                session.phase = Phase.CLEARED
                ground = self.controller_at(station.ident, "GND")
                if ground:
                    return Reply(
                        p.readback_correct(
                            session.aircraft,
                            next_station=self._station_name(ground, p, short=True),
                            next_freq=ground.mhz,
                        ),
                        station,
                        handoff_to=ground,
                    )
                return Reply(p.readback_correct(session.aircraft), station)

            if pending.kind == "handoff" and pending.values.get("frequency"):
                return Reply("", station, silent=True)  # they are leaving; say nothing more

            if pending.kind in ("takeoff_clearance", "landing_clearance"):
                return Reply("", station, silent=True)

            return Reply(p.readback_correct(session.aircraft), station)

        # Something was wrong or missing: correct just that item, and from
        # here on judge the answer on that item alone. The pilot is replying
        # to the question they were asked, not reciting the clearance again.
        pending.outstanding = set(wrong)
        pending.confirming = None
        pending.attempts += 1
        if pending.attempts >= ATTEMPTS_BEFORE_REISSUE:
            # Correcting the same item a fourth time is not working, so the
            # whole instruction is read again. It is still not accepted: an
            # instruction that has not been read back correctly is not one the
            # aircraft is operating under, and pretending otherwise is the
            # thing a readback exists to prevent.
            pending.attempts = 0
            return Reply(
                self._repeat(pending, station, p, session.aircraft),
                station, instruction=pending,
            )

        return Reply(
            p.correction(
                session.aircraft, wrong, pending.values,
                current_altitude_ft=getattr(state, "altitude_ft", 0.0) or 0.0,
                transition_altitude=self._transition_altitude(
                    self.navdb.airport(station.ident)),
            ),
            station, instruction=pending,
        )

    def _vfr_items_wrong(self, parsed, session, pending, station,
                         p) -> list[str]:
        """The VFR items of a release or a transit read back as something
        else: the ceiling ("not above one thousand five hundred") and the
        point to leave by.

        Only a figure or a point that *was* said and is wrong; one the
        recogniser did not catch is not chased, because neither is on the
        list of items a pilot must read back word for word (FAA AIM 4-4-7,
        ICAO Doc 4444 4.5.7.5) and insisting on them would be insisting on
        the recogniser. Logged, either way, so the arrival log shows what
        was heard against what was given.
        """
        if not self.realism.readback_items:
            return []
        values = pending.values
        if not ("vfr_ceiling_ft" in values or "exit_point" in values):
            return []
        heard = parsed.readback_items or {}
        # Answering a correction is answering that item alone.
        asked = pending.outstanding
        wrong = []
        altitude = heard.get("altitude_ft")
        # A code read as a height ("squawk one two zero zero" is also twelve
        # hundred feet) is not a ceiling.
        if altitude and str(int(float(altitude))) == str(values.get("squawk", "")).lstrip("0"):
            altitude = None
        if ("vfr_ceiling_ft" in values and altitude
                and (asked is None or "vfr_ceiling_ft" in asked)
                and int(float(altitude))
                != int(float(values["vfr_ceiling_ft"]))):
            wrong.append("vfr_ceiling_ft")
        said_point = None
        if "exit_point" in values and (asked is None or "exit_point" in asked):
            if self._mentions(parsed, values["exit_point"], p):
                said_point = values["exit_point"]
            else:
                # Only a point said as the way out ("via Sierra Alpha"): a
                # point's name on its own is as likely to be the callsign --
                # "November one seven two Sierra Papa" is not November.
                text = f" {parsed.normalized} "
                said_point = next(
                    (v.name for v in self.navdb.vrps(station.ident)
                     if v.spoken and v.name != values["exit_point"]
                     and re.search(rf" {_EXIT_WORDS} ({re.escape(p.vrp(v.name).lower())}"
                                   rf"|{re.escape(v.name.lower())}) ", text)),
                    None)
                if said_point is not None:
                    wrong.append("exit_point")
        decision("readback_vfr", station=station.callsign, instruction=pending.kind,
                 ceiling_given=values.get("vfr_ceiling_ft"),
                 ceiling_heard=heard.get("altitude_ft"),
                 point_given=values.get("exit_point"), point_heard=said_point,
                 wrong=wrong or None)
        return wrong

