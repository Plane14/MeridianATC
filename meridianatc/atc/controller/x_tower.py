from __future__ import annotations

from ._shared import (
    Instruction,
    Intent,
    Phase,
    Reply,
    _airborne,
    station_key,
)


class _Tower:
    # ------------------------------------------------------------------
    # tower
    # ------------------------------------------------------------------

    def _tower(self, parsed, session, station, weather, p, state) -> Reply:
        aircraft = session.aircraft
        airport = self.navdb.airport(station.ident)

        # Everything a tower says depends on whether the aircraft is on the
        # ground, so that is settled before the intent is looked at. Without
        # it a departure that called back after rotating was answered with a
        # takeoff clearance, which is the one instruction that cannot possibly
        # apply to an aircraft already in the air.
        airborne = _airborne(state)

        # An IFR arrival checking in on the final (the IFR plan, Phase 5
        # (e)): "established on the localizer" was not understood here and
        # was answered "expecting inbound for landing, or field in sight".
        if (self.realism.tower_continue and session.ifr and airborne
                and not session.phase.outbound
                and parsed.intent in (Intent.REPORT_ESTABLISHED,
                                      Intent.CHECK_IN)):
            return self._tower_check_in_on_final(parsed, session, station,
                                                 weather, p, state)

        if parsed.intent is Intent.REQUEST_TAKEOFF:
            if airborne:
                return Reply(p.already_airborne(aircraft), station)
            return self._issue_takeoff(session, station, weather, p, parsed)

        if parsed.intent is Intent.REQUEST_PATTERN_WORK:
            session.pattern_work = True
            if not airborne:
                return self._issue_takeoff(session, station, weather, p, parsed)
            return self._work_the_circuit(session, station, weather, p, parsed,
                                          state)

        if parsed.intent is Intent.REQUEST_TRANSITION:
            return self._issue_transition(session, station, weather, p, parsed,
                                          state)

        # A field whose tower is its only position releases its own VFR
        # departures; the request fell through to the list of calls the
        # tower was "expecting".
        if parsed.intent is Intent.REQUEST_VFR_DEPARTURE and not airborne:
            return self._issue_vfr_release(session, station, p, parsed, weather)

        if parsed.intent is Intent.CHECK_IN and session.phase.outbound:
            if not airborne:
                return self._issue_takeoff(session, station, weather, p, parsed)
            # Already off the ground and still on the tower frequency: what
            # they need is the next controller, not a clearance to do the
            # thing they have already done.
            return self._hand_off_departure(session, station, p)

        if parsed.intent in (Intent.REQUEST_LANDING, Intent.POSITION_REPORT,
                             Intent.REPORT_FIELD_IN_SIGHT):
            if (not airborne and session.phase.outbound
                    and session.phase is not Phase.PREFLIGHT):
                return Reply(p.roger(aircraft), station)
            # A VFR arrival is sequenced round the circuit, not vectored onto
            # final. "Continue approach" to an aeroplane at eight miles VFR is
            # IFR phraseology used on a flight that is not flying one.
            if session.visual:
                return self._work_the_circuit(session, station, weather, p,
                                              parsed, state)
            # Airborne, so the phase tracker's opinion is beside the point: an
            # aircraft in the circuit is outbound and inbound at once, and it
            # is asking to land.
            return self._issue_landing(session, station, weather, p, parsed, state)

        if parsed.intent is Intent.CHECK_IN:
            if session.visual and airborne:
                return self._work_the_circuit(session, station, weather, p,
                                              parsed, state)
            return self._issue_landing(session, station, weather, p, parsed, state)

        if parsed.intent in (Intent.GOING_AROUND, Intent.MISSED_APPROACH):
            if not airborne:
                return Reply(p.already_on_the_ground(aircraft), station)
            return self.go_around(session, station, weather, p, airport)

        # A clearance asked of the tower: whoever delivers them here, or the
        # tower itself where nobody else does. It had no case at all, and the
        # pilot was told the tower was "expecting ready for departure".
        if parsed.intent is Intent.REQUEST_CLEARANCE:
            if airborne:
                return Reply(p.already_airborne(aircraft), station)
            sent = self._send_on(session, station, p, "DEL", "GND")
            if sent is not None:
                return sent
            return self._delivery(parsed, session, station, weather, p, state)

        if parsed.intent in (Intent.REQUEST_TAXI,
                             Intent.REQUEST_TAXI_TO_PARKING):
            if airborne:
                # An aeroplane in the air does not taxi anywhere, and telling
                # it to call Ground is telling it to call somebody who cannot
                # do anything for it until it has landed.
                return Reply(p.already_airborne(aircraft), station)
            ground = self.controller_at(station.ident, "GND")
            if ground:
                return Reply(
                    p.handoff(aircraft, self._station_name(ground, p, short=True), ground.mhz),
                    station, handoff_to=ground,
                )
            # A field whose tower also works the ground taxis them itself,
            # rather than asking them to say again what they clearly said --
            # to parking when they are in, to the runway when they are out.
            if parsed.intent is Intent.REQUEST_TAXI_TO_PARKING \
                    or session.phase in (Phase.LANDED, Phase.TAXI_IN):
                return self._issue_taxi_in(session, station, p, state)
            return self._issue_taxi(session, station, weather, p, state)

        if parsed.intent is Intent.RADIO_CHECK:
            return Reply(p.radio_check(aircraft), station)

        if parsed.intent in (Intent.ACKNOWLEDGE, Intent.AFFIRMATIVE):
            return Reply("", station, silent=True)

        return self._not_understood(session, station, p, state)

    def _hand_off_departure(self, session, station, p) -> Reply:
        """Send an airborne aircraft on to whoever works departures."""
        nxt = (self.controller_at(station.ident, "DEP")
               or self.controller_at(station.ident, "APP"))
        if nxt is None:
            return Reply(p.roger(session.aircraft), station)
        text = p.handoff(session.aircraft,
                         self._station_name(nxt, p, short=True), nxt.mhz)
        instruction = session.remember(
            Instruction("handoff", station_key(station), text,
                        {"frequency": nxt.mhz})
        )
        return Reply(text, station, instruction=instruction, handoff_to=nxt)

