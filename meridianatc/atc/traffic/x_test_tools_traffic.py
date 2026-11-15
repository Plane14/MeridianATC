from __future__ import annotations

from ._shared import (
    _FLYING_PHASES,
    CIRCUIT_SHARE,
    GLIDE_FT_PER_NM,
    REST_OF_TAXI_M,
    Aircraft,
    Flight,
    _local_or_english,
    _spell_route,
    decision,
    every_language,
    fleets,
    livemod,
    log,
    math,
    perftable,
    wake_suffix,
)


class _Test_tools_traffic:
    # -- test tools (traffic.debug.triggers) ------------------------------

    # How long a departure boarded by hand stands booked before its boarding
    # starts: long enough for its model to be swapped with the doors shut.
    BOARD_NOW_LEAD_S = 10.0
    # Where an arrival asked for by hand is put: on final, this far out.
    SHORT_FINAL_NM = 4.0

    def _nearest_to_user(self, options, xy_of):
        user = self.user
        if user is None or not user.has_position or self.ground is None:
            return None
        ux, uz = self.ground.to_xy(user.latitude, user.longitude)
        return min(options, key=lambda one: math.hypot(xy_of(one)[0] - ux,
                                                       xy_of(one)[1] - uz))

    def board_nearest(self, now: float) -> tuple[Flight | None, str]:
        """Board the parked aeroplane nearest the pilot, now.

        With the schedule it is booked to start boarding in
        :attr:`BOARD_NOW_LEAD_S` -- that much pre-boarding so its model can
        change with every door shut -- and departs once it has boarded, as
        any booked departure does. Without it, it is a departure at once.
        Returns the flight and "", or None and a reason key.
        """
        if self.ground is None:
            return None, "no_stands"
        user = self.user
        if user is None or not user.has_position:
            return None, "no_position"
        ready = [f for f in self.statics
                 if (f.drawn or not self.requires_sim) and not f.boarding]
        if not ready:
            return None, "nobody_parked"
        flight = self._nearest_to_user(
            ready, lambda f: self.ground.to_xy(f.latitude, f.longitude))
        flight.by_hand = True
        if self.schedule:
            flight.boarding_at = now + min(self.boarding_lead_s,
                                           self.BOARD_NOW_LEAD_S)
            log.info("traffic.debug: %s booked by hand, boarding at +%.0f s",
                     flight.written, flight.boarding_at - now)
            return flight, ""
        # Due already: the traffic's own tick sends it, on its own thread,
        # rather than this one taking it off the apron under the tick.
        flight.boarding = True
        flight.boarding_at = now - self.boarding_s - 1.0
        log.info("traffic.debug: %s boarded by hand", flight.written)
        return flight, ""

    def arrival_to_nearest_stand(self, now: float) -> tuple[Flight | None, str]:
        """An arrival on short final, holding the free stand nearest the
        pilot. Returns the flight and "", or None and a reason key."""
        if not self.field.ident:
            return None, "no_field"
        if self.requires_sim and not self.can_invent:
            return None, "not_in_sim"
        if self.ground is None:
            return None, "no_stands"
        user = self.user
        if user is None or not user.has_position:
            return None, "no_position"
        ends = getattr(self.field, "arrival_runways", None) \
            or [self.field.arrival_runway]
        if not any(self._threshold(e) is not None for e in ends):
            return None, "no_runway"
        occupied = self._on_ground_obstacles() + [
            (*self.ground.to_xy(lat, lon), 20.0)
            for lat, lon in self._parked_seen]
        stand = self.ground.free_stand(
            True, occupied, self._stands_taken(),
            choose=lambda options: self._nearest_to_user(
                options, lambda s: (s.x, s.z)))
        if stand is None:
            return None, "no_free_stand"
        flight = self._new_flight(now, arriving=True)
        if flight is None:
            return None, "no_callsign"
        flight.planned_stand = stand.index
        if flight.ifr:
            ahead = [f.distance_nm for f in self.traffic_ahead(True)]
            flight.distance_nm = max([self.SHORT_FINAL_NM]
                                     + [d + 3.0 for d in ahead
                                        if d < self.SHORT_FINAL_NM + 3.0])
            flight.altitude_ft = (self.field.elevation_ft
                                  + flight.distance_nm * GLIDE_FT_PER_NM)
            flight.sequence = len(ahead) + 1
            self._start_flying(flight)
        self._asked_for.append(flight)
        log.info("traffic.debug: %s inbound by hand, %.1f nm out, for stand %s",
                 flight.written, flight.distance_nm, stand.index)
        return flight, ""

    def _legs_from_the_ramp(self, flight: Flight):
        """The way from the ramp to this runway, for an aeroplane put on it
        by hand: somewhere to be held and somewhere to line up."""
        if self.ground is None:
            return None
        runway = self._runway_xy(flight.runway)
        if runway is None:
            return None
        threshold, heading = runway
        for stand in self.ground.stands:
            legs = self.ground.departure(stand, (stand.x, stand.z), threshold,
                                         heading, self.RUNWAY_LENGTH_M)
            if legs is not None:
                return legs
        return None

    def _put(self, flight: Flight, xy: tuple[float, float],
             heading: float | None) -> None:
        flight.on_layout = True
        flight.stand = None
        flight.latitude, flight.longitude = self.ground.to_latlon(*xy)
        if heading is not None:
            flight.heading = heading
        flight.altitude_ft = self.field.elevation_ft
        flight.ground_speed_kt = 0.0
        flight.on_ground = True

    # How long a departure waits for the runway before the tower stops
    # taking more arrivals to make a gap for it.
    DEPARTURE_PATIENCE_S = 120.0

    def _departures_kept_waiting(self, now: float) -> bool:
        return any(not f.done and not f.arriving and f.held_since
                   and now - f.held_since > self.DEPARTURE_PATIENCE_S
                   for f in self.flights)

    def _rest_of_taxi(self, flight: Flight, now: float, *,
                      inbound: bool) -> None:
        """The second half of a taxi cut in two, said once the aeroplane is
        at the taxiway it was told to hold short of: to the runway, or to
        the stand. Read back short, as the first half was."""
        if not flight.taxi_rest or flight.taxi_split_xy is None \
                or self.ground is None:
            return
        x, z = self.ground.to_xy(flight.latitude, flight.longitude)
        sx, sz = flight.taxi_split_xy
        if math.hypot(x - sx, z - sz) > REST_OF_TAXI_M:
            return
        rest, flight.taxi_rest, flight.taxi_split_xy = \
            flight.taxi_rest, [], None
        position = "GND" if "GND" in self.field.stations else "TWR"
        if inbound:
            self._exchange(
                flight, now, position,
                controller=lambda c, a, s: c.taxi_to_parking(a, s, rest),
                pilot_back=lambda p, a, s: p.leaving(a))
            return
        runway = flight.runway or self.field.departure_runway
        self._exchange(
            flight, now, position,
            controller=lambda c, a, s: c.taxi_to_runway(a, s, runway, rest),
            pilot_back=lambda p, a, s: p.taxi(a, runway,
                                              route=_spell_route(rest)))

    def _retire_at_gate(self) -> None:
        """Invented flights whose callsign a real one has turned up with go
        once they are standing at a gate."""
        if any(s.retire for s in self.statics):
            for s in self.statics:
                if s.retire:
                    self._used_callsigns.discard(s.written)
            self.statics = [s for s in self.statics if not s.retire]

    def _say_the_blend(self) -> None:
        """A row whenever the mix changes: the density by category, the real
        ones in it (live and simulator), the invented ones, and why."""
        live = {c: 0 for c in livemod.CATEGORIES}
        sim = {c: 0 for c in livemod.CATEGORIES}
        for f in self.flights:
            if f.observed and not f.done:
                (live if f.source == "live" else sim)[
                    livemod.flight_category(f)] += 1
        wanted = self._density_by_category()
        invented = self._invented_by_category()
        summary = (tuple(wanted.values()), tuple(live.values()),
                   tuple(sim.values()), tuple(invented.values()))
        if summary == getattr(self, "_blend_said", None):
            return
        self._blend_said = summary
        room = livemod.room(wanted, {c: live[c] + sim[c] for c in live})
        why = ("no real traffic here: all invented" if not any(live.values())
               and not any(sim.values())
               else "real traffic fills the density: none invented"
               if not any(room.values())
               else "invented fill the difference, by category")
        decision("blend", airport=self.field.ident, wanted=wanted,
                 live=live, simulator=sim, invented=invented, room=room,
                 why=why)

    # Codes a discrete squawk must never be: the emergency triplet, the
    # conspicuity codes VFR carries, and the blocks' own headers.
    _RESERVED_SQUAWKS = {"0000", "1200", "2000", "7000", "7500", "7600",
                       "7700", "7777"}

    def _new_squawk(self) -> str:
        """A discrete code nobody else on the scope already has.

        Drawn octal so 8 and 9 never appear -- and never an emergency or a
        conspicuity code, which a random decimal draw could otherwise hand
        an airliner.
        """
        taken = {f.squawk for f in self.flights} | self._RESERVED_SQUAWKS
        for _ in range(64):
            code = format(self.random.randint(0o1000, 0o7777), "04o")
            if code not in taken:
                return code
        return "4521"

    def _new_flight(self, now: float, arriving: bool | None = None,
                    parked: bool = True,
                    category: str | None = None) -> Flight | None:
        if arriving is None:
            arriving = (self.random.random() < 0.5
                        and len(self._approaching()) < self._max_on_approach())
            if arriving and self._departures_kept_waiting(now):
                # A departure has waited long enough. The stream of arrivals
                # is broken for it, the way a tower leaves a gap; at a
                # runway that has to be backtracked there is otherwise never
                # one long enough.
                arriving = False
        if not arriving and parked and self.ground is not None:
            if self.schedule:
                # Booked to board; it calls once it has. Nothing to admit
                # yet -- unless there is nobody parked to book.
                if self._book_boarding(now):
                    return None
            else:
                boarded = self._board_static(now)
                if boarded is not None:
                    return boarded
        # Which kind of aeroplane this is. Normally the field decides -- an
        # airline field carries airlines -- but a pilot can ask for a
        # frequency of registrations instead, which is a great deal easier to
        # follow than a roomful of telephony names you have to know by heart.
        airline_field = self._airline_field
        if self.callsigns == "registration":
            # A registration, but still the aeroplane the field would have.
            # Giving Heathrow a frequency full of Cessnas to get its
            # frequency full of registrations would be trading one wrong
            # thing for a louder one; a jet on a private registration is a
            # corporate aeroplane, which every large field really does have.
            made = self._light_flight(airline_fleet=airline_field)
        elif airline_field:
            made = self._airline_flight(cargo=(
                None if category is None else category == livemod.CARGO))
            if made is None:
                # An airline field whose runway takes none of the roster's
                # aeroplanes still flies -- the ones that do come there.
                made = self._light_flight()
        else:
            made = self._light_flight()
        if made is None:
            return None
        written, type_code, operator = made
        if self.force_types:
            type_code = self.random.choice(self.force_types)

        language = (operator.speaks_at(self.field.ident) if operator
                    else _local_or_english(self.field.ident))
        # Read the way this field reads it: digit by digit under ICAO, in
        # pairs only in US airspace. Without the dialect every invented flight
        # was grouped the American way -- "Air Portugal two fourteen" for
        # TAP0214 at Madeira, which no Portuguese controller would say and
        # which the user's own callsign, read correctly, sat next to.
        forms = every_language(written, type_code,
                               dialect=self.profile.dialect_for(self.field.ident))
        aircraft = Aircraft(forms["en"], type_code, wake_suffix(type_code),
                            callsigns=forms, language=language)

        runway = self._pick_runway(arriving, type_code)
        flight = Flight(
            squawk=self._new_squawk(),
            written=written,
            type_code=type_code,
            aircraft=aircraft,
            language=language,
            operator=operator,
            arriving=arriving,
            runway=runway,
            destination=self._elsewhere(),
            origin=self._elsewhere(),
            phase="inbound" if arriving else "clearance",
            next_at=now + self.random.uniform(2.0, 12.0),
            ifr=airline_field,
            wind_deg=self.field.weather.wind_dir
            if self.field.weather else 0.0,
            wind_kt=self.field.weather.wind_kt
            if self.field.weather else 0.0,
            performance=perftable.traffic_performance(
                type_code, light=operator is None),
            category=(livemod.CARGO if operator is not None
                      and operator.fleet == livemod.FREIGHT
                      else livemod.AIRLINE if airline_field else livemod.GA),
        )
        if not flight.ifr and (arriving or parked) and not self._vmc():
            # A VFR aeroplane does not fly in this: the field is under
            # its own ceiling and the light ones are staying home. The
            # parked ones may stand there -- it is the flying that the
            # weather stops.
            return None
        if not flight.ifr:
            # A VFR flight squawks the conspicuity code, which is regional, and
            # is given one by nobody: it sets it before it starts the engine.
            flight.squawk = self._phrase(language).vfr_squawk
            # A business jet comes and goes; the circuit share is the
            # field's (meridianatc.atc.fleet) -- and nobody does circuits in
            # the clouds at all.
            kind = self.field.kind
            share = kind.circuit_share if kind is not None else CIRCUIT_SHARE
            flight.pattern_work = (type_code not in fleets.BUSINESS
                                   and self._vmc()
                                   and self.random.random() < share)
            # And it comes from, or goes to, a field near this one, not Nice.
            flight.origin = self._nearby_field() or flight.origin
            flight.destination = self._nearby_field() or flight.destination
            flight.pattern_direction = (
                "right" if self.random.random() < 0.2 else "left")
        elif flight.ifr:
            # An IFR flight's ends are real aerodromes inside what it can
            # fly and what its operator flies -- the field at this end, a
            # navdata airport at the other -- not two names off a list that
            # had a SkyWest regional jet filed to Warsaw.
            self._assign_route(flight, arriving)
        if arriving and not flight.ifr:
            # Far enough out to be arriving rather than already in the
            # circuit, close enough that it joins within a few minutes.
            flight.distance_nm = self.random.uniform(4.0, 7.0)
            flight.sequence = len(self.traffic_ahead(True)) + 1
        elif arriving:
            ahead = self.traffic_ahead(True)
            # Far enough behind whoever is already on the approach to be a
            # separate aeroplane rather than a formation.
            flight.distance_nm = max(
                [f.distance_nm for f in ahead] + [10.0]) + self.random.uniform(
                    5.0, 11.0)
            flight.altitude_ft = (self.field.elevation_ft
                                  + flight.distance_nm * GLIDE_FT_PER_NM)
            flight.sequence = len(ahead) + 1
        else:
            flight.sequence = len(self.traffic_ahead(False)) + 1
            # A VFR departure has no clearance to collect, and a light field
            # has no delivery position to collect one from. Either way it
            # starts on the ground frequency.
            if not flight.ifr or "DEL" not in self.field.stations:
                flight.phase = "taxi"
        self._used_callsigns.add(written)
        if (arriving and self.schedule and self.ground is not None
                and (flight.operator is not None or flight.ifr)):
            # Its stand, held from now: where it will turn round decides
            # which model it is drawn with, so it is drawn right from the
            # first frame and never swapped at the gate.
            stand = self._free_stand(airliner=True)
            if stand is not None:
                flight.planned_stand = stand.index
        if flight.phase in _FLYING_PHASES:
            self._start_flying(flight)
        elif self.ground is not None and parked:
            # On a real stand, nobody else's, and not the pilot's. No stand
            # free is no departure: a full apron is a quiet frequency, not an
            # aeroplane parked on top of another one.
            stand = self._free_stand(airliner=flight.operator is not None
                                     or flight.ifr)
            if stand is None:
                # No departure and no aeroplane: the callsign is given
                # back too, or a full apron burns one every attempt.
                self._used_callsigns.discard(flight.written)
                return None
            self._park(flight, stand)
        else:
            self._place(flight)
        return flight

