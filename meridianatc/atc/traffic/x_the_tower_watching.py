from __future__ import annotations

from ._shared import (
    BASE_LOOKAHEAD_NM,
    CIRCUIT_BASE_NM,
    CIRCUIT_WIDTH_NM,
    STABLE_BY_NM,
    Aircraft,
    Flight,
    _turn_radius_nm,
    approach_speed,
    article,
    every_language,
    make_of,
    runway_traffic,
    wake_suffix,
    windshear,
)


class _The_tower_watching:
    # ------------------------------------------------------------------
    # the tower, watching
    # ------------------------------------------------------------------

    # How close an arrival gets to a blocked runway before it is sent around,
    # and how much room it needs behind the one in front.
    #
    # The spacing is what it is because of the runway rather than because of
    # wake: an aeroplane is on it for the best part of a minute after it
    # lands, so a stream built to anything tighter than that is a stream that
    # has to send every other aeroplane round again.
    GO_AROUND_NM = 1.3
    SPACING_NM = 3.5

    def _watch(self, now: float) -> None:
        """Look at what is actually happening and say something about it.

        Everything else in this module is a flight following its own script.
        This is the part that is not: a controller looking at a runway with
        somebody on it and an aeroplane a mile from it, and doing what a
        controller does about that.
        """
        if now < self._watched_at:
            return
        self._watched_at = now + 1.0
        for flight in list(self.flights):
            if flight.done or flight.observed or not flight.airborne_model:
                # An observed aeroplane is flown by whoever injected it and
                # will not go around, slow down or extend for anybody. Telling
                # it to would be a controller talking to itself.
                continue
            if flight.arriving and flight.phase in ("inbound", "final"):
                self._watch_final(flight, now)
                self._watch_spacing(flight, now)
            elif flight.phase == "downwind":
                self._watch_downwind(flight, now)

    # A stable approach, the crew's own gate: inside this distance it is on
    # the centreline to within this, and pointing down the runway to within
    # this, or it is not landing.
    STABLE_BY_NM = 1.5
    STABLE_BY_NM_LIGHT = 0.5
    STABLE_ACROSS_M = 60.0
    STABLE_HEADING_DEG = 20.0

    def _stable_by_nm(self, flight: Flight) -> float:
        """Where this crew's own stable-approach gate is: an airliner's a mile
        and a half out, a light aircraft's much closer in."""
        return (self.STABLE_BY_NM if flight.performance.approach_kt >= 110
                else self.STABLE_BY_NM_LIGHT)

    def _base_nm(self, flight: Flight) -> float:
        """How far out the base turn is, so final is long enough to be
        stable by the gate. A fixed mile and a half suited a Cessna and put
        an airliner on a final shorter than its own gate, so it went round
        every time."""
        radius = _turn_radius_nm(flight.performance.approach_kt * 1.15)
        return max(CIRCUIT_BASE_NM, self._stable_by_nm(flight) + 1.0 + radius)

    def _unstable(self, flight: Flight) -> bool:
        end = self._threshold(flight.runway)
        if end is None or flight.distance_nm > self._stable_by_nm(flight):
            return False
        _along, across = self._runway_frame(flight, end[2])
        off = abs((flight.heading - end[2] + 540.0) % 360.0 - 180.0)
        return (abs(across) * 1852.0 > self.STABLE_ACROSS_M
                or off > self.STABLE_HEADING_DEG)

    def _weather_go_around(self, flight: Flight) -> bool:
        """Whether this approach ends for the weather.

        Rolled once for an approach, not watched every tick: a gusty day
        does not send everybody around, it sends the one whose luck came
        in -- a spread of ten knots and more between the wind and its
        gusts, which a light aeroplane baulks at oftener than a heavy
        one, or a ceiling under three hundred feet, which is under what
        anybody lands out of. A field that does not report sends nobody
        around: weather cannot be bad until it is known to be.
        """
        if getattr(flight, "weather_ga_done", False):
            return False
        weather = self.field.weather
        if weather is None or not getattr(weather, "raw", ""):
            return False
        roll = getattr(flight, "weather_ga_roll", None)
        if roll is None:
            roll = self.random.random()
            flight.weather_ga_roll = roll
        gust = float(getattr(weather, "gust_kt", 0.0) or 0.0)
        wind = float(getattr(weather, "wind_kt", 0.0) or 0.0)
        if gust - wind >= 10.0:
            light = getattr(flight.performance, "approach_kt", 130.0) < 110
            chance = 0.35 if light else 0.12
        else:
            chance = 0.0
        ceiling = min((h for cover, h in getattr(weather, "clouds", [])
                       if cover in ("BKN", "OVC", "VV")), default=None)
        if ceiling is not None and ceiling < 300:
            chance = max(chance, 0.5)
        if roll < chance:
            flight.weather_ga_done = True
            return True
        return False

    def _watch_final(self, flight: Flight, now: float) -> None:
        """An aeroplane on short final to a runway that may not be free."""
        if flight.phase != "final" or flight.on_ground:
            return
        if (flight.distance_nm > self.GO_AROUND_NM
                and not self._unstable(flight)):
            return
        if self._already_told(flight, "go_around"):
            return
        if not flight.observed and self._weather_go_around(flight):
            # The weather sent it: a gust spread the pilot did not like,
            # or the field gone under the cloud before the threshold.
            # Rolled once per approach in _weather_go_around, and said by
            # the crew, the way a windshear call comes from the aeroplane.
            self._exchange(
                flight, now, "TWR",
                pilot_first=lambda p, a, s: p.going_around(a),
                controller=lambda c, a, s: c.go_around(
                    a, reason=windshear(flight.language)),
                command=("go_around", {}),
                urgent=True,
            )
            return
        if not flight.observed and self._unstable(flight):
            # Not lined up with the runway by now, and a crew does not land
            # from there: they go around and say so. Without this an
            # aeroplane that turned final late touched down two hundred
            # metres to the side of the runway, on the apron, beside the
            # pilot's own stand.
            self._exchange(
                flight, now, "TWR",
                pilot_first=lambda p, a, s: p.going_around(a),
                controller=lambda c, a, s: c.roger(a),
                command=("go_around", {}),
                urgent=True,
            )
            return
        if not self._runway_blocked_for(flight, now):
            return
        reason = runway_traffic(flight.language)
        self._exchange(
            flight, now, "TWR",
            controller=lambda c, a, s: c.go_around(a, reason=reason),
            pilot_back=lambda p, a, s: p.going_around(a),
            command=("go_around", {}),
            urgent=True,
        )

    # How far past the normal turning point an aeroplane is asked to go before
    # it is brought back in regardless. An extension is a delay, not a
    # dismissal, and a crew told to extend and never told anything else would
    # fly to the coast.
    MAX_EXTEND_NM = 6.0
    # And past this it is simply gone: out of sight, and somebody else's.
    GIVE_UP_EXTENDING_NM = 20.0

    def _watch_downwind(self, flight: Flight, now: float) -> None:
        """An aeroplane at the point it would turn base, with somebody on final.

        Two decisions, and the second is the one that was missing: an aircraft
        told to extend has to be told when to stop. Without that it flies
        downwind until the map runs out, which is a controller who has
        forgotten about it.
        """
        # Where it would roll out if it turned base now, and whether that
        # leaves anybody crowded -- by then, not now.
        _along, across = self._runway_frame(
            flight, self._threshold(flight.runway)[2]) \
            if self._threshold(flight.runway) else (0.0, CIRCUIT_WIDTH_NM)
        room = self._room_on_final(flight.distance_nm, ignore=flight,
                                   within_s=self._rollout_s(flight, across))

        if flight.extending:
            if not room:
                # Still no gap. It keeps going until there is one -- the
                # arrivals are spaced five miles and more apart, so one comes
                # within a minute of the one abeam passing. Turning it in at
                # a fixed distance regardless put it on base beside an
                # arrival and across its approach.
                if flight.distance_nm > self.GIVE_UP_EXTENDING_NM:
                    flight.done = True
                return
            if self._already_told(flight, "continue"):
                return
            self._exchange(
                flight, now, "TWR",
                controller=lambda c, a, s: c.continue_approach(a),
                pilot_back=lambda p, a, s: p.wilco(a),
                command=("continue", {}),
            )
            return

        if (flight.distance_nm < self._base_nm(flight) - BASE_LOOKAHEAD_NM
                or room or self._already_told(flight, "extend")):
            return
        ahead = self._ahead_of(flight) or self._traffic_on_final(
            self.SPACING_NM * 3.0, ignore=flight)
        self._exchange(
            flight, now, "TWR",
            controller=lambda c, a, s: c.extend_downwind(
                a, reason=self._describe_arrival(ahead, flight.language)),
            pilot_back=lambda p, a, s: p.wilco(a),
            command=("extend", {}),
        )

    def _runway_blocked_for(self, flight: Flight, now: float) -> bool:
        """Whether this arrival will have something in its way when it gets there.

        The question a controller asks about an aeroplane a mile out is not
        whether the runway is clear now -- it rarely is, a mile out -- but
        whether it will be clear in the half minute the aeroplane still has to
        run. Asking the first question sent an arrival around every time the
        one in front of it was still rolling out, which on a single runway is
        every time.
        """
        if self._user_on_runway():
            return True
        if self._somebody_on_runway(ignore=flight):
            # Crossing it, backtracking along it, still turning off it:
            # whatever the timetable says, the runway is not clear.
            return True
        arrives_in = self._seconds_to_threshold(flight)
        runway = flight.runway or self.field.arrival_runway
        if self._runway_free_at(runway) > now + arrives_in:
            return True
        for other in self.flights:
            if other is flight or other.done:
                continue
            # An aeroplane sitting on the runway waiting for a clearance is
            # not going anywhere on its own. Everything else that is on it --
            # rolling, landing -- is already counted by the time above.
            if other.phase == "lineup":
                return True
            if (other.arriving and other.phase == "final"
                    and other.distance_nm < flight.distance_nm
                    and flight.distance_nm - other.distance_nm < 0.8):
                return True
        return False

    def _approaching(self, ignore: Flight | None = None) -> list[Flight]:
        """Everybody committed to this runway, in miles still to run.

        The person flying is in the list on the same terms as anybody else,
        because to everything that reads it they are another aeroplane on
        final.
        """
        found = [f for f in self.flights
                 if f is not ignore and not f.done and not f.stuck
                 and f.arriving and f.phase in ("inbound", "final", "base")]
        # Only when they are on final, or in the circuit close to turning on
        # to it. The stand-in was added whatever the user was doing, at a
        # distance of nought when they were not on final -- so a pilot parked
        # at the gate was an arrival at the threshold, every departure was
        # held for them for ever, and nothing at the field ever took off.
        # On base or late downwind they are at the distance they still have
        # to fly, which is what makes the AI behind them extend rather than
        # turn in front of them (atc.circuit).
        to_run = self._user_on_final()
        if to_run is None:
            to_run = self._user_in_circuit()
        if to_run is not None:
            user = self._user_flight(to_run)
            if user is not None:
                found.append(user)
        found.sort(key=lambda f: f.distance_nm)
        return found

    def _room_on_final(self, at_nm: float,
                       ignore: Flight | None = None,
                       within_s: float = 0.0) -> bool:
        """Whether an aeroplane could roll out there without crowding anybody.

        This is what the base turn actually waits for. Turning when the
        aeroplane ahead merely happens to be outside some radius is how two
        arrivals end up a fifth of a mile apart on a four mile final -- which
        is what this used to do.
        """
        # Where everybody will be by the time it rolls out, not where they
        # are now. An arrival at a hundred and eighty knots closes three
        # miles in the minute a base leg takes, and judging the gap as it
        # stood turned an aeroplane in front of one that then ran into it.
        return all(abs(max(0.0, other.distance_nm
                           - other.ground_speed_kt * within_s / 3600.0)
                       - at_nm) >= self.SPACING_NM
                   for other in self._approaching(ignore))

    def _rollout_s(self, flight: Flight, across_nm: float) -> float:
        """How long from here to rolling out on final: the base leg and the
        turn at the end of it."""
        speed = max(60.0, flight.performance.approach_kt * 1.15)
        return (abs(across_nm) + 1.6 * _turn_radius_nm(speed)) \
            / speed * 3600.0

    def _ahead_of(self, flight: Flight) -> Flight | None:
        """The aeroplane immediately in front of this one on the approach."""
        nearer = [f for f in self._approaching(flight)
                  if f.distance_nm < flight.distance_nm]
        return nearer[-1] if nearer else None

    def _watch_spacing(self, flight: Flight, now: float) -> None:
        """Keep an arrival off the back of the one in front of it.

        A speed is what a controller has for this, and it is the instruction
        with the least drama in it: nobody is sent around, the aeroplane just
        slows down and the gap opens. It only works while there is still room
        to lose speed, so it is not offered to anything already committed.
        """
        # Below this there is nothing left to give: an aeroplane at two miles
        # is at its threshold speed and slowing it further is not a spacing
        # instruction, it is a stall warning.
        if flight.distance_nm < STABLE_BY_NM:
            return
        ahead = self._ahead_of(flight)
        gap = (flight.distance_nm - ahead.distance_nm) if ahead else 99.0

        if flight.assigned_speed_kt is not None:
            if gap >= self.SPACING_NM * 1.6 or ahead is None:
                if self._already_told(flight, "resume"):
                    return
                self._exchange(
                    flight, now, flight.position,
                    controller=lambda c, a, s: c.resume_normal_speed(a),
                    pilot_back=lambda p, a, s: p.wilco(a),
                    command=("resume", {}),
                )
            return

        if gap >= self.SPACING_NM or self._already_told(flight, "speed"):
            return
        # Slower than the one in front, or the gap merely stops shrinking
        # instead of opening.
        wanted = min(approach_speed(ahead.distance_nm, flight.performance) - 15.0,
                     flight.performance.approach_kt)
        slower = max(round(wanted / 10.0) * 10,
                     round(flight.performance.approach_kt * 0.85 / 10.0) * 10)
        self._exchange(
            flight, now, flight.position,
            controller=lambda c, a, s: c.speed_restriction(a, slower),
            pilot_back=lambda p, a, s: p.speed(a, slower),
            command=("speed", {"speed_kt": slower}),
        )

    def _traffic_on_final(self, miles: float,
                          ignore: Flight | None = None) -> Flight | None:
        """Anybody on final inside that distance, the user counted.

        The user does not have a :class:`Flight`, so a stand-in is made for
        them. It is never added to the roster: it exists for the length of
        this answer and says only what has to be known about it.
        """
        for other in self.flights:
            if other is ignore or other.done:
                continue
            if (other.arriving and other.phase in ("final", "base")
                    and other.distance_nm <= miles):
                return other
        to_run = self._user_on_final()
        if to_run is not None and to_run <= miles:
            return self._user_flight()
        return None

    # How far round the circuit the person flying can be and still count as
    # the next to land: base, or late on the downwind.
    USER_IN_CIRCUIT_NM = 4.0

    def _user_in_circuit(self) -> float | None:
        """What the person flying still has to fly to the threshold, when
        they are on base or late downwind; None anywhere else."""
        user = self.user
        if user is None or user.on_ground or not user.has_position:
            return None
        runway = self.field.arrival_runway or self.field.departure_runway
        if not runway or not self.field.ident:
            return None
        from ..circuit import circuit_for

        published = self.navdb.circuit_data(self.field.ident, runway)
        side = published.side if published is not None and published.side else "left"
        circuit = circuit_for(self.navdb, self.field.ident, runway, side=side)
        if circuit is None:
            return None
        here = circuit.locate_at(user.latitude, user.longitude,
                                 user.altitude_ft, user.heading)
        if here.leg not in ("base", "downwind", "final"):
            return None
        if here.to_go_nm > self.USER_IN_CIRCUIT_NM:
            return None
        return here.to_go_nm

    def _user_ahead_of(self, flight: Flight) -> str:
        """The person flying, said as traffic, when they are ahead of this
        aeroplane in the circuit at a field nobody works -- the "number two,
        following the Cessna" an AI pilot announces. "" otherwise."""
        if self.field.stations.get("TWR") is not None or self.user is None:
            return ""
        to_run = self._user_on_final()
        if to_run is None:
            to_run = self._user_in_circuit()
        if to_run is None or to_run >= flight.distance_nm:
            return ""
        return article(make_of(self.user.type_code), flight.language)

    def _user_flight(self, distance_nm: float | None = None) -> Flight | None:
        """The person flying, in the shape the rest of this module reads."""
        user = self.user
        if user is None:
            return None
        forms = every_language(user.callsign or "UNKNOWN", user.type_code,
                               dialect=self.profile.dialect_for(self.field.ident))
        aircraft = Aircraft(forms["en"], user.type_code,
                            wake_suffix(user.type_code), callsigns=forms)
        return Flight(
            written=user.callsign, type_code=user.type_code,
            aircraft=aircraft, language="en", operator=None,
            arriving=True, runway=self.field.arrival_runway,
            phase="final", latitude=user.latitude, longitude=user.longitude,
            altitude_ft=user.altitude_ft, heading=user.heading,
            distance_nm=(distance_nm if distance_nm is not None
                         else self._user_on_final() or 0.0),
        )

