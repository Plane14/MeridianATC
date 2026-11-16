from __future__ import annotations

from ._shared import (
    BASE_LOOKAHEAD_NM,
    CIRCUIT_BASE_NM,
    CIRCUIT_WIDTH_NM,
    STABLE_BY_NM,
    Aircraft,
    Flight,
    _runway_key,
    _strips_parallel,
    _turn_radius_nm,
    approach_speed,
    article,
    every_language,
    make_of,
    runway_traffic,
    wake_suffix,
    wind_moved,
    wind_snapshot,
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
                self._watch_wind(flight, now)
                self._watch_spacing(flight, now)
            elif flight.phase == "downwind":
                self._watch_downwind(flight, now)
            elif flight.phase in ("takeoff", "landed"):
                self._watch_the_roll(flight, now)
            if not flight.on_ground and flight.phase != "takeoff":
                self._watch_conflict(flight, now)

    # How far ahead the tower looks for two aeroplanes meeting in the
    # air, and how close counts as a conflict: the figures a short-term
    # conflict alert works to, scaled to a tower's patch rather than a
    # centre's.
    CONFLICT_LOOKAHEAD_S = 75.0
    CONFLICT_NM = 1.0
    CONFLICT_FT = 600.0
    AVOID_TURN_DEG = 40.0
    # "Head-on" is a few degrees either side of the nose, not the one
    # side of it: both of a meeting pair turn right.
    HEAD_ON_DEG = 20.0

    def _conflict(self, flight: Flight, other_lat: float, other_lon: float,
                  other_alt: float, other_hdg: float,
                  other_gs: float, *, heading: float | None = None
                  ) -> tuple | None:
        """(miss_nm, seconds, vertical_gap_ft) where this flight and the
        other come closest -- None where the meeting is already past or
        no conflict is predicted. ``heading`` asks the question of a
        track other than the one being flown."""
        from ..trafficinfo import closest_approach

        try:
            miss, t, vgap = closest_approach(
                flight.latitude, flight.longitude,
                float(flight.altitude_ft or 0.0),
                float(flight.heading or 0.0) if heading is None
                else float(heading),
                float(flight.ground_speed_kt or 0.0),
                other_lat, other_lon, float(other_alt or 0.0),
                float(other_hdg or 0.0), float(other_gs or 0.0),
                horizon_s=self.CONFLICT_LOOKAHEAD_S)
        except Exception:
            return None
        if t <= 0.0:
            # The closest point is behind them: they are opening.
            return None
        if miss < self.CONFLICT_NM and vgap < self.CONFLICT_FT:
            return miss, t, vgap
        return None

    def _watch_conflict(self, flight: Flight, now: float) -> None:
        """Look a minute ahead and move this aeroplane before a merge,
        the way a conflict alert moves a controller.

        A tower does not wait for two returns to touch: it sees the
        tracks meeting and turns one of them while there is still time
        for the turn to matter. The look is pointed at everything
        airborne in the picture -- the person flying, the simulator's
        own aeroplanes, and the program's -- but the turn goes only to
        the aeroplane under this controller, which is the one that can
        be told. A flight on short final is never the one turned: it
        has its own answers, and the conflict is the other aircraft's
        to resolve anyway.
        """
        user = self.user
        threat = None
        for other in self.flights:
            if other is flight or other.done or other.on_ground:
                continue
            hit = self._conflict(flight, other.latitude, other.longitude,
                                 other.altitude_ft, other.heading,
                                 other.ground_speed_kt)
            if hit is not None:
                threat = (other, hit)
                break
        if threat is None and user is not None and user.has_position \
                and not user.on_ground:
            hit = self._conflict(flight, user.latitude, user.longitude,
                                 user.altitude_ft, user.heading,
                                 user.ground_speed_kt)
            if hit is not None:
                threat = (user, hit)

        avoiding = getattr(flight, "_avoid_key", "")
        if threat is None:
            # Nothing predicted on the heading being flown. That is what
            # the turn was for: whether the meeting is past is asked of
            # the track the flight was taken off, or "resume" hands it
            # straight back to the conflict and the pair is turned and
            # released for as long as they are in sight of each other.
            if not avoiding:
                return
            left = getattr(flight, "_avoid_from", None)
            if left is not None and self._still_meets(flight, avoiding, left):
                return
            flight._avoid_key = ""
            flight._avoid_from = None
            # Sent back to its own navigation once -- where the vector
            # is still the one it is flying; a phase that has since
            # taken the heading away has nothing to be released from.
            if flight.assigned_heading is not None \
                    and not self._already_told(flight, "resume"):
                self._exchange(
                    flight, now, flight.position,
                    controller=lambda c, a, s: c.resume_own_navigation(a),
                    pilot_back=lambda p, a, s: p.wilco(a),
                    command=("resume", {"only": "heading"}))
            return

        other, _hit = threat
        key = getattr(other, "written", "") or "user"
        if avoiding == key or self._already_told(flight, "heading"):
            return
        # The final's own machinery owns the approach: a conflict on it
        # is a go-around or the other aircraft's turn, not a vector off
        # the centreline at a mile.
        if flight.phase == "final":
            return
        o_lat, o_lon = other.latitude, other.longitude
        bearing = self._bearing(flight.latitude, flight.longitude,
                              o_lat, o_lon)
        off = (bearing - flight.heading + 540.0) % 360.0 - 180.0
        # Head-on is a right turn -- the rules of the air's answer, and
        # the one that does not turn the pair into each other; anything
        # else is a turn away from the side the threat is on.
        side = 1.0 if off <= self.HEAD_ON_DEG else -1.0
        hdg = (flight.heading + side * self.AVOID_TURN_DEG) % 360.0
        direction = "right" if side > 0 else "left"
        if not avoiding:
            flight._avoid_from = flight.heading
        flight._avoid_key = key
        self._exchange(
            flight, now, flight.position,
            controller=lambda c, a, s: c.turn_heading(a, hdg, direction),
            pilot_back=lambda p, a, s: p.heading(a, hdg),
            command=("heading", {"heading": hdg, "turn": direction}))

    def _still_meets(self, flight: Flight, key: str, heading: float) -> bool:
        """Whether the track ``heading`` would still meet the aeroplane
        this flight was turned away from."""
        user = self.user
        if key == "user":
            others = [user] if user is not None and user.has_position \
                and not user.on_ground else []
        else:
            others = [f for f in self.flights
                      if f is not flight and not f.done and not f.on_ground
                      and f.written == key]
        return any(
            self._conflict(flight, o.latitude, o.longitude, o.altitude_ft,
                           o.heading, o.ground_speed_kt,
                           heading=heading) is not None
            for o in others)

    @staticmethod
    def _bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        from ...navdata.build import bearing_deg
        return bearing_deg(lat1, lon1, lat2, lon2)

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

    def _watch_wind(self, flight: Flight, now: float) -> None:
        """"The wind is now ..." when it has moved since the clearance.

        The clearance carried the field's wind at the time; when it has
        moved enough that the ATIS would have re-issued, the tower says
        so on final -- the way an arrival hears "wind is now two six
        zero at one eight, gusting two five" a mile out.
        """
        if flight.phase != "final" or flight.on_ground:
            return
        was = flight.wind_at_clearance
        if not wind_moved(was, self.field.weather):
            return
        flight.wind_at_clearance = wind_snapshot(self.field.weather)
        self._exchange(
            flight, now, "TWR",
            controller=lambda c, a, s: c.wind_check(
                self.field.weather, a),
            pilot_back=lambda p, a, s: p.roger(a),
        )

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

    # Told to roll or told to vacate, and still barely moving this long
    # with the next arrival this close behind it: "expedite". UNVERIFIED,
    # both figures -- the tower's version of the same watch is twelve
    # seconds and three miles (atc.departure.DepartureWatch).
    EXPEDITE_ROLL_AFTER_S = 10.0
    EXPEDITE_ROLL_WITHIN_NM = 3.0

    def _watch_the_roll(self, flight: Flight, now: float) -> None:
        """Hurry an aeroplane that is standing where the runway wants it
        moving -- a departure that has not rolled, an arrival not yet
        vacating -- when somebody is on final behind it."""
        if not getattr(self, "expedite", True) or not flight.on_ground \
                or flight.ground_speed_kt >= 30.0 \
                or self._already_told(flight, "expedite"):
            flight.__dict__.pop("slow_roll_since", None)
            return
        if self._traffic_on_final(self.EXPEDITE_ROLL_WITHIN_NM,
                                  ignore=flight,
                                  runway=flight.runway) is None:
            flight.__dict__.pop("slow_roll_since", None)
            return
        since = float(getattr(flight, "slow_roll_since", 0.0) or 0.0)
        if not since:
            flight.slow_roll_since = now
            return
        if now - since < self.EXPEDITE_ROLL_AFTER_S:
            return
        flight.slow_roll_since = 0.0
        reason = ("vacating" if flight.phase == "landed"
                  else "your departure")
        self._exchange(
            flight, now, "TWR",
            controller=lambda c, a, s: c.expedite(a, reason),
            command=("expedite", {}))

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

        # There is room to turn in, and somebody is standing at the hold
        # point waiting for it: tighten the circuit rather than stretch
        # it, the way a tower gets a departure out between arrivals.
        if room and not flight.extending \
                and not getattr(flight, "short_approach", False) \
                and self._departure_waiting(flight):
            flight.short_approach = True
            self._exchange(
                flight, now, "TWR",
                controller=lambda c, a, s: c.make_short_approach(
                    a, reason="traffic waiting to depart"),
                pilot_back=lambda p, a, s: p.wilco(a),
                command=("short_approach", {}),
            )
            return

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
            self.SPACING_NM * 3.0, ignore=flight,
            runway=flight.runway)
        self._exchange(
            flight, now, "TWR",
            controller=lambda c, a, s: c.extend_downwind(
                a, reason=self._describe_arrival(ahead, flight.language)),
            pilot_back=lambda p, a, s: p.wilco(a),
            command=("extend", {}),
        )

    def _departure_waiting(self, ignore: Flight) -> bool:
        """Somebody standing at the hold point for the same runway."""
        runway = getattr(ignore, "runway", None)
        return any(
            not f.done and f is not ignore and not f.arriving
            and f.on_ground and f.runway == runway
            and f.phase in ("ready", "waiting", "lineup")
            for f in self.flights)

    def _runway_blocked_for(self, flight: Flight, now: float) -> bool:
        """Whether this arrival will have something in its way when it gets there.

        The question a controller asks about an aeroplane a mile out is not
        whether the runway is clear now -- it rarely is, a mile out -- but
        whether it will be clear in the half minute the aeroplane still has to
        run. Asking the first question sent an arrival around every time the
        one in front of it was still rolling out, which on a single runway is
        every time.
        """
        runway = flight.runway or self.field.arrival_runway
        if self._user_on_runway(runway):
            return True
        if self._somebody_on_runway(ignore=flight, runway=runway):
            # Crossing it, backtracking along it, still turning off it:
            # whatever the timetable says, the runway is not clear.
            return True
        arrives_in = self._seconds_to_threshold(flight)
        if self._runway_free_at(runway) > now + arrives_in:
            return True
        strip = _runway_key(runway)
        for other in self.flights:
            if other is flight or other.done:
                continue
            theirs = _runway_key(
                other.runway
                or (self.field.arrival_runway if other.arriving
                    else self.field.departure_runway))
            if theirs != strip and _strips_parallel(theirs, strip):
                # Another runway's queue: a parallel lined up or landing
                # does not put anything on this strip.
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
                    command=("resume", {"only": "speed"}),
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
                          ignore: Flight | None = None,
                          runway: str = "") -> Flight | None:
        """Anybody on final inside that distance, the user counted.
        ``runway`` scopes it to a strip: a parallel's final is somebody
        else's runway, not this one's.

        The user does not have a :class:`Flight`, so a stand-in is made for
        them. It is never added to the roster: it exists for the length of
        this answer and says only what has to be known about it.
        """
        for other in self.flights:
            if other is ignore or other.done:
                continue
            if runway and not self._conflicts(
                    runway, other, self.field.arrival_runway):
                continue        # another strip's final does not count
            if (other.arriving and other.phase in ("final", "base")
                    and other.distance_nm <= miles):
                return other
        to_run = self._user_on_final()
        if to_run is not None and to_run <= miles:
            stand = self._user_flight()
            if not runway or self._conflicts(
                    runway, stand, self.field.arrival_runway):
                return stand
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
        ends = getattr(self.field, "arrival_runways", None) \
            or [self.field.arrival_runway or self.field.departure_runway]
        if not any(ends) or not self.field.ident:
            return None
        from ..circuit import circuit_for

        for runway in ends:
            if not runway:
                continue
            published = self.navdb.circuit_data(self.field.ident, runway)
            side = (published.side
                    if published is not None and published.side else "left")
            circuit = circuit_for(self.navdb, self.field.ident, runway,
                                  side=side)
            if circuit is None:
                continue
            here = circuit.locate_at(user.latitude, user.longitude,
                                     user.altitude_ft, user.heading)
            if here.leg not in ("base", "downwind", "final"):
                continue
            if here.to_go_nm > self.USER_IN_CIRCUIT_NM:
                continue
            return here.to_go_nm
        return None

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
        final = self._final_for(user)
        return Flight(
            written=user.callsign, type_code=user.type_code,
            aircraft=aircraft, language="en", operator=None,
            arriving=True,
            runway=(final[1] if final else self.field.arrival_runway),
            phase="final", latitude=user.latitude, longitude=user.longitude,
            altitude_ft=user.altitude_ft, heading=user.heading,
            distance_nm=(distance_nm if distance_nm is not None
                         else self._user_on_final() or 0.0),
        )

