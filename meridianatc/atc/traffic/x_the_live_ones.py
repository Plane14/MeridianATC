from __future__ import annotations

from ._shared import (
    FT_PER_HPA,
    GIVE_WAY_DEG,
    GIVE_WAY_HEAD_ON_DEG,
    GIVE_WAY_HEAD_ON_M,
    GIVE_WAY_M,
    GIVE_WAY_STUCK_M,
    GIVE_WAY_STUCK_S,
    HOLD_SHORT_EVERY_S,
    INITIAL_CLIMB_FT,
    LANDING_CLEARANCE_NM,
    LIFTOFF_AFTER_S,
    LINED_UP_DEG,
    LIVE_AHEAD_DEG,
    LIVE_ALIGNED_DEG,
    LIVE_BEHIND_NM,
    LIVE_CENTRELINE_M,
    LIVE_FINAL_M,
    LIVE_GO_AROUND_NM,
    LIVE_PARKED_S,
    LIVE_ROLL_MS,
    LIVE_RUNWAY_M,
    LIVE_STAND_M,
    LIVE_TAXI_IN_KT,
    LIVE_TAXIWAY_M,
    M_PER_DEG,
    OBSERVED_CEILING_FT,
    PARKED_RANGE_NM,
    PILOT_FINAL_NM,
    PUSHBACK_DEG,
    PUSHBACK_KT,
    RELEASE_GAP_S,
    ROLLING_KT,
    RUNWAY_LENGTH_NM,
    RUNWAY_WIDTH_NM,
    TAXIING_KT,
    Aircraft,
    Flight,
    ParkedObserved,
    UserAircraft,
    _runway_key,
    _said_route,
    _spell_route,
    _squawk,
    bearing_deg,
    by_icao,
    decision,
    every_language,
    haversine_nm,
    livemod,
    math,
    observed_type,
    offset_from_track,
    perftable,
    reply_language,
    runway_traffic,
    wake_suffix,
)


class _The_live_ones:
    # -- the live ones on the scenery (atc/live.py) ------------------------

    def _live_pose(self, one, now: float):
        """A live position, put where the simulator's scenery has it.

        The reports are a receiver's, metres off: a departure on its roll
        drawn beside the centreline, a taxiing one on the grass, a parked one
        across two stands, and every one of them at the height of a pressure
        altitude rather than of the simulator's ground. So:

        - on the runway or its final, aligned with it and within 40 m of the
          centreline, it is put on the centreline (the pilot's figure,
          30 Oct 2026);
        - taxiing, on the nearest taxiway, within 40 m;
        - parked, on the stand it is on;
        - on the ground, at the ground's height; in the air, its pressure
          altitude corrected to the field's QNH, never below the field.

        And an arrival taxiing in near a free stand is rolled onto it
        (:meth:`_roll_in`), where it then stays.
        """
        from dataclasses import replace

        if self.ground is not None:
            rolled = self._roll_in(one, now)
            if rolled is not None:
                return rolled
        lat, lon, heading = one.latitude, one.longitude, one.heading
        snapped = ""
        runway = self._on_a_centreline(lat, lon, heading)
        if runway is not None:
            _, lat, lon, course = runway
            if one.on_ground:
                heading = course
            snapped = "runway"
        elif one.on_ground and self.ground is not None:
            if one.ground_speed_kt < livemod.MOVING_KT:
                x, z = self.ground.to_xy(lat, lon)
                stand = self.ground.stand_at(x, z)
                if stand is not None:
                    lat, lon = self.ground.to_latlon(stand.x, stand.z)
                    heading, snapped = stand.heading, "stand"
            else:
                taxiway = self._on_a_taxiway(lat, lon)
                if taxiway is not None:
                    lat, lon = taxiway
                    snapped = "taxiway"
        elevation = self.field.elevation_ft
        if one.on_ground:
            altitude = elevation
        else:
            altitude = max(elevation, self._qnh_altitude(one.altitude_ft))
        if not one.on_ground and one.hex:
            self._live_flew.add(one.hex)
        self.live_snapped[(one.callsign or "").upper()] = snapped
        return replace(one, latitude=lat, longitude=lon, heading=heading,
                       altitude_ft=altitude)

    def _qnh_altitude(self, pressure_ft: float) -> float:
        """A pressure altitude (1013.25) as the altitude above the sea the
        simulator draws at: about 27 ft a hectopascal near the ground (ISA).
        Above the transition the difference is nobody's business, and it is
        not applied above FL180."""
        weather = self.field.weather
        qnh = getattr(weather, "qnh_hpa", None) if weather is not None else None
        if not qnh or pressure_ft > 18000.0:
            return pressure_ft
        return pressure_ft + (float(qnh) - 1013.25) * FT_PER_HPA

    def _on_a_centreline(self, lat: float, lon: float, heading: float):
        """(runway, lat, lon, course) on the centreline of the runway it is
        lined up with -- on it, or on its final -- when it is within
        LIVE_CENTRELINE_M of it; else None."""
        for runway, (lat0, lon0, course) in (
                getattr(self.field, "thresholds", None) or {}).items():
            if abs(livemod._wrap(heading - course)) > LIVE_ALIGNED_DEG:
                continue
            north = (lat - lat0) * M_PER_DEG
            east = (lon - lon0) * M_PER_DEG * math.cos(math.radians(lat0))
            rad = math.radians(course)
            along = north * math.cos(rad) + east * math.sin(rad)
            across = -north * math.sin(rad) + east * math.cos(rad)
            surface = (getattr(self.field, "surfaces", None)
                       or {}).get(runway)
            length_m = (surface[2] * 1852.0 if surface else LIVE_RUNWAY_M)
            if not (-LIVE_FINAL_M <= along <= length_m + 300.0):
                continue
            if abs(across) > LIVE_CENTRELINE_M:
                continue
            north_c, east_c = along * math.cos(rad), along * math.sin(rad)
            return (runway,
                    lat0 + north_c / M_PER_DEG,
                    lon0 + east_c / (M_PER_DEG * math.cos(math.radians(lat0))),
                    course)
        return None

    def _on_a_taxiway(self, lat: float, lon: float):
        """The nearest point of the nearest taxiway, within
        LIVE_TAXIWAY_M; None away from all of them."""
        layout = self.ground
        x, z = layout.to_xy(lat, lon)
        best, found = LIVE_TAXIWAY_M, None
        for a, others in layout.edges.items():
            ax, az = layout.xy[a]
            for b, _length, runway, _label in others:
                if runway:
                    continue
                bx, bz = layout.xy[b]
                dx, dz = bx - ax, bz - az
                span = dx * dx + dz * dz
                u = 0.0 if span <= 0 else max(0.0, min(
                    1.0, ((x - ax) * dx + (z - az) * dz) / span))
                px, pz = ax + u * dx, az + u * dz
                distance = math.hypot(x - px, z - pz)
                if distance < best:
                    best, found = distance, (px, pz)
        return layout.to_latlon(*found) if found is not None else None

    def _roll_in(self, one, now: float):
        """An arrival taxiing in at taxi speed within 150 m of a free stand
        ahead of it is rolled onto the stand at a walking pace and stays
        there (after LocalTC): the real one goes to a gate the scenery may
        not have where it is, and drawn on the grass beside the stand it
        would not be parked anywhere. Let go once the feed has heard it stop
        and then move again -- its next flight -- or after 45 minutes."""
        from dataclasses import replace

        held = self._live_stands.get(one.object_id)
        if held is None:
            if not (one.on_ground and one.hex in self._live_flew
                    and livemod.MOVING_KT <= one.ground_speed_kt
                    <= LIVE_TAXI_IN_KT):
                return None
            stand = self._stand_ahead(one)
            if stand is None:
                return None
            self.live_snapped[(one.callsign or "").upper()] = "rolled in"
            held = self._live_stands[one.object_id] = {
                "stand": stand, "since": now, "at": (one.latitude,
                                                     one.longitude),
                "heading": one.heading, "stopped": False}
            self._live_moving.discard(one.object_id)
            decision("live_traffic", event="rolled onto a stand",
                     callsign=one.callsign, stand=stand.index)
        stand = held["stand"]
        # Heard stopping, then moving again: its next flight. And a real
        # aeroplane that leaves the stand's reach without stopping was
        # never parking here -- it taxied past, or turned away: the drawn
        # copy is let go where it is rather than walked to a stand the
        # real one is not at, which for 45 minutes was the oddity.
        if one.ground_speed_kt < livemod.MOVING_KT:
            held["stopped"] = True
        elif (held["stopped"] or now - held["since"] > LIVE_PARKED_S
                or self._stand_left(held["stand"], one)):
            del self._live_stands[one.object_id]
            self._live_moving.add(one.object_id)
            return None
        layout = self.ground
        target = layout.to_latlon(stand.x, stand.z)
        lat, lon = held["at"]
        x, z = layout.to_xy(lat, lon)
        tx, tz = stand.x, stand.z
        left = math.hypot(tx - x, tz - z)
        step = LIVE_ROLL_MS * max(0.0, now - held.get("moved", now))
        held["moved"] = now
        if left <= step or left < 0.5:
            held["at"] = target
            return replace(one, latitude=target[0], longitude=target[1],
                           heading=stand.heading, ground_speed_kt=0.0,
                           altitude_ft=self.field.elevation_ft)
        x += (tx - x) / left * step
        z += (tz - z) / left * step
        held["at"] = layout.to_latlon(x, z)
        heading = math.degrees(math.atan2(tx - x, tz - z)) % 360.0
        return replace(one, latitude=held["at"][0], longitude=held["at"][1],
                       heading=heading,
                       ground_speed_kt=LIVE_ROLL_MS / 0.514444,
                       altitude_ft=self.field.elevation_ft)

    def _stand_left(self, stand, one) -> bool:
        """The real aeroplane is outside the stand's capture reach -- gone
        somewhere else rather than parking on it."""
        x, z = self.ground.to_xy(one.latitude, one.longitude)
        return math.hypot(stand.x - x, stand.z - z) > LIVE_STAND_M

    def _stand_ahead(self, one):
        """A free stand within LIVE_STAND_M, ahead of the aeroplane."""
        layout = self.ground
        x, z = layout.to_xy(one.latitude, one.longitude)
        airliner = livemod.category_of(one.callsign or "",
                                       one.type_code or "") != livemod.GA
        taken = {held["stand"].index for held in self._live_stands.values()}
        taken |= {f.stand for f in self.statics if f.stand is not None}
        occupied = [layout.to_xy(lat, lon) for lat, lon in self._parked_seen]
        best, found = LIVE_STAND_M, None
        for stand in layout.stands_for(airliner=airliner):
            if stand.index in taken:
                continue
            distance = math.hypot(stand.x - x, stand.z - z)
            if distance > best:
                continue
            bearing = math.degrees(math.atan2(stand.x - x, stand.z - z))
            if abs(livemod._wrap(bearing - one.heading)) > LIVE_AHEAD_DEG:
                continue
            if any(math.hypot(stand.x - ox, stand.z - oz) < stand.radius
                   for ox, oz in occupied):
                continue
            best, found = distance, stand
        return found

    # How close an observed aeroplane has to be to the pilot's own position
    # to be the pilot. Twenty-five metres: the next stand is forty away.
    USER_IS_HERE_M = 25.0

    def _is_user(self, one) -> bool:
        """Whether an observed aeroplane is the pilot's own.

        The simulator reports the user alongside the AI, and the name it
        reports them under is not always the one the session knows: the
        reader rebuilds a callsign from the airline and flight number, and
        a user aircraft with neither answers with its registration. So the
        pilot is also recognised by where they are, which no name can get
        wrong -- a parked aeroplane twenty-five metres from the pilot's
        reference point, on the ground as they are, is the pilot.
        """
        user = self.user
        if user is None or not user.has_position:
            return False
        if bool(one.on_ground) != bool(user.on_ground):
            return False
        apart_m = haversine_nm(user.latitude, user.longitude,
                               one.latitude, one.longitude) * 1852.0
        return apart_m <= self.USER_IS_HERE_M

    def _note_parked(self, standing, known: dict, now: float) -> None:
        """Keep the parked observed aeroplanes, and when each one stopped.

        One that had a strip a moment ago -- it taxied in and has just
        stopped -- is an arrival and its turnaround starts now; one found
        standing there has been there for some unknown while. An aeroplane
        on a runway is not parked whatever its speed.
        """
        here: dict[int, ParkedObserved] = {}
        for one in standing:
            if self._on_runway(one):
                continue
            kept = self._parked_observed.get(one.object_id)
            if kept is None:
                flight = known.get(one.object_id)
                if flight is not None:
                    # It had a strip. Stopped on a stand it has parked and
                    # the strip is finished; stopped anywhere else it is
                    # holding short, which is the aeroplane the tower is
                    # about to talk to, and it keeps its strip.
                    if not self._parked_for_good(flight, one):
                        continue
                    flight.done = True
                kept = ParkedObserved(
                    object_id=one.object_id,
                    callsign=(one.callsign or "").strip().upper().strip("@")
                             .strip(),
                    registration=getattr(one, "registration", "") or "",
                    type_code=observed_type(one.type_code),
                    title=getattr(one, "title", "") or "",
                    since=now if flight is not None else 0.0,
                    live=getattr(one, "source", "") == "live",
                    operator=(one.callsign[:3].upper() if livemod.category_of(
                        one.callsign or "", one.type_code or "") in (
                            livemod.AIRLINE, livemod.CARGO) else ""),
                    parked_at=now,
                    category=(getattr(one, "category", "") or
                              livemod.category_of(one.callsign or "",
                                                  one.type_code or "")))
            kept.latitude = one.latitude
            kept.longitude = one.longitude
            kept.heading = one.heading
            kept.altitude_ft = one.altitude_ft
            here[one.object_id] = kept
        # A live one the feed no longer hears is still at its gate: a parked
        # aeroplane's transponder goes off. It stays its 45 minutes, unless
        # the feed hears it moving again (_live_pose lets it go).
        heard = getattr(self, "_live_heard", set())
        for object_id, kept in self._parked_observed.items():
            if (kept.live and object_id not in here
                    and object_id not in heard
                    and object_id not in self._live_moving
                    and now - kept.parked_at < LIVE_PARKED_S):
                here[object_id] = kept
        self._parked_observed = here

    def _parked_for_good(self, flight: Flight, one) -> bool:
        """Whether a worked aeroplane that has stopped has parked.

        With the airport's stands to hand, standing on one is the answer.
        Without them, an arrival that has taxied in and stopped has parked
        and a departure that has stopped is holding somewhere.
        """
        if self.ground is not None:
            x, z = self.ground.to_xy(one.latitude, one.longitude)
            return self.ground.stand_at(x, z) is not None
        return flight.phase in ("taxi_in", "landed")

    @property
    def parked_observed(self) -> list[ParkedObserved]:
        """Somebody else's aeroplanes standing on this airport."""
        return list(self._parked_observed.values())

    def _works_here(self, one, known: bool = False) -> bool:
        """Whether an observed aeroplane is this airport's business.

        Three ways of not being. A busy region has hundreds of aircraft in it
        and almost all of them are somewhere else. The few that are overhead
        are at thirty thousand feet on their way past. And a big field under
        FSLTL has a hundred aeroplanes parked on it, which are not on anybody's
        frequency -- putting those on strips would sequence the user behind
        sixty aircraft that are not going anywhere.

        ``known`` is whether it already has a strip. One that has been worked
        keeps its strip when it stops, because an aeroplane holding short with
        the brakes on is exactly the aeroplane the tower is about to talk to.
        """
        if not getattr(one, "positioned", False):
            return False
        if not self.field.latitude and not self.field.longitude:
            return False
        distance = haversine_nm(self.field.latitude, self.field.longitude,
                                one.latitude, one.longitude)
        if distance > self.observed_nm:
            return False
        if not one.on_ground:
            return (one.altitude_ft - self.field.elevation_ft
                    ) <= OBSERVED_CEILING_FT
        if not known and distance > PARKED_RANGE_NM:
            # Moving on the ground this far out is taxiing at the next
            # airport, not on this one's taxiways: the feed hears much
            # further than a field's own ground is its business.
            return False
        return (known or one.ground_speed_kt >= TAXIING_KT
                or self._on_runway(one))

    def _observed_flight(self, one) -> Flight | None:
        """A strip for an aeroplane that has just come into view.

        The callsign is whatever the simulator calls it, which for FSLTL is
        the real flight's: that is the whole point, and it is why the operator
        and the language fall out of the same tables the invented traffic uses.
        An aeroplane with no callsign at all is skipped rather than given an
        invented one -- there is nothing to call it, and "unknown traffic" on
        a strip is worse than an aeroplane that is only on the map.
        """
        written = (one.callsign or "").strip().upper()
        if not written:
            return None
        type_code = observed_type(one.type_code)
        operator = by_icao(written[:3]) if (
            len(written) > 3 and written[:3].isalpha()
            and written[3].isdigit()) else None
        language = (operator.speaks_at(self.field.ident) if operator
                    else reply_language(self.field.ident, "en"))
        forms = every_language(written, type_code,
                               dialect=self.profile.dialect_for(self.field.ident))
        aircraft = Aircraft(forms["en"], type_code, wake_suffix(type_code),
                            callsigns=forms)
        # Reserved, so the invented traffic never hands the same number to a
        # second aeroplane while this one is on the field.
        self._used_callsigns.add(written)
        source = getattr(one, "source", "") or "simulator"
        category = (getattr(one, "category", "")
                    or livemod.category_of(written, type_code,
                                           operator=operator))
        final = self._final_for(one)
        runway = (final[1] if final else self._pick_runway(
            arriving=not one.on_ground, type_code=type_code))
        return Flight(
            written=written, type_code=type_code, aircraft=aircraft,
            language=language, operator=operator, arriving=not one.on_ground,
            ifr=True, observed=True, observed_id=one.object_id,
            runway=runway,
            phase="taxi", step=0, source=source, category=category,
            origin=getattr(one, "origin", "") or "",
            destination=getattr(one, "destination", "") or "",
            hex=getattr(one, "hex", "") or "",
            registration=getattr(one, "registration", "") or "",
            squawk=_squawk(getattr(one, "squawk", "")),
        )

    def _read_observed(self, flight: Flight, one, now: float) -> None:
        """Put what the simulator says onto the strip, and work out the phase."""
        flight.latitude = one.latitude
        flight.longitude = one.longitude
        flight.altitude_ft = one.altitude_ft
        flight.heading = one.heading
        flight.ground_speed_kt = one.ground_speed_kt
        flight.vertical_rate_fpm = one.vertical_speed_fpm
        flight.on_ground = one.on_ground

        to_run = self._final_distance(one)
        flight.distance_nm = to_run if to_run is not None else haversine_nm(
            self.field.latitude, self.field.longitude,
            one.latitude, one.longitude)

        # A live one is drawn 30 s behind the feed; what it is doing is read
        # off where the real one is now (its lead), so each thing it does is
        # said before it is seen to do it -- a takeoff clearance half a
        # minute before the roll. Held by our tower, it is doing nothing.
        lead = getattr(one, "lead", None) or one
        flight.lead_at = (lead.latitude, lead.longitude)
        lead_to_run = to_run if lead is one else self._final_distance(lead)
        flight.lead_distance_nm = (lead_to_run if lead_to_run is not None
                                   else flight.distance_nm)
        was = flight.phase
        if flight.live_held:
            return
        flight.phase = self._observed_phase(flight, lead, lead_to_run)
        flight.arriving = flight.phase in ("inbound", "final")
        if flight.phase != was:
            # A phase change is the only thing worth saying anything about, and
            # it is checked on the next pass rather than here so that the words
            # are composed on the situation loop like everybody else's.
            flight.step = 0
            flight.next_at = now
        if (under := self._runway_under(one)) and (
                flight.phase in ("takeoff", "final", "landed")
                or self._lined_up(one)):
            # The strip the aeroplane is actually on, not the one its strip
            # happened to be filed under: on a mixed-ops field the two are
            # not the same, and a booking on the wrong runway protects
            # nobody on the right one.
            self._hold_runway(under, now, 8.0)

    def _observed_phase(self, flight: Flight, one, to_run) -> str:
        """What an observed aeroplane is doing, from what it is doing.

        There is no plan to read and no instruction it was given, so this is
        entirely geometry and speed. It is deliberately coarse: the phase is
        used for the strip, for the frequency an aeroplane is worked on, and
        for the landing sequence, and none of those needs to know the
        difference between a base leg and a long final.
        """
        if one.on_ground:
            # An arrival first: it rolls down the runway at the same speed
            # a departure does, and the one thing that tells them apart is
            # that it was on final a moment ago.
            if flight.phase in ("final", "landed", "taxi_in"):
                return "landed" if one.ground_speed_kt >= ROLLING_KT else "taxi_in"
            if one.ground_speed_kt >= ROLLING_KT and self._on_runway(one):
                return "takeoff"
            if self._lined_up(one) and flight.phase in (
                    "taxi", "holding", "lineup", "pushback"):
                return "lineup"
            track = getattr(one, "track", None)
            if (track is not None and flight.phase in ("holding", "pushback")
                    and TAXIING_KT <= one.ground_speed_kt < PUSHBACK_KT
                    and abs((track - one.heading + 540.0) % 360.0 - 180.0)
                    > PUSHBACK_DEG):
                # Moving tail first off a stand: being pushed back.
                return "pushback"
            if one.ground_speed_kt >= TAXIING_KT:
                return "taxi"
            return "holding"
        if to_run is not None and to_run <= 12.0:
            return "final"
        climbing = one.vertical_speed_fpm > 300.0
        if climbing and flight.phase in ("takeoff", "departed", "taxi",
                                         "holding", "lineup"):
            return "departed"
        return "inbound"

    def _final_for(self, one):
        """``(distance to run, runway)`` on the final it is on, or None.

        A big field lands two strips at once, so the check runs over every
        arrival end in use and the answer is the one the aeroplane is most
        squarely behind -- the parallel three hundred metres off is a
        different approach, not this one's.
        """
        if one.on_ground:
            return None
        names = getattr(self.field, "arrival_runways", None) \
            or [self.field.arrival_runway or self.field.departure_runway]
        best = None
        for name in names:
            end = self._threshold(name)
            if end is None:
                continue
            along, across = offset_from_track(one.latitude, one.longitude,
                                              end[0], end[1], end[2])
            to_run = -along
            if not 0.0 < to_run <= 14.0 or abs(across) > 1.2:
                continue
            if abs((one.heading - end[2] + 540.0) % 360.0 - 180.0) > 30.0:
                continue
            if best is None or abs(across) < best[0]:
                best = (abs(across), to_run, name)
        return None if best is None else (best[1], best[2])

    def _final_distance(self, one) -> float | None:
        """How far an observed aeroplane has to run, if it is on final.

        The same test the user gets: lined up with a landing runway to
        within thirty degrees, inside the centreline, and closing. Anything
        else is an aeroplane near the airport rather than one landing at it.
        """
        found = self._final_for(one)
        return found[0] if found is not None else None

    def _lined_up(self, one) -> bool:
        """Whether an observed aeroplane is sitting on the runway, ready to go.

        The occupancy test used to read the phase, and the phase of a stopped
        aeroplane is "holding" whether it is holding at the point or holding
        on the pavement. So an airliner lined up and waiting was not counted,
        and the user was cleared for takeoff onto a runway with somebody
        already on it -- which is what "it ignores the traffic" looks like
        from the flight deck.

        Alignment is the discriminator, the same one the surface watch uses on
        the user: an aeroplane holding short is pointing across the runway and
        one lined up is pointing along it.
        """
        if not getattr(one, "on_ground", False):
            return False
        heading = getattr(one, "heading", None)
        if heading is None:
            return False
        for name in getattr(self.field, "thresholds", None) or {}:
            end = self._threshold(name)
            if end is None:
                continue
            along, across = offset_from_track(one.latitude, one.longitude,
                                              end[0], end[1], end[2])
            if not (abs(across) <= RUNWAY_WIDTH_NM
                    and -0.15 <= along <= RUNWAY_LENGTH_NM):
                continue
            apart = abs(((float(heading) - end[2]) + 180.0) % 360.0 - 180.0)
            if apart <= LINED_UP_DEG:
                return True
        return False

    def _runway_under(self, one) -> str:
        """The runway an observed aeroplane is on, or "".

        Every published end is tried, not just the ones in use: what the
        feed actually put there is the truth to book occupancy against,
        and an aeroplane on a strip our configuration did not pick is
        still an aeroplane on a strip.
        """
        for name in getattr(self.field, "thresholds", None) or {}:
            end = self._threshold(name)
            if end is None:
                continue
            along, across = offset_from_track(one.latitude, one.longitude,
                                              end[0], end[1], end[2])
            if (abs(across) <= RUNWAY_WIDTH_NM
                    and -0.15 <= along <= RUNWAY_LENGTH_NM):
                return name
        return ""

    def _on_runway(self, one) -> bool:
        """Whether an observed aeroplane is on any runway."""
        return bool(self._runway_under(one))

    def _on_the_strip(self, one):
        """(runway, lat, lon, along_nm, length_nm) when it is on a
        runway's centreline and on the pavement; else None.

        Where :meth:`_on_runway` is deliberately generous -- an aeroplane
        on a strip our configuration did not pick is still an aeroplane on
        a strip -- adopting is the narrower question: ours is drawn on the
        centreline and rolled down it, so a read off the edge is not an
        aeroplane to adopt on it.
        """
        if not one.on_ground or one.heading is None:
            return None
        centreline = self._on_a_centreline(
            one.latitude, one.longitude, one.heading)
        if centreline is None:
            return None
        name, lat, lon, _course = centreline
        end = self._threshold(name)
        if end is None:
            return None
        along, _across = offset_from_track(
            lat, lon, end[0], end[1], end[2])
        surface = (getattr(self.field, "surfaces", None) or {}).get(name)
        length_nm = surface[2] if surface else RUNWAY_LENGTH_NM
        if not (0.0 <= along <= length_nm):
            return None
        return name, lat, lon, along, length_nm

    def _room_to_roll(self, one, flight: Flight, ahead_nm: float) -> bool:
        """Whether the runway left ahead of it can still take the roll.

        From its own speed to rotation at the type's takeoff push, plus
        the beat the wheels stay on the ground after the nose comes up --
        the same figures the model rolls with, so a strip too short for
        the one is too short for the other.
        """
        performance = perftable.traffic_performance(
            flight.type_code,
            light=livemod.flight_category(flight) == livemod.GA)
        rotate = performance.rotate_kt or 120.0
        accel = performance.takeoff_kt_s or 3.0
        gs = min(max(0.0, one.ground_speed_kt or 0.0), rotate)
        t_s = (rotate - gs) / accel
        roll_nm = (gs * t_s + 0.5 * accel * t_s * t_s
                   + rotate * LIFTOFF_AFTER_S) / 3600.0
        return roll_nm <= ahead_nm

    def _advance_observed(self, flight: Flight, now: float) -> None:
        """What the controller says to an aeroplane it cannot instruct.

        Four moments, and every one of them is a description rather than an
        order: the aeroplane is going to do this whatever is said, so the only
        way to be wrong is to say something it is not about to do. Each is said
        once per phase -- ``step`` is set by whichever branch actually speaks,
        so a phase that is not ready to be talked about yet is looked at again
        on the next pass rather than being used up.

        The landing clearance is the one with a condition on it. A controller
        with somebody on the runway would send an arrival around, and this one
        will not go around -- so nothing is said at all rather than a clearance
        that is wrong. Silence on a busy frequency is normal; a landing
        clearance over an occupied runway is not.
        """
        flight.next_at = now + 3.0
        if flight.live_held:
            return
        # A live departure's cut taxi: the rest when it is drawn there.
        self._rest_of_taxi(flight, now, inbound=False)
        if flight.step:
            return
        phase = flight.phase
        runway_out = flight.runway or self.field.departure_runway

        # The departure, as it happens -- for a live one half a minute
        # before it is seen to happen: its clearance and its push, its taxi
        # on the airport's own taxiways, its line-up and its takeoff. Nothing
        # it does is unannounced, and nothing announced is not then done,
        # because each is read off what the real one has just done.
        if phase == "pushback":
            flight.step = 1
            if ("DEL" in self.field.stations and flight.destination
                    and not flight.cleared):
                flight.cleared = True
                self._exchange(
                    flight, now, "DEL",
                    pilot_first=lambda p, a, s: p.request_clearance(
                        a, s, flight.destination,
                        atis_letter=self._atis_letter(flight.language),
                        type_name=self._type_of(flight)),
                    controller=lambda c, a, s: c.ifr_clearance(
                        a, s, flight.destination,
                        initial_altitude=INITIAL_CLIMB_FT,
                        squawk=flight.squawk,
                        departure_freq=self._freq("DEP")),
                    pilot_back=lambda p, a, s: p.clearance(
                        a, flight.destination, flight.squawk,
                        INITIAL_CLIMB_FT),
                )
            self._exchange(
                flight, now, "GND" if "GND" in self.field.stations else "TWR",
                controller=lambda c, a, s: c.pushback_approved(a),
                pilot_back=lambda p, a, s: p.wilco(a),
            )
            return

        if phase == "taxi" and not flight.arriving:
            flight.step = 1
            route: list[str] = []
            hold_at = ""
            if self.ground is not None:
                planned = self._plan_departure(flight)
                if planned is not None:
                    route, hold_at, flight.taxi_rest, flight.taxi_split_xy = \
                        _said_route(self.ground, planned[0], self.field.ident,
                                    inbound=False)
            self._exchange(
                flight, now, "GND" if "GND" in self.field.stations else "TWR",
                controller=lambda c, a, s: c.taxi_to_runway(
                    a, s, runway_out, route, hold_short_taxiway=hold_at),
                pilot_back=lambda p, a, s: p.taxi(
                    a, runway_out, route=_spell_route(route),
                    hold_short_taxiway=_spell_route([hold_at]) if hold_at
                    else ""),
            )
            return

        if phase == "lineup":
            # The same rule the landing clearance lives by: with somebody
            # else on the strip a "line up and wait" is a clearance that is
            # wrong, so nothing is said. ``ignore=flight`` -- it is the one
            # on the runway being described.
            if self.runway_occupied(now, ignore=flight, runway=runway_out):
                return
            flight.step = 1
            occupier = self._runway_occupier(flight)
            reason = self._lineup_reason(occupier, flight.language)
            self._exchange(
                flight, now, "TWR",
                controller=lambda c, a, s: c.line_up_and_wait(
                    a, runway_out, reason=reason, station=s,
                    intersection=self._said_intersection(flight)),
                pilot_back=lambda p, a, s: p.line_up(
                    a, runway_out,
                    intersection=self._said_intersection(flight)),
            )
            return

        if phase == "takeoff":
            if self.runway_occupied(now, ignore=flight, runway=runway_out):
                return
            flight.step = 1
            self._exchange(
                flight, now, "TWR",
                controller=lambda c, a, s: c.cleared_for_takeoff(
                    a, runway_out, self.field.weather, station=s,
                    intersection=self._said_intersection(flight),
                    remaining_m=flight.remaining_m),
                pilot_back=lambda p, a, s: p.takeoff(
                    a, runway_out,
                    intersection=self._said_intersection(flight)),
            )
            return

        if phase == "inbound":
            flight.step = 1
            self._exchange(
                flight, now, "APP",
                pilot_first=lambda p, a, s: p.checking_in(
                    a, s, flight.altitude_ft, climbing=False,
                    atis_letter=self._atis_letter(flight.language),
                    type_name=self._type_of(flight)),
                controller=lambda c, a, s: c.radar_contact(a, s),
            )
            return

        if phase == "final":
            if flight.lead_distance_nm > LANDING_CLEARANCE_NM:
                return                     # not yet close enough to clear
            runway = flight.runway or self.field.arrival_runway
            if self.runway_occupied(now, runway=runway):
                return                     # and not while somebody is on it
            flight.step = 1
            self._exchange(
                flight, now, "TWR",
                controller=lambda c, a, s: c.cleared_to_land(
                    a, runway, self.field.weather),
                pilot_back=lambda p, a, s: p.land(a, runway),
            )
            return

        if phase == "departed":
            if flight.handed_off:
                # One of ours given back to the feed on its way out:
                # already sent to the next frequency, once.
                flight.step = 1
                return
            station, mhz = self._next_station("DEP", flight.language)
            if not station:
                return
            flight.step = 1
            self._exchange(
                flight, now, "TWR",
                controller=lambda c, a, s: c.handoff(a, station, mhz),
                pilot_back=lambda p, a, s: p.contact(a, station, mhz),
            )
            flight.handed_off = True
            return

        if phase == "taxi_in":
            if flight.handed_off:
                flight.step = 1
                return
            station, mhz = self._next_station("GND", flight.language)
            if not station:
                return
            flight.step = 1
            self._exchange(
                flight, now, "TWR",
                controller=lambda c, a, s: c.handoff(
                    a, station, mhz, when=c.when_able()),
                pilot_back=lambda p, a, s: p.contact(a, station, mhz),
            )
            flight.handed_off = True
            if flight.source == "live":
                # And where it goes, which it then does: to the ramp, the
                # stand being the one it rolls onto.
                self._exchange(
                    flight, now, "GND",
                    controller=lambda c, a, s: c.taxi_to_parking(a, s, []),
                    pilot_back=lambda p, a, s: p.leaving(a),
                )
            return

    def _protect_the_pilot(self, now: float) -> None:
        """The pilot's runway, against the real aeroplanes (occupancy):

        - with the pilot on the runway, a live arrival on short final for it
          is sent around -- taken over and flown round by us, the real one's
          positions ignored from then on;
        - with the pilot on the runway or on a final inside 4 nm, a live
          departure about to go onto it is held short, and let go once the
          pilot is clear, one at a time, 90 s apart;
        - on the ground, a live aeroplane taxiing at the pilot stops and
          gives way.

        Said on the tower's frequency, where the pilot hears it only when
        tuned there, and a hold short at most once a minute. After LocalTC's
        traffic manager, which does the same three things.
        """
        hold = self.hold_live
        on_runway = self._user_on_runway()
        to_run = self._user_on_final()
        busy = on_runway or (to_run is not None and to_run <= PILOT_FINAL_NM)
        for flight in list(self.flights):
            if flight.done or not flight.observed or flight.source != "live":
                continue
            # Arrivals: one about to land on top of the pilot goes around.
            if (flight.phase == "final" and not flight.on_ground
                    and flight.distance_nm <= LIVE_GO_AROUND_NM
                    and (on_runway or (to_run is not None
                                       and flight.distance_nm - to_run
                                       < LIVE_BEHIND_NM
                                       and flight.distance_nm > to_run))):
                self._take_over_for_a_go_around(flight, now)
                continue
            if hold is None:
                continue
            # Departures: kept off the pilot's runway, and held before the
            # runway rather than once the nose is on it -- the observed
            # "lineup" phase only exists once it is there, so a taxiing
            # departure that has reached the strip's edge is stopped while
            # the pilot is on it or turning inside its final. The strip is
            # only marked held when the drawing actually stopped -- a
            # flight the feed has already lost keeps moving whatever this
            # says, and a frozen strip about a moving aeroplane is the
            # worse lie.
            if (flight.live_held != "give way"
                    and (flight.phase in ("ready", "waiting", "lineup",
                                          "takeoff")
                         or (flight.phase in ("pushback", "taxi")
                             and self._near_departure_runway(flight)))
                    and not flight.live_held and busy
                    and not self._on_runway(flight)
                    and hold(flight.hex, True)):
                flight.live_held = "runway"
                decision("live_traffic", event="held short",
                         callsign=flight.written,
                         why="the pilot is on the runway or its final")
                if now - self._hold_short_said >= HOLD_SHORT_EVERY_S:
                    self._hold_short_said = now
                    self._exchange(
                        flight, now, "TWR",
                        controller=lambda c, a, s, f=flight:
                        c.hold_short_instruction(
                            a, f.runway or self.field.departure_runway),
                        pilot_back=lambda p, a, s: p.wilco(a),
                    )
            # On the ground: give way to the pilot taxiing.
            ahead = self._pilot_ahead(flight, now)
            if ahead and not flight.live_held and hold(flight.hex, True):
                flight.live_held = "give way"
            elif flight.live_held == "give way" and not ahead:
                flight.live_held = ""
                hold(flight.hex, False)
        # Let the held departures go once the pilot is clear: one at a time.
        if hold is not None and not busy \
                and now - self._last_release >= RELEASE_GAP_S:
            waiting = [f for f in self.flights
                       if f.live_held == "runway" and not f.done]
            if waiting:
                first = waiting[0]
                first.live_held = ""
                first.step = 0                 # its line-up is said now
                hold(first.hex, False)
                self._last_release = now
                decision("live_traffic", event="released",
                         callsign=first.written, why="the pilot is clear")

    # The last bit of taxiway before the strip -- where an observed
    # departure can still be stopped short of it.
    NEAR_DEPARTURE_RUNWAY_NM = 0.15

    def _near_departure_runway(self, flight: Flight) -> bool:
        """Whether an observed departure's taxi has brought it to the
        strip's edge -- the last moment holding it still keeps it off."""
        end = self._threshold(
            flight.runway or self.field.departure_runway)
        if end is None:
            return False
        along, across = offset_from_track(
            flight.latitude, flight.longitude, end[0], end[1], end[2])
        near = self.NEAR_DEPARTURE_RUNWAY_NM
        return (abs(across) <= RUNWAY_WIDTH_NM + near
                and -0.15 - near <= along <= RUNWAY_LENGTH_NM + near)

    def _release_held(self, flight) -> None:
        """A live aeroplane frozen for the pilot is let go as its strip
        goes: the roster keeps emitting the held pose for as long as the
        feed reports it, so an unreleased hold draws a statue forever."""
        if getattr(flight, "live_held", "") and self.hold_live is not None:
            self.hold_live(flight.hex, False)
            flight.live_held = ""

    def _release_all_held(self) -> None:
        """Every live aeroplane this injector froze, let go -- the roster
        outlives the strips, so a clear that skips this leaves statues."""
        for flight in self.flights:
            self._release_held(flight)

    def _rejoin_the_feed(self, seen, now: float) -> list:
        """The live arrivals we sent around and flew, given back to the feed
        once ours is parked (the pilot's rule, 30 Oct 2026): not before --
        the real one landed half a circuit ahead of ours -- and not never.
        Ours goes the moment the feed hears the real one; a real one the
        feed does not hear (a transponder off at the gate) leaves ours on
        the stand, as one of the parked, for LIVE_PARKED_S. Returns ``seen``
        without the feed's copies still being ignored."""
        for object_id, ours in list(self._flown_for.items()):
            if object_id in self._parked_since:
                continue
            if ours.done or ours.static or ours.parked_at:
                self._parked_since[object_id] = now
            elif (ours is None
                  or (all(f is not ours for f in self.flights)
                      and all(f is not ours for f in self.statics)
                      and all(f is not ours for f in self._materialising))):
                # Ours was dropped before it ever parked -- a field change
                # or the invented switch took it whole. The suppression
                # dies with it: held forever, the real aeroplane is
                # filtered out of this feed for the rest of the session.
                self._taken_over.discard(object_id)
                self._flown_for.pop(object_id, None)
        heard = {one.object_id for one in seen
                 if getattr(one, "source", "") == "live"}
        for object_id, since in list(self._parked_since.items()):
            ours = self._flown_for.get(object_id)
            if object_id in heard:
                if ours is not None and not (
                        ours.static or ours.parked_at or ours.done):
                    # Ours parked once, was boarded and is moving again: a
                    # real aeroplane heard on the feed does not put down a
                    # live departure of ours -- the suppression stands
                    # until ours is done or parked again.
                    continue
                if ours is not None:
                    ours.done = True
                    self._used_callsigns.discard(ours.written)
                    if ours in self.statics:
                        self.statics.remove(ours)
                    decision("live_traffic", event="rejoined the feed",
                             callsign=ours.written,
                             why="ours is parked and the real one is heard")
                self._taken_over.discard(object_id)
            elif now - since > LIVE_PARKED_S:
                # Never heard again: ours stays, an ordinary parked one.
                self._taken_over.discard(object_id)
            else:
                continue
            self._flown_for.pop(object_id, None)
            self._parked_since.pop(object_id, None)
        return [one for one in seen if one.object_id not in self._taken_over]

    def _pilot_ahead(self, flight: Flight, now: float) -> bool:
        """The pilot on the ground, in a live one's path -- now or soon."""
        user = self.user
        if (user is None or not user.has_position or not user.on_ground
                or not flight.on_ground
                or flight.ground_speed_kt < TAXIING_KT
                and flight.live_held != "give way"):
            flight._nose_to_nose_at = 0.0
            return False
        distance_m = haversine_nm(flight.latitude, flight.longitude,
                                  user.latitude, user.longitude) * 1852.0
        bearing = bearing_deg(flight.latitude, flight.longitude,
                              user.latitude, user.longitude)
        ahead = abs((bearing - flight.heading + 540.0) % 360.0
                    - 180.0) <= GIVE_WAY_DEG
        # Nose-to-nose on the same lane is a conflict long before the
        # noses are there: a head-on pilot is yielded to from well ahead
        # of the meeting point, not once the noses have met.
        head_on = abs((flight.heading - user.heading + 540.0) % 360.0
                      - 180.0) > GIVE_WAY_HEAD_ON_DEG
        reach = GIVE_WAY_HEAD_ON_M if head_on else GIVE_WAY_M
        if getattr(flight, "_unstuck", False):
            # It deadlocked with this pilot once already: released to keep
            # right and pass, it is not frozen a second time in the same
            # lane -- only clear geometry re-arms the yield.
            if distance_m <= reach and ahead:
                return False
            flight._unstuck = False
        if not ahead or distance_m > reach:
            flight._nose_to_nose_at = 0.0
            return False
        if (head_on and distance_m <= GIVE_WAY_STUCK_M
                and flight.ground_speed_kt < TAXIING_KT
                and user.ground_speed_kt < TAXIING_KT):
            # Already nose to nose and nobody moving: a hold here is the
            # conflict, not its prevention.
            if not getattr(flight, "_nose_to_nose_at", 0.0):
                flight._nose_to_nose_at = now
            elif now - flight._nose_to_nose_at >= GIVE_WAY_STUCK_S:
                flight._unstuck = True
                flight._nose_to_nose_at = 0.0
                return False
        else:
            flight._nose_to_nose_at = 0.0
        return True

    def _take_over_for_a_go_around(self, flight: Flight, now: float) -> None:
        """A live arrival sent around: the real one will land, so from here
        it is ours, flown round by the flight model like any invented one,
        and the feed's copy of it is ignored -- until ours has landed and
        is parked, where the real one already is (_rejoin_the_feed)."""
        self._taken_over.add(flight.observed_id)
        self._adopted_live.add(flight.observed_id)
        self._flown_for[flight.observed_id] = flight
        self._live_stands.pop(flight.observed_id, None)
        self._release_held(flight)
        flight.observed = False
        flight.source = "taken over"
        flight.arriving = True
        final = self._final_for(flight)
        flight.runway = (final[1] if final else flight.runway
                         or self.field.arrival_runway)
        flight.performance = perftable.traffic_performance(
            flight.type_code, light=livemod.flight_category(flight)
            == livemod.GA)
        flight.airborne_model = True
        decision("live_traffic", event="sent around", callsign=flight.written,
                 why="the pilot is on the runway", to_run_nm=round(
                     flight.distance_nm, 1))
        reason = runway_traffic(flight.language)
        self._exchange(
            flight, now, "TWR",
            controller=lambda c, a, s: c.go_around(a, reason=reason),
            pilot_back=lambda p, a, s: p.going_around(a),
            command=("go_around", {}),
            urgent=True,
        )

    def _live_arriving(self, flight: Flight, one) -> bool | None:
        """Whether a live aeroplane is this field's arrival or its
        departure: the route the feed looked up first, what it is doing
        second. None airborne and just passing -- an overflight is not
        adopted, however near it comes."""
        ident = self.field.ident
        if (flight.origin or "").upper() == ident:
            return False
        if (flight.destination or "").upper() == ident:
            return True
        if flight.destination:
            # Bound for somewhere the feed knows: a transit whose
            # descent happens to read like this final is still the
            # other field's arrival -- the route outranks the
            # coincidence of shape, or every Porto approach over Vigo
            # would be worked to a ramp it never filed for. The wheels'
            # word is no stronger: the feed calls a low pass on_ground
            # often enough that the same transit, hex already seen
            # flying, was taxied to a ramp at a field it overflew.
            return None
        if one.on_ground:
            # Taxiing in or taxiing out: the one it flew here for first.
            return one.hex in self._live_flew
        if self._final_distance(one) is not None:
            return True                     # lined up and closing
        if (one.altitude_ft
                > self.field.elevation_ft + self.OUT_OF_SIGHT_FT
                or haversine_nm(self.field.latitude, self.field.longitude,
                                one.latitude, one.longitude)
                > self.radius_nm):
            # Already past where a worked departure is kept: adopted
            # anyway, it would be handed straight back at its next
            # word -- nobody's to fly it.
            return None
        if (one.vertical_speed_fpm > 300.0
                and flight.phase in ("taxi", "holding", "lineup",
                                     "takeoff", "departed")):
            return False                    # climbing out of the field
        return None

    def _adopt_live(self, flight: Flight, one, now: float) -> bool:
        """Take a live aeroplane off the feed and fly it like an invented
        one (traffic.live.managed).

        The callsign, type, route and where it is are the real flight's;
        what it does from here is the tower's to say. The feed's copy of it
        is suppressed the way a sent-around one's is (``_taken_over``), and
        handed back when ours is parked and the real one is heard again
        (:meth:`_rejoin_the_feed`).

        False for the ones that are nobody's to fly from here: a parked
        aeroplane (adopted when the real one starts moving), an overflight
        just passing, anything on the ground at a field whose taxiways
        the simulator has not given -- with no layout a mid-taxi adoption
        would have nowhere to drive it -- and one already flown once.
        """
        if one.object_id in self._adopted_live:
            # Adopted once is the feed's for the rest of the session.
            # Handed back, its strip is read off the feed like any
            # observed one's; adopted again it would be worked and
            # handed off a second time, and a third, for as long as the
            # real aeroplane stays in reach.
            return False
        arriving = self._live_arriving(flight, one)
        if arriving is None:
            return False
        strip = self._on_the_strip(one)
        layout = self.ground
        stand = None
        taxi = None
        if one.on_ground and strip is None:
            if self._on_runway(one):
                # On the runway's wide box but not on a centreline:
                # crossing the strip, or far enough to its side that
                # what it is really on -- a parallel taxiway -- this
                # read cannot tell apart. Nobody's to fly it from here;
                # the feed's own drawing stays, where an adopted one
                # rolled the runway's heading off whatever it was on.
                return False
            if layout is None:
                return False
            x, z = layout.to_xy(one.latitude, one.longitude)
            stand = layout.stand_at(x, z)
            if one.ground_speed_kt < TAXIING_KT:
                # Standing still is nobody's to fly: parked on a stand or
                # stopped in a queue, it is adopted when the real one
                # moves. Adopted while parked, an arrival that had
                # already reached its stand was taxied on to a different
                # one, and a dormant aeroplane departed and re-adopted
                # every time ours finished -- the same real callsign
                # forever leaving.
                return False
            if arriving:
                stand = self._free_stand(
                    airliner=flight.operator is not None or flight.ifr,
                    gate=self._wants_gate(flight))
                taxi = stand is not None and layout.to_stand(
                    x, z, stand,
                    closed=self._closed_runway_edges(
                        flight.runway or self.field.arrival_runway))
                if not taxi:
                    return False            # nowhere to send it
        if (strip is not None and not arriving
                and not self._room_to_roll(one, flight,
                                           strip[4] - strip[3])):
            # This far down the strip the real one was taxiing it or
            # already most of the way down its roll: adopted anyway,
            # ours runs the whole roll off what is left and out over
            # the far end, where the feed's picture of it stays right.
            return False
        # Committed. Ours now, the feed's copy suppressed.
        self._taken_over.add(flight.observed_id)
        self._adopted_live.add(flight.observed_id)
        self._flown_for[flight.observed_id] = flight
        self._live_stands.pop(flight.observed_id, None)
        self._release_held(flight)
        flight.observed = False
        flight.step = 0
        flight.next_at = now + 1.0
        flight.commands.clear()
        flight.doing = None
        flight.lead_at = None
        flight.lead_distance_nm = 0.0
        flight.live_held = ""
        flight.performance = perftable.traffic_performance(
            flight.type_code,
            light=livemod.flight_category(flight) == livemod.GA)
        flight.latitude = one.latitude
        flight.longitude = one.longitude
        flight.heading = one.heading
        flight.altitude_ft = one.altitude_ft
        flight.ground_speed_kt = one.ground_speed_kt
        flight.vertical_rate_fpm = one.vertical_speed_fpm
        flight.arriving = arriving
        final = self._final_for(one) if arriving and not one.on_ground \
            else None
        flight.runway = (final[1] if final
                         else self._pick_runway(arriving, flight.type_code))
        if not one.on_ground:
            flight.airborne_model = True
            flight.on_ground = False
            if arriving:
                to_run = self._final_distance(one)
                end = self._threshold(flight.runway)
                flight.distance_nm = (
                    to_run if to_run is not None
                    else haversine_nm(flight.latitude, flight.longitude,
                                      end[0], end[1]) if end else 0.0)
                flight.phase = "final" if to_run is not None else "inbound"
            else:
                flight.phase = "departed"
                flight.live_airborne = True
                flight.distance_nm = haversine_nm(
                    self.field.latitude, self.field.longitude,
                    flight.latitude, flight.longitude)
        elif strip is not None:
            runway, lat, lon, _along, _length = strip
            # The strip it is actually on, not the one the
            # configuration picked -- and on its centreline, the way
            # _live_pose draws it, rather than at the receiver's
            # hundred-foot read off to the side.
            flight.runway = runway
            flight.latitude, flight.longitude = lat, lon
            flight.airborne_model = True
            flight.on_ground = True
            if arriving:
                flight.phase = "landed"
            else:
                flight.phase = ("takeoff" if one.ground_speed_kt
                                >= ROLLING_KT else "lineup")
        else:
            flight.on_layout = True
            if arriving:
                flight.stand = stand.index if stand is not None else None
                flight.planned_stand = None
                flight.taxi = taxi
                flight.phase = "taxi_in"
            else:
                flight.stand = stand.index if stand is not None else None
                flight.phase = "taxi"
        if not arriving and not flight.destination:
            if flight.ifr:
                flight.destination, flight.destination_icao = self._route_end(
                    flight.operator, flight.type_code,
                    getattr(flight.performance, "range_nm", 0.0) or 0.0)
            else:
                flight.destination = (self._nearby_field()
                                      or self._elsewhere())
        decision("live_traffic", event="adopted", callsign=flight.written,
                 phase=flight.phase,
                 arriving=flight.arriving,
                 why="traffic.live.managed")
        return True

    def _unadopt(self, flight: Flight, now: float,
                 why: str = "traffic.live.managed off") -> None:
        """Hand an adopted live aeroplane back to the feed: the managed
        switch gone off, or ours flown out of sight, it is read rather
        than flown again, and drawn wherever the real one now is -- the
        same snap a fed one takes when it first appears."""
        object_id = flight.observed_id
        self._taken_over.discard(object_id)
        self._flown_for.pop(object_id, None)
        self._parked_since.pop(object_id, None)
        self._live_stands.pop(object_id, None)
        self._release_all(flight)
        flight.observed = True
        flight.airborne_model = False
        flight.on_layout = False
        flight.taxi = None
        flight.line_up = None
        flight.planned = None
        flight.stand = None
        flight.commands.clear()
        flight.doing = None
        flight.assigned_heading = None
        flight.assigned_altitude_ft = None
        flight.assigned_speed_kt = None
        flight.target.clear()
        flight.extending = False
        flight.live_airborne = False
        flight.held_since = 0.0
        flight.step = 0
        flight.next_at = 0.0
        # Nothing we said to it is still its to answer: the exchanges half
        # queued go with the strip, not onto the frequency.
        self.pending = [c for c in self.pending if c.flight is not flight]
        if flight.static:
            # A parked one becomes somebody else's parked one again: the
            # feed's copy of the real aeroplane stands there instead.
            flight.static = False
            flight.done = True
            if flight in self.statics:
                self.statics.remove(flight)
        decision("live_traffic", event="back to the feed",
                 callsign=flight.written, why=why)

    def set_user(self, user: UserAircraft | None) -> None:
        """Tell the traffic where the person flying actually is.

        Without this the AI works a field with one aeroplane missing from it:
        the one that matters. With it, the runway the user is sitting on is an
        occupied runway, and an aeroplane on short final behind them is sent
        around by a controller that can see both.
        """
        self.user = user

    def _strip_ends(self, runway: str) -> list[str]:
        """The ends on the same pavement as ``runway``, as threshold names.

        "09L" and "27R" share one strip's occupancy; "09R" does not, which
        is what makes one parallel's landing not block the other's.
        """
        key = _runway_key(runway)
        ends = [name
                for name in (getattr(self.field, "thresholds", None) or {})
                if _runway_key(name) == key]
        return ends or ([runway] if runway else [])

    def _user_on_runway(self, runway: str = "") -> bool:
        """Whether the person flying is on a runway in use -- or on the
        strip ``runway`` names, when a particular one is being asked about."""
        user = self.user
        if user is None or not user.on_ground or not user.has_position:
            return False
        names = self._strip_ends(runway) if runway else self._in_use()
        for name in names:
            end = self._threshold(name)
            if end is None:
                continue
            along, across = offset_from_track(user.latitude, user.longitude,
                                              end[0], end[1], end[2])
            # The pavement, not the safety strip: ninety metres off the
            # centreline counted the pilot holding short as on the runway.
            if (abs(across) <= self.ON_RUNWAY_NM
                    and -0.15 <= along <= RUNWAY_LENGTH_NM):
                return True
        return False

    def _user_on_final(self) -> float | None:
        """How far the person flying has to run, if they are on final.

        Airborne, lined up with the runway to within thirty degrees, inside
        the centreline and closing. Anything else is an aeroplane in the
        circuit or somewhere else entirely, and not a reason to hold anybody.
        """
        user = self.user
        if user is None or user.on_ground or not user.has_position:
            return None
        best = None
        names = getattr(self.field, "arrival_runways", None) \
            or [self.field.arrival_runway or self.field.departure_runway]
        for name in names:
            end = self._threshold(name)
            if end is None:
                continue
            along, across = offset_from_track(user.latitude, user.longitude,
                                              end[0], end[1], end[2])
            to_run = -along
            if not 0.0 < to_run <= 12.0 or abs(across) > 1.0:
                continue
            if abs((user.heading - end[2] + 540.0) % 360.0 - 180.0) > 30.0:
                continue
            best = to_run if best is None else min(best, to_run)
        return best

    def runway_occupied(self, now: float, ignore: Flight | None = None,
                        runway: str = "") -> bool:
        """Whether anybody at all is on the runway -- the user included.

        :meth:`runway_is_busy` deliberately does not count the user, because
        the controller asks it whether the user may be cleared. This one is
        what the traffic asks before it rolls.

        ``runway`` names the strip being cleared: its own ends are what
        count, so an aeroplane on the parallel does not block this one the
        way two independently worked runways should not block each other.
        Without one, every end in use counts, as it always has.

        On an airport whose taxiways are driven, an aeroplane is on the
        runway because it is on it -- still rolling out, crossing, turning
        off -- and not only while a timer says so.
        """
        return (self.runway_is_busy(now, runway)
                or self._user_on_runway(runway)
                or self._somebody_on_runway(ignore, runway))

    def _somebody_on_runway(self, ignore: Flight | None = None,
                            runway: str = "") -> bool:
        names = self._strip_ends(runway) if runway else self._in_use()
        ends = [self._threshold(name) for name in names]
        ends = [end for end in ends if end is not None]
        if not ends:
            return False
        for other in self.flights:
            if other is ignore or other.done or not other.positioned:
                continue
            if not other.on_ground or not other.on_layout:
                continue
            if other.phase in ("lineup",) and other.taxi is None:
                continue          # counted as lined up, not as in the way
            for end in ends:
                along, across = offset_from_track(
                    other.latitude, other.longitude, end[0], end[1], end[2])
                # The runway itself, not the strip around it: an aeroplane
                # at the holding point is forty-five metres off the
                # centreline and counting it sent every arrival round.
                if abs(across) <= self.ON_RUNWAY_NM and \
                        -0.15 <= along <= RUNWAY_LENGTH_NM:
                    return True
        return False

    # Thirty metres: half a wide runway and a little more.
    ON_RUNWAY_NM = 30.0 / 1852.0

