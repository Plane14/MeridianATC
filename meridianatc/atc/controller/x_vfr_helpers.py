from __future__ import annotations

from ._shared import (
    Any,
    Instruction,
    Phase,
    Reply,
    _airborne,
    bearing_deg,
    decision,
    haversine_nm,
    math,
    normalize_runway,
    re,
    squawk_for,
    station_key,
)


class _Vfr_helpers:
    # ------------------------------------------------------------------
    # VFR helpers
    # ------------------------------------------------------------------

    def zone_weather(self, airport, weather):
        """The field's weather against its region's VFR rules
        (:mod:`meridianatc.atc.svfr`), or None where they are not applied:
        switched off (``immersion.controller.special_vfr``), no field, or no
        weather."""
        if not self.realism.special_vfr or airport is None or weather is None:
            return None
        from ..svfr import conditions

        return conditions(weather, self.region_for(airport.ident),
                          airport.lat, airport.lon)

    def _below_vmc(self, session, station, weather, p, airport, *,
                   asked: str, outside: bool = False,
                   offer: bool = True) -> Reply | None:
        """The answer to a plain VFR request into weather below the zone's
        VFR minima, or None when the weather allows it.

        Special VFR is offered where it may be given (``offer``, and only
        when the minima for it are met); an aeroplane outside the zone is
        kept out of it (``outside``).
        """
        if session.ifr or session.special_vfr:
            return None
        rules = self.zone_weather(airport, weather)
        if rules is None or rules.vmc:
            return None
        decision("vmc", airport=airport.ident, asked=asked,
                 said="below_minima", **rules.as_log())
        text = p.vfr_below_minima(
            session.aircraft,
            self._station_name(station, p)
            if not session.established_with(station) else "",
            offer=offer and rules.svfr,
            airspace=(p.airspace(self._airspace_class(airport, station))
                      if outside else ""))
        instruction = session.remember(
            Instruction("below_minima", station_key(station), text, {},
                        readback_required=False))
        return Reply(text, station, instruction=instruction)

    def _special_vfr(self, parsed, session, station, weather, p, state) -> Reply:
        """Special VFR, asked for: out of the zone from the ground, into it
        from the air.

        Refused below the Special VFR minima, and at night only where a
        source says the region refuses it (:mod:`meridianatc.atc.svfr`). Above VFR minima nobody
        needs it, and the ordinary release or join is given instead. The
        route is by the field's Special VFR points (kind "svfr" in
        ``vrps.csv``) where it has them.
        """
        from ..svfr import svfr_altitude

        aircraft = session.aircraft
        airport = self.navdb.airport(station.ident)
        airborne = _airborne(state)
        session.ifr = False
        rules = self.zone_weather(airport, weather)
        if rules is not None:
            decision("special_vfr", airport=station.ident, airborne=airborne,
                     **rules.as_log())
        if rules is None or rules.vmc:
            if airborne:
                return self._work_the_circuit(session, station, weather, p,
                                              parsed, state)
            return self._issue_vfr_release(session, station, p, parsed, weather)
        station_name = (self._station_name(station, p)
                        if not session.established_with(station) else "")
        if not rules.svfr:
            text = p.special_vfr_unable(aircraft, rules.refused_for, station_name)
            instruction = session.remember(
                Instruction("svfr_unable", station_key(station), text, {},
                            readback_required=False))
            return Reply(text, station, instruction=instruction)

        elevation = float(getattr(airport, "elev_ft", 0.0) or 0.0)
        altitude = svfr_altitude(elevation, rules.ceiling_ft,
                                 self._vfr_ceiling(airport))
        session.vfr_ceiling_ft = altitude
        session.special_vfr = True
        # The pressure is read in the approval, whichever way it goes.
        session.altimeter_issued = True
        airspace = p.airspace(self._airspace_class(airport, station))
        zone = getattr(airport, "spoken", "") or ""
        pressure = (weather.altimeter_inhg if p.region == "faa"
                    else weather.qnh_hpa)

        if airborne:
            point = None
            if getattr(state, "has_position", False):
                point = self.vrp_toward(
                    airport, bearing_deg(airport.lat, airport.lon,
                                         state.latitude, state.longitude),
                    departing=False, svfr=True)
            said = p.vrp(point.name) if point is not None else ""
            if point is not None:
                session.expected_vrp = point.name
                decision("vrp", airport=airport.ident, use="svfr_entry",
                         point=point.name, source=point.source)
            session.phase = Phase.APPROACH
            text = p.special_vfr_enter(
                aircraft, station_name, zone=zone, airspace=airspace,
                entry_point=said, altitude_ft=altitude, pressure=pressure,
                report=said)
            decision("special_vfr", airport=station.ident, said="enter",
                     altitude_ft=altitude, point=point.name if point else None)
            instruction = session.remember(
                Instruction("svfr_entry", station_key(station), text,
                            {"vfr_ceiling_ft": altitude}))
            return Reply(text, station, instruction=instruction)

        departure_station = (self.navdb.station_for(station.ident, "DEP")
                             or self.navdb.station_for(station.ident, "APP"))
        freq = departure_station.mhz if departure_station else 0.0
        session.squawk = (squawk_for(session, station.ident) if freq
                          else p.vfr_squawk)
        asked = self._direction_asked(parsed)
        point = (self.vrp_toward(airport, self._COMPASS_DEG[asked],
                                 departing=True, svfr=True)
                 if asked in self._COMPASS_DEG else None)
        if point is not None:
            session.expected_exit = point.name
            decision("vrp", airport=airport.ident, use="svfr_exit",
                     point=point.name, source=point.source)
        text = p.special_vfr_leave(
            aircraft, station_name, zone=zone, airspace=airspace,
            exit_point=p.vrp(point.name) if point is not None else "",
            direction=p.bearing(asked) if asked else "",
            altitude_ft=altitude, pressure=pressure, squawk=session.squawk,
            departure_freq=freq)
        decision("special_vfr", airport=station.ident, said="leave",
                 altitude_ft=altitude, point=point.name if point else None)
        values: dict[str, Any] = {"squawk": session.squawk,
                                  "vfr_ceiling_ft": float(altitude)}
        if freq:
            values["frequency"] = freq
        if point is not None:
            values["exit_point"] = point.name
        instruction = session.remember(
            Instruction("vfr_release", station_key(station), text, values))
        return Reply(text, station, instruction=instruction)

    # The parser hands back English compass names whatever was spoken, so the
    # decision layer works in those and the phraseology turns one into the
    # local word. Compounds first: "northwest" contains "north".
    _COMPASS = ("northeast", "northwest", "southeast", "southwest",
                "north", "south", "east", "west")

    @classmethod
    def _direction_asked(cls, parsed) -> str:
        """Which way the pilot said they were going, if they said."""
        if parsed is None:
            return ""
        asked = parsed.get("direction") or ""
        if asked:
            return str(asked).lower()
        text = getattr(parsed, "normalized", "") or ""
        for word in cls._COMPASS:
            if word in text:
                return word
        # French: "vers le sud-est", "au nord", "vers l'est". Compound
        # points first, so "sud est" is not read as "sud".
        for french, english in cls._COMPASS_FR:
            if re.search(rf"\b(vers (le |l )?|au |direction ){french}\b", text):
                return english
        return ""

    _COMPASS_FR = (("nord est", "northeast"), ("nord ouest", "northwest"),
                   ("sud est", "southeast"), ("sud ouest", "southwest"),
                   ("nord", "north"), ("sud", "south"), ("est", "east"),
                   ("ouest", "west"))

    # A point this far either side of the way the flight is going is on its
    # way; further round it is not.
    VRP_SECTOR_DEG = 60.0

    def _points(self, airport, *, departing: bool, svfr: bool = False) -> list:
        """The field's reporting points a controller can say, for a
        departure or for an arrival.

        Special VFR points (kind "svfr") for a Special VFR flight; for an
        ordinary one only where the field has no other point -- Lille
        publishes coordinates for its Special VFR points and none for the
        ordinary ones, which are drawn on the chart.
        """
        if airport is None:
            return []
        spoken = [v for v in self.navdb.vrps(airport.ident) if v.spoken]
        special = [v for v in spoken if v.for_svfr]
        if svfr:
            return special
        ordinary = [v for v in spoken
                    if (v.for_departures if departing else v.for_arrivals)]
        return ordinary or special

    def atis_briefing(self, airport, runway: str, p) -> tuple[str, list[str]]:
        """The VFR part of a field's ATIS: the published circuit side for
        ``runway`` ("" where none is published) and the ordinary reporting
        points, said as a controller says them. Nothing with
        ``immersion.controller.atis_briefing`` off."""
        if not self.realism.atis_briefing or airport is None:
            return "", []
        published = self.navdb.circuit_data(airport.ident, runway)
        side = (published.side or "") if published is not None else ""
        points = [p.vrp(v.name) for v in self.navdb.vrps(airport.ident)
                  if v.spoken and not v.for_svfr
                  and (v.for_departures or v.for_arrivals)]
        return side, points

    def vrp_toward(self, airport, bearing: float, *, departing: bool,
                   svfr: bool = False):
        """The point nearest the direction a flight is going, or None."""
        best = None
        for point in self._points(airport, departing=departing, svfr=svfr):
            towards = bearing_deg(airport.lat, airport.lon, point.lat, point.lon)
            off = abs(((towards - bearing) + 540.0) % 360.0 - 180.0)
            if off <= self.VRP_SECTOR_DEG and (best is None or off < best[0]):
                best = (off, point)
        return best[1] if best else None

    def vrp_ahead(self, airport, state):
        """The arrival point between an aeroplane and the field: on its side
        of the field, and nearer the field than it is. None once it is
        inside them all."""
        if airport is None or state is None or not getattr(state, "has_position", False):
            return None
        from_field = bearing_deg(airport.lat, airport.lon,
                                 state.latitude, state.longitude)
        point = self.vrp_toward(airport, from_field, departing=False)
        if point is None:
            return None
        out = state.distance_to(airport.lat, airport.lon)
        there = state.distance_to(point.lat, point.lon)
        point_out = haversine_nm(airport.lat, airport.lon, point.lat, point.lon)
        return point if out > point_out and there < out else None

    _COMPASS_DEG = {"north": 0.0, "northeast": 45.0, "east": 90.0,
                    "southeast": 135.0, "south": 180.0, "southwest": 225.0,
                    "west": 270.0, "northwest": 315.0}

    # How far under the floor of the airspace above a VFR ceiling is set.
    UNDER_THE_SHELF_FT = 500.0
    # The top of the airspace a VFR flight may use where Class A begins
    # (FAA: 18,000 ft MSL).
    BELOW_CLASS_A_FT = 17500.0

    def _vfr_ceiling(self, airport) -> float:
        """A level to keep a VFR departure or transit at or below, in feet
        AMSL: under the floor of the airspace above the field -- a Class B
        or C shelf, a TMA base -- where one is published
        (``NavDB.airspace_over``), else field elevation plus
        VFR_CEILING_AGL_FT. That figure put Centennial's departures at 8,500
        ft, inside Denver's Class B, which starts at 8,000 above the field.
        """
        elevation = float(getattr(airport, "elev_ft", 0.0) or 0.0)
        floors = []
        if airport is not None:
            for volume in self.navdb.airspace_over(airport.lat, airport.lon):
                floor = volume.floor_msl(elevation)
                if volume.cls in "ABC" and volume.airport != airport.ident \
                        and floor > elevation + 300.0:
                    floors.append(floor)
        if floors:
            ceiling = math.floor((min(floors) - self.UNDER_THE_SHELF_FT) / 500.0) * 500.0
            decision("vfr_ceiling", airport=getattr(airport, "ident", ""),
                     ceiling_ft=ceiling, under_ft=min(floors))
            return min(ceiling, self.BELOW_CLASS_A_FT)
        return round((elevation + self.VFR_CEILING_AGL_FT) / 500.0) * 500.0

    def _airspace_class(self, airport, station) -> str:
        """Which class of airspace a facility owns.

        The published volume where there is one (``NavDB.airspace_over``):
        the field's own surface volume for a tower, the field's B or C for
        its radar positions. Where nothing is published, the size of the
        field and the position being worked, which gets the common cases
        right: a tower-only field is a Delta, a radar field is a Charlie, and
        a large scheduled airport is a Bravo. The name it turns into is the
        region's own (``airspace()``).
        """
        if airport is not None:
            own = [v for v in self.navdb.airspace_over(airport.lat, airport.lon)
                   if v.airport == airport.ident]
            radar = station is not None and station.position in ("APP", "DEP", "CTR")
            wanted = [v for v in own if v.cls in ("BC" if radar else "BCD")]
            if wanted:
                return min(v.cls for v in wanted)
        rank = int(getattr(airport, "size_rank", 0) or 0)
        if station is not None and station.position in ("APP", "DEP", "CTR"):
            return "B" if rank >= 3 else "C"
        return "B" if rank >= 3 else "D"

    def _pattern_direction(self, session, parsed, p, airport=None,
                           runway: str = "") -> str:
        """Which way round the circuit goes, as this language says it.

        The pilot's own word first; then the side published for the runway
        end (``NavDB.circuit_data``: the FAA's, or a hand-kept row); then
        whatever the session had, which starts left-hand. Every circuit was
        left-hand, so Centennial's 35R -- right-hand -- was joined on the
        wrong side of the runway.
        """
        asked = (parsed.get("pattern_direction") if parsed else "") or ""
        if asked in ("left", "right"):
            session.pattern_direction = asked
            session.pattern_direction_said = True
        elif not session.pattern_direction_said and airport is not None and runway:
            published = self.navdb.circuit_data(airport.ident, runway)
            if published is not None and published.side:
                session.pattern_direction = published.side
        return p.side(session.pattern_direction)

    def circuit_height(self, airport, runway: str) -> tuple[float, str]:
        """The circuit height above the field, and where it came from: the
        circuit files, the simulator's runway data, or the default."""
        from ..circuit import DEFAULT_HEIGHT_AGL_FT

        if airport is None:
            return DEFAULT_HEIGHT_AGL_FT, "default"
        published = self.navdb.circuit_data(airport.ident, runway)
        if published is not None and published.height_ft_agl:
            return float(published.height_ft_agl), published.source or "file"
        sim = self.sim_patterns.get(airport.ident, {}).get(normalize_runway(runway))
        if sim:
            return float(sim), "simulator"
        return DEFAULT_HEIGHT_AGL_FT, "default"

