from __future__ import annotations

from ._shared import (
    Any,
    Approach,
    Choice,
    Instruction,
    ParsedIntent,
    Phase,
    Reply,
    Station,
    Weather,
    choose,
    decision,
    legacy_kind,
    math,
    plan_approach,
    station_key,
    terrain,
    transition_level,
    vfr_region,
)


class _What_the_arrival:
    # ------------------------------------------------------------------
    # what the arrival watch composes with
    # ------------------------------------------------------------------
    #
    # :mod:`meridianatc.atc.arrival` says the things a controller says about an
    # arrival that nobody asked about -- the descent, the approach, the
    # clearance to land. It composes them from the same pieces this class
    # does, and those pieces were all private to it, so this is the seam
    # rather than four reaching-in calls. Nothing new is decided here; each
    # one is the answer this class already had.

    def release_for_takeoff(self, session, station, weather, p,
                            state=None) -> Reply:
        """The takeoff clearance for an aeroplane already sitting on the runway.

        The same clearance a pilot who asks gets. What it is for is the
        aeroplane that asked once, was told to line up and wait because
        somebody was landing, and then heard nothing more for the rest of its
        life -- there was no path in this class that ever went back to a
        departure it had held.
        """
        return self._issue_takeoff(session, station, weather, p,
                                   ParsedIntent())

    def clear_to_land(self, session, station, weather, p, state=None) -> Reply:
        """The landing clearance for an arrival that has not asked for one.

        Identical to what a pilot who calls for it gets: the runway, the
        sequence, the wind, and the phase and the runway allowance that go
        with it. A tower that waits to be asked is the one thing a tower
        never does.
        """
        return self._issue_landing(session, station, weather, p,
                                   ParsedIntent(), state)

    def approach_for(self, session, airport, runway: str,
                     weather=None) -> str:
        """The approach to name for this runway, as charted ("ILS Z",
        "RNAV", "VISUAL"): :meth:`approach_choice`'s."""
        return self.approach_choice(session, airport, runway,
                                    weather).approach.name

    def _current_runway_heard(self, parsed, session, station) -> None:
        """A runway the pilot names by the number the database had and the
        simulator's scenery no longer has ("25" where the signs now say
        "26"): understood, and answered with the current one, which is the
        one every reply, readback check and clearance then uses."""
        values = getattr(parsed, "values", None)
        named = values.get("runway") if isinstance(values, dict) else None
        if not named:
            return
        fields = [getattr(station, "ident", "")]
        for field in (getattr(session, "destination", None),
                      getattr(session, "departure", None)):
            if field is not None and field.ident not in fields:
                fields.append(field.ident)
        for ident in fields:
            if not ident or self.navdb.has_runway(ident, named):
                return
            current = self.navdb.current_runway(ident, named)
            if current and current != named:
                values["runway"] = current
                decision("runway_renamed", airport=ident, heard=named,
                         current=current,
                         why="renumbered in the simulator's scenery")
                return

    def _sim_ils(self, ident: str, runway: str) -> bool | None:
        """Whether the simulator's runway end has an ILS, when the runways
        are the simulator's: True for a navaid its approaches show is a
        localizer, False for none or a VOR, None while that is not known
        (a navaid whose kind the approaches have not told yet) or when the
        runways are the database's, which does not say."""
        if not ident or self.navdb.runway_source(ident) == "db":
            return None
        from ...navdata.db import normalize_runway

        key = normalize_runway(runway)
        for rw in self.navdb.runways(ident):
            for end, navaid, kind in ((rw.le_ident, rw.le_ils, rw.le_ils_kind),
                                      (rw.he_ident, rw.he_ils, rw.he_ils_kind)):
                if normalize_runway(end) != key:
                    continue
                if not navaid:
                    return False
                return {"ILS": True, "VOR": False}.get(kind)
        return None

    def approach_choice(self, session, airport, runway: str,
                        weather=None) -> Choice:
        """The approach a controller names for landing on ``runway``
        (atc/approaches.py ``plan``), chosen once and kept: it changes with
        the runway or a request, never because the weather got known on the
        way in. Never an ILS the simulator says is not there."""
        ident = getattr(airport, "ident", "") or ""
        requested = getattr(session, "requested_approach", "") if session else ""
        key = (ident, runway, requested)
        kept = getattr(session, "approach_choice", None) if session else None
        if kept is not None and getattr(session, "approach_key", None) == key:
            return kept
        known = self.approach_data.get(ident)
        if not self.realism.approach_catalogue:
            available = known.types(runway) if known is not None else None
            return Choice(Approach(choose(available, requested), runway),
                          runway, why="the catalogue is switched off")
        headings = {}
        for end, heading, _rw in self.navdb.runway_ends(ident) if ident else ():
            headings[end] = heading
        choice = plan_approach(
            self.approach_data.get(ident), runway, weather,
            requested=requested,
            airliner=bool(getattr(getattr(session, "profile", None),
                                  "airliner", True)),
            # The visual "in the US and Canada": Canada's fields are C...
            region=("faa" if ident.upper().startswith("C")
                    else vfr_region(ident)),
            runway_headings=headings,
            ils=self._sim_ils(ident, runway))
        if session is not None:
            session.approach_choice = choice
            session.approach_key = key
            decision("approach", airport=ident, runway=runway,
                     approach=choice.approach.name, line=choice.line or None,
                     circle_to=choice.circle_to or None,
                     circle_side=choice.circle_side or None, why=choice.why,
                     requested=requested or None)
        return choice

    def approach_said(self, p, choice: Choice) -> str:
        """The approach as this phrasebook says it, letter and all."""
        return p.approach_name(choice.approach.name)

    def circle_said(self, p, choice: Choice) -> str:
        """", circle to runway 07, north of the airport", or ""."""
        if not choice.circle_to:
            return ""
        return p.circle_clause(choice.circle_to, choice.circle_side)

    # Inside this of the field the procedures take over from the sector
    # altitude: the radar floor is the intercept altitude, not the MSA.
    MSA_RADIUS_NM = 30.0

    @staticmethod
    def _procedure_default(airport) -> float:
        """Field elevation plus 2500 ft, up to the next thousand: 3000 ft at
        a field near sea level, 5000 at Innsbruck and Las Vegas. Where no
        procedure is known, the intercept and the missed approach both."""
        elevation = float(getattr(airport, "elev_ft", 0.0) or 0.0)
        return float(math.ceil((elevation + 2500.0) / 1000.0) * 1000.0)

    def _published(self, airport, runway: str, kind: str, which: int) -> float:
        """A procedure altitude from the simulator, if it is believable:
        between 1000 and 12,000 ft above the field. Outside that it is a unit
        the documentation got wrong, and the default is safer than it."""
        known = self.approach_data.get(getattr(airport, "ident", "") or "")
        if known is None or not runway:
            return 0.0
        value = known.altitudes(runway, legacy_kind(kind))[which]
        elevation = float(getattr(airport, "elev_ft", 0.0) or 0.0)
        believed = elevation + 1000.0 <= value <= elevation + 12000.0
        if value:
            decision("procedure_altitude", airport=airport.ident, runway=runway,
                     which=("faf", "missed")[which], value_ft=round(value),
                     field_ft=round(elevation), accepted=believed)
        if believed:
            return float(round(value / 100.0) * 100.0)
        return 0.0

    def altitude_source(self, airport, runway: str = "", kind: str = "",
                        which: int = 0) -> str:
        """Where an intercept (0) or missed-approach (1) altitude came from."""
        return ("published" if self._published(airport, runway, kind, which)
                else "field+2500")

    def intercept_altitude(self, airport, runway: str = "", kind: str = "") -> float:
        """The altitude to hold until established on the approach.

        The final approach fix altitude where the simulator publishes one,
        else field elevation plus about 2500 ft. Never the sector altitude:
        procedures descend below an MSA, and using it told Innsbruck to
        "maintain flight level one five zero until established" from seven
        thousand feet on a sixteen-mile final.
        """
        return (self._published(airport, runway, kind, 0)
                or self._procedure_default(airport))

    def go_around_altitude(self, airport, runway: str = "") -> float:
        """Where a go-around climbs to: the published missed-approach
        altitude, else field elevation plus about 2500 ft -- never the
        sector altitude, which sent Innsbruck round to fifteen thousand."""
        return (self._published(airport, runway, "", 1)
                or self._procedure_default(airport))

    def descent_floor(self, airport, state=None, runway: str = "",
                      cleared: bool = False) -> float:
        """The lowest a radar position descends an arrival to.

        The sector altitude (:meth:`minimum_for`) while the aeroplane is
        beyond MSA_RADIUS_NM and not cleared for an approach; inside it, or
        once cleared, the intercept altitude, because the approach is the
        terrain protection from there.
        """
        intercept = self.intercept_altitude(airport, runway)
        if cleared or airport is None or state is None \
                or not getattr(state, "has_position", False):
            return intercept if cleared or state is None else max(
                intercept, self.minimum_for(airport, state))
        if state.distance_to(airport.lat, airport.lon) <= self.MSA_RADIUS_NM:
            return intercept
        return max(intercept, self.minimum_for(airport, state))

    def go_around(self, session, station, weather, p, airport,
                  told: bool = False) -> Reply:
        """An arrival that has gone round, announced or seen.

        The landing clearance is spent and is not given back until the
        aeroplane is on the final again and descending (``going_around``).
        The approach clearance and both tower handoffs are armed again, so
        the flight is worked round a second time by the same sequence as the
        first. A tower gives the missed-approach instruction; a radar
        position, which the pilot has told, gives the climb. ``told`` is the
        tower seeing it climb away without a word, which it tells to go
        around exactly as if it had asked.
        """
        aircraft = session.aircraft
        if session.visual:
            return self._back_into_the_circuit(session, station, weather, p,
                                               airport, told)
        session.cleared_landing = False
        session.going_around = True
        session.final_low_ft = 0.0
        session.climb_since = 0.0
        session.phase = Phase.APPROACH
        session.advisories.difference_update({
            "arrival:cleared_approach", "handoff:tower", "handoff:missed"})
        if self.realism.go_around_vectors:
            # Re-vectored into the pattern from where it is (the IFR plan,
            # Phase 5 (d)): the pattern it flew is over, and its speed with
            # it. Left as it was, the next vector was the old intercept.
            session.vector_leg, session.vector_side = "", 0.0
            session.vector_at = 0.0
            session.speed_given, session.speed_lifted = "", False
            session.assigned_speed_kt = None
            session.joined_final = False
        session.arrival_runway = (
            session.arrival_runway or self._arrival_runway(airport, weather))
        altitude = self.go_around_altitude(airport, session.arrival_runway)
        session.assigned_altitude_ft = altitude
        decision("go_around", airport=getattr(airport, "ident", ""),
                 runway=session.arrival_runway, seen=told,
                 announced_on=station.position, altitude_ft=altitude,
                 source=self.altitude_source(airport, session.arrival_runway,
                                             which=1))
        transition = self._transition_altitude(airport)
        if station.position == "TWR" or told:
            text = p.go_around_standard(aircraft, altitude, transition)
        else:
            text = p.climb_maintain(aircraft, altitude,
                                    transition_altitude=transition)
        instruction = session.remember(
            Instruction("go_around", station_key(station), text,
                        {"altitude_ft": int(altitude)}))
        return Reply(text, station, instruction=instruction)

    def _back_into_the_circuit(self, session, station, weather, p, airport,
                               told: bool) -> Reply:
        """A VFR aeroplane that has gone round, announced or seen.

        It was given the instrument missed approach -- an altitude to climb
        to and, from anywhere but the tower, approach control -- which is
        how a Cessna that bounced at Centennial was sent to Denver Approach.
        A VFR go-around climbs out on the upwind and flies the circuit
        again: the tower says which way round and asks for the downwind.
        The words are the join with no particular leg the tower already
        uses -- the FAA's "make left/right traffic" (JO 7110.65 chapter 3,
        section 10; paragraph not checked) and "intégrez le circuit" in
        French -- until the regional phrasebook gives each region its own
        (Phase 3 of the VFR plan).
        """
        session.cleared_landing = False
        session.cleared_option = False
        # Not the instrument go-around state: that holds the landing
        # clearance back until the aeroplane is re-established on an approach
        # and hands it back to approach control on the way.
        session.going_around = False
        session.phase = Phase.APPROACH
        session.pattern_leg = "upwind"
        session.expected_report = "downwind"
        session.expected_report_chased = False
        session.base_owed_behind = ""
        session.arrival_runway = (
            session.arrival_runway or self._arrival_runway(airport, weather))
        decision("go_around", airport=getattr(airport, "ident", ""),
                 runway=session.arrival_runway, seen=told, rules="vfr",
                 announced_on=station.position)
        text = p.enter_pattern(
            session.aircraft, "", session.arrival_runway, leg="",
            direction=self._pattern_direction(session, None, p, airport,
                                              session.arrival_runway),
            report=p.join_report())
        instruction = session.remember(
            Instruction("circuit_report", station_key(station), text, {},
                        readback_required=False))
        return Reply(text, station, instruction=instruction)

    def re_established(self, session, airport, state) -> bool:
        """Back on the final after a go-around: in the cone and descending.

        Clears ``going_around`` when it is so. Without the final in the data
        a descent inside the landing-clearance range is taken instead.
        """
        if not session.going_around:
            return True
        if state is None or not getattr(state, "has_position", False):
            return False
        if float(getattr(state, "vertical_speed_fpm", 0.0) or 0.0) > -300.0:
            return False
        from ..final import final_for

        final = final_for(self.navdb, getattr(airport, "ident", ""),
                          session.arrival_runway)
        if final is not None:
            back = final.in_cone(state, self.LANDING_CLEARANCE_NM)
        else:
            back = self._distance_to(state, airport) <= self.LANDING_CLEARANCE_NM
        if back:
            session.going_around = False
        return back

    def departure_runway(self, airport, weather) -> str:
        """The runway in use for departures: the plan's if the wind allows,
        else the wind's."""
        return self._departure_runway(airport, weather)

    def arrival_runway(self, airport, weather) -> str:
        """The runway in use for arrivals, from the wind."""
        return self._arrival_runway(airport, weather)

    def transition_altitude(self, airport) -> int:
        """Where altitudes become flight levels at this field."""
        return self._transition_altitude(airport)

    def descent_terms(self, airport, weather: Weather | None, p,
                      from_ft: float, target_ft: float) -> dict[str, Any]:
        """How a descent to ``target_ft`` is worded, as keyword arguments.

        A descent is read against the transition *level*: flight levels down
        to it, altitudes below it. The first clearance that leaves the levels
        carries the pressure setting -- QNH in hectopascals, or the altimeter
        in inches where the phrasebook is the FAA's -- and none after it.
        ``from_ft`` is where the aeroplane was cleared or is, whichever the
        caller knows.
        """
        ta = self._transition_altitude(airport)
        wx = weather or Weather()
        tl = transition_level(ta, wx.qnh_hpa, getattr(airport, "ident", ""))
        terms: dict[str, Any] = {"transition_altitude": ta,
                                 "transition_level": tl}
        if target_ft < tl <= from_ft + 50:
            terms["pressure"] = (wx.altimeter_inhg
                                 if getattr(p, "dialect", "") == "faa"
                                 else wx.qnh_hpa)
        return terms

    def minimum_for(self, airport, state=None) -> float:
        """The lowest altitude this controller will clear an arrival to.

        Field elevation plus three thousand feet, raised to two MSA-style
        floors read off the terrain grid (:mod:`meridianatc.navdata.terrain`):
        the field's -- the highest terrain within 25 NM of it plus 1000 ft,
        or 2000 ft where that is mountainous -- and the same circle round the
        aeroplane, for the terrain on the way in that the field's circle
        does not reach. An area, not a point: the ground directly under the
        aircraft used to be the floor, which is the valley at Innsbruck until
        the moment it is a ridge.

        ``minimum_altitudes.csv`` overrides the field's figure, because a
        published MSA is the authority and is often lower than a grid allows;
        inside its 25 NM it overrides the aircraft's circle too, which would
        otherwise put the grid's figure straight back.
        """
        elevation = float(getattr(airport, "elev_ft", 0.0) or 0.0)
        floor = round((elevation + 3000.0) / 1000.0) * 1000.0
        if airport is None:
            return float(floor)
        published = float(self.navdb.minimum_altitude(airport.ident))
        if published:
            floor = max(floor, published)
        else:
            floor = max(floor, terrain.floor_ft(airport.lat, airport.lon, elevation))
        if state is not None and getattr(state, "has_position", False):
            inside = (state.distance_to(airport.lat, airport.lon)
                      <= terrain.RADIUS_NM)
            if not (published and inside):
                floor = max(floor, terrain.floor_ft(
                    state.latitude, state.longitude, elevation))
        return float(floor)

    def out_of_the_layer(self, airport, weather: Weather | None,
                         target_ft: float) -> float:
        """A descent target moved out of the transition layer.

        Between the transition altitude and level nothing is assigned: the
        level would be a flight level nobody may use and the altitude one
        above the altitude everybody changes over at. The target goes down to
        the last whole thousand at or under the transition altitude.
        """
        ta = self._transition_altitude(airport)
        tl = transition_level(ta, (weather or Weather()).qnh_hpa,
                              getattr(airport, "ident", ""))
        if ta < target_ft < tl:
            return float(int(ta / 1000.0) * 1000)
        return float(target_ft)

    def bearing_to(self, state, airport) -> tuple[int, float]:
        """Where a field is from the aircraft: a clock position and a range."""
        return self._bearing_to(state, airport)

    def station_name(self, station: Station, p, short: bool = False) -> str:
        """What a station calls itself, in the language being worked in."""
        return self._station_name(station, p, short)

