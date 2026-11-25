from __future__ import annotations

from ._shared import (
    Airport,
    contextlib,
    haversine_nm,
    log,
)


class _Fuel_planning:
    # -- fuel planning -------------------------------------------------------

    def _is_turbine(self, state) -> bool:
        """Whether the aeroplane burns Jet-A: a jet, a turboprop or a
        turbine helicopter does; a piston -- including a piston
        helicopter -- does not.

        The simulator's ``ENGINE TYPE`` is decisive where it has
        answered; the type's Doc 8643 description answers where it has
        not; the class the session began with is the fallback, and the
        commonest class (a piston single) is what it falls back to.
        """
        from .. import aircraft_class as ac

        # ``ENGINE TYPE`` is -1 until the sim answers; 0 is a real answer
        # -- a piston -- and must not be folded into "unknown".
        raw = getattr(state, "engine_type", -1) if state is not None else -1
        kind = -1 if raw is None else int(raw)
        if kind in (ac.SIM_JET, ac.SIM_HELO_TURBINE, ac.SIM_TURBOPROP):
            return True
        if kind == ac.SIM_PISTON:
            return False
        # The designator's power letter: 'J' and 'T' are Jet-A, 'P' is
        # avgas -- which is also what an H1P helicopter burns.
        code = (getattr(getattr(getattr(self, "session", None), "aircraft", None),
                        "type_code", "") or "").strip().upper()
        description = ac.DOC_8643.get(code)
        if description:
            return description[-1] in ("T", "J")
        if getattr(self, "_is_helicopter", False):
            # The helicopter rows the table does not know are the turbine
            # ones almost everywhere; the pistons are listed above.
            return True
        name = getattr(self, "_aircraft_class", "")
        if not name:
            name = ac.classify(code).name
        return name not in (ac.PISTON_SINGLE, ac.PISTON_TWIN)

    def _lb_per_gal(self, state=None) -> float:
        """The fuel's weight per gallon: what the tanks themselves say,
        else Jet-A on a turbine and avgas else."""
        gal = float(getattr(state, "fuel_total_gal", 0.0) or 0.0)
        lb = float(getattr(state, "fuel_total_lb", 0.0) or 0.0)
        if gal > 0.5 and lb > 0.0:
            return lb / gal
        return 6.7 if self._is_turbine(state) else 6.0

    def _fuel_flow_pph(self, state, estimate: bool = False) -> float:
        """Pounds an hour the type burns in cruise, or 0 where unknown.

        The model's own ESTIMATED FUEL FLOW is the only honest answer for
        an in-flight emergency -- a guessed flow is how an aeroplane with
        plenty aboard gets a minimum-fuel call. For the turnaround's fuel
        order, where erring costs a fuller tank rather than a distress,
        the engine type's rough figure stands in.
        """
        flow = float(getattr(state, "est_fuel_flow_pph", 0.0) or 0.0)
        if flow > 0.0 or not estimate:
            return flow
        # 0 is a real answer -- a piston, eighty pounds an hour -- and
        # ``or -1`` used to fold it into "unknown", which is the airliner
        # figure: a Cessna held short for more fuel than its tanks hold.
        raw = getattr(state, "engine_type", -1)
        kind = -1 if raw is None else int(raw)
        if self._is_helicopter or kind == 3:
            return 300.0
        # The figures are per engine: a twin's order is twice one's, and
        # an estimate that forgets the second engine fuels it for half
        # the leg.
        count = int(getattr(state, "engine_count", 0) or 0) or 1
        return {0: 80.0, 1: 4500.0, 5: 900.0}.get(kind, 1200.0) * count

    def _reserve_hours(self) -> float:
        """The reserve the legs are planned to, in hours.

        Per the ruleset being flown: the IFR figure (14 CFR 91.167's
        forty-five minutes past the alternate) is the configured one;
        VFR wants only thirty minutes past the destination (91.151),
        and a VFR leg priced at the IFR figure calls fuel-critical a
        Sunday flight that never filed a plan.
        """
        if not getattr(self.session, "ifr", True):
            return max(0.25, min(30.0, self._reserve_min) / 60.0)
        return max(0.25, self._reserve_min / 60.0)

    def _pick_alternate(self, state, destination=None) -> Airport | None:
        """The nearest field that can take the aeroplane, for the reserve.

        What a crew files: the closest suitable airport to the
        destination that is not the destination itself. None where there
        is no other field in reach -- an aeroplane with no alternate gets
        no alternate fuel figure, and is not penalised for a field that
        does not exist.
        """
        dest = destination or self.session.destination
        if dest is None:
            return None
        # The thirty nearest, not the eight: the suitability check is
        # what rules a strip out, and a jet ringed by strips too short
        # still wants the field a little further on that takes it.
        for airport, _dist in self.navdb.nearest_airports(
                dest.lat, dest.lon, limit=30, max_nm=120.0,
                require_runway=not self._is_helicopter):
            if airport.ident == dest.ident:
                continue
            if self._airport_suitable(airport):
                return airport
        return None

    def _fuel_needed_gal(self, state, destination=None,
                         alternate=None, estimate: bool = False) -> dict | None:
        """Gallons the rest of the leg needs, broken down for the caller.

        ``leg_gal`` reaches the destination; ``total_gal`` is leg +
        diversion to the alternate + the reserve, the figure a crew may
        not go below without saying so. None where the burn or the field
        is not known -- then no emergency is ever projected on a guess.
        """
        flow = self._fuel_flow_pph(state, estimate=estimate)
        dest = destination or self.session.destination
        if flow <= 0.0 or dest is None \
                or not getattr(state, "has_position", False):
            return None
        # The rest of the leg is flown at the type's own speed, not the
        # one the aeroplane happens to be making now: a leg priced at
        # climb-out groundspeed calls a full tank short for no reason.
        # The figure is the same one the autopilot will actually command
        # -- the model's estimate where it publishes one, its Vno pulled
        # back to a cruise setting next, the class's number last.
        cruise = self._cruise_speed_kt(state) or 120.0
        gs = float(getattr(state, "ground_speed_kt", 0.0) or 0.0)
        # Priced at the type's own speed, not the one the aeroplane
        # happens to be making: a climb-out's groundspeed calls a full
        # tank short for no reason. A groundspeed near the cruise figure,
        # though, is a headwind being made good -- and a headwind does
        # burn more.
        speed = max(60.0, min(cruise, gs) if gs > 0.8 * cruise else cruise)
        leg_nm = state.distance_to(dest.lat, dest.lon)
        leg_hr = leg_nm / speed
        alt = alternate if alternate is not None \
            else self._pick_alternate(state, dest)
        divert_hr = 0.0
        if alt is not None:
            divert_hr = max(15.0, haversine_nm(
                dest.lat, dest.lon, alt.lat, alt.lon)) / speed
        per_gal = self._lb_per_gal(state)
        leg_gal = leg_hr * flow / per_gal
        # The pieces the great-circle figure is short by, priced the way
        # a load sheet prices them: the taxi and run-up at idle flow, the
        # climb out at climb power, and the contingency the ruleset
        # holds against the weather and the routing -- five per cent of
        # the leg, ICAO Annex 6's figure.
        taxi_gal = 0.25 * flow * 0.3 / per_gal
        climb_alt = float(getattr(self.session, "cruise_altitude_ft",
                                  0.0) or 8000.0) or 8000.0
        climb_hr = min(leg_hr, climb_alt / (1200.0 * 60.0))
        climb_gal = climb_hr * flow * 0.5 / per_gal
        contingency_gal = (leg_gal + climb_gal) * 0.05
        # Alternate fuel is an IFR requirement (91.169); a VFR leg does
        # not file one and is not priced for a field it will never
        # divert to.
        ifr = getattr(self.session, "ifr", True)
        divert_gal = divert_hr * flow / per_gal if ifr else 0.0
        reserve_gal = self._reserve_hours() * flow / per_gal
        unusable = float(getattr(state, "fuel_unusable_gal", 0.0) or 0.0)
        total = (leg_gal + taxi_gal + climb_gal + contingency_gal
                 + divert_gal + reserve_gal + unusable)
        return {"leg_gal": leg_gal + unusable,
                "taxi_gal": taxi_gal,
                "climb_gal": climb_gal,
                "contingency_gal": contingency_gal,
                "divert_gal": divert_gal,
                "reserve_gal": reserve_gal,
                "total_gal": total,
                "alternate": alt if ifr else None}

    def _declare_emergency(self, kind: str, state, mayday: bool = True) -> None:
        """The call, the squawk, a divert, and the plan changed to match.

        A distress is MAYDAY and squawked 7700; an urgency is PAN PAN --
        one engine of several, minimum fuel, an electrical problem that
        still leaves a radio. The session hears either way, because the
        transmission goes through the ordinary parser like a pilot's.
        """
        # Severity outranks, it does not silence: a PAN PAN already made
        # does not swallow a MAYDAY after it, and a MAYDAY is not
        # repeated for every lesser thing that follows.
        level = 2 if mayday else 1
        if level <= getattr(self, "_emergency_level", 0):
            return
        self._emergency_level = level
        self.state.emergency = kind
        self.state.emergency_declared = True
        # An emergency ends the chain: nobody flies the next leg on one.
        self._chained = False
        # 7700 is the emergency squawk for urgency and distress alike.
        with contextlib.suppress(Exception):
            self.sim.set_squawk("7700")
        # Nearest airport that can take the aeroplane, for the divert --
        # "can take" as in the runway fits it, not as in the services a
        # scheduled leg would book: the fuel truck a dead aeroplane
        # never uses does not rule a field out.
        current = self._get_current_airport()
        if current is not None:
            for airport, _dist in self.navdb.nearest_airports(
                    current.lat, current.lon, limit=60,
                    max_nm=max(120.0,
                               float(self.state.aircraft_range_nm or 0.0)),
                    require_runway=not self._is_helicopter):
                if airport.ident != current.ident \
                        and self._airport_suitable(airport):
                    self.state.emergency_destination = airport
                    break
        dest = self.state.emergency_destination
        dest_name = dest.ident if dest is not None else "nearest airport"
        callsign = getattr(getattr(self.session.aircraft, "callsign", None),
                           "written", "") or "this aircraft"
        prefix = "Mayday mayday mayday" if mayday else "Pan pan pan pan"
        text = (f"{prefix}, {callsign} "
                f"{kind} emergency, diverting to {dest_name}")
        if self.crew.transmit:
            with contextlib.suppress(Exception):
                self.crew.transmit(text)
        else:
            log.warning("Emergency declared but no radio to say it: %s", text)
        self._emit("error", f"Emergency: {kind}; diverting to {dest_name}",
                   "autonomous.emergency")
        # Fly the diversion: the destination changes, the route goes
        # direct, and the phases in flight pick the new field up on their
        # own -- an aircraft on final does not climb back to cruise to
        # divert, it descends to the new one.
        if dest is not None:
            # Below the maneuvering floor the distress gets the call,
            # the squawk and the filed divert -- but not the turn.
            # "Aviate" at seventy feet is the departure flown straight
            # ahead while the aeroplane climbs; a MAYDAY banked toward
            # a new field at treetop height is the stall the emergency
            # did not cause.
            agl = float(getattr(state, "altitude_agl_ft", 0.0) or 0.0)
            if agl >= self.DIVERT_MIN_AGL_FT:
                self._execute_diversion(state)
            else:
                self._divert_pending = True

    def _execute_diversion(self, state) -> None:
        """Turn the filed divert into the flying: new destination,
        direct route, emergency descent."""
        dest = self.state.emergency_destination
        if dest is None:
            return
        self.session.destination = dest
        # The filing is moot; the guidance is not: IFR stays so the
        # diversion can still fly the new field's ILS, and the
        # below-minima gate still stands, precisely when both are
        # needed most.
        # An emergency descent: get down to a survivable height now,
        # the approach machinery handles the rest. On final the
        # approach is waved off for the new field, with the cached
        # runway geometry dropped so it resolves again.
        self._route_diversion()
        # Aviate: get down toward a height the field can be landed
        # from, now -- a MAYDAY does not fly the plan's top of
        # descent, it flies the ground it is over.
        elev = (getattr(dest, "elev_ft", 0.0) or 0.0) \
            or self._field_elevation(state)
        self.session.assigned_altitude_ft = elev + 2500.0
        self.state.emergency_descent = True

