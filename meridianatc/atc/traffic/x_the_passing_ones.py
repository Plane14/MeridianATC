"""The aeroplanes just passing through.

Ambient fill for the X-Plane native renderer (traffic.xplane_fill).
Every AI-plane slot the field's own flights do not claim is flown by
us -- an airliner crossing high on its way somewhere else, a light
single on a short hop, a helicopter low across the bay -- rather than
by the simulator's own AI, whose legs are random and know nothing of
an aeroplane's range or an operator's network.

They are not on the frequency: no strip, no calls, no sequencing, and
the panel never lists them. Each still flies a real leg -- climbing
away from a nearby field, crossing at cruise, or letting down toward
one -- on a route the type and the operator would actually fly, which
is the difference between an aeroplane and a spawn.
"""

from __future__ import annotations

from dataclasses import dataclass

from ._shared import (
    bearing_deg,
    haversine_nm,
    perftable,
    project,
)

# What fills a spare slot: scheduled flights above all, then the light
# aeroplanes, then rotorcraft -- roughly the shape a radar screen has.
_AMBIENT_MIX = (0.62, 0.26)      # IFR share, VFR share; the rest is rotary

# Rotorcraft types the native renderer has a default model and a law
# for -- the ones common enough to belong in the background.
_HELI_TYPES = (
    "R44", "R66", "B06", "B407", "EC30", "EC35", "EC45", "AS50",
    "S76", "A109", "A139", "B412", "UH1", "H145", "H160", "AS32",
)

# How the letdown is drawn: three hundred feet a mile, the glide every
# aeroplane is planned down.
_GLIDE_FT_PER_NM = 300.0


@dataclass
class _Transit:
    """One ambient aeroplane on a real leg.

    ``state`` is where in its flight it is: ``climb`` just off a nearby
    field and still going up, ``enroute`` established in the cruise, or
    ``descend`` letting down toward a nearby one. Only ``descend`` has
    anywhere to arrive at; the others leave when they have crossed the
    sky they were put in to fill.
    """

    callsign: str
    type_code: str
    operator: str            # the operator's ICAO, or "" for a private one
    ifr: bool
    state: str               # climb | enroute | descend
    lat: float
    lon: float
    alt_ft: float
    hdg: float               # demanded track, degrees
    gs_kt: float
    cruise_ft: float         # level-off height (climb, enroute)
    dest_lat: float
    dest_lon: float
    dest_ele_ft: float
    vs_fpm: float = 0.0
    age_s: float = 0.0


class _The_passing_ones:
    """The ambient pool: kept on the injector because it wants the same
    navdata, the same fleets and the same callsign book the frequency
    uses."""

    def ambient_positions(self, now: float, room: int) -> list[dict]:
        """Snapshots for the slots the real traffic did not claim.

        ``room`` is how many planes the renderer can still take this
        tick; the pool is topped up to it and only that many are
        offered, in the order they joined, so an aeroplane mid-leg is
        not swapped out for a fresh spawn.
        """
        if not getattr(self, "ambient_fill", False) or room <= 0:
            return []
        if not (self.field.latitude or self.field.longitude):
            return []
        pool = getattr(self, "_ambient", None)
        if pool is None:
            pool = self._ambient = []
            self._ambient_at = now
        dt = min(5.0, max(0.0, now - self._ambient_at))
        self._ambient_at = now
        for transit in pool:
            self._fly_transit(transit, dt)
        gone = self.radius_nm + 20.0
        for transit in [t for t in pool
                        if t.age_s > 1500.0
                        or haversine_nm(t.lat, t.lon, t.dest_lat,
                                        t.dest_lon) < 3.0
                        or haversine_nm(t.lat, t.lon, self.field.latitude,
                                        self.field.longitude) > gone]:
            pool.remove(transit)
            self._used_callsigns.discard(transit.callsign)
        for _ in range(room - len(pool)):
            transit = self._new_transit()
            if transit is None:
                break
            pool.append(transit)
            # Booked like the frequency's own callsigns: a real flight is
            # never handed a name the sky already wears.
            self._used_callsigns.add(transit.callsign)
        return [self._transit_snapshot(t) for t in pool[:room]]

    # ---- one aeroplane -------------------------------------------------

    def _new_transit(self) -> _Transit | None:
        roll = self.random.random()
        if roll < _AMBIENT_MIX[0]:
            return self._transit_airline()
        if roll < _AMBIENT_MIX[0] + _AMBIENT_MIX[1]:
            return self._transit_light()
        return self._transit_heli()

    def _transit_airline(self) -> _Transit | None:
        drawn = self._airline_flight()
        if drawn is None:
            return None
        callsign, type_code, operator = drawn
        perf = perftable.traffic_performance(type_code)
        # The true cruise, not the terminal model's: traffic_performance
        # deliberately answers the FL150 climb speed for "cruise".
        tas = self._cruise_tas(type_code, perf)
        jet = tas >= 380.0
        ceiling = perf.ceiling_ft or (41000.0 if jet else 25000.0)
        cruise = min(
            ceiling - 1000.0,
            self.random.uniform(30000.0, 39000.0) if jet
            else self.random.uniform(15000.0, 24000.0))
        dest = self._far_end(operator, type_code, perf)
        mode = self.random.random()
        if mode < 0.22:
            transit = self._transit_descend(
                callsign, type_code, operator.icao, perf, tas, jet=jet,
                low_ft=18000.0 if jet else 9000.0)
            if transit is not None:
                return transit
        if mode < 0.44:
            transit = self._transit_climb(
                callsign, type_code, operator.icao, perf, tas, cruise, dest)
            if transit is not None:
                return transit
        return self._transit_enroute(
            callsign, type_code, operator.icao, perf, tas, cruise, dest,
            speed=0.95)

    def _transit_light(self) -> _Transit | None:
        drawn = self._light_flight()
        if drawn is None:
            return None
        callsign, type_code, _ = drawn
        perf = perftable.traffic_performance(type_code, light=True)
        tas = self._cruise_tas(type_code, perf)
        # A light aeroplane's cruise altitude is above the ground, like
        # the helicopter's below -- at a high field the bare figure put
        # it inside the terrain.
        cruise = self.field.elevation_ft + self.random.uniform(2500.0,
                                                             7500.0)
        if self.random.random() < 0.3:
            transit = self._transit_descend(
                callsign, type_code, "", perf, tas, jet=False,
                low_ft=1500.0)
            if transit is not None:
                transit.ifr = False
                return transit
        dest = self._nearby_end(30.0, 70.0)
        if dest is None:
            return None
        transit = self._transit_enroute(
            callsign, type_code, "", perf, tas, cruise, dest, speed=1.0)
        if transit is not None:
            transit.ifr = False
        return transit

    def _transit_heli(self) -> _Transit | None:
        drawn = self._light_flight()
        if drawn is None:
            return None
        callsign, _, _ = drawn
        type_code = self.random.choice(_HELI_TYPES)
        perf = perftable.traffic_performance(type_code, light=True)
        tas = self._cruise_tas(type_code, perf)
        cruise = self.random.uniform(800.0, 3000.0)
        dest = self._nearby_end(15.0, 45.0)
        if dest is None:
            return None
        transit = self._transit_enroute(
            callsign, type_code, "", perf, tas,
            self.field.elevation_ft + cruise, dest, speed=1.0)
        if transit is not None:
            transit.ifr = False
        return transit

    # ---- legs ----------------------------------------------------------

    @staticmethod
    def _cruise_tas(type_code: str, perf) -> float:
        """The type's true cruise speed where the table knows it; the
        class figure where it does not."""
        row = perftable.type_perf(type_code)
        if row is not None and row.cruise_tas_kt > 0:
            return row.cruise_tas_kt
        return perf.cruise_kt

    def _far_end(self, operator, type_code: str, perf):
        """Where an IFR leg is going: the airport _route_end would have
        filed, as (lat, lon, elevation). A fallback point keeps the leg
        alive when the navdata has nothing."""
        name, icao = self._route_end(
            operator, type_code, perf.range_nm or 0.0)
        far = self.navdb.airport(icao) if icao else None
        if far is not None and far.lat:
            return far.lat, far.lon, far.elev_ft
        reach = (perf.range_nm or 800.0) * 0.4
        lat, lon = project(self.field.latitude, self.field.longitude,
                           self.random.uniform(0.0, 360.0),
                           max(150.0, reach))
        return lat, lon, self.field.elevation_ft

    def _nearby_end(self, low_nm: float, high_nm: float):
        """An airport near the field to leave from or let down into, as
        (lat, lon, elevation), or None."""
        try:
            found = self.navdb.nearest_airports(
                self.field.latitude, self.field.longitude, limit=8,
                max_nm=high_nm, min_rank=1, require_runway=True)
        except Exception:
            return None
        near = [(a, d) for a, d in found
                if a.ident != self.field.ident and d >= low_nm]
        if not near:
            return None
        airport, _ = self.random.choice(near)
        return airport.lat, airport.lon, airport.elev_ft

    def _transit_enroute(self, callsign, type_code, operator, perf,
                         tas: float, cruise_ft, dest, speed: float):
        """Established at cruise, crossing the sky on its way to ``dest``
        -- entering upwind of the field so the track passes overhead."""
        reach = min(55.0, self.radius_nm * 0.55)
        hdg = bearing_deg(self.field.latitude, self.field.longitude,
                          dest[0], dest[1])
        lat, lon = project(self.field.latitude, self.field.longitude,
                           (hdg + 180.0) % 360.0,
                           self.random.uniform(30.0, reach))
        return _Transit(
            callsign=callsign, type_code=type_code, operator=operator,
            ifr=True, state="enroute", lat=lat, lon=lon, alt_ft=cruise_ft,
            hdg=bearing_deg(lat, lon, dest[0], dest[1]),
            gs_kt=max(80.0, tas * speed), cruise_ft=cruise_ft,
            dest_lat=dest[0], dest_lon=dest[1], dest_ele_ft=dest[2])

    def _transit_climb(self, callsign, type_code, operator, perf,
                       tas: float, cruise_ft, dest):
        """Just off a nearby field, still climbing to its level."""
        home = self._nearby_end(12.0, 45.0)
        if home is None:
            return None
        hdg = bearing_deg(home[0], home[1], dest[0], dest[1])
        lat, lon = project(home[0], home[1], hdg,
                           self.random.uniform(8.0, 18.0))
        alt = min(cruise_ft, home[2] + self.random.uniform(2500.0, 7000.0))
        return _Transit(
            callsign=callsign, type_code=type_code, operator=operator,
            ifr=True, state="climb", lat=lat, lon=lon, alt_ft=alt,
            hdg=hdg, gs_kt=max(80.0, tas * 0.7),
            cruise_ft=cruise_ft,
            dest_lat=dest[0], dest_lon=dest[1], dest_ele_ft=dest[2])

    def _transit_descend(self, callsign, type_code, operator, perf,
                         tas: float, jet: bool, low_ft: float):
        """Letting down toward a nearby field -- the aeroplane that
        disappears because it landed, not because it timed out."""
        dest = self._nearby_end(12.0, 45.0)
        if dest is None:
            return None
        reach = min(60.0, self.radius_nm * 0.6)
        dist = self.random.uniform(35.0, reach)
        entry = self.random.uniform(0.0, 360.0)
        lat, lon = project(dest[0], dest[1], entry, dist)
        # High up the leg starts -- but never above where the type lives:
        # a Cherokee does not let down from flight levels.
        cap = min(28000.0, perf.ceiling_ft - 1000.0
                  if perf.ceiling_ft else 10000.0)
        alt = min(cap, max(low_ft,
                           dest[2] + 1500.0 + dist * _GLIDE_FT_PER_NM))
        return _Transit(
            callsign=callsign, type_code=type_code, operator=operator,
            ifr=True, state="descend", lat=lat, lon=lon, alt_ft=alt,
            hdg=bearing_deg(lat, lon, dest[0], dest[1]),
            gs_kt=max(80.0, tas * (0.85 if jet else 1.0)),
            cruise_ft=alt,
            dest_lat=dest[0], dest_lon=dest[1], dest_ele_ft=dest[2])

    # ---- flying --------------------------------------------------------

    def _fly_transit(self, transit: _Transit, dt: float) -> None:
        if dt <= 0.0:
            return
        transit.age_s += dt
        transit.lat, transit.lon = project(
            transit.lat, transit.lon, transit.hdg,
            transit.gs_kt * dt / 3600.0)
        transit.hdg = bearing_deg(transit.lat, transit.lon,
                                  transit.dest_lat, transit.dest_lon)
        left = haversine_nm(transit.lat, transit.lon,
                            transit.dest_lat, transit.dest_lon)
        if transit.state == "climb":
            step = 1600.0 * dt / 60.0
            transit.alt_ft = min(transit.cruise_ft, transit.alt_ft + step)
            transit.vs_fpm = 1600.0 if transit.alt_ft < transit.cruise_ft \
                else 0.0
        elif transit.state == "descend":
            want = transit.dest_ele_ft + 1200.0 + left * _GLIDE_FT_PER_NM
            step = 1800.0 * dt / 60.0
            if transit.alt_ft > want:
                transit.alt_ft = max(want, transit.alt_ft - step)
                transit.vs_fpm = -1800.0
            else:
                transit.vs_fpm = 0.0
        else:
            transit.vs_fpm = 0.0

    def _transit_snapshot(self, transit: _Transit) -> dict:
        # The point a little way ahead is what the slot's pilot steers
        # to, so position recovery always pulls forward along the leg.
        ahead_lat, ahead_lon = project(transit.lat, transit.lon,
                                       transit.hdg, 1.5)
        demand_alt = (transit.dest_ele_ft + 1200.0
                      + haversine_nm(transit.lat, transit.lon,
                                     transit.dest_lat, transit.dest_lon)
                      * _GLIDE_FT_PER_NM) \
            if transit.state == "descend" else transit.cruise_ft
        return {
            "callsign": transit.callsign,
            "type": transit.type_code,
            "operator": transit.operator,
            "ifr": transit.ifr,
            "lat": transit.lat,
            "lon": transit.lon,
            "altitude_ft": transit.alt_ft,
            "heading": transit.hdg,
            "ground_speed_kt": transit.gs_kt,
            "vertical_rate_fpm": transit.vs_fpm,
            "on_ground": False,
            "static": False,
            "observed": False,
            "source": "ambient",
            "phase": "enroute",
            "motion": {
                "lat": ahead_lat,
                "lon": ahead_lon,
                "alt": demand_alt,
                "heading": transit.hdg,
                "gs": transit.gs_kt,
                "vs": transit.vs_fpm,
                "on_ground": False,
            },
        }
