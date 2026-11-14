from __future__ import annotations

from ._shared import (
    _SPEED,
    Flight,
    Station,
    Weather,
    _Field,
    _is_enroute_standin,
    decision,
    fleets,
    groundwork,
    normalize_runway,
    project,
)


class _The_field:
    # ------------------------------------------------------------------
    # the field
    # ------------------------------------------------------------------

    def set_field(self, ident: str, departure_runway: str = "",
                  arrival_runway: str = "", weather: Weather | None = None,
                  size_rank: int | None = None,
                  tuned: Station | None = None) -> None:
        """Point the injector at an airport, or move it to a new one.

        Moving fields clears the roster: the aeroplanes at the last airport are
        no longer on this frequency and are not carried along.
        """
        ident = (ident or "").upper()
        if ident != self.field.ident:
            # Held live aeroplanes are let go before the roster is
            # emptied: the feed outlives a field change, and a hold left
            # on the roster draws a statue at the field we just left.
            # The taken-over bookkeeping goes the same way -- those
            # aeroplanes are gone, so nothing of theirs stays suppressed.
            self._release_all_held()
            self._taken_over.clear()
            self._flown_for.clear()
            self._parked_since.clear()
            self.flights.clear()
            self._asked_for.clear()
            self._reserved.clear()
            if self.ground is not None and self.ground.ident != ident:
                self.ground = None
            self._materialising.clear()
            self._materialise_by.clear()
            self.statics.clear()
            self._stands_filled = False
            self._parked_seen = []
            self._parked_observed.clear()
            self.pending.clear()
            self._used_callsigns.clear()
            self._phraseology.clear()
            self._pilots.clear()
            self._runway_free.clear()
            self._watched_at = 0.0
            airport = self.navdb.airport(ident)
            self.field = _Field(
                ident=ident,
                name=airport.spoken or airport.name if airport else ident,
                size_rank=(size_rank if size_rank is not None
                           else (airport.size_rank if airport else 3)),
            )
            # The lookup falls back to the traffic frequency for every
            # position, which is right for a pilot deciding what to tune and
            # wrong here: it would give this field a clearance delivery that
            # does not exist. Only real controllers are kept, and a field with
            # none of them is worked the way it really is -- everybody
            # announcing themselves on one frequency, nobody answering.
            #
            # The fallback also runs *upwards*, and that is the case this has
            # to refuse. A grass strip with no tower and one published Center
            # frequency -- which is most of the United States -- answered
            # ``station_for("APP")`` with Albuquerque Center, because that is
            # genuinely the next thing above it. Kept, it made the field look
            # controlled: the traffic worked an approach controller a hundred
            # miles away, nothing went out on the traffic frequency, and a
            # pilot sitting on it heard an empty channel. So a position only
            # counts when the station really is that position. An enroute
            # centre overhead is not an aerodrome's controller.
            self.field.stations = {
                position: station
                for position in ("DEL", "GND", "TWR", "APP", "DEP")
                if (station := self.navdb.station_for(ident, position))
                and station.is_controller
                and not _is_enroute_standin(station, position)
            }
            self.field.advisory = self.navdb.station_for(ident, "CTAF")
            self.field.afis = self._aerodrome_information(ident)
            self.field.kind = fleets.classify(self.navdb, ident, size_rank)
            decision("fleet", airport=ident, **self.field.kind.log_fields(),
                     density=self.density,
                     in_play_scaled=self._in_play(self.field.kind),
                     movement_gap_scaled_s=round(
                         self._movement_gap_s(self.field.kind)),
                     parked=fleets.parked_for(self.field.kind, self.parked))
            if airport is not None:
                self.field.latitude = airport.lat
                self.field.longitude = airport.lon
                self.field.elevation_ft = airport.elev_ft
            self.field.thresholds = self._thresholds(ident)
            self.field.surfaces = self._surfaces(ident)
        # A field that publishes several frequencies for one position --
        # Brussels has four Grounds -- had its traffic on the one the lookup
        # ranked first while the pilot sat on another, and since a call is
        # heard by its channel (engine._on_this_channel) the pilot heard
        # nobody at all, for a day, at every such field. The traffic works
        # the position on the frequency the pilot is tuned to.
        if (tuned is not None and (tuned.ident or "").upper() == ident
                and tuned.position in self.field.stations
                and tuned.is_controller):
            self.field.stations[tuned.position] = tuned
        if weather is not None:
            self.field.weather = weather
        self.field.departure_runway = departure_runway or self.field.departure_runway
        self.field.arrival_runway = (arrival_runway or departure_runway
                                     or self.field.arrival_runway)

        # The user may not have been given a runway yet -- they have not asked
        # for anything -- but the traffic is already flying, and a takeoff
        # clearance without a runway in it is not a takeoff clearance. So the
        # field runs the configuration the wind implies until the controller
        # says otherwise, which is the same runway the user will be given.
        if not self.field.departure_runway:
            self.field.departure_runway = self._wind_runway()
            self.field.arrival_runway = (self.field.arrival_runway
                                         or self.field.departure_runway)

    def set_ground(self, layout: groundwork.GroundLayout | None) -> None:
        """The taxiways of the field being worked, from the simulator.

        Only for this field, and only for aeroplanes made from now on: one
        already sitting somewhere it was placed is left to finish as it
        started.
        """
        if layout is None or not layout:
            return
        if layout.ident != self.field.ident:
            return
        if layout is not self.ground:
            # A static aeroplane's stand is a number in the old layout.
            self.statics.clear()
            self._stands_filled = False
        self.ground = layout

    # How long a runway is taken to be where nothing says otherwise. Only used
    # to bound where an entry or an exit may be found.
    RUNWAY_LENGTH_M = 3500.0

    def _runway_xy(self, runway: str):
        """A threshold in the ground layout's plane, with its heading."""
        end = self._threshold(runway)
        if end is None or self.ground is None:
            return None
        return self.ground.to_xy(end[0], end[1]), end[2]

    def _thresholds(self, ident: str) -> dict[str, tuple[float, float, float]]:
        """Every runway end, as (latitude, longitude, heading).

        Ends without published coordinates are left out rather than guessed:
        an aeroplane on a centreline that is not where the runway is would be
        worse than one drawn at the field.
        """
        found: dict[str, tuple[float, float, float]] = {}
        for runway in self.navdb.runways(ident):
            for end, heading, lat, lon in runway.ends():
                if not end or lat is None or lon is None:
                    continue
                if heading is None:
                    from ...navdata.db import _heading_from_ident

                    heading = _heading_from_ident(end)
                if heading is None:
                    continue
                found[normalize_runway(end)] = (float(lat), float(lon),
                                                float(heading))
        return found

    def _threshold(self, runway: str):
        """The end an aeroplane is using, or None if it is not published."""
        return self.field.thresholds.get(normalize_runway(runway))

    def _surfaces(self, ident: str) -> dict[str, tuple[float, float, float]]:
        """Every runway end's elevation, the far end's, and the length.

        The published airport elevation is the highest point of its
        runways. Funchal's is 192 ft and the threshold of runway 05 is at
        147: an arrival aimed at the published figure levelled off 45 ft
        above the runway, slowed to a walk up there, and taxied to the gate
        through the air.
        """
        found: dict[str, tuple[float, float, float]] = {}
        for runway in self.navdb.runways(ident):
            low, high = runway.le_elev_ft, runway.he_elev_ft
            if low is None or high is None:
                continue
            length_nm = max(0.1, (runway.length_ft or 0.0) / 6076.1)
            if runway.le_ident:
                found[normalize_runway(runway.le_ident)] = (
                    float(low), float(high), length_nm)
            if runway.he_ident:
                found[normalize_runway(runway.he_ident)] = (
                    float(high), float(low), length_nm)
        return found

    def _threshold_ft(self, runway: str) -> float:
        """How high the landing end of a runway is."""
        surface = self.field.surfaces.get(normalize_runway(runway))
        return surface[0] if surface else self.field.elevation_ft

    def _surface_ft(self, flight: Flight) -> float:
        """How high the ground is under an aeroplane on or near its runway.

        Short of the threshold it is the threshold; along the runway it
        follows the slope to the far end; past that, the far end.
        """
        surface = self.field.surfaces.get(normalize_runway(flight.runway))
        end = self._threshold(flight.runway)
        if surface is None or end is None:
            return self.field.elevation_ft
        near, far, length_nm = surface
        along, _across = self._runway_frame(flight, end[2])
        share = min(1.0, max(0.0, along / length_nm))
        return near + (far - near) * share

    def _place(self, flight: Flight) -> None:
        """Where an aeroplane is when the flight model is not flying it.

        Everything on a taxiway: the stand it is pushing back from, the queue
        at the holding point, the walk to the gate afterwards. No open dataset
        publishes taxiway geometry this could fly an aeroplane along, so those
        are placed from the phase rather than invented as a route.

        From lining up to slowing down after landing it is flown instead, by
        :mod:`meridianatc.atc.flying`, and this leaves it alone.
        """
        field_lat, field_lon = self.field.latitude, self.field.longitude
        if not field_lat and not field_lon:
            return
        if flight.airborne_model:
            return

        elevation = self.field.elevation_ft
        end = self._threshold(flight.runway)
        phase = flight.phase
        flight.ground_speed_kt = _SPEED.get(phase, 0.0)
        flight.vertical_rate_fpm = 0.0
        flight.on_ground = True
        flight.altitude_ft = elevation

        if phase in ("holding", "ready", "waiting") and end is not None:
            lat, lon, heading = end
            # One aeroplane lines up; the rest wait behind it. Without this
            # they all sit on the threshold, which looks like one aeroplane on
            # a map and like a pile-up in the simulator.
            waiting = [f for f in self.flights
                       if f is not flight and not f.arriving
                       and f.phase in ("holding", "ready", "waiting")
                       and f.next_at <= flight.next_at]
            along, across = -0.03, 0.02 + 0.025 * len(waiting)
            flight.latitude, flight.longitude = project(lat, lon, heading, along)
            flight.latitude, flight.longitude = project(
                flight.latitude, flight.longitude,
                (heading + 90.0) % 360.0, across)
            flight.heading = heading
            return

        # Everything else is on the apron. Spread them around the field rather
        # than stacking them on the centre of it.
        bearing = (hash(flight.written) % 360)
        flight.latitude, flight.longitude = project(
            field_lat, field_lon, bearing, 0.35)
        flight.heading = (bearing + 90.0) % 360.0

