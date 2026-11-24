"""Enroute convective-weather avoidance by the traffic Meridian owns."""

from __future__ import annotations

from ._shared import Flight, log


class _Weather_avoidance:
    """Scan invented/adopted airborne traffic and give it executable deviations.

    Observed traffic is deliberately absent here: Meridian did not create it and
    must never move it. Adopted traffic has ``observed`` cleared when ownership
    passes to Meridian, so it participates exactly like invented traffic.
    """

    WEATHER_SCAN_S = 12.0

    def _avoid_enroute_weather(self, flight: Flight, now: float) -> None:
        if flight.observed or flight.on_ground or not flight.positioned:
            return
        deviation_phases = ("departed", "inbound", "joining", "go_around")
        if flight.weather_deviation_until:
            if flight.phase not in deviation_phases:
                # A final or landing clearance owns the aeroplane now; a
                # temporary weather vector must not keep overriding it.
                flight.assigned_heading = None
                flight.weather_deviation_until = 0.0
                return
            if now < flight.weather_deviation_until:
                return
            # The avoidance vector is a temporary lateral clearance, not a new
            # route. Resuming own navigation is the model clearing the vector.
            flight.assigned_heading = None
            flight.weather_deviation_until = 0.0
            return
        if flight.phase not in deviation_phases:
            return
        if now - flight.weather_scan_at < self.WEATHER_SCAN_S:
            return
        flight.weather_scan_at = now
        cells_for = getattr(self, "enroute_cells", None)
        if not callable(cells_for):
            return
        try:
            cells = cells_for()
        except Exception:
            log.debug("traffic enroute weather scan failed", exc_info=True)
            return
        if not cells:
            return
        from ...weather.enroute import avoidance_plan

        course = float(flight.track or flight.heading or 0.0)
        plan = avoidance_plan(cells, flight.latitude, flight.longitude,
                              course, flight.altitude_ft)
        if not plan.threatening:
            return
        gs = max(100.0, float(flight.ground_speed_kt or 100.0))
        duration = plan.miles_nm / gs * 3600.0 + 90.0
        flight.weather_deviation_until = now + duration

        def pilot_first(p, a, s, plan=plan):
            return p.request_deviation(a, s, plan.degrees, plan.side)

        def controller(c, a, s, plan=plan):
            return c.deviation_approved(a, plan.degrees, plan.side)

        def pilot_back(p, a, s, plan=plan):
            return p.deviation(a, plan.degrees, plan.side)

        position = getattr(flight, "position", "") \
            or ("DEP" if flight.phase == "departed" else "APP")
        self._exchange(
            flight, now, position, pilot_first=pilot_first,
            controller=controller, pilot_back=pilot_back,
            command=("heading", {"heading": plan.heading_true}))
