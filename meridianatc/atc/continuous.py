"""Continuous flights -- StormATC's Mission Builder, ported.

One leg at a time against the live session: when the flight lands, a
destination inside the configured range band is picked from the navigation
database, a minimal plan is built (origin and destination fixes only -- the
service does not invent navdata it does not have), and it is attached the
same way a SimBrief import is. ``reserve_factor`` scales the upper band the
same way StormATC used it: the leg must fit inside ``max_nm`` of what the
aircraft can do, which the caller is responsible for knowing; the factor
discounts it for reserves so a leg is never planned to the last drop.
"""

from __future__ import annotations

import logging
import random
import threading
from dataclasses import dataclass

log = logging.getLogger(__name__)

MAX_SCANNED = 250


@dataclass(frozen=True)
class MissionRequest:
    """What a continuous mission may do: range band, turnaround, reserve."""
    min_nm: float = 30.0
    max_nm: float = 150.0
    service_minutes: int = 30
    reserve_factor: float = 1.2
    flight_rules: str = "ifr"

    def validate(self) -> list[str]:
        errors = []
        if self.min_nm < 1:
            errors.append("min range must be at least 1 NM")
        if self.max_nm <= self.min_nm:
            errors.append("max range must exceed min range")
        if self.service_minutes < 5:
            errors.append("service time must be at least 5 minutes")
        if self.reserve_factor < 1.0:
            errors.append("reserve factor must be at least 1.0")
        return errors


@dataclass(frozen=True)
class Leg:
    """A planned leg: the two ends and which leg of the mission it is."""
    number: int
    departure: str
    destination: str
    distance_nm: float
    callsign: str

    def summary(self) -> str:
        return f"{self.departure} \u2192 {self.destination}"


class ContinuousFlights:
    """Plans legs for a running mission against the live session."""

    def __init__(self, navdb, seed: int | None = None):
        self.navdb = navdb
        self._random = random.Random(seed)
        self._lock = threading.Lock()
        self._request: MissionRequest | None = None
        self._legs: list[Leg] = []

    @property
    def running(self) -> bool:
        with self._lock:
            return self._request is not None

    @property
    def legs(self) -> list[Leg]:
        with self._lock:
            return list(self._legs)

    @property
    def last_route(self) -> str:
        with self._lock:
            if not self._legs:
                return ""
            leg = self._legs[-1]
            return f"Leg {leg.number}: {leg.summary()} ({leg.distance_nm:.0f} NM)"

    def start(self, request: MissionRequest) -> list[str]:
        """Begin a mission; returns validation errors (empty means running)."""
        errors = request.validate()
        if errors:
            return errors
        with self._lock:
            self._request = request
            self._legs = []
        log.info("continuous mission: %.0f-%.0f NM, service %d min, "
                 "reserve %.2f", request.min_nm, request.max_nm,
                 request.service_minutes, request.reserve_factor)
        return []

    def stop(self) -> None:
        with self._lock:
            self._request = None

    def plan_next_leg(self, at_lat: float, at_lon: float,
                      current_ident: str, callsign: str) -> Leg | None:
        """Pick the next destination from where the flight now is.

        The nearest airport inside the band, among the scanned candidates,
        excluding the field the flight is on. ``reserve_factor`` shrinks the
        usable upper band so the leg is never the aircraft's last mile.
        ``None`` when nothing qualifies or no mission is running.
        """
        with self._lock:
            request = self._request
        if request is None:
            return None
        max_nm = request.max_nm / request.reserve_factor
        candidates = self.navdb.nearest_airports(
            at_lat, at_lon, limit=MAX_SCANNED, max_nm=max_nm,
            require_runway=True)
        current = (current_ident or "").strip().upper()
        eligible = [(ap, nm) for ap, nm in candidates
                    if ap.ident.upper() != current
                    and request.min_nm <= nm <= max_nm]
        if not eligible:
            log.warning("no destination inside %.0f-%.0f NM",
                        request.min_nm, max_nm)
            return None
        # Nearest-first is deterministic; a little shuffle over the nearest
        # few keeps a mission from repeating the same field every leg.
        pool = eligible[:min(5, len(eligible))]
        airport, distance = self._random.choice(pool)
        with self._lock:
            leg = Leg(number=len(self._legs) + 1,
                      departure=current or "",
                      destination=airport.ident,
                      distance_nm=distance,
                      callsign=callsign or "")
            self._legs.append(leg)
        log.info("planned leg %s (%.0f NM)", leg.summary(), distance)
        return leg


__all__ = ["ContinuousFlights", "Leg", "MissionRequest"]
