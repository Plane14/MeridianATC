from __future__ import annotations

from ._shared import (
    FieldApproaches,
    NavDB,
    Phraseology,
    Station,
)
from .x_clearance_delivery import _Clearance_delivery
from .x_emergencies import _Emergencies
from .x_entry_point import _Entry_point
from .x_ground import _Ground
from .x_helpers import _Helpers
from .x_radar_positions_departure import _Radar_positions_departure
from .x_readback_checking import _Readback_checking
from .x_the_circuit import _The_circuit
from .x_tower import _Tower
from .x_uncontrolled_fields import _Uncontrolled_fields
from .x_vfr_helpers import _Vfr_helpers
from .x_what_the_arrival import _What_the_arrival
from .x_what_the_controller import _What_the_controller
from .x_working_the_user import _Working_the_user


class ControllerBrain(_Entry_point, _Emergencies, _What_the_controller, _Readback_checking, _Clearance_delivery, _Vfr_helpers, _Ground, _Tower, _Working_the_user, _The_circuit, _What_the_arrival, _Radar_positions_departure, _Uncontrolled_fields, _Helpers):
    """Chooses the correct instruction for a situation."""

    def __init__(self, navdb: NavDB, phraseology: Phraseology | None = None,
                 strict_icao_digits: bool = False,
                 ground_control: bool = True):
        self.navdb = navdb
        # ``atc.ground_control``. The brain is where it is decided because
        # every handoff, every unprompted call and every answer already asks
        # the brain who is working a position.
        self.ground_control = ground_control
        # What the simulator says each airport publishes, by ident, filled in
        # by the engine as the answers arrive. Empty means nothing is known,
        # and nothing known is the ILS (see :mod:`meridianatc.atc.approaches`).
        self.approach_data: dict[str, FieldApproaches] = {}
        # The same airports' SIDs and STARs, decoded from the facility data
        # by integrations.terminal_procedures for the First Officer.
        self.terminal_procedures: dict = {}
        # Each runway end's pattern height above the field, as the
        # simulator's facility data gives it (integrations.runways), by
        # field. Behind the circuit files, ahead of the default.
        self.sim_patterns: dict[str, dict[str, int]] = {}
        # The flight plan's runways and SID, by airport: {"KJFK":
        # {"departing": "31L", "sid": "SKORR5"}, "KBOS": {"landing": "04R"}}.
        # Used whenever the wind allows them, never against it.
        self.plan_runways: dict[str, dict[str, str]] = {}
        # The flight plan's fixes in order (integrations.simbrief.PlanFix):
        # what a centre grants a direct to. Empty without a plan.
        self.plan_fixes: list = []
        # The filed route (atc/route.py), when a plan has one: the minutes
        # to the cruise, the SID's fixes, where a departure is sent direct.
        self.route = None
        # The arrival watch (arrival.py), set by the engine: a requested hold
        # or descent is answered with what it would have said unprompted.
        self.arrivals = None
        # Where the aeroplane is along it (route.Progress), kept by the
        # engine: what the descent is measured on, the miles still to fly.
        self.route_progress = None
        self.phrase = phraseology or Phraseology("faa")
        self.strict_icao_digits = (
            strict_icao_digits or getattr(self.phrase, "strict_icao_digits", False)
        )
        # Phraseology objects are cheap but not free; one per language,
        # facility and region is plenty and keeps them out of the hot path.
        self._phraseology: dict[tuple[str, str, str], object] = {}
        # The pilot's profile (atc.profile): its pinned region is every
        # controller's. The engine hands it in with each session.
        from ..profile import DEFAULT as DEFAULT_PROFILE

        self.profile = DEFAULT_PROFILE
        # ``immersion.controller``: which of the realism behaviours are on.
        # The engine hands in the configured one; on its own, all of them.
        from ...config import ControllerRealismConfig, SpeechConfig

        self.realism = ControllerRealismConfig()
        # The recogniser's settings (config ``speech``), handed in by the
        # engine: ``confirm_below`` is where a readback heard nearly is
        # asked about rather than refused.
        self.hearing = SpeechConfig()
        # The runway in use at each field, landing and departing, kept until
        # the wind takes it away (``_wind_runway``); and the last change at
        # each, ``(was, now)``, for the tower to announce (watch.RunwayWatch).
        self.runway_in_use: dict[tuple[str, bool], str] = {}
        self.runway_changes: dict[str, tuple[str, str]] = {}

    def controller_at(self, ident: str, position: str,
                      runway: str = "") -> Station | None:
        """The controller working a position, or ``None`` if nobody is.

        Not the same question as :meth:`NavDB.station_for`, which falls back
        down the chain to the traffic frequency so that a pilot tuning it at an
        uncontrolled field is talking to the right place. That fallback must
        not reach a handoff: an aerodrome traffic frequency has nobody on it,
        and telling a pilot to contact one is telling them to talk to an empty
        room. Where this returns nothing, the position that was asked keeps the
        aircraft and deals with the request itself.

        ``runway`` picks between the frequencies of a split position, where
        their descriptions say which runways each works.
        """
        found = (self.navdb.station_for_runway(ident, position, runway)
                 if runway else None) or self.navdb.station_for(ident, position)
        if not self.staffed(found) or not found.mhz:
            return None
        return found

    def closed(self, station: Station | None) -> bool:
        """A published controller position this program has been told not to work."""
        return (station is not None and station.position == "GND"
                and not self.ground_control)

    def staffed(self, station: Station | None) -> bool:
        """Whether anybody answers on this station.

        Not :attr:`Station.is_controller` on its own: that says what the
        frequency is, and a Ground frequency is still a controller's frequency
        when Ground is switched off. Everything that speaks unprompted asks
        this instead, so a closed position is as quiet as a traffic frequency.
        """
        if station is None:
            return False
        # Inside its hours (afis.is_open, from service_hours.csv; with none
        # known, always). An information service is somebody on the
        # frequency too, and a field's own one only while its tower is shut.
        from .. import afis

        if station.position == "INFO":
            return afis.is_open(self.navdb, station)
        return (station.is_controller and not self.closed(station)
                and afis.is_open(self.navdb, station))

    def issues_clearances(self, station: Station | None) -> bool:
        """Whether this station clears anybody to do anything: staffed, and
        not an information service. An AFIS gives the runway "at your
        discretion" and a FIS gives information; neither owes a clearance, so
        nothing that watches for a missing one may run on either."""
        return (self.staffed(station) and station is not None
                and station.position != "INFO")

    def for_station(self, station: Station, language: str = "en"):
        """Phraseology for a station, in the language being spoken.

        Two independent axes meet here. The *language* is whatever the pilot
        used; the *dialect* is where the facility is, so English at Orly is
        ICAO English rather than US English -- unless the pilot pinned a
        region, which is then every facility's.
        """
        from ..language import phraseology_for

        language = (language or "en").lower()[:2]
        ident = station.ident if station else ""
        region = self.profile.region
        cached = self._phraseology.get((language, ident, region))
        if cached is None:
            cached = phraseology_for(language, ident, self.strict_icao_digits,
                                     region=region)
            self._phraseology[(language, ident, region)] = cached
        return cached

    def region_for(self, ident: str) -> str:
        """The VFR wording's region at a field, pinned or its own."""
        return self.profile.region_for(ident)

