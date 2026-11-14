"""The engine that runs a flight.

Ties the pieces together and owns the two loops that matter:

*The radio loop.* Push to talk, record, transcribe, parse, decide, synthesise,
play. Everything in that chain is local, and on a modern desktop the whole
round trip takes on the order of a second and a half for a normal transmission.

*The situation loop.* Polls the simulator, notices when the pilot tunes a new
frequency, keeps the ATIS current, and offers a handoff when the flight has
outrun the controller it is talking to.

The engine never composes controller speech itself; it asks
:class:`~meridianatc.atc.controller.ControllerBrain` what to say and
:mod:`~meridianatc.atc.phraseology` supplies the words.
"""

from __future__ import annotations

import hashlib
import logging
import math
import os
import threading
import time
from collections import deque
from collections.abc import Callable
from concurrent.futures import Future
from dataclasses import dataclass, field, replace
from typing import Any

from ..atc import handling as handling_spots
from ..atc import interphone, letters
from ..atc import lod as lod_windows
from ..atc.arrival import ArrivalWatch
from ..atc.atis import AtisGenerator, AtisReport
from ..atc.autonomous import AutonomousFlight, create_autonomous_flight
from ..atc.callsigns import every_language, heard_registration
from ..atc.climb import leaving_departure
from ..atc.controller import ControllerBrain, Reply
from ..atc.crew import CrewDirector
from ..atc.departure import ClimbWatch, DepartureWatch, SidWatch
from ..atc.detect import by_words, clearly_in, language_of_text
from ..atc.handling import Handling, fuselage_length
from ..atc.intents import Intent, IntentParser, ParsedIntent
from ..atc.landing import LandingMonitor
from ..atc.language import (
    LANGUAGE_NAMES,
    languages_for,
    profile_for,
    reply_language,
)
from ..atc.lod import Candidate, Lod
from ..atc.phraseology import Aircraft, Phraseology, Weather, wake_suffix
from ..atc.profile import FlightProfile
from ..atc.routing import RouteWatch
from ..atc.servicelayout import fittings_for, load_layout, spots_for
from ..atc.session import FlightSession, Instruction, Phase, station_key
from ..atc.systems import wanted as systems_wanted
from ..atc.traffic import TrafficInjector, UserAircraft
from ..atc.watch import (
    AirspaceWatch,
    CrossingWatch,
    DeviationAssistWatch,
    RadarServiceWatch,
    ReportingPointWatch,
    RestrictionWatch,
    RunwayWatch,
    SurfaceWatch,
    TaxiWatch,
    TransponderWatch,
)
from ..atc.written import as_written
from ..audio.announcements import FOLDER as ANNOUNCEMENTS
from ..audio.announcements import AnnouncementLibrary
from ..audio.controllers import ControllerMissing, ControllersUnavailable
from ..audio.imperfections import from_config as imperfections_for
from ..audio.io import (
    CabinPlayer,
    JoystickPushToTalk,
    MicrophoneCapture,
    PushToTalk,
    RadioPlayer,
)
from ..audio.prosody import from_config as prosody_for
from ..audio.radio_fx import PROFILE_ATIS, PROFILE_HF
from ..audio.stt import Recognizer, SpeechModelMissing, engine_model
from ..audio.tone import from_config as tone_for
from ..audio.tts import Synthesizer
from ..audio.voices import VoicesMissing
from ..config import Config
from ..integrations import gsx
from ..integrations.aitraffic import SimTraffic
from ..integrations.doors import DoorTable, SimDoors
from ..integrations.facilities import SimFacilities
from ..integrations.groundvehicles import SimVehicles, gsx_installed
from ..integrations.gsx import GsxLink
from ..integrations.network import OnlineNetwork
from ..integrations.servicepoints import SimServicePoints
from ..integrations.simaircraft import SimAircraft
from ..integrations.systems import SimSystems
from ..integrations.xptraffic import XPlaneTraffic
from ..llm.providers import build_provider
from ..llm.understanding import Understanding
from ..logs import RADIO, decision, timed
from ..navdata.channels import is_valid_channel, same_channel
from ..navdata.db import (
    STATION_RANGE_NM,
    NavDB,
    Station,
    haversine_nm,
    normalize_runway,
)
from ..navdata.magvar import variation
from ..paths import announcements_dir, command_hint
from ..paths import resolve as _resolve
from ..security.secrets import resolve_secret
from ..sim.capabilities import autofly_limit, capabilities
from ..sim.source import ManualSource, StateSource, open_source
from ..sim.state import AircraftState
from ..weather.metar import (
    WeatherProvider,
    WeatherSample,
    at_the_surface,
    in_the_circuit,
    merge_weather,
    near_the_field,
    parse_metar,
    weather_from_sim,
)

log = logging.getLogger(__name__)
status_log = logging.getLogger("meridianatc.traffic.debug")

#: How long COM1 has to stay on a frequency nobody answers before that is
#: said. A swap passes through the standby frequency, and a knob being
#: turned through every one between.
EMPTY_NOTICE_S = 2.5


@dataclass
class Event:
    """Something worth showing the user.

    ``text`` is English and is what the terminal console prints. ``key`` and
    ``params``, when present, let the panel say the same thing in whatever
    language it is being read in; a notice with no key, or a language with no
    entry for it, falls back to the English.
    """

    kind: str          # "pilot" / "atc" / "info" / "error" / "atis" / "traffic"
    text: str
    speaker: str = ""
    at: float = field(default_factory=lambda: time.time())
    key: str = ""
    params: dict = field(default_factory=dict)
    #: What the panel can do about it, when there is something. Only
    #: ``"setup"`` so far, which means "put the download dialog up": a
    #: missing download is the one failure a pilot can fix from the window,
    #: and a line of red text in the transcript does not tell them that.
    #: ``"spoken"`` on a pilot transmission that came through the
    #: microphone: the panel empties its text box.
    action: str = ""


EventSink = Callable[[Event], None]


def announcement_roots(config) -> list:
    """Where to look for recorded cabin announcements.

    The folder beside the program always, so that dropping a file in works
    with no configuration at all -- and whatever else the pilot named, which
    is how an existing Fenix pack is used where it already sits rather than
    being copied.

    A name that is not a path is looked for inside the announcements folder
    as well as beside the program. "SWR" is a folder somebody has in their
    pack, not a folder next to the executable, and typing it in the settings
    box is a pilot pointing at the one they can see; resolving it only
    against the program's own folder made that setting a silent no-op.
    """
    roots = [announcements_dir()]
    named = (getattr(config.immersion, "recordings_dir", "") or "").strip()
    if not named:
        return roots
    for candidate in _named_roots(named):
        if candidate.is_dir():
            roots.insert(0, candidate)
            return roots
    # Nothing there under any reading of it. The path is still returned, so
    # the settings screen shows what was looked for rather than quietly
    # substituting something else, and `announcements_missing` is what turns
    # that into a line the pilot can act on.
    roots.insert(0, _resolve(named))
    return roots


def _named_roots(named: str):
    """Every place a typed announcements folder might mean, best first."""
    here = announcements_dir()
    return [_resolve(named), here / named, here / ANNOUNCEMENTS / named]


def announcements_missing(config) -> str:
    """The folder the pilot named, if it is not there. Empty if it is fine."""
    named = (getattr(config.immersion, "recordings_dir", "") or "").strip()
    if not named:
        return ""
    if any(path.is_dir() for path in _named_roots(named)):
        return ""
    return named


class NotAFlightPlan(ValueError):
    """A file handed in as a ``.pln`` that the simulator did not write."""



__all__ = [
    "log",
    "status_log",
    "EMPTY_NOTICE_S",
    "Event",
    "EventSink",
    "announcement_roots",
    "_named_roots",
    "announcements_missing",
    "NotAFlightPlan",
    "hashlib",
    "logging",
    "math",
    "os",
    "threading",
    "time",
    "deque",
    "Callable",
    "Future",
    "dataclass",
    "field",
    "replace",
    "Any",
    "handling_spots",
    "interphone",
    "letters",
    "lod_windows",
    "ArrivalWatch",
    "AtisGenerator",
    "AtisReport",
    "AutonomousFlight",
    "create_autonomous_flight",
    "every_language",
    "heard_registration",
    "leaving_departure",
    "ControllerBrain",
    "Reply",
    "CrewDirector",
    "ClimbWatch",
    "DepartureWatch",
    "SidWatch",
    "by_words",
    "clearly_in",
    "language_of_text",
    "Handling",
    "fuselage_length",
    "Intent",
    "IntentParser",
    "ParsedIntent",
    "LandingMonitor",
    "LANGUAGE_NAMES",
    "languages_for",
    "profile_for",
    "reply_language",
    "Candidate",
    "Lod",
    "Aircraft",
    "Phraseology",
    "Weather",
    "wake_suffix",
    "FlightProfile",
    "RouteWatch",
    "fittings_for",
    "load_layout",
    "spots_for",
    "FlightSession",
    "Instruction",
    "Phase",
    "station_key",
    "systems_wanted",
    "TrafficInjector",
    "UserAircraft",
    "AirspaceWatch",
    "CrossingWatch",
    "DeviationAssistWatch",
    "RadarServiceWatch",
    "ReportingPointWatch",
    "RestrictionWatch",
    "RunwayWatch",
    "SurfaceWatch",
    "TaxiWatch",
    "TransponderWatch",
    "as_written",
    "ANNOUNCEMENTS",
    "AnnouncementLibrary",
    "ControllerMissing",
    "ControllersUnavailable",
    "imperfections_for",
    "CabinPlayer",
    "JoystickPushToTalk",
    "MicrophoneCapture",
    "PushToTalk",
    "RadioPlayer",
    "prosody_for",
    "PROFILE_ATIS",
    "PROFILE_HF",
    "Recognizer",
    "SpeechModelMissing",
    "engine_model",
    "tone_for",
    "Synthesizer",
    "VoicesMissing",
    "Config",
    "gsx",
    "SimTraffic",
    "DoorTable",
    "SimDoors",
    "SimFacilities",
    "SimVehicles",
    "gsx_installed",
    "GsxLink",
    "OnlineNetwork",
    "SimServicePoints",
    "SimAircraft",
    "SimSystems",
    "XPlaneTraffic",
    "build_provider",
    "Understanding",
    "RADIO",
    "decision",
    "timed",
    "is_valid_channel",
    "same_channel",
    "STATION_RANGE_NM",
    "NavDB",
    "Station",
    "haversine_nm",
    "normalize_runway",
    "variation",
    "announcements_dir",
    "command_hint",
    "_resolve",
    "resolve_secret",
    "autofly_limit",
    "capabilities",
    "ManualSource",
    "StateSource",
    "open_source",
    "AircraftState",
    "WeatherProvider",
    "WeatherSample",
    "at_the_surface",
    "in_the_circuit",
    "merge_weather",
    "near_the_field",
    "parse_metar",
    "weather_from_sim",
]
