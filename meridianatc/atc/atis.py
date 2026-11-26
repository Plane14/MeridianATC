"""Automatic Terminal Information Service.

The ATIS is the first thing a pilot listens to and the easiest thing to get
visibly wrong, because it states the runway in use, the pressure setting and
the information letter that the controller will later expect to hear read back.
All three have to agree with what the rest of the system believes.

The broadcast is assembled in the published order (FAA JO 7110.65 2-9-3 for the
US form, ICAO Annex 11 for the international one) and re-issued with the next
letter of the alphabet whenever the conditions behind it change materially --
not on a timer, because a controller does not roll the letter for a one-knot
wind shift.
"""

from __future__ import annotations

import logging
import re
import time
from dataclasses import dataclass, field
from datetime import UTC, datetime

from ..navdata.db import Airport, NavDB
from ..navdata.db import transition_altitude as transition_for
from . import speech as sp
from .phraseology import Phraseology, Weather

log = logging.getLogger(__name__)

# The information letter cycles through the phonetic alphabet.
LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"


@dataclass
class AtisReport:
    """One issued ATIS, with everything derived from it."""

    airport: str
    letter: str
    text: str
    weather: Weather
    landing_runways: list[str] = field(default_factory=list)
    departing_runways: list[str] = field(default_factory=list)
    issued_utc: datetime = field(default_factory=lambda: datetime.now(UTC))
    issued_monotonic: float = field(default_factory=lambda: time.monotonic())

    @property
    def letter_spoken(self) -> str:
        return sp.letters(self.letter)

    approach: str = ""

    @property
    def approach_in_use(self) -> str:
        return self.approach if self.landing_runways else ""


def _round_to(value: float, step: int) -> int:
    return int(round(float(value) / step) * step)


class AtisGenerator:
    """Builds and maintains the ATIS for one airport."""

    def __init__(
        self,
        navdb: NavDB,
        airport: Airport,
        phraseology: Phraseology | None = None,
        *,
        transition_altitude: int = 0,
        start_letter: str | None = None,
        letter_state: dict | None = None,
    ):
        self.navdb = navdb
        self.airport = airport
        self.phrase = phraseology or Phraseology(airport.dialect)
        # Where the airport is, unless the caller knows better. It used to be
        # eighteen thousand feet whatever the airport was, which is the
        # American figure and made a European ATIS quote a transition level
        # nobody there uses.
        self.transition_altitude = (
            transition_altitude or transition_for(airport.ident))
        first = (
            LETTERS.index(start_letter.upper())
            if start_letter and start_letter.upper() in LETTERS
            else self._letter_from_clock()
        )
        # The letter belongs to the aerodrome, not to the language a
        # broadcast was rendered in. Engines serving English and Spanish
        # ATIS share this mutable state, so a material change advances the
        # letter once and every language then composes the same one.
        self._letter_state = letter_state if letter_state is not None else {}
        self._letter_state.setdefault("index", first)
        self._letter_state.setdefault("current", "")
        self._letter_state.setdefault("revision", 0)
        self._letter_revision = -1
        self.current: AtisReport | None = None
        # Which approach a landing runway has, from the same place the
        # controllers ask (``ControllerBrain.approach_for``), so the broadcast
        # never promises an ILS the clearance will not give. The ILS where the
        # engine has not said.
        self.approach_for = lambda runway: "ILS"
        # And the runway, from the same place. Unset, the wind decides.
        self.runway_for = None
        # The whole set in use, where the caller knows it (the traffic's
        # own runway lists, scenery flows included). Unset, the runway
        # above is expanded from the database the way it always was.
        self.runways_for = None
        # The VFR part of the departure briefing for a runway: ``(side,
        # [point, ...])`` -- the published circuit side, "" where none is
        # published, and the reporting points a controller would name.
        # Unset, the broadcast has none, as it had before.
        self.vfr_briefing = None

    def _letter_from_clock(self) -> int:
        """Start on a plausible letter rather than always on Alpha."""
        now = datetime.now(UTC)
        return (now.hour * 2 + now.minute // 30) % 26

    # ------------------------------------------------------------------

    def update(self, weather: Weather, force: bool = False) -> AtisReport:
        """Issue a new ATIS if conditions have moved enough to warrant one."""
        if self.current is not None and not force:
            # The runway the controllers are using now, against the one the
            # broadcast named: with the runway kept until the wind takes it
            # away (ControllerBrain._wind_runway), the old weather no longer
            # gives the old runway, so comparing the two weathers missed it.
            moved = (self.current.landing_runways
                     != list(self._runway_choice(weather))
                     or self.current.departing_runways
                     != list(self._runway_choice(weather, True)))
            if not moved and not self._materially_changed(self.current.weather,
                                                          weather):
                return self.current
        return self.issue(weather)

    def _materially_changed(self, old: Weather, new: Weather) -> bool:
        """Whether the change justifies a new information letter.

        The thresholds mirror what actually triggers a re-record: a pressure
        change of a hundredth of an inch, a wind shift big enough to matter for
        runway selection, a visibility or ceiling change that crosses a
        reportable step.
        """
        if abs(old.altimeter_inhg - new.altimeter_inhg) >= 0.01:
            return True
        if abs(old.wind_kt - new.wind_kt) >= 5:
            return True
        if abs(((old.wind_dir - new.wind_dir) + 540) % 360 - 180) >= 30 and new.wind_kt >= 5:
            return True
        if _round_to(old.visibility_sm, 1) != _round_to(new.visibility_sm, 1):
            return True
        # Ceilings, not just coverage: BKN008 to BKN045 is a new letter.
        if [c[:2] for c in old.clouds] != [c[:2] for c in new.clouds]:
            return True
        # A runway change always forces a new letter; update() already
        # compares the broadcast's runways against the live choice, which
        # this cannot do -- a stateful runways_for answers both weathers
        # the same way.
        return abs(old.temperature_c - new.temperature_c) >= 2

    def _runway_choice(self, weather: Weather,
                       departing: bool = False) -> tuple[str, ...]:
        if self.runways_for is not None:
            found = self.runways_for(weather, departing)
            if found:
                return tuple(found)
        if self.runway_for is not None:
            end = self.runway_for(weather, departing)
            if not end:
                return ()
        else:
            best = self.navdb.best_runway(
                self.airport.ident, weather.wind_dir, weather.wind_kt,
                variable=weather.variable, departing=departing,
                gust_kt=weather.gust_kt,
            )
            if not best:
                return ()
            end, _rw = best
        # The ends the field really works at once -- the curated '+'
        # groups and, where this is a scheduled-service field, the chosen
        # runway's parallels -- announced as they are used.
        airport = self.navdb.airport(self.airport.ident)
        big = bool(getattr(airport, "scheduled", False)
                   or getattr(airport, "size_rank", 0) >= 3)
        active = self.navdb.active_runways(
            self.airport.ident, end, departing, parallels=big)
        if active:
            return tuple(active)
        parallels = self.navdb.parallel_runways(self.airport.ident, end)
        return tuple([end] + sorted(parallels))

    # ------------------------------------------------------------------

    def issue(self, weather: Weather) -> AtisReport:
        """Force a new ATIS with the aerodrome's current letter."""
        state = self._letter_state
        if state["current"] and self._letter_revision != state["revision"]:
            # Another language's generator already rolled this revision.
            letter = state["current"]
        else:
            letter = LETTERS[state["index"] % 26]
            state["index"] += 1
            state["current"] = letter
            state["revision"] += 1
        self._letter_revision = state["revision"]

        runways = list(self._runway_choice(weather))
        # A field with a preferential system lands on one runway and departs
        # from another (Brussels 25L and 25R), and the broadcast has to name
        # the one the departure will be cleared from.
        departing = list(self._runway_choice(weather, departing=True)) or runways
        text = self.compose(letter, weather, runways, departing)

        report = AtisReport(
            airport=self.airport.ident,
            letter=letter,
            text=text,
            weather=weather,
            landing_runways=runways,
            departing_runways=departing,
            approach=self._approach_kind(runways),
        )
        self.current = report
        return report

    # ------------------------------------------------------------------

    def compose(self, letter: str, wx: Weather, runways: list[str],
                departing: list[str] | None = None) -> str:
        """Assemble the spoken broadcast."""
        split = bool(runways and departing and departing[0] != runways[0])
        if getattr(self.phrase, "dialect", "") == "fr":
            return self._compose_french(letter, wx, runways,
                                        departing if split else None)
        p = self.phrase
        now = self._obs_time(wx)
        name = self.airport.spoken

        parts: list[str] = []

        # --- identification and time ---
        if p.dialect == "faa":
            parts.append(
                f"{name} airport information {sp.letters(letter)}, "
                f"{sp.time_utc(now.hour, now.minute, p.digit_dialect)}"
            )
        else:
            parts.append(
                f"This is {name} information {sp.letters(letter)}, "
                f"{sp.time_utc(now.hour, now.minute, p.digit_dialect)}"
            )

        # --- the weather, first in both dialects: JO 7110.65 2-9-3 puts
        # it before the approach and runways, and Annex 11 reads the
        # same way ---
        weather = [p.wind(wx)]
        weather.extend(self._visibility(wx))
        weather.append(self._clouds(wx))
        weather.append(
            f"temperature {self._signed(wx.temperature_c)}, "
            f"dew point {self._signed(wx.dewpoint_c)}"
        )
        weather.append(p.pressure(
            wx.altimeter_inhg if p.dialect == "faa" else wx.qnh_hpa
        ))
        parts.extend(weather)

        # --- runways and the approach in use ---
        if split:
            landing = " and ".join(p.rwy(r) for r in runways)
            taking_off = " and ".join(p.rwy(r) for r in departing)
            if p.dialect == "icao":
                # Annex 11 order: the approach, then the runway.
                parts.append(
                    f"expect {self._approach_type(runways)} approach")
            parts.append(
                f"landing runway{'s' if len(runways) > 1 else ''} "
                f"{landing}, departure runway"
                f"{'s' if len(departing) > 1 else ''} {taking_off}")
        elif runways:
            if p.dialect == "icao":
                parts.append(
                    f"expect {self._approach_type(runways)} approach")
            if len(runways) == 1:
                parts.append(f"landing and departing runway {p.rwy(runways[0])}")
            else:
                spoken = " and ".join(p.rwy(r) for r in runways)
                parts.append(f"landing and departing runways {spoken}")
        if runways and p.dialect == "faa":
            # JO 7110.65 2-9-3: the instrument approach in use, by name.
            parts.append(f"{self._approach_type(runways)} runway "
                         f"{p.rwy(runways[0])} approach in use")
        parts.extend(self._vfr_part(runways))

        # --- transition level: ATIS item h for every ICAO broadcast ---
        if p.dialect != "faa" and self.transition_altitude:
            level = self._transition_level(wx)
            if level:
                parts.append(f"transition level {sp.flight_level(level)}")

        # --- remarks ---
        parts.extend(self._remarks(wx))

        # --- closing ---
        if p.dialect == "faa":
            parts.append(
                f"advise on initial contact you have information {sp.letters(letter)}"
            )
        else:
            parts.append(f"acknowledge receipt of information {sp.letters(letter)}")

        return ". ".join(part.strip().rstrip(".") for part in parts if part.strip()) + "."

    def _vfr_part(self, runways: list[str], french: bool = False) -> list[str]:
        """The circuit and the reporting points, for the VFR departures.

        The FAA's ATIS names a non-standard pattern ("right traffic") and no
        reporting points, which are codes nobody says in the US; elsewhere
        the hand of the circuit and the points are said. UNVERIFIED: the
        words, and that a UK or French ATIS gives the points. No noise
        abatement is said: nothing the program reads publishes it.
        """
        if not runways or self.vfr_briefing is None:
            return []
        try:
            side, points = self.vfr_briefing(runways[0])
        except Exception:
            log.debug("VFR briefing failed", exc_info=True)
            return []
        p = self.phrase
        said: list[str] = []
        if french:
            if side:
                said.append(f"tour de piste {'à droite' if side == 'right' else 'à gauche'} "
                            f"piste {p.rwy(runways[0])}")
            if points:
                said.append("points de report VFR " + ", ".join(points))
            return said
        if p.dialect == "faa":
            if side == "right":
                said.append(f"right traffic runway {p.rwy(runways[0])}")
            return said
        if side:
            said.append(f"{side}-hand circuit runway {p.rwy(runways[0])}")
        if points:
            said.append("VFR reporting points " + ", ".join(points))
        return said

    @staticmethod
    def _remarks(wx: Weather) -> list[str]:
        """The notices the weather itself makes necessary.

        A real ATIS carries what the day asks for: low visibility
        procedures when the field is working to them, a wet runway when
        it is raining on it, a braking-action caution when what is
        falling freezes on it, a thunderstorm pointed at when there is
        one. Each is derived from the report, not invented -- a calm
        clear ATIS has no remarks to make, same as a real one.
        """
        present = (wx.present or "").upper()
        remarks: list[str] = []
        # Low visibility procedures: what a field below CAT I weather is
        # actually running, said the way ATISes say it.
        ceiling = min((h for cover, h in (wx.clouds or [])
                       if cover in ("BKN", "OVC", "VV")), default=None)
        if wx.visibility_sm < 0.5 \
                or (ceiling is not None and ceiling < 200):
            remarks.append("low visibility procedures in effect")
        # Wet, then freezing: the two braking-action stories a crew is
        # briefed for, in the order a real broadcast hedges them.
        freezing = any(tag in present
                       for tag in ("FZRA", "FZDZ", "FZFG", "SN", "SG", "PL"))
        precip = any(tag in present
                     for tag in ("RA", "DZ", "SN", "SG", "PL", "GR",
                                 "GS", "SH", "UP"))
        if freezing or (precip and wx.temperature_c <= 1.0):
            remarks.append("caution, braking action may be reduced")
        elif precip:
            remarks.append("runways wet")
        # A thunderstorm on the field is never just cloud.
        if wx.cb or "TS" in present:
            remarks.append("thunderstorm in the vicinity")
        return remarks

    def _compose_french(self, letter: str, wx: Weather,
                        runways: list[str],
                        departing: list[str] | None = None) -> str:
        """The French form of the broadcast.

        Order and wording follow the French AIP rather than being a
        translation: the identification carries the airport and the letter, the
        runway in use comes next, and the closing asks the pilot to acknowledge
        the letter on first contact.
        """
        from . import speech_fr as sfr

        p = self.phrase
        now = self._obs_time(wx)
        parts: list[str] = [
            f"Ici {self.airport.spoken}, information {sfr.letters(letter)}, "
            f"{sfr.time_utc(now.hour, now.minute)}"
        ]

        if runways and departing:
            # The form heard on French ATIS; not checked against a DGAC
            # publication.
            parts.append(f"piste en service à l'atterrissage {p.rwy(runways[0])}, "
                         f"au décollage {p.rwy(departing[0])}")
            parts.append(f"approche {self._approach_type(runways)} à prévoir")
        elif runways:
            if len(runways) == 1:
                parts.append(f"piste en service {p.rwy(runways[0])}")
            else:
                spoken = " et ".join(p.rwy(r) for r in runways)
                parts.append(f"pistes en service {spoken}")
            parts.append(f"approche {self._approach_type(runways)} à prévoir")
        parts.extend(self._vfr_part(runways, french=True))

        parts.append(p.wind(wx))
        parts.append(sfr.visibility(wx.visibility_sm))
        parts.append(self._clouds_french(wx))
        parts.append(
            f"température {sfr.temperature(wx.temperature_c)}, "
            f"point de rosée {sfr.temperature(wx.dewpoint_c)}"
        )
        parts.append(p.pressure(wx.qnh_hpa))
        if self.transition_altitude:
            level = self._transition_level(wx)
            if level:
                parts.append(f"niveau de transition {sfr.flight_level(level)}")
        parts.append(f"accusez réception de l'information {sfr.letters(letter)}")

        return ". ".join(part.strip().rstrip(".") for part in parts if part.strip()) + "."

    @staticmethod
    def _obs_time(wx: Weather) -> datetime:
        """The weather's own time where the report gives one: a METAR's
        observation group, so the time and the weather agree. The
        compose moment where it does not -- sim-only weather has no
        observation time to quote."""
        stamp = getattr(wx, "observed_utc", "") or ""
        m = re.fullmatch(r"(\d{2})(\d{2})(\d{2})Z", stamp)
        if m:
            now = datetime.now(UTC)
            day, hour, minute = (int(part) for part in m.groups())
            # A METAR stamp has no month. Try the current month and the
            # adjacent ones, then take the nearest valid date: on June 1,
            # 312350Z is May 31, not an invalid June 31 or a distant June 1.
            choices = []
            month_index = now.year * 12 + now.month - 1
            for offset in (-1, 0, 1):
                year, month0 = divmod(month_index + offset, 12)
                try:
                    choices.append(datetime(year, month0 + 1, day, hour,
                                            minute, tzinfo=UTC))
                except ValueError:
                    continue
            if choices:
                return min(choices, key=lambda when: abs((when - now).total_seconds()))
        return datetime.now(UTC)

    def _visibility(self, wx: Weather) -> list[str]:
        """The visibility item, or CAVOK when the sky allows it.

        Annex 10: CAVOK when the visibility is ten kilometres or better,
        no cloud below five thousand feet or the highest minimum sector
        altitude, no cumulonimbus or towering cumulus, and no significant
        weather -- said instead of the visibility and the cloud.
        """
        p = self.phrase
        if p.dialect == "faa":
            return [sp.visibility(wx.visibility_sm, p.digit_dialect)]
        metres = int(min(9999, round(wx.visibility_sm * 1609.34 / 100.0) * 100))
        low_cloud = any(h < 5000 for _cover, h in (wx.clouds or []))
        if metres >= 9999 and not low_cloud and not wx.cb \
                and not (wx.present or "").strip():
            return ["CAVOK"]
        if metres >= 9999:
            return ["visibility one zero kilometres or more"]
        # Whole hundreds and thousands are read as such, not digit by
        # digit: two thousand metres, not two zero zero zero.
        if metres % 1000 == 0:
            words = f"{sp.cardinal(metres // 1000)} thousand"
        elif metres % 100 == 0:
            words = f"{sp.cardinal(metres // 100)} hundred"
        else:
            words = p.d(metres)
        return [f"visibility {words} metres"]

    def _transition_level(self, wx: Weather) -> int:
        """The transition level in flight levels, from the airport's
        transition altitude and the QNH: the altitude plus five hundred
        feet for every step the pressure is below standard, which is how
        the lowest usable flight level stays clear of it."""
        base = int(self.transition_altitude)
        if base <= 0:
            return 0
        level = base // 100
        qnh = float(wx.qnh_hpa or 1013.25)
        if qnh < 1013.25:
            level += 5
        if qnh < 980.0:
            level += 5
        return level

    def _clouds_french(self, wx: Weather) -> str:
        from . import speech_fr as sfr

        words = {
            "FEW": "quelques nuages à", "SCT": "nuages épars à",
            "BKN": "nuages fragmentés à", "OVC": "ciel couvert à",
            "VV": "plafond indéfini à",
        }
        if not wx.clouds:
            return "nuages non détectés"
        spoken = []
        for cover, height in wx.clouds[:3]:
            spoken.append(f"{words.get(cover, 'nuages à')} {sfr.height(height)}")
        return ", ".join(spoken)

    def _approach_kind(self, runways: list[str]) -> str:
        return self.approach_for(runways[0]) if runways else ""

    def _approach_type(self, runways: list[str] | None = None) -> str:
        """The approach to expect, as the broadcast says it."""
        runways = runways if runways is not None else (
            self.current.landing_runways if self.current else [])
        kind = self._approach_kind(runways) or "ILS"
        return self.phrase.approach_name(kind) if hasattr(
            self.phrase, "approach_name") else kind

    def _clouds(self, wx: Weather) -> str:
        """Cloud layers, or the phrase used when there are none."""
        from ..weather.metar import CLOUD_WORDS

        if not wx.clouds:
            return "sky clear" if self.phrase.dialect == "faa" else "no cloud detected"
        spoken = []
        for cover, height in wx.clouds[:3]:
            words = CLOUD_WORDS.get(cover, "clouds at")
            if cover == "VV":
                spoken.append(f"indefinite ceiling {self.phrase.alt(height)}")
            else:
                spoken.append(f"{words} {self.phrase.alt(height, 99999)}")
        return ", ".join(spoken)

    def _signed(self, celsius: float) -> str:
        value = int(round(celsius))
        if value < 0:
            return f"minus {self.phrase.d(abs(value))}"
        return self.phrase.d(value)


def letter_from_text(text: str) -> str:
    """Recover the information letter a pilot reported having.

    Kept for callers that add it back; nothing in the program uses it
    today.
    """
    from .speech import _ALPHABET_REVERSE

    words = [w.strip(".,").lower() for w in (text or "").split()]
    for _i, word in enumerate(words):
        if word in _ALPHABET_REVERSE:
            letter = _ALPHABET_REVERSE[word]
            if len(letter) == 1 and letter.isalpha():
                return letter
    return ""
