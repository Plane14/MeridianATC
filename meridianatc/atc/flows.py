"""Which runways the airport is using, from the scenery's own rules.

X-Plane does not publish the runway its ATC is using on any dataref -- the
choice is made inside the simulator and never written out. What it does
publish is the data the choice is made from: the airport traffic flows in
every apt.dat (row codes 1000 onward). A flow is the scenery author's
"when the weather is like this, we run like that", with the runways in use
and what each is for written out as runway-in-use rules.

The structure, per the apt.dat specification:

* a flow header (row 1000) opens a named flow;
* wind (1001), ceiling (1002), visibility (1003) and time (1004) rules say
  when the flow applies. Rules of one class 'or' together, every class must
  pass, and the first flow whose rules all pass is the one in use;
* runway-in-use rules (1100, or 1110 for 8.33 kHz fields) name a runway
  end, whether it takes arrivals, departures or both, the aircraft classes
  it serves, and two heading ranges -- the on-course range (the last leg
  of an arrival's plan, the first of a departure's) and, for departures,
  the initial heading ATC will assign. Several rules are in use at once;
  each aircraft is sent to the first rule it matches, which is how one
  flow lands jets on one parallel and heavies on another;
* a VFR pattern rule (1101) fixes the circuit direction for a runway.

Seattle's own file is the shape of the thing: ``Calm and South flow``
arrives on 16C for traffic inbound from the north, 16R from the south,
and 16L for heavies -- three strips working at once, chosen by where the
aeroplane is coming from.

This module holds the parsed model and the evaluation; the apt.dat
parsing itself lives in :mod:`meridianatc.integrations.xpground`, and the
consumer is the traffic field (``_Field.departure_runways`` and friends).
MSFS has nothing like it -- its facility data is geometry only -- so the
non-flow paths fall back to the same wind-and-preference logic as before.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# The aircraft classes apt.dat rules name, and which of our own classes
# each maps to (meridianatc.atc.aircraft_class). "gliders" has no flying
# model behind it yet; it stays so a rule written for them still parses.
XP_TYPES = ("heavy", "jets", "turboprops", "props", "helos", "gliders")

_OURS_TO_XP = {
    "piston_single": "props",
    "piston_twin": "props",
    "turboprop_single": "turboprops",
    "turboprop_twin": "turboprops",
    "business_jet": "jets",
    "airliner": "jets",
    "helicopter": "helos",
}

# The wake-heavy types an apt.dat "heavy" rule really means -- widebodies.
# A 737 is a jet that is not heavy; an A388 is both, and Seattle sends it
# to the heavy strip. Kept beside aircraft_class's own list in spirit;
# that one is about needing two usable runways, this one about which rule
# X-Plane would match.
_HEAVIES = frozenset({
    "A332", "A333", "A339", "A342", "A343", "A345", "A346", "A359", "A35K",
    "A388", "B742", "B744", "B748", "B763", "B764", "B772", "B773", "B77L",
    "B77W", "B77X", "B788", "B789", "B78X", "MD11", "A124", "AN22", "C5M",
    "C17", "IL76", "IL96", "DC10", "L101",
})


def xp_type(class_name: str, type_code: str = "") -> str:
    """The apt.dat class a flight of ours matches in a runway-in-use rule."""
    code = (type_code or "").strip().upper()
    if class_name == "airliner" and code in _HEAVIES:
        return "heavy"
    return _OURS_TO_XP.get(class_name, "props")


def in_range(heading: float, start: float, end: float) -> bool:
    """Whether ``heading`` sits in the (possibly wrapping) range start..end.

    apt.dat ranges wrap through north -- "341 159" means 341-359 and
    000-159 -- so the range is a slice of the circle, not of the number
    line.
    """
    heading = float(heading) % 360.0
    start, end = float(start) % 360.0, float(end) % 360.0
    if end == 359.0:
        # "through 359" means to the top of the compass; a course of
        # 359.5 is inside a 000-359 'anywhere' rule.
        end = 360.0
    if start <= end:
        return start <= heading <= end
    return heading >= start or heading <= end


@dataclass(frozen=True)
class RunwayUse:
    """One runway-in-use rule: an end, its job, and who it serves.

    ``course`` is the on-course heading range (an arrival's last leg, a
    departure's first); ``departure`` the initial-heading range for
    departures only. Both are (from, to) degrees, wrapping like
    :func:`in_range`; 000/359 pairs from scenery authors mean 'any'.
    """

    end: str
    frequency: float = 0.0          # MHz; the runway's own tower, kept for later
    arrivals: bool = False
    departures: bool = False
    types: frozenset = frozenset()
    course: tuple = (0.0, 359.0)
    departure: tuple = (0.0, 359.0)
    name: str = ""

    def serves(self, xp_class: str) -> bool:
        """Whether this rule takes an aircraft of the given apt.dat class.

        An empty list is the scenery author's "everything"; otherwise the
        class must be named exactly -- 'heavy' is its own class in the
        taxonomy, which is how Seattle lands its widebodies on a runway
        the rule for 'jets' does not name.
        """
        return not self.types or xp_class in self.types

    def covers(self, course: float, departing: bool) -> bool:
        """Whether the aircraft's route bearing falls inside this rule.

        The initial-heading half of a departure rule is the vector ATC
        will assign after takeoff, which is not known when the strip is
        written, so the on-course range -- the first leg's direction --
        does the choosing.
        """
        return in_range(course, *self.course)


@dataclass
class Flow:
    """One traffic flow: when it applies, and the runways it uses."""

    name: str = ""
    wind: list = field(default_factory=list)       # (dir_min, dir_max, max_kt)
    ceiling_ft: int | None = None
    visibility_sm: float | None = None
    times: list = field(default_factory=list)      # (start_hhmm, end_hhmm)
    runways: list = field(default_factory=list)    # RunwayUse
    pattern: dict = field(default_factory=dict)    # end -> "left"/"right"

    def passes(self, weather, minute_utc: int | None = None) -> bool:
        """Whether this flow is the one the weather and clock select.

        The spec's AND/OR: a class with no rules passes; within a class any
        passing rule passes the class. A None ``minute_utc`` (the sim does
        not report a clock to us) takes the time rules as met, the way
        X-Plane's own AI treats a flow with no clock to check.
        """
        if self.wind:
            if weather is None:
                return False
            direction = (float(weather.wind_dir)
                         - float(getattr(weather, "variation", 0.0) or 0.0)) % 360
            speed = float(weather.wind_kt or 0.0)
            variable = bool(getattr(weather, "variable", False))
            if not any(speed <= high
                       and (variable or in_range(direction, low, hi_dir))
                       for low, hi_dir, high in self.wind):
                return False
        if self.ceiling_ft is not None and weather is not None:
            layers = [h for code, h in getattr(weather, "clouds", []) or []
                      if str(code).upper() in ("BKN", "OVC", "VV")]
            if layers and min(layers) < self.ceiling_ft:
                return False
        if self.visibility_sm is not None and weather is not None:
            if float(getattr(weather, "visibility_sm", 10.0) or 10.0) \
                    < self.visibility_sm:
                return False
        if self.times and minute_utc is not None:
            if not any((start <= minute_utc <= end) if start <= end
                       else (minute_utc >= start or minute_utc <= end)
                       for start, end in self.times):
                return False
        return True

    def ends(self, departing: bool) -> list[str]:
        """Every runway end the flow has working this direction, in order."""
        seen, found = set(), []
        for use in self.runways:
            if (departing and use.departures) \
                    or (not departing and use.arrivals):
                if use.end not in seen:
                    seen.add(use.end)
                    found.append(use.end)
        return found

    def assign(self, departing: bool, xp_class: str = "",
               course: float | None = None) -> str | None:
        """The runway end this aircraft's rule match gives it.

        X-Plane's own order: the first runway-in-use rule the aircraft
        matches -- class, role and course range -- wins. Without a course
        to test (a circuit flight, a filed route we cannot see) the first
        class-matching rule stands in, and without a class the first rule
        for the direction.
        """
        role = [u for u in self.runways
                if (u.departures if departing else u.arrivals)]
        if xp_class:
            served = [u for u in role if u.serves(xp_class)]
        else:
            served = role
        if course is not None:
            for use in served:
                if use.covers(course, departing):
                    return use.end
        return served[0].end if served else None


def pick_flow(flows, weather, minute_utc: int | None = None):
    """The flow the current weather selects, or None.

    Flows are evaluated in order and the first to pass wins; a field whose
    scenery defines no passing flow gets None, and X-Plane's own AI then
    invents a flow the same way -- the caller falls back to wind-picked
    runways, which is the same answer for a single-strip field.
    """
    for flow in flows or ():
        if flow.passes(weather, minute_utc):
            return flow
    return None


def parse_course_range(raw: str) -> tuple:
    """"160340" -> (160.0, 340.0): two headings packed into six digits."""
    digits = "".join(c for c in (raw or "") if c.isdigit())
    if len(digits) >= 6:
        return float(digits[:3]), float(digits[3:6])
    if len(digits) >= 3:
        return float(digits[:3]), 359.0
    return 0.0, 359.0


__all__ = ["Flow", "RunwayUse", "XP_TYPES", "in_range",
           "parse_course_range", "pick_flow", "xp_type"]
