"""The next-call hint, ported from StormATC's AtcGuidanceAdvisor.

Non-authoritative: it reads what the session already knows and suggests the
pilot's next radio call. It never changes session state, never grants a
clearance, and says nothing the deterministic engine has not already worked
out -- a hint that cannot steer the flight is a hint that cannot hurt it.
"""

from __future__ import annotations

from dataclasses import dataclass

from .session import Phase


@dataclass(frozen=True)
class Guidance:
    """What to say next: a title, the why, and the words themselves."""
    title: str
    detail: str
    example: str


def for_session(session, plan_known: bool, spoken_callsign: str,
                readback_due: list[str] = ()) -> Guidance:
    """The most useful next call for where the flight is."""
    if session is None:
        return Guidance("Wait for the simulator",
                        "MeridianATC needs a live aircraft state.", "")
    if not plan_known:
        return Guidance(
            "Import a flight plan",
            "Open Flight Plan and import your SimBrief OFP before "
            "calling ATC.", "")
    if readback_due:
        return Guidance(
            "Read back the clearance",
            "Still required: " + ", ".join(readback_due) + ".",
            ", ".join(readback_due))
    cs = spoken_callsign or "your callsign"
    phase = session.phase
    if phase == Phase.PREFLIGHT:
        rules = getattr(session, "flight_rules", "ifr")
        if rules == "vfr":
            return Guidance("Request VFR clearance",
                            "Call Delivery when you are ready to depart "
                            "under visual flight rules.",
                            f"{cs}, request VFR clearance")
        return Guidance("Request IFR clearance",
                        "Call Delivery for your clearance before asking "
                        "for pushback or taxi.",
                        f"{cs}, request clearance to destination")
    if phase == Phase.CLEARED:
        return Guidance("Request pushback and start",
                        "Call Ground for pushback when the clearance is "
                        "copied and the aircraft is ready.",
                        f"{cs}, request pushback and start")
    if phase == Phase.PUSHBACK:
        return Guidance("Request taxi",
                        "When the tug is clear, ask Ground for taxi.",
                        f"{cs}, ready for taxi")
    if phase == Phase.TAXI_OUT:
        return Guidance("Report holding short",
                        "At the runway, hold short and report to Tower.",
                        f"{cs}, holding short")
    if phase == Phase.HOLDING_SHORT:
        return Guidance("Report ready for departure",
                        "Tell Tower you are ready for takeoff.",
                        f"{cs}, ready for departure")
    if phase == Phase.DEPARTURE:
        return Guidance("Follow the departure",
                        "Fly the cleared heading and altitude; "
                        "departure control will hand you on.", "")
    if phase == Phase.ENROUTE:
        return Guidance("Maintain assigned level",
                        "Monitor the frequency; report when asked or "
                        "request a level change when needed.", "")
    if phase == Phase.DESCENT:
        return Guidance("Prepare the approach",
                        "Brief the arrival and request the approach when "
                        "in range.",
                        f"{cs}, request approach")
    if phase == Phase.APPROACH:
        return Guidance("Report established",
                        "Report established on the approach when "
                        "cleared.",
                        f"{cs}, established")
    if phase == Phase.LANDING:
        return Guidance("Vacate when clear",
                        "Land, slow, and vacate at the first available "
                        "taxiway; report vacated.",
                        f"{cs}, runway vacated")
    if phase in (Phase.TAXI_IN, Phase.LANDED):
        return Guidance("Request taxi to stand",
                        "Ask Ground for taxi to parking.",
                        f"{cs}, request taxi to stand")
    return Guidance("Monitor the frequency", "", "")


__all__ = ["Guidance", "for_session"]
