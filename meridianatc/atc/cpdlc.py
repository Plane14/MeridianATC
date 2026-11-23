"""Controller-pilot data link: the wire the clearances come down on.

Where the voice circuit does not reach -- over the ocean before anything
else -- the controller and the flight deck talk in messages instead:
an uplink carries the instruction, a downlink answers it, and a WILCO
is the readback the way one is owed on the radio. This module is the
message model and the link state; the engine decides when a flight
carries the link (oceanic, and the aircraft must have the box), and
the panel is where the pilot's answers are clicked.

The semantics are the radio's, not a chat's: an uplink with readback
owed stays pending until WILCO settles it exactly as a spoken readback
would, UNABLE hands the decision back to the controller, and ROGER
acknowledges what needed acknowledging.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any

# The classes of aircraft that carry the box: the airlines, the business
# jets and the bigger turboprops. A Cessna on the way to the coast does
# not have CPDLC, and pretending it does is how an oceanic clearance
# ends up unread.
EQUIPPED = frozenset({"airliner", "business_jet", "turboprop_twin"})

# What may be answered: WILCO and UNABLE for anything owed, ROGER for
# what only needed acknowledging. STANDBY exists but buys nothing here.
ANSWERS = ("wilco", "unable", "roger")

# An UNABLE does not make the clearance wrong, it makes this pilot
# unable: the same instruction is not offered again inside this many
# seconds, or an exchange becomes a loop with no way out of it.
REFUSED_REPEAT_S = 120.0


def equipped(session) -> bool:
    """Whether this flight carries the box, by what it is."""
    kind = getattr(getattr(session, "profile", None), "kind", "") or ""
    return kind in EQUIPPED


@dataclass
class Message:
    """One element in the exchange, in either direction."""

    id: int
    direction: str                        # "up" | "down"
    text: str
    at: float = field(default_factory=time.monotonic)
    # How it ended: "wilco", "unable", "roger", "expired"; "" while open.
    closed: str = ""
    # The instruction an uplink delivers, so a WILCO can settle it.
    instruction: Any = None

    def view(self) -> dict[str, Any]:
        return {"id": self.id, "direction": self.direction,
                "text": self.text, "closed": self.closed,
                "at": self.at}


@dataclass
class CpdlcLink:
    """The link to one data authority, while the flight is on it.

    Held on the session: the authority is the station the logon is with,
    and an uplink is only valid for instructions that station issued --
    a different sector's call still comes over the voice circuit.
    """

    authority: str = ""
    logged: bool = True
    next_id: int = 1
    messages: list[Message] = field(default_factory=list)
    # The uplink awaiting a downlink, and what an UNABLE already refused
    # (instruction kind -> when), so a refused clearance is not re-sent
    # inside the same exchange.
    pending: Message | None = None
    refused: dict[str, float] = field(default_factory=dict)

    def takes(self, instruction) -> bool:
        """Whether this instruction rides the link rather than the radio."""
        if not self.logged or self.pending is not None:
            return False
        if instruction.station_key != self.authority:
            return False
        if not instruction.readback_required:
            return False
        refused = self.refused.get(instruction.kind)
        return refused is None or refused < time.monotonic() - REFUSED_REPEAT_S

    def uplink(self, instruction) -> Message:
        """Send an instruction down to the flight deck, pending its answer."""
        msg = Message(id=self.next_id, direction="up",
                      text=instruction.text, instruction=instruction)
        self.next_id += 1
        instruction.delivery = "cpdlc"
        self.messages.append(msg)
        self.pending = msg
        return msg

    def respond(self, answer: str, session=None) -> Message | None:
        """The downlink: the pilot's answer to the open uplink.

        ``wilco`` settles the instruction the way a spoken readback does
        -- read back, applied, done. ``unable`` refuses it: the clearance
        never existed, the kind is remembered so it is not offered again
        immediately, and the controller's next decision is its own.
        ``roger`` closes what only wanted acknowledging, without applying
        it. Returns the downlink message, or None when nothing is open.
        """
        msg = self.pending
        if msg is None or answer not in ANSWERS:
            return None
        instruction = msg.instruction
        if answer == "roger" and instruction is not None \
                and getattr(instruction, "readback_required", True):
            # ROGER acknowledges; it is not compliance. An uplink whose
            # instruction still needs a readback stays open for WILCO or
            # UNABLE, exactly as the spoken "roger" settles nothing.
            return None
        msg.closed = answer
        self.pending = None
        self.messages.append(Message(
            id=self.next_id, direction="down", text=answer.upper()))
        self.next_id += 1
        if instruction is not None:
            if answer in ("wilco", "roger"):
                instruction.read_back = True
                if answer == "wilco" and session is not None:
                    session.apply(instruction.values)
            else:
                self.refused[instruction.kind] = time.monotonic()
                instruction.readback_required = False
            if session is not None:
                session.clear_pending()
        return self.messages[-1]

    def view(self, limit: int = 20) -> dict[str, Any]:
        """What the panel draws: the link, the open uplink, the log."""
        return {
            "logged": self.logged,
            "authority": self.authority,
            "pending": self.pending.view() if self.pending else None,
            "messages": [m.view() for m in self.messages[-limit:]],
        }


def open_link(authority: str) -> CpdlcLink:
    """Log on with a data authority."""
    return CpdlcLink(authority=authority)


__all__ = ["ANSWERS", "CpdlcLink", "EQUIPPED", "Message", "equipped",
           "open_link"]
