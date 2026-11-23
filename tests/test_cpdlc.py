"""The data link: uplinks, downlinks, and who carries the box.

The link's semantics are the radio's -- a WILCO settles an instruction
the way a spoken readback does, an UNABLE hands it back to the
controller, and a Cessna has no box to uplink to.
"""

import time
from types import SimpleNamespace

from meridianatc.atc.cpdlc import (
    REFUSED_REPEAT_S,
    equipped,
    open_link,
)
from meridianatc.atc.session import FlightSession, Instruction


def _session(kind="airliner"):
    session = FlightSession.__new__(FlightSession)
    session.aircraft = SimpleNamespace(language="en")
    session.profile = SimpleNamespace(kind=kind)
    session.cpdlc = None
    session.pending = None
    session.last_instruction = None
    session.last_clearance = None
    session.applied = []
    session.apply = lambda values: session.applied.append(values)
    return session


def _instruction(kind="descend", authority="Shanwick Radio",
                 readback=True, values=None):
    return Instruction(
        kind=kind, station_key=authority, text="DESCEND FL280",
        values=values or {"altitude_ft": 28000},
        readback_required=readback)


class TestEquipage:
    def test_airliner_carries_the_box(self):
        assert equipped(_session("airliner"))

    def test_piston_single_does_not(self):
        assert not equipped(_session("piston_single"))

    def test_missing_profile_is_not_equipped(self):
        session = _session()
        session.profile = None
        assert not equipped(session)


class TestUplink:
    def test_authority_instruction_rides_the_link(self):
        link = open_link("Shanwick Radio")
        instruction = _instruction()
        assert link.takes(instruction)
        msg = link.uplink(instruction)
        assert instruction.delivery == "cpdlc"
        assert link.pending is msg
        assert msg.direction == "up"

    def test_other_station_stays_voice(self):
        link = open_link("Shanwick Radio")
        assert not link.takes(_instruction(authority="Gander Radio"))

    def test_advisory_stays_voice(self):
        link = open_link("Shanwick Radio")
        assert not link.takes(_instruction(readback=False))

    def test_one_uplink_at_a_time(self):
        link = open_link("Shanwick Radio")
        link.uplink(_instruction())
        assert not link.takes(_instruction(kind="climb"))


class TestDownlink:
    def test_wilco_settles_like_a_readback(self):
        session = _session()
        link = open_link("Shanwick Radio")
        session.cpdlc = link
        instruction = _instruction()
        session.remember(instruction)
        assert session.pending is instruction
        down = link.respond("wilco", session)
        assert down.text == "WILCO"
        assert instruction.read_back
        assert session.applied == [{"altitude_ft": 28000}]
        assert session.pending is None
        assert link.pending is None

    def test_unable_hands_it_back(self):
        session = _session()
        link = open_link("Shanwick Radio")
        session.cpdlc = link
        instruction = _instruction()
        session.remember(instruction)
        down = link.respond("unable", session)
        assert down.text == "UNABLE"
        assert not instruction.read_back
        assert session.applied == []
        assert link.refused["descend"]

    def test_refused_kind_is_not_reoffered(self):
        link = open_link("Shanwick Radio")
        link.uplink(_instruction())
        link.respond("unable", _session())
        assert not link.takes(_instruction())
        # Until the refusal ages out -- then it may be tried again.
        link.refused["descend"] = time.monotonic() - REFUSED_REPEAT_S - 1
        assert link.takes(_instruction())

    def test_roger_acknowledges_without_applying(self):
        session = _session()
        link = open_link("Shanwick Radio")
        link.uplink(_instruction(readback=False))
        down = link.respond("roger", session)
        assert down.text == "ROGER"
        assert session.applied == []

    def test_roger_does_not_settle_a_readback(self):
        session = _session()
        link = open_link("Shanwick Radio")
        link.uplink(_instruction())
        assert link.respond("roger", session) is None
        assert link.pending is not None
        assert session.applied == []

    def test_answer_with_nothing_open_is_none(self):
        link = open_link("Shanwick Radio")
        assert link.respond("wilco", _session()) is None

    def test_remember_marks_delivery(self):
        session = _session()
        session.cpdlc = open_link("Shanwick Radio")
        instruction = session.remember(_instruction())
        assert instruction.delivery == "cpdlc"
        assert session.cpdlc.pending is not None

    def test_view_hides_instruction_objects(self):
        link = open_link("Shanwick Radio")
        link.uplink(_instruction())
        view = link.view()
        assert view["authority"] == "Shanwick Radio"
        assert view["pending"]["text"] == "DESCEND FL280"
        assert view["pending"]["direction"] == "up"
