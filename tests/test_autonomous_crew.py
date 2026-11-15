"""What the crew says while the machine is the one flying.

With the autonomous first officer engaged, a reminder for the pilot
about a switch the FO owns is a note it writes to itself, and a
"checklist when you're ready?" offer is a voice nobody can answer --
the FO started that checklist already. Safety notices still go: they
report the aeroplane, whoever flies it.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

from meridianatc.atc.crew import CrewDirector


def _crew(autonomous: bool, verbosity: str = "normal") -> CrewDirector:
    crew = CrewDirector.__new__(CrewDirector)
    crew.config = SimpleNamespace(immersion=SimpleNamespace(
        enabled=True, copilot=True, cabin=True, callouts=True,
        checklists=True, briefings=True, announcements=True,
        service=True, boarding_music=True, applause=True,
        verbosity=verbosity,
        first_officer=SimpleNamespace(notices=True),
        autonomous_flight=autonomous))
    crew.cockpit = "normal"
    crew._class = None
    crew._session = None
    crew.here = None
    crew.fresh = None
    return crew


def test_a_notice_for_the_pilot_is_silent_while_the_fo_flies():
    crew = _crew(autonomous=True)
    assert crew.refused("landing_light_off") == \
        "the first officer is flying"
    assert crew.refused("altimeter_not_set") == \
        "the first officer is flying"
    assert crew.refused("frequency_not_tuned") == \
        "the first officer is flying"


def test_the_safety_notices_still_go():
    crew = _crew(autonomous=True)
    assert crew.refused("gear_up_final") == ""
    assert crew.refused("fuel_reserve") == ""
    assert crew.refused("parking_brake_power") == ""


def test_every_notice_goes_when_the_pilot_flies():
    crew = _crew(autonomous=False)
    assert crew.refused("landing_light_off") == ""
    assert crew.refused("gear_up_final") == ""


def test_the_chatty_cockpit_is_flying_the_aeroplane():
    chatty = _crew(autonomous=True, verbosity="chatty")
    assert chatty.refused("cruise_chat") == "the first officer is flying"
    pilot = _crew(autonomous=False, verbosity="chatty")
    assert pilot.refused("cruise_chat") == ""


def _ground_state() -> SimpleNamespace:
    return SimpleNamespace(title="Baron", altitude_agl_ft=0.0,
                           altitude_ft=500.0)


def test_no_checklist_is_offered_to_a_pilot_who_is_not_flying():
    crew = _crew(autonomous=True)
    crew.checklists = Mock()
    crew._know_the_aeroplane = Mock()
    session = SimpleNamespace(ifr=False)
    crew._run_checklists(_ground_state(), session)
    assert crew.checklists.offers is False
    # The FO's own starts still reach the runner.
    assert crew.checklists.tick.called


def test_checklist_offers_survive_a_pilot_at_the_controls():
    crew = _crew(autonomous=False)
    crew.checklists = Mock()
    crew._know_the_aeroplane = Mock()
    session = SimpleNamespace(ifr=False)
    crew._run_checklists(_ground_state(), session)
    assert crew.checklists.offers is True
