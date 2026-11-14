from __future__ import annotations

from ._shared import (
    CONCURRENT_BY_RANK,
    GLIDE_FT_PER_NM,
    Call,
    Flight,
    NavDB,
    ParkedObserved,
    PilotSpeech,
    UserAircraft,
    _Field,
    approach_speed,
    article,
    groundwork,
    landing_traffic,
    make_of,
    math,
    random,
    runway_traffic,
)
from .x_aeroplanes_somebody_else import _Aeroplanes_somebody_else
from .x_an_aeroplane_asked import _An_aeroplane_asked
from .x_arrivals import _Arrivals
from .x_departures import _Departures
from .x_flying_them import _Flying_them
from .x_instructions_and_being import _Instructions_and_being
from .x_making_aeroplanes import _Making_aeroplanes
from .x_pilots_talking import PilotsTalking
from .x_putting_words_together import _Putting_words_together
from .x_running_them import _Running_them
from .x_small_facts_about import _Small_facts_about
from .x_test_tools_traffic import _Test_tools_traffic
from .x_the_aeroplane_the import _The_aeroplane_the
from .x_the_field import _The_field
from .x_the_ground_driven import _The_ground_driven
from .x_the_live_ones import _The_live_ones
from .x_the_ones_standing import _The_ones_standing
from .x_the_tower_watching import _The_tower_watching
from .x_the_watchdog import _The_watchdog
from .x_what_the_pilot import _What_the_pilot
from .x_where_it_is import _Where_it_is


class TrafficInjector(_The_field, _Flying_them, _What_the_pilot, _Making_aeroplanes, _The_ones_standing, _An_aeroplane_asked, _Test_tools_traffic, _The_ground_driven, _Running_them, _The_watchdog, _Instructions_and_being, _The_aeroplane_the, _Aeroplanes_somebody_else, _The_live_ones, _The_tower_watching, _Departures, _Where_it_is, _Arrivals, PilotsTalking, _Putting_words_together, _Small_facts_about):
    """Runs the other aeroplanes on the frequency.

    The engine calls :meth:`poll` on the situation loop and speaks whatever
    comes back. Everything else -- who exists, what phase they are in, whether
    the runway is free -- is decided here.
    """

    def __init__(self, navdb: NavDB, seed: int | None = None,
                 density: float = 1.0, callsigns: str = "auto"):
        self.navdb = navdb
        self.density = max(0.0, float(density))
        # The energy model moves the aeroplanes: on by default, off puts
        # them back on the plain integrator it replaced.
        self.energy_model = True
        # The hour of day sets the field's rhythm: the banks busy, the
        # small hours nearly quiet. On by default; off, the density alone
        # decides, as it always did.
        self.diurnal = True
        # Mayday discipline: while the pilot's emergency stands, everybody
        # else holds their calls. Set by the engine each tick; urgent
        # exchanges -- a go-around -- still go out.
        self.radio_silence = False
        # "auto" is whatever the field would carry; "registration" makes every
        # generated aeroplane a private one, spelled out.
        self.callsigns = (callsigns or "auto").strip().lower()
        # No seed means a different session every time, which is what the
        # setting promises. This used to fall back to a constant, and every
        # visit to a field replayed the same draw: the same two airlines at
        # Funchal, in the same order, on every flight.
        self.random = random.Random(seed)
        self.field = _Field()
        self.flights: list[Flight] = []
        self.pending: list[Call] = []
        # Numbers the exchanges, so the halves of one can be recognised as
        # halves of one after they have been queued separately.
        self._exchanges = 0
        self.runway_free_at = 0.0
        # Every callsign this field has issued, live or finished. A number is
        # never handed out twice: an aeroplane that has gone to the gate is
        # still the aeroplane the user heard, and hearing the same callsign
        # come back on a different type an hour later is the exact opposite of
        # what a consistent callsign means.
        self._used_callsigns: set[str] = set()
        self._spawn_at = 0.0
        # When the flights were last moved, so they travel at their own speed
        # rather than at the rate the caller happens to poll.
        self._flown_at = 0.0
        self._phraseology: dict[tuple[str, str], object] = {}
        # The pilot's profile: a pinned region is the controller's words to
        # the traffic as well as to the pilot, on the same frequency.
        from ..profile import DEFAULT as DEFAULT_PROFILE

        self.profile = DEFAULT_PROFILE
        self._pilots: dict[str, PilotSpeech] = {}
        # The aeroplane the person is flying, when the engine has told us
        # about it. None means the traffic works the field as though it were
        # alone on it, which is what a session with no simulator gets.
        self.user: UserAircraft | None = None
        # The tower does not re-examine the whole field on every poll; once a
        # second is faster than anything it could usefully say.
        self._watched_at = 0.0
        # Whether anything is reporting the simulator's own traffic. Read only
        # to decide how much invented traffic to add on top of it.
        self.observed_count = 0
        # Aeroplanes the pilot asked for, waiting for the next tick to join
        # the roster -- see :meth:`spawn_on_runway`.
        self._asked_for: list[Flight] = []
        # The airport's taxiways, when the simulator has given them. Without
        # them the ground is placed, as it always was; with them it is driven.
        self.ground: groundwork.GroundLayout | None = None
        # Taxiway intersections and who has them, so two aeroplanes do not
        # arrive at one from different directions at once.
        self._reserved: dict[object, str] = {}

        # Whether an invented aeroplane has to be in the simulator before it
        # exists here. The engine turns this on: the rule is that nothing is
        # heard, drawn on the map or sequenced against unless the pilot can
        # also see it out of the window. Off, the injector works on its own,
        # which is what the tests of the traffic itself want.
        self.requires_sim = False
        # Whether the simulator is drawing aeroplanes at all right now. With
        # ``requires_sim`` and this off, nothing is invented.
        self.can_invent = True
        # Whether invented aeroplanes are made at all (traffic.invented).
        # Off, nothing new is made and the ones flying stop existing; the
        # real ones from the live feed and what somebody else injected are
        # somebody else's and stay.
        self.invented = True
        # Whether the live aeroplanes are adopted and flown like the invented
        # ones (traffic.live.managed). Off, they are observed: read off the
        # feed and described rather than instructed.
        self.live_managed = False
        # Invented aeroplanes asked of the simulator and not yet confirmed by
        # it, with the moment each is given up on. Kept off ``flights`` so
        # that nothing -- the radio, the map, the runway, the sequence --
        # can see an aeroplane that may never appear.
        self._materialising: list[Flight] = []
        self._materialise_by: dict[str, float] = {}
        # How many the simulator will draw at once. More than that would
        # never appear, time out and be invented again, round and round.
        self.max_invented: int | None = None
        # Whether an airline has its own livery on a type, when something
        # can say -- FSLTL's table, through the engine. With it, an airline
        # is given the types it is painted for, so an Air France flight is
        # an Air France aeroplane out of the window rather than a white one.
        self.has_livery = None
        # Whether an aeroplane may push: its doors read shut. Set by the
        # engine from the door layer; None means nobody is reading doors
        # and nothing waits. A departure that is not ready does not ask for
        # taxi -- a real crew does not call for push with a door open -- and
        # tries again a few seconds later.
        self.can_push = None

        # The aeroplanes standing at the gates, going nowhere yet. Kept off
        # ``flights`` because nothing about them is the radio's business:
        # they are not sequenced, not spoken to and not in anybody's way on
        # a taxiway. ``parked`` is how many there should be, and 0 -- the
        # default, which the tests of the moving traffic want -- is none.
        self.statics: list[Flight] = []
        # The schedule: departures board on their stands before they call,
        # and arrivals hold their stand from the start. Off, a departure
        # is a parked aeroplane chosen the moment it is wanted, as always.
        self.schedule = False
        self.boarding_s = 240.0
        self.boarding_lead_s = 120.0
        self.parked = 0
        self._parked_at = 0.0
        self._stands_filled = False
        # Whether to wait, before setting the stands out, for the simulator
        # to have reported what is already standing on them. The first fill
        # used to go on the first tick, two seconds before FSLTL's parked
        # aeroplanes had been read, and put an aeroplane on every stand one
        # of theirs was already on.
        self.await_observed = False
        self._observed_read = False
        self._first_poll: float | None = None
        # Invent only these types (ICAO codes), for testing a model: empty
        # is the airlines' real fleets.
        self.force_types: tuple[str, ...] = ()
        # Where somebody else's parked aeroplanes are -- FSLTL, AIG, the
        # simulator's own -- which are not worked (see _works_here) but are
        # standing on stands nothing here may be put on.
        self._parked_seen: list[tuple[float, float]] = []
        # And what they are, by simulator object, for the map and the ground
        # handling: an FSLTL apron is forty aeroplanes going nowhere, and the
        # map is about the window. ``since`` is when one stopped, if it was
        # seen moving first, else 0 for "it was there when we looked".
        self._parked_observed: dict[int, ParkedObserved] = {}
        # The live ones (atc/live.py): rolled onto a stand and held there,
        # let go to move again, and seen in the air (so taxiing in).
        self._live_stands: dict[int, dict] = {}
        self._live_moving: set[int] = set()
        self._live_flew: set[str] = set()
        # The pilot's runway (_protect_the_pilot): how a live one is held and
        # let go (LiveRoster.hold, handed in by the engine), the ones sent
        # around and flown by us, and when a hold short was last said and a
        # held departure last let go.
        self.hold_live = None
        self._taken_over: set[int] = set()
        # The one we fly for each taken over, and since when it has been
        # parked, waiting for the feed to hear the real one again.
        self._flown_for: dict[int, Flight] = {}
        self._parked_since: dict[int, float] = {}
        self._hold_short_said = -math.inf
        self._last_release = -math.inf
        # What each live one was last put on (runway, taxiway, stand, or
        # nothing), for traffic --live --dry-run and the status line.
        self.live_snapped: dict[str, str] = {}

__all__ = ["TrafficInjector", "Flight", "Call", "UserAircraft", "make_of",
           "article", "GLIDE_FT_PER_NM", "approach_speed",
           "landing_traffic", "runway_traffic", "CONCURRENT_BY_RANK"]
