"""What the other aeroplanes say.

The controller's half of a transmission is already written, six times over, in
:mod:`meridianatc.atc.phraseology` and :mod:`meridianatc.atc.phrasebook`. This is
the other half: the requests and readbacks an AI aircraft makes, in whichever
language its crew is speaking.

A readback is not free composition. It is the instruction's key elements said
back, in the order they were given, with the callsign on the end -- so almost
everything here is a short frame around a value the controller's own number
renderer produces. That is why a :class:`PilotSpeech` is built around a
phraseology object rather than duplicating its digits: a runway is spelled the
same way whoever is saying it, and a French crew reading back "piste zéro six"
has to say it exactly as the controller did or the readback is not a readback.

The requests are the part that genuinely differs, and there are not many of
them, because an aeroplane on frequency mostly listens.
"""

from __future__ import annotations

import contextlib
import re
from dataclasses import dataclass

from .phraseology import Aircraft, _join


@dataclass(frozen=True)
class PilotBook:
    """The pilot's side of the radio, in one language."""

    language: str

    # Requests, made once each per flight.
    request_clearance: str        # {dest}
    request_taxi: str
    ready_for_departure: str      # {runway}
    checking_in_climbing: str     # {altitude}
    checking_in_descending: str   # {altitude}
    established: str              # {runway}
    field_in_sight: str
    traffic_in_sight: str

    # Readbacks: the instruction, said back.
    taxi: str                     # {route} {runway}
    hold_short: str               # {runway}
    line_up: str                  # {runway}
    takeoff: str                  # {runway}
    land: str                     # {runway}
    go_around: str
    contact: str                  # {station} {frequency}
    squawk: str                   # {code}
    climb: str                    # {altitude}
    descend: str                  # {altitude}
    heading: str                  # {heading}
    speed: str                    # {speed}
    with_information: str         # {letter}

    # The short ones.
    roger: str
    wilco: str
    good_day: str

    # A greeting to open with, and the calls a controller can name when it is
    # waiting for one. These are the plain form of the request, with no number
    # in them, because a controller telling a pilot what to say cannot know the
    # runway or the level the pilot is about to give it.
    hello: str = ""
    check_in: str = ""
    request_vfr_departure: str = ""
    request_pushback: str = ""
    request_pattern_work: str = ""
    request_landing: str = ""
    request_taxi_to_parking: str = ""
    request_transition: str = ""
    request_flight_following: str = ""
    ready_for_departure_plain: str = ""
    # A hold read back: the fix and the level (the IFR plan, Phase 6).
    hold: str = "hold at {fix}, {altitude}"
    # A diversion or a return read back: where to, and the level.
    cleared_to: str = "cleared to {airport}, {altitude}"
    # The clearance request with no destination in it, for a flight
    # that has not filed one: naming the call must not leave the
    # preposition dangling ("request IFR clearance to").
    request_clearance_plain: str = ""
    # The readback of a join: {leg} {direction} {runway}.
    pattern_entry: str = "{direction} {leg} runway {runway}"
    # An approach clearance, read back: "cleared ILS approach runway two five
    # left". Each book sets its own.
    cleared_approach: str = "cleared {approach} approach runway {runway}"
    # Reporting a leg of the circuit, and reading back the option.
    on_leg: str = "turning {leg}"
    # A circuit report the pilot handed the first officer to make.
    runway_vacated: str = "runway vacated"
    # The rejected takeoff, told to the tower while the speed is still
    # coming off -- "Speedbird one two three stopping".
    stopping: str = "stopping"
    # Saying who is ahead on an uncontrolled field ("number two, following
    # the Cessna"). UNVERIFIED in every language but English.
    following: str = "number two, following {traffic}"
    option: str = "cleared for the option runway {runway}"
    # A long taxi cut in two, read back: the taxiway held short of.
    # UNVERIFIED in every language but English.
    hold_short_taxiway: str = "hold short {taxiway}"
    # A runway crossing clearance, read back. UNVERIFIED in every language
    # but English.
    cross_runway: str = "cross runway {runway}"
    # A VFR release, read back: the way out where one was named, the
    # ceiling it stays under, the departure frequency and the code.
    # UNVERIFIED in every language but English.
    vfr_release_via: str = "via {point}"
    vfr_release_ceiling: str = "not above {altitude}"
    departure_frequency: str = "departure frequency {frequency}"
    # The taxiway an intersection departure joins (AIM 4-3-10): said in
    # the ready call ("ready for departure at taxiway alpha") and read
    # back with the line-up and takeoff clearances. UNVERIFIED in every
    # language but English.
    at_intersection: str = "at taxiway {taxiway}"
    # The answer "say your intentions" gets when the plan is the approach
    # it was cleared for: "continuing approach runway 21".
    continuing_approach: str = "continuing approach runway {runway}"
    # Convective-weather avoidance: the request, the safety report where no
    # clearance arrived in time, the readback, and the resume report.
    request_deviation: str = "request deviation {degrees} degrees {side} for weather"
    deviating_weather: str = "deviating {side} for weather"
    deviation_readback: str = "deviation {degrees} degrees {side}"
    clear_of_weather: str = "clear of weather, resuming course"


ENGLISH = PilotBook(
    language="en",
    request_deviation="request deviation {degrees} degrees {side} for weather",
    deviating_weather="deviating {side} for weather",
    deviation_readback="deviation {degrees} degrees {side}",
    clear_of_weather="clear of weather, resuming course",
    request_clearance="request IFR clearance to {dest}",
    request_clearance_plain="request IFR clearance",
    request_taxi="request taxi",
    ready_for_departure="ready for departure runway {runway}",
    checking_in_climbing="climbing {altitude}",
    checking_in_descending="descending {altitude}",
    established="established on the localiser runway {runway}",
    field_in_sight="field in sight",
    traffic_in_sight="traffic in sight",
    taxi="taxi via {route} runway {runway}",
    hold_short="hold short runway {runway}",
    line_up="line up and wait runway {runway}",
    takeoff="cleared for takeoff runway {runway}",
    land="cleared to land runway {runway}",
    go_around="going around",
    contact="contact {station} {frequency}",
    squawk="squawk {code}",
    climb="climb {altitude}",
    descend="descend {altitude}",
    heading="heading {heading}",
    speed="{speed} knots",
    with_information="with information {letter}",
    roger="roger",
    wilco="wilco",
    good_day="good day",
    hello="good day",
    check_in="with you",
    request_vfr_departure="request VFR departure",
    request_pushback="request pushback",
    request_pattern_work="request closed traffic",
    request_landing="inbound for landing",
    request_taxi_to_parking="request taxi to parking",
    request_transition="request transition",
    request_flight_following="request flight following",
    ready_for_departure_plain="ready for departure",
    at_intersection="at taxiway {taxiway}",
)

FRENCH = PilotBook(
    language="fr",
    request_deviation="demande déviation de {degrees} degrés à {side} pour météo",
    deviating_weather="dévions à {side} pour météo",
    deviation_readback="déviation de {degrees} degrés à {side}",
    clear_of_weather="clair de la météo, reprise de route",
    cleared_approach="autorisés approche {approach} piste {runway}",
    request_clearance="demande la clairance IFR pour {dest}",
    request_clearance_plain="demande la clairance IFR",
    request_taxi="demande le roulage",
    ready_for_departure="prêt au départ piste {runway}",
    checking_in_climbing="en montée {altitude}",
    checking_in_descending="en descente {altitude}",
    established="établi sur le localizer piste {runway}",
    field_in_sight="terrain en vue",
    traffic_in_sight="trafic en vue",
    taxi="roulage par {route} piste {runway}",
    hold_short="point d'attente piste {runway}",
    hold_short_taxiway="maintiens avant {taxiway}",
    cross_runway="traversée piste {runway}",
    line_up="alignement piste {runway}",
    takeoff="autorisé décollage piste {runway}",
    land="autorisé atterrissage piste {runway}",
    go_around="remise de gaz",
    stopping="décollage interrompu",
    contact="contact {station} {frequency}",
    squawk="affiche {code}",
    vfr_release_via="par {point}",
    vfr_release_ceiling="sans dépasser {altitude}",
    departure_frequency="fréquence départ {frequency}",
    climb="montons {altitude}",
    descend="descendons {altitude}",
    heading="cap {heading}",
    speed="{speed} nœuds",
    with_information="avec information {letter}",
    roger="bien reçu",
    wilco="wilco",
    good_day="bonne journée",
    hello="bonjour",
    check_in="pour vous",
    request_vfr_departure="demande un départ VFR",
    request_pushback="demande le repoussage",
    request_pattern_work="pour un tour de piste",
    request_landing="pour l'atterrissage",
    request_taxi_to_parking="demande le parking",
    request_transition="demande la traversée",
    request_flight_following="demande le service d'information de vol",
    ready_for_departure_plain="prêt au départ",
    at_intersection="à l'intersection {taxiway}",
    pattern_entry="{leg} main {direction} piste {runway}",
    on_leg="en {leg}",
    continuing_approach="poursuivons l'approche piste {runway}",
    runway_vacated="piste dégagée",
    following="numéro deux, derrière {traffic}",
    option="autorisé option piste {runway}",
    hold="attente à {fix}, {altitude}",
    cleared_to="autorisés vers {airport}, {altitude}",
)

SPANISH = PilotBook(
    language="es",
    request_deviation="solicito desviación de {degrees} grados a la {side} por meteorología",
    deviating_weather="desviando a la {side} por meteorología",
    deviation_readback="desviación de {degrees} grados a la {side}",
    clear_of_weather="fuera de la meteorología, reanudando ruta",
    cleared_approach="autorizados aproximación {approach} pista {runway}",
    request_clearance="solicito autorización IFR para {dest}",
    request_clearance_plain="solicito autorización IFR",
    request_taxi="solicito rodaje",
    ready_for_departure="listo para salida pista {runway}",
    checking_in_climbing="ascendiendo {altitude}",
    checking_in_descending="descendiendo {altitude}",
    established="establecidos en el localizador pista {runway}",
    field_in_sight="campo a la vista",
    traffic_in_sight="tráfico a la vista",
    taxi="rodaje por {route} pista {runway}",
    hold_short="punto de espera pista {runway}",
    hold_short_taxiway="mantengo antes de {taxiway}",
    cross_runway="autorizado a cruzar pista {runway}",
    line_up="alineados y esperando pista {runway}",
    takeoff="autorizado a despegar pista {runway}",
    land="autorizado a aterrizar pista {runway}",
    go_around="motor y al aire",
    stopping="abortamos el despegue",
    contact="contactamos {station} {frequency}",
    squawk="transpondedor {code}",
    vfr_release_via="vía {point}",
    vfr_release_ceiling="no por encima de {altitude}",
    departure_frequency="frecuencia de salida {frequency}",
    climb="ascendemos {altitude}",
    descend="descendemos {altitude}",
    heading="rumbo {heading}",
    speed="{speed} nudos",
    with_information="con información {letter}",
    roger="recibido",
    wilco="wilco",
    good_day="buen día",
    hello="buenos días",
    check_in="le escuchamos",
    request_vfr_departure="solicito salida VFR",
    request_pushback="solicito retroceso",
    request_pattern_work="solicito circuitos de tráfico",
    request_landing="para el aterrizaje",
    request_taxi_to_parking="solicito rodaje a plataforma",
    request_transition="solicito el tránsito",
    request_flight_following="solicito servicio de información",
    ready_for_departure_plain="listo para salida",
    at_intersection="en {taxiway}",
    pattern_entry="{leg} por la {direction} pista {runway}",
    on_leg="en {leg}",
    continuing_approach="continuamos aproximación pista {runway}",
    following="número dos, detrás de {traffic}",
    option="autorizada la opción pista {runway}",
    hold="espera sobre {fix}, {altitude}",
    cleared_to="autorizados a {airport}, {altitude}",
)

GERMAN = PilotBook(
    language="de",
    request_deviation="erbitte Ausweichen {degrees} Grad {side} wegen Wetter",
    deviating_weather="weichen {side} wegen Wetter aus",
    deviation_readback="Ausweichen {degrees} Grad {side}",
    clear_of_weather="frei vom Wetter, setzen Kurs fort",
    cleared_approach="freigegeben {approach} Anflug Piste {runway}",
    request_clearance="erbitte IFR Freigabe nach {dest}",
    request_clearance_plain="erbitte IFR Freigabe",
    request_taxi="erbitte Rollen",
    ready_for_departure="startbereit Piste {runway}",
    checking_in_climbing="im Steigflug {altitude}",
    checking_in_descending="im Sinkflug {altitude}",
    established="etabliert auf dem Landekurs Piste {runway}",
    field_in_sight="Platz in Sicht",
    traffic_in_sight="Verkehr in Sicht",
    taxi="rollen über {route} Piste {runway}",
    hold_short="Rollhalt Piste {runway}",
    hold_short_taxiway="halte vor {taxiway}",
    cross_runway="Bahnquerung Piste {runway}",
    line_up="einreihen und warten Piste {runway}",
    takeoff="Start frei Piste {runway}",
    land="Landung frei Piste {runway}",
    go_around="wir starten durch",
    stopping="Startabbruch",
    contact="rufen {station} {frequency}",
    squawk="Squawk {code}",
    vfr_release_via="über {point}",
    vfr_release_ceiling="nicht über {altitude}",
    departure_frequency="Abflugfrequenz {frequency}",
    climb="steigen {altitude}",
    descend="sinken {altitude}",
    heading="Kurs {heading}",
    speed="{speed} Knoten",
    with_information="mit Information {letter}",
    roger="verstanden",
    wilco="wilco",
    good_day="schönen Tag",
    hello="guten Tag",
    check_in="bei Ihnen",
    request_vfr_departure="erbitte VFR Abflug",
    request_pushback="erbitte Pushback",
    request_pattern_work="erbitte Platzrunden",
    request_landing="zur Landung",
    request_taxi_to_parking="erbitte Rollen zur Parkposition",
    request_transition="erbitte den Durchflug",
    request_flight_following="erbitte Fluginformationsdienst",
    ready_for_departure_plain="startbereit",
    at_intersection="an {taxiway}",
    pattern_entry="{leg} {direction} Piste {runway}",
    on_leg="im {leg}",
    continuing_approach="setzen den Anflug fort Piste {runway}",
    following="Nummer zwei, hinter {traffic}",
    option="Option genehmigt Piste {runway}",
    hold="Warteverfahren über {fix}, {altitude}",
    cleared_to="frei nach {airport}, {altitude}",
)

ITALIAN = PilotBook(
    language="it",
    request_deviation="chiedo deviazione di {degrees} gradi a {side} per maltempo",
    deviating_weather="deviamo a {side} per maltempo",
    deviation_readback="deviazione di {degrees} gradi a {side}",
    clear_of_weather="fuori dal maltempo, riprendiamo la rotta",
    cleared_approach="autorizzati avvicinamento {approach} pista {runway}",
    request_clearance="chiedo autorizzazione IFR per {dest}",
    request_clearance_plain="chiedo autorizzazione IFR",
    request_taxi="chiedo rullaggio",
    ready_for_departure="pronti al decollo pista {runway}",
    checking_in_climbing="in salita {altitude}",
    checking_in_descending="in discesa {altitude}",
    established="stabilizzati sul localizzatore pista {runway}",
    field_in_sight="campo in vista",
    traffic_in_sight="traffico in vista",
    taxi="rulliamo via {route} pista {runway}",
    hold_short="punto attesa pista {runway}",
    hold_short_taxiway="mantengo prima di {taxiway}",
    cross_runway="autorizzati all'attraversamento pista {runway}",
    line_up="allineati e in attesa pista {runway}",
    takeoff="autorizzati al decollo pista {runway}",
    land="autorizzati all'atterraggio pista {runway}",
    go_around="riattacchiamo",
    stopping="decollo interrotto",
    contact="contattiamo {station} {frequency}",
    squawk="transponder {code}",
    vfr_release_via="via {point}",
    vfr_release_ceiling="non oltre {altitude}",
    departure_frequency="frequenza di partenza {frequency}",
    climb="saliamo {altitude}",
    descend="scendiamo {altitude}",
    heading="prua {heading}",
    speed="{speed} nodi",
    with_information="con informazione {letter}",
    roger="ricevuto",
    wilco="wilco",
    good_day="buona giornata",
    hello="buongiorno",
    check_in="con voi",
    request_vfr_departure="chiedo partenza VFR",
    request_pushback="chiedo push back",
    request_pattern_work="chiedo circuiti",
    request_landing="per l'atterraggio",
    request_taxi_to_parking="chiedo rullaggio al parcheggio",
    request_transition="chiedo il transito",
    request_flight_following="chiedo servizio informazioni",
    ready_for_departure_plain="pronti al decollo",
    at_intersection="al raccordo {taxiway}",
    pattern_entry="{leg} {direction} pista {runway}",
    on_leg="in {leg}",
    continuing_approach="continuiamo l'avvicinamento pista {runway}",
    following="numero due, dietro {traffic}",
    option="autorizzato all'opzione pista {runway}",
    hold="attesa su {fix}, {altitude}",
    cleared_to="autorizzati a {airport}, {altitude}",
)

PORTUGUESE = PilotBook(
    language="pt",
    request_deviation="peço desvio de {degrees} graus à {side} devido a meteorologia",
    deviating_weather="desviando à {side} devido a meteorologia",
    deviation_readback="desvio de {degrees} graus à {side}",
    clear_of_weather="livre da meteorologia, retomando a rota",
    cleared_approach="autorizados aproximação {approach} pista {runway}",
    request_clearance="peço autorização IFR para {dest}",
    request_clearance_plain="peço autorização IFR",
    request_taxi="peço táxi",
    ready_for_departure="prontos para descolar pista {runway}",
    checking_in_climbing="a subir {altitude}",
    checking_in_descending="a descer {altitude}",
    established="estabelecidos no localizador pista {runway}",
    field_in_sight="campo à vista",
    traffic_in_sight="tráfego à vista",
    taxi="taxiamos por {route} pista {runway}",
    hold_short="ponto de espera pista {runway}",
    hold_short_taxiway="mantenho antes de {taxiway}",
    cross_runway="autorizados a atravessar pista {runway}",
    line_up="alinhados e a aguardar pista {runway}",
    takeoff="autorizados a descolar pista {runway}",
    land="autorizados a aterrar pista {runway}",
    go_around="arremetemos",
    stopping="descolagem abortada",
    contact="contactamos {station} {frequency}",
    squawk="transponder {code}",
    vfr_release_via="via {point}",
    vfr_release_ceiling="não acima de {altitude}",
    departure_frequency="frequência de partida {frequency}",
    climb="subimos {altitude}",
    descend="descemos {altitude}",
    heading="proa {heading}",
    speed="{speed} nós",
    with_information="com informação {letter}",
    roger="recebido",
    wilco="wilco",
    good_day="bom dia",
    hello="bom dia",
    check_in="convosco",
    request_vfr_departure="peço partida VFR",
    request_pushback="peço push back",
    request_pattern_work="peço circuitos",
    request_landing="para a aterragem",
    request_taxi_to_parking="peço táxi para o estacionamento",
    request_transition="peço o trânsito",
    request_flight_following="peço serviço de informação",
    ready_for_departure_plain="prontos para descolar",
    at_intersection="na interseção {taxiway}",
    pattern_entry="{leg} pela {direction} pista {runway}",
    on_leg="na {leg}",
    continuing_approach="continuamos aproximação pista {runway}",
    following="número dois, atrás de {traffic}",
    option="autorizada a opção pista {runway}",
    hold="espera sobre {fix}, {altitude}",
    cleared_to="autorizados para {airport}, {altitude}",
)


BOOKS: dict[str, PilotBook] = {
    "en": ENGLISH, "fr": FRENCH, "es": SPANISH, "de": GERMAN,
    "it": ITALIAN, "pt": PORTUGUESE,
}


class PilotSpeech:
    """One aircraft's side of the radio.

    Built around a phraseology object so the numbers come out identically to
    the way the controller said them, which is the whole point of a readback.
    """

    def __init__(self, language: str, phraseology):
        self.book = BOOKS.get((language or "en").lower()[:2], ENGLISH)
        self.p = phraseology
        # Whether the next call should open with a greeting. You say good
        # morning the first time you speak to a position, not every time --
        # and at an uncontrolled field, where every call goes to the same
        # traffic frequency, greeting each one sounds like a stuck record.
        self.greet = True

    @property
    def language(self) -> str:
        return self.book.language

    # ------------------------------------------------------------------
    # calling up
    # ------------------------------------------------------------------

    def _sign(self, aircraft: Aircraft, abbreviated: bool = False) -> str:
        return aircraft.spoken(abbreviated)

    def call(self, aircraft: Aircraft, station: str, body: str,
             greeting: bool = False) -> str:
        """An initial call: who you are calling, who you are, what you want.

        A first call to a position carries a greeting, because a real one does.
        "Orly Sol, Air France mille deux cent trente-quatre, bonjour, demande
        le roulage" is how the frequency actually sounds, and a roomful of
        aeroplanes that never says good morning sounds like a machine.
        """
        parts = [station, self._sign(aircraft)]
        if greeting and self.greet and self.book.hello:
            parts.append(self.book.hello)
        parts.append(body)
        return _join(*parts) + "."

    def report(self, aircraft: Aircraft, body: str,
               farewell: bool = False) -> str:
        """A report or a readback: what you are doing, then who you are.

        The callsign goes last, which is the real convention: the controller
        already knows they are being spoken to and needs the identity to file
        what they just heard against. A transmission that is the last one on
        this frequency signs off before it, which is the other half of the
        greeting.
        """
        # A slot left empty -- a handoff with no station name -- leaves a
        # double space or a space before a comma in the frame: "contact  one
        # one eight". Closed up here, once, for every frame.
        body = re.sub(r"\s+", " ", body).replace(" ,", ",").strip()
        parts = [body]
        if farewell and self.book.good_day:
            parts.append(self.book.good_day)
        # The pilot abbreviates the callsign only after the station has:
        # that is the rule (Doc 4444 12.3.1.9), and the latch it hinges
        # on is ``Aircraft.kept_short``, set the first time the
        # controller's own side of the exchange went short.
        parts.append(self._sign(aircraft, aircraft.kept_short))
        return _join(*parts) + "."

    # ------------------------------------------------------------------
    # requests
    # ------------------------------------------------------------------

    def _first_call(self, body: str, type_name: str = "",
                    atis_letter: str = "") -> str:
        """What a first call carries beyond the request itself: what the
        aeroplane is ("Heathrow Ground, G-ABCD, Cessna, request taxi"), and
        the information letter the crew is holding."""
        if type_name:
            body = _join(type_name, body)
        if atis_letter:
            body = _join(body, self.book.with_information.format(
                letter=self.p.d(atis_letter) if atis_letter.isdigit()
                else _spell(atis_letter)))
        return body

    def request_clearance(self, aircraft: Aircraft, station: str,
                          destination: str, atis_letter: str = "",
                          type_name: str = "") -> str:
        body = self.book.request_clearance.format(dest=destination)
        return self.call(aircraft, station,
                         self._first_call(body, type_name, atis_letter),
                         greeting=True)

    def request_taxi(self, aircraft: Aircraft, station: str,
                     atis_letter: str = "", type_name: str = "") -> str:
        body = self._first_call(self.book.request_taxi, type_name, atis_letter)
        return self.call(aircraft, station, body, greeting=True)

    def request_pushback(self, aircraft: Aircraft, station: str,
                         atis_letter: str = "", type_name: str = "") -> str:
        body = self._first_call(self.book.request_pushback, type_name,
                                atis_letter)
        return self.call(aircraft, station, body, greeting=True)

    def request_vfr_departure(self, aircraft: Aircraft, station: str,
                              atis_letter: str = "",
                              type_name: str = "") -> str:
        body = self._first_call(self.book.request_vfr_departure, type_name,
                                atis_letter)
        return self.call(aircraft, station, body, greeting=True)

    def inbound_for_landing(self, aircraft: Aircraft, station: str,
                            atis_letter: str = "", type_name: str = "") -> str:
        """A VFR arrival calling the tower up, from outside the circuit."""
        body = self._first_call(self.book.request_landing, type_name,
                                atis_letter)
        return self.call(aircraft, station, body, greeting=True)

    def pattern_entry(self, aircraft: Aircraft, runway: str,
                      leg: str = "", direction: str = "") -> str:
        """The readback of a join: the leg, the side and the runway."""
        return self.report(aircraft, self.book.pattern_entry.format(
            leg=leg or self.p.leg("downwind"),
            direction=direction or self.p.side("left"),
            runway=self.p.rwy(runway)))

    def _say_taxiway(self, name: str) -> str:
        """The taxiway's name spelled the way this crew says it --
        "B4" as "Bravo four", the field's words as words."""
        if self.language == "fr":
            from .speech_fr import taxiway
        else:
            from .speech import taxiway
        return taxiway(name)

    def ready_for_departure(self, aircraft: Aircraft, runway: str,
                            station: str = "", intersection: str = "") -> str:
        """"Ready for departure at taxiway alpha": a pilot calling for
        the runway off an intersection says where the aeroplane is,
        because the tower cannot always see the point it is standing at
        (AIM 4-3-10)."""
        body = self.book.ready_for_departure.format(runway=self.p.rwy(runway))
        if intersection:
            body += ", " + self.book.at_intersection.format(
                taxiway=self._say_taxiway(intersection))
        return (self.call(aircraft, station, body, greeting=True) if station
                else self.report(aircraft, body))

    def checking_in(self, aircraft: Aircraft, station: str, altitude_ft: float,
                    climbing: bool, atis_letter: str = "",
                    type_name: str = "") -> str:
        frame = (self.book.checking_in_climbing if climbing
                 else self.book.checking_in_descending)
        body = frame.format(altitude=self.p.alt(altitude_ft))
        return self.call(aircraft, station,
                         self._first_call(body, type_name, atis_letter),
                         greeting=True)

    def maintaining(self, aircraft: Aircraft, station: str,
                    altitude_ft: float) -> str:
        """Checking in level: "Maastricht Radar, Speedbird four two one,
        flight level three five zero" -- the level alone, as a check-in
        at cruise is said in every language here."""
        return self.call(aircraft, station, self.p.alt(altitude_ft),
                         greeting=True)

    def established(self, aircraft: Aircraft, runway: str) -> str:
        return self.report(
            aircraft, self.book.established.format(runway=self.p.rwy(runway)))

    def field_in_sight(self, aircraft: Aircraft) -> str:
        return self.report(aircraft, self.book.field_in_sight)

    def traffic_in_sight(self, aircraft: Aircraft) -> str:
        return self.report(aircraft, self.book.traffic_in_sight)

    def going_around(self, aircraft: Aircraft, altitude_ft: float = 0.0) -> str:
        """"Going around, climb three thousand": with the altitude the
        controller gave, which is what the readback is for."""
        body = self.book.go_around
        if altitude_ft:
            body += ", " + self.book.climb.format(altitude=self.p.alt(altitude_ft))
        return self.report(aircraft, body)

    def approach(self, aircraft: Aircraft, approach: str, runway: str,
                 altitude_ft: float = 0.0, pressure: float = 0.0) -> str:
        """An approach clearance read back: the approach and runway, the
        altitude, and the pressure setting if one came with it."""
        body = self.book.cleared_approach.format(
            approach=approach, runway=self.p.rwy(runway))
        if altitude_ft:
            body += ", " + self.p.alt(altitude_ft)
        if pressure:
            body += ", " + self.p.pressure(pressure)
        return self.report(aircraft, body)

    def intercept(self, aircraft: Aircraft, heading: float, approach: str,
                  runway: str, altitude_ft: float = 0.0) -> str:
        """The last vector read back with its clearance: the heading first,
        as it was given, then the approach and runway, then the altitude
        to hold until established."""
        body = self.book.heading.format(heading=self.p.hdg(heading)) + ", " \
            + self.book.cleared_approach.format(approach=approach,
                                                runway=self.p.rwy(runway))
        if altitude_ft:
            body += ", " + self.p.alt(altitude_ft)
        return self.report(aircraft, body)

    def on_leg(self, aircraft: Aircraft, leg: str, following: str = "") -> str:
        """Reporting a leg of the circuit: "turning base", "en finale"; on a
        field nobody works, with who is ahead ("number two, following the
        Cessna")."""
        body = self.book.on_leg.format(leg=leg)
        if following:
            body += ", " + self.book.following.format(traffic=following)
        return self.report(aircraft, body)

    def circuit_report(self, aircraft: Aircraft, leg: str) -> str:
        """"Downwind": the report the pilot handed the first officer, the leg
        on its own as CAP 413 has it. With the runway after it the parser
        reads the call as a readback ("downwind runway two four" is how a
        join is read back), so the runway is left to the controller."""
        return self.report(aircraft, leg)

    def runway_vacated(self, aircraft: Aircraft) -> str:
        return self.report(aircraft, self.book.runway_vacated)

    def stopping(self, aircraft: Aircraft) -> str:
        """"N123 stopping": the reject a crew owes the tower while the
        speed is still coming off -- the call is made as soon as the
        aeroplane is under control, not when it has stopped."""
        return self.report(aircraft, self.book.stopping)

    def option(self, aircraft: Aircraft, runway: str) -> str:
        return self.report(
            aircraft, self.book.option.format(runway=self.p.rwy(runway)))

    def request_deviation(self, aircraft: Aircraft, station: str,
                          degrees: int, side: str) -> str:
        """The IFR weather-avoidance request: direction and magnitude before
        manoeuvring, because the cleared route is not the crew's to leave."""
        body = self.book.request_deviation.format(
            degrees=str(int(degrees)), side=self.p.side(side))
        return self.call(aircraft, station, body)

    def deviating_weather(self, aircraft: Aircraft, side: str) -> str:
        """The emergency report made while manoeuvring when the weather is at
        the windshield and no ATC answer has arrived."""
        return self.report(aircraft, self.book.deviating_weather.format(
            side=self.p.side(side)))

    def deviation(self, aircraft: Aircraft, degrees: int, side: str) -> str:
        """The approved lateral deviation, read back with its limits."""
        return self.report(aircraft, self.book.deviation_readback.format(
            degrees=str(int(degrees)), side=self.p.side(side)))

    def clear_of_weather(self, aircraft: Aircraft) -> str:
        return self.report(aircraft, self.book.clear_of_weather)

    # ------------------------------------------------------------------
    # readbacks
    # ------------------------------------------------------------------

    def taxi(self, aircraft: Aircraft, runway: str, route: str = "",
             hold_short: str = "", hold_short_taxiway: str = "") -> str:
        """The readback of a taxi clearance: the taxiways, where it holds,
        the runway -- not the whole clearance said back."""
        parts = []
        if route:
            parts.append(self.book.taxi.format(route=route,
                                               runway=self.p.rwy(runway)))
        else:
            parts.append(self.book.hold_short.format(runway=self.p.rwy(runway)))
        if hold_short_taxiway:
            parts.append(self.book.hold_short_taxiway.format(
                taxiway=hold_short_taxiway))
        if hold_short:
            parts.append(self.book.hold_short.format(
                runway=self.p.rwy(hold_short)))
        return self.report(aircraft, _join(*parts))

    def cross_runway(self, aircraft: Aircraft, runway: str) -> str:
        return self.report(
            aircraft, self.book.cross_runway.format(runway=self.p.rwy(runway)))

    def line_up(self, aircraft: Aircraft, runway: str,
                intersection: str = "") -> str:
        body = self.book.line_up.format(runway=self.p.rwy(runway))
        if intersection:
            body += ", " + self.book.at_intersection.format(
                taxiway=self._say_taxiway(intersection))
        return self.report(aircraft, body)

    def takeoff(self, aircraft: Aircraft, runway: str,
                intersection: str = "") -> str:
        body = self.book.takeoff.format(runway=self.p.rwy(runway))
        if intersection:
            body += ", " + self.book.at_intersection.format(
                taxiway=self._say_taxiway(intersection))
        return self.report(aircraft, body)

    def land(self, aircraft: Aircraft, runway: str) -> str:
        return self.report(
            aircraft, self.book.land.format(runway=self.p.rwy(runway)))

    def contact(self, aircraft: Aircraft, station: str, mhz: float) -> str:
        """The readback of a handoff, which is the last call on this frequency."""
        return self.report(aircraft, self.book.contact.format(
            station=station, frequency=self.p.freq(mhz)), farewell=True)

    def leaving(self, aircraft: Aircraft) -> str:
        """Signing off a frequency with nowhere in particular to go next."""
        return self.report(aircraft, self.book.wilco, farewell=True)

    def squawk(self, aircraft: Aircraft, code: str) -> str:
        return self.report(aircraft,
                           self.book.squawk.format(code=self.p.d(code)))

    def vfr_release(self, aircraft: Aircraft, squawk: str,
                    mhz: float = 0.0, ceiling_ft: float = 0.0,
                    exit_point: str = "") -> str:
        """A VFR release read back: the way out where one was named, the
        ceiling, the departure frequency and the code. The frequency and
        the code are what the controller is holding the readback to; the
        ceiling and the point are what a pilot says along with them."""
        parts = []
        if exit_point:
            with contextlib.suppress(Exception):
                exit_point = self.p.vrp(exit_point)
            parts.append(self.book.vfr_release_via.format(point=exit_point))
        if ceiling_ft:
            parts.append(self.book.vfr_release_ceiling.format(
                altitude=self.p.vfr_alt(ceiling_ft)))
        if mhz:
            parts.append(self.book.departure_frequency.format(
                frequency=self.p.freq(mhz)))
        if squawk:
            parts.append(self.book.squawk.format(code=self.p.d(squawk)))
        return self.report(aircraft, _join(*parts))

    def clearance(self, aircraft: Aircraft, destination: str, squawk: str,
                  initial_altitude: float = 0.0, sid: str = "") -> str:
        """The readback of a departure clearance: limit, SID, altitude,
        squawk. The SID is its name alone, as the controller said it, which
        reads the same in every language."""
        parts = [destination]
        if sid:
            parts.append(self.p.procedure(sid))
        if initial_altitude:
            parts.append(self.book.climb.format(
                altitude=self.p.alt(initial_altitude)))
        parts.append(self.book.squawk.format(code=self.p.d(squawk)))
        return self.report(aircraft, _join(*parts))

    def cleared_to(self, aircraft: Aircraft, airport: str,
                   altitude_ft: float) -> str:
        return self.report(aircraft, self.book.cleared_to.format(
            airport=airport, altitude=self.p.alt(altitude_ft)))

    def hold(self, aircraft: Aircraft, fix: str, altitude_ft: float) -> str:
        """A hold read back: where, and at what level."""
        from .phraseology import fix_name

        return self.report(aircraft, self.book.hold.format(
            fix=fix_name(fix), altitude=self.p.alt(altitude_ft)))

    def descend_via(self, aircraft: Aircraft, arrival: str) -> str:
        """The readback of "descend via": the STAR by name, as the
        controller said it. Only the FAA says it, so only in English."""
        return self.report(aircraft,
                           f"descend via the {self.p.procedure(arrival)} arrival")

    def level(self, aircraft: Aircraft, altitude_ft: float,
              climbing: bool = True) -> str:
        frame = self.book.climb if climbing else self.book.descend
        return self.report(aircraft,
                           frame.format(altitude=self.p.alt(altitude_ft)))

    def heading(self, aircraft: Aircraft, degrees: float) -> str:
        return self.report(
            aircraft, self.book.heading.format(heading=self.p.hdg(degrees)))

    def speed(self, aircraft: Aircraft, knots: float) -> str:
        return self.report(
            aircraft, self.book.speed.format(speed=self.p.d(int(knots))))

    # ------------------------------------------------------------------
    # the short ones
    # ------------------------------------------------------------------

    def roger(self, aircraft: Aircraft) -> str:
        return self.report(aircraft, self.book.roger)

    def wilco(self, aircraft: Aircraft) -> str:
        return self.report(aircraft, self.book.wilco)

    def farewell(self, aircraft: Aircraft) -> str:
        return self.report(aircraft, self.book.good_day)


def _spell(letter: str) -> str:
    """The ICAO word for an ATIS letter, which is the same in every language."""
    from .speech import _ALPHABET

    return _ALPHABET.get((letter or "").upper()[:1], letter)


__all__ = ["PilotBook", "PilotSpeech", "BOOKS", "ENGLISH", "FRENCH", "SPANISH",
           "GERMAN", "ITALIAN", "PORTUGUESE"]
