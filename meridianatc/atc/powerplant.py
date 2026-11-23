"""Published engine and rotor limits, and the ice protection a type carries.

The autonomous controllers fly against the simulator's own instruments --
torque, gas-producer speed, turbine temperature, rotor RPM, crankshaft RPM --
and the figures those instruments are compared against live here. They come
from flight manuals, POHs and type-certificate data sheets; the source is
kept with each entry, and figures that are training-manual or kin-engine
values rather than the type's own AFM are marked ``unverified``.

A value of 0.0 means "not published here", and a protection that has no
figure never engages -- the same rule the rest of the autonomous code
follows for a variable the model does not report.

Sources used (the limit pages of each):
- Robinson R44 POH (robinsonheli.com), R66 POH, R22 POH excerpts (NTSB
  docket copies).
- Bell 407 RFM (BHT-407-FM-1), Bell 206B3 RFM (BHT-206B3-FM-1), Bell 412
  RFM supplement (BHT-412-FM-2), Bell 212 pilot training manual.
- EASA TCDS E.001 (Arriel), R.009 (EC135/PW206), UK.TC.E.00148 (Arrano 1A),
  EASA TCDS IM.E.070 (PT6A-66/67), EASA TCDS R.006 (AW139), FAA TCDS for
  Cabri G2 and the H160 ANAC operational-evaluation report.
- Daher TBM 930 PIM; CASA S-76 performance handbook (S-76C++ flight
  manual figures, exam extract).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EngineLimits:
    """One powerplant's ceilings, per cent and Celsius. 0.0 = unpublished.

    ``tq_*`` is engine or mast torque where the model reports
    ``ENG TORQUE PERCENT``; ``ng_max_pct`` is the gas-producer/fan speed the
    ``TURB ENG N1`` variable reads; ``itt_*`` is the turbine temperature --
    ITT, MGT, TOT, T5 or EGT depending on what the manual calls it;
    ``nr_*`` is main-rotor speed per cent where the sim publishes it;
    ``rpm_max`` is the piston crankshaft red line. ``top_max_ias_kt`` is
    the airspeed ceiling published while takeoff power is set, where a
    manual publishes one (the H125's 40 kt, the Bell 407's 100 kt).
    """

    tq_takeoff_pct: float = 0.0
    tq_cont_pct: float = 0.0
    # The same ceilings in shaft N-m, for the model that reports real
    # torque but never fills in the red-line field the per-cent figure
    # needs -- the King Air's painted 1,315 ft-lb gauge reads nothing.
    tq_takeoff_nm: float = 0.0
    tq_max_nm: float = 0.0
    ng_max_pct: float = 0.0
    itt_takeoff_c: float = 0.0
    itt_max_c: float = 0.0
    itt_start_c: float = 0.0
    itt_idle_c: float = 0.0
    nr_min_pct: float = 0.0
    nr_max_pct: float = 0.0
    nr_auto_min_pct: float = 0.0
    nr_auto_max_pct: float = 0.0
    rpm_max: float = 0.0
    top_max_ias_kt: float = 0.0
    source: str = ""
    unverified: bool = False


@dataclass(frozen=True)
class IceKit:
    """The ice protection a type carries, and whether it may fly into ice.

    ``carb_heat`` is the carburetted piston's induction heat (which MSFS
    models on the anti-ice switch for a piston engine); ``engine_inlet`` is
    a turbine's engine/inlet anti-ice -- the turbine helicopters' required
    "on in visible moisture below 4-5C" system; ``inertial_separator`` is
    the PT6's bypass door; ``airframe_deice`` covers boots, bleed wing and
    TKS alike because the sim's structural-deice switch does not
    distinguish them; ``known_ice`` is the certification: a type without
    it is not flown into forecast icing by the book, so the accretion
    advisory fires earlier.
    """

    carb_heat: bool = False
    engine_inlet: bool = False
    inertial_separator: bool = False
    prop_deice: bool = False
    airframe_deice: bool = False
    windshield: bool = False
    pitot: bool = True
    known_ice: bool = False
    source: str = ""


# ---------------------------------------------------------------------------
# The rotorcraft table. Every type in ``perftable.HELI_TYPES`` has an entry;
# where a type's own manual was not found, the figures are the kin engine's
# or the light-turbine pattern and marked unverified.
# ---------------------------------------------------------------------------

_HELI_LIMITS: dict[str, EngineLimits] = {
    # Robinson R44 (Lycoming O-540, carburetted): engine max continuous
    # 102% / 2718 RPM, transient 105% / 2800; power-on rotor 101-102%,
    # power-off 90-108%.
    "R44": EngineLimits(nr_min_pct=101.0, nr_max_pct=102.0,
                        nr_auto_min_pct=90.0, nr_auto_max_pct=108.0,
                        rpm_max=2800.0,
                        source="R44 POH section 2"),
    # Robinson R66 (RR300): torque 100% 5-min / 83% continuous; N1 max 105%;
    # N2 99-101% continuous, 106% transient; MGT 706 cont / 782 5-min /
    # 843 transient / 927 start; rotor 99-101% on, 88-106% off.
    "R66": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=83.0,
                        ng_max_pct=105.0, itt_takeoff_c=782.0,
                        itt_max_c=706.0, itt_start_c=927.0,
                        nr_min_pct=99.0, nr_max_pct=101.0,
                        nr_auto_min_pct=88.0, nr_auto_max_pct=106.0,
                        source="R66 POH section 2"),
    # Airbus H125 / AS350 B3 (Arriel 2B1/2D): the FLI's takeoff figures are
    # near torque 90%, TOT 680 C, Ng 100%; takeoff power is airspeed-
    # limited to 40 KIAS. Rotor numbers not extracted: kin figure.
    "H125": EngineLimits(tq_takeoff_pct=90.0, tq_cont_pct=84.0,
                         ng_max_pct=103.0, itt_takeoff_c=846.0,
                         itt_max_c=775.0, itt_start_c=865.0,
                         nr_min_pct=94.0, nr_max_pct=104.0,
                         nr_auto_min_pct=85.0, nr_auto_max_pct=110.0,
                         top_max_ias_kt=40.0,
                         source="AS350 B3 RFM (FLI examples) + Arriel 2B TCDS",
                         unverified=True),
    # AS355/AS50 twin (2x 250-C20F): kin to the Bell 206 figures.
    "AS50": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=85.0,
                         ng_max_pct=105.0, itt_takeoff_c=810.0,
                         itt_max_c=738.0, itt_start_c=927.0,
                         nr_min_pct=95.0, nr_max_pct=104.0,
                         nr_auto_min_pct=85.0, nr_auto_max_pct=110.0,
                         source="kin: 250-C20 limits (Bell 206 RFM)",
                         unverified=True),
    # EC130 B4 (Arriel 2B1): same engine family as the H125.
    "EC30": EngineLimits(tq_takeoff_pct=90.0, tq_cont_pct=84.0,
                         ng_max_pct=103.0, itt_takeoff_c=846.0,
                         itt_max_c=775.0, itt_start_c=865.0,
                         nr_min_pct=94.0, nr_max_pct=104.0,
                         nr_auto_min_pct=85.0, nr_auto_max_pct=110.0,
                         source="kin: AS350 B3 / Arriel 2B TCDS",
                         unverified=True),
    # Bell 206 JetRanger III (250-C20B/J): torque 100% 5-min / 85% cont /
    # 110% transient; TOT 738 cont / 810 takeoff / 927 start; engine
    # anti-ice ON in visible moisture below 4.4 C, OFF above it.
    "B06": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=85.0,
                        ng_max_pct=105.0, itt_takeoff_c=810.0,
                        itt_max_c=738.0, itt_start_c=927.0,
                        itt_idle_c=540.0,
                        nr_min_pct=90.0, nr_max_pct=104.0,
                        nr_auto_min_pct=90.0, nr_auto_max_pct=107.0,
                        source="BHT-206B3-FM-1 + JetRanger checklist",
                        unverified=True),
    # Bell 407 (250-C47B): torque 93.5% cont / 100% 5-min / 105% 5-s
    # transient; Ng 105% cont; Np 99-100% cont; MGT 727 cont / 779 takeoff
    # / 927 start; takeoff power limited to 100 KIAS.
    "B407": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=93.5,
                         ng_max_pct=105.0, itt_takeoff_c=779.0,
                         itt_max_c=727.0, itt_start_c=927.0,
                         itt_idle_c=540.0,
                         nr_min_pct=99.0, nr_max_pct=104.0,
                         nr_auto_min_pct=90.0, nr_auto_max_pct=107.0,
                         top_max_ias_kt=100.0,
                         source="BHT-407-FM-1"),
    # EC135 T2 (PW206B): torque per engine 75% AEO TOP / 69% AEO MCP /
    # 100% OEI 2.5-min / 86% OEI cont; gas generator 97.4-102.4%;
    # TOT 820/854/930; rotor power-on 95-104%, off 80/85-106%.
    "EC35": EngineLimits(tq_takeoff_pct=75.0, tq_cont_pct=69.0,
                         ng_max_pct=102.4, itt_takeoff_c=854.0,
                         itt_max_c=820.0,
                         nr_min_pct=95.0, nr_max_pct=104.0,
                         nr_auto_min_pct=80.0, nr_auto_max_pct=106.0,
                         source="EASA TCDS R.009 (EC135 T2)"),
    # EC145/BK117 C-2 (Arriel 1E2): Ng takeoff 102%, MCP 100.3%; torque
    # and rotor figures not extracted -- the medium-twin pattern.
    "EC45": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=100.0,
                         ng_max_pct=103.3, itt_takeoff_c=915.0,
                         itt_max_c=877.0,
                         nr_min_pct=95.0, nr_max_pct=104.0,
                         nr_auto_min_pct=80.0, nr_auto_max_pct=110.0,
                         source="Arriel 1E2 TCDS (Ng) + pattern figures",
                         unverified=True),
    # Airbus H145 (Arriel 2E): not extracted; medium-twin pattern.
    "H145": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=100.0,
                         ng_max_pct=103.0, itt_takeoff_c=915.0,
                         itt_max_c=877.0,
                         nr_min_pct=95.0, nr_max_pct=104.0,
                         nr_auto_min_pct=80.0, nr_auto_max_pct=110.0,
                         source="pattern (medium twin)", unverified=True),
    # Airbus H160 (Arrano 1A): torque AEO TOP/30-min 940 N.m, MCP 816 --
    # about 87% of the takeoff figure; N2 min stabilised 90%, max 109%;
    # rotor power-on 92-107.8%, off 92-109.8%.
    "H160": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=87.0,
                         nr_min_pct=92.0, nr_max_pct=107.8,
                         nr_auto_min_pct=92.0, nr_auto_max_pct=109.8,
                         source="UK TCDS E.00148 (Arrano) + ANAC H160 report"),
    # AS332 Super Puma (Makila 1A1): not extracted; heavy-twin pattern.
    "AS32": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=90.0,
                         ng_max_pct=103.0, itt_takeoff_c=900.0,
                         itt_max_c=850.0,
                         nr_min_pct=95.0, nr_max_pct=104.0,
                         nr_auto_min_pct=80.0, nr_auto_max_pct=110.0,
                         source="pattern (heavy twin)", unverified=True),
    # Sikorsky S-76 (Arriel 2S2): engine torque 120% takeoff and max
    # continuous on the engine limit; the transmission limit is 100% --
    # the gauge the FO reads is the transmission's. N1 100% TOP/MCP,
    # transient 105.35%; T5 845, start 865; N2 to 114% by torque;
    # rotor power-on 100-107%, power-off 87-115%.
    "S76": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=100.0,
                        ng_max_pct=105.35, itt_takeoff_c=845.0,
                        itt_max_c=845.0, itt_start_c=865.0,
                        nr_min_pct=100.0, nr_max_pct=107.0,
                        nr_auto_min_pct=87.0, nr_auto_max_pct=115.0,
                        source="S-76C++ flight manual (CASA handbook extract)"),
    # Leonardo AW109 (PW206C): not extracted; the EC135's PW206-family
    # figures are the closest published kin.
    "A109": EngineLimits(tq_takeoff_pct=75.0, tq_cont_pct=69.0,
                         ng_max_pct=102.4, itt_takeoff_c=854.0,
                         itt_max_c=820.0,
                         nr_min_pct=95.0, nr_max_pct=104.0,
                         nr_auto_min_pct=80.0, nr_auto_max_pct=106.0,
                         source="kin: PW206 limits (EC135 TCDS)",
                         unverified=True),
    # AW139 (PT6C-67C): power-assurance ceiling -- 775 C ITT, 102.4% Ng,
    # 100% torque.
    "A139": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=100.0,
                         ng_max_pct=102.4, itt_takeoff_c=775.0,
                         itt_max_c=775.0,
                         nr_min_pct=95.0, nr_max_pct=107.0,
                         nr_auto_min_pct=80.0, nr_auto_max_pct=110.0,
                         source="AW139 QRH (power assurance figures)",
                         unverified=True),
    # Bell 412 (PT6T-3B): mast torque 81% continuous, 81-100% takeoff
    # (5-min); N1 61-101.8%; ITT 765 cont / 810 takeoff / 850 transient /
    # 1090 start; power turbine min 97% cruise, 100% hover, max 104.5%.
    "B412": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=81.0,
                         ng_max_pct=101.8, itt_takeoff_c=810.0,
                         itt_max_c=765.0, itt_start_c=1090.0,
                         nr_min_pct=97.0, nr_max_pct=104.5,
                         nr_auto_min_pct=90.0, nr_auto_max_pct=107.0,
                         source="BHT-412-FM-2 + Bell 212 PTM"),
    # UH-1H (T53-L-13): torque limited by the transmission (~50 psi,
    # 1100 shp) rather than the engine; per-cent figures are pattern
    # values. Rotor 324 RPM nominal; teetering.
    "UH1": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=100.0,
                        ng_max_pct=102.0, itt_takeoff_c=649.0,
                        itt_max_c=625.0, itt_start_c=760.0,
                        nr_min_pct=90.0, nr_max_pct=107.0,
                        nr_auto_min_pct=80.0, nr_auto_max_pct=110.0,
                        source="UH-1H FM notes (torque) + pattern figures",
                        unverified=True),
    # Guimbal Cabri G2 (O-360-J2A carburetted): engine max 2700 RPM,
    # power-on range 2585-2700; rotor nominal 530 RPM -- power-on
    # 515-540, power-off 450-610; CHT 260 C, oil 118 C.
    "CBRI": EngineLimits(nr_min_pct=97.2, nr_max_pct=101.9,
                         nr_auto_min_pct=84.9, nr_auto_max_pct=115.1,
                         rpm_max=2700.0,
                         source="Cabri G2 flight manual issue 08"),
}


# ---------------------------------------------------------------------------
# The fixed-wing table: entries only where a type's own figures were found.
# Everything else resolves through the class ladder below.
# ---------------------------------------------------------------------------

_FW_LIMITS: dict[str, EngineLimits] = {
    # TBM 850/930 family (PT6A-66D): ITT 850 takeoff, 840 max continuous;
    # start <1000 for 5 s / <870 for 20 s; Ng 104.1%; Np 2000 +- 40.
    "TBM9": EngineLimits(ng_max_pct=104.1, itt_takeoff_c=850.0,
                         itt_max_c=840.0, itt_start_c=1000.0,
                         source="TBM 930 PIM + EASA TCDS IM.E.070"),
    "TBM8": EngineLimits(ng_max_pct=104.1, itt_takeoff_c=850.0,
                         itt_max_c=840.0, itt_start_c=1000.0,
                         source="kin: TBM 930 PIM"),
    # PC-12 (PT6A-67B): ITT 800 takeoff and continuous; NG ~101-102.
    "PC12": EngineLimits(ng_max_pct=102.0, itt_takeoff_c=800.0,
                         itt_max_c=800.0, itt_start_c=1000.0,
                         source="EASA TCDS IM.E.070 (PT6A-67B)",
                         unverified=True),
    # King Air C90B (PT6A-21, flat-rated 550 shp): 1,315 ft-lb torque
    # takeoff and max continuous alike (1,500 for 2 s transient),
    # 695 C ITT for both ratings, start transient to 1,090, N1 101.5%.
    # The N-m figures are the ones that matter on X-Plane, whose
    # C90 model reports real shaft torque but leaves every red-line
    # dataref at paint values (red_hi_TRQ = 1.0 ft-lb).
    "BE9L": EngineLimits(tq_takeoff_nm=1783.0, tq_max_nm=1783.0,
                         ng_max_pct=101.5,
                         itt_takeoff_c=700.0, itt_max_c=695.0,
                         itt_start_c=1090.0,
                         source="King Air C90B POH / PT6A-21"),
    # CFM56 family (A320, 737NG): EGT 950 C takeoff, ~925 continuous;
    # N1 ceiling ~104%.
    "A319": EngineLimits(ng_max_pct=104.0, itt_takeoff_c=950.0,
                         itt_max_c=925.0, itt_start_c=725.0,
                         source="CFM56-5B figures, widely published",
                         unverified=True),
    "A320": EngineLimits(ng_max_pct=104.0, itt_takeoff_c=950.0,
                         itt_max_c=925.0, itt_start_c=725.0,
                         source="CFM56-5B figures, widely published",
                         unverified=True),
    "A321": EngineLimits(ng_max_pct=104.0, itt_takeoff_c=950.0,
                         itt_max_c=925.0, itt_start_c=725.0,
                         source="CFM56-5B figures, widely published",
                         unverified=True),
    "B738": EngineLimits(ng_max_pct=104.0, itt_takeoff_c=950.0,
                         itt_max_c=925.0, itt_start_c=725.0,
                         source="CFM56-7B figures, widely published",
                         unverified=True),
    "B37M": EngineLimits(ng_max_pct=105.0, itt_takeoff_c=1060.0,
                         itt_max_c=1040.0, itt_start_c=750.0,
                         source="LEAP-1B figures, widely published",
                         unverified=True),
    "B38M": EngineLimits(ng_max_pct=105.0, itt_takeoff_c=1060.0,
                         itt_max_c=1040.0, itt_start_c=750.0,
                         source="LEAP-1B figures, widely published",
                         unverified=True),
    "A20N": EngineLimits(ng_max_pct=105.0, itt_takeoff_c=1060.0,
                         itt_max_c=1040.0, itt_start_c=750.0,
                         source="LEAP-1A figures, widely published",
                         unverified=True),
    "A21N": EngineLimits(ng_max_pct=105.0, itt_takeoff_c=1060.0,
                         itt_max_c=1040.0, itt_start_c=750.0,
                         source="LEAP-1A figures, widely published",
                         unverified=True),
    # The Cessna pistons' red lines.
    "C152": EngineLimits(rpm_max=2550.0, source="C152 POH (O-235)",
                         unverified=True),
    "C172": EngineLimits(rpm_max=2700.0, source="C172 POH (IO-360)",
                         unverified=True),
    "C182": EngineLimits(rpm_max=2700.0, source="C182 POH (IO-540)",
                         unverified=True),
}

# The class ladder: what a type resolves to when its own manual's numbers
# are not in the tables above. Deliberately inside the published ranges of
# the commonest engines in the class, and marked unverified -- a figure
# that is a pattern is labelled as one.
_CLASS_LIMITS: dict[str, EngineLimits] = {
    "piston_single": EngineLimits(rpm_max=2700.0,
                                  source="class pattern (Lycoming/Continental)",
                                  unverified=True),
    "piston_twin": EngineLimits(rpm_max=2700.0,
                                source="class pattern (Lycoming/Continental)",
                                unverified=True),
    "turboprop_single": EngineLimits(ng_max_pct=104.0, itt_takeoff_c=850.0,
                                     itt_max_c=810.0, itt_start_c=1000.0,
                                     source="class pattern (PT6)", unverified=True),
    "turboprop_twin": EngineLimits(ng_max_pct=104.0, itt_takeoff_c=850.0,
                                   itt_max_c=810.0, itt_start_c=1000.0,
                                   source="class pattern (PW100/PT6)", unverified=True),
    "business_jet": EngineLimits(ng_max_pct=105.0, itt_takeoff_c=950.0,
                                 itt_max_c=925.0, itt_start_c=750.0,
                                 source="class pattern (jet)", unverified=True),
    "airliner": EngineLimits(ng_max_pct=105.0, itt_takeoff_c=950.0,
                             itt_max_c=925.0, itt_start_c=750.0,
                             source="class pattern (CFM56-class)", unverified=True),
    "helicopter": EngineLimits(tq_takeoff_pct=100.0, tq_cont_pct=90.0,
                               ng_max_pct=105.0,
                               nr_min_pct=90.0, nr_max_pct=104.0,
                               nr_auto_min_pct=80.0, nr_auto_max_pct=110.0,
                               source="class pattern (turbine rotorcraft)",
                               unverified=True),
    "": EngineLimits(unverified=True),
}


def limits_for(type_code: str, aircraft_class: str = "") -> EngineLimits:
    """The published limits for this type, or the class's pattern figures."""
    code = (type_code or "").strip().upper()
    found = _HELI_LIMITS.get(code) or _FW_LIMITS.get(code)
    if found is not None:
        return found
    return _CLASS_LIMITS.get(aircraft_class or "", _CLASS_LIMITS[""])


# ---------------------------------------------------------------------------
# The ice kits. A type not listed resolves by class; a class not listed is
# treated as "pitot heat only", which is the honest minimum.
# ---------------------------------------------------------------------------

_HELI_KITS: dict[str, IceKit] = {
    # Robinsons are not certified for known or suspected icing; the R44's
    # carb heat is used OAT -4..30 C and under 18 in MAP regardless.
    "R44": IceKit(carb_heat=True, source="R44 POH section 4"),
    "R66": IceKit(engine_inlet=True,
                  source="R66 POH: anti-ice on in visible moisture <=4C"),
    "H125": IceKit(engine_inlet=True, source="AS350 RFM: no icing flight"),
    "AS50": IceKit(engine_inlet=True, source="class: turbine rotorcraft",
                   ),
    "EC30": IceKit(engine_inlet=True, source="class: turbine rotorcraft"),
    "B06": IceKit(engine_inlet=True,
                  source="206B3 RFM: anti-ice ON <4.4C in moisture"),
    "B407": IceKit(engine_inlet=True,
                   source="407 RFM: anti-ice ON <5C in moisture"),
    "EC35": IceKit(engine_inlet=True, source="class: turbine rotorcraft"),
    "EC45": IceKit(engine_inlet=True, source="class: turbine rotorcraft"),
    "H145": IceKit(engine_inlet=True, source="class: turbine rotorcraft"),
    "H160": IceKit(engine_inlet=True, source="class: turbine rotorcraft"),
    "AS32": IceKit(engine_inlet=True, source="class: turbine rotorcraft"),
    "S76": IceKit(engine_inlet=True, source="class: turbine rotorcraft"),
    "A109": IceKit(engine_inlet=True, source="class: turbine rotorcraft"),
    "A139": IceKit(engine_inlet=True, source="AW139 RFM: no icing flight"),
    # The 412's particle separators are its anti-ice -- automatic above
    # ~53% N1, but the write is harmless where a model has a switch.
    "B412": IceKit(engine_inlet=True,
                   source="Bell 412 PTM (particle separators)"),
    "UH1": IceKit(engine_inlet=True, source="class: turbine rotorcraft"),
    "CBRI": IceKit(carb_heat=True, source="Cabri G2 FM (O-360-J2A)"),
}

_FW_KITS: dict[str, IceKit] = {
    # Carburetted pistons: induction ice, not turbine inlet ice.
    "C152": IceKit(carb_heat=True, source="C152 POH"),
    "J3": IceKit(carb_heat=True, source="class: carburetted piston"),
    "CUB": IceKit(carb_heat=True, source="class: carburetted piston"),
    "DV20": IceKit(carb_heat=True, source="class: carburetted piston"),
    "PA18": IceKit(carb_heat=True, source="class: carburetted piston"),
    # PT6 singles carry the inertial separator; the TBM and PC-12 are
    # certified into known ice with the full kit on.
    "TBM9": IceKit(engine_inlet=True, inertial_separator=True,
                   prop_deice=True, airframe_deice=True, windshield=True,
                   known_ice=True, source="TBM 930 PIM"),
    "TBM8": IceKit(engine_inlet=True, inertial_separator=True,
                   prop_deice=True, airframe_deice=True, windshield=True,
                   known_ice=True, source="kin: TBM 930 PIM"),
    "PC12": IceKit(engine_inlet=True, inertial_separator=True,
                   prop_deice=True, airframe_deice=True, windshield=True,
                   known_ice=True, source="PC-12 AFM figures"),
    "C208": IceKit(engine_inlet=True, inertial_separator=True,
                   prop_deice=True, airframe_deice=True, windshield=True,
                   known_ice=True, source="C208 AFM figures"),
    "KODI": IceKit(engine_inlet=True, inertial_separator=True,
                   prop_deice=True, airframe_deice=True, windshield=True,
                   source="Kodiak 100 POH figures"),
    "M600": IceKit(engine_inlet=True, inertial_separator=True,
                   prop_deice=True, airframe_deice=True, windshield=True,
                   source="M600 AFM figures"),
    # The turboprop twins are FIKI-certified across the board.
    "AT76": IceKit(engine_inlet=True, prop_deice=True, airframe_deice=True,
                   windshield=True, known_ice=True, source="ATR 72 AFM"),
    "AT72": IceKit(engine_inlet=True, prop_deice=True, airframe_deice=True,
                   windshield=True, known_ice=True, source="ATR 72 AFM"),
    "DH8D": IceKit(engine_inlet=True, prop_deice=True, airframe_deice=True,
                   windshield=True, known_ice=True, source="Q400 AFM"),
    # The DA62's FIKI option (TKS) is modelled as airframe de-ice.
    "DA62": IceKit(prop_deice=True, airframe_deice=True, windshield=True,
                   known_ice=True, source="DA62 AFM (FIKI option)",
                   ),
}

_CLASS_KITS: dict[str, IceKit] = {
    "piston_single": IceKit(source="class: piston single"),
    "piston_twin": IceKit(source="class: piston twin"),
    "turboprop_single": IceKit(engine_inlet=True, prop_deice=True,
                               airframe_deice=True, windshield=True,
                               source="class: turboprop single"),
    "turboprop_twin": IceKit(engine_inlet=True, prop_deice=True,
                             airframe_deice=True, windshield=True,
                             known_ice=True,
                             source="class: turboprop twin"),
    "business_jet": IceKit(engine_inlet=True, airframe_deice=True,
                           windshield=True, known_ice=True,
                           source="class: business jet"),
    "airliner": IceKit(engine_inlet=True, airframe_deice=True,
                       windshield=True, known_ice=True,
                       source="class: airliner"),
    "helicopter": IceKit(source="class: rotorcraft"),
    "": IceKit(),
}


def ice_kit_for(type_code: str, aircraft_class: str = "") -> IceKit:
    """The ice protection this type carries, or the class's kit."""
    code = (type_code or "").strip().upper()
    found = _HELI_KITS.get(code) or _FW_KITS.get(code)
    if found is not None:
        return found
    return _CLASS_KITS.get(aircraft_class or "", _CLASS_KITS[""])


# ---------------------------------------------------------------------------
# The cooling kits: which types put cowl flaps in the pilot's hands, and
# the cylinder-head red line the flap logic works against. Cowl flaps are
# cooling, not anti-ice -- they feed ram air across the cylinders and are
# an air-cooled piston's fixture: turboprops and jets cool themselves.
# The rules the autonomous watch applies are the POH ones: flaps open on
# the ground, for takeoff, in the climb and on the go-around (high power
# at low speed), closed in cruise and descent, and opened again whenever
# the CHT gauge runs toward its ceiling.
#
# ``cht_max_c`` is the published CHT red line in Celsius -- 460 F for the
# Continental big-bores, 500 F for the Lycomings -- and a type not listed
# resolves by class: pistons get the conservative 238 C, and
# ``cowl_flaps`` False, because the commonest trainers (C172, PA28, DA40)
# have fixed cowls and writing a lever the aeroplane does not have is the
# failure the tables exist to prevent.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class CoolingKit:
    """The engine cooling a type carries. ``cht_max_c`` 0.0 = unpublished."""

    cowl_flaps: bool = False
    cht_max_c: float = 0.0
    source: str = ""


_FW_COOLING: dict[str, CoolingKit] = {
    # Beech Bonanzas and Barons (Continental IO-520/IO-550): red line
    # 460 F = 238 C; POH wants flaps open for ground ops, takeoff and
    # climb, closed in cruise and descent, "as required" for CHT.
    "BE35": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="Bonanza V35 POH (IO-520)"),
    "BE36": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="A36 Bonanza POH (IO-550)"),
    "BE55": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="Baron 55 POH (IO-470)"),
    "BE58": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="Baron 58 POH (IO-550)"),
    # Cessna singles and twins. The 172's fixed cowl is the exception
    # that proves the rule: from the Cardinal up the cowl flaps are
    # pilot-operated.
    "C177": CoolingKit(cowl_flaps=True, cht_max_c=260.0,
                       source="C177 Cardinal POH (IO-360)"),
    "C182": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="C182 Skylane POH (O-470/TIO-540)"),
    "C206": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="C206 Stationair POH (IO-540)"),
    "C210": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="C210 Centurion POH (IO-550)"),
    "C310": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="C310 POH (IO-470)"),
    "C340": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="C340 POH (TSIO-520)"),
    "C414": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="C414 POH (TSIO-520)"),
    "C421": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="C421 POH (GTSIO-520)"),
    # Piper's bigger pistons: the Comanche, Seneca and Malibu lines.
    # The PA-28's fixed cowl and the PA-44 Seminole's fixed scoops do
    # not appear -- they have nothing to write.
    "PA24": CoolingKit(cowl_flaps=True, cht_max_c=260.0,
                       source="PA-24 Comanche POH (IO-540)"),
    "P28R": CoolingKit(cowl_flaps=True, cht_max_c=260.0,
                       source="PA-28R Arrow POH (IO-360)"),
    "PA34": CoolingKit(cowl_flaps=True, cht_max_c=260.0,
                       source="PA-34 Seneca POH (IO-360)"),
    "PA46": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="PA-46 Malibu/Matrix POH (TIO-540)"),
    # Mooneys and Cirruses both carry the levers.
    "M20P": CoolingKit(cowl_flaps=True, cht_max_c=260.0,
                       source="M20J POH (IO-360)"),
    "M20T": CoolingKit(cowl_flaps=True, cht_max_c=260.0,
                       source="M20M Bravo POH (TIO-540)"),
    "SR20": CoolingKit(cowl_flaps=True, cht_max_c=260.0,
                       source="SR20 POH (IO-360)"),
    "SR22": CoolingKit(cowl_flaps=True, cht_max_c=238.0,
                       source="SR22 POH (IO-550)"),
    # Socata's TIO-540 singles.
    "TB20": CoolingKit(cowl_flaps=True, cht_max_c=260.0,
                       source="TB-20 Trinidad POH (TIO-540)"),
    "TB30": CoolingKit(cowl_flaps=True, cht_max_c=260.0,
                       source="TB-30 Epsilon POH (TIO-540)"),
}

# Helicopter cowl flaps: the Robinsons' cooling is fan-driven and the
# turbines cool through oil, so no rotorcraft entry opens flaps. The
# piston helis' CHT ceilings are still worth the warning.
_HELI_COOLING: dict[str, CoolingKit] = {
    "R44": CoolingKit(cht_max_c=260.0, source="R44 POH (O-540): "
                      "500 F CHT red line"),
    "CBRI": CoolingKit(cht_max_c=260.0, source="Cabri G2 FM: "
                       "CHT 260 C"),
}

_CLASS_COOLING: dict[str, CoolingKit] = {
    # The conservative Continental figure stands in for the class: a
    # model publishing CHT gets the warning; flaps stay unwritten where
    # the type's own entry does not say it has them.
    "piston_single": CoolingKit(cht_max_c=238.0,
                                source="class pattern (Continental red line)"),
    "piston_twin": CoolingKit(cht_max_c=238.0,
                              source="class pattern (Continental red line)"),
    "helicopter": CoolingKit(cht_max_c=238.0,
                             source="class pattern (piston rotorcraft)"),
    "": CoolingKit(),
}


def cooling_for(type_code: str, aircraft_class: str = "") -> CoolingKit:
    """The cooling this type carries, or the class's figures."""
    code = (type_code or "").strip().upper()
    found = _HELI_COOLING.get(code) or _FW_COOLING.get(code)
    if found is not None:
        return found
    return _CLASS_COOLING.get(aircraft_class or "", _CLASS_COOLING[""])


__all__ = ["EngineLimits", "IceKit", "CoolingKit", "limits_for",
           "ice_kit_for", "cooling_for"]
