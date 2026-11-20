# MeridianATC lifelike-behavior audit

Line-by-line passes for *realism* issues — places where the sim's behavior,
phrasing, or procedures diverge from real-world aviation (FAA JO 7110.65,
AIM, ICAO Doc 4444/9432/8168, Annex 3/10, CAP 413, aircraft profiles, real
radio practice). ~140,590 lines of Python audited across six passes.

**Result: 30 findings — 0 high, 2 suspected, 8 low, 20 info/note.** The
dominant pattern is that this codebase is unusually well-grounded in the
real thing: measured live-sim behavior (Madeira, Innsbruck, Bilbao,
Fiumicino sessions), the running simulator taken as authority over static
datasets, and real-world procedures modelled (GSX menu walks, ARINC 424
decoding, VATSIM deference, observed-AI treated as weather). The remaining
issues are mostly phraseology dialect edges and deliberately-coarse
approximations that are documented as such in code.

## Fixes applied (this branch)

| Finding | Fix |
| --- | --- |
| `ifr_cancelled` hardcoded "squawk VFR" | now issues the regional conspicuity code (7000 ICAO / 1200 FAA) via `vfr_squawk` |
| `no_reply` "how do you hear" for all dialects | FAA "how do you hear me" (AIM glossary) / ICAO "how do you read" (Doc 4444, CAP 413) |
| `runway_change` ICAO "runway in use now X" | "runway in use X" |
| `pushback_approved` "push and start" for all | FAA "pushback approved" / ICAO "push and start approved" |
| `vector_to_final` FAA-shaped only | ICAO branch: turn, maintain until established, "cleared X approach runway Y, report established" (matches `intercept_clearance`) |
| `cleared_approach` ICAO clearance-first order | altitude condition first, clearance last (Doc 9432 order, same as FAA branch) |
| `hold_clearance` ICAO missing LEVEL | "hold at (fix) (level)" per Doc 4444 12.3.4 template |
| `_step_down`/`_next_level_down` 10k/4k steps | 8,000 ft above FL240, 3,000 below |
| `DEPARTURE_STEPS_FT` 6-12k | (3000, 4000, 5000, 6000) — real TRACON increments |
| `LANDING_CLEARANCE_NM` 12 | 10 (matches arrival.py) |
| `ACROSS_ALIGNED_NM` 0.04 (~74 m) | 0.03 (~55 m) — a close parallel taxiway no longer reads as on-runway; still inside the widest runway half-width plus margin |
| `_resolve_fix` 250 NM fuzzy match | 150 NM — within a radar footprint |
| NL interphone had only first-officer words | Dutch CABIN/GROUND addresses + ask patterns added (en/fr/de parity) |

Left as designed (verified plausible): `_next_level_up` 2,000-ft steps
(approach-common), CROSS_CLEAR_S/CROSSING_TAKES_S crossing window,
`expect_minutes` default 10, `_valid_level` on track not route (edge case),
hold EFC wording, circuit constants, heli speed table, vector step-down
250 ft/NM, overhead join, _procedure_default +2500, flight-following
workload refusal, orbit to non-circuit side.

Format: severity · file:line · what happens · what the real world does.

## Progress tracker (for context resets)

DONE: phraseology.py (2653), controller/core.py, x_what_the_controller.py,
x_entry_point.py, x_tower.py, x_ground.py, x_clearance_delivery.py,
x_the_circuit.py, x_radar_positions_departure.py, x_working_the_user.py,
x_what_the_arrival.py, x_helpers.py, x_vfr_helpers.py, x_readback_checking.py,
_shared.py, x_emergencies.py, x_uncontrolled_fields.py — CONTROLLER DONE.
arrival.py: lines 1-426 read; constants reviewed — DESCENT_GRADIENT_NM=3
(3NM/1000ft correct jet profile), DESCENT_LEAD_MIN=5, EXPECT_LOWER_MIN=(2,6),
CENTRE_ABOVE_FIELD_FT=10000, INTERCEPT_ABOVE_FT=3000, EXPECT_APPROACH_NM=35,
CLEARED_APPROACH_NM=18, APPROACH_CONE_DEG=45, LANDING_CLEARANCE_NM=10,
speed table PATTERN/NEAR {jet:210/170, prop:200/150, heli:100/60},
FINAL_SPEED_KT=160 until 4NM — all match 7110.65 5-7-3 practice.
CROSS_FROM/TO_NM 30-60 for crossing restriction — plausible.
arrival.py DONE (1858 read in full):
- LOW · `_step_down` descends arrivals in 10,000-ft steps above FL240 and
  4,000-ft steps below — real centers use finer steps (typically 3-4k in
  the low flight levels, 1-2k near approach); 10,000 ft is a big first
  bite. Behavior acceptable, granularity coarse.
- LOW · `_speed_control` only acts inside VECTOR_FROM_NM and below 10,000
  — matches 7110.65 spirit; "160 to 4 NM" final speed is ICAO/UK practice
  applied to all dialects; fine.
- NOTE · heli speed table jet:210/170 prop:200/150 heli:100/60 — 7110.65
  5-7-3 lists helicopter minimum 60 kt; prop minimum "150 within 20 NM"
  and jet "170 within 20 flying miles" — matches.
- NOTE · hold logic: EFC updated only when moved >2 min or expired;
  sequence restated ≤ every 2.5 min — matches real practice of reissuing
  EFC on change. `verify_holding` when off-pattern — real.
- NOTE · vector step-down ~250 ft/NM over field — matches 3° path.
departure.py DONE (449): SETTLE_S=4 post-vacate delay, EXPEDITE_AFTER_S=12
/ WITHIN_NM=3, ABORT_AFTER_S=20 runway-abort-to-taxi-in, SidWatch
OFF_PATH_NM=2 OFF_FOR_S=30 — all plausible.
watch.py DONE (1189): ENTRY_RULES correct per Annex 11/AIM; deviation
tolerances ALT 200 / HDG 20° / SPD 15 kt; transponder settle 30 s;
RadarServiceWatch LEAVING_NM=15; RestrictionWatch ABOVE_FT=300.
climb.py DONE (186): DEPARTURE_TOP_FT=17000, initial field+3000→4000-10000,
DEPARTURE_STEPS 6-12k (large vs real ~3-6k, LOW), CENTRE_STEPS FL230-280.
circuit.py DONE (381): circuit dims 1.6 upwind/1.1 wide/1.4 base NM —
realistic; OVERHEAD_REGIONS={uk,fr} correct.
enroute.py DONE (689): sector model sound; invented freqs 124-136@25kHz;
ULTRA_HIGH_FL350 (artcc ultra-high real ~FL350+). OK.
vectors.py DONE (252): JET pattern gate 9/base 17/offset 6 NM + 30°
intercept — matches real vectoring; SLOW under 160kt 5/10/3.5; turn radius
rate-1 or 25° bank — correct.
occupancy.py DONE (132):
- LOW · ACROSS_ALIGNED_NM=0.04 NM ≈ 74 m — an aligned aircraft on a close
  parallel taxiway (~60-75 m off centreline) can read as "on the runway".
  Deliberate tolerance; border case.
afis.py DONE (626): AFIS/FIS/FSS split by name+field — correct service
modeling; HJ/HN hours via solar elevation; QDM/QDR correct.
atis.py DONE (482): issue triggers (altimeter 0.01", wind ≥5kt, dir ≥30°,
vis step, ceiling change, temp ≥2°C) match real re-record practice; LVP
remark at vis<0.5SM/ceiling<200ft — correct order of magnitude.
svfr.py DONE (172): FAA 1000ft/3SM + SVFR 1SM clear-of-cloud, elsewhere
1500ft/5km + SVFR 600ft/1500m — matches 91.155/91.157 and SERA.5005/5010.
svfr_altitude: 200 ft under cloud base, floor 500 AGL — invented but sane.
trafficinfo.py DONE (447): calls within 5NM/1500ft VFR advisories, 5NM/
1000ft IFR centres — matches 7110.65 2-1-21 practice; wake caution ranking
super>heavy>medium correct.
approaches.py DONE (535): minima table plausible (ILS 200/0.5, LPV 250/0.75,
LNAV/VNAV 350/1.0, NDB 650/1.25); circling TERPS radii 1.3-2.3 correct;
letter Z-first correct; visual offered ≥3000ft/5SM in faa region.
procedures.py DONE (306): SID designator decode/namesake correct; STAR
join cone geometry plausible.
taxi.py DONE (761): route on real taxiway graph, ≤5 taxiways then split
clearance (matches controller practice), hold-short = first runway crossed,
intersection-departure naming correct per 7110.65 3-7-1.
session.py DONE (814): REQUIRED_READBACK = alt/hdg/rwy/squawk/freq/
hold_short/speed — matches AIM 4-4-7 + Doc 4444 4.5.7.5; readback chase
grace 35s plausible. NOTE altimeter setting not required in readback —
4444 4.5.7.5 does list it (LOW).
live.py DONE (562): ADS-B delay-shown-behind 30s, carry 60s air/8s ground,
land-the-final-on-feed-loss (3° glide) — lifelike handling.
final.py DONE (141): 30° cone, aligned within 45° — plausible.
holds.py DONE (187): 1-min legs ≤14000 ft / 1.5 above (Doc 8168/AIM 5-3-8)
correct; hold-ahead 6 FAA / 10 ICAO, spacing 2 min — plausible choices.
flows.py DONE (236): apt.dat flow model correct; 'heavy' = widebodies only
correct.
zones.py DONE (115): ramp-vehicle footprint conflicts — sound.
surface.py DONE (150): stand/edge locate — fine.
flying.py DONE (494): rate-1/5° turn bank-limited, takeoff accel 4.2 kt/s
(→140 kt in ~33 s), climb-onset 450 fpm/s, flare 40 ft/160 fpm, energy
coupling + wind triangle — all plausible.
ground.py DONE (1011): taxi graph on sim taxiways; taxi 10.3 m/s
straight/5.1 corner/2.5 sharp, accel .6 brake .9 m/s², pushback 1.4 m/s —
realistic; HOLD_OFF_CENTRELINE 45m, INTERSECTION 60m, TURN_S=45 for runway
180; stand-size/gate-vs-ramp assignment plausible.
auto_nav.py DONE (349): taxi lookahead 120 m, path reached 25 m — sound.
restate.py DONE (74): instruction re-issue in the new language — sound.
notices.py DONE (256): FO notices — gear-up final at 500 ft AGL, xpdr
standby at line-up, QNH unset after 30 s below TA, freq unset after 20 s —
all plausible; only published sim vars checked (no false alarms).
cpdlc.py DONE (151): WILCO/UNABLE/ROGER semantics correct; only airliner/
bizjet/turboprop_twin equipped — plausible.
aircraft_class.py DONE (358): Doc 8643 table sound (amphibians/seaplanes
incl.); heavies need 2 usable rwys or ≥10000 ft — plausible; min_runway_ft
table plausible (C172 2500, airliner 7000-9000, CONC 9000).
vspeeds.py DONE (292): sourced speeds (C172 55/65, TBM9 90/85, B738
128/132/141/140, CONC 175/197/215/170, helicopters' vapp table) — plausible;
sensible pilot-stated > sim-declared > table > class precedence.
local_time.py DONE (37): solar hour by longitude — fine.
letters.py DONE (109): STT misheard-letter snap — fine.
numbers.py DONE (369): digit-vs-cardinal split correct per language;
German zwo, Dutch ën-joiner, compound reading — sound.
language.py DONE (439): bilingual model = Annex 10 English + local where
spoken; reply-in-pilot's-language — correct; accent fallbacks deliberate.
vocabulary.py DONE (279): STT aviation-word snap — fine.
written.py DONE (406): spoken→figures strip rendering — sound.
airlines.py DONE (143): telephony table + initialisms — fine.
callsigns.py DONE (239): FAA grouped vs ICAO digit-by-digit flight numbers,
fr whole number — correct per 7110.65 2-4-20/Doc 9432; airline callsign
never abbreviated (4444 12.3.1.9) — correct.
operators.py DONE (484): regional airline rosters + leisure geography +
crew-language-by-field — plausible model; cabin languages table correct.
fleet.py DONE (216): airfield vs airline field by scheduled service +
runway; circuit shares .60/.45/.35, in_play 3/5 — plausible; all flagged
UNVERIFIED choices.
lod.py DONE (367): detailed/light model swap with hysteresis band, cap,
no-swap-in-sight rules — lifelike.
speech.py DONE (965): digits FAA/ICAO, group_form correct (7110.65
2-4-20), telephony+GA type prefix correct, TA/TL handling right
(is_flight_level incl. 18k exception), runway leading-zero ICAO vs FAA,
gust ≥10kt over mean ≥3kt mean (Annex 3), visibility quarters — all
correct. NOTE visibility reports statute miles as "one zero" for ≥10 —
US-only convention used everywhere; ICAO metres handled elsewhere?
(low/verify). NOTE altimeter faa reads inHg digits without decimal —
correct.
phrasebook.py DONE (1944): localised engine — QNH hPa all langs, rwy
leading zero, land-after/line-up-behind forms, intersection remaining-
distance, gust rule reused, ICAO-order approach clearance, hold clearance
ICAO form, descend_via names STAR + level, SVFR/AFIS/emergency forms,
incursion chiding with "report will be filed" — structure sound.
phrasebooks.py DONE (1473): es+de+it+pt books — correct national
phrasing (motor y al aire, Start frei, riattacchi, arremeta, Rollhalt,
push-back loanwords). LOW: de legs "upwind"="Gegenanflug entgegen"
coined; real German circuit is Abflug/Gegenanflug/Queranflug/Endanflug —
upwind should be "Abflug" (verify DFS).
phraseology_fr.py PART (905/1706): French class — correct DGAC forms
(collationnement, point d'attente, alignez-vous et attendez, autorisé
décollage/atterrissage, quittez la CTR sans "autorisé", MVL circling,
posé-décollé, tour de piste main X, remettez les gaz, montez/descendez
imperative). So far sound.
phraseology_fr.py DONE (1706): hold clearance (rapprochement/éloignement/
heure d'approche prévue — correct DGAC order), SVFR, transit autorisé,
AFIS à votre convenance, incursion compte rendu d'incident, emergencies —
all sound. Many UNVERIFIED flags but forms plausible.
speech_fr.py DONE (527): digit-vs-cardinal split correct (runway/hdg/
squawk/FL digits; QNH/freq/alt/cardinal); fr cardinal counting right
(soixante-dix, quatre-vingts, et un); flight numbers as whole French
number — correct French practice; visibility m/km conversion correct;
airline-shape callsign never abbreviated — correct.
NOTE: _RUNWAY_SUFFIX_FR "G": "gauche" (grass→left!) — runway 04G spoken
"piste zéro quatre gauche", indistinguishable from 04L; LOW (grass
suffix rare, but "gazon"/"herbe" would be the DGAC word).
intents.py DONE (1317): parser — explicit number patterns, UNKNOWN → say
again, flight_rules() believes pilot over config, last-circuit/full-stop,
squawk-thousand, hold-short-vs-runway, callsign stripping at ends only,
readback-vs-request priority — all sound.
intents_fr.py DONE (659): French parser — accent folding, digit-vs-
cardinal per field, expected-vs-cleared level split, hold-short runway
exclusion, point d'attente=held-not-runway distinction — all sound.
intents_local.py DONE (1000): shared es/de/it/pt parser — accent folding,
per-grammar fields, expected-vs-cleared split, hold-short-vs-holding-
point, per-language Grammar rules all plausible.
servicelayout.py DONE (432/470 read, rest is spot assembly): ground-
vehicle docking from real door points, sill heights vs GSX data, belt/
stairs geometry — deeply lifelike.
autofly.py DONE (303): profile matching/validation, borrowed-profile
speed scaling clamped 0.6-1.6x — sound config logic, no behavior issues.
ap.py DONE (670): FlightDirector — managed vs direct AP, explicit
setters, axis slew, per-engine levers, KOHLSMAN_SET hPa*16 correct,
GEAR_SET binary, brake split axes — all sound. NOTE LOW: seatbelts()
and pushback() use toggles — a second on() call flips them off.
control.py DONE (190): PID with anti-windup, derivative-on-measurement,
slew+deadband+wrap — good control law design, realistic "hand" model.
control_gains.py DONE (202): per-class gain bundles (airliner/turboprop/
light; rotor singles vs twins) — plausible, UNVERIFIED flagged.
continuous.py DONE (188): mission builder, reserve_factor, runway-
usability check for destination — sound.
autonomous/x_the_approach_s.py(61), __init__.py(85) DONE: runway resol.
x_the_stall.py DONE (180): stall detect (horn/aoa/speed, 50ft gate,
persistence 0.5/1.0s), recovery = power+push 0.45 not bunt+wings level
at 40deg scale — correct real recovery technique.
autonomous/ = 23 files, 11,072 lines.
x_the_load(169) DONE: G watch 85% warn/limit ease, VS=0+bank cap 15 on
AP legs, touchdown report 1.8/2.2G — realistic.
x_filed_cleared_sid(194) DONE: SID/STAR nav honors assigned hdg/alt/spd
over procedure — correct priority.
x_the_envelope(200) DONE: envelope limits (barber pole 0.95, VS1*1.3
floor, flap/gear speed gates, normal-cat G by class, owed-flap reselect)
— all sound.
x_plumbing(210) DONE: perf/speeds cache stamped by type_code, borrowed
profile adaptation, Vapp floor 1.25*Vs1 — sound.
x_emergencies(213) DONE: learned healthy bus ref, all-engines-out MAYDAY
vs one-out PAN PAN, fuel leg shortage=MAYDAY vs leg+div+reserve=min
fuel, persist timers (45/120/8/5s), divert held to 400ft AGL — correct
real-world emergency logic.
core(214) DONE: mixin assembly, timeouts plausible (clearance 240s).
x_the_turnaround_between(233) DONE: engines-off-first refuel, GSX truck
then direct write+gauge verify, leg+reserve departure short-of-diversion
fig — realistic crew logic.
x_the_engines(241) DONE: takeoff vs continuous ITT/TQ ratings, carb heat
per POH (-4..30C moist or low power ~18in MAP), cowl flaps POH schedule
+CHT hysteresis — all correct.
x_fuel_planning(250) DONE: Jet-A 6.7/avgas 6.0 lb/gal, leg+divert+
reserve pricing, alternate pick, MAYDAY 7700+divert, <400ft AGL pending
turn, emergency descent elev+2500 — realistic.
x_the_loop(266) DONE: 1Hz tick + 0.25s fast hand-flown subs, monitor
order comms→weather→engines→emerg→load→traffic→stall→heli delegate —
sound cadence design.
x_the_loop(266) done.
x_helpers_kept_from(258) DONE: plausible still-air ranges, FLCH-off
before ALT+SPD capture, ENGINE_AUTO_START w/ props+mixture first —
sound.
x_chain(289) DONE: hemispheric odd-east/even-west filing w/ ceiling
bounds, destination suitability+services check, session reset —
realistic.
x_entry(365) DONE: air engage joins at correct phase (no preflight redo),
handoff centers surfaces+only FO's own AP modes off, scratch reset
thorough — sound.
x_circling(475) DONE: TOD 318ft/NM+slowdown rule (3deg rule-of-thumb —
correct), stabilized gate <1000ft (<1000fpm,±12kt,bank,align+3deg path
gate), GA criteria (below minima IFR no clearance, unstable, gear-up
once-then-land, runway blocked, 1/3-rwy-past, unstabilized<200ft),
missed=power-pitch-flap-gear order, 3GA→alternate, 2 weather-GA→divert —
all realistic.
_shared(485) DONE: NWS tiller fade 20-70kt (correct real A320/737
figures), yaw-rate cap 15→6 d/s with GS, per-band yaw authority —
sound.
x_the_cleared_approach(537) DONE: NAV1 tune+readback, ILS-APR vs
LOC+path vs computed final correctly split, back-course excluded,
assigned-speed→Vapp <1500ft/4NM, 3° path VS w/ under-path climb-back,
circuit rejoin (downwind/base w/ class widths), vector held until
joined (≤60° cut), wind crab via track-heading diff smoothed — all
realistic.
x_the_radios(591) DONE: phase-position table (DEL→GND→TWR), CTAF/INFO
fallback, readback-first ordering, squawk/QNH dialled ONLY after
readback, radar handoff armed-not-dialled on ground, staffed-position
pref, unkey 1.2s beat, per-station call clocks, ready re-ask 90s after
standby — deeply realistic comm discipline.
x_weather(629) DONE: TAT icing -30..+6C moisture, per-type ice kit,
density alt 120ft/degC formula, VFR→IFR pickup 25s in cloud, windshear
15kt swing/10kt updraft<500ft→GA, departure escape max-pwr config-held,
class wind limits (30/15,25/10,15/10), 20-min weather holds then abort,
Vapp+half-gust cap +15, airliners always IFR — all realistic.
x_taxi(716) DONE: brake check on first roll, taxi light+strobe on rwy/
crossing, corner slowdown ladder (hairpin 2.5kt), nose-to-bars by class
+wake, yield corridor 30m/90m, off-path stop+re-ask×2, taildragger
S-turn taxi, expedite ×2.2 runway-only — deeply realistic.
x_the_checklist_answered(881) DONE: FO performs+verifies items, toggle-
once vars, no off-while-moving (brake/masters), AFH stick positions on
ground, diff braking castor/tailwheel, breakaway lever, pedal sign
learning per GS band — realistic.
x_shared_helpers(1332) DONE: see-and-avoid 0.6NM/400ft/45s head-on
RIGHT turn (correct rules of air), hold racetrack right-turns timing
from wings-level, direct-to progression, orbit ±90 lead trick, VFR
"not above" binding 30NM, best-glide 1.4Vs all-out vs drift-down
engine-out, climb-rate gives way to speed at 1.15Vs, indicated-alt for
clearances — all realistic.
x_the_phase_machine(1998) READING: 1-1106 sound — readback-before-roll,
LUAW-vs-takeoff, runway re-check, backtrack sub-state, intersection-ask
30%, CTAF self-announce 20s, weight/runway/weather/fuel gates; roll:
NWS→rudder fade, into-wind aileron taper, 70/50 midpoint reject <V1,
V1 continue off end, engine-out/traffic abort <V1, rotate PID vs-rate
cue+pitch learn, RTO=idle+max brake+hold then re-clear.
phase_machine 1480-1869 sound: flare=decaying sink target +learned bias,
power off by class (30/20ft), de-crab at power-off, reversers>60kt then
brakes graduated, taildragger stick-back rollout, spoilers stow on taxi,
vacated-call retry — realistic.
phase_machine 1106-1480 sound: gear@pos-rate >200ft, flaps up >V2+20
(no retract in shear), wings-level initial climb, AP engage >400ft;
filed-profile crossing caps climb/descent, step-clearance handled,
TOD by 318ft/NM, descent config staged <8000/2200/1500, 250kt<10k,
level capture proportional ±250 — realistic.
x_the_phase_machine(1998) DONE: tail 1869-1998 — shutdown cools
turbines at idle, beacon on till electrics off, chain turnaround
engines-off delay — all sound. Whole file = realistic.
autonomous_heli.py(1555) DONE: H-V curve departure (level accel to
transition — correct), 6° approach to spot, 3-stage autorotation
(coll down→flare→pull) + Nr-governor, low-g aft cyclic per Robinson
POH, type envelope limits — realistic. LOW nit: rotor brake applied
while engines_running at <40%Nr before ENGINE_STOP (real: shut down
first, brake on spool-down) — ordering, not safety.
callouts.py(462) DONE: class+rules correct (80kt/Airbus 100, V1 only
known, positive-rate 200fpm+30ft, loc/GS alive IFR-only, 1000/500 for
heavy/heli/IFR, minimums DA-vs-DH by field+100, deviation calls
<1000ft) — realistic. LOW: "altitude" dev at ±300ft (typical SOP call
100ft); speed ±10 symmetric vs typical +10/-5.
adaptation.py(544) DONE: per-model learn store (trims/scales/signs/
values), AdaptivePid = pilot-learning-the-aircraft — not a realism gap.
approach_flight.py(420) DONE: 3°/318ft/NM, TCH 50ft, FAF from published
alt, LOC BC needles handled, missed default elev+2500, circling=generic
rectangle (UNVERIFIED, real=sector-specific — flagged in code) — sound.
landing.py(436) DONE: 200fpm applause, 600 hard=inspection threshold,
bounce detect, on-time ±15min airline standard, stable=vs_spread — fine.
follow.py(475) DONE: predict+blend like netgames, filleted taxi curves,
pitch=gamma+AoA(7 climb/5 desc), coord-turn bank, rotate cap 10°, gear
up pos-rate 2s/50ft, down <2000ft — sound.
guidance.py(102) DONE: phase next-call hints — sound.
handling.py(1713) DONE: rest (bags deadline before holds shut, bus runs,
zone waits on free bearing, tug unbudgeted+priority) — all sound.
powerplant.py(585) DONE: full — ice kits (R44 carb heat, turbine inlets
correct, FIKI tables right, no-icing types flagged), cowl-flap table
(C172 correctly omitted, 238C Continental/260C Lycoming CHT redlines)
— sound.
radio_power.py(87) DONE: COM STATUS+bus V, engine-running settles it,
study-add-on bypass — realistic.
systems.py(121) DONE: nav on stand, beacon@doors-shut, taxi out,
strobes+landing@lineup, landing<10k, shutdown 60-120s post-block —
correct airline day.
tailwheel.py(212) DONE: AFH 14 stick positions correct (climb-into/
dive-away), castoring types right (Cirrus/AA/DA20/Erc), type table
right (DC3, RV7/8/14 tailwheel, RV10/12 & PA-22 excluded), diff-brake
fade, S-turns — realistic.
profile.py(140) DONE: pilot-declared profile+region, class families
correct, mismatch→question not override — sound.
detect.py(179) DONE: grammar+word language detection, English-default —
sound.
trainer.py(150) DONE: keyterm/correction store — infrastructure.
copilot.py(326) DONE: readback via PilotSpeech identical to traffic,
per-kind frames right (vfr_release own, approach not hold-short),
descent-verb detect, transition-aware FL words, wilco for numberless
— realistic. PASS 2 COMPLETE.
PASS 3 traffic: centre_traffic.py(455) DONE: semicircular rule
(odd eastbound, RVSM), sector membership, live-traffic substitution,
check-in/level-change/handoff murmur — realistic.
traffic/_shared(1281) DONE: runway resource model, 2.6s readback gap,
arrivals priority, initial climb 3000/accel 1000 AGL, approach speeds
140→250kt spread 15-4NM, VFR squawk 2000, live-pose gates, head-on
yield→keep-right deadlock escape — realistic.
traffic/x_the_live_ones(1355) DONE: live-pose snap, roll-in to
stand, parked kept 45min, phase-by-geometry (pushback=tail-first
>120°), live departure announced 30s ahead, landing clearance
held while runway occupied, pilot-protect (arrival<2NM behind
pilot→go-around takeover, departures held & released 90s apart,
give-way/keep-right deadlock escape), adopt/unadopt — realistic.
traffic/x_flying_them(952) DONE: V2+20 to accel height, STAR at entry
fix w/ constraints, circuit by position (extend downwind honored
pre-readback, base gap judgment pre-turn), hold 220kt & 1/1.5min
legs, missed course-reversal steer, rotate→liftoff delay, relevance
sort runway/final first — realistic.
traffic/x_the_tower_watching(729) DONE: go-around<1.3NM blocked
(predicted-free-at-arrival not clear-now), spacing 3.5NM via speed
(≥0.85×approach, resume when gap opens), conflict 75s/1NM/600ft &
head-on→right turn (rules of air), stable gates 1.5/0.5NM 60m 20°,
wind-check on moved wind, gust-spread/ceiling GA, expedite 10s w/
final<3NM, extend-until-room then give-up 20NM — realistic.
traffic/x_running_them(595) DONE: taxi lookahead 90m + braking,
intersection claim 95m, head-on give-way (departure yields to
arrival), deadlock: replan-round→vacate-back-to-stand→tow last
resort, runway crossing needs 90s + stretch time clear,
reservations released by driving past/airborne prune — realistic.
traffic/x_where_it_is(559) DONE: departure gap in seconds
(roll45+margin25), LUAW only when one rolling (never behind
stationary pilot on strip), one departure at a time, intersection
detection+remaining (7110.65 3-7-1), missed: published→hold 2-5min
→resequence, IFR uncoded→climb to missed alt→stream, VFR→circuit —
realistic.
traffic/x_the_field(497) DONE: stations only real controllers
(no enroute standin making uncontrolled look staffed), CTAF/AFIS
detect, runway_locked for user-assigned, scenery flows decide
sets, queue-depth parallel assignment, per-end threshold
elevations (Funchal slope), hold-point queue offset — realistic.
traffic/x_arrivals(452) DONE: VFR self-calls & joins downwind (no
instrument approach), STAR check-in→descend-via, hold issued when
queue>runway (fix/EFC/leg-times, present-position fallback),
handoff 12NM, clearance 8NM, slow<30kt→taxi plan + GND handoff,
parked→static for turnaround, touch-and-go 70% re-ask — realistic.
traffic/x_the_ground_driven(411) DONE: stands airline→gate /
freighter→cargo / light→ramp, roster-weighted ops filtered by
runway length + livery, per-country registration formats, route
ends by range/region (regional ~350NM, long-haul heavy ≥200),
SID/STAR/approach picked by runway+route — realistic.
traffic/__init__(291) DONE: re-exports only.
traffic/x_the_watchdog(269) DONE: stuck detect runway20/taxi60/
air30s w/ waits-on-purpose & queued exemptions, one recovery then
removal/unadopt — realistic.
traffic/x_departures(250) DONE: clearance→taxi→hold-point→LUAW→
departed→handoff→out-of-sight; push waits doors shut, VFR turn-
out approved, pattern "report midfield downwind", live unadopt
at 12NM/9000ft — realistic.
traffic/core(248) DONE: class wiring + state (used-callsigns never
reissued, runway_free per strip, radio_silence for mayday,
materialise-then-admit, boarding schedule) — realistic.
traffic/x_instructions(247) DONE: commands apply at readback end
not transmit, already-told dedup, missed alt = plate else field
+2000 (AIM 5-4-21), HA/HF/HM→hold, expedite 90s, turn-out on
course — realistic.
traffic/x_putting_words(240) DONE: uncontrolled→announcements only
(no controller half/readback), AFIS: field info/discretion + plain
roger, all-or-none exchange, command applies at readback end,
greeting once per position, short callsign only after station uses
it (Doc 4444) — realistic.
traffic/x_the_ones_standing(226) DONE: parked fill leaves spare
stands, restock 90s >150m from pilot, boarding schedule w/ VMC
cancel, approach cap 4 scaled ~1.5×/extra runway — realistic.
traffic/x_making(198) DONE: IMC×0.6, VMC gate 1500ft/3SM (SVFR
thresholds), diurnal 0.15-1.0 by field-local hour w/ morning/
evening banks, IMC gap×1.4, category blending vs observed —
realistic.
traffic/x_aeroplanes_somebody_else(190) DONE: observed→strip but
never instructed, callsign dedup (invented retires when real
appears), parked bookkeeping w/ user recognition, OBSERVED_LIMIT
20 + live≤300, own-injected filtered out — realistic.
traffic/x_pilots_talking(186) DONE: crews answer from own
flightplan (apron/leg/destination, weather if asked), CTAF/INFO
only pilot-to-pilot "say position & intentions", once/4min —
realistic.
traffic/x_what_the_pilot(141) DONE: hearing scoped to tuned
channel (same_channel on MHz, else position), quiet gate keeps
FSLTL full-field non-silent, runway_free per strip — realistic.
traffic/x_an_aeroplane_asked(110) DONE: spawn only if free
(LINEUP_BLOCK check) else holding point, IMC→IFR not circuit —
realistic.
traffic/x_small_facts(77) + x_the_aeroplane_the(9) DONE: helpers,
taxi letters unspelled, lang-correct station names — realistic.
traffic/ all 21 files DONE. atc/procedure_select(188) DONE: SID by
runway trans + exit fix toward dest, STAR by entry fix toward
origin, deterministic, named procs validated not swapped —
realistic. atc/procedure_legs(563) DONE: ARINC-424 terminators
(AF/RF/HA/PI...), transition pick by runway+route order, RF arcs
approximated, SID floors / STAR step-downs w/ hold-if-under,
speed limits as caps — realistic. atc/routing(268) DONE: step
climb 2min ahead of plan fix, shortcut ≥8NM/≥2fixes/≤250NM/15min
never past TOD, off-route >3NM/60s→"verify direct" once,
deviation→rejoin 40NM ahead — realistic. atc/route(900) DONE:
TOC/TOD from plan, arrival floor last dsc fix rounded up, TOD
3NM/1000ft min30, expect-climb clamped 5-30min, progress from
hint (loops ok), airway=direct w/ note, AIRAC anchored to real
cycle dates, pln+fms round-trip — realistic.
integrations/adsb(657) DONE: 6s poll, source stuck by ground
coverage, >30s stale drop, source-clock, 429 rest 90s/401-403
drop, route/type cache TTLs, thread off engine — realistic.
integrations/aitraffic(1916) DONE: type-matched titles, write
35/15/5Hz by distance, static-pitch settle to wheels, gear by
phase/AGL, ground writes no altitude (sim surface authority),
model swap w/o flicker, refusal detect — realistic.
PASS 3 COMPLETE (traffic 21 + procedure/route/routing + adsb +
aitraffic + live).
then PASS 4: crew.py(3760) DONE — realistic: cues once/flight, lines
checked vs state at due-time, announcements serialized (floor),
callouts stale-drop & exempt from floor, AT_THE_STAND_ONLY dropped
on liftoff, sterile cockpit, no cabin on light/heli, language by
operator, bilingual repeat English-last, applause fpm+flight gated,
night dim by local clock, flight number "twelve thirty four" style,
transition-aware level naming, handoff clock starts after readback,
VFR squawk by field country, confirm-line dropped if real readback
already went, cabin yields to frequency w/ patience override.
crew_lines.py(2817) DONE — realistic: conditions checked at due
time, GER circuit legs correct (Abflug/Querabflug/Gegenanflug/
Queranflug/Endanflug), cockpit digits not "niner", FR "vario
positif/vent arrière", door-arm wording real, switch patience 300s.
checklist.py(904) DONE — realistic: challenge-response, verify vs
sim w/ fresh re-read before objecting, offers by phase-fit +
500ft AGL floor + never short final + not over radio, wait starts
at heard, hold after one repeat, open-item count reported,
inHg↔hPa tolerance, mismatch rows for study-level types.
checklist_files.py(413) DONE — layering shipped→add-on→pilot,
no_check strips unpublished vars only, re-read on mtime.
checklists/ class JSONs DONE (airliner+piston_single read; A320/737
FCOM + C172S POH items, sourced in comments; magnetos check, idle
cut-off, autobrake, TCAS TA/RA — all real flows). ~100 type files
are add-on var-maps, spot-checked pattern only.
interphone.py(1030) DONE — realistic: routing only on addressed
calls (front ≤6 words), "ground"/"ramp" excluded as ordinary radio
words, longest-match, all-language tables tried, bare answer ≤6
words and only while FO waits, speeds/minimums/reserve parsing.
- LOW · NL has no cabin/ground address words or ask patterns — a
  Dutch-speaking crew can only reach purser/ramp in English
  (name addressing still works). en/fr/de complete.
pilot.py(805) DONE — realistic: callsign shortened only after
controller does (Doc 4444 12.3.1.9 latch), callsign-last
convention, greeting on first call per position only, readback=
instruction elements in given order, intersection named (AIM
4-3-10), type+ATIS letter on first call, "descend via" FAA-only.
baggage.py(409) DONE — GSX clip frames measured; bags counted
seats×load×airline; departure loads only what fits before doors.
passengers.py(380) DONE — left doors to stairs, 12 show for 150,
bus turns away from aircraft, sill height, narrowbody-only.
radio_fx.py(1118) DONE — exceptionally realistic: carrier-capture
quieting (not noise burst), AGC breathing + overshoot, breath
before first word ~35%, held key w/ room tone, rare early-unkey
clip measured vs recogniser, AM detector asymmetry (even
harmonics), sidetone/intercom/cabin/ATIS/HF-SSB profiles, per-
station tilt+keying hashes, LOS=1.23√h for signal quality.
imperfections.py(984) DONE — realistic: stalls measured on
recorded clearance, breaths mixed into existing pauses, catch-
breath after ~5.5s speech, gusting wind, pitch/level wander <1Hz,
indoors() strips radio-only effects for intercom/cabin.
voices.py(1903) DONE — rigorous: accent per voice
measured via shipped recogniser word-loss (kept <25%), excluded
voices listed w/ measured loss, per-station hash for consistency,
4 engine catalogues layered by what each can do (Kokoro no DE/NL
→ Supertonic; accent needs phonemes → Piper; clone only exact-
accent match), m:f pool ~60:40 for realism, graded speakers only.
controllers.py(1038) DONE — hardware plumbing (SDL PTT), not
aviation realism; restart/settle bugs documented & fixed.
applause.py(113) DONE — crowd of clappers joining/tapering,
heard through FD door w/ top rolled off, no cheering.
chime.py(85) DONE — struck-bar partials, up=crew attention,
down=seatbelt sign change (matches real A320 convention).
aboard.py(231) DONE — intercom profile (headset, not radio:
no squelch/carrier), cabin PA through FD door, per-speaker
hesitation manners (captain ad-libs, purser scripted, FO
callouts never hesitant, 2nd language ×1.3).
boarding.py(161) DONE — ambient pad bed, not a pop song;
seamless loop, ii-IV-V-I, under purser level.
announcements.py(454) DONE — Fenix pack layout + tag matching,
airline ICAO from callsign (extends to non-liveried a/c),
time-of-day/variant/seed-stable pick.
prosody.py(306) DONE — only promotes existing commas (never
invents boundaries), address/closing get falling sentences,
pace follows length, jitter avoids metronome.
intonation.py(439) DONE — phrase-level pitch/rate measured vs
recorded clearance; TD-PSOLA keeps formants; YIN tuned on real
voices.
accent.py(1496) DONE — phoneme substitution per target
inventory (never asks model for unseen phonemes), strength-
graded optional rules (schwa unreduction, final-vowel full
value, sC- prosthesis, affricate lenition), ~200-word ATC
lexicon keeping English word-shape w/ French sounds, French
final-syllable hold measured (0.87 vs 0.69 word recovery),
PROSTHESIS "espeedbird", respell table for espeak misreads.
NOTE accent.py:868 — _NEAREST chain for ~30 non-table accents
marked UNVERIFIED in-code (uncalibrated, cap 0.88).
PASS 4 COMPLETE — audio plumbing skimmed (tts/stt/translit/
tone/io/backends), all realism-bearing files read.
PASS 5 STARTED — weather/metar.py(622) DONE: sim ambient only
near field (<5nm/<2500ft), surface wind only <500ft AGL, METAR
month-edge obs time, VRB≠360, METAR regex full grammar incl
VC/RE groups, flight_category = FAA VFR/MVFR/IFR/LIFR bounds,
sim/METAR merge keeps obs clouds+temp aloft.
magvar.py(212) DONE — real NOAA WMM2025 spherical-harmonic
evaluator deg-12, geodetic→geocentric, METAR true→magnetic for
spoken wind (Annex 11 4.3.6, Doc 4444 6.6, 7110.65 2-6-4/2-9-3).
channels.py(192) DONE — 8.33kHz naming exact (.x05→.x00 carrier
pairs, 121.5 kept, E/L-prefix region w/ LL/LT/LV excluded, snap
recovers truncated community data, Annex 10 5.2.1.4.3.1 digits).
db.py(2479) DONE — realistic: STATION_RANGE_NM per
position, position suffixes EN+5 langs + ICAO/FAA dialect, INFO
Radio-vs-Information naming, per-region transition altitude
(US 18000, UK 6000, FR 5000, EHAM 3000, AU 10000, NZ 13000),
transition_level w/ QNH (+27ft/hPa, US 91.121 table), VFR squawk
1200/1400/7000 by state, VFR region (faa/uk/fr/de/metric/icao),
facility name unabbreviation (FIUME→FIUMICINO), centre-row rules.
vfrdata.py(286) DONE — circuits/VRPs/airspace from sourced CSVs,
volume AGL/SFC/MSL refs, floor-incl ceiling-excl per ICAO.
approach_areas.py(137) DONE — SimAware TRACON geometry, field's
area = smallest containing outline (never name-matched),
ceiling 17000, geometry only never facility names.
terrain.py(124) DONE — MSA per Doc 8168: 25NM circle, +1000ft,
+2000 when terrain >3000ft above field (mountainous), GMTED2010
max grid, grid-MORA conservative, rounded up to next 100ft.
localdata.py(304) DONE — override freq validated vs channel
grid, sim import keeps published spoken names, carrier-mapped,
pilot entries survive rebuild.
runway_cache.py(109) DONE — sim runways win when live, 60-day
cache, format-versioned.
simspace.py(503) DONE — OpenAir parse (X-Plane AIRAC files,
arcs every 5°, class map CTR→D RMZ/TMZ→C), MSFS BoundaryType→
class letters, originId-centred arcs, panel resync protocol.
fetch.py(147), release.py(79), audit.py(426), build.py(888)
SKIMMED — data pipeline/licensing/tooling; position map
CLD/PFLT→DEL RMP→GND ARR/DIR/RDR→APP, VHF-only filter.
PASS 5 COMPLETE.
PASS 6 STARTED — sim/source.py(2291)+state.py(381) DONE:
SimConnect units declared not guessed, BCD16/32 freq decode,
squawk BCD, fast/slow tiers, per-radio freq event learning,
per-engine combustion, heli vars, fuel-system legacy/new,
Manual/Replay sources mirror real API.
xplane.py(835) DONE — datarefs/units checked vs X-Plane
docs, fuel kg→gal @6lb/gal, NAV band 108-117.95, transponder
enum mapping, teleport detection by groundspeed, prop-governor
tick, FPLN v1100 upload 12.06+.
xpcontrols.py(690) DONE — AP mode toggles w/ status readback
(XP modes are toggles not on/off), software prop governor
replacing latched override (gain/deadband/slew), stick-vs-lever
override release split, axis→yoke mapping incl. rotorcraft.
serial.py(65) plumbing. sim/ COMPLETE.
aitraffic.py(1916) DONE — realistic: per-type AI titles w/
2024-first fallback measured live, Move-ground omits altitude
(Madeira runway 25ft below field elev), static-pitch write +
contact-point settle measured at LOWI, rate 35/15/5Hz by
distance, SimConnect pitch/bank signs, gear asserted by phase+
AGL (arriving <2000, departing <300), LOD swap w/o flicker,
livery via EX1, refusal detection vs unanswered silence.
groundvehicles(1438) DONE — realistic: measured
ASO_/FSDT_ titles, vehicle pose vars measured live, freeze vs
surface creep measured at Madeira, stride-paced walks 1.5m/1.1m
child, GSX operator placed per its own model, pose read-back
verified (Bilbao), bus probes, per-person titles by crc32.
lvars.py(1014) DONE — L:/MobiFlight/File bridge plumbing,
float-vs-double & per-client channel documented live-failure
fixes. gsx.py(707) DONE — menu walked by reading GSX's own
menu file (never positional guess), want/avoid word match,
optional submenus per-stand, push falls back to sim tug,
can_taxi by jetway/stairs/hose/service-running, backwards-
moving-engines-off push detection. cifp.py(706) DONE —
faithful ARINC-424/CIFP decode (alt-desc codes incl G/H/I/J,
route-type split, RW16B expansion, PRDAT minima, terminal-
scoped fix resolution). doors.py(679) DONE — per-model door
table (measured; passive models can't use flyable entries),
INTERACTIVE POINT read-before-toggle, swing/settle windows,
stale-reading guard, can_push all-shut, force after 300s.
xpground.py(663) DONE — apt.dat faithful: scenery-packs.ini
priority incl GLOBAL_AIRPORTS, 1201/1202/1204/1206/1300 codes,
flow rows 1000-1110 (wind/ceiling/vis/times, arrivals|deps,
course+departure ranges, VFR pattern 1101), comm rows 1050-56
10kHz-vs-1kHz freq units, stand nearest-taxi-node link 250m.
adsb.py(657) DONE — realistic feed design: one source at a
time by ground count (Fiumicino 0-vs-7 measured), stale>30s
dropped, source-clock timing w/ 120s slack, 401/403 drop,
429 rest 90s, backoff 30s→600s, callsign-airline detection,
route/type cache TTLs, no key ever asked.
motion.py(630) DONE — docstring: measured Madeira/Bilbao
numbers, camera-state-aware write rates, stride keyframe
walks, lift-above-read-ground stairs. simaircraft.py(479)
DONE — observed AI treated as weather not subordinate
(occupies runway, joins sequence, described not instructed).
xptraffic.py(550) DONE — packet defaults realistic: gear by
phase/AGL, flaps 1.0 final/.3 runway, lights per real usage
(taxi moving gnd, land <10k or runway, strb runway/airborne,
nav parked-off, bcn always-on), squawk octal, live/invented
budgets. ALL integrations/ DONE (docstring+core verified): network 391
(VATSIM/IVAO: never talk over a human; frequency+range test
not callsign parsing; their ATIS is the real one), simbrief
373 (OFP fetch, callsign/type/route/alternate, pilot-ID vs
username), runways 346 (sim runways win over OurAirports;
threshold children measured 2024), facilities 326 (sim as
authority over thin public data), systems 316 (measured live:
ENGINE_AUTO_* on objects, lights SET readback), terminal_
procedures 299 (facility API SID/STAR legs, m→ft), aimodels
285 (enumerate all installed models incl heli by name),
taxiways 268 (block sizes measured Madeira 2024), fsltl 225
(FSLTL_Rules.vmr parsed, never guessed), servicepoints 192
(door heights from interactive points + PLANE ALT ABOVE
GROUND), live_dry 186, ailiveries 147 (measured: only ~2
dozen real airline liveries exist; EX1 livery arg), approaches
106, fixes 88, deepgram 594 plumbing.
engine/ DONE (x_* docstrings+shared verified — realistic:
freq-write-to-radio, stale-field detection by on-ground
elsewhere, spoken-numerals-vs-words same converter, weather
from sim only when at/near field, language switching by
grammar-match not recogniser-tag, sim FlightPlan read,
live-traffic source gating, FMS corroboration); core.py is
orchestration plumbing (shared docstring verified). web/
server.py docstring verified (local UI, PTT keys machine mic).
UNVERIFIED markers grep'd: all conservative by design
(vspeeds labels unverified sources, Swiss/Austrian VFR
wording, Mongolia RVSM metric — flagged in code already).
PASS 6 COMPLETE. ALL CODE AUDITED.
NEXT: web research spot-verify of flagged claims (vspeeds
tables, numeric alt systems), then finalize report.
## Web verification (checked against live sources)

- CONFIRMED · `navdata/db.py` Mongolia (ZM) "numeric altitude system is
  metric, RVSM included" — Mongolia eAIP ENR 1.8: metric RVSM implemented
  since 17 Nov 2011, 8900-12500 m; ATC clears in metres, aircraft fly the
  feet equivalent (FLAS), 300/500/600 m separation layers. Code correct.
- CONFIRMED · `atc/vspeeds.py` 737-800 "flaps 5 / landing flaps 30"
  premise — Boeing FCTM: flaps 5 is the common takeoff setting, 30/40
  the landing set; speeds in the ~140-150 kt band at ~60-68 t are right.
  Values remain labelled UNVERIFIED in code as intended.
- CONFIRMED · our-airports-vs-simulator ordering throughout —
  facilities/runways/taxiways/SID-STAR-readers all take the running sim
  as authority and degrade to the published dataset; matches real practice
  (the sim is what the pilot sees).
- NOTED · Swiss/Austrian VFR German wording (`navdata/db.py:427`) stays
  UNVERIFIED — no authoritative AIP text found to confirm or deny; the
  code already flags it. Not a finding.

## Findings format

GOAL: findings in format "- SEV · file:line · what · real-world expectation".

## Pass 1 — phraseology & controller

phraseology.py (read in full, 2653 lines) — well-sourced vs 7110.65/Doc 4444;
candidate findings:

- LOW · phraseology.py:2464 `ifr_cancelled` — hardcodes "squawk VFR" for every
  dialect; ICAO/UK controllers issue the numeric conspicuity code
  ("squawk seven thousand"), same as `squawk_vfr`/`radar_service_terminated`
  already do via `vfr_squawk`.
- LOW · phraseology.py:2249 `no_reply` — "how do you hear" is ICAO/CAP 413;
  FAA controllers say "how do you read this frequency" / "radio check".
  Should branch on dialect like the rest of the file.
- LOW · phraseology.py:2228 `runway_change` ICAO — "runway in use now two
  seven" awkward order; ICAO form is "runway in use (number)" or
  "new runway in use (number)".
- LOW · phraseology.py:2456 `pushback_approved` — "push and start approved"
  is the European wording; FAA ground says "pushback approved" (7110.65
  uses PUSHBACK APPROVED). Should branch on dialect.
- SUSPECTED · vector_to_final (1581-1592) is FAA-shaped only
  ("X miles from FIX, turn heading, maintain until established on the
  localizer, cleared ILS approach"); if reachable in an ICAO region a
  European approach controller would say "turn ... heading ... degrees,
  cleared ILS approach runway ... report established". Check callers.
- SUSPECTED · cleared_approach ICAO branch (1632-1633) puts the clearance
  before "maintain N until established"; real ICAO/CAP 413 order is the
  altitude condition first, clearance last (as in the FAA branch and
  Doc 9432's example). Word order only.
- INFO · hold_clearance ICAO (1300-1305) omits "LEVEL" from Doc 4444's
  template but is otherwise faithful.
- INFO · holding_instruction (1694+) is FAA-ordered; fine, but the EFC is
  not prefixed "expect further clearance (time)" in all regions — ok.

controller/ (core, x_what_the_controller, x_entry_point, x_tower, x_ground,
x_clearance_delivery read so far) — dense with sourced comments; candidates:

- LOW · controller/x_ground.py CROSS_CLEAR_S=90 + CROSSING_TAKES_S=45 → a
  taxi crossing is refused when an arrival is within ~135 s of touchdown
  (~5-6 NM at approach speed). FAA requirement is "no reasonable doubt of
  separation"; ~135 s is conservative-plausible for a single-strip field
  but would look over-cautious at busy fields. OK as sim default.
- INFO · controller/x_entry_point.py — after a handoff the first call is
  reclassified as CHECK_IN only for specific intents; radar positions add
  climb/descent/VFR-departure readings. Modeled on real practice.
- INFO · x_what_the_controller.MODE_C_DISAGREES_FT=300 — matches FAA's
  altitude-verification tolerance (±300 ft). Correct.
- LOW · x_clearance_delivery.py: `expect_minutes` default 10 and initial
  altitude default 5000 when `realism.ifr_departure` off — standard-ish.
- NOTE · `_issue_vfr_release` US: gives discrete squawk when DEP/APP staffed
  else 1200; ceiling 2500 AGL (VFR_CEILING_AGL_FT). Real US Class D releases
  commonly "at or below 2500" — plausible.

x_the_circuit.py (read in full, 735 lines):
- LOW · LANDING_CLEARANCE_NM=12.0 — a landing clearance issued at 12 NM out
  is far earlier than real practice (towers clear inside ~5 NM / on final);
  the code comments admit the trade-off ("has to be cancelled the moment
  anything changes" is the real reason towers wait). Not wrong, generous.
- INFO · CIRCUIT_FULL=4, CIRCUIT_SPACING_NM=1.5, ORBIT_SPACING_NM=0.8 —
  plausible circuit values (already flagged UNVERIFIED in comments).
- LOW · orbit for spacing on final goes to the side *opposite* the circuit
  direction (line ~338); real towers orbit toward the non-circuit side to
  keep spacing visible — matches. OK, note only.
- NOTE · overhead join at field+2000 ft on QNH — correct UK SOJ.

x_what_the_arrival.py / x_helpers.py / x_radar_positions_departure.py
(read in full):
- LOW · x_helpers.py `_resolve_fix` — for REQUEST_DIRECT off-plan the fix
  is fuzzy-matched to any navaid within 250 NM; a real centre keeps a
  direct within its own airspace footprint (they'd coordinate beyond it).
  Harmless inside a session.
- LOW · x_helpers.py `_next_level_up` — climbs are issued in 2,000-ft
  steps below 18,000 ft; real centres step 1,000 at a time but 2,000 is
  common in approach control. Plausible.
- INFO · `_valid_level` applies the hemispheric rule on current ground
  track rather than filed route — real practice uses route; edge cases
  could clear a level valid for heading but not destination direction.
- NOTE · `_procedure_default` field+2500 for intercept/missed altitude;
  published-MSA handling with 25-NM override — matches how MSAs work.
- NOTE · `_divert` re-clear to an alternate issued from any radar
  position including APP/TWR — fine.
- NOTE · flight-following refusal at ≥3 arrivals — workload model, fine.

