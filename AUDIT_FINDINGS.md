# MeridianATC runtime-behavior audit

Line-by-line audit by 10 parallel readers + manual verification. Each finding:
severity · status (CONFIRMED / SUSPECTED) · file:line · effect · real-world expectation.

## Fixed (turnaround + verification pass, 2026-11-19)

- **#54** Go-around releases armed AP/VS/SPD modes — `_start_go_around`
  now drops `autopilot`/`airspeed_hold`/`vs_hold`/`localizer` when a mode
  was armed, the same release the landing rollout and the stall monitor
  make; the first 8 s stay hand-flown as intended.
- **#55** NAV-fallback order corrected in climb and descent — NAV is
  armed *before* `_apply_flight_director`, so a fallback heading
  (plan steering nothing) wins the engagement; NAV armed last flew the
  dead course and the fallback was dead code.
- **#56** Level cruise now consults `_nav_fallback_heading` too — a dead
  NAV course at altitude engages the fallback heading instead of
  drifting on the last course.
- **#58** `_nav_fallback_heading` converts its true bearing through
  `_magnetic` before returning — the bug it is handed to reads magnetic.
- **#59** The on-plan direct grant stores `session.route_direct` +
  `route_direct_index`, so the FO's `_fly_direct` law flies the
  clearance instead of the filed legs. (Off-plan `REQUEST_DIRECT`
  grants remain unflown — no fix on the route to aim the law at.)
- **#60** Takeoff roll falls back to `state.heading_true` when
  `runway_heading <= 0` instead of steering for 000.
- **#61** Electrical-collapse guard is `volts >= 0.0` — a fully dead bus
  (0.0 V) now declares; the unpublished sentinel is -1.
- **#62** `_reset_scratch` clears the fuel/electrical/engine/NAV-off
  debounce clocks and the spent departure-FMS flags
  (`_fms_direct_done`, `_vector_fms_hdg`) — leg 2+ gets its own watches
  and its own FMS writes.
- **#63** `if assigned:` → `is not None` in `autonomous_heli` pedal
  heading, and the go-around heading pick treats `assigned_heading = 0`
  (north) as assigned, not absent.
- **#64** Checklist gear-retraction requires `not on_ground` **and**
  `vs > 300 fpm` — a rollout bounce no longer retracts the gear.
- **#65** `set_observed` calls `_release_held` on done flights before
  filtering them — a held live aeroplane leaving the feed no longer
  leaks its roster hold.
- **#57** Already fixed before this pass — `_hold_hover` writes
  `PEDAL_SET, 0` with a `TAIL_ROTOR_SET` fallback.
- **#74** Verified already fixed — the stale-ATIS invalidation iterates
  `_atis.items()` and matches `atis_key[0]` against the airport; no
  3-tuple unpack remains.

Still open (LOW/MEDIUM, verified or carried): #66–#73, #75–#76 — all
recorded in place below.

Also fixed this pass (new findings, not in the numbered list):

- Turnaround never verified the fuel write: `_service_turnaround`
  returned "done" on the write's say-so. Now the uplift is
  cross-checked against the tanks (15 s grace) and a write the model
  ignored falls through to the depart-short/shutdown rules.
- Engines idled through the stand delay burning the next leg's fuel —
  `_phase_chain_next` "delay" now sends `_shutdown_engines`.
- `_load_next_flight` dropped `aircraft_floats` on the leg change — a
  floatplane's next leg tried to land on the runway.
- GSX-done path trusted the readback fuel figure before the write: now
  verified the same way.


## HIGH

1. **MSFS design speeds inflated ~3.28x** — CONFIRMED
   `sim/source.py:1325-1328` `fps_kt` multiplies by 1.94384449 (the m/s->kt factor)
   on values the request table asks for in **Feet per second**
   (`DESIGN_SPEED_VS0/VS1/VC`, `ESTIMATED_CRUISE_SPEED`, `MIN_DRAG_VELOCITY`;
   SimConnect RequestList confirms units). Correct factor: 0.592484.
   Effect: C172 Vs1 48 kt -> reported 157 kt. FO approach floor ~197 kt,
   cruise target ~2x Vmo, stall floor inflated, `estimated_cruise_kt` lands
   inside the 50-600 window so it is used directly. Consumers:
   `atc/autonomous/x_plumbing.py:56-63`, `x_shared_helpers.py:249`,
   `x_the_envelope.py:20-30`, `vspeeds.py`. X-Plane path is correct (native
   knots) -> MSFS-only. No test exercises `fps_kt`.
   Fix: `value * 0.592484` (or request the vars in Knots).

2. **Requested approach cleared even when unflyable** — CONFIRMED
   `atc/approaches.py:388-394`: the `requested` loop returns on the first
   suffix-sorted match; `usable()` -> None still returns `Choice(line="")`.
   Second procedure of the same kind never tried. Controller clears the
   aircraft for an approach it cannot legally fly.
   Fix: `continue` on `usable() is None`; don't clear with empty line.

3. **circle_side clearance dropped before flying** — CONFIRMED
   `atc/approach_flight.py`: `ApproachRecipe` has no `circle_side` field;
   `resolve()` copies only `circle_to`->circling (line 318);
   `circle_legs()` picks side from current position (lines 378-379).
   Controller says "circle north", FO circles whichever side it's on.

4. **Crossing check protects the wrong runway on taxi-in** — CONFIRMED
   `atc/controller/x_ground.py:496-498`:
   `active = departure_runway or arrival_runway` — `departure_runway` is set
   at clearance and never cleared after landing, so for taxi-in `active`
   is a stale runway from the departure airport. A crossing over the field's
   actual arrival runway skips `runway_occupied` and the
   `CROSS_CLEAR_S + CROSSING_TAKES_S` arrival-window check -> "cross runway
   35R" granted with an arrival inside the margin.

5. **Localized "standby" collides with readback phraseology (es/it/pt/fr)** —
   CONFIRMED. `intents_local.py` es:475/576, it:723/817, pt:840/934;
   `intents_fr.py:550`. The standby regex (`espere|aguarde|attenda|attendez`)
   matches inside the controller's own LUAW/hold/expect words that the pilot
   echoes back -> `Intent.STANDBY` -> readback-exempt -> neither settled nor
   corrected -> unbreakable `chase_readback` loop (7110.65 2-4-3 / Doc 4444
   4.5.7.5 require correction). German avoided by vocabulary choice.
   Fix: match order/word-boundary scoping so a readback echo cannot
   classify as STANDBY.

6. **Web request threads mutate engine/session with no lock** — CONFIRMED
   `web/server.py:1100-1187` (`/api/tune`, `/api/swap`, `/api/ptt`,
   `/api/talk_radio`, `/api/say`, `/api/squawk`, settings, flight, trainer)
   call `engine.tune`, `engine.set_radio`, `engine.begin/end_transmission`,
   `engine.handle_text`, `engine._radio_talk` directly on the uvicorn loop /
   `to_thread`, while `engine/x_situation_loop.py:82-219` mutates
   `self.station`, `_last_com1/2`, `session.*`, `station2` on the engine
   thread. Only the spoken pipeline takes `_busy`; `handle_text` itself is
   unserialized. Effects: wrong station after retune, readback silently
   cleared, two "pilot" events, `begin_session` twice.

7. **Setup app writes API keys to config.yaml in plaintext** — CONFIRMED
   `web/server.py:2634-2659` `write()` has no `path in SECRETS` branch
   (contrast `_apply_settings` 1435-1442 -> DPAPI store). `set_path` is a
   bare setattr; `Config._save` (`config.py:2080-2104`) writes `asdict(self)`
   including `speech.deepgram_key`/`ai.api_key` -> DPAPI design promise
   ("never written to this file", config.py:226) broken.

8. **METAR `M1/4SM` parsed as 4 SM; `"9999" in body` hijacks visibility** —
   CONFIRMED (live-tested). `weather/metar.py:75,190`:
   `_RE_VIS_SM` matches the `4SM` tail of `M1/4SM` -> 4.0 SM for a 1/4-mile
   fog report; `"9999" in body` returns 10.0 for any body containing an RVR
   group like `R36/9999N`. Feeds ATIS + `svfr.conditions` + flight_category
   -> VFR release into sub-minima weather.

9. **ADS-B `alt_geom` never read** — CONFIRMED
   `integrations/adsb.py:171-181`: `alt = a.get("alt_baro")` only; targets
   reporting only `alt_geom` (some GA/gliders/TIS-B) get `alt_ft=None`,
   `on_ground=False` -> `atc/live.py:467` draws them at field elevation
   while flagged airborne; `_on_final` skips them.

## MEDIUM

10. **ATIS: departure-runway-only change never reissues; ceiling heights
    ignored** — CONFIRMED. `atc/atis.py:119-148`: `moved` checks only
    `landing_runways[:1]`; `_materially_changed` ends comparing
    `_runway_choice(old,True)` vs `(new,True)` — both ask the stateful
    `runways_for`/`runway_for` callback "what's in use now" -> always equal.
    Cloud check compares only cover codes (`[c[0] for c in clouds]`):
    BKN005->BKN045 invisible. Pilot reads back a stale letter and the check
    passes.

11. **Compound hundreds/tens misparse in English intents** — CONFIRMED
    (live-tested). `atc/intents.py:263-292,477-559`: "twenty three hundred"
    -> 300 ft; "heading two seventy" -> 002. Digit-form equivalents parse
    correctly -> correct-but-colloquial speech gets unfixable "negative".

12. **Bare "going around" eaten by interphone, never reaches ATC** —
    CONFIRMED routing. `atc/interphone.py:635-637,727-733,979-984`: a bare
    "going around" routes to the FO (correct crew call) but
    `Intent.GOING_AROUND` never reaches the controller -> tower keeps
    sequencing a phantom arrival and chasing the landing readback.

13. **No relocation detection on MSFS** — CONFIRMED. X-Plane marks
    `state.relocated` on impossible-position jumps (`sim/xplane.py:554-567`);
    `sim/source.py` never sets it; `x_situation_loop.py:82` guards only on
    `paused or relocated`. A slew/map teleport on MSFS fires every
    change-watcher on a fake delta (takeoff, touchdown, handoff, checklist).

14. **RadioPlayer.busy has no wedge guard** — CONFIRMED. `audio/io.py:557-562`
    vs CabinPlayer's `_stall_at`/`_trouble` (812-820). An `sd.play` hang
    (device dies mid-transmission) leaves `busy` True forever ->
    `radio_busy()` suppresses all unprompted ATC calls, ATIS, crew for the
    rest of the flight, unreported. Cabin path was hardened; radio (which
    gates more) was not.

15. **Deepgram stream leaks on the too-quiet path** — CONFIRMED.
    `engine/x_transmission.py:178-181` returns without `abort_stream()`
    (every other exit path aborts: lines 104, 191). Idle billable websocket
    left open until next PTT or close().

16. **METAR fetch/parse hardening gaps** — CONFIRMED.
    `metar.py:262-264` `_load_cache` AttributeError/TypeError on malformed
    cache (crashes WeatherProvider.__init__ -> engine construction);
    `metar.py:379-390` `rows[0]`/`row.get` outside the try -> crash on
    non-list JSON + re-fetch spin; `metar.py` observed-time group (DDHHMMZ)
    never used -> hours-old METAR stays "fresh"; `network.py` controller
    picture has no TTL -> silent forever after a VATSIM outage.

17. **candidate_stations LOS uses MSL not AGL** — SUSPECTED.
    `navdata/db.py:1168-1204`: `los = 1.23*sqrt(altitude_ft)` MSL -> at
    high-elevation fields a distant tower on a shared channel can answer.

18. **ADS-B clock domain** — CONFIRMED design flaw.
    `integrations/adsb.py:148-154`: position stamps use server clock when
    within 120 s slack but consumers compare to local `time.time()` ->
    live traffic empties on >~1 min server drift while reporting LIVE.

19. **update.py batch handoff** — CONFIRMED (partial). PID substring match
    (`find "{pid}"` matches 12345) -> spurious failed update; robocopy /MIR
    no rollback -> half-copied install can be left unstartable.

20. **Controller phraseology gaps** — CONFIRMED (batch):
    visual approach silently retunes to tower, no "contact tower"
    (`x_radar_positions_departure.py:618-637`); IFR flight told "radar
    service terminated" on freq-change request (251-269); "proceed direct"
    granted to unresolvable fix names (`x_helpers.py:507-530`); takeoff
    clearance with no at-runway check (`x_tower.py:39-42`); "final" report
    after landing clearance re-issues a join and resets phase
    (`x_the_circuit.py:351,403`).

21. **Blocking calls on the event loop** — CONFIRMED.
    `web/server.py:1137,1104-1106`: `set_radio` (SimConnect write),
    `end_transmission` (`microphone.stop()` joins capture) run inside async
    handlers -> a hung write stalls the WS pump and all endpoints.

22. **CPDLC ROGER settles a clearance uplink without applying it** —
    SUSPECTED. `atc/cpdlc.py:104-133`: `roger` marks read_back + clears
    pending but never `session.apply` -> clearance vanishes; ROGER isn't a
    valid action response anyway (WILCO/UNABLE only).

## LOW

23. `guidance.py:43` `getattr(session,"flight_rules","ifr")` — attr doesn't
    exist (session has `ifr: bool`) -> VFR pilots always get IFR hint.
    CONFIRMED.
24. `xplane.py:638` `set_com` readback `<2.0` kHz fails exactly at the
    8.33 snap boundary (e.g. tune 121.502 -> snaps 121.500, diff 2.0 ->
    reported failure though tuned). CONFIRMED edge.
25. `normalize_angle` (state.py:333-348): values within +/-2pi with
    `source_is_radians=None` treated as radians -> true heading 006 deg
    becomes 344 deg. Callers mostly pass explicit kinds; heuristic risk.
26. ADS-B `track` falls back to `true_heading` -> wind-drift wrong curved
    paths (`adsb.py:183`); TIS-B `~`-stripped hex can collide with real
    ICAO keys; negative `seen_pos` unclamped.
27. `adsb.py` FlightBook unvalidated entry shapes + `stop()` races in-flight
    `look_up` -> possible torn cache write / poisoned lookups.
28. `deepgram.py:577` odd-length body -> ValueError escapes synthesize;
    `_note` crashes stream pump on malformed message shapes (REST fallback
    covers).
29. `network.py:265-268` `poll()` sets `_busy=True` before `Thread.start()`
    -> start() raising wedges network polling forever.
30. `gsx.py:428-433` `menu_file()` `p.stat()` inside `max()` races GSX
    rewrites -> OSError propagates and fails the whole tick.
31. `sim/brief.py`-adjacent: `_ifr` misreads ICAO rules "Z" (VFR-first) as
    IFR (`simbrief.py:175-190`); `_altitude` treats <1000 as FL.
32. `aimodels.py`: `_HELICOPTER_TYPES` missing H125/H60 -> rotorcraft live
    traffic gets fixed-wing stand-in; `("DA62","DA42")` maps DA62->DA42.
33. `tts.py` cache `_remember` mutates `_cache_order` unguarded vs
    lock-held clearers -> rare IndexError on render lane.
34. `piper.py:163-183` sentence pause sized at running rate not
    `model.config.sample_rate` -> mistimed pauses if a voice reports no
    chunk rate (e.g. 16 kHz x_low).
35. `live_dry.py:106` banner prints `live.radius_nm` raw -> "0 nm" while
    actually polling the default radius. Cosmetic/misleading.
36. `approaches.py:327`/`svfr.py`: missing `visibility_sm` -> every minima
    fails -> indefinite hold; SVFR visibility maps missing->0 -> refused.
37. `surface.py:111-140` remaining-distance overcount on multi-edge
    taxiways (training display only).
38. `vectors.py:136-144` `cut_in()` can overshoot centreline then snap ->
    fishtail on final.
39. `routing.py:165-190` ETA dict keyed by fix ident collapses duplicate
    idents -> step-climb timing on wrong fix.
40. `procedure_select.py:123` `trans.runway != runway` raw compare —
    `_runway_key`/normalize_runway exist elsewhere; format mismatch can
    silently discard the correct transition.
41. `x_the_circuit.py:679` landing sequence mixes distance-to-field vs
    distance-to-threshold -> "number N" can be off.
42. `x_ground.py` refused crossing's hold-short has no Instruction -> no
    readback chased; IFR flight never hears altimeter on the ground
    (line 363); transit ceiling approves pilot's altitude verbatim above
    the shelf (`x_the_circuit.py:574`); SVFR request on radar position
    falls to expected-calls (`x_entry_point.py:299`); taxi-to-parking on
    Delivery unanswered; VFR release issued by GND not DEL.
43. `occupancy.py:121` `on_final` uses contact heading as true track —
    magnetic/true mix possible at cone edge.
44. `app.js:1060` heading displays "000°" for 359.5-360 — aviation shows 360.
45. `star_join` never evaluates STAR's first fix as join point (rare).
46. Watch hysteresis: repeated <threshold excursions reset timers so a
    chronically weaving flight is never warned; once-warned never re-warns.
47. `history.py:156-158` record() racing close() -> ProgrammingError at
    shutdown (stray error only).
48. `quit_process` `os._exit(0)` after 8s grace bypasses engine.stop ->
    possible truncated trailing log line.
49. `gui --host 0.0.0.0` serves full unauthenticated control API on LAN —
    documented trade, worth a warning.
50. `fixes.py:48` `kind` not normalized (case-sensitive compare).
51. METAR wind variation group `200V290` never parsed -> `variable` only
    for VRB; `P`-prefixed gusts >99kt unparseable.
52. `health.py` `str(exc)[:80]` into UI can carry credential fragment
    (narrow; keys travel in headers).
53. `redact.py` `_CREDENTIAL_FIELD` needs closing quote — unquoted
    `api_key=abc` relies on `_SECRET` name-match alone.

## Intentional/accepted (not defects)

- Broad `except` around tick/crew/autonomous/traffic work: resilient loop,
  can hide persistent failures — noted as design trade.
- ATIS letter frozen while user on final: deliberate.
- Traffic requires simulator: `engine/core.py` `traffic.requires_sim=True`.
- MSFS wrapper: `AIRSPEED_BARBER_POLE` requested Knots; `DESIGN_TAKEOFF_/
  MIN_ROTATION` requested Knots via OUR_VARIABLES — both correct.
- `com1_standy_flip` spelling is X-Plane's own command name.
- 8.33 channel naming (channels.py) correctly implements ICAO scheme.
- VATSIM same-frequency-anywhere silences program: conservative, documented.

## Autonomous / FO (agent 02b934d8)

### HIGH (cont.)

54. **Go-around flies raw elevator against armed VS-descent AP mode** —
    CONFIRMED. `autonomous/x_circling.py:327-389`: `d.go_around()` is only
    `toga(True)` (autothrottle); `d.approach(False)` releases APR only;
    `autopilot(False)`/`vs_hold(False)` never sent. On non-ILS/visual
    recipes where `ap_vs_mode` was armed with a descent rate, the AP keeps
    commanding the descent on the elevator channel while `d.elevator`
    pulls fight it for 8 s -> aircraft can continue descending under TOGA
    power after "going around" was called. `_monitor_stall`
    (x_the_stall.py:94-96) does `autopilot(False)` first for the same
    reason; the comment at 385-388 describes the fight it leaves armed.

55. **NAV fallback dead code in climb/descent** — CONFIRMED.
    `x_the_phase_machine.py:1049-1057,1279-1289`:
    `_apply_flight_director(heading=_nav_fallback_heading(...))` arms HDG
    hold, then `self.director.nav(True)` immediately re-arms NAV —
    last-engaged-wins -> NAV wins every tick, fallback never flown.
    Cruise emergency-descent path (1085-1087) uses the correct order
    (nav first, then director); climb/descent inverted.

56. **NAV fallback absent from level cruise** — CONFIRMED.
    `x_the_phase_machine.py:1133-1144`: `abs(alt-target)<=200` branch runs
    `alt_hold(True)`/optional heading and never calls
    `_apply_flight_director` -> `_nav_fallback_heading` unreachable in
    level cruise, the canonical "NAV armed on dead course" case.
    (2)+(3): the fallback only ever engages in step climbs/descents.

57. **Heli `_hold_hover` never writes PEDAL_SET** — CONFIRMED.
    `autonomous_heli.py:717-730,1536-1541`: collective at hover trim,
    cyclic trims, steering 0 — anti-torque channel uncommanded and no
    heading/drift hold -> torque-yaw and drift a few feet off the ramp
    during the "feet on the ground" stage.

### MEDIUM (cont.)

58. **Fallback heading is true-bearing written to a magnetic bug** —
    CONFIRMED. `x_shared_helpers.py:60-73` returns `bearing_to` (true)
    straight to `d.heading()` (magnetic bug); every other true value goes
    through `self._magnetic(...)` -> fallback steering off by magvar
    (up to ~25 deg).

59. **Granted pilot-requested direct is never flown** — CONFIRMED.
    `x_shared_helpers.py:995-1018` vs `controller/x_helpers.py:497-505`:
    `_direct_on_the_plan` grants "direct X" but stores `values={}`; only
    writer of `session.route_direct` is the controller's proactive
    shortcut (`routing.py:238-239`) -> FO acknowledges then continues the
    filed route; clearance silently unexecuted.

60. **runway_heading<=0 rolls on heading 000** — CONFIRMED.
    `x_the_phase_machine.py:477-483,786-804`: LINE_UP holds when heading
    unknown, but on `cleared_takeoff` the `runway_heading>0` gate is
    skipped and roll steers `_ground_directional(keep_hdg=0.0)` -> drives
    off the pavement side on non-north runways.

61. **Electrical-collapse misses exactly 0.0 V** — SUSPECTED.
    `x_emergencies.py:55-57`: `volts > 0.0 and (volts<12 or 15<volts<20)`
    — a fully dead bus (0.0 V) never declares; a sagging one does.

### LOW (cont.)

62. `x_entry.py:202-241` `_reset_scratch` doesn't clear
    `_electrical_since`/`_engines_since`/`_fuel_*_since` -> a failure
    straddling `begin_flight` can declare instantly on the new leg.
63. `autonomous_heli.py:1519` / `x_circling.py:404` `if assigned:` /
    `if not hdg` treat heading 0.0 (north) as unassigned.
64. `x_the_checklist_answered.py:126-129` `gear_down want_on=False`
    retracts whenever `not on_ground` — no positive-rate/height gate ->
    wheels up during a rollout bounce.

## Traffic (agent f7e8399d)

### HIGH (cont.)

65. **Held live flight leaving the feed leaks its LiveRoster hold forever** —
    CONFIRMED. `traffic/x_aeroplanes_somebody_else.py:178-181` filters done
    flights out of `self.flights` without `_release_held`; every release
    path scans only `self.flights`
    (`x_what_the_pilot.py:46-49,82-85`; `_release_all_held`
    `x_the_live_ones.py:907-911`). `live.py:412-413` keeps a held hex
    forever (only `_forget` purges `_held`/`history`/`_behind`) -> if the
    hex is re-heard it is drawn stationary at the held position for the
    rest of the session (the "statue" the docstring at
    x_the_live_ones.py:899-902 describes); history also accumulates
    unboundedly (pinned to frozen `held[1]`, live.py:376-380).
    Triggers: feed flap, out-of-range, the 300-aircraft `live.limit` cut,
    gone-held carry-out.

### MEDIUM (cont.)

66. **`_plan_departure` last-resort routing drops `closed` runway edges** —
    CONFIRMED (arg missing); SUSPECTED (incursion).
    `traffic/x_where_it_is.py:221-231`: attempts 1-2 pass `closed=closed`;
    the third fallback calls `layout.departure(...)` without it -> a taxi
    route planned across closed pavement exactly on the constrained
    layouts where the guard matters; outcome is a dead-end hold or a
    runway entry that isn't a recognised claim point.

67. **`_live_flew` never cleared -> re-departing live aircraft misread as
    arrival** — CONFIRMED. Written `x_the_live_ones.py:111`, read :184,
    :1027 (`_live_arriving`); the field-change reset `x_the_field.py:34-67`
    clears the other live sets but not this one -> a live aircraft seen
    airborne once is forever "arriving" on the ground at taxi speed:
    wrong sequence side, wrong lifecycle, wrong adoption direction.

### LOW (cont.)

68. `"pushback"` phase missing from `_POSITION_OF_PHASE`
    (`traffic/_shared.py:582-607`) -> a pushback flight reports "TWR"
    instead of "GND" -> wrong strip label/channel.
69. `live.py:550-554` `hash(hex_) % 2**24` for non-hex feed ids —
    process-seeded, changes per run, 24-bit collisions can merge two real
    aircraft into one strip.
70. `x_the_field.py:488` `hash(flight.written) % 360` apron bearing —
    process-seeded; same callsign parks differently each session.
71. `x_pilots_talking.py:15,111-112` module-level `random` humor injection
    — contradicts the module's "all deterministic" comment.
72. Toggling live traffic off leaves `traffic.hold_live` bound to a dead
    roster (`x_the_real_aeroplanes.py:34` nulls `live_roster` but never
    clears the lambda bound at :127-129).
73. `_strips_parallel` treats two empty runway keys as parallel
    (`_shared.py:137-146`) -> cross-strip conflict checks skipped when
    both runway names are unknown (non-conservative, needs navdata gap).

## Engine (agent 39d5b966)

### HIGH (cont.)

74. **ATIS stale-invalidation throws once per airport, then serves stale
    ATIS forever** — CONFIRMED.
    `engine/x_state_and_weather.py:170-174`: `self._atis` keys are
    3-tuples `(airport_ident, language, region)` (built :168, stored :209)
    but the loop unpacks `(field, _spoken)` -> `ValueError: too many
    values to unpack` on the first item. The `_atis_stale` marker is
    discarded at :171 BEFORE the loop, so the exception fires exactly
    once and the invalidation is lost for good -> `AtisGenerator.update`
    keeps returning the cached report: the broadcast never re-issues on
    the sim's real approach/runway data (the feature the comment at
    `x_the_other_aeroplanes.py:447-450` exists for). Marker is added for
    essentially every airport that gets ATIS in a sim-connected session
    (async approach reader `:451-452` + `_ask_for_approaches` at :210).
    Callers: `play_atis` unguarded -> HTTP 500 on `/api/atis` + one
    situation-tick abort via RunwayWatch; guarded callers swallow it
    once. The stale report then plays as if nothing happened.

### MEDIUM (cont.)

75. **`handle_text` runs on three threads with no lock** — CONFIRMED
    (corroborates web agent finding; adds the third caller).
    `engine/x_transmission.py:289-484` mutates `session`/`pending`/
    `atis_acknowledged`/`_held_reply`/`handoff_pending`/language maps.
    Entry points: PTT/STT worker under `_busy`; situation thread via
    `first_officer_transmits` (`core.py:869` from `crew.py:2546`) and
    `_copilot_reads_back` (`x_the_other_aeroplanes.py:2150`); web
    `/api/say` via `to_thread` ungated. `radio_busy()` gates only against
    `_busy` + pending renders -> a typed call inside an FO call or
    copilot readback can double-consume `session.pending`, rebuild the
    session under `brain.respond`, or clobber `_held_reply`.

### LOW (cont.)

76. `_refresh_live_feed` unions `source` into `_live_asked` BEFORE
    `start_live_feed()` (`x_the_real_aeroplanes.py:41-44`) -> a raise in
    construction/`start()` leaves live traffic silently off for the
    session, never retried.
77. ADS-B feed keeps polling when master `traffic.enabled` goes off
    (`x_situation_loop.py:234-247` cleanup branch never stops
    `live_feed`/`live_roster`) -> wasted network/API quota.
78. `x_setup.py:508` `start_live_feed()` bypasses `_live_asked` -> the
    "Live traffic stays off: FSLTL" notice emits twice at startup.
79. `_live_down_said` latch survives feed restarts
    (`x_the_real_aeroplanes.py:28-36`) -> a new feed that immediately
    fails never re-emits the "no source answering" warning.

## Sim/integrations — X-Plane files (agent ca4c66a1)

### HIGH (cont.)

80. **`XPlaneFixes` rejects every real `earth_fix.dat` row — all enroute
    fix resolution dead on X-Plane** — CONFIRMED (same-repo parser
    contradicts it; test fixture encodes the wrong format).
    `integrations/xpground.py:634-643` accepts only rows of exactly 3
    fields (`lat lon IDENT`); real XP11/XP12 rows are `lat lon IDENT
    terminal region numeric-id [name]` (6+). `integrations/cifp.py:
    282-295` parses the same file correctly (`len(parts) < 6: continue`).
    `_fixes` ends up empty on any real install -> `engine/x_session.py:
    353-357` wires it as the sim tier of `atc.route.resolver()`, and the
    navdb "has no intersections" (`route.py:36-40`) -> every named fix in
    a `.pln` lands in notes as "could not be placed" (`route.py:520-522`)
    and legs collapse to airport-to-navaid directs. Fixture
    `tests/test_xpground.py:87-93` writes only 3-field rows, so the
    suite validates the obsolete format.

### MEDIUM (cont.)

81. **apt.dat airport scan rejects below-sea-level airports** — CONFIRMED.
    `integrations/xpground.py:105-112` gates airport header rows on
    `fields[1].isdigit()`; `"-11".isdigit()` is False, so EHAM, LIPZ,
    KMSY, and Dead-Sea fields are never indexed -> no taxi graph,
    stands, frequencies, or flows for them on X-Plane (falls back to
    invented straight-line taxi + offline db). The block parser
    (`:192-196`) itself handles negative elevations fine.

### LOW (cont.)

82. X-Plane `bank_angle_mode` read as direct 0-based index — CONFIRMED.
    `sim/xplane.py:442-448` maps mode m -> `bank_angle{m}`; dataref is
    enum 0..6 where 0=automatic and 1..6 select `bank_angle0..5`
    (writer `xpcontrols.py:593-603` sends `nearest+1`). Off-by-one for
    modes 1-5, mode 6 (highest preset, which our own `_cap_bank` writes
    can produce) reports 0.0 and lands in `state.unavailable`. Benign
    today — no atc/engine code consumes `ap_max_bank_deg`.

All 10 audit scopes complete — 82 findings.
