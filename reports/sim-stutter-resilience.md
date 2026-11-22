# Simulator stutter resilience

What happens to MeridianATC when the simulator stops answering for a few
seconds -- a long frame, a scenery load, a hung draw, a SimConnect queue
backed up -- and what changed so a stutter is no longer treated as a
crash.

## What a stutter used to cost

Silence from the simulator was binary: fresh or gone.

- **X-Plane:** `XPlaneSource.connected` went false after `disconnect_s`
  (5 s) without an RREF packet. RREF output is produced by the sim's
  flight loop, so a multi-second freeze looks identical to a quit.
- **MSFS:** `EMPTY_READS_BEFORE_GONE = 8` empty reads (~3 s at the poll
  rate, each already slowed by 400 ms timeouts) flipped `_connected`.

`Engine._watch_simulator_locked` read that one bit and, on it going
false, ran the whole crash path: "Lost the simulator",
`_detach_simulator_integrations()` (which sends the X-Plane plugin a
`clear` -- every injected AI aircraft vanishing at once, and closing
GSX, doors, and the facility readers), a `ManualSource` hand-off, then
reconnect and `begin_session()` -- a brand-new `FlightSession` with a
fresh squawk, phase reset to PREFLIGHT/ENROUTE, crew and departure
state rebuilt mid-flight.

Secondary damage, counted even when the link survived:

- `aitraffic` / `groundvehicles` aged pending creation requests against
  `ANSWER_WITHIN_S = 10 s`; requests asked just before a stall timed
  out immediately after it and counted toward `REFUSALS_BEFORE_GIVING_UP
  = 6` -- permanently latching "the simulator refuses to create
  aircraft" and `_failed` callsigns.
- `xptraffic`'s `_wanted` timestamps aged the same way, marking
  aeroplanes `failed()` the plugin never had a chance to draw.
- While stalled, the MSFS slow-tier rotation still paid
  `request_timeout_ms` per variable on every pass, so each tick blocked
  for seconds inside the situation loop.

## What already held

- `Pid` (atc/control.py): `dt` outside `0 < dt <= 5.0` skips integral,
  derivative and slew -- no windup across a freeze.
- `_fly` (x_the_watchdog.py): `elapsed` clamped to `MAX_STEP_S = 5.0`.
- `NativePilot` and the autonomous slews drive from the engine tick
  clock and clamp `dt` to 1 s.
- `state.paused` / `state.relocated` already had the right idea: hold
  the crew, skip every change-watcher, keep reading the state.

## The change: silence has two tiers now

`state.stalled` sits between "fresh" and "gone".

- **X-Plane** (`sim/xplane.py`): `connected` still means fresh
  (`disconnect_s = 5 s`); new `alive` property means answered and silent
  less than `gone_s = 30 s`. `read()` returns `stalled=True` in the
  gap -- and `connected=False`, so per-tick writers that gate on it
  (dataref writes, FMS/procedure readers) do not keep talking to a
  frozen sim.
- **MSFS** (`sim/source.py`): `EMPTY_READS_BEFORE_STALL = 8` marks the
  state stalled; `EMPTY_READS_BEFORE_GONE = 80` (~20-40 s) is when the
  link is actually declared lost. Once stalled, the slow-tier fetch
  rotation is skipped so a stalled read costs one fast pass, not
  seconds of timeouts per variable.
- **Engine** (`x_the_simulator_link.py`): teardown now keys on
  `link.alive` where the source provides it (`XPlaneSource`); sources
  without it keep `connected`, which already flips only at gone.
  The held `ManualSource` copy clears `stalled` so the fallback does
  not hold forever.
- **Situation loop** (`x_situation_loop.py`): `stalled` takes the same
  hold path as `paused`/`relocated` -- crew holds, every change-watcher
  and every `sync()` is skipped -- with its own log lines: "The
  simulator stopped answering -- everything holds where it is until it
  does or is gone." / "The simulator is answering again."
- **Writers** (`aitraffic`, `groundvehicles`, `xptraffic`): a new
  `stall_ended(now)` restarts every pending answer window when the sim
  resumes, so the stall's seconds are not counted against
  `ANSWER_WITHIN_S` and pending creates are not refused.

Net effect: a 5-30 s freeze is a hold, not a crash -- session, squawk,
phase, injected traffic and integrations all survive; a real quit still
tears down and reconnects exactly as before, just on the `gone_s` /
80-empty-read horizon instead of the first missed packets.

## Remaining edges, not fixed

- A stall past `gone_s` still tears down; that is the intended "really
  gone" path, only more conservative.
- The X-Plane plugin's own status stream (`STATUS_WITHIN_S = 3 s`) goes
  quiet while X-Plane is frozen, so `xptraffic.drawing`/`available`
  dip false for the duration; nothing consumes them during the hold
  and they recover when the plugin does.
- `simdoors`/`simsystems` have no pending-request deadlines of the kind
  the three writers carried; they skip with the rest during a stall.
