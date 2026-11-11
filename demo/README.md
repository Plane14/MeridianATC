# Demo pack (fictional)

StormATC's demonstration data, carried over: two invented airports
(**STMR** Storm International, **STMD** Storm Meadows), the sector that
holds them, and one sample flight plan (STORM123 STMR→STMD). Everything
here is **fictional** — the `ST..` prefix is not a real ICAO region and
none of it may be used for navigation, in or out of the simulator.

Its purpose is what StormATC used it for: exercising the pipeline
end-to-end with no SimBrief account, no API keys, no simulator, and no
network. The test suite parses every file to keep the pack honest.

| Path | What it is |
| --- | --- |
| `sample-airports/` | Airport definitions: frequencies, runways, stands, taxiways |
| `sample-sectors/` | The STORM FIR around them |
| `sample-flights/` | The sample OFP, squawk pinned for reproducibility |
| `phraseology/` | The ICAO-English dialect file the demo speaks |
