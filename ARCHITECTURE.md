# Architecture

```
MSFS 2024 ──SimConnect──> meridianatc/sim ──> meridianatc/engine.py
                                │                    │
                navdata db (SQLite)          atc/* deterministic engine
                                │                    │ approved text
                                ▼                    ▼
                web/server.py (FastAPI)      llm/* (opt-in, validated)
                     │  REST + /ws                  │
        ┌────────────┴───────────────┐              │
        ▼                            ▼              ▼
  webview panel (pywebview)   desktop/ WPF shell   audio/* (STT→intent,
                                              TTS←phraseology)
```

## The rule

* **The simulator is the physical truth.** State comes from SimConnect.
* **The deterministic engine decides what is permitted.** Phraseology,
  clearances, frequencies, runways, separation — all fixed rules and
  fixed templates.
* **A language model may only listen or rephrase.** `llm/understanding`
  classifies pilot transmissions the rules could not place;
  `llm/rendering` rephrases engine-approved text with every fact token
  validated against the approved wording. Either can be absent entirely.

## Layout

| Path | What it is |
| --- | --- |
| `meridianatc/engine.py` | the situation loop, radio state, session orchestration |
| `meridianatc/atc/` | phraseology, intents, sessions, ground/departure/arrival, checklists, crew, autonomous flight, guidance, continuous, surface, trainer |
| `meridianatc/audio/` | Whisper STT, Supertonic/Piper/Chatterbox TTS, radio effects, PTT |
| `meridianatc/integrations/` | SimConnect, GSX, FSLTL, SimBrief, Deepgram, network, LVARs |
| `meridianatc/llm/` | providers, understanding, opt-in phrasing renderer |
| `meridianatc/navdata/` | the worldwide frequency/airport/airspace database |
| `meridianatc/security/` | DPAPI secrets, endpoint policy, log redaction |
| `meridianatc/web/` | the FastAPI server + translated panel UI |
| `desktop/` | the .NET 10 WPF shell: a client of the local API, nothing more |
| `prompts/` | StormATC's prompt set, shipped for the renderer/intent parser |
| `tests/` | pytest suite for the merged ports |
| `tools/` | data converters (checklists, etc.) |
