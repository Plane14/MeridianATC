# Roadmap

## Done in this merge

- WilcoATC engine + StormATC feature set in one Python package
- Deepgram STT/TTS (opt-in, endpoint-pinned)
- OpenAI-compatible LLM providers (OpenRouter, OpenAI, Groq, DeepSeek, custom)
- Opt-in phrasing renderer with fact validation
- Voice trainer (keyterms + corrections, no audio stored)
- Continuous flights / Mission Builder
- Guidance advisor (non-authoritative next-call hints)
- SQLite session/transcript history + `/api/history`
- DPAPI secrets, endpoint allow-list, log redaction, offline mode, cloud consent
- Surface-training indicators on the airport taxi graph
- 66 converted checklists + StormATC prompt files
- .NET 10 WPF shell (10 screens) + xunit tests
- pytest suite, ruff, CI, Inno installer script

## Next

- Streaming Deepgram STT (the WS path the StormATC client used) in addition
  to the REST path this build ships
- Checklist simvar bindings for the converted sets (the source files ship
  them empty; the `check` fields wait on measured add-on variables)
- Package the WPF shell + engine into the Inno installer CI artifact
- Rebuild the MSFS toolbar `.spb` so the panel's internal identifiers can
  carry the MeridianATC name too
