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
- X-Plane 12 UDP aircraft-state/radio link and automatic/explicit simulator selection
- Native MeridianATC-XP XPMP2 traffic renderer and Python snapshot/status backend
- X-Plane plugin discovery, preserving install/uninstall CLI, Windows setup option,
  optional native payload packaging, and doctor installation/UDP/CSL checks
- X-Plane AP-level FO assistance through standard commands/datarefs (airborne fixed-wing only)

## Next

- Live X-Plane rendering/terrain/labels/TCAS validation with real CSL packages
- Live-aircraft validation of X-Plane AP assistance; taxi/takeoff/flare/go-around
  and rotorcraft axis control remain explicitly manual
- Reliable native X-Plane traffic identity before importing TCAS contacts
- Validate macOS native plugin packaging/signing and installer UI on real systems

- Streaming Deepgram STT (the WS path the StormATC client used) in addition
  to the REST path this build ships
- Checklist simvar bindings for the converted sets (the source files ship
  them empty; the `check` fields wait on measured add-on variables)
- Package the WPF shell + engine into the Inno installer CI artifact
- Rebuild the MSFS toolbar `.spb` so the panel's internal identifiers can
  carry the MeridianATC name too
