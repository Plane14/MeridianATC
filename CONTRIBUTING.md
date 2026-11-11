# Contributing

## Running from source

```powershell
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt
.venv\Scripts\python -m meridianatc doctor   # what is missing
.venv\Scripts\python -m meridianatc           # run the engine + panel
```

## Tests

```powershell
.venv\Scripts\pip install pytest ruff
.venv\Scripts\python -m pytest tests
.venv\Scripts\python -m ruff check meridianatc tests tools
dotnet test desktop/MeridianATC.Desktop.Tests
```

## The one rule

The deterministic engine is authoritative for aviation facts. If you are
adding anything that touches an LLM, a cloud service, or network access:
it may only interpret or rephrase what the engine already approved, and it
must degrade silently to the deterministic path when it is absent,
unreachable, or wrong.

## Checklists

Aircraft checklists live in `meridianatc/atc/checklists/` (per class) and
`checklists/types/` (per add-on, matched on the simulator title). The
StormATC per-aircraft set was converted by
`tools/convert_storm_checklists.py`; run it again if the source data
changes.
