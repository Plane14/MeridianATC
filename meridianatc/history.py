"""Session and transcript history in SQLite, ported from StormATC.

One database in the data root holding every radio exchange and every flight
the engine ran: where StormATC kept sessions in its own schema, here the
transcript is the unit -- every pilot transmission and every controller
reply, stamped, with the session it belonged to. The web panel reads it
through ``/api/history``; nothing here feeds back into the engine, so a
corrupt history file costs the past, never the flight.
"""

from __future__ import annotations

import json
import logging
import sqlite3
import threading
import time
from dataclasses import dataclass
from pathlib import Path

log = logging.getLogger(__name__)

_SCHEMA = """
PRAGMA journal_mode=WAL;
CREATE TABLE IF NOT EXISTS sessions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    started_at REAL NOT NULL,
    ended_at REAL,
    departure TEXT,
    destination TEXT,
    callsign TEXT,
    aircraft TEXT,
    rules TEXT
);
CREATE TABLE IF NOT EXISTS transcript (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id INTEGER NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    at REAL NOT NULL,
    who TEXT NOT NULL,
    station TEXT,
    text TEXT NOT NULL,
    intent TEXT,
    meta TEXT
);
CREATE INDEX IF NOT EXISTS transcript_session ON transcript(session_id, at);
"""


@dataclass(frozen=True)
class SessionRow:
    id: int
    started_at: float
    ended_at: float | None
    departure: str
    destination: str
    callsign: str
    aircraft: str
    rules: str


@dataclass(frozen=True)
class TranscriptRow:
    id: int
    session_id: int
    at: float
    who: str
    station: str
    text: str
    intent: str
    meta: dict


class History:
    """A WAL-mode transcript database; safe for one writer, many readers."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._conn = sqlite3.connect(str(self.path), check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        with self._lock:
            self._conn.executescript(_SCHEMA)
        self._session_id: int | None = None

    def open_session(self, departure: str = "", destination: str = "",
                     callsign: str = "", aircraft: str = "",
                     rules: str = "") -> int:
        """Start a session row; the id is remembered for transcript writes.

        The previous session, if any was left open, is closed first -- a
        flight that ends by the next one beginning still gets an end time.
        """
        with self._lock:
            if self._session_id is not None:
                self._conn.execute(
                    "UPDATE sessions SET ended_at = ? WHERE id = ? "
                    "AND ended_at IS NULL",
                    (time.time(), self._session_id))
            cur = self._conn.execute(
                "INSERT INTO sessions (started_at, departure, destination, "
                "callsign, aircraft, rules) VALUES (?, ?, ?, ?, ?, ?)",
                (time.time(), departure, destination, callsign, aircraft,
                 rules))
            self._session_id = int(cur.lastrowid)
            self._conn.commit()
            return self._session_id

    def close_session(self) -> None:
        with self._lock:
            if self._session_id is not None:
                self._conn.execute(
                    "UPDATE sessions SET ended_at = ? WHERE id = ?",
                    (time.time(), self._session_id))
                self._conn.commit()

    def record(self, who: str, text: str, station: str = "",
               intent: str = "", meta: dict | None = None,
               session_id: int | None = None) -> None:
        """Append one transcript line to the current (or named) session."""
        sid = session_id if session_id is not None else self._session_id
        if sid is None:
            return
        with self._lock:
            self._conn.execute(
                "INSERT INTO transcript (session_id, at, who, station, text, "
                "intent, meta) VALUES (?, ?, ?, ?, ?, ?, ?)",
                (sid, time.time(), who, station, text, intent,
                 json.dumps(meta or {})))
            self._conn.commit()

    def sessions(self, limit: int = 50, offset: int = 0) -> list[SessionRow]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM sessions ORDER BY started_at DESC "
                "LIMIT ? OFFSET ?", (limit, offset)).fetchall()
        return [SessionRow(r["id"], r["started_at"], r["ended_at"],
                           r["departure"] or "", r["destination"] or "",
                           r["callsign"] or "", r["aircraft"] or "",
                           r["rules"] or "") for r in rows]

    def transcript(self, session_id: int,
                   limit: int = 500) -> list[TranscriptRow]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM transcript WHERE session_id = ? "
                "ORDER BY at LIMIT ?", (session_id, limit)).fetchall()
        return [TranscriptRow(r["id"], r["session_id"], r["at"], r["who"],
                              r["station"] or "", r["text"], r["intent"] or "",
                              json.loads(r["meta"] or "{}"))
                for r in rows]

    def close(self) -> None:
        with self._lock:
            self._conn.close()


__all__ = ["History", "SessionRow", "TranscriptRow"]
