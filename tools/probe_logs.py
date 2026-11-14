"""Watch meridianatc logs for issues, scanning every few seconds.

Tails the files in the log directory instead of re-reading them: each
pass picks up where the last one stopped, parses the new records, and
reports anything that looks like a problem:

- meridianatc.jsonl: records at WARNING and above (the .log mirror is
  skipped so nothing is reported twice).
- radio.jsonl / crew.jsonl: event records carrying failure flags such
  as "fired": false, "verified": false, "event": "abandoned".

Repeated identical findings are collapsed: the first occurrence prints
at once, repeats are tallied and flushed as a single "(xN)" line so a
per-second warning does not bury everything else.

Usage:
    python tools/probe_logs.py [--dir D:\\MeridianATC\\logs] [--every 4]
                               [--from-start] [--heartbeat 8]
"""
import argparse
import json
import os
import re
import sys
import time

LEVELS = {"WARNING": 30, "ERROR": 40, "CRITICAL": 50}
ALERT_LEVEL = 30

# Keys whose value marks an event record as a problem when it looks
# negative.  Checked against the decoded json of radio/crew records.
BAD_EVENT_VALUES = {"abandoned", "failed", "denied", "rejected", "timeout", "missed"}
BAD_FLAG_KEYS = re.compile(r"^(verified|kept|heard|understood|ok|success)$")
BAD_VALUE_KEYS = re.compile(r"(fail|error|denied|reject|timeout|missed|abort|exception)", re.I)
NUM = re.compile(r"\d+(?:\.\d+)?")
HEX = re.compile(r"\b[0-9a-f]{8,}\b")


def signature(text):
    """Collapse a message to a stable shape so repeats dedupe."""
    text = NUM.sub("N", text)
    return HEX.sub("H", text)


def bad_event_fields(rec):
    """Return 'key=value' strings for fields that mark an event as bad."""
    out = []
    for key, val in rec.items():
        if BAD_FLAG_KEYS.match(key) and val is False:
            out.append(f"{key}=false")
        elif isinstance(val, str) and (
            key in ("event", "why", "what", "result")
            and val in BAD_EVENT_VALUES
            or BAD_VALUE_KEYS.search(key) and val not in ("none", "ok")
        ):
            out.append(f"{key}={val!r}")
    return out


class Tail:
    def __init__(self, path, from_start):
        self.path = path
        self.pos = os.path.getsize(path) if not from_start else 0
        self.buf = b""

    def new_lines(self):
        try:
            size = os.path.getsize(self.path)
        except OSError:
            return []
        if size < self.pos:  # rotated or truncated: start over
            self.pos, self.buf = 0, b""
        if size == self.pos:
            return []
        with open(self.path, "rb") as fh:
            fh.seek(self.pos)
            chunk = fh.read()
        self.pos = size
        data = self.buf + chunk
        lines = data.split(b"\n")
        self.buf = lines.pop()  # last piece may be a partial line
        return [ln.decode("utf-8", "replace") for ln in lines if ln.strip()]


class Scan:
    def __init__(self, kind, path, from_start):
        self.kind = kind
        self.tail = Tail(path, from_start)
        self.name = os.path.basename(path)

    def poll(self):
        findings = []
        for line in self.tail.new_lines():
            try:
                rec = json.loads(line)
            except ValueError:
                findings.append((f"{self.name}: unparseable line", line.strip()[:200]))
                continue
            if self.kind == "leveled":
                if LEVELS.get(str(rec.get("level", "")).upper(), 0) >= ALERT_LEVEL:
                    text = f"{rec.get('logger', '?')}: {rec.get('message', line)}"
                    findings.append((signature(text), f"{rec.get('level')} {text}"))
            else:
                bad = bad_event_fields(rec)
                if bad:
                    text = (
                        f"{self.name}: {rec.get('kind', '?')}.{rec.get('event', rec.get('what', '?'))}"
                        f" flagged ({', '.join(bad)})"
                    )
                    if rec.get("why"):
                        text += f" — {rec['why']}"
                    if rec.get("cue"):
                        text += f" [{rec['cue']}]"
                    elif rec.get("checklist"):
                        text += f" [{rec['checklist']}]"
                    findings.append((signature(text), text))
        return findings


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=r"D:\MeridianATC\logs")
    ap.add_argument("--every", type=float, default=4.0)
    ap.add_argument("--from-start", action="store_true")
    ap.add_argument("--heartbeat", type=int, default=8,
                    help="print a quiet status line every N scans (0=off)")
    ap.add_argument("--min-level", default="WARNING",
                    choices=list(LEVELS) + ["INFO"])
    args = ap.parse_args()

    global ALERT_LEVEL
    ALERT_LEVEL = LEVELS.get(args.min_level, 0)

    scans = [
        Scan("leveled", os.path.join(args.dir, "meridianatc.jsonl"), args.from_start),
        Scan("events", os.path.join(args.dir, "radio.jsonl"), args.from_start),
        Scan("events", os.path.join(args.dir, "crew.jsonl"), args.from_start),
    ]
    missing = [s.name for s in scans if not os.path.exists(s.tail.path)]
    if missing:
        sys.exit(f"no such log file(s): {', '.join(missing)} in {args.dir}")

    print(f"probe_logs: watching {args.dir} every {args.every}s "
          f"(level>={args.min_level}, from {'start' if args.from_start else 'now'})", flush=True)

    pending = {}  # signature -> [unflushed count, text, total]
    cycles = 0
    while True:
        cycles += 1
        now = time.strftime("%H:%M:%S")
        for scan in scans:
            for sig, text in scan.poll():
                if sig in pending:
                    pending[sig][0] += 1
                    pending[sig][2] += 1
                else:
                    pending[sig] = [0, text, 1]
                    print(f"{now} {text}", flush=True)
        if args.heartbeat and cycles % args.heartbeat == 0:
            for sig, (count, text, total) in pending.items():
                if count:
                    print(f"{now}   ... {text}  (x{count} more, {total} total)",
                          flush=True)
                    pending[sig][0] = 0
            print(f"{now} -- scan {cycles}: watching", flush=True)
        time.sleep(args.every)


if __name__ == "__main__":
    main()
