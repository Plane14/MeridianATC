"""X-Plane's localizers and VORs from earth_nav.dat.

X-Plane publishes every localizer in plain navigation data rather than the
airport's apt.dat. The rows needed here are the documented XP-NAV format:
``4/5 lat lon elev frequency*100 range bearing ident airport region runway``.
"""

from __future__ import annotations

import logging
from pathlib import Path

from ..xplane_plugin import install_roots

log = logging.getLogger(__name__)


def _rows(path: Path):
    try:
        lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    except OSError:
        return
    for line in lines:
        fields = line.split()
        if len(fields) < 11 or fields[0] not in ("3", "4", "5"):
            continue
        try:
            freq_units = float(fields[4])
            yield {
                "row": int(fields[0]),
                "lat": float(fields[1]), "lon": float(fields[2]),
                "elev_ft": float(fields[3]),
                # VORs and localizers are both written MHz * 100.
                "freq_khz": freq_units * 10.0,
                "ident": fields[7].upper(),
                "airport": fields[8].upper(),
                "region": fields[9].upper(),
                "runway": fields[10].upper(),
            }
        except (ValueError, IndexError):
            continue


class XPlaneNavaids:
    """An ident/airport/runway lookup over every earth_nav.dat."""

    def __init__(self, root=None):
        self._roots = install_roots(root) if root is not None else install_roots()
        self._records: list[dict] | None = None

    def _load(self) -> list[dict]:
        if self._records is not None:
            return self._records
        records = []
        for root in self._roots:
            for path in (
                    root / "Resources" / "default data" / "earth_nav.dat",
                    root / "Custom Data" / "earth_nav.dat"):
                records.extend(_rows(path) or ())
        self._records = records
        return records

    def navaid(self, ident: str, airport: str = "", runway: str = "") -> dict | None:
        ident = (ident or "").strip().upper()
        airport = (airport or "").strip().upper()
        runway = (runway or "").strip().upper()
        if not ident:
            return None
        candidates = [row for row in self._load() if row["ident"] == ident]
        if airport:
            exact = [row for row in candidates if row["airport"] == airport]
            if exact:
                candidates = exact
        if runway:
            exact = [row for row in candidates if row["runway"] == runway]
            if exact:
                candidates = exact
        if not candidates:
            return None
        row = candidates[0]
        return {
            "ident": row["ident"], "name": "",
            "type": "ILS" if row["row"] == 4 else
                    "LOC" if row["row"] == 5 else "VOR",
            "freq_khz": row["freq_khz"], "lat": row["lat"],
            "lon": row["lon"], "elev_ft": row["elev_ft"],
            "country": "", "airport": row["airport"],
        }


__all__ = ["XPlaneNavaids"]
