"""A simulator's own navaid record: ident and position to frequency.

The static navigation database covers VORs and NDBs but no localizers, so a
runway that correctly names ``IBRU`` still left the approach untuned. MSFS's
facility API can answer the navaid itself; X-Plane's earth_nav.dat reader is
in ``xpnavaids.py``. Only the fields needed here are requested -- the API
returns exactly the asked layout.
"""

from __future__ import annotations

import logging
import struct

from .taxiways import ask_facility

log = logging.getLogger(__name__)

_VOR = 19
_FIELDS = (
    b"OPEN VOR", b"VOR_LATITUDE", b"VOR_LONGITUDE", b"FREQUENCY",
    b"TYPE", b"HAS_GLIDE_SLOPE", b"NAME", b"CLOSE VOR",
)
_SHAPE = "<ddIii64s"
FT_PER_M = 3.28084


def decode_vor(raw: bytes, ident: str) -> dict | None:
    """One VOR/localizer facility block as the navaid lookup's row."""
    if len(raw) != struct.calcsize(_SHAPE):
        log.info("%s: a %d-byte VOR block where %d were expected",
                 ident, len(raw), struct.calcsize(_SHAPE))
        return None
    lat, lon, freq_hz, kind, has_gs, raw_name = struct.unpack(_SHAPE, raw)
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    name = raw_name.split(b"\0", 1)[0].decode("utf-8", "replace")
    return {
        "ident": ident.strip().upper(), "name": name,
        "type": "ILS" if kind == 4 or has_gs else "VOR",
        "freq_khz": float(freq_hz) / 1000.0,
        "lat": lat, "lon": lon, "elev_ft": None,
        "country": "", "airport": "",
    }


class SimNavaids:
    """MSFS facility-data navaids, one ident at a time."""

    TIMEOUT_S = 6.0

    def __init__(self, dll_path: str = ""):
        self.dll_path = dll_path
        self._cache: dict[tuple[str, str], dict | None] = {}

    def navaid(self, ident: str, region: str = "") -> dict | None:
        ident = (ident or "").strip().upper()
        region = (region or "").strip().upper()
        if not ident:
            return None
        key = (ident, region)
        if key in self._cache:
            return self._cache[key]
        found = None
        try:
            blocks = ask_facility(self.dll_path, b"MeridianATC navaid",
                                  _FIELDS, ident, self.TIMEOUT_S, region)
            for raw in (blocks or {}).get(_VOR, []):
                found = decode_vor(raw, ident)
                if found is not None:
                    break
        except Exception:
            log.info("asking the simulator for navaid %s raised", ident,
                     exc_info=True)
        self._cache[key] = found
        return found


__all__ = ["SimNavaids", "decode_vor"]
