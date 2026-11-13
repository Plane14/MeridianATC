"""The hour of day where the aeroplane is, not where the computer is.

A field's traffic rhythm, a purser's "the local time is", and the cabin
lights dimming for a night departure are the field's own clock's
business -- the host's timezone was reading them for wherever the PC
happened to sit, which said "good evening" at fields where it was three
in the morning. No civil timezone table is carried in the navdata, so
the hour is solar: UTC plus the longitude's fifteenth of a day. That is
inside an hour of the civil clock almost everywhere, and exact on the
meridian each timezone is pinned to. The same trick afis's sun angle
uses, pulled apart so traffic and the cabin can share it.
"""

from __future__ import annotations

import time


def field_hour(lon_deg: float, utc_seconds: float | None = None) -> float:
    """The local hour at a longitude, 0.0 <= h < 24.0, solar.

    UTC -- which the host's clock always has right whatever zone it is
    pretending to be in -- plus the longitude's share of a day.
    """
    utc = time.gmtime(utc_seconds)
    return (utc.tm_hour + utc.tm_min / 60.0 + utc.tm_sec / 3600.0
            + float(lon_deg or 0.0) / 15.0) % 24.0


def host_hour() -> float:
    """The computer's own local hour: the fallback when no field and no
    aeroplane can say where they are."""
    t = time.localtime()
    return t.tm_hour + t.tm_min / 60.0


__all__ = ["field_hour", "host_hour"]
