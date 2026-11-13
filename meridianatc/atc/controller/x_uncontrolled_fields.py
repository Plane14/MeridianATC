from __future__ import annotations

from ._shared import (
    Reply,
)


class _Uncontrolled_fields:
    # ------------------------------------------------------------------
    # uncontrolled fields
    # ------------------------------------------------------------------

    # Set by the engine: ``ident -> Weather``, the observation at a field.
    # Flight service reads the weather where the pilot is going, which is
    # not the field the station is filed at.
    weather_for = None

    def _info(self, parsed, session, station, weather, p, state) -> Reply:
        """An information frequency: AFIS, FIS or flight service
        (:mod:`meridianatc.atc.afis`). Information, never a clearance."""
        from .. import afis

        return afis.answer(self, parsed, session, station, weather, p, state)

    def _ctaf(self, parsed, session, station, weather, p, state) -> Reply:
        """An uncontrolled field has nobody to answer.

        Staying silent is the correct behaviour and the realistic one: on CTAF
        the pilot self-announces and no controller replies. Reached only if a
        broadcast position slips past the check at the top of :meth:`respond`,
        which is why that check is the one that matters.
        """
        return Reply("", station, silent=True)

