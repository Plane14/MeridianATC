from __future__ import annotations

from ._shared import (
    gsx_installed,
    log,
    time,
)


class _Frequency:
    # ------------------------------------------------------------------
    # frequency
    # ------------------------------------------------------------------

    def set_radio(self, mhz: float, radio: int = 1,
                  standby: bool = False) -> tuple[bool, str]:
        """Put a frequency into the aeroplane's radio, and follow it here.

        The panel used to be able to tune only when there was no simulator: a
        frequency picked on the Frequencies screen was refused with a note
        saying COM1 belongs to the aircraft. It does -- but a radio is a thing
        you set, and SimConnect has an event for setting it. So the panel
        writes into the aircraft's radio and then reads it back like any other
        change, which means the number in the cockpit, the number on this
        screen and the controller who answers are the same three things.

        Returns whether it took, and a sentence saying why not when it did
        not.
        """
        try:
            value = round(float(mhz), 3)
        except (TypeError, ValueError):
            return False, "that is not a frequency"
        if not (118.0 <= value <= 136.999):
            return False, "that is not an airband frequency"

        setter = getattr(self.source, "set_com", None)
        if setter is None:
            return False, "this aircraft state source has no radios"
        if not setter(value, radio=radio, standby=standby):
            if getattr(self.source, "connected", False):
                return False, ("the simulator would not accept a frequency "
                               "for this aircraft")
            return False, "no aircraft to tune"

        # Believe the write until the next read confirms it. Without this the
        # panel shows the old frequency for a poll interval, which reads as
        # the button not having worked.
        if radio == 1 and not standby:
            self.state.com1_active = value
            self._tuned_by_us = value
            return True, ""
        field_name = f"com{int(radio)}_{'standby' if standby else 'active'}"
        if hasattr(self.state, field_name):
            setattr(self.state, field_name, value)
        return True, ""

    def swap_radio(self, radio: int = 1) -> tuple[bool, str]:
        """Swap active and standby in the aircraft, as the radio's button does."""
        swap = getattr(self.source, "swap_com", None)
        if swap is None or not swap(radio):
            return False, "no aircraft to tune"
        active = f"com{int(radio)}_active"
        standby = f"com{int(radio)}_standby"
        if hasattr(self.state, active) and hasattr(self.state, standby):
            was = getattr(self.state, active)
            setattr(self.state, active, getattr(self.state, standby))
            setattr(self.state, standby, was)
            if radio == 1:
                self._tuned_by_us = self.state.com1_active
        return True, ""

    def set_squawk(self, code: str) -> tuple[bool, str]:
        """Dial a transponder code into the aircraft."""
        setter = getattr(self.source, "set_squawk", None)
        if setter is None or not setter(code):
            return False, "the transponder would not take that code"
        self.state.transponder_code = "".join(
            c for c in str(code) if c.isdigit())[:4].rjust(4, "0")
        return True, ""

    def spawn_on_runway(self) -> tuple[bool, str, str]:
        """Line an AI departure up on the runway in use, for the panel.

        Returns whether it was asked for, the callsign, and on refusal a
        reason key the panel maps to a ``comms.spawn_`` line in the
        catalogues. A busy runway is not a refusal: the aeroplane waits at
        the holding point. Nor is it on the runway yet: it goes into the
        simulator on the next tick, through the same sync every other flight
        takes, and the notice goes out once the simulator has drawn it -- or
        a different one if it cannot.
        """
        if not (self.config.traffic.enabled
                and getattr(self.config.traffic, "invented", True)):
            return False, "", "traffic_off"
        self.traffic.can_invent = self.simtraffic.drawing
        flight, why = self.traffic.spawn_on_runway(time.monotonic())
        if flight is None:
            return False, "", why
        # Said when the simulator has made it, not now: until then there is
        # nothing on the runway to look for.
        self._spawned[flight.written] = flight.runway
        return True, flight.written, ""

    def use_gsx_vehicles(self, wanted: bool) -> bool:
        """GSX Pro's vehicles for the traffic, or the simulator's own, from
        the panel and without a restart. Returns whether GSX's are now used:
        never without GSX installed, whatever was asked."""
        self.config.traffic.handling.gsx_vehicles = bool(wanted)
        use = bool(wanted) and gsx_installed()
        self.simvehicles.set_prefer_gsx(use)
        log.info("ground vehicles: %s", "GSX Pro's" if use
                 else "the simulator's own")
        return use

