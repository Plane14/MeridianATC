from __future__ import annotations

from ._shared import time


class _The_oxygen:
    # -- oxygen ------------------------------------------------------------

    # Masks go on before the regulatory altitude, not after the first
    # hypoxia symptom: 9,500 ft cabin altitude is the last convenient
    # thousand-foot mark below the 10,000-ft planning figure.
    OXYGEN_ON_FT = 9500.0

    def _manage_oxygen(self, state) -> None:
        """Use the crew oxygen system before the cabin reaches 10,000 ft.

        Prevention is primarily procedural: an unpressurised aeroplane
        with a bottle does not have to stay low, but the bottle only
        counts when the crew is plugged in and the valve is open. Where
        the model has no system at all the planning ceiling in
        ``x_the_envelope`` keeps the flight below the hypoxia heights;
        here the pilot is told once if manual flying has put the cabin
        there anyway.
        """
        cabin = float(getattr(state, "cabin_altitude_ft", 0.0) or 0.0)
        if not cabin:
            cabin = float(getattr(state, "pressure_altitude_ft", 0.0)
                          or getattr(state, "altitude_ft", 0.0) or 0.0)
        felt = float(getattr(state, "pilot_felt_altitude_ft", 0.0) or 0.0)
        if max(cabin, felt) < self.OXYGEN_ON_FT:
            return
        if not self._oxygen_system_available(state):
            if not getattr(self, "_no_oxygen_warned", False):
                self._no_oxygen_warned = True
                self._emit(
                    "info",
                    f"No supplemental oxygen: cabin altitude {cabin:.0f} ft "
                    "is in the hypoxia range",
                    "autonomous.oxygen.none")
            return
        now = time.monotonic()
        if now - getattr(self, "_oxygen_action_at", 0.0) < 5.0:
            return
        self._oxygen_action_at = now
        plugged = int(getattr(state, "oxygen_plugged_in", 0) or 0)
        if plugged < 1:
            self.director.write("OXYGEN PLUGGED IN", 2.0)
        if int(getattr(state, "oxygen_flow_setting", 0) or 0) < 1:
            # X-Plane's normal demand setting: oxygen when needed now,
            # with the lower night flow. Zero is no flow at all even
            # with the bottle valve open.
            self.director.write("OXYGEN FLOW SETTING", 2.0)
        if getattr(state, "oxygen_on", False) is not True \
                and self.director.write("OXYGEN SWITCH", 1.0) \
                and not getattr(self, "_oxygen_on_said", False):
            self._oxygen_on_said = True
            self._emit("info", "Crew oxygen on before 10,000 ft",
                       "autonomous.oxygen.on")
