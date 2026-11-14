"""Capabilities are properties of the state provider, not the UI using it."""

from dataclasses import dataclass

from .state import AircraftState


@dataclass(frozen=True)
class SimulatorCapabilities:
    name: str
    live: bool = False
    facilities: bool = False
    lvars: bool = False
    autofly: bool = False
    xplane_traffic: bool = False
    flight_axes: bool = False
    # The tug pushes until it is told to stop (X-Plane's ground-ops
    # commands), rather than a push of fixed length that lets go itself
    # (SimConnect's PUSHBACK service).
    pushback_open_ended: bool = False


_MANUAL = SimulatorCapabilities("Manual / replay")
_PROVIDERS = {
    "simconnect": SimulatorCapabilities("Microsoft Flight Simulator", True, True, True, True,
                                       flight_axes=True),
    "xplane": SimulatorCapabilities("X-Plane 12", live=True, autofly=True, xplane_traffic=True,
                                   flight_axes=True, pushback_open_ended=True),
}


def capabilities(source: str) -> SimulatorCapabilities:
    return _PROVIDERS.get(source, _MANUAL)


def autofly_limit(state: AircraftState) -> str:
    cap = capabilities(state.source)
    if not cap.autofly:
        return "Autonomous aircraft control is not available for this source."
    if not cap.flight_axes and (state.on_ground or state.category == "Helicopter"):
        return ("Full aircraft control is not available on this simulator. "
                "Use airborne fixed-wing autopilot assistance; taxi, takeoff, "
                "landing, go-around and rotorcraft controls remain manual.")
    return ""
