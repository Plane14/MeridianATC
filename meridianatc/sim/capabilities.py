"""Capabilities are properties of the state provider, not the UI using it."""

from dataclasses import dataclass


@dataclass(frozen=True)
class SimulatorCapabilities:
    name: str
    live: bool = False
    facilities: bool = False
    lvars: bool = False
    autofly: bool = False
    xplane_traffic: bool = False


_MANUAL = SimulatorCapabilities("Manual / replay")
_PROVIDERS = {
    "simconnect": SimulatorCapabilities("Microsoft Flight Simulator", True, True, True, True),
    "xplane": SimulatorCapabilities("X-Plane 12", live=True, xplane_traffic=True),
}


def capabilities(source: str) -> SimulatorCapabilities:
    return _PROVIDERS.get(source, _MANUAL)
