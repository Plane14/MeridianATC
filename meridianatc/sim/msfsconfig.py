"""The MSFS aircraft configuration SimConnect does not expose.

SimConnect reports the loaded aircraft's title but not the ``engines.cfg``
propeller ``rotation`` list. That list is static configuration, so it is
read from the installed package once and cached by title. Nothing here is
written; the packages belong to the simulator and their authors.
"""

from __future__ import annotations

import re
from functools import lru_cache
from pathlib import Path

from ..integrations.fsltl import packages_folders

_TITLE = re.compile(r'^\s*title\s*=\s*"?([^"\r\n;]+)"?\s*(?:;.*)?$',
                    re.I | re.M)
_ROTATION = re.compile(r'^\s*rotation\s*=\s*([^;\r\n]+)', re.I | re.M)
_SECTION = re.compile(r'^\s*\[([^\]]+)\]', re.I | re.M)


def _normal_title(title: str) -> str:
    return " ".join((title or "").split()).casefold()


def _aircraft_cfgs(package_root: Path) -> list[Path]:
    """The aircraft.cfg files in one InstalledPackagesPath tree."""
    found: list[Path] = []
    containers = [package_root / "Community", package_root / "Official"]
    one_store = package_root / "Official" / "OneStore"
    if one_store.is_dir():
        containers.append(one_store)
    for container in containers:
        try:
            packages = list(container.iterdir())
        except OSError:
            continue
        for package in packages:
            if not package.is_dir() or package.name == "OneStore":
                continue
            for pattern in (
                    # MSFS 2020 and legacy packages: one aircraft.cfg in
                    # the SimObject container, with engines.cfg beside it.
                    "SimObjects/Airplanes/*/aircraft.cfg",
                    "SimObjects/aircraft/*/aircraft.cfg",
                    # MSFS 2024 modular aircraft: each preset has its own
                    # config folder, merged over the aircraft's Common one.
                    "SimObjects/Airplanes/*/Presets/*/config/aircraft.cfg",
                    "SimObjects/Airplanes/*/presets/*/config/aircraft.cfg"):
                try:
                    found.extend(package.glob(pattern))
                except OSError:
                    continue
    return sorted(set(found))


def _titles(path: Path) -> set[str]:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return set()
    return {_normal_title(match.group(1)) for match in _TITLE.finditer(text)}


@lru_cache(maxsize=8)
def _title_index(folder_keys: tuple[str, ...]) -> dict[str, Path]:
    """Aircraft title -> its container's aircraft.cfg, for these roots."""
    found: dict[str, Path] = {}
    for key in folder_keys:
        for cfg in _aircraft_cfgs(Path(key)):
            for title in _titles(cfg):
                # The first package wins, matching the simulator's own
                # package-order idea closely enough for static config.
                found.setdefault(title, cfg)
    return found


def _propeller_section(text: str) -> str:
    """Only the [PROPELLER] section, so another rotation= cannot answer."""
    starts = list(_SECTION.finditer(text))
    for index, match in enumerate(starts):
        if match.group(1).strip().casefold() != "propeller":
            continue
        end = starts[index + 1].start() if index + 1 < len(starts) else len(text)
        return text[match.end():end]
    return ""


def parse_rotation(engines_cfg: str, count: int) -> tuple[float, ...]:
    """The propeller rotation list, +1 clockwise and -1 anticlockwise.

    MSFS documents clockwise as the default, so a short list is padded in
    that direction; an absent list is empty, meaning unpublished.
    """
    match = _ROTATION.search(_propeller_section(engines_cfg))
    if match is None:
        return ()
    signs: list[float] = []
    for part in match.group(1).split(","):
        try:
            value = float(part.strip().strip('"'))
        except ValueError:
            continue
        if value:
            signs.append(1.0 if value > 0.0 else -1.0)
    if not signs:
        return ()
    while len(signs) < count:
        signs.append(1.0)
    return tuple(signs[:max(1, count)])


def propeller_rotation(title: str, count: int,
                       folders: list[Path] | tuple[Path, ...] | None = None,
                       ) -> tuple[float, ...]:
    """The loaded aircraft's propeller directions, or empty if unknown."""
    title_key = _normal_title(title)
    if not title_key or count <= 0:
        return ()
    if folders is None:
        folders = packages_folders()
    keys = tuple(str(folder) for folder in folders)
    cfg = _title_index(keys).get(title_key)
    if cfg is None:
        return ()
    candidates = [cfg.parent / "engines.cfg"]
    for ancestor in cfg.parents:
        if ancestor.name.casefold() == "presets":
            common = ancestor.parent / "Common" / "config" / "engines.cfg"
            candidates.append(common)
            break
    for engines in candidates:
        try:
            text = engines.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        found = parse_rotation(text, count)
        if found:
            return found
    return ()
