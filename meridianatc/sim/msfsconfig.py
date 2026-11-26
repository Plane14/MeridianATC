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


def _section(text: str, name: str) -> str:
    """One CFG section, so same-named keys elsewhere cannot answer."""
    starts = list(_SECTION.finditer(text))
    for index, match in enumerate(starts):
        if match.group(1).strip().casefold() != name.casefold():
            continue
        end = starts[index + 1].start() if index + 1 < len(starts) else len(text)
        return text[match.end():end]
    return ""


def _parameter(section: str, name: str) -> str:
    match = re.search(r"^\s*" + re.escape(name) + r"\s*=\s*([^;\r\n]+)",
                      section, re.I | re.M)
    return match.group(1).strip().strip('"') if match else ""


def _boolean(section: str, name: str, default: bool) -> bool:
    value = _parameter(section, name)
    if not value:
        return default
    if value.casefold() in ("true", "yes", "on"):
        return True
    if value.casefold() in ("false", "no", "off"):
        return False
    try:
        return float(value.split()[0]) != 0.0
    except (ValueError, IndexError):
        return default


def _modern_propellers(section: str) -> bool:
    """Whether the modern propeller model owns the physical direction."""
    version_text = _parameter(section, "prop_mod_version")
    try:
        version = int(float(version_text.split()[0])) if version_text else None
    except (ValueError, IndexError):
        version = None
    use_modern = _boolean(section, "prop_mod_use_modern", False)
    if version is None:
        return use_modern
    # MSFS documents the legacy switch as winning even over an explicit
    # version 0; only a positive version selects the modern model by itself.
    return version > 0 or use_modern


def _engine_laterals(text: str) -> dict[int, float]:
    """Engine index -> lateral offset, negative left and positive right."""
    section = _section(text, "GENERALENGINEDATA")
    found: dict[int, float] = {}
    for match in re.finditer(r"^\s*engine\.(\d+)\s*=\s*([^;\r\n]+)",
                             section, re.I | re.M):
        parts = [part.strip() for part in match.group(2).split(",")]
        if len(parts) < 2:
            continue
        try:
            found[int(match.group(1))] = float(parts[1])
        except ValueError:
            continue
    return found


def _modern_rotation(engines_cfg: str, count: int) -> tuple[float, ...]:
    """Physical directions under MSFS's modern propeller model.

    Unlike legacy ``rotation``, these switches are by aircraft side rather
    than an ordered list. A clear reverse flag is an anticlockwise propeller;
    a false one is clockwise. Their documented defaults are left reversed,
    right and centre not reversed.
    """
    section = _propeller_section(engines_cfg)
    reverse = {
        "left": _boolean(section, "prop_mod_reverse_left", True),
        "right": _boolean(section, "prop_mod_reverse_right", False),
        "centre": _boolean(section, "prop_mod_reverse_centre", False),
    }
    laterals = _engine_laterals(engines_cfg)
    signs: list[float] = []
    for index in range(max(1, count)):
        lateral = laterals.get(index)
        if lateral is None:
            if count <= 1 or count % 2 and index == count // 2:
                lateral = 0.0
            else:
                lateral = -1.0 if index < count / 2.0 else 1.0
        side = "centre" if abs(lateral) < 0.05 else (
            "left" if lateral < 0.0 else "right")
        signs.append(-1.0 if reverse[side] else 1.0)
    return tuple(signs[:max(1, count)])


def _propeller_section(text: str) -> str:
    """Only the [PROPELLER] section, so another rotation= cannot answer."""
    return _section(text, "PROPELLER")


def parse_rotation(engines_cfg: str, count: int) -> tuple[float, ...]:
    """The physical propeller directions, +1 clockwise and -1 anticlockwise.

    Legacy aircraft use the ordered ``rotation`` list. MSFS 2024's modern
    propeller model ignores that list for physics and uses the three
    ``prop_mod_reverse_*`` switches instead, mapped onto each engine's
    lateral position. There is no direct SimConnect variable for either:
    ``PROP ROTATION ANGLE`` is only the animated angle in radians.
    """
    if _modern_propellers(_propeller_section(engines_cfg)):
        return _modern_rotation(engines_cfg, count)
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
