"""Discover X-Plane's registered installs and manage the optional traffic plugin."""

from __future__ import annotations

import json
import os
import select
import shutil
import socket
import sys
import time
from pathlib import Path

from .paths import bundle_root

NAME = "MeridianATC"
PAYLOAD_FOLDER = "xplane-plugin"
INI = "meridianatc-xp.ini"


def platform_folder() -> str:
    return "win_x64" if sys.platform == "win32" else "mac_x64" if sys.platform == "darwin" else "lin_x64"


def install_roots(path=None, *, platform=None, home=None, localappdata=None) -> list[Path]:
    """Read Laminar's install lists, not guessed Steam/library locations.

    https://developer.x-plane.com/article/how-to-programmatically-locate-x-plane-9-or-10/
    An explicit path is an X-Plane root, never the plugins directory itself.
    """
    if path is not None:
        root = Path(path).expanduser().resolve()
        if not (root / "Resources" / "plugins").is_dir():
            raise FileNotFoundError(f"Not an X-Plane install: {root} (Resources/plugins is missing)")
        return [root]
    platform = platform or sys.platform
    home = Path(home) if home is not None else Path.home()
    if platform == "win32":
        base = Path(localappdata or os.environ.get("LOCALAPPDATA") or home / "AppData" / "Local")
    elif platform == "darwin":
        base = home / "Library" / "Preferences"
    else:
        base = home / ".x-plane"
    found, seen = [], set()
    for listing in sorted(base.glob("x-plane_install_12*.txt")):
        try:
            lines = listing.read_text(encoding="utf-8-sig").splitlines()
        except (OSError, UnicodeError):
            continue
        for line in lines:
            text = line.strip().strip('"')
            if not text or "\0" in text:
                continue
            root = Path(text).expanduser()
            if not root.is_absolute() or not (root / "Resources" / "plugins").is_dir():
                continue
            root = root.resolve()
            key = str(root).casefold() if platform == "win32" else str(root)
            if key not in seen:
                seen.add(key)
                found.append(root)
    return found


def payload(path=None, *, app_root=None) -> Path:
    """Find a built native payload in a bundle, installed app or checkout."""
    roots = [Path(app_root)] if app_root is not None else [bundle_root(), Path(sys.executable).resolve().parent]
    candidates = [Path(path)] if path is not None else [
        candidate for root in roots for candidate in (
            root / PAYLOAD_FOLDER / NAME,
            root / "_internal" / PAYLOAD_FOLDER / NAME,
            root / "xplane" / "plugin" / "build" / NAME,
        )
    ]
    for candidate in candidates:
        if ((candidate / platform_folder() / f"{NAME}.xpl").is_file()
                and (candidate / "Resources" / "Doc8643.txt").is_file()):
            return candidate.resolve()
    raise FileNotFoundError(
        "MeridianATC-XP payload is absent or not built for this platform. "
        "Build xplane/plugin with CMake or download a native plugin artifact; see xplane/README.md.")


def plugin_directory(root) -> Path:
    return install_roots(root)[0] / "Resources" / "plugins" / NAME


def _ini_port(directory: Path) -> int | None:
    """The port one plugin folder's INI puts its listener on.

    Read the way the plugin's own ``ReadConfig`` does: blank lines and
    ``#``/``;``/section lines are skipped, the first ``=`` splits the
    pair, and a port outside 1-65535 is not a port at all.
    """
    try:
        lines = (directory / INI).read_text(encoding="utf-8",
                                            errors="replace").splitlines()
    except OSError:
        return None
    for line in lines:
        line = line.strip(" \t\r\n")
        if not line or line[0] in "#;[":
            continue
        key, sep, value = line.partition("=")
        if not sep or key.strip() != "port":
            continue
        try:
            port = int(value.strip())
        except ValueError:
            continue
        if 1 <= port <= 65535:
            return port
    return None


def plugin_ports(path=None, *, home=None, localappdata=None) -> list[int]:
    """The UDP ports the installed plugins' INIs put their listeners on.

    The plugin's own INI is the authority: the install preserves it
    across updates, so it outlives the port ``config.yaml`` was last
    written with. An engine that only knows the config's number talks
    into the void -- the traffic link, the follow-me car and the FMS
    readback all die together, and silently. Only roots where the
    plugin itself is installed count: the INI is kept after an
    uninstall, and a file with no .xpl to read it governs nothing.
    """
    try:
        roots = install_roots(path, home=home, localappdata=localappdata)
    except (FileNotFoundError, OSError):
        return []
    ports = []
    for root in roots:
        directory = root / "Resources" / "plugins" / NAME
        if not (directory / platform_folder() / f"{NAME}.xpl").is_file():
            continue
        port = _ini_port(directory)
        if port is not None and port not in ports:
            ports.append(port)
    return ports


def resolve_port(host="127.0.0.1", configured=49010, timeout=.4) -> int:
    """The port the plugin link should use.

    An INI port wins over the config's: it is what the loaded .xpl
    bound. Several installs can disagree, though, and an inactive
    one's file would poison the link just as a stale config did -- so
    each candidate is asked first, and the plugin that answers is the
    running sim's. Unanswered, a single install's INI still wins (the
    sim may not be up yet); several unanswered are ambiguous, and the
    configured port is the safer guess.
    """
    try:
        ini = [port for port in plugin_ports() if port != configured]
    except Exception:
        ini = []
    if not ini:
        return configured
    for candidate in ini:
        if probe(host, candidate, timeout=timeout):
            return candidate
    return ini[0] if len(ini) == 1 else configured


def installed(root) -> dict:
    directory = plugin_directory(root)
    present = (directory / platform_folder() / f"{NAME}.xpl").is_file()
    version = "unknown"
    try:
        metadata = json.loads((directory / "version.json").read_text(encoding="utf-8"))
        if isinstance(metadata, dict) and isinstance(metadata.get("version"), str):
            version = metadata["version"]
    except (OSError, ValueError):
        pass
    return {"path": str(directory), "installed": present, "version": version if present else "not installed"}


def install(root, source=None) -> Path:
    """Update binaries/resources without overwriting an existing INI or CSL tree."""
    source = payload(source)
    target = plugin_directory(root)
    if target.is_symlink() or target == source or source in target.parents or target in source.parents:
        raise ValueError("Plugin source and destination must be separate real directories")
    for old in target.rglob("*") if target.exists() else ():
        rel = old.relative_to(target)
        if rel == Path(INI) or rel == Path("Resources/CSL") or Path("Resources/CSL") in rel.parents:
            continue
        if old.is_symlink():
            raise ValueError(f"Refusing to overwrite a plugin symlink: {old}")

    def preserve(directory, names):
        rel = Path(directory).relative_to(source)
        keep = []
        if rel == Path(".") and ((target / INI).exists() or (target / INI).is_symlink()):
            keep.append(INI)
        if rel == Path("Resources") and ((target / "Resources/CSL").exists() or (target / "Resources/CSL").is_symlink()):
            keep.append("CSL")
        return keep

    shutil.copytree(source, target, dirs_exist_ok=True, ignore=preserve)
    return target


def uninstall(root) -> Path:
    """Remove the plugin, retaining the pilot's INI and installed CSL models."""
    target = plugin_directory(root)
    if target.is_symlink():
        raise ValueError(f"Refusing to uninstall a plugin symlink: {target}")
    if not target.exists():
        return target
    for item in target.iterdir():
        if item.name.casefold() == INI:
            continue
        if item.name.casefold() == "resources" and not item.is_symlink():
            for resource in item.iterdir():
                if resource.name.casefold() != "csl":
                    _remove(resource)
            if not any(item.iterdir()):
                item.rmdir()
        else:
            _remove(item)
    if not any(target.iterdir()):
        target.rmdir()
    return target


def _remove(path):
    if path.is_dir() and not path.is_symlink():
        shutil.rmtree(path)
    else:
        path.unlink()


def probe(host="127.0.0.1", port=49010, timeout=.4) -> dict | None:
    """Request status only: never clear traffic or send an aircraft snapshot."""
    address = socket.gethostbyname(host)
    if address != "127.0.0.1":
        raise ValueError("MeridianATC-XP only accepts localhost traffic")
    endpoint = (address, int(port))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as link:
        link.bind(("127.0.0.1", 0))
        link.setblocking(False)
        link.sendto(b'{"v":1,"t":"hello"}', endpoint)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not select.select([link], [], [], max(0, deadline - time.monotonic()))[0]:
                break
            try:
                wire, sender = link.recvfrom(65536)
                status = json.loads(wire)
                if (sender == endpoint and isinstance(status, dict)
                        and type(status.get("v")) is int and status["v"] == 1
                        and status.get("t") == "status" and status.get("plugin") == "MeridianATC-XP"
                        and isinstance(status.get("version"), str)
                        and type(status.get("csl_models")) is int and status["csl_models"] >= 0):
                    return status
            except (OSError, ValueError, RecursionError):
                continue
    return None
