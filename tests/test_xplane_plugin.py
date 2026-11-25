"""Installation is filesystem-only; diagnostics use real fake UDP peers."""

from __future__ import annotations

import socket
import zipfile

import pytest
from test_xplane import FakeXPlane
from test_xptraffic import FakePlugin

from meridianatc import health
from meridianatc import xplane_plugin as xp
from meridianatc.config import Config
from meridianatc.installer import app, core


@pytest.fixture
def root(tmp_path):
    root = tmp_path / "René X-Plane 12"
    (root / "Resources/plugins").mkdir(parents=True)
    return root


@pytest.fixture
def source(tmp_path):
    source = tmp_path / "payload" / xp.NAME
    (source / xp.platform_folder()).mkdir(parents=True)
    (source / xp.platform_folder() / "MeridianATC.xpl").write_bytes(b"native payload")
    (source / "Resources").mkdir()
    (source / "Resources/Doc8643.txt").write_text("models", encoding="utf-8")
    (source / "version.json").write_text('{"version":"0.1.0"}', encoding="utf-8")
    return source


@pytest.mark.parametrize("platform", ["win32", "darwin", "linux"])
def test_official_lists(platform, tmp_path, root):
    base = {"win32": tmp_path / "Local", "darwin": tmp_path / "Library/Preferences",
            "linux": tmp_path / ".x-plane"}[platform]
    base.mkdir(parents=True)
    second = tmp_path / "second"
    (second / "Resources/plugins").mkdir(parents=True)
    (base / "x-plane_install_12.txt").write_text(
        f'\ufeff{root}\n\n"{root}"\n{tmp_path / "stale"}\n{second}\nrelative\n', encoding="utf-8")
    (base / "x-plane_install_12_extra.txt").write_text(str(root), encoding="utf-8")
    (base / "x-plane_install_12_corrupt.txt").write_bytes(b"\xff")
    (base / "x-plane_install_11.txt").write_text(str(tmp_path), encoding="utf-8")
    assert xp.install_roots(platform=platform, home=tmp_path, localappdata=tmp_path / "Local") == [root.resolve(), second.resolve()]


def test_empty_and_explicit_discovery(tmp_path, root):
    assert xp.install_roots(home=tmp_path, localappdata=tmp_path / "absent") == []
    assert xp.install_roots(root) == [root.resolve()]
    with pytest.raises(FileNotFoundError, match="Resources/plugins"):
        xp.install_roots(tmp_path)


def test_windows_localappdata_environment(monkeypatch, tmp_path, root):
    monkeypatch.setenv("LOCALAPPDATA", str(tmp_path))
    (tmp_path / "x-plane_install_12.txt").write_text(str(root), encoding="utf-8")
    assert xp.install_roots(platform="win32", home=tmp_path / "home") == [root.resolve()]


@pytest.mark.parametrize("relative", ["xplane-plugin/MeridianATC",
                                     "_internal/xplane-plugin/MeridianATC",
                                     "xplane/plugin/build/MeridianATC"])
def test_payload_lookup(tmp_path, source, relative, monkeypatch):
    destination = tmp_path / "app" / relative
    destination.parent.mkdir(parents=True)
    source.rename(destination)
    assert xp.payload(app_root=tmp_path / "app") == destination.resolve()
    monkeypatch.setattr(xp, "bundle_root", lambda: tmp_path / "app")
    assert xp.payload() == destination.resolve()


def test_missing_or_wrong_platform_payload(tmp_path, source):
    with pytest.raises(FileNotFoundError, match="CMake"):
        xp.payload(app_root=tmp_path)
    (source / xp.platform_folder() / "MeridianATC.xpl").unlink()
    with pytest.raises(FileNotFoundError, match="not built for this platform"):
        xp.payload(source)


def test_install_upgrade_uninstall_preserves_user_files(root, source):
    (source / xp.INI).write_text("port=49010", encoding="utf-8")
    (source / "Resources/CSL").mkdir()
    (source / "Resources/CSL/README").write_text("stock", encoding="utf-8")
    target = xp.install(root, source)
    assert xp.installed(root) == {"path": str(target), "installed": True, "version": "0.1.0"}
    (target / xp.INI).write_text("port=49011", encoding="utf-8")
    (target / "Resources/CSL/README").write_text("pilot", encoding="utf-8")
    (target / "Resources/CSL/model.obj").write_bytes(b"CSL model")
    (source / "Resources/Doc8643.txt").write_text("updated", encoding="utf-8")
    xp.install(root, source)
    assert (target / "Resources/Doc8643.txt").read_text() == "updated"
    assert (target / xp.INI).read_text() == "port=49011"
    assert (target / "Resources/CSL/README").read_text() == "pilot"
    xp.uninstall(root)
    assert not xp.installed(root)["installed"]
    assert (target / xp.INI).read_text() == "port=49011"
    assert (target / "Resources/CSL/model.obj").read_bytes() == b"CSL model"
    assert not (target / "Resources/Doc8643.txt").exists()
    xp.uninstall(root)


def _install_dir(root):
    directory = xp.plugin_directory(root)
    (directory / xp.platform_folder()).mkdir(parents=True)
    (directory / xp.platform_folder() / f"{xp.NAME}.xpl").write_bytes(b"xpl")
    return directory


def test_plugin_ports_reads_installed_inis(root, tmp_path, monkeypatch):
    directory = _install_dir(root)
    (directory / xp.INI).write_text(
        "# comment\n; other\n[section]\nlabels=1\nport = 52510 \nport=oops\n",
        encoding="utf-8")
    # A kept INI under a root whose plugin was uninstalled governs nothing.
    stale = tmp_path / "uninstalled"
    (stale / "Resources/plugins").mkdir(parents=True)
    stale_dir = _install_dir(stale)
    (stale_dir / xp.INI).write_text("port=49999\n", encoding="utf-8")
    (stale_dir / xp.platform_folder() / f"{xp.NAME}.xpl").unlink()
    monkeypatch.setattr(xp, "install_roots", lambda *a, **kw: [stale, root])
    assert xp.plugin_ports() == [52510]


def test_plugin_ports_absent_invalid_or_out_of_range(root, monkeypatch):
    monkeypatch.setattr(xp, "install_roots", lambda *a, **kw: [root])
    assert xp.plugin_ports() == []
    ini = _install_dir(root) / xp.INI
    ini.write_text("tcas=1\n", encoding="utf-8")
    assert xp.plugin_ports() == []
    ini.write_text("port=99999\nport=0\nport=-1\nport=abc\n", encoding="utf-8")
    assert xp.plugin_ports() == []
    monkeypatch.setattr(xp, "install_roots",
                        lambda *a, **kw: (_ for _ in ()).throw(FileNotFoundError))
    assert xp.plugin_ports() == []


def test_resolve_port_probes_disagreeing_inis(monkeypatch):
    monkeypatch.setattr(xp, "plugin_ports", lambda: [52510, 53000])
    answers = {53000: {"v": 1, "t": "status", "plugin": "MeridianATC-XP",
                     "version": "x", "csl_models": 1}}
    monkeypatch.setattr(xp, "probe", lambda host, port, timeout=.4: answers.get(port))
    # The install whose plugin actually answers is the running sim's.
    assert xp.resolve_port(configured=49010) == 53000
    monkeypatch.setattr(xp, "plugin_ports", lambda: [52510])
    assert xp.resolve_port(configured=49010) == 52510
    # Several silent installs are ambiguous; the config is the safer guess.
    monkeypatch.setattr(xp, "plugin_ports", lambda: [52510, 53000])
    monkeypatch.setattr(xp, "probe", lambda *a, **kw: None)
    assert xp.resolve_port(configured=49010) == 49010
    monkeypatch.setattr(xp, "plugin_ports", lambda: [])
    assert xp.resolve_port(configured=49010) == 49010


def test_clean_uninstall_and_older_metadata(root, source):
    target = xp.install(root, source)
    (target / "version.json").unlink()
    assert xp.installed(root)["version"] == "unknown"
    (target / "version.json").write_text("[]", encoding="utf-8")
    assert xp.installed(root)["version"] == "unknown"
    xp.uninstall(root)
    assert not target.exists()
    xp.uninstall(root)
    assert xp.installed(root)["version"] == "not installed"


def test_invalid_targets_do_not_copy(tmp_path, root, source):
    with pytest.raises(FileNotFoundError):
        xp.install(tmp_path, source)
    with pytest.raises(FileNotFoundError):
        xp.install(root, tmp_path / "missing")
    assert not xp.plugin_directory(root).exists()


def test_cli(root, source, tmp_path, monkeypatch, capsys):
    pytest.importorskip("rich")
    from meridianatc.cli import main

    args = ["xplane-plugin", "install", "--to", str(root), "--from", str(source)]
    assert main(args + ["--check"]) == 0
    assert not xp.plugin_directory(root).exists()
    assert main(args) == 0
    assert xp.installed(root)["installed"]
    assert main(["xplane-plugin", "uninstall", "--to", str(root)]) == 0
    assert not xp.installed(root)["installed"]
    assert main(["xplane-plugin", "install", "--to", str(tmp_path)]) == 1
    monkeypatch.setattr(xp, "install_roots", lambda path=None: [])
    assert main(["xplane-plugin", "install"]) == 1
    assert "No X-Plane 12 installs found" in capsys.readouterr().out


def test_installer_payload_and_opt_out(tmp_path, source, root):
    archive = tmp_path / "release.zip"
    with zipfile.ZipFile(archive, "w") as package:
        package.writestr("MeridianATC/MeridianATC.exe", b"app")
        for item in source.rglob("*"):
            if item.is_file():
                package.write(item, "MeridianATC/_internal/xplane-plugin/MeridianATC/" + item.relative_to(source).as_posix())
    payload = core.Payload(archive)
    target = tmp_path / "installed app"
    try:
        options = core.Options(toolbar_panel=False, xplane_plugin=False)
        core.install(payload, target, options, register=False, xplanes=[root])
        assert not xp.installed(root)["installed"]
        options.xplane_plugin = True
        result = core.install(payload, target, options, register=False, xplanes=[root])
    finally:
        payload.close()
    assert result["xplane"] == [str(root)]
    assert result["xplane_failed"] == []
    assert xp.installed(root)["installed"]
    assert (target / core.XPLANE_LIST).read_text(encoding="utf-8-sig").splitlines() == [str(root)]
    assert "xplane-plugin uninstall --to $root" in core.uninstaller_script()


def test_installer_empty_and_missing_payload(tmp_path, root, source, monkeypatch):
    assert core.install_xplane_plugin(tmp_path, []) == {"xplane": [], "xplane_failed": []}
    assert core.install_xplane_plugin(tmp_path, [root])["xplane_failed"] == [str(root)]
    monkeypatch.setattr(xp, "payload", lambda **kwargs: source)
    monkeypatch.setattr(xp, "install", lambda *args: (_ for _ in ()).throw(PermissionError("locked")))
    assert core.install_xplane_plugin(tmp_path, [root])["xplane_failed"] == [str(root)]


def test_installer_catalogue_fallback():
    _, strings = app.catalogue("fr")
    assert strings["sim.xplane"] == "X-Plane 12"
    assert strings["sim.xplane_plugin"] == "X-Plane traffic plugin"


def test_health_install_status(root, source, monkeypatch):
    config = Config()
    monkeypatch.setattr(xp, "install_roots", lambda *args: [root])
    # installed() validates explicit roots through the same function.
    assert health._xplane_installs(config).status == "warn"
    xp.install(root, source)
    row = health._xplane_installs(config)
    assert row.status == "ok" and "0.1.0" in row.detail and str(root) in row.detail
    monkeypatch.setattr(xp, "install_roots", lambda *args: [])
    assert health._xplane_installs(config).status == "warn"


def test_health_udp():
    peer = FakeXPlane()
    config = Config()
    config.sim.simulator = "xplane"
    config.sim.xplane_port = peer.port
    try:
        row = health._xplane_udp(config)
        assert row.status == "ok" and "RREF answered" in row.detail
        peer.respond = False
        assert health._xplane_udp(config).status == "warn"
        config.sim.simulator = "msfs"
        assert health._xplane_udp(config).status == "off"
    finally:
        peer.close()


def test_read_only_plugin_probe_and_health(monkeypatch):
    monkeypatch.setattr(xp, "plugin_ports", lambda: [])
    peer = FakePlugin()
    config = Config()
    config.sim.simulator = "xplane"
    config.traffic.xplane_plugin_port = peer.port
    try:
        row = health._xplane_plugin(config)
        assert row.status == "ok" and "0.1.0" in row.detail and "100 CSL models" in row.detail
        peer.models = 0
        assert health._xplane_plugin(config).status == "warn"
        assert peer.messages == [{"v": 1, "t": "hello", "probe": True}] * 2
        config.sim.simulator = "msfs"
        assert health._xplane_plugin(config).status == "off"
    finally:
        peer.close()


@pytest.mark.parametrize("overrides", [{"v": True}, {"plugin": "other"}, {"csl_models": -1}, {"version": 123}])
def test_bad_plugin_status_ignored(overrides, monkeypatch):
    peer = FakePlugin()
    original = peer.status
    monkeypatch.setattr(peer, "status", lambda: original(**overrides))
    try:
        assert xp.probe(port=peer.port, timeout=.05) is None
    finally:
        peer.close()


def test_no_plugin_and_remote_host(monkeypatch):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as quiet:
        quiet.bind(("127.0.0.1", 0))
        assert xp.probe(port=quiet.getsockname()[1], timeout=.01) is None
    with pytest.raises(ValueError, match="localhost"):
        xp.probe("192.0.2.1")
    monkeypatch.setattr(xp, "probe", lambda *args: None)
    config = Config()
    config.sim.simulator = "xplane"
    assert health._xplane_plugin(config).status == "warn"
