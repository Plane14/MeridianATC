from __future__ import annotations

import threading
from concurrent.futures import ThreadPoolExecutor

import pytest
import yaml

from meridianatc import config, paths, update


@pytest.mark.parametrize("content", ["- sim\n- voice\n", "false\n", "0\n", "sim: []\n", "sim: bad\n"])
def test_invalid_config_shapes_are_rejected(tmp_path, content):
    path = tmp_path / "config.yaml"
    path.write_text(content, encoding="utf-8")
    with pytest.raises(ValueError, match="mapping"):
        config.Config.load(path)


@pytest.mark.parametrize("content, setting", [
    ("sim:\n  poll_interval_s: null\n", "sim.poll_interval_s"),
    ("sim:\n  poll_interval_s: -.1\n", "sim.poll_interval_s"),
    ("sim:\n  xplane_port: 70000\n", "sim.xplane_port"),
    ("sim:\n  simulator: not-a-sim\n", "sim.simulator"),
    ("sim:\n  xplane_host: {}\n", "sim.xplane_host"),
    ("speech:\n  languages: [42]\n", "speech.languages"),
])
def test_file_settings_are_validated_before_the_engine_uses_them(tmp_path, content, setting):
    path = tmp_path / "config.yaml"
    path.write_text(content, encoding="utf-8")
    with pytest.raises(ValueError, match=setting):
        config.Config.load(path)


def test_loaded_settings_are_normalized_and_invalid_local_overrides_are_rejected(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text('sim:\n  simulator: XPLANE\n  xplane_port: "49000"\n  enabled: "false"\n', encoding="utf-8")
    settings = config.Config.load(path)
    assert settings.sim.simulator == "xplane" and settings.sim.xplane_port == 49000
    assert settings.sim.enabled is False
    path.with_name(config.LOCAL_NAME).write_text("sim:\n  xplane_port: 0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="sim.xplane_port"):
        config.Config.load(path)


def test_blank_sections_keep_defaults_and_do_not_mask_later_saves(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("sim:\ntraffic:\n  handling: false\n", encoding="utf-8")
    overlay = path.with_name(config.LOCAL_NAME)
    overlay.write_text("sim:\n", encoding="utf-8")
    settings = config.Config.load(path)
    assert settings.sim.enabled
    assert not settings.traffic.handling.enabled
    settings.sim.xplane_host = "192.0.2.1"
    settings.save()
    assert yaml.safe_load(path.read_text(encoding="utf-8"))["sim"]["xplane_host"] == "192.0.2.1"


def test_saving_a_copy_does_not_advance_the_original_files_baseline(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("# keep this\nflight:\n  callsign: OLD\n", encoding="utf-8")
    settings = config.Config.load(path)
    settings.flight.callsign = "NEW"
    settings.save(tmp_path / "export" / "copy.yaml")
    settings.save()
    assert config.Config.load(path).flight.callsign == "NEW"
    assert path.read_text(encoding="utf-8").startswith("# keep this")


def test_legacy_boolean_overlay_only_masks_the_enable_switch(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("{}\n", encoding="utf-8")
    path.with_name(config.LOCAL_NAME).write_text("traffic:\n  handling: false\n", encoding="utf-8")
    settings = config.Config.load(path)
    settings.traffic.handling.vehicles = False
    settings.save()
    saved = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert saved["traffic"]["handling"]["vehicles"] is False
    assert "enabled" not in saved["traffic"]["handling"]


def test_failed_atomic_save_keeps_the_previous_file_and_removes_temporary(monkeypatch, tmp_path):
    path = tmp_path / "config.yaml"
    original = "flight:\n  callsign: OLD\n"
    path.write_text(original, encoding="utf-8")
    settings = config.Config.load(path)
    settings.flight.callsign = "NEW"

    def fail(*args):
        raise PermissionError("replacement failed")

    with monkeypatch.context() as patch:
        patch.setattr(config.os, "replace", fail)
        with pytest.raises(PermissionError, match="replacement failed"):
            settings.save()
    assert path.read_text(encoding="utf-8") == original
    assert not list(tmp_path.glob("*.tmp"))
    settings.save()
    assert config.Config.load(path).flight.callsign == "NEW"


@pytest.mark.parametrize("content", ["broken: [\n", "- invalid\n"])
def test_repairing_invalid_yaml_writes_the_whole_config(tmp_path, content):
    path = tmp_path / "config.yaml"
    path.write_text("flight:\n  callsign: KEEP\n", encoding="utf-8")
    settings = config.Config.load(path)
    path.write_text(content, encoding="utf-8")
    settings.sim.simulator = "xplane"
    settings.save()
    loaded = config.Config.load(path)
    assert loaded.flight.callsign == "KEEP" and loaded.sim.simulator == "xplane"


def test_writability_checks_never_delete_existing_probe_names(tmp_path):
    for name in (".write-test", ".update-test"):
        (tmp_path / name).write_text("user data", encoding="utf-8")
    with ThreadPoolExecutor(max_workers=4) as pool:
        assert all(pool.map(paths._writable, [tmp_path] * 8))
    assert update._writable(tmp_path)
    for name in (".write-test", ".update-test"):
        assert (tmp_path / name).read_text(encoding="utf-8") == "user data"


def test_concurrent_config_saves_merge_distinct_changes(monkeypatch, tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("{}\n", encoding="utf-8")
    first, second = config.Config.load(path), config.Config.load(path)
    first.flight.callsign = "FIRST"
    second.sim.xplane_host = "192.0.2.2"
    staged, release, second_started, second_replaced = (threading.Event() for _ in range(4))
    replace = config.os.replace

    def replace_after_waiting(source, destination):
        if threading.current_thread().name == "first-save":
            staged.set()
            assert release.wait(3)
        else:
            second_replaced.set()
        return replace(source, destination)

    def save_second():
        second_started.set()
        second.save()

    monkeypatch.setattr(config.os, "replace", replace_after_waiting)
    one = threading.Thread(target=first.save, name="first-save")
    two = threading.Thread(target=save_second)
    one.start()
    try:
        assert staged.wait(3)
        two.start()
        assert second_started.wait(3)
        assert not second_replaced.wait(.1)
    finally:
        release.set()
        one.join(3)
        if two.ident is not None:
            two.join(3)
    assert not one.is_alive() and not two.is_alive()
    loaded = config.Config.load(path)
    assert loaded.flight.callsign == "FIRST" and loaded.sim.xplane_host == "192.0.2.2"


@pytest.mark.parametrize("content", ["sim: [\n", "sim: []\n"])
def test_cli_reports_invalid_config_without_a_traceback(tmp_path, capsys, content):
    pytest.importorskip("rich")
    from meridianatc.cli import main

    path = tmp_path / "config.yaml"
    path.write_text(content, encoding="utf-8")
    assert main(["--config", str(path), "doctor"]) == 1
    output = capsys.readouterr()
    assert output.out and "Traceback" not in output.out + output.err
