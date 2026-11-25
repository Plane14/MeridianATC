from __future__ import annotations

import hashlib
import io
import threading
import zipfile
from unittest.mock import Mock

from meridianatc import update


def test_update_check_is_single_flight_and_does_not_replace_pending_release(monkeypatch):
    entered, release_check = threading.Event(), threading.Event()
    # Newer than whatever this checkout calls itself, so the check still
    # finds an update after the next version bump.
    newer = f"{int(update.__version__.split('.')[0]) + 1}.0"
    release = update.Release(newer, f"v{newer}", asset="app.zip",
                             asset_url="https://example.invalid/app.zip")

    def fetch():
        entered.set()
        assert release_check.wait(3)
        return release

    monkeypatch.setattr(update, "why_not_install", lambda: "")
    updater = update.Updater(fetch=Mock(side_effect=fetch))
    worker = threading.Thread(target=updater.check)
    worker.start()
    try:
        assert entered.wait(3)
        assert updater.check()["checking"]
        assert not updater.download()
    finally:
        release_check.set()
        worker.join(3)
    assert not worker.is_alive()
    updater._fetch.assert_called_once()
    assert updater.snapshot()["can_install"]
    updater._set(phase="ready")
    updater.check()
    updater._fetch.assert_called_once()
    assert updater._release is release


def test_download_claims_phase_before_notifying_listeners(monkeypatch, tmp_path):
    entered, finish = threading.Event(), threading.Event()
    release = update.Release("3.0", "v3.0", asset="app.zip")
    updater = update.Updater()
    updater._release = release
    updater.state.can_install = True
    calls, duplicate = [], []

    def download(chosen):
        calls.append(chosen)
        entered.set()
        assert finish.wait(3)

    def changed(state):
        if state["phase"] == "downloading":
            duplicate.append(updater.download())

    updater._download = download
    updater._on_change = changed
    assert updater.download()
    try:
        assert entered.wait(3)
        assert not updater.download()
        assert not updater.check()["checking"]
        assert calls == [release] and duplicate == [False]
    finally:
        finish.set()


def test_download_folder_failure_is_reported(monkeypatch):
    updater = update.Updater()
    updater.state.phase = "downloading"

    def inaccessible():
        raise PermissionError("updates folder is read-only")

    monkeypatch.setattr(update, "updates_dir", inaccessible)
    updater._download(update.Release("3.0", "v3.0"))
    assert updater.snapshot()["phase"] == "failed"
    assert "read-only" in updater.snapshot()["error"]


def test_download_thread_start_failure_clears_the_busy_state(monkeypatch):
    updater = update.Updater()
    updater._release = update.Release("3.0", "v3.0", asset="app.zip")
    updater.state.can_install = True
    thread = Mock()
    thread.start.side_effect = RuntimeError("thread limit")
    monkeypatch.setattr(update.threading, "Thread", Mock(return_value=thread))
    assert not updater.download()
    assert updater.snapshot()["phase"] == "failed"
    assert "thread limit" in updater.snapshot()["error"]


def test_download_extracts_the_claimed_release_and_cancels_cleanly(monkeypatch, tmp_path):
    data = io.BytesIO()
    with zipfile.ZipFile(data, "w") as archive:
        archive.writestr("MeridianATC/MeridianATC.exe", b"program")
        archive.writestr("MeridianATC/_internal/library", b"library")
    response = io.BytesIO(data.getvalue())
    response.headers = {}
    updater = update.Updater(opener=lambda *args, **kwargs: response)
    monkeypatch.setattr(update, "updates_dir", lambda: tmp_path)
    digest = hashlib.sha256(data.getvalue()).hexdigest()
    updater._download(update.Release("3.0", "v3.0", asset="app.zip",
                                     asset_url="https://example.invalid/app.zip",
                                     digest=f"sha256:{digest}"))
    assert updater.snapshot()["phase"] == "ready"
    assert (updater._staged / update.EXECUTABLE).read_bytes() == b"program"
    updater._cancel.set()
    updater._fetch_to = lambda release, path: path.write_bytes(b"partial")
    updater._download(update.Release("3.0", "v3.0", asset="cancel.zip"))
    assert updater.snapshot()["phase"] == "idle"
    assert not (tmp_path / "cancel.zip.part").exists()


def test_apply_launch_failure_can_be_retried_without_quitting(monkeypatch, tmp_path):
    monkeypatch.setattr(update, "updates_dir", lambda: tmp_path)
    launch = Mock(side_effect=OSError("cannot launch cmd.exe"))
    monkeypatch.setattr(update.subprocess, "Popen", launch)
    updater = update.Updater()
    updater._staged = tmp_path / "staged"
    updater.state.phase = "ready"
    quit_program = Mock()
    assert not updater.apply(quit_program)
    quit_program.assert_not_called()
    assert updater.snapshot()["phase"] == "ready"
    assert "cannot launch" in updater.snapshot()["error"]
    launch.side_effect = None
    assert updater.apply(quit_program)
    quit_program.assert_called_once()
    assert updater.snapshot()["phase"] == "applying"


def test_apply_script_never_copies_over_a_process_that_did_not_exit():
    script = update.apply_script(123)
    assert "if %tries% GEQ 120 goto still_running" in script
    timeout = script.split(":still_running\r\n", 1)[1]
    assert "robocopy" not in timeout and 'start ""' not in timeout
    assert "exit /b 1" in timeout


def test_update_script_carries_simulator_payload_directories():
    script = update.apply_script(1234)
    assert 'for %%D in (msfs-panel xplane-plugin)' in script
    assert r'robocopy "%SRC%\%%D" "%DST%\%%D" /E' in script
