from __future__ import annotations

import json
import threading
from pathlib import Path
from unittest.mock import Mock

import pytest

from meridianatc import health, install, msfs_panel
from meridianatc.config import Config
from meridianatc.history import History
from meridianatc.sim.state import AircraftState


def test_history_session_selection_is_serialized_with_recording(tmp_path):
    history = History(tmp_path / "history.db")
    first = history.open_session(callsign="FIRST")
    second = history.open_session(callsign="SECOND")
    history._session_id = first
    entered = threading.Event()

    class ObservedLock:
        def __init__(self):
            self.lock = threading.Lock()

        def __enter__(self):
            if threading.current_thread().name == "writer":
                entered.set()
            self.lock.acquire()

        def __exit__(self, *args):
            self.lock.release()

    history._lock = ObservedLock()
    worker = threading.Thread(target=history.record, args=("pilot", "current flight"), name="writer")
    try:
        with history._lock:
            worker.start()
            assert entered.wait(3)
            history._session_id = second
        worker.join(3)
        assert not worker.is_alive()
        assert not history.transcript(first)
        assert history.transcript(second)[0].text == "current flight"
    finally:
        worker.join(3)
        history.close()


@pytest.mark.parametrize("lookup_fails", [False, True])
def test_doctor_closes_navigation_database_and_simulator(monkeypatch, lookup_fails):
    from meridianatc.navdata import db as databases
    from meridianatc.sim import source as sources

    source = Mock()
    source.read.return_value = AircraftState(source="xplane", connected=True, latitude=40, longitude=-70)
    monkeypatch.setattr(sources, "open_source", lambda **kwargs: source)
    database = Mock()
    database.home_airport.return_value = None
    database.nearest_airport.return_value = None
    monkeypatch.setattr(databases, "NavDB", lambda: database)
    if lookup_fails:
        database.home_airport.side_effect = RuntimeError("broken database")
        with pytest.raises(RuntimeError, match="broken database"):
            health._simulator(Config())
    else:
        assert health._simulator(Config()).status == "ok"
    database.close.assert_called_once()
    source.close.assert_called_once()


def test_doctor_does_not_report_a_disconnected_source_as_healthy(monkeypatch):
    from meridianatc.sim import source as sources

    source = Mock()
    source.read.return_value = AircraftState(source="xplane", connected=False)
    monkeypatch.setattr(sources, "open_source", lambda **kwargs: source)
    assert health._simulator(Config()).status == "warn"
    source.close.assert_called_once()
    assert health._com1(Config()).status == "warn"
    assert source.close.call_count == 2


def test_installer_empty_and_unknown_selections_do_not_start_downloads(monkeypatch):
    installer = install.Installer()
    monkeypatch.setattr(installer, "_publish", Mock())
    monkeypatch.setattr(installer, "_run", Mock())
    assert not installer.start([])
    assert not installer.start(["unknown"])
    assert installer.start(["voices", "voices"])
    installer.wait(3)
    assert installer._run.call_args.args[0] == ["voices"]


def test_installer_thread_start_failure_does_not_leave_setup_running(monkeypatch):
    installer = install.Installer()
    monkeypatch.setattr(installer, "_publish", Mock())
    thread = Mock()
    thread.start.side_effect = RuntimeError("thread limit")
    monkeypatch.setattr(install.threading, "Thread", Mock(return_value=thread))
    assert not installer.start(["voices"])
    assert not installer._state.running
    assert installer._state.steps[0].state == "failed"
    assert installer._state.steps[0].error == "thread limit"
    installer.wait()


@pytest.mark.parametrize("query_fails", [False, True])
def test_doctor_closes_database_used_for_counts(monkeypatch, query_fails):
    from meridianatc.navdata import db as databases

    database = Mock()
    database._conn.execute.return_value.fetchone.side_effect = [(3,), (2,)]
    if query_fails:
        database._conn.execute.side_effect = RuntimeError("query failed")
    monkeypatch.setattr(databases, "NavDB", lambda: database)
    check = health._navdata(Config())
    assert check.status == ("error" if query_fails else "ok")
    database.close.assert_called_once()


def _panel_source(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    (source / "manifest.json").write_text("{}", encoding="utf-8")
    (source / "panel.html").write_text("new", encoding="utf-8")
    return source


def test_panel_install_replaces_old_files_and_generates_layout(tmp_path):
    source = _panel_source(tmp_path)
    community = tmp_path / "Community"
    target = msfs_panel.install(source, community)
    (target / "obsolete.html").write_text("old", encoding="utf-8")
    assert msfs_panel.install(source, community) == target
    assert not (target / "obsolete.html").exists()
    layout = json.loads((target / "layout.json").read_text(encoding="utf-8"))
    assert [row["path"] for row in layout["content"]] == ["panel.html"]
    assert msfs_panel.uninstall(community)
    assert not msfs_panel.uninstall(community)


@pytest.mark.parametrize("failure", ["missing", "self", "copy", "rename"])
def test_panel_update_failure_preserves_the_installed_package(monkeypatch, tmp_path, failure):
    source = _panel_source(tmp_path)
    community = tmp_path / "Community"
    target = msfs_panel.install(source, community)
    (target / "panel.html").write_text("keep", encoding="utf-8")
    if failure == "missing":
        source = tmp_path / "missing"
    elif failure == "self":
        source = target
    elif failure == "copy":
        monkeypatch.setattr(msfs_panel.shutil, "copytree", Mock(side_effect=OSError("copy failed")))
    else:
        rename = Path.rename

        def failing_rename(path, destination):
            if path.parent.name.startswith(".meridianatc-panel-"):
                raise OSError("swap failed")
            return rename(path, destination)

        monkeypatch.setattr(Path, "rename", failing_rename)
    with pytest.raises((ValueError, OSError)):
        msfs_panel.install(source, community)
    assert (target / "panel.html").read_text(encoding="utf-8") == "keep"


def test_panel_uninstall_does_not_claim_success_on_permission_failure(monkeypatch, tmp_path):
    target = msfs_panel.install(_panel_source(tmp_path), tmp_path / "Community")
    monkeypatch.setattr(msfs_panel.shutil, "rmtree", Mock(side_effect=PermissionError("locked")))
    with pytest.raises(PermissionError, match="locked"):
        msfs_panel.uninstall(target.parent)


@pytest.mark.parametrize("read_failure", [True, False])
def test_failed_candidate_read_never_replaces_or_closes_the_current_source(read_failure):
    pytest.importorskip("scipy")
    from meridianatc.engine import Engine

    engine = Engine.__new__(Engine)
    previous = engine.source = Mock()
    original = engine.state = AircraftState(source="simconnect")
    candidate = Mock()
    if read_failure:
        candidate.read.side_effect = RuntimeError("state read failed")
    else:
        candidate.read.return_value = AircraftState(source="xplane", connected=False)
    assert not engine._adopt_simulator(candidate)
    assert engine.source is previous and engine.state is original
    previous.close.assert_not_called()
    candidate.close.assert_called_once()


def test_engine_shutdown_releases_controls_and_closes_remaining_resources_after_errors(caplog):
    pytest.importorskip("scipy")
    from meridianatc.engine import Engine

    engine = Engine.__new__(Engine)
    engine._shutdown_lock, engine._sim_lock = threading.Lock(), threading.RLock()
    engine._shutdown_done, engine._sim_attempt = False, None
    engine._stop = threading.Event()
    engine._situation_thread = Mock()
    engine._situation_thread.is_alive.return_value = True
    for name in ("live_feed", "cabin", "_ptt", "autonomous_flight", "player", "simvehicles",
                 "simdoors", "simsystems", "servicepoints", "simtraffic", "simwatch",
                 "facilities", "gsx", "source", "history", "navdb"):
        setattr(engine, name, Mock())
    order = []
    engine.autonomous_flight.disengage.side_effect = lambda: order.append("controls")
    engine.source.close.side_effect = lambda: order.append("source")
    engine.simvehicles.detach.side_effect = RuntimeError("detach failed")
    engine.history.close_session.side_effect = RuntimeError("history failed")
    engine.stop()
    engine.stop()
    assert order == ["controls", "source"]
    engine.gsx.bridge.close.assert_called_once()
    engine.simtraffic.detach.assert_called_once()
    engine.history.close.assert_called_once()
    engine.navdb.close.assert_called_once()
    assert "could not close simvehicles" in caplog.text
    assert "could not close history session" in caplog.text
    assert "still finishing a tick during shutdown" in caplog.text
    engine._situation_thread.join.assert_called_once_with(timeout=2.0)


def test_situation_loop_logs_failures_once_then_reports_recovery(caplog):
    pytest.importorskip("scipy")
    from meridianatc.engine import Engine

    engine = Engine.__new__(Engine)
    engine.config = Config()
    engine._stop = Mock()
    engine._stop.is_set.side_effect = [False, False, False, True]
    engine._tick = Mock(side_effect=[OSError("sim disconnected"), OSError("sim disconnected"), None])
    with caplog.at_level("INFO", logger="meridianatc.engine"):
        engine._situation_loop()
    assert engine._tick.call_count == 3
    assert caplog.text.count("situation update failed;") == 1
    assert "situation updates recovered" in caplog.text
