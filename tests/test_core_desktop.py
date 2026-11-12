from __future__ import annotations

import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


@pytest.fixture
def desktop():
    pytest.importorskip("scipy")
    pytest.importorskip("fastapi")
    from meridianatc import desktop

    return desktop


def test_local_api_closes_failed_toolbar_and_unstarted_listener(desktop, monkeypatch):
    primary, toolbar = Mock(), Mock()
    primary.getsockname.return_value = ("127.0.0.1", 12345)
    toolbar.bind.side_effect = OSError("already bound")
    monkeypatch.setattr(desktop.socket, "socket", Mock(side_effect=[primary, toolbar]))
    api = desktop.LocalApi(None, Mock(), toolbar_port=12346)
    toolbar.close.assert_called_once()
    assert api.toolbar_socket is None
    api.stop()
    primary.close.assert_called_once()


def test_local_api_closes_its_socket_if_primary_bind_fails(desktop, monkeypatch):
    listener = Mock()
    listener.bind.side_effect = OSError("cannot bind")
    monkeypatch.setattr(desktop.socket, "socket", lambda *args: listener)
    with pytest.raises(OSError, match="cannot bind"):
        desktop.LocalApi(None, Mock())
    listener.close.assert_called_once()


def test_local_api_closes_listeners_if_worker_cannot_start(desktop, monkeypatch):
    monkeypatch.setitem(sys.modules, "uvicorn", SimpleNamespace(Config=Mock(), Server=Mock()))
    listener = Mock()
    listener.getsockname.return_value = ("127.0.0.1", 12345)
    monkeypatch.setattr(desktop.socket, "socket", lambda *args: listener)
    monkeypatch.setattr(desktop, "create_app", Mock())
    worker = Mock()
    worker.start.side_effect = RuntimeError("thread limit")
    monkeypatch.setattr(desktop.threading, "Thread", Mock(return_value=worker))
    api = desktop.LocalApi(Mock(), Mock(), toolbar_port=0)
    with pytest.raises(RuntimeError, match="thread limit"):
        api.start()
    listener.close.assert_called_once()
    worker.join.assert_not_called()


class Closing:
    def __init__(self):
        self.handlers = []

    def __iadd__(self, handler):
        self.handlers.append(handler)
        return self

    def fire(self):
        for handler in self.handlers:
            handler()


@pytest.mark.parametrize("scenario", ["normal", "close_during_warm", "window_failure", "api_failure"])
def test_window_lifecycle_is_cleaned_once_and_never_starts_after_close(desktop, monkeypatch, scenario):
    engine, api = Mock(), Mock()
    api.wait_until_ready.return_value = True
    closing = Closing()
    window = SimpleNamespace(events=SimpleNamespace(closing=closing))
    webview = Mock()
    webview.create_window.return_value = window
    webview.start.side_effect = lambda callback, **kwargs: callback()
    monkeypatch.setitem(sys.modules, "webview", webview)
    monkeypatch.setattr(desktop, "available", lambda: (True, ""))
    monkeypatch.setattr(desktop, "Engine", lambda *args: engine)
    monkeypatch.setattr(desktop, "NavDB", Mock())
    monkeypatch.setattr(desktop, "LocalApi", lambda *args, **kwargs: api)
    monkeypatch.setattr(desktop, "_icon", lambda *args: {})
    if scenario == "close_during_warm":
        engine.warm.side_effect = closing.fire
    elif scenario == "window_failure":
        webview.create_window.side_effect = RuntimeError("window failed")
    elif scenario == "api_failure":
        api.start.side_effect = RuntimeError("API failed")
    ready = Mock()
    if scenario.endswith("failure"):
        with pytest.raises(RuntimeError):
            desktop.run(desktop.Config(), on_ready=ready)
    else:
        assert desktop.run(desktop.Config(), on_ready=ready) == 0
    engine.stop.assert_called_once()
    api.stop.assert_called_once()
    if scenario == "normal":
        engine.start.assert_called_once()
        ready.assert_called_once()
    else:
        engine.start.assert_not_called()
        ready.assert_not_called()
