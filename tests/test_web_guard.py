"""The panel answers its own pages only -- a stranger's tab cannot fly it.

The API carries no login and the panel binds a fixed port, so what stands
between the aeroplane and every web page the pilot opens is the Origin and
Sec-Fetch-Site the browser stamps a cross-site call with. These drive the
_OwnMachineOnly middleware as ASGI messages, no TestClient needed.
"""

from __future__ import annotations

import asyncio
import inspect

import pytest

pytest.importorskip("fastapi")

from meridianatc import config as cfg
from meridianatc.web import server


async def _run(headers, kind="http"):
    """Feed the middleware one scope; say whether the app ran and what answered."""
    reached = []
    sent = []

    async def inner(scope, receive, send):
        reached.append(True)
        if scope["type"] == "websocket":
            await send({"type": "websocket.accept"})
        else:
            await send({"type": "http.response.start",
                        "status": 200, "headers": []})
            await send({"type": "http.response.body", "body": b"ok"})

    async def receive():
        return {"type": "http.request", "body": b"{}"}

    async def send(message):
        sent.append(message)

    app = server._OwnMachineOnly(inner)
    scope = {"type": kind, "method": "POST", "path": "/api/say",
             "headers": headers}
    await app(scope, receive, send)
    return bool(reached), sent


def _headers(*pairs):
    return [(k.lower().encode(), v.encode()) for k, v in pairs]


def _status(sent):
    for message in sent:
        if message["type"] == "http.response.start":
            return message["status"]
    return None


def test_origin_ok_accepts_own_pages():
    for origin in ("http://127.0.0.1:8787", "http://localhost:8787",
                   "http://[::1]:8787", "https://127.0.0.1:8787",
                   "coui://html_ui/InGamePanels/WilcoATC", "null"):
        assert server._origin_ok(origin), origin


def test_origin_ok_refuses_strangers():
    for origin in ("https://evil.example.com",
                   "http://127.0.0.1.evil.example.com",
                   "javascript://x", "ftp://127.0.0.1",
                   "not a url at all"):
        assert not server._origin_ok(origin), origin


@pytest.mark.parametrize("origin", [
    "https://evil.example.com",
    "http://127.0.0.1.evil.example.com",
])
def test_post_from_another_page_is_refused(origin):
    reached, sent = asyncio.run(_run(_headers(("Origin", origin))))
    assert not reached
    assert _status(sent) == 403


@pytest.mark.parametrize("origin", [
    "http://127.0.0.1:8787", "http://localhost:8787", "coui://panel", "null",
])
def test_post_from_own_pages_passes(origin):
    reached, _ = asyncio.run(_run(_headers(("Origin", origin))))
    assert reached


def test_post_without_an_origin_passes():
    # The simulator, the console and the pilot's own scripts send none.
    reached, _ = asyncio.run(_run([]))
    assert reached


def test_cross_site_fetch_is_refused_without_an_origin():
    reached, sent = asyncio.run(_run(
        _headers(("Sec-Fetch-Site", "cross-site"))))
    assert not reached
    assert _status(sent) == 403


def test_cross_site_fetch_is_refused_even_opaque():
    # A sandboxed iframe sends Origin: null but still stamps cross-site.
    reached, sent = asyncio.run(_run(
        _headers(("Origin", "null"), ("Sec-Fetch-Site", "cross-site"))))
    assert not reached
    assert _status(sent) == 403


def test_same_site_fetch_passes():
    reached, _ = asyncio.run(_run(
        _headers(("Origin", "http://127.0.0.1:8787"),
                 ("Sec-Fetch-Site", "same-origin"))))
    assert reached


def test_preflight_from_another_page_is_refused():
    reached, sent = asyncio.run(_run(
        _headers(("Origin", "https://evil.example.com"),
                 ("Access-Control-Request-Method", "POST")),
        kind="http"))
    assert not reached
    assert _status(sent) == 403


def test_websocket_from_another_page_is_closed():
    reached, sent = asyncio.run(_run(
        _headers(("Origin", "https://evil.example.com")), kind="websocket"))
    assert not reached
    assert sent == [{"type": "websocket.close", "code": 1008}]


def test_websocket_from_the_panel_passes():
    reached, sent = asyncio.run(_run(
        _headers(("Origin", "http://127.0.0.1:8787")), kind="websocket"))
    assert reached
    assert sent == [{"type": "websocket.accept"}]


def test_websocket_without_an_origin_passes():
    reached, _ = asyncio.run(_run([], kind="websocket"))
    assert reached


def test_both_apps_wrap_the_guard():
    assert "_OwnMachineOnly" in inspect.getsource(server.create_app)
    assert "_OwnMachineOnly" in inspect.getsource(server.create_setup_app)


def test_setup_app_has_the_guard_outermost():
    app = server.create_setup_app(cfg.Config(), server.Broadcaster())
    names = [entry.cls for entry in app.user_middleware]
    assert names[0] is server._OwnMachineOnly
