from __future__ import annotations

import json
import logging
import sys

from meridianatc import logs, paths
from meridianatc.config import Config


def test_configured_sinks_redact_interpolated_messages_exceptions_and_fields(monkeypatch, tmp_path, capsys):
    root = logging.getLogger()
    previous, level = list(root.handlers), root.level
    root.handlers.clear()
    settings = Config()
    settings.logging.directory = "relative-logs"
    settings.logging.console_level = "INFO"
    monkeypatch.setattr(paths, "data_root", lambda: tmp_path)
    monkeypatch.setattr(logs, "log_dir", lambda: tmp_path / "unused")
    monkeypatch.setattr(logs, "_install_hooks", lambda: None)
    monkeypatch.setattr(logs, "_configured", False)
    monkeypatch.setattr(logs, "_directory", None)
    monkeypatch.setattr(logs, "RING", logs.RingHandler())
    monkeypatch.setattr(logs, "RADIO", logs.RadioLog())
    monkeypatch.setattr(logs, "CREW", logs.RadioLog())
    apron_levels = {name: logging.getLogger(name).level for name in logs.APRON_LOGGERS}
    try:
        folder = logs.configure(settings)
        assert folder == tmp_path / "relative-logs"
        logger = logging.getLogger("meridianatc.audit.child")
        try:
            raise RuntimeError("password=synthetic-error-value")
        except RuntimeError:
            logger.exception("request %s", "api_key=synthetic-message-value",
                             extra={"api_key": "synthetic-extra-value",
                                    "request": {"password": "synthetic-nested-value"}})
        logs.RADIO.write("parse", api_key="synthetic-radio-value")
        console = capsys.readouterr().err
        plain = (folder / "meridianatc.log").read_text(encoding="utf-8")
        structured = (folder / "meridianatc.jsonl").read_text(encoding="utf-8")
        ring = json.dumps(logs.RING.tail())
        radio = (folder / "radio.jsonl").read_text(encoding="utf-8")
        for output in (console, plain, structured, ring, radio):
            assert "synthetic-" not in output
            assert "redacted" in output.lower()
        assert "request" in plain and "RuntimeError" in plain
        assert "Logging error" not in console
    finally:
        for handler in list(root.handlers):
            root.removeHandler(handler)
            handler.close()
        root.handlers.extend(previous)
        root.setLevel(level)
        for name, old_level in apron_levels.items():
            logging.getLogger(name).setLevel(old_level)


def test_empty_and_invalid_log_arguments_do_not_raise_or_expose_values():
    record = logging.LogRecord("audit", logging.INFO, "", 0, "", (), None)
    assert logs.SessionFilter().filter(record)
    assert record.getMessage() == ""
    record = logging.LogRecord("audit", logging.INFO, "", 0, "no placeholders", ("synthetic-value",), None)
    assert logs.SessionFilter().filter(record)
    assert "invalid log arguments" in record.getMessage()
    assert not record.args


def test_structured_traceback_and_nested_fields_are_scrubbed():
    try:
        raise ValueError('api_key="synthetic-value"')
    except ValueError:
        record = logging.LogRecord("audit", logging.ERROR, "", 0, "failed", (), sys.exc_info())
    record.request = [{"Authorization": "synthetic-header", "token_count": 12}]
    payload = json.loads(logs.JsonlFormatter().format(record))
    assert "synthetic-" not in json.dumps(payload)
    assert payload["fields"]["request"][0]["token_count"] == 12
