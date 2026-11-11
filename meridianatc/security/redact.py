"""Last-line defence against credential-shaped text reaching a log or export.

A port of StormATC's ``SensitiveDataRedactor``, plus a ``logging.Filter``
that applies it to every record's message and arguments before a handler
writes them.
"""

from __future__ import annotations

import logging
import re

REPLACEMENT = "[REDACTED]"

_AUTH_HEADER = re.compile(
    r"\b(authorization\s*[:=]\s*(?:bearer|token)\s+)[^\s,;\"']+",
    re.IGNORECASE)
_CREDENTIAL_FIELD = re.compile(
    r"([\"']?(?:api[_-]?key|access[_-]?token|secret|password)[\"']?"
    r"\s*[:=]\s*[\"'])([^\"'\r\n]+)([\"'])",
    re.IGNORECASE)
_PROVIDER_KEY = re.compile(r"\b(?:sk-or-v1-|sk-)[A-Za-z0-9_-]{12,}\b")


def redact(value: str | None) -> str:
    """Mask authorization headers, api-key fields and known key prefixes."""
    if not value:
        return value or ""
    redacted = _AUTH_HEADER.sub(lambda m: m.group(1) + REPLACEMENT, value)
    redacted = _CREDENTIAL_FIELD.sub(
        lambda m: m.group(1) + REPLACEMENT + m.group(3), redacted)
    return _PROVIDER_KEY.sub(REPLACEMENT, redacted)


def _scrub(obj):
    if isinstance(obj, str):
        return redact(obj)
    if isinstance(obj, (tuple, list)):
        return type(obj)(_scrub(o) for o in obj)
    if isinstance(obj, dict):
        return {k: _scrub(v) for k, v in obj.items()}
    return obj


class RedactingFilter(logging.Filter):
    """Applies :func:`redact` to a record's message and arguments."""

    def filter(self, record: logging.LogRecord) -> bool:
        if record.args:
            record.args = _scrub(record.args)
            record.msg = _scrub(record.msg)
        else:
            record.msg = _scrub(record.msg)
        return True
