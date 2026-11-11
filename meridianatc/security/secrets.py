"""API keys stored under Windows DPAPI, tied to the logged-in account.

A port of StormATC's ``DpapiSecretStore``. DPAPI ties the ciphertext to the
Windows user, so the file is unreadable by other accounts and useless if
copied elsewhere -- the right trade for a desktop program: no master
password, no key material in the repository, no credential service to run.

The rules the store keeps:

* a key never appears in ``config.yaml``, in SQLite, in a log, in an
  exception message, or in a diagnostics export;
* the only way a key leaves the store is through :meth:`SecretStore.get`,
  straight into an Authorization header;
* a store that cannot be decrypted -- copied from another machine, or
  corrupted -- is treated as empty rather than failing the program.

On a platform without DPAPI the same file is written with owner-only
permissions and a warning is logged once; the contract is unchanged.
"""

from __future__ import annotations

import contextlib
import ctypes
import ctypes.wintypes as w
import json
import logging
import os
import threading
from pathlib import Path

log = logging.getLogger(__name__)

#: Additional entropy mixed into the DPAPI protection. Not a secret -- it is
#: compiled into the program -- but it scopes the ciphertext to MeridianATC
#: so another application running as the same user cannot trivially decrypt.
ENTROPY = b"MeridianATC.SecretStore.v1"


# --------------------------------------------------------------------------
# DPAPI
# --------------------------------------------------------------------------


class _Blob(ctypes.Structure):
    _fields_ = [("cbData", w.DWORD), ("pbData", ctypes.POINTER(ctypes.c_char))]


def _dpapi_protect(plaintext: bytes) -> bytes:
    """CryptProtectData under the current user. Raises OSError on failure."""
    in_buffer = ctypes.create_string_buffer(plaintext, len(plaintext))
    in_blob = _Blob(len(plaintext), ctypes.cast(in_buffer,
                                                ctypes.POINTER(ctypes.c_char)))
    entropy_buffer = ctypes.create_string_buffer(ENTROPY, len(ENTROPY))
    entropy_blob = _Blob(len(ENTROPY), ctypes.cast(
        entropy_buffer, ctypes.POINTER(ctypes.c_char)))
    out_blob = _Blob()
    if not ctypes.windll.crypt32.CryptProtectData(
            ctypes.byref(in_blob), None, ctypes.byref(entropy_blob),
            None, None, 0, ctypes.byref(out_blob)):
        raise ctypes.WinError()
    try:
        return ctypes.string_at(out_blob.pbData, out_blob.cbData)
    finally:
        ctypes.windll.kernel32.LocalFree(out_blob.pbData)


def _dpapi_unprotect(protected: bytes) -> bytes:
    """CryptUnprotectData under the current user. Raises OSError on failure."""
    in_buffer = ctypes.create_string_buffer(protected, len(protected))
    in_blob = _Blob(len(protected), ctypes.cast(in_buffer,
                                                ctypes.POINTER(ctypes.c_char)))
    entropy_buffer = ctypes.create_string_buffer(ENTROPY, len(ENTROPY))
    entropy_blob = _Blob(len(ENTROPY), ctypes.cast(
        entropy_buffer, ctypes.POINTER(ctypes.c_char)))
    out_blob = _Blob()
    if not ctypes.windll.crypt32.CryptUnprotectData(
            ctypes.byref(in_blob), None, ctypes.byref(entropy_blob),
            None, None, 0, ctypes.byref(out_blob)):
        raise ctypes.WinError()
    try:
        return ctypes.string_at(out_blob.pbData, out_blob.cbData)
    finally:
        ctypes.windll.kernel32.LocalFree(out_blob.pbData)


def dpapi_available() -> bool:
    """Whether CryptProtectData exists on this platform."""
    return os.name == "nt" and hasattr(
        getattr(ctypes, "windll", None) or (), "crypt32")


def mask(secret: str | None) -> str:
    """A secret for display: first four and last four, the middle elided.

    Short values are masked entirely -- showing six of eight characters
    would leak most of the secret.
    """
    if not secret:
        return ""
    if len(secret) <= 12:
        return "•" * min(len(secret), 16)
    return f"{secret[:4]}{'•' * 12}{secret[-4:]}"


class SecretStore:
    """A dictionary of named secrets, DPAPI-protected, on disk.

    ``secrets.dat`` under the data root. Writes go through a temporary file
    so an interrupted write cannot leave a truncated store that would
    silently lose every key.
    """

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self._lock = threading.Lock()
        self._protected = dpapi_available()
        if not self._protected:
            log.warning("DPAPI is unavailable on this platform; secrets are "
                        "stored in an owner-only file instead")

    # -- the contract ---------------------------------------------------

    def get(self, key: str) -> str | None:
        return self._load().get(key)

    def set(self, key: str, value: str) -> None:
        if not key or not key.strip():
            raise ValueError("a secret needs a name")
        if value is None:
            raise ValueError("a secret needs a value")
        with self._lock:
            secrets = self._load()
            secrets[key] = value
            self._save(secrets)
        # The name is safe to log; the value must never reach a log sink.
        log.info("stored secret %r", key)

    def remove(self, key: str) -> bool:
        with self._lock:
            secrets = self._load()
            if key not in secrets:
                return False
            del secrets[key]
            self._save(secrets)
        log.info("removed secret %r", key)
        return True

    def has(self, key: str) -> bool:
        return key in self._load()

    def keys(self) -> list[str]:
        """The names held, never the values."""
        return sorted(self._load())

    # -- the file ---------------------------------------------------------

    def _load(self) -> dict[str, str]:
        if not self.path.is_file():
            return {}
        try:
            raw = self.path.read_bytes()
            if self._protected:
                raw = _dpapi_unprotect(raw)
            secrets = json.loads(raw.decode("utf-8"))
            return secrets if isinstance(secrets, dict) else {}
        except Exception:
            # The exception's own message is not logged: it can echo input.
            log.warning("the secret store could not be read; treating it as "
                        "empty, and any saved keys will have to be re-entered")
            return {}

    def _save(self, secrets: dict[str, str]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        raw = json.dumps(secrets, separators=(",", ":")).encode("utf-8")
        if self._protected:
            raw = _dpapi_protect(raw)
        temp = self.path.with_suffix(self.path.suffix + ".tmp")
        temp.write_bytes(raw)
        if not self._protected:
            with contextlib.suppress(OSError):
                os.chmod(temp, 0o600)
        os.replace(temp, self.path)


class InMemorySecretStore:
    """The same contract, holding nothing. For tests and offline runs."""

    def __init__(self):
        self._secrets: dict[str, str] = {}

    def get(self, key: str) -> str | None:
        return self._secrets.get(key)

    def set(self, key: str, value: str) -> None:
        self._secrets[key] = value

    def remove(self, key: str) -> bool:
        return self._secrets.pop(key, None) is not None

    def has(self, key: str) -> bool:
        return key in self._secrets

    def keys(self) -> list[str]:
        return sorted(self._secrets)


_default: SecretStore | None = None
_default_lock = threading.Lock()


def default_store() -> SecretStore:
    """The store the application uses: ``secrets.dat`` at the data root."""
    global _default
    with _default_lock:
        if _default is None:
            from ..paths import data_root
            _default = SecretStore(data_root() / "secrets.dat")
        return _default


def resolve_secret(config, path: str, env: str = "") -> str:
    """The one order every credential resolves in.

    A key saved through the Settings screen lives in the DPAPI store and
    wins, because it is the most recent deliberate act; an environment
    variable next, for headless runs; and the ``config.yaml`` value last,
    kept working for whoever wrote one there by hand.
    """
    try:
        stored = default_store().get(path)
    except Exception:
        stored = None
    if stored:
        return stored
    if env:
        value = os.environ.get(env, "")
        if value:
            return value
    from ..config import get_path
    try:
        value = get_path(config, path)
    except Exception:
        value = ""
    return str(value) if value else ""
