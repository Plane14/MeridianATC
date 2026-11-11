"""Credential handling: storage, endpoint policy and log redaction.

Three small pieces, each a port of the StormATC feature of the same name:

* :mod:`secrets` -- API keys under Windows DPAPI, tied to the logged-in
  account, in one file under the data root. Never in ``config.yaml``, never
  in SQLite, never in a log.
* :mod:`endpoints` -- credential-bearing requests pinned to each provider's
  official HTTPS endpoint, so a stray setting or environment variable cannot
  send a key to another host.
* :mod:`redact` -- a last-line log filter that masks anything shaped like a
  key or an Authorization header before a record is written.
"""

from .secrets import SecretStore, InMemorySecretStore, default_store, mask
from .redact import redact, RedactingFilter
from . import endpoints

__all__ = [
    "SecretStore", "InMemorySecretStore", "default_store", "mask",
    "redact", "RedactingFilter", "endpoints",
]
