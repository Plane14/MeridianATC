# Security model

MeridianATC is a local program that can optionally reach a few cloud
services. This file documents how credentials, endpoints, and logs are
protected.

## Secrets

* API keys are **never written to `config.yaml`**. Settings whose
  `Setting.kind` is `"secret"` are stored in the OS credential store —
  Windows DPAPI on Windows (`meridianatc/security/secrets.py`), with a
  file-based store as the documented fallback on other platforms.
* Resolution order is always: secure store → environment variable →
  legacy config value. See `security.secrets.resolve_secret`.
* The settings API never echoes a secret back: `_settings_view` blanks
  every path listed in `web/server.py:SECRETS`. Writing an empty value
  keeps what is set; a non-empty value goes to the store, not the file.

## Endpoint pinning

Every credentialed request is checked against
`security/endpoints.py:PROVIDERS` immediately before the key is attached:
scheme, host, path prefix, no embedded credentials, no query, no fragment,
default port. Redirects are disabled on credentialed calls. A settings
file, environment variable, or command line can never steer a key to a
different host.

## Log redaction

`logs.configure` installs `security.redact.RedactingFilter` on every log
handler. Anything that looks like a credential (Bearer tokens, API keys,
key=value secrets) is rewritten in the log line before it is written.

## Offline mode and consent

* `network.offline` is a hard kill-switch: the online network is built as
  `none`, weather fetches are off, the Deepgram recognizer is not
  registered, and no cloud LLM provider is constructed.
* `network.cloud_consent` is the first-run consent. A cloud provider named
  in a config without it is built as `none`. Both flags require a restart
  to change, so a running session cannot silently acquire connectivity.

## The authority rule

The deterministic engine is the source of truth for aircraft facts,
clearances, frequencies, runways, squawks, and readback correctness. LLM
providers may only classify pilot speech or rephrase engine-approved
transmissions; the phrasing renderer validates every fact token in the
model output against the approved wording and discards any mismatch.
