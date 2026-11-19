"""Where the AI layer's model runs.

The options, in the order most people will want them:

*Ollama*, on this machine. Free, private, and needs no account. A small
instruction-tuned model is entirely adequate for picking one label out of thirty
and reading a couple of numbers out of a sentence.

*A local OpenAI-compatible server* -- the OpenATC AI server, llama.cpp's,
LM Studio -- on this machine or the LAN. Plain http, no key: the host is
pinned to loopback or a private network address.

*The Claude API*, in the cloud. More accurate on the awkward transmissions,
costs a fraction of a cent each, and needs a key.

Both are optional. With neither configured the system runs exactly as it did
before -- the rule parser handles the phraseology, and anything it cannot place
gets "say again", which is what a real controller would say anyway.
"""

from __future__ import annotations

import ipaddress
import logging
import math
import os
import time
from urllib.parse import urlsplit

import requests

from .base import RESPONSE_SCHEMA

log = logging.getLogger(__name__)

#: Providers whose calls never leave the machine or the LAN: they send no
#: credential, so they need neither cloud consent nor the endpoint pinning
#: that guards the keyed providers. Used for the consent checks in
#: engine/core.py.
LOCAL_PROVIDER_NAMES = frozenset({"ollama", "local", "openatc"})


def _timeout(value) -> float:
    timeout = float(value)
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("AI timeout must be positive and finite")
    return timeout


# --------------------------------------------------------------------------
# local: Ollama
# --------------------------------------------------------------------------


class OllamaProvider:
    """Talks to a local Ollama server over HTTP.

    Nothing leaves the machine. Install from ollama.com, then::

        ollama pull qwen3:4b
    """

    name = "ollama"

    def __init__(
        self,
        model: str = "qwen3:4b",
        host: str = "http://127.0.0.1:11434",
        timeout_s: float = 12.0,
    ):
        self.model = model
        self.host = host.rstrip("/")
        self.timeout_s = _timeout(timeout_s)
        self._checked: bool | None = None
        self._check_after = 0.0

    def available(self) -> bool:
        """Whether the server is up and has the model."""
        if self._checked is True or (self._checked is False and time.monotonic() < self._check_after):
            return self._checked
        try:
            response = requests.get(f"{self.host}/api/tags", timeout=2.0)
            response.raise_for_status()
            names = {m.get("name", "") for m in response.json().get("models", [])}
        except Exception as exc:
            log.info("Ollama not reachable (%s); retrying after backoff", type(exc).__name__)
            self._checked = False
            self._check_after = time.monotonic() + 10.0
            return False

        # Ollama reports "llama3.2:3b"; a bare "llama3.2" should still match.
        wanted = self.model if ":" in self.model else self.model + ":latest"
        self._checked = wanted in names or self.model in names
        self._check_after = time.monotonic() + 10.0
        if not self._checked:
            log.warning(
                "Ollama is running but %s is not installed. Run: ollama pull %s",
                self.model, self.model,
            )
        return self._checked

    def classify(self, prompt: str, system: str) -> str:
        response = requests.post(
            f"{self.host}/api/chat",
            json={
                "model": self.model,
                "messages": [
                    {"role": "system", "content": system},
                    {"role": "user", "content": prompt},
                ],
                "stream": False,
                # Ollama's structured-output mode: the response is constrained
                # to the schema, so there is no prose to strip.
                "format": RESPONSE_SCHEMA,
                "options": {"temperature": 0.0, "num_predict": 256},
            },
            timeout=self.timeout_s,
        )
        response.raise_for_status()
        return response.json().get("message", {}).get("content", "")

    def complete(self, prompt: str, system: str) -> str:
        """A free-form answer -- the phrasing renderer's sentence, which is
        not a schema-shaped classification."""
        response = requests.post(
            f"{self.host}/api/chat",
            json={
                "model": self.model,
                "messages": [
                    {"role": "system", "content": system},
                    {"role": "user", "content": prompt},
                ],
                "stream": False,
                "options": {"temperature": 0.1, "num_predict": 256},
            },
            timeout=self.timeout_s,
        )
        response.raise_for_status()
        return response.json().get("message", {}).get("content", "")


# --------------------------------------------------------------------------
# cloud: Claude
# --------------------------------------------------------------------------


class AnthropicProvider:
    """Classifies through the Claude API.

    Uses structured outputs so the reply is schema-valid JSON, and low effort
    because picking one label from a list is not a reasoning problem and the
    round trip sits in the middle of a radio exchange.
    """

    name = "anthropic"

    def __init__(
        self,
        model: str = "claude-opus-5",
        api_key: str | None = None,
        timeout_s: float = 12.0,
        max_tokens: int = 512,
    ):
        self.model = model
        self.api_key = api_key or os.environ.get("ANTHROPIC_API_KEY", "")
        self.timeout_s = _timeout(timeout_s)
        self.max_tokens = max_tokens
        self._client = None
        self._checked: bool | None = None

    def available(self) -> bool:
        if self._checked is not None:
            return self._checked
        try:
            import anthropic as _sdk  # noqa: F401  (presence check only)
        except ImportError:
            log.info("anthropic package not installed; run: pip install anthropic")
            self._checked = False
            return False
        # A key in the environment is the common case, but the SDK also
        # resolves an `ant auth login` profile, so an unset key is not proof
        # that there are no credentials.
        self._checked = True
        return True

    def _get_client(self):
        if self._client is None:
            import anthropic
            import httpx

            options = {"api_key": self.api_key} if self.api_key else {}
            http_client = httpx.Client(follow_redirects=False, trust_env=False)
            try:
                self._client = anthropic.Anthropic(
                    **options, base_url="https://api.anthropic.com",
                    timeout=self.timeout_s, max_retries=0,
                    http_client=http_client)
            except Exception:
                http_client.close()
                raise
        return self._client

    def close(self) -> None:
        if self._client is not None:
            self._client.close()

    def classify(self, prompt: str, system: str) -> str:
        import anthropic

        client = self._get_client()
        try:
            response = client.messages.create(
                model=self.model,
                max_tokens=self.max_tokens,
                system=system,
                messages=[{"role": "user", "content": prompt}],
                output_config={
                    "format": {"type": "json_schema", "schema": RESPONSE_SCHEMA},
                    # Classification is not a reasoning problem, and this call
                    # sits inside a radio exchange where latency is felt.
                    "effort": "low",
                },
            )
        except anthropic.NotFoundError:
            log.error("Model %s not available to this account", self.model)
            raise
        except anthropic.RateLimitError as exc:
            log.warning("Claude API rate limited (%s)", type(exc).__name__)
            raise
        except anthropic.APIStatusError as exc:
            log.warning("Claude API error %s", exc.status_code)
            raise
        except anthropic.APIConnectionError as exc:
            log.warning("Claude API unreachable (%s)", type(exc).__name__)
            raise

        if response.stop_reason == "refusal":
            log.info("Claude declined to classify the transmission")
            return ""
        return "".join(
            block.text for block in response.content if block.type == "text"
        )

    def complete(self, prompt: str, system: str) -> str:
        """A free-form answer for the phrasing renderer: no schema, just the
        sentence the prompt asks for."""
        client = self._get_client()
        response = client.messages.create(
            model=self.model,
            max_tokens=self.max_tokens,
            system=system,
            messages=[{"role": "user", "content": prompt}],
        )
        if response.stop_reason == "refusal":
            log.info("Claude declined to render the transmission")
            return ""
        return "".join(
            block.text for block in response.content if block.type == "text"
        )


# --------------------------------------------------------------------------


class OpenAICompatibleProvider:
    """One provider class for every OpenAI-shaped chat endpoint.

    StormATC shipped five -- OpenRouter, OpenAI, Groq, DeepSeek and a custom
    base-URL -- that differed only in endpoint, auth header and default
    model. The OpenAI-compatible chat-completions body is the same for all
    of them, so one class with the endpoint pinned per name carries them
    all. The endpoint is the provider's official one by default and, per
    the endpoint policy, cannot be redirected away by the server.
    """

    #: Official endpoints per provider name; ``custom`` takes ``host``.
    ENDPOINTS = {
        "openrouter": "https://openrouter.ai/api/v1/chat/completions",
        "openai": "https://api.openai.com/v1/chat/completions",
        "groq": "https://api.groq.com/openai/v1/chat/completions",
        "deepseek": "https://api.deepseek.com/chat/completions",
    }
    DEFAULT_MODELS = {
        "openrouter": "google/gemini-3-flash-preview",
        "openai": "gpt-5.2",
        "groq": "openai/gpt-oss-20b",
        "deepseek": "deepseek-chat",
    }

    def __init__(self, provider: str, api_key: str, model: str = "",
                 host: str = "", timeout_s: float = 12.0):
        self.provider = (provider or "openrouter").strip().lower()
        self.name = self.provider
        self.api_key = api_key
        self.model = model or self.DEFAULT_MODELS.get(self.provider, "")
        if self.provider == "custom":
            # There is no official endpoint to pin to, so the user's URL is
            # itself the pin: https, a real host, no credentials, query or
            # fragment -- the same bars the policy holds named providers to.
            base = (host or "").rstrip("/")
            from urllib.parse import urlsplit

            try:
                uri = urlsplit(base)
                port = uri.port
            except ValueError as exc:
                raise ValueError(
                    "custom provider needs a valid URL") from exc
            if (uri.scheme.lower() != "https" or not uri.hostname
                    or uri.username or uri.password
                    or uri.query or uri.fragment or port not in (None, 443)):
                raise ValueError(
                    "custom provider needs an https URL without credentials, "
                    "query or fragment")
            self.url = f"{base}/chat/completions"
            self._custom_ok = True
        else:
            self.url = self.ENDPOINTS[self.provider]
            self._custom_ok = False
        self._pinned_url = self.url
        self.timeout_s = _timeout(timeout_s)
        self._session = requests.Session()
        self._session.max_redirects = 0
        self._session.trust_env = False

    def available(self) -> bool:
        if self.provider == "custom":
            return bool(self.api_key) and self._custom_ok and self.url == self._pinned_url
        from ..security.endpoints import is_allowed

        return bool(self.api_key) and is_allowed(self.provider, self.url)

    def _headers(self) -> dict[str, str]:
        headers = {"Authorization": f"Bearer {self.api_key}",
                   "Content-Type": "application/json"}
        if self.provider == "openrouter":
            headers.update({"HTTP-Referer": "https://meridianatc.local",
                            "X-Title": "MeridianATC"})
        return headers

    def chat(self, messages: list[dict[str, str]],
             max_tokens: int = 700) -> str:
        """One completion round-trip; an exception propagates on failure."""
        if not self.available():
            raise ValueError("AI endpoint blocked or credentials missing")
        body = {"model": self.model, "messages": messages,
                "temperature": 0.1, "max_tokens": max_tokens,
                "stream": False}
        response = self._session.post(
            self.url, json=body, headers=self._headers(),
            timeout=self.timeout_s, allow_redirects=False)
        response.raise_for_status()
        if 300 <= response.status_code < 400:
            raise ValueError("AI endpoint redirect refused")
        return (response.json()["choices"][0]["message"]["content"]
                or "").strip()

    def close(self) -> None:
        self._session.close()

    def classify(self, prompt: str, system: str) -> str:
        """The contract the understanding layer calls on every provider."""
        return self.chat(
            [{"role": "system", "content": system},
             {"role": "user", "content": prompt}])

    def complete(self, prompt: str, system: str) -> str:
        """Free-form for the phrasing renderer: same chat, no schema."""
        return self.classify(prompt, system)


def build_provider(kind: str, **options):
    """Construct a provider by name, or ``None`` when the AI layer is off."""
    kind = (kind or "none").strip().lower()
    if kind in ("", "none", "off", "disabled"):
        return None
    if kind == "ollama":
        return OllamaProvider(
            model=options.get("model") or "qwen3:4b",
            host=options.get("host") or "http://127.0.0.1:11434",
            timeout_s=float(options.get("timeout_s", 12.0)),
        )
    if kind in ("local", "openatc"):
        host = options.get("host") or ""
        if kind == "openatc" and host in ("", "http://127.0.0.1:11434"):
            # An untouched host still carries Ollama's default; the OpenATC
            # AI server listens on its own port.
            host = LocalOpenAIProvider.DEFAULT_HOST
        return LocalOpenAIProvider(
            model=options.get("model") or "",
            host=host,
            timeout_s=float(options.get("timeout_s", 12.0)),
        )
    if kind in ("anthropic", "claude"):
        return AnthropicProvider(
            model=options.get("model") or "claude-opus-5",
            api_key=options.get("api_key") or None,
            timeout_s=float(options.get("timeout_s", 12.0)),
        )
    if kind in OpenAICompatibleProvider.ENDPOINTS or kind == "custom":
        return OpenAICompatibleProvider(
            provider=kind,
            api_key=options.get("api_key") or "",
            model=options.get("model") or "",
            host=options.get("host") or "",
            timeout_s=float(options.get("timeout_s", 12.0)),
        )
    raise ValueError(f"Unknown AI provider {kind!r}")


# --------------------------------------------------------------------------
# local: an OpenAI-compatible server on this machine or the LAN
# --------------------------------------------------------------------------


def _is_local_host(hostname: str) -> bool:
    """Loopback or private-network only -- the guarantee under which plain
    http is acceptable here, since no credential ever crosses the wire."""
    name = (hostname or "").strip().lower()
    if name == "localhost":
        return True
    try:
        address = ipaddress.ip_address(name)
    except ValueError:
        return False
    return address.is_loopback or address.is_private


class LocalOpenAIProvider:
    """A local server speaking the OpenAI chat-completions shape: the
    OpenATC AI server, llama.cpp's, LM Studio, vLLM -- anything on the LAN
    that answers ``/v1/chat/completions``.

    Plain ``http`` is fine *because* nothing secret crosses: the class sends
    no key at all, and the host is pinned to a loopback or private address
    so a settings file cannot aim pilot transmissions at an internet
    endpoint. A remote server belongs behind ``custom``, which demands
    https and a key.
    """

    name = "local"

    #: The OpenATC AI server's port, which is what this provider exists for.
    DEFAULT_HOST = "http://127.0.0.1:8099"

    def __init__(self, model: str = "", host: str = "",
                 timeout_s: float = 12.0):
        self.model = model
        self.timeout_s = _timeout(timeout_s)
        base = (host or self.DEFAULT_HOST).rstrip("/")
        try:
            uri = urlsplit(base)
            _ = uri.port  # forces validation of the port field
        except ValueError as exc:
            raise ValueError("local provider needs a valid URL") from exc
        if (uri.scheme.lower() not in ("http", "https")
                or not uri.hostname
                or uri.username or uri.password
                or uri.query or uri.fragment):
            raise ValueError(
                "local provider needs an http(s) URL without credentials, "
                "query or fragment")
        if not _is_local_host(uri.hostname):
            raise ValueError(
                "local provider host must be a loopback or private-network "
                "address; use 'custom' (https + key) for a remote endpoint")
        self._origin = f"{uri.scheme}://{uri.netloc}"
        # A base already carrying /v1 keeps it; a bare origin gets it added.
        self._chat_base = base if base.lower().endswith("/v1") \
            else base + "/v1"
        self.url = f"{self._chat_base}/chat/completions"
        self._checked: bool | None = None
        self._check_after = 0.0
        # The same hardening the keyed provider carries: no redirects (a
        # local server must not forward the conversation elsewhere), and no
        # environment inheritance (netrc credentials, external proxies).
        self._session = requests.Session()
        self._session.max_redirects = 0
        self._session.trust_env = False

    def available(self) -> bool:
        """Whether the configured address serves the OpenAI chat shape."""
        if self._checked is True or (self._checked is False
                                   and time.monotonic() < self._check_after):
            return self._checked
        # /health is the OpenATC server's liveness probe; /v1/models is the
        # OpenAI shape most of the rest answer. A 4xx means the endpoint is
        # not there -- it does not prove the chat shape is served.
        for probe in (f"{self._origin}/health", f"{self._chat_base}/models"):
            try:
                response = self._session.get(probe, timeout=2.0)
            except Exception:
                continue
            if response.status_code < 400:
                self._checked = True
                break
        else:
            self._checked = False
        self._check_after = time.monotonic() + 10.0
        if not self._checked:
            log.info("local OpenAI server not reachable at %s", self._origin)
        return self._checked

    def close(self) -> None:
        self._session.close()

    def _post(self, body: dict):
        response = self._session.post(
            self.url, json=body, timeout=self.timeout_s,
            allow_redirects=False)
        if 300 <= response.status_code < 400:
            raise ValueError("AI endpoint redirect refused")
        return response

    def _chat(self, messages: list[dict[str, str]], temperature: float,
              response_format: dict | None = None) -> str:
        body = {"model": self.model, "messages": messages,
                "temperature": temperature, "max_tokens": 256,
                "stream": False}
        if response_format:
            body["response_format"] = response_format
        response = self._post(body)
        if response_format and response.status_code == 400:
            # A server without structured-output support gets the same call
            # without the schema; parse_response still validates the answer.
            del body["response_format"]
            response = self._post(body)
        response.raise_for_status()
        return (response.json()["choices"][0]["message"]["content"] or "").strip()

    def classify(self, prompt: str, system: str) -> str:
        return self._chat(
            [{"role": "system", "content": system},
             {"role": "user", "content": prompt}],
            temperature=0.0,
            response_format={"type": "json_schema",
                             "json_schema": {"name": "intent",
                                             "schema": RESPONSE_SCHEMA}})

    def complete(self, prompt: str, system: str) -> str:
        return self._chat(
            [{"role": "system", "content": system},
             {"role": "user", "content": prompt}],
            temperature=0.1)


__all__ = ["OllamaProvider", "AnthropicProvider", "OpenAICompatibleProvider",
           "LocalOpenAIProvider", "LOCAL_PROVIDER_NAMES", "build_provider"]
