"""Credential-bearing requests pinned to each provider's official endpoint.

A port of StormATC's ``ProviderEndpointPolicy``. Provider URLs may remain
configurable for diagnostics, but a settings file, an environment variable
or a command-line argument must never be able to redirect an API key to
another host. Every client checks this immediately before attaching a key,
and redirect-following is disabled on credentialed requests.
"""

from __future__ import annotations

from urllib.parse import urlsplit

#: provider name -> (scheme, host, path prefix)
PROVIDERS: dict[str, tuple[str, str, str]] = {
    "openrouter": ("https", "openrouter.ai", "/api/v1"),
    "openai": ("https", "api.openai.com", "/v1"),
    "groq": ("https", "api.groq.com", "/openai/v1"),
    "deepseek": ("https", "api.deepseek.com", "/"),
    "anthropic": ("https", "api.anthropic.com", "/"),
    "deepgram": ("https", "api.deepgram.com", "/"),
    "deepgram_stream": ("wss", "api.deepgram.com", "/v1/listen"),
    "simbrief": ("https", "www.simbrief.com", "/api/xml.fetcher.php"),
}


def is_allowed(provider: str, url: str | None) -> bool:
    """True only when ``url`` is that provider's official endpoint.

    Strictly: the right scheme, the right host, the right path, no embedded
    credentials, no query, no fragment, and the default port -- anything else
    would be a way to steer a key somewhere it was never meant to go.
    """
    spec = PROVIDERS.get(provider)
    if spec is None or not url:
        return False
    scheme, host, path = spec
    try:
        uri = urlsplit(url.strip())
    except ValueError:
        return False
    if (uri.scheme.lower() != scheme
            or uri.hostname is None
            or uri.hostname.lower() != host
            or uri.username or uri.password
            or uri.query or uri.fragment):
        return False
    port = uri.port
    if port is not None and port != (443 if scheme in ("https", "wss") else 80):
        return False
    return uri.path.rstrip("/").lower() == path.rstrip("/").lower()


def allowed_url(provider: str, url: str | None) -> str | None:
    """The URL if it passes :func:`is_allowed`, else ``None``."""
    return url if is_allowed(provider, url) else None
