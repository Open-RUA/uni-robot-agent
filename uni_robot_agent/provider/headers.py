"""Validate custom upstream HTTP headers shared by configuration and transport."""

from __future__ import annotations

import re
from collections.abc import Iterable, Mapping
from typing import Any


_HEADER_NAME = re.compile(r"[!#$%&'*+\-.^_`|~0-9A-Za-z]+")

# Headers owned by the HTTP connection or the local ingress. Custom provider
# headers cannot replace them without changing message framing.
_PROVIDER_HEADER_BLOCKLIST = frozenset({
    "connection",
    "content-length",
    "host",
    "keep-alive",
    "proxy-authenticate",
    "proxy-authorization",
    "te",
    "trailer",
    "transfer-encoding",
    "upgrade",
})


def _split_provider_header(entry: Any) -> tuple[str, str]:
    if not isinstance(entry, str):
        raise ValueError("provider header entries must be NAME=VALUE strings")
    name, separator, value = entry.partition("=")
    if not separator:
        raise ValueError("provider header entries must use NAME=VALUE syntax")
    return name, value


def _validate_provider_header(name: Any, value: Any) -> tuple[str, str]:
    if not isinstance(name, str) or not isinstance(value, str):
        raise ValueError("provider header names and values must be strings")
    if not name:
        raise ValueError("provider header name must not be empty")
    if _HEADER_NAME.fullmatch(name) is None:
        raise ValueError(f"provider header name {name!r} is not a valid HTTP token")
    if name.lower() in _PROVIDER_HEADER_BLOCKLIST:
        raise ValueError(
            f"provider header {name!r} is hop-by-hop or transport-managed"
        )
    if any(character in value for character in "\r\n\0"):
        raise ValueError(
            f"provider header {name!r} value must not contain CR, LF or NUL"
        )
    return name, value


def parse_provider_headers(
    values: Mapping[str, str] | Iterable[str] | None,
) -> dict[str, str]:
    """Validate custom upstream headers from repeatable entries or a mapping.

    Names are compared case-insensitively and a later duplicate replaces the
    earlier value. Malformed entries, invalid names, transport-managed names and
    values containing CR, LF or NUL raise ValueError before any connection.
    """
    if values is None:
        return {}
    if isinstance(values, Mapping):
        entries: Iterable[tuple[Any, Any]] = values.items()
    elif isinstance(values, (str, bytes)):
        raise ValueError(
            "provider headers must be a mapping or an iterable of NAME=VALUE strings"
        )
    else:
        entries = (_split_provider_header(entry) for entry in values)
    parsed: dict[str, str] = {}
    spellings: dict[str, str] = {}
    for name, value in entries:
        name, value = _validate_provider_header(name, value)
        key = name.lower()
        if key in spellings:
            del parsed[spellings[key]]
        spellings[key] = name
        parsed[name] = value
    return parsed
