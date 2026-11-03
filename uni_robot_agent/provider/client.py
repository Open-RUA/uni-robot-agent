"""Upstream model calls, context preparation and bounded exchange retries."""

from __future__ import annotations
import json
import time
from typing import Any
from uni_robot_agent.context import ContextManager, PreparedContext
from uni_robot_agent.context.history import create_context_policy
from uni_robot_agent.provider.contracts import (
    ProviderAdapter,
    SessionAdapter,
    ModelProtocol,
    ModelResponse,
    EncodedRequest,
)
from uni_robot_agent.provider.registry import (
    build_provider_adapter,
    validate_provider_profile,
)
from uni_robot_agent.provider.protocols.responses import ResponsesProtocol
from uni_robot_agent.provider.recording.exchanges import response_error_detail
from uni_robot_agent.services.errors import HttpCallError
from uni_robot_agent.services.http import HttpTransport
from uni_robot_agent.telemetry.logging import get_logger

logger = get_logger("provider.client")


class ModelClient:
    """Request a model without starting an HTTP server or sharing role state."""

    def __init__(
        self,
        base_url: str,
        *,
        adapter: ProviderAdapter | None = None,
        protocol: ModelProtocol | None = None,
        context_manager: ContextManager | None = None,
        request_timeout_s: float = 600,
        max_retries: int = 0,
        retry_backoff_initial_s: float = 2,
        retry_http_errors: bool = True,
        retry_response_errors: bool = True,
        retryable_exceptions: tuple[type[Exception], ...] = (Exception,),
        tracer=None,
        attempt_observer=None,
    ) -> None:
        if request_timeout_s <= 0 or max_retries < 0 or retry_backoff_initial_s < 0:
            raise ValueError("Invalid provider request timeout/retry budget")
        self.base_url = base_url.rstrip("/")
        self.protocol = protocol or ResponsesProtocol()
        self.adapter = adapter or build_provider_adapter(
            f"passthrough.{self.protocol.name}", client_protocol=self.protocol.name
        )
        profile = validate_provider_profile(
            self.adapter.profile_name, client_protocol=self.protocol.name
        )
        if profile.upstream_protocol == "messages":
            from uni_robot_agent.provider.protocols.messages import response_failure
        elif profile.upstream_protocol == "chat_completions":
            from uni_robot_agent.provider.protocols.chat_completions import (
                response_failure,
            )
        else:
            from uni_robot_agent.provider.protocols.responses import response_failure
        self._response_failure = response_failure
        self._upstream_protocol = profile.upstream_protocol
        self._attempt_observer = attempt_observer
        self.context_manager = context_manager or ContextManager(
            create_context_policy("identity")
        )
        self._transport = HttpTransport(
            timeout_s=request_timeout_s, trust_env=True, tracer=tracer
        )
        self._session = (
            self.adapter.open_session(self._transport, self.base_url)
            if isinstance(self.adapter, SessionAdapter)
            else None
        )
        self._max_retries = max_retries
        self._retry_backoff_initial_s = retry_backoff_initial_s
        self._retry_http_errors = retry_http_errors
        self._retry_response_errors = retry_response_errors
        self._retryable_exceptions = retryable_exceptions

    def prepare_request(
        self,
        request: dict[str, Any],
        *,
        preserve_history: bool = False,
    ) -> tuple[PreparedContext, EncodedRequest]:
        self.protocol.validate_context(self.context_manager)
        prepared = self.context_manager.prepare(
            request, preserve_history=preserve_history
        )
        return prepared, self.adapter.encode_request(prepared.effective)

    def request(
        self,
        body: dict[str, Any],
        *,
        headers: dict[str, str],
        request_path: str | None = None,
    ) -> ModelResponse:
        """Run one direct request with this role's context, adapter, and retry policy."""
        _, encoded = self.prepare_request(body)
        exchange: dict[str, Any] = {"provider_attempts": []}
        self.prepare_wire(encoded.body, headers, exchange)
        status, response_headers, response = self.send(
            target=self.adapter.target_url(
                self.base_url, request_path or f"/{self.protocol.name}"
            ),
            request_bytes=_encode_json(encoded.body),
            headers=headers,
            exchange=exchange,
        )
        if 200 <= status < 300 and not exchange.get("response_error"):
            response = self.adapter.decode_response(response, encoded.decode_state).body
        return ModelResponse(
            status, response_headers, response, tuple(exchange["provider_attempts"])
        )

    def prepare_wire(self, body: dict, headers: dict, exchange: dict) -> None:
        if self._session is not None:
            self._session.prepare_request(body, headers, exchange)

    def send(
        self,
        *,
        target: str,
        request_bytes: bytes,
        headers: dict[str, str],
        exchange: dict[str, Any] | None = None,
    ) -> tuple[int, Any, bytes]:
        try:
            return self._send(
                target=target,
                request_bytes=request_bytes,
                headers=headers,
                exchange=exchange,
            )
        finally:
            if self._session is not None:
                self._session.finish_request(exchange)

    def _send(
        self,
        *,
        target: str,
        request_bytes: bytes,
        headers: dict[str, str],
        exchange: dict[str, Any] | None = None,
    ) -> tuple[int, Any, bytes]:
        """Retry one fixed body only before any bytes reach its consumer."""
        attempts = exchange["provider_attempts"] if exchange is not None else []
        for attempt in range(self._max_retries + 1):
            started_at = time.monotonic()
            self._observe_attempt(exchange, attempt + 1, "started")
            try:
                options = {} if self._retry_http_errors else {"raise_http_errors": True}
                status, response_headers, body = _send_provider_request(
                    self._transport,
                    target,
                    request_bytes=request_bytes,
                    headers=_headers_for_attempt(headers, attempt),
                    **options,
                )
            except Exception as exc:
                self._observe_attempt(exchange, attempt + 1, "finished")
                attempts.append(
                    {
                        "attempt": attempt + 1,
                        "status": None,
                        "elapsed_s": round(time.monotonic() - started_at, 3),
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                )
                if (
                    attempt >= self._max_retries
                    or isinstance(exc, HttpCallError)
                    and not self._retry_http_errors
                    or not isinstance(exc, self._retryable_exceptions)
                ):
                    raise
                self._wait_before_retry(attempt, "transport error")
                continue
            self._observe_attempt(exchange, attempt + 1, "finished", status=status, body=body)
            record = {
                "attempt": attempt + 1,
                "status": status,
                "elapsed_s": round(time.monotonic() - started_at, 3),
            }
            if request_id := _header_value(response_headers, "x-request-id"):
                record["request_id"] = request_id
            failure = (
                self._response_failure(body)
                if self._retry_response_errors
                and exchange is not None
                and 200 <= status < 300
                else None
            )
            if failure:
                record["response_error"] = failure
                record["error"] = f"{failure['code']}: {failure['message']}"
            elif not 200 <= status < 300:
                if detail := response_error_detail(body, max_chars=500):
                    record["error"] = detail
            attempts.append(record)
            succeeded = 200 <= status < 300 and failure is None
            if succeeded or attempt >= self._max_retries:
                if failure and exchange is not None:
                    exchange["response_error"] = failure
                    exchange["error"] = record["error"]
                if attempt:
                    logger.info(
                        "Provider request %s after %d retries",
                        "completed" if succeeded else "failed",
                        attempt,
                    )
                return status, response_headers, body
            self._wait_before_retry(
                attempt, failure["code"] if failure else f"HTTP {status}"
            )
        raise RuntimeError("provider retry loop ended unexpectedly")

    def _observe_attempt(self, exchange, attempt, phase, *, status=None, body=b""):
        if self._attempt_observer is None:
            return
        from uni_robot_agent.provider.usage import token_counts
        try:
            counts = token_counts(body, self._upstream_protocol) if phase == "finished" else {}
            self._attempt_observer(exchange or {}, attempt, phase, status, counts)
        except Exception:
            logger.exception("Provider attempt recording failed")

    def close(self) -> None:
        try:
            if self._session is not None:
                self._session.close()
        finally:
            self._transport.close()

    def _wait_before_retry(self, attempt: int, reason: str) -> None:
        delay_s = self._retry_backoff_initial_s * (2**attempt)
        logger.warning(
            "Provider request failed (%s); retry %d/%d in %.1fs",
            reason,
            attempt + 1,
            self._max_retries,
            delay_s,
        )
        if delay_s:
            time.sleep(delay_s)


def _encode_json(value: dict[str, Any]) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode()


def _send_provider_request(
    transport: HttpTransport,
    target: str,
    *,
    request_bytes: bytes,
    headers: dict[str, str],
    raise_http_errors: bool = False,
) -> tuple[int, Any, bytes]:
    # HTTPX consumes the complete response and rejects truncated Content-Length bodies.
    response = transport.request(
        "POST",
        target,
        content=request_bytes,
        headers=headers,
        check=raise_http_errors,
    )
    # Keep original casing for persisted provider headers and diagnostics.
    return (
        response.status_code,
        {
            key.decode("latin-1"): value.decode("latin-1")
            for key, value in response.headers.raw
        },
        response.content,
    )


def _headers_for_attempt(headers: dict[str, str], attempt: int) -> dict[str, str]:
    if attempt == 0:
        return headers
    return {
        key: value
        for key, value in headers.items()
        if key.lower() != "x-client-request-id"
    }


def _header_value(headers: Any, name: str) -> str | None:
    for key, value in headers.items():
        if key.lower() == name:
            return str(value)
    return None
