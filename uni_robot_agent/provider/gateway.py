"""Shared model HTTP requests and an observable protocol-aware gateway."""

from __future__ import annotations

import asyncio
import json
import time
import traceback
from collections.abc import Mapping
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

import anyio
from fastapi import APIRouter, Request
from starlette.concurrency import run_in_threadpool
from starlette.responses import Response

from uni_robot_agent.context import ContextManager, PreparedContext
from uni_robot_agent.context.history import create_context_policy
from uni_robot_agent.provider.contracts import (
    DecodedResponse,
    EncodedRequest,
    ProviderAdapter,
)
from uni_robot_agent.provider.headers import parse_provider_headers
from uni_robot_agent.provider.recording.exchanges import (
    ExchangeStore,
    attach_wire_views,
)
from uni_robot_agent.services.hosting import HttpServer, create_app
from uni_robot_agent.telemetry.logging import get_logger
from uni_robot_agent.telemetry import NullTracer

logger = get_logger("provider.gateway")


from uni_robot_agent.provider.contracts import RequestContext, PreparedReply, ModelBinding, ModelProtocol
from uni_robot_agent.provider.protocols.responses import ResponsesProtocol
from uni_robot_agent.provider.client import ModelClient
from uni_robot_agent.provider.registry import build_provider_adapter



_HOP_BY_HOP_HEADERS = {
    "connection",
    "content-length",
    "keep-alive",
    "proxy-authenticate",
    "proxy-authorization",
    "te",
    "trailer",
    "transfer-encoding",
    "upgrade",
}


class ProviderGateway:
    """Forward requests and retain client, context-policy, and provider views."""

    def __init__(
        self,
        upstream_base_url: str,
        *,
        host: str = "127.0.0.1",
        trace_dir: Path | str | None = None,
        artifacts=None,
        save_raw_request: bool = False,
        save_exchanges: bool = True,
        adapter: ProviderAdapter | None = None,
        protocol: ModelProtocol | None = None,
        context_manager: ContextManager | None = None,
        context_policy: str = "identity",
        context_options: Mapping[str, Any] | None = None,
        request_timeout_s: float = 600,
        max_retries: int = 0,
        retry_backoff_initial_s: float = 2,
        model_parallel_tool_calls: bool = True,
        model_binding: ModelBinding | None = None,
        provider_headers: Mapping[str, str] | None = None,
        event_bus=None,
        tracer=None,
    ) -> None:
        self._upstream_base_url = upstream_base_url.rstrip("/")
        self._host = host
        self._provider_headers = parse_provider_headers(provider_headers)
        self._tracer = tracer or NullTracer()
        self._protocol = protocol or ResponsesProtocol()
        self._adapter = adapter or build_provider_adapter(
            f"passthrough.{self._protocol.name}", client_protocol=self._protocol.name)
        if context_manager is None:
            policy_name = context_policy
            policy = create_context_policy(policy_name, **dict(context_options or {}))
            context_manager = ContextManager(policy)
        self._context_manager = context_manager
        self._trace = ExchangeStore(
            trace_dir,
            artifacts=artifacts,
            save_raw_request=save_raw_request,
            save_exchanges=save_exchanges,
            client_name=self._protocol.trace_client,
            usage_reader=self._protocol.response_usage,
        )
        self._client = ModelClient(
            upstream_base_url,
            adapter=self._adapter,
            protocol=self._protocol,
            context_manager=context_manager,
            request_timeout_s=request_timeout_s,
            max_retries=max_retries,
            retry_backoff_initial_s=retry_backoff_initial_s,
            tracer=tracer,
        )
        self._model_parallel_tool_calls = model_parallel_tool_calls
        self._model_binding = model_binding
        self._event_bus = event_bus
        self._server: HttpServer | None = None

    @property
    def base_url(self) -> str:
        if self._server is None:
            raise RuntimeError("Provider gateway is not running")
        return f"http://{self._host}:{self._server.server_port}/v1"

    @property
    def trace_records(self) -> list[dict[str, Any]]:
        return self._trace.records

    def iter_training_exchanges(self, *, thread_id: str | None = None):
        return self._trace.iter_training_exchanges(thread_id=thread_id)

    def release_training_payloads(self, *, thread_id: str | None = None) -> None:
        self._trace.release_training_payloads(thread_id=thread_id)

    def start(self) -> str:
        if self._server is not None:
            raise RuntimeError("Provider gateway is already running")
        router = APIRouter()

        @router.post("/{path:path}")
        async def forward(request: Request, path: str):
            # Buffer before entering the worker; model retries never expose partial output.
            raw_request = await request.body()
            request_path = request.url.path
            if request.url.query:
                request_path += "?" + request.url.query
            def forward_buffered():
                return self._forward(request_path, raw_request, request.headers, request)

            if self._protocol.matches(request_path):
                response = self._protocol.buffered_stream_response(
                    _decode_json(raw_request), request, forward_buffered,
                )
                if response is not None:
                    return response
            return await run_in_threadpool(forward_buffered)

        # SDK image histories arrive before the adapter's lossless encoding.
        # Keep their ingress budget separate from small tool/service requests.
        app = create_app(router, on_close=self._client.close,
                         max_request_bytes=512 * 1024 * 1024, tracer=self._tracer)
        self._server = HttpServer(app, self._host)
        self._server.start_background()
        return self.base_url

    def _forward(self, request_path, raw_request, incoming_headers, request):
        from uni_robot_agent.provider.contracts import LocalRequestProtocol, RoutedModelProtocol
        if isinstance(self._protocol, LocalRequestProtocol):
            local = self._protocol.local_response(request_path)
            if local is not None:
                path = urlsplit(request_path).path
                logger.info('Local %s response: path=%s status=%s',
                            self._protocol.name, path, local.status)
                with self._tracer.span('provider.local_response', path=path, status_code=local.status):
                    return Response(content=local.body, status_code=local.status,
                                    headers=dict(local.headers))
        target = self._adapter.target_url(
            self._upstream_base_url, request_path
        )
        custom_names = {name.lower() for name in self._provider_headers}
        headers = {
            key: value
            for key, value in incoming_headers.items()
            if key.lower() not in _HOP_BY_HOP_HEADERS
            and key.lower() not in {"host", "accept-encoding"}
            and key.lower() not in custom_names
        }
        headers["Accept-Encoding"] = "identity"
        # Custom names were removed above so overriding Authorization or X-Api-Key
        # cannot leave two differently-cased copies on the wire.
        headers.update(self._provider_headers)

        exchange: dict[str, Any] | None = None
        prepared: PreparedContext | None = None
        encoded: EncodedRequest | None = None
        reply: PreparedReply | None = None
        request_bytes = raw_request

        try:
            normalized = None
            routed = isinstance(self._protocol, RoutedModelProtocol)
            if routed or self._protocol.matches(request_path):
                request_json = _decode_json(raw_request)
                exchange = {
                    "wire_profile": self._adapter.profile_name,
                    "request_url": target,
                    "response": b"",
                    "response_headers": {},
                    "status": None,
                    "provider_complete": False,
                    "delivered": False,
                    "complete": False,
                    "client_disconnected": False,
                    "delivery_error": None,
                    "error": None,
                    "error_traceback": None,
                    "provider_attempts": [],
                    "started_at": time.time(),
                    "finished_at": None,
                    "persistence_error": None,
                    "context_policy": (
                        self._context_manager.policy_metadata
                    ),
                }
                attach_wire_views(
                    exchange,
                    client_request=request_json,
                    request_after_context=None,
                    request_to_provider=None,
                    response_from_provider=b"",
                    client_response=b"",
                    profile=self._adapter.profile_name,
                )
                # Persist raw requests first so policy or adapter failures remain diagnosable.
                self._trace.begin(exchange, request_bytes=raw_request)
                request_context = (
                    self._model_binding.request_context(request_json)
                    if self._model_binding
                    else RequestContext()
                )
                exchange["agent_metadata"] = request_context.metadata
                self._model_event(exchange, "started")
                # Rejected model routes and unsupported operations must be recorded
                # and stop the owner before a native CLI can enter its retry loop.
                if routed:
                    normalized = self._protocol.normalize_request(request_path, request_json)

                # Control model generation only; MCP/runtime owns execution exclusion.
                # Copy so generation controls do not change the original client request.
                source = self._protocol.generation_request(
                    normalized if normalized is not None else request_json,
                    self._model_parallel_tool_calls,
                )
                # Select context before encoding the upstream protocol and vendor fields.
                prepared, encoded = self._client.prepare_request(
                    source,
                    preserve_history=request_context.preserve_history,
                )
                provider_request = encoded.body
                exchange["context_policy"] = prepared.decision.to_dict()
                attach_wire_views(
                    exchange,
                    client_request=request_json,
                    request_after_context=prepared.effective,
                    request_to_provider=provider_request,
                    response_from_provider=b"",
                    client_response=b"",
                    profile=self._adapter.profile_name,
                )
                exchange["request"] = encoded.training_body
                try:
                    self._client.prepare_wire(provider_request, headers, exchange)
                finally:
                    # Keep local images and upload evidence even when preparation fails.
                    self._trace.prepared(exchange)
                request_bytes = _encode_json(provider_request)

            status, response_headers, provider_response = self._client.send(
                target=target,
                request_bytes=request_bytes,
                headers=headers,
                exchange=exchange,
            )
            if exchange is not None:
                exchange["status"] = status
                exchange["response_headers"] = dict(response_headers.items())

            failure = (
                exchange.get("response_error") if exchange is not None else None
            )
            # Failed streams may contain partial calls. After retries are exhausted,
            # deliver only the error so the SDK cannot execute calls from a failed attempt.
            decoded = (
                self._adapter.decode_response(
                    provider_response, encoded.decode_state
                )
                if encoded is not None and status == 200 and not failure
                else DecodedResponse(provider_response, provider_response)
            )
            client_response = (
                _encode_json(self._protocol.error_response(failure)) if failure else decoded.body
            )
            if (
                exchange is not None
                and status == 200
                and not failure
                and self._model_binding
            ):
                reply = self._model_binding.prepare_response(
                    client_response,
                    f"exchange_{exchange['index']:04d}",
                    exchange["agent_metadata"],
                )
                client_response = reply.body
            if exchange is not None:
                attach_wire_views(
                    exchange,
                    client_request=exchange.get("client_request"),
                    request_after_context=exchange.get("request_after_context"),
                    request_to_provider=exchange.get("request_to_provider"),
                    response_from_provider=provider_response,
                    client_response=client_response,
                    profile=self._adapter.profile_name,
                )
                exchange["request"] = encoded.training_body
                exchange["response"] = decoded.training_body
                exchange["provider_complete"] = True

            if normalized is not None and not failure and status == 200:
                response_headers = self._protocol.response_headers(normalized, response_headers)

            return _GatewayResponse(
                self, request, exchange, reply,
                status=502 if failure else status,
                headers={"Content-Type": "application/json"} if failure else response_headers,
                body=client_response,
                complete=not failure,
            )
        except Exception as exc:  # noqa: BLE001 - provider boundary records the failed exchange.
            if exchange is not None:
                exchange["error"] = f"{type(exc).__name__}: {exc}"
                exchange["error_traceback"] = traceback.format_exc()
            return _GatewayResponse(
                self, request, exchange, reply,
                status=502, headers={"Content-Type": "application/json"},
                body=_encode_json(self._protocol.exception_response(exc)), complete=False,
            )

        finally:
            if exchange is not None and "index" in exchange:
                self._model_event(exchange, "finished")

    def _model_event(self, exchange, phase):
        if self._event_bus is not None:
            from uni_robot_agent.events import ModelRequest
            self._event_bus.emit(ModelRequest(
                exchange_id=f"exchange_{exchange['index']:04d}",
                agent_id=exchange.get("agent_metadata", {}).get("thread_id", ""), phase=phase,
                success=(exchange.get("status") == 200 and not exchange.get("error")
                         and not exchange.get("response_error")) if phase == "finished" else None))

    def stop(self, *, drain_timeout_s: float = 5) -> None:
        # Stop model I/O before draining ASGI workers. Adapter upload/session
        # cleanup remains in lifespan, after all request workers have finished.
        self._client.cancel()
        if self._server is None:
            self._client.close()
            return
        try:
            self._server.stop(timeout_s=drain_timeout_s)
        except TimeoutError as exc:
            # Retain the host so a later stop can finish draining and freeze the traces.
            raise TimeoutError(
                "Provider gateway still has active request(s); model traces are not frozen"
            ) from exc
        self._server = None

    def persist_report(self, report: dict[str, Any]) -> None:
        self._trace.persist_report(report)


class _GatewayResponse(Response):
    """Commit response bindings only after the final ASGI body send succeeds."""

    def __init__(self, gateway, request, exchange, reply, *, status, headers, body, complete):
        outgoing = {
            key: value for key, value in headers.items()
            if key.lower() not in _HOP_BY_HOP_HEADERS | {"content-encoding"}
        }
        # HTTPX decodes content encodings; provider requests ask for identity encoding.
        super().__init__(content=body, status_code=status, headers=outgoing)
        self.gateway, self.request = gateway, request
        self.exchange, self.reply, self.complete = exchange, reply, complete

    async def __call__(self, scope, receive, send):
        async def checked_send(message):
            # Uvicorn can silently ignore sends on a closed connection. Poll before
            # each write, while response_complete is false, to revoke lost batches.
            if await self.request.is_disconnected():
                raise ConnectionResetError("Client disconnected before response delivery")
            await send(message)

        try:
            await super().__call__(scope, receive, checked_send)
        except (Exception, asyncio.CancelledError) as exc:
            # Shutdown cancellation must finish revocation and trace persistence too.
            with anyio.CancelScope(shield=True):
                await run_in_threadpool(self._failed, exc, traceback.format_exc())
            if isinstance(exc, asyncio.CancelledError):
                raise
        else:
            with anyio.CancelScope(shield=True):
                await run_in_threadpool(self._written)
        finally:
            if self.exchange is not None:
                self.exchange["finished_at"] = time.time()
                with anyio.CancelScope(shield=True):
                    await run_in_threadpool(self.gateway._trace.finish, self.exchange)

    def _failed(self, error, error_traceback):
        if self.exchange is not None:
            self.exchange["client_disconnected"] = isinstance(error, ConnectionError)
            self.exchange["delivery_error"] = f"{type(error).__name__}: {error}"
            if not isinstance(error, ConnectionError):
                self.exchange["error"] = self.exchange["delivery_error"]
                self.exchange["error_traceback"] = error_traceback
        if self.reply is not None and self.gateway._model_binding:
            self.gateway._model_binding.response_failed(self.reply)

    def _written(self):
        if self.exchange is not None:
            self.exchange["delivered"] = True
            self.exchange["complete"] = self.complete
        if self.reply is not None and self.gateway._model_binding:
            if self.complete:
                self.gateway._model_binding.response_written(self.reply)
            else:
                self.gateway._model_binding.response_failed(self.reply)


def _decode_json(value: bytes) -> dict[str, Any] | None:
    try:
        decoded = json.loads(value)
    except (TypeError, ValueError):
        return None
    return decoded if isinstance(decoded, dict) else None


def _encode_json(value: dict[str, Any]) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode()




def _header_value(headers: Any, name: str) -> str | None:
    for key, value in headers.items():
        if key.lower() == name:
            return str(value)
    return None
