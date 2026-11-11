"""Body RPC transport: shared call helpers, the HTTP client side and the HTTP server."""

from __future__ import annotations

import base64
import gzip
import json
import threading
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from logging import getLogger as get_logger
from typing import Any, Callable, Protocol

import numpy as np

from .contract import DeviceError, make_envelope, new_id
from .server import BodyServer


__all__ = ["BodyHttpServer", "BodyRpcFacade"]


class RpcError(RuntimeError):
    """Raised when a remote method call returns an error."""

    def __init__(self, method: str, message: str, *, traceback: str | None = None):
        super().__init__(f"{method}: {message}")
        self.method = method
        self.server_traceback = traceback


class RpcClient(Protocol):
    """Generic method-call RPC transport (agent → out-of-process server)."""

    def call(
        self,
        method: str,
        args: tuple = (),
        kwargs: dict | None = None,
        *,
        timeout_s: float | None = None,
    ) -> Any:
        """Invoke a remote method and return its result."""


def make_error_response(exc: Exception) -> dict:
    """Build the error envelope for a caught exception; the traceback stays in the server log."""
    logger.debug("rpc call failed", exc_info=exc)
    return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}


def check_response(response: Any, method: str) -> Any:
    """Validate RPC response envelope; raise ``RpcError`` on failure, return result."""
    if not isinstance(response, dict):
        raise RpcError(method, f"bad response type: {type(response).__name__}")
    if not response.get("ok"):
        raise RpcError(
            method,
            str(response.get("error", "<no error message>")),
            traceback=response.get("traceback"),
        )
    return response.get("result")


DEFAULT_TIMEOUT_S = 30.0

# Opener that ignores HTTP(S)_PROXY: see HttpRpcClient.call.
_DIRECT_OPENER = urllib.request.build_opener(
    urllib.request.ProxyHandler({})
)

logger = get_logger("rpc")
# Requests are small JSON calls; larger bodies are refused before they are read.
MAX_REQUEST_BYTES = 4 * 1024 * 1024


def _from_json(obj: Any) -> Any:
    """Rehydrate ``{"__ndarray__": <b64>, "dtype": ..., "shape": [...]}``
    back into ndarrays. Everything else is passed through unchanged.
    """
    if isinstance(obj, dict):
        if "__ndarray__" in obj and set(obj) <= {"__ndarray__", "dtype", "shape"}:
            raw = base64.b64decode(obj["__ndarray__"])
            arr = np.frombuffer(raw, dtype=obj.get("dtype"))
            # frombuffer returns a read-only view of the base64 bytes; copy
            # so callers can mutate the returned array like they would with
            # a pickle round-tripped one.
            return arr.reshape(obj.get("shape", (-1,))).copy()
        return {k: _from_json(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_from_json(v) for v in obj]
    return obj


class HttpRpcClient:
    """RPC client that talks to an RPC server via HTTP POST.

    Parameters
    ----------
    base_url : str
        Server address, e.g. ``"http://127.0.0.1:8080"``.
    """

    def __init__(self, base_url: str, *, persistent: bool = False) -> None:
        """Initialize with a base URL, e.g. ``"http://127.0.0.1:8080"``."""
        self._base_url = base_url.rstrip("/")
        self._client = None
        if persistent:
            import httpx

            self._client = httpx.Client(
                trust_env=False,
                transport=httpx.HTTPTransport(
                    retries=0,
                    limits=httpx.Limits(max_connections=4, max_keepalive_connections=4),
                ),
            )

    def close(self) -> None:
        if self._client is not None:
            self._client.close()

    def call(
        self,
        method: str,
        args: tuple = (),
        kwargs: dict | None = None,
        *,
        timeout_s: float | None = None,
    ) -> Any:
        """Invoke a remote method via HTTP POST and return the result."""
        payload = {
            "method": method,
            "args": list(args),
            "kwargs": kwargs or {},
        }
        body = json.dumps(payload, cls=_NumpyEncoder).encode("utf-8")
        url = f"{self._base_url}/call"
        request_timeout = timeout_s if timeout_s is not None else DEFAULT_TIMEOUT_S

        if self._client is not None:
            import httpx

            try:
                raw = self._client.post(
                    url, content=body, headers={"Content-Type": "application/json"},
                    timeout=request_timeout,
                ).content
            except httpx.RequestError as exc:
                # Never replay a write after an ambiguous transport failure.
                raise RpcError(method, f"HTTP request failed: {exc}") from exc
            try:
                return check_response(_from_json(json.loads(raw)), method)
            except json.JSONDecodeError as exc:
                raise RpcError(method, f"invalid JSON response: {exc}") from exc

        req = urllib.request.Request(
            url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            # Deliberately proxy-free: Body RPC stays on the local network.
            # The agent process may export HTTP(S)_PROXY for its model API,
            # and urllib's default opener would silently route Body calls
            # into that proxy (NO_PROXY cannot express IP ranges).
            with _DIRECT_OPENER.open(req, timeout=request_timeout) as resp:
                raw = resp.read()
        except urllib.error.HTTPError as exc:
            # HTTPError is an OSError subclass; catch first so we can parse
            # the ok=False body the server sent alongside the status code.
            raw = exc.read()
        except OSError as exc:
            raise RpcError(method, f"HTTP request failed: {exc}") from exc

        try:
            response = _from_json(json.loads(raw))
        except json.JSONDecodeError as exc:
            raise RpcError(method, f"invalid JSON response: {exc}") from exc

        return check_response(response, method)


# ---------------------------------------------------------------------------
#  Server
# ---------------------------------------------------------------------------


class _NumpyEncoder(json.JSONEncoder):
    """JSON encoder that tags numpy arrays and normalizes numpy scalars."""

    def default(self, obj: Any) -> Any:
        if isinstance(obj, np.ndarray):
            return {
                "__ndarray__": base64.b64encode(obj.tobytes()).decode("ascii"),
                "dtype": str(obj.dtype),
                "shape": list(obj.shape),
            }
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, (np.bool_,)):
            return bool(obj)
        return super().default(obj)


class _HttpRpcHandler(BaseHTTPRequestHandler):
    """Handles POST /call with JSON-RPC body, dispatches to server.dispatch()."""

    protocol_version = "HTTP/1.1"
    disable_nagle_algorithm = True
    timeout = 15

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
        return

    def do_POST(self) -> None:
        if self.path != "/call":
            self.send_response(404)
            self.send_header("Content-Length", "0")
            self.send_header("Connection", "close")
            self.close_connection = True
            self.end_headers()
            return

        try:
            content_length = int(self.headers.get("Content-Length", 0))
        except ValueError:
            content_length = -1
        if not 0 <= content_length <= MAX_REQUEST_BYTES:
            # Refuse before reading: the caller is not authenticated yet.
            self.send_response(413)
            self.send_header("Content-Length", "0")
            self.send_header("Connection", "close")
            self.close_connection = True
            self.end_headers()
            return
        body = self.rfile.read(content_length) if content_length > 0 else b""
        try:
            request = json.loads(body)
            method = request["method"]
            args = tuple(_from_json(v) for v in request.get("args", []))
            kwargs = {k: _from_json(v) for k, v in request.get("kwargs", {}).items()}
            result = self.server.dispatch(method, args, kwargs)  # type: ignore[attr-defined]
            response: dict = {"ok": True, "result": result}
        except Exception as exc:
            response = make_error_response(exc)

        data = json.dumps(response, cls=_NumpyEncoder).encode("utf-8")
        compressed = False
        for option in self.headers.get("Accept-Encoding", "").lower().split(","):
            parts = [p.strip() for p in option.split(";")]
            if parts[0] != "gzip":
                continue
            try:
                quality = next((float(p[2:]) for p in parts[1:] if p.startswith("q=")), 1.0)
            except ValueError:
                quality = 0.0
            if quality > 0 and len(data) >= 16384:
                data = gzip.compress(data, compresslevel=1, mtime=0)
                compressed = True
            break
        # Always 200; failures are described inside the body via ok=False.
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Vary", "Accept-Encoding")
        if compressed:
            self.send_header("Content-Encoding", "gzip")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError) as exc:
            # Client went away mid-response — log for visibility and move on.
            logger.debug("rpc http write failed: %s", exc)


class HttpRpcServer(ThreadingHTTPServer):
    """HTTP server that dispatches JSON-RPC calls posted to /call."""

    allow_reuse_address = True
    daemon_threads = True
    # Listen backlog. The stdlib default (5) overflows when many clients connect
    # in the same instant; the kernel then resets connections.
    request_queue_size = 256

    def __init__(
        self,
        server_address: tuple[str, int],
        dispatch: Callable[[str, tuple, dict], Any],
    ) -> None:
        super().__init__(server_address, _HttpRpcHandler)
        self.dispatch = dispatch


class BodyRpcFacade:
    def __init__(self, server: BodyServer) -> None:
        self.server = server

    def _dispatch(self, method: str, args: tuple, kwargs: dict) -> Any:
        if method != "call":
            raise ValueError(f"unknown RPC method: {method!r} (only 'call')")
        tool = args[0] if args else kwargs.pop("tool", None)
        payload = dict(kwargs)
        token = payload.pop("_token", None)
        try:
            principal = self.server.principal_for_token(token)
        except DeviceError as exc:
            # Authentication failures are contract errors, not transport errors.
            return make_envelope({}, self.server._meta(new_id("trace")), error=exc)
        return self.server.call(str(tool), payload, principal)


class BodyHttpServer:
    """Run the facade on a background thread (tests and in-process bodies)."""

    def __init__(self, server: BodyServer, *, host: str = "127.0.0.1", port: int = 0) -> None:

        self.facade = BodyRpcFacade(server)

        def dispatch(method: str, args: tuple, kwargs: dict) -> Any:
            if method == "healthz":
                return {"status": "ok"}
            return self.facade._dispatch(method, args, kwargs)

        self._httpd = HttpRpcServer((host, port), dispatch)
        self._thread = threading.Thread(target=self._httpd.serve_forever, name="body-http", daemon=True)
        self._lifecycle_lock = threading.RLock()
        self._closed = False
        host_bound, port_bound = self._httpd.server_address[:2]
        self.url = f"http://{'127.0.0.1' if host_bound == '0.0.0.0' else host_bound}:{port_bound}"

    def start(self) -> str:
        with self._lifecycle_lock:
            if self._closed:
                raise RuntimeError("Body HTTP server is closed")
            if self._thread.ident is not None:
                raise RuntimeError("Body HTTP server is already started")
            try:
                self._thread.start()
            except BaseException:
                self.stop()
                raise
        return self.url

    def stop(self) -> None:
        with self._lifecycle_lock:
            if self._closed:
                return
            try:
                # BaseServer.shutdown waits forever if serve_forever never ran.
                if self._thread.is_alive():
                    self._httpd.shutdown()
                    self._thread.join()
            finally:
                self._httpd.server_close()
                self._closed = True
