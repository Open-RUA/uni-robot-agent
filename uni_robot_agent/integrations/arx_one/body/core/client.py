"""Device clients: in-process, HTTP, and the foreground control session.

``DeviceClient.call`` returns unwrapped ``data`` and raises
:class:`~uni_robot_agent.integrations.arx_one.body.core.contract.DeviceError`; helpers cover the fixed tool
set. :class:`ControlSession` renews leases from a foreground thread so
no separate daemon exists: whoever
writes holds and renews the lease and releases it on exit.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Iterable
from typing import Any, Protocol

from .contract import DeviceError, ErrorCode, Principal, TERMINAL_STATES, new_request_key, unwrap
from .manifest import BodyManifest
from .rpc import RpcError


class DeviceClient(Protocol):
    def call(self, tool: str, args: dict[str, Any] | None = None, *, timeout_s: float | None = None) -> Any: ...


class BaseClient:
    """Convenience wrappers shared by every transport."""

    def call(self, tool: str, args: dict[str, Any] | None = None, *, timeout_s: float | None = None) -> Any:  # pragma: no cover - abstract
        raise NotImplementedError

    # reads
    def health(self) -> dict[str, Any]:
        return self.call("get_health")

    def describe(self, device_ids: Iterable[str] | None = None) -> dict[str, Any]:
        return self.call("describe", {"device_ids": list(device_ids)} if device_ids else {})

    def manifest(self) -> BodyManifest:
        return BodyManifest.from_dict(self.describe())

    def read_state(self, device_ids: Iterable[str], include: Iterable[str] | None = None, *, max_age_s: float | None = None, on_stale: str = "report", timeout_s: float | None = None) -> dict[str, Any]:
        args: dict[str, Any] = {"device_ids": list(device_ids)}
        if include:
            args["include"] = list(include)
        if max_age_s is not None or on_stale != "report":
            args["freshness"] = {"on_stale": on_stale, **({"max_age_s": max_age_s} if max_age_s is not None else {})}
        return self.call("read_state", args, timeout_s=timeout_s)

    def check(self, device_ids: Iterable[str]) -> dict[str, Any]:
        return self.call("check", {"device_ids": list(device_ids)})

    def read_observation_at(self, device_id: str, stream: str, at: dict, *, mode: str = "bracket", max_gap_s: float = 0.05, include: Iterable[str] | None = None, wait_s: float = 0.0, timeout_s: float = 2.0) -> dict[str, Any]:
        """Query passive history; only retry a missing later sample, within a budget."""
        import math

        if (type(wait_s) not in (int, float) or not math.isfinite(wait_s) or not 0 <= wait_s <= 2
                or type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 10):
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "wait_s must be in [0, 2]; timeout_s in (0, 10]")
        args = {"device_id": device_id, "stream": stream, "at": at, "mode": mode, "max_gap_s": max_gap_s}
        if include is not None:
            args["include"] = list(include)
        start = time.monotonic()
        deadline = start + timeout_s
        wait_deadline = min(deadline, start + wait_s)
        while True:
            try:
                result = self.call("read_observation_at", args, timeout_s=max(0.001, deadline - time.monotonic()))
            except DeviceError as exc:
                if exc.code != ErrorCode.OBSERVATION_STALE or (exc.details or {}).get("reason") != "pending" or time.monotonic() >= wait_deadline:
                    raise
                time.sleep(min(0.01, max(0.0, wait_deadline - time.monotonic())))
                continue
            if time.monotonic() > deadline:
                raise DeviceError(ErrorCode.OBSERVATION_STALE, "observation query exceeded its time budget")
            return result

    def get_operation(self, operation_id: str, *, timeout_s: float | None = None) -> dict[str, Any]:
        return self.call("get_operation", {"operation_id": operation_id}, timeout_s=timeout_s)

    def list_operations(self, **kwargs: Any) -> dict[str, Any]:
        return self.call("list_operations", kwargs)

    def lookup_request(self, request_key: str) -> dict[str, Any]:
        return self.call("lookup_request", {"request_key": request_key})

    def read_events(self, cursor: str | None = None, **kwargs: Any) -> dict[str, Any]:
        return self.call("read_events", {**({"cursor": cursor} if cursor else {}), **kwargs})

    def wait_operation(self, operation_id: str, timeout_s: float | None = 30.0, poll_s: float = 0.05) -> dict[str, Any]:
        deadline = None if timeout_s is None else time.monotonic() + max(0.0, timeout_s)
        op = None
        while True:
            if op is not None and deadline is not None and time.monotonic() >= deadline:
                return {**op, "timed_out": True}
            # Bound each transport wait by the remaining caller budget. A zero
            # budget still permits one bounded status query, never a motion retry.
            remaining = None if deadline is None else max(0.001, deadline - time.monotonic())
            op = self.get_operation(operation_id, timeout_s=remaining)
            if op.get("state") in TERMINAL_STATES:
                return op
            if deadline is not None and time.monotonic() >= deadline:
                return {**op, "timed_out": True}
            delay = poll_s if deadline is None else min(poll_s, max(0.0, deadline - time.monotonic()))
            time.sleep(delay)

    # control
    def acquire_control(self, device_ids: Iterable[str], request_key: str | None = None) -> dict[str, Any]:
        return self.call("acquire_control", {"device_ids": list(device_ids), "request_key": request_key or new_request_key()})

    def renew_control(self, control_id: str) -> dict[str, Any]:
        return self.call("renew_control", {"control_id": control_id}, timeout_s=2.0)

    def release_control(self, control_id: str) -> dict[str, Any]:
        return self.call("release_control", {"control_id": control_id, "request_key": new_request_key()})

    # writes
    def write(self, device_id: str, command: str, value: Any, *, control_id: str, request_key: str | None = None) -> dict[str, Any]:
        return self.call("write", {"device_id": device_id, "command": command, "value": value, "control_id": control_id, "request_key": request_key or new_request_key()})

    def invoke(self, device_ids: str | Iterable[str], procedure: str, params: dict[str, Any] | None = None, *, control_id: str, request_key: str | None = None) -> dict[str, Any]:
        ids = [device_ids] if isinstance(device_ids, str) else list(device_ids)
        return self.call("invoke", {"device_ids": ids, "procedure": procedure, "params": params or {}, "control_id": control_id, "request_key": request_key or new_request_key()})

    def cancel_operation(self, operation_id: str, *, control_id: str) -> dict[str, Any]:
        return self.call("cancel_operation", {"operation_id": operation_id, "control_id": control_id, "request_key": new_request_key()})

    def estop(self, device_ids: Iterable[str] | None = None) -> dict[str, Any]:
        return self.call("estop", {"device_ids": list(device_ids)} if device_ids else {})

    # streams
    def stream_open(self, device_id: str, *, control_id: str, rate_hz: float = 30.0, envelope: dict[str, Any] | None = None) -> dict[str, Any]:
        return self.call("stream_open", {"device_id": device_id, "control_id": control_id, "rate_hz": rate_hz, "envelope": envelope or {}})

    def stream_push(self, stream_id: str, seq: int, targets: dict[str, Any]) -> dict[str, Any]:
        return self.call("stream_push", {"stream_id": stream_id, "seq": int(seq), "targets": targets}, timeout_s=2.0)

    def stream_close(self, stream_id: str) -> dict[str, Any]:
        return self.call("stream_close", {"stream_id": stream_id})


class InProcessClient(BaseClient):
    """Talks to a :class:`~uni_robot_agent.integrations.arx_one.body.core.server.BodyServer` object directly."""

    def __init__(self, server: Any, principal: Principal | None = None) -> None:
        self.server = server
        self.principal = principal or server.principal_for_token(None)

    def call(self, tool: str, args: dict[str, Any] | None = None, *, timeout_s: float | None = None) -> Any:
        return unwrap(self.server.call(tool, args or {}, self.principal), tool=tool)


class HttpDeviceClient(BaseClient):
    """Talks to a Body server exposed by :mod:`uni_robot_agent.integrations.arx_one.body.core.rpc` over HTTP."""

    def __init__(self, base_url: str, *, token: str | None = None, timeout_s: float = 10.0, persistent: bool = False) -> None:
        from .rpc import HttpRpcClient

        self.base_url = base_url.rstrip("/")
        self.token = token
        self.timeout_s = float(timeout_s)
        self._rpc = HttpRpcClient(self.base_url, persistent=persistent)

    def close(self) -> None:
        self._rpc.close()

    def call(self, tool: str, args: dict[str, Any] | None = None, *, timeout_s: float | None = None) -> Any:
        payload = dict(args or {})
        if self.token:
            payload["_token"] = self.token
        try:
            envelope = self._rpc.call("call", (tool,), payload, timeout_s=self.timeout_s if timeout_s is None else timeout_s)
        except RpcError as exc:
            raise DeviceError(ErrorCode.TRANSPORT_ERROR, f"{tool}: {exc}", details={"url": self.base_url}) from exc
        return unwrap(envelope, tool=tool)


class ControlSession:
    """Foreground lease holder: acquire, renew on a thread, release on close.

    Never re-acquires or replays a write after a failed renewal; a lost lease
    surfaces as ``control_expired`` on the next write so the caller decides.
    """

    def __init__(self, client: BaseClient, *, renew_every_s: float | None = None) -> None:
        self.client = client
        self._renew_every_s = renew_every_s
        self._controls: dict[str, dict[str, Any]] = {}
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None
        self.lost: list[str] = []
        self._quiescing = False

    def __enter__(self) -> ControlSession:
        self.start()
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    def start(self) -> None:
        with self._lock:
            if self._thread is None:
                self._thread = threading.Thread(target=self._heartbeat, name="control-session", daemon=True)
                self._thread.start()

    @property
    def control_ids(self) -> list[str]:
        with self._lock:
            return list(self._controls)

    def control_for(self, device_id: str) -> str | None:
        with self._lock:
            for control_id, lease in self._controls.items():
                if device_id in lease.get("device_ids", []):
                    return control_id
        return None

    def acquire(self, device_ids: Iterable[str], request_key: str | None = None) -> str:
        ids = list(device_ids)
        with self._lock:
            if self._quiescing or self._stop.is_set():
                raise RuntimeError("control session is closing; new acquisitions are rejected")
            existing = [cid for cid, lease in self._controls.items() if set(ids) <= set(lease.get("device_ids", []))]
            if existing:
                return existing[0]
            lease = self.client.acquire_control(ids, request_key)
            self._controls[lease["control_id"]] = lease
        self.start()
        self._wake.set()  # renew on the lease's own schedule from now on
        return str(lease["control_id"])

    def begin_quiesce(self) -> list[str]:
        """Block acquisition and snapshot owned leases while keeping renewal alive."""
        with self._lock:
            self._quiescing = True
            return list(self._controls)

    def release(self, control_id: str) -> None:
        with self._lock:
            self._controls.pop(control_id, None)
        try:
            self.client.release_control(control_id)
        except DeviceError:
            pass

    def _heartbeat(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                items = list(self._controls.items())
            interval = self._renew_every_s
            for control_id, lease in items:
                try:
                    renewed = self.client.renew_control(control_id)
                    with self._lock:
                        if control_id in self._controls:
                            self._controls[control_id] = renewed
                    if interval is None:
                        interval = float(renewed.get("heartbeat_interval_s", 1.0))
                except DeviceError:
                    with self._lock:
                        self._controls.pop(control_id, None)
                    self.lost.append(control_id)
            if not items:
                interval = 0.1
            self._wake.clear()
            self._wake.wait(interval if interval is not None else 1.0)

    def close(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        with self._lock:
            controls = list(self._controls)
            self._controls.clear()
        for control_id in controls:
            try:
                self.client.release_control(control_id)
            except DeviceError:
                pass


__all__ = ["BaseClient", "ControlSession", "DeviceClient", "HttpDeviceClient", "InProcessClient"]
