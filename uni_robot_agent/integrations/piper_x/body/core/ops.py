"""Durable Operations, request_key idempotency and per-device serial execution."""

from __future__ import annotations

import json
import os
import threading
import time
from collections.abc import Callable, Iterable
from pathlib import Path
from typing import Any

from .contract import (
    DeviceError, ErrorCode, OP_ACCEPTED, OP_CANCELLED, OP_FAILED, OP_RUNNING, OP_SUCCEEDED, OP_UNKNOWN, Operation,
    TERMINAL_STATES, new_id, utc_now,
)


class OperationTable:
    """In-memory Operation store with an optional JSONL journal.

    On load, non-terminal journaled operations become ``unknown``: the server
    restarted while they were in flight and nobody can prove what happened.
    """

    def __init__(self, *, journal_path: Path | str | None = None, on_change: Callable[[Operation], None] | None = None) -> None:
        self._lock = threading.Condition()
        self._ops: dict[str, Operation] = {}
        self._by_request: dict[tuple[str, str], str] = {}
        self._cancel_events: dict[str, threading.Event] = {}
        self._journal = Path(journal_path) if journal_path else None
        self._journal_error: DeviceError | None = None
        self._on_change = on_change
        if self._journal is not None:
            self._load_journal()

    # -- journal -----------------------------------------------------------------------------

    def _load_journal(self) -> None:
        assert self._journal is not None
        if not self._journal.exists():
            return
        raw = self._journal.read_bytes()
        complete, newline, tail = raw.rpartition(b"\n")
        if tail.strip():
            # A record without its newline is an append interrupted by power loss or a crash.
            # Keep it aside for inspection and continue from the last complete record; that
            # operation is then restored from its earlier record and marked unknown below.
            torn = self._journal.with_name(self._journal.name + ".torn")
            torn.write_bytes(tail)
            self._journal.write_bytes(complete + newline)
        for line_number, line in enumerate((complete + newline).decode().splitlines(), 1):
            line = line.strip()
            if not line:
                continue
            try:
                op = Operation.from_dict(json.loads(line))
            except Exception as exc:
                raise ValueError(f"invalid operation journal record at line {line_number}") from exc
            self._ops[op.operation_id] = op
            self._by_request[(op.principal, op.request_key)] = op.operation_id
        for op in self._ops.values():
            if op.state not in TERMINAL_STATES:
                op.state = OP_UNKNOWN
                op.execution_outcome = "unknown"
                op.ended_at = utc_now()
                op.error = DeviceError(ErrorCode.OUTCOME_UNKNOWN, "server restarted while the operation was in flight", execution_outcome="unknown").to_dict()

    def _journal_locked(self, op: Operation) -> None:
        if self._journal is None:
            return
        self.require_writable()
        try:
            self._journal.parent.mkdir(parents=True, exist_ok=True)
            with self._journal.open("a") as handle:
                handle.write(json.dumps({**op.to_dict(), "fingerprint": op.fingerprint}, default=str) + "\n")
                handle.flush()
                os.fsync(handle.fileno())
        except OSError as exc:
            self._journal_error = DeviceError(
                ErrorCode.OUTCOME_UNKNOWN, f"operation journal is unavailable: {exc}",
                execution_outcome="unknown",
            )
            raise self._journal_error from exc

    def require_writable(self) -> None:
        """A failed journal blocks further motion until the Body is repaired/restarted."""
        if self._journal_error is not None:
            raise self._journal_error

    def _changed_locked(self, op: Operation) -> None:
        try:
            self._journal_locked(op)
        except DeviceError as exc:
            op.state = OP_UNKNOWN
            op.execution_outcome = "unknown"
            op.error = exc.to_dict()
            op.ended_at = utc_now()
            self._lock.notify_all()
            raise
        self._lock.notify_all()
        if self._on_change is not None:
            try:
                self._on_change(op)
            except Exception:  # pragma: no cover
                pass

    # -- creation / idempotency --------------------------------------------------------------

    def existing(self, principal: str, request_key: str, fingerprint: str) -> Operation | None:
        with self._lock:
            op_id = self._by_request.get((principal, request_key))
            if op_id is None:
                return None
            op = self._ops[op_id]
            if op.fingerprint != fingerprint:
                raise DeviceError(ErrorCode.IDEMPOTENCY_CONFLICT, "request_key reused with different arguments", details={"request_key": request_key, "operation_id": op_id})
            return op

    def create(
        self,
        *,
        kind: str,
        args: dict[str, Any],
        device_ids: Iterable[str],
        principal: str,
        request_key: str,
        fingerprint: str,
        completion_semantics: str,
        trace_id: str | None = None,
        parent_id: str | None = None,
    ) -> Operation:
        with self._lock:
            op = Operation(
                operation_id=new_id("op"),
                kind=kind,
                device_ids=tuple(device_ids),
                request_key=request_key,
                principal=principal,
                control_id=args.get("control_id"),
                command={k: v for k, v in args.items() if k not in ("control_id", "request_key", "_token")},
                completion_semantics=completion_semantics,
                trace_id=trace_id,
                fingerprint=fingerprint,
                parent_id=parent_id,
            )
            self._ops[op.operation_id] = op
            self._by_request[(principal, request_key)] = op.operation_id
            self._cancel_events[op.operation_id] = threading.Event()
            self._changed_locked(op)
            return op

    # -- transitions -------------------------------------------------------------------------

    def start(self, op_id: str) -> None:
        with self._lock:
            op = self._ops[op_id]
            if op.state == OP_ACCEPTED:
                op.state = OP_RUNNING
                op.started_at = utc_now()
                self._changed_locked(op)

    def progress(self, op_id: str, value: float, kind: str = "trajectory_time_fraction") -> None:
        with self._lock:
            op = self._ops[op_id]
            op.progress = {"kind": kind, "value": max(0.0, min(1.0, float(value)))}

    def finish(self, op_id: str, result: dict[str, Any] | None) -> None:
        with self._lock:
            op = self._ops[op_id]
            if op.is_terminal:
                return
            op.state = OP_SUCCEEDED
            op.execution_outcome = "completed"
            op.result = result if result is not None else {}
            op.ended_at = utc_now()
            self._changed_locked(op)

    def fail(self, op_id: str, error: DeviceError) -> None:
        with self._lock:
            op = self._ops[op_id]
            if op.is_terminal:
                return
            op.state = OP_UNKNOWN if error.execution_outcome == "unknown" else OP_FAILED
            op.execution_outcome = error.execution_outcome
            op.error = error.to_dict()
            op.ended_at = utc_now()
            self._changed_locked(op)

    def cancel(self, op_id: str) -> Operation:
        with self._lock:
            op = self._ops[op_id]
            event = self._cancel_events.get(op_id)
            if event is not None:
                event.set()
            if op.state == OP_ACCEPTED:
                op.state = OP_CANCELLED
                op.execution_outcome = "none"
                op.ended_at = utc_now()
                self._changed_locked(op)
            return op

    def mark_cancelled(self, op_id: str, partial: bool) -> None:
        with self._lock:
            op = self._ops[op_id]
            if op.is_terminal:
                return
            op.state = OP_CANCELLED
            op.execution_outcome = "partial" if partial else "none"
            op.ended_at = utc_now()
            self._changed_locked(op)

    def cancel_event(self, op_id: str) -> threading.Event:
        with self._lock:
            return self._cancel_events.setdefault(op_id, threading.Event())

    # -- queries -----------------------------------------------------------------------------

    def get(self, op_id: str) -> Operation:
        with self._lock:
            op = self._ops.get(op_id)
            if op is None:
                raise DeviceError(ErrorCode.OPERATION_NOT_FOUND, "unknown operation_id", details={"operation_id": op_id})
            return op

    def lookup_request(self, principal: str, request_key: str) -> Operation | None:
        with self._lock:
            op_id = self._by_request.get((principal, request_key))
            return self._ops.get(op_id) if op_id else None

    def list(self, *, device_ids: Iterable[str] | None = None, states: Iterable[str] | None = None, limit: int = 20, cursor: str | None = None) -> tuple[list[Operation], str | None]:
        wanted_devices = set(device_ids) if device_ids else None
        wanted_states = set(states) if states else None
        with self._lock:
            ops = [
                op
                for op in self._ops.values()
                if (wanted_devices is None or wanted_devices.intersection(op.device_ids)) and (wanted_states is None or op.state in wanted_states)
            ]
        ops.sort(key=lambda o: o.created_at, reverse=True)
        start = int(cursor) if cursor else 0
        page = ops[start : start + max(1, int(limit))]
        next_cursor = str(start + len(page)) if start + len(page) < len(ops) else None
        return page, next_cursor

    def active(self, device_id: str | None = None) -> list[Operation]:
        with self._lock:
            return [op for op in self._ops.values() if not op.is_terminal and (device_id is None or device_id in op.device_ids)]

    def wait(self, op_id: str, timeout_s: float | None) -> Operation:
        deadline = None if timeout_s is None else time.monotonic() + max(0.0, float(timeout_s))
        with self._lock:
            while True:
                op = self.get(op_id)
                if op.is_terminal:
                    return op
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    return op
                self._lock.wait(remaining if remaining is not None else 1.0)


class Executor:
    """Runs Operations on worker threads while serializing per device.

    Device locks are taken in sorted order so multi-device operations never
    deadlock; a device therefore executes one physical Operation at a time.
    """

    def __init__(self) -> None:
        self._locks: dict[str, threading.Lock] = {}
        self._guard = threading.Lock()
        self._threads: dict[str, threading.Thread] = {}

    def _lock_for(self, device_id: str) -> threading.Lock:
        with self._guard:
            return self._locks.setdefault(device_id, threading.Lock())

    def submit(self, op_id: str, device_ids: Iterable[str], fn: Callable[[], None]) -> threading.Thread:
        locks = [self._lock_for(d) for d in sorted(set(device_ids))]

        def run() -> None:
            for lock in locks:
                lock.acquire()
            try:
                fn()
            finally:
                for lock in reversed(locks):
                    lock.release()
                with self._guard:
                    self._threads.pop(op_id, None)

        thread = threading.Thread(target=run, name=f"op-{op_id}", daemon=True)
        with self._guard:
            self._threads[op_id] = thread
        thread.start()
        return thread

    def busy(self, device_id: str) -> bool:
        lock = self._lock_for(device_id)
        if lock.acquire(blocking=False):
            lock.release()
            return False
        return True

    def join_all(self, timeout_s: float = 5.0) -> None:
        with self._guard:
            threads = list(self._threads.values())
        deadline = time.monotonic() + timeout_s
        for thread in threads:
            thread.join(max(0.0, deadline - time.monotonic()))


__all__ = ["Executor", "OperationTable"]
