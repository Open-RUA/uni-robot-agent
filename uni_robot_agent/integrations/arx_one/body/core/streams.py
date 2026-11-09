"""Incremental Body events and watchdog-guarded command streams."""

from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from typing import Any

from .contract import DeviceError, ErrorCode, new_id, utc_now
from .manifest import Command, DeviceManifest


__all__ = ["EventLog", "Stream", "StreamTable", "clamp_targets"]


class EventLog:
    def __init__(self, *, maxlen: int = 20000) -> None:
        self._lock = threading.Condition()
        self._events: deque[dict[str, Any]] = deque(maxlen=maxlen)
        self._seq = 0

    def emit(self, type_: str, *, device_ids: Iterable[str] = (), **payload: Any) -> dict[str, Any]:
        with self._lock:
            self._seq += 1
            event = {"seq": self._seq, "type": type_, "device_ids": list(device_ids), "at": utc_now(), **payload}
            self._events.append(event)
            self._lock.notify_all()
            return event

    def read(
        self,
        *,
        cursor: str | None = None,
        device_ids: Iterable[str] | None = None,
        types: Iterable[str] | None = None,
        limit: int = 50,
        wait_s: float = 0.0,
    ) -> tuple[list[dict[str, Any]], str | None]:
        after = int(cursor) if cursor else 0
        wanted_devices = set(device_ids) if device_ids else None
        wanted_types = set(types) if types else None
        deadline = time.monotonic() + max(0.0, float(wait_s))
        with self._lock:
            while True:
                selected = [
                    e
                    for e in self._events
                    if e["seq"] > after
                    and (wanted_types is None or e["type"] in wanted_types)
                    and (wanted_devices is None or not e["device_ids"] or wanted_devices.intersection(e["device_ids"]))
                ]
                if selected or wait_s <= 0:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._lock.wait(remaining)
            selected = selected[: max(1, int(limit))]
            next_cursor = str(selected[-1]["seq"]) if selected else (str(self._seq) if self._seq else None)
            return selected, next_cursor

    @property
    def size(self) -> int:
        with self._lock:
            return len(self._events)


@dataclass
class Stream:
    stream_id: str
    device_id: str
    control_id: str
    principal: str
    rate_hz: float
    envelope: dict[str, dict[str, Any]]
    opened_at: str = field(default_factory=utc_now)
    opened_mono: float = 0.0
    last_push_mono: float = 0.0
    seq: int = 0
    pushes: int = 0
    clamped: int = 0
    rejected: int = 0
    state: str = "open"  # open | stalled | closed
    last_targets: dict[str, list[float]] = field(default_factory=dict)
    stall_count: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "stream_id": self.stream_id,
            "device_id": self.device_id,
            "control_id": self.control_id,
            "principal": self.principal,
            "rate_hz": self.rate_hz,
            "state": self.state,
            "opened_at": self.opened_at,
            "seq": self.seq,
            "pushes": self.pushes,
            "clamped": self.clamped,
            "rejected": self.rejected,
            "stall_count": self.stall_count,
            "last_targets": self.last_targets,
        }


def _as_list(value: Any, length: int, name: str) -> list[float]:
    if isinstance(value, (int, float)):
        values = [float(value)]
    else:
        try:
            values = [float(v) for v in value]
        except (TypeError, ValueError) as exc:
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, f"{name}: targets must be numbers", details={"command": name}) from exc
    if length and len(values) != length:
        raise DeviceError(ErrorCode.INVALID_ARGUMENTS, f"{name}: expected {length} values, got {len(values)}", details={"command": name})
    for v in values:
        if v != v or v in (float("inf"), float("-inf")):
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, f"{name}: non-finite target", details={"command": name})
    return values


def _bounds(bound: float | list[float] | None, length: int) -> list[float] | None:
    if bound is None:
        return None
    if isinstance(bound, list):
        return [float(b) for b in bound]
    return [float(bound)] * max(1, length)


def clamp_targets(command: Command, values: list[float], previous: list[float] | None, dt_s: float | None, envelope: dict[str, Any] | None) -> tuple[list[float], bool]:
    """Apply declared min/max and per-second delta limits; return (values, clamped)."""
    length = len(values)
    minimum = _bounds(command.minimum, length)
    maximum = _bounds(command.maximum, length)
    max_delta = _bounds(command.max_delta_per_s, length)
    env = envelope or {}
    if env.get("minimum") is not None:
        env_min = _bounds(env["minimum"], length)
        minimum = env_min if minimum is None else [max(a, b) for a, b in zip(minimum, env_min)]
    if env.get("maximum") is not None:
        env_max = _bounds(env["maximum"], length)
        maximum = env_max if maximum is None else [min(a, b) for a, b in zip(maximum, env_max)]
    if env.get("max_delta_per_s") is not None:
        env_delta = _bounds(env["max_delta_per_s"], length)
        max_delta = env_delta if max_delta is None else [min(a, b) for a, b in zip(max_delta, env_delta)]
    out = list(values)
    clamped = False
    for i in range(length):
        v = out[i]
        if minimum is not None and v < minimum[i]:
            v, clamped = minimum[i], True
        if maximum is not None and v > maximum[i]:
            v, clamped = maximum[i], True
        if max_delta is not None and previous is not None and dt_s is not None and dt_s > 0 and i < len(previous):
            limit = max_delta[i] * dt_s
            if abs(v - previous[i]) > limit:
                v = previous[i] + (limit if v > previous[i] else -limit)
                clamped = True
        out[i] = v
    return out, clamped


class StreamTable:
    def __init__(
        self,
        *,
        apply: Callable[[str, dict[str, list[float]]], None],
        hold: Callable[[str], None],
        clock: Callable[[], float] = time.monotonic,
        on_event: Callable[[str, Stream], None] | None = None,
    ) -> None:
        self._apply = apply
        self._hold = hold
        self._clock = clock
        self._on_event = on_event
        self._lock = threading.RLock()
        self._streams: dict[str, Stream] = {}
        self._by_device: dict[str, str] = {}
        self._manifests: dict[str, DeviceManifest] = {}

    def register(self, manifest: DeviceManifest) -> None:
        self._manifests[manifest.device_id] = manifest

    def for_device(self, device_id: str) -> Stream | None:
        with self._lock:
            sid = self._by_device.get(device_id)
            return self._streams.get(sid) if sid else None

    def open(self, *, principal: str, control_id: str, device_id: str, rate_hz: float, envelope: dict[str, Any] | None) -> Stream:
        manifest = self._manifests.get(device_id)
        if manifest is None:
            raise DeviceError(ErrorCode.UNKNOWN_DEVICE, "unknown device", details={"device_id": device_id})
        if manifest.stream is None:
            raise DeviceError(ErrorCode.NOT_SUPPORTED, "device does not support command streams", details={"device_id": device_id})
        rate = float(rate_hz)
        if rate <= 0 or rate > manifest.stream.rate_hz_max:
            raise DeviceError(ErrorCode.LIMIT_VIOLATION, f"rate_hz must be in (0, {manifest.stream.rate_hz_max}]", details={"rate_hz": rate_hz})
        env = dict(envelope or {})
        for name in env:
            if name not in manifest.stream.commands:
                raise DeviceError(ErrorCode.INVALID_ARGUMENTS, f"envelope names unknown stream command {name!r}", details={"commands": manifest.stream.commands})
        with self._lock:
            current = self.for_device(device_id)
            if current is not None and current.state != "closed":
                raise DeviceError(ErrorCode.STREAM_CONFLICT, "a stream is already open on this device", details={"stream_id": current.stream_id})
            now = self._clock()
            stream = Stream(stream_id=new_id("stm"), device_id=device_id, control_id=control_id, principal=principal, rate_hz=rate, envelope=env, opened_mono=now, last_push_mono=now)
            self._streams[stream.stream_id] = stream
            self._by_device[device_id] = stream.stream_id
        self._emit("stream_opened", stream)
        return stream

    def push(self, *, stream_id: str, principal: str, seq: int, targets: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            stream = self._streams.get(stream_id)
            if stream is None or stream.state == "closed":
                raise DeviceError(ErrorCode.STREAM_CLOSED, "stream is closed", details={"stream_id": stream_id})
            if stream.principal != principal:
                raise DeviceError(ErrorCode.CONTROL_NOT_OWNED, "stream belongs to another principal", details={"stream_id": stream_id})
            manifest = self._manifests[stream.device_id]
            assert manifest.stream is not None
            if int(seq) <= stream.seq:
                stream.rejected += 1
                return {"accepted": False, "reason": "stale_seq", "seq": stream.seq}
            now = self._clock()
            dt = now - stream.last_push_mono if stream.pushes else None
            applied: dict[str, list[float]] = {}
            clamped_any = False
            for name, value in targets.items():
                if name not in manifest.stream.commands:
                    raise DeviceError(ErrorCode.UNKNOWN_COMMAND, f"{name!r} is not a stream command", details={"commands": manifest.stream.commands})
                command = manifest.commands[name]
                length = command.shape[0] if command.shape else 0
                values = _as_list(value, length, name)
                values, clamped = clamp_targets(command, values, stream.last_targets.get(name), dt, stream.envelope.get(name))
                clamped_any = clamped_any or clamped
                applied[name] = values
            self._apply(stream.device_id, applied)
            stream.seq = int(seq)
            stream.pushes += 1
            stream.clamped += int(clamped_any)
            stream.last_push_mono = now
            stream.last_targets.update(applied)
            resumed = stream.state == "stalled"
            stream.state = "open"
        if resumed:
            self._emit("stream_resumed", stream)
        return {"accepted": True, "seq": int(seq), "clamped": clamped_any, "applied": applied}

    def close(self, *, stream_id: str, principal: str | None = None) -> Stream:
        with self._lock:
            stream = self._streams.get(stream_id)
            if stream is None:
                raise DeviceError(ErrorCode.STREAM_CLOSED, "unknown stream", details={"stream_id": stream_id})
            if principal is not None and stream.principal != principal:
                raise DeviceError(ErrorCode.CONTROL_NOT_OWNED, "stream belongs to another principal", details={"stream_id": stream_id})
            if stream.state == "closed":
                return stream
            stream.state = "closed"
            if self._by_device.get(stream.device_id) == stream_id:
                del self._by_device[stream.device_id]
        self._hold(stream.device_id)
        self._emit("stream_closed", stream)
        return stream

    def close_for_device(self, device_id: str) -> Stream | None:
        stream = self.for_device(device_id)
        if stream is None or stream.state == "closed":
            return None
        return self.close(stream_id=stream.stream_id)

    def close_for_control(self, control_id: str) -> list[Stream]:
        with self._lock:
            targets = [s for s in self._streams.values() if s.control_id == control_id and s.state != "closed"]
        return [self.close(stream_id=s.stream_id) for s in targets]

    def watchdog(self) -> list[Stream]:
        """Mark streams without recent pushes as stalled and hold their devices."""
        stalled = []
        with self._lock:
            now = self._clock()
            for stream in self._streams.values():
                if stream.state != "open":
                    continue
                manifest = self._manifests[stream.device_id]
                assert manifest.stream is not None
                if now - stream.last_push_mono > manifest.stream.watchdog_s:
                    stream.state = "stalled"
                    stream.stall_count += 1
                    stalled.append(stream)
        for stream in stalled:
            self._hold(stream.device_id)
            self._emit("stream_stalled", stream)
        return stalled

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {s.stream_id: s.to_dict() for s in self._streams.values() if s.state != "closed"}

    def _emit(self, type_: str, stream: Stream) -> None:
        if self._on_event is not None:
            try:
                self._on_event(type_, stream)
            except Exception:  # pragma: no cover
                pass
