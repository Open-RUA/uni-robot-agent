"""Wire contract shared by Body servers and clients.

Everything here is data: envelopes, error codes with recovery guidance, leases
and Operations. The shapes keep the behaviour verified on real arms; the contract
version records that RUA adds streams and a kind-agnostic manifest.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import json
import uuid
from dataclasses import dataclass, field
from typing import Any

CONTRACT_VERSION = "2.0"

OP_ACCEPTED = "accepted"
OP_RUNNING = "running"
OP_SUCCEEDED = "succeeded"
OP_FAILED = "failed"
OP_CANCELLED = "cancelled"
OP_UNKNOWN = "unknown"
OP_STATES = (OP_ACCEPTED, OP_RUNNING, OP_SUCCEEDED, OP_FAILED, OP_CANCELLED, OP_UNKNOWN)
TERMINAL_STATES = frozenset({OP_SUCCEEDED, OP_FAILED, OP_CANCELLED, OP_UNKNOWN})

SCOPE_OBSERVE = "observe"
SCOPE_OPERATE = "operate"
SCOPE_LIFECYCLE = "lifecycle"
SCOPE_MAINTENANCE = "maintenance"
SCOPE_ADMIN = "admin"
SCOPES = (SCOPE_OBSERVE, SCOPE_OPERATE, SCOPE_LIFECYCLE, SCOPE_MAINTENANCE, SCOPE_ADMIN)


class ErrorCode:
    """Stable machine-readable error codes (strings on the wire)."""

    UNAUTHORIZED = "unauthorized"
    FORBIDDEN_SCOPE = "forbidden_scope"
    FORBIDDEN_DEVICE = "forbidden_device"
    UNKNOWN_TOOL = "unknown_tool"
    UNKNOWN_DEVICE = "unknown_device"
    UNKNOWN_COMMAND = "unknown_command"
    UNKNOWN_PROCEDURE = "unknown_procedure"
    INVALID_ARGUMENTS = "invalid_arguments"
    RESOURCE_BUSY = "resource_busy"
    CONTROL_EXPIRED = "control_expired"
    CONTROL_NOT_OWNED = "control_not_owned"
    IDEMPOTENCY_CONFLICT = "idempotency_conflict"
    OPERATION_NOT_FOUND = "operation_not_found"
    OPERATION_FINISHED = "operation_finished"
    OBSERVATION_STALE = "observation_stale"
    QUARANTINED = "quarantined"
    CALIBRATION_MISSING = "calibration_missing"
    PRECONDITION_FAILED = "precondition_failed"
    LIMIT_VIOLATION = "limit_violation"
    TRACKING_ERROR = "tracking_error"
    PATH_TRUNCATED = "path_truncated"
    HARDWARE_ERROR = "hardware_error"
    NOT_SUPPORTED = "not_supported"
    STREAM_CONFLICT = "stream_conflict"
    STREAM_CLOSED = "stream_closed"
    OUTCOME_UNKNOWN = "outcome_unknown"
    INTERNAL_ERROR = "internal_error"
    TRANSPORT_ERROR = "transport_error"


#: Recovery guidance per error code, returned to clients as ``next_step``.
NEXT_STEPS: dict[str, str] = {
    ErrorCode.UNAUTHORIZED: "the principal is unknown; fix the token or config before retrying",
    ErrorCode.FORBIDDEN_SCOPE: "this principal lacks the scope; observe-only sessions cannot write",
    ErrorCode.FORBIDDEN_DEVICE: "this principal is not allowed on the device; choose another device or principal",
    ErrorCode.UNKNOWN_DEVICE: "call describe to list device ids",
    ErrorCode.INVALID_ARGUMENTS: "fix the arguments; nothing was sent to hardware",
    ErrorCode.RESOURCE_BUSY: "another controller or stream owns the device; wait, or release_control from that owner",
    ErrorCode.CONTROL_EXPIRED: "the lease lapsed; acquire_control again before writing (no automatic retry of physical actions)",
    ErrorCode.CONTROL_NOT_OWNED: "use the control_id returned by your own acquire_control",
    ErrorCode.IDEMPOTENCY_CONFLICT: "the request_key was used with different arguments; use a new request_key",
    ErrorCode.OPERATION_NOT_FOUND: "check the operation_id; use lookup_request with the request_key if the response was lost",
    ErrorCode.OBSERVATION_STALE: "feedback is stale; check the device and re-read before commanding",
    ErrorCode.QUARANTINED: "an earlier operation ended with unknown outcome; an administrator must acknowledge it after inspecting the robot",
    ErrorCode.CALIBRATION_MISSING: "record the gripper calibration before using opening fractions",
    ErrorCode.PRECONDITION_FAILED: "the device is not ready; run check and fix the reported item",
    ErrorCode.LIMIT_VIOLATION: "preflight rejected the path; nothing was sent, choose a reachable pose or joint configuration instead of retrying the same unreachable target",
    ErrorCode.TRACKING_ERROR: "motion stopped after measured pose deviated; inspect the robot and replan, do not automatically retry or release a held object",
    ErrorCode.PATH_TRUNCATED: "the verified reachable prefix was executed; replan from the returned pose to the remaining target, defer dependent grasp/release actions and do not retry the same unreachable pose",
    ErrorCode.HARDWARE_ERROR: "the driver reported a hardware fault; read diagnostics before retrying",
    ErrorCode.NOT_SUPPORTED: "this device kind does not support the command or procedure",
    ErrorCode.STREAM_CONFLICT: "a command stream is open on the device; close it before discrete writes",
    ErrorCode.STREAM_CLOSED: "the stream was closed or stalled; open a new stream",
    ErrorCode.OUTCOME_UNKNOWN: "the physical outcome is unknown; observe the robot before any further action",
    ErrorCode.INTERNAL_ERROR: "server bug; capture the trace_id and report it",
    ErrorCode.TRANSPORT_ERROR: "the server was unreachable; check the URL and that the Body server is running",
}


class DeviceError(Exception):
    """An expected request failure with a stable code."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        details: dict[str, Any] | None = None,
        stage: str | None = None,
        execution_outcome: str = "none",
        recovery: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.details = details
        self.stage = stage
        self.execution_outcome = execution_outcome
        self.recovery = recovery

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {"code": self.code, "message": self.message, "execution_outcome": self.execution_outcome}
        if self.details is not None:
            out["details"] = self.details
        if self.stage is not None:
            out["stage"] = self.stage
        recovery = self.recovery
        if recovery is None and self.code in NEXT_STEPS:
            recovery = {"action": NEXT_STEPS[self.code]}
        if recovery is not None:
            out["recovery"] = recovery
        return out

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> DeviceError:
        return cls(
            str(data.get("code", ErrorCode.INTERNAL_ERROR)),
            str(data.get("message", "")),
            details=data.get("details"),
            stage=data.get("stage"),
            execution_outcome=str(data.get("execution_outcome", "none")),
            recovery=data.get("recovery"),
        )


def utc_now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="milliseconds")


def new_id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex[:16]}"


def new_request_key() -> str:
    return str(uuid.uuid4())


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), default=str)


def fingerprint(kind: str, args: dict[str, Any]) -> str:
    """Intent fingerprint used for request_key idempotency (control_id excluded)."""
    intent = {k: v for k, v in args.items() if k not in ("control_id", "request_key", "_token")}
    return hashlib.sha256((kind + "\n" + canonical_json(intent)).encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class Principal:
    """Who is calling: a name, granted scopes and (optionally) allowed device ids."""

    name: str
    scopes: frozenset[str] = frozenset({SCOPE_OBSERVE})
    device_ids: frozenset[str] | None = None  # None = every device of the body

    def allows_scope(self, scope: str) -> bool:
        return scope in self.scopes or SCOPE_ADMIN in self.scopes

    def allows_device(self, device_id: str) -> bool:
        return self.device_ids is None or device_id in self.device_ids

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "scopes": sorted(self.scopes), "device_ids": None if self.device_ids is None else sorted(self.device_ids)}

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Principal:
        devices = data.get("device_ids")
        return cls(str(data["name"]), frozenset(data.get("scopes", [SCOPE_OBSERVE])), None if devices is None else frozenset(devices))


ANONYMOUS = Principal("anonymous", frozenset())


def make_envelope(
    data: Any,
    meta: dict[str, Any],
    *,
    status: str = "ok",
    error: DeviceError | dict[str, Any] | None = None,
    warnings: list[Any] | None = None,
) -> dict[str, Any]:
    out: dict[str, Any] = {"contract_version": CONTRACT_VERSION, "status": status, "data": data if data is not None else {}, "meta": meta}
    if error is not None:
        out["error"] = error.to_dict() if isinstance(error, DeviceError) else dict(error)
        out["status"] = "error"
    if warnings:
        out["warnings"] = list(warnings)
    return out


def unwrap(envelope: Any, *, tool: str = "") -> Any:
    """Return ``data`` of an ok/partial envelope; raise :class:`DeviceError` otherwise."""
    if not isinstance(envelope, dict) or "status" not in envelope:
        raise DeviceError(ErrorCode.TRANSPORT_ERROR, f"malformed envelope for {tool or 'call'}: {type(envelope).__name__}")
    if envelope.get("status") == "error":
        error = envelope.get("error") or {"code": ErrorCode.INTERNAL_ERROR, "message": "error without body"}
        exc = DeviceError.from_dict(error)
        meta = envelope.get("meta") or {}
        if isinstance(exc.details, dict):
            exc.details.setdefault("trace_id", meta.get("trace_id"))
        elif exc.details is None and meta.get("trace_id"):
            exc.details = {"trace_id": meta.get("trace_id")}
        raise exc
    return envelope.get("data", {})


@dataclass
class Lease:
    control_id: str
    principal: str
    device_ids: tuple[str, ...]
    ttl_s: float
    expires_at: float
    generation: int
    created_at: str = field(default_factory=utc_now)
    state: str = "active"  # active | expired | released | revoked
    request_key: str | None = None

    @property
    def heartbeat_interval_s(self) -> float:
        return max(0.05, self.ttl_s / 4.0)

    def to_dict(self, now: float | None = None) -> dict[str, Any]:
        out = {
            "control_id": self.control_id,
            "principal": self.principal,
            "device_ids": list(self.device_ids),
            "state": self.state,
            "ttl_s": self.ttl_s,
            "expires_at": self.expires_at,
            "heartbeat_interval_s": self.heartbeat_interval_s,
            "generation": self.generation,
            "created_at": self.created_at,
        }
        if now is not None:
            out["remaining_s"] = round(max(0.0, self.expires_at - now), 3)
        return out


@dataclass
class Operation:
    operation_id: str
    kind: str
    device_ids: tuple[str, ...]
    request_key: str
    principal: str
    command: dict[str, Any]
    completion_semantics: str = "command_stream_completed"
    state: str = OP_ACCEPTED
    execution_outcome: str = "none"  # none | partial | completed | unknown
    result: dict[str, Any] | None = None
    error: dict[str, Any] | None = None
    progress: dict[str, Any] | None = None
    created_at: str = field(default_factory=utc_now)
    started_at: str | None = None
    ended_at: str | None = None
    trace_id: str | None = None
    fingerprint: str = ""
    control_id: str | None = None
    children: list[str] = field(default_factory=list)
    parent_id: str | None = None

    @property
    def is_terminal(self) -> bool:
        return self.state in TERMINAL_STATES

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "operation_id": self.operation_id,
            "kind": self.kind,
            "device_ids": list(self.device_ids),
            "request_key": self.request_key,
            "principal": self.principal,
            "state": self.state,
            "completion_semantics": self.completion_semantics,
            "execution_outcome": self.execution_outcome,
            "command": self.command,
            "created_at": self.created_at,
            "started_at": self.started_at,
            "ended_at": self.ended_at,
            "trace_id": self.trace_id,
            "children": list(self.children),
            "parent_id": self.parent_id,
            "control_id": self.control_id,
            # Software completion is never physical confirmation.
            "physical_confirmed": False,
        }
        if self.result is not None:
            out["result"] = self.result
        if self.error is not None:
            out["error"] = self.error
        if self.progress is not None:
            out["progress"] = self.progress
        return out

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Operation:
        return cls(
            operation_id=str(data["operation_id"]),
            kind=str(data.get("kind", "")),
            device_ids=tuple(data.get("device_ids", ())),
            request_key=str(data.get("request_key", "")),
            principal=str(data.get("principal", "")),
            command=dict(data.get("command") or {}),
            completion_semantics=str(data.get("completion_semantics", "")),
            state=str(data.get("state", OP_UNKNOWN)),
            execution_outcome=str(data.get("execution_outcome", "unknown")),
            result=data.get("result"),
            error=data.get("error"),
            progress=data.get("progress"),
            created_at=str(data.get("created_at", "")),
            started_at=data.get("started_at"),
            ended_at=data.get("ended_at"),
            trace_id=data.get("trace_id"),
            fingerprint=str(data.get("fingerprint", "")),
            control_id=data.get("control_id"),
            children=list(data.get("children", [])),
            parent_id=data.get("parent_id"),
        )


__all__ = [
    "ANONYMOUS",
    "CONTRACT_VERSION",
    "NEXT_STEPS",
    "OP_ACCEPTED",
    "OP_CANCELLED",
    "OP_FAILED",
    "OP_RUNNING",
    "OP_STATES",
    "OP_SUCCEEDED",
    "OP_UNKNOWN",
    "SCOPES",
    "SCOPE_ADMIN",
    "SCOPE_LIFECYCLE",
    "SCOPE_MAINTENANCE",
    "SCOPE_OBSERVE",
    "SCOPE_OPERATE",
    "TERMINAL_STATES",
    "DeviceError",
    "ErrorCode",
    "Lease",
    "Operation",
    "Principal",
    "canonical_json",
    "fingerprint",
    "make_envelope",
    "new_id",
    "new_request_key",
    "unwrap",
    "utc_now",
]
