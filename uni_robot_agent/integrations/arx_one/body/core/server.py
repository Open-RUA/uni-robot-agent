"""``BodyServer``: the shared engine every hardware kind runs behind.

A driver implements :class:`Driver` (read / write / invoke / stream / estop for
its devices); the server adds what every
Body needs: authentication and scopes, exclusive leases, request_key
idempotency, durable Operations executed one-at-a-time per device, ``check``
dry runs, quarantine after unknown outcomes, incremental events and command
streams with a watchdog. ``call(tool, args, principal)`` is the single entry
used by the in-process client and the HTTP facade alike.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
import threading
import time
import traceback
from collections.abc import Callable, Iterable
from pathlib import Path
from typing import Any

from .contract import (
    ANONYMOUS, CONTRACT_VERSION, DeviceError, ErrorCode, Lease, Operation, Principal, SCOPE_ADMIN, SCOPE_LIFECYCLE,
    SCOPE_MAINTENANCE, SCOPE_OBSERVE, SCOPE_OPERATE, fingerprint, make_envelope, new_id, utc_now,
)
from .driver import Driver
from .leases import LeaseTable
from .manifest import BodyManifest, DeviceManifest, validate_manifest
from .ops import Executor, OperationTable
from .streams import EventLog, StreamTable, clamp_targets


#: tool name -> (required scope, is_write)
TOOL_SCOPES: dict[str, tuple[str, bool]] = {
    "get_health": (SCOPE_OBSERVE, False),
    "describe": (SCOPE_OBSERVE, False),
    "read_state": (SCOPE_OBSERVE, False),
    "read_observation_at": (SCOPE_OBSERVE, False),
    "check": (SCOPE_OBSERVE, False),
    "get_operation": (SCOPE_OBSERVE, False),
    "list_operations": (SCOPE_OBSERVE, False),
    "lookup_request": (SCOPE_OBSERVE, False),
    "read_events": (SCOPE_OBSERVE, False),
    "list_leases": (SCOPE_OBSERVE, False),
    "acquire_control": (SCOPE_OPERATE, True),
    "renew_control": (SCOPE_OPERATE, True),
    "release_control": (SCOPE_OPERATE, True),
    "quiesce_control": (SCOPE_OPERATE, True),
    "write": (SCOPE_OPERATE, True),
    "invoke": (SCOPE_OPERATE, True),
    "cancel_operation": (SCOPE_OPERATE, True),
    "stream_open": (SCOPE_OPERATE, True),
    "stream_push": (SCOPE_OPERATE, True),
    "stream_close": (SCOPE_OPERATE, True),
    "estop": (SCOPE_OPERATE, True),
    "acknowledge": (SCOPE_ADMIN, True),
}
PROCEDURE_SCOPE = {"operate": SCOPE_OPERATE, "lifecycle": SCOPE_LIFECYCLE, "maintenance": SCOPE_MAINTENANCE}


def _require(args: dict[str, Any], *names: str) -> None:
    missing = [n for n in names if args.get(n) in (None, "")]
    if missing:
        raise DeviceError(ErrorCode.INVALID_ARGUMENTS, f"missing arguments: {', '.join(missing)}", details={"missing": missing})


class BodyServer:
    def __init__(
        self,
        driver: Driver,
        *,
        tokens: dict[str, Principal] | None = None,
        lease_ttl_s: float = 5.0,
        journal_dir: Path | str | None = None,
        clock: Callable[[], float] = time.monotonic,
        watchdog: bool = True,
    ) -> None:
        self.driver = driver
        self._manifest = driver.manifest()
        validate_manifest(self._manifest)
        self._devices: dict[str, DeviceManifest] = {d.device_id: d for d in self._manifest.devices}
        self._tokens = dict(tokens or {})
        self._clock = clock
        self.backend_epoch = new_id("epoch")
        self.started_at = utc_now()
        self._started_mono = clock()
        self.events = EventLog()
        self.leases = LeaseTable(ttl_s=lease_ttl_s, clock=clock, on_revoke=self._on_lease_revoked)
        journal = Path(journal_dir) / "operations.jsonl" if journal_dir else None
        self.ops = OperationTable(journal_path=journal, on_change=self._on_operation_changed)
        self.executor = Executor()
        self.streams = StreamTable(apply=driver.stream_apply, hold=driver.stream_hold, clock=clock, on_event=self._on_stream_event)
        for device in self._manifest.devices:
            self.streams.register(device)
        self._quarantine: dict[str, set[str]] = {}
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self._quiesced_controls: set[str] = set()
        self._ack_path = Path(journal_dir) / "acknowledgements.json" if journal_dir else None
        self._acknowledgements = self._load_acknowledgements()
        acknowledged = {
            (record["device_id"], record["operation_id"])
            for record in self._acknowledgements
        }
        # Operations that were in flight at the last shutdown start quarantined.
        for op in self.ops.list(states=["unknown"], limit=1000)[0]:
            for device in op.device_ids:
                if (device, op.operation_id) not in acknowledged:
                    self._quarantine.setdefault(device, set()).add(op.operation_id)
        self._watchdog_thread: threading.Thread | None = None
        if watchdog:
            self._watchdog_thread = threading.Thread(target=self._watchdog_loop, name="body-watchdog", daemon=True)
            self._watchdog_thread.start()

    # -- public ------------------------------------------------------------------------------

    @property
    def manifest(self) -> BodyManifest:
        return self._manifest

    def principal_for_token(self, token: str | None) -> Principal:
        if not self._tokens:
            # No token table: local trusted mode (in-process fakes, loopback tests).
            return Principal("local", frozenset({SCOPE_OBSERVE, SCOPE_OPERATE, SCOPE_LIFECYCLE, SCOPE_MAINTENANCE, SCOPE_ADMIN}))
        principal = self._tokens.get(token or "")
        if principal is None:
            raise DeviceError(ErrorCode.UNAUTHORIZED, "unknown token")
        return principal

    def call(self, tool: str, args: dict[str, Any] | None, principal: Principal | None = None) -> dict[str, Any]:
        """Dispatch one contract call and always return an envelope."""
        args = dict(args or {})
        principal = principal or ANONYMOUS
        trace_id = new_id("trace")
        meta = self._meta(trace_id)
        try:
            scope, _is_write = TOOL_SCOPES.get(tool, (None, False))
            if scope is None:
                raise DeviceError(ErrorCode.UNKNOWN_TOOL, f"unknown tool {tool!r}", details={"tools": sorted(TOOL_SCOPES)})
            # A closing server still accepts estop: shutdown is when a stop is most likely needed.
            if _is_write and self._closed.is_set() and tool != "estop":
                raise DeviceError(ErrorCode.RESOURCE_BUSY, "Body server is closed")
            if not principal.allows_scope(scope):
                raise DeviceError(ErrorCode.FORBIDDEN_SCOPE, f"{tool} needs scope {scope!r}", details={"scopes": sorted(principal.scopes)})
            handler = getattr(self, f"_t_{tool}")
            data = handler(args, principal, trace_id)
            return make_envelope(data, meta)
        except DeviceError as exc:
            return make_envelope({}, meta, error=exc)
        except Exception as exc:  # pragma: no cover - defensive
            error = DeviceError(ErrorCode.INTERNAL_ERROR, f"{type(exc).__name__}: {exc}", details={"traceback": traceback.format_exc()[-2000:]}, execution_outcome="unknown")
            return make_envelope({}, meta, error=error)

    def close(self) -> None:
        self._closed.set()
        for stream_id in list(self.streams.snapshot()):
            try:
                self.streams.close(stream_id=stream_id)
            except Exception:
                pass
        for op in self.ops.active():
            try:
                self.ops.cancel(op.operation_id)
            except Exception:
                self._quarantine_devices(op)
        self.executor.join_all(timeout_s=5.0)
        self.leases.revoke_all("server_closed")

    # -- helpers -----------------------------------------------------------------------------

    def _meta(self, trace_id: str) -> dict[str, Any]:
        return {"trace_id": trace_id, "server_time": utc_now(), "backend_epoch": self.backend_epoch, "capability_revision": self._manifest.capability_revision}

    def _device(self, device_id: Any) -> DeviceManifest:
        manifest = self._devices.get(str(device_id))
        if manifest is None:
            raise DeviceError(ErrorCode.UNKNOWN_DEVICE, f"unknown device {device_id!r}", details={"device_ids": list(self._devices)})
        return manifest

    def _device_ids(self, args: dict[str, Any], principal: Principal) -> list[str]:
        ids = args.get("device_ids")
        if ids is None and args.get("device_id") is not None:
            ids = [args["device_id"]]
        if not ids:
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "device_id or device_ids is required")
        out = []
        for device_id in ids:
            self._device(device_id)
            if not principal.allows_device(str(device_id)):
                raise DeviceError(ErrorCode.FORBIDDEN_DEVICE, f"principal may not use {device_id}", details={"device_id": device_id})
            out.append(str(device_id))
        return list(dict.fromkeys(out))

    def _guard_write(self, device_ids: Iterable[str], *, allow_stream: bool = False) -> None:
        self.ops.require_writable()
        for device_id in device_ids:
            quarantined = self._quarantine.get(device_id)
            if quarantined:
                raise DeviceError(ErrorCode.QUARANTINED, "device quarantined after an operation with unknown outcome", details={"device_id": device_id, "operation_ids": sorted(quarantined)})
            if not allow_stream:
                stream = self.streams.for_device(device_id)
                if stream is not None and stream.state != "closed":
                    raise DeviceError(ErrorCode.STREAM_CONFLICT, "a command stream is open on the device", details={"device_id": device_id, "stream_id": stream.stream_id})

    def _on_lease_revoked(self, lease: Lease, reason: str) -> None:
        # Cancel active work and close streams that rode on this lease.
        for op in self.ops.active():
            if op.control_id == lease.control_id or (op.control_id is None and op.principal == lease.principal and set(op.device_ids) & set(lease.device_ids)):
                try:
                    self.ops.cancel(op.operation_id)
                except Exception:
                    self._quarantine_devices(op)
        try:
            self.streams.close_for_control(lease.control_id)
        except Exception:
            pass
        self.events.emit("lease_" + reason, device_ids=lease.device_ids, control_id=lease.control_id, principal=lease.principal)

    def _on_operation_changed(self, op: Operation) -> None:
        self.events.emit("operation_" + op.state, device_ids=op.device_ids, operation_id=op.operation_id, kind=op.kind)

    def _on_stream_event(self, type_: str, stream: Any) -> None:
        self.events.emit(type_, device_ids=[stream.device_id], stream_id=stream.stream_id)

    def _watchdog_loop(self) -> None:
        while not self._closed.wait(0.05):
            try:
                self.streams.watchdog()
                self.leases.sweep()
            except Exception:  # pragma: no cover
                pass

    # -- read tools --------------------------------------------------------------------------

    def _t_get_health(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        try:
            driver_health = self.driver.health()
        except Exception as exc:
            driver_health = {"ok": False, "error": str(exc)}
        return {
            "ok": bool(driver_health.get("ok", True)),
            "body_id": self._manifest.body_id,
            "kind_family": self._manifest.kind_family,
            "contract_version": CONTRACT_VERSION,
            "capability_revision": self._manifest.capability_revision,
            "backend_epoch": self.backend_epoch,
            "started_at": self.started_at,
            "uptime_s": round(self._clock() - self._started_mono, 3),
            "devices": list(self._devices),
            "leases": len(self.leases.active()),
            "active_operations": len(self.ops.active()),
            "open_streams": len(self.streams.snapshot()),
            "quarantined": {k: sorted(v) for k, v in self._quarantine.items() if v},
            "driver": driver_health,
            "runtime": dict(getattr(self, "runtime_info", {})),
        }

    def refresh_manifest(self) -> None:
        """Re-read the driver manifest (calibration and availability change at runtime)."""
        manifest = self.driver.manifest()
        validate_manifest(manifest)
        with self._lock:
            self._manifest = manifest
            self._devices = {d.device_id: d for d in manifest.devices}
            for device in manifest.devices:
                self.streams.register(device)

    def _t_describe(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        try:
            self.refresh_manifest()
        except Exception:  # keep serving the last good manifest if the driver hiccups
            pass
        data = self._manifest.to_dict()
        wanted = args.get("device_ids")
        if wanted:
            data["devices"] = [d for d in data["devices"] if d["device_id"] in set(wanted)]
        for device in data["devices"]:
            device_id = device["device_id"]
            holder = self.leases.holder(device_id)
            device["control"] = None if holder is None else {"control_id": holder.control_id, "principal": holder.principal}
            device["quarantined"] = sorted(self._quarantine.get(device_id, ()))
            stream = self.streams.for_device(device_id)
            # ``stream`` stays the manifest's StreamSpec; live status goes next to it.
            device["active_stream"] = None if stream is None else {"stream_id": stream.stream_id, "state": stream.state}
        return data

    def _t_read_state(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        device_ids = self._device_ids(args, principal)
        include = args.get("include")
        freshness = args.get("freshness") or {}
        on_stale = freshness.get("on_stale", "report")
        max_age = freshness.get("max_age_s")
        devices: dict[str, Any] = {}
        stale = []
        for device_id in device_ids:
            try:
                reading = self.driver.read(device_id, include)
            except DeviceError as exc:
                devices[device_id] = {"availability": "error", "error": exc.to_dict()}
                continue
            except Exception as exc:
                devices[device_id] = {"availability": "error", "error": DeviceError(ErrorCode.HARDWARE_ERROR, str(exc)).to_dict()}
                continue
            reading = dict(reading)
            reading.setdefault("availability", "available")
            reading.setdefault("freshness", "unknown")
            if max_age is not None and reading.get("age_s") is not None and float(reading["age_s"]) > float(max_age):
                reading["freshness"] = "stale"
            if reading.get("freshness") == "stale":
                stale.append(device_id)
            devices[device_id] = reading
        if stale and on_stale == "error":
            raise DeviceError(ErrorCode.OBSERVATION_STALE, "stale feedback", details={"device_ids": stale})
        return {"observation_id": new_id("obs"), "assembled_at": utc_now(), "data_origin": getattr(self.driver, "data_origin", "real"), "devices": devices}

    def _t_read_observation_at(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        from .observations import validate_query

        _require(args, "device_id", "stream", "at")
        # Resolve exactly the device passed to the driver, never an alternate device_ids ACL.
        device_id = self._device_ids({"device_id": args["device_id"]}, principal)[0]
        stream = args["stream"]
        if not isinstance(stream, str):
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "stream must be a string")
        reader = getattr(self.driver, "read_observation_at", None)
        if stream not in self._devices[device_id].observation_streams or not callable(reader):
            raise DeviceError(ErrorCode.NOT_SUPPORTED, "device does not expose this observation stream")
        mode, max_gap_s = args.get("mode", "bracket"), args.get("max_gap_s", 0.05)
        validate_query(args["at"], mode, max_gap_s)
        include = args.get("include")
        allowed = self._devices[device_id].observation_streams[stream].signals
        if include is not None and (not isinstance(include, list) or any(not isinstance(s, str) or s not in allowed for s in include)):
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "include must name signals declared by the observation stream")
        result = reader(device_id, stream, args["at"], mode=mode, max_gap_s=max_gap_s, include=include)
        return {**result, "device_id": device_id, "stream": stream, "body_id": self._manifest.body_id,
                "backend_epoch": self.backend_epoch, "assembled_at": utc_now(),
                "data_origin": getattr(self.driver, "data_origin", "real")}

    def _t_check(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        device_ids = self._device_ids(args, principal)
        out: dict[str, Any] = {}
        for device_id in device_ids:
            try:
                result = dict(self.driver.check(device_id))
            except Exception as exc:
                result = {"readiness": "unknown", "checks": {"driver": {"state": "fail", "reason": str(exc)}}}
            result["checks"] = dict(result.get("checks", {}))
            result["checks"]["quarantine"] = {"state": "fail" if self._quarantine.get(device_id) else "pass", "reason": "operations with unknown outcome" if self._quarantine.get(device_id) else ""}
            holder = self.leases.holder(device_id)
            result["control"] = None if holder is None else {"control_id": holder.control_id, "principal": holder.principal}
            if any(c.get("state") == "fail" for c in result["checks"].values()):
                result["readiness"] = "unsatisfied"
            result.setdefault("readiness", "unknown")
            out[device_id] = result
        return {"devices": out, "evidence_semantics": "read_only_check_not_motor_feedback_or_physical_safety", "captured_at": utc_now()}

    def _t_get_operation(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        _require(args, "operation_id")
        return self.ops.get(str(args["operation_id"])).to_dict()

    def _t_list_operations(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        ops, cursor = self.ops.list(device_ids=args.get("device_ids"), states=args.get("states"), limit=int(args.get("limit", 20)), cursor=args.get("cursor"))
        return {"operations": [op.to_dict() for op in ops], "next_cursor": cursor}

    def _t_lookup_request(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        _require(args, "request_key")
        op = self.ops.lookup_request(principal.name, str(args["request_key"]))
        lease = None
        if op is None:
            for candidate in self.leases.active():
                if candidate.principal == principal.name and candidate.request_key == args["request_key"]:
                    lease = candidate
        return {"found": op is not None or lease is not None, "request_key": args["request_key"], "result": op.to_dict() if op else (lease.to_dict(self._clock()) if lease else {})}

    def _t_read_events(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        events, cursor = self.events.read(cursor=args.get("cursor"), device_ids=args.get("device_ids"), types=args.get("types"), limit=int(args.get("limit", 50)), wait_s=float(args.get("wait_s", 0)))
        return {"events": events, "next_cursor": cursor}

    def _t_list_leases(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        return {"leases": self.leases.snapshot(), "streams": self.streams.snapshot()}

    # -- control -----------------------------------------------------------------------------

    def _t_acquire_control(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        _require(args, "request_key")
        device_ids = self._device_ids(args, principal)
        lease = self.leases.acquire(principal.name, device_ids, str(args["request_key"]))
        self.events.emit("lease_acquired", device_ids=device_ids, control_id=lease.control_id, principal=principal.name)
        return lease.to_dict(self._clock())

    def _t_renew_control(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        _require(args, "control_id")
        return self.leases.renew(principal.name, str(args["control_id"])).to_dict(self._clock())

    def _t_release_control(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        _require(args, "control_id")
        lease = self.leases.release(principal.name, str(args["control_id"]))
        return lease.to_dict(self._clock())

    def _t_quiesce_control(self, args, principal, trace_id):
        """Close one lease's motion admission, cancel its work and request hold.

        This does not release the lease, disable motors, or certify a stop.
        """
        _require(args, "control_id")
        control_id = str(args["control_id"])
        errors = {}
        with self._lock:
            lease = self.leases.require(principal.name, control_id)
            self._quiesced_controls.add(control_id)
            cancelled = []
            for op in self.ops.active():
                if op.control_id == control_id:
                    try:
                        self.ops.cancel(op.operation_id)
                    except DeviceError as exc:
                        errors[op.operation_id] = str(exc)
                    cancelled.append(op.operation_id)
            try:
                self.streams.close_for_control(control_id)
            except Exception as exc:
                errors["streams"] = str(exc)
        held = []
        for device_id in lease.device_ids:
            try:
                self.driver.stream_hold(device_id)
                held.append(device_id)
            except Exception as exc:
                errors[device_id] = str(exc)
        return {"control_id": control_id, "device_ids": list(lease.device_ids),
                "cancelled_operations": cancelled, "hold_requested": held,
                "errors": errors, "physical_stop_guaranteed": False}

    def _require_motion_admission(self, control_id):
        if self._closed.is_set():
            raise DeviceError(ErrorCode.RESOURCE_BUSY, "Body server is closed")
        if control_id in self._quiesced_controls:
            raise DeviceError(ErrorCode.CONTROL_NOT_OWNED,
                              "control session is quiesced; new motion is rejected")

    # -- writes ------------------------------------------------------------------------------

    def _submit(self, *, kind: str, args: dict[str, Any], device_ids: list[str], principal: Principal, trace_id: str, completion: str, run: Callable[[Operation, threading.Event], dict[str, Any]]) -> dict[str, Any]:
        with self._lock:
            _require(args, "control_id", "request_key")
            self._require_motion_admission(str(args["control_id"]))
            request_key = str(args["request_key"])
            fp = fingerprint(kind, args)
            existing = self.ops.existing(principal.name, request_key, fp)
            if existing is not None:
                return {**existing.to_dict(), "replayed": True}
            self.leases.require(principal.name, str(args["control_id"]), device_ids)
            self._guard_write(device_ids)
            op = self.ops.create(kind=kind, args=args, device_ids=device_ids, principal=principal.name, request_key=request_key, fingerprint=fp, completion_semantics=completion, trace_id=trace_id)
            cancel = self.ops.cancel_event(op.operation_id)

            def execute() -> None:
                if cancel.is_set():
                    self.ops.mark_cancelled(op.operation_id, partial=False)
                    return
                # The lease must still be alive when the physical action starts.
                try:
                    with self._lock:
                        self._require_motion_admission(str(args["control_id"]))
                        self.leases.require(principal.name, str(args["control_id"]), device_ids)
                        # Earlier work may have quarantined a device while this
                        # operation waited for its execution lock.
                        self._guard_write(device_ids)
                        self.ops.start(op.operation_id)
                except DeviceError as exc:
                    self.ops.fail(op.operation_id, exc)
                    return
                try:
                    result = run(op, cancel)
                except DeviceError as exc:
                    if exc.execution_outcome == "unknown":
                        self._quarantine_devices(op)
                    self.ops.fail(op.operation_id, exc)
                    return
                except Exception as exc:
                    error = DeviceError(ErrorCode.OUTCOME_UNKNOWN, f"driver raised {type(exc).__name__}: {exc}", execution_outcome="unknown", details={"traceback": traceback.format_exc()[-2000:]})
                    self._quarantine_devices(op)
                    self.ops.fail(op.operation_id, error)
                    return
                if cancel.is_set():
                    self.ops.mark_cancelled(op.operation_id, partial=bool(result.get("partial", True)) if isinstance(result, dict) else True)
                    return
                try:
                    self.ops.finish(op.operation_id, result if isinstance(result, dict) else {"value": result})
                except DeviceError:
                    self._quarantine_devices(op)

            def execute_guarded() -> None:
                try:
                    execute()
                except DeviceError as exc:
                    # Persistence may fail on start/cancel/finish. The table
                    # already records unknown; never continue into the driver.
                    if exc.execution_outcome == "unknown":
                        self._quarantine_devices(op)
                    else:
                        raise

            self.executor.submit(op.operation_id, device_ids, execute_guarded)
            return op.to_dict()

    def _quarantine_devices(self, op: Operation) -> None:
        for device_id in op.device_ids:
            self._quarantine.setdefault(device_id, set()).add(op.operation_id)
        self.events.emit("quarantined", device_ids=op.device_ids, operation_id=op.operation_id)

    def _t_write(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        _require(args, "device_id", "command")
        device_ids = self._device_ids({"device_id": args["device_id"]}, principal)
        manifest = self._device(device_ids[0])
        command = manifest.commands.get(str(args["command"]))
        if command is None:
            raise DeviceError(ErrorCode.UNKNOWN_COMMAND, f"unknown command {args['command']!r}", details={"commands": sorted(manifest.commands)})
        raw = args.get("value")
        if raw is None:
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "value is required")
        values = [float(v) for v in (raw if isinstance(raw, (list, tuple)) else [raw])]
        if not all(math.isfinite(v) for v in values):
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "command values must be finite")
        if command.shape and len(values) != command.shape[0]:
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, f"{command.name}: expected {command.shape[0]} values", details={"got": len(values)})
        clamped, was_clamped = clamp_targets(command, values, None, None, None)
        if was_clamped:
            raise DeviceError(ErrorCode.LIMIT_VIOLATION, f"{command.name}: value outside declared limits", details={"value": values, "minimum": command.minimum, "maximum": command.maximum})

        def run(op: Operation, cancel: threading.Event) -> dict[str, Any]:
            return self.driver.write(device_ids[0], command.name, clamped, cancel=cancel)

        return self._submit(kind=f"write:{command.name}", args=args, device_ids=device_ids, principal=principal, trace_id=trace_id, completion="command_applied", run=run)

    def _t_invoke(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        _require(args, "procedure")
        device_ids = self._device_ids(args, principal)
        name = str(args["procedure"])
        procedures = [self._device(d).procedures.get(name) for d in device_ids]
        if any(p is None for p in procedures):
            missing = [d for d, p in zip(device_ids, procedures) if p is None]
            raise DeviceError(ErrorCode.UNKNOWN_PROCEDURE, f"procedure {name!r} not offered by {missing}", details={"device_ids": missing})
        procedure = procedures[0]
        assert procedure is not None
        needed = PROCEDURE_SCOPE.get(procedure.permission, SCOPE_OPERATE)
        if not principal.allows_scope(needed):
            raise DeviceError(ErrorCode.FORBIDDEN_SCOPE, f"{name} needs scope {needed!r}", details={"scopes": sorted(principal.scopes)})
        params = dict(args.get("params") or {})
        missing_params = [p for p in procedure.required if p not in params]
        if missing_params:
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, f"{name}: missing params {missing_params}", details={"required": procedure.required})

        def run(op: Operation, cancel: threading.Event) -> dict[str, Any]:
            return self.driver.invoke(list(device_ids), name, params, cancel=cancel, progress=lambda v: self.ops.progress(op.operation_id, v))

        return self._submit(kind=f"invoke:{name}", args=args, device_ids=device_ids, principal=principal, trace_id=trace_id, completion=procedure.completion, run=run)

    def _t_cancel_operation(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        _require(args, "operation_id", "control_id")
        op = self.ops.get(str(args["operation_id"]))
        if op.principal != principal.name and SCOPE_ADMIN not in principal.scopes:
            raise DeviceError(ErrorCode.CONTROL_NOT_OWNED, "operation belongs to another principal", details={"operation_id": op.operation_id})
        self.leases.require(principal.name, str(args["control_id"]), op.device_ids)
        if op.control_id and op.control_id != str(args["control_id"]) and SCOPE_ADMIN not in principal.scopes:
            raise DeviceError(ErrorCode.CONTROL_NOT_OWNED, "operation belongs to a different control session")
        if op.is_terminal:
            return {**op.to_dict(), "cancel_requested": False, "reason": "already terminal"}
        op = self.ops.cancel(op.operation_id)
        return {**op.to_dict(), "cancel_requested": True, "physical_stop_guaranteed": False}

    # -- streams -----------------------------------------------------------------------------

    def _t_stream_open(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        with self._lock:
            _require(args, "device_id", "control_id")
            self._require_motion_admission(str(args["control_id"]))
            device_ids = self._device_ids({"device_id": args["device_id"]}, principal)
            self.leases.require(principal.name, str(args["control_id"]), device_ids)
            self._guard_write(device_ids, allow_stream=True)
            if self.executor.busy(device_ids[0]) or self.ops.active(device_ids[0]):
                raise DeviceError(ErrorCode.RESOURCE_BUSY, "an Operation is running on the device; wait or cancel it before streaming", details={"device_id": device_ids[0]})
            stream = self.streams.open(principal=principal.name, control_id=str(args["control_id"]), device_id=device_ids[0], rate_hz=float(args.get("rate_hz", 30.0)), envelope=args.get("envelope"))
            return stream.to_dict()

    def _t_stream_push(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        _require(args, "stream_id")
        stream = self.streams._streams.get(str(args["stream_id"]))
        if stream is None:
            raise DeviceError(ErrorCode.STREAM_CLOSED, "unknown stream", details={"stream_id": args["stream_id"]})
        self.leases.require(principal.name, stream.control_id, [stream.device_id])
        self._require_motion_admission(stream.control_id)
        self._guard_write([stream.device_id], allow_stream=True)
        return self.streams.push(stream_id=stream.stream_id, principal=principal.name, seq=int(args.get("seq", stream.seq + 1)), targets=dict(args.get("targets") or {}))

    def _t_stream_close(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        _require(args, "stream_id")
        stream = self.streams.close(stream_id=str(args["stream_id"]), principal=None if SCOPE_ADMIN in principal.scopes else principal.name)
        return stream.to_dict()

    # -- safety ------------------------------------------------------------------------------

    def _load_acknowledgements(self) -> list[dict[str, str]]:
        if self._ack_path is None or not self._ack_path.exists():
            return []
        try:
            data = json.loads(self._ack_path.read_text())
            records = data["records"]
            if data["body_id"] != self._manifest.body_id or not isinstance(records, list):
                raise ValueError("acknowledgement identity/format mismatch")
            if any(not isinstance(r, dict) or any(
                not isinstance(r.get(key), str) or not r[key]
                for key in ("device_id", "operation_id", "principal", "note", "at")
            ) for r in records):
                raise ValueError("invalid acknowledgement record")
            return records
        except (OSError, ValueError, KeyError, TypeError):
            # An unreadable acknowledgement must never release quarantine.
            self.events.emit("acknowledgement_journal_invalid")
            return []

    def _save_acknowledgements(self, records: list[dict[str, str]]) -> None:
        if self._ack_path is None:
            return
        self._ack_path.parent.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(prefix=".ack-", dir=self._ack_path.parent)
        try:
            with os.fdopen(fd, "w") as stream:
                json.dump({"body_id": self._manifest.body_id, "records": records}, stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, self._ack_path)
        finally:
            Path(name).unlink(missing_ok=True)

    def _t_estop(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        device_ids = self._device_ids(args, principal) if (args.get("device_ids") or args.get("device_id")) else list(self._devices)
        results: dict[str, Any] = {}
        for device_id in device_ids:
            # Each step is isolated: a journal or stream failure must not keep
            # driver.estop from running on this or any other device.
            problems = []
            for op in self.ops.active(device_id):
                try:
                    self.ops.cancel(op.operation_id)
                except Exception as exc:
                    self._quarantine_devices(op)
                    problems.append(f"cancel {op.operation_id}: {exc}")
            try:
                self.streams.close_for_device(device_id)
            except Exception as exc:
                problems.append(f"close stream: {exc}")
            try:
                results[device_id] = {"ok": not problems, **(self.driver.estop(device_id) or {})}
            except Exception as exc:
                problems.append(f"driver estop: {exc}")
                results[device_id] = {"ok": False}
            if problems:
                results[device_id]["errors"] = problems
        self.events.emit("estop", device_ids=device_ids, principal=principal.name)
        return {"devices": results, "physical_stop_guaranteed": False, "note": "queued work cancelled and streams closed; vendor execution already issued may continue"}

    def _t_acknowledge(self, args: dict[str, Any], principal: Principal, trace_id: str) -> dict[str, Any]:
        _require(args, "operation_ids", "note")
        device_ids = self._device_ids(args, principal)
        with self._lock:
            records = [
                {"device_id": device_id, "operation_id": op_id,
                 "principal": principal.name, "note": str(args["note"]), "at": utc_now()}
                for device_id in device_ids
                for op_id in sorted(set(args["operation_ids"]))
                if op_id in self._quarantine.get(device_id, set())
            ]
            if records:
                # Persist before clearing; a failed write leaves devices blocked.
                self._save_acknowledgements(self._acknowledgements + records)
                self._acknowledgements.extend(records)
                for record in records:
                    self._quarantine[record["device_id"]].discard(record["operation_id"])
            acknowledged = [r["operation_id"] for r in records]
        self.events.emit("acknowledged", device_ids=device_ids, operation_ids=acknowledged, note=str(args["note"]), principal=principal.name)
        return {"device_ids": device_ids, "acknowledged_operation_ids": sorted(set(acknowledged)), "remaining_quarantine": {d: sorted(self._quarantine.get(d, ())) for d in device_ids}, "old_operations_resumed": False, "physical_safe_confirmed": False}


__all__ = ["BodyServer", "Driver", "TOOL_SCOPES"]
