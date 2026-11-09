"""Body RPC client, control lease, admission and bounded shutdown for one physical episode."""

import base64
import json
import math
import os
import threading
import time
import uuid
from pathlib import Path

import numpy as np

from uni_robot_agent.services.http import HttpTransport


#: Renewal failures where the Body did not answer; anything else ends the control.
TRANSIENT_RENEWAL_ERRORS = {"transport_error", "rpc_error"}


class BodyError(RuntimeError):
    def __init__(self, code, *, outcome="unknown", details=None):
        super().__init__(f"Body {code} (execution outcome: {outcome})")
        self.code, self.outcome, self.details = code, outcome, details or {}


def decode(value):
    if isinstance(value, dict):
        if "__ndarray__" in value:
            dtype = np.dtype(value["dtype"])
            if dtype.kind not in "biuf" or len(value["shape"]) > 4:
                raise ValueError("Invalid Body array type or rank")
            return np.frombuffer(base64.b64decode(value["__ndarray__"], validate=True),
                                 dtype=dtype).reshape(value["shape"]).copy()
        return {k: decode(v) for k, v in value.items()}
    if isinstance(value, list):
        return [decode(x) for x in value]
    return value


class BodyClient:
    def __init__(self, config, *, timeout_s=2., transport=None):
        self._token = os.environ[config.token_env]
        if not self._token:
            raise ValueError("Body token environment variable is empty")
        self.timeout_s = timeout_s
        self.transport = transport or HttpTransport(config.url, timeout_s=timeout_s,
                                                    connect_timeout_s=min(1., timeout_s))

    def call(self, tool, args=None, *, timeout_s=None):
        payload = {"method": "call", "args": [tool],
                   "kwargs": {**(args or {}), "_token": self._token}}
        try:
            reply = self.transport.request("POST", "/call", content=json.dumps(payload, allow_nan=False).encode(),
                headers={"Content-Type": "application/json"},
                timeout_s=self.timeout_s if timeout_s is None else timeout_s).json()
        except Exception:
            # Transport exceptions/remote messages may contain credentials. Keep the
            # public failure sanitized; a missing response cannot prove no execution.
            raise BodyError("transport_error") from None
        if not isinstance(reply, dict) or reply.get("ok") is not True:
            raise BodyError("rpc_error")
        envelope = reply.get("result", {})
        if envelope.get("contract_version") != "2.0":
            raise BodyError("contract_mismatch")
        if envelope.get("status") != "ok":
            error = envelope.get("error") or {}
            raise BodyError(error.get("code", "invalid_envelope"),
                outcome=error.get("execution_outcome", "unknown"), details=error.get("details"))
        return decode(envelope["data"])

    def observation(self, device_id, stream, at, *, mode="bracket", timeout_s=2., include=None, max_gap_s=.05):
        deadline = time.monotonic() + timeout_s
        last_details = {}
        query = {"device_id": device_id, "stream": stream, "at": dict(at),
                 "mode": mode, "max_gap_s": max_gap_s, "timeout_s": timeout_s}
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise BodyError("observation_stale", outcome="none", details={
                    **query, "reason": "query_timeout", "last_error": last_details})
            try:
                reply = self.call("read_observation_at", {"device_id": device_id, "stream": stream,
                    "at": at, "mode": mode, "max_gap_s": .05, **({"include": include} if include is not None else {})}, timeout_s=min(remaining, self.timeout_s))
            except BodyError as exc:
                exc.details = {**query, **exc.details}
                last_details = dict(exc.details)
                if exc.code != "observation_stale" or exc.details.get("reason") != "pending":
                    raise
                time.sleep(min(.01, max(0., deadline - time.monotonic())))
                continue
            if reply.get("at") != at or reply.get("mode") != mode or time.monotonic() > deadline:
                raise BodyError("observation_identity_mismatch")
            return reply["samples"]

    def close(self):
        self.transport.close()


TERMINAL = {"succeeded", "failed", "cancelled", "unknown"}
ACTIVE = ["accepted", "running"]
STATIONARY_SAMPLES = 2


class PreflightBlocked(RuntimeError):
    """All group members settled; at least one was rejected before execution."""
    def __init__(self, operations):
        super().__init__("Body command limits rejected a waypoint before execution")
        self.operations = operations


class OperationInterrupted(BodyError):
    """A terminal failure was observed; the rest of its group still needs stopping."""
    def __init__(self, operation_ids, failed, settled=None):
        super().__init__("operation_failed", outcome=failed.get("execution_outcome", "unknown"),
                         details={"operation": failed})
        self.operation_ids = list(operation_ids)
        self.operations = {**(settled or {}), failed["operation_id"]: failed}


def _stationary_sample(reading, *, boundary, began):
    """Feedback that proves stillness and was published after the stop boundary."""
    age = reading.get("age_s")
    stamp = (reading.get("timing") or {}).get("source_timestamp_ns")
    return (reading.get("freshness") == "fresh" and reading.get("moving") is False
            and not reading.get("error") and type(age) in (float, int) and math.isfinite(age)
            and 0 <= age <= .5 and began - age > boundary and str(stamp).isdecimal())


def _count_stationary(readings, ids, *, boundary, began, previous, counts):
    for device in ids:
        reading = readings[device]
        if not _stationary_sample(reading, boundary=boundary, began=began):
            counts[device] = 0
            continue
        stamp = reading["timing"]["source_timestamp_ns"]
        if int(stamp) > int(previous.get(device, 0)):
            counts[device] += 1
            previous[device] = stamp
    return all(count >= STATIONARY_SAMPLES for count in counts.values())


def confirm_unowned_idle(config, client):
    """Require fresh stillness with no owner and no active work before a new lease."""
    ids = sorted(a.device_id for a in config.arms.values())
    boundary = time.monotonic()
    deadline = boundary + config.stop_timeout_s
    previous, counts = {}, {device: 0 for device in ids}

    def call(method, args):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise BodyError("idle_feedback_unverified")
        return client.call(method, args, timeout_s=min(1., remaining))

    def require_idle():
        data = call("describe", {"device_ids": ids})
        devices = data["devices"]
        if data.get("body_id") != config.arms_body.body_id or sorted(d["device_id"] for d in devices) != ids:
            raise BodyError("idle_identity_mismatch")
        if any((d.get("control") or {}).get("control_id") or d.get("quarantined")
               or (d.get("active_stream") or {}).get("state", "closed") != "closed" for d in devices):
            raise BodyError("idle_device_blocked")
        if call("list_operations", {"device_ids": ids, "states": ACTIVE, "limit": 100})["operations"]:
            raise BodyError("idle_work_active")

    while time.monotonic() < deadline:
        require_idle()
        began = time.monotonic()
        readings = call("read_state", {"device_ids": ids,
            "freshness": {"max_age_s": .5, "on_stale": "error"}})["devices"]
        if _count_stationary(readings, ids, boundary=boundary, began=began, previous=previous, counts=counts):
            require_idle()
            return {"status": "confirmed_idle", "samples": previous, "physical_stop_guaranteed": False}
        time.sleep(.02)
    raise BodyError("idle_feedback_unverified")


class Control:
    def __init__(self, client, device_ids, config, journal):
        self.client, self.ids, self.config = client, list(device_ids), config
        self.control_id = None
        self.blocked = threading.Event()
        self._done = threading.Event()
        self._lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._thread = None
        self._deadline_lock = threading.RLock()
        self.phase = "idle"
        self._deadline = time.monotonic() + config.idle_timeout_s
        self.evidence = None
        self.failure = None
        self._acquisition_unknown = False
        path = Path(journal)
        path.parent.mkdir(parents=True, exist_ok=True)
        # Never resume a physical episode from an earlier process's journal.
        self._journal = path.open("x", encoding="utf-8")

    def record(self, event, **data):
        with self._lock:
            self._journal.write(json.dumps({"event": event, "time_ns": str(time.time_ns()), **data}, allow_nan=False) + "\n")
            self._journal.flush()
            os.fsync(self._journal.fileno())

    def acquire(self):
        key = uuid.uuid4().hex
        self.record("acquire_requested", request_key=key, device_ids=self.ids)
        try:
            lease = self.client.call("acquire_control", {"device_ids": self.ids, "request_key": key})
        except Exception:
            self._acquisition_unknown = True
            # Read-only reconciliation may discover a lost acquisition response.
            # It must never issue a second acquire with a different identity.
            try:
                found = self.client.call("lookup_request", {"request_key": key})
                if found.get("found"):
                    self.control_id = found["result"]["control_id"]
                    self._acquisition_unknown = False
            finally:
                self.blocked.set()
                self.stop("acquisition_uncertain")
            raise
        self.control_id = lease["control_id"]
        try:
            if set(lease["device_ids"]) != set(self.ids):
                raise BodyError("lease_device_mismatch")
            interval = float(lease["heartbeat_interval_s"])
            if not math.isfinite(interval) or not 0 < interval <= 5:
                raise BodyError("lease_interval_invalid")
            self.record("acquired", control_id=self.control_id)
            self._thread = threading.Thread(target=self._watch, args=(interval,),
                                             name="body-control", daemon=True)
            self._thread.start()
        except BaseException:
            self.stop("acquisition_invalid")
            raise

    def _watch(self, interval):
        renew_at = time.monotonic() + interval
        renewed_at = time.monotonic()
        while not self._done.wait(min(.05, interval)):
            now = time.monotonic()
            with self._deadline_lock:
                expired = now >= self._deadline
                if expired:
                    self.failure = ("control_idle_timeout" if self.phase == "idle"
                                    else "control_action_timeout")
                    # Close admission atomically with the timeout decision.
                    self.blocked.set()
            if expired:
                self.stop(self.failure)
                return
            if now < renew_at:
                continue
            try:
                lease = self.client.call("renew_control", {"control_id": self.control_id},
                                         timeout_s=min(interval, self.config.rpc_timeout_s))
                if lease["control_id"] != self.control_id or set(lease["device_ids"]) != set(self.ids):
                    raise BodyError("lease_identity_changed")
                renewed_at = time.monotonic()
                renew_at = renewed_at + interval
            except Exception as error:
                # Only an unanswered renewal is retried; a Body that answers that the
                # lease lapsed or changed has ended this control.
                transient = isinstance(error, (OSError, TimeoutError)) or (
                    isinstance(error, BodyError) and error.code in TRANSIENT_RENEWAL_ERRORS)
                if self._renewal_failed(renewed_at, interval, retry=transient):
                    return
                renew_at = time.monotonic() + interval / 4

    def _renewal_failed(self, renewed_at, interval, *, retry):
        """Renewal is idempotent: an unanswered one is retried within two heartbeat
        intervals of the last success. Returns True once control is treated as lost
        (or the episode already stopped)."""
        # stop() releases the lease after confirming idle, so a renewal racing that
        # release fails; that is not a lost control and must not rewrite the outcome.
        if self._done.is_set() or self.evidence is not None:
            return True
        if retry and time.monotonic() - renewed_at < 2 * interval:
            return False
        self.failure = "control_lost"
        self.stop(self.failure)
        return True

    def require(self):
        if self.blocked.is_set() or not self.control_id:
            raise BodyError(self.failure or "control_admission_closed", outcome="not_started")

    def begin_action(self):
        with self._deadline_lock:
            self.require()
            self.phase = "action"
            self._deadline = time.monotonic() + self.config.action_timeout_s

    def end_action(self):
        with self._deadline_lock:
            self.require()
            self.phase = "idle"
            self._deadline = time.monotonic() + self.config.idle_timeout_s

    def observed(self):
        """A scene capture shows the agent is alive; it extends only the idle deadline."""
        with self._deadline_lock:
            if self.phase == "idle" and not self.blocked.is_set():
                self._deadline = time.monotonic() + self.config.idle_timeout_s

    def action_deadline(self):
        with self._deadline_lock:
            return self._deadline

    def invoke(self, device_id, procedure, params):
        return self.invoke_group([device_id], procedure, params)

    def invoke_group(self, device_ids, procedure, params):
        self.require()
        key = uuid.uuid4().hex
        self.record("invoke_requested", request_key=key, device_ids=device_ids,
                    procedure=procedure, params=params)
        self.require()
        op = self.client.call("invoke", {"device_ids": device_ids, "procedure": procedure,
            "params": params, "control_id": self.control_id, "request_key": key})
        self.record("invoke_accepted", request_key=key, operation_id=op["operation_id"])
        return op["operation_id"]

    def wait_operations(self, operations):
        pending = set(operations)
        settled = {}
        while pending:
            self.require()
            deadline = self.action_deadline()
            if time.monotonic() >= deadline:
                raise BodyError("operation_deadline_exceeded")
            for operation in list(pending):
                op = self.client.call("get_operation", {"operation_id": operation},
                    timeout_s=min(self.config.rpc_timeout_s, max(.001, deadline - time.monotonic())))
                if op["state"] in TERMINAL:
                    error = op.get("error") or {}
                    succeeded = op["state"] == "succeeded" and op.get("execution_outcome") in {"completed", "succeeded"}
                    blocked = (op["state"] == "failed" and op.get("execution_outcome") == "none"
                               and error.get("execution_outcome") == "none"
                               and error.get("stage") == "preflight"
                               and error.get("code") == "limit_violation")
                    if not succeeded and not blocked:
                        self.record("operation_interrupted", operation=op, operation_ids=list(operations))
                        raise OperationInterrupted(operations, op, settled)
                    settled[operation] = op
                    pending.remove(operation)
            if pending:
                self.blocked.wait(.01)
        if any(op["state"] == "failed" for op in settled.values()):
            self.record("waypoint_preflight_blocked", operations=settled)
            raise PreflightBlocked(settled)
        return settled

    def stopped_operations(self, operation_ids):
        """Read every terminal result after quiescence; never replay an invocation."""
        if not self.evidence or self.evidence.get("status") != "confirmed_idle":
            raise BodyError("stop_unconfirmed")
        results = {}
        deadline = time.monotonic() + self.config.rpc_timeout_s
        for operation_id in operation_ids:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise BodyError("operation_receipt_timeout")
            op = self.client.call("get_operation", {"operation_id": operation_id}, timeout_s=remaining)
            if (op.get("operation_id") != operation_id or op.get("state") not in TERMINAL - {"unknown"}
                    or op.get("execution_outcome") not in {"none", "partial", "completed", "succeeded"}
                    or not op.get("device_ids") or not set(op["device_ids"]) <= set(self.ids)
                    or op.get("control_id") != self.control_id):
                raise BodyError("operation_receipt_unverified")
            if ((op["state"] == "succeeded") != (op["execution_outcome"] in {"completed", "succeeded"})
                    or (op["state"] == "failed" and (
                        not op.get("error") or op["error"].get("execution_outcome") != op["execution_outcome"]))):
                raise BodyError("operation_outcome_inconsistent")
            results[operation_id] = op
        if {device for op in results.values() for device in op["device_ids"]} != set(self.ids):
            raise BodyError("operation_group_incomplete")
        self.record("stopped_operation_receipts", operations=results, stop=self.evidence)
        return results

    def stop(self, reason):
        # This path does not acquire the environment motion lock, and Body closes
        # admission before holding. An in-flight request cannot re-open admission.
        self.blocked.set()
        with self._stop_lock:
            if self.evidence is not None:
                return self.evidence
            evidence = {"reason": reason, "status": "unknown", "physical_stop_guaranteed": False}
            deadline = time.monotonic() + self.config.stop_timeout_s

            def rpc(method, args):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("Stop confirmation deadline exceeded")
                return self.client.call(method, args, timeout_s=min(1., remaining))

            try:
                if not self.control_id:
                    evidence["status"] = "unknown" if self._acquisition_unknown else "not_required"
                    return evidence
                result = rpc("quiesce_control", {"control_id": self.control_id})
                if (result.get("control_id") != self.control_id or result.get("errors")
                        or set(result.get("device_ids", [])) != set(self.ids)):
                    raise BodyError("hold_request_incomplete", details={"quiesce_result": result})
                boundary = time.monotonic()
                previous, counts = {}, {d: 0 for d in self.ids}
                while time.monotonic() < deadline:
                    description = rpc("describe", {"device_ids": self.ids})
                    devices = {d["device_id"]: d for d in description["devices"]}
                    if set(devices) != set(self.ids):
                        raise BodyError("stop_missing_device")
                    for d in devices.values():
                        if (d.get("control") or {}).get("control_id") != self.control_id:
                            raise BodyError("stop_ownership_changed")
                        if d.get("quarantined"):
                            raise BodyError("stop_quarantined")
                    ops = rpc("list_operations", {"device_ids": self.ids, "states": ACTIVE, "limit": 100})
                    active = bool(ops["operations"]) or any(
                        (d.get("active_stream") or {}).get("state", "closed") != "closed" for d in devices.values())
                    began = time.monotonic()
                    readings = rpc("read_state", {"device_ids": self.ids,
                        "freshness": {"max_age_s": .5, "on_stale": "error"}})["devices"]
                    if active:
                        counts = {d: 0 for d in self.ids}
                    elif _count_stationary(readings, self.ids, boundary=boundary, began=began,
                                           previous=previous, counts=counts):
                        if self.failure == "control_lost":
                            raise BodyError("control_lost")
                        evidence.update(status="confirmed_idle", samples=previous)
                        break
                    time.sleep(min(.02, max(0., deadline - time.monotonic())))
                # Lease expiry is not stop evidence.
            except Exception as error:
                evidence["error"] = type(error).__name__
                if isinstance(error, BodyError):
                    evidence["body_error"] = {"code": error.code,
                        "execution_outcome": error.outcome, "details": error.details}
            finally:
                self._done.set()
                if self.control_id and evidence["status"] == "confirmed_idle":
                    try:
                        self.client.call("release_control", {"control_id": self.control_id,
                            "request_key": uuid.uuid4().hex}, timeout_s=min(1., self.config.rpc_timeout_s))
                    except Exception:
                        evidence["release_error"] = True
                        evidence["status"] = "unknown"
                self.evidence = evidence
                self.record("stopped", **evidence)
            return evidence

    def close(self):
        self.stop("close")
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=self.config.stop_timeout_s + self.config.rpc_timeout_s + 1.)
        with self._lock:
            if not self._journal.closed:
                self._journal.close()
