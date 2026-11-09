"""Fleet-wide stop acceptance: quiesce every Body of a fleet and confirm its devices are idle."""
from __future__ import annotations

import math
import time


def quiesce(fleet, *, reason, timeout_s=5.0, poll_s=0.05, owned_controls=None, request_stop=True):
    """Block submissions, stop owned work, then require two post-request samples.

    The deadline bounds RPC waits (in-process drivers must implement bounded I/O).
    No estop, controller teardown, new leases or quarantine clearing is performed.
    request_stop=False verifies a completed group cancellation without closing its
    leases. It requires explicit owned controls; it never substitutes for cancel.
    """
    if not math.isfinite(timeout_s) or timeout_s <= 0 or not math.isfinite(poll_s) or poll_s <= 0:
        raise ValueError("positive finite stop timeout and poll interval required")
    if not request_stop and owned_controls is None:
        raise ValueError('idle-only verification requires explicit owned controls')
    started = time.monotonic()
    deadline = started + timeout_s
    evidence = {"reason": reason, "status": "unknown", "controls": {},
                "physical_stop_guaranteed": False}
    owned = []
    lost = {}
    def rpc(body, method, args):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("stop acceptance deadline exceeded")
        return body.client.call(method, args, timeout_s=min(1.0, remaining))
    # A group supplies its already-owned controls; it must not close admission for
    # unrelated devices. Session shutdown still freezes the entire fleet.
    selected = list(owned_controls) if owned_controls is not None else None
    bodies = {id(b): b for b, _ in selected}.values() if selected is not None else fleet.bodies.values()
    for body in bodies:
        controls = [cid for b, cid in selected if b is body] if selected is not None else body.begin_quiesce()
        if body.session is None:
            continue
        lost_ids = set(body.session.lost)
        if selected is not None: lost_ids &= set(controls)
        if lost_ids:
            lost[body.spec.name] = sorted(lost_ids)
        for cid in controls:
            owned.append((body, cid))
    try:
        for body, cid in owned:
            try:
                # Renewal verifies the existing lease and yields its device set;
                # it neither acquires a new lease nor clears a quiesced lease.
                result = rpc(body, "quiesce_control" if request_stop else "renew_control", {"control_id": cid})
                evidence["controls"][cid] = {"body": body.spec.name, "request": result}
                if not result.get("device_ids") or result.get("errors"):
                    evidence["controls"][cid]["error"] = "hold request incomplete"
            except Exception as exc:
                evidence["controls"][cid] = {"body": body.spec.name, "error": str(exc)}
        if any(record.get("error") for record in evidence["controls"].values()):
            evidence["error"] = "one or more stop requests were not accepted"
            return evidence
        if lost:
            evidence.update(error="control lease was lost; stop outcome is unknown", lost_controls=lost)
            return evidence
        feedback_after = time.monotonic()
        # Nothing owned: never touch other sessions' hardware.
        if not owned:
            evidence["status"] = "not_required"
            return evidence
        accepted_samples, previous_samples, sample_ids = {}, {}, {}
        while time.monotonic() < deadline:
            all_idle = True
            for body, cid in owned:
                record = evidence["controls"][cid]
                ids = record["request"]["device_ids"]
                try:
                    description = rpc(body, "describe", {"device_ids": ids})
                    devices = {d["device_id"]: d for d in description["devices"]}
                    if set(devices) != set(ids):
                        raise ValueError("missing owned device description")
                    for d in devices.values():
                        if (d.get("control") or {}).get("control_id") != cid:
                            raise ValueError("control ownership changed during stop")
                        if d.get("quarantined") or (d.get("active_stream") or {}).get("state", "closed") != "closed":
                            raise ValueError("device has quarantine or an open stream")
                    operations = rpc(body, "list_operations", {"device_ids": ids,
                                      "states": ["accepted", "running"], "limit": 100})
                    if operations["operations"]:
                        raise ValueError("owned device still has active operations")
                    read_started = time.monotonic()
                    readings = rpc(body, "read_state", {"device_ids": ids,
                        "freshness": {"max_age_s": 0.5, "on_stale": "error"}})
                    if set(readings["devices"]) != set(ids):
                        raise ValueError("missing owned device feedback")
                    for device_id, reading in readings["devices"].items():
                        age = reading.get("age_s")
                        if (reading.get("freshness") != "fresh" or reading.get("moving") is not False
                                or reading.get("error") or not isinstance(age, (int, float))
                                or not math.isfinite(age) or not 0 <= age <= .5):
                            raise ValueError("feedback is stale, moving or unavailable")
                        sample_time = read_started - age
                        key = (body.spec.name, cid, device_id)
                        if sample_time <= feedback_after:
                            raise ValueError("feedback predates the stop request")
                        timing = reading.get("timing") or {}
                        sample_id = timing.get("source_timestamp_ns")
                        distinct = sample_id is None or sample_ids.get(key) != sample_id
                        if distinct and sample_time > previous_samples.get(key, feedback_after) + .001:
                            accepted_samples[key] = accepted_samples.get(key, 0) + 1
                            previous_samples[key] = sample_time
                            sample_ids[key] = sample_id
                        if accepted_samples.get(key, 0) < 2:
                            all_idle = False
                    record.update(readings=readings, operations=operations, error=None)
                except Exception as exc:
                    record["error"] = str(exc)
                    for device_id in ids:
                        accepted_samples.pop((body.spec.name, cid, device_id), None)
                    all_idle = False
            if all_idle:
                evidence["status"] = "confirmed_idle"
                return evidence
            time.sleep(min(poll_s, max(0, deadline - time.monotonic())))
        evidence["error"] = "stop acceptance timed out"
        return evidence
    except Exception as exc:
        evidence["error"] = str(exc)
        return evidence
    finally:
        evidence['request_stop'] = request_stop
        evidence["elapsed_s"] = round(time.monotonic() - started, 4)
