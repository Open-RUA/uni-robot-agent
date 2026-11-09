"""Robot-independent observation service over an already connected RobotHandle."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, wait, FIRST_COMPLETED
from copy import deepcopy
import math
import threading
import time

from ...body.core.contract import DeviceError
from ...body.core.manifest import KIND_CAMERA
from ...body.core.timing import host_clock_id
from ...perception.extrinsics import profile_id
from ...perception.observation_store import ObservationStore
from ...perception.snapshot import RGBDSnapshot, check_cancel


def _error(exc):
    if isinstance(exc, DeviceError):
        return exc.to_dict()
    return {"code": type(exc).__name__, "message": str(exc)}


class ObservationService:
    """Capture read-only evidence; model processing is a separate, later stage.

    Reads within one request run concurrently. A second request is rejected while
    the first or a timed-out read is still active, so unresponsive adapters cannot
    grow an unbounded executor queue. No control leases are acquired.
    """

    def __init__(self, robot, store: ObservationStore, *, max_workers=8, clock_id=None):
        if type(max_workers) is not int or not 1 <= max_workers <= 32:
            raise ValueError("max_workers must be in 1..32")
        self.robot, self.store = robot, store
        self.clock_id = host_clock_id() if clock_id is None else clock_id
        self._pool = ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="rua-observe")
        self._lock = threading.Lock()
        self._pending = set()
        self._closed = False
        self._closing = threading.Event()
        self._calibrations = {}
        for name, spec in robot.fleet.site.calibrations.items():
            if spec["robot"] == robot.robot_id:
                self._calibrations[name] = (spec["camera"], robot.fleet.site.calibration(name))

    def close(self):
        self._closing.set()
        with self._lock:
            self._closed = True
            self._pool.shutdown(wait=False, cancel_futures=True)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def observe(self, cameras=None, *, timeout_s=3.0, max_age_s=0.5,
                pair_joints=True, max_joint_gap_s=0.05, max_frame_gap_s=0.05,
                wait_s=0.0, cancel=None) -> dict:
        received_ns = str(time.time_ns())
        for key, value, limit in (("timeout_s", timeout_s, 30), ("max_age_s", max_age_s, 30),
                                  ("max_joint_gap_s", max_joint_gap_s, 0.2),
                                  ("max_frame_gap_s", max_frame_gap_s, 0.2)):
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= limit:
                raise ValueError(f"{key} must be in (0, {limit}]")
        if type(pair_joints) is not bool:
            raise ValueError("pair_joints must be boolean")
        if type(wait_s) not in (float, int) or not math.isfinite(wait_s) or not 0 <= wait_s <= 30:
            raise ValueError("wait_s must be in [0, 30]")
        if not isinstance(self.clock_id, str) or not self.clock_id.startswith("linux-boot:") or not self.clock_id[11:]:
            raise ValueError("observation requires the Body host clock; run on that Linux host or supply an explicit test clock")
        roles = list(self.robot.spec.cameras) if cameras is None else cameras
        if not isinstance(roles, (list, tuple)) or not roles or any(not isinstance(r, str) for r in roles):
            raise ValueError("cameras must contain one or more camera roles")
        if len(set(roles)) != len(roles) or any(r not in self.robot.spec.cameras for r in roles):
            raise ValueError("camera roles must be unique and declared by the robot")
        if not self._lock.acquire(blocking=False):
            raise RuntimeError("observation service is busy")
        try:
            if self._closed:
                raise RuntimeError("observation service is closed")
            if any(not f.done() for f in self._pending):
                raise RuntimeError("a previous read is still active")
            return self._observe(list(roles), timeout_s, max_age_s, pair_joints,
                                 max_joint_gap_s, max_frame_gap_s, wait_s, received_ns, cancel)
        finally:
            self._lock.release()

    def _observe(self, roles, timeout_s, max_age_s, pair_joints, max_gap, frame_gap, wait_s, received_ns, external_cancel):
        wait_deadline = time.monotonic() + wait_s
        while time.monotonic() < wait_deadline:
            if self._closing.is_set() or external_cancel is not None and external_cancel.is_set():
                break
            self._closing.wait(min(.02, wait_deadline-time.monotonic()))
        began = time.monotonic()
        deadline = began + timeout_s
        # Receipt and selection target have different meanings. Set the target
        # after the requested wait; every worker receives this exact same value.
        target_ns = str(time.time_ns())
        at = {"source_timestamp_ns": target_ns, "clock_id": self.clock_id, "source_clock": "unix_realtime"}
        oid = self.store.begin({"robot_id": self.robot.robot_id, "cameras": roles,
                                "t_received": received_ns, "t_request": target_ns,
                                "clock_id": self.clock_id, "source_clock": "unix_realtime", "selection": "after",
                                "timeout_s": timeout_s, "wait_s": wait_s,
                                "max_age_s": max_age_s, "pair_joints": pair_joints,
                                "max_joint_gap_s": max_gap,
                                "max_frame_gap_s": frame_gap})
        stop = threading.Event()
        snapshots, errors, readings = {}, {}, {}
        cancelled = False
        futures = {}

        def remaining():
            check_cancel(stop)
            value = deadline - time.monotonic()
            if value <= 0:
                raise TimeoutError("observation read deadline exceeded")
            return value

        def camera_read(role):
            camera = self.robot.cameras[role]
            snap = RGBDSnapshot.capture_at(camera, at, timeout_s=remaining(), max_age_s=max_age_s,
                                           max_gap_s=frame_gap, cancel=stop)
            mount = self.robot.spec.cameras[role]
            snap.metadata.update(observe_id=oid, camera_role=role, camera_device=camera.key,
                                 mount=mount.mount, frame_motion=mount.frame_motion)
            pairs, pair_errors, calibrations = {}, {}, {}
            if pair_joints:
                for side, arm in self.robot.arms.items():
                    try:
                        pairs[side] = snap.joint_state_at_capture(
                            arm, timeout_s=min(0.5, remaining()), max_gap_s=max_gap, cancel=stop)
                    except Exception as exc:
                        pair_errors[side] = _error(exc)
            for name, (bound_role, bound) in self._calibrations.items():
                if bound_role != role:
                    continue
                record = bound.calibration.record
                matches = (snap.metadata["serial"] == bound.camera_serial
                           and snap.metadata["frame_id"] == bound.source_frame
                           and profile_id(snap.intrinsics["color"]) == record["profile_id"])
                calibrations[name] = {
                    "sha256": bound.calibration.sha256, "record": record,
                    "camera_device": bound.camera_device, "target_device": bound.target_device,
                    "observed_identity_matches": matches, "mount_id": bound.mount_id,
                    "mount_evidence": "site_configuration_only",
                    "physical_mount_verified": False,
                }
                if not matches:
                    pair_errors[f"calibration:{name}"] = {"code": "identity_mismatch", "message": "camera identity, frame or profile differs from calibration"}
            snap.metadata.update(joint_states=pairs, joint_errors=pair_errors, calibrations=calibrations)
            if not snap.metadata["source_times"]:
                pair_errors["source_time"] = {"code": "unavailable", "message": "camera source timing is unavailable"}
            remaining()
            return snap

        def state_read(handles):
            result = handles[0].client.read_state([h.device_id for h in handles],
                max_age_s=max_age_s, on_stale="error", timeout_s=remaining())
            remaining()
            return {h.key: result["devices"].get(h.device_id, {"availability": "missing"}) for h in handles}

        try:
            if self._closing.is_set() or external_cancel is not None and external_cancel.is_set():
                cancelled = True
            else:
                for role in roles:
                    if role not in self.robot.cameras:
                        errors[role] = {"code": "unavailable", "message": self.robot.degraded.get(role, "camera unavailable")}
                    else:
                        futures[self._pool.submit(camera_read, role)] = ("camera", role)
                groups = {}
                for handle in self.robot.devices.values():
                    if handle.kind != KIND_CAMERA:
                        groups.setdefault(handle.body.spec.name, []).append(handle)
                for body, handles in groups.items():
                    futures[self._pool.submit(state_read, handles)] = ("state", body)
                pending = set(futures)
                self._pending = pending.copy()
                while pending:
                    if self._closing.is_set() or external_cancel is not None and external_cancel.is_set():
                        cancelled = True
                        break
                    if time.monotonic() >= deadline:
                        break
                    done, pending = wait(pending, timeout=min(0.02, deadline-time.monotonic()), return_when=FIRST_COMPLETED)
                    for future in done:
                        kind, key = futures[future]
                        try:
                            value = future.result()
                            if kind == "camera":
                                snapshots[key] = value
                                if value.metadata["joint_errors"]:
                                    errors[key] = value.metadata["joint_errors"]
                            else:
                                readings.update(value)
                                for device, reading in value.items():
                                    if reading.get("availability") in {"missing", "error"} or reading.get("error") or reading.get("freshness") == "stale":
                                        errors[f"state:{device}"] = reading.get("error") or {"code": "unavailable", "message": "state missing or stale"}
                        except Exception as exc:
                            errors[f"{kind}:{key}"] = _error(exc)
                for future in pending:
                    kind, key = futures[future]
                    errors[f"{kind}:{key}"] = {"code": "cancelled" if cancelled else "timeout", "message": "read not completed within request"}
            stop.set()
            for future in futures:
                future.cancel()
            # Disk commit is outside the device read budget. Its duration is explicit.
            archive_started = time.monotonic()
            state = {"observe_id": oid, "t_request": target_ns,
                     "robot_id": self.robot.robot_id, "current_state": readings,
                     "current_state_is_image_paired": False, "errors": errors,
                     "hardware_synchronized": False, "read_elapsed_s": archive_started-began}
            self.store.pin(oid, snapshots, state)
            status = "cancelled" if cancelled else ("failed" if not snapshots else "partial" if errors else "completed")
            result = {"observe_id": oid, "t_received": received_ns, "t_request": target_ns,
                      "request_clock_id": self.clock_id, "selection": "after", "status": status, "errors": errors,
                      "cameras": {r: self._public_camera(s) for r, s in snapshots.items()},
                      "read_elapsed_s": archive_started-began,
                      "archive_elapsed_s": time.monotonic()-archive_started,
                      "model_processing": "not_requested", "hardware_synchronized": False}
            self.store.finish(oid, result, state=status)
            return result
        except Exception as exc:
            stop.set()
            for future in futures:
                future.cancel()
            # A storage failure must never be returned as a successful observation.
            try:
                self.store.finish(oid, {"observe_id": oid, "error": _error(exc)}, state="failed")
            except Exception:
                pass
            raise

    @staticmethod
    def _public_camera(snap):
        meta = snap.metadata
        return {key: deepcopy(meta.get(key)) for key in (
            "observe_id", "camera_role", "camera_device", "sequence", "serial", "frame_id",
            "source_times", "source_time_status", "age_s_at_receipt", "frame_motion",
            "t_request", "t_query_started", "t_query_finished", "selection_offset_s", "data_origin"
        )} | {"intrinsics": deepcopy(snap.intrinsics), "depth_source": "measured", "paired_arms": list(meta["joint_states"]),
             "calibrations": {name: {k: v for k, v in item.items() if k != "record"}
                              for name, item in meta["calibrations"].items()}}

    def get_observation(self, observe_id, *, include_raw=False):
        return self.store.get(observe_id, include_raw=include_raw)

    def list_observations(self, *, limit=20, before_seq=None):
        return self.store.list(limit=limit, before_seq=before_seq)
