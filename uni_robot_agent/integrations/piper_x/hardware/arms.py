"""One robot's arms share admission, preflight and stop scope in one Body."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from uni_robot_agent.integrations.piper_x.body.core.arm.eef import coordinate_metadata
from uni_robot_agent.integrations.piper_x.body.core.arm.arrival import ArrivalWindow
from uni_robot_agent.integrations.piper_x.body.core.arm.tracking import PoseTrackingGuard, TrackingLimits
from uni_robot_agent.integrations.piper_x.body.core.arm.motion_limits import MotionLimits
from uni_robot_agent.integrations.piper_x.body.core.contract import DeviceError, ErrorCode
from uni_robot_agent.integrations.piper_x.body.core.manifest import BodyManifest, DeviceManifest, Signal, Procedure, ObservationStreamSpec, num
from uni_robot_agent.integrations.piper_x.body.core.observations import ObservationBuffer
from uni_robot_agent.integrations.piper_x.body.core.resources import ResourceLocks
from ..config import Installation
from .identity import verify_can
from .planning import RobotGeometry
from .transport import PassiveCan, SdkWriter
from uni_robot_agent.integrations.piper_x.environment.kinematics import distance, from_eef, bounded_target


class ArmBody:
    data_origin = "physical"

    def __init__(self, deployment, *, body_id, execute=False):
        self.installation = Installation(deployment)
        self.robot = self.installation.robot
        self.body_id = body_id
        self.execute = execute
        self._cleanup = ExitStack()
        self._blocked = threading.Event()
        self._planning_lock = threading.Lock()
        self._dispatch_lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._stop_receipt = None
        self.tracking_limits = TrackingLimits()
        self._readers, self._writers = {}, {}
        self._by_device = {a.device_id: name for name, a in self.robot.arms.items()}
        self._histories = {d: ObservationBuffer() for d in self._by_device}
        self._last_stamp = {}
        self._history_lock = threading.Lock()
        self._closed = threading.Event()
        if self.installation.host.bodies[self.robot.arms_body].body_id != body_id:
            raise ValueError("Wrong arm Body identity")
        if execute:
            self.installation.require_motion()
        try:
            resources = [verify_can(self.installation.host.arms[a.binding]) for a in self.robot.arms.values()]
            locks = ResourceLocks(resources, self.installation.host.lock_dir)
            self._cleanup.callback(locks.close)
            self.geometry = RobotGeometry(self.installation) if execute else None
            for name, arm in self.robot.arms.items():
                reader = PassiveCan(self.installation.host.arms[arm.binding].interface,
                    max_age_s=self.robot.feedback_max_age_s, max_skew_s=self.robot.feedback_max_skew_s)
                self._cleanup.callback(reader.close)
                self._readers[name] = reader
            self._history_thread = threading.Thread(target=self._collect, daemon=True, name=f"history-{body_id}")
            self._history_thread.start()
            self._cleanup.callback(self._end_history)
            if execute:
                for name in self.robot.arms:
                    deadline = time.monotonic() + self.robot.action_timeout_s
                    while True:
                        try:
                            self._require_feedback(name)
                            break
                        except RuntimeError:
                            if time.monotonic() >= deadline:
                                raise
                            self._closed.wait(.01)
                    writer = SdkWriter(self.installation, name)
                    self._cleanup.callback(writer.close)
                    self._writers[name] = writer
        except BaseException:
            self._cleanup.close()
            raise

    def _collect(self):
        while not self._closed.wait(.005):
            for device, name in self._by_device.items():
                try:
                    data = self._readers[name].read()
                    with self._history_lock:
                        for row in data["joint_history"]:
                            stamp = int(row["timing"]["source_timestamp_ns"])
                            if stamp <= self._last_stamp.get(device, 0):
                                continue
                            self._histories[device].append(row["timing"], {"joint_position_rad": row["joint_position_rad"]},
                                generation=1, sequence=stamp)
                            self._last_stamp[device] = stamp
                except Exception:
                    # No fabricated samples: freshness gates and history queries fail closed.
                    continue

    def _end_history(self):
        self._closed.set()
        self._history_thread.join(timeout=1)

    def manifest(self):
        devices = []
        for name, arm in self.robot.arms.items():
            c = self.installation.calibration.arms[name]
            devices.append(DeviceManifest(arm.device_id, "arm", hardware_profile="piper_x",
                frames=[f"{arm.device_id}/base", f"{arm.device_id}/flange"],
                signals={n: Signal(n) for n in ("joint_position_rad", "eef_position_m", "eef_rpy_rad", "gripper_opening", "gripper_opening_measured",
                    "controller_motion_status_code", "controller_target_reached")},
                observation_streams={"state": ObservationStreamSpec(["joint_position_rad"])},
                procedures={
                    "move_eef_group": Procedure("move_eef_group", "Preflight every arm before any dispatch", params={
                        "targets": {"type": "object"}, "max_segments": {"type": "integer", "minimum": 1, "maximum": 24},
                        "segment_m": num(.0001, .06), "max_duration_s": num(.1, 60)},
                        required=["targets", "max_segments", "segment_m", "max_duration_s"], completion="measured_target_arrival")} if self.execute else {},
                metadata={"identity": self.installation.identity(name, "arms"),
                    "joint_limits_rad": {"lower": c.joint_lower, "upper": c.joint_upper},
                    "motion_limits": c.motion_limits.model_dump(), "eef_coordinates": coordinate_metadata(arm.device_id),
                    "tracking_limits": self.tracking_limits.to_dict(),
                    "model_sha256": self.installation.model.urdf.sha256, "motion_enabled": self.execute},
                limitations=["Host CAN receive timestamps; no controller hardware clock", "Software stop is not a certified E-stop",
                    *(["Collision checks cover the URDF only; omitted tooling requires operator supervision"]
                      if self.geometry and self.geometry.collision.scope == "urdf_only" else [])]))
        return BodyManifest(self.body_id, "piper_x", devices)

    def read(self, device_id, include=None):
        name = self._by_device[device_id]
        data = self._readers[name].read()
        c = self.installation.calibration.arms[name]
        width = data["agx_gripper"].get("width_m")
        if width is None or not c.gripper_closed_m - .001 <= width <= c.gripper_open_m + .001:
            raise DeviceError(ErrorCode.OBSERVATION_STALE, "Gripper mode or calibration mismatch")
        opening = float(np.clip((width - c.gripper_closed_m) / (c.gripper_open_m - c.gripper_closed_m), 0, 1))
        return {"signals": {"joint_position_rad": data["joints_rad"], "eef_position_m": data["flange_position_m"],
                "eef_rpy_rad": data["flange_rpy_rad"], "gripper_opening": opening, "gripper_opening_measured": True,
                "controller_motion_status_code": data.get("controller_motion_status_code"),
                "controller_target_reached": data.get("controller_target_reached")},
                "freshness": "fresh", "age_s": data["age_s"], "moving": data["moving"], "timing": data["timing"],
                "fault": data["fault"], "data_origin": self.data_origin}

    def _require_feedback(self, name):
        data = self._readers[name].read()
        if data["fault"] or not data["can_control"] or not data["joint_mode"]:
            raise DeviceError(ErrorCode.PRECONDITION_FAILED, "Commission fault-free CAN joint mode externally; no automatic mode changes")
        return data

    def check(self, device_id):
        try:
            data = self.read(device_id)
            if data["fault"] or self._blocked.is_set():
                raise RuntimeError("Fault or blocked motion")
            if self.execute:
                self._require_feedback(self._by_device[device_id])
            return {"readiness": "satisfied", "checks": {"feedback": {"state": "pass"}}, "motion_enabled": self.execute}
        except Exception as error:
            return {"readiness": "unsatisfied", "checks": {"feedback": {"state": "fail", "error": str(error)}}}

    def _preflight_feedback(self, names, checkpoint):
        """Wait briefly for coherent fresh feedback before any physical write."""
        deadline = time.monotonic() + min(.1, self.robot.feedback_max_age_s)
        while True:
            checkpoint()
            try:
                return {name: self._require_feedback(name) for name in names}
            except DeviceError as error:
                error.stage = "preflight"
                raise
            except RuntimeError as error:
                if time.monotonic() >= deadline:
                    raise DeviceError(ErrorCode.OBSERVATION_STALE,
                        "Fresh coherent feedback unavailable before dispatch: " + str(error),
                        execution_outcome="none", stage="preflight") from error
                time.sleep(.005)

    def _arrival_error(self, name, data, target_q, *, held):
        """Cartesian requests settle at the grasp site; held arms settle in joints."""
        c = self.installation.calibration.arms[name]
        joint_error = float(np.max(np.abs(np.asarray(data["joints_rad"]) - target_q)))
        evidence = {"max_joint_error_rad": joint_error}
        if held:
            evidence["joint_tolerance_rad"] = self.tracking_limits.settle_joint_rad
            return joint_error <= self.tracking_limits.settle_joint_rad, evidence
        offset = np.asarray(c.T_flange_from_grasp)
        measured = from_eef(data["flange_position_m"], data["flange_rpy_rad"]) @ offset
        target = self.geometry.fk(name, target_q) @ offset
        position, rotation = distance(measured, target)
        # Apply the strictest configured profile: a native call cannot loosen
        # the selected tool profile, even though that profile is not sent here.
        position_limit = min(p.position_tolerance_m for p in self.robot.motion_profiles.values())
        rotation_limit = min(p.orientation_tolerance_rad for p in self.robot.motion_profiles.values())
        evidence.update(position_error_m=position, rotation_error_rad=rotation,
                        position_tolerance_m=position_limit, orientation_tolerance_rad=rotation_limit)
        return position <= position_limit and rotation <= rotation_limit, evidence

    @staticmethod
    def _tracking_pose(data):
        return [*data["flange_position_m"], *data["flange_rpy_rad"]]

    def _tracking_target(self, name, joints):
        pose = self.geometry.fk(name, joints)
        return [*pose[:3, 3], *Rotation.from_matrix(pose[:3, :3]).as_euler("xyz")]

    def _check_pose_tracking(self, guards, actual, *, segment):
        for name, data in actual.items():
            stamp = int(data["timing"]["received_monotonic_ns"]) / 1e9
            violation = guards[name].check(stamp, self._tracking_pose(data))
            if violation is not None:
                raise DeviceError(ErrorCode.TRACKING_ERROR, "Sustained end-effector tracking deviation",
                    details={"arm": name, "segment": segment, **violation})

    def health(self):
        return {"ok": not self._closed.is_set(), "robot_id": self.robot.robot_id, "motion_enabled": self.execute}

    def read_observation_at(self, device_id, stream, at, *, mode="bracket", max_gap_s=.05, include=None):
        if stream != "state":
            raise DeviceError(ErrorCode.NOT_SUPPORTED, "Unknown observation stream")
        result = self._histories[device_id].query(at, mode=mode, max_gap_s=max_gap_s)
        for sample in result["samples"]:
            sample["signals"] = sample.pop("payload")
        return result

    def invoke(self, device_ids, procedure, params, *, cancel, progress):
        if not self.execute or self.geometry is None or self._blocked.is_set():
            raise DeviceError(ErrorCode.PRECONDITION_FAILED, "Motion is disabled or blocked")
        if procedure != "move_eef_group" or set(device_ids) != set(self._by_device):
            raise DeviceError(ErrorCode.NOT_SUPPORTED, "Use one coordinated path naming all robot arms")
        if set(params["targets"]) != set(device_ids):
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "Targets differ from invoked arms")
        with self._planning_lock:
            deadline = time.monotonic() + params["max_duration_s"]
            planning_deadline = time.monotonic() + min(8., params["max_duration_s"] / 2)

            def checkpoint():
                if cancel.is_set() or self._blocked.is_set():
                    raise DeviceError(ErrorCode.PRECONDITION_FAILED, "Path cancelled before dispatch", stage="preflight")
                if time.monotonic() >= planning_deadline:
                    raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Path planning time budget exceeded before dispatch", stage="preflight")

            checkpoint()
            before = self._preflight_feedback(self.robot.arms, checkpoint)
            measured = {n: d["joints_rad"] for n, d in before.items()}
            targets = {n: dict(params["targets"][a.device_id]) for n, a in self.robot.arms.items()}
            requested_durations = [target.get("duration_s", .2) for target in targets.values()]
            if any(type(value) not in (float, int) or not np.isfinite(value)
                   or not .1 <= value <= params["max_duration_s"] for value in requested_durations):
                raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "Target duration exceeds the action budget", stage="preflight")
            minimum_duration = max(requested_durations)
            travel_limited = False
            for n, target in targets.items():
                if "hold_joints" not in target and target.get("target_frame") != self.robot.arms[n].device_id + "/base":
                    raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "Wrong native target frame")
                opening = target.get("gripper_opening")
                if type(opening) not in (float, int) or not np.isfinite(opening) or not 0 <= opening <= 1:
                    raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "Expected normalized gripper target")
                if "hold_joints" not in target:
                    start = self.geometry.fk(n, measured[n])
                    goal = from_eef(target["target_position_m"], target["target_rpy_rad"])
                    dp, angle = distance(start, goal)
                    travel = dp + self.installation.calibration.arms[n].rotation_radius_m * angle
                    budget = params["segment_m"] * params["max_segments"]
                    if travel > budget:
                        fraction = budget * .999 / travel
                        goal = bounded_target(start, goal, translation=dp * fraction, rotation=angle * fraction)
                        target["target_position_m"] = goal[:3, 3].tolist()
                        target["target_rpy_rad"] = Rotation.from_matrix(goal[:3, :3]).as_euler("xyz").tolist()
                        travel_limited = True
            try:
                path = self.geometry.plan(measured, targets, segment_m=params["segment_m"],
                                          max_segments=params["max_segments"], checkpoint=checkpoint)
            except DeviceError as error:
                if error.code == ErrorCode.LIMIT_VIOLATION and error.execution_outcome == "none":
                    error.stage = "preflight"
                raise
            checkpoint()
            period = 1 / self.robot.control_frequency_hz
            durations = []
            for a, b in zip(path, path[1:]):
                duration = period
                for name in a:
                    limits = MotionLimits.from_config(self.installation.calibration.arms[name].motion_limits.model_dump())
                    joint_delta = float(np.max(np.abs(b[name] - a[name])))
                    dp, angle = distance(self.geometry.fk(name, a[name]), self.geometry.fk(name, b[name]))
                    if joint_delta > limits.joint_step_rad or dp > limits.eef_step_m or angle > limits.eef_rotation_step_rad:
                        raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Path exceeds station segment limits", stage="preflight")
                    duration = max(duration, 1.875 * joint_delta / limits.joint_speed_rad_s, limits.eef_duration_s(dp, angle))
                durations.append(duration)
            gripper_start = {n: (before[n]["agx_gripper"]["width_m"] - self.installation.calibration.arms[n].gripper_closed_m) /
                            (self.installation.calibration.arms[n].gripper_open_m - self.installation.calibration.arms[n].gripper_closed_m)
                            for n in before}
            gripper_duration = max(abs(targets[n]["gripper_opening"] - gripper_start[n]) /
                                   self.installation.calibration.arms[n].motion_limits.gripper_speed_fraction_s for n in before)
            # Preserve the shared hold/grasp/release duration and bounded gripper slew.
            while sum(durations) < max(gripper_duration, minimum_duration):
                path.append({n: q.copy() for n, q in path[-1].items()})
                durations.append(period)
            # A time-bounded prefix is a new, fully checked command. Its receipt
            # requires a fresh observation and new model plan, never automatic replay.
            available = deadline - time.monotonic() - self.robot.stop_timeout_s
            count, planned_duration = 0, 0.
            for duration in durations:
                if planned_duration + duration > available:
                    break
                planned_duration += duration
                count += 1
            if count == 0:
                raise DeviceError(ErrorCode.LIMIT_VIOLATION, "No checked segment fits the action duration before dispatch", stage="preflight")
            truncated = count < len(durations)
            path, durations = path[:count + 1], durations[:count]
            with self._dispatch_lock:
                checkpoint()
                # Planning can be slow. Do not execute a path from an obsolete start.
                current = self._preflight_feedback(measured, checkpoint)
                if any(np.max(np.abs(np.array(current[n]["joints_rad"]) - measured[n])) >
                       self.tracking_limits.preflight_joint_rad for n in measured):
                    raise DeviceError(ErrorCode.OBSERVATION_STALE, "Measured start changed during preflight; observe and plan a new path", stage="preflight")
                # Every next connection is checked again from measured joints.
                # Admission tolerance is not a continuous collision guarantee.
                self.geometry.check({n: current[n]["joints_rad"] for n in measured})
                submitted = False
                elapsed = 0.
                completed_segments = 0
                residual_prefix = False
                last_target = path[0]
                command_trace = []
                guards = {n: PoseTrackingGuard(self.tracking_limits,
                    int(data["timing"]["received_monotonic_ns"]) / 1e9, self._tracking_pose(data))
                    for n, data in current.items()}
                try:
                    for index, (row, duration) in enumerate(zip(path[1:], durations), 1):
                        if cancel.is_set() or self._blocked.is_set():
                            raise InterruptedError("Robot stop requested")

                        def motion_checkpoint():
                            if cancel.is_set() or self._blocked.is_set() or time.monotonic() >= deadline:
                                raise InterruptedError("Path interrupted before segment dispatch")

                        segment_start = {n: np.asarray(self._require_feedback(n)["joints_rad"]) for n in row}
                        if not self.geometry.check_measured_segment(segment_start, row, checkpoint=motion_checkpoint):
                            if not submitted:
                                raise DeviceError(ErrorCode.LIMIT_VIOLATION,
                                    "Measured start exceeds next command step", stage="preflight")
                            # Do not jump from a lagging arm to an assumed waypoint.
                            # Finish the known prefix and request a fresh plan.
                            residual_prefix = True
                            break
                        # A measured-start connection may be longer than the
                        # nominal segment. Preserve the same command rate limits.
                        for name in row:
                            limits = MotionLimits.from_config(self.installation.calibration.arms[name].motion_limits.model_dump())
                            dp, angle = distance(self.geometry.fk(name, segment_start[name]), self.geometry.fk(name, row[name]))
                            duration = max(duration, 1.875 * float(np.max(np.abs(row[name]-segment_start[name]))) /
                                           limits.joint_speed_rad_s, limits.eef_duration_s(dp, angle))
                        if time.monotonic() + duration + self.robot.stop_timeout_s >= deadline:
                            if not submitted:
                                raise DeviceError(ErrorCode.LIMIT_VIOLATION,
                                    "Measured segment exceeds remaining duration", stage="preflight")
                            truncated = True
                            break
                        for name, q in row.items():
                            if cancel.is_set() or self._blocked.is_set():
                                raise InterruptedError("Stop during group submission")
                            c = self.installation.calibration.arms[name]
                            fraction = min(1., (elapsed + duration) / max(gripper_duration, 1e-9))
                            command = gripper_start[name] + fraction * (targets[name]["gripper_opening"] - gripper_start[name])
                            opening = c.gripper_closed_m + command * (c.gripper_open_m - c.gripper_closed_m)
                            submitted = True  # A thrown write may already have transmitted frames.
                            record = {"arm": name, "segment": index, "sent_realtime_ns": str(time.time_ns()),
                                      "target_joints_rad": q.tolist(), "opening_m": opening,
                                      "measured_start_rad": segment_start[name].tolist(), "send_outcome": "unknown"}
                            command_trace.append(record)
                            guards[name].sent(time.monotonic(), self._tracking_target(name, q))
                            self._writers[name].move(q, opening, c.gripper_effort)
                            record["send_outcome"] = "transport_returned"
                        final_sent_at = time.monotonic()
                        last_target = row
                        end = time.monotonic() + duration
                        while time.monotonic() < end:
                            if cancel.wait(.005) or self._blocked.is_set() or time.monotonic() >= deadline:
                                raise InterruptedError("Path interrupted")
                            actual = {n: self._require_feedback(n) for n in row}
                            self.geometry.check({n: data["joints_rad"] for n, data in actual.items()})
                            self._check_pose_tracking(guards, actual, segment=index)
                        progress(index / (len(path) - 1))
                        elapsed += duration
                        completed_segments += 1
                    windows = {n: ArrivalWindow() for n in measured}
                    while time.monotonic() < deadline:
                        actual = {n: self._require_feedback(n) for n in measured}
                        self.geometry.check({n: data["joints_rad"] for n, data in actual.items()})
                        self._check_pose_tracking(guards, actual, segment=completed_segments)
                        arrivals = {}
                        for n, data in actual.items():
                            stamp = int(data["timing"]["received_monotonic_ns"]) / 1e9
                            within, errors = self._arrival_error(n, data, last_target[n], held="hold_joints" in targets[n])
                            settled = windows[n].update(stamp, stamp >= final_sent_at and
                                data["moving"] is False and within)
                            arrivals[n] = {"verified": settled, **errors,
                                "feedback_samples": windows[n].samples,
                                "controller_target_reached": data.get("controller_target_reached")}
                        if all(row["verified"] for row in arrivals.values()):
                            return {"requested_targets_reached": not (truncated or travel_limited or residual_prefix), "robot_id": self.robot.robot_id,
                                    "prefix_reason": "measured_start_step" if residual_prefix else
                                        "action_duration" if truncated else "travel_budget" if travel_limited else None,
                                    "planned_segments": completed_segments, "planned_duration_s": elapsed,
                                    "arrival": arrivals,
                                    "command_trace": command_trace,
                                    "tracking_limits": self.tracking_limits.to_dict(),
                                    "completion_basis": "consecutive_fresh_target_feedback_after_final_command"}
                        if cancel.wait(.01) or self._blocked.is_set():
                            raise InterruptedError("Interrupted during final arrival")
                    raise TimeoutError("Measured arrival was not verified")
                except Exception as error:
                    if not submitted and isinstance(error, DeviceError):
                        raise
                    stop = self._stop_all() if submitted else {"status": "not_dispatched", "physical_stop_guaranteed": False}
                    raise DeviceError(ErrorCode.OUTCOME_UNKNOWN if submitted else ErrorCode.PRECONDITION_FAILED,
                        str(error), execution_outcome="unknown" if submitted else "none",
                        details={"stop": stop, "cause": getattr(error, "details", {}), "command_trace": command_trace},
                        stage="execution" if submitted else "preflight") from error

    def _stop_all(self):
        self._blocked.set()
        with self._stop_lock:
            if self._stop_receipt is not None:
                return self._stop_receipt
            def stop(item):
                name, writer = item
                try:
                    return name, writer.stop(self._readers[name].read)
                except Exception as error:
                    return name, {"status": "unknown", "error": str(error)}
            # This lock is independent of dispatch and stops only this Body's arms.
            with ThreadPoolExecutor(max_workers=max(1, len(self._writers))) as pool:
                evidence = dict(pool.map(stop, self._writers.items()))
            self._stop_receipt = {"robot_id": self.robot.robot_id, "devices": evidence,
                "status": "requested" if evidence else "not_required", "physical_stop_guaranteed": False}
            return self._stop_receipt

    def estop(self, device_id):
        if device_id not in self._by_device:
            raise DeviceError(ErrorCode.UNKNOWN_DEVICE, "Unknown arm")
        return self._stop_all()

    def stream_hold(self, device_id):
        if device_id not in self._by_device:
            raise DeviceError(ErrorCode.UNKNOWN_DEVICE, "Unknown arm")
        # Body admission is quiesced before this hook. An idle position controller
        # already holds its completed target; emergency stop is not normal release.
        if not self.execute:
            return {"status": "not_required", "physical_stop_guaranteed": False}
        if self._dispatch_lock.acquire(blocking=False):
            try:
                if not self._blocked.is_set() and all(
                        self._require_feedback(name)["moving"] is False for name in self.robot.arms):
                    return {"status": "retained_position_hold", "physical_stop_guaranteed": False}
            except Exception:
                pass  # Unknown feedback must use the stop path.
            finally:
                self._dispatch_lock.release()
        return self._stop_all()

    def write(self, *args, **kwargs):
        raise DeviceError(ErrorCode.NOT_SUPPORTED, "Only preflighted group paths are supported")

    stream_apply = write

    def close(self):
        if self.execute and self._writers:
            self.stream_hold(next(iter(self._by_device)))
        self._cleanup.close()
