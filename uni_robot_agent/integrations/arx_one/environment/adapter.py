"""Physical arm adapter on the shared environment contract, for arms and cameras served by Body.

Failures go back to the model as receipts and never end the episode. Movement
admission has three states: active; replan required after a verified stop, which
a fresh capture clears; and blocked, which only an operator restart clears.
Blocked covers an unverified outcome or stop, a controller fault and lost
ownership. Observation and finish stay available in every state.
"""
from concurrent.futures import ThreadPoolExecutor
import threading
import time
from uuid import uuid4

import numpy as np
from scipy.spatial.transform import Rotation

from uni_robot_agent.services.environment.contracts import (
    ArmDescription, ArmState, CameraDescription, EnvironmentDescription, EnvironmentState,
    JointTarget, PoseCommand, RequestRejected, StepReceipt,
)
from uni_robot_agent.services.environment.snapshots import CapturedObservation
from .path_command import decode
from .control import BodyClient, BodyError
from .control import Control, OperationInterrupted, PreflightBlocked, confirm_unowned_idle
from .depth import complete_frames
from .kinematics import bounded_target, distance, from_eef, matrix, pose
from .kinematics import Kinematics
from .native_compat import verify, verify_cameras
from .observation import capture
from .tracking_recovery import tracking_reason
from ..body.core.arm import contract
from ..body.core.timing import host_clock_id, timestamp_ns

OPERATOR_RECOVERY = ("Movement stays blocked until an operator checks the robot and restarts the "
                     "environment service. capture_scene and finish remain available.")
REPLAN = ("Call capture_scene, then plan a new target from the measured state. A new command rechecks "
          "readiness and acquires a new lease; the interrupted command is not replayed.")


def _pose_rpy(t):
    """World [x,y,z,roll,pitch,yaw], the form a move target uses."""
    rpy = Rotation.from_matrix(t[:3, :3]).as_euler("xyz")
    return np.round([*t[:3, 3], *rpy], 5).tolist()


class Adapter:
    def __init__(self, config, output_dir, *, episode_id, client=None, camera_client=None, depth_backend=None):
        self.config, self.episode_id = config, episode_id
        # The native path command lists the arms in config order with their joint counts.
        self.layout = tuple((name, len(arm.joint_names)) for name, arm in config.arms.items())
        self.client = client or BodyClient(config.arms_body, timeout_s=config.rpc_timeout_s)
        self.camera_client = camera_client
        self.depth_backend = depth_backend
        self.control = None
        self._pool = None
        self.steps = 0
        self.ended = self.closed = False
        self._commands = {}
        self._outcome = None
        self._output_dir = output_dir
        self._last_state = None
        self._replan = None
        self._blocked = None
        self._recovery_count = 0
        self._recovery_lock = threading.RLock()
        try:
            self.camera_client = camera_client or BodyClient(config.cameras_body, timeout_s=config.rpc_timeout_s)
            self.limits = verify(self.client, config)
            self.camera_signals = verify_cameras(self.camera_client, config)
            self.kinematics = {name: Kinematics(arm) for name, arm in config.arms.items() if arm.kinematics}
            self.description = self._describe()
            state = self._fresh_state()
            if self.kinematics:
                self._check_fk(state)
            if config.execute:
                # A finished process does not prove the arms stopped.
                confirm_unowned_idle(config, self.client)
                self.control = self._new_control("body-journal.jsonl")
                self.control.acquire()
            self._pool = ThreadPoolExecutor(max_workers=len(config.arms), thread_name_prefix="arm-command")
        except BaseException:
            self.close()
            raise

    def _describe(self):
        config = self.config
        metadata = {"robot_id": config.robot_id, "physical_hardware": True, "execute": config.execute,
            "T_native_from_grasp": {n: a.T_native_from_grasp for n, a in config.arms.items()},
            "joint_kinematics": {n: k.joint_description(config.arms[n]) for n, k in self.kinematics.items()},
            "control_frequency_semantics": "maximum_submission_rate_not_realtime_guarantee"}
        metadata["finish_statuses"] = ["success", "failure", "give_up"]
        metadata["normalized_opening_tolerance"] = 0.05  # Body's open-gripper settle tolerance.
        metadata["arm_devices"] = {name: arm.device_id for name, arm in config.arms.items()}
        metadata["path_layout"] = [list(entry) for entry in self.layout]
        # One preflighted path for every arm (move_eef_group), including joint holds for idle arms.
        metadata["continuous_motion"] = all(l["continuous_motion"] for l in self.limits.values())
        if config.motion_profiles:
            metadata["motion_profiles"] = {n: p.model_dump() for n, p in config.motion_profiles.items()}
        return EnvironmentDescription(
            arms=[ArmDescription(name=name, control_frame=arm.grasp_site,
                grasp_site_description=arm.grasp_site_description, rotation_radius_m=arm.rotation_radius_m,
                joint_names=arm.joint_names, joint_tracking_tolerance_rad=arm.tracking_tolerance_rad)
                for name, arm in config.arms.items()],
            cameras=[CameraDescription(name=name, role="scene" if camera.parent == "world" else "wrist",
                has_calibration=camera.T_parent_from_optical is not None)
                for name, camera in config.cameras.items()],
            control_frequency_hz=config.control_frequency_hz, max_episode_steps=config.max_episode_steps,
            # The service already publishes one snapshot per request; a smaller
            # interval would capture every camera a second time.
            observation_interval_steps=config.max_episode_steps + 1, metadata=metadata)

    def _new_control(self, journal):
        return Control(self.client, [a.device_id for a in self.config.arms.values()], self.config,
                       self._output_dir / journal)

    # Movement admission

    def _sync_admission(self):
        """Turn a watchdog stop into the recovery state the model can act on."""
        control = self.control
        with self._recovery_lock:
            if (control is None or not control.blocked.is_set() or control.evidence is None
                    or self._blocked or self.ended):
                return
            stop = control.evidence
            if control.failure == "control_idle_timeout" and stop["status"] == "confirmed_idle":
                if not self._replan:
                    self._replan = {"reason": "control_idle_timeout", "observed": False}
            elif control.failure or not self._replan:
                # A watchdog failure (lost control, an action timeout, an unconfirmed idle stop)
                # goes to the operator even while a replan is pending for another reason.
                self._replan = None
                self._blocked = {"reason": control.failure or stop["reason"], "stop": stop}

    def _control_status(self):
        self._sync_admission()
        control = self.control
        if control is None:
            return {"phase": "read_only"}
        stop = control.evidence or {}
        common = {"stop_status": stop.get("status", "pending"), "old_actions_replayed": False}
        if self._blocked:
            return {"phase": "blocked", "reason": self._blocked["reason"], "can_replan": False,
                    "next_action": OPERATOR_RECOVERY, **common}
        if self._replan:
            observed = self._replan["observed"]
            return {"phase": "ready_to_replan" if observed else "capture_required",
                    "reason": self._replan["reason"], "can_replan": observed, "next_action": REPLAN, **common}
        if control.blocked.is_set():
            return {"phase": "stopping", "reason": control.failure or "control_admission_closed",
                    "can_replan": False, **common}
        return {"phase": "active"}

    def _require_admission(self):
        self._sync_admission()
        if self._blocked:
            raise RequestRejected(f"Movement is blocked ({self._blocked['reason']}). {OPERATOR_RECOVERY}")
        if self._replan:
            if not self._replan["observed"]:
                raise RequestRejected(f"The previous action stopped early ({self._replan['reason']}). "
                                      f"No command was sent. {REPLAN}")
            self._resume_control()
        try:
            self.control.require()
        except BodyError as error:
            raise RequestRejected(f"Control is stopping ({error.code}); no command was sent. {REPLAN}") from error

    def _resume_control(self):
        """A new model command may obtain a new lease; an interrupted command cannot."""
        with self._recovery_lock:
            previous = self.control
            if not previous.blocked.is_set():
                # An observation failure leaves the current lease valid.
                self._replan = None
                return
            try:
                # Fresh post-stop samples, no owner, no active work, and the pinned
                # pinned identity plus device readiness.
                confirm_unowned_idle(self.config, self.client)
                self.limits = verify(self.client, self.config)
                self._fresh_state()
                previous.close()
                self._recovery_count += 1
                self.control = self._new_control(f"body-journal-recovery-{self._recovery_count}.jsonl")
                self.control.record("recovery_requested", recovery_reason=self._replan["reason"])
                self.control.acquire()
                self._replan = None
                self.control.record("recovery_ready", replayed_operations=False)
            except Exception as error:
                self._replan = None
                self._blocked = {"reason": f"recovery_failed: {type(error).__name__}", "stop": previous.evidence}
                self.control.stop("recovery_failed")
                raise RequestRejected(f"Recovery preflight failed; no command was sent. {OPERATOR_RECOVERY}") from error

    def _ready(self, command):
        self.validate_commands([command], "direct")
        if not self.config.execute or self.ended or self.steps >= self.config.max_episode_steps:
            raise RequestRejected("Physical execution is disabled or the episode has ended")
        self._require_admission()
        try:
            state = self._fresh_state()
        except Exception as error:
            raise RequestRejected(f"Arm feedback is unavailable ({type(error).__name__}); no command was sent") from error
        for name, target in command.arms.items():
            if isinstance(target, JointTarget):
                delta = np.max(np.abs(np.asarray(target.joints) - state.arms[name].joints))
                if delta > self.limits[name]["joint_step_rad"]:
                    raise RequestRejected("Joint target exceeds the measured-state delta bound")
        return state

    # State

    def _fresh_state(self, *, after_ns=None):
        began = time.monotonic()
        readings = self.client.call("read_state", {"device_ids": [a.device_id for a in self.config.arms.values()],
            "freshness": {"max_age_s": self.config.max_age_s, "on_stale": "error"}})["devices"]
        arms = {}
        for name, config in self.config.arms.items():
            reading = readings[config.device_id]
            age = reading.get("age_s")
            if (type(age) not in (int, float) or not np.isfinite(age) or age < 0
                    or age + time.monotonic() - began > self.config.max_age_s
                    or reading.get("freshness") != "fresh" or reading.get("error")):
                raise RuntimeError("Arm feedback is stale or unavailable")
            timing = reading.get("timing") or {}
            source_ns = timestamp_ns(timing.get("source_timestamp_ns"))
            now_ns = time.time_ns()
            if (timing.get("clock_id") != host_clock_id() or timing.get("source_clock") != "unix_realtime"
                    or timing.get("source_semantics") != "controller_status_publication"
                    or source_ns > now_ns + 1_000_000 or now_ns - source_ns > self.config.max_age_s * 1e9
                    or (after_ns is not None and source_ns <= after_ns)):
                raise RuntimeError("Arm feedback source time is stale or uses a different clock")
            signals = reading["signals"]
            joints = signals["joint_position_rad"]
            grip = signals["gripper_opening"]
            if len(joints) != len(config.joint_names) or not contract.opening_measured(signals):
                raise RuntimeError("Calibrated measured joints and gripper feedback required")
            t = np.asarray(config.T_world_from_controller) @ from_eef(signals["eef_position_m"], signals["eef_rpy_rad"])
            arms[name] = ArmState(control_pose=pose(t @ np.asarray(config.T_native_from_grasp)), joints=joints, gripper=grip,
                gripper_command=self._commands.get(name, grip),
                metadata={"control_pose_rpy": _pose_rpy(t @ np.asarray(config.T_native_from_grasp)),
                          "timing": reading.get("timing"), "moving": reading.get("moving"),
                          "gripper_opening_measured": contract.opening_measured(signals)})
        state = EnvironmentState(arms=arms, state_version=self.steps, native_steps=self.steps, ended=self.ended,
            metadata={"physical_hardware": True, "feedback_fresh": True, "control_status": self._control_status()})
        self._last_state = state
        return state

    def read_state(self):
        """Report the last measured state, marked stale, rather than failing the episode.

        The service treats a failed state read as unknown execution. Movement never
        relies on this value: admission and action receipts read fresh feedback.
        """
        try:
            return self._fresh_state()
        except Exception as error:
            if self._last_state is None:
                raise
            state = self._last_state.model_copy(deep=True)
            state.state_version = state.native_steps = self.steps
            state.ended = self.ended
            state.metadata.update(feedback_fresh=False, feedback_error=type(error).__name__,
                                  control_status=self._control_status())
            return state

    def _measured_commands(self, state):
        # A stopped or rejected arm keeps its measured gripper, not the requested one.
        for name, arm in state.arms.items():
            self._commands[name] = arm.gripper
            arm.gripper_command = arm.gripper

    # Actions

    def validate_commands(self, commands, mode):
        if not commands:
            raise RequestRejected("Empty command sequence")
        for command in commands:
            if not isinstance(command, PoseCommand) or set(command.arms) != set(self.config.arms):
                raise RequestRejected(f"Commands must name every arm: {sorted(self.config.arms)}")
            if command.solver != "native":
                raise RequestRejected("This environment uses the Body native IK solver")
            for name, target in command.arms.items():
                if isinstance(target, JointTarget):
                    config = self.config.arms[name]
                    if len(target.joints) != len(config.joint_names) or any(not lo <= q <= hi for q, lo, hi in
                            zip(target.joints, config.joint_lower, config.joint_upper)):
                        raise RequestRejected("Joint target outside commissioned limits")

    def _parallel(self, calls):
        futures = [self._pool.submit(fn, *args) for fn, args in calls]
        results, failure = [], None
        for future in futures:
            try:
                results.append(future.result())
            except Exception as error:
                failure = failure or error
        if failure:
            raise failure
        return results

    def _blocked_receipt(self, reason, message, **evidence):
        self._replan = None
        self._blocked = {"reason": reason, "stop": self.control.evidence}
        state = self.read_state()
        if state.metadata.get("feedback_fresh"):
            self._measured_commands(state)
        diagnostics = {"reason": reason, "message": f"{message} {OPERATOR_RECOVERY}", "recoverable": False,
                       "requires_operator_recovery": True, "stop": self.control.evidence, **evidence}
        self.control.record("blocked_receipt", **diagnostics)
        return StepReceipt(accepted=False, state=state, diagnostics=diagnostics)

    def _unknown_receipt(self, error):
        """Never replay a command whose acknowledgement or feedback was lost."""
        stop = self.control.stop("physical_action_failed")
        code = getattr(error, "code", type(error).__name__)
        return self._blocked_receipt("execution_unknown",
            f"The outcome of the last physical command is unverified ({code}); stop status: {stop['status']}.",
            execution_outcome="unknown")

    def _interrupted_receipt(self, error):
        """Publish known execution only after all group results and the stop are verified."""
        with self._recovery_lock:
            stop = self.control.stop("physical_action_failed")
            native = (error.details["operation"].get("error") or {})
            try:
                operations = self.control.stopped_operations(error.operation_ids)
            except Exception as receipt_error:
                # Quiescence may fail precisely because a motor is faulted. Report the
                # observed Body error without claiming a stop or a peer's outcome.
                code = getattr(receipt_error, "code", type(receipt_error).__name__)
                return self._blocked_receipt("execution_unknown",
                    f"The arm failed ({native.get('message', 'Body operation failed')}), and its stop or receipt "
                    f"could not be verified ({code}); stop status: {stop['status']}.",
                    execution_outcome="unknown", native_error=native)
            failures = [op for op in operations.values() if op["state"] == "failed"]
            # The first terminal report may be a cancelled peer, not the failed arm.
            native = next((op["error"] for op in failures if op.get("error")), native)
            reasons = [tracking_reason(op.get("error")) for op in failures]
            no_execution = all(op["execution_outcome"] == "none" for op in operations.values())
            if no_execution:
                self.steps -= 1
            outcome = "none" if no_execution else "partial"
            blocker = self._recovery_blocker(operations, failures, reasons)
            if blocker:
                return self._blocked_receipt("native_action_failed",
                    f"The arm failed ({native.get('message', blocker)}) and stopped; this failure needs an operator ({blocker}).",
                    execution_outcome=outcome, native_error=native, operations=operations)
            self._replan = {"reason": "tracking_interrupted", "observed": False}
            state = self.read_state()
            if not state.metadata.get("feedback_fresh") or any(
                    arm.metadata.get("moving") is not False for arm in state.arms.values()):
                return self._blocked_receipt("native_action_failed",
                    "The action stopped, but stationary feedback could not be read.",
                    execution_outcome=outcome, operations=operations)
            self._measured_commands(state)
            diagnostics = {"reason": "tracking_interrupted", "execution_outcome": outcome,
                "message": f"The arm stopped before the waypoint ({', '.join(reasons)}); the group is confirmed idle. {REPLAN}",
                "recoverable": True, "requires_operator_recovery": False, "tracking_reasons": reasons,
                "stop": stop, "operations": operations}
            self.control.record("interrupted_receipt", **diagnostics)
            return StepReceipt(accepted=False, state=state, diagnostics=diagnostics)

    def _recovery_blocker(self, operations, failures, reasons):
        """Unknown outcomes, controller faults and lost control stay with the operator."""
        if not failures or self.control.failure or self.ended or self.closed:
            return self.control.failure or "not_a_tracking_failure"
        if any(reason is None for reason in reasons):
            return "not_a_tracking_failure"
        for op in failures:
            devices = ((op.get("error") or {}).get("details") or {}).get("devices")
            if devices and set(devices) != set(op["device_ids"]):
                return "incomplete_group_report"
        try:
            confirm_unowned_idle(self.config, self.client)
            verify(self.client, self.config)
        except Exception as error:
            return f"recovery_preflight_failed: {type(error).__name__}"
        return None

    def _execute(self, action, grippers):
        try:
            self.control.begin_action()
        except BodyError as error:
            raise RequestRejected(f"Control is stopping ({error.code}); no command was sent. {REPLAN}") from error
        # Count a dispatched attempt before crossing the physical boundary so a
        # later validation or transport error cannot be reported as not started.
        self.steps += 1
        rejected = None
        try:
            try:
                diagnostics = action()
            except OperationInterrupted as error:
                return self._interrupted_receipt(error)
            except PreflightBlocked as error:
                rejected = error
                diagnostics = {"reason": "command_limit", "operations": error.operations, "recoverable": True,
                    "message": "The Body command limits rejected this waypoint before execution. Choose a different reachable target."}
                if all(op["state"] == "failed" for op in error.operations.values()):
                    self.steps -= 1  # Every member explicitly reports no execution.
            after_ns = time.time_ns()
            # Wait for feedback published after dispatch completion, never reuse
            # a still-fresh pre-action sample as the physical action receipt.
            deadline = min(self.control.action_deadline(), time.monotonic() + self.config.rpc_timeout_s)
            while True:
                self.control.require()
                try:
                    state = self._fresh_state(after_ns=after_ns)
                    break
                except RuntimeError:
                    if time.monotonic() >= deadline:
                        raise
                    self.control.blocked.wait(.01)
            self._commands.update(grippers)
            for name, grip in grippers.items():
                state.arms[name].gripper_command = grip
            self.control.end_action()
            stationary = all(arm.metadata.get("moving") is False for arm in state.arms.values())
            if rejected:
                if not stationary:
                    raise RuntimeError("Preflight rejection group has no stationary feedback")
                self._measured_commands(state)
            if stationary:
                # Terminal operations and fresh stillness; not a lease release or an E-stop.
                diagnostics["stop"] = {"status": "confirmed_idle", "physical_stop_guaranteed": False,
                                       "basis": "terminal_operations_and_fresh_stationary_feedback"}
            accepted = diagnostics["reason"] == "completed" or (
                diagnostics["reason"] == "reachable_prefix" and diagnostics.get("execution") == "continuous")
            return StepReceipt(accepted=accepted, state=state, diagnostics=diagnostics)
        except Exception as error:
            return self._unknown_receipt(error)

    def step_pose(self, command):
        state = self._ready(command)
        calls = []
        targets = {}
        for name, target in command.arms.items():
            config, limits = self.config.arms[name], self.limits[name]
            measured = state.arms[name]
            grip_duration = abs(target.gripper - measured.gripper) / limits["gripper_speed_fraction_s"]
            if isinstance(target, JointTarget):
                duration = max(.1, grip_duration, 1.875 * np.max(np.abs(np.asarray(target.joints) - measured.joints)) / limits["joint_speed_rad_s"])
                procedure, params = "move_joints", {"kind": "absolute", "values": target.joints, "duration_s": float(duration)}
            else:
                grasp_from_native = np.linalg.inv(config.T_native_from_grasp)
                start = matrix(measured.control_pose.position, measured.control_pose.quaternion)
                desired = matrix(target.pose.position, target.pose.quaternion)
                bounded_grasp = bounded_target(start, desired, translation=min(.04, limits["eef_step_m"]),
                    rotation=min(.3, limits["eef_rotation_step_rad"]), rotation_radius_m=float(np.linalg.norm(grasp_from_native[:3, 3])))
                bounded = bounded_grasp @ grasp_from_native
                dp, angle = distance(start @ grasp_from_native, bounded)
                duration = max(.1, grip_duration, 1.875 * dp / limits["eef_speed_m_s"],
                    (10 / np.sqrt(3) * dp / limits["eef_accel_m_s2"]) ** .5,
                    1.875 * angle / limits["eef_rotation_speed_rad_s"])
                local = np.linalg.inv(config.T_world_from_controller) @ bounded
                procedure, params = "move_eef", {"target_position_m": local[:3, 3].tolist(),
                    "target_rpy_rad": Rotation.from_matrix(local[:3, :3]).as_euler("xyz").tolist(),
                    "target_frame": f"{config.device_id}/base", "duration_s": float(duration)}
                targets[name] = pose(bounded_grasp).model_dump()
            if duration + self.config.rpc_timeout_s >= self.config.action_timeout_s:
                raise RequestRejected("Motion duration exceeds the configured action deadline")
            params["gripper_opening"] = target.gripper
            calls.append((self.control.invoke, (config.device_id, procedure, params)))

        def action():
            operations = self.control.wait_operations(self._parallel(calls))
            # Body may correct a goal at the workspace boundary and still succeed.
            adjustments = [item for op in operations.values() for item in
                           [(op.get("result") or {}).get("workspace_clipping"),
                            *(op.get("result") or {}).get("workspace_clippings", [])] if item]
            diagnostics = {"operations": operations, "bounded_targets": targets, "reason": "completed", "recoverable": True,
                           "completion_basis": "body_operation_and_measured_feedback_not_task_success"}
            if any(item.get("requires_replan", True) for item in adjustments):
                diagnostics.update(reason="reachable_prefix", message="Body stopped this waypoint at the workspace "
                    "boundary. Inspect the measured pose and choose a reachable target.")
            return diagnostics
        return self._execute(action, {n: t.gripper for n, t in command.arms.items()})

    def step_joints(self, command):
        raise RequestRejected("Direct control uses pose commands")

    def step_native(self, command):
        """One preflighted path for both arms; Body monitors it on every control tick."""
        request = decode(command.values, layout=self.layout)
        self._ready(request.command)
        if not self.description.metadata["continuous_motion"]:
            raise RequestRejected("Body continuous motion capability is unavailable")
        # Reserve the RPC allowance; Body may run a shorter verified prefix instead.
        budget = min(30., self.config.action_timeout_s - self.config.rpc_timeout_s - .01)
        if budget < .2:
            raise RequestRejected("Action deadline leaves no bounded motion window")
        targets = {}
        for name, target in request.command.arms.items():
            config = self.config.arms[name]
            if isinstance(target, JointTarget):
                targets[config.device_id] = {"hold_joints": list(target.joints), "gripper_opening": target.gripper}
                continue
            native = np.asarray(config.T_native_from_grasp)
            local = (np.linalg.inv(config.T_world_from_controller)
                     @ matrix(target.pose.position, target.pose.quaternion) @ np.linalg.inv(native))
            targets[config.device_id] = {"target_position_m": local[:3, 3].tolist(),
                "target_rpy_rad": Rotation.from_matrix(local[:3, :3]).as_euler("xyz").tolist(),
                "target_frame": f"{config.device_id}/base", "duration_s": .2,
                "control_offset_m": native[:3, 3].tolist(), "rotation_radius_m": config.rotation_radius_m,
                "gripper_opening": target.gripper}

        def action():
            operation = self.control.invoke_group(list(targets), "move_eef_group",
                {"targets": targets, "max_segments": request.max_segments, "segment_m": request.segment_m,
                 "max_duration_s": budget})
            operations = self.control.wait_operations([operation])
            result = operations[operation].get("result") or {}
            prefix = result.get("requested_targets_reached") is False
            diagnostics = {"operations": operations, "execution": "continuous", "recoverable": True,
                           "reason": "reachable_prefix" if prefix else "completed",
                           "hold_reports": result.get("hold_reports", {})}
            if prefix:
                diagnostics["message"] = ("Body executed the reachable part of this path. Inspect the measured "
                                          "pose and prefix reason, observe again and plan a new target.")
            return diagnostics
        return self._execute(action, {n: t.gripper for n, t in request.command.arms.items()})

    def preview_joints(self, commands):
        raise RequestRejected("Direct control has no joint preview")

    def _check_fk(self, state):
        checks = {}
        for name, fk in self.kinematics.items():
            measured = state.arms[name].control_pose
            predicted = np.asarray(self.config.arms[name].T_world_from_controller) @ fk.controller(state.arms[name].joints)
            dp, angle = distance(predicted, matrix(measured.position, measured.quaternion)
                                 @ np.linalg.inv(self.config.arms[name].T_native_from_grasp))
            if dp > .005 or angle > .03:
                raise RequestRejected("Pinned FK disagrees with controller feedback")
            checks[name] = {"position_error_m": dp, "rotation_error_rad": angle}
        return checks

    # Observation

    def _complete_depth(self, frames, provenance):
        """A failed fill keeps the measured depth; the capture itself still succeeds."""
        try:
            return complete_frames(frames, provenance, self.depth_backend,
                                   observe_id=f"{self.episode_id}/{uuid4().hex}")
        except Exception as error:
            return frames, {"status": "failed", "error": f"{type(error).__name__}: {error}"[:300]}

    def capture(self, cameras=None, *, high_resolution=False):
        names = list(self.config.cameras) if cameras is None else cameras
        if not set(names) <= self.config.cameras.keys():
            raise RequestRejected("Unknown camera")
        # Only an observation begun after the verified stop can enable recovery;
        # snapshots published while an action runs or fails cannot.
        self._sync_admission()
        pending = self._replan if high_resolution else None
        try:
            frames, provenance = capture(self, names)
            state = self._fresh_state()
        except Exception as error:
            # No physical action ran, and the episode stays open. The next action
            # waits for a good observation rather than continuing without images.
            with self._recovery_lock:
                if names and self.control is not None and not (self._replan or self._blocked or self.ended):
                    self._replan = {"reason": "observation_failed", "observed": False}
            raise RequestRejected(f"Observation failed ({type(error).__name__}: {error}); "
                                  "no physical action was executed") from error
        state.metadata["camera_provenance"] = provenance
        if self.control is not None:
            self.control.observed()
        if high_resolution and self.depth_backend is not None:
            frames, report = self._complete_depth(frames, provenance)
            state.metadata["depth_completion"] = report
        if pending is not None and pending is self._replan and frames and all(
                arm.metadata.get("moving") is False for arm in state.arms.values()):
            pending["observed"] = True
            state.metadata["control_status"] = self._control_status()
        return CapturedObservation(self.episode_id, self.steps, time.time(), state.model_dump(mode="json"),
                                   frames, terminated=self.ended)

    # Lifecycle

    def finish(self, reason, status=None):
        with self._recovery_lock:
            if self._outcome is None:
                self.ended = True
                self._replan = None
                stop = self._final_stop(reason)
                # Physical task success is assessed outside this process.
                self._outcome = {"task_success": None, "physical_task_evaluated": False,
                    "completion_claim": status, "reason": reason, "stop": stop,
                    "physical_hardware": True, "robot_id": self.config.robot_id,
                    "retryable": False, "native_steps": self.steps}
        return self._outcome

    def _final_stop(self, reason):
        """Stop evidence for the outcome, current at finish rather than cached from an earlier stop."""
        if not self.control:
            return {"status": "not_required", "physical_stop_guaranteed": False}
        earlier = self.control.evidence
        if earlier is None:
            return self.control.stop(reason)
        try:
            current = confirm_unowned_idle(self.config, self.client)
        except Exception as error:
            return {"status": "unknown", "reason": reason, "physical_stop_guaranteed": False,
                    "error": type(error).__name__, "earlier_stop": earlier}
        return {**current, "reason": reason, "earlier_stop": earlier}

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.control:
                self.control.close()
        finally:
            if self._pool:
                self._pool.shutdown(wait=True, cancel_futures=True)
            for resource in (self.depth_backend, self.camera_client, self.client):
                if resource:
                    resource.close()
