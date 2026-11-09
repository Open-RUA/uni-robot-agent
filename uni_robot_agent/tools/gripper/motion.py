"""Model-level gripper programs built from primitive environment steps.

The program is intentionally client-side.  It plans waypoints, checks tracking
progress, and chooses when to stop; each individual command is sent through
``EnvironmentClient.step`` by the small backend adapter used by the caller.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable
from typing import Any

import numpy as np

from uni_robot_agent.services.environment.contracts import (
    GoalRegionRequest,
    ArmCommand,
    EnvironmentState,
    JointCommand,
    NativeCommand,
    Pose,
    PoseCommand,
    RequestRejected,
    StepReceipt,
    StepRequest,
)
from uni_robot_agent.tools.gripper.contracts import (
    GripperRequest,
    HoldRequest,
    MotionParameters,
    MoveRequest,
    motion_profiles,
)
from uni_robot_agent.tools.gripper.pose import (
    pose_waypoint,
    quaternion,
    quaternion_distance,
    rpy_to_quaternion,
    quaternion_to_rpy,
    vector,
)


from uni_robot_agent.tools.gripper.opening import opening_tracker


class PoseTracker:
    """Per-arm tracking; the executor alone advances the complete world."""

    def __init__(
        self, *, initial_pose, target_xyz, target_quaternion, gripper: float,
        rotation_radius_m: float, profile: MotionParameters,
    ) -> None:
        self.previous = vector(initial_pose, 7, "initial_pose")
        self.target_xyz = vector(target_xyz, 3, "xyz")
        self.target_quaternion = quaternion(target_quaternion)
        self.execution_xyz = self.target_xyz.copy()
        self.execution_quaternion = self.target_quaternion.copy()
        self.gripper = gripper
        self.rotation_radius_m = rotation_radius_m
        self.profile = profile
        self.position_errors: list[float] = []
        self.orientation_errors: list[float] = []
        self.prior_score = self.position_error + .1 * self.orientation_error
        self.worsening = 0
        self.hold_pose: np.ndarray | None = None

    def position(self, pose) -> np.ndarray:
        return np.asarray(pose[:3], dtype=float)

    @property
    def position_error(self) -> float:
        return float(np.linalg.norm(self.position(self.previous) - self.target_xyz))

    @property
    def orientation_error(self) -> float:
        return quaternion_distance(self.previous[3:], self.target_quaternion)

    @property
    def reached(self) -> bool:
        return (self.position_error <= self.profile.position_tolerance_m
                and self.orientation_error <= self.profile.orientation_tolerance_rad)

    def set_execution_goal(self, pose: Pose) -> None:
        xyz, orientation = vector(pose.position, 3, "execution_goal"), quaternion(pose.quaternion)
        if (np.linalg.norm(xyz - self.target_xyz) > self.profile.position_tolerance_m
                or quaternion_distance(orientation, self.target_quaternion) > self.profile.orientation_tolerance_rad):
            raise RequestRejected("Recovered goal exceeds the original target tolerance")
        self.execution_xyz, self.execution_quaternion = xyz, orientation
        self.position_errors.clear()
        self.orientation_errors.clear()
        self.prior_score = (np.linalg.norm(self.previous[:3] - xyz)
                            + .1 * quaternion_distance(self.previous[3:], orientation))
        self.worsening = 0
        self.hold_pose = None

    def waypoint(self) -> np.ndarray:
        xyz = self.execution_xyz if self.hold_pose is None else self.position(self.hold_pose)
        orientation = self.execution_quaternion if self.hold_pose is None else self.hold_pose[3:]
        return pose_waypoint(
            self.previous, xyz, orientation, max_step_m=self.profile.max_step_m,
            rotation_radius_m=self.rotation_radius_m,
        )

    def update(self, actual) -> tuple[str, dict] | None:
        self.previous = vector(actual, 7, "actual_pose")
        # Progress follows the execution goal; acceptance always uses the original goal.
        position_error = float(np.linalg.norm(self.previous[:3] - self.execution_xyz))
        orientation_error = quaternion_distance(self.previous[3:], self.execution_quaternion)
        self.position_errors.append(position_error)
        self.orientation_errors.append(orientation_error)
        score = position_error + .1 * orientation_error
        if self.reached:
            self.prior_score, self.worsening = score, 0
            if self.hold_pose is None:
                self.hold_pose = self.previous.copy()
            return None
        self.worsening = self.worsening + 1 if score > self.prior_score + .01 else 0
        if self.worsening >= 2:
            return "diverging", {
                "previous_score": self.prior_score, "current_score": score,
                "orientation_weight": .1, "score_increase_threshold": .01,
                "consecutive_increases": self.worsening,
            }
        self.prior_score = score
        position_threshold = self.profile.position_tolerance_m * .1
        orientation_threshold = self.profile.orientation_tolerance_rad * .1
        if (len(self.position_errors) >= 4
                and self.position_errors[-4] - min(self.position_errors[-3:]) <= position_threshold
                and self.orientation_errors[-4] - min(self.orientation_errors[-3:]) <= orientation_threshold):
            return "stalled", {
                "window_steps": 4,
                "position_improvement_m": self.position_errors[-4] - min(self.position_errors[-3:]),
                "orientation_improvement_rad": self.orientation_errors[-4] - min(self.orientation_errors[-3:]),
                "position_improvement_threshold_m": position_threshold,
                "orientation_improvement_threshold_rad": orientation_threshold,
            }
        return None

    def feedback(self) -> dict:
        return {
            "target_pose": [*self.target_xyz.tolist(), *quaternion_to_rpy(self.target_quaternion).tolist()],
            "actual_pose": [*self.position(self.previous).tolist(), *quaternion_to_rpy(self.previous[3:]).tolist()],
            "target_position": self.target_xyz.tolist(), "target_quaternion": self.target_quaternion.tolist(),
            "actual_position": self.position(self.previous).tolist(), "actual_quaternion": self.previous[3:].tolist(),
            "position_error_m": self.position_error, "orientation_error_rad": self.orientation_error,
            "position_tolerance_m": self.profile.position_tolerance_m,
            "orientation_tolerance_rad": self.profile.orientation_tolerance_rad,
            "pose_reached": self.reached,
            "execution_target_position": self.execution_xyz.tolist(),
            "execution_target_quaternion": self.execution_quaternion.tolist(),
        }


def _pose_values(pose: Pose) -> np.ndarray:
    return np.array([*pose.position, *pose.quaternion], dtype=float)


def _pose(values) -> Pose:
    return Pose(position=tuple(values[:3]), quaternion=tuple(values[3:]))


def _rejection_evidence(diagnostics: dict) -> dict:
    """Keep the failed planning evidence; full diagnostics live in primitive receipts."""
    evidence = {key: diagnostics[key] for key in ("reason", "arm", "solver", "message") if key in diagnostics}
    arms = {
        name: {key: item[key] for key in (
            "ik_status", "max_joint_delta_rad", "joint_delta_limit_rad", "exceeded_joint_indices", "recovery",
        ) if key in item}
        for name, item in diagnostics.get("arms", {}).items()
        if "arm" not in diagnostics or name == diagnostics["arm"]
    }
    if arms:
        evidence["arms"] = arms
    return evidence


class MotionExecutor:
    """Run one bounded model action against a primitive-step backend."""

    def __init__(self, backend: Any, on_step: Callable[[EnvironmentState], None] | None = None) -> None:
        self.backend = backend
        self.on_step = on_step
        self.arms = {arm.name: arm for arm in backend.description.arms}

    def _participants(self, names: list[str]) -> list[str]:
        if not names or len(names) != len(set(names)) or not set(names) <= self.arms.keys():
            raise RequestRejected("arms must name distinct supported arms")
        return [name for name in self.arms if name in names]

    def _state(self) -> EnvironmentState:
        state = self.backend.read_state()
        if set(state.arms) != set(self.arms):
            raise RuntimeError("Environment state does not match its arm description")
        return state

    def _held(self, state: EnvironmentState) -> dict[str, ArmCommand]:
        return {
            name: ArmCommand(pose=arm.control_pose, gripper=arm.gripper_command)
            for name, arm in state.arms.items()
        }

    def _advance(self, command) -> StepReceipt:
        if isinstance(command, PoseCommand):
            receipt = self.backend.step_pose(command)
        elif isinstance(command, JointCommand):
            receipt = self.backend.step_joints(command)
        elif isinstance(command, NativeCommand):
            receipt = self.backend.step_native(command)
        else:
            raise TypeError("Unsupported control command")
        if receipt.accepted and self.on_step is not None:
            self.on_step(receipt.state)
        return receipt

    def _motion(self, before: EnvironmentState, after: EnvironmentState, active: set[str]) -> tuple[dict, list[str]]:
        motions, unsafe = {}, []
        for name in self.arms:
            old = _pose_values(before.arms[name].control_pose)
            new = _pose_values(after.arms[name].control_pose)
            moved = float(np.linalg.norm(new[:3] - old[:3]))
            rotated = quaternion_distance(new[3:], old[3:])
            motions[name] = {"translation_m": moved, "rotation_rad": rotated}
            translation_limit, rotation_limit = (.2, .8) if name in active else (.05, .25)
            if moved > translation_limit or rotated > rotation_limit:
                unsafe.append(name)
        return motions, unsafe

    def move(self, request: MoveRequest) -> dict:
        targets = {target.arm: target for target in request.targets}
        participants = self._participants([target.arm for target in request.targets])
        start = state = self._state()
        metadata = self.backend.description.metadata
        profile = request.parameters(motion_profiles(metadata.get("motion_profiles")))
        trackers, openings = {}, {}
        for name in participants:
            target, arm = targets[name], state.arms[name]
            trackers[name] = PoseTracker(
                initial_pose=_pose_values(arm.control_pose), target_xyz=target.pose[:3],
                target_quaternion=rpy_to_quaternion(target.pose[3:]),
                gripper=arm.gripper_command if target.gripper is None else target.gripper,
                rotation_radius_m=self.arms[name].rotation_radius_m,
                profile=profile,
            )
            if target.gripper is not None and (tracker := opening_tracker(target.gripper, name, metadata)):
                openings[name] = tracker
                openings[name].update(arm)
        executed = 0
        reason, evidence = "max_steps", None
        recovery_attempted, goal_recovery = False, None
        ik_statistics = Counter()
        while executed < request.max_steps:
            if state.ended:
                reason = "native_end"
                break
            command_arms = self._held(start)
            waypoints = {name: tracker.waypoint() for name, tracker in trackers.items()}
            command_arms.update({
                name: ArmCommand(pose=_pose(waypoint), gripper=trackers[name].gripper)
                for name, waypoint in waypoints.items()
            })
            receipt = self._advance(PoseCommand(arms=command_arms))
            for arm in receipt.diagnostics.get("arms", {}).values():
                ik_statistics.update(arm.get("recovery", {}).get("stats", {}))
            if not receipt.accepted:
                reason, evidence = "planning_rejected", _rejection_evidence(receipt.diagnostics)
                motion, unsafe = self._motion(state, receipt.state, set(participants))
                state = receipt.state
                # Rejection advances no command, but pooled physics may change observations.
                # Refresh measurements without counting an attempt as tracking progress.
                for name, tracker in trackers.items():
                    tracker.previous = _pose_values(state.arms[name].control_pose)
                for name, opening in openings.items():
                    opening.update(state.arms[name], advanced=False)
                if state.ended:
                    reason = "native_end"
                    break
                if unsafe:
                    reason, evidence = "unsafe_motion", {"arms": motion, "unsafe_arms": unsafe,
                                                        "rejection": evidence}
                    break
                failed_arm = receipt.diagnostics.get("arm")
                if (not recovery_attempted and failed_arm in trackers
                        and receipt.diagnostics.get("reason") in {"ik_failed", "joint_jump_rejected", "ik_candidate_rejected"}
                        and self.backend.description.metadata.get("goal_region_search")):
                    recovery_attempted = True
                    tracker = trackers[failed_arm]
                    recovered = self.backend.solve_goal_region(GoalRegionRequest(
                        arm=failed_arm, goal=_pose(np.r_[tracker.target_xyz, tracker.target_quaternion]),
                        position_tolerance_m=profile.position_tolerance_m,
                        orientation_tolerance_rad=profile.orientation_tolerance_rad,
                        expected_state_version=state.state_version,
                    ))
                    goal_recovery = {"arm": failed_arm, "trigger": evidence,
                                     **recovered.model_dump(mode="json")}
                    if recovered.state_version != state.state_version:
                        raise RequestRejected("Recovered goal is based on a stale state")
                    if recovered.found:
                        tracker.set_execution_goal(recovered.pose)
                        reason, evidence = "max_steps", None
                        continue
                break
            motion, unsafe = self._motion(state, receipt.state, set(participants))
            state = receipt.state
            executed += 1
            failures = {}
            for name, tracker in trackers.items():
                failure = tracker.update(_pose_values(state.arms[name].control_pose))
                if failure is not None:
                    failures[name] = {"stop_reason": failure[0], **failure[1]}
            for name, opening in openings.items():
                opening.update(state.arms[name])
            if state.ended:
                reason = "native_end"
                break
            if unsafe:
                reason, evidence = "unsafe_motion", {"arms": motion, "unsafe_arms": unsafe}
                break
            if all(tracker.reached for tracker in trackers.values()):
                if all(opening.reached is True for opening in openings.values()):
                    reason = "targets_reached" if openings else "pose_reached"
                    break
                if any(opening.reached is None for opening in openings.values()):
                    reason = "opening_unavailable"
                    break
                if any(opening.stalled for opening in openings.values()):
                    reason = "opening_stalled"
                    evidence = {"arms": {name: opening.feedback() for name, opening in openings.items()}}
                    break
            if failures:
                name = next(iter(failures))
                reason = failures[name]["stop_reason"]
                evidence = {"arm": name, **failures[name], "failures": failures}
                break
        pose_reached = all(tracker.reached for tracker in trackers.values())
        opening_results = [opening.reached for opening in openings.values()]
        unmeasured = any(target.gripper is not None and name not in openings for name, target in targets.items())
        targets_reached = (False if not pose_reached or False in opening_results else
                           None if unmeasured or None in opening_results else True)
        feedback = {
            "targets_reached": targets_reached,
            "goal_recovery": goal_recovery,
            "ik_statistics": dict(ik_statistics) if ik_statistics else None,
            "control_frames": {name: self.arms[name].control_frame for name in participants},
            "profile": request.profile,
            "arms": {name: {**tracker.feedback(), "target_gripper": tracker.gripper,
                            "actual_gripper": state.arms[name].gripper,
                            "gripper_command_submitted": executed > 0,
                            **(openings[name].feedback() if name in openings else
                               {"opening_reached": None, "opening_status": "not_requested" if targets[name].gripper is None else "unavailable"})}
                     for name, tracker in trackers.items()},
            "steps_executed": executed, "requested_max_steps": request.max_steps,
            "executed_native_steps": state.native_steps - start.native_steps,
            "pose_reached": all(tracker.reached for tracker in trackers.values()),
            "stop_reason": reason, "stop_evidence": evidence,
            "no_execution": executed == 0,
        }
        return feedback

    def _stationary(self, steps: int, grippers: dict[str, float]) -> dict:
        start = state = self._state()
        metadata = self.backend.description.metadata
        openings = {name: tracker for name, value in grippers.items()
                    if (tracker := opening_tracker(value, name, metadata)) is not None}
        for name, opening in openings.items():
            opening.update(state.arms[name])
        executed = 0
        reason, evidence = "requested_step_budget", None
        for _ in range(steps):
            if state.ended:
                reason = "native_end"
                break
            arms = self._held(state)
            for name, opening in grippers.items():
                arms[name] = ArmCommand(pose=arms[name].pose, gripper=opening)
            receipt = self._advance(PoseCommand(arms=arms))
            if not receipt.accepted:
                reason, evidence = "planning_rejected", _rejection_evidence(receipt.diagnostics)
                motion, unsafe = self._motion(state, receipt.state, set())
                state = receipt.state
                for name, opening in openings.items():
                    opening.update(state.arms[name], advanced=False)
                if state.ended:
                    reason = "native_end"
                elif unsafe:
                    reason, evidence = "unsafe_motion", {"arms": motion, "unsafe_arms": unsafe,
                                                        "rejection": evidence}
                break
            motion, unsafe = self._motion(state, receipt.state, set())
            state = receipt.state
            executed += 1
            for name, opening in openings.items():
                opening.update(state.arms[name])
            if state.ended:
                reason = "native_end"
                break
            if unsafe:
                reason, evidence = "unsafe_motion", {"arms": motion, "unsafe_arms": unsafe}
                break
        feedback = {
            "arms": {name: {"target_gripper": value,
                            **(openings[name].feedback() if name in openings else
                               {"opening_reached": None, "opening_status": "unavailable"})}
                     for name, value in grippers.items()},
            "requested_steps": steps, "steps_executed": executed,
            "executed_native_steps": state.native_steps - start.native_steps,
            "stop_reason": reason, "stop_evidence": evidence,
            "no_execution": executed == 0,
        }
        return feedback

    def gripper(self, request: GripperRequest) -> dict:
        participants = self._participants(request.arms)
        target = 0.0 if request.action == "grasp" else 1.0
        if not set(request.openings) <= set(participants) or any(not 0 <= value <= 1 for value in request.openings.values()):
            raise RequestRejected("Invalid gripper targets")
        if request.action == "grasp" and request.openings:
            raise RequestRejected("grasp preserves the native closing preload")
        targets = {name: request.openings.get(name, target) for name in participants}
        feedback = self._stationary(request.steps, targets)
        feedback["action"] = request.action
        return feedback

    def hold(self, request: HoldRequest) -> dict:
        feedback = self._stationary(request.steps, {})
        feedback["action"] = "hold"
        return feedback



class ClientPrimitiveBackend:
    """Adapt ``EnvironmentClient`` to the motion program's step interface.

    The adapter keeps the latest state returned by each HTTP receipt, so the
    next command carries the exact ``state_version`` it was planned against.
    This avoids a read-before-every-step request while preserving the service's
    stale-observation guard.  The service remains responsible for one physical
    step; this object only translates its reply for tool-side orchestration.
    """

    def __init__(self, client, state: EnvironmentState | None = None, *, on_reply=None) -> None:
        self.client = client
        if client.info is None:
            raise RuntimeError("EnvironmentClient.connect() must precede motion tools")
        self.description = client.info.environment
        self._state = state.model_copy(deep=True) if state is not None else None
        self.last_reply = None
        self.on_reply = on_reply

    def read_state(self) -> EnvironmentState:
        if self._state is None:
            manifest = self.client.record(-1)
            self._state = EnvironmentState.model_validate(manifest["state"])
        return self._state.model_copy(deep=True)

    def _step(self, command) -> StepReceipt:
        before = self.read_state()
        reply = self.client.step(StepRequest(
            command=command, expected_state_version=before.state_version,
        ))
        self.last_reply = reply
        self._state = reply.state_after.model_copy(deep=True)
        if self.on_reply is not None:
            self.on_reply(reply)
        feedback = dict(reply.feedback)
        return StepReceipt(
            accepted=bool(feedback.get("accepted", True)),
            state=self._state,
            diagnostics=dict(feedback.get("diagnostics", feedback)),
        )

    def step_pose(self, command: PoseCommand) -> StepReceipt:
        return self._step(command)

    def step_joints(self, command: JointCommand) -> StepReceipt:
        return self._step(command)

    def step_native(self, command: NativeCommand) -> StepReceipt:
        return self._step(command)

    def solve_goal_region(self, request: GoalRegionRequest):
        return self.client.solve_goal_region(request)

    def preview_joints(self, commands) -> dict:
        return self.client.preview(commands)
