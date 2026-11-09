"""The real-robot motion program on the shared gripper tools; tool names and arguments do not change.

One move is one path that Body preflights and runs without intermediate stops, gripper-only
actions hold every joint at its measured angle, each action starts from a fresh reading, and the
feedback separates goal, execution and stop. This depends on Body's continuous-motion capability,
so it stays here until the shared protocol has a typed path command. A read-only episode is
composed without the motion tools.
"""
import numpy as np

from uni_robot_agent.services.environment.contracts import (
    ArmCommand, CaptureRequest, EnvironmentState, JointTarget, RequestRejected,
)
from uni_robot_agent.tools.gripper.contracts import motion_profiles
from uni_robot_agent.tools.gripper.motion import (
    ClientPrimitiveBackend, MotionExecutor, PoseTracker,
    _pose, _pose_values, _rejection_evidence, rpy_to_quaternion,
)
from uni_robot_agent.tools.gripper.opening import opening_tracker
from uni_robot_agent.tools.gripper.provider import GripperTools
from uni_robot_agent.tools.toolkit import ActionOutcome
from ..environment.path_command import MAX_SEGMENT_M, encode
from .feedback import FAILURE_REASONS, complete_motion

# Shared move guidance that describes the waypoint loop, not one preflighted Body path.
STEPWISE_ONLY = (
    "On supported backends, a rejected waypoint may trigger one search for a temporary execution goal within 80% "
    "of those tolerances. Native waypoint IK remains strict; arrival still uses measured error from the original "
    "goal. There is no automatic return to the exact center or coarse-to-fine phase. Inspect goal_recovery and "
    "per-arm execution_target_position/execution_target_quaternion when recovery occurs. ",
    "Rotation consumes displacement budget. ",
)
# Gripper-only actions on the continuous Body hold every joint at its measured angle.
GRIPPER_ONLY_HOLD = (
    " Here every arm joint keeps its measured angle and only the named grippers change, in one Body command; "
    "steps does not add time. An arm resting outside the end-effector box is reported in hold_reports and does "
    "not stop the gripper command. grasp reports opening_status closed or stopped_before_closed, which can mean "
    "contact or an obstruction, not a verified grasp; release and explicit openings are checked against the "
    "measured opening.")
# Shared opening guidance written for the stepwise loop, which keeps tracking after pose arrival;
# one continuous path checks the opening once, after the path.
CONTINUOUS_OPENING = (
    (", within the same max_steps budget;", " after the path;"),
    (" after pose arrival the arm holds while the opening continues.",
     " a blocked or slipping gripper reports opening_status=not_reached. Opening arrival does not establish a "
     "grasp."),
)


def _settled(opening):
    """One continuous command is over when its receipt returns, so an opening short of its target missed it."""
    feedback = opening.feedback()
    if feedback["opening_status"] == "pending":
        feedback["opening_status"] = "not_reached"
    return feedback


def _layout(metadata):
    """The arms and joint counts of the native path command, as the environment publishes them."""
    return tuple((name, int(joints)) for name, joints in metadata["path_layout"])


class ContinuousMotionExecutor(MotionExecutor):
    """One verified path or reachable prefix; Body monitors every control tick."""

    def move(self, request):
        metadata = self.backend.description.metadata
        if not metadata.get("continuous_motion"):
            return super().move(request)
        targets = {target.arm: target for target in request.targets}
        participants = self._participants([target.arm for target in request.targets])
        start = self._state()
        profile = request.parameters(motion_profiles(metadata.get("motion_profiles")))
        trackers, openings = {}, {}
        for name in participants:
            target, arm = targets[name], start.arms[name]
            trackers[name] = PoseTracker(
                initial_pose=_pose_values(arm.control_pose), target_xyz=target.pose[:3],
                target_quaternion=rpy_to_quaternion(target.pose[3:]),
                gripper=arm.gripper_command if target.gripper is None else target.gripper,
                rotation_radius_m=self.arms[name].rotation_radius_m, profile=profile)
            if target.gripper is not None and (opening := opening_tracker(target.gripper, name, metadata)):
                openings[name] = opening
                opening.update(arm)
        # Never apply the per-waypoint displacement guard to a whole path, and
        # never retry a partially executed operation inside this tool call.
        arms = ({name: JointTarget(joints=list(arm.joints), gripper=arm.gripper_command)
                 for name, arm in start.arms.items()}
                if metadata.get("continuous_motion") else self._held(start))
        arms.update({name: ArmCommand(pose=_pose(np.r_[tracker.target_xyz, tracker.target_quaternion]),
                                      gripper=tracker.gripper) for name, tracker in trackers.items()})
        receipt = self._advance(encode(arms, layout=_layout(metadata), max_segments=request.max_steps,
                                       segment_m=profile.max_step_m))
        state = receipt.state
        for name, tracker in trackers.items():
            tracker.update(_pose_values(state.arms[name].control_pose))
        for name, opening in openings.items():
            opening.update(state.arms[name], advanced=receipt.accepted)
        native_reason = receipt.diagnostics.get("reason")
        pose_reached = (receipt.accepted and native_reason != "reachable_prefix"
                        and all(tracker.reached for tracker in trackers.values()))
        opening_results = [opening.reached for opening in openings.values()]
        unmeasured = any(target.gripper is not None and name not in openings for name, target in targets.items())
        targets_reached = (False if not pose_reached or False in opening_results else
                           None if unmeasured or None in opening_results else True)
        reason = ("targets_reached" if openings and targets_reached else
                  "pose_reached" if pose_reached else "target_not_reached")
        evidence = None
        if not receipt.accepted or native_reason == "reachable_prefix":
            reason = native_reason or "planning_rejected"
            evidence = _rejection_evidence(receipt.diagnostics)
        steps = state.native_steps - start.native_steps
        return {
            "execution": "continuous", "targets_reached": targets_reached,
            "hold_reports": receipt.diagnostics.get("hold_reports", {}),
            "control_frames": {name: self.arms[name].control_frame for name in participants},
            "arm_devices": self.backend.description.metadata.get("arm_devices", {}),
            "profile": request.profile,
            "arms": {name: {**tracker.feedback(), "target_gripper": tracker.gripper,
                            "actual_gripper": state.arms[name].gripper, "gripper_command_submitted": steps > 0,
                            **(_settled(openings[name]) if name in openings else
                               {"opening_reached": None, "opening_status":
                                "not_requested" if targets[name].gripper is None else "unavailable"})}
                     for name, tracker in trackers.items()},
            "steps_executed": int(receipt.accepted), "requested_max_steps": request.max_steps,
            "executed_native_steps": steps, "pose_reached": pose_reached,
            "stop_reason": reason, "stop_evidence": evidence, "no_execution": steps == 0,
        }

    def _stationary(self, steps, grippers):
        """Gripper-only actions: every joint keeps its measured angle, only the named grippers change.

        Joint holds never enter the Body's end-effector workspace check, so an arm resting just outside it
        is reported in hold_reports instead of truncating the gripper command.
        """
        metadata = self.backend.description.metadata
        if not metadata.get("continuous_motion"):
            return super()._stationary(steps, grippers)
        start = self._state()
        arms = {name: JointTarget(joints=list(arm.joints), gripper=grippers.get(name, arm.gripper_command))
                for name, arm in start.arms.items()}
        receipt = self._advance(encode(arms, layout=_layout(metadata), max_segments=1, segment_m=MAX_SEGMENT_M))
        state = receipt.state
        feedback_arms = {}
        for name, value in grippers.items():
            tracker = opening_tracker(value, name, metadata)
            if tracker is not None:
                tracker.update(state.arms[name], advanced=receipt.accepted)
            feedback_arms[name] = {"target_gripper": value, "actual_gripper": state.arms[name].gripper,
                                   **(_settled(tracker) if tracker is not None else
                                      {"opening_reached": None, "opening_status": "unavailable"})}
        reason, evidence = "gripper_command_completed", None
        if not receipt.accepted or receipt.diagnostics.get("reason") == "reachable_prefix":
            reason = receipt.diagnostics.get("reason") or "planning_rejected"
            evidence = _rejection_evidence(receipt.diagnostics)
        steps_done = state.native_steps - start.native_steps
        return {"execution": "measured_joint_hold", "arms": feedback_arms,
                "hold_reports": receipt.diagnostics.get("hold_reports", {}),
                "requested_steps": steps, "steps_executed": int(receipt.accepted),
                "executed_native_steps": steps_done, "stop_reason": reason, "stop_evidence": evidence,
                "no_execution": steps_done == 0}

    def gripper(self, request):
        feedback = super().gripper(request)
        if feedback.get("execution") != "measured_joint_hold":
            return feedback
        if request.action == "grasp":
            # Closing stops on an object, so an unfinished close is evidence, not a missed target.
            for arm in feedback["arms"].values():
                closed = arm.get("opening_reached")
                arm.update(opening_reached=None, opening_status=(
                    "unknown" if closed is None else "closed" if closed else "stopped_before_closed"))
        else:
            # Release and explicit openings have a definite target; an unmeasured opening stays unjudged.
            reached = [arm.get("opening_reached") for arm in feedback["arms"].values()]
            if reached and None not in reached:
                feedback["targets_reached"] = all(reached)
        return feedback


class BodyGripperTools(GripperTools):
    def _bind_tools(self):
        metadata = self.episode.description.environment.metadata
        if not metadata.get("execute"):
            return []
        tools = super()._bind_tools()
        # The group path also holds joints natively, which the gripper-only hold below relies on.
        continuous = metadata.get("continuous_motion")
        for tool in tools:
            if tool.name in {"grasp", "release", "hold"} and continuous:
                tool.spec["description"] += GRIPPER_ONLY_HOLD
                continue
            if tool.name != "move":
                continue
            text = tool.spec["description"]
            if continuous:
                for sentence in STEPWISE_ONLY:
                    text = text.replace(sentence, "")
                for stepwise, path in CONTINUOUS_OPENING:
                    text = text.replace(stepwise, path)
                text += (" One move is one preflighted path: Body checks the whole path before dispatch, runs it "
                         "without intermediate stops and verifies the final arrival once. The profile increment "
                         "is a path scale, not a speed, and profile increment times max_steps bounds travel. A "
                         "distant or blocked target may stop at reachable_prefix, which is not the original goal.")
            tool.spec["description"] = text
        return tools

    def _adopt_measured_state(self):
        """Plan from a fresh reading; the robot may have moved since the last receipt."""
        manifest = self.episode.env.capture(CaptureRequest(cameras=[], high_resolution=False))
        state = EnvironmentState.model_validate(manifest["state"])
        if state.state_version < self.episode.state.state_version:
            raise RequestRejected("Measured state would roll back acknowledged action progress")
        self.episode.state = state

    def action_outcome(self, feedback, *, uncertain=False):
        """The shared outcome, refined by Body's completion fields; only a confirmed idle stop that
        allows a replan counts as stopped (ToolExecutionPolicy.replan_on_failure)."""
        shared = super().action_outcome(feedback, uncertain=uncertain)
        status = feedback.get("execution_status")
        executed = "unknown" if status == "unknown" else shared.executed
        complete = (shared.complete and executed != "unknown" and status != "partial"
                    and feedback.get("stop_reason") not in FAILURE_REASONS)
        stopped = (executed != "unknown" and feedback.get("stop_status") == "confirmed_idle"
                   and feedback.get("can_replan") is True)
        return ActionOutcome(complete=complete, executed=executed, stopped=stopped)

    def _run_motion_program(self, request, method, *, cameras=None, opening_feedback=None):
        self.episode.require_active()
        self._adopt_measured_state()
        start_state = self.episode.state.model_copy(deep=True)
        backend = ClientPrimitiveBackend(self.episode.env, start_state, on_reply=self.episode.accept_primitive)
        feedback = getattr(ContinuousMotionExecutor(backend), method)(request)
        if opening_feedback:
            feedback["opening_requests"] = opening_feedback
        reply = backend.last_reply
        if reply is None:
            raise RequestRejected("The motion program did not execute a primitive step")
        diagnostics = reply.feedback.get("diagnostics", {})
        feedback = complete_motion(feedback, start_state, reply.state_after, diagnostics)
        return self.action_output(reply.model_copy(update={"state_before": start_state, "feedback": feedback}),
                                  cameras=cameras)
