"""Motion feedback for the model: completion fields, compact failure context and joint-limit summaries."""
from copy import deepcopy

from uni_robot_agent.tools.gripper.pose import quaternion_to_rpy

# Native failure reasons that the shared loop reports as planning_rejected.
FAILURE_REASONS = {"tracking_interrupted", "command_limit", "reachable_prefix", "execution_unknown",
                   "native_action_failed"}
EVIDENCE_KEYS = ("operations", "recoverable", "stop", "execution_outcome", "tracking_reasons",
                 "requires_operator_recovery", "native_error")
# Endpoint-only IK verdicts for a path stopped at a joint limit.
GOAL_JOINT_CHECK = {
    "within_joint_limits": "The requested endpoint alone has an in-limit IK solution; the path to it is not checked.",
    "outside_joint_limits": ("The IK solutions tried for the requested endpoint alone need {violations} beyond "
                             "its limit; other IK branches were not searched."),
    "outside_workspace": "The requested endpoint is outside the arm's end-effector workspace.",
}


def failure_context(feedback, state, diagnostics):
    evidence = feedback.get("stop_evidence") or {}
    operations = diagnostics.get("operations") or evidence.get("operations") or {}
    failures = []
    executions = []

    def add_execution(operation_id, device_ids, result):
        fields = {key: deepcopy(result[key]) for key in (
            "original_requested_end_pos", "requested_end_pos", "goal_end_pos",
            "effective_command_end_pos", "workspace_clipping", "workspace_clippings",
            "workspace_path_clipping", "last_sent_end_pos",
            "feedback_eef_position_m", "feedback_eef_rpy_rad", "feedback_gripper_opening",
            "hardware_command_sent", "completed", "executed_path_fraction", "path_fraction_reference",
            "measured_workspace", "workspace_recovery_required", "entry_recovery", "eef_arrival",
        ) if key in result}
        if fields:
            executions.append({"operation_id": operation_id, "device_ids": list(device_ids),
                "pose_frame": "Body native controller EEF; not world TCP and not a delta", **fields})

    def add(operation_id, device_ids, error, source, goal_check=None):
        details = error.get("details") or {}
        # Retain exact Body fields and index conventions; absence is explicit.
        joint = {key: deepcopy(details[key]) for key in (
            "joint", "target_rad", "lower_rad", "upper_rad", "limit_source",
            "exceeded_joint_indices", "joint_index_base", "current_joints_rad",
            "target_joints_rad", "joint_delta_rad", "joint_delta_limit_rad",
        ) if key in details}
        path = {key: deepcopy(details[key]) for key in (
            "path_fraction", "previous_path_fraction", "planned_path_fraction",
            "command_fraction", "fraction_semantics", "start_end_pos",
            "requested_end_pos", "original_requested_end_pos",
            "effective_command_end_pos", "workspace_clipping",
            "workspace_path_clipping", "last_sent_end_pos",
        ) if key in details}
        tracking = {key: deepcopy(details[key]) for key in (
            "position_error_m", "rotation_error_rad", "violation_s", "allowed_lag_s",
            "exceeded_components", "tracking_limits",
            "reason", "hardware_command_sent", "hold_command_sent",
        ) if key in details}
        failures.append({
            "operation_id": operation_id, "device_ids": list(device_ids),
            "source": source, "code": error.get("code"), "stage": error.get("stage"),
            "message": error.get("message"),
            "joint_evidence": joint or None, "path_evidence": path or None,
            "tracking_evidence": tracking or None,
            "workspace_evidence": deepcopy(details.get("workspace") or error.get("workspace")),
            "path_pose_frame": "Body native controller EEF, not world control_frame" if any(
                key in details for key in ("start_end_pos", "requested_end_pos", "original_requested_end_pos")
            ) else None,
            **({"goal_joint_check": deepcopy(goal_check)} if goal_check else {}),
        })

    def add_prefix(operation_id, device_ids, result):
        prefix = result.get("prefix_reason")
        if isinstance(prefix, dict):
            add(operation_id, device_ids, prefix.get("error") or prefix, "reachable_prefix",
                prefix.get("goal_joint_check"))

    for operation_id, operation in operations.items():
        error = operation.get("error") or {}
        devices = (error.get("details") or {}).get("devices") or {}
        child_errors = [(device, value["error"]) for device, value in devices.items()
                        if value.get("error")]
        if child_errors:
            for device, child in child_errors:
                add(operation_id, [device], child, "per_device_error")
        elif error:
            add(operation_id, operation.get("device_ids", []), error, "operation_error")
        # A completed reachable prefix can carry a rejected IK candidate.
        result = operation.get("result") or {}
        add_execution(operation_id, operation.get("device_ids", []), result)
        add_prefix(operation_id, operation.get("device_ids", []), result)
        for device, value in result.get("devices", {}).items():
            add_execution(operation_id, [device], value)
            add_prefix(operation_id, [device], value)
        for device, value in devices.items():
            add_execution(operation_id, [device], value.get("result") or (value.get("error") or {}).get("details") or {})

    arms = {device: name for name, device in (feedback.get("arm_devices") or {}).items()}
    return {
        "scope": "this request and its attempted path only; not a global reachability conclusion",
        "reason": feedback.get("stop_reason", diagnostics.get("reason")),
        "original_targets": {
            name: {"control_frame": feedback.get("control_frames", {}).get(name),
                   "pose": arm.get("target_pose"), "gripper": arm.get("target_gripper")}
            for name, arm in feedback.get("arms", {}).items()
        },
        "pose_convention": "world control_frame [x,y,z,roll,pitch,yaw]; m/rad; Rz@Ry@Rx",
        "measured_state_version": state.state_version,
        "step_evidence": {key: deepcopy(evidence[key]) for key in (
            "blocked_step_index", "interrupted_step_index", "completed_steps",
            "attempted_steps", "blocked_targets",
        ) if key in evidence},
        "body_commands": {
            operation_id: deepcopy(operation["command"])
            for operation_id, operation in operations.items() if operation.get("command")
        },
        "actual_state": {
            name: {
                "control_pose_rpy": [*arm.control_pose.position,
                                     *quaternion_to_rpy(arm.control_pose.quaternion).tolist()],
                "joints_rad": list(arm.joints), "gripper": arm.gripper,
                "timing": deepcopy(arm.metadata.get("timing")),
                "moving": arm.metadata.get("moving"),
            } for name, arm in state.arms.items()
        },
        "failures": failures,
        "joint_limit_summary": joint_limit_summary(failures, executions, arms, state),
        "executed_commands": executions,
        "stop_status": feedback.get("stop_status"),
        "can_replan": feedback.get("can_replan"),
        "requires_operator_recovery": feedback.get(
            "requires_operator_recovery", diagnostics.get("requires_operator_recovery", False)),
        "interpretation": (
            "Null/missing evidence is unknown, not zero. Command/path fractions are not measured "
            "completion. steps_executed=0 alone does not mean no motion. Joint numbers/axes "
            "must be read from device joint_kinematics, never guessed as EEF roll/pitch/yaw. "
            "Do not retry if can_replan is false; no automatic replay."
        ),
    }


def joint_limit_summary(failures, executions, arms, state):
    """One entry per joint limit that stopped a path: where, the measured margin, the endpoint check."""
    summary, seen = [], set()
    for failure in failures:
        joint = failure.get("joint_evidence") or {}
        if not {"joint", "target_rad", "lower_rad", "upper_rad"} <= joint.keys():
            continue
        device = failure["device_ids"][0] if len(failure["device_ids"]) == 1 else None
        if (device, joint["joint"]) in seen:
            continue
        seen.add((device, joint["joint"]))
        arm = arms.get(device)
        side = "upper" if joint["target_rad"] > joint["upper_rad"] else "lower"
        limit = joint[f"{side}_rad"]
        # joint_limit_details numbers joints from 1.
        measured = state.arms[arm].joints[joint["joint"] - 1] if arm in state.arms else None
        goal = failure.get("goal_joint_check") or {}
        summary.append({
            "arm": arm, "device_id": device, "joint": joint["joint"], "joint_index_base": 1,
            "limit_side": side, "limit_rad": limit, "path_ik_rad": joint["target_rad"],
            "measured_rad": measured,
            "measured_margin_rad": None if measured is None else round(
                limit - measured if side == "upper" else measured - limit, 4),
            "executed_path_fraction": next((item["executed_path_fraction"] for item in executions
                                            if item["device_ids"] == [device]
                                            and "executed_path_fraction" in item), None),
            "goal_joint_check": goal.get("status", "not_checked"),
            "goal_joint_violations": [
                {key: violation[key] for key in ("joint", "target_rad", "lower_rad", "upper_rad")}
                for violation in goal.get("joint_violations", [])],
        })
    return summary


def complete_motion(feedback, start, state, diagnostics):
    """Keep Body's failure reason and separate goal completion from transport."""
    feedback["no_execution"] = state.native_steps == start.native_steps
    native = diagnostics.get("reason")
    failed = native in FAILURE_REASONS
    if failed and feedback.get("stop_reason") == "planning_rejected":
        feedback["stop_reason"] = native
    evidence = feedback.get("stop_evidence")
    if failed and isinstance(evidence, dict):
        evidence.update({key: diagnostics[key] for key in EVIDENCE_KEYS if key in diagnostics})
    if failed and native != "execution_unknown":
        feedback["targets_reached"] = False
        if "pose_reached" in feedback:
            feedback["pose_reached"] = False
    reached = feedback.get("targets_reached")
    if reached is None and "targets_reached" not in feedback and not failed:
        return feedback  # Gripper and hold results that finished normally.
    # This installation has no opening model: arrival is measured, the gripper is not.
    arrived = reached is None and feedback.get("pose_reached") is True and not failed
    stop = diagnostics.get("stop") or {}
    stop_status = stop.get("status", "unverified")
    outcome = diagnostics.get("execution_outcome")
    blocked = bool(state.ended or diagnostics.get("requires_operator_recovery") or outcome == "unknown")
    can_replan = (reached is not True and not blocked and stop_status == "confirmed_idle"
                  and diagnostics.get("recoverable") is True)
    feedback.update(
        goal_status="reached" if reached is True else "unverified" if reached is None else "not_reached",
        execution_status=("unknown" if outcome == "unknown" else "not_executed" if feedback["no_execution"]
                          else "completed" if reached is True or arrived else "partial"),
        stop_status=stop_status, can_replan=can_replan, measured_state_version=state.state_version,
        requires_operator_recovery=blocked,
        completion_summary=(
            "The original goal was reached; this does not establish a grasp, contact or task success."
            if reached is True else
            "The pose arrived; the requested gripper opening is not measured here, so read the measured gripper."
            if arrived else
            "The original goal is incomplete and the stop is confirmed. Read the measured pose, "
            "observe again and plan a new target from that state; nothing is replayed." if can_replan else
            "The original goal is not confirmed and continuing is not permitted yet. Check the stop "
            "evidence and fault; a successful call is not goal completion."))
    if any(report.get("status") == "deviation_warning" for report in feedback.get("hold_reports", {}).values()):
        feedback["completion_summary"] += (" A holding arm reported a deviation warning; this did not cancel "
                                           "the moving arm. Read hold_reports.")
    if reached is not True and not arrived:
        feedback["failure_context"] = failure_context(feedback, state, diagnostics)
        for limit in feedback["failure_context"]["joint_limit_summary"]:
            feedback["completion_summary"] += " " + joint_limit_sentence(limit)
    return feedback

def joint_limit_sentence(limit):
    """The joint that stopped the path, its measured margin and the endpoint check, in one line."""
    measured = ("measured margin unknown" if limit["measured_margin_rad"] is None else
                f"measured {limit['measured_rad']:.4f} rad, margin {limit['measured_margin_rad']:.4f} rad")
    violations = " and ".join(f"J{item['joint']} {item['target_rad']:.4f} rad"
                              for item in limit["goal_joint_violations"])
    goal = GOAL_JOINT_CHECK.get(limit["goal_joint_check"],
                                "Whether the requested endpoint is within joint limits is unknown.")
    return (f"{(limit['arm'] or limit['device_id'] or 'An').capitalize()} arm J{limit['joint']} stopped the path "
            f"at its {limit['limit_side']} limit {limit['limit_rad']:.4f} rad ({measured}). "
            + goal.format(violations=violations))
