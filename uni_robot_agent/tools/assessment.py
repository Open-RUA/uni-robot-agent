"""Pure call/effect diagnostics over this call's existing feedback, not rewards."""
from __future__ import annotations
import math


VERSION = "tool_assessment_1"


def assess_call(output: dict) -> dict:
    """Only producer-typed policy faults are eligible for downstream penalties."""
    fault = output.get("call_error") or {}
    if output.get("success") is True and not output.get("error"):
        status, owner, reason = "success", "none", "returned"
    else:
        status = "unknown" if output.get("execution_uncertain") else "failure"
        owner = fault.get("fault_owner", "unknown")
        reason = fault.get("code", "unattributed_error")
    return {"version": VERSION, "status": status, "fault_owner": owner,
            "reason": reason, "evidence": fault, "penalty_eligible": (
                True if status == "failure" and owner == "policy" else
                None if owner == "unknown" else False)}


def assess_effect(name: str, output: dict, *, image_count: int = 0) -> dict:
    """Use the released tool contracts, including metric gripper opening feedback."""
    feedback = output.get("feedback") or {}
    status, reason = "unknown", "missing_feedback"
    evidence = {}
    if name in {"finish", "get_tool_result", "wait_for_tool", "wait_for_tools",
                "list_tool_jobs", "cancel_tool", "queue_status", "flush_queue",
                "list_skills", "read_skill", "write_skill"}:
        reason = "not_applicable"
    elif not output.get("success") or output.get("error") or feedback.get("no_execution"):
        reason = "execution_unconfirmed"
    elif name in {"view_env_state", "capture_scene"}:
        status = "success" if image_count > 0 and not output.get("artifact_error") else "unknown"
        reason = "image_evidence"
        evidence = {"image_count": image_count}
    elif name == "locate":
        if output.get("found") is False:
            status, reason = "failure", "target_not_found"
        elif output.get("geometry_valid") is True:
            status, reason = "success", "geometry_valid"
        elif output.get("method") == "triangulate" and type((output.get("geometry") or {}).get("valid")) is bool:
            status = "success" if output["geometry"]["valid"] else "failure"
            reason = "triangulation_geometry"
        elif output.get("method") == "pixels":
            geometry = output.get("geometry") or {}
            points, count = geometry.get("points_xyz"), geometry.get("valid_count")
            if isinstance(points, list) and type(count) is int and count == len(points):
                if count > 0 and all(isinstance(p, list) and len(p) == 3 and all(
                        type(v) in (int, float) and math.isfinite(v) for v in p) for p in points):
                    status, reason = "success", "finite_projected_points"
                elif count == 0:
                    status, reason = "failure", "no_valid_points"
        evidence = {key: output.get(key) for key in ("method", "found", "geometry_valid", "geometry_quality")}
    elif name == "move":
        reached = feedback.get("targets_reached", feedback.get("pose_reached"))
        if type(reached) is bool and feedback.get("steps_executed", 0) > 0:
            status, reason = ("success" if reached else "failure"), "controller_targets"
        evidence = feedback
    elif name in {"grasp", "release"}:
        arms = feedback.get("arms") or {}
        reached = [arm.get("opening_reached") for arm in arms.values()]
        if feedback.get("steps_executed", 0) > 0 and reached and all(type(v) is bool for v in reached):
            status = "success" if all(reached) else "failure"
            reason = "measured_opening_only"
        evidence = feedback
    elif name == "hold":
        executed, requested = feedback.get("steps_executed"), feedback.get("requested_steps")
        if isinstance(executed, int) and isinstance(requested, int) and executed > 0:
            if executed == requested or feedback.get("stop_reason") == "native_end":
                status, reason = "success", "hold_completed"
            elif feedback.get("stop_reason") in {"unsafe_motion", "planning_rejected"}:
                status, reason = "failure", "hold_stopped"
        evidence = feedback
    return {"version": VERSION, "status": status, "reason": reason, "evidence": evidence}
