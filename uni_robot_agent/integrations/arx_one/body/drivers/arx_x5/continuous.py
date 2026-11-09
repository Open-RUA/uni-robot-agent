"""One preflighted group operation; no intermediate stops or physical retries."""

import math
import threading
from concurrent.futures import ThreadPoolExecutor

from ...core.contract import DeviceError, ErrorCode
from ...core.arm.eef import (
    finite_vector, interpolate_pose, pose_error, quaternion, resolve_goal, rotation_distance, validate_target,
)
from .safety import clip_eef_xyz


def _reject(message):
    raise DeviceError(ErrorCode.LIMIT_VIOLATION, message, stage="preflight")


def rotate(rpy, vector):
    w, x, y, z = quaternion(rpy)
    vx, vy, vz = vector
    return [(1-2*(y*y+z*z))*vx + 2*(x*y-w*z)*vy + 2*(x*z+w*y)*vz,
            2*(x*y+w*z)*vx + (1-2*(x*x+z*z))*vy + 2*(y*z-w*x)*vz,
            2*(x*z-w*y)*vx + 2*(y*z+w*x)*vy + (1-2*(x*x+y*y))*vz]


class ControlSitePath:
    """Linear registered-site position + SLERP, converted to native EEF.

    Right-multiplying orientation by a fixed calibration preserves SLERP.
    Only the native-to-site translation is needed to retain the site path.
    """
    def __init__(self, start, goal, offset):
        self.start, self.goal, self.offset = list(start), list(goal), list(offset)
        self.first = [p+r for p, r in zip(start[:3], rotate(start[3:], offset))]
        self.last = [p+r for p, r in zip(goal[:3], rotate(goal[3:], offset))]
        self.angle = rotation_distance(start[3:], goal[3:])
        self.radius = math.hypot(*offset)
        self.length_bound = math.dist(self.first, self.last) + self.radius*self.angle

    def at(self, fraction):
        if fraction == 0.:
            return list(self.start)
        if fraction == 1.:
            return list(self.goal)
        native = interpolate_pose(self.start, self.goal, fraction)
        displacement = rotate(native[3:], self.offset)
        first_offset = rotate(self.start[3:], self.offset)
        last_offset = rotate(self.goal[3:], self.offset)
        # Keep translation and rotation correction separate. Subtracting two
        # ~15 cm values can turn a mathematically zero native X into negative.
        native[:3] = [a+(b-a)*fraction + ((u+(v-u)*fraction)-r)
                      for a,b,u,v,r in zip(self.start[:3], self.goal[:3], first_offset, last_offset, displacement)]
        return native

    def minimum_duration(self, limits):
        # Bound both tangential and rotational-offset centripetal acceleration.
        return max(limits.eef_duration_s(self.length_bound, self.angle), math.sqrt(
            ((10/math.sqrt(3))*self.length_bound + 1.875**2*self.radius*self.angle**2)
            / limits.eef_accel_m_s2))


class PreflightCancelled(Exception):
    pass


def prepare(driver, device_id, target, max_segments, segment_m, cancel=None, max_duration_s=30.):
    """Find a verified reachable prefix of the requested path without writes.

    Segments bound IK work and retain the existing travel bounds; they are NOT
    separate movement operations. Execution follows one global quintic curve.
    """
    if isinstance(target, dict) and "hold_joints" in target:
        from .safety import check_joint_target
        if set(target) != {"hold_joints", "gripper_opening"}:
            _reject("Invalid joint hold target")
        joints, start = driver._require_fresh(device_id)
        goal = target["hold_joints"]
        check_joint_target(goal, profile=driver._profiles[device_id])
        if max(abs(a-b) for a,b in zip(goal, joints[:6])) > driver.tracking_limits.hold_admission_joint_rad:
            _reject("Arm moved before joint hold admission")
        opening = target["gripper_opening"]
        if type(opening) not in (int, float) or not math.isfinite(opening) or not 0 <= opening <= 1:
            _reject("Invalid hold gripper opening")
        duration = .2
        current, _ = driver.raw_to_opening(device_id, joints[6])
        if current is not None:
            duration = max(duration, 1.875*abs(opening-current)/driver.motion_limits.gripper_speed_fraction_s)
        if duration > max_duration_s:
            _reject("Hold gripper duration exceeds budget")
        return {"start": list(start[:6]), "joints": list(joints[:6]), "goal": list(start[:6]),
            "passive_hold": True, "hold_joints": list(joints[:6]), "opening": opening, "duration": duration,
            "requested_goal": list(start[:6]), "path_fraction": 1., "prefix_reason": None,
            "report": {"checked": True, "execution": "measured_joint_hold"}}
    allowed = {"target_position_m", "target_rpy_rad", "target_frame", "duration_s", "gripper_opening", "control_offset_m", "rotation_radius_m"}
    if not isinstance(target, dict) or set(target) - allowed:
        _reject("Unsupported continuous target fields")
    try:
        parsed = validate_target(target.get("target_position_m"), target.get("target_rpy_rad"),
                                 target.get("target_frame"), device_id)
    except ValueError as error:
        _reject(str(error))
    joints, start = driver._require_fresh(device_id)
    joints, start = list(joints[:6]), list(start[:6])
    goal = resolve_goal(start, parsed)
    requested_goal = list(goal)
    goal, workspace_clipping = clip_eef_xyz(goal)
    limits = driver.motion_limits
    try:
        offset = finite_vector(target.get("control_offset_m", [0., 0., 0.]), "control_offset_m")
    except ValueError as error:
        _reject(str(error))
    path = ControlSitePath(start, goal, offset)
    radius = target.get("rotation_radius_m", path.radius)
    if type(radius) not in (int, float) or not math.isfinite(radius) or radius < path.radius - 1e-9:
        _reject("Invalid rotational work budget radius")
    work = math.dist(path.first, path.last) + radius*path.angle
    fraction = min(1., max_segments*min(segment_m, limits.eef_step_m)/max(work, 1e-12),
                   max_segments*min(.3, limits.eef_rotation_step_rad)/max(path.angle, 1e-12))
    prefix_reason = None if fraction == 1. else {"code": "motion_budget"}
    full_path = path
    path = ControlSitePath(start, full_path.at(fraction), offset)
    if path.minimum_duration(limits) > max_duration_s:
        low, high = 0., fraction
        for _ in range(40):
            middle = (low+high)/2
            trial = ControlSitePath(start, full_path.at(middle), offset)
            if trial.minimum_duration(limits) <= max_duration_s:
                low = middle
            else:
                high = middle
        fraction = low
        prefix_reason = {"code": "action_duration_budget"}
        path = ControlSitePath(start, full_path.at(fraction), offset)
    goal = path.goal
    n = max(1, math.ceil(path.length_bound / min(segment_m, limits.eef_step_m) - 1e-9),
            math.ceil(path.angle / min(.3, limits.eef_rotation_step_rad) - 1e-9))
    duration = target.get("duration_s", .2)
    opening = target.get("gripper_opening")
    if (type(duration) not in (int, float) or not math.isfinite(duration) or not .1 <= duration <= 30
            or (opening is not None and (type(opening) not in (int, float)
                or not math.isfinite(opening) or not 0 <= opening <= 1))):
        _reject("Invalid continuous duration or gripper opening")
    duration = max(duration, path.minimum_duration(limits))
    if opening is not None:
        current_opening, _ = driver.raw_to_opening(device_id, driver._require_fresh(device_id)[0][6])
        if current_opening is not None:
            duration = max(duration, 1.875 * abs(opening-current_opening) / limits.gripper_speed_fraction_s)
    if duration > max_duration_s + 1e-9:
        _reject("Gripper or requested duration exceeds bounded action duration")
    previous, seed, reports = start, joints, []
    checked_fraction = 0.
    for i in range(1, n + 1):
        if cancel is not None and cancel.is_set():
            raise PreflightCancelled()
        endpoint = path.at(i / n)
        # This also checks SDK workspace, IK branch continuity and joint limits.
        try:
            report = driver._check_eef_path(device_id, seed, previous, endpoint,
                path=lambda fraction, index=i: path.at((index-1+fraction)/n),
                path_length_m=path.length_bound/n)
            if report.get("entry_recovery"):
                recovery = report["entry_recovery"]
                q = recovery["target_joints_rad"]
                duration = max(.2, 1.875*max(abs(a-b) for a,b in zip(joints,q))/limits.joint_speed_rad_s)
                if duration > max_duration_s:
                    _reject("Recovery exceeds action duration budget")
                return {"start": start, "joints": joints, "goal": report["effective_goal_end_pos"],
                    "hold_joints": q, "opening": None, "duration": duration,
                    "requested_goal": requested_goal, "path_fraction": 0.,
                    "prefix_reason": {"code": "joint_entry_recovery", "requires_replan": True},
                    "entry_recovery": recovery, "report": report}
            if report.get("truncated"):
                local_fraction = report.get("path_fraction", 0.)
                if not 0 < local_fraction <= 1:
                    _reject("Invalid safe prefix from native preflight")
                checked_fraction = (i-1+local_fraction)/n
                seed = report["goal_joints_rad"]
                reports.append(report)
                prefix_reason = {"code": "command_limit", "error": report.get("limit")}
                break
        except DeviceError as error:
            error.details = {**(error.details or {}), "device_id": device_id,
                "workspace_clipping": workspace_clipping,
                "full_requested_goal": requested_goal, "preflight_segment_index": i,
                "preflight_segments": n, "completed_motion_segments": 0,
                "hardware_command_sent": False}
            if error.code != ErrorCode.LIMIT_VIOLATION or not reports:
                raise
            # Preserve previously verified progress. Nothing beyond the last
            # checked endpoint is sent; braking is replanned to that endpoint.
            prefix_reason = {"code": "command_limit", "error": error.to_dict()}
            break
        reports.append(report)
        checked_fraction = i/n
        seed, previous = report["goal_joints_rad"], endpoint
    if prefix_reason and prefix_reason["code"] == "command_limit":
        # Tells a path limit from an unreachable endpoint; it admits no motion.
        prefix_reason["goal_joint_check"] = driver._check_eef_goal(
            device_id, [("last_verified_prefix", list(seed)), ("measured_start", joints)], full_path.goal)
    if checked_fraction < 1.:
        fraction *= checked_fraction
        goal = path.at(checked_fraction)
        path = ControlSitePath(start, goal, offset)
        duration = max(.2, target.get("duration_s", .2), path.minimum_duration(limits))
        if opening is not None and current_opening is not None:
            duration = max(duration, 1.875*abs(opening-current_opening)/limits.gripper_speed_fraction_s)
    return {"start": start, "joints": joints, "goal": goal, "opening": opening, "duration": duration,
            "path": path, "requested_goal": requested_goal, "path_fraction": fraction,
            "workspace_clipping": workspace_clipping,
            "prefix_reason": prefix_reason,
            "report": {"checked": True, "complete_goal_checked": fraction == 1., "truncated": False,
                       "segments_checked": len(reports), "samples": sum(r.get("samples", 0) for r in reports),
                       "goal_joints_rad": seed, "collision_checked": False,
                       "execution": "single_quintic_no_intermediate_settle",
                       "path_reference": "registered_control_site", "control_offset_m": offset}}


class GroupCancel:
    def __init__(self, external):
        self.external, self.local = external, threading.Event()

    def is_set(self):
        return self.external.is_set() or self.local.is_set()


def move_group(driver, device_ids, params, cancel, progress):
    if set(params) - {"targets", "max_segments", "segment_m", "max_duration_s"}:
        _reject("Unsupported continuous group fields")
    targets = params.get("targets")
    count, segment_m = params.get("max_segments", 24), params.get("segment_m", .04)
    max_duration = params.get("max_duration_s", 30.)
    if (not isinstance(targets, dict) or set(targets) != set(device_ids) or not 1 <= len(device_ids) <= 2
            or type(count) is not int or not 1 <= count <= 24
            or type(segment_m) not in (int, float) or not math.isfinite(segment_m) or not 0 < segment_m <= .06
            or type(max_duration) not in (int, float) or not math.isfinite(max_duration) or not .2 <= max_duration <= 30):
        _reject("Invalid continuous group targets or budget")
    # No worker starts until every selected execution prefix has been checked.
    def cancelled():
        return {"completed": False, "partial": False, "hardware_command_sent": False,
                "devices": {}, "execution": "cancelled_before_dispatch"}
    if cancel.is_set():
        return cancelled()
    try:
        plans = {d: prepare(driver, d, targets[d], count, segment_m, cancel, max_duration) for d in device_ids}
    except PreflightCancelled:
        return cancelled()
    if cancel.is_set():
        return cancelled()
    for d, plan in plans.items():
        q, end = driver._require_fresh(d)
        dp, dr = pose_error(end, plan["start"])
        if max(abs(a-b) for a, b in zip(q[:6], plan["joints"])) > driver.tracking_limits.preflight_joint_rad or dp > driver.tracking_limits.preflight_position_m or dr > driver.tracking_limits.preflight_rotation_rad:
            raise DeviceError(ErrorCode.PRECONDITION_FAILED, "Arm moved during group preflight", stage="preflight")
    # Do not execute the original companion-arm action while recovering.
    if any(p.get("entry_recovery") for p in plans.values()):
        for p in plans.values():
            if not p.get("entry_recovery"):
                p.update(passive_hold=True, hold_joints=p["joints"], opening=None, goal=p["start"], path_fraction=0.,
                         prefix_reason={"code": "companion_recovery_hold", "requires_replan": True})
    # Keep held/shorter-path arms under feedback monitoring for the whole group.
    group_duration = max(p["duration"] for p in plans.values())
    for plan in plans.values():
        plan["duration"] = group_duration
    token, results, failures = GroupCancel(cancel), {}, {}
    lock = threading.Lock()
    fractions = {d: 0. for d in device_ids}

    def execute(d):
        plan = plans[d]
        def update(value):
            with lock:
                fractions[d] = value
                progress(min(fractions.values()))
        try:
            mode = ({"goal_joints": plan["hold_joints"], "passive_hold": plan.get("passive_hold", False)} if "hold_joints" in plan else
                    {"goal_end_pos": plan["goal"], "prepared_motion": plan})
            result = driver._interpolate(d, goal_opening=plan["opening"],
                duration_s=plan["duration"], cancel=token, progress=update, **mode)
            with lock:
                results[d] = {**result, "original_requested_end_pos": plan["requested_goal"],
                              "effective_command_end_pos": plan["goal"],
                              "workspace_clipping": plan.get("workspace_clipping"),
                              "executed_path_fraction": plan["path_fraction"],
                              "path_fraction_reference": "XYZ-clipped path, not original requested path",
                              "prefix_reason": result.get("prefix_reason") or plan["prefix_reason"], "entry_recovery": plan.get("entry_recovery"),
                              "command_joint_target_rad": plan.get("hold_joints"),
                              "requested_targets_reached": False if plan.get("prefix_reason") else result.get("requested_targets_reached", False)}
            if not result.get("completed"):
                token.local.set()
        except BaseException as error:
            if isinstance(error, DeviceError):
                error.details = {**(error.details or {}),
                    "original_requested_end_pos": plan["requested_goal"],
                    "effective_command_end_pos": plan["goal"],
                    "workspace_clipping": plan.get("workspace_clipping"),
                    "planned_path_fraction": plan["path_fraction"],
                    "prefix_reason": plan["prefix_reason"], "entry_recovery": plan.get("entry_recovery"),
                    "command_joint_target_rad": plan.get("hold_joints")}
            with lock:
                failures[d] = error
            token.local.set()

    with ThreadPoolExecutor(max_workers=len(device_ids), thread_name_prefix="arx-continuous") as pool:
        list(pool.map(execute, device_ids))
    if failures:
        known = all(isinstance(e, DeviceError) and e.execution_outcome in {"none", "partial"}
                    for e in failures.values())
        error = next(iter(failures.values()))
        details = dict(error.details or {}) if isinstance(error, DeviceError) else {}
        details["devices"] = {d: {"result": results[d]} if d in results else
                              {"error": failures[d].to_dict() if isinstance(failures[d], DeviceError)
                               else {"code": "unknown_execution"}} for d in device_ids}
        details["group_all_arrival_timeouts"] = all(isinstance(e, DeviceError)
            and e.code == ErrorCode.TRACKING_ERROR and (e.details or {}).get("reason") == "arrival_timeout"
            for e in failures.values())
        raise DeviceError(error.code if isinstance(error, DeviceError) else ErrorCode.OUTCOME_UNKNOWN,
            "Continuous group interrupted; inspect per-arm results", details=details,
            stage=error.stage if isinstance(error, DeviceError) else "execution",
            execution_outcome=("partial" if any(r.get("hardware_command_sent", True) for r in results.values()) or any(
                getattr(e, "execution_outcome", "unknown") == "partial" for e in failures.values()) else "none")
                if known else "unknown")
    completed = all(r.get("completed") for r in results.values())
    sent = any(r.get("hardware_command_sent", True) for r in results.values())
    if not completed and not cancel.is_set():
        # OperationTable.finish treats normal returns as success. An internal
        # abort is therefore an interruption, never a successful false result.
        raise DeviceError(ErrorCode.PRECONDITION_FAILED, "Continuous group aborted",
            stage="execution", execution_outcome="partial" if sent else "none",
            details={"devices": results, "group_all_arrival_timeouts": False})
    return {"completed": completed, "partial": not completed and sent,
            "hardware_command_sent": sent, "devices": results,
            "hold_reports": {d: r["hold_report"] for d, r in results.items() if r.get("hold_report")},
            "workspace_reports": {d: r["measured_workspace"] for d, r in results.items() if r.get("measured_workspace")},
            "requested_targets_reached": completed and all(
                p["path_fraction"] == 1. and not (p.get("workspace_clipping") or {}).get("requires_replan")
                and (p.get("passive_hold") or results[d].get("requested_targets_reached", True))
                for d, p in plans.items()),
            "execution": "single_quintic_no_intermediate_settle"}
