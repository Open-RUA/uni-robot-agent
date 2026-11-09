"""X5 command limits, sampled EEF preflight and feedback tracking protection.

The CAD URDF's +/-10 rad entries are not validated mechanical ranges. These
command bounds mirror the pinned SDK's ControllerBase constructor and
post-IK clipping loop; they are software compatibility bounds, not measured
mechanical stops. Newer vendor SDK documentation lists wider ranges, so any
profile change must also reconcile the installed controller's clipping.
They are not operator speed settings or a collision checker.
"""

from __future__ import annotations

import ctypes
import hashlib
import math
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, fields
from pathlib import Path

from ...core.contract import DeviceError, ErrorCode
from ...core.arm.eef import interpolate_pose, pose_error, rotation_distance
from .limits import EEF_LOWER, EEF_UPPER, controller_profile, joint_limit_details


TRUNCATION_JOINT_MARGIN_RAD = 0.03
# Raw IK joint spans, not Euler-angle differences or a per-Move rotation cap.
# Keep the existing finer continuity/refinement guard independently.
IK_JOINT_JUMP_REJECT_RAD = 1.0
IK_JOINT_REFINE_RAD = 0.15
# Recovery must enter the interval rather than landing exactly on its edge.
# This is an anti-chatter margin, not a guaranteed bound on tracking error.
RECOVERY_ATTITUDE_MARGIN_RAD = 0.02
RECOVERY_BRIDGE_SAMPLES = 128


def workspace_status(pose):
    """Describe native measured/command pose limits without clipping feedback."""
    if pose is None or len(pose) != 6 or not all(math.isfinite(v) for v in pose):
        return {"valid": False, "within_limits": False, "reason": "invalid_pose",
                "frame": "controller_eef_native", "violations": []}
    margins = [min(v-lo, hi-v) for v, lo, hi in zip(pose, EEF_LOWER, EEF_UPPER)]
    return {"valid": True, "within_limits": all(m >= 0 for m in margins),
            "frame": "controller_eef_native", "pose": list(pose),
            "margins": margins, "units": ["m"]*3 + ["rad"]*3,
            "violations": [{"axis": i, "value": pose[i], "lower": EEF_LOWER[i],
                            "upper": EEF_UPPER[i], "excess": -m}
                           for i, m in enumerate(margins) if m < 0]}


def clip_eef_xyz(goal):
    """Project a command goal into the pinned SDK XYZ box, never its attitude.

    This is goal selection, not IK success or collision clearance. The caller
    must preflight and execute the same corrected path and retain this evidence.
    """
    if len(goal) != 6 or not all(math.isfinite(v) for v in goal):
        raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "EEF pose must contain six finite values", stage="preflight")
    effective = list(goal)
    axes = []
    for i in range(3):
        effective[i] = min(EEF_UPPER[i], max(EEF_LOWER[i], goal[i]))
        if effective[i] != goal[i]:
            axes.append({"axis": "xyz"[i], "requested_m": goal[i],
                         "effective_m": effective[i], "lower_m": EEF_LOWER[i], "upper_m": EEF_UPPER[i]})
    correction = math.dist(goal[:3], effective[:3])
    evidence = None if not axes else {
        "code": "workspace_clipped", "frame": "controller_eef_native",
        "requested_end_pos": list(goal), "effective_end_pos": effective,
        "axes": axes, "correction_distance_m": correction,
        "requires_replan": correction > 1e-12,
        "kind": "xyz_clipping" if correction > 1e-12 else "floating_point_normalization",
    }
    return effective, evidence


def check_joint_target(goal, *, profile=None, start=None):
    """Check that a joint destination lies inside the controller profile's bounds.

    Only the destination is checked: a start beyond a bound may move back inside it.
    ``start`` is accepted for call-site readability and is not used.
    """
    if len(goal) != 6 or not all(math.isfinite(v) for v in goal):
        raise DeviceError(
            ErrorCode.INVALID_ARGUMENTS,
            "expected six finite joint targets",
            stage="preflight",
        )
    profile = controller_profile(profile)
    for i, (v, lo, hi) in enumerate(zip(goal, profile.joint_lower, profile.joint_upper)):
        if lo <= v <= hi:
            continue
        raise DeviceError(
            ErrorCode.LIMIT_VIOLATION,
            f"joint {i + 1} target exceeds SDK command limits",
            stage="preflight",
            details=joint_limit_details(i, v, profile),
        )



class NativeKinematics:
    """One bounded KDL solver per arm; ctypes releases the GIL during IK."""

    def __init__(self, root: Path, *, profile=None):
        self.profile = controller_profile(profile)
        base = root / "native/ros"
        model = base / "share/arx_x5_controller" / self.profile.model_filename
        vendor = base / "lib" / self.profile.sdk_filename
        for path, expected in ((model, self.profile.model_sha256), (vendor, self.profile.sdk_sha256)):
            if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                raise ValueError(f"unverified X5 safety profile: {path.name}")
        self.lib = ctypes.CDLL(str(base / "lib/librua_arx_kinematics.so"))
        pointer = ctypes.POINTER(ctypes.c_double)
        for name in ("fk", "ik"):
            fn = getattr(self.lib, f"rua_kinematics_{name}")
            fn.argtypes = [ctypes.c_void_p] + [pointer] * (3 if name == "ik" else 2)
            fn.restype = ctypes.c_int
        self.lib.rua_kinematics_create.argtypes = [ctypes.c_char_p]
        self.lib.rua_kinematics_create.restype = ctypes.c_void_p
        self.lib.rua_kinematics_destroy.argtypes = [ctypes.c_void_p]
        self.lib.rua_kinematics_destroy.restype = None
        self.handle = self.lib.rua_kinematics_create(str(model).encode())
        if not self.handle:
            raise ValueError("cannot load X5 kinematic chain")
        self.lock = threading.RLock()
        self.lib.rua_kinematics_origin.argtypes = [ctypes.c_void_p, pointer]
        self.lib.rua_kinematics_origin.restype = ctypes.c_int
        origin = (ctypes.c_double * 3)()
        self.lib.rua_kinematics_origin(self.handle, origin)
        self.origin = list(origin)

    @staticmethod
    def _array(values):
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise ValueError("expected six finite values")
        return (ctypes.c_double * 6)(*values)

    def fk(self, joints):
        out = (ctypes.c_double * 6)()
        with self.lock:
            if not self.handle or self.lib.rua_kinematics_fk(
                self.handle, self._array(joints), out
            ):
                raise ValueError("FK failed")
        return list(out)

    def ik(self, seed, pose):
        # SDK stateEndControl clips the native EEF pose BEFORE its IK solver
        # adds the zero-joint URDF origin. Match that coordinate frame here;
        # rua_kinematics_ik remains responsible for the one origin addition.
        native_pose = self._array(pose)
        # Only suppress machine-precision dust at XYZ boundaries. Meaningful
        # goal clipping belongs before path planning, never inside the IK loop.
        for i in range(3):
            clipped = min(EEF_UPPER[i], max(EEF_LOWER[i], native_pose[i]))
            if abs(clipped - native_pose[i]) <= 1e-12:
                native_pose[i] = clipped
        for i, (value, lo, hi) in enumerate(zip(native_pose, EEF_LOWER, EEF_UPPER)):
            if not lo <= value <= hi:
                raise DeviceError(
                    ErrorCode.LIMIT_VIOLATION,
                    "EEF target exceeds SDK workspace bounds",
                    stage="preflight",
                    details={
                        "axis": i,
                        "target": value,
                        "lower": lo,
                        "upper": hi,
                        "frame": "controller_eef_native",
                    },
                )
        out = (ctypes.c_double * 8)()
        with self.lock:
            if not self.handle or self.lib.rua_kinematics_ik(
                self.handle, self._array(seed), native_pose, out
            ):
                raise ValueError("IK failed")
        return list(out[:6]), out[6], out[7]

    def close(self):
        with self.lock:
            if self.handle:
                self.lib.rua_kinematics_destroy(self.handle)
                self.handle = None


class EefPreflight:
    def __init__(self, solver, *, profile=None):
        self.solver = solver
        self.profile = controller_profile(profile if profile is not None else getattr(solver, "profile", None))
        if hasattr(solver, "profile") and self.profile != solver.profile:
            raise ValueError("IK solver and command limit profiles differ")

    def _entry_recovery(self, joints, start, goal, count, path):
        """Find an in-range IK endpoint, then validate the real joint bridge.

        No hardware action here. Invalid IK is not a skippable limit prefix.
        The execution layer must stop at this joint endpoint and re-observe.
        """
        lo, hi = self.profile.joint_lower, self.profile.joint_upper
        def outside(values, lower, upper):
            return [max(a-v, 0., v-b) for v, a, b in zip(values, lower, upper)]
        initial_q = outside(joints, lo, hi)
        initial_workspace = outside(start, EEF_LOWER, EEF_UPPER)
        recovery_axes = [i for i in range(3, 6) if initial_workspace[i] > 0]
        skipped = []
        seed = list(joints)
        for i in range(1, count + 1):
            fraction = i/count
            target = path(fraction) if path else interpolate_pose(start, goal, fraction)
            status = workspace_status(target)
            if not status["valid"]:
                raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Invalid recovery pose", stage="preflight")
            if not status["within_limits"] or any(
                    status["margins"][a] < RECOVERY_ATTITUDE_MARGIN_RAD for a in recovery_axes):
                skipped.append({"fraction": fraction, "reason": "workspace_entry_not_interior"})
                continue
            try:
                q, pos, rot = self.solver.ik(seed, target)
            except (ValueError, RuntimeError) as exc:
                raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Recovery IK failed", stage="preflight",
                                  details={"reason": "recovery_ik_failed"}) from exc
            if len(q) != 6 or not all(math.isfinite(v) for v in q) or not math.isfinite(pos) or not math.isfinite(rot) or pos > .0005 or rot > .005:
                raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Invalid recovery IK", stage="preflight")
            if max(abs(a-b) for a,b in zip(seed,q)) > IK_JOINT_JUMP_REJECT_RAD:
                raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Recovery IK branch jump", stage="preflight", details={"reason": "joint_jump_rejected"})
            if any(outside(q,lo,hi)):
                skipped.append({"fraction": fraction, "reason": "joint_outside", "joints_rad": list(q)})
                seed = list(q)
                continue
            # A distant first legal point is not a short automatic recovery.
            if max(abs(a-b) for a,b in zip(joints,q)) > IK_JOINT_REFINE_RAD:
                raise DeviceError(ErrorCode.LIMIT_VIOLATION, "First legal recovery point is too far from measured start", stage="preflight", details={"reason": "recovery_bridge_too_large"})
            previous_q, previous_workspace = initial_q, initial_workspace
            for k in range(1, RECOVERY_BRIDGE_SAMPLES + 1):
                bridge = [a+(b-a)*k/RECOVERY_BRIDGE_SAMPLES for a,b in zip(joints,q)]
                dq = outside(bridge,lo,hi)
                # SDK clips the joint command; validate the effective geometry.
                effective = [min(b,max(a,v)) for v,a,b in zip(bridge,lo,hi)]
                pose = self.solver.fk(bridge)
                effective_pose = self.solver.fk(effective)
                if not all(math.isfinite(v) for v in [*pose, *effective_pose]):
                    raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Invalid recovery FK", stage="preflight")
                dx = [max(a,b) for a,b in zip(outside(pose, EEF_LOWER, EEF_UPPER),
                      outside(effective_pose, EEF_LOWER, EEF_UPPER))]
                if any(b > a+1e-12 for a,b in zip(previous_q,dq)) or any(b > a+1e-12 for a,b in zip(previous_workspace,dx)):
                    raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Recovery bridge increases or introduces a limit violation", stage="preflight")
                previous_q, previous_workspace = dq, dx
            if any(previous_workspace):
                raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Recovery endpoint remains outside workspace", stage="preflight")
            endpoint_status = workspace_status(effective_pose)
            if any(endpoint_status["margins"][a] < RECOVERY_ATTITUDE_MARGIN_RAD for a in recovery_axes):
                raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Recovery FK endpoint lacks inward margin", stage="preflight")
            check_joint_target(q, profile=self.profile)
            return {"checked": True, "truncated": False, "path_fraction": 0.,
                    "requested_goal_end_pos": list(goal), "effective_goal_end_pos": list(self.solver.fk(q)),
                    "goal_joints_rad": list(q), "samples": i, "collision_checked": False,
                    "entry_recovery": {"kind": "joint_entry_recovery", "planned_only": True,
                        "start_joints_rad": list(joints), "start_end_pos": list(start),
                        "target_joints_rad": list(q), "candidate_fraction": fraction,
                        "skipped_prefix_count": len(skipped), "skipped_prefix": skipped,
                        "start_workspace": workspace_status(start),
                        "endpoint_workspace": endpoint_status,
                        "attitude_margin_rad": RECOVERY_ATTITUDE_MARGIN_RAD,
                        "bridge_samples": RECOVERY_BRIDGE_SAMPLES, "must_replan_after_idle": True}}
        raise DeviceError(ErrorCode.LIMIT_VIOLATION, "No legal recovery point in bounded path", stage="preflight",
                          details={"skipped_prefix": skipped, "hardware_command_sent": False})

    def goal_check(self, seeds, goal):
        """IK for the endpoint alone against the SDK bounds. No hardware action.

        The path to the endpoint is not checked, and only the given seeds are
        tried: other IK branches are not searched, so a missing solution is
        unknown rather than unreachable.
        """
        basis = {"frame": "controller_eef_native", "goal_end_pos": list(goal),
                 "path_checked": False, "ik_branches": "seeded_local_solutions_only"}
        workspace = workspace_status(goal)
        if not workspace["within_limits"]:
            return {"status": "outside_workspace", "workspace_violations": workspace["violations"], **basis}
        lo, hi = self.profile.joint_lower, self.profile.joint_upper
        outside = None
        for name, seed in seeds:
            try:
                q, pos, rot = self.solver.ik(seed, goal)
            except (DeviceError, ValueError, RuntimeError):
                continue
            if len(q) != 6 or not all(math.isfinite(v) for v in q) or pos > 0.0005 or rot > 0.005:
                continue
            violations = [joint_limit_details(i, v, self.profile)
                          for i, (v, a, b) in enumerate(zip(q, lo, hi)) if not a <= v <= b]
            candidate = {"seed": name, "joints_rad": list(q)}
            if not violations:
                return {"status": "within_joint_limits", **candidate, **basis}
            outside = outside or {**candidate, "joint_violations": violations}
        if outside:
            return {"status": "outside_joint_limits", **outside, **basis}
        return {"status": "no_verified_solution", "seeds": [name for name, _ in seeds], **basis}

    def check(self, joints, start, goal, *, on_limit="reject", path=None, path_length_m=None, clip_xyz=False):
        validate_eef_limit_policy(on_limit)
        began = time.perf_counter()
        error = pose_error(self.solver.fk(joints), start)
        if error[0] > 0.005 or error[1] > 0.05:
            raise DeviceError(
                ErrorCode.PRECONDITION_FAILED,
                "joint/EEF feedback does not match the verified URDF frame",
                stage="preflight",
                details={"position_error_m": error[0], "rotation_error_rad": error[1]},
            )
        if len(joints) != 6 or not all(math.isfinite(v) for v in joints):
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "invalid measured joint start", stage="preflight")
        start_violations = [joint_limit_details(i, v, self.profile)
                            for i, (v, lo, hi) in enumerate(zip(joints, self.profile.joint_lower, self.profile.joint_upper))
                            if not lo <= v <= hi]
        # Preserve real feedback. Only intermediate IK candidates may recover
        # monotonically from an already violated bound; never relax a target.
        previous_raw = list(joints)
        recovery_samples = []
        count = max(
            1,
            math.ceil((math.dist(start[:3], goal[:3]) if path_length_m is None else path_length_m) / 0.002),
            math.ceil(rotation_distance(start[3:], goal[3:]) / 0.01),
        )
        if count > 256:
            raise DeviceError(
                ErrorCode.LIMIT_VIOLATION,
                "EEF path exceeds bounded preflight sampling budget",
                stage="preflight",
            )
        # A legal joint vector can still put the EEF outside its command box.
        # Such feedback cannot be fed through ordinary EEF interpolation: its
        # first points would be rejected before an inward goal can be reached.
        # Keep ordinary XYZ clipping for its existing zero-origin tolerance;
        # attitude recovery must use an explicitly checked joint bridge.
        if start_violations or any(not EEF_LOWER[i] <= start[i] <= EEF_UPPER[i] for i in range(3, 6)):
            return self._entry_recovery(joints, start, goal, count, path)
        samples = 0
        safe_prefix = None
        clipped_samples = []

        def solve_segment(seed, previous, fraction, depth=0):
            nonlocal samples, safe_prefix, previous_raw
            samples += 1
            target = interpolate_pose(start, goal, fraction) if path is None else path(fraction)
            if clip_xyz:
                target, clipping = clip_eef_xyz(target)
                if clipping is not None:
                    clipped_samples.append({"path_fraction": fraction, **clipping})
            try:
                q, pos, rot = self.solver.ik(seed, target)
            except DeviceError as exc:
                exc.details = {**(exc.details or {}), "path_fraction": fraction}
                if exc.details.get("frame") == "controller_eef_native":
                    axis = exc.details["axis"]
                    exc.details.update(start_axis_value=start[axis], goal_axis_value=goal[axis],
                        start_outside_workspace=not EEF_LOWER[axis] <= start[axis] <= EEF_UPPER[axis],
                        target_exact=repr(target[axis]),
                        recovery_hint="Do not replay this path. Use a verified joint-space recovery or a newly verified reachable path; workspace limits remain unchanged.")
                raise
            details = {
                "path_fraction": fraction,
                "position_residual_m": pos,
                "rotation_residual_rad": rot,
            }
            if pos > 0.0005 or rot > 0.005:
                raise DeviceError(
                    ErrorCode.LIMIT_VIOLATION,
                    "EEF path has no verified IK solution on the current branch",
                    stage="preflight",
                    details=details,
                )
            if len(q) != 6 or not all(math.isfinite(v) for v in q):
                raise DeviceError(ErrorCode.LIMIT_VIOLATION,
                                  "EEF IK returned invalid joint targets",
                                  stage="preflight",
                                  details={**details, "reason": "invalid_joint_target"})
            # Do not wrap modulo 2*pi: these are bounded hardware joint
            # commands, so wrapping would hide a commanded branch jump.
            delta = [b - a for a, b in zip(seed, q)]
            jump_details = {
                **details,
                "reason": "joint_jump_rejected",
                "previous_path_fraction": previous,
                "current_joints_rad": list(seed),
                "target_joints_rad": list(q),
                "joint_delta_rad": delta,
                "max_joint_delta_rad": max(abs(v) for v in delta),
                "joint_delta_limit_rad": IK_JOINT_JUMP_REJECT_RAD,
                "continuity_refine_limit_rad": IK_JOINT_REFINE_RAD,
                "refinement_depth": depth,
                "recovery_hint": "Read the execution receipt and actual pose; after confirmed idle, replan a different continuous path. Do not replay this rejected IK target.",
            }
            if jump_details["max_joint_delta_rad"] > IK_JOINT_JUMP_REJECT_RAD:
                raise DeviceError(
                    ErrorCode.LIMIT_VIOLATION,
                    "EEF IK joint jump exceeds the command continuity guard",
                    stage="preflight",
                    details={**jump_details, "exceeded_joint_indices": [
                        i for i, value in enumerate(delta)
                        if abs(value) > IK_JOINT_JUMP_REJECT_RAD
                    ], "joint_index_base": 0},
                )
            # Refine large changes near singularities instead of jumping branches.
            if jump_details["max_joint_delta_rad"] > IK_JOINT_REFINE_RAD:
                if depth >= 5 or samples >= 1024:
                    raise DeviceError(
                        ErrorCode.LIMIT_VIOLATION,
                        "EEF path has an IK branch discontinuity",
                        stage="preflight",
                        details={**jump_details, "reason": "ik_branch_discontinuity"},
                    )
                mid = (previous + fraction) / 2
                return solve_segment(
                    solve_segment(seed, previous, mid, depth + 1),
                    mid,
                    fraction,
                    depth + 1,
                )
            raw_q = list(q)
            effective_q = list(q)
            for i, (value, old, lo, hi) in enumerate(zip(raw_q, previous_raw, self.profile.joint_lower, self.profile.joint_upper)):
                if lo <= value <= hi:
                    continue
                inward = (old < lo and old <= value < lo) or (old > hi and hi < value <= old)
                if not inward or fraction >= 1.0:
                    raise DeviceError(ErrorCode.LIMIT_VIOLATION,
                        "IK target does not recover into SDK joint bounds",
                        stage="preflight", details={**joint_limit_details(i, value, self.profile),
                        **details, "reason": "joint_recovery_not_inward" if fraction < 1.0 else "joint_goal_outside_limits"})
                effective_q[i] = min(hi, max(lo, value))
            if effective_q != raw_q:
                # The SDK clips IK joints. Verify that actual command geometry,
                # not an unreachable raw solution, with the existing residuals.
                residual = pose_error(self.solver.fk(effective_q), target)
                if residual[0] > 0.0005 or residual[1] > 0.005:
                    raise DeviceError(ErrorCode.LIMIT_VIOLATION,
                        "SDK-clipped recovery IK does not match the requested path",
                        stage="preflight", details={**details, "reason": "recovery_clipped_fk_mismatch",
                        "position_residual_m": residual[0], "rotation_residual_rad": residual[1]})
                recovery_samples.append({"path_fraction": fraction, "raw_joints_rad": raw_q,
                                         "sdk_effective_joints_rad": effective_q})
            check_joint_target(effective_q, profile=self.profile)
            previous_raw = raw_q
            q = effective_q
            if all(
                min(v - lo, hi - v) >= TRUNCATION_JOINT_MARGIN_RAD
                for v, lo, hi in zip(q, self.profile.joint_lower, self.profile.joint_upper)
            ):
                safe_prefix = (fraction, list(target), list(q))
            return q

        seed = joints
        fraction, effective_goal, limit = 1.0, list(goal), None
        try:
            # The first published EEF command is also clipped. Verify that
            # boundary entry against the real joint seed, never a fabricated
            # in-bounds starting state. All normal IK/joint guards still apply.
            if clip_xyz and clip_eef_xyz(start)[1] is not None:
                seed = solve_segment(seed, 0., 0.)
            for i in range(1, count + 1):
                seed = solve_segment(seed, (i - 1) / count, i / count)
        except DeviceError as exc:
            if (
                on_limit != "truncate"
                or exc.code != ErrorCode.LIMIT_VIOLATION
                or safe_prefix is None
            ):
                exc.details = {**(exc.details or {}), "start_joint_limit_violations": start_violations,
                               "joint_recovery_samples": recovery_samples[:8], "workspace_path_clipping": {
                    "sample_count": len(clipped_samples),
                    "samples": clipped_samples[:8],
                }}
                raise
            fraction, effective_goal, seed = safe_prefix
            distance, rotation = pose_error(start, effective_goal)
            if distance < 0.002 and rotation < 0.01:
                raise  # No useful prefix; never send a token motion on failure.
            limit = exc.to_dict()
        return {
            "checked": True,
            "start_joint_limit_violations": start_violations,
            "joint_recovery": {"required": bool(start_violations),
                               "planned_only": True,
                               # A truncated path never solved its goal.
                               "goal_within_limits": True if limit is None else None,
                               "clipped_intermediate_samples": len(recovery_samples),
                               "samples": recovery_samples[:8]},
            "truncated": limit is not None,
            "path_fraction": fraction,
            "requested_goal_end_pos": list(goal),
            "effective_goal_end_pos": effective_goal,
            "limit": limit,
            "truncation_joint_margin_rad": TRUNCATION_JOINT_MARGIN_RAD,
            "samples": samples,
            "elapsed_ms": round((time.perf_counter() - began) * 1000, 3),
            "goal_joints_rad": seed,
            "max_sample_translation_m": 0.002,
            "max_sample_rotation_rad": 0.01,
            "ik_joint_jump_reject_rad": IK_JOINT_JUMP_REJECT_RAD,
            "ik_joint_refine_rad": IK_JOINT_REFINE_RAD,
            "orientation_interpolation": "quaternion_shortest_arc_slerp",
            "collision_checked": False,
            "workspace_path_clipping": {
                "sample_count": len(clipped_samples),
                "samples": clipped_samples[:8],
                "max_correction_m": max((s["correction_distance_m"] for s in clipped_samples), default=0.),
                "sample_basis": "preflight, not executed commands",
            },
        }


def validate_eef_limit_policy(value):
    if value not in ("reject", "truncate"):
        raise ValueError("eef_limit_policy must be reject or truncate")
    return value


@dataclass(frozen=True)
class TrackingLimits:
    position_m: float = 0.030
    rotation_rad: float = 0.20
    lag_s: float = 0.12
    violation_s: float = 0.06
    settle_position_m: float = 0.030
    settle_rotation_rad: float = 0.20
    settle_joint_rad: float = 0.1
    hold_admission_joint_rad: float = 0.1
    preflight_joint_rad: float = 0.1
    preflight_position_m: float = 0.01
    preflight_rotation_rad: float = 0.1
    settle_gripper_opening: float = 0.05
    settle_duration_s: float = 0.05
    settle_max_gap_s: float = 0.04
    settle_timeout_s: float = 0.6

    @classmethod
    def from_config(cls, config=None):
        if config is None:
            return cls()
        if not isinstance(config, dict) or set(config) - {f.name for f in fields(cls)}:
            raise ValueError("unknown or invalid tracking_limits fields")
        if any(
            isinstance(v, bool)
            or not isinstance(v, (int, float))
            or not math.isfinite(v)
            or v <= 0
            for v in config.values()
        ):
            raise ValueError("tracking_limits values must be finite positive numbers")
        result = cls(**config)
        if (
            result.settle_position_m > result.position_m
            or result.settle_rotation_rad > result.rotation_rad
        ):
            raise ValueError("settle tolerances cannot exceed tracking tolerances")
        if result.settle_duration_s >= result.settle_timeout_s:
            raise ValueError("settle confirmation must fit inside settle_timeout_s")
        if result.settle_gripper_opening >= 1:
            raise ValueError("settle_gripper_opening must be less than one")
        return result

    def to_dict(self):
        return asdict(self)


class ArrivalWindow:
    """Confirm tolerance over distinct feedback samples, never over poll time."""

    def __init__(self, limits):
        self.limits = limits
        self.last_stamp = None
        self.since = None
        self.samples = 0

    def update(self, stamp, in_tolerance):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        gap = self.last_stamp is not None and (
            stamp - self.last_stamp > self.limits.settle_max_gap_s
        )
        self.last_stamp = stamp
        if gap or not in_tolerance:
            self.since, self.samples = None, 0
        if not in_tolerance:
            return False
        if self.since is None:
            self.since = stamp
        self.samples += 1
        return (
            self.samples >= 3
            and stamp - self.since + 1e-9 >= self.limits.settle_duration_s
        )


class PoseTrackingGuard:
    def __init__(self, limits, now, start):
        self.limits = limits
        self.commands = deque([(now, list(start))])
        self.violation_since = None
        self.last_feedback_at = None

    def sent(self, now, pose):
        self.commands.append((now, list(pose)))

    def check(self, feedback_at, pose):
        if feedback_at == self.last_feedback_at:
            return None
        self.last_feedback_at = feedback_at
        # Preserve the bracketing command at the lag boundary. Compare position
        # and orientation to the SAME command, not separate convenient samples.
        while (
            len(self.commands) > 1
            and self.commands[1][0] < feedback_at - self.limits.lag_s
        ):
            self.commands.popleft()
        errors = [
            pose_error(pose, target)
            for stamp, target in self.commands
            if stamp <= feedback_at
        ]
        if not errors:
            return None
        pos, rot = min(
            errors,
            key=lambda e: max(
                e[0] / self.limits.position_m, e[1] / self.limits.rotation_rad
            ),
        )
        if pos <= self.limits.position_m and rot <= self.limits.rotation_rad:
            self.violation_since = None
            return None
        if self.violation_since is None:
            self.violation_since = feedback_at
        if feedback_at - self.violation_since + 1e-9 < self.limits.violation_s:
            return None
        return {
            "reason": "tracking_deviation",
            "exceeded_components": [name for name, value, limit in (
                ("position", pos, self.limits.position_m), ("rotation", rot, self.limits.rotation_rad))
                if value > limit],
            "tracking_limits": self.limits.to_dict(),
            "position_error_m": pos,
            "rotation_error_rad": rot,
            "violation_s": feedback_at - self.violation_since,
            "allowed_lag_s": self.limits.lag_s,
        }
