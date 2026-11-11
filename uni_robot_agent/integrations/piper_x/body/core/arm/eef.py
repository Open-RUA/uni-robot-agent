"""End-effector rotations, poses and paths shared by Body drivers."""

from __future__ import annotations

import bisect
import math

from .motion_limits import MotionLimits


def normalize(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError("expected four finite quaternion components (w,x,y,z)")
    norm = math.hypot(*q)
    if norm < 1e-15:
        raise ValueError("zero quaternion has no orientation")
    return tuple(v / norm for v in q)


def quaternion(rpy):
    r, p, y = (v / 2 for v in rpy)
    cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
    sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
    return normalize((
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ))


def rotation_distance(a, b):
    """Geodesic angle in radians between two RPY orientations, in [0, pi]."""
    qa, qb = quaternion(a), quaternion(b)
    if sum(x * y for x, y in zip(qa, qb)) < 0:
        qb = tuple(-v for v in qb)
    # Chord form retains precision for small corrections (unlike acos(dot)).
    return 4 * math.atan2(math.dist(qa, qb), math.hypot(*(x + y for x, y in zip(qa, qb))))


def slerp(a, b, fraction):
    """Interpolate unit quaternions; q and -q describe the same rotation."""
    if not math.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError("SLERP fraction must be between zero and one")
    a, b = normalize(a), normalize(b)
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-v for v in b), -dot
    theta = math.acos(min(1.0, dot))
    if theta < 1e-8:
        return normalize(tuple(x + fraction * (y - x) for x, y in zip(a, b)))
    scale = math.sin(theta)
    u, v = math.sin((1 - fraction) * theta) / scale, math.sin(fraction * theta) / scale
    return normalize(tuple(u * x + v * y for x, y in zip(a, b)))


def rpy(q, reference):
    """Equivalent RPY near reference; unwrap the SDK representation only.

    At pitch +/-pi/2, keep reference roll and solve the coupled yaw instead
    of extracting two undefined angles. This does not remove a robot's IK
    singularities or bypass its workspace limits.
    """
    w, x, y, z = normalize(q)
    r00, r10 = 1 - 2 * (y*y + z*z), 2 * (x*y + w*z)
    sin_pitch = 2 * (w*y - z*x)
    cos_pitch = math.hypot(r00, r10)
    pitch = math.atan2(sin_pitch, cos_pitch)
    if cos_pitch < 1e-10:
        roll = reference[0]
        yaw = math.atan2(2 * (w*z - x*y), 1 - 2 * (x*x + z*z))
        yaw += math.copysign(1.0, sin_pitch) * roll
    else:
        roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
        yaw = math.atan2(r10, r00)
    candidates = ((roll, pitch, yaw), (roll + math.pi, math.pi - pitch, yaw + math.pi))
    unwrapped = [
        [ref + math.atan2(math.sin(v-ref), math.cos(v-ref)) for v, ref in zip(candidate, reference)]
        for candidate in candidates
    ]
    return min(unwrapped, key=lambda candidate: math.dist(candidate, reference))


def interpolate_pose(start, goal, fraction):
    """Shared geometric path for IK preflight, truncation and command streaming."""
    if not math.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError("pose fraction must be between zero and one")
    if fraction == 0:
        return list(start)
    position = [a + fraction * (b-a) for a, b in zip(start[:3], goal[:3])]
    # Keep exact hold values rather than accumulating conversion noise.
    if start[3:] == goal[3:]:
        return position + list(start[3:])
    orientation = slerp(quaternion(start[3:]), quaternion(goal[3:]), fraction)
    return position + rpy(orientation, start[3:])


CONTRACT_VERSION = "eef_absolute_v2"
VECTOR_SCHEMA = {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3}
TARGET_PROPERTIES = {
    "target_position_m": {**VECTOR_SCHEMA, "description": "Absolute controller-EEF origin position, in metres. Not a surface point or gripper contact centre."},
    "target_rpy_rad": {**VECTOR_SCHEMA, "description": "Absolute roll/pitch/yaw in radians, R=Rz(yaw)Ry(pitch)Rx(roll). Omit to hold orientation; [0,0,0] requests zero orientation."},
    "target_frame": {"type": "string", "description": "Exact selected arm controller frame from describe_robot, e.g. right_arm/base. Camera, bare base, URDF base_link and the other arm are rejected."},
}
TARGET_REQUIRED = ["target_position_m", "target_frame"]


def finite_vector(value, name):
    if (not isinstance(value, list) or len(value) != 3 or any(
        isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
        for v in value
    )):
        raise ValueError(f"{name} needs three finite numbers")
    return [float(v) for v in value]


def validate_target(position, orientation, frame, device_id, *, expected_frame=None):
    expected = expected_frame or f"{device_id}/base"
    if frame != expected:
        raise ValueError(f"target_frame must be exactly {expected}; no implicit frame conversion")
    result = {"target_position_m": finite_vector(position, "target_position_m"), "target_frame": expected}
    if orientation is not None:
        rpy = finite_vector(orientation, "target_rpy_rad")
        if any(abs(v) > math.pi + 1e-9 for v in rpy):
            raise ValueError("target_rpy_rad must use radians in [-pi, pi], not degrees")
        result["target_rpy_rad"] = rpy
    return result


def angular_delta(target, current):
    """Shortest equivalent per-RPY-component difference, not a rotation vector."""
    return [math.atan2(math.sin(t - c), math.cos(t - c)) for t, c in zip(target, current)]


def resolve_goal(start, target, limits=None):
    """Resolve omitted orientation and travel from one fresh execution snapshot.

Absolute XYZ is never added to feedback. RPY output keeps the controller's
Euler convention, choosing the nearest equivalent angle across +/-pi.
"""
    position = finite_vector(target.get("target_position_m"), "target_position_m")
    rotation = angular_delta(target.get("target_rpy_rad", start[3:6]), start[3:6])
    delta = [t - c for t, c in zip(position, start[:3])]
    if limits is not None and (
        math.hypot(*delta) > limits.eef_step_m + 1e-12
        or rotation_distance(start[3:6], target.get("target_rpy_rad", start[3:6]))
        > limits.eef_rotation_step_rad + 1e-12
    ):
        raise ValueError("absolute EEF target exceeds per-segment translation/rotation limits from the current pose")
    return position + [c + d for c, d in zip(start[3:6], rotation)]


def coordinate_metadata(device_id):
    return {
        "contract": CONTRACT_VERSION,
        "target_kind": "absolute",
        "frame_id": f"{device_id}/base",
        "eef_frame_id": f"{device_id}/eef",
        "controlled_point": "controller_eef_origin",
        "position_unit": "m",
        "orientation_unit": "rad",
        "orientation_convention": "Rz(yaw) Ry(pitch) Rx(roll)",
        "orientation_interpolation": "quaternion_shortest_arc_slerp",
        "origin": "driver-defined; relationship to URDF must be declared separately",
        "not_equivalent_to": ["URDF base_link", "camera optical frame", "gripper contact centre"],
    }


def pose_error(a, b):
    """Euclidean translation and shortest rotation distance for XYZ/RPY poses."""
    return math.dist(a[:3], b[:3]), rotation_distance(a[3:], b[3:])


MAX_WAYPOINTS = 128
MAX_PATH_DURATION_S = 600.0
WAYPOINTS_SCHEMA = {
    "type": "array",
    "minItems": 1,
    "maxItems": MAX_WAYPOINTS,
    "items": {
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "target_position_m": VECTOR_SCHEMA,
            "target_rpy_rad": VECTOR_SCHEMA,
            "duration_s": {"type": "number", "minimum": 0.2, "maximum": 30},
        },
        "required": ["target_position_m"],
    },
}


def validate_waypoints(waypoints, limits: MotionLimits | None = None):
    """Validate absolute waypoints; execution checks travel from fresh feedback.

    Omitted orientation holds the preceding waypoint orientation (the execution
    start orientation for the first waypoint). Old delta fields are rejected.
    """
    if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= MAX_WAYPOINTS:
        raise ValueError(f"path needs 1..{MAX_WAYPOINTS} waypoints")
    result = []
    for point in waypoints:
        if not isinstance(point, dict) or set(point) - WAYPOINTS_SCHEMA["items"]["properties"].keys():
            raise ValueError("invalid waypoint fields; use absolute target_position_m/target_rpy_rad")
        item = {"target_position_m": finite_vector(point.get("target_position_m"), "target_position_m")}
        if "target_rpy_rad" in point:
            rpy = finite_vector(point["target_rpy_rad"], "target_rpy_rad")
            if any(abs(v) > math.pi + 1e-9 for v in rpy):
                raise ValueError("target_rpy_rad must be in [-pi, pi] radians")
            item["target_rpy_rad"] = rpy
        duration = point.get("duration_s", 0.2)
        if isinstance(duration, bool) or not isinstance(duration, (float, int)) or not math.isfinite(duration) or not 0.2 <= duration <= 30:
            raise ValueError("waypoint duration_s must be in [0.2, 30]")
        item["duration_s"] = float(duration)
        if result and limits is not None:
            if math.dist(result[-1]["target_position_m"], item["target_position_m"]) > limits.eef_step_m + 1e-12:
                raise ValueError("waypoint separation exceeds Body motion limits")
        result.append(item)
    return result


def _same_direction(a, b):
    # Merge only identical lines in all six coordinates. No rounded corners or
    # unchecked shortcuts; reversals retain a zero-speed/zero-acceleration knot.
    aa, bb = sum(v * v for v in a), sum(v * v for v in b)
    if aa < 1e-18 or bb < 1e-18:
        return False
    scale = sum(x * y for x, y in zip(a, b)) / aa
    return scale > 0 and max(abs(y - scale * x) for x, y in zip(a, b)) < 1e-10


class EefTrajectory:
    """Exact checked polyline; quintic spans, collinear knots traversed at speed."""

    def __init__(self, start, segments, limits: MotionLimits, minimum_s=0.2):
        spans = []
        origin = list(start)
        for goal, requested_s in segments:
            delta = [g - s for g, s in zip(goal, origin)]
            # Only merge translation-only spans. Merging rotating spans can
            # erase an orientation waypoint or change the checked SLERP arc.
            if (
                spans
                and rotation_distance(spans[-1][0][3:], spans[-1][1][3:]) < 1e-12
                and rotation_distance(origin[3:], goal[3:]) < 1e-12
                and _same_direction(spans[-1][2][:3], delta[:3])
            ):
                prior = spans[-1]
                prior[1] = list(goal)
                prior[2] = [g - s for g, s in zip(goal, prior[0])]
                prior[3] += requested_s
            else:
                spans.append([origin, list(goal), delta, requested_s])
            origin = list(goal)
        durations = [
            max(
                s[3],
                limits.eef_duration_s(math.hypot(*s[2][:3]), rotation_distance(s[0][3:], s[1][3:])),
            )
            for s in spans
        ]
        total = sum(durations)
        scale = max(1.0, minimum_s / total)
        self.ends, self.spans = [], []
        cursor = 0.0
        for span, duration in zip(spans, durations):
            duration *= scale
            self.spans.append((span[0], span[1], cursor, duration))
            cursor += duration
            self.ends.append(cursor)
        if cursor > MAX_PATH_DURATION_S:
            raise ValueError(
                f"path exceeds {MAX_PATH_DURATION_S:g}s; split at a verified milestone"
            )
        self.duration = cursor
        self.goal = origin

    def at(self, elapsed):
        index = min(bisect.bisect_right(self.ends, elapsed), len(self.spans) - 1)
        start, goal, began, duration = self.spans[index]
        t = min(1.0, max(0.0, (elapsed - began) / duration))
        blend = min(1.0, max(0.0, t**3 * (10 + t * (-15 + 6 * t))))
        return interpolate_pose(start, goal, blend)
