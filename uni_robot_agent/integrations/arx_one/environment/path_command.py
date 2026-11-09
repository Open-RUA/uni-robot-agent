"""One whole-path move for every arm, carried by the protocol's native action vector.

The layout lists the arms in order with their joint counts; the adapter derives it from the
configuration and publishes it as ``path_layout``. Only this integration's tools and adapter read
this command; other environments reject native commands.
"""
from dataclasses import dataclass

from uni_robot_agent.services.environment.contracts import (
    ArmCommand, JointTarget, NativeCommand, Pose, PoseCommand, RequestRejected,
)

VERSION = 1.
_POSE, _HOLD = 1., 0.
#: Longest straight segment of one continuous path; motion profiles may not exceed it.
MAX_SEGMENT_M = .06


def _fields(joints):
    """Per arm: mode, a pose (xyz + wxyz) or the joint values padded to the same width, gripper."""
    return 2 + max(7, joints)


@dataclass(frozen=True)
class ContinuousRequest:
    command: PoseCommand
    max_segments: int
    segment_m: float


def encode(arms, *, layout, max_segments, segment_m) -> NativeCommand:
    names = [name for name, _ in layout]
    if set(arms) != set(names):
        raise RequestRejected(f"A continuous path must command every arm: {names}")
    values = [VERSION, float(max_segments), float(segment_m)]
    for name, joints in layout:
        target, width = arms[name], max(7, joints)
        if isinstance(target, JointTarget):
            if len(target.joints) != joints:
                raise RequestRejected(f"A held {name} arm needs {joints} joints")
            values += [_HOLD, *map(float, target.joints), *[0.] * (width - joints)]
        else:
            values += [_POSE, *map(float, target.pose.position), *map(float, target.pose.quaternion),
                       *[0.] * (width - 7)]
        values.append(float(target.gripper))
    return NativeCommand(values=values)


def decode(values, *, layout) -> ContinuousRequest:
    if len(values) != 3 + sum(_fields(joints) for _, joints in layout) or values[0] != VERSION:
        raise RequestRejected("Unsupported continuous path command")
    max_segments, segment_m = values[1], values[2]
    if max_segments != int(max_segments) or not 1 <= max_segments <= 24 or not 0 < segment_m <= MAX_SEGMENT_M:
        raise RequestRejected("Continuous path budget is out of range")
    arms, start = {}, 3
    for name, joints in layout:
        size = _fields(joints)
        mode, fields, gripper = values[start], values[start + 1:start + size - 1], values[start + size - 1]
        if mode == _POSE:
            arms[name] = ArmCommand(pose=Pose(position=fields[:3], quaternion=fields[3:7]), gripper=gripper)
        elif mode == _HOLD:
            arms[name] = JointTarget(joints=list(fields[:joints]), gripper=gripper)
        else:
            raise RequestRejected("Unknown continuous arm mode")
        start += size
    return ContinuousRequest(PoseCommand(arms=arms), int(max_segments), float(segment_m))
