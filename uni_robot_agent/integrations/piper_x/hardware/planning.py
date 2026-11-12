"""Deterministic all-arm IK and conservative capsule swept-volume preflight.

Capsules must be commissioned enclosing geometry, including the gripper at its
maximum opening. They are hash-pinned model assets, not inferred from URDF names.
"""
from __future__ import annotations

from itertools import combinations
import math
import xml.etree.ElementTree as ET
from typing import Callable, Literal

import numpy as np
from pydantic import Field
from scipy.spatial.transform import Rotation

from uni_robot_agent.services.environment.contracts import Contract, FiniteFloat
from uni_robot_agent.integrations.piper_x.body.core.contract import DeviceError, ErrorCode
from uni_robot_agent.integrations.piper_x.body.core.arm.tracking import TrackingLimits
from uni_robot_agent.integrations.piper_x.environment.kinematics import from_eef, bounded_target, distance
from ..config import FileRef


class Capsule(Contract):
    link: str
    start: tuple[FiniteFloat, FiniteFloat, FiniteFloat]
    end: tuple[FiniteFloat, FiniteFloat, FiniteFloat]
    radius_m: FiniteFloat = Field(gt=0)


class MeshAsset(Contract):
    link: str
    collision_index: int = Field(default=0, ge=0)
    file: FileRef


class CollisionModel(Contract):
    urdf_sha256: str
    capsules: list[Capsule] = Field(min_length=1)
    ignored_self_pairs: list[tuple[str, str]]
    table_exempt_links: list[str]
    enclosure_verified: bool
    scope: Literal["installed_robot", "urdf_only"] = "installed_robot"
    meshes: list[MeshAsset] = Field(default_factory=list)


def segment_distance(a, b, c, d):
    """Distance between finite segments, including parallel and zero-length ones."""
    u, v, w = b - a, d - c, a - c
    aa, bb, cc, dd, ee = u @ u, u @ v, v @ v, u @ w, v @ w
    denom = aa * cc - bb * bb
    candidates = []
    if aa > 1e-16:
        for t in (0., 1.):
            s = np.clip((bb * t - dd) / aa, 0, 1)
            candidates.append(np.linalg.norm(w + s * u - t * v))
    if cc > 1e-16:
        for s in (0., 1.):
            t = np.clip((bb * s + ee) / cc, 0, 1)
            candidates.append(np.linalg.norm(w + s * u - t * v))
    if denom > 1e-16:
        s, t = (bb * ee - cc * dd) / denom, (aa * ee - bb * dd) / denom
        if 0 <= s <= 1 and 0 <= t <= 1:
            candidates.append(np.linalg.norm(w + s * u - t * v))
    return float(min(candidates, default=np.linalg.norm(w)))


class RobotGeometry:
    def __init__(self, installation):
        self.installation = installation
        self.tracking_limits = TrackingLimits()
        self.model = installation.model
        self.collision = CollisionModel.model_validate_json(installation.collision.read_text())
        if self.collision.scope == "urdf_only" and not (
                installation.calibration.operator_supervised
                and installation.calibration.geometry_limitations_accepted.strip()):
            raise ValueError("URDF-only geometry requires explicit operator acceptance of omitted tooling")
        root = ET.fromstring(installation.urdf.read_bytes())
        self.joints = list(root.findall("joint"))
        self.links = {link.attrib["name"] for link in root.findall("link")}
        geometry_links = {link.attrib["name"] for link in root.findall("link") if link.findall("collision")}
        covered = {capsule.link for capsule in self.collision.capsules}
        if (not self.collision.enclosure_verified or self.collision.urdf_sha256 != self.model.urdf.sha256
                or not geometry_links <= covered or not covered <= self.links):
            raise ValueError("Collision model is unverified, incomplete or belongs to another URDF")
        if set(self.collision.table_exempt_links) - {self.model.root_link}:
            raise ValueError("Only the fixed base can be exempted from the table check")
        adjacent = {frozenset((j.find("parent").get("link"), j.find("child").get("link"))) for j in self.joints}
        self.ignored = {frozenset(pair) for pair in self.collision.ignored_self_pairs}
        if not self.ignored <= adjacent:
            raise ValueError("Only adjacent URDF links may be excluded from self collision")
        for joint in self.joints:
            if joint.get("type") not in {"fixed", "revolute", "continuous"} or joint.find("mimic") is not None:
                raise ValueError("Collision URDF requires independent rotational or fixed joints")
            if joint.get("name") in self.model.joint_names and joint.get("type") == "revolute":
                index = self.model.joint_names.index(joint.get("name"))
                limit = joint.find("limit")
                if (limit is None or self.model.joint_lower[index] < float(limit.get("lower"))
                        or self.model.joint_upper[index] > float(limit.get("upper"))):
                    raise ValueError("Model limits exceed the pinned URDF limits")
        # Bounds displacement of any enclosed point per radian for every joint.
        self.reach = sum(np.linalg.norm(self._origin(j)[:3, 3]) for j in self.joints) + max(
            max(np.linalg.norm(c.start), np.linalg.norm(c.end)) + c.radius_m for c in self.collision.capsules)
        if self.reach <= 0:
            raise ValueError("Degenerate collision geometry")
        self.mesh_checks = None
        if self.collision.meshes:
            from .mesh_collision import MeshChecks
            self.mesh_checks = MeshChecks(self, root, self.collision.meshes)

    @staticmethod
    def _origin(joint):
        element = joint.find("origin")
        values = {} if element is None else element.attrib
        return from_eef([float(x) for x in values.get("xyz", "0 0 0").split()],
                        [float(x) for x in values.get("rpy", "0 0 0").split()])

    def links_at(self, name, q):
        if len(q) != 6 or not np.isfinite(q).all():
            raise ValueError("Expected six finite joint angles")
        c = self.installation.calibration.arms[name]
        frames = {self.model.root_link: np.asarray(c.T_base_from_urdf)}
        values = dict(zip(self.model.joint_names, np.asarray(q) + c.kinematic_joint_offsets_rad))
        pending = list(self.joints)
        while pending:
            progress = False
            for joint in list(pending):
                parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
                if parent not in frames:
                    continue
                t = frames[parent] @ self._origin(joint)
                if joint.get("type") != "fixed":
                    if joint.get("name") not in values:
                        raise ValueError("URDF has an unmodelled movable joint")
                    axis = joint.find("axis")
                    direction = np.array([float(v) for v in ("1 0 0" if axis is None else axis.get("xyz")).split()])
                    if np.linalg.norm(direction) < 1e-12:
                        raise ValueError("Invalid URDF axis")
                    turn = np.eye(4)
                    turn[:3, :3] = Rotation.from_rotvec(direction / np.linalg.norm(direction) * values[joint.get("name")]).as_matrix()
                    t = t @ turn
                frames[child] = t
                pending.remove(joint)
                progress = True
            if not progress:
                raise ValueError("Disconnected or cyclic URDF")
        return frames

    def fk(self, name, q):
        return self.links_at(name, q)[self.model.tip_link] @ np.asarray(self.installation.calibration.arms[name].T_tip_from_flange)

    def ik(self, name, target, initial, *, checkpoint: Callable[[], None] | None = None):
        c = self.installation.calibration.arms[name]
        q = np.asarray(initial, dtype=float).copy()
        def error(q):
            t = self.fk(name, q)
            return np.r_[target[:3, 3] - t[:3, 3], Rotation.from_matrix(target[:3, :3] @ t[:3, :3].T).as_rotvec() * .15]
        for _ in range(120):
            if checkpoint is not None:
                checkpoint()
            e = error(q)
            if np.linalg.norm(e[:3]) <= .001 and np.linalg.norm(e[3:]) <= .003:
                return q
            j = np.column_stack([(error(q + np.eye(6)[i] * 1e-5) - e) / 1e-5 for i in range(6)])
            dq = -j.T @ np.linalg.solve(j @ j.T + np.eye(6) * 1e-5, e)
            q = np.clip(q + np.clip(dq, -.1, .1), c.joint_lower, c.joint_upper)
        raise DeviceError(ErrorCode.LIMIT_VIOLATION, f"{name}: IK failed before dispatch", stage="preflight")

    def check(self, positions, inflation=None):
        capsules = []
        cal = self.installation.calibration
        if self.mesh_checks is not None:
            for name, q in positions.items():
                c = cal.arms[name]
                q = np.asarray(q)
                if q.shape != (6,) or not np.isfinite(q).all() or np.any(q < c.joint_lower) or np.any(q > c.joint_upper):
                    raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Joint range exceeded", stage="preflight")
            return self.mesh_checks.check(positions, inflation)
        for name, q in positions.items():
            c = cal.arms[name]
            if np.any(np.asarray(q) < c.joint_lower) or np.any(np.asarray(q) > c.joint_upper):
                raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Joint range exceeded", stage="preflight")
            frames = self.links_at(name, q)
            for shape in self.collision.capsules:
                t = np.asarray(c.T_world_from_base) @ frames[shape.link]
                a, b = [(t @ np.r_[p, 1])[:3] for p in (shape.start, shape.end)]
                # Include the commissioned tracking tube around the planned state.
                # This is conservative even when joints do not advance synchronously.
                radius = shape.radius_m + (inflation or {}).get(name, 0.)
                if shape.link not in self.collision.table_exempt_links and min(a[2], b[2]) - radius <= cal.table_height_m + cal.collision_margin_m:
                    raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Path intersects table clearance", stage="preflight")
                capsules.append((name, shape.link, a, b, radius))
        for a, b in combinations(capsules, 2):
            if a[0] == b[0] and (a[1] == b[1] or frozenset((a[1], b[1])) in self.ignored):
                continue
            if segment_distance(a[2], a[3], b[2], b[3]) <= a[4] + b[4] + cal.collision_margin_m:
                raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Self or inter-arm collision clearance failed", stage="preflight")

    def _within_command_step(self, start, end):
        for name in start:
            limits = self.installation.calibration.arms[name].motion_limits
            if np.max(np.abs(end[name] - start[name])) > limits.joint_step_rad:
                return False
            dp, angle = distance(self.fk(name, start[name]), self.fk(name, end[name]))
            if dp > limits.eef_step_m or angle > limits.eef_rotation_step_rad:
                return False
        return True

    def check_measured_segment(self, start, end, *, checkpoint=None):
        """Validate the actual start to the next target, not an assumed arrival.

        This is the same swept-volume proof used in preflight. A stationary
        servo residual does not authorize skipping the unchecked connection.
        """
        self.check(start)
        if not self._within_command_step(start, end):
            return False
        steps = max(1, *(math.ceil(self.reach * np.sum(np.abs(end[n] - start[n])) /
                        (self.installation.calibration.collision_margin_m / 4)) for n in start))
        if steps > 2000:
            raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Measured segment subdivision budget exceeded")
        for i in range(1, steps + 1):
            if checkpoint is not None:
                checkpoint()
            row = {n: start[n] + (end[n] - start[n]) * i / steps for n in start}
            inflation = {n: (np.abs(end[n] - start[n]) / steps +
                        self.installation.calibration.arms[n].tracking_tolerance_rad
                        if self.mesh_checks is not None else
                        self.reach * (np.sum(np.abs(end[n] - start[n])) / steps +
                        6 * self.installation.calibration.arms[n].tracking_tolerance_rad)) for n in start}
            self.check(row, inflation)
        return True

    def plan(self, measured, targets, *, segment_m, max_segments,
             checkpoint: Callable[[], None] | None = None):
        """Check the complete joint path, including non-participating measured holds."""
        if measured.keys() != targets.keys():
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "A path must name every arm")
        current = {n: np.array(q, dtype=float) for n, q in measured.items()}
        starts = {n: self.fk(n, q) for n, q in current.items()}
        goals, segments = {}, 1
        for name, target in targets.items():
            if "hold_joints" in target:
                tolerance = self.tracking_limits.hold_admission_joint_rad
                if not np.allclose(target["hold_joints"], current[name], atol=tolerance, rtol=0):
                    raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Held joints differ from measured state")
                goals[name] = starts[name]
            else:
                goals[name] = from_eef(target["target_position_m"], target["target_rpy_rad"])
                dp, angle = distance(starts[name], goals[name])
                radius = self.installation.calibration.arms[name].rotation_radius_m
                segments = max(segments, math.ceil((dp + radius * angle) / segment_m))
        if segments > max_segments:
            raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Path exceeds the requested travel budget")
        if checkpoint is not None:
            checkpoint()
        self.check(current)
        path = [current]
        for index in range(1, segments + 1):
            if checkpoint is not None:
                checkpoint()
            next_q = {}
            for name in current:
                if "hold_joints" in targets[name]:
                    next_q[name] = current[name].copy()
                else:
                    dp, angle = distance(starts[name], goals[name])
                    waypoint = bounded_target(starts[name], goals[name], translation=max(dp, 1e-12) * index / segments,
                                              rotation=max(angle, 1e-12) * index / segments)
                    next_q[name] = self.ik(name, waypoint, current[name], checkpoint=checkpoint)
            # Cover each joint interpolation interval with inflated endpoint capsules.
            # The reach bound covers all joints, making this conservative between samples.
            steps = max(1, *(math.ceil(self.reach * np.sum(np.abs(next_q[n] - current[n])) /
                            (self.installation.calibration.collision_margin_m / 4)) for n in current))
            if steps > 2000:
                raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Path subdivision budget exceeded")
            command_start, previous = current, current
            for i in range(1, steps + 1):
                if checkpoint is not None:
                    checkpoint()
                row = {n: current[n] + (next_q[n] - current[n]) * i / steps for n in current}
                if self.mesh_checks is not None:
                    inflation = {n: np.abs(next_q[n] - current[n]) / steps +
                        self.installation.calibration.arms[n].tracking_tolerance_rad for n in current}
                else:
                    inflation = {n: self.reach * (np.sum(np.abs(next_q[n] - current[n])) / steps +
                        6 * self.installation.calibration.arms[n].tracking_tolerance_rad) for n in current}
                self.check(row, inflation)
                # These are collision proof samples on one linear joint interval.
                # Merge only collinear samples; never change the checked joint path
                # by taking a shortcut across two IK waypoints.
                if not self._within_command_step(command_start, row):
                    if previous is command_start or not self._within_command_step(previous, row):
                        raise DeviceError(ErrorCode.LIMIT_VIOLATION, "Collision sample exceeds command step limits")
                    path.append(previous)
                    command_start = previous
                previous = row
            path.append(next_q)
            current = next_q
        return path
