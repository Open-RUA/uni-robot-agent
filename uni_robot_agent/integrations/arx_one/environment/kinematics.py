"""Pose conventions and read-only FK from an explicitly pinned URDF with an explicit controller origin."""

import hashlib
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from uni_robot_agent.services.environment.contracts import Pose


def matrix(position, quaternion):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(np.asarray(quaternion)[[1, 2, 3, 0]]).as_matrix()
    t[:3, 3] = position
    return t


def pose(t):
    q = Rotation.from_matrix(t[:3, :3]).as_quat()
    return Pose(position=t[:3, 3].tolist(), quaternion=q[[3, 0, 1, 2]].tolist())


def from_eef(position, rpy):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    t[:3, 3] = position
    return t


def distance(a, b):
    return (float(np.linalg.norm(a[:3, 3] - b[:3, 3])),
            float(Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude()))


def bounded_target(start, target, *, translation=.04, rotation=.3, rotation_radius_m=0.):
    dp, angle = distance(start, target)
    fraction = min(1., translation / max(dp + rotation_radius_m * angle, 1e-12), rotation / max(angle, 1e-12))
    result = start.copy()
    result[:3, 3] += fraction * (target[:3, 3] - start[:3, 3])
    dr = Rotation.from_matrix(start[:3, :3].T @ target[:3, :3]).as_rotvec()
    result[:3, :3] = start[:3, :3] @ Rotation.from_rotvec(fraction * dr).as_matrix()
    return result


class Kinematics:
    def __init__(self, arm):
        self.config = arm.kinematics
        self.T_native_from_grasp = np.asarray(arm.T_native_from_grasp)
        data = self.config.urdf.read_bytes()
        if hashlib.sha256(data).hexdigest() != self.config.sha256:
            raise ValueError("URDF hash mismatch")
        root = ET.fromstring(data)
        by_child = {j.find("child").attrib["link"]: j for j in root.findall("joint")}
        chain, link, visited = [], self.config.tip_link, set()
        while link != self.config.root_link:
            if link in visited or link not in by_child:
                raise ValueError("URDF has no unique root-to-tip chain")
            visited.add(link)
            joint = by_child[link]
            chain.append(joint)
            link = joint.find("parent").attrib["link"]
        self.chain = list(reversed(chain))
        names = [j.attrib["name"] for j in self.chain if j.attrib["type"] != "fixed"]
        if names != arm.joint_names:
            raise ValueError("URDF joint order differs from controller order")
        self.dof = len(names)
        for joint in self.chain:
            if joint.attrib["type"] not in {"fixed", "revolute", "continuous"} or joint.find("mimic") is not None:
                raise ValueError("Only fixed and independent rotational joints are supported")

    def joint_description(self, arm):
        """Controller order is verified against the pinned URDF, never inferred."""
        joints = []
        for joint in self.chain:
            if joint.attrib["type"] == "fixed":
                continue
            index = len(joints)
            axis = joint.find("axis")
            joints.append({
                "joint_number": index + 1, "state_index": index,
                "urdf_joint": joint.attrib["name"],
                "parent_link": joint.find("parent").attrib["link"],
                "child_link": joint.find("child").attrib["link"],
                "axis_xyz": [float(v) for v in
                             ("1 0 0" if axis is None else axis.attrib["xyz"]).split()],
                "axis_frame": "URDF joint-local frame after joint origin transform",
                "command_limits_rad": [arm.joint_lower[index], arm.joint_upper[index]],
            })
        return {
            "source": "sha256-pinned URDF; controller order and command limits verified against Body",
            "urdf_sha256": self.config.sha256,
            "joints": joints,
            "interpretation": (
                "Joint numbers are one-based; state joints[] indices are zero-based. "
                "Axes are joint-local, not world axes or end-effector roll/pitch/yaw controls. "
                "Do not infer an EEF rotation restriction from a joint number. A rejected "
                "IK candidate/path does not prove all paths or directions are unreachable."
            ),
        }

    def grasp(self, joints):
        return self.controller(joints) @ self.T_native_from_grasp

    def controller(self, joints):
        """Native Body EEF; camera calibration and FK checks depend on this frame."""
        if len(joints) != self.dof or not np.isfinite(joints).all():
            raise ValueError(f"FK requires {self.dof} finite joints")
        t, index = np.asarray(self.config.T_controller_from_urdf).copy(), 0
        for joint in self.chain:
            origin = joint.find("origin")
            attr = {} if origin is None else origin.attrib
            xyz = [float(x) for x in attr.get("xyz", "0 0 0").split()]
            rpy = [float(x) for x in attr.get("rpy", "0 0 0").split()]
            t = t @ from_eef(xyz, rpy)
            if joint.attrib["type"] != "fixed":
                axis = joint.find("axis")
                axis = np.asarray([float(x) for x in ("1 0 0" if axis is None else axis.attrib["xyz"]).split()])
                if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-12:
                    raise ValueError("Invalid URDF axis")
                turn = np.eye(4)
                turn[:3, :3] = Rotation.from_rotvec(axis / np.linalg.norm(axis) * joints[index]).as_matrix()
                t = t @ turn
                index += 1
        return t @ np.asarray(self.config.T_tip_from_control)
