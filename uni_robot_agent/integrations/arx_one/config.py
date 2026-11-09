"""Commissioned hardware configuration; no implicit frame or controller defaults."""
from pathlib import Path
from typing import Literal
from urllib.parse import urlsplit

import numpy as np
from pydantic import Field, field_validator, model_validator

from uni_robot_agent.services.environment.contracts import Contract, FiniteFloat

Matrix = list[list[FiniteFloat]]


def rigid(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-6)):
        raise ValueError("Expected a rigid homogeneous transform")
    return value


class BodyConfig(Contract):
    body_id: str = Field(min_length=1)
    url: str
    token_env: str = Field(min_length=1)

    @field_validator("url")
    @classmethod
    def endpoint(cls, value):
        url = urlsplit(value)
        if (url.scheme not in {"http", "https"} or not url.hostname or url.username
                or url.password or url.query or url.fragment):
            raise ValueError("Body URL must be HTTP(S), without credentials or query")
        return value


class KinematicsConfig(Contract):
    urdf: Path
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    root_link: str
    tip_link: str
    T_controller_from_urdf: Matrix
    T_tip_from_control: Matrix
    _transforms = field_validator("T_controller_from_urdf", "T_tip_from_control")(rigid)


class ArmConfig(Contract):
    device_id: str
    # Copied from the Body (``operator_cli identity``): ``{"identity": {...}}``, pinned exactly.
    expected_metadata: dict
    T_world_from_controller: Matrix
    grasp_site: str = Field(min_length=1)
    grasp_site_description: str = Field(min_length=1)
    # Full commissioned transform from public grasp site to native Body EEF.
    T_native_from_grasp: Matrix
    rotation_radius_m: FiniteFloat = Field(ge=0)
    # One entry per arm joint, in the order the Body reports joint_position_rad.
    joint_names: list[str] = Field(min_length=1, max_length=16)
    joint_lower: list[FiniteFloat] = Field(min_length=1, max_length=16)
    joint_upper: list[FiniteFloat] = Field(min_length=1, max_length=16)
    tracking_tolerance_rad: FiniteFloat = Field(gt=0, le=0.2)
    kinematics: KinematicsConfig | None = None
    _transform = field_validator("T_world_from_controller", "T_native_from_grasp")(rigid)

    @model_validator(mode="after")
    def commissioned(self):
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("Joint names must be distinct")
        if not len(self.joint_lower) == len(self.joint_upper) == len(self.joint_names):
            raise ValueError("joint_lower and joint_upper need one bound per joint name")
        if self.rotation_radius_m < np.linalg.norm(np.asarray(self.T_native_from_grasp)[:3, 3]):
            raise ValueError("Rotation radius must cover the native EEF displacement about the grasp site")
        if any(lo >= hi for lo, hi in zip(self.joint_lower, self.joint_upper)):
            raise ValueError("Invalid joint limits")
        pinned_identity(self.expected_metadata)
        return self


def pinned_identity(expected_metadata):
    if not isinstance(expected_metadata.get("identity"), dict) or not expected_metadata["identity"]:
        raise ValueError("Pin expected_metadata.identity from the Body: run operator_cli identity")
    return expected_metadata


class CameraConfig(Contract):
    device_id: str
    # Copied from the Body (``operator_cli identity``): ``{"identity": {...}}``, pinned exactly.
    expected_metadata: dict
    # "world" for a fixed scene camera, or the name of the arm whose native Body EEF carries the
    # camera (not the public grasp site).
    parent: str = Field(default="world", min_length=1)
    T_parent_from_optical: Matrix | None = None
    # Time between frames of the RGB-D stream; a capture accepts the first frame within 1.5 periods.
    frame_period_s: FiniteFloat = Field(default=1 / 30, gt=0, le=1)

    @field_validator("T_parent_from_optical")
    @classmethod
    def transform(cls, value):
        return None if value is None else rigid(value)

    _identity = field_validator("expected_metadata")(pinned_identity)


class MotionProfile(Contract):
    """Arrival criteria for one tool profile, matched to this arm's accuracy."""
    # At most the continuous path segment (environment/path_command.py MAX_SEGMENT_M).
    max_step_m: FiniteFloat = Field(gt=0, le=.06)
    position_tolerance_m: FiniteFloat = Field(gt=0, le=.1)
    orientation_tolerance_rad: FiniteFloat = Field(gt=0, le=.5)


class Config(Contract):
    robot_id: str = Field(min_length=1)
    arms_body: BodyConfig
    cameras_body: BodyConfig
    # Arm and camera names are free; the arm order is the order of the native path command.
    arms: dict[str, ArmConfig] = Field(min_length=1)
    cameras: dict[str, CameraConfig]
    execute: bool = False
    # Replaces the shared tool defaults for the named profiles; absent names keep them.
    motion_profiles: dict[Literal["standard", "approach", "precision"], MotionProfile] = Field(default_factory=dict)
    control_frequency_hz: FiniteFloat = Field(default=10., gt=0, le=30)
    max_episode_steps: int = Field(default=2000, gt=0)
    rpc_timeout_s: FiniteFloat = Field(default=2., gt=0, le=10)
    action_timeout_s: FiniteFloat = Field(default=15., gt=0, le=60)
    idle_timeout_s: FiniteFloat = Field(default=120., gt=0, le=600)
    stop_timeout_s: FiniteFloat = Field(default=5., gt=0, le=30)
    max_age_s: FiniteFloat = Field(default=.5, gt=0, le=.5)

    @model_validator(mode="after")
    def complete(self):
        if len({x.device_id for x in self.arms.values()}) != len(self.arms):
            raise ValueError("Arm device identities must differ")
        if len({x.device_id for x in self.cameras.values()}) != len(self.cameras):
            raise ValueError("Camera device identities must differ")
        for name, camera in self.cameras.items():
            if camera.parent != "world" and camera.parent not in self.arms:
                raise ValueError(f"camera {name}: parent must be world or a configured arm")
            if camera.parent != "world" and camera.T_parent_from_optical is not None:
                if self.arms[camera.parent].kinematics is None:
                    raise ValueError("Calibrated wrist geometry requires verified FK")
        return self

    @classmethod
    def load(cls, path):
        path = Path(path).resolve()
        result = cls.model_validate_json(path.read_text())
        for arm in result.arms.values():
            if arm.kinematics and not arm.kinematics.urdf.is_absolute():
                arm.kinematics.urdf = path.parent / arm.kinematics.urdf
        return result
