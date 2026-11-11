"""Offline deployment contracts. Hardware discovery never runs during validation.

One manifest selects exactly one model, host, robot and calibration. Paths are
relative to the file declaring them; there is no implicit configuration overlay.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Literal

from pydantic import Field, field_validator, model_validator

from uni_robot_agent.services.environment.contracts import Contract, FiniteFloat
from uni_robot_agent.integrations.piper_x.body.core.arm.tracking import TrackingLimits
from uni_robot_agent.integrations.piper_x.environment.config import Matrix, rigid, MotionProfile


def digest(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


class FileRef(Contract):
    path: Path
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")

    def resolve(self, base: Path) -> Path:
        path = (base / self.path).resolve()
        if hashlib.sha256(path.read_bytes()).hexdigest() != self.sha256:
            raise ValueError(f"File hash mismatch: {path.name}")
        return path


class MotionLimits(Contract):
    eef_step_m: FiniteFloat = Field(gt=0)
    eef_speed_m_s: FiniteFloat = Field(gt=0)
    eef_accel_m_s2: FiniteFloat = Field(gt=0)
    eef_rotation_step_rad: FiniteFloat = Field(gt=0)
    eef_rotation_speed_rad_s: FiniteFloat = Field(gt=0)
    joint_step_rad: FiniteFloat = Field(gt=0)
    joint_speed_rad_s: FiniteFloat = Field(gt=0)
    gripper_speed_fraction_s: FiniteFloat = Field(gt=0)


class Model(Contract):
    model_id: str = Field(min_length=1)
    protocol: Literal["piper-x-can-v1"]
    sdk_version: str = Field(min_length=1)
    sdk_driver_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    firmware_profile: Literal["default", "V183", "V188", "V189"]
    firmware_version: str = Field(min_length=1)
    urdf: FileRef
    collision: FileRef
    root_link: str = Field(min_length=1)
    tip_link: str = Field(min_length=1)
    joint_names: list[str] = Field(min_length=6, max_length=6,
        description="URDF joint names in CAN motor 1 through 6 order")
    joint_lower: list[FiniteFloat] = Field(min_length=6, max_length=6)
    joint_upper: list[FiniteFloat] = Field(min_length=6, max_length=6)
    motion_limits: MotionLimits
    gripper_max_opening_m: FiniteFloat = Field(gt=0)
    gripper_max_effort_n: FiniteFloat = Field(gt=0)

    @model_validator(mode="after")
    def bounds(self):
        if len(set(self.joint_names)) != 6 or any(a >= b for a, b in zip(self.joint_lower, self.joint_upper)):
            raise ValueError("Invalid model joint order or limits")
        return self


class CanBinding(Contract):
    interface: str = Field(pattern=r"^[A-Za-z0-9_-]{1,15}$")
    usb_vendor: str = Field(pattern=r"^[0-9a-f]{4}$")
    usb_product: str = Field(pattern=r"^[0-9a-f]{4}$")
    usb_serial: str | None = None
    usb_topology: str | None = None
    channel: int = Field(ge=0, strict=True)

    @model_validator(mode="after")
    def identity(self):
        if bool(self.usb_serial) == bool(self.usb_topology):
            raise ValueError("Bind CAN by one unique USB serial or an explicit USB topology")
        return self

    def resource(self) -> str:
        identity = f"serial:{self.usb_serial}" if self.usb_serial else f"topology:{self.usb_topology}"
        return f"can-usb:{self.usb_vendor}:{self.usb_product}:{identity}:channel:{self.channel}"


class VideoStream(Contract):
    width: int = Field(gt=0, le=4096)
    height: int = Field(gt=0, le=2160)
    fps: int = Field(gt=0, le=90)
    format: Literal["rgb8", "bgr8", "z16"]


class CameraBinding(Contract):
    serial: str = Field(min_length=1, description="RealSense camera_info.serial_number, not the USB descriptor serial")
    model: str = Field(min_length=1)
    color: VideoStream
    depth: VideoStream
    max_pair_skew_s: FiniteFloat = Field(gt=0, le=.1)
    timeout_ms: int = Field(gt=0, le=5000)
    max_frame_age_s: FiniteFloat = Field(gt=0, le=2)

    @model_validator(mode="after")
    def formats(self):
        if self.color.format not in {"rgb8", "bgr8"} or self.depth.format != "z16":
            raise ValueError("RGB-D needs rgb8/bgr8 color and z16 depth")
        if self.color.fps != self.depth.fps:
            raise ValueError("RGB-D streams must have the same frame rate")
        return self


class BodyInstance(Contract):
    body_id: str = Field(pattern=r"^[A-Za-z0-9_-]+$")
    robot_id: str = Field(pattern=r"^[A-Za-z0-9_-]+$")
    kind: Literal["arms", "cameras"]
    port: int = Field(ge=1024, le=65535, strict=True)
    token_env: str = Field(pattern=r"^[A-Z][A-Z0-9_]+$")


class Host(Contract):
    host_id: str = Field(pattern=r"^[A-Za-z0-9_-]+$")
    operator: str = Field(min_length=1)
    listen: str = Field(min_length=1)
    connect_host: str = Field(min_length=1)
    state_dir: Path
    lock_dir: Path
    tokens_file: Path
    bodies: dict[str, BodyInstance] = Field(min_length=1)
    arms: dict[str, CanBinding] = Field(min_length=1)
    cameras: dict[str, CameraBinding]

    @model_validator(mode="after")
    def distinct(self):
        for values in ([b.port for b in self.bodies.values()], [b.body_id for b in self.bodies.values()],
                       [a.interface for a in self.arms.values()], [a.resource() for a in self.arms.values()],
                       [c.serial for c in self.cameras.values()]):
            if len(set(values)) != len(values):
                raise ValueError("Duplicate Body port, identity or physical resource")
        return self


class Arm(Contract):
    device_id: str = Field(min_length=1)
    binding: str = Field(min_length=1)
    role: str = Field(min_length=1)
    grasp_site: str = Field(min_length=1)
    grasp_site_description: str = Field(min_length=1)


class Camera(Contract):
    device_id: str = Field(min_length=1)
    binding: str = Field(min_length=1)
    role: Literal["scene", "wrist"]
    parent_arm: str | None = None

    @model_validator(mode="after")
    def parent(self):
        if (self.role == "wrist") != (self.parent_arm is not None):
            raise ValueError("Wrist cameras require parent_arm; scene cameras must be fixed")
        return self


class Robot(Contract):
    robot_id: str = Field(pattern=r"^[A-Za-z0-9_-]+$")
    arms_body: str
    cameras_body: str
    arms: dict[str, Arm] = Field(min_length=1)
    cameras: dict[str, Camera]
    required_arm_roles: list[str] = Field(min_length=1)
    required_cameras: list[str]
    calibration: FileRef
    motion_profiles: dict[Literal["standard", "approach", "precision"], MotionProfile]
    action_timeout_s: FiniteFloat = Field(gt=0, le=60)
    stop_timeout_s: FiniteFloat = Field(gt=0, le=30)
    feedback_max_age_s: FiniteFloat = Field(gt=0, le=.1)
    feedback_max_skew_s: FiniteFloat = Field(gt=0, le=.05)
    control_frequency_hz: FiniteFloat = Field(gt=0, le=30)
    max_episode_steps: int = Field(gt=0)

    @model_validator(mode="after")
    def roles(self):
        roles = [a.role for a in self.arms.values()]
        if len(set(roles)) != len(roles) or not set(self.required_arm_roles) <= set(roles):
            raise ValueError("Missing or duplicate arm role")
        for items in (self.arms.values(), self.cameras.values()):
            items = list(items)
            for values in ([a.device_id for a in items], [a.binding for a in items]):
                if len(set(values)) != len(values):
                    raise ValueError("Duplicate device or physical binding")
        if not set(self.required_cameras) <= self.cameras.keys():
            raise ValueError("Missing required camera")
        for camera in self.cameras.values():
            if camera.parent_arm is not None and camera.parent_arm not in self.arms:
                raise ValueError("Unknown wrist parent_arm")
        if set(self.motion_profiles) != {"standard", "approach", "precision"}:
            raise ValueError("Commission all motion profiles; no robot-default fallback")
        return self


class ArmCalibration(Contract):
    T_world_from_base: Matrix
    T_base_from_urdf: Matrix
    T_tip_from_flange: Matrix
    T_flange_from_grasp: Matrix
    joint_lower: list[FiniteFloat] = Field(min_length=6, max_length=6)
    joint_upper: list[FiniteFloat] = Field(min_length=6, max_length=6)
    motion_limits: MotionLimits
    rotation_radius_m: FiniteFloat = Field(gt=0)
    tracking_tolerance_rad: FiniteFloat = Field(gt=0, le=.2)
    gripper_closed_m: FiniteFloat = Field(ge=0)
    gripper_open_m: FiniteFloat = Field(gt=0)
    gripper_effort: FiniteFloat = Field(gt=0)
    gripper_tolerance_m: FiniteFloat = Field(gt=0)
    _transforms = field_validator("T_world_from_base", "T_base_from_urdf", "T_tip_from_flange", "T_flange_from_grasp")(rigid)

    @model_validator(mode="after")
    def bounds(self):
        if self.gripper_open_m <= self.gripper_closed_m or any(a >= b for a, b in zip(self.joint_lower, self.joint_upper)):
            raise ValueError("Invalid commissioned joint/gripper range")
        return self


class CameraCalibration(Contract):
    parent_frame: str
    optical_frame: str
    T_parent_from_optical: Matrix
    capture_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    _transform = field_validator("T_parent_from_optical")(rigid)


class Calibration(Contract):
    robot_id: str
    host_id: str
    model_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    bindings_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    collected_at: str = Field(min_length=1)
    operator: str = Field(min_length=1)
    world_frame: str = Field(min_length=1)
    arms: dict[str, ArmCalibration]
    cameras: dict[str, CameraCalibration]
    table_height_m: FiniteFloat
    collision_margin_m: FiniteFloat = Field(gt=0)
    self_collision_margin_m: FiniteFloat | None = Field(default=None, gt=0)
    # An external commissioning record; a successful software test cannot set these.
    accepted_checks: set[Literal["identity", "fk_tcp", "cameras", "grippers", "independent_stop", "dual_motion"]]
    operator_supervised: bool = False
    skipped_checks: dict[Literal["fk_tcp", "grippers", "independent_stop", "dual_motion"], str] = Field(default_factory=dict)
    geometry_limitations_accepted: str = ""

    @model_validator(mode="after")
    def execution_evidence(self):
        if self.accepted_checks & self.skipped_checks.keys():
            raise ValueError("A check cannot be both accepted and skipped")
        if any(not reason.strip() for reason in self.skipped_checks.values()):
            raise ValueError("Skipped checks require an operator-attributed reason")
        if (self.skipped_checks or self.geometry_limitations_accepted) and not self.operator_supervised:
            raise ValueError("Explicit limitations require operator-supervised execution")
        return self


class Deployment(Contract):
    schema_version: Literal[1] = 1
    model: Path
    host: Path
    robot: Path


class Installation:
    """Validated, hash-pinned installation used by hosts and the environment."""

    def __init__(self, manifest: Path):
        self.path = Path(manifest).resolve()
        spec = Deployment.model_validate_json(self.path.read_text())
        self.model_path = (self.path.parent / spec.model).resolve()
        self.host_path = (self.path.parent / spec.host).resolve()
        self.robot_path = (self.path.parent / spec.robot).resolve()
        self.model = Model.model_validate_json(self.model_path.read_text())
        self.host = Host.model_validate_json(self.host_path.read_text())
        # Keep the declared host namespace for cross-OS planner identity checks.
        # Runtime path resolution below belongs to the hardware host, not its mirror.
        self._host_record = self.host.model_dump(mode="json")
        self.robot = Robot.model_validate_json(self.robot_path.read_text())
        self.model_hash = digest(self.model.model_dump(mode="json"))
        for key in ("state_dir", "lock_dir", "tokens_file"):
            setattr(self.host, key, (self.host_path.parent / getattr(self.host, key)).resolve())
        self.urdf = self.model.urdf.resolve(self.model_path.parent)
        self.collision = self.model.collision.resolve(self.model_path.parent)
        for ref, kind in ((self.robot.arms_body, "arms"), (self.robot.cameras_body, "cameras")):
            body = self.host.bodies.get(ref)
            if body is None or body.robot_id != self.robot.robot_id or body.kind != kind:
                raise ValueError("Wrong robot or Body instance binding")
        if not {a.binding for a in self.robot.arms.values()} <= self.host.arms.keys():
            raise ValueError("Unknown physical arm binding")
        if not {c.binding for c in self.robot.cameras.values()} <= self.host.cameras.keys():
            raise ValueError("Unknown physical camera binding")
        self.bindings = {
            "arms": {n: self.host.arms[a.binding].model_dump(mode="json") for n, a in self.robot.arms.items()},
            "cameras": {n: self.host.cameras[c.binding].model_dump(mode="json") for n, c in self.robot.cameras.items()},
        }
        self.calibration_path = self.robot.calibration.resolve(self.robot_path.parent)
        self.calibration = Calibration.model_validate_json(self.calibration_path.read_text())
        self._verify_calibration()

    def _verify_calibration(self):
        c, r, m = self.calibration, self.robot, self.model
        if (c.robot_id != r.robot_id or c.host_id != self.host.host_id or c.model_sha256 != self.model_hash
                or c.bindings_sha256 != digest(self.bindings)):
            raise ValueError("Calibration belongs to a different robot, model, host or capture configuration")
        if c.arms.keys() != r.arms.keys() or c.cameras.keys() != r.cameras.keys():
            raise ValueError("Calibration must cover every configured arm and camera")
        for name, a in c.arms.items():
            if a.gripper_open_m > m.gripper_max_opening_m or a.gripper_effort > m.gripper_max_effort_n:
                raise ValueError(f"{name}: gripper calibration exceeds model limits")
            if a.gripper_tolerance_m >= (a.gripper_open_m - a.gripper_closed_m) / 2:
                raise ValueError(f"{name}: invalid gripper tolerance")
            if any(lo < ml or hi > mh for lo, hi, ml, mh in zip(a.joint_lower, a.joint_upper, m.joint_lower, m.joint_upper)):
                raise ValueError(f"{name}: site joint limits exceed model limits")
            if any(value > m.motion_limits.model_dump()[key] for key, value in a.motion_limits.model_dump().items()):
                raise ValueError(f"{name}: site motion limits exceed model limits")
        for name, cam in r.cameras.items():
            expected_parent = c.world_frame if cam.parent_arm is None else r.arms[cam.parent_arm].device_id + "/flange"
            if c.cameras[name].parent_frame != expected_parent or c.cameras[name].optical_frame != cam.device_id + "/color_optical":
                raise ValueError(f"{name}: calibration coordinate-frame mismatch")
            if c.cameras[name].capture_sha256 != digest(self.bindings["cameras"][name]):
                raise ValueError(f"{name}: calibration capture settings mismatch")

    def require_motion(self):
        required = {"identity", "fk_tcp", "cameras", "grippers", "independent_stop", "dual_motion"}
        recorded = self.calibration.accepted_checks | self.calibration.skipped_checks.keys()
        if recorded != required:
            raise ValueError("Missing station acceptance checks: " + ", ".join(sorted(required - recorded)))

    def identity(self, name: str, kind: str) -> dict:
        return {"robot_id": self.robot.robot_id, "host_id": self.host.host_id,
                "model_sha256": self.model_hash, "calibration_sha256": self.robot.calibration.sha256,
                "binding": self.bindings[kind][name]}

    def audit(self) -> dict:
        # Hashes bind the declared deployment. Public records omit serials,
        # endpoints, credential references and absolute deployment paths.
        return {"robot_id": self.robot.robot_id, "host_id": self.host.host_id,
                "model_sha256": self.model_hash, "calibration_sha256": self.robot.calibration.sha256,
                "configuration_sha256": digest({"host": self._host_record,
                                               "robot": self.robot.model_dump(mode="json")}),
                "sdk_version": self.model.sdk_version, "protocol": self.model.protocol,
                "urdf_sha256": self.model.urdf.sha256, "collision_sha256": self.model.collision.sha256,
                "firmware_profile": self.model.firmware_profile,
                "motion_limits": {n: a.motion_limits.model_dump() for n, a in self.calibration.arms.items()},
                "arm_roles": {n: a.role for n, a in self.robot.arms.items()},
                "cameras": {n: {"role": c.role, "parent_arm": c.parent_arm} for n, c in self.robot.cameras.items()},
                "accepted_checks": sorted(self.calibration.accepted_checks),
                "skipped_checks": self.calibration.skipped_checks,
                "operator_supervised": self.calibration.operator_supervised,
                "geometry_limitations_accepted": self.calibration.geometry_limitations_accepted}

    def redacted_configuration(self) -> dict:
        """Preserve resolved commissioning settings without deployment identifiers."""
        model = self.model.model_dump(mode="json")
        host = self.host.model_dump(mode="json")
        robot = self.robot.model_dump(mode="json")
        for asset in (model["urdf"], model["collision"], robot["calibration"]):
            asset["path"] = "[redacted]"
        for key in ("listen", "connect_host", "state_dir", "lock_dir", "tokens_file"):
            host[key] = "[redacted]"
        for body in host["bodies"].values():
            body.update(port="[redacted]", token_env="[redacted]")
        for binding in host["arms"].values():
            for key in ("interface", "usb_serial", "usb_topology"):
                binding[key] = "[redacted]"
        for binding in host["cameras"].values():
            binding["serial"] = "[redacted]"
        return {"model": model, "host": host, "robot": robot,
                "calibration": self.calibration.model_dump(mode="json"), "audit": self.audit()}

    def environment(self, *, execute=False):
        from uni_robot_agent.integrations.piper_x.environment.config import Config
        if execute:
            self.require_motion()
        def body(instance):
            b = self.host.bodies[instance]
            return {"body_id": b.body_id, "url": f"http://{self.host.connect_host}:{b.port}", "token_env": b.token_env}
        arms = {}
        for name, a in self.robot.arms.items():
            c = self.calibration.arms[name]
            arms[name] = {"device_id": a.device_id, "expected_metadata": {"identity": self.identity(name, "arms")},
                "T_world_from_controller": c.T_world_from_base, "T_native_from_grasp": c.T_flange_from_grasp,
                "grasp_site": a.grasp_site, "grasp_site_description": a.grasp_site_description,
                "rotation_radius_m": c.rotation_radius_m, "joint_names": self.model.joint_names,
                "joint_lower": c.joint_lower, "joint_upper": c.joint_upper,
                "tracking_tolerance_rad": TrackingLimits().settle_joint_rad,
                "kinematics": {"urdf": self.urdf, "sha256": self.model.urdf.sha256,
                    "root_link": self.model.root_link, "tip_link": self.model.tip_link,
                    "T_controller_from_urdf": c.T_base_from_urdf, "T_tip_from_control": c.T_tip_from_flange}}
        cameras = {(name if cam.role == "scene" else f"{cam.parent_arm}_wrist_{name}"): {"device_id": cam.device_id, "expected_metadata": {"identity": self.identity(name, "cameras")},
                    "parent": cam.parent_arm or "world", "T_parent_from_optical": self.calibration.cameras[name].T_parent_from_optical,
                    "max_age_s": self.host.cameras[cam.binding].max_frame_age_s,
                    "frame_period_s": 1 / self.host.cameras[cam.binding].color.fps}
                   for name, cam in self.robot.cameras.items()}
        if len(cameras) != len(self.robot.cameras):
            raise ValueError("Tool-side camera names collide; rename the scene camera")
        return Config(robot_id=self.robot.robot_id, arms_body=body(self.robot.arms_body),
            cameras_body=body(self.robot.cameras_body), arms=arms, cameras=cameras, execute=execute,
            motion_profiles=self.robot.motion_profiles, action_timeout_s=self.robot.action_timeout_s,
            normalized_opening_tolerance=min(a.gripper_tolerance_m / (a.gripper_open_m-a.gripper_closed_m)
                                             for a in self.calibration.arms.values()),
            stop_timeout_s=self.robot.stop_timeout_s, max_age_s=self.robot.feedback_max_age_s,
            control_frequency_hz=self.robot.control_frequency_hz, max_episode_steps=self.robot.max_episode_steps)
