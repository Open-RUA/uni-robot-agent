"""Versioned episode protocol and the simulator-independent native boundary.

Positions use world metres. Quaternions use wxyz. Every public arm pose is the
registered grasp site; native controller conversion belongs to the integration.
"""
from __future__ import annotations

from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

FiniteFloat = Annotated[float, Field(strict=True, allow_inf_nan=False)]
Vector3 = tuple[FiniteFloat, FiniteFloat, FiniteFloat]
Quaternion = tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]


class Contract(BaseModel):
    model_config = ConfigDict(extra="forbid", validate_assignment=True)


class Pose(Contract):
    position: Vector3
    quaternion: Quaternion

    @field_validator("quaternion")
    @classmethod
    def normalize_quaternion(cls, value: Quaternion) -> Quaternion:
        length = sum(x * x for x in value) ** 0.5
        if length < 1e-12:
            raise ValueError("quaternion cannot be zero")
        return tuple(x / length for x in value)


class ArmDescription(Contract):
    name: str
    control_frame: str = Field(min_length=1)
    grasp_site_description: str = Field(min_length=1)
    # Conservative radius for rotational waypoint budgeting, independent of frame naming.
    # Covers at least the displacement of the native controller point about the site.
    rotation_radius_m: FiniteFloat = Field(ge=0)
    joint_names: list[str] = Field(default_factory=list)
    joint_tracking_tolerance_rad: FiniteFloat | None = None


class CameraDescription(Contract):
    name: str
    role: str = "scene"
    has_depth: bool = True
    has_calibration: bool = True


class EnvironmentDescription(Contract):
    arms: list[ArmDescription] = Field(min_length=1)
    cameras: list[CameraDescription] = Field(default_factory=list)
    control_frequency_hz: FiniteFloat = Field(gt=0)
    max_episode_steps: int = Field(gt=0)
    # Model bindings are selected from the harness's explicit registry.
    action_binding: str | None = None
    observation_interval_steps: int = Field(default=5, ge=1)
    metadata: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def unique_names(self):
        for items in (self.arms, self.cameras):
            names = [item.name for item in items]
            if len(names) != len(set(names)):
                raise ValueError("descriptor names must be unique")
        return self


class EpisodeInfo(Contract):
    protocol_version: Literal["3"] = "3"
    tool_schema_version: Literal["7"] = "7"
    episode_id: str = Field(min_length=1)
    task: str
    environment: EnvironmentDescription
    capabilities: list[str] = Field(default_factory=list)
    integration: str = Field(min_length=1)
    metadata: dict[str, Any] = Field(default_factory=dict)
    lifecycle: Literal["active", "finishing", "finished", "failed"] = "active"
    next_sequence: int = 1


class ArmState(Contract):
    control_pose: Pose
    joints: list[FiniteFloat] = Field(default_factory=list)
    gripper: FiniteFloat = Field(ge=0, le=1)
    # Complete accepted drive intent; an integration may expose its applied slew target separately.
    gripper_command: FiniteFloat = Field(ge=0, le=1)
    metadata: dict[str, Any] = Field(default_factory=dict)


class EnvironmentState(Contract):
    arms: dict[str, ArmState]
    state_version: int = Field(ge=0)
    native_steps: int = Field(ge=0)
    ended: bool = False
    # Public metadata must never contain privileged task-success or score fields.
    metadata: dict[str, Any] = Field(default_factory=dict)


class ArmCommand(Contract):
    pose: Pose
    gripper: FiniteFloat = Field(ge=0, le=1)


class JointTarget(Contract):
    joints: list[FiniteFloat] = Field(min_length=1)
    gripper: FiniteFloat = Field(ge=0, le=1)


class PoseCommand(Contract):
    kind: Literal["pose"] = "pose"
    # A held arm may retain measured joints while participating arms use poses.
    # Each arm has exactly one command, without override precedence rules.
    arms: dict[str, ArmCommand | JointTarget]
    solver: Literal["native", "dls"] = "native"


class JointCommand(Contract):
    kind: Literal["joints"] = "joints"
    arms: dict[str, JointTarget]


class NativeCommand(Contract):
    kind: Literal["native"] = "native"
    values: list[FiniteFloat] = Field(min_length=1)


ActionCommand = Annotated[PoseCommand | JointCommand | NativeCommand, Field(discriminator="kind")]


class StepReceipt(Contract):
    accepted: bool
    state: EnvironmentState
    diagnostics: dict[str, Any] = Field(default_factory=dict)


class StepRequest(Contract):
    """Request one primitive environment step.

    The command is deliberately a low-level protocol value.  A model-facing
    tool may turn one intent into many such requests, while the environment
    service only validates the state version and forwards this one command to
    the native adapter.
    """

    command: ActionCommand
    expected_state_version: int = Field(ge=0, strict=True)


class PreviewRequest(Contract):
    commands: list[JointCommand] = Field(min_length=1, max_length=50)
    expected_state_version: int = Field(ge=0, strict=True)


class GoalRegionRequest(Contract):
    """Solve-only query about an immutable final control-frame goal."""
    arm: str = Field(min_length=1)
    goal: Pose
    position_tolerance_m: FiniteFloat = Field(gt=0)
    orientation_tolerance_rad: FiniteFloat = Field(gt=0)
    expected_state_version: int = Field(ge=0, strict=True)


class GoalRegionReply(Contract):
    found: bool
    state_version: int = Field(ge=0, strict=True)
    pose: Pose | None = None
    diagnostics: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def found_has_pose(self):
        if self.found != (self.pose is not None):
            raise ValueError("A found goal must have exactly one candidate pose")
        return self


class CaptureRequest(Contract):
    cameras: list[str] | None = None
    high_resolution: bool = True


class FinishRequest(Contract):
    reason: str = Field(min_length=1)
    status: Literal["success", "failure", "give_up"] | None = None


class RequestIdentity(Contract):
    episode_id: str = Field(min_length=1)
    request_id: str = Field(min_length=1)
    sequence: int = Field(ge=1, strict=True)
    execution_id: str | None = None


class RequestEnvelope(Contract):
    identity: RequestIdentity
    payload: dict[str, Any]


class ActionReply(Contract):
    identity: RequestIdentity
    state_before: EnvironmentState
    state_after: EnvironmentState
    feedback: dict[str, Any]
    snapshot: dict[str, Any] | None = None
    trace: dict[str, Any] | None = None
    artifact_error: str | None = None
    execution_status: Literal["completed", "partial", "rejected", "unknown"] = "completed"


class ProtocolError(Contract):
    code: str
    message: str
    request_id: str | None = None
    execution_status: Literal["not_started", "partial", "unknown"] = "not_started"
    details: dict[str, Any] = Field(default_factory=dict)


class RequestRejected(ValueError):
    """A request was rejected before its next native action."""

    def __init__(self, message, *, code="request_rejected", fault_owner="unknown"):
        super().__init__(message)
        self.code = code
        self.fault_owner = fault_owner
