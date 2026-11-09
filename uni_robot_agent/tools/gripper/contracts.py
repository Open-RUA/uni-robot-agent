"""Model-facing gripper intents.

The environment service speaks only the primitive protocol.  These request
models describe higher-level tool intentions and therefore live beside the
tool programs that expand them into one or more ``StepRequest`` values.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Literal

from pydantic import Field

from uni_robot_agent.services.environment.contracts import (
    Contract,
    FiniteFloat,
)

Profile = Literal["standard", "approach", "precision"]


class MoveTarget(Contract):
    arm: str
    pose: tuple[FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat, FiniteFloat]
    gripper: FiniteFloat | None = Field(default=None, ge=0, le=1)


class MotionParameters(Contract):
    max_step_m: FiniteFloat = Field(gt=0)
    position_tolerance_m: FiniteFloat = Field(gt=0)
    orientation_tolerance_rad: FiniteFloat = Field(gt=0)


MOTION_PROFILES: dict[str, MotionParameters] = {
    "standard": MotionParameters(max_step_m=0.04, position_tolerance_m=0.03, orientation_tolerance_rad=0.1),
    "approach": MotionParameters(max_step_m=0.04, position_tolerance_m=0.02, orientation_tolerance_rad=0.05),
    "precision": MotionParameters(max_step_m=0.01, position_tolerance_m=0.005, orientation_tolerance_rad=0.05),
}


def motion_profiles(overrides: Mapping[str, Any] | None = None) -> dict[str, MotionParameters]:
    """The shared profiles with an environment's ``motion_profiles`` metadata applied.

    An environment that knows its arm's accuracy replaces whole named profiles; the
    profile names themselves stay fixed because they are part of the move schema.
    """
    if not overrides:
        return dict(MOTION_PROFILES)
    if unknown := set(overrides) - set(MOTION_PROFILES):
        raise ValueError(f"Unknown motion profiles: {sorted(unknown)}")
    return {**MOTION_PROFILES, **{name: MotionParameters.model_validate(values)
                                  for name, values in overrides.items()}}


class MoveRequest(Contract):
    targets: list[MoveTarget] = Field(min_length=1)
    profile: Profile = "standard"
    max_steps: int = Field(default=24, ge=1, le=24, strict=True)
    position_tolerance_m: FiniteFloat | None = Field(default=None, ge=0.001, le=0.1)
    orientation_tolerance_rad: FiniteFloat | None = Field(default=None, ge=0.02, le=0.5)

    def parameters(self, profiles: Mapping[str, MotionParameters] = MOTION_PROFILES) -> MotionParameters:
        defaults = profiles[self.profile]
        return defaults.model_copy(update={key: getattr(self, key) for key in (
            "position_tolerance_m", "orientation_tolerance_rad"
        ) if getattr(self, key) is not None})


class GripperRequest(Contract):
    arms: list[str] = Field(min_length=1)
    action: Literal["grasp", "release"]
    openings: dict[str, FiniteFloat] = Field(default_factory=dict)
    steps: int = Field(default=6, ge=1, le=12, strict=True)


class HoldRequest(Contract):
    steps: int = Field(default=1, ge=1, le=40, strict=True)
