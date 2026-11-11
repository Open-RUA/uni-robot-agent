"""Operator-configured command trajectory limits, shared by the drivers and the host configuration."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass, fields

QUINTIC_PEAK_SPEED = 1.875
QUINTIC_PEAK_ACCELERATION = 10 / math.sqrt(3)


def validate_publish_hz(value: float) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValueError("publish_hz must be a finite positive number")
    return float(value)


@dataclass(frozen=True)
class MotionLimits:
    # Defaults retain the conservative speeds. New EEF steps use Euclidean
    # distance, so simultaneous xyz increments cannot exceed the stated travel.
    eef_step_m: float = 0.05
    eef_speed_m_s: float = 0.03
    eef_accel_m_s2: float = 0.20
    eef_rotation_step_rad: float = 0.35
    eef_rotation_speed_rad_s: float = 0.25
    joint_step_rad: float = 0.35
    joint_speed_rad_s: float = 0.5
    gripper_speed_fraction_s: float = 2.0

    @classmethod
    def from_config(cls, config: Mapping | None = None) -> MotionLimits:
        if config is None:
            return cls()
        if not isinstance(config, Mapping):
            raise ValueError("motion_limits must be an object")
        unknown = set(config) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError(f"unknown motion_limits fields: {sorted(unknown)}")
        values = {}
        for key, value in config.items():
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(
                    f"motion_limits.{key} must be a finite positive number"
                )
            values[key] = float(value)
        return cls(**values)

    def to_dict(self) -> dict[str, float]:
        return asdict(self)

    def eef_duration_s(self, distance_m: float, rotation_rad: float = 0) -> float:
        """Minimum quintic duration; rotation is the SO(3) shortest-arc angle."""
        return max(
            QUINTIC_PEAK_SPEED * distance_m / self.eef_speed_m_s,
            math.sqrt(QUINTIC_PEAK_ACCELERATION * distance_m / self.eef_accel_m_s2),
            QUINTIC_PEAK_SPEED * rotation_rad / self.eef_rotation_speed_rad_s,
        )
