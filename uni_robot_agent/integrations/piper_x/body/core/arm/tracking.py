"""Pose tracking adapted from ARX ONE PR #35 (731b110), safety.py, MIT.

Compare each measured pose with one recent command, allowing native servo lag
and a continuous deviation interval. Callers own coordinate and clock matching,
freshness, native limits, collision checks and interruption.
"""
from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from dataclasses import asdict, dataclass
import math

from .eef import pose_error


@dataclass(frozen=True)
class TrackingLimits:
    position_m: float = .030
    rotation_rad: float = .20
    lag_s: float = .12
    violation_s: float = .06
    settle_joint_rad: float = .1
    hold_admission_joint_rad: float = .1
    preflight_joint_rad: float = .1

    def __post_init__(self) -> None:
        if any(isinstance(v, bool) or not math.isfinite(v) or v <= 0
               for v in asdict(self).values()):
            raise ValueError("Tracking limits must be finite positive numbers")

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


class PoseTrackingGuard:
    def __init__(self, limits: TrackingLimits, now: float, start: Sequence[float]):
        self.limits = limits
        self.commands: deque[tuple[float, list[float]]] = deque([(now, list(start))])
        self.violation_since: float | None = None
        self.last_feedback_at: float | None = None

    def sent(self, now: float, pose: Sequence[float]) -> None:
        self.commands.append((now, list(pose)))

    def check(self, feedback_at: float, pose: Sequence[float]) -> dict | None:
        if self.last_feedback_at is not None and feedback_at <= self.last_feedback_at:
            return None
        self.last_feedback_at = feedback_at
        # Keep the command bracketing the lag boundary. Both pose components
        # must match the same command; mixing convenient references is invalid.
        while len(self.commands) > 1 and self.commands[1][0] < feedback_at - self.limits.lag_s:
            self.commands.popleft()
        errors = [pose_error(pose, target) for stamp, target in self.commands if stamp <= feedback_at]
        if not errors:
            return None
        position, rotation = min(errors, key=lambda e: max(
            e[0] / self.limits.position_m, e[1] / self.limits.rotation_rad))
        if position <= self.limits.position_m and rotation <= self.limits.rotation_rad:
            self.violation_since = None
            return None
        if self.violation_since is None:
            self.violation_since = feedback_at
        if feedback_at - self.violation_since + 1e-9 < self.limits.violation_s:
            return None
        return {
            "reason": "tracking_deviation",
            "exceeded_components": [name for name, value, limit in (
                ("position", position, self.limits.position_m),
                ("rotation", rotation, self.limits.rotation_rad)) if value > limit],
            "tracking_limits": self.limits.to_dict(),
            "position_error_m": position, "rotation_error_rad": rotation,
            "violation_s": feedback_at - self.violation_since,
            "allowed_lag_s": self.limits.lag_s,
        }
