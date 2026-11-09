"""What an arm driver reports so the episode environment can drive it; checked against ``describe``.

A Body device of kind ``arm`` is usable by ``environment/`` when its ``describe`` entry provides:

Metadata
  ``identity``          A mapping the driver chooses: vendor, model, firmware or SDK version and hash,
                        limit profile. The environment pins it exactly and refuses a Body whose
                        identity differs, so put only facts that change behaviour, never prose.
  ``joint_limits_rad``  ``{"lower": [...], "upper": [...]}``, one bound per joint, as enforced.
  ``motion_limits``     Positive numbers for every field of :class:`MotionLimits`.
  ``eef_coordinates``   The end-effector convention from :func:`eef.coordinate_metadata`.
  ``model_sha256``      Optional: SHA-256 of the kinematic model the controller uses.

Signals (``read``)
  ``joint_position_rad``, ``eef_position_m``, ``eef_rpy_rad``, ``gripper_opening`` (0 closed, 1
  open). Optional ``gripper_opening_measured``: false when the opening is an estimate, for
  example before gripper calibration; absent means measured.

Procedures (``invoke``)
  ``move_eef``, ``move_joints`` and ``set_gripper``, with the parameters of the reference drivers.
  Optional ``move_eef_group``: one preflighted path for several arms at once, each target either a
  pose or ``{"hold_joints": [...]}``. Optional ``calibrate_gripper``.

Stopping and holding are part of every driver (``Driver.estop``, ``Driver.stream_hold``).
"""

from __future__ import annotations

import math
from dataclasses import fields
from typing import Any

from .eef import CONTRACT_VERSION
from .motion_limits import MotionLimits

KIND = "arm"
REQUIRED_METADATA = ("identity", "joint_limits_rad", "motion_limits", "eef_coordinates")
MOTION_LIMITS = tuple(field.name for field in fields(MotionLimits))
REQUIRED_SIGNALS = ("joint_position_rad", "eef_position_m", "eef_rpy_rad", "gripper_opening")
GRIPPER_MEASURED = "gripper_opening_measured"
REQUIRED_PROCEDURES = ("move_eef", "move_joints", "set_gripper")
GROUP_PATH = "move_eef_group"


def problems(device: dict[str, Any]) -> list[str]:
    """Every way one ``describe`` device entry falls short of the arm contract; empty when it conforms."""
    found = []
    if device.get("kind") != KIND:
        found.append(f"kind is {device.get('kind')!r}, not {KIND!r}")
    metadata = device.get("metadata") or {}
    for key in REQUIRED_METADATA:
        if key not in metadata:
            found.append(f"metadata.{key} is missing")
    if "identity" in metadata and (not isinstance(metadata["identity"], dict) or not metadata["identity"]):
        found.append("metadata.identity must be a non-empty mapping")
    limits = metadata.get("joint_limits_rad")
    if limits is not None and not _joint_limits_valid(limits):
        found.append("metadata.joint_limits_rad needs lower < upper, one finite pair per joint")
    motion = metadata.get("motion_limits")
    if motion is not None:
        bad = [key for key in MOTION_LIMITS if not _positive(motion.get(key))]
        if bad:
            found.append(f"metadata.motion_limits needs positive {', '.join(bad)}")
    eef = metadata.get("eef_coordinates")
    if eef is not None and (eef.get("contract") != CONTRACT_VERSION or eef.get("position_unit") != "m"
                            or eef.get("orientation_unit") != "rad"):
        found.append(f"metadata.eef_coordinates must follow {CONTRACT_VERSION} in metres and radians")
    signals = device.get("signals") or {}
    found += [f"signal {name} is not declared" for name in REQUIRED_SIGNALS if name not in signals]
    procedures = device.get("procedures") or {}
    found += [f"procedure {name} is not declared" for name in REQUIRED_PROCEDURES if name not in procedures]
    return found


def supports_group_path(device: dict[str, Any]) -> bool:
    """Whether the driver runs one preflighted path for several arms (``move_eef_group``)."""
    return GROUP_PATH in (device.get("procedures") or {})


def opening_measured(signals: dict[str, Any]) -> bool:
    """Whether a reading's gripper opening is measured rather than estimated."""
    return signals.get(GRIPPER_MEASURED, True) is True


def _positive(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(value) and value > 0


def _joint_limits_valid(limits: Any) -> bool:
    if not isinstance(limits, dict):
        return False
    lower, upper = limits.get("lower"), limits.get("upper")
    if not isinstance(lower, list) or not isinstance(upper, list) or not lower or len(lower) != len(upper):
        return False
    return all(type(lo) in (int, float) and type(hi) in (int, float) and math.isfinite(lo)
               and math.isfinite(hi) and lo < hi for lo, hi in zip(lower, upper))
