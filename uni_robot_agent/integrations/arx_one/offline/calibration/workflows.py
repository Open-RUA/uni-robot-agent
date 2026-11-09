"""Offline fitting and frozen-pair verification. No motion or data acquisition."""
from __future__ import annotations

import copy
import hashlib
import json

import numpy as np

from ...perception.extrinsics import (
    profile_id, rigid_matrix, transform_points, validate_identity, validate_record,
)


def verify_frozen_pair(record: dict, data: dict) -> dict:
    """Check new poses against frozen camera and board transforms without refitting.

    Historical records without fitting poses need their original training poses
    supplied for disjointness checking. The output is a report, never an adopted
    calibration or a change to the input status.
    """
    from .hand_eye import _calibration_board

    validate_record(record)
    validate_identity(data)
    for key in ("source_frame", "target_frame", "camera_serial", "mount_id"):
        if data[key] != record.get(key):
            raise ValueError(f"frozen verification {key} mismatch")
    if profile_id(data["intrinsics"]) != record.get("profile_id"):
        raise ValueError("frozen verification intrinsics mismatch")
    board, spec = _calibration_board(record.get("board"))
    if "board" in data and _calibration_board(data["board"])[1] != spec:
        raise ValueError("verification board differs from the frozen record")
    camera = rigid_matrix(record["transform_m"])
    mount = rigid_matrix(record["tool_from_target"])
    fitting = record.get("fit_base_from_tool") or data.get("fit_base_from_tool")
    if not fitting or len(fitting) != record.get("fit_count"):
        raise ValueError("supply all original fitting poses to verify holdout independence")
    fitting = [rigid_matrix(p) for p in fitting]
    samples = data["samples"]
    if len(samples) < 4:
        raise ValueError("need at least four new verification poses")
    corners = (board.getChessboardCorners() if spec["type"] == "charuco"
               else np.array(board.getObjPoints()).reshape(-1, 3))
    errors, rotations, seen = [], [], []
    for sample in samples:
        if sample.get("stationary") is not True:
            raise ValueError("verification requires stationary samples")
        if "board" in sample and _calibration_board(sample["board"])[1] != spec:
            raise ValueError("sample board differs from the frozen record")
        pose = rigid_matrix(sample["base_from_tool"])
        if any(np.allclose(pose, p, atol=1e-4, rtol=0) for p in [*fitting, *seen]):
            raise ValueError("verification poses must be new and distinct")
        seen.append(pose)
        measured = camera @ rigid_matrix(sample["camera_from_target"])
        expected = pose @ mount
        errors.extend(np.linalg.norm(transform_points(corners, measured) - transform_points(corners, expected), axis=1))
        cosine = (np.trace(measured[:3, :3] @ expected[:3, :3].T) - 1) / 2
        rotations.append(float(np.rad2deg(np.arccos(np.clip(cosine, -1, 1)))))
    # A verification may tighten the record's tolerance, never loosen it.
    limits = [value for value in (record.get("max_error_m"), data.get("max_error_m")) if value is not None]
    if not limits or any(type(v) not in (int, float) or not np.isfinite(v) or not 0 < v <= .02 for v in limits):
        raise ValueError("max_error_m must be in (0, 0.02]")
    limit = min(limits)
    maximum = float(max(errors))
    return {
        "schema_version": 1, "report_type": "frozen_pair_verification",
        "passed": maximum <= limit, "record_status_unchanged": record["status"],
        "validation_count": len(samples), "validation_max_m": maximum,
        "validation_rms_m": float(np.sqrt(np.mean(np.square(errors)))),
        "rotation_max_deg": max(rotations), "max_error_m": limit,
        "transform_m": copy.deepcopy(record["transform_m"]),
        "tool_from_target": copy.deepcopy(record["tool_from_target"]),
        "refitted": False, "motion_ready": False,
        "record_content_sha256": hashlib.sha256(json.dumps(record, sort_keys=True, allow_nan=False).encode()).hexdigest(),
        "scope": "RGB target versus supplied robot pose closure, not depth, TCP, clearance or physical readiness",
    }
