"""Rigid camera-to-base calibration records and a binding that checks observation identities against them."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


def profile_id(intrinsics: dict) -> str:
    from .geometry import pixel_rays

    pixel_rays(np.empty((0, 2)), intrinsics)
    return hashlib.sha256(json.dumps(intrinsics, sort_keys=True, allow_nan=False).encode()).hexdigest()


def rigid_matrix(value: Any) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("transform must be a finite 4x4 matrix")
    rotation = matrix[:3, :3]
    if (
        not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0)
        or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5, rtol=0)
    ):
        raise ValueError("transform must be a proper rigid transform in metres")
    return matrix


def transform_points(points: np.ndarray, matrix: Any) -> np.ndarray:
    m = rigid_matrix(matrix)
    return finite_points(points) @ m[:3, :3].T + m[:3, 3]


def finite_points(points: Any) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("points must be finite Nx3 coordinates in metres")
    return points


def _pairs(source: Any, target: Any, minimum: int) -> tuple[np.ndarray, np.ndarray]:
    a, b = np.asarray(source, dtype=float), np.asarray(target, dtype=float)
    if (
        a.shape != b.shape
        or a.ndim != 2
        or a.shape[1] != 3
        or len(a) < minimum
        or not np.isfinite([a, b]).all()
    ):
        raise ValueError(f"need at least {minimum} finite matched 3-D points in metres")
    if np.linalg.svd(a - a.mean(axis=0), compute_uv=False)[1] < 0.03:
        raise ValueError(
            "calibration landmarks need spatial spread on at least two axes"
        )
    return a, b


def fit_calibration(data: dict) -> dict:
    validate_identity(data)
    a, b = _pairs(data["camera_points_m"], data["base_points_m"], 6)
    va, vb = _pairs(
        data["validation_camera_points_m"], data["validation_base_points_m"], 4
    )
    if any(np.linalg.norm(a - point, axis=1).min() < 1e-5 for point in va):
        raise ValueError(
            "validation landmarks must be independent of fitting landmarks"
        )
    u, _, vt = np.linalg.svd((a - a.mean(0)).T @ (b - b.mean(0)))
    correction = np.diag([1, 1, np.linalg.det(vt.T @ u.T)])
    rotation = vt.T @ correction @ u.T
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = b.mean(0) - rotation @ a.mean(0)
    train = np.linalg.norm(transform_points(a, matrix) - b, axis=1)
    validation = np.linalg.norm(transform_points(va, matrix) - vb, axis=1)
    max_error = float(data.get("max_error_m", 0.01))
    if not 0 < max_error <= 0.02:
        raise ValueError("max_error_m must be in (0, 0.02]")
    passed = max(train.max(), validation.max()) <= max_error
    result = {
        key: data[key]
        for key in (
            "source_frame",
            "target_frame",
            "camera_serial",
            "intrinsics",
            "mount_id",
            "measured_at",
            "method",
        )
    }
    result.update(
        schema_version=1,
        status="validated" if passed else "rejected",
        transform_m=matrix.tolist(),
        profile_id=profile_id(data["intrinsics"]),
        fit_rms_m=float(np.sqrt(np.mean(train**2))),
        validation_rms_m=float(np.sqrt(np.mean(validation**2))),
        validation_max_m=float(validation.max()),
        max_error_m=max_error,
        fit_count=len(a),
        validation_count=len(va),
        calibrated_volume_camera_m=[
            np.min(np.vstack((a, va)), axis=0).tolist(),
            np.max(np.vstack((a, va)), axis=0).tolist(),
        ],
        convention="p_target = R_target_source @ p_source + t_target_source",
        units="m",
        motion_ready=False,
    )
    return result


def within_calibration_volume(points: np.ndarray, record: dict) -> np.ndarray:
    """Report sampled-region coverage, not reachability or contact accuracy."""
    points = np.asarray(points, dtype=float)
    bounds = np.asarray(record["calibrated_volume_camera_m"], dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("points must be finite Nx3 coordinates in metres")
    if (
        bounds.shape != (2, 3)
        or not np.isfinite(bounds).all()
        or np.any(bounds[0] > bounds[1])
    ):
        raise ValueError("calibration volume must contain finite ordered 3-D bounds")
    return np.all((points >= bounds[0]) & (points <= bounds[1]), axis=1)


def calibrated_points(
    points: np.ndarray,
    record: dict,
    *,
    serial: str,
    intrinsics: dict,
    source_frame: str,
    target_frame: str,
    mount_id: str,
) -> np.ndarray:
    validate_record(record, require_validated=True)
    if (
        record.get("status") != "validated"
        or not mount_id
        or record.get("mount_id") != mount_id
    ):
        raise ValueError("camera mount has no matching validated calibration")
    if record.get("camera_serial") != serial or record.get("profile_id") != profile_id(
        intrinsics
    ):
        raise ValueError("camera identity or stream profile differs from calibration")
    if (
        record.get("source_frame") != source_frame
        or record.get("target_frame") != target_frame
    ):
        raise ValueError("calibration coordinate frames do not match")
    error = float(record.get("validation_max_m", float("inf")))
    if not np.isfinite(error) or error > min(float(record.get("max_error_m", 0)), 0.02):
        raise ValueError("calibration has no acceptable independent validation")
    # Validate the input and coverage record without treating the sample bounds
    # as a physical limit of the rigid transform.
    within_calibration_volume(points, record)
    return transform_points(points, record["transform_m"])


CONVENTION = "p_target = R_target_source @ p_source + t_target_source"
STATUSES = {"validated", "candidate", "estimated", "rejected"}


def _name(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip() or value != value.strip():
        raise ValueError(f"{label} must be a nonempty, trimmed string")
    return value


def validate_identity(record: dict) -> None:
    if record.get("units", "m") != "m" or record.get("convention", CONVENTION) != CONVENTION:
        raise ValueError("calibration inputs must use the target-from-source metre convention")
    for field in ("source_frame", "target_frame", "camera_serial", "mount_id", "measured_at"):
        _name(record.get(field), field)
    if record["source_frame"] == record["target_frame"]:
        raise ValueError("camera source and target frames must differ")
    profile_id(record["intrinsics"])


def validate_record(record: dict, *, require_validated: bool = False) -> None:
    """Validate evidence, never promote an estimated/candidate record by filename.

    Legacy schema 1 uses transform_m and the exact metre convention. Its omitted
    units field is accepted only under that existing convention.
    """
    if not isinstance(record, dict):
        raise ValueError("calibration record must be an object")
    if type(record.get("schema_version")) is not int or record["schema_version"] != 1:
        raise ValueError("unsupported calibration schema_version")
    if record.get("status") not in STATUSES:
        raise ValueError("unknown calibration status")
    rigid_matrix(record.get("transform_m"))
    if record.get("units", "m") != "m" or record.get("convention") != CONVENTION:
        raise ValueError("calibration requires the explicit target-from-source metre convention")
    for field in ("source_frame", "target_frame"):
        _name(record.get(field), field)
    if record["source_frame"] == record["target_frame"]:
        raise ValueError("camera source and target frames must differ")
    if "tool_from_target" in record:
        rigid_matrix(record["tool_from_target"])
    if require_validated and record["status"] != "validated":
        raise ValueError("calibration is not independently validated")
    if record["status"] != "validated":
        return  # Historical estimates remain inspectable, never usable via convert().
    validate_identity(record)
    _name(record.get("method"), "method")
    if record.get("profile_id") != profile_id(record["intrinsics"]):
        raise ValueError("stored intrinsics do not match profile_id")
    for field, minimum in (("fit_count", 6), ("validation_count", 4)):
        if type(record.get(field)) is not int or record[field] < minimum:
            raise ValueError(f"{field} must be at least {minimum}")
    limit, error = record.get("max_error_m"), record.get("validation_max_m")
    if (type(limit) not in (int, float) or not np.isfinite(limit) or not 0 < limit <= .02
            or type(error) not in (int, float) or not np.isfinite(error) or not 0 <= error <= limit):
        raise ValueError("calibration has no acceptable independent validation")
    if record.get("rejection_reasons"):
        raise ValueError("validated record retains active rejection reasons")
    check = record.get("known_mount_check", {})
    if check.get("passed") is False:
        raise ValueError("known mount acceptance check failed")
    within_calibration_volume(np.empty((0, 3)), record)


@dataclass(frozen=True)
class RigidTransform:
    """Frame-labelled mathematical transform, not calibration or motion evidence."""

    source_frame: str
    target_frame: str
    values: tuple

    def __post_init__(self):
        _name(self.source_frame, "source_frame")
        _name(self.target_frame, "target_frame")
        object.__setattr__(self, "values", tuple(tuple(float(x) for x in row) for row in rigid_matrix(self.values)))

    @property
    def matrix(self) -> np.ndarray:
        return np.array(self.values)

    def apply(self, points: Any) -> np.ndarray:
        return transform_points(points, self.matrix)

    def inverse(self) -> RigidTransform:
        return RigidTransform(self.target_frame, self.source_frame, np.linalg.inv(self.matrix))

    def then(self, following: RigidTransform) -> RigidTransform:
        if self.target_frame != following.source_frame:
            raise ValueError("transform chain has incompatible coordinate frames")
        return RigidTransform(self.source_frame, following.target_frame, following.matrix @ self.matrix)


@dataclass(frozen=True)
class Calibration:
    """Immutable input bytes; metadata access returns an independent JSON value."""

    raw: bytes
    origin: str

    def __post_init__(self):
        object.__setattr__(self, "raw", bytes(self.raw))
        validate_record(self.record)

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.raw).hexdigest()

    @property
    def record(self) -> dict:
        return json.loads(self.raw)

    def convert(self, points: Any, *, serial: str, intrinsics: dict,
                source_frame: str, target_frame: str, mount_id: str) -> dict:
        record = self.record
        converted = calibrated_points(points, record, serial=serial, intrinsics=intrinsics,
                                      source_frame=source_frame, target_frame=target_frame, mount_id=mount_id)
        covered = within_calibration_volume(points, record)
        return {
            "target_xyz_m": converted.tolist(),
            "source_frame": source_frame, "target_frame": target_frame,
            "calibration_status": record["status"], "calibration_sha256": self.sha256,
            "within_validation_volume": covered.tolist(),
            "validation_max_m": [record["validation_max_m"] if ok else None for ok in covered],
            "warning": None if covered.all() else "Some points lie outside the independently sampled volume; accuracy there is unverified.",
            "motion_ready": False, "absolute_accuracy_m": None,
        }


def load_calibration(path: str | Path, *, expected_sha256: str | None = None) -> Calibration:
    """Read exactly one explicitly selected file. No fallback or device access."""
    path = Path(path).expanduser().resolve()
    return calibration_from_bytes(path.read_bytes(), origin=str(path), expected_sha256=expected_sha256)


def calibration_from_bytes(raw: bytes, *, origin: str, expected_sha256: str | None = None) -> Calibration:
    digest = hashlib.sha256(raw).hexdigest()
    if expected_sha256 is not None and expected_sha256 != digest:
        raise ValueError("calibration SHA-256 does not match the configured artifact")
    return Calibration(raw, origin)


@dataclass(frozen=True)
class BoundCalibration:
    calibration: Calibration
    camera_device: str
    target_device: str
    camera_serial: str
    mount_id: str
    source_frame: str
    target_frame: str

    def convert(self, points, *, camera_device: str, target_device: str,
                serial: str, intrinsics: dict, mount_id: str,
                source_frame: str, target_frame: str) -> dict:
        """The caller supplies observation identities, not values inferred from config."""
        if (camera_device, target_device, serial, mount_id) != (
                self.camera_device, self.target_device, self.camera_serial, self.mount_id):
            raise ValueError("observation/device/mount identity differs from Site binding")
        if (source_frame, target_frame) != (self.source_frame, self.target_frame):
            raise ValueError("observation/target coordinate frames differ from Site binding")
        return self.calibration.convert(points, serial=serial, intrinsics=intrinsics,
                                        source_frame=source_frame, target_frame=target_frame,
                                        mount_id=mount_id)
