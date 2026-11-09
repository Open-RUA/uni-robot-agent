"""Eye-in-hand calibration: a camera rigidly mounted on the tool, observing a target fixed to the base.

The result is ``tool_from_camera``: for a wrist camera, the native Body EEF from the camera optical
frame, which is ``cameras.<wrist>.T_parent_from_optical`` in environment.json.
"""

from __future__ import annotations

import numpy as np

from ...perception.extrinsics import profile_id, rigid_matrix, transform_points
from .hand_eye import board_corners, fitting_samples, rigid_mean


def solve_eye_in_hand(data: dict) -> dict:
    """Solve tool-from-camera from varied robot poses and validate it on held-out poses.

    Each sample gives ``base_from_tool`` (the tool frame the camera is mounted on, from fresh
    stationary feedback) and ``camera_from_target`` for a target that stays fixed in the base.
    OpenCV's gripper-to-base and target-to-camera inputs are these transforms directly.
    """
    import cv2

    board, board_spec, samples, train, check = fitting_samples(data)
    robot = [rigid_matrix(s["base_from_tool"]) for s in train]
    target = [rigid_matrix(s["camera_from_target"]) for s in train]
    rotation, translation = cv2.calibrateHandEye(
        [m[:3, :3] for m in robot],
        [m[:3, 3] for m in robot],
        [m[:3, :3] for m in target],
        [m[:3, 3] for m in target],
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    tool_from_camera = np.eye(4)
    tool_from_camera[:3, :3], tool_from_camera[:3, 3] = rotation, translation.ravel()
    rigid_matrix(tool_from_camera)
    # The target stays fixed in the base; every fitting pose gives one estimate of it.
    base_from_target = rigid_mean([b @ tool_from_camera @ c for b, c in zip(robot, target)])
    corners = board_corners(board, board_spec)
    errors, camera_points = [], []
    for sample in check:
        actual = transform_points(corners, rigid_matrix(sample["base_from_tool"]) @ tool_from_camera
                                  @ rigid_matrix(sample["camera_from_target"]))
        expected = transform_points(corners, base_from_target)
        errors.extend(np.linalg.norm(actual - expected, axis=1).tolist())
    for sample in samples:
        camera_points.extend(transform_points(corners, sample["camera_from_target"]).tolist())
    error = float(max(errors))
    tolerance = float(data.get("max_error_m", 0.01))
    if not 0 < tolerance <= 0.02:
        raise ValueError("max_error_m must be in (0, 0.02]")
    record = {key: data[key] for key in
              ("source_frame", "target_frame", "camera_serial", "intrinsics", "mount_id", "measured_at")}
    record.update(
        schema_version=1,
        method=f"{board_spec['type']}_eye_in_hand_park",
        board=board_spec,
        status="validated" if error <= tolerance else "rejected",
        rejection_reasons=[] if error <= tolerance else ["heldout_error_exceeds_limit"],
        transform_m=tool_from_camera.tolist(),
        base_from_target=base_from_target.tolist(),
        profile_id=profile_id(data["intrinsics"]),
        fit_count=len(train),
        validation_count=len(check),
        validation_max_m=error,
        validation_rms_m=float(np.sqrt(np.mean(np.asarray(errors) ** 2))),
        max_error_m=tolerance,
        calibrated_volume_camera_m=[np.min(camera_points, axis=0).tolist(),
                                    np.max(camera_points, axis=0).tolist()],
        convention="p_target = R_target_source @ p_source + t_target_source",
        units="m",
        motion_ready=False,
        fit_base_from_tool=[m.tolist() for m in robot],
    )
    return record
