"""Eye-to-hand calibration: fixed head camera, target rigidly fixed to the tool."""

from __future__ import annotations

import numpy as np

from ...perception.extrinsics import profile_id, rigid_matrix, transform_points, validate_identity
from ...perception.geometry import pixel_rays

DEFAULT_CHARUCO = {
    "type": "charuco",
    "columns": 5,
    "rows": 7,
    "square_size_m": 0.030,
    "tag_size_m": 0.022,
    "dictionary": "DICT_4X4_50",
}


def charuco_board():
    import cv2

    return cv2.aruco.CharucoBoard(
        (5, 7), 0.030, 0.022, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    )


def _calibration_board(spec: dict | None):
    """Build the physical target; AprilGrid dimensions have no guessed defaults."""
    import cv2

    if spec is None:
        raise ValueError("supply explicit measured board geometry")
    spec = dict(spec)
    if spec == DEFAULT_CHARUCO:
        return charuco_board(), spec
    if spec.get("type") != "aprilgrid":
        raise ValueError("board must be the default ChArUco or an explicit AprilGrid")
    allowed = {
        "type",
        "columns",
        "rows",
        "tag_size_m",
        "tag_spacing_m",
        "dictionary",
        "ids",
        "origin",
        "marker_border_bits",
    }
    if set(spec) - allowed:
        raise ValueError("unrecognised AprilGrid geometry fields")
    columns, rows = spec.get("columns"), spec.get("rows")
    if any(type(n) is not int or n < 2 for n in (columns, rows)):
        raise ValueError("AprilGrid needs at least two rows and two columns")
    size, gap = spec.get("tag_size_m"), spec.get("tag_spacing_m")
    if any(
        type(v) not in (int, float) or not np.isfinite(v) or v <= 0 for v in (size, gap)
    ):
        raise ValueError(
            "supply measured positive tag_size_m and tag_spacing_m in metres"
        )
    name = spec.get("dictionary")
    if (
        not isinstance(name, str)
        or not name.startswith("DICT_APRILTAG_")
        or not hasattr(cv2.aruco, name)
    ):
        raise ValueError("supply the AprilTag dictionary printed on this board")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
    ids = spec.get("ids", list(range(columns * rows)))
    if (
        not isinstance(ids, list)
        or len(ids) != columns * rows
        or any(
            type(i) is not int or not 0 <= i < len(dictionary.bytesList) for i in ids
        )
        or len(set(ids)) != len(ids)
    ):
        raise ValueError(
            "AprilGrid ids must be unique dictionary IDs in row-major order"
        )
    spec["ids"] = ids
    if spec.get("marker_border_bits", 1) != 1:
        raise ValueError("this AprilGrid detector requires marker_border_bits=1")
    origin = spec.get("origin", "top_left")
    if origin not in {"top_left", "pattern_center"}:
        raise ValueError("AprilGrid origin must be top_left or pattern_center")
    grid = cv2.aruco.GridBoard(
        (columns, rows),
        float(size),
        float(gap),
        dictionary,
        np.array(ids, dtype=np.int32),
    )
    if origin == "pattern_center":
        extent = np.array(
            [columns * size + (columns - 1) * gap, rows * size + (rows - 1) * gap, 0],
            dtype=np.float32,
        )
        points = np.array(grid.getObjPoints()) - extent / 2
        # x right, y down, z into the board, matching the board's design drawing.
        grid = cv2.aruco.Board(points, dictionary, grid.getIds())
    return grid, spec


def _planar_pose(objects, rays, intrinsics):
    """Check both IPPE solutions before accepting the refined planar pose."""
    import cv2

    objects = np.ascontiguousarray(objects, dtype=np.float64)
    ok, rotations, translations, _ = cv2.solvePnPGeneric(
        objects, rays, np.eye(3), np.zeros(5), flags=cv2.SOLVEPNP_IPPE
    )
    candidates = []
    focal = np.array([intrinsics["fx"], intrinsics["fy"]], dtype=float)
    if ok:
        for rvec, tvec in zip(rotations, translations):
            if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                continue
            rvec, tvec = cv2.solvePnPRefineLM(
                objects, rays, np.eye(3), np.zeros(5), rvec.copy(), tvec.copy()
            )
            rotation = cv2.Rodrigues(rvec)[0]
            camera_points = objects @ rotation.T + tvec.ravel()
            if not np.isfinite(camera_points).all() or np.any(camera_points[:, 2] <= 0):
                continue
            projected = camera_points[:, :2] / camera_points[:, 2:]
            error = float(
                np.sqrt(np.mean(np.sum(((projected - rays) * focal) ** 2, axis=1)))
            )
            candidates.append((error, rotation, tvec.ravel()))
    candidates.sort(key=lambda item: item[0])
    if not candidates:
        raise ValueError("board pose could not be resolved with positive depth")
    best = candidates[0]
    if best[0] > 0.75:
        raise ValueError("board reprojection error exceeds 0.75 px")
    if len(candidates) > 1:
        alternative = candidates[1]
        angle = float(np.linalg.norm(cv2.Rodrigues(best[1] @ alternative[1].T)[0]))
        if angle > np.deg2rad(5) and alternative[0] - best[0] < 0.1:
            raise ValueError(
                "ambiguous planar board pose; collect a clearer tilted view"
            )
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = best[1], best[2]
    return matrix, best[0], [candidate[0] for candidate in candidates]


def _refine_aprilgrid_edges(gray, corners):
    """Locate black/white boundaries away from attached corner decorations.

    Candidate decoding supplies corner order only. All subpixel measurements
    come from the original image, without a board-layout or pose fit.
    """
    import cv2

    corners = np.asarray(corners, dtype=float).reshape(4, 2)
    lines = []
    offsets = np.linspace(-3, 3, 49)
    midpoints = (offsets[:-1] + offsets[1:]) / 2
    for start, end in zip(corners, np.roll(corners, -1, axis=0)):
        direction = end - start
        length = np.linalg.norm(direction)
        if length < 12:
            raise ValueError("AprilGrid marker edge is too short")
        direction /= length
        normal = np.array([direction[1], -direction[0]])
        centers = start + np.linspace(0.18, 0.82, 32)[:, None] * (end - start)
        samples = centers[:, None, :] + offsets[None, :, None] * normal
        if (
            np.any(samples < 0)
            or np.any(samples[:, :, 0] >= gray.shape[1] - 1)
            or np.any(samples[:, :, 1] >= gray.shape[0] - 1)
        ):
            raise ValueError("AprilGrid border sampling leaves the image")
        values = cv2.remap(
            gray,
            samples[:, :, 0].astype(np.float32),
            samples[:, :, 1].astype(np.float32),
            cv2.INTER_LINEAR,
        )
        gradient = np.maximum(np.diff(values, axis=1), 0)
        peaks = np.argmax(gradient, axis=1)
        near_peak = abs(np.arange(48)[None, :] - peaks[:, None]) <= 7
        weights = (gradient * near_peak) ** 2
        sums = weights.sum(1)
        useful = (gradient.max(1) > 2) & (sums > 0) & (peaks > 3) & (peaks < 44)
        if useful.sum() < 16:
            raise ValueError("insufficient AprilGrid border contrast")
        distances = (weights[useful] @ midpoints) / sums[useful]
        points = centers[useful] + distances[:, None] * normal
        for _ in range(3):
            center = points.mean(0)
            _, _, vectors = np.linalg.svd(points - center)
            residual = abs((points - center) @ vectors[-1])
            keep = residual <= max(0.2, np.median(residual) * 3)
            if keep.all():
                break
            points = points[keep]
            if len(points) < 12:
                raise ValueError("AprilGrid border is not a stable straight edge")
        center = points.mean(0)
        _, _, vectors = np.linalg.svd(points - center)
        normal = vectors[-1]
        lines.append((normal, normal @ center))
    result = np.array(
        [
            np.linalg.solve(
                np.vstack([lines[i - 1][0], lines[i][0]]),
                [lines[i - 1][1], lines[i][1]],
            )
            for i in range(4)
        ]
    )
    if (
        not np.isfinite(result).all()
        or np.linalg.norm(result - corners, axis=1).max() > 4
    ):
        raise ValueError("AprilGrid corner refinement moved too far")
    return result.astype(np.float32).reshape(1, 4, 2)


def _detect_aprilgrid(gray, board):
    """Decode at multiple scales, then measure full-resolution border edges."""
    import cv2

    parameters = cv2.aruco.DetectorParameters()
    parameters.markerBorderBits = 1
    parameters.adaptiveThreshWinSizeMax = 61
    detector = cv2.aruco.ArucoDetector(board.getDictionary(), parameters)
    known = {int(i) for i in board.getIds()}
    matched = {}
    original = gray.astype(np.float32)
    # The embossed pattern's corner squares may join neighbouring contours.
    # Smaller candidate images can separate them; never refine on resized data.
    for scale, separate_contours in (
        (1.0, False),
        (0.75, False),
        (0.5, False),
        (1.0, True),
        (0.75, True),
    ):
        if len(matched) == len(known):
            break
        height, width = gray.shape
        scaled = (
            gray
            if scale == 1
            else cv2.resize(
                gray,
                (round(width * scale), round(height * scale)),
                interpolation=cv2.INTER_AREA,
            )
        )
        candidates = (
            cv2.dilate(scaled, np.ones((2, 2), np.uint8))
            if separate_contours
            else scaled
        )
        corners, ids, _ = detector.detectMarkers(candidates)
        if ids is None:
            continue
        ratio = np.array([scaled.shape[1] / width, scaled.shape[0] / height])
        for tag_id, candidate in zip(ids.ravel(), corners):
            tag_id = int(tag_id)
            if tag_id not in known or tag_id in matched:
                continue
            candidate = (candidate + 0.5) / ratio - 0.5
            try:
                matched[tag_id] = _refine_aprilgrid_edges(original, candidate)
            except (ValueError, np.linalg.LinAlgError):
                continue
    return list(matched.items())


def target_pose(
    rgb: np.ndarray, intrinsics: dict, board_spec: dict | None = None
) -> dict:
    """Detect a measured target and solve its pose using undistorted optical rays."""
    import cv2

    board, spec = _calibration_board(board_spec)
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    if spec["type"] == "charuco":
        corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(gray)
        if ids is None or len(ids) < 12:
            raise ValueError("need at least 12 visible ChArUco corners")
        objects = board.getChessboardCorners()[ids.ravel()]
        pixels = corners.reshape(-1, 2)
    else:
        matched = _detect_aprilgrid(gray, board)
        if len(matched) < 4 or len({i for i, _ in matched}) != len(matched):
            raise ValueError("need at least four distinct visible AprilGrid tags")
        indices = [spec["ids"].index(i) for i, _ in matched]
        if (
            len({i // spec["columns"] for i in indices}) < 2
            or len({i % spec["columns"] for i in indices}) < 2
        ):
            raise ValueError(
                "visible AprilGrid tags must span at least two rows and columns"
            )
        ids = np.array([i for i, _ in matched], dtype=np.int32)
        objects, pixels = board.matchImagePoints([c for _, c in matched], ids)
        objects, pixels = objects.reshape(-1, 3), pixels.reshape(-1, 2)
    rays = np.ascontiguousarray(pixel_rays(pixels[:, ::-1], intrinsics)[:, :2])
    matrix, pixel_error, candidate_errors = _planar_pose(objects, rays, intrinsics)
    return {
        "camera_from_target": matrix.tolist(),
        "board": spec,
        "detected_ids": ids.ravel().tolist(),
        "corners": len(pixels),
        "reprojection_rms_px": pixel_error,
        "pnp_method": "ippe_lm",
        "corner_refinement": (
            "original_image_border_gradients"
            if spec["type"] == "aprilgrid"
            else "charuco_subpixel"
        ),
        "candidate_reprojection_rms_px": candidate_errors,
    }


def fitting_samples(data: dict):
    """Validate hand-eye inputs; returns the board, its spec and the fitting/validation samples.

    Both hand-eye solvers need stationary, distinct robot poses with rotation about at least
    two axes, at least 12 fitting poses and 4 independent validation poses on one board.
    """
    import cv2

    validate_identity(data)
    board, board_spec = _calibration_board(data.get("board"))
    samples = data["samples"]
    train = [s for s in samples if not s.get("validation", False)]
    check = [s for s in samples if s.get("validation", False)]
    if len(train) < 12 or len(check) < 4:
        raise ValueError(
            "collect at least 12 fitting poses and 4 independent validation poses"
        )
    for sample in samples:
        if type(sample.get("validation", False)) is not bool:
            raise ValueError("validation must be a boolean")
        if "board" in sample and _calibration_board(sample["board"])[1] != board_spec:
            raise ValueError("all samples must use the same measured board geometry")
        if sample.get("stationary") is not True:
            raise ValueError(
                "samples must be captured with stationary, fresh robot feedback"
            )
        rigid_matrix(sample["base_from_tool"])
        rigid_matrix(sample["camera_from_target"])
    poses = [rigid_matrix(s["base_from_tool"]) for s in samples]
    if any(np.allclose(pose, other, atol=1e-4, rtol=0)
           for i, pose in enumerate(poses) for other in poses[:i]):
        raise ValueError("fitting and validation poses must be distinct")
    robot = [rigid_matrix(s["base_from_tool"]) for s in train]
    variations = np.array(
        [cv2.Rodrigues(r[:3, :3] @ robot[0][:3, :3].T)[0].ravel() for r in robot]
    )
    if np.linalg.svd(variations, compute_uv=False)[1] < 0.25:
        raise ValueError(
            "robot poses need substantial rotation about at least two axes"
        )
    for sample in check:
        if any(np.allclose(sample["base_from_tool"], m, atol=1e-4) for m in robot):
            raise ValueError("validation robot poses must be independent")
    return board, board_spec, samples, train, check


def board_corners(board, board_spec):
    return (
        board.getChessboardCorners()
        if board_spec["type"] == "charuco"
        else np.array(board.getObjPoints()).reshape(-1, 3)
    )


def rigid_mean(matrices):
    """The rigid transform nearest to the mean of nearly equal rigid transforms."""
    u, _, vt = np.linalg.svd(np.mean([m[:3, :3] for m in matrices], axis=0))
    mean = np.eye(4)
    mean[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    mean[:3, 3] = np.mean([m[:3, 3] for m in matrices], axis=0)
    return mean


def solve_eye_to_hand(data: dict) -> dict:
    """Solve and independently validate a fixed camera using varied robot poses.

    OpenCV expects gripper-to-base for eye-in-hand. Inverting those inputs
    swaps base/gripper roles for eye-to-hand; output is then base-from-camera.
    See OpenCV's test_calibration_hand_eye.cpp eye-to-hand test.
    """
    import cv2

    board, board_spec, samples, train, check = fitting_samples(data)
    robot = [rigid_matrix(s["base_from_tool"]) for s in train]
    target = [rigid_matrix(s["camera_from_target"]) for s in train]
    inverted = [np.linalg.inv(m) for m in robot]
    rotation, translation = cv2.calibrateHandEye(
        [m[:3, :3] for m in inverted],
        [m[:3, 3] for m in inverted],
        [m[:3, :3] for m in target],
        [m[:3, 3] for m in target],
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rotation, translation.ravel()
    rigid_matrix(result)
    mean = rigid_mean([np.linalg.inv(b) @ result @ c for b, c in zip(robot, target)])
    corners = board_corners(board, board_spec)
    errors, camera_points = [], []
    for sample in check:
        c = transform_points(corners, sample["camera_from_target"])
        actual = transform_points(c, result)
        expected = transform_points(
            corners, rigid_matrix(sample["base_from_tool"]) @ mean
        )
        errors.extend(np.linalg.norm(actual - expected, axis=1).tolist())
    for sample in samples:
        camera_points.extend(
            transform_points(corners, sample["camera_from_target"]).tolist()
        )
    error = float(max(errors))
    tolerance = float(data.get("max_error_m", 0.01))
    if not 0 < tolerance <= 0.02:
        raise ValueError("max_error_m must be in (0, 0.02]")
    metadata = {
        k: data[k]
        for k in (
            "source_frame",
            "target_frame",
            "camera_serial",
            "intrinsics",
            "mount_id",
            "measured_at",
        )
    }
    metadata.update(
        schema_version=1,
        method=f"{board_spec['type']}_eye_to_hand_park",
        board=board_spec,
        status="validated" if error <= tolerance else "rejected",
        rejection_reasons=(
            [] if error <= tolerance else ["heldout_error_exceeds_limit"]
        ),
        transform_m=result.tolist(),
        tool_from_target=mean.tolist(),
        profile_id=profile_id(data["intrinsics"]),
        fit_count=len(train),
        validation_count=len(check),
        validation_max_m=error,
        validation_rms_m=float(np.sqrt(np.mean(np.asarray(errors) ** 2))),
        max_error_m=tolerance,
        calibrated_volume_camera_m=[
            np.min(camera_points, axis=0).tolist(),
            np.max(camera_points, axis=0).tolist(),
        ],
        convention="p_target = R_target_source @ p_source + t_target_source",
        units="m",
        motion_ready=False,
        fit_base_from_tool=[m.tolist() for m in robot],
    )
    known_mount = data.get("known_mount")
    if known_mount is not None:
        if board_spec.get("origin") != "pattern_center":
            raise ValueError(
                "known mount center check requires pattern_center board origin"
            )
        expected = np.asarray(known_mount["position_m"], dtype=float)
        if expected.shape != (3,) or not np.isfinite(expected).all():
            raise ValueError("known mount position must be a finite 3-vector in metres")
        difference = mean[:3, 3] - expected
        metadata["known_mount_check"] = {
            "expected_position_m": expected.tolist(),
            "estimated_position_m": mean[:3, 3].tolist(),
            "difference_m": difference.tolist(),
            "distance_m": float(np.linalg.norm(difference)),
            "used_in_fit": False,
            "source": known_mount.get("source"),
        }
        if "max_position_error_m" in known_mount:
            mount_tolerance = known_mount["max_position_error_m"]
            if (
                type(mount_tolerance) not in (int, float)
                or not np.isfinite(mount_tolerance)
                or mount_tolerance <= 0
            ):
                raise ValueError(
                    "known mount max_position_error_m must be positive and finite"
                )
            check = metadata["known_mount_check"]
            check["max_position_error_m"] = float(mount_tolerance)
            check["passed"] = check["distance_m"] <= mount_tolerance
            if not check["passed"]:
                metadata["status"] = "rejected"
                metadata["rejection_reasons"].append(
                    "known_mount_position_exceeds_limit"
                )
    return metadata
