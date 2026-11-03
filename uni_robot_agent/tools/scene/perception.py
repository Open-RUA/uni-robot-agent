"""SAM inference and calibrated geometry over immutable remote snapshots."""

from __future__ import annotations

from io import BytesIO

import numpy as np
from PIL import Image, ImageDraw

from uni_robot_agent.services.environment.contracts import RequestRejected
from uni_robot_agent.services.environment.snapshots import CameraFrame
from uni_robot_agent.services.perception.contracts import PointQuery, TextQuery
from uni_robot_agent.tools.presentation import round_physical
from uni_robot_agent.tools.scene.geometry import (
    mask_geometry_quality,
    project_mask,
    project_pixels,
)
from uni_robot_agent.tools.toolkit import ImageAttachment, ToolOutput
from uni_robot_agent.tools.scene.triangulation import triangulate


def snapshot_images(client, manifest, cameras=None, *, mirror):
    available = manifest["cameras"]
    selected = list(available) if cameras is None else cameras
    if not selected or len(set(selected)) != len(selected) or not set(selected) <= set(available):
        raise RequestRejected("Requested cameras must be distinct and available in the recorded snapshot")
    return [ImageAttachment(mirror.store.read_bytes(mirror.mirror_ref(available[name]["rgb"])["artifact_id"]),
                            mime_type="image/png", name=name) for name in selected]


def snapshot_metadata(manifest, cameras, *, mirror):
    """Project only the attached image grids; never substitute live calibration."""
    return {
        "snapshot_id": manifest["snapshot_id"],
        "image_state_version": manifest["state_version"],
        "captured_at": manifest["captured_at"],
        "camera_calibration": {name: {
            "image_size": manifest["cameras"][name]["image_size"],
            "intrinsic_K": manifest["cameras"][name].get("intrinsic_K"),
            "T_world_from_optical": manifest["cameras"][name].get("T_world_from_optical"),
            "image_path": str(mirror.local_path(manifest["cameras"][name]["rgb"])),
            **({"calibration": {key: value for key, value in manifest["cameras"][name]["calibration"].items()
                                if key in ("status", "source", "reason", "pixel_coordinates")
                                or (manifest["state"].get("kind") == "devices" and key in (
                                    "rgb_timestamp_unix", "depth_timestamp_unix", "received_at_unix",
                                    "synchronized", "calibration_id", "max_pair_skew_s", "distortion_model"))}}
               if manifest["cameras"][name].get("calibration") else {}),
        } for name in cameras},
    }


def load_frame(client, manifest, camera, *, mirror):
    return _frame_and_image(client, manifest, camera, mirror=mirror)[0]


def _frame_and_image(client, manifest, camera, *, mirror, load_depth=True):
    if camera not in manifest["cameras"]:
        raise RequestRejected(f"Camera {camera!r} is absent from the snapshot")
    item = manifest["cameras"][camera]
    rgb_data = mirror.store.read_bytes(mirror.mirror_ref(item["rgb"])["artifact_id"])
    with Image.open(BytesIO(rgb_data)) as image:
        rgb = np.asarray(image.convert("RGB"))
    depth = item.get("depth_m") if load_depth else None
    if depth is None:
        depth_data = None
    else:
        depth_data = mirror.store.read_bytes(mirror.mirror_ref(depth)["artifact_id"])
    frame = CameraFrame(
        rgb=rgb,
        depth_m=None if depth_data is None else np.load(BytesIO(depth_data), allow_pickle=False),
        intrinsic_K=None if item.get("intrinsic_K") is None else np.asarray(item["intrinsic_K"], dtype=float),
        T_world_from_optical=None if item.get("T_world_from_optical") is None else np.asarray(item["T_world_from_optical"], dtype=float),
        calibration=item.get("calibration"),
    )
    return frame, rgb_data


def locate(episode, *, perception, method, snapshot_id, camera=None, prompt=None, point=None, points=None,
           observations=None, min_score=.2, correspondences=None, fit_table=False):
    """Localization cannot change the frozen scene or advance simulator state."""
    cameras = {item.name: item for item in episode.description.environment.cameras}
    extended = episode.description.environment.metadata.get("scene_geometry", False)
    if (correspondences is not None or fit_table) and not extended:
        raise RequestRejected("Batch scene geometry is not declared")
    if method != "triangulate" and (correspondences is not None or fit_table):
        raise RequestRejected("Batch correspondences and plane fitting require triangulate")
    if method == "triangulate":
        if any(value is not None for value in (camera, prompt, point, points)):
            raise RequestRejected("triangulate requires observations only, without single-camera arguments")
        if correspondences is not None:
            if observations is not None or not isinstance(correspondences, list) or not 1 <= len(correspondences) <= 64:
                raise RequestRejected("Provide 1-64 correspondences without observations")
            results = [_triangulate(episode, snapshot_id, item, cameras) for item in correspondences]
            payload = {"snapshot_id": snapshot_id, "method": method, "points": [r.extra for r in results]}
            if fit_table:
                from .geometry import fit_plane
                if any(not r.extra["geometry"]["valid"] for r in results):
                    raise RequestRejected("Plane fitting requires valid correspondences")
                payload["table_plane"] = fit_plane([r.extra["geometry"]["point_xyz"] for r in results])
            return ToolOutput(extra=payload, images=[image for result in results for image in result.images])
        if fit_table:
            raise RequestRejected("Plane fitting requires a batch of correspondences")
        return _triangulate(episode, snapshot_id, observations, cameras)
    if observations is not None:
        raise RequestRejected("observations is only supported by triangulate")
    if camera not in cameras:
        raise RequestRejected("Select an available camera for segment or pixels")
    depth_enabled = cameras[camera].has_depth and cameras[camera].has_calibration
    if method == "segment":
        if points is not None or bool(prompt and prompt.strip()) == (point is not None):
            raise RequestRejected("segment requires exactly one nonempty prompt or point, and no points",
                                  code="segment_argument_combination", fault_owner="policy")
        if perception is None:
            raise RequestRejected("Segmentation requires a configured SAM service")
    elif method == "pixels":
        if not depth_enabled:
            raise RequestRejected("pixels is unavailable for this camera")
        if prompt is not None or point is not None or points is None:
            raise RequestRejected("pixels requires points only",
                                  code="pixels_argument_combination", fault_owner="policy")
    else:
        raise RequestRejected("method must be segment, pixels or triangulate")
    manifest = episode.env.snapshot(snapshot_id)
    mirror = episode.artifact_mirror
    frame, image_data = _frame_and_image(episode.env, manifest, camera, mirror=mirror, load_depth=depth_enabled)
    overlay = Image.fromarray(frame.rgb)
    common = {"snapshot_id": snapshot_id, "state_version": manifest["state_version"],
              "camera": camera, "method": method, "coordinate_order": "row_col"}
    if method == "pixels":
        geometry = project_pixels(frame, points)
        draw = ImageDraw.Draw(overlay)
        for index, (row, col) in enumerate(points, 1):
            draw.ellipse((col - 4, row - 4, col + 4, row + 4), outline="red", width=2)
            draw.text((col + 5, row), str(index), fill="red", stroke_width=1, stroke_fill="white")
        payload = {**common, "geometry": geometry, "input_pixels": points,
                   "image_size": [overlay.height, overlay.width],
                   "message": "Numbered markers follow input [row,col] order. Depth validity does not verify object identity."}
    else:
        query = TextQuery(text=prompt) if point is None else PointQuery(point=point)
        result = perception.segment(image_data, query=query, min_score=min_score)
        if result.status != "matched":
            return ToolOutput(extra={**common, "found": False, "reason": result.reason, "score": round_physical(result.score)})
        mask = np.asarray(result.mask, dtype=bool)
        if mask.shape != frame.rgb.shape[:2] or not mask.any():
            raise RequestRejected("Matched mask must be nonempty and match the captured RGB grid")
        pixels = frame.rgb.copy()
        pixels[mask, 0] = 255
        pixels[mask, 1:] = (pixels[mask, 1:] * .5).astype(np.uint8)
        overlay = Image.fromarray(pixels)
        payload = {**common, "found": True, "score": result.score, "box": result.box_xyxy}
        if depth_enabled:
            geometry = project_mask(frame, mask)
            quality = mask_geometry_quality(geometry, workspace_bounds=episode.description.environment.metadata.get("workspace_bounds"))
            payload.update(geometry=geometry, geometry_valid=quality["valid"], geometry_quality=quality)
    buffer = BytesIO()
    overlay.save(buffer, format="PNG")
    data = buffer.getvalue()
    ref = episode.artifacts.put_bytes(data, "image/png")
    payload.update({"overlay": {"artifact_id": ref["artifact_id"]}, "image_cameras": ["locate_overlay"]})
    return ToolOutput(extra=round_physical(payload), images=[ImageAttachment(data, name="locate_overlay")])


def _triangulate(episode, snapshot_id, observations, cameras):
    if not isinstance(observations, list) or not 2 <= len(observations) <= min(3, len(cameras)):
        raise RequestRejected("triangulate requires 2-3 distinct camera observations")
    names = [item["camera"] for item in observations]
    if len(set(names)) != len(names) or not set(names) <= cameras.keys():
        raise RequestRejected("triangulate requires distinct available cameras")
    if any(not cameras[name].has_calibration for name in names):
        raise RequestRejected("triangulate requires cameras with calibration enabled")
    manifest = episode.env.snapshot(snapshot_id)
    frames = [_frame_and_image(episode.env, manifest, name, mirror=episode.artifact_mirror, load_depth=False)[0]
              for name in names]
    pixels = [item["pixel"] for item in observations]
    geometry = triangulate(frames, pixels)
    images, overlays = [], {}
    for name, frame, (row, col) in zip(names, frames, pixels):
        overlay = Image.fromarray(frame.rgb)
        draw = ImageDraw.Draw(overlay)
        draw.ellipse((col - 4, row - 4, col + 4, row + 4), outline="red", width=2)
        buffer = BytesIO()
        overlay.save(buffer, format="PNG")
        data = buffer.getvalue()
        overlays[name] = {"artifact_id": episode.artifacts.put_bytes(data, "image/png")["artifact_id"]}
        images.append(ImageAttachment(data, name=f"triangulate_{name}"))
    return ToolOutput(extra=round_physical({
        "method": "triangulate", "snapshot_id": snapshot_id, "state_version": manifest["state_version"],
        "coordinate_order": "row_col", "observations": observations, "geometry": geometry,
        "overlays": overlays, "image_cameras": [image.name for image in images],
        "message": "Inspect both markers: geometric consistency does not verify that they depict the same physical point.",
    }), images=images)
