"""Offline region geometry and depth processing for explicit immutable observations; no device or model I/O."""

from __future__ import annotations

import io
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

from ...perception.extrinsics import calibrated_points, within_calibration_volume
from ...perception.depth_filter import filter_depth
from ...perception.geometry import back_project, pixel_rays
from ...perception.observation_store import ObservationStore


def region_depth(depth_mm, region=None, *, mask=None, intrinsics, border_px=2, max_points=100000):
    """Keep every accepted pixel in a caller-selected RGB region.

    A box is [top, left, bottom, right], with exclusive bottom/right bounds;
    polygon vertices are [row, col]. A region is not automatic segmentation.
    """
    depth_mm = np.asarray(depth_mm)
    if type(max_points) is not int or not 1 <= max_points <= 100000:
        raise ValueError("max_points must be an integer in 1..100000")
    if depth_mm.ndim != 2 or not depth_mm.size or depth_mm.dtype.kind not in "uif":
        raise ValueError("region depth requires a 2-D registered depth image")
    h, w = depth_mm.shape
    if (intrinsics.get("height", h), intrinsics.get("width", w)) != (h, w):
        raise ValueError("depth dimensions do not match RGB intrinsics")
    if isinstance(border_px, bool) or int(border_px) != border_px or not 0 <= border_px <= 10:
        raise ValueError("border_px must be an integer in 0..10")
    border_px = int(border_px)
    if mask is not None:
        mask = np.asarray(mask)
        if region is not None or mask.shape != (h,w) or not np.isin(mask,[0,1]).all():
            raise ValueError("mask must be binary and match the original image")
        mask_image = Image.fromarray(mask.astype(np.uint8)*255)
    else:
        if not isinstance(region, dict) or set(region) not in ({"box"}, {"polygon"}):
            raise ValueError("region needs exactly one box or polygon")
        values = np.asarray(next(iter(region.values())))
        if values.dtype.kind not in "iu" or values.dtype.kind == "b":
            raise ValueError("region coordinates must be integers")
        mask_image = Image.new("L", (w, h))
        draw = ImageDraw.Draw(mask_image)
        if "box" in region:
            if values.shape != (4,):
                raise ValueError("box must be [top, left, bottom, right]")
            top, left, bottom, right = map(int, values)
            if not (0 <= top < bottom <= h and 0 <= left < right <= w):
                raise ValueError("box must be nonempty and inside the RGB image")
            draw.rectangle((left, top, right - 1, bottom - 1), fill=255)
        else:
            if values.ndim != 2 or values.shape[1] != 2 or not 3 <= len(values) <= 128:
                raise ValueError("polygon needs 3..128 [row, col] vertices")
            if np.any(values < 0) or np.any(values >= [h, w]):
                raise ValueError("polygon must be inside the RGB image")
            if np.linalg.matrix_rank(values[1:] - values[0]) < 2:
                raise ValueError("polygon must enclose an area")
            draw.polygon([(int(c), int(r)) for r, c in values], fill=255)
    selected = np.asarray(mask_image) > 0
    interior = selected
    if border_px:
        padded = Image.fromarray(np.pad(np.asarray(mask_image), border_px))
        interior = np.asarray(padded.filter(ImageFilter.MinFilter(2 * border_px + 1)))[
            border_px:-border_px, border_px:-border_px
        ] > 0
    if not interior.any():
        raise ValueError("region has no interior after border exclusion")
    accepted = interior & np.isfinite(depth_mm) & (depth_mm > 0)

    if int(accepted.sum()) > max_points:
        raise ValueError("region exceeds max_points; select a smaller region")
    pixels = np.argwhere(accepted)
    depths = depth_mm[accepted].astype(float)
    xyz = pixel_rays(pixels, intrinsics) * depths[:, None] / 1000
    points = [
        {"pixel": pixel.tolist(), "valid": True, "xyz_m": position.tolist()}
        for pixel, position in zip(pixels, xyz)
    ]
    percentiles = np.percentile(depths, [10, 50, 90]).tolist() if depths.size else None
    summary = {
        "selection": region,
        "selection_semantics": "archived_sam_mask" if mask is not None else "caller_supplied_region_not_automatic_object_segmentation",
        "border_px": border_px,
        "selected_pixels": int(selected.sum()),
        "interior_pixels": int(interior.sum()),
        "accepted_pixels": len(points),
        "accepted_fraction": float(accepted.sum() / interior.sum()),
        "depth_p10_p50_p90_mm": percentiles,
        "depth_min_max_mm": [float(depths.min()), float(depths.max())] if depths.size else None,
        "source": "sensor",
        "depth_semantics": "RGB camera optical Z; each cloud point keeps its own depth",
        "surface_identity_verified": False,
        "absolute_accuracy_m": None,
        "warning": "A region may contain several surfaces or background through a transparent object. Percentiles are descriptive, not a grasp target; holes are not filled.",
    }
    return points, summary, selected, accepted


def region_overlay(rgb, depth_mm, selected, accepted):
    """Overlay registered measurements at their original RGB pixels."""
    if rgb.shape != (*depth_mm.shape, 3):
        raise ValueError("overlay RGB and registered depth dimensions differ")
    overlay = rgb.copy()
    values = depth_mm[accepted].astype(float)
    scale = np.percentile(values, [5, 95]).tolist() if values.size else [0., 0.]
    if values.size:
        t = np.clip((values - scale[0]) / max(1., scale[1] - scale[0]), 0., 1.)
        colors = np.column_stack((255 * t, 180 * (1 - np.abs(2 * t - 1)), 255 * (1 - t)))
        overlay[accepted] = (0.4 * rgb[accepted] + 0.6 * colors).astype(np.uint8)
    eroded = np.asarray(Image.fromarray(selected.astype(np.uint8) * 255).filter(ImageFilter.MinFilter(3))) > 0
    outline = selected & ~eroded
    overlay[outline] = [255, 230, 0]
    rejected = selected & ~accepted
    rows, cols = np.indices(selected.shape)
    overlay[rejected & ((rows + cols) % 7 == 0)] = [180, 180, 180]
    image = Image.fromarray(overlay)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, image.width - 1, 42), fill=(15, 15, 15))
    draw.text((6, 3), f"REGION DEPTH: {int(accepted.sum())}/{int(selected.sum())} pixels; gray = excluded", fill="white")
    draw.text((6, 22), f"Optical Z: blue {scale[0]:.0f} mm -> red {scale[1]:.0f} mm (p5-p95)", fill="white")
    return np.asarray(image)


class DepthService:
    """Read only pinned RGB-D. Derived results are evidence, never motion approval."""

    def __init__(self, store: ObservationStore):
        self.store = store

    def filter_depth(self, observe_id: str, camera: str, *, config=None) -> dict:
        snapshot = self.store.snapshot(observe_id, camera)
        with Image.open(io.BytesIO(snapshot.jpeg)) as image:
            rgb = np.asarray(image.convert("RGB"))
        filtered, report = filter_depth(snapshot.depth_mm, rgb, config)
        return self.store.put_derived(
            observe_id, camera, kind="filtered_measured_depth",
            arrays={"depth_mm": filtered},
            metadata={"unit": "mm", "aligned_to": "color", "report": report,
                      "depth_source": "filtered", "geometry_current": False,
                      "motion_ready": False},
        )

    def back_project(self, observe_id: str, camera: str, *, pixels=None, region=None,
                     window=2, border_px=2, max_points=100000,
                     depth_source="filled", depth_artifact=None,
                     calibration=None, segmentation_artifact=None) -> dict:
        """Default to filled depth from an explicit same-observation artifact.

        Measured depth must be selected explicitly; no implicit fallback occurs.

        `calibration` selects a binding frozen in this observation. Static camera
        transforms can be inspected after reopening an archive, but are never a
        statement about the current scene, TCP accuracy, collision or reachability.
        """
        started = time.perf_counter()
        if sum(x is not None for x in (pixels, region, segmentation_artifact)) != 1:
            raise ValueError("provide exactly one of pixels, region or segmentation_artifact")
        mask = None
        if segmentation_artifact is not None:
            segmentation, arrays = self.store.derived(observe_id,camera,segmentation_artifact)
            if segmentation["kind"] != "segmentation" or not segmentation["metadata"].get("found"):
                raise ValueError("segmentation artifact has no matched mask")
            mask = arrays.get("mask")
            if mask is None: raise ValueError("segmentation artifact has no mask")
        if depth_source not in {"measured", "filtered", "model", "filled"}:
            raise ValueError("depth_source must be measured, filtered, model or filled")
        if (depth_source != "measured") != (depth_artifact is not None):
            raise ValueError("every derived depth source requires an explicit depth_artifact")
        snapshot = self.store.snapshot(observe_id, camera)
        depth = snapshot.depth_mm
        if depth_source == "filtered":
            artifact, arrays = self.store.derived(observe_id, camera, depth_artifact)
            if artifact["kind"] != "filtered_measured_depth" or artifact["metadata"].get("unit") != "mm" or artifact["metadata"].get("aligned_to") != "color":
                raise ValueError("not a registered filtered depth artifact")
            depth = arrays.get("depth_mm")
            if depth is None or depth.shape != snapshot.depth_mm.shape or depth.dtype != np.uint16:
                raise ValueError("filtered depth has incompatible shape or type")
            if np.any((snapshot.depth_mm == 0) & (depth != 0)):
                raise ValueError("filtered depth must preserve original holes")
        if depth_source in {"model", "filled"}:
            artifact, arrays = self.store.derived(observe_id, camera, depth_artifact)
            if artifact["kind"] != "depth_completion" or artifact["metadata"].get("unit") != "m":
                raise ValueError("not a registered metric completion artifact")
            values = arrays.get(depth_source + "_m")
            if values is None or values.shape != depth.shape or not np.isfinite(values).all() or np.any(values < 0):
                raise ValueError("invalid completed depth")
            if depth_source == "filled" and not np.array_equal(values[depth > 0], depth[depth > 0].astype(float) / 1000):
                raise ValueError("filled depth must preserve valid measured pixels")
            depth = values * 1000.
        intrinsics = snapshot.intrinsics
        if pixels is not None:
            points, stats = back_project(depth, pixels, color_intrinsics=intrinsics["color"],
                                         depth_intrinsics=intrinsics.get("depth"), window=window)
        else:
            points, stats, _, _ = region_depth(depth, region, mask=mask, intrinsics=intrinsics["color"],
                                               border_px=border_px, max_points=max_points)
        for point in points:
            point["source"] = depth_source
            if depth_source == "filled":
                r, c = point["pixel"]
                if 0 <= r < depth.shape[0] and 0 <= c < depth.shape[1]:
                    patch = snapshot.depth_mm[max(0,r-window):r+window+1,max(0,c-window):c+window+1] if pixels is not None else snapshot.depth_mm[r:r+1,c:c+1]
                    point["source"] = "measured" if np.all(patch > 0) else "model" if not np.any(patch > 0) else "mixed"
                    point["center_source"] = "measured" if snapshot.depth_mm[r,c] > 0 else "model" if depth[r,c] > 0 else "invalid"
        meta = snapshot.metadata
        conversion = None
        if calibration is not None:
            if meta.get("frame_motion") != "fixed":
                raise ValueError("static calibration requires a fixed camera")
            frozen = meta.get("calibrations", {}).get(calibration)
            if not frozen or frozen.get("observed_identity_matches") is not True:
                raise ValueError("no matching frozen calibration for this observation")
            if frozen["camera_device"] != meta.get("camera_device"):
                raise ValueError("frozen calibration camera identity differs")
            record = frozen["record"]
            valid = [p for p in points if p["valid"]]
            xyz = np.asarray([p["xyz_m"] for p in valid], dtype=float).reshape(-1, 3)
            transformed = calibrated_points(xyz, record, serial=meta["serial"],
                intrinsics=intrinsics["color"], source_frame=meta["frame_id"],
                target_frame=record["target_frame"], mount_id=frozen["mount_id"])
            coverage = within_calibration_volume(xyz, record)
            for point, target, covered in zip(valid, transformed, coverage):
                point.update(target_xyz_m=target.tolist(), target_frame=record["target_frame"],
                    within_validation_volume=bool(covered),
                    calibration_validation_max_m=record["validation_max_m"] if covered else None)
            conversion = {"binding": calibration, "sha256": frozen["sha256"],
                          "target_device": frozen["target_device"], "target_frame": record["target_frame"],
                          "physical_mount_verified": frozen["physical_mount_verified"],
                          "mount_evidence": frozen["mount_evidence"]}
        stats["source"] = depth_source
        raw = self.store.get(observe_id)["raw"]["cameras"][camera]
        return {"observe_id": observe_id, "camera": camera, "raw_sha256": raw["sha256"],
                "depth_source": depth_source, "depth_artifact": depth_artifact, "segmentation_artifact": segmentation_artifact,
                "frame_id": meta.get("frame_id"), "t_request": meta.get("t_request"),
                "source_times": meta.get("source_times"), "joint_states": meta.get("joint_states"),
                "calibration": conversion, "points": points, "statistics": stats,
                "geometry_current": False, "motion_ready": False,
                "processing_s": time.perf_counter() - started}
