"""Camera intrinsics, pixel rays and registered RGB-D geometry with explicit validity and distortion handling."""

from __future__ import annotations

from typing import Any

import numpy as np


__all__ = ["back_project", "depth_preview", "pixel_rays"]


def _pinhole(intrinsics: dict[str, Any]) -> tuple[float, float, float, float]:
    if "fx" in intrinsics:
        values = tuple(float(intrinsics[k]) for k in ("fx", "fy", "cx", "cy"))
    else:
        k = intrinsics.get("k", [])
        if len(k) != 9:
            raise ValueError("intrinsics need fx/fy/cx/cy or a 3x3 k matrix")
        values = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if not np.isfinite(values).all() or min(values[:2]) <= 0:
        raise ValueError("invalid camera intrinsics")
    return values


def pixel_rays(pixels: np.ndarray, intrinsics: dict[str, Any]) -> np.ndarray:
    """[row, col] -> optical rays with Z=1, matching librealsense Brown models.

    RealSense inverse Brown (2) differs from OpenCV standard Brown (4).
    Unsupported models are refused instead of silently using pinhole geometry.
    """
    fx, fy, cx, cy = _pinhole(intrinsics)
    pixels = np.asarray(pixels, dtype=float)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or not np.isfinite(pixels).all():
        raise ValueError("pixels must be a finite Nx2 array of [row, col]")
    model = intrinsics.get("model", 0)
    names = {"none": 0, "inverse_brown_conrady": 2, "brown_conrady": 4, "plumb_bob": 4}
    if isinstance(model, str):
        model = names.get(model.removeprefix("distortion."), -1)
    if model not in (0, 2, 4):
        raise ValueError(f"unsupported distortion model: {model}")
    coefficients = intrinsics.get("d", intrinsics.get("coeffs", [0.0] * 5))
    if len(coefficients) != 5 or not np.isfinite(coefficients).all():
        raise ValueError("distortion requires five finite coefficients")
    k1, k2, p1, p2, k3 = map(float, coefficients)
    x0, y0 = (pixels[:, 1] - cx) / fx, (pixels[:, 0] - cy) / fy
    x, y = x0.copy(), y0.copy()
    if model:
        for _ in range(10):
            r2 = x * x + y * y
            radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
            if np.any(np.abs(radial) < 1e-8):
                raise ValueError("distortion inversion is singular")
            qx, qy = (x * radial, y * radial) if model == 2 else (x, y)
            dx = 2 * p1 * qx * qy + p2 * (r2 + 2 * qx * qx)
            dy = 2 * p2 * qx * qy + p1 * (r2 + 2 * qy * qy)
            x, y = (x0 - dx) / radial, (y0 - dy) / radial
    rays = np.column_stack((x, y, np.ones(len(x))))
    if not np.isfinite(rays).all():
        raise ValueError("non-finite optical rays")
    return rays


def back_project(
    depth_mm: np.ndarray,
    pixels: list[list[int]],
    *,
    color_intrinsics: dict[str, Any],
    depth_intrinsics: dict[str, Any] | None = None,
    window: int = 2,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Project registered depth. Holes and mixed-depth edges are not filled.

    Local spread is repeatability evidence, not an absolute accuracy bound or
    proof that the return came from the object rather than its background.
    """
    depth_mm = np.asarray(depth_mm)
    if depth_mm.ndim != 2 or not depth_mm.size or depth_mm.dtype.kind not in "uif" or not 1 <= len(pixels) <= 50:
        raise ValueError("need a 2-D depth image and 1..50 pixels")
    if isinstance(window, bool) or int(window) != window or not 0 <= window <= 10:
        raise ValueError("window must be an integer in 0..10")
    window = int(window)
    if depth_intrinsics is not None and (
        _pinhole(depth_intrinsics) != _pinhole(color_intrinsics)
        or depth_intrinsics.get("d", depth_intrinsics.get("coeffs", [])) != color_intrinsics.get("d", color_intrinsics.get("coeffs", []))
        or depth_intrinsics.get("model", 0) != color_intrinsics.get("model", 0)
    ):
        raise ValueError(
            "depth must first be registered to RGB using actual depth-to-RGB extrinsics"
        )
    height, width = depth_mm.shape
    if (
        int(color_intrinsics.get("height", height)),
        int(color_intrinsics.get("width", width)),
    ) != (height, width):
        raise ValueError("depth dimensions do not match RGB intrinsics")
    rays = pixel_rays(np.asarray(pixels), color_intrinsics)
    points = []
    for pixel, ray in zip(pixels, rays):
        if any(isinstance(v, bool) or int(v) != v for v in pixel):
            raise ValueError("pixel coordinates must be integers")
        r, c = map(int, pixel)
        point = {
            "pixel": [r, c],
            "depth_pixel": [r, c],
            "valid": False,
            "source": "sensor",
        }
        points.append(point)
        if not (0 <= r < height and 0 <= c < width):
            point["reason"] = "outside RGB image"
            continue
        center = float(depth_mm[r, c])
        if not np.isfinite(center) or center <= 0:
            point["reason"] = "no measured depth at selected pixel (hole)"
            continue
        patch = depth_mm[
            max(0, r - window) : r + window + 1, max(0, c - window) : c + window + 1
        ]
        values = patch[np.isfinite(patch) & (patch > 0)].astype(float)
        median = float(np.median(values))
        spread = float(np.percentile(values, 90) - np.percentile(values, 10))
        tolerance = max(10.0, center * 0.025)
        support = float(values.size / patch.size)
        point.update(
            raw_depth_mm=center,
            support_fraction=round(support, 3),
            spread_mm=round(spread, 2),
        )
        if support < 0.6 or spread > tolerance or abs(median - center) > tolerance:
            point["reason"] = (
                "insufficient support or depth discontinuity; select an opaque interior pixel"
            )
            continue
        z = median / 1000.0
        point.update(
            valid=True,
            depth_mm=median,
            xyz_m=np.round(ray * z, 6).tolist(),
            surface_identity_verified=False,
            absolute_accuracy_m=None,
        )
    valid = depth_mm[np.isfinite(depth_mm) & (depth_mm > 0)]
    stats = {
        "valid_fraction": round(float(valid.size / depth_mm.size), 4),
        "min_mm": float(valid.min()) if valid.size else None,
        "max_mm": float(valid.max()) if valid.size else None,
        "frame": "camera optical frame (+X right, +Y down, +Z forward)",
        "warning": "Transparent, reflective and occluded surfaces may return background depth. Local consistency does not prove object depth.",
    }
    return points, stats


def depth_preview(depth_mm: np.ndarray, points: list[dict[str, Any]]) -> np.ndarray:
    valid = depth_mm[np.isfinite(depth_mm) & (depth_mm > 0)]
    far = float(np.percentile(valid, 99)) if valid.size else 1.0
    preview = np.where(
        depth_mm > 0, 255 - np.clip(depth_mm / max(far, 1.0) * 255, 0, 255), 0
    ).astype(np.uint8)
    rgb = np.stack([preview] * 3, axis=-1)
    for point in points:
        r, c = point["depth_pixel"]
        if 0 <= r < rgb.shape[0] and 0 <= c < rgb.shape[1]:
            rgb[max(0, r - 3) : r + 4, max(0, c - 3) : c + 4] = (
                [255, 40, 40] if point["valid"] else [255, 200, 0]
            )
    return rgb
