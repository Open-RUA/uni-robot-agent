"""Optional single-frame depth filtering for inspection and model input; camera measurements stay raw."""
from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class DepthFilterConfig:
    method: str = "edge_preserving_inverse_depth_v1"
    min_depth_mm: float = 70.0
    max_depth_mm: float = 2000.0
    spatial_delta_mm: float = 4.0
    spatial_delta_relative: float = .02
    spatial_min_neighbors: int = 3
    spatial_rgb_delta: float = 30.0

    def __post_init__(self):
        if self.method != "edge_preserving_inverse_depth_v1":
            raise ValueError("unsupported depth filter method")
        for name, (lo, hi) in {
            "min_depth_mm": (1, 65534), "max_depth_mm": (2, 65535),
            "spatial_delta_mm": (.1, 100), "spatial_delta_relative": (0, .1),
            "spatial_min_neighbors": (1, 8), "spatial_rgb_delta": (1, 255),
        }.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f"{name} must be finite in [{lo}, {hi}]")
        if type(self.spatial_min_neighbors) is not int:
            raise ValueError("spatial_min_neighbors must be an integer")
        if self.min_depth_mm >= self.max_depth_mm:
            raise ValueError("min_depth_mm must be below max_depth_mm")


def filter_depth(depth_mm, rgb, config=None):
    """3x3 depth/RGB-gated inverse-depth average, without temporal history.

    Depth agreement counts support; RGB edges restrict averaging. Unsupported
    measurements are rejected, and an input hole always remains a hole.
    """
    cfg = DepthFilterConfig(**(config or {}))
    started = time.perf_counter()
    depth, rgb = np.asarray(depth_mm), np.asarray(rgb)
    if depth.ndim != 2 or not depth.size or depth.dtype.kind not in "uif" or rgb.shape != (*depth.shape, 3) or rgb.dtype != np.uint8:
        raise ValueError("need numeric HxW depth in mm and matching uint8 RGB")
    measured = np.isfinite(depth) & (depth > 0)
    valid = measured & (depth >= cfg.min_depth_mm) & (depth <= cfg.max_depth_mm)
    current = np.where(valid, depth, 0).astype(np.float32)
    gray = rgb.astype(np.float32).mean(axis=2)
    tolerance = np.maximum(cfg.spatial_delta_mm, current * cfg.spatial_delta_relative)
    inverse = np.divide(1., current, out=np.zeros_like(current), where=valid)
    inverse_sum, weights = inverse * 2, valid.astype(np.float32) * 2
    neighbors = np.zeros(depth.shape, np.uint8)
    h, w = depth.shape
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == dy == 0:
                continue
            ys, xs = slice(max(0, -dy), min(h, h - dy)), slice(max(0, -dx), min(w, w - dx))
            yn, xn = slice(max(0, dy), min(h, h + dy)), slice(max(0, dx), min(w, w + dx))
            agree = valid[ys, xs] & valid[yn, xn]
            agree &= np.abs(current[ys, xs] - current[yn, xn]) <= tolerance[ys, xs]
            neighbors[ys, xs] += agree
            color = np.abs(gray[ys, xs] - gray[yn, xn]) <= cfg.spatial_rgb_delta
            weight = (agree & color).astype(np.float32) * (.7071068 if dx and dy else 1.)
            weights[ys, xs] += weight
            inverse_sum[ys, xs] += weight * inverse[yn, xn]
    accepted = valid & (neighbors >= cfg.spatial_min_neighbors)
    filtered = np.divide(weights, inverse_sum, out=np.zeros_like(current), where=accepted)
    output = np.rint(filtered).astype(np.uint16)
    return output, {
        "method": cfg.method, "config": asdict(cfg), "temporal_enabled": False,
        "raw_valid_fraction": float(measured.mean()), "accepted_fraction": float(accepted.mean()),
        "rejected_pixels": int((measured & ~accepted).sum()), "holes_filled": 0,
        "processing_ms": (time.perf_counter() - started) * 1000,
        "accuracy": "consistency filter; absolute accuracy not established",
    }
