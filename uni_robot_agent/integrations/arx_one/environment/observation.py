"""Source-stamped RGB-D packets (body/core/camera/contract.py) and image-time wrist geometry."""
import base64
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
import io
import threading
import time

import numpy as np
from PIL import Image

from uni_robot_agent.services.environment.snapshots import CameraFrame
from ..body.core.timing import host_clock_id, timestamp_ns
from ..perception.synchronization import joint_state_at_time


# Process-local geometry only: never cache frames, timestamps or observations.
_MAP_CACHE = OrderedDict()
_MAP_CACHE_LOCK = threading.Lock()
_MAP_CACHE_MAX_ENTRIES = 8
_MAP_CACHE_MAX_BYTES = 32 * 1024 * 1024


def rectify(rgb, depth, intrinsics):
    import cv2

    k = np.array([[intrinsics["fx"], 0, intrinsics["cx"]],
                  [0, intrinsics["fy"], intrinsics["cy"]], [0, 0, 1]], dtype=float)
    coeffs = np.asarray(intrinsics.get("d", [0.] * 5), dtype=float)
    model = intrinsics.get("model")  # One of camera contract DISTORTION_MODELS; drivers translate SDK enums.
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0 or coeffs.shape != (5,) or not np.isfinite(coeffs).all():
        raise ValueError("Invalid camera intrinsics")
    if model == "none":
        if np.any(coeffs):
            raise ValueError("Pinhole model has nonzero distortion")
        return rgb, depth, k
    h, w = depth.shape
    mx, my = _rectification_maps(h, w, k, coeffs, model)
    return (cv2.remap(rgb, mx, my, cv2.INTER_LINEAR),
            cv2.remap(depth, mx, my, cv2.INTER_NEAREST), k)


def _rectification_maps(h, w, k, coeffs, model):
    # All inputs to map construction are in the key. A changed calibration or
    # resolution cannot reuse incompatible geometry; identical geometry may be
    # shared between cameras without sharing any camera state.
    key = (h, w, model, *k.ravel(), *coeffs)
    with _MAP_CACHE_LOCK:
        if key in _MAP_CACHE:
            _MAP_CACHE.move_to_end(key)
            return _MAP_CACHE[key]
    # Keep independent cameras' first-time builds parallel. Concurrent misses
    # for the same geometry may build twice, but only one immutable pair is kept.
    maps = _build_rectification_maps(h, w, k, coeffs, model)
    for array in maps:
        array.setflags(write=False)
    size = sum(array.nbytes for array in maps)
    with _MAP_CACHE_LOCK:
        if key in _MAP_CACHE:
            _MAP_CACHE.move_to_end(key)
            return _MAP_CACHE[key]
        if size <= _MAP_CACHE_MAX_BYTES:
            while _MAP_CACHE and (
                len(_MAP_CACHE) >= _MAP_CACHE_MAX_ENTRIES
                or sum(a.nbytes for pair in _MAP_CACHE.values() for a in pair) + size > _MAP_CACHE_MAX_BYTES
            ):
                _MAP_CACHE.popitem(last=False)
            _MAP_CACHE[key] = maps
    return maps


def _build_rectification_maps(h, w, k, coeffs, model):
    import cv2

    if model == "brown_conrady":
        mx, my = cv2.initUndistortRectifyMap(k, coeffs, None, k, (w, h), cv2.CV_32FC1)
    elif model == "inverse_brown_conrady":
        y, x = np.mgrid[:h, :w]
        nx, ny = (x - k[0, 2]) / k[0, 0], (y - k[1, 2]) / k[1, 1]
        k1, k2, p1, p2, k3 = coeffs
        r2 = nx * nx + ny * ny
        radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
        qx, qy = nx * radial, ny * radial
        # Invert the inverse-Brown deprojection fixed point (as librealsense defines it): the
        # tangential terms use radially scaled x/y.
        xd = qx + 2 * p1 * qx * qy + p2 * (r2 + 2 * qx * qx)
        yd = qy + 2 * p2 * qx * qy + p1 * (r2 + 2 * qy * qy)
        mx = (xd * k[0, 0] + k[0, 2]).astype(np.float32)
        my = (yd * k[1, 1] + k[1, 2]).astype(np.float32)
    else:
        raise ValueError(f"Unsupported distortion model {model!r}")
    return mx, my


def decode_packet(sample, at, *, max_age_s, window_s=.05):
    """Decode one RGB-D packet taken within ``window_s`` after the request time ``at``."""
    signals = sample["signals"]
    rgb_packet, depth_packet = signals["rgb"], signals.get("depth_png", signals.get("depth"))
    h, w = rgb_packet["height"], rgb_packet["width"]
    if type(h) is not int or type(w) is not int or min(h, w) <= 0 or h * w > 16_777_216:
        raise ValueError("Invalid image dimensions")
    if (depth_packet["height"], depth_packet["width"]) != (h, w) or depth_packet.get("aligned_to") != "color" or depth_packet.get("unit") != "mm":
        raise ValueError("Depth must be registered to color in millimetres")
    sequence = rgb_packet["sequence"]
    if type(sequence) is not int or sequence < 0 or sequence != depth_packet["sequence"] or sequence != sample["sequence"]:
        raise ValueError("RGB-D history sequence mismatch")
    with Image.open(io.BytesIO(base64.b64decode(rgb_packet["data_b64"], validate=True))) as image:
        if image.format != "JPEG" or image.size != (w, h):
            raise ValueError("Expected matching JPEG color image")
        rgb = np.asarray(image.convert("RGB")).copy()
    raw = base64.b64decode(depth_packet["data_b64"], validate=True)
    if depth_packet["format"] == "u16le_b64":
        depth = np.frombuffer(raw, dtype="<u2").reshape(h, w).copy()
    elif depth_packet["format"] == "png16_b64":
        with Image.open(io.BytesIO(raw)) as image:
            if image.format != "PNG" or image.size != (w, h) or image.mode not in {"I", "I;16", "I;16B", "I;16L"}:
                raise ValueError("Expected matching uint16 PNG depth")
            depth = np.array(image)
        if np.any(depth < 0) or np.any(depth > 65535):
            raise ValueError("Depth outside uint16 range")
    else:
        raise ValueError("Unsupported depth encoding")
    timing = sample["timing"]
    if timing.get("source_time_error"):
        raise ValueError("Body camera clock mapping is invalid")
    times = timing.get("source_times") or {}
    if not (times.get("pairing") or {}).get("valid") or not all(isinstance(times.get(s), dict) for s in ("rgb", "depth")):
        raise ValueError("Camera reported no paired RGB-D source times")
    stamps = [timestamp_ns(times[s]["source_timestamp_ns"]) for s in ("rgb", "depth")]
    now, target = time.time_ns(), timestamp_ns(at["source_timestamp_ns"])
    if (any(clock != at["clock_id"] for clock in (timing.get("clock_id"), times["rgb"].get("clock_id"),
                                                    times["depth"].get("clock_id")))
            or min(stamps) < target or stamps[0] - target > window_s * 1e9
            or max(stamps) > now + 1_000_000 or now - min(stamps) > max_age_s * 1e9
            or timestamp_ns(timing["source_timestamp_ns"]) != stamps[0]):
        raise ValueError("RGB-D source times do not match the request window")
    color = signals["intrinsics"]["color"]
    if (color["height"], color["width"]) != (h, w):
        raise ValueError("Intrinsics resolution mismatch")
    rgb, depth, k = rectify(rgb, depth.astype(np.float32) / 1000., color)
    return rgb, depth, k, {"sequence": sequence, "source_times": times,
        "request": at, "hardware_synchronized": False, "rectified": True}


def capture(adapter, names):
    clock = host_clock_id()
    if not clock:
        raise ValueError("Run the environment service on the same Linux host as Body")
    at = {"source_timestamp_ns": str(time.time_ns()), "clock_id": clock, "source_clock": "unix_realtime"}
    frames, provenance = {}, {}
    names = list(names)

    def read_camera(name):
        config = adapter.config.cameras[name]
        started = time.monotonic()
        # The first frame after the request arrives within one frame period, plus jitter.
        window = 1.5 * config.frame_period_s
        samples = adapter.camera_client.observation(config.device_id, "rgbd", at, mode="after",
            include=adapter.camera_signals[name], max_gap_s=window)
        received = time.monotonic()
        if len(samples) != 1:
            raise ValueError("Expected one post-request RGB-D sample")
        rgb, depth, k, metadata = decode_packet(samples[0], at, max_age_s=adapter.config.max_age_s, window_s=window)
        metadata["capture_timing"] = {
            "query_s": received - started,
            "decode_rectify_s": time.monotonic() - received,
        }
        return rgb, depth, k, metadata

    # Share one fixed target, not one timestamp per worker. Finish all readers
    # before propagating failure so none outlive capture/adapter shutdown.
    # Keep geometry below serial: a kinematics instance may be stateful.
    if len(names) > 1:
        with ThreadPoolExecutor(max_workers=len(names), thread_name_prefix="camera-read") as pool:
            packets = list(pool.map(read_camera, names))
    else:
        packets = [read_camera(name) for name in names]
    for name, (rgb, depth, k, metadata) in zip(names, packets):
        config = adapter.config.cameras[name]
        extrinsic = None
        if config.T_parent_from_optical is not None:
            extrinsic = np.asarray(config.T_parent_from_optical)
            if config.parent != "world":
                arm = adapter.config.arms[config.parent]
                image_time = metadata["source_times"]["rgb"]
                query = {key: image_time[key] for key in ("source_timestamp_ns", "clock_id", "source_clock")}
                history = adapter.client.observation(arm.device_id, "state", query)
                matched = joint_state_at_time([{**s["signals"], "timing": s["timing"]} for s in history], image_time)
                extrinsic = (np.asarray(arm.T_world_from_controller)
                    @ adapter.kinematics[config.parent].controller(matched["joint_position_rad"]) @ extrinsic)
                metadata["joint_alignment"] = matched
        frames[name] = CameraFrame(rgb, depth, k if extrinsic is not None else None, extrinsic)
        provenance[name] = metadata
    # The packet must remain fresh when all cameras have been collected.
    now = time.time_ns()
    if any(now - timestamp_ns(m["source_times"]["rgb"]["source_timestamp_ns"]) > adapter.config.max_age_s * 1e9
           for m in provenance.values()):
        raise ValueError("Camera packet expired while assembling observation")
    return frames, provenance

