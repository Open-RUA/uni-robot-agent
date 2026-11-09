"""RealSense D405 Body with a bounded native Linux capture subprocess.

RGB8 and Z16 are paired at the source. Native SDK projection registers depth
onto the colour grid, with target-camera Z and no hole filling. The Python
backend is available explicitly for development; it is not the Linux default.
"""

from __future__ import annotations

import base64
import io
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np

from ...core.contract import DeviceError, ErrorCode
from ...core.manifest import BodyManifest, DeviceManifest, KIND_CAMERA, ObservationStreamSpec, Signal


def validate_cameras(cameras):
    """Validate identities without importing an SDK or enumerating USB devices."""
    if not isinstance(cameras, dict) or not 1 <= len(cameras) <= 16:
        raise ValueError("an explicit camera map with 1 to 16 devices is required")
    serials, sdk_serials = set(), set()
    for name, spec in cameras.items():
        if not isinstance(name, str) or not name or not isinstance(spec, dict):
            raise ValueError("invalid camera configuration")
        for field, seen in (("serial", serials), ("sdk_serial", sdk_serials)):
            serial = str(spec.get(field, ""))
            if not serial.isascii() or not serial.isdigit() or serial in seen:
                raise ValueError(f"camera {name}: explicit unique numeric {field} required")
            seen.add(serial)
    return {name: dict(spec) for name, spec in cameras.items()}


FRAME_STALE_S = 0.5


@dataclass
class Frame:
    color_rgb: np.ndarray  # HxWx3 uint8
    depth_mm: np.ndarray  # HxW uint16, aligned to colour
    intrinsics_color: dict[str, Any]
    intrinsics_depth: dict[str, Any]
    depth_scale_m: float
    captured_mono: float
    sequence: int
    metadata: dict[str, Any] = field(default_factory=dict)


class Capture(Protocol):
    def open(self, serial: str, *, width: int, height: int, fps: int) -> Any: ...

    def latest(self, handle: Any) -> Frame | None: ...

    def close(self) -> None: ...


def distortion_name(model: Any) -> str:
    """The camera contract's name for a librealsense distortion model (enum value or ``str(enum)``)."""
    names = {0: "none", 2: "inverse_brown_conrady", 4: "brown_conrady"}
    return model.removeprefix("distortion.") if isinstance(model, str) else names.get(model, str(model))


def _intrinsics_dict(intr: Any) -> dict[str, Any]:
    return {
        "width": int(intr.width),
        "height": int(intr.height),
        "fx": float(intr.fx),
        "fy": float(intr.fy),
        "cx": float(intr.ppx),
        "cy": float(intr.ppy),
        "model": distortion_name(str(intr.model)),
        "d": [float(c) for c in intr.coeffs],
        "k": [float(intr.fx), 0.0, float(intr.ppx), 0.0, float(intr.fy), float(intr.ppy), 0.0, 0.0, 1.0],
    }


def import_pyrealsense2() -> Any:
    """Import pyrealsense2; a self-built module ships ``librealsense2.so`` next to itself, so preload it."""
    try:
        import pyrealsense2 as rs

        return rs
    except ImportError as exc:
        if "librealsense2" not in str(exc):
            raise
    import ctypes
    import importlib.util
    from pathlib import Path

    spec = importlib.util.find_spec("pyrealsense2")
    folder = Path(spec.origin).parent if spec and spec.origin else None
    for candidate in sorted(folder.glob("librealsense2.so*")) if folder else []:
        try:
            ctypes.CDLL(str(candidate), mode=ctypes.RTLD_GLOBAL)
            break
        except OSError:
            continue
    import pyrealsense2 as rs  # noqa: F811

    return rs


class Rs2Capture:
    """pyrealsense2 pipelines with a capture thread per camera."""

    def __init__(self) -> None:
        rs = import_pyrealsense2()

        self.rs = rs
        self._context = rs.context()
        self._lifecycle_lock = threading.Lock()
        self._threads: dict[str, threading.Thread] = {}
        self._latest: dict[str, Frame | None] = {}
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._errors: dict[str, str] = {}

    def open(
        self, serial: str, *, width: int = 640, height: int = 480, fps: int = 30
    ) -> str:
        rs = self.rs
        # USB/ASIC serials differ from the SDK selection serial on these D405s.
        # Share one context and serialize SDK startup; do not select by USB order.
        with self._lifecycle_lock:
            if self._stop.is_set():
                raise RuntimeError("capture is closing")
            sdk_serial = resolve_sdk_serial(rs, self._context, serial)
            pipeline = rs.pipeline(self._context)
            config = rs.config()
            config.enable_device(sdk_serial)
            config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            profile = pipeline.start(config)
        depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
        align = rs.align(rs.stream.color)
        self._latest[serial] = None

        def loop() -> None:
            sequence = 0
            while not self._stop.is_set():
                try:
                    frames = pipeline.wait_for_frames(2000)
                    stamp = time.monotonic()  # preserve delivery time through processing
                    aligned = align.process(frames)
                    color = aligned.get_color_frame()
                    depth = aligned.get_depth_frame()
                    if not color or not depth:
                        continue
                    color_np = np.asanyarray(color.get_data()).copy()
                    depth_np = np.asanyarray(depth.get_data()).astype(np.uint16)
                    depth_mm = np.clip(
                        np.rint(depth_np.astype(np.float64) * depth_scale * 1000.0),
                        0,
                        65535,
                    ).astype(np.uint16)
                    sequence += 1
                    frame = Frame(
                        color_np,
                        depth_mm,
                        _intrinsics_dict(
                            color.profile.as_video_stream_profile().get_intrinsics()
                        ),
                        _intrinsics_dict(
                            depth.profile.as_video_stream_profile().get_intrinsics()
                        ),
                        depth_scale,
                        stamp,
                        sequence,
                    )
                    with self._lock:
                        self._latest[serial] = frame
                        self._errors.pop(serial, None)
                except (
                    Exception
                ) as exc:  # keep capturing; surface the last error through health
                    with self._lock:
                        self._errors[serial] = str(exc)
                    time.sleep(0.05)
            with self._lifecycle_lock:
                pipeline.stop()

        thread = threading.Thread(target=loop, name=f"rs2-{serial}", daemon=True)
        thread.start()
        self._threads[serial] = thread
        return serial

    def latest(self, handle: str) -> Frame | None:
        with self._lock:
            return self._latest.get(handle)

    def errors(self) -> dict[str, str]:
        with self._lock:
            return dict(self._errors)

    def close(self) -> None:
        self._stop.set()
        for thread in self._threads.values():
            thread.join(timeout=3.0)


def resolve_sdk_serial(rs: Any, context: Any, configured_serial: str) -> str:
    """Accept an exact USB/ASIC or SDK identity, return its SDK selection serial."""
    matches = []
    for device in context.query_devices():
        sdk = device.get_info(rs.camera_info.serial_number)
        asic_key = rs.camera_info.asic_serial_number
        asic = device.get_info(asic_key) if device.supports(asic_key) else None
        if configured_serial in (sdk, asic):
            matches.append(sdk)
    if len(matches) != 1:
        raise RuntimeError(
            f"Camera identity {configured_serial} matched {len(matches)} devices"
        )
    return matches[0]


def encode_jpeg(rgb: np.ndarray, quality: int = 85) -> bytes:
    from PIL import Image

    buffer = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(rgb.astype(np.uint8))).save(
        buffer, format="JPEG", quality=quality
    )
    return buffer.getvalue()


class RealSenseDriver:
    data_origin = "real"

    def __init__(
        self,
        capture: Capture,
        *,
        body_id: str = "realsense_cameras",
        cameras: dict[str, dict[str, Any]] | None = None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        clock=time.monotonic,
        open_timeout_s: float = 5.0,
    ) -> None:
        self.capture = capture
        self.body_id = body_id
        self.cameras = validate_cameras(cameras)
        self._clock = clock
        self.width, self.height, self.fps = int(width), int(height), int(fps)
        self._handles: dict[str, Any] = {}
        self._open_errors: dict[str, str] = {}
        self._closing = threading.Event()
        self._lock = threading.Lock()
        # A camera in a bad USB state can block pipeline.start for a long time; open each one on
        # its own thread so the Body serves immediately and reports the stragglers as missing.
        self._openers: dict[str, threading.Thread] = {}
        for device_id, spec in self.cameras.items():
            thread = threading.Thread(
                target=self._open_camera,
                args=(device_id, str(spec["serial"])),
                name=f"rs-open-{device_id}",
                daemon=True,
            )
            self._openers[device_id] = thread
            thread.start()
        for thread in self._openers.values():
            thread.join(timeout=open_timeout_s)

    def _open_camera(self, device_id: str, serial: str) -> None:
        backoff_s = 2.0
        while not self._closing.is_set():
            try:
                handle = self.capture.open(
                    serial, width=self.width, height=self.height, fps=self.fps
                )
            except Exception as exc:
                with self._lock:
                    self._open_errors[device_id] = f"{type(exc).__name__}: {exc}"
                self._closing.wait(backoff_s)
                backoff_s = min(30.0, backoff_s * 2)
                continue
            with self._lock:
                self._handles[device_id] = handle
                self._open_errors.pop(device_id, None)
            return

    def _frame(self, device_id: str) -> Frame | None:
        with self._lock:
            handle = self._handles.get(device_id)
        return None if handle is None else self.capture.latest(handle)

    def _open_error(self, device_id: str) -> str | None:
        with self._lock:
            if device_id in self._open_errors:
                return self._open_errors[device_id]
            if device_id not in self._handles:
                return "camera still opening (USB device not answering)"
        return None

    def manifest(self) -> BodyManifest:
        devices = []
        for device_id, spec in self.cameras.items():
            frame = self._frame(device_id)
            height, width = (
                (frame.color_rgb.shape[0], frame.color_rgb.shape[1])
                if frame is not None
                else (self.height, self.width)
            )
            devices.append(
                DeviceManifest(
                    device_id=device_id,
                    kind=KIND_CAMERA,
                    hardware_profile="realsense-d405",
                    components=["rgb", "depth"],
                    frames=[f"{device_id}/color_optical"],
                    signals={
                        "rgb": Signal(
                            "rgb", "jpeg", (height, width, 3), "colour frame"
                        ),
                        "depth": Signal(
                            "depth",
                            "mm",
                            (height, width),
                            "16-bit depth aligned to the colour grid",
                        ),
                        "intrinsics": Signal(
                            "intrinsics",
                            "pixels",
                            (),
                            "colour intrinsics (also valid for aligned depth) and native depth intrinsics",
                        ),
                        "depth_png": Signal(
                            "depth_png", "mm", (height, width),
                            "lossless PNG encoding of the same registered uint16 depth",
                        ),
                        "calibration": Signal(
                            "calibration",
                            "",
                            (),
                            "factory depth-to-RGB calibration; not camera-to-arm calibration",
                        ),
                        "frame": Signal(
                            "frame",
                            "",
                            (),
                            "source frame pairing and producer identity",
                        ),
                    },
                    metadata={
                        "identity": {"model": "realsense-d405", "serial": spec["serial"]},
                        "role_hint": spec.get("role_hint", ""),
                        "open_error": self._open_error(device_id),
                    },
                    observation_streams={"rgbd": ObservationStreamSpec(
                        ["rgb", "depth", "depth_png", "intrinsics", "calibration", "frame"],
                        max_samples=60, max_bytes=64 * 1024**2,
                        source_semantics="sdk_frame_timestamp_not_verified_exposure_midpoint",
                    )} if callable(getattr(self.capture, "read_observation_at", None)) else {},
                    limitations=[
                        "no extrinsics: coordinates are in the camera optical frame"
                    ],
                )
            )
        return BodyManifest(
            self.body_id,
            "realsense_d405",
            devices,
            description="RealSense D405 cameras via librealsense",
        )

    def health(self) -> dict[str, Any]:
        now = self._clock()
        ages = {}
        for device_id in self.cameras:
            frame = self._frame(device_id)
            ages[device_id] = (
                None if frame is None else round(now - frame.captured_mono, 3)
            )
        errors = {d: e for d in self.cameras if (e := self._open_error(d))}
        timing_health = {}
        for device_id in self.cameras:
            frame = self._frame(device_id)
            timing = frame.metadata.get("timing", {}) if frame else {}
            error = timing.get("source_time_error")
            timing_health[device_id] = {"error": error, "source_times": timing.get("source_times"),
                                        "rejected_frames": timing.get("clock_rejected_frames", 0)}
            diagnostic_reader = getattr(self.capture, "diagnostics", None)
            with self._lock:
                handle = self._handles.get(device_id)
            if callable(diagnostic_reader) and handle is not None:
                timing_health[device_id]["diagnostics"] = diagnostic_reader(handle)
            if error:
                errors[device_id] = error
        if hasattr(self.capture, "errors"):
            errors.update(self.capture.errors())
        return {
            "ok": all(a is not None and a < FRAME_STALE_S for a in ages.values())
            and not errors,
            "frames_age_s": ages,
            "timing": timing_health,
            "errors": errors,
        }

    def read(self, device_id: str, include: list[str] | None = None) -> dict[str, Any]:
        if device_id not in self.cameras:
            raise DeviceError(ErrorCode.UNKNOWN_DEVICE, f"unknown camera {device_id!r}")
        frame = self._frame(device_id)
        if frame is None:
            return {
                "signals": {},
                "availability": "missing",
                "freshness": "unknown",
                "age_s": None,
                "error": {
                    "code": ErrorCode.HARDWARE_ERROR,
                    "message": self._open_error(device_id) or "no frame captured yet",
                },
            }
        return self._read_frame(device_id, frame, include)

    def read_observation_at(self, device_id, stream, at, *, mode="bracket", max_gap_s=0.05, include=None):
        reader = getattr(self.capture, "read_observation_at", None)
        if stream != "rgbd" or not callable(reader):
            raise DeviceError(ErrorCode.NOT_SUPPORTED, "camera backend has no source-stamped history")
        with self._lock:
            handle = self._handles.get(device_id)
        if handle is None:
            raise DeviceError(ErrorCode.PRECONDITION_FAILED, "camera capture is not open")
        # Query the retained valid candidates first. A bad/warming latest frame
        # does not veto history; missing future data returns pending for bounded wait.
        result = reader(handle, at, mode=mode, max_gap_s=max_gap_s)
        for sample in result["samples"]:
            frame = sample.pop("payload")
            # Metadata by default. Encode only explicitly requested historical pixels.
            reading = self._read_frame(device_id, frame, ["frame", "intrinsics"] if include is None else include)
            sample["signals"] = reading["signals"]
        return result

    def _read_frame(self, device_id, frame, include):
        wanted = set(["rgb", "intrinsics"] if include is None else include)
        signals: dict[str, Any] = {}
        if "rgb" in wanted:
            signals["rgb"] = {
                "format": "jpeg",
                "data_b64": base64.b64encode(encode_jpeg(frame.color_rgb)).decode(
                    "ascii"
                ),
                "width": int(frame.color_rgb.shape[1]),
                "height": int(frame.color_rgb.shape[0]),
                "sequence": frame.sequence,
            }
        for name in wanted & {"depth", "depth_png"}:
            if name == "depth_png":
                from PIL import Image

                buffer = io.BytesIO()
                Image.fromarray(frame.depth_mm).save(buffer, format="PNG", compress_level=3)
                encoded = buffer.getvalue()
            else:
                encoded = frame.depth_mm.astype("<u2").tobytes()
            signals[name] = {
                "format": "png16_b64" if name == "depth_png" else "u16le_b64",
                "unit": "mm",
                "data_b64": base64.b64encode(encoded).decode("ascii"),
                "width": int(frame.depth_mm.shape[1]),
                "height": int(frame.depth_mm.shape[0]),
                "frame_id": f"{device_id}/color_optical",
                "aligned_to": "color",
                "depth_scale_m": frame.depth_scale_m,
                "sequence": frame.sequence,
            }
        if "intrinsics" in wanted:
            signals["intrinsics"] = {
                "color": frame.intrinsics_color,
                "depth": frame.intrinsics_color,
                "depth_native": frame.intrinsics_depth,
            }
        if "calibration" in wanted:
            signals["calibration"] = frame.metadata.get("calibration", {})
        if "frame" in wanted:
            signals["frame"] = {
                k: v for k, v in frame.metadata.items() if k != "calibration"
            }
        age = self._clock() - frame.captured_mono
        timing = dict(frame.metadata.get("timing", {}))
        if timing:
            timing["body_read_monotonic_raw_ns"] = str(
                time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
            )
            timing["body_read_realtime_ns"] = str(time.time_ns())
        return {
            "signals": signals,
            "freshness": "stale" if age > FRAME_STALE_S else "fresh",
            "age_s": round(age, 4),
            "timing": timing,
        }

    def check(self, device_id: str) -> dict[str, Any]:
        frame = self._frame(device_id)
        fresh = (
            frame is not None and self._clock() - frame.captured_mono <= FRAME_STALE_S
        )
        timing_error = frame.metadata.get("timing", {}).get("source_time_error") if frame else "no frame"
        return {
            "readiness": "satisfied" if fresh and not timing_error else "unsatisfied",
            "checks": {
                "source_time": {"state": "fail" if timing_error else "pass", "reason": timing_error or ""},
                "frames": {
                    "state": "pass" if fresh else "fail",
                    "reason": ""
                    if fresh
                    else (self._open_error(device_id) or "no recent frame"),
                }
            },
        }

    def write(
        self,
        device_id: str,
        command: str,
        value: list[float],
        *,
        cancel: threading.Event,
    ) -> dict[str, Any]:
        raise DeviceError(ErrorCode.NOT_SUPPORTED, "cameras have no commands")

    def invoke(
        self,
        device_ids: list[str],
        procedure: str,
        params: dict[str, Any],
        *,
        cancel: threading.Event,
        progress,
    ) -> dict[str, Any]:
        raise DeviceError(ErrorCode.NOT_SUPPORTED, "cameras have no procedures")

    def stream_apply(self, device_id: str, targets: dict[str, list[float]]) -> None:
        raise DeviceError(ErrorCode.NOT_SUPPORTED, "cameras have no streams")

    def stream_hold(self, device_id: str) -> None:
        return None

    def estop(self, device_id: str) -> dict[str, Any]:
        return {"held": False, "note": "camera"}

    def close(self) -> None:
        self._closing.set()
        self.capture.close()


def driver(
    *,
    body_id: str = "realsense_cameras",
    fps: int = 30,
    cameras: dict[str, dict[str, Any]] | None = None,
    cameras_file: str | None = None,
    backend: str = "native",
) -> RealSenseDriver:
    """``cameras`` maps device ids to ``{"serial": ..., "role_hint": ...}``; ``cameras_file`` is a JSON file with the same mapping."""
    if cameras is not None and cameras_file is not None:
        raise ValueError("choose cameras or cameras_file, not both")
    if cameras is None and cameras_file is not None:
        import json
        from pathlib import Path

        path = Path(cameras_file).expanduser()
        cameras = json.loads(path.read_text())
    cameras = validate_cameras(cameras)
    if backend == "native":
        from .native_camera import NativeCapture

        if fps != 30:
            raise ValueError("Native D405 profile is fixed at 30 FPS")
        capture = NativeCapture(
            [str(v["serial"]) for v in cameras.values()],
            cameras=cameras,
        )
    elif backend == "pyrealsense2":
        capture = Rs2Capture()
    else:
        raise ValueError(f"Unknown camera backend: {backend}")
    try:
        return RealSenseDriver(capture, body_id=body_id, fps=fps, cameras=cameras)
    except BaseException:
        capture.close()
        raise


__all__ = [
    "validate_cameras",
    "Capture",
    "Frame",
    "RealSenseDriver",
    "Rs2Capture",
    "driver",
    "encode_jpeg",
    "import_pyrealsense2",
]
