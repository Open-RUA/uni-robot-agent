"""Serial-bound RealSense Body with explicit RGB-D profiles and bounded history."""
from __future__ import annotations

import base64
from contextlib import ExitStack
import io
import threading
import time

import numpy as np
from PIL import Image

from uni_robot_agent.integrations.piper_x.body.core.contract import DeviceError, ErrorCode
from uni_robot_agent.integrations.piper_x.body.core.manifest import BodyManifest, DeviceManifest, Signal, ObservationStreamSpec
from uni_robot_agent.integrations.piper_x.body.core.observations import ObservationBuffer
from uni_robot_agent.integrations.piper_x.body.core.resources import ResourceLocks
from uni_robot_agent.integrations.piper_x.body.core.timing import host_clock_id
from ..config import Installation
from .camera_identity import devices_by_serial


class CameraBody:
    data_origin = "physical"

    def __init__(self, deployment, *, body_id):
        import pyrealsense2 as rs
        self.rs = rs
        self.installation = Installation(deployment)
        self.body_id = body_id
        self.robot = self.installation.robot
        if self.installation.host.bodies[self.robot.cameras_body].body_id != body_id:
            raise ValueError("Wrong camera Body identity")
        if not host_clock_id():
            raise ValueError("Camera Body needs Linux host clock identity")
        self._cleanup = ExitStack()
        self._stop = threading.Event()
        self._threads, self._pipelines, self._latest, self._errors = [], {}, {}, {}
        self._capture_failures = {}
        self._lock = threading.Lock()
        self._by_device = {c.device_id: n for n, c in self.robot.cameras.items()}
        self._histories = {d: ObservationBuffer(max_bytes=32 * 1024**2) for d in self._by_device}
        try:
            bindings = {n: self.installation.host.cameras[c.binding] for n, c in self.robot.cameras.items()}
            locks = ResourceLocks(["physical-camera:" + b.serial for b in bindings.values()], self.installation.host.lock_dir)
            self._cleanup.callback(locks.close)
            found = devices_by_serial(rs.context(), rs)
            for name, binding in bindings.items():
                if binding.serial not in found:
                    raise ValueError(f"Configured camera {name} is absent")
                device = found[binding.serial]
                if device.get_info(rs.camera_info.name) != binding.model:
                    raise ValueError(f"{name}: RealSense model mismatch")
                for sensor in device.query_sensors():
                    if sensor.supports(rs.option.global_time_enabled):
                        sensor.set_option(rs.option.global_time_enabled, 1)
                pipeline, config = rs.pipeline(), rs.config()
                config.enable_device(binding.serial)
                for stream, profile in ((rs.stream.color, binding.color), (rs.stream.depth, binding.depth)):
                    config.enable_stream(stream, profile.width, profile.height, getattr(rs.format, profile.format), profile.fps)
                profile = pipeline.start(config)
                self._cleanup.callback(pipeline.stop)
                if profile.get_device().get_info(rs.camera_info.serial_number) != binding.serial:
                    raise ValueError("RealSense opened a different serial")
                self._pipelines[name] = pipeline
                scale = profile.get_device().first_depth_sensor().get_depth_scale()
                thread = threading.Thread(target=self._collect, args=(name, binding, pipeline, scale),
                                          daemon=True, name=f"rgbd-{body_id}-{name}")
                self._threads.append(thread)
                thread.start()
        except BaseException:
            self.close()
            raise

    def manifest(self):
        signals = ("rgb", "depth", "intrinsics")
        return BodyManifest(self.body_id, "realsense", [DeviceManifest(cam.device_id, "camera",
            signals={n: Signal(n) for n in signals},
            observation_streams={"rgbd": ObservationStreamSpec(list(signals), max_bytes=32 * 1024**2)},
            metadata={"identity": self.installation.identity(name, "cameras")}) for name, cam in self.robot.cameras.items()])

    def _collect(self, name, binding, pipeline, scale):
        rs = self.rs
        align = rs.align(rs.stream.color)
        device = self.robot.cameras[name].device_id
        sequence = 0
        while not self._stop.is_set():
            try:
                frames = pipeline.wait_for_frames(binding.timeout_ms)
                raw_color, raw_depth = frames.get_color_frame(), frames.get_depth_frame()
                if not raw_color or not raw_depth:
                    raise ValueError("Incomplete RGB-D frameset")
                if any(f.get_frame_timestamp_domain() != rs.timestamp_domain.global_time for f in (raw_color, raw_depth)):
                    raise ValueError("RealSense source timestamps are not mapped to host global time")
                stamps = [int(round(f.get_timestamp() * 1_000_000)) for f in (raw_color, raw_depth)]
                if abs(stamps[0] - stamps[1]) > binding.max_pair_skew_s * 1e9:
                    raise ValueError("RGB-D capture skew exceeds commissioned bound")
                start = time.monotonic_ns()
                wall, mono = time.time_ns(), time.monotonic_ns()
                if max(stamps) > wall + 1_000_000 or wall - min(stamps) > binding.max_frame_age_s * 1e9:
                    raise ValueError("RealSense source clock is stale or ahead of the host")
                frames = align.process(frames)
                color, depth = frames.get_color_frame(), frames.get_depth_frame()
                rgb = np.asanyarray(color.get_data())
                if binding.color.format == "bgr8":
                    rgb = rgb[:, :, ::-1]
                depth_m = np.asanyarray(depth.get_data()).astype(float) * scale
                depth_mm = np.where((depth_m > 0) & (depth_m < 65.535), np.rint(depth_m * 1000), 0).astype("<u2")
                intr = color.profile.as_video_stream_profile().get_intrinsics()
                distortion = str(intr.model).rsplit(".", 1)[-1]
                if distortion not in {"none", "brown_conrady", "inverse_brown_conrady"}:
                    raise ValueError("Unsupported RealSense RGB distortion model")
                encoded = io.BytesIO()
                Image.fromarray(rgb).save(encoded, format="JPEG", quality=90)
                sequence += 1
                source_times = {key: {"source_timestamp_ns": str(stamp), "clock_id": host_clock_id(), "source_clock": "unix_realtime"}
                                for key, stamp in zip(("rgb", "depth"), stamps)}
                source_times["pairing"] = {"valid": True, "hardware_synchronized": False, "max_skew_s": binding.max_pair_skew_s}
                timing = {**source_times["rgb"], "source_times": source_times, "received_realtime_ns": str(wall),
                          "received_monotonic_ns": str(mono), "clock_pair_span_ns": str(mono - start)}
                shape = {"width": intr.width, "height": intr.height, "sequence": sequence}
                signals = {"rgb": {"format": "jpeg", "data_b64": base64.b64encode(encoded.getvalue()).decode(), **shape},
                    "depth": {"format": "u16le_b64", "aligned_to": "color", "unit": "mm",
                              "data_b64": base64.b64encode(depth_mm.tobytes()).decode(), **shape},
                    "intrinsics": {"color": {"fx": intr.fx, "fy": intr.fy, "cx": intr.ppx, "cy": intr.ppy,
                        "width": intr.width, "height": intr.height, "model": distortion, "d": list(intr.coeffs)}}}
                self._histories[device].append(timing, signals, generation=1, sequence=sequence)
                with self._lock:
                    self._latest[device] = {"signals": signals, "timing": timing, "freshness": "fresh", "moving": False}
                    self._errors.pop(device, None)
            except Exception as error:
                with self._lock:
                    self._errors[device] = str(error)
                    previous = self._capture_failures.get(device, {})
                    self._capture_failures[device] = {
                        "count": previous.get("count", 0) + 1,
                        "last_error": str(error), "time_ns": str(time.time_ns()),
                    }
                self._stop.wait(.02)

    def read(self, device_id, include=None):
        with self._lock:
            if device_id not in self._latest:
                raise DeviceError(ErrorCode.OBSERVATION_STALE, self._errors.get(device_id, "Camera warming up"))
            packet = dict(self._latest[device_id])
            warning = self._errors.get(device_id)
        packet["age_s"] = (time.time_ns() - int(packet["timing"]["source_timestamp_ns"])) / 1e9
        binding = self.installation.host.cameras[self.robot.cameras[self._by_device[device_id]].binding]
        if not 0 <= packet["age_s"] <= binding.max_frame_age_s:
            raise DeviceError(ErrorCode.OBSERVATION_STALE, warning or "Camera disconnected or stale")
        # One rejected incoming frame does not invalidate an earlier, still-fresh
        # valid RGB-D pair. Its original source timestamp remains authoritative.
        if warning:
            packet["collection_warning"] = warning
        return packet

    def read_observation_at(self, device_id, stream, at, *, mode="bracket", max_gap_s=.05, include=None):
        if stream != "rgbd":
            raise DeviceError(ErrorCode.NOT_SUPPORTED, "Unknown camera stream")
        self.read(device_id)
        result = self._histories[device_id].query(at, mode=mode, max_gap_s=max_gap_s)
        for sample in result["samples"]:
            sample["signals"] = sample.pop("payload")
        return result

    def check(self, device_id):
        try:
            packet = self.read(device_id)
            return {"readiness": "satisfied", "checks": {"rgbd": {
                "state": "pass", "age_s": packet["age_s"],
                "collection_warning": packet.get("collection_warning"),
            }}}
        except Exception as error:
            return {"readiness": "unsatisfied", "checks": {"rgbd": {"state": "fail", "error": str(error)}}}

    def health(self):
        with self._lock:
            failures = dict(self._capture_failures)
        return {"ok": not self._stop.is_set() and all(
            self.check(device)["readiness"] == "satisfied" for device in self._by_device),
            "robot_id": self.robot.robot_id, "capture_failures": failures}

    def write(self, *args, **kwargs):
        raise DeviceError(ErrorCode.NOT_SUPPORTED, "Camera Body is observation-only")

    invoke = stream_apply = write

    def stream_hold(self, device_id):
        return None

    def estop(self, device_id):
        return {"status": "not_required", "physical_stop_guaranteed": False}

    def close(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=5.1)
        self._cleanup.close()
