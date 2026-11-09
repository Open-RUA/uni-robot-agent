"""One RPC supplies the exact RGB, depth and calibration used for pixel picking."""

from __future__ import annotations

import base64
import io
import math
import time
from copy import deepcopy
from dataclasses import dataclass

import numpy as np

from ..body.core.contract import DeviceError, ErrorCode
from ..body.core.observations import validate_query
from ..body.core.timing import TIMESTAMP_TOLERANCE_NS, timestamp_ns
from .geometry import _pinhole
from .synchronization import JointHistoryPending, joint_state_at_time


OPTIONAL_SIGNALS = ("calibration", "frame")


def check_cancel(cancel) -> None:
    if cancel is not None and cancel.is_set():
        raise InterruptedError("observation cancelled")


@dataclass
class RGBDSnapshot:
    jpeg: bytes
    depth_mm: np.ndarray
    intrinsics: dict
    metadata: dict
    received_mono: float

    @classmethod
    def capture(cls, handle, *, not_before_mono=None, timeout_s=2.0, max_age_s=0.5, cancel=None) -> RGBDSnapshot:
        for name, value in (("timeout_s", timeout_s), ("max_age_s", max_age_s)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= 30:
                raise ValueError(f"{name} must be finite and in (0, 30]")
        if not_before_mono is not None and (isinstance(not_before_mono, bool) or not isinstance(not_before_mono, (int, float)) or not math.isfinite(not_before_mono)):
            raise ValueError("not_before_mono must be a finite local monotonic time")
        deadline = time.monotonic() + timeout_s
        available = getattr(getattr(handle, "manifest", None), "signals", {})
        depth_signal = "depth_png" if "depth_png" in available else "depth"
        # The camera contract requires rgb, depth and intrinsics; calibration and frame are kept when offered.
        extra = [name for name in OPTIONAL_SIGNALS if name in available]
        while True:
            check_cancel(cancel)
            requested = time.monotonic()
            remaining = deadline - requested
            if remaining <= 0:
                raise DeviceError(
                    ErrorCode.OBSERVATION_STALE, "no camera frame within capture budget"
                )
            reading = handle.read(
                ["rgb", depth_signal, "intrinsics", *extra],
                max_age_s=max_age_s,
                on_stale="error",
                timeout_s=remaining,
            )
            received = time.monotonic()
            check_cancel(cancel)
            age = reading.get("age_s")
            if (
                isinstance(age, bool)
                or not isinstance(age, (int, float))
                or not math.isfinite(age)
                or age < 0
                or reading.get("freshness") == "stale"
            ):
                raise DeviceError(
                    ErrorCode.OBSERVATION_STALE,
                    "camera frame age is unavailable or invalid",
                )
            # Body and agent clocks need not share an epoch. Count the full RPC
            # as uncertainty: this is a conservative lower bound, not a synced
            # exposure timestamp. Never relabel an old frame as post-motion.
            captured_lower_bound = requested - age
            if (max_age_s is not None and received - captured_lower_bound > max_age_s) or received > deadline:
                raise DeviceError(
                    ErrorCode.OBSERVATION_STALE, "camera frame expired in transit"
                )
            if not_before_mono is None or captured_lower_bound >= not_before_mono:
                break
            time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))
        snapshot = cls._decode(handle, reading, depth_signal)
        age_at_receipt = snapshot.received_mono - captured_lower_bound
        if age_at_receipt > max_age_s or snapshot.received_mono > deadline:
            raise DeviceError(ErrorCode.OBSERVATION_STALE, "camera frame expired during decoding")
        snapshot.metadata.update(age_s_at_receipt=age_at_receipt,
                                 rpc_elapsed_s=received-requested,
                                 timing_basis="body_frame_age_plus_full_rpc_upper_bound",
                                 after_observation_boundary=not_before_mono is not None)
        return snapshot

    @classmethod
    def _decode(cls, handle, reading, depth_signal):
        """Decode one selected packet without a second device read."""
        signals = reading.get("signals", {})
        rgb, depth = signals["rgb"], signals[depth_signal]
        if (
            depth.get("aligned_to") != "color"
            or depth.get("unit") != "mm"
            or depth.get("format") not in {"u16le_b64", "png16_b64"}
        ):
            raise ValueError(
                "camera must supply registered uint16 depth in millimetres"
            )
        shape = (int(depth["height"]), int(depth["width"]))
        if shape != (rgb["height"], rgb["width"]):
            raise ValueError("registered depth and RGB dimensions differ")
        if type(rgb.get("sequence")) is not int or rgb["sequence"] < 0 or depth.get("sequence") != rgb["sequence"]:
            raise ValueError("RGB and depth frame sequences differ")
        if not shape[0] > 0 or not shape[1] > 0 or shape[0] * shape[1] > 16_777_216:
            raise ValueError("invalid RGB-D dimensions")
        encoded = base64.b64decode(depth["data_b64"], validate=True)
        if depth["format"] == "png16_b64":
            from PIL import Image

            with Image.open(io.BytesIO(encoded)) as image:
                if image.format != "PNG" or image.mode not in {"I;16", "I;16B", "I;16L", "I"} or image.size != shape[::-1]:
                    raise ValueError("camera must supply matching 16-bit PNG depth")
                values = np.array(image)
                if np.any(values < 0) or np.any(values > 65535):
                    raise ValueError("depth PNG values exceed uint16")
                raw = values.astype(np.uint16)
        else:
            raw = np.frombuffer(encoded, dtype="<u2").reshape(shape).copy()
        frame = signals.get("frame", {})
        timing = deepcopy(reading.get("timing") or frame.get("timing", {}))
        if timing.get("source_time_error") or frame.get("timing", {}).get("source_time_error"):
            raise ValueError(timing.get("source_time_error") or frame["timing"]["source_time_error"])
        # The driver establishes source times (core/camera/contract.py); the snapshot checks and carries them.
        source_times = deepcopy(timing.get("source_times") or {})
        if source_times:
            if not (source_times.get("pairing") or {}).get("valid"):
                raise ValueError("camera did not pair its RGB and depth images")
            latest = max(timestamp_ns(source_times[name]["source_timestamp_ns"]) for name in ("rgb", "depth"))
            if latest > time.time_ns() + TIMESTAMP_TOLERANCE_NS:
                raise ValueError("camera source time is later than the host clock")
        intrinsics = deepcopy(signals["intrinsics"])
        color = intrinsics["color"]
        _pinhole(color)
        if (color.get("height"), color.get("width")) != shape:
            raise ValueError("RGB intrinsics dimensions differ from the frame")
        meta = {
            "frame_id": depth.get("frame_id"),
            "sequence": rgb.get("sequence"),
            "serial": (handle.manifest.metadata.get("identity") or {}).get("serial"),
            "age_s_at_capture": reading.get("age_s"),
            "width": shape[1],
            "height": shape[0],
            "source": deepcopy(frame),
            "source_times": source_times,
            "source_time_status": "available" if source_times else "unavailable",
            "timing": timing,
            "factory_calibration": deepcopy(signals.get("calibration")),
            "hardware_synchronized": False,
            "data_origin": frame.get("data_origin", reading.get("data_origin", "device")),
        }
        jpeg = base64.b64decode(rgb["data_b64"], validate=True)
        if not jpeg:
            raise ValueError("empty RGB payload")
        from PIL import Image

        with Image.open(io.BytesIO(jpeg)) as image:
            if image.format != "JPEG" or image.size != shape[::-1]:
                raise ValueError("RGB must be a matching JPEG image")
            image.verify()
        decoded_at = time.monotonic()
        return cls(
            jpeg,
            raw,
            intrinsics,
            meta,
            decoded_at,
        )

    @classmethod
    def capture_at(cls, handle, at, *, timeout_s=2.0, max_gap_s=0.05,
                   max_age_s=0.5, cancel=None):
        """Select after one fixed request time; never replace it with latest.

        The request must originate in the same host clock domain as the Body.
        Retry only the absence of a future sample, always using the same target.
        """
        target = validate_query(at, "after", max_gap_s)
        for name, value in (("timeout_s", timeout_s), ("max_age_s", max_age_s)):
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 30:
                raise ValueError(f"{name} must be finite and in (0, 30]")
        if "rgbd" not in handle.manifest.observation_streams:
            raise DeviceError(ErrorCode.NOT_SUPPORTED, "camera has no source-stamped RGB-D history")
        at = deepcopy(at)
        began = time.monotonic()
        query_started = str(time.time_ns())
        deadline = began + timeout_s
        depth_signal = "depth_png" if "depth_png" in handle.manifest.signals else "depth"
        extra = [name for name in OPTIONAL_SIGNALS if name in handle.manifest.observation_streams["rgbd"].signals]
        while True:
            check_cancel(cancel)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise DeviceError(ErrorCode.OBSERVATION_STALE, "no post-request frame within capture budget")
            try:
                reply = handle.read_at(at, stream="rgbd", mode="after",
                    max_gap_s=max_gap_s, wait_s=min(.1, remaining), timeout_s=min(1.0, remaining),
                    include=["rgb", depth_signal, "intrinsics", *extra])
                break
            except DeviceError as exc:
                if exc.code != ErrorCode.OBSERVATION_STALE or (exc.details or {}).get("reason") != "pending":
                    raise
                if cancel is None:
                    time.sleep(min(.01, max(0., deadline-time.monotonic())))
                else:
                    cancel.wait(min(.01, max(0., deadline-time.monotonic())))
        check_cancel(cancel)
        if reply.get("at") != at or reply.get("mode") != "after" or len(reply.get("samples", [])) != 1:
            raise ValueError("camera history reply does not match the requested selection")
        sample = reply["samples"][0]
        snapshot = cls._decode(handle, {"signals": sample["signals"], "timing": sample.get("timing", {})}, depth_signal)
        times = snapshot.metadata["source_times"]
        if not times:
            raise ValueError("selected frame has no RGB-D source timestamps")
        stamps = [timestamp_ns(times[name]["source_timestamp_ns"]) for name in ("rgb", "depth")]
        if any(times[name]["clock_id"] != at["clock_id"] for name in ("rgb", "depth")):
            raise ValueError("selected frame and request clocks differ")
        if min(stamps) < target or max(stamps)-target > max_gap_s*1e9:
            raise ValueError("selected RGB-D frame is outside the fixed request window")
        if (sample.get("sequence") != snapshot.metadata["sequence"]
                or timestamp_ns(sample["timing"].get("source_timestamp_ns")) != stamps[0]):
            raise ValueError("history index and RGB-D packet identities differ")
        received_ns = time.time_ns()
        if max(stamps) > received_ns + TIMESTAMP_TOLERANCE_NS:
            raise ValueError("selected source frame is in the future of the request host clock")
        age = max(0., (received_ns - min(stamps))/1e9)
        if age > max_age_s or time.monotonic() > deadline:
            raise DeviceError(ErrorCode.OBSERVATION_STALE, "selected frame expired during history retrieval")
        snapshot.metadata.update(t_request=str(target), t_query_started=query_started,
            t_query_finished=str(received_ns), request_clock_id=at["clock_id"],
            timing_basis="source_history_after_fixed_t_request", selection="after",
            selection_offset_s=(stamps[0]-target)/1e9, age_s_at_receipt=age,
            rpc_elapsed_s=time.monotonic()-began, after_observation_boundary=True)
        return snapshot

    def joint_state_at_capture(self, arm, *, timeout_s=0.5, max_gap_s=0.05, stream="rgb", cancel=None) -> dict:
        """Read source history and wait only for a missing post-frame sample."""
        if (
            isinstance(timeout_s, bool)
            or not math.isfinite(timeout_s)
            or not 0 < timeout_s <= 2
        ):
            raise ValueError("timeout_s must be positive and at most 2 s")
        if (
            isinstance(max_gap_s, bool)
            or not math.isfinite(max_gap_s)
            or not 0 < max_gap_s <= 0.2
        ):
            raise ValueError("max_gap_s must be positive and at most 0.2 s")
        check_cancel(cancel)
        began = time.monotonic()
        try:
            timing = self.metadata.get("timing", {})
            if timing.get("source_time_error"):
                raise ValueError(timing["source_time_error"])
            image_time = (self.metadata.get("source_times") or {}).get(stream)
            if not image_time:
                raise ValueError(f"camera reported no {stream} source time")
        except ValueError as exc:
            raise DeviceError(ErrorCode.OBSERVATION_STALE, str(exc)) from exc
        streams = getattr(getattr(arm, "manifest", None), "observation_streams", {})
        if isinstance(streams, dict) and "state" in streams:
            bracket = arm.read_at(image_time, stream="state", wait_s=timeout_s,
                                  timeout_s=timeout_s, max_gap_s=max_gap_s)
            check_cancel(cancel)
            if time.monotonic() - began > timeout_s:
                raise DeviceError(ErrorCode.OBSERVATION_STALE, "joint pairing exceeded its budget")
            history = [{**s["signals"], "timing": s["timing"]} for s in bracket["samples"]]
            try:
                matched = joint_state_at_time(history, image_time, max_gap_s=max_gap_s)
            except (ValueError, TypeError, JointHistoryPending) as exc:
                raise DeviceError(ErrorCode.OBSERVATION_STALE, str(exc)) from exc
            return {**matched, "device_id": arm.device_id,
                    "camera_sequence": self.metadata.get("sequence"),
                    "bracket_samples": bracket["samples"]}
        deadline = time.monotonic() + timeout_s
        while True:
            check_cancel(cancel)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise DeviceError(
                    ErrorCode.OBSERVATION_STALE,
                    "joint source history did not reach image time within budget",
                )
            reading = arm.read(
                ["joint_history"], max_age_s=0.2, on_stale="error", timeout_s=remaining
            )
            history = reading.get("signals", {}).get("joint_history")
            check_cancel(cancel)
            if history is None:
                raise DeviceError(
                    ErrorCode.PRECONDITION_FAILED,
                    "Body does not expose joint source history; update the arms runtime",
                )
            try:
                matched = joint_state_at_time(history, image_time, max_gap_s=max_gap_s)
            except JointHistoryPending:
                time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))
                continue
            except (ValueError, TypeError) as exc:
                raise DeviceError(ErrorCode.OBSERVATION_STALE, str(exc)) from exc
            if time.monotonic() > deadline:
                raise DeviceError(
                    ErrorCode.OBSERVATION_STALE,
                    "joint history reply exceeded pairing budget",
                )
            return {
                **matched,
                "device_id": arm.device_id,
                "camera_sequence": self.metadata.get("sequence"),
            }

    @property
    def age_s(self) -> float:
        if self.metadata.get("archived"):
            return float("inf")
        return (
            time.monotonic()
            - self.received_mono
            + float(
                self.metadata.get(
                    "age_s_at_receipt", self.metadata.get("age_s_at_capture")
                )
                or 0
            )
        )
