"""Bounded Linux D405 capture; USB access lives in an owned C++ child process.

It runs as an ordinary user. No external process, SDK Python binding,
network endpoint, device reset, or firmware write is used.
"""

from __future__ import annotations

import ctypes as c
import json
import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from pathlib import Path

import numpy as np

from ...core.contract import DeviceError
from ...core.observations import ObservationBuffer
from ...core.timing import host_clock_id
from .clock import CameraClockMonitor, camera_source_time
from .driver import Frame, distortion_name
from .native_camera_ipc import LinuxHelperConnection


class Intrinsics(c.Structure):
    _fields_ = [
        ("width", c.c_int),
        ("height", c.c_int),
        ("ppx", c.c_float),
        ("ppy", c.c_float),
        ("fx", c.c_float),
        ("fy", c.c_float),
        ("model", c.c_int),
        ("coeffs", c.c_float * 5),
    ]


class Extrinsics(c.Structure):
    _fields_ = [("rotation", c.c_float * 9), ("translation", c.c_float * 3)]


def geometry(v):
    return Intrinsics(
        v["width"],
        v["height"],
        v["ppx"],
        v["ppy"],
        v["fx"],
        v["fy"],
        v["model"],
        (c.c_float * 5)(*v["coeffs"]),
    )


def public_intrinsics(v):
    return {
        "width": v["width"],
        "height": v["height"],
        "fx": v["fx"],
        "fy": v["fy"],
        "cx": v["ppx"],
        "cy": v["ppy"],
        "model": distortion_name(v["model"]),
        "d": v["coeffs"],
        "k": [v["fx"], 0, v["ppx"], 0, v["fy"], v["ppy"], 0, 0, 1],
    }


class NativeCapture:
    def __init__(self, serials, *, cameras=None):
        self._serials = tuple(dict.fromkeys(str(s) for s in serials))
        from .driver import validate_cameras

        selected = validate_cameras(cameras)
        self._roles = {
            str(v["serial"]): (1 << i, str(v["sdk_serial"]))
            for i, v in enumerate(selected.values())
        }
        if (
            not self._serials
            or len(self._roles) > 16
            or len(self._roles) != len(selected)
            or any(s not in self._roles for s in self._serials)
        ):
            raise ValueError(
                "Native D405 backend requires the commissioned USB/ASIC identities"
            )
        from ...core.resources import runtime_root

        libpath = runtime_root() / "native/cameras/librua-rgbd-processing.so"
        if not libpath.is_file():
            libpath = (
                Path(__file__).resolve().parent / "capture/librua-rgbd-processing.so"
            )
        self._lib = c.CDLL(str(libpath))  # memory-only; creates no SDK context
        # The sized entry point; a library built before buffer sizes were passed fails to load here.
        self._align = self._lib.rua_align_z16_sized
        self._align.argtypes = [
            c.c_void_p,
            c.c_size_t,
            c.c_void_p,
            c.c_size_t,
            c.c_float,
            c.POINTER(Intrinsics),
            c.POINTER(Intrinsics),
            c.POINTER(Extrinsics),
        ]
        self._align.restype = c.c_int
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._frames = {}
        self._clock_monitors = {s: CameraClockMonitor() for s in self._serials}
        self._histories = {s: ObservationBuffer(max_samples=60, max_bytes=64 * 1024**2) for s in self._serials}
        self._errors = dict.fromkeys(self._serials, "awaiting native frames")
        from ...core.resources import ResourceLocks

        self._ownership = ResourceLocks(["camera:" + s for s in self._serials])
        self._selected = selected
        try:
            self._connection = self._connect()
        except BaseException:
            self._ownership.close()
            raise
        self._thread = threading.Thread(
            target=self._poll, name="rua-camera-ipc", daemon=True
        )
        self._thread.start()

    def _connect(self):
        """Start the capture helper and acquire the configured cameras."""
        connection = LinuxHelperConnection(cameras=self._selected)
        try:
            code, _, _, _ = connection.request(1)
            if code:
                raise RuntimeError("Native capture HELLO failed")
            code, _, _, _ = connection.request(
                2, role_mask=sum(self._roles[s][0] for s in self._serials)
            )
            if code:
                raise RuntimeError("Native capture ACQUIRE failed")
        except BaseException:
            connection.close()
            raise
        return connection

    def open(self, serial, *, width=640, height=480, fps=30):
        if self._stop.is_set():
            raise RuntimeError("capture is closed")
        if serial not in self._serials or (width, height, fps) != (640, 480, 30):
            raise ValueError(
                "Native capture requires an acquired identity and 640x480@30"
            )
        return serial

    def _decode(self, serial, meta, rgb, depth):
        cal = meta["calibration"]
        if (
            meta["serial"] != serial
            or cal["asic_serial"] != serial
            or cal["sdk_serial"] != self._roles[serial][1]
        ):
            raise ValueError("Camera frame identity mismatch")
        if (meta["width"], meta["height"]) != (640, 480):
            raise ValueError("Invalid camera dimensions")
        if len(rgb) != 640 * 480 * 3 or len(depth) != 640 * 480 * 2:
            raise ValueError("Camera packet payload size does not match 640x480 RGB and Z16")
        decode_started = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
        source, target = (
            geometry(cal["depth_intrinsics"]),
            geometry(cal["rgb_intrinsics"]),
        )
        transform = cal["depth_to_rgb"]
        extr = Extrinsics(
            (c.c_float * 9)(*transform["rotation"]),
            (c.c_float * 3)(*transform["translation"]),
        )
        aligned = c.create_string_buffer(640 * 480 * 2)
        scale = float(cal["depth_scale_m"])
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError("Invalid depth scale")
        if self._align(
            depth, len(depth), aligned, len(aligned), scale, c.byref(source), c.byref(target), c.byref(extr)
        ):
            raise ValueError("Native depth registration failed")
        received_mono = time.monotonic()
        age = (
            time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
            - int(meta["arrival_monotonic_ns"]) / 1e9
        )
        if age < 0 or age > 0.5:
            raise ValueError("Native frame is stale")
        stamp = received_mono - age
        mm = np.clip(
            np.rint(
                np.frombuffer(aligned.raw, dtype="<u2")
                .reshape(480, 640)
                .astype(np.float64)
                * scale
                * 1000
            ),
            0,
            65535,
        ).astype(np.uint16)
        rgb_array = np.frombuffer(rgb, dtype=np.uint8).reshape(480, 640, 3)
        timing = {
            **meta.get("timing", {}),
            "clock_id": host_clock_id(),
            "capture_queue": dict(meta.get("capture_queue", {})),
            "decode_started_monotonic_raw_ns": str(decode_started),
            "decode_completed_monotonic_raw_ns": str(
                time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
            ),
        }
        generation = (meta.get("producer_instance"), meta.get("source", {}).get("stream_generation"))
        times, error = self._clock_monitors[serial].check(meta.get("source", {}), timing, generation)
        timing["clock_epoch"] = self._clock_monitors[serial].clock_epoch
        timing["source_times"] = times
        timing["source_time"] = times.get("rgb")
        timing["source_time_error"] = error
        timing["clock_rejected_frames"] = self._clock_monitors[serial].rejected
        timing["clock_diagnostics"] = self._clock_monitors[serial].diagnostics()
        return Frame(
            rgb_array,
            mm,
            public_intrinsics(cal["rgb_intrinsics"]),
            public_intrinsics(cal["depth_intrinsics"]),
            scale,
            stamp,
            int(meta["sequence"]),
            {
                "calibration": cal,
                "source": meta.get("source", {}),
                "producer_instance": meta.get("producer_instance"),
                "sequence": int(meta["sequence"]),
                "timing": timing,
            },
        )

    #: Consecutive failed reconnects before the capture stays disconnected.
    RECONNECT_LIMIT = 5

    def _poll(self):
        """Serve frames; after a helper failure, reconnect with backoff a bounded number of times."""
        failures = 0
        while not self._stop.is_set():
            delivered = self._serve()
            if self._stop.is_set():
                break
            failures = 0 if delivered else failures + 1
            if failures > self.RECONNECT_LIMIT:
                return
            if self._stop.wait(min(10.0, 0.5 * 2 ** failures)):
                break
            try:
                self._connection = self._connect()
            except Exception as exc:
                with self._lock:
                    self._errors = dict.fromkeys(self._serials, f"native capture reconnect failed: {exc}")
                failures += 1

    def _serve(self):
        """Run one helper session until it fails or capture stops; returns whether frames arrived."""
        delivered = False
        sequences = dict.fromkeys(self._serials, 0)
        pending = {}
        try:
            # One owner of the ordered IPC channel; independent cameras process
            # concurrently. At most ONE in-flight packet per camera, with no
            # executor backlog and no within-camera reordering/clock races.
            with ThreadPoolExecutor(max_workers=len(self._serials),
                                    thread_name_prefix="rua-rgbd") as pool:
                while not self._stop.is_set():
                    progressed = False
                    for serial in self._serials:
                        if self._stop.is_set():
                            break
                        future = pending.get(serial)
                        if future is not None:
                            if not future.done():
                                continue
                            del pending[serial]
                            progressed = True
                            try:
                                frame = future.result()
                            except Exception as exc:
                                with self._lock:
                                    self._frames.pop(serial, None)
                                    # Bad packets advance the cursor, but do not
                                    # invalidate earlier source-stamped history.
                                    self._errors[serial] = str(exc)
                                    self._histories[serial].record_rejection("decode: " + str(exc))
                            else:
                                self._retain(serial, frame)
                                delivered = True
                                with self._lock:
                                    self._frames[serial] = frame
                                    self._errors.pop(serial, None)
                        if self._stop.is_set():
                            break
                        code, meta, rgb, depth = self._connection.request(
                            7,  # ReadNext: oldest queued packet after this cursor
                            role_mask=self._roles[serial][0],
                            after_sequence=sequences[serial],
                        )
                        if code == 4:
                            continue
                        if code:
                            with self._lock:
                                self._frames.pop(serial, None)
                                self._histories[serial].clear(reason="capture_ipc_error")
                                self._errors[serial] = str(
                                    meta.get("detail", meta.get("error", "capture unavailable"))
                                )
                            continue
                        sequence = int(meta["sequence"])
                        if sequence <= sequences[serial]:
                            raise ValueError("native history sequence did not advance")
                        sequences[serial] = sequence
                        pending[serial] = pool.submit(self._decode, serial, meta, rgb, depth)
                        progressed = True
                    # Drain ready streams immediately. Sleep only when waiting
                    # for new packets or in-flight processing, not at camera FPS.
                    if not progressed:
                        self._stop.wait(0.002)
        except Exception as exc:
            with self._lock:
                self._frames.clear()
                for history in self._histories.values():
                    history.clear(reason="capture_disconnected")
                self._errors = dict.fromkeys(
                    self._serials, f"native capture disconnected: {exc}"
                )
            self._connection.close()
        return delivered

    def _retain(self, serial, frame):
        history = self._histories[serial]
        timing = frame.metadata.get("timing", {})
        generation = (frame.metadata.get("producer_instance"),
                      frame.metadata.get("source", {}).get("stream_generation"),
                      timing.get("clock_epoch", 0))
        history.begin_generation(timing, generation)
        if timing.get("source_time_error"):
            self._record_rejection(serial, frame, timing["source_time_error"])
            return
        try:
            source_time = camera_source_time(frame.metadata.get("source", {}), timing)
        except ValueError as exc:
            self._record_rejection(serial, frame, str(exc))
            return  # Untimed images remain inspectable via latest(), not historical pairing.
        size = 8192 + frame.color_rgb.nbytes + frame.depth_mm.nbytes
        history.append({**timing, **source_time}, frame, generation=generation,
                       sequence=frame.sequence, size_bytes=size)

    def _record_rejection(self, serial, frame, reason):
        history = self._histories[serial]
        history.record_rejection(reason, sequence=frame.sequence)
        # Warm-up is visible in metadata; persistent failures log at most every
        # five seconds per camera, including source-time conversion failures.
        if reason == "camera clock needs three consistent source samples":
            return
        if not hasattr(self, "_diagnostic_log_at"):
            self._diagnostic_log_at = {}
        now = time.monotonic()
        if now - self._diagnostic_log_at.get(serial, float("-inf")) >= 5:
            self._diagnostic_log_at[serial] = now
            logging.getLogger(__name__).warning("camera_history_rejected %s", json.dumps({
                "serial": serial, "sequence": frame.sequence, "reason": reason,
                "clock_diagnostics": frame.metadata.get("timing", {}).get("clock_diagnostics"),
                "history_diagnostics": history.diagnostics(),
            }, ensure_ascii=True))

    def diagnostics(self, handle):
        with self._lock:
            frame = self._frames.get(handle)
            timing = frame.metadata.get("timing", {}) if frame else {}
            result = deepcopy({
                "serial": handle, "latest_sequence": frame.sequence if frame else None,
                "latest_frame_age_s": max(0., time.monotonic() - frame.captured_mono)
                    if frame is not None and hasattr(frame, "captured_mono") else None,
                "preview_available": frame is not None,
                "capture_error": self._errors.get(handle),
                "source_time_error": timing.get("source_time_error"),
                "clock_diagnostics": timing.get("clock_diagnostics"),
            })
        history = self._histories[handle].diagnostics()
        result["history"] = history
        result["history_available"] = history["retained_count"] > 0
        error = result["source_time_error"]
        result["status"] = (
            "capture_error" if result["capture_error"] else
            "awaiting_frames" if not result["preview_available"] else
            "clock_warming" if error == "camera clock needs three consistent source samples" else
            "clock_rejected" if error else
            "history_rejected" if history["consecutive_rejected"] else
            "ready" if result["history_available"] else "history_empty")
        return result

    def read_observation_at(self, handle, at, *, mode="bracket", max_gap_s=0.05):
        try:
            return self._histories[handle].query(at, mode=mode, max_gap_s=max_gap_s)
        except DeviceError as exc:
            exc.details = {**(exc.details or {}),
                           "camera_diagnostics": self.diagnostics(handle)}
            raise

    def latest(self, handle):
        with self._lock:
            return self._frames.get(handle)

    def errors(self):
        with self._lock:
            return dict(self._errors)

    def close(self):
        self._stop.set()
        self._connection.close()
        self._thread.join(3)
        if self._thread.is_alive():
            # Do not release physical ownership while processing still exists.
            raise RuntimeError("camera processing did not stop; ownership retained")
        for history in self._histories.values():
            history.clear(reason="capture_closed")
        self._ownership.close()
