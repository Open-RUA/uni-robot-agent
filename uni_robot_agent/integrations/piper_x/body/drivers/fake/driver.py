"""In-memory fake drivers: the smallest complete Driver (two scalar devices), the smallest arm that meets
``core/arm/contract.py`` and a synthetic RGB-D camera. None of them touches hardware."""

from __future__ import annotations

import base64
import io
import math
import threading
import time
from copy import deepcopy

import numpy as np
from PIL import Image

from ...core.arm import eef
from ...core.arm.motion_limits import MotionLimits
from ...core.contract import DeviceError, ErrorCode
from ...core.manifest import (
    BodyManifest, Command, DeviceManifest, ObservationStreamSpec, Procedure, Signal, StreamSpec, num,
)
from ...core.observations import ObservationBuffer


class FakeDriver:
    """Two independent scalar devices; no robot SDK, camera or physical I/O."""

    data_origin = "fake"

    def __init__(self, body_id: str = "fake", device_ids=("left", "right")):
        self.body_id = body_id
        self._lock = threading.RLock()
        self._positions = {name: 0.0 for name in device_ids}
        self._active: dict[str, threading.Event] = {}
        self.closed = False

    def manifest(self):
        return BodyManifest(self.body_id, "fake", [DeviceManifest(
            name, "chassis", hardware_profile="fake_scalar",
            signals={"position": Signal("position", shape=(1,))},
            commands={"position": Command("position", shape=(1,), minimum=-1, maximum=1)},
            procedures={"move": Procedure("move", params={
                "target": {"type": "number", "minimum": -1, "maximum": 1},
                "duration_s": {"type": "number", "minimum": 0, "maximum": 30},
            }, required=["target"], completion="fake_target_reached")},
            stream=StreamSpec(commands=["position"], rate_hz_max=100),
            limitations=["In-memory fake; never physical evidence"],
        ) for name in self._positions])

    def health(self):
        return {"ok": True, "data_origin": "fake"}

    def read(self, device_id, include=None):
        with self._lock:
            return {"signals": {"position": [self._positions[device_id]]},
                    "moving": device_id in self._active,
                    "freshness": "fresh", "age_s": 0.0,
                    "timing": {"source_timestamp_ns": time.monotonic_ns()},
                    "source": {"driver": "fake"}}

    def check(self, device_id):
        return {"readiness": "satisfied", "checks": {"fake": {"state": "pass"}}}

    def _move(self, device_ids, target, duration_s, cancel, progress):
        if (type(target) not in (float, int) or not math.isfinite(target) or not -1 <= target <= 1
                or type(duration_s) not in (float, int) or not math.isfinite(duration_s)
                or not 0 <= duration_s <= 30):
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "target in [-1,1] and duration_s in [0,30] required")
        hold = threading.Event()
        with self._lock:
            if self.closed:
                raise RuntimeError("fake driver is closed")
            if cancel.is_set():
                return {"partial": False}
            for device_id in device_ids:
                self._active[device_id] = hold
        started = time.monotonic()
        try:
            while time.monotonic() - started < duration_s:
                if cancel.is_set() or hold.wait(min(.005, duration_s)):
                    return {"partial": True}
                progress(min(1., (time.monotonic() - started) / duration_s))
            with self._lock:
                if cancel.is_set() or hold.is_set():
                    return {"partial": True}
                for device_id in device_ids:
                    self._positions[device_id] = float(target)
            return {"target": target, "data_origin": "fake"}
        finally:
            with self._lock:
                for device_id in device_ids:
                    self._active.pop(device_id, None)

    def write(self, device_id, command, value, *, cancel):
        return self._move([device_id], value[0], .02, cancel, lambda _: None)

    def invoke(self, device_ids, procedure, params, *, cancel, progress):
        return self._move(device_ids, params["target"], params.get("duration_s", .02), cancel, progress)

    def stream_apply(self, device_id, targets):
        with self._lock:
            self._positions[device_id] = float(targets["position"][0])

    def stream_hold(self, device_id):
        with self._lock:
            if active := self._active.get(device_id):
                active.set()

    def estop(self, device_id):
        self.stream_hold(device_id)
        return {"hold_requested": True, "physical_stop_guaranteed": False}

    def close(self):
        with self._lock:
            self.closed = True
            for active in self._active.values():
                active.set()


class FakeArmDriver:
    """An arm with no kinematics: joints, end-effector pose and gripper are independent stored values.

    Copy its manifest to start a real arm driver: every name it declares is one the environment reads
    (see ``core/arm/contract.py``); everything else in a real driver is its own diagnostics.
    """

    data_origin = "fake"
    JOINTS = 6

    def __init__(self, body_id: str = "fake-arm", device_ids=("left_arm", "right_arm"), motion_limits=None):
        self.body_id = body_id
        self.motion_limits = MotionLimits.from_config(motion_limits)
        self._lock = threading.RLock()
        self._state = {name: {"joint_position_rad": [0.0] * self.JOINTS, "eef_position_m": [0.3, 0.0, 0.2],
                              "eef_rpy_rad": [0.0, 0.0, 0.0], "gripper_opening": [1.0]} for name in device_ids}
        self._moving: set[str] = set()

    def manifest(self):
        pose = {**eef.TARGET_PROPERTIES, "duration_s": num(0.0, 30), "gripper_opening": num(0, 1)}
        joints = {"type": "array", "items": {"type": "number"}, "minItems": self.JOINTS, "maxItems": self.JOINTS}
        return BodyManifest(self.body_id, "fake-arm", [DeviceManifest(
            name, "arm", hardware_profile="fake_arm",
            frames=[f"{name}/base"],
            signals={
                "joint_position_rad": Signal("joint_position_rad", "rad", (self.JOINTS,)),
                "eef_position_m": Signal("eef_position_m", "m", (3,), "arm base frame"),
                "eef_rpy_rad": Signal("eef_rpy_rad", "rad", (3,)),
                "gripper_opening": Signal("gripper_opening", "fraction", (1,), "1 open, 0 closed"),
            },
            procedures={
                "move_eef": Procedure("move_eef", "Set the end-effector pose.", pose,
                                      [*eef.TARGET_REQUIRED, "duration_s"], completion="eef_pose_feedback_verified"),
                "move_joints": Procedure("move_joints", "Set absolute or relative joint positions.", {
                    "kind": {"type": "string", "enum": ["absolute", "relative"]}, "values": joints,
                    "duration_s": num(0.0, 30), "gripper_opening": num(0, 1)}, ["kind", "values", "duration_s"]),
                "set_gripper": Procedure("set_gripper", "Set the gripper opening.",
                                         {"opening": num(0, 1), "duration_s": num(0.0, 10)}, ["opening"]),
                "move_eef_group": Procedure("move_eef_group", "Move several arms at once; each target is a pose or "
                                            "{hold_joints}.", {"targets": {"type": "object"}}, ["targets"],
                                            completion="eef_pose_feedback_verified"),
            },
            metadata={
                "identity": {"model": "fake_arm", "joints": self.JOINTS},
                "joint_limits_rad": {"lower": [-math.pi] * self.JOINTS, "upper": [math.pi] * self.JOINTS},
                "motion_limits": self.motion_limits.to_dict(),
                "eef_coordinates": eef.coordinate_metadata(name),
            },
            limitations=["In-memory fake; never physical evidence"],
        ) for name in self._state])

    def health(self):
        return {"ok": True, "data_origin": self.data_origin}

    def read(self, device_id, include=None):
        with self._lock:
            return {"signals": deepcopy(self._state[device_id]), "moving": device_id in self._moving,
                    "freshness": "fresh", "age_s": 0.0, "timing": {"source_timestamp_ns": time.monotonic_ns()},
                    "source": {"driver": "fake_arm"}}

    def check(self, device_id):
        return {"readiness": "satisfied", "checks": {"fake": {"state": "pass"}}}

    def _apply(self, device_id, target):
        """Validate one target against the declared contract, then return the state it leads to."""
        state = deepcopy(self._state[device_id])
        if "hold_joints" in target:
            target = {"kind": "absolute", "values": target["hold_joints"], **target}
        if "values" in target:
            values = [float(v) for v in target["values"]]
            if len(values) != self.JOINTS or not all(math.isfinite(v) for v in values):
                raise DeviceError(ErrorCode.INVALID_ARGUMENTS, f"{self.JOINTS} finite joint values required")
            if target.get("kind") == "relative":
                values = [a + b for a, b in zip(state["joint_position_rad"], values)]
            if any(abs(v) > math.pi for v in values):
                raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "joint target outside joint_limits_rad")
            state["joint_position_rad"] = values
        if "target_position_m" in target:
            if target.get("target_frame") != f"{device_id}/base":
                raise DeviceError(ErrorCode.INVALID_ARGUMENTS, f"target_frame must be {device_id}/base")
            state["eef_position_m"] = eef.finite_vector(target["target_position_m"], "target_position_m")
            if "target_rpy_rad" in target:
                state["eef_rpy_rad"] = eef.finite_vector(target["target_rpy_rad"], "target_rpy_rad")
        opening = target.get("opening", target.get("gripper_opening"))
        if opening is not None:
            state["gripper_opening"] = [min(1.0, max(0.0, float(opening)))]
        return state

    def invoke(self, device_ids, procedure, params, *, cancel, progress):
        if procedure == "move_eef_group":
            targets = params["targets"]
            if set(targets) != set(device_ids):
                raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "targets must name exactly the invoked arms")
        else:
            targets = {device_id: params for device_id in device_ids}
        with self._lock:
            planned = {device_id: self._apply(device_id, target) for device_id, target in targets.items()}
            self._moving |= set(planned)
        try:
            if cancel.wait(min(float(params.get("duration_s", 0.0)), 0.02)):
                return {"partial": True}
            with self._lock:
                self._state.update(planned)
            progress(1.0)
            return {"requested_targets_reached": True, "data_origin": self.data_origin}
        finally:
            with self._lock:
                self._moving -= set(planned)

    def write(self, *args, **kwargs):
        raise DeviceError(ErrorCode.NOT_SUPPORTED, "the fake arm declares no direct commands")

    stream_apply = write

    def stream_hold(self, device_id):
        return None

    def estop(self, device_id):
        return {"hold_requested": True, "physical_stop_guaranteed": False}

    def close(self):
        pass


def driver(kind: str = "scalar", **kwargs):
    """Site/host factory entry point: ``driver_args.kind`` picks ``scalar`` (default) or ``arm``."""
    if kind == "arm":
        return FakeArmDriver(**kwargs)
    if kind != "scalar":
        raise ValueError(f"unknown fake driver kind {kind!r}")
    return FakeDriver(**kwargs)


class FakeObservationDriver:
    data_origin = "synthetic"

    def __init__(self, body_id="fake-observation"):
        self.body_id = body_id
        self.closed = False
        self._clock_id = "linux-boot:synthetic-observation-fixture"
        pixels = np.full((24, 32, 3), [40, 100, 160], np.uint8)
        buf = io.BytesIO()
        Image.fromarray(pixels).save(buf, format="JPEG")
        self._jpeg = base64.b64encode(buf.getvalue()).decode()
        self._history = ObservationBuffer(max_bytes=4 * 1024**2)
        self._stop = threading.Event()
        self._latest = None
        self._produce()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="synthetic-rgbd")
        self._thread.start()

    def manifest(self):
        return BodyManifest(self.body_id, "fake-observation", [
            DeviceManifest("camera", "camera", signals={name: Signal(name) for name in ("rgb", "depth", "intrinsics", "frame", "calibration")},
                           metadata={"identity": {"model": "synthetic", "serial": "SYNTHETIC-CAMERA"}},
                           observation_streams={"rgbd": ObservationStreamSpec(["rgb", "depth", "intrinsics", "frame", "calibration"])}, limitations=["Synthetic data; no physical evidence"]),
            DeviceManifest("arm", "arm", signals={name: Signal(name) for name in ("joint_position_rad", "joint_history")},
                           limitations=["Read-only synthetic joint feedback"]),
        ])

    def health(self):
        return {"ok": True, "data_origin": self.data_origin}

    def read(self, device_id, include=None):
        if self.closed:
            raise RuntimeError("synthetic Body is closed")
        now = time.time_ns()
        if device_id == "arm":
            mono = time.monotonic_ns()
            history = [{"joint_position_rad": [0.1] * 6, "timing": {
                "clock_id": self._clock_id, "source_clock": "unix_realtime",
                "source_semantics": "controller_status_publication", "source_timestamp_ns": str(now-delta),
                "received_realtime_ns": str(now-delta+1000), "received_monotonic_ns": str(mono-delta+1000),
                "clock_pair_span_ns": "500"}} for delta in range(200_000_000, -1, -10_000_000)]
            return {"signals": {"joint_position_rad": [0.1]*6, "joint_history": history},
                    "freshness": "fresh", "age_s": 0.0, "moving": False, "data_origin": self.data_origin}
        if device_id != "camera":
            raise DeviceError(ErrorCode.UNKNOWN_DEVICE, "unknown synthetic device")
        return deepcopy(self._latest)

    def _camera_packet(self):
        now = time.time_ns()
        stamp = now - 25_000_000
        sequence = stamp // 33_333_333
        depth = np.full((24, 32), 500, np.uint16)
        depth[10, 10] = 0
        source = {name: {"timestamp_domain": "Global Time", "timestamp_ms": f"{(stamp+delta)//1_000_000}.{(stamp+delta)%1_000_000:06d}",
                         "frame_number": str(sequence), "metadata": {"Sensor Timestamp": str(1_000_000+delta//1000)}}
                  for name, delta in (("rgb", 0), ("depth", 12_000_000))}
        timing = {"clock_id": self._clock_id, "callback_realtime_ns": str(now), "callback_monotonic_raw_ns": str(time.monotonic_ns()), "clock_pair_span_ns": "500"}
        # What core/camera/contract.py asks of every camera: per-image source times on the host clock.
        timing["source_times"] = {name: {"source_timestamp_ns": str(stamp + delta), "clock_id": self._clock_id,
                                         "source_clock": "unix_realtime"} for name, delta in (("rgb", 0), ("depth", 12_000_000))}
        timing["source_times"]["pairing"] = {"valid": True, "hardware_synchronized": False}
        intrinsic = {"fx": 40., "fy": 40., "cx": 16., "cy": 12., "width": 32, "height": 24, "model": "none",
                     "d": [0.] * 5}
        return {"freshness": "fresh", "age_s": .025, "data_origin": self.data_origin, "timing": timing, "signals": {
            "rgb": {"format": "jpeg", "data_b64": self._jpeg, "width": 32, "height": 24, "sequence": sequence},
            "depth": {"format": "u16le_b64", "unit": "mm", "aligned_to": "color", "frame_id": "camera/color_optical",
                      "data_b64": base64.b64encode(depth.astype('<u2').tobytes()).decode(), "width": 32, "height": 24, "sequence": sequence},
            "intrinsics": {"color": intrinsic, "depth": intrinsic}, "calibration": {},
            "frame": {"source": source, "timing": timing, "data_origin": self.data_origin}}}

    def _produce(self):
        packet = self._camera_packet()
        timing = packet["timing"]
        indexed_timing = {**timing, **timing["source_times"]["rgb"]}
        self._history.append(indexed_timing, packet["signals"], generation=1,
                             sequence=packet["signals"]["rgb"]["sequence"])
        self._latest = packet

    def _loop(self):
        while not self._stop.wait(.01):
            self._produce()

    def read_observation_at(self, device_id, stream, at, *, mode="bracket", max_gap_s=.05, include=None):
        if device_id != "camera" or stream != "rgbd":
            raise DeviceError(ErrorCode.NOT_SUPPORTED, "no synthetic history for this stream")
        result = self._history.query(at, mode=mode, max_gap_s=max_gap_s)
        for sample in result["samples"]:
            sample["signals"] = sample.pop("payload")
        return result

    def check(self, device_id):
        return {"readiness": "satisfied", "checks": {"synthetic": {"state": "pass"}}}

    def write(self, *args, **kwargs):
        raise DeviceError(ErrorCode.NOT_SUPPORTED, "synthetic observation driver is read-only")

    invoke = write
    stream_apply = write

    def stream_hold(self, device_id):
        return None

    def estop(self, device_id):
        return {"data_origin": self.data_origin, "physical_stop_guaranteed": False}

    def close(self):
        self.closed = True
        self._stop.set()
        self._thread.join(timeout=1)


def observation_driver(**kwargs):
    return FakeObservationDriver(**kwargs)
