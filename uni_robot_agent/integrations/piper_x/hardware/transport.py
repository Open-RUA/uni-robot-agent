"""Versioned SDK writes and passive SocketCAN feedback; no bridge dependency."""
from __future__ import annotations

from collections import deque
import hashlib
import inspect
from pathlib import Path
import socket
import threading
import time

import numpy as np

from ..environment.inspect import CAN_FILTER, CAN_FRAME, FEEDBACK_IDS, STANDARD_DATA_MASK, FeedbackWindow
from uni_robot_agent.integrations.piper_x.body.core.timing import host_clock_id


class FeedbackUnavailable(RuntimeError):
    """No coherent sample currently satisfies the configured receive bounds."""


class PassiveCan:
    """Receive-only feedback. Host receive times are not controller source times."""

    def __init__(self, interface, *, max_age_s, max_skew_s):
        self.max_age_s, self.max_skew_s = max_age_s, max_skew_s
        self.window = FeedbackWindow()
        self.lock = threading.Lock()
        self.history = deque(maxlen=512)
        self.stop_event = threading.Event()
        self.bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                b"".join(CAN_FILTER.pack(i, STANDARD_DATA_MASK) for i in FEEDBACK_IDS))
            self.bus.bind((interface,))
            self.bus.settimeout(.05)
        except BaseException:
            self.bus.close()
            raise
        self.thread = threading.Thread(target=self._receive, daemon=True, name=f"piper-feedback-{interface}")
        self.thread.start()

    def _receive(self):
        while not self.stop_event.is_set():
            try:
                frame = self.bus.recv(CAN_FRAME.size)
            except TimeoutError:
                continue
            except OSError:
                return
            before = time.monotonic_ns()
            wall, mono = time.time_ns(), time.monotonic_ns()
            with self.lock:
                self.window.accept(frame, mono / 1e9)
                if CAN_FRAME.unpack(frame)[0] != 0x2A7:
                    continue
                report = self.window.report(now=mono / 1e9, elapsed=1,
                    max_age=self.max_age_s, max_skew=self.max_skew_s)
                if not report["feedback_complete_and_recent"]:
                    continue
                self.history.append({"joint_position_rad": report["decoded_feedback"]["joints_rad"],
                    "timing": {"clock_id": host_clock_id(), "source_clock": "unix_realtime",
                        "source_semantics": "host_can_receive", "source_timestamp_ns": str(wall),
                        "received_realtime_ns": str(wall), "received_monotonic_ns": str(mono),
                        "clock_pair_span_ns": str(mono - before)}})

    def read(self):
        # A reader may run between fragments of a CAN update. Wait only for
        # new valid data; never extend sample age/skew or substitute a cache.
        deadline = time.monotonic() + min(.025, self.max_age_s, self.max_skew_s)
        while True:
            try:
                return self._read_once()
            except FeedbackUnavailable:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise
                time.sleep(min(.001, remaining))

    def _read_once(self):
        with self.lock:
            report = self.window.report(now=time.monotonic(), elapsed=1,
                max_age=self.max_age_s, max_skew=self.max_skew_s)
            if not report["feedback_complete_and_recent"] or len(self.history) < 2:
                raise FeedbackUnavailable("CAN feedback is incomplete, stale or incoherent: "
                    f"missing={report.get('missing_ids')}, stale={report.get('stale_ids')}, "
                    f"skew_s={report.get('receive_skew_s')}, joint_samples={len(self.history)}")
            d = report["decoded_feedback"]
            history = list(self.history)
            recent = [h for h in history if int(history[-1]["timing"]["source_timestamp_ns"])
                      - int(h["timing"]["source_timestamp_ns"]) <= 150_000_000]
            age = (time.time_ns() - int(history[-1]["timing"]["source_timestamp_ns"])) / 1e9
            if age > self.max_age_s:
                raise FeedbackUnavailable("Joint feedback stopped")
            status = bytes.fromhex(d["arm_status_payload_hex"])
            measured_motion = np.ptp(np.array([h["joint_position_rad"] for h in recent]), axis=0).max() > .001
            return {**d, "timing": history[-1]["timing"], "joint_history": history, "age_s": age,
                    # Target arrival is a controller diagnostic, not velocity.
                    # After manual recovery it may remain false while joints are still.
                    "moving": bool(measured_motion),
                    "controller_motion_status_code": status[4],
                    "controller_target_reached": (status[4] == 0 if status[4] in (0, 1) else None),
                    "fault": status[1] != 0 or status[6:8] != b"\x00\x00",
                    "can_control": status[0] == 1, "joint_mode": status[2] == 1}

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=1)
        self.bus.close()


class SdkWriter:
    """Writes only after external commissioning. Never enable, reset or change mode.

    A send error may occur after frames left the host. Callers must interrupt
    the group and classify the result as unknown, never repeat this write.
    Interruption holds fresh measured joints; the vendor damped E-stop releases
    position holding and must not be used for software cancellation or cleanup.
    """

    def __init__(self, installation, name):
        import pyAgxArm
        from pyAgxArm import AgxArmFactory, create_agx_arm_config
        m, r = installation.model, installation.robot
        c = installation.calibration.arms[name]
        installation.require_motion()
        if pyAgxArm.__version__ != m.sdk_version:
            raise ValueError("Piper SDK version mismatch")
        # Model joint_names bind URDF joints in CAN motor order. SDK keys are
        # protocol names, independent of the URDF's spelling.
        sdk_limits = {f"joint{index}": [lo, hi] for index, (lo, hi) in
                      enumerate(zip(c.joint_lower, c.joint_upper), 1)}
        config = create_agx_arm_config(robot="piper_x", firmeware_version=m.firmware_profile.lower(),
            channel=installation.host.arms[r.arms[name].binding].interface, interface="socketcan",
            joint_limits=sdk_limits)
        self.calibration = c
        self.arm = AgxArmFactory.create_arm(config)
        self._write_lock = threading.Lock()
        self._stopped = threading.Event()
        try:
            # Pin the implementation that actually supplies move_j, not a wrapper file.
            source = Path(inspect.getsourcefile(self.arm.move_j)).read_bytes()
            if hashlib.sha256(source).hexdigest() != m.sdk_driver_sha256:
                raise ValueError("Piper SDK driver hash mismatch")
            self.arm.set_auto_set_motion_mode_enabled(False)
            self.arm.set_joint_limits_enabled(True)
            self.arm.connect()
            # Bound the SDK's public communication send operation. Its default
            # timeout is None; a stop must not wait indefinitely behind CAN I/O.
            comm = self.arm.get_context().get_comm()
            send = comm.send
            comm.send = lambda msg, timeout=None: send(msg, timeout=.05)
            # The SDK discards firmware fragments until its receive FPS is positive.
            deadline = time.monotonic() + 2.
            while self.arm.get_fps() <= 0:
                if time.monotonic() >= deadline:
                    raise ValueError("SDK feedback did not become ready before firmware query")
                time.sleep(.01)
            observed = self.arm.get_firmware()
            if not isinstance(observed, dict) or observed.get("software_version") != m.firmware_version:
                raise ValueError("Live firmware differs from commissioned firmware")
            self.gripper = self.arm.init_effector("agx_gripper")
        except BaseException:
            self.arm.disconnect()
            raise

    def move(self, joints, opening_m, effort):
        with self._write_lock:
            if self._stopped.is_set():
                raise RuntimeError("Writer stopped; no further targets are admitted")
            c = self.calibration
            q = np.asarray(joints, dtype=float)
            if (q.shape != (6,) or not np.isfinite(q).all()
                    or np.any(q < c.joint_lower) or np.any(q > c.joint_upper)
                    or not c.gripper_closed_m <= opening_m <= c.gripper_open_m
                    or not 0 < effort <= c.gripper_effort):
                raise ValueError("SDK target exceeds commissioned joint or gripper bounds")
            self.arm.move_j(list(joints))
            if self.arm.has_comm_error():
                raise RuntimeError("SDK CAN write outcome is unknown")
            if self._stopped.is_set():
                raise RuntimeError("Stop requested between joint and gripper dispatch")
            self.gripper.move_gripper_m(value=float(opening_m), force=float(effort))
            if self.arm.has_comm_error():
                raise RuntimeError("SDK gripper write outcome is unknown")

    def stop(self, feedback):
        """Latch out motion and hold fresh joints without changing torque mode.

        Read feedback under the write lock, after any in-flight frame batch.
        Preserve the gripper command. Unknown feedback or a failed send remains
        unknown: never fall back to damping, reset, enable, or a cached target.
        """
        self._stopped.set()
        with self._write_lock:
            data = feedback()
            if data["fault"] or not data["can_control"] or not data["joint_mode"]:
                raise RuntimeError("Position hold unavailable in the current controller state")
            q = np.asarray(data["joints_rad"], dtype=float)
            c = self.calibration
            if (q.shape != (6,) or not np.isfinite(q).all()
                    or np.any(q < c.joint_lower) or np.any(q > c.joint_upper)):
                raise RuntimeError("Measured hold joints exceed native limits; no clamping or mode change")
            self.arm.move_j(q.tolist())
            if self.arm.has_comm_error():
                raise RuntimeError("Position-hold transmission outcome is unknown")
        return {"stop_requested": True, "hold_command_sent": True,
                "joint_target_rad": q.tolist(), "feedback_timing": data["timing"],
                "mode": "measured_position_hold", "physical_stop_guaranteed": False}

    def close(self):
        self.arm.disconnect()
