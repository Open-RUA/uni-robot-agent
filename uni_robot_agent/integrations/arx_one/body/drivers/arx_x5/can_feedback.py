"""Passive feedback for the locked X5 profiles; never transmits CAN.

SocketCAN also delivers other sockets' local TX echoes. They are not motor
feedback. IDs 1/2/4 use MotorType4; IDs 5/6/7/8 return MotorType2 frames on ID 0.
These layouts are tied to the SDK/hardware identity checked by ArxX5Driver.
"""

from __future__ import annotations

import math
import select
import socket
import struct
import threading
import time

MOTOR_IDS = (1, 2, 4, 5, 6, 7, 8)
# Keep the manufacturer's 100 ms threshold for the first three motors. The
# remaining feedback must also satisfy RUA's existing 200 ms freshness bound.
TIMEOUT_S = {motor: 0.1 if motor in (1, 2, 4) else 0.2 for motor in MOTOR_IDS}
# Linux's 64-bit SO_TIMESTAMPNS ABI on the supported ARM64 and AMD64 targets.
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)
SO_RXQ_OVFL = getattr(socket, "SO_RXQ_OVFL", 40)
# Absorb brief reader scheduling delays without losing kernel-stamped frames.
# Freshness thresholds still apply to the original receive time after draining.
RECEIVE_BUFFER_BYTES = 2 * 1024 * 1024
TIMESPEC = struct.Struct("=qq")


def kernel_received_at(ancillary, *, monotonic_now, realtime_now):
    """Convert the kernel RX timestamp, never refresh an old queued frame."""
    for level, kind, data in ancillary:
        if level != socket.SOL_SOCKET or kind != SO_TIMESTAMPNS:
            continue
        if len(data) != TIMESPEC.size:
            break
        seconds, nanos = TIMESPEC.unpack(data)
        if seconds < 0 or not 0 <= nanos < 1_000_000_000:
            break
        age = realtime_now - (seconds + nanos / 1_000_000_000)
        # A future timestamp indicates a clock change or corrupt ancillary data.
        # Do not mark it fresh by clamping an unbounded negative age to zero.
        if not math.isfinite(age) or age < -0.001:
            break
        return monotonic_now - max(0, age)
    raise ValueError("missing or invalid kernel CAN receive timestamp")


def decode_feedback(frame: bytes, flags: int = 0):
    if flags & (socket.MSG_DONTROUTE | socket.MSG_TRUNC) or len(frame) != 16:
        return None
    can_id, size, data = struct.unpack("=IB3x8s", frame)
    if size != 8 or can_id & 0xE0000000:  # extended, RTR, or error frame
        return None
    if can_id in (1, 2, 4) and data[0] >> 5 == 1:
        return can_id, data[0] & 0x1F
    if can_id == 0 and (motor := data[0] & 0xF) in (5, 6, 7, 8):
        # MotorType2 state 1 means enabled; state 0 is disabled, not healthy.
        state = data[0] >> 4
        return motor, 0 if state == 1 else (state or -1)
    return None


class FeedbackWindow:
    """Bounded counters and monotonic freshness, independent of ROS timestamps."""

    def __init__(self, *, clock=time.monotonic):
        self.clock = clock
        self.started_at = clock()
        self.records = {}
        self.lock = threading.Lock()

    def ingest(self, frame: bytes, flags: int = 0, *, received_at=None):
        decoded = decode_feedback(frame, flags)
        if decoded is None:
            return
        motor, fault = decoded
        now = self.clock() if received_at is None else received_at
        with self.lock:
            previous = self.records.get(motor)
            if previous is not None and now < previous["last"]:
                return
            gap = 0 if previous is None else max(0, now - previous["last"])
            restarted = (
                previous is None or gap > TIMEOUT_S[motor] or previous["fault"] != 0
            )
            self.records[motor] = {
                "last": now,
                "count": 1 if previous is None else previous["count"] + 1,
                "fault": fault,
                "healthy_since": now
                if restarted or fault
                else previous["healthy_since"],
                "max_gap_s": max(gap, 0 if previous is None else previous["max_gap_s"]),
                "gap_count": (0 if previous is None else previous["gap_count"])
                + int(previous is not None and gap > TIMEOUT_S[motor]),
            }

    def snapshot(self):
        now = self.clock()
        with self.lock:
            records = {k: dict(v) for k, v in self.records.items()}
        motors = {}
        for motor in MOTOR_IDS:
            record = records.get(motor)
            age = None if record is None else max(0, now - record["last"])
            motors[str(motor)] = {
                "joint": MOTOR_IDS.index(motor) + 1,
                "age_s": None if age is None else round(age, 4),
                "online": age is not None and age <= TIMEOUT_S[motor],
                "fault": None if record is None else record["fault"],
                "frames": 0 if record is None else record["count"],
                "max_gap_s": None if record is None else round(record["max_gap_s"], 4),
                "gap_count": 0 if record is None else record["gap_count"],
            }
        online = all(m["online"] for m in motors.values())
        fault_free = all(m["fault"] == 0 for m in motors.values())
        stable = (
            now - max(r["healthy_since"] for r in records.values())
            if online and fault_free
            else 0
        )
        return {
            "available": True,
            "motors": motors,
            "all_online": online,
            "all_fault_free": fault_free,
            "stable_s": round(max(0, stable), 3),
            "observed_s": round(max(0, now - self.started_at), 3),
            "source": "passive_can_rx_excluding_local_tx",
        }


class CanFeedbackMonitor:
    """Passive receive-only monitor of one CAN interface.

    A bad frame timestamp is an event that expires after ERROR_HOLD_S; a socket
    failure closes the socket, and the receive thread reopens it every
    REOPEN_INTERVAL_S, so neither leaves the monitor unavailable until a restart.
    """

    ERROR_HOLD_S = 1.0
    REOPEN_INTERVAL_S = 0.5

    def __init__(self, interface: str):
        self.interface = interface
        self.window = FeedbackWindow()
        self._error = None  # (message, monotonic time)
        self.stop = threading.Event()
        self.socket = None
        self.receive_lock = threading.Lock()
        self.backlog_pending = False
        self.max_receive_delay_s = 0.0
        self.socket_drops = 0
        self.receive_buffer_bytes = 0
        self._open()
        self.thread = threading.Thread(target=self._receive, name=f"rx-{interface}", daemon=True)
        self.thread.start()

    @property
    def error(self):
        if self.socket is None and self._error is None:
            return "CAN socket is not open"
        if self._error is None:
            return None
        message, at = self._error
        if self.socket is None or time.monotonic() - at < self.ERROR_HOLD_S:
            return message
        return None

    def _fail(self, message):
        self._error = (str(message), time.monotonic())

    def _open(self):
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        except (OSError, AttributeError) as exc:
            self._fail(exc)
            return
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECEIVE_BUFFER_BYTES)
            self.receive_buffer_bytes = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
            sock.setsockopt(socket.SOL_SOCKET, SO_RXQ_OVFL, 1)
            # Exclude unrelated command IDs before they compete for Python time.
            filters = b"".join(
                struct.pack("=II", can_id, 0xC00007FF) for can_id in (0, 1, 2, 4)
            )
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            sock.setblocking(False)
            sock.bind((self.interface,))
        except (OSError, AttributeError) as exc:
            sock.close()
            self._fail(exc)
            return
        self.socket = sock
        self._error = None

    def _close_socket(self):
        with self.receive_lock:
            sock, self.socket = self.socket, None
        if sock is not None:
            sock.close()

    def _receive(self):
        while not self.stop.is_set():
            if self.socket is None:
                if self.stop.wait(self.REOPEN_INTERVAL_S):
                    break
                self._open()
                continue
            try:
                if select.select([self.socket], [], [], 0.02)[0]:
                    self._drain()
            except (OSError, ValueError) as exc:
                if self.stop.is_set():
                    break
                self._fail(exc)
                self._close_socket()

    def _drain(self):
        """Bounded, nonblocking catch-up shared with snapshot's caller.

        The monitor thread can be delayed by Python/ROS work while fresh CAN
        frames are already queued. Drain them before judging freshness, using
        kernel time so a genuinely stale backlog still fails the same threshold.
        One receive lock preserves per-motor order across both readers.
        """
        with self.receive_lock:
            if self.socket is None:
                return
            self.backlog_pending = False
            for _ in range(512):
                try:
                    frame, ancillary, flags, _ = self.socket.recvmsg(
                        16, socket.CMSG_SPACE(TIMESPEC.size) + socket.CMSG_SPACE(4)
                    )
                except BlockingIOError:
                    return
                except OSError as exc:
                    self._fail(exc)
                    raise
                for level, kind, data in ancillary:
                    if level == socket.SOL_SOCKET and kind == SO_RXQ_OVFL and len(data) == 4:
                        self.socket_drops = max(self.socket_drops, struct.unpack("=I", data)[0])
                if decode_feedback(frame, flags) is None:
                    continue
                try:
                    if flags & socket.MSG_CTRUNC:
                        raise ValueError("truncated kernel CAN timestamp")
                    now = time.monotonic()
                    stamp = kernel_received_at(
                        ancillary, monotonic_now=now, realtime_now=time.time()
                    )
                except ValueError as exc:
                    self._fail(exc)
                    continue
                self.max_receive_delay_s = max(self.max_receive_delay_s, now - stamp)
                self.window.ingest(frame, flags, received_at=stamp)
            self.backlog_pending = bool(select.select([self.socket], [], [], 0)[0])

    def snapshot(self):
        try:
            self._drain()
        except (OSError, ValueError) as exc:
            self._fail(exc)
            self._close_socket()
        result = self.window.snapshot()
        result["interface"] = self.interface
        result["timestamp_source"] = "kernel_rx_realtime_to_monotonic"
        result["max_receive_delay_s"] = round(self.max_receive_delay_s, 4)
        result["socket_drops"] = self.socket_drops
        result["receive_buffer_bytes"] = self.receive_buffer_bytes
        result["backlog_pending"] = self.backlog_pending
        if self.error is not None or self.backlog_pending:
            result.update(
                available=False,
                all_online=False,
                stable_s=0,
                error=self.error or "CAN receive backlog exceeds bounded drain",
            )
        return result

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=0.5)
        self._close_socket()


def diagnose_fault(controller: dict, feedback: dict) -> dict:
    """Explain a fault without clearing history or authorizing resumed motion."""
    if not feedback.get("available"):
        kind, reason = (
            "feedback_monitor_unavailable",
            "Per-motor CAN feedback could not be verified.",
        )
    elif not feedback.get("all_online"):
        missing = [k for k, v in feedback["motors"].items() if not v["online"]]
        kind, reason = (
            "motor_feedback_missing",
            "No fresh motor feedback for CAN IDs: " + ", ".join(missing),
        )
    elif not feedback.get("all_fault_free"):
        kind, reason = (
            "motor_fault_active",
            "Fresh CAN feedback reports a motor fault or disabled motor.",
        )
    elif controller.get("vendor_error_count", 0) or controller.get("vendor_errors"):
        codes = controller.get("vendor_errors") or []
        offline_only = codes and all(
            (c & 15) == 15 and 1 <= (c >> 4) <= 6 for c in codes
        )
        if (
            offline_only
            and not controller.get("vendor_errors_truncated")
            and feedback["stable_s"] >= 1
        ):
            kind = "controller_offline_latched"
            reason = "Motor feedback has recovered, but the SDK offline fault is latched; controlled controller restart is required after checking pose and object support."
        else:
            kind, reason = (
                "controller_fault_latched",
                "Controller fault history remains latched; inspect its codes and physical state before recovery.",
            )
    elif (
        not controller.get("configuration")
        or not controller.get("status_source")
        or not controller.get("exclusive_writer")
    ):
        kind, reason = (
            "controller_not_ready",
            "Controller identity, configuration, service, or command ownership check failed.",
        )
    else:
        kind, reason = "none", ""
    return {"kind": kind, "reason": reason, "automatic_resume": False}
