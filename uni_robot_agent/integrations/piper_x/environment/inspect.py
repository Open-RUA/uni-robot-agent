"""Receive-only Piper feedback inspection on an already configured SocketCAN bus.

This operator diagnostic does not construct a vendor controller, send CAN frames,
change the interface, or grant motion clearance. See the integration README for scope
and the pinned vendor protocol reference. This file can run standalone using
only the standard library, without installing the harness or native SDK.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import re
import socket
import struct
import time
from typing import Any


FEEDBACK_IDS = tuple(range(0x2A1, 0x2A9))
CAN_FRAME = struct.Struct("=IB3x8s")
CAN_FILTER = struct.Struct("=II")
# CAN_ERR_FLAG must not appear here: SocketCAN treats it as an error subscription.
STANDARD_DATA_MASK = 0xC00007FF
MILLIDEGREE_TO_RAD = math.pi / 180_000
PROTOCOL_REFERENCE = "pyAgxArm@e7aef17d54cac80cbaeb1b4110ab3d8f1337a95b"


class FeedbackWindow:
    """Track each feedback fragment separately, without synthesizing missing data."""

    def __init__(self) -> None:
        self.latest: dict[int, tuple[float, bytes]] = {}
        self.counts: Counter[int] = Counter()
        self.ignored_frames = 0

    def accept(self, frame: bytes, received_at: float) -> None:
        if len(frame) != CAN_FRAME.size:
            self.ignored_frames += 1
            return
        identifier, length, payload = CAN_FRAME.unpack(frame)
        # Extended, remote and error frames must not alias standard feedback IDs.
        if identifier not in FEEDBACK_IDS or length != 8:
            self.ignored_frames += 1
            return
        self.latest[identifier] = (received_at, payload)
        self.counts[identifier] += 1

    def report(self, *, now: float, elapsed: float, max_age: float, max_skew: float) -> dict[str, Any]:
        missing = [identifier for identifier in FEEDBACK_IDS if identifier not in self.latest]
        ages = {identifier: max(0.0, now - stamp) for identifier, (stamp, _) in self.latest.items()}
        stale = [identifier for identifier, age in ages.items() if age > max_age]
        stamps = [stamp for stamp, _ in self.latest.values()]
        skew = max(stamps) - min(stamps) if stamps else None
        recent = not missing and not stale and skew is not None and skew <= max_skew
        return {
            "protocol_reference": PROTOCOL_REFERENCE,
            "expected_robot_family": "piper_x",
            "hardware_identity_verified": False,
            "firmware_verified": False,
            "motion_validated": False,
            "timestamp_basis": "host_monotonic_receive; not device measurement time",
            "elapsed_s": elapsed,
            "feedback_complete_and_recent": recent,
            "max_receive_age_s": max_age,
            "max_receive_skew_s": max_skew,
            "receive_skew_s": skew,
            "missing_ids": [hex(identifier) for identifier in missing],
            "stale_ids": [hex(identifier) for identifier in stale],
            "frames": {
                hex(identifier): {
                    "count": self.counts[identifier],
                    "receive_hz": self.counts[identifier] / elapsed if elapsed > 0 else 0.0,
                    "last_receive_age_s": ages[identifier],
                    "last_payload_hex": payload.hex(),
                }
                for identifier, (_, payload) in sorted(self.latest.items())
            },
            "ignored_frames": self.ignored_frames,
            # Keep incomplete/stale raw evidence, but do not present it as a current pose.
            "decoded_feedback": self._decode() if recent else None,
        }

    def _decode(self) -> dict[str, Any]:
        def pair(identifier: int) -> tuple[int, int]:
            return struct.unpack(">ii", self.latest[identifier][1])

        xyz_rpy = [value for identifier in (0x2A2, 0x2A3, 0x2A4) for value in pair(identifier)]
        joints = [value * MILLIDEGREE_TO_RAD for identifier in (0x2A5, 0x2A6, 0x2A7)
                  for value in pair(identifier)]
        value, force, status, mode = struct.unpack(">ihBB", self.latest[0x2A8][1])
        gripper: dict[str, Any] = {"mode_code": mode, "status_code": status, "force_n": force * .001}
        if mode == 0:
            gripper["width_m"] = value * 1e-6
        elif mode == 1:
            gripper["angle_rad"] = value * MILLIDEGREE_TO_RAD
        else:
            gripper["uninterpreted_value"] = value
        return {
            "joints_rad": joints,
            "flange_position_m": [value * 1e-6 for value in xyz_rpy[:3]],
            "flange_rpy_rad": [value * MILLIDEGREE_TO_RAD for value in xyz_rpy[3:]],
            "pose_frame": "native controller base/flange; no public grasp-site conversion",
            "agx_gripper": gripper,
            "arm_status_payload_hex": self.latest[0x2A1][1].hex(),
        }


def inspect(interface: str, *, duration: float = 1.0, max_age: float = .1,
            max_skew: float = .025) -> dict[str, Any]:
    """Passively sample feedback; no SDK instance or send-capable wrapper is exposed."""
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", interface):
        raise ValueError("Expected a Linux CAN interface name of 1 to 15 characters")
    for name, value in (("duration", duration), ("max_age", max_age), ("max_skew", max_skew)):
        if not math.isfinite(value) or not 0 < value <= 30:
            raise ValueError(f"{name} must be finite, positive and at most 30 seconds")
    if not hasattr(socket, "PF_CAN"):
        raise RuntimeError("Receive-only inspection requires Linux SocketCAN")
    feedback = FeedbackWindow()
    # Raw sockets do not reconfigure bitrate, enable devices or emit application frames.
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        filters = b"".join(CAN_FILTER.pack(identifier, STANDARD_DATA_MASK) for identifier in FEEDBACK_IDS)
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        bus.bind((interface,))
        start = time.monotonic()
        deadline = start + duration
        while (remaining := deadline - time.monotonic()) > 0:
            bus.settimeout(remaining)
            try:
                frame = bus.recv(CAN_FRAME.size)
            except TimeoutError:
                break
            feedback.accept(frame, time.monotonic())
        now = time.monotonic()
    return {
        "schema_version": 1,
        "interface": interface,
        "probe_can_transmit_calls": 0,
        "transmit_scope": "this probe only; other processes may still control this bus",
        **feedback.report(now=now, elapsed=now - start, max_age=max_age, max_skew=max_skew),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--interface", required=True, help="Existing SocketCAN interface; never reconfigured")
    parser.add_argument("--duration", type=float, default=1.0, help="Receive window in seconds (0, 30]")
    parser.add_argument("--max-age", type=float, default=.1, help="Maximum host receive age in seconds")
    parser.add_argument("--max-skew", type=float, default=.025, help="Maximum host receive skew in seconds")
    args = parser.parse_args(argv)
    try:
        report = inspect(args.interface, duration=args.duration, max_age=args.max_age, max_skew=args.max_skew)
    except (OSError, RuntimeError, ValueError) as error:
        print(json.dumps({"error": str(error), "motion_validated": False}))
        return 1
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["feedback_complete_and_recent"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
