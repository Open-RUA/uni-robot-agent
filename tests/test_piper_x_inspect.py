"""Protocol fixtures and receive-only transport checks; no hardware is opened."""
import contextlib
import io
import math
from pathlib import Path
import socket
import struct
import subprocess
import sys
import unittest
from unittest.mock import Mock, patch

from uni_robot_agent.integrations.piper_x.environment import inspect as probe


def frame(identifier, *values):
    payload = struct.pack(">ii", *values) if values else b"\0" * 8
    return probe.CAN_FRAME.pack(identifier, 8, payload)


def window():
    result = probe.FeedbackWindow()
    for identifier in probe.FEEDBACK_IDS:
        result.accept(frame(identifier), 1.0)
    return result


def report(feedback, now=1.01):
    return feedback.report(now=now, elapsed=1.0, max_age=.1, max_skew=.025)


class FeedbackChecks(unittest.TestCase):
    def test_signed_units_and_fragment_order(self):
        feedback = window()
        feedback.accept(frame(0x2A2, 123456, -250000), 1.0)
        feedback.accept(frame(0x2A3, 500000, 90000), 1.0)
        feedback.accept(frame(0x2A4, -180000, 45000), 1.0)
        feedback.accept(frame(0x2A5, -90000, 0), 1.0)
        feedback.accept(frame(0x2A6, 180000, -45000), 1.0)
        feedback.accept(frame(0x2A7, 90000, 360000), 1.0)
        actual = report(feedback)["decoded_feedback"]
        for value, expected in zip(actual["flange_position_m"], [.123456, -.25, .5]):
            self.assertAlmostEqual(value, expected)
        for value, expected in zip(actual["flange_rpy_rad"], [math.pi / 2, -math.pi, math.pi / 4]):
            self.assertAlmostEqual(value, expected)
        for value, expected in zip(actual["joints_rad"], [-.5, 0, 1, -.25, .5, 2]):
            self.assertAlmostEqual(value, expected * math.pi)

    def test_gripper_mode_is_not_assumed(self):
        for mode, key, expected in ((0, "width_m", .045), (1, "angle_rad", math.pi / 4),
                                    (7, "uninterpreted_value", 45000)):
            with self.subTest(mode=mode):
                feedback = window()
                payload = struct.pack(">ihBB", 45000, -2000, 64, mode)
                feedback.accept(probe.CAN_FRAME.pack(0x2A8, 8, payload), 1.0)
                actual = report(feedback)["decoded_feedback"]["agx_gripper"]
                self.assertAlmostEqual(actual[key], expected)
                self.assertEqual(actual["force_n"], -2.0)
                if mode != 0:
                    self.assertNotIn("width_m", actual)

    def test_missing_fragment_is_not_zero_filled(self):
        feedback = window()
        del feedback.latest[0x2A6]
        actual = report(feedback)
        self.assertEqual(actual["missing_ids"], ["0x2a6"])
        self.assertFalse(actual["feedback_complete_and_recent"])
        self.assertIsNone(actual["decoded_feedback"])

    def test_each_fragment_must_be_recent(self):
        feedback = window()
        feedback.accept(frame(0x2A5), .5)
        actual = report(feedback)
        self.assertEqual(actual["stale_ids"], ["0x2a5"])
        self.assertIsNone(actual["decoded_feedback"])

    def test_recent_fragments_must_have_bounded_receive_skew(self):
        feedback = window()
        feedback.accept(frame(0x2A5), .96)
        actual = report(feedback)
        self.assertEqual(actual["stale_ids"], [])
        self.assertFalse(actual["feedback_complete_and_recent"])

    def test_flags_lengths_and_unknown_frames_are_rejected(self):
        feedback = probe.FeedbackWindow()
        for value in (frame(0x2A5 | 0x80000000), frame(0x2A5 | 0x40000000),
                      frame(0x2A5 | 0x20000000), frame(0x151), b"short",
                      probe.CAN_FRAME.pack(0x2A5, 7, b"\0" * 8)):
            feedback.accept(value, 1.0)
        self.assertEqual(feedback.ignored_frames, 6)
        self.assertEqual(feedback.latest, {})

    def test_telemetry_does_not_certify_motion_or_identity(self):
        actual = report(window())
        self.assertTrue(actual["feedback_complete_and_recent"])
        for name in ("motion_validated", "hardware_identity_verified", "firmware_verified"):
            self.assertFalse(actual[name])


class TransportChecks(unittest.TestCase):
    def test_standalone_help_without_site_packages(self):
        result = subprocess.run([sys.executable, "-S", str(Path(probe.__file__)), "--help"],
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--interface", result.stdout)

    def run_with_bus(self, bus):
        constants = dict(PF_CAN=29, CAN_RAW=1, SOL_CAN_RAW=101, CAN_RAW_FILTER=1)
        with patch.multiple(socket, create=True, **constants), patch.object(socket, "socket") as factory:
            factory.return_value.__enter__.return_value = bus
            try:
                result = probe.inspect("can1", duration=.005)
            finally:
                factory.return_value.__exit__.assert_called_once()
            return result

    def test_transport_only_receives_and_closes_on_timeout(self):
        bus = Mock(spec=["setsockopt", "bind", "settimeout", "recv"])
        bus.recv.side_effect = [frame(identifier) for identifier in probe.FEEDBACK_IDS] + [TimeoutError()]
        result = self.run_with_bus(bus)
        self.assertTrue(result["feedback_complete_and_recent"])
        self.assertEqual(result["probe_can_transmit_calls"], 0)
        bus.bind.assert_called_once_with(("can1",))
        filters = bus.setsockopt.call_args.args[2]
        self.assertEqual(len(filters), 8 * probe.CAN_FILTER.size)
        self.assertEqual(list(probe.CAN_FILTER.iter_unpack(filters)),
                         [(identifier, 0xC00007FF) for identifier in probe.FEEDBACK_IDS])

    def test_failure_closes_socket(self):
        bus = Mock(spec=["setsockopt", "bind", "settimeout", "recv"])
        bus.recv.side_effect = OSError("interface down")
        with self.assertRaisesRegex(OSError, "interface down"):
            self.run_with_bus(bus)

    def test_invalid_arguments_do_not_open_socket(self):
        with patch.object(socket, "socket") as factory:
            for duration in (float("nan"), float("inf"), 0, -1, 31):
                with self.assertRaises(ValueError):
                    probe.inspect("can1", duration=duration)
            with self.assertRaises(ValueError):
                probe.inspect("can1;command")
            factory.assert_not_called()

    def test_help_does_not_open_socket(self):
        with patch.object(socket, "socket") as factory, contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                probe.main(["--help"])
            self.assertEqual(error.exception.code, 0)
            factory.assert_not_called()


if __name__ == "__main__":
    unittest.main()
