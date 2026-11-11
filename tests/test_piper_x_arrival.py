"""Arrival semantics use measured feedback, never a retained controller target bit."""
from collections import deque
import threading
import time
from unittest.mock import Mock

import pytest

from piper_fixtures import timing, deployment
from uni_robot_agent.integrations.piper_x.hardware.transport import PassiveCan, FeedbackUnavailable
from uni_robot_agent.integrations.piper_x.hardware.planning import RobotGeometry
from uni_robot_agent.integrations.piper_x.body.core.arm.arrival import ArrivalWindow


@pytest.mark.parametrize("status,delta,reached,moving", [
    (0, 0., True, False), (1, 0., False, False), (255, 0., None, False),
    (0, .01, True, True), (1, .01, False, True),
])
def test_controller_arrival_and_measured_motion_are_independent(status, delta, reached, moving):
    now = time.time_ns()
    reader = object.__new__(PassiveCan)
    reader.lock = threading.Lock()
    reader.max_age_s, reader.max_skew_s = .1, .025
    reader.history = deque([
        {"joint_position_rad": [q] * 6, "timing": timing(now + offset)}
        for q, offset in ((0., -100_000_000), (delta, 0))
    ])
    payload = bytes([1, 0, 1, 0, status, 0, 0, 0]).hex()
    reader.window = Mock()
    reader.window.report.return_value = {"feedback_complete_and_recent": True,
        "decoded_feedback": {"arm_status_payload_hex": payload}}
    actual = reader.read()
    assert actual["moving"] is moving
    assert actual["controller_target_reached"] is reached
    assert actual["controller_motion_status_code"] == status
    reader.window.report.return_value["feedback_complete_and_recent"] = False
    with pytest.raises(RuntimeError, match="stale"):
        reader.read()


def test_arrival_needs_distinct_consecutive_feedback_not_repeated_polling():
    window = ArrivalWindow()
    assert not window.update(1., True)
    for _ in range(100):
        assert not window.update(1., True)
    assert not window.update(1.025, True)
    assert window.update(1.05, True)
    assert window.samples == 3
    assert not window.update(1.06, False)
    assert not window.update(1.07, True)
    assert not window.update(1.095, True)
    assert window.update(1.12, True)


def test_fragment_gap_waits_for_new_valid_feedback_without_replaying_writes(monkeypatch):
    from uni_robot_agent.integrations.piper_x.hardware import transport
    clock = [0.]
    monkeypatch.setattr(transport.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(transport.time, "sleep", lambda delay: clock.__setitem__(0, clock[0]+delay))
    reader = object.__new__(PassiveCan)
    reader.max_age_s, reader.max_skew_s = .1, .025
    fresh = {"fault": True, "timing": timing()}
    reader._read_once = Mock(side_effect=[FeedbackUnavailable("fragment gap"), fresh])
    assert reader.read() is fresh  # Faults remain visible; only unavailable reads retry.
    assert clock[0] == .001
    assert (reader.max_age_s, reader.max_skew_s) == (.1, .025)


def test_missing_feedback_wait_is_bounded_and_other_failures_are_not_retried(monkeypatch):
    from uni_robot_agent.integrations.piper_x.hardware import transport
    clock = [0.]
    monkeypatch.setattr(transport.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(transport.time, "sleep", lambda delay: clock.__setitem__(0, clock[0]+delay))
    reader = object.__new__(PassiveCan)
    reader.max_age_s, reader.max_skew_s = .1, .025
    reader._read_once = Mock(side_effect=FeedbackUnavailable("stale"))
    with pytest.raises(FeedbackUnavailable, match="stale"):
        reader.read()
    assert clock[0] == pytest.approx(.025)
    reader._read_once = Mock(side_effect=ValueError("malformed"))
    with pytest.raises(ValueError, match="malformed"):
        reader.read()
    reader._read_once.assert_called_once()


def test_feedback_gap_and_old_samples_cannot_certify_arrival():
    window = ArrivalWindow()
    assert not window.update(1., True)
    assert not window.update(1.025, True)
    assert not window.update(1.1, True)
    assert window.samples == 1
    assert not window.update(1.09, True)
    assert not window.update(1.125, True)
    assert window.update(1.15, True)


def test_hold_admission_uses_arx_tolerance_and_holds_fresh_measurement(tmp_path):
    import numpy as np
    from uni_robot_agent.integrations.piper_x.body.core.contract import DeviceError
    geometry = RobotGeometry(deployment(tmp_path))
    measured = {n: np.zeros(6) for n in geometry.installation.robot.arms}
    targets = {n: {"hold_joints": [.02] * 6} for n in measured}
    path = geometry.plan(measured, targets, segment_m=.03, max_segments=1)
    for row in path:
        for q in row.values():
            np.testing.assert_array_equal(q, np.zeros(6))
    next(iter(targets.values()))["hold_joints"] = [.101] * 6
    with pytest.raises(DeviceError, match="Held joints differ"):
        geometry.plan(measured, targets, segment_m=.03, max_segments=1)
