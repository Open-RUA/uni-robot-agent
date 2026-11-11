"""Explicit station limitations and idle release do not fabricate acceptance."""
import json
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from piper_fixtures import deployment, write
from uni_robot_agent.integrations.piper_x.config import Calibration, Installation, Model, digest
from uni_robot_agent.integrations.piper_x.hardware.arms import ArmBody
from uni_robot_agent.integrations.piper_x.hardware.planning import RobotGeometry


def test_skipped_checks_remain_distinct_and_require_supervision(tmp_path):
    installation = deployment(tmp_path)
    cal = installation.calibration.model_dump(mode="json")
    cal["accepted_checks"].remove("independent_stop")
    cal["skipped_checks"] = {"independent_stop": "Operator explicitly declined this test."}
    with pytest.raises(ValueError, match="supervised"):
        Calibration.model_validate(cal)
    cal["operator_supervised"] = True
    installation.calibration = Calibration.model_validate(cal)
    installation.require_motion()
    audit = installation.audit()
    assert "independent_stop" not in audit["accepted_checks"]
    assert "independent_stop" in audit["skipped_checks"]
    cal["accepted_checks"].append("independent_stop")
    with pytest.raises(ValueError, match="both accepted and skipped"):
        Calibration.model_validate(cal)


def test_audit_uses_declared_host_namespace_not_planner_path_resolution(tmp_path):
    installation = deployment(tmp_path)
    fingerprint = installation.audit()["configuration_sha256"]
    installation.host.state_dir = tmp_path / "different-local-path-resolution"
    installation.host.tokens_file = tmp_path / "other-local-token-path"
    installation.host.lock_dir = tmp_path / "other-local-lock-path"
    assert installation.audit()["configuration_sha256"] == fingerprint
    raw = json.loads(installation.host_path.read_text())
    raw["lock_dir"] = "different-declared-lock-directory"
    write(installation.host_path, raw)
    assert Installation(installation.path).audit()["configuration_sha256"] != fingerprint


@pytest.mark.parametrize("field", ["identity", "cameras"])
def test_identity_and_camera_binding_cannot_be_skipped(tmp_path, field):
    cal = deployment(tmp_path).calibration.model_dump(mode="json")
    cal["operator_supervised"] = True
    cal["accepted_checks"].remove(field)
    cal["skipped_checks"] = {field: "Requested skip"}
    with pytest.raises(ValueError):
        Calibration.model_validate(cal)


def test_urdf_scope_still_requires_verified_geometry_and_explicit_acceptance(tmp_path):
    installation = deployment(tmp_path)
    shape = json.loads(installation.collision.read_text())
    shape["scope"] = "urdf_only"
    model = installation.model.model_dump(mode="json")
    model["collision"] = write(installation.collision, shape)
    write(installation.model_path, model)
    cal = installation.calibration.model_dump(mode="json")
    cal["model_sha256"] = digest(Model.model_validate(model).model_dump(mode="json"))
    robot = installation.robot.model_dump(mode="json")
    robot["calibration"] = write(installation.calibration_path, cal)
    write(installation.robot_path, robot)
    installation = Installation(installation.path)
    with pytest.raises(ValueError, match="omitted tooling"):
        RobotGeometry(installation)
    installation.calibration.operator_supervised = True
    installation.calibration.geometry_limitations_accepted = "Operator accepts omitted accessories."
    assert RobotGeometry(installation).collision.scope == "urdf_only"
    shape["enclosure_verified"] = False
    installation.collision.write_text(json.dumps(shape))
    with pytest.raises(ValueError, match="unverified"):
        RobotGeometry(installation)


def body():
    obj = object.__new__(ArmBody)
    obj._by_device = {"left": "left", "right": "right"}
    obj.robot = SimpleNamespace(arms={"left": None, "right": None}, feedback_max_age_s=.1)
    obj.execute = True
    obj._dispatch_lock = threading.Lock()
    obj._planning_lock = threading.Lock()
    obj._blocked = threading.Event()
    obj._require_feedback = Mock(return_value={"moving": False})
    obj._stop_all = Mock(return_value={"status": "requested"})
    return obj


def test_idle_release_retains_completed_targets_without_emergency_stop():
    obj = body()
    assert obj.stream_hold("left")["status"] == "retained_position_hold"
    assert obj.stream_hold("right")["status"] == "retained_position_hold"
    obj._stop_all.assert_not_called()
    assert not obj._blocked.is_set()


@pytest.mark.parametrize("cause", ["dispatch", "moving", "stale", "blocked"])
def test_release_stops_active_or_uncertain_motion(cause):
    obj = body()
    if cause == "dispatch":
        obj._dispatch_lock.acquire()
    elif cause == "moving":
        obj._require_feedback.return_value = {"moving": True}
    elif cause == "stale":
        obj._require_feedback.side_effect = RuntimeError("stale")
    else:
        obj._blocked.set()
    assert obj.stream_hold("left")["status"] == "requested"
    obj._stop_all.assert_called_once()


def test_cancel_during_preflight_retains_hold_and_never_dispatches():
    from concurrent.futures import ThreadPoolExecutor
    from uni_robot_agent.integrations.piper_x.body.core.contract import DeviceError

    obj = body()
    obj.robot.arms = {n: SimpleNamespace(device_id=n) for n in obj._by_device}
    obj._require_feedback.return_value = {"moving": False, "joints_rad": [0.] * 6}
    obj._writers = {n: Mock() for n in obj._by_device}
    started, release, cancel = threading.Event(), threading.Event(), threading.Event()

    def plan(*args, checkpoint, **kwargs):
        started.set()
        assert release.wait(2)
        checkpoint()
        pytest.fail("Cancelled planning must not produce a path")

    obj.geometry = SimpleNamespace(plan=plan)
    params = {"targets": {n: {"hold_joints": [0.] * 6, "gripper_opening": 1.} for n in obj._by_device},
              "segment_m": .01, "max_segments": 24, "max_duration_s": 15.}
    with ThreadPoolExecutor(max_workers=1) as pool:
        result = pool.submit(obj.invoke, list(obj._by_device), "move_eef_group", params,
                             cancel=cancel, progress=lambda _: None)
        assert started.wait(2)
        # Match Body quiesce ordering: close admission/cancel before requesting hold.
        cancel.set()
        assert obj.stream_hold("left")["status"] == "retained_position_hold"
        release.set()
        with pytest.raises(DeviceError, match="cancelled before dispatch") as error:
            result.result(2)
    assert error.value.execution_outcome == "none"
    obj._stop_all.assert_not_called()
    for writer in obj._writers.values():
        writer.move.assert_not_called()


def test_dense_collision_checks_do_not_become_dense_motion_commands(tmp_path, monkeypatch):
    import numpy as np
    from scipy.spatial.transform import Rotation

    geometry = RobotGeometry(deployment(tmp_path))
    current = {n: np.zeros(6) for n in geometry.installation.robot.arms}
    moving = next(iter(current))
    goal = current[moving].copy()
    goal[-1] = .04
    frame = geometry.fk(moving, goal)
    targets = {n: {"hold_joints": q.tolist()} for n, q in current.items()}
    targets[moving] = {"target_position_m": frame[:3, 3],
                       "target_rpy_rad": Rotation.from_matrix(frame[:3, :3]).as_euler("xyz")}
    checks = Mock(wraps=geometry.check)
    monkeypatch.setattr(geometry, "check", checks)
    monkeypatch.setattr(geometry, "ik", lambda *args, **kwargs: goal.copy())
    path = geometry.plan(current, targets, segment_m=.03, max_segments=1)
    assert checks.call_count > len(path) >= 2
    assert len(path) == 2
    np.testing.assert_allclose(path[-1][moving], goal)
    assert geometry._within_command_step(path[0], path[1])
