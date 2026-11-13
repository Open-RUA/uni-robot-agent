"""Command-limit supervision omits geometry, never native command constraints."""
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from piper_fixtures import deployment
from uni_robot_agent.integrations.piper_x.body.core.contract import DeviceError
from uni_robot_agent.integrations.piper_x.hardware.planning import RobotGeometry


def supervised(tmp_path):
    installation = deployment(tmp_path)
    installation.robot.motion_validation = "command_limits"
    installation.calibration.operator_supervised = True
    return installation


def test_default_keeps_geometry_requirement(tmp_path):
    installation = deployment(tmp_path)
    assert installation.robot.motion_validation == "collision"
    collision = json.loads(installation.collision.read_text())
    collision["enclosure_verified"] = False
    installation.collision.write_text(json.dumps(collision))
    with pytest.raises(ValueError, match="unverified"):
        RobotGeometry(installation)


def test_command_limits_requires_supervision_and_reports_scope(tmp_path):
    installation = supervised(tmp_path)
    installation.calibration.operator_supervised = False
    for operation in (installation.require_motion, lambda: RobotGeometry(installation)):
        with pytest.raises(ValueError, match="supervision"):
            operation()
    installation.calibration.operator_supervised = True
    installation.require_motion()
    assert installation.audit()["motion_validation"] == "command_limits"
    assert installation.audit()["collision_checking_enabled"] is False


def test_command_limits_needs_no_enclosure_and_does_not_check_table(tmp_path):
    installation = supervised(tmp_path)
    # Construction must not import mesh collision or require verified tooling.
    installation.collision.write_text("not a collision model")
    installation.calibration.table_height_m = 100
    geometry = RobotGeometry(installation)
    assert geometry.mesh_checks is None
    geometry.check({n: np.zeros(6) for n in installation.robot.arms})
    installation.calibration.accepted_checks.remove("identity")
    with pytest.raises(ValueError, match="identity"):
        installation.require_motion()


@pytest.mark.parametrize("bad", [[0.] * 5, [float("nan")] * 6, [100.] * 6])
def test_invalid_or_out_of_bounds_commands_still_rejected(tmp_path, bad):
    geometry = RobotGeometry(supervised(tmp_path))
    with pytest.raises(DeviceError, match="Joint range"):
        geometry.check({next(iter(geometry.installation.robot.arms)): bad})


def test_command_limit_path_preserves_idle_arm_and_step_bounds(tmp_path):
    geometry = RobotGeometry(supervised(tmp_path))
    current = {n: np.zeros(6) for n in geometry.installation.robot.arms}
    moving = next(iter(current))
    goal = current[moving].copy()
    goal[-1] = .12
    frame = geometry.fk(moving, goal)
    targets = {n: {"hold_joints": q.tolist()} for n, q in current.items()}
    targets[moving] = {"target_position_m": frame[:3, 3],
                       "target_rpy_rad": Rotation.from_matrix(frame[:3, :3]).as_euler("xyz")}
    path = geometry.plan(current, targets, segment_m=.03, max_segments=24)
    assert len(path) >= 2
    for start, end in zip(path, path[1:]):
        assert geometry._within_command_step(start, end)
        assert geometry.check_measured_segment(start, end)
        for name in current.keys() - {moving}:
            np.testing.assert_array_equal(end[name], current[name])
    final = geometry.fk(moving, path[-1][moving])
    np.testing.assert_allclose(final[:3, 3], frame[:3, 3], atol=.001)


@pytest.mark.parametrize("side", [-1, 1])
def test_near_bound_entry_is_monotone_bounded_and_preserves_other_joints(tmp_path, side):
    geometry = RobotGeometry(supervised(tmp_path))
    start = {n: np.zeros(6) for n in geometry.installation.robot.arms}
    arm = next(iter(start))
    start[arm][1] = side * 3.008
    goal = geometry.entry_recovery(start)
    assert goal[arm][1] == pytest.approx(side * 2.97)
    assert geometry.check_measured_segment(start, goal)
    with pytest.raises(DeviceError):
        geometry.check(start)
    for name in start:
        for index in range(6):
            if name != arm or index != 1:
                assert goal[name][index] == start[name][index]
    start[arm][1] = side * 3.021
    with pytest.raises(DeviceError, match="Joint range"):
        geometry.entry_recovery(start)


@pytest.mark.parametrize("side", [-1, 1])
def test_entry_protects_other_boundary_joints_only_on_the_recovering_arm(tmp_path, side):
    geometry = RobotGeometry(supervised(tmp_path))
    start = {n: np.zeros(6) for n in geometry.installation.robot.arms}
    recovering, idle = list(start)
    start[recovering][1:4] = [side * 3.008, -side * 3., side * 2.995]
    start[idle][1:3] = [-3., 3.]
    goal = geometry.entry_recovery(start)
    np.testing.assert_allclose(goal[recovering][1:4], [side * 2.97, -side * 2.97, side * 2.97])
    np.testing.assert_array_equal(goal[idle], start[idle])
    assert geometry.check_measured_segment(start, goal)
    # Returning small residuals on multiple boundary joints still enters the range.
    actual = {name: q.copy() for name, q in goal.items()}
    actual[recovering][1:4] += [side * .012, -side * .0011, side * .001]
    geometry.check(actual)
    # Exact-bound feedback alone never starts a recovery action.
    start[recovering][1] = side * 3.
    assert geometry.entry_recovery(start) is None


@pytest.mark.parametrize("residual", [0., .012, .032])
def test_entry_is_an_acknowledged_prefix_and_retains_gripper(tmp_path, residual):
    import threading
    from unittest.mock import Mock
    from piper_fixtures import timing
    from uni_robot_agent.integrations.piper_x.hardware.arms import ArmBody
    from uni_robot_agent.integrations.piper_x.body.core.arm.tracking import TrackingLimits

    installation = supervised(tmp_path)
    body = object.__new__(ArmBody)
    body.installation, body.robot = installation, installation.robot
    body.geometry = RobotGeometry(installation)
    body.execute = True
    body._blocked = threading.Event()
    body._planning_lock, body._dispatch_lock = threading.Lock(), threading.Lock()
    body._by_device = {a.device_id: n for n, a in body.robot.arms.items()}
    body.tracking_limits = TrackingLimits()
    state = {n: np.zeros(6) for n in body.robot.arms}
    arm = next(iter(state))
    state[arm][1] = -3.008

    def measured(name):
        frame = body.geometry.fk(name, state[name])
        return {"joints_rad": state[name].tolist(), "agx_gripper": {"width_m": .04},
                "flange_position_m": frame[:3, 3].tolist(),
                "flange_rpy_rad": Rotation.from_matrix(frame[:3, :3]).as_euler("xyz").tolist(),
                "timing": timing(), "moving": False}

    body._require_feedback = measured
    body._writers = {n: Mock() for n in state}
    body._stop_all = Mock(return_value={"status": "unknown"})
    for n, writer in body._writers.items():
        def move(q, width, effort, name=n):
            actual = q.copy()
            if name == arm:
                # A small residual must leave the measured joint inside its bound.
                actual[1] -= residual
            state[name] = actual
        writer.move.side_effect = move
    params = {"targets": {device: {"hold_joints": state[name].tolist(), "gripper_opening": 0.}
                          for device, name in body._by_device.items()},
              "max_duration_s": 3., "max_segments": 1, "segment_m": .03}
    if residual > .03:
        with pytest.raises(DeviceError) as failure:
            body.invoke(list(body._by_device), "move_eef_group", params,
                        cancel=threading.Event(), progress=lambda _: None)
        assert failure.value.execution_outcome == "unknown"
        body._stop_all.assert_called_once()
        assert all(writer.move.call_count == 1 for writer in body._writers.values())
        return
    result = body.invoke(list(body._by_device), "move_eef_group", params,
                         cancel=threading.Event(), progress=lambda _: None)
    assert result["prefix_reason"] == "joint_entry_recovery"
    assert not result["requested_targets_reached"]
    assert result["collision_checked"] is False
    assert state[arm][1] == pytest.approx(-2.97 - residual)
    for writer in body._writers.values():
        for call in writer.move.call_args_list:
            assert call.args[1] == pytest.approx(.04)
            assert np.all(np.abs(call.args[0]) <= 3)
