"""Per-arm model corrections must preserve native command and feedback identity."""
from copy import deepcopy
from unittest.mock import Mock

import numpy as np
import pytest
from pydantic import ValidationError
from scipy.spatial.transform import Rotation

from piper_fixtures import deployment, timing
from uni_robot_agent.integrations.piper_x.config import ArmCalibration
from uni_robot_agent.integrations.piper_x.environment.config import ArmConfig
from uni_robot_agent.integrations.piper_x.environment.kinematics import Kinematics, from_eef
from uni_robot_agent.integrations.piper_x.hardware.arms import ArmBody
from uni_robot_agent.integrations.piper_x.hardware.planning import RobotGeometry


def corrected_rig(tmp_path):
    installation = deployment(tmp_path)
    left, right = installation.robot.arms
    nominal = RobotGeometry(installation)
    q = np.array([.1, -.2, .3, -.1, .2, -.1])
    native = nominal.fk(right, q)
    original_left = nominal.fk(left, q)
    offsets = np.array([0., -.02, .025, .015, -.01, 0.])
    expected = nominal.fk(right, q + offsets)
    installation.calibration.arms[right].kinematic_joint_offsets_rad = offsets.tolist()
    geometry = RobotGeometry(installation)
    kinematics = Kinematics(installation.environment().arms[right])
    return installation, left, right, q, native, original_left, expected, geometry, kinematics


def test_planner_observation_ik_and_link_geometry_share_per_arm_correction(tmp_path):
    installation, left, right, q, native, old_left, expected, geometry, kinematics = corrected_rig(tmp_path)
    assert not np.allclose(expected, native)
    np.testing.assert_allclose(geometry.fk(right, q), expected, atol=1e-12)
    np.testing.assert_allclose(kinematics.controller(q), expected, atol=1e-12)
    np.testing.assert_allclose(geometry.links_at(right, q)[installation.model.tip_link], expected, atol=1e-12)
    np.testing.assert_allclose(geometry.fk(left, q), old_left, atol=1e-12)
    # An already satisfied target returns native q, not q plus its correction.
    np.testing.assert_allclose(geometry.ik(right, expected, q), q, atol=1e-12)
    assert installation.environment().arms[right].joint_lower == installation.calibration.arms[right].joint_lower


def test_native_residual_is_retained_without_double_applying_offsets(tmp_path):
    _, _, _, q, native, _, expected, _, kinematics = corrected_rig(tmp_path)
    residual = from_eef([.001, -.002, .003], [.003, -.002, .001])
    np.testing.assert_allclose(kinematics.correct_feedback(q, native @ residual), expected @ residual, atol=1e-12)


def test_corrected_body_state_tracking_and_arrival_use_same_frame(tmp_path):
    installation, _, right, q, native, _, expected, geometry, kinematics = corrected_rig(tmp_path)
    body = object.__new__(ArmBody)
    body.installation, body.robot, body.geometry = installation, installation.robot, geometry
    body._kinematics = {right: kinematics}
    device = installation.robot.arms[right].device_id
    body._by_device = {device: right}
    data = dict(joints_rad=q.tolist(), flange_position_m=native[:3, 3].tolist(),
                flange_rpy_rad=Rotation.from_matrix(native[:3, :3]).as_euler("xyz").tolist(),
                agx_gripper={"width_m": .04}, moving=False, fault=False, age_s=.001, timing=timing())
    original = deepcopy(data)
    body._readers = {right: Mock(read=Mock(return_value=data))}
    state = body.read(device)["signals"]
    np.testing.assert_allclose(from_eef(state["eef_position_m"], state["eef_rpy_rad"]), expected, atol=1e-12)
    assert state["joint_position_rad"] == q.tolist()
    assert state["native_eef_position_m"] == data["flange_position_m"]
    assert state["native_eef_rpy_rad"] == data["flange_rpy_rad"]
    assert data == original
    tracking = body._tracking_pose(right, data)
    np.testing.assert_allclose(from_eef(tracking[:3], tracking[3:]), expected, atol=1e-12)
    arrived, error = body._arrival_error(right, data, q, held=False)
    assert arrived and error["position_error_m"] < 1e-10


def test_zero_offset_preserves_native_pose_residual_and_values(tmp_path):
    installation = deployment(tmp_path)
    name = next(iter(installation.robot.arms))
    kinematics = Kinematics(installation.environment().arms[name])
    native = from_eef([.2, .3, .4], [.6, -.7, .8])
    np.testing.assert_array_equal(kinematics.correct_feedback([0.] * 6, native), native)


@pytest.mark.parametrize("offsets", [[0.] * 5, [0.] * 7, [float("nan")] * 6])
def test_installation_rejects_malformed_offsets(tmp_path, offsets):
    installation = deployment(tmp_path)
    data = next(iter(installation.calibration.arms.values())).model_dump()
    data["kinematic_joint_offsets_rad"] = offsets
    with pytest.raises(ValidationError):
        ArmCalibration.model_validate(data)


def test_environment_rejects_offset_order_length_mismatch(tmp_path):
    installation = deployment(tmp_path)
    data = next(iter(installation.environment().arms.values())).model_dump()
    data["kinematics"]["joint_offsets_rad"] = [0.] * 5
    with pytest.raises(ValidationError, match="one entry"):
        ArmConfig.model_validate(data)
