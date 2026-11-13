"""Observation startup must not initialize or advertise a motion writer."""
import json
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from piper_fixtures import deployment, timing, write
from uni_robot_agent.integrations.piper_x.config import Installation, Model, digest
from uni_robot_agent.integrations.piper_x.hardware import arms
from uni_robot_agent.integrations.piper_x.hardware.cameras import CameraBody
from uni_robot_agent.integrations.piper_x.body.core.contract import DeviceError
from uni_robot_agent.integrations.piper_x.body.core.arm import contract


def unaccepted_geometry(root, *, motion_checks=False):
    installation = deployment(root)
    collision = json.loads(installation.collision.read_text())
    collision["enclosure_verified"] = False
    model = json.loads(installation.model_path.read_text())
    model["collision"] = write(installation.collision, collision)
    write(installation.model_path, model)
    calibration = json.loads(installation.calibration_path.read_text())
    calibration["model_sha256"] = digest(Model.model_validate(model).model_dump(mode="json"))
    if not motion_checks:
        calibration["accepted_checks"] = []
    robot = json.loads(installation.robot_path.read_text())
    robot["calibration"] = write(installation.calibration_path, calibration)
    write(installation.robot_path, robot)
    return Installation(installation.path)


class Reader:
    def __init__(self, *args, **kwargs):
        self.closed = False

    def read(self):
        return {"agx_gripper": {"width_m": .04}, "joints_rad": [0.] * 6,
                "flange_position_m": [.1, .2, .3], "flange_rpy_rad": [0.] * 3,
                "moving": False, "fault": False, "age_s": .001,
                "timing": timing(), "joint_history": []}

    def close(self):
        self.closed = True


def test_readonly_reads_without_geometry_or_writer_and_exposes_no_motion(tmp_path, monkeypatch):
    installation = unaccepted_geometry(tmp_path)
    geometry = Mock(side_effect=AssertionError("Read-only mode must not open the planner"))
    writer = Mock(side_effect=AssertionError("Read-only mode must not open the SDK writer"))
    monkeypatch.setattr(arms, "RobotGeometry", geometry)
    monkeypatch.setattr(arms, "SdkWriter", writer)
    monkeypatch.setattr(arms, "verify_can", lambda binding: binding.resource())
    monkeypatch.setattr(arms, "PassiveCan", Reader)
    body = arms.ArmBody(installation.path, body_id=installation.host.bodies["arms"].body_id)
    readers = list(body._readers.values())
    try:
        for device in body.manifest().devices:
            assert device.procedures == {}
            assert device.metadata["motion_enabled"] is False
            assert body.read(device.device_id)["signals"]["joint_position_rad"] == [0.] * 6
        from dataclasses import asdict
        description = asdict(body.manifest().devices[0])
        assert contract.problems(description, require_motion=False) == []
        assert any("procedure" in item for item in contract.problems(description))
        with pytest.raises(DeviceError, match="Motion is disabled"):
            body.invoke(list(body._by_device), "move_eef_group", {},
                        cancel=threading.Event(), progress=lambda _: None)
        assert body.estop(next(iter(body._by_device)))["status"] == "not_required"
        geometry.assert_not_called()
        writer.assert_not_called()
    finally:
        body.close()
    assert all(reader.closed for reader in readers)


@pytest.mark.parametrize("motion_checks", [False, True])
def test_execute_still_rejects_unaccepted_configuration_before_opening_feedback_or_writer(
        tmp_path, monkeypatch, motion_checks):
    installation = unaccepted_geometry(tmp_path, motion_checks=motion_checks)
    reader = Mock(side_effect=AssertionError("Rejected startup must not open CAN readers"))
    writer = Mock(side_effect=AssertionError("Rejected startup must not open the SDK writer"))
    monkeypatch.setattr(arms, "verify_can", lambda binding: binding.resource())
    monkeypatch.setattr(arms, "PassiveCan", reader)
    monkeypatch.setattr(arms, "SdkWriter", writer)
    message = "Collision model is unverified" if motion_checks else "Missing station acceptance"
    with pytest.raises(ValueError, match=message):
        arms.ArmBody(installation.path, body_id=installation.host.bodies["arms"].body_id, execute=True)
    reader.assert_not_called()
    writer.assert_not_called()


def test_camera_read_resolves_device_binding_and_rejects_stale_packet():
    body = object.__new__(CameraBody)
    body._lock = threading.Lock()
    body._errors = {}
    body._by_device = {"imager-1": "overview"}
    body.robot = SimpleNamespace(cameras={"overview": SimpleNamespace(binding="camera-usb")})
    body.installation = SimpleNamespace(host=SimpleNamespace(cameras={
        "camera-usb": SimpleNamespace(max_frame_age_s=.5)}))
    body._latest = {"imager-1": {"signals": {"rgb": "test-frame"},
        "timing": {"source_timestamp_ns": str(time.time_ns())}, "freshness": "fresh"}}
    assert body.read("imager-1")["signals"]["rgb"] == "test-frame"
    body._latest["imager-1"]["timing"]["source_timestamp_ns"] = str(time.time_ns()-1_000_000_000)
    with pytest.raises(DeviceError, match="disconnected or stale"):
        body.read("imager-1")
