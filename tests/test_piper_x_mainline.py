"""Piper native semantics against unchanged mainline tools and session assembly."""
from types import SimpleNamespace
import threading
import time
from unittest.mock import Mock

import numpy as np
import pytest
from pydantic import ValidationError

from piper_fixtures import deployment, timing
from test_piper_x import rig, clock_and_tokens  # noqa: F401 - register the autouse fixture.
from uni_robot_agent import SessionConfig
from uni_robot_agent.integrations.piper_x.hardware.arms import ArmBody
from uni_robot_agent.integrations.piper_x.hardware.planning import RobotGeometry
from uni_robot_agent.integrations.piper_x.body.core.arm.tracking import TrackingLimits
from uni_robot_agent.services.environment.contracts import PathCommand
from uni_robot_agent.tools.gripper.contracts import HoldRequest, MoveRequest
from uni_robot_agent.tools.gripper.motion import ClientPrimitiveBackend, MotionExecutor


def test_mainline_segment_limit_is_enforced_before_dispatch():
    with pytest.raises(ValidationError):
        PathCommand(arms={}, max_segments=25, segment_m=.03)
    with pytest.raises(ValidationError):
        MoveRequest(targets=[{"arm": "left", "pose": [0.] * 6}], max_steps=25)


def test_shared_hold_duration_reaches_native_group(tmp_path, monkeypatch):
    with rig(tmp_path) as (_, arms, _, _, _, client, _):
        invoked = Mock(wraps=arms.invoke)
        monkeypatch.setattr(arms, "invoke", invoked)
        backend = ClientPrimitiveBackend(client)
        feedback = MotionExecutor(backend).hold(HoldRequest(steps=7))
        duration = 7 / backend.description.control_frequency_hz
        assert feedback["requested_duration_s"] == duration
        assert invoked.call_count == 1
        for target in invoked.call_args.args[2]["targets"].values():
            assert target["duration_s"] == duration
            assert "hold_joints" in target


def test_native_joint_hold_runs_for_requested_duration(tmp_path):
    installation = deployment(tmp_path)
    body = object.__new__(ArmBody)
    body.installation, body.robot = installation, installation.robot
    body.geometry = RobotGeometry(installation)
    body.execute = True
    body._blocked = threading.Event()
    body._planning_lock, body._dispatch_lock = threading.Lock(), threading.Lock()
    body._by_device = {a.device_id: name for name, a in body.robot.arms.items()}
    body._writers = {name: Mock() for name in body.robot.arms}
    body.tracking_limits = TrackingLimits()

    def measured(name):
        native = body.geometry.fk(name, np.zeros(6))
        return {"joints_rad": [0.] * 6, "agx_gripper": {"width_m": .04},
                "flange_position_m": native[:3, 3].tolist(), "flange_rpy_rad": [0.] * 3,
                "timing": timing(), "moving": False}

    body._require_feedback = measured
    params = {"targets": {device: {"hold_joints": [0.] * 6, "gripper_opening": .5,
                                  "duration_s": .35} for device in body._by_device},
              "max_duration_s": 3., "max_segments": 1, "segment_m": .03}
    started = time.monotonic()
    result = body.invoke(list(body._by_device), "move_eef_group", params,
                         cancel=threading.Event(), progress=lambda _: None)
    assert time.monotonic() - started >= .35
    assert result["planned_duration_s"] >= .35
    assert result["requested_targets_reached"]
    for writer in body._writers.values():
        assert writer.move.called
        for call in writer.move.call_args_list:
            np.testing.assert_array_equal(call.args[0], np.zeros(6))


def test_wrist_mapping_selects_only_the_acting_arm(tmp_path):
    from uni_robot_agent.tools.gripper.provider import GripperTools
    with rig(tmp_path) as (installation, _, _, adapter, _, _, _):
        provider = object.__new__(GripperTools)
        provider.episode = SimpleNamespace(description=SimpleNamespace(environment=adapter.description))
        for arm in installation.robot.arms:
            selected = provider._action_cameras([arm])
            for name in selected:
                camera = adapter.config.cameras[name]
                assert camera.parent in {"world", arm}
            assert any(adapter.config.cameras[name].parent == arm for name in selected)


@pytest.mark.parametrize("profile,execute", [("direct", True), ("readonly", False)])
def test_official_session_assembles_existing_tools(tmp_path, profile, execute):
    from uni_robot_agent.execution.replanning import ReplanningToolExecutor
    from uni_robot_agent.session import assemble_session
    with rig(tmp_path / "native", execute=execute) as (installation, arms, _, _, _, client, _):
        configuration = SessionConfig(integration="piper_x", tool_profile=profile,
            env_url=client.episode_url, no_sam3=True,
            integration_options={"robot_id": installation.robot.robot_id})
        args = configuration.resolve()
        args.command, args._session_dir = "run", str(tmp_path / "planner")
        assembly = assemble_session(args)
        try:
            names = {item["name"] for item in assembly.executor.get_tool_schemas()}
            assert {"capture_scene", "finish"} <= names
            motion = {"move", "grasp", "release", "hold"}
            assert (motion <= names) if execute else not (motion & names)
            assert isinstance(assembly.executor, ReplanningToolExecutor)
            assert arms.calls == 0
        finally:
            assembly.executor.close()
            assembly.integration_session.close()
            assembly.telemetry.close()
            assembly.artifacts.close()


def test_direct_session_rejects_readonly_environment(tmp_path):
    from uni_robot_agent.session import assemble_session
    with rig(tmp_path / "native", execute=False) as (installation, arms, _, _, _, client, _):
        args = SessionConfig(integration="piper_x", env_url=client.episode_url,
            no_sam3=True, integration_options={"robot_id": installation.robot.robot_id}).resolve()
        args.command, args._session_dir = "run", str(tmp_path / "planner")
        with pytest.raises(ValueError, match="readonly"):
            assemble_session(args)
        assert arms.calls == 0 and not arms.stopped.is_set()
