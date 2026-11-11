"""Piper configuration isolation and shared Body/environment contract regression."""
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack, contextmanager
from copy import deepcopy
import json
from pathlib import Path
import sys
import threading
import time
from unittest.mock import patch

import numpy as np
import pytest
from pydantic import ValidationError

from piper_fixtures import CLOCK, TOKEN, Arms, Cameras, body_server, deployment, write
from uni_robot_agent.integrations.piper_x.config import Installation, Host, Robot
from uni_robot_agent.integrations.piper_x.hardware.planning import RobotGeometry, segment_distance
from uni_robot_agent.integrations.piper_x.hardware.kit import host_config
from uni_robot_agent.integrations.piper_x.environment.adapter import Adapter
from uni_robot_agent.integrations.piper_x.environment.control import BodyClient
from uni_robot_agent.integrations.piper_x.body.core.resources import ResourceLocks
from uni_robot_agent.integrations.piper_x.body.host.config import load_host
from uni_robot_agent.services.environment.contracts import (
    EpisodeInfo, PathCommand, PoseCommand, JointTarget, StepRequest, RequestIdentity, CaptureRequest,
)
from uni_robot_agent.services.environment.client import EnvironmentClient
from uni_robot_agent.services.environment.service import EnvironmentService
from uni_robot_agent.services.environment.routes import build_router
from uni_robot_agent.services.hosting import HttpServer, create_app
from uni_robot_agent.artifacts.store import ArtifactStore
from uni_robot_agent.tools.gripper.motion import ClientPrimitiveBackend
from uni_robot_agent.tools.gripper.motion import MotionExecutor
from uni_robot_agent.tools.gripper.contracts import MoveRequest, GripperRequest


@pytest.fixture(autouse=True)
def clock_and_tokens(monkeypatch):
    monkeypatch.setenv("FIXTURE_ARMS_TOKEN", TOKEN)
    monkeypatch.setenv("FIXTURE_CAMERAS_TOKEN", TOKEN)
    monkeypatch.setattr("uni_robot_agent.integrations.piper_x.environment.adapter.host_clock_id", lambda: CLOCK)
    monkeypatch.setattr("uni_robot_agent.integrations.piper_x.environment.observation.host_clock_id", lambda: CLOCK)


def edit_json(path, edit):
    data = json.loads(path.read_text())
    edit(data)
    write(path, data)


def test_four_distinct_configurations_same_implementation(tmp_path):
    installations = [deployment(tmp_path / str(i), i) for i in range(4)]
    assert len({i.model.firmware_profile for i in installations}) == 4
    for i in installations:
        config = i.environment(execute=True)
        assert len(config.arms) == 2 and len(config.cameras) == 3
        assert {c.parent for c in config.cameras.values()} == {"world", *config.arms}
        resolved = host_config([i])
        path = tmp_path / (i.robot.robot_id + "-host.json")
        write(path, resolved)
        assert load_host(path)["host_id"] == i.host.host_id
        public = json.dumps(i.audit())
        assert "USB-" not in public and "CAM-" not in public and str(tmp_path) not in public
    assert installations[1].host.arms["port_0"].interface == "can3"


def test_multiple_deployments_share_one_host_config(tmp_path):
    installations = [deployment(tmp_path / str(n), n) for n in range(4)]
    host = installations[0].host.model_dump(mode="json")
    host.update(bodies={}, arms={}, cameras={})
    for index, i in enumerate(installations):
        robot = json.loads(i.robot_path.read_text())
        for kind in ("arms", "cameras"):
            body = f"robot_{index}_{kind}"
            host["bodies"][body] = i.host.bodies[kind].model_dump(mode="json")
            robot[f"{kind}_body"] = body
            for device in robot[kind].values():
                old = device["binding"]; new = f"robot_{index}_{old}"
                host[kind][new] = getattr(i.host, kind)[old].model_dump(mode="json")
                device["binding"] = new
        calibration = json.loads(i.calibration_path.read_text())
        calibration["host_id"] = host["host_id"]
        robot["calibration"] = write(i.calibration_path, calibration)
        write(i.robot_path, robot)
        manifest = json.loads(i.path.read_text()); manifest["host"] = "../shared-host.json"
        write(i.path, manifest)
    write(tmp_path / "shared-host.json", host)
    installations = [Installation(i.path) for i in installations]
    resolved = host_config(installations)
    assert len(resolved["bodies"]) == 8
    write(tmp_path / "resolved.json", resolved)
    assert len(load_host(tmp_path / "resolved.json")["bodies"]) == 8


@pytest.mark.parametrize("case", ["port", "body_id", "interface", "usb", "camera"])
def test_duplicate_host_resources_rejected(tmp_path, case):
    i = deployment(tmp_path)
    data = i.host.model_dump(mode="json")
    if case in {"port", "body_id"}:
        data["bodies"]["cameras"][case] = data["bodies"]["arms"][case]
    elif case == "camera":
        data["cameras"]["sensor_1"]["serial"] = data["cameras"]["sensor_0"]["serial"]
    elif case == "usb":
        data["arms"]["port_1"]["usb_serial"] = data["arms"]["port_0"]["usb_serial"]
    else:
        data["arms"]["port_1"]["interface"] = data["arms"]["port_0"]["interface"]
    with pytest.raises(ValueError, match="Duplicate"):
        Host.model_validate(data)


def test_wrong_calibration_and_capture_settings_rejected(tmp_path):
    a, b = [deployment(tmp_path / str(n), n) for n in range(2)]
    wrong = write(a.robot_path.parent / "wrong.json", json.loads(b.calibration_path.read_text()))
    edit_json(a.robot_path, lambda r: r.update(calibration=wrong))
    with pytest.raises(ValueError, match="different robot"):
        Installation(a.path)
    b.host.cameras["sensor_0"].color.width += 8
    write(b.host_path, b.host.model_dump(mode="json"))
    with pytest.raises(ValueError, match="capture configuration"):
        Installation(b.path)


@pytest.mark.parametrize("case", ["parent", "role", "required_camera", "body"])
def test_missing_roles_and_wrong_body_rejected(tmp_path, case):
    i = deployment(tmp_path)
    data = i.robot.model_dump(mode="json")
    if case == "parent":
        next(c for c in data["cameras"].values() if c["role"] == "wrist")["parent_arm"] = None
    elif case == "role":
        next(iter(data["arms"].values()))["role"] = "right"
    elif case == "required_camera":
        data["required_cameras"].append("missing")
    else:
        data["arms_body"] = "cameras"
    write(i.robot_path, data)
    with pytest.raises(ValueError):
        Installation(i.path)


def test_incomplete_acceptance_and_station_limits(tmp_path):
    i = deployment(tmp_path)
    cal = json.loads(i.calibration_path.read_text())
    cal["accepted_checks"].remove("independent_stop")
    reference = write(i.calibration_path, cal)
    edit_json(i.robot_path, lambda r: r.update(calibration=reference))
    i = Installation(i.path)
    i.environment(execute=False)
    with pytest.raises(ValueError, match="independent_stop"):
        i.environment(execute=True)
    next(iter(cal["arms"].values()))["joint_lower"][0] = -4.
    reference = write(i.calibration_path, cal)
    edit_json(i.robot_path, lambda r: r.update(calibration=reference))
    with pytest.raises(ValueError, match="limits exceed"):
        Installation(i.path)


def test_file_hash_and_physical_lock(tmp_path):
    i = deployment(tmp_path)
    i.urdf.write_text(i.urdf.read_text() + " ")
    with pytest.raises(ValueError, match="hash mismatch"):
        Installation(i.path)
    lock = ResourceLocks(["physical-resource"], tmp_path / "locks")
    try:
        with pytest.raises(RuntimeError, match="already owned"):
            ResourceLocks(["physical-resource"], tmp_path / "locks")
        other = ResourceLocks(["other-robot-resource"], tmp_path / "locks")
        other.close()
    finally:
        lock.close()


def test_geometry_checks_all_arms_and_table(tmp_path):
    i = deployment(tmp_path)
    geometry = RobotGeometry(i)
    q = {name: np.zeros(6) for name in i.robot.arms}
    geometry.check(q)
    targets = {name: {"hold_joints": [0.]*6} for name in q}
    assert len(geometry.plan(q, targets, segment_m=.03, max_segments=1)) == 2
    targets[next(iter(q))]["hold_joints"][0] = .101
    with pytest.raises(Exception, match="Held joints"):
        geometry.plan(q, targets, segment_m=.03, max_segments=1)
    values = list(i.calibration.arms.values())
    values[1].T_world_from_base = values[0].T_world_from_base
    with pytest.raises(Exception, match="collision"):
        geometry.check(q)
    i.calibration.table_height_m = 1.
    with pytest.raises(Exception, match="table"):
        geometry.check(q)


def test_segment_distance_degenerate_and_parallel():
    def a(x): return np.array(x, float)
    assert segment_distance(a([0,0,0]), a([1,0,0]), a([0,1,0]), a([1,1,0])) == 1
    assert segment_distance(a([0,0,0]), a([0,0,0]), a([1,0,0]), a([2,0,0])) == 1
    assert segment_distance(a([0,0,0]), a([1,1,0]), a([0,1,0]), a([1,0,0])) == 0


@contextmanager
def rig(path, index=0, execute=True):
    i = deployment(path / "deployment", index)
    with ExitStack() as stack:
        arms, cameras = Arms(i), Cameras(i)
        arm_server, arm_url = body_server(stack, arms, path / "arm-journal")
        camera_server, camera_url = body_server(stack, cameras, path / "camera-journal")
        config = i.environment(execute=execute)
        config.arms_body.url, config.cameras_body.url = arm_url, camera_url
        (path / "episode").mkdir()
        adapter = Adapter(config, path / "episode", episode_id=f"episode-{index}")
        stack.callback(adapter.close)
        info = EpisodeInfo(episode_id=f"episode-{index}", task="fixture", integration="piper_x",
                           environment=adapter.description)
        service = EnvironmentService(adapter, info, ArtifactStore(path / "artifacts"))
        stack.callback(service.close)
        service.initialize()
        http = HttpServer(create_app(build_router(service)))
        stack.callback(http.stop)
        url = http.start_background()
        client = EnvironmentClient(url)
        stack.callback(client.close)
        client.connect()
        yield i, arms, cameras, adapter, service, client, arm_server


def held_command(adapter):
    state = adapter.read_state()
    return PathCommand(arms={n: JointTarget(joints=a.joints, gripper=a.gripper_command) for n,a in state.arms.items()},
                       max_segments=1, segment_m=.03)


def test_four_bodies_full_stack_isolation(tmp_path):
    with ExitStack() as stack:
        rigs = [stack.enter_context(rig(tmp_path / str(n), n)) for n in range(4)]
        for i, arms, cameras, adapter, service, client, _ in rigs:
            backend = ClientPrimitiveBackend(client)
            name = next(n for n,a in i.robot.arms.items() if a.role == "left")
            start = backend.read_state().arms[name].control_pose
            feedback = MotionExecutor(backend).move(MoveRequest(targets=[{
                "arm":name, "pose":[start.position[0]+.01, *start.position[1:], 0,0,0]}]))
            assert feedback["pose_reached"] is True
            assert arms.calls == 1
            assert backend.last_reply.snapshot is not None
            assert len(backend.last_reply.snapshot["cameras"]) == 3
            assert backend.last_reply.state_after.native_steps == 1
        victim = rigs[0]
        stop = victim[3].finish("operator_cancelled")["stop"]
        assert stop["status"] == "confirmed_idle"
        assert victim[1].stopped.is_set()
        assert all(not r[1].stopped.is_set() and r[3].control.control_id for r in rigs[1:])


@pytest.mark.parametrize("prefix_reason", ["action_duration", "travel_budget", "workspace"])
def test_acknowledged_prefix_retains_reason_and_allows_new_request(tmp_path, monkeypatch, prefix_reason):
    with rig(tmp_path / "prefix") as (i, arms, cameras, adapter, service, client, _):
        original = arms.invoke

        def bounded(*args, **kwargs):
            result = original(*args, **kwargs)
            return {**result, "requested_targets_reached": False, "prefix_reason": prefix_reason}

        monkeypatch.setattr(arms, "invoke", bounded)
        backend = ClientPrimitiveBackend(client)
        name = next(n for n, arm in i.robot.arms.items() if arm.role == "left")
        for _ in range(2):
            start = backend.read_state().arms[name].control_pose
            feedback = MotionExecutor(backend).move(MoveRequest(targets=[{
                "arm": name, "pose": [start.position[0] + .01, *start.position[1:], 0, 0, 0]}]))
            assert feedback["stop_reason"] == "reachable_prefix"
            assert feedback["targets_reached"] is False
            assert backend.last_reply.feedback["diagnostics"]["prefix_reason"] == prefix_reason
            assert backend.last_reply.feedback["requires_operator_recovery"] is False
        assert arms.calls == 2
        assert not arms.stopped.is_set()


def test_preflight_observation_rejection_preserves_idle_and_next_request(tmp_path, monkeypatch):
    from uni_robot_agent.integrations.piper_x.body.core.contract import DeviceError, ErrorCode
    with rig(tmp_path / "feedback") as (i, arms, cameras, adapter, service, client, _):
        original = arms.invoke
        rejected = False

        def invoke(*args, **kwargs):
            nonlocal rejected
            if not rejected:
                rejected = True
                raise DeviceError(ErrorCode.OBSERVATION_STALE, "Synthetic feedback gap",
                                  execution_outcome="none", stage="preflight")
            return original(*args, **kwargs)

        monkeypatch.setattr(arms, "invoke", invoke)
        backend = ClientPrimitiveBackend(client)
        name = next(n for n, arm in i.robot.arms.items() if arm.role == "left")
        start = backend.read_state().arms[name].control_pose
        request = MoveRequest(targets=[{"arm": name, "pose": [*start.position, 0, 0, 0]}])
        first = MotionExecutor(backend).move(request)
        assert first["stop_reason"] == "planning_rejected" and first["no_execution"]
        assert backend.last_reply.feedback["stop_confirmed"] is True
        assert not arms.stopped.is_set() and arms.calls == 0
        second = MotionExecutor(backend).move(request)
        assert second["pose_reached"] and arms.calls == 1


def test_native_preflight_failure_reaches_model_without_replaying(tmp_path, monkeypatch):
    from uni_robot_agent.integrations.piper_x.body.core.contract import DeviceError, ErrorCode
    with rig(tmp_path / "ik-rejection") as (i, arms, cameras, adapter, service, client, _):
        attempts = []

        def reject(*args, **kwargs):
            attempts.append((args, kwargs))
            raise DeviceError(ErrorCode.LIMIT_VIOLATION, "left: IK failed before dispatch",
                              execution_outcome="none", stage="preflight")

        monkeypatch.setattr(arms, "invoke", reject)
        backend = ClientPrimitiveBackend(client)
        name = next(n for n, arm in i.robot.arms.items() if arm.role == "left")
        start = backend.read_state().arms[name].control_pose
        feedback = MotionExecutor(backend).move(MoveRequest(
            targets=[{"arm": name, "pose": [*start.position, 0, 0, 0]}]))
        assert feedback["stop_reason"] == "planning_rejected"
        assert feedback["no_execution"] is True
        assert feedback["targets_reached"] is False
        evidence = feedback["stop_evidence"]
        assert "left: IK failed before dispatch" in evidence["message"]
        assert backend.last_reply.feedback["execution_outcome"] == "none"
        assert "operations" not in evidence
        assert len(attempts) == 1 and arms.calls == 0
        assert not arms.stopped.is_set()


def test_connect_wrong_robot_body_rejected(tmp_path):
    with rig(tmp_path / "a", 0) as a, rig(tmp_path / "b", 1) as b:
        config = a[3].config.model_copy(deep=True)
        config.arms_body.url = b[3].config.arms_body.url
        out = tmp_path / "wrong"; out.mkdir()
        with pytest.raises(ValueError, match="identity differs"):
            Adapter(config, out, episode_id="wrong")


def test_duplicate_body_request_and_lost_ack_are_not_replayed(tmp_path):
    with rig(tmp_path) as (_, arms, _, adapter, service, client, _):
        original = adapter.client.call
        calls = []
        def lost(method, args=None, **kwargs):
            calls.append(method)
            reply = original(method, args, **kwargs)
            if method == "invoke":
                raise TimeoutError("injected lost response")
            return reply
        adapter.client.call = lost
        reply = client.step(StepRequest(command=held_command(adapter), expected_state_version=0))
        assert reply.feedback["accepted"]
        assert arms.calls == 1
        assert calls.count("invoke") == 1 and "lookup_request" in calls
        record = service.request_status(reply.identity.request_id)
        assert record["status"] == "completed"
        assert service.request_status(reply.identity.request_id) == record
        assert arms.calls == 1


@pytest.mark.parametrize("fault", ["partial", "feedback_lost"])
def test_physical_failure_status_and_blocking(tmp_path, fault):
    with rig(tmp_path) as (_, arms, _, adapter, _, client, _):
        arms.fault = fault
        reply = client.step(StepRequest(command=held_command(adapter), expected_state_version=0))
        assert reply.feedback["execution_outcome"] in {"partial", "unknown"}
        assert not reply.feedback["accepted"]
        assert arms.calls == 1
        refused = client.step(StepRequest(command=held_command(adapter), expected_state_version=reply.state_after.state_version))
        assert refused.feedback["requires_operator_recovery"] and not refused.feedback["accepted"]
        assert arms.calls == 1


def test_lifecycle_stop_does_not_wait_for_motion_lock(tmp_path):
    with rig(tmp_path) as (_, arms, _, adapter, service, client, _):
        arms.fault = "slow"
        command = held_command(adapter)
        with ThreadPoolExecutor(1) as pool:
            motion = pool.submit(client.step, StepRequest(command=command, expected_state_version=0))
            assert arms.started.wait(2)
            started = time.monotonic()
            stop = adapter.finish("operator_cancelled")["stop"]
            assert time.monotonic() - started < 1.5
            assert arms.stopped.is_set()
            assert stop["status"] in {"confirmed_idle", "unknown"}
            assert motion.result(timeout=5).feedback["execution_outcome"] in {"partial", "unknown"}


def test_image_publication_failure_keeps_motion_receipt(tmp_path):
    with rig(tmp_path) as (_, arms, _, adapter, service, client, _):
        with patch.object(service.snapshots, "publish", side_effect=OSError("disk full")):
            reply = client.step(StepRequest(command=held_command(adapter), expected_state_version=0))
        assert reply.feedback["accepted"] and reply.artifact_error
        assert reply.state_after.native_steps == 1 and arms.calls == 1


def test_camera_disconnect_blocks_next_motion(tmp_path):
    with rig(tmp_path) as (_, arms, cameras, adapter, service, client, _):
        cameras.disconnected.add(next(iter(cameras.devices)))
        with pytest.raises(Exception):
            client.capture(CaptureRequest(high_resolution=True))
        with pytest.raises(Exception):
            client.step(StepRequest(command=held_command(adapter), expected_state_version=0))
        assert arms.calls == 0


def test_camera_loss_during_action_requests_group_stop(tmp_path):
    with rig(tmp_path) as (_, arms, cameras, adapter, service, client, _):
        arms.fault = "slow"
        command = held_command(adapter)
        with ThreadPoolExecutor(1) as pool:
            running = pool.submit(client.step, StepRequest(command=command, expected_state_version=0))
            assert arms.started.wait(2)
            cameras.disconnected.add(next(iter(cameras.devices)))
            reply = running.result(timeout=5)
            assert arms.stopped.is_set()
            assert reply.feedback["execution_outcome"] in {"partial", "unknown"}
            assert arms.calls == 1
