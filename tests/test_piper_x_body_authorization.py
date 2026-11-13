"""Device-scoped credentials and restart recovery with synthetic Body devices."""
import json
from unittest.mock import Mock

import pytest

from uni_robot_agent.integrations.piper_x.body.core.contract import (
    DeviceError, ErrorCode, Operation, Principal, unwrap,
)
from uni_robot_agent.integrations.piper_x.body.core.server import BodyServer
from uni_robot_agent.integrations.piper_x.body.drivers.fake.driver import FakeDriver


FULL = Principal("operator", frozenset({"admin"}))
LEFT = Principal("operator", frozenset({"observe", "operate"}), frozenset({"left"}))
EMPTY = Principal("operator", frozenset({"observe", "operate"}), frozenset())


@pytest.fixture
def server():
    driver = FakeDriver()
    driver.estop = Mock(return_value={"physical_stop_guaranteed": False})
    driver.stream_hold = Mock()
    body = BodyServer(driver, watchdog=False)
    try:
        yield body
    finally:
        body.close()
        driver.close()


def call(server, method, args=None, principal=FULL):
    return unwrap(server.call(method, args or {}, principal))


def operation(server, devices, key):
    return server.ops.create(kind="invoke:move", args={}, device_ids=devices,
        principal=FULL.name, request_key=key, fingerprint=key,
        completion_semantics="fake_target_reached")


@pytest.mark.parametrize("acknowledged", [[], ["op_0"], ["op_0", "op_1"]])
def test_restart_restores_all_unknown_pages_and_only_explicit_acknowledgements(tmp_path, acknowledged):
    # The only left-arm records are older than the first 1000 unknown records.
    records = [Operation(operation_id=f"op_{n}", kind="invoke:move",
        device_ids=("left" if n < 2 else "right",), request_key=f"request_{n}",
        principal="operator", command={}, state="unknown", execution_outcome="unknown",
        created_at=f"2026-01-01T00:{n // 60:02d}:{n % 60:02d}Z").to_dict() for n in range(1002)]
    (tmp_path / "operations.jsonl").write_text("".join(json.dumps(r) + "\n" for r in records))
    (tmp_path / "acknowledgements.json").write_text(json.dumps({"body_id": "fake", "records": [
        {"device_id": "left", "operation_id": op_id, "principal": "operator",
         "note": "fixture recovery", "at": "2026-01-01T01:00:00Z"} for op_id in acknowledged]}))
    driver = FakeDriver()
    body = BodyServer(driver, watchdog=False, journal_dir=tmp_path)
    try:
        expected = {"op_0", "op_1"} - set(acknowledged)
        assert body._quarantine.get("left", set()) == expected
        assert len(body._quarantine["right"]) == 1000
        if expected:
            with pytest.raises(DeviceError) as error:
                body._guard_write(["left"])
            assert error.value.code == ErrorCode.QUARANTINED
        else:
            body._guard_write(["left"])
        assert body.ops.get("op_0").state == "unknown"
    finally:
        body.close()
        driver.close()


@pytest.mark.parametrize("principal,expected", [(FULL, ["left", "right"]), (LEFT, ["left"]), (EMPTY, [])])
def test_omitted_stop_scope_only_reaches_authorized_devices(server, principal, expected):
    result = call(server, "estop", principal=principal)
    assert list(result["devices"]) == expected
    assert [c.args[0] for c in server.driver.estop.call_args_list] == expected


@pytest.mark.parametrize("args", [{"device_id": "right"}, {"device_ids": ["left", "right"]}, {"device_ids": []}])
def test_invalid_explicit_stop_scope_is_rejected_before_any_driver_call(server, args):
    with pytest.raises(DeviceError):
        call(server, "estop", args, LEFT)
    server.driver.estop.assert_not_called()


def test_scoped_queries_hide_whole_cross_device_records_before_pagination(server):
    left = operation(server, ["left"], "left")
    for devices, key in [(["right"], "right"), (["left", "right"], "both")]:
        server.ops.finish(operation(server, devices, key).operation_id, {})
    lease = call(server, "acquire_control", {"device_ids": ["right"], "request_key": "lease"})
    stream = call(server, "stream_open", {"device_id": "right", "control_id": lease["control_id"]})
    server.events.emit("global_diagnostic", private_payload="not device scoped")
    server.driver.health = Mock(return_value={"ok": True, "private_right_reading": 42})
    result = call(server, "list_operations", {"limit": 1}, LEFT)
    assert [op["operation_id"] for op in result["operations"]] == [left.operation_id]
    assert result["next_cursor"] is None
    assert call(server, "list_operations", principal=EMPTY)["operations"] == []
    assert [d["device_id"] for d in call(server, "describe", principal=LEFT)["devices"]] == ["left"]
    assert call(server, "describe", principal=EMPTY)["devices"] == []
    assert call(server, "list_leases", principal=LEFT) == {"leases": {}, "streams": {}}
    assert stream["stream_id"] in call(server, "list_leases")["streams"]
    events = call(server, "read_events", {"limit": 1}, LEFT)
    assert len(events["events"]) == 1
    assert events["events"][0]["device_ids"] == ["left"]
    assert call(server, "read_events", principal=EMPTY)["events"] == []
    health = call(server, "get_health", principal=LEFT)
    assert health["devices"] == ["left"] and health["leases"] == 0
    assert health["active_operations"] == 1 and health["open_streams"] == 0
    assert "private_right_reading" not in health["driver"]


@pytest.mark.parametrize("method", ["describe", "list_operations", "read_events"])
def test_explicit_read_scope_cannot_expand_device_access(server, method):
    with pytest.raises(DeviceError) as error:
        call(server, method, {"device_ids": ["right"]}, LEFT)
    assert error.value.code == ErrorCode.FORBIDDEN_DEVICE


def test_describe_preserves_body_identity_for_an_absent_device(server):
    result = call(server, "describe", {"device_ids": ["other-installation-arm"]})
    assert result["body_id"] == "fake"
    assert result["devices"] == []


@pytest.mark.parametrize("method", ["renew_control", "release_control", "quiesce_control"])
def test_same_named_restricted_credential_cannot_mutate_wider_lease(server, method):
    lease = call(server, "acquire_control", {"device_ids": ["right"], "request_key": "lease"})
    with pytest.raises(DeviceError) as error:
        call(server, method, {"control_id": lease["control_id"]}, LEFT)
    assert error.value.code == ErrorCode.FORBIDDEN_DEVICE
    assert server.leases.get(lease["control_id"]).state == "active"
    assert lease["control_id"] not in server._quiesced_controls
    server.driver.stream_hold.assert_not_called()


@pytest.mark.parametrize("method", ["get_operation", "cancel_operation", "lookup_request"])
def test_same_named_restricted_credential_cannot_access_wider_operation(server, method):
    op = operation(server, ["right"], "right")
    lease = call(server, "acquire_control", {"device_ids": ["right"], "request_key": "lease"})
    with pytest.raises(DeviceError) as error:
        call(server, method, {"operation_id": op.operation_id,
             "control_id": lease["control_id"], "request_key": "right"}, LEFT)
    assert error.value.code == ErrorCode.FORBIDDEN_DEVICE
    assert not server.ops.cancel_event(op.operation_id).is_set()


def test_same_named_restricted_credential_cannot_lookup_wider_lease(server):
    call(server, "acquire_control", {"device_ids": ["right"], "request_key": "lease"})
    with pytest.raises(DeviceError) as error:
        call(server, "lookup_request", {"request_key": "lease"}, LEFT)
    assert error.value.code == ErrorCode.FORBIDDEN_DEVICE


@pytest.mark.parametrize("method,closed", [("stream_push", False), ("stream_close", False), ("stream_close", True)])
def test_same_named_restricted_credential_cannot_mutate_wider_stream(server, method, closed):
    lease = call(server, "acquire_control", {"device_ids": ["right"], "request_key": "lease"})
    stream = call(server, "stream_open", {"device_id": "right", "control_id": lease["control_id"]})
    if closed:
        call(server, "stream_close", {"stream_id": stream["stream_id"]})
        server.driver.stream_hold.reset_mock()
    with pytest.raises(DeviceError) as error:
        call(server, method, {"stream_id": stream["stream_id"], "targets": {"position": [.1]}}, LEFT)
    assert error.value.code == ErrorCode.FORBIDDEN_DEVICE
    assert server.streams.get(stream["stream_id"]).pushes == 0
    server.driver.stream_hold.assert_not_called()
