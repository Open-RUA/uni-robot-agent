"""Dropped incoming frames do not renew or discard the last valid RGB-D pair."""
import threading
from types import SimpleNamespace

import pytest

from uni_robot_agent.integrations.piper_x.hardware.cameras import CameraBody
from uni_robot_agent.integrations.piper_x.body.core.contract import DeviceError


def camera(monkeypatch, age_s):
    obj = object.__new__(CameraBody)
    obj._lock = threading.Lock()
    obj._errors = {"camera": "Rejected RGB-D skew"}
    obj._latest = {"camera": {"timing": {"source_timestamp_ns": str(int((10 - age_s) * 1e9))},
                              "signals": {"rgb": "unchanged", "depth": "same-pair"}}}
    obj._by_device = {"camera": "top"}
    obj.robot = SimpleNamespace(cameras={"top": SimpleNamespace(binding="sensor")})
    obj.installation = SimpleNamespace(host=SimpleNamespace(cameras={"sensor": SimpleNamespace(max_frame_age_s=.5)}))
    monkeypatch.setattr("uni_robot_agent.integrations.piper_x.hardware.cameras.time.time_ns", lambda: 10_000_000_000)
    return obj


def test_dropped_frame_keeps_a_fresh_valid_pair_and_its_original_time(monkeypatch):
    obj = camera(monkeypatch, .1)
    packet = obj.read("camera")
    assert packet["age_s"] == .1
    assert packet["timing"]["source_timestamp_ns"] == "9900000000"
    assert packet["signals"]["depth"] == "same-pair"
    assert packet["collection_warning"] == "Rejected RGB-D skew"
    assert obj.check("camera")["readiness"] == "satisfied"


@pytest.mark.parametrize("age", [.501, -.001])
def test_stale_or_future_pair_is_still_rejected(monkeypatch, age):
    obj = camera(monkeypatch, age)
    with pytest.raises(DeviceError):
        obj.read("camera")
    assert obj.check("camera")["readiness"] == "unsatisfied"


def test_no_valid_frame_is_not_readiness(monkeypatch):
    obj = camera(monkeypatch, .1)
    obj._latest.clear()
    with pytest.raises(DeviceError):
        obj.read("camera")
