"""Keep SDK camera identities distinct from USB descriptor serials."""
from types import SimpleNamespace

import pytest

from uni_robot_agent.integrations.piper_x.hardware.camera_identity import devices_by_serial, usb_identity


def test_sdk_serial_and_usb_serial_are_separate(tmp_path):
    usb = tmp_path / "sys/devices/usb2/2-4"
    node = usb / "2-4:1.0/video4linux/video0"
    node.mkdir(parents=True)
    for name, value in {"idVendor":"8086", "idProduct":"0b07", "serial":"USB-DESCRIPTOR-ID"}.items():
        (usb / name).write_text(value)
    rs = SimpleNamespace(camera_info=SimpleNamespace(serial_number="serial"))
    device = SimpleNamespace(get_info=lambda _:"SDK-DEVICE-ID")
    found = devices_by_serial(SimpleNamespace(query_devices=lambda:[device]), rs)
    assert list(found) == ["SDK-DEVICE-ID"]
    assert usb_identity(str(node), sysfs=tmp_path/"sys") == {
        "vendor":"8086", "product":"0b07", "serial":"USB-DESCRIPTOR-ID", "topology":"2-4"}
    assert usb_identity(str(tmp_path/"outside"), sysfs=tmp_path/"sys") is None


def test_duplicate_sdk_identity_fails_before_selection():
    rs = SimpleNamespace(camera_info=SimpleNamespace(serial_number="serial"))
    device = SimpleNamespace(get_info=lambda _:"same-sdk-id")
    with pytest.raises(ValueError, match="duplicate"):
        devices_by_serial(SimpleNamespace(query_devices=lambda:[device, device]), rs)
