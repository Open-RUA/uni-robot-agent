"""Linux physical-resource identity, independent of robot and Body names."""
from pathlib import Path

from ..config import CanBinding


def verify_can(binding: CanBinding, *, sysfs=Path("/sys")) -> str:
    device = (sysfs / "class/net" / binding.interface / "device").resolve(strict=True)
    usb = next((p for p in (device, *device.parents) if (p / "idVendor").exists()), None)
    if usb is None:
        raise ValueError("CAN interface has no verifiable USB parent")
    read = lambda p: p.read_text().strip()
    if (read(usb / "idVendor"), read(usb / "idProduct")) != (binding.usb_vendor, binding.usb_product):
        raise ValueError("CAN USB vendor/product mismatch")
    if binding.usb_serial:
        if not (usb / "serial").exists() or read(usb / "serial") != binding.usb_serial:
            raise ValueError("CAN USB serial mismatch")
        matches = [p for p in (sysfs / "bus/usb/devices").glob("*/serial")
                   if read(p) == binding.usb_serial and (p.parent / "idVendor").exists()
                   and read(p.parent / "idVendor") == binding.usb_vendor
                   and read(p.parent / "idProduct") == binding.usb_product]
        if len(matches) != 1:
            raise ValueError("USB serial is not unique; explicitly commission topology binding")
    elif usb.name != binding.usb_topology:
        raise ValueError("CAN USB topology mismatch")
    # Linux net_device dev_id is the driver's channel identifier, not enumeration order.
    channel_file = sysfs / "class/net" / binding.interface / "dev_id"
    if int(read(channel_file), 0) != binding.channel:
        raise ValueError("CAN channel mismatch")
    # Resolve both serial and topology aliases to the same lock key.
    return f"physical-can:{usb.resolve()}:{binding.channel}"
