"""RealSense SDK identity and Linux USB identity are separate namespaces."""
from pathlib import Path


def usb_identity(physical_port: str, *, sysfs: Path = Path("/sys")) -> dict | None:
    """Associate a reported V4L node with its USB parent, without guessing a role."""
    path, root = Path(physical_port).resolve(), sysfs.resolve()
    if not path.is_relative_to(root):
        return None
    for parent in (path, *path.parents):
        if not parent.is_relative_to(root):
            break
        if (parent / "idVendor").is_file() and (parent / "idProduct").is_file():
            read = lambda name: (parent / name).read_text().strip()
            return {"vendor": read("idVendor"), "product": read("idProduct"),
                    "serial": read("serial") if (parent / "serial").is_file() else None,
                    "topology": parent.name}
    return None


def devices_by_serial(context, rs) -> dict:
    """Select by the SDK serial; duplicate IDs must not silently replace devices."""
    devices = {}
    for device in context.query_devices():
        serial = device.get_info(rs.camera_info.serial_number)
        if not serial or serial in devices:
            raise ValueError("Missing or duplicate RealSense SDK serial number")
        devices[serial] = device
    return devices


def inventory() -> dict:
    """Enumerate identity only. No pipeline, controller or role selection is opened."""
    import pyrealsense2 as rs
    from importlib.metadata import version
    records = []
    for serial, device in sorted(devices_by_serial(rs.context(), rs).items()):
        port = device.get_info(rs.camera_info.physical_port)
        record = {"sdk_serial": serial, "model": device.get_info(rs.camera_info.name),
                  "firmware": device.get_info(rs.camera_info.firmware_version),
                  "physical_port": port, "usb": usb_identity(port)}
        asic = getattr(rs.camera_info, "asic_serial_number", None)
        if asic is not None and device.supports(asic):
            record["asic_serial"] = device.get_info(asic)
        records.append(record)
    return {"schema_version": 1, "pyrealsense2_version": version("pyrealsense2"),
            "binding_field": "sdk_serial", "role_assignment": "explicit_operator_configuration",
            "streams_started": False, "devices": records}
