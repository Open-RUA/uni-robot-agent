"""Host kit for ARX X5 arms: CAN and ROS configuration checks, controller hooks, install rules and build inputs."""

from __future__ import annotations

import importlib.util
import json
import re
import subprocess
import sys
from functools import partial
from pathlib import Path

from ...host.kit import BaseKit, Build, InstalledFile

ROS_NAME = r"(?:/[A-Za-z_][A-Za-z0-9_]*)+"
CAN_FIELDS = ("can_interface", "device", "serial", "vendor_id", "product_id", "arm_end_type")


class ArxX5Kit(BaseKit):
    def validate(self, host, config, bodies):
        if set(config) != {"ros_domain_id", "can"}:
            raise ValueError("arx_x5 kit configuration needs exactly ros_domain_id and can")
        domain = config["ros_domain_id"]
        if type(domain) is not int or not 0 <= domain <= 232:
            raise ValueError("ros_domain_id must be an integer in 0..232")
        self._validate_can(config["can"])
        ros_names = set()
        for name, body in bodies.items():
            self._validate_body(name, body["driver_args"], config["can"], ros_names)

    @staticmethod
    def _validate_can(can):
        if not isinstance(can, dict) or not can:
            raise ValueError("arx_x5 needs a nonempty can map")
        seen = {"usb": set(), "interface": set(), "device": set()}
        for key, spec in can.items():
            if not re.fullmatch(r"[A-Za-z0-9_-]+", key) or key == "both":
                raise ValueError(f"invalid CAN key {key!r}")
            if not isinstance(spec, dict) or set(spec) != set(CAN_FIELDS):
                raise ValueError(f"CAN {key}: fields must be exactly {sorted(CAN_FIELDS)}")
            for label, value in (("usb", (spec["vendor_id"], spec["product_id"], spec["serial"])),
                                 ("interface", spec["can_interface"]), ("device", spec["device"])):
                if value in seen[label]:
                    raise ValueError(f"duplicate CAN {label}: {value}")
                seen[label].add(value)

    @staticmethod
    def _validate_body(name, args, can, ros_names):
        from ...core.arm.motion_limits import MotionLimits, validate_publish_hz
        from .driver import validate_arms
        from .limits import controller_profile
        from .safety import TrackingLimits, validate_eef_limit_policy

        MotionLimits.from_config(args.get("motion_limits"))
        TrackingLimits.from_config(args.get("tracking_limits"))
        validate_eef_limit_policy(args.get("eef_limit_policy", "reject"))
        validate_publish_hz(args.get("publish_hz", 50.0))
        arms = args.get("arms", {})
        if not arms:
            raise ValueError(f"{name}: explicit arms map required")
        for spec in arms.values():
            if args.get("profile"):
                if spec.get("profile") not in (None, args["profile"]):
                    raise ValueError("Body and arm controller profiles disagree")
                spec["profile"] = args["profile"]
        validate_arms(arms)
        node = args.get("node_name", "rua_" + name)
        for device, spec in arms.items():
            # Only the Body's own ROS node may publish arm commands.
            if spec.get("writer_node", node) != node:
                raise ValueError(f"{device}: writer_node {spec['writer_node']!r} must be the Body node {node!r}")
            for field in ("status_topic", "command_topic", "controller"):
                if not re.fullmatch(ROS_NAME, spec[field]):
                    raise ValueError(f"{device}: {field} needs an absolute ROS name")
                if spec[field] in ros_names:
                    raise ValueError(f"duplicate ROS name: {spec[field]}")
                ros_names.add(spec[field])
            bus = can.get(spec["can_key"])
            if not bus or bus["can_interface"] != spec["can"]:
                raise ValueError(f"{device}: CAN configuration mismatch")
            if bus["arm_end_type"] != controller_profile(spec).end_type:
                raise ValueError(f"{device}: CAN end type differs from controller profile")

    def resources(self, body):
        return [("can", spec["can"]) for spec in body["driver_args"]["arms"].values()]

    def driver_args(self, host, instance):
        from .host.controller import lifecycle, recover

        configured = host["bodies"][instance]["driver_args"]
        args = {"controller_recovery": partial(recover, host["_path"], instance),
                "controller_lifecycle": partial(lifecycle, host["_path"], instance)}
        if "calibration_path" not in configured:
            args["calibration_path"] = str(Path(host["state_dir"]) / instance / "gripper_calibration.json")
        if "node_name" not in configured:
            args["node_name"] = "rua_" + instance
        return args

    def native_files(self, root):
        return {"controller": root / "native/ros/lib/arx_x5_controller/X5Controller",
                "slcand": root / "native/bin/slcand"}

    def requirements(self):
        return {"rclpy": importlib.util.find_spec("rclpy") is not None}

    def probe(self, body):
        return list(body["driver_args"]["arms"]), ["joint_position_rad"]

    def install_files(self, host, config, operator):
        can = config["can"]
        # Placeholders pass validate; the privileged files need real identities.
        for key, spec in can.items():
            if not re.fullmatch(r"/dev/[A-Za-z0-9_-]+", spec["device"]):
                raise ValueError(f"CAN {key}: invalid device path")
            if not re.fullmatch("[A-Za-z0-9_-]{1,15}", spec["can_interface"]):
                raise ValueError(f"CAN {key}: invalid interface")
            if any(not re.fullmatch("[A-Za-z0-9]+", spec[field]) for field in ("serial", "vendor_id", "product_id")):
                raise ValueError(f"CAN {key}: invalid USB identity")
        sudoers = "".join(f"{operator} ALL=(root) NOPASSWD: /usr/local/sbin/rua-can {action} {key}\n"
                          for action in ("start", "stop", "status") for key in [*can, "both"])
        udev = "".join(
            f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{c["vendor_id"]}", ATTRS{{idProduct}}=="{c["product_id"]}", '
            f'ATTRS{{serial}}=="{c["serial"]}", SYMLINK+="{Path(c["device"]).name}", GROUP="dialout", MODE="0660"\n'
            for c in can.values())
        return [InstalledFile("/etc/rua/arms.json", json.dumps(can, indent=2) + "\n"),
                InstalledFile("/etc/sudoers.d/rua-can", sudoers, 0o440, ("visudo", "-cf", "{}")),
                InstalledFile("/etc/udev/rules.d/99-rua-can.rules", udev)]

    def after_install(self):
        return [("udevadm", "control", "--reload-rules")]

    def check_build(self, kit_dir, lock, target):
        subprocess.run([sys.executable, str(kit_dir / "models/export.py"), "--check"], check=True, capture_output=True)
        if not lock.get("controller_profile"):
            return {}
        from .limits import controller_profile

        profile = controller_profile(lock["controller_profile"])
        sdk = next(a for a in lock["dependencies"]["arx_x5"]["artifacts"] if a["target"] == target)
        if profile.target != target or sdk.get("output_sha256", sdk["sha256"]) != profile.sdk_sha256:
            raise ValueError("Dependency lock and controller profile disagree")
        return {"controller_profile": lock["controller_profile"]}


kit = ArxX5Kit(
    name="arx_x5",
    factory=__package__ + ".driver:driver",
    commands={"controllers": __package__ + ".host.controller:main", "can": __package__ + ".host.can:command"},
    build=Build(
        packages=("host",),
        native_inputs=(("AMD64_J4_90.md", "AMD64_J4_90.md"), ("ros", "ros"), ("models", "models"),
                       ("native", "native")),
        deb_depends=("ros-jazzy-rclpy", "ros-jazzy-kdl-parser", "ros-jazzy-orocos-kdl-vendor", "libusb-1.0-0",
                     "iproute2"),
    ),
)
