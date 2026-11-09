"""Host kit for the fake driver: no host configuration, files or native build; proves the host needs no robot."""

from __future__ import annotations

from ...host.kit import BaseKit, Build


class FakeKit(BaseKit):
    def probe(self, body):
        args = body["driver_args"]
        if args.get("kind") == "arm":
            return list(args.get("device_ids", ("left_arm", "right_arm"))), ["joint_position_rad", "eef_position_m"]
        return list(args.get("device_ids", ("left", "right"))), ["position"]


kit = FakeKit(name="fake", factory=__package__ + ".driver:driver", build=Build(deb_depends=("python3-pil",)))
