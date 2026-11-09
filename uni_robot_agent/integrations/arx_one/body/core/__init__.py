"""Generic Body-server base shared by every hardware kind.

The contract (version 2.0) covers envelopes, stable error codes,
per-device leases with foreground renewal, ``request_key`` idempotency,
durable Operations, ``check`` dry runs and incremental events. It adds a
streaming command channel and a manifest that describes any
device kind (arm, gripper, camera, hand, chassis...). Hardware drivers implement
:class:`~uni_robot_agent.integrations.arx_one.body.core.driver.Driver`; everything else is shared.
"""

from .contract import CONTRACT_VERSION, DeviceError, ErrorCode, Lease, Operation, Principal, new_request_key, unwrap
from .driver import Driver
from .manifest import (
    BodyManifest, Command, DeviceManifest, KIND_ARM, KIND_CAMERA, KIND_CHASSIS, KIND_GRIPPER, KIND_HAND, KIND_WAIST,
    Procedure, Signal, StreamSpec,
)
from .server import BodyServer


__all__ = [
    "CONTRACT_VERSION",
    "BodyManifest",
    "BodyServer",
    "Command",
    "DeviceError",
    "DeviceManifest",
    "Driver",
    "ErrorCode",
    "KIND_ARM",
    "KIND_CAMERA",
    "KIND_CHASSIS",
    "KIND_GRIPPER",
    "KIND_HAND",
    "KIND_WAIST",
    "Lease",
    "Operation",
    "Principal",
    "Procedure",
    "Signal",
    "StreamSpec",
    "new_request_key",
    "unwrap",
]
