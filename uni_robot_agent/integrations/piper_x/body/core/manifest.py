"""Kind-agnostic device manifests (what a Body server declares about its devices).

A :class:`DeviceManifest` is data: signals you can read, commands you can
write (with limits), procedures you can invoke, presets, an optional command
stream, passive observation streams and calibration. Clients discover a Body's
devices from its manifest, so adding a device kind does not change the server.
"""

from __future__ import annotations

import hashlib
from dataclasses import asdict, dataclass, field
from typing import Any

from .contract import CONTRACT_VERSION, canonical_json


KIND_ARM = "arm"
KIND_GRIPPER = "gripper"
KIND_CAMERA = "camera"
KIND_HAND = "hand"
KIND_CHASSIS = "chassis"
KIND_WAIST = "waist"
KNOWN_KINDS = (KIND_ARM, KIND_GRIPPER, KIND_CAMERA, KIND_HAND, KIND_CHASSIS, KIND_WAIST)


# -- JSON-schema helpers ---------------------------------------------------------------------


def obj(properties: dict[str, Any] | None = None, required: tuple[str, ...] | list[str] = (), extra: bool = False) -> dict[str, Any]:
    return {"type": "object", "properties": dict(properties or {}), "required": list(required), "additionalProperties": extra}


def arr(items: dict[str, Any], count: int | None = None, minimum: int = 0, maximum: int = 10000) -> dict[str, Any]:
    return {"type": "array", "items": items, "minItems": minimum if count is None else count, "maxItems": maximum if count is None else count}


def num(minimum: float | None = None, maximum: float | None = None, description: str = "") -> dict[str, Any]:
    out: dict[str, Any] = {"type": "number"}
    if minimum is not None:
        out["minimum"] = minimum
    if maximum is not None:
        out["maximum"] = maximum
    if description:
        out["description"] = description
    return out


def text(minimum: int = 1, maximum: int = 256, description: str = "") -> dict[str, Any]:
    out: dict[str, Any] = {"type": "string", "minLength": minimum, "maxLength": maximum}
    if description:
        out["description"] = description
    return out


def enum(*values: str, description: str = "") -> dict[str, Any]:
    out: dict[str, Any] = {"type": "string", "enum": list(values)}
    if description:
        out["description"] = description
    return out


# -- manifest dataclasses ---------------------------------------------------------------------


@dataclass
class Signal:
    """A readable quantity (``read_state`` returns it under ``signals``)."""

    name: str
    unit: str = ""
    shape: tuple[int, ...] = ()
    description: str = ""


@dataclass
class Command:
    """A writable target; limits are enforced by the server, not only suggested."""

    name: str
    unit: str = ""
    shape: tuple[int, ...] = ()
    minimum: float | list[float] | None = None
    maximum: float | list[float] | None = None
    max_delta_per_s: float | list[float] | None = None
    description: str = ""


@dataclass
class Procedure:
    """A named action the driver executes as an Operation."""

    name: str
    description: str = ""
    params: dict[str, Any] = field(default_factory=dict)  # JSON-schema properties
    required: list[str] = field(default_factory=list)
    is_async: bool = True
    completion: str = "command_stream_completed"
    effects: str = ""
    permission: str = "operate"  # operate | lifecycle | maintenance

    def schema(self) -> dict[str, Any]:
        return obj(self.params, self.required)


@dataclass
class StreamSpec:
    """Continuous command channel for clients that stream setpoints."""

    commands: list[str]
    rate_hz_max: float = 50.0
    watchdog_s: float = 0.25
    description: str = ""


@dataclass
class ObservationStreamSpec:
    """Passive, source-stamped history owned by a device's acquisition loop."""

    signals: list[str]
    retention_s: float = 2.0
    max_samples: int = 512
    max_bytes: int = 1048576
    source_semantics: str = ""


@dataclass
class DeviceManifest:
    device_id: str
    kind: str
    hardware_profile: str = ""
    components: list[str] = field(default_factory=list)
    frames: list[str] = field(default_factory=list)
    signals: dict[str, Signal] = field(default_factory=dict)
    commands: dict[str, Command] = field(default_factory=dict)
    procedures: dict[str, Procedure] = field(default_factory=dict)
    presets: dict[str, list[float]] = field(default_factory=dict)
    stream: StreamSpec | None = None
    calibration: dict[str, Any] | None = None
    unsupported: list[str] = field(default_factory=list)
    limitations: list[str] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)
    observation_streams: dict[str, ObservationStreamSpec] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        for signal in data["signals"].values():
            signal["shape"] = list(signal["shape"])
        for command in data["commands"].values():
            command["shape"] = list(command["shape"])
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> DeviceManifest:
        signals = {k: Signal(**{**v, "shape": tuple(v.get("shape", ()))}) for k, v in (data.get("signals") or {}).items()}
        commands = {k: Command(**{**v, "shape": tuple(v.get("shape", ()))}) for k, v in (data.get("commands") or {}).items()}
        procedures = {k: Procedure(**v) for k, v in (data.get("procedures") or {}).items()}
        stream = data.get("stream")
        return cls(
            device_id=str(data["device_id"]),
            kind=str(data["kind"]),
            hardware_profile=str(data.get("hardware_profile", "")),
            components=list(data.get("components", [])),
            frames=list(data.get("frames", [])),
            signals=signals,
            commands=commands,
            procedures=procedures,
            presets={k: [float(x) for x in v] for k, v in (data.get("presets") or {}).items()},
            stream=StreamSpec(**stream) if stream else None,
            calibration=data.get("calibration"),
            unsupported=list(data.get("unsupported", [])),
            limitations=list(data.get("limitations", [])),
            metadata=dict(data.get("metadata") or {}),
            observation_streams={k: ObservationStreamSpec(**v) for k, v in (data.get("observation_streams") or {}).items()},
        )


@dataclass
class BodyManifest:
    body_id: str
    kind_family: str
    devices: list[DeviceManifest]
    contract_version: str = CONTRACT_VERSION
    description: str = ""

    def device(self, device_id: str) -> DeviceManifest | None:
        for device in self.devices:
            if device.device_id == device_id:
                return device
        return None

    @property
    def device_ids(self) -> list[str]:
        return [d.device_id for d in self.devices]

    @property
    def capability_revision(self) -> str:
        payload = canonical_json([d.to_dict() for d in self.devices])
        return "sha256:" + hashlib.sha256(payload.encode("utf-8")).hexdigest()[:24]

    def to_dict(self) -> dict[str, Any]:
        return {
            "body_id": self.body_id,
            "kind_family": self.kind_family,
            "contract_version": self.contract_version,
            "description": self.description,
            "capability_revision": self.capability_revision,
            "devices": [d.to_dict() for d in self.devices],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> BodyManifest:
        return cls(
            body_id=str(data["body_id"]),
            kind_family=str(data.get("kind_family", "")),
            devices=[DeviceManifest.from_dict(d) for d in data.get("devices", [])],
            contract_version=str(data.get("contract_version", CONTRACT_VERSION)),
            description=str(data.get("description", "")),
        )


def validate_manifest(body: BodyManifest) -> None:
    """Raise ``ValueError`` on an inconsistent manifest (before any hardware is touched)."""
    if not body.body_id:
        raise ValueError("body_id is required")
    seen: set[str] = set()
    for device in body.devices:
        if device.device_id in seen:
            raise ValueError(f"duplicate device_id {device.device_id!r}")
        seen.add(device.device_id)
        if device.kind not in KNOWN_KINDS:
            raise ValueError(f"{device.device_id}: unknown kind {device.kind!r} (known: {', '.join(KNOWN_KINDS)})")
        for name, command in device.commands.items():
            if command.name != name:
                raise ValueError(f"{device.device_id}: command key {name!r} != name {command.name!r}")
            for bound in (command.minimum, command.maximum, command.max_delta_per_s):
                if isinstance(bound, list) and command.shape and len(bound) != command.shape[0]:
                    raise ValueError(f"{device.device_id}.{name}: bound length {len(bound)} != shape {command.shape}")
        for name, procedure in device.procedures.items():
            if procedure.name != name:
                raise ValueError(f"{device.device_id}: procedure key {name!r} != name {procedure.name!r}")
            if procedure.permission not in ("operate", "lifecycle", "maintenance"):
                raise ValueError(f"{device.device_id}.{name}: bad permission {procedure.permission!r}")
        if device.stream is not None:
            missing = [c for c in device.stream.commands if c not in device.commands]
            if missing:
                raise ValueError(f"{device.device_id}: stream commands not declared: {missing}")
        for name, spec in device.observation_streams.items():
            if not name or not spec.signals or any(s not in device.signals for s in spec.signals):
                raise ValueError(f"{device.device_id}: observation stream {name!r} needs declared signals")
            from .observations import ObservationBuffer

            ObservationBuffer(retention_s=spec.retention_s, max_samples=spec.max_samples, max_bytes=spec.max_bytes)
        for name, values in device.presets.items():
            if not values:
                raise ValueError(f"{device.device_id}: preset {name!r} is empty")


__all__ = [
    "KIND_ARM",
    "KIND_CAMERA",
    "KIND_CHASSIS",
    "KIND_GRIPPER",
    "KIND_HAND",
    "KIND_WAIST",
    "KNOWN_KINDS",
    "BodyManifest",
    "Command",
    "DeviceManifest",
    "ObservationStreamSpec",
    "Procedure",
    "Signal",
    "StreamSpec",
    "arr",
    "enum",
    "num",
    "obj",
    "text",
    "validate_manifest",
]
