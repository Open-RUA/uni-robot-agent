"""Site manifest: which Body servers exist, how robots are composed, which actors and calibrations bind to them.

Everything a deployment knows that is not code lives here: body URLs or
in-process driver factories, robot compositions (arms / hands / cameras and
their mounts), actor endpoints and the bindings that map an actor's
observation keys and action vector onto concrete devices.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from importlib.resources import files
from pathlib import Path, PurePosixPath
from typing import Any

from ...perception.extrinsics import BoundCalibration, calibration_from_bytes, load_calibration, validate_record


@dataclass(frozen=True)
class DeviceRef:
    body: str
    device_id: str

    @property
    def key(self) -> str:
        return f"{self.body}/{self.device_id}"

    @classmethod
    def parse(cls, text: str, *, default_body: str | None = None) -> DeviceRef:
        text = str(text).strip()
        if "/" in text:
            body, _, device = text.partition("/")
            if not body or not device:
                raise ValueError(f"bad device reference {text!r}; expected body/device_id")
            return cls(body, device)
        if default_body is None:
            raise ValueError(f"device reference {text!r} needs a body prefix (body/device_id)")
        return cls(default_body, text)

    def __str__(self) -> str:
        return self.key


@dataclass(frozen=True)
class SignalRef:
    device: DeviceRef
    signal: str

    @property
    def key(self) -> str:
        return f"{self.device.key}.{self.signal}"

    @classmethod
    def parse(cls, text: str) -> SignalRef:
        text = str(text).strip()
        head, dot, signal = text.rpartition(".")
        if not dot or not head or not signal:
            raise ValueError(f"bad signal reference {text!r}; expected body/device_id.signal")
        return cls(DeviceRef.parse(head), signal)


@dataclass
class HostSpec:
    host_id: str
    address: str
    description: str = ""


@dataclass
class BodySpec:
    name: str
    host: str | None = None
    body_id: str | None = None
    timeout_s: float = 3.0
    kind_family: str = ""
    driver: str | None = None  # module:factory for in-process bodies (fakes, tests)
    driver_args: dict[str, Any] = field(default_factory=dict)
    url: str | None = None  # HTTP Body server
    token_env: str | None = None  # environment variable holding the bearer token
    lease_ttl_s: float = 5.0
    description: str = ""

@dataclass
class CameraMount:
    role: str
    device: DeviceRef
    mount: str = "unknown"
    extrinsics: str | None = None
    frame_motion: str = "unknown"


@dataclass
class RobotSpec:
    robot_id: str
    description: str = ""
    arms: dict[str, DeviceRef] = field(default_factory=dict)  # side -> device
    hands: dict[str, DeviceRef] = field(default_factory=dict)  # side -> device
    cameras: dict[str, CameraMount] = field(default_factory=dict)  # role -> mount
    devices: list[DeviceRef] = field(default_factory=list)  # other devices (chassis, waist...)
    mounts: dict[str, str] = field(default_factory=dict)  # device key -> mount description
    tags: list[str] = field(default_factory=list)
    geometry: dict[str, Any] = field(default_factory=dict)

    def all_devices(self) -> list[DeviceRef]:
        refs: list[DeviceRef] = []
        for ref in [*self.arms.values(), *self.hands.values(), *(m.device for m in self.cameras.values()), *self.devices]:
            if ref not in refs:
                refs.append(ref)
        return refs

    @property
    def bodies(self) -> list[str]:
        return list(dict.fromkeys(ref.body for ref in self.all_devices()))

@dataclass
class ActorSpec:
    name: str
    type: str = "fake"  # fake | pi05_rpc | pi05_ws
    url: str | None = None
    observation_keys: list[str] = field(default_factory=list)
    action_dim: int = 0
    proprio_dim: int | None = None
    rate_hz: float = 20.0
    chunk: int = 5
    options: dict[str, Any] = field(default_factory=dict)
    description: str = ""


@dataclass
class ActionSlice:
    device: DeviceRef
    command: str
    start: int
    stop: int
    rate_hz: float | None = None
    transform: dict[str, list[float]] | None = None  # {"from": [a, b], "to": [c, d]}: linear map of actor units to command units

    @property
    def length(self) -> int:
        return self.stop - self.start

    def apply(self, values: list[float]) -> list[float]:
        if not self.transform:
            return list(values)
        (a, b), (c, d) = self.transform["from"], self.transform["to"]
        if a == b:
            raise ValueError("transform 'from' range is degenerate")
        return [c + (float(v) - a) * (d - c) / (b - a) for v in values]


@dataclass
class Binding:
    actor: str
    robot: str
    observation: dict[str, str] = field(default_factory=dict)  # actor key -> camera role
    proprio: list[SignalRef] = field(default_factory=list)
    action: list[ActionSlice] = field(default_factory=list)
    envelope: dict[str, dict[str, Any]] = field(default_factory=dict)  # device key -> {command: {limits}}
    name: str = ""

    @property
    def action_dim(self) -> int:
        return max((s.stop for s in self.action), default=0)

    def devices(self) -> list[DeviceRef]:
        return list(dict.fromkeys(s.device for s in self.action))


@dataclass
class Site:
    name: str
    bodies: dict[str, BodySpec] = field(default_factory=dict)
    robots: dict[str, RobotSpec] = field(default_factory=dict)
    actors: dict[str, ActorSpec] = field(default_factory=dict)
    bindings: list[Binding] = field(default_factory=list)
    perception: dict[str, Any] = field(default_factory=dict)
    path: Path | None = None
    hosts: dict[str, HostSpec] = field(default_factory=dict)
    calibrations: dict[str, dict[str, Any]] = field(default_factory=dict)

    def calibration(self, name: str):
        """Freeze an explicitly selected record; never connect to a Body."""
        return load_binding(self, name)

    def robot(self, robot_id: str) -> RobotSpec:
        try:
            return self.robots[robot_id]
        except KeyError:
            raise KeyError(f"unknown robot {robot_id!r}; site has {sorted(self.robots)}") from None

    def bindings_for(self, robot_id: str, actor: str | None = None) -> list[Binding]:
        return [b for b in self.bindings if b.robot == robot_id and (actor is None or b.actor == actor)]

    def actors_for(self, robot_id: str) -> list[str]:
        return list(dict.fromkeys(b.actor for b in self.bindings if b.robot == robot_id))


# -- parsing -----------------------------------------------------------------------------------


def _parse_robot(robot_id: str, data: dict[str, Any]) -> RobotSpec:
    cameras = {}
    for role, entry in (data.get("cameras") or {}).items():
        if isinstance(entry, str):
            entry = {"device": entry}
        cameras[role] = CameraMount(role=role, device=DeviceRef.parse(entry["device"]), mount=str(entry.get("mount", "unknown")), extrinsics=entry.get("extrinsics"), frame_motion=str(entry.get("frame_motion", "fixed" if entry.get("mount") in ("torso", "world", "fixed") else "unknown")))
    return RobotSpec(
        robot_id=robot_id,
        description=str(data.get("description", "")),
        arms={str(side): DeviceRef.parse(ref) for side, ref in (data.get("arms") or {}).items()},
        hands={str(side): DeviceRef.parse(ref) for side, ref in (data.get("hands") or {}).items()},
        cameras=cameras,
        devices=[DeviceRef.parse(ref) for ref in (data.get("devices") or [])],
        mounts={str(k): str(v) for k, v in (data.get("mounts") or {}).items()},
        tags=list(data.get("tags") or []),
        geometry=dict(data.get("geometry") or {}),
    )


def _parse_binding(index: int, data: dict[str, Any]) -> Binding:
    slices = []
    for entry in data.get("action") or []:
        start, stop = entry["slice"]
        transform = entry.get("transform")
        if transform is not None:
            transform = {"from": [float(v) for v in transform["from"]], "to": [float(v) for v in transform["to"]]}
            if len(transform["from"]) != 2 or len(transform["to"]) != 2:
                raise ValueError(f"binding action transform for {entry['device']}.{entry['command']} needs from/to pairs")
        slices.append(ActionSlice(device=DeviceRef.parse(entry["device"]), command=str(entry["command"]), start=int(start), stop=int(stop), rate_hz=entry.get("rate_hz"), transform=transform))
    return Binding(
        actor=str(data["actor"]),
        robot=str(data["robot"]),
        observation={str(k): str(v) for k, v in (data.get("observation") or {}).items()},
        proprio=[SignalRef.parse(ref) for ref in (data.get("proprio") or [])],
        action=slices,
        envelope={str(k): dict(v) for k, v in (data.get("envelope") or {}).items()},
        name=str(data.get("name") or f"{data['actor']}->{data['robot']}#{index}"),
    )


def site_from_dict(data: dict[str, Any], *, path: Path | None = None) -> Site:
    hosts = {str(k): HostSpec(str(k), str(v["address"]), str(v.get("description", ""))) for k, v in data.get("hosts", {}).items()}
    bodies = {
        str(name): BodySpec(
            name=str(name),
            host=entry.get("host"),
            body_id=entry.get("body_id"),
            timeout_s=float(entry.get("timeout_s", 3.0)),
            kind_family=str(entry.get("kind_family", "")),
            driver=entry.get("driver"),
            driver_args=dict(entry.get("driver_args") or {}),
            url=entry.get("url") or (f"http://{hosts[entry['host']].address}:{int(entry['port'])}" if entry.get("host") in hosts and entry.get("port") else None),
            token_env=entry.get("token_env"),
            lease_ttl_s=float(entry.get("lease_ttl_s", 5.0)),
            description=str(entry.get("description", "")),
        )
        for name, entry in (data.get("bodies") or {}).items()
    }
    robots = {str(rid): _parse_robot(str(rid), entry or {}) for rid, entry in (data.get("robots") or {}).items()}
    actors = {}
    for name, entry in (data.get("actors") or {}).items():
        entry = entry or {}
        actors[str(name)] = ActorSpec(
            name=str(name),
            type=str(entry.get("type", "fake")),
            url=entry.get("url"),
            observation_keys=list(entry.get("observation") or []),
            action_dim=int(entry.get("action_dim", 0)),
            proprio_dim=entry.get("proprio_dim"),
            rate_hz=float(entry.get("rate_hz", 20.0)),
            chunk=int(entry.get("chunk", 5)),
            options=dict(entry.get("options") or {}),
            description=str(entry.get("description", "")),
        )
    bindings = [_parse_binding(i, entry) for i, entry in enumerate(data.get("bindings") or [])]
    site = Site(name=str(data.get("site", "site")), bodies=bodies, robots=robots, actors=actors, bindings=bindings, perception=dict(data.get("perception") or {}), path=path, hosts=hosts, calibrations=dict(data.get("calibrations") or {}))
    errors = validate_site(site)
    if errors:
        raise ValueError("invalid site manifest:\n  - " + "\n  - ".join(errors))
    return site


def load_site(path: Path | str) -> Site:
    path = Path(path).expanduser().resolve()
    text = path.read_text()
    if path.suffix.lower() in (".yaml", ".yml"):
        import yaml

        data = yaml.safe_load(text) or {}
    else:
        data = json.loads(text)
    return site_from_dict(data, path=path)


def validate_site(site: Site) -> list[str]:
    """Static checks that need no hardware; live manifests are checked by the fleet."""
    errors: list[str] = []
    if site.perception.get("calibration"):
        errors.append("legacy perception.calibration is unsupported; use explicit top-level calibrations bindings")
    endpoints = set()
    for body in site.bodies.values():
        if body.host and body.host not in site.hosts:
            errors.append(f"body {body.name}: unknown host {body.host}")
        if not math.isfinite(body.timeout_s) or body.timeout_s <= 0 or body.timeout_s > 30:
            errors.append(f"body {body.name}: timeout_s must be in (0,30]")
        if not math.isfinite(body.lease_ttl_s) or body.lease_ttl_s <= 0:
            errors.append(f"body {body.name}: lease_ttl_s must be positive and finite")
        if bool(body.url) == bool(body.driver):
            errors.append(f"body {body.name}: configure exactly one of url or driver")
        if body.url and body.url in endpoints:
            errors.append(f"duplicate Body endpoint {body.url}; share the same Body reference instead")
        endpoints.add(body.url)
    for robot in site.robots.values():
        for ref in robot.all_devices():
            if ref.body not in site.bodies:
                errors.append(f"robot {robot.robot_id}: device {ref.key} references unknown body {ref.body!r}")
        for side in list(robot.arms) + list(robot.hands):
            if not side.strip() or "/" in side:
                errors.append(f"robot {robot.robot_id}: device role must be nonempty and contain no slash: {side!r}")
        for camera in robot.cameras.values():
            if camera.frame_motion not in ("fixed", "articulated", "unknown"):
                errors.append(f"robot {robot.robot_id}: invalid camera frame_motion {camera.frame_motion!r}")
    for binding in site.bindings:
        if binding.robot not in site.robots:
            errors.append(f"binding {binding.name}: unknown robot {binding.robot!r}")
            continue
        if binding.actor not in site.actors:
            errors.append(f"binding {binding.name}: unknown actor {binding.actor!r}")
            continue
        robot = site.robots[binding.robot]
        actor = site.actors[binding.actor]
        robot_devices = {ref.key for ref in robot.all_devices()}
        for key, role in binding.observation.items():
            if key not in actor.observation_keys:
                errors.append(f"binding {binding.name}: observation key {key!r} is not declared by actor {actor.name} ({actor.observation_keys})")
            if role not in robot.cameras:
                errors.append(f"binding {binding.name}: camera role {role!r} not in robot {robot.robot_id} ({sorted(robot.cameras)})")
        for key in actor.observation_keys:
            if key not in binding.observation:
                errors.append(f"binding {binding.name}: actor key {key!r} has no camera role")
        for signal in binding.proprio:
            if signal.device.key not in robot_devices:
                errors.append(f"binding {binding.name}: proprio device {signal.device.key} is not part of robot {robot.robot_id}")
        covered = [False] * actor.action_dim
        for s in binding.action:
            if s.device.key not in robot_devices:
                errors.append(f"binding {binding.name}: action device {s.device.key} is not part of robot {robot.robot_id}")
            if s.start < 0 or s.stop <= s.start:
                errors.append(f"binding {binding.name}: bad slice [{s.start}, {s.stop}) for {s.device.key}.{s.command}")
                continue
            if s.stop > actor.action_dim:
                errors.append(f"binding {binding.name}: slice [{s.start}, {s.stop}) exceeds action_dim {actor.action_dim}")
                continue
            for i in range(s.start, s.stop):
                if covered[i]:
                    errors.append(f"binding {binding.name}: action index {i} covered twice")
                covered[i] = True
        if actor.action_dim and not all(covered):
            gaps = [i for i, c in enumerate(covered) if not c]
            errors.append(f"binding {binding.name}: action indices {gaps} are not mapped to any device")
    for name in site.calibrations:
        try:
            site.calibration(name)
        except (ValueError, KeyError, TypeError, OSError) as exc:
            errors.append(f"calibration {name}: {exc}")
    return errors


__all__ = ["ActionSlice", "ActorSpec", "Binding", "BodySpec", "CameraMount", "DeviceRef", "RobotSpec", "SignalRef", "Site", "load_site", "site_from_dict", "validate_site"]


def load_binding(site: Site, name: str) -> BoundCalibration:
    try:
        spec = site.calibrations[name]
    except KeyError:
        raise ValueError(f"no explicit calibration binding named {name!r}") from None
    fields = {"robot", "camera", "target_arm", "record", "sha256", "camera_serial", "mount_id", "source_frame", "target_frame"}
    if not isinstance(spec, dict) or set(spec) != fields:
        raise ValueError(f"calibration {name}: fields must be exactly {sorted(fields)}")
    if any(not isinstance(v, str) or not v.strip() or v != v.strip() for v in spec.values()):
        raise ValueError("calibration binding values must be nonempty trimmed strings")
    if not re.fullmatch(r"[0-9a-f]{64}", spec["sha256"]):
        raise ValueError("calibration binding requires a lowercase SHA-256")
    robot = site.robot(spec["robot"])
    try:
        camera, target = robot.cameras[spec["camera"]], robot.arms[spec["target_arm"]]
    except KeyError as exc:
        raise ValueError("calibration camera/target role is absent from the robot") from exc
    if camera.frame_motion != "fixed":
        raise ValueError("static calibration binding requires a fixed camera; articulated cameras need timed joint transforms")
    if not spec["source_frame"].startswith(camera.device.device_id + "/") or not spec["target_frame"].startswith(target.device_id + "/"):
        raise ValueError("calibration frames must belong to the bound devices")
    location = spec["record"]
    if location.startswith("package:"):
        relative = PurePosixPath(location.removeprefix("package:"))
        if relative.is_absolute() or ".." in relative.parts or not relative.parts:
            raise ValueError("invalid package calibration path")
        resource = files("uni_robot_agent.integrations.arx_one").joinpath(str(relative))
        calibration = calibration_from_bytes(resource.read_bytes(), origin=location, expected_sha256=spec["sha256"])
    else:
        path = Path(location).expanduser()
        if not path.is_absolute():
            if site.path is None:
                raise ValueError("relative calibration path requires a Site file path")
            path = site.path.parent / path
        calibration = load_calibration(path, expected_sha256=spec["sha256"])
    record = calibration.record
    validate_record(record, require_validated=True)
    for key in ("source_frame", "target_frame", "camera_serial", "mount_id"):
        if record[key] != spec[key]:
            raise ValueError(f"calibration {key} differs from the Site binding")
    return BoundCalibration(calibration, camera.device.key, target.key,
                            spec["camera_serial"], spec["mount_id"], spec["source_frame"], spec["target_frame"])
