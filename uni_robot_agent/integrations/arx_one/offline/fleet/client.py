"""Fleet: connect the site's Body servers and resolve robots to device handles."""

from __future__ import annotations

import base64
import importlib
import os
import threading
from collections.abc import Iterable
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from typing import Any

import numpy as np

from ...body.core.client import BaseClient, ControlSession, HttpDeviceClient, InProcessClient
from ...body.core.contract import DeviceError, ErrorCode
from ...body.core.manifest import BodyManifest, DeviceManifest, KIND_ARM, KIND_CAMERA, KIND_HAND
from ...body.core.server import BodyServer
from .site import Binding, BodySpec, DeviceRef, RobotSpec, Site


class BodyConnection:
    def __init__(self, spec: BodySpec, *, http_keepalive: bool = False) -> None:
        self.spec = spec
        self._http_keepalive = http_keepalive
        self.client: BaseClient | None = None
        self.server: BodyServer | None = None
        self.manifest: BodyManifest | None = None
        self.session: ControlSession | None = None
        self.error: str | None = None
        self._session_lock = threading.Lock()
        self._quiescing = False
        self._resources = ExitStack()

    @property
    def available(self) -> bool:
        return self.manifest is not None and self.error is None

    def connect(self) -> None:
        with ExitStack() as cleanup:
            cleanup.callback(self.close)
            spec = self.spec
            if spec.url:
                token = os.environ.get(spec.token_env) if spec.token_env else None
                if spec.token_env and not token:
                    # A Body with a token table would refuse every call; fail with the cause instead.
                    raise ValueError(f"Body {spec.name}: environment variable {spec.token_env} is not set")
                self.client = HttpDeviceClient(spec.url, token=token, timeout_s=spec.timeout_s, persistent=self._http_keepalive)
                self._resources.callback(self.client.close)
            elif spec.driver:
                module_name, _, factory_name = spec.driver.partition(":")
                factory = getattr(importlib.import_module(module_name), factory_name or "driver")
                driver = factory(**spec.driver_args)
                close = getattr(driver, "close", None)
                if callable(close):
                    self._resources.callback(close)
                self.server = BodyServer(driver, lease_ttl_s=spec.lease_ttl_s)
                self._resources.callback(self.server.close)
                self.client = InProcessClient(self.server)
            else:
                raise ValueError(f"body {spec.name}: needs url or driver")
            self.manifest = self.client.manifest()
            if spec.body_id and self.manifest.body_id != spec.body_id:
                raise ValueError(f"Body identity mismatch: expected {spec.body_id}, got {self.manifest.body_id}")
            if spec.host:
                health = self.client.health()
                if health.get("runtime", {}).get("host_id") != spec.host:
                    raise ValueError(f"Body is not installed on expected host {spec.host}")
            self.error = None
            cleanup.pop_all()

    def control(self) -> ControlSession:
        assert self.client is not None
        with self._session_lock:
            if self._quiescing:
                raise RuntimeError("Body connection is quiescing; new control is rejected")
            if self.session is None:
                self.session = ControlSession(self.client)
                self._resources.callback(self.session.close)
                self.session.start()
            return self.session

    def begin_quiesce(self) -> list[str]:
        with self._session_lock:
            self._quiescing = True
            return self.session.begin_quiesce() if self.session else []

    def close(self) -> None:
        try:
            self._resources.close()
        finally:
            self.session = self.server = self.client = self.manifest = None


class DeviceHandle:
    def __init__(self, ref: DeviceRef, body: BodyConnection, manifest: DeviceManifest, *, side: str | None = None, role: str | None = None) -> None:
        self.ref = ref
        self.body = body
        self.manifest = manifest
        self.side = side
        self.role = role

    @property
    def key(self) -> str:
        return self.ref.key

    @property
    def device_id(self) -> str:
        return self.ref.device_id

    @property
    def kind(self) -> str:
        return self.manifest.kind

    @property
    def client(self) -> BaseClient:
        assert self.body.client is not None
        return self.body.client

    def read(self, include: Iterable[str] | None = None, *, max_age_s: float | None = None, on_stale: str = "report", timeout_s: float | None = None) -> dict[str, Any]:
        data = self.client.read_state([self.device_id], include, max_age_s=max_age_s, on_stale=on_stale, timeout_s=timeout_s)
        return data["devices"][self.device_id]

    def control_id(self) -> str:
        return self.body.control().acquire([self.device_id])

    def read_at(self, at: dict, *, stream: str | None = None, **kwargs: Any) -> dict[str, Any]:
        streams = self.manifest.observation_streams
        if stream is None:
            if len(streams) != 1:
                raise DeviceError(ErrorCode.NOT_SUPPORTED, "select one of the device's declared observation streams")
            stream = next(iter(streams))
        return self.client.read_observation_at(self.device_id, stream, at, **kwargs)

    def to_dict(self) -> dict[str, Any]:
        return {"key": self.key, "device_id": self.device_id, "body": self.body.spec.name, "kind": self.kind, "side": self.side, "role": self.role, "components": list(self.manifest.components), "commands": sorted(self.manifest.commands), "procedures": sorted(self.manifest.procedures), "presets": sorted(self.manifest.presets), "stream": None if self.manifest.stream is None else {"commands": self.manifest.stream.commands, "rate_hz_max": self.manifest.stream.rate_hz_max}, "calibration": self.manifest.calibration, "metadata": self.manifest.metadata, "frames": list(self.manifest.frames), "observation_streams": self.manifest.to_dict()["observation_streams"]}


class RobotHandle:
    def __init__(self, spec: RobotSpec, fleet: Fleet) -> None:
        self.spec = spec
        self.fleet = fleet
        self.devices: dict[str, DeviceHandle] = {}
        self.arms: dict[str, DeviceHandle] = {}
        self.hands: dict[str, DeviceHandle] = {}
        self.cameras: dict[str, DeviceHandle] = {}
        self.degraded: dict[str, str] = {}  # camera role -> reason (camera body down)
        for side, ref in spec.arms.items():
            self.arms[side] = self._attach(ref, KIND_ARM, side=side)
        for side, ref in spec.hands.items():
            self.hands[side] = self._attach(ref, KIND_HAND, side=side)
        for role, mount in spec.cameras.items():
            body = fleet.bodies.get(mount.device.body)
            if body is None or not body.available:
                self.degraded[role] = f"camera body {mount.device.body} unavailable: {getattr(body, 'error', 'unknown body')}"
                continue
            self.cameras[role] = self._attach(mount.device, KIND_CAMERA, role=role)
        for ref in spec.devices:
            self._attach(ref, None)

    def _attach(self, ref: DeviceRef, expected_kind: str | None, *, side: str | None = None, role: str | None = None) -> DeviceHandle:
        body = self.fleet.bodies.get(ref.body)
        if body is None or body.manifest is None:
            raise ValueError(f"robot {self.spec.robot_id}: body {ref.body!r} is not connected")
        manifest = body.manifest.device(ref.device_id)
        if manifest is None:
            raise ValueError(f"robot {self.spec.robot_id}: body {ref.body} has no device {ref.device_id!r} (has {body.manifest.device_ids})")
        if expected_kind is not None and manifest.kind != expected_kind:
            raise ValueError(f"robot {self.spec.robot_id}: {ref.key} is kind {manifest.kind!r}, expected {expected_kind!r}")
        handle = self.devices.get(ref.key) or DeviceHandle(ref, body, manifest, side=side, role=role)
        self.devices[ref.key] = handle
        return handle

    @property
    def robot_id(self) -> str:
        return self.spec.robot_id

    @property
    def kinds(self) -> set[str]:
        return {d.kind for d in self.devices.values()}

    @property
    def bodies(self) -> list[BodyConnection]:
        return list(dict.fromkeys(d.body for d in self.devices.values()))

    def resolve(self, selector: str, kind: str | None = None) -> DeviceHandle:
        """Accept a device key, a bare device id, an arm/hand side or a camera role."""
        if selector in self.devices and (kind is None or self.devices[selector].kind == kind):
            return self.devices[selector]
        if kind in (None, KIND_ARM) and selector in self.arms:
            return self.arms[selector]
        if kind in (None, KIND_HAND) and selector in self.hands:
            return self.hands[selector]
        if kind in (None, KIND_CAMERA) and selector in self.cameras:
            return self.cameras[selector]
        matches = [d for d in self.devices.values() if d.device_id == selector and (kind is None or d.kind == kind)]
        if len(matches) == 1:
            return matches[0]
        options = sorted(self.devices)
        if kind == KIND_ARM:
            options = sorted(self.arms)
        elif kind == KIND_HAND:
            options = sorted(self.hands)
        elif kind == KIND_CAMERA:
            options = sorted(self.cameras)
        raise DeviceError(ErrorCode.UNKNOWN_DEVICE, f"robot {self.robot_id} has no {kind or 'device'} {selector!r}", details={"options": options})

    def read_all(self, *, kinds: Iterable[str] | None = None, include: Iterable[str] | None = None) -> dict[str, dict[str, Any]]:
        """One read_state per body; camera images are excluded unless asked for."""
        wanted_kinds = set(kinds) if kinds else None
        out: dict[str, dict[str, Any]] = {}
        per_body: dict[str, list[DeviceHandle]] = {}
        for handle in self.devices.values():
            if wanted_kinds is not None and handle.kind not in wanted_kinds:
                continue
            if wanted_kinds is None and handle.kind == KIND_CAMERA:
                continue
            per_body.setdefault(handle.body.spec.name, []).append(handle)
        for handles in per_body.values():
            client = handles[0].client
            try:
                data = client.read_state([h.device_id for h in handles], include)
            except DeviceError as exc:
                for h in handles:
                    out[h.key] = {"availability": "error", "error": exc.to_dict()}
                continue
            for h in handles:
                out[h.key] = data["devices"].get(h.device_id, {"availability": "missing"})
        return out

    def frames(self, roles: Iterable[str] | None = None) -> dict[str, bytes]:
        """Independent latest RGB reads; use ObservationService for a shared RGB-D snapshot."""
        out: dict[str, bytes] = {}
        for role in roles or self.cameras:
            handle = self.cameras[role]
            try:
                reading = handle.read(["rgb"], max_age_s=0.5, on_stale="error")
            except DeviceError:
                continue
            rgb = (reading.get("signals") or {}).get("rgb")
            if rgb and rgb.get("data_b64"):
                out[role] = base64.b64decode(rgb["data_b64"])
        return out

    def depth(self, role: str) -> tuple[np.ndarray, dict[str, Any]]:
        """Independent latest depth read; not paired with a preceding frames() call."""
        reading = self.cameras[role].read(["depth"])
        depth = (reading.get("signals") or {}).get("depth")
        if not depth:
            raise DeviceError(ErrorCode.NOT_SUPPORTED, f"camera {role} has no depth")
        array = np.frombuffer(base64.b64decode(depth["data_b64"]), dtype="<u2").reshape(int(depth["height"]), int(depth["width"])).astype(np.uint16)
        return array, {k: v for k, v in depth.items() if k != "data_b64"}

    def intrinsics(self, role: str) -> dict[str, Any]:
        """Independent latest intrinsics read; a frozen observation retains its own."""
        reading = self.cameras[role].read(["intrinsics"])
        return (reading.get("signals") or {}).get("intrinsics") or {}

    def check(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        per_body: dict[str, list[DeviceHandle]] = {}
        for handle in self.devices.values():
            per_body.setdefault(handle.body.spec.name, []).append(handle)
        for handles in per_body.values():
            data = handles[0].client.check([h.device_id for h in handles])
            for h in handles:
                out[h.key] = data["devices"].get(h.device_id)
        return out

    def describe(self) -> dict[str, Any]:
        describes = {body.spec.name: body.client.describe() for body in self.bodies if body.client is not None}
        devices = []
        for handle in self.devices.values():
            live = next((d for d in describes[handle.body.spec.name]["devices"] if d["device_id"] == handle.device_id), {})
            devices.append({**handle.to_dict(), "control": live.get("control"), "quarantined": live.get("quarantined", []), "active_stream": live.get("active_stream"), "calibration": live.get("calibration")})
        return {
            "robot_id": self.robot_id,
            "description": self.spec.description,
            "arms": {side: h.key for side, h in self.arms.items()},
            "hands": {side: h.key for side, h in self.hands.items()},
            "cameras": {role: {"device": h.key, "mount": self.spec.cameras[role].mount, "extrinsics": self.spec.cameras[role].extrinsics} for role, h in self.cameras.items()},
            "kinds": sorted(self.kinds),
            "degraded": dict(self.degraded),
            "devices": devices,
            "actors": self.fleet.site.actors_for(self.robot_id),
            "bodies": {body.spec.name: {"kind_family": body.manifest.kind_family if body.manifest else None, "url": body.spec.url or "in-process", "capability_revision": body.manifest.capability_revision if body.manifest else None} for body in self.bodies},
        }


class Fleet:
    """All Body connections of a site plus per-body foreground control sessions."""

    def __init__(self, site: Site, *, strict: bool = False) -> None:
        self.site = site
        self.strict = strict
        self.bodies: dict[str, BodyConnection] = {name: BodyConnection(spec) for name, spec in site.bodies.items()}
        self._robots: dict[str, RobotHandle] = {}
        self.unavailable: dict[str, str] = {}  # robot_id -> reason
        self._lock = threading.RLock()
        self.connected = False

    def connect(self) -> Fleet:
        """Connect every body; with ``strict=False`` unreachable bodies only mark their robots unavailable."""
        def connect_one(body):
            try:
                body.connect()
            except Exception as exc:
                if self.strict:
                    raise ValueError(f"body {body.spec.name} ({body.spec.url or body.spec.driver}) failed to connect: {exc}") from exc
                body.error = f"{type(exc).__name__}: {exc}"

        with ExitStack() as cleanup:
            cleanup.callback(self.close)
            with ThreadPoolExecutor(max_workers=max(1, min(8, len(self.bodies)))) as pool:
                list(pool.map(connect_one, self.bodies.values()))
            self.connected = True
            for robot_id, spec in self.site.robots.items():
                essential = [ref.body for ref in [*spec.arms.values(), *spec.hands.values(), *spec.devices]]
                missing = [name for name in dict.fromkeys(essential) if not self.bodies[name].available]
                if missing:
                    self.unavailable[robot_id] = "body unavailable: " + "; ".join(f"{name} ({self.bodies[name].error})" for name in missing)
                    continue
                self.robot(robot_id)  # camera bodies that are down leave the robot degraded, not unavailable
            if not self._robots:
                raise ValueError("no robot of site %r is reachable:\n  - " % self.site.name + "\n  - ".join(f"{rid}: {why}" for rid, why in self.unavailable.items()))
            problems = self.validate_bindings()
            if problems:
                raise ValueError("site bindings do not match the live manifests:\n  - " + "\n  - ".join(problems))
            cleanup.pop_all()
            return self

    def robot(self, robot_id: str) -> RobotHandle:
        with self._lock:
            if robot_id in self.unavailable:
                raise DeviceError(ErrorCode.TRANSPORT_ERROR, f"robot {robot_id} is unavailable: {self.unavailable[robot_id]}", details={"robot_id": robot_id})
            if robot_id not in self._robots:
                self._robots[robot_id] = RobotHandle(self.site.robot(robot_id), self)
            return self._robots[robot_id]

    def robots(self) -> list[RobotHandle]:
        return [self.robot(rid) for rid in self.site.robots if rid not in self.unavailable]

    def control(self, body_name: str) -> ControlSession:
        return self.bodies[body_name].control()

    def validate_bindings(self) -> list[str]:
        errors: list[str] = []
        for binding in self.site.bindings:
            if binding.robot in self.unavailable:
                continue
            robot = self.robot(binding.robot)
            for s in binding.action:
                handle = robot.devices.get(s.device.key)
                if handle is None:
                    errors.append(f"binding {binding.name}: {s.device.key} not in robot")
                    continue
                command = handle.manifest.commands.get(s.command)
                if command is None:
                    errors.append(f"binding {binding.name}: {s.device.key} has no command {s.command!r} (has {sorted(handle.manifest.commands)})")
                    continue
                if handle.manifest.stream is None or s.command not in handle.manifest.stream.commands:
                    errors.append(f"binding {binding.name}: {s.device.key}.{s.command} is not streamable")
                length = command.shape[0] if command.shape else 1
                if s.length != length:
                    errors.append(f"binding {binding.name}: slice length {s.length} != {s.device.key}.{s.command} shape {length}")
            for signal in binding.proprio:
                handle = robot.devices.get(signal.device.key)
                if handle is None:
                    errors.append(f"binding {binding.name}: proprio device {signal.device.key} not in robot")
                elif signal.signal not in handle.manifest.signals:
                    errors.append(f"binding {binding.name}: {signal.device.key} has no signal {signal.signal!r} (has {sorted(handle.manifest.signals)})")
        return errors

    def binding(self, robot_id: str, actor: str) -> Binding:
        matches = self.site.bindings_for(robot_id, actor)
        if not matches:
            raise DeviceError(ErrorCode.NOT_SUPPORTED, f"actor {actor!r} is not bound to robot {robot_id!r}", details={"actors": self.site.actors_for(robot_id)})
        return matches[0]

    def estop(self, robot_id: str | None = None) -> dict[str, Any]:
        out: dict[str, Any] = {}
        bodies = self.robot(robot_id).bodies if robot_id else list(self.bodies.values())
        for body in bodies:
            if body.client is None or not body.available:
                continue
            try:
                targets = None if robot_id is None else [h.device_id for h in self.robot(robot_id).devices.values() if h.body is body]
                out[body.spec.name] = body.client.estop(targets)
            except DeviceError as exc:
                out[body.spec.name] = {"error": exc.to_dict()}
        return out

    def quiesce(self, *, reason: str = "shutdown", timeout_s: float = 5.0) -> dict[str, Any]:
        """Reject new control and obtain bounded stop evidence for owned devices."""
        from .lifecycle import quiesce

        return quiesce(self, reason=reason, timeout_s=timeout_s)

    def shutdown(self, *, reason: str = "shutdown", timeout_s: float = 5.0) -> dict[str, Any]:
        """Release resources only after idle confirmation; unknown retains ownership."""
        evidence = self.quiesce(reason=reason, timeout_s=timeout_s)
        if evidence["status"] in ("confirmed_idle", "not_required"):
            self.close()
        return evidence

    def close(self) -> None:
        """Release resources. This low-level cleanup is not stop confirmation."""
        with ExitStack() as cleanup:
            for body in self.bodies.values():
                cleanup.callback(body.close)
            self.connected = False

    # ProcessDaemon-compatible alias so the CLI can tear the fleet down.
    def stop(self) -> None:
        self.close()

    def poll(self) -> None:
        return None


__all__ = ["BodyConnection", "DeviceHandle", "Fleet", "RobotHandle"]
