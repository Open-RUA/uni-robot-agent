"""Synthetic deployment assets and Body devices; never physical acceptance evidence."""
from contextlib import ExitStack
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import threading
import time

import numpy as np

from uni_robot_agent.integrations.piper_x.config import Installation, Model, CanBinding, CameraBinding, digest
from uni_robot_agent.integrations.piper_x.body.core.contract import Principal, DeviceError, ErrorCode
from uni_robot_agent.integrations.piper_x.body.core.rpc import BodyHttpServer
from uni_robot_agent.integrations.piper_x.body.core.server import BodyServer
from uni_robot_agent.integrations.piper_x.body.core.manifest import BodyManifest, ObservationStreamSpec
from uni_robot_agent.integrations.piper_x.body.drivers.fake.driver import FakeArmDriver, FakeObservationDriver

CLOCK = "linux-boot:offline-fixture"
TOKEN = "test-only-token"
IDENTITY = np.eye(4).tolist()
LIMITS = dict(eef_step_m=.04, eef_speed_m_s=.03, eef_accel_m_s2=.2, eef_rotation_step_rad=.3,
              eef_rotation_speed_rad_s=.2, joint_step_rad=.3, joint_speed_rad_s=.3, gripper_speed_fraction_s=1.)


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2))
    return {"path": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def deployment(root, index=0):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    urdf = root / "model.urdf"
    links = '<link name="base"/>' + ''.join(f'<link name="l{i}"><collision><geometry><sphere radius="0.01"/></geometry></collision></link>' for i in range(1, 7))
    joints = ''.join(f'<joint name="j{i}" type="revolute"><parent link="{"base" if i==1 else "l"+str(i-1)}"/><child link="l{i}"/><origin xyz="0.05 0 0.0333333333333333"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>' for i in range(1, 7))
    urdf.write_text('<robot name="synthetic">' + links + joints + '</robot>')
    urdf_ref = {"path": urdf.name, "sha256": hashlib.sha256(urdf.read_bytes()).hexdigest()}
    collision = write(root / "collision.json", {"urdf_sha256": urdf_ref["sha256"], "enclosure_verified": True,
        "capsules": [{"link": f"l{i}", "start": [0,0,0], "end": [0,0,0], "radius_m": .01} for i in range(1,7)],
        "ignored_self_pairs": [[f"l{i}", f"l{i+1}"] for i in range(1,6)], "table_exempt_links": ["base"]})
    model = {"model_id": f"fixture-model-{index}", "protocol": "piper-x-can-v1", "sdk_version": f"fixture-{index}",
        "sdk_driver_sha256": "0" * 64, "firmware_profile": ("default", "V183", "V188", "V189")[index % 4],
        "firmware_version": f"fixture-firmware-{index}", "urdf": urdf_ref, "collision": collision,
        "root_link": "base", "tip_link": "l6", "joint_names": [f"j{i}" for i in range(1,7)],
        "joint_lower": [-3.]*6, "joint_upper": [3.]*6, "motion_limits": LIMITS,
        "gripper_max_opening_m": .1, "gripper_max_effort_n": 2.}
    robot_id = f"fixture-{index}"
    host = {"host_id": f"host-{index}", "operator": "fixture", "listen": "127.0.0.1", "connect_host": "127.0.0.1",
        "state_dir": f"runtime-{index}", "lock_dir": "../physical-locks", "tokens_file": "tokens.json",
        "bodies": {kind: {"body_id": f"{robot_id}-{kind}", "robot_id": robot_id, "kind": kind,
                    "port": 21000 + index*20 + offset, "token_env": f"FIXTURE_{kind.upper()}_TOKEN"}
                   for offset, kind in enumerate(("arms", "cameras"))},
        "arms": {f"port_{n}": {"interface": f"can{(index*2+(1-n if index%2 else n))}", "usb_vendor": "1234",
                    "usb_product": "5678", "usb_serial": f"USB-{index}-{n}", "channel": 0} for n in range(2)},
        "cameras": {f"sensor_{n}": {"serial": f"CAM-{index}-{n}", "model": "fixture-D435", "color": {
                    "width": 32+index*8, "height": 24+index*6, "fps": 30, "format": "rgb8"},
                    "depth": {"width": 32+index*8, "height": 24+index*6, "fps": 30, "format": "z16"},
                    "max_pair_skew_s": .02, "timeout_ms": 1000, "max_frame_age_s": .5} for n in reversed(range(3))}}
    arm_names = (f"arm_{index}_a", f"arm_{index}_b")
    camera_names = (f"overview_{index}", f"wrist_z_{index}", f"wrist_y_{index}")
    arms = {name: {"device_id": f"actuator-{index}-{n}", "binding": f"port_{n}", "role": ("left", "right")[n],
                   "grasp_site": f"{name}/grasp", "grasp_site_description": "Synthetic test frame"} for n, name in enumerate(arm_names)}
    cameras = {name: {"device_id": f"imager-{index}-{n}", "binding": f"sensor_{n}", "role": "scene" if n==0 else "wrist",
                      "parent_arm": None if n==0 else arm_names[n-1]} for n, name in reversed(list(enumerate(camera_names)))}
    bindings = {"arms": {n: CanBinding.model_validate(host["arms"][a["binding"]]).model_dump(mode="json") for n, a in arms.items()},
                "cameras": {n: CameraBinding.model_validate(host["cameras"][c["binding"]]).model_dump(mode="json") for n, c in cameras.items()}}
    cal_arms = {}
    for n, name in enumerate(arm_names):
        base = deepcopy(IDENTITY); base[1][3] = -.5 if n==0 else .5
        cal_arms[name] = {"T_world_from_base": base, "T_base_from_urdf": IDENTITY, "T_tip_from_flange": IDENTITY,
            "T_flange_from_grasp": IDENTITY, "joint_lower": [-3.]*6, "joint_upper": [3.]*6,
            "motion_limits": LIMITS, "rotation_radius_m": .05, "tracking_tolerance_rad": .01,
            "gripper_closed_m": 0., "gripper_open_m": .08, "gripper_effort": 1., "gripper_tolerance_m": .002}
    cal = {"robot_id": robot_id, "host_id": host["host_id"], "model_sha256": digest(Model.model_validate(model).model_dump(mode="json")),
        "bindings_sha256": digest(bindings), "collected_at": "synthetic-fixture", "operator": "test-only", "world_frame": "world",
        "arms": cal_arms, "cameras": {n: {"parent_frame": "world" if c["parent_arm"] is None else arms[c["parent_arm"]]["device_id"] + "/flange",
            "optical_frame": c["device_id"]+"/color_optical", "T_parent_from_optical": IDENTITY,
            "capture_sha256": digest(bindings["cameras"][n])} for n,c in cameras.items()},
        "table_height_m": -.1, "collision_margin_m": .005,
        "accepted_checks": ["identity", "fk_tcp", "cameras", "grippers", "independent_stop", "dual_motion"]}
    calibration = write(root / "calibration.json", cal)
    robot = {"robot_id": robot_id, "arms_body": "arms", "cameras_body": "cameras", "arms": arms, "cameras": cameras,
        "required_arm_roles": ["left", "right"], "required_cameras": list(cameras), "calibration": calibration,
        "motion_profiles": {p: {"max_step_m": .03, "position_tolerance_m": .005, "orientation_tolerance_rad": .05}
                            for p in ("standard", "approach", "precision")},
        "action_timeout_s": 5., "stop_timeout_s": 1., "feedback_max_age_s": .1, "feedback_max_skew_s": .025,
        "control_frequency_hz": 10., "max_episode_steps": 100}
    write(root / "model.json", model); write(root / "host.json", host); write(root / "robot.json", robot)
    path = root / "deployment.json"
    write(path, {"schema_version": 1, "model": "model.json", "host": "host.json", "robot": "robot.json"})
    return Installation(path)


class Arms(FakeArmDriver):
    def __init__(self, installation):
        self.installation = installation
        self.names = {a.device_id:n for n,a in installation.robot.arms.items()}
        super().__init__(installation.host.bodies[installation.robot.arms_body].body_id,
                         list(self.names), motion_limits=LIMITS)
        self.calls = 0
        self.fault = None
        self.started = threading.Event()
        self.stopped = threading.Event()

    def manifest(self):
        manifest = super().manifest()
        for d in manifest.devices:
            name = self.names[d.device_id]
            d.metadata["identity"] = self.installation.identity(name, "arms")
            d.metadata["joint_limits_rad"] = {"lower": [-3.]*6, "upper": [3.]*6}
            d.observation_streams["state"] = ObservationStreamSpec(["joint_position_rad"])
            d.procedures["move_eef_group"].params.update(max_segments={"type":"integer"}, segment_m={"type":"number"}, max_duration_s={"type":"number"})
        return manifest

    def read(self, device_id, include=None):
        if self.fault == "feedback_lost" and self.calls:
            raise RuntimeError("injected feedback loss")
        result = super().read(device_id, include)
        result["signals"]["gripper_opening"] = result["signals"]["gripper_opening"][0]
        result["timing"] = timing()
        return result

    def invoke(self, device_ids, procedure, params, *, cancel, progress):
        self.calls += 1
        self.started.set()
        if self.fault == "slow":
            self._moving.update(device_ids)
            cancel.wait(2)
            self._moving.clear()
            raise DeviceError(ErrorCode.TRACKING_ERROR, "injected cancellation", execution_outcome="partial")
        if self.fault == "partial":
            self._state[device_ids[0]] = self._apply(device_ids[0], params["targets"][device_ids[0]])
            raise DeviceError(ErrorCode.TRACKING_ERROR, "injected second-arm failure", execution_outcome="partial")
        return super().invoke(device_ids, procedure, params, cancel=cancel, progress=progress)

    def estop(self, device_id):
        self.stopped.set()
        self._moving.clear()
        return {"stop_requested": True, "physical_stop_guaranteed": False}

    stream_hold = estop

    def read_observation_at(self, device_id, stream, at, *, mode="bracket", max_gap_s=.05, include=None):
        stamp = int(at["source_timestamp_ns"])
        return {"at": at, "mode": mode, "samples": [{"sequence":stamp+d,"generation":1,
            "timing":timing(stamp+d), "signals":{"joint_position_rad":self._state[device_id]["joint_position_rad"]}}
            for d in (-1_000_000,1_000_000)]}


def timing(stamp=None):
    stamp = stamp or time.time_ns()
    return {"clock_id": CLOCK, "source_clock":"unix_realtime", "source_semantics":"host_can_receive",
        "source_timestamp_ns":str(stamp), "received_realtime_ns":str(stamp),
        "received_monotonic_ns":str(time.monotonic_ns() + stamp-time.time_ns()), "clock_pair_span_ns":"0"}


class Cameras:
    def __init__(self, installation):
        self.installation = installation
        self.body_id = installation.host.bodies[installation.robot.cameras_body].body_id
        self.devices = {c.device_id: n for n,c in installation.robot.cameras.items()}
        self.drivers = {d: FakeObservationDriver() for d in self.devices}
        for driver in self.drivers.values():
            driver._clock_id = CLOCK
        self.disconnected = set()

    def manifest(self):
        devices = []
        for device, name in self.devices.items():
            item = self.drivers[device].manifest().devices[0]
            item.device_id = device
            item.metadata["identity"] = self.installation.identity(name, "cameras")
            devices.append(item)
        return BodyManifest(self.body_id, "synthetic", devices)

    def read(self, device_id, include=None):
        if device_id in self.disconnected:
            raise DeviceError(ErrorCode.OBSERVATION_STALE, "injected camera disconnect")
        return self.drivers[device_id].read("camera", include)

    def read_observation_at(self, device_id, stream, at, **kwargs):
        self.read(device_id)
        return self.drivers[device_id].read_observation_at("camera", stream, at, **kwargs)

    def check(self, device_id):
        return {"readiness": "unsatisfied" if device_id in self.disconnected else "satisfied", "checks": {}}

    def health(self):
        return {"ok":True}

    def estop(self, device_id):
        return {"status":"not_required"}

    stream_hold = estop

    def write(self, *args, **kwargs):
        raise DeviceError(ErrorCode.NOT_SUPPORTED, "Synthetic camera is observation-only")

    invoke = stream_apply = write

    def close(self):
        for driver in self.drivers.values():
            driver.close()


def body_server(stack, driver, path):
    stack.callback(driver.close)
    server = BodyServer(driver, tokens={TOKEN: Principal("fixture", ["observe","operate","lifecycle","admin"])}, journal_dir=path)
    stack.callback(server.close)
    http = BodyHttpServer(server, host="127.0.0.1", port=0)
    stack.callback(http.stop)
    url = http.start()
    return server, url
