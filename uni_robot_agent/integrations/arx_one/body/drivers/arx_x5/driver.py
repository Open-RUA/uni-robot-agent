"""ARX X5 arms behind the vendor ``normal`` controllers (ROS 2).

Wire facts (X5Controller with ``arm_control_type=normal``, ``arm_end_type=2``):

* feedback ``arx5_arm_msg/RobotStatus`` on ``/<side>_status`` at ~100 Hz:
  ``joint_pos[7]`` (six joints in rad plus the gripper position), ``joint_vel[7]``,
  ``joint_cur[7]``, ``end_pos[6]`` (vendor xyz + rpy in the arm base frame);
* commands ``arx5_arm_msg/RobotCmd`` on ``/<side>_cmd``: a *complete* command every
  tick — ``joint_pos[6]``, ``end_pos[6]``, ``gripper`` and ``mode`` (5 = joint
  targets, 4 = end-effector targets, 1 = vendor go-home); the gripper command
  shares the feedback scale (closed ≈ 0, open ≈ -3.4);
* trajectories are streamed at 50 Hz with quintic interpolation; fields not
  being interpolated hold their initial feedback snapshot.

Planners never see raw gripper values: the Body speaks *opening* (1 = open,
0 = closed). Command endpoints (raw for opening 0 / 1) are stored per arm and
the ``calibrate_gripper`` procedure measures the feedback endpoints so opening
read-backs are trustworthy — the equivalent of Robotiq ``auto_calibrate`` or
LeRobot range recording, run once in a new robot's setup session.

The ROS side is a small transport protocol (scripted in tests);
:class:`RclpyTransport` is the real one. Processes must share the controllers'
ROS environment (``ROS_DOMAIN_ID`` etc.), configured by ``rua-host``.
"""

from __future__ import annotations

import json
import math
import os
import statistics
import tempfile
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any, Protocol

from ...core.contract import DeviceError, ErrorCode
from ...core.arm.eef import (
    EefTrajectory, MAX_WAYPOINTS, TARGET_PROPERTIES, TARGET_REQUIRED, WAYPOINTS_SCHEMA, coordinate_metadata,
    interpolate_pose, resolve_goal, rotation_distance, validate_target, validate_waypoints,
)
from ...core.manifest import (
    BodyManifest, Command, DeviceManifest, KIND_ARM, ObservationStreamSpec, Procedure, Signal, StreamSpec, enum, num,
)
from ...core.arm.motion_limits import MotionLimits, QUINTIC_PEAK_SPEED, validate_publish_hz
from ...core.observations import ObservationBuffer
from .can_feedback import CanFeedbackMonitor, diagnose_fault
from .limits import JOINT_LOWER, JOINT_UPPER, controller_identity_matches, controller_profile, identity
from .ros_support import RclpyContext, Ticker
from .safety import (
    ArrivalWindow, EefPreflight, NativeKinematics, PoseTrackingGuard,
    TRUNCATION_JOINT_MARGIN_RAD, TrackingLimits, check_joint_target, clip_eef_xyz, pose_error,
    validate_eef_limit_policy, workspace_status,
)


# Vendor controller command modes used here: EEF pose and joint position.
MODE_EEF, MODE_JOINT = 4, 5
# Scalar manifest envelope; actual per-joint bounds are checked before dispatch.
JOINT_LIMIT_RAD = max(abs(v) for v in (*JOINT_LOWER, *JOINT_UPPER))
FEEDBACK_STALE_S = 0.2
# Gripper openings derived from calibrated feedback endpoints, as opposed to an estimate.
MEASURED_BASIS = "measured_feedback_endpoints"
JOINT_HISTORY_S = 2.0
JOINT_HISTORY_LIMIT = 512
PUBLISH_HZ = 50.0

def validate_arms(arms):
    """Require explicit hardware identities before opening ROS or CAN resources."""
    if not isinstance(arms, dict) or not arms:
        raise ValueError("an explicit nonempty arms map is required")
    sides, buses, topics = set(), set(), set()
    for device, spec in arms.items():
        if not isinstance(device, str) or not device or not isinstance(spec, dict):
            raise ValueError("invalid arm configuration")
        if spec.get("side") not in ("left", "right") or spec["side"] in sides:
            raise ValueError("arm sides must be distinct left/right")
        if not spec.get("can") or spec["can"] in buses:
            raise ValueError("arm CAN interfaces must be explicit and distinct")
        if not spec.get("profile"):
            raise ValueError("each arm needs an explicit controller profile")
        controller_profile(spec)
        sides.add(spec["side"]); buses.add(spec["can"])
        for field in ("status_topic", "command_topic"):
            value = spec.get(field)
            if not isinstance(value, str) or not value.startswith("/") or value in topics:
                raise ValueError("arm ROS topics must be absolute and distinct")
            topics.add(value)
    return {name: dict(spec) for name, spec in arms.items()}


def _arx_coordinate_metadata(device_id):
    return {**coordinate_metadata(device_id), "origin": "URDF link6 zero-joint position; XYZ = URDF FK position minus this origin"}


class ArxTransport(Protocol):
    def subscribe_status(
        self, topic: str, callback: Callable[[dict[str, Any]], None]
    ) -> None: ...

    def publish_command(self, topic: str, command: dict[str, Any]) -> None: ...

    def close(self) -> None: ...


class RclpyTransport:
    """Real ROS 2 transport (arx5_arm_msg must be on the Python path: source the X5 workspace)."""

    requires_can = True

    def __init__(
        self,
        node_name: str = "rua_arx_body",
        *,
        exclusive: bool = True,
        arms: dict | None = None,
    ) -> None:
        self.arms = validate_arms(arms)
        from arx5_arm_msg.msg import RobotCmd, RobotStatus
        self._ctx = RclpyContext(node_name)
        self._RobotCmd, self._RobotStatus = RobotCmd, RobotStatus
        self._pubs: dict[str, Any] = {}
        self._diagnostic_cache: dict[str, tuple[float, dict[str, Any]]] = {}
        self._clients: dict[str, Any] = {}
        from ...core.resources import ResourceLocks

        try:
            self._ownership = ResourceLocks(
                ["can:" + v["can"] for v in self.arms.values()] if exclusive else []
            )
        except BaseException:
            self._ctx.close()
            raise
        self._can_monitors = {
            spec["side"]: CanFeedbackMonitor(spec["can"]) for spec in self.arms.values()
        }

    def motor_feedback(self, side: str) -> dict[str, Any]:
        return self._can_monitors[side].snapshot()

    def subscribe_status(
        self, topic: str, callback: Callable[[dict[str, Any]], None]
    ) -> None:
        def on_msg(msg: Any) -> None:
            mono_before = time.monotonic_ns()
            received_wall = time.time_ns()
            mono_after = time.monotonic_ns()
            received_mono_ns = (mono_before + mono_after) // 2
            source_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            ros_clock = self._ctx.node.get_clock()
            simulated = ros_clock.ros_time_is_active
            source_age = (ros_clock.now().nanoseconds if simulated else received_wall) - source_ns
            from ...core.timing import host_clock_id

            callback(
                {
                    "joint_pos": list(msg.joint_pos),
                    "joint_vel": list(msg.joint_vel),
                    "joint_cur": list(msg.joint_cur),
                    "end_pos": list(msg.end_pos),
                    "stamp": source_ns / 1e9,
                    "source_age_s": source_age / 1e9,
                    "received_mono": received_mono_ns / 1e9,
                    "timing": {
                        "clock_id": host_clock_id(),
                        "source_timestamp_ns": str(source_ns),
                        "source_semantics": "controller_status_publication",
                        "source_clock": "ros_sim_time"
                        if simulated
                        else "unix_realtime",
                        "received_realtime_ns": str(received_wall),
                        "received_monotonic_ns": str(received_mono_ns),
                        "clock_pair_span_ns": str(mono_after - mono_before),
                    },
                }
            )

        self._ctx.node.create_subscription(self._RobotStatus, topic, on_msg, 1)

    def publish_command(self, topic: str, command: dict[str, Any]) -> None:
        pub = self._pubs.get(topic)
        if pub is None:
            pub = self._ctx.node.create_publisher(self._RobotCmd, topic, 10)
            self._pubs[topic] = pub
        msg = self._RobotCmd()
        msg.header.stamp = self._ctx.node.get_clock().now().to_msg()
        msg.joint_pos = [float(v) for v in command["joint_pos"]][:6]
        msg.end_pos = [float(v) for v in command["end_pos"]][:6]
        msg.gripper = float(command["gripper"])
        msg.mode = int(command["mode"])
        pub.publish(msg)

    def driver_command(self, side, operation):
        from arx5_arm_msg.srv import DriverCommand
        spec = next(v for v in self.arms.values() if v['side'] == side)
        service = spec.get('controller', f'/{side}_x5_controller') + '/driver_command'
        client = self._clients.get(service)
        if client is None:
            client = self._ctx.node.create_client(DriverCommand, service)
            self._clients[service] = client
        if not client.wait_for_service(timeout_sec=.2):
            raise RuntimeError('controller driver service unavailable')
        req = DriverCommand.Request(); req.operation = operation
        future = client.call_async(req)
        done = threading.Event(); future.add_done_callback(lambda _: done.set())
        if not done.wait(1.):
            future.cancel()
            raise TimeoutError('controller reply unknown; do not replay')
        return future.result()

    def teach_state(self, side):
        response = self.driver_command(side, 'get_teach_state')
        if not response.success:
            if response.message == 'Unknown driver operation: get_teach_state':
                return {'supported': False, 'active': None}
            raise RuntimeError(response.message)
        if list(response.values) not in ([1., 0.], [1., 1.]):
            raise RuntimeError('invalid teaching capability/state')
        return {'supported': True, 'active': response.values[1] == 1., 'version': 1}

    def diagnostics(self, side: str, *, refresh: bool = False) -> dict[str, Any]:
        cached = self._diagnostic_cache.get(side)
        if not refresh and cached and time.monotonic() - cached[0] < 0.25:
            return cached[1]
        node = self._ctx.node
        spec = next(v for v in self.arms.values() if v["side"] == side)
        controller = spec.get("controller", f"/{side}_x5_controller")
        expected = controller.rsplit("/", 1)[-1]
        namespace = controller.rsplit("/", 1)[0] or "/"
        publishers = node.get_publishers_info_by_topic(spec["status_topic"])
        writers = node.get_publishers_info_by_topic(spec["command_topic"])
        result = {
            "status_source": len(publishers) == 1
            and publishers[0].node_name == expected
            and publishers[0].node_namespace == namespace,
            "exclusive_writer": all(
                p.node_name
                in {node.get_name(), spec.get("writer_node", node.get_name())}
                for p in writers
            ),
        }
        from arx5_arm_msg.srv import DriverCommand
        from rcl_interfaces.srv import GetParameters

        def query(service, typ, request):
            client = self._clients.get(service)
            if client is None:
                client = node.create_client(typ, service)
                self._clients[service] = client
            if not client.wait_for_service(timeout_sec=0.2):
                raise RuntimeError(f"{service} unavailable")
            future = client.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(0.4):
                future.cancel()
                raise RuntimeError(f"{service} timed out")
            return future.result()

        try:
            req = DriverCommand.Request()
            req.operation = "get_error_summary"
            response = query(f"{controller}/driver_command", DriverCommand, req)
            if (
                not response.success
                or len(response.values) != 2
                or not math.isfinite(response.values[0])
                or response.values[0] != int(response.values[0])
                or response.values[0] < len(response.error_codes)
                or response.values[1] not in (0, 1)
            ):
                raise RuntimeError(
                    "controller has no valid bounded error summary; update controller and Body together"
                )
            result["vendor_error_count"] = int(response.values[0])
            result["vendor_errors_truncated"] = bool(response.values[1])
            result["vendor_errors"] = (
                sorted(set(response.error_codes))[:64] if response.success else None
            )
            req = GetParameters.Request()
            req.names = [
                "arm_control_type",
                "arm_end_type",
                "arm_can_id",
                "arm_pub_topic_name",
                "arm_sub_topic_name",
                "startup_policy",
                "vendor_sdk_sha256",
                "kinematic_model_sha256",
            ]
            params = query(f"{controller}/get_parameters", GetParameters, req).values
            result["startup_policy"] = (
                params[5].string_value if len(params) == 8 else None
            )
            result["sdk_identity_matches"] = len(
                params
            ) == 8 and controller_identity_matches(
                params[6].string_value, params[7].string_value, controller_profile(spec)
            )
            result["vendor_sdk_sha256"] = (
                params[6].string_value if len(params) == 8 else None
            )
            result["kinematic_model_sha256"] = (
                params[7].string_value if len(params) == 8 else None
            )
            result["configuration"] = len(params) == 8 and (
                params[0].string_value == "normal"
                and params[1].integer_value == controller_profile(spec).end_type
                and params[2].string_value == spec["can"]
                and params[3].string_value == spec["status_topic"]
                and params[4].string_value == spec["command_topic"]
                and params[5].string_value == "post_vendor_init_snapshot_hold_v1"
                and result["sdk_identity_matches"]
            )
        except Exception as exc:
            result["error"] = str(exc)
        feedback = self.motor_feedback(side)
        result["motor_feedback"] = feedback
        result["fault"] = diagnose_fault(result, feedback)
        result["ready"] = (
            result["status_source"]
            and result["exclusive_writer"]
            and result.get("configuration") is True
            and result.get("vendor_errors") == []
            and result.get("vendor_error_count") == 0
            and result.get("vendor_errors_truncated") is False
            and feedback.get("available") is True
            and feedback.get("all_online") is True
            and feedback.get("all_fault_free") is True
        )
        result['hardware_ready'] = result['ready']
        try:
            result['teaching'] = self.teach_state(side)
            if result['teaching']['active'] is True:
                result['ready'] = False
        except Exception as exc:
            result['teaching'] = {'supported': False, 'active': None, 'error': str(exc)}
            result['ready'] = result['hardware_ready'] = False
        self._diagnostic_cache[side] = (time.monotonic(), result)
        return result

    def close(self) -> None:
        for monitor in self._can_monitors.values():
            monitor.close()
        self._ctx.close()
        self._ownership.close()


class _ArmState:
    def __init__(self, clock=time.monotonic) -> None:
        self.lock = threading.Lock()
        self.feedback_changed = threading.Condition(self.lock)
        self.command_lock = threading.RLock()
        self.joint_pos: list[float] | None = None
        self.joint_vel: list[float] | None = None
        self.joint_cur: list[float] | None = None
        self.end_pos: list[float] | None = None
        self.received_at: float | None = None
        self.source_stamp_ns: int | None = None
        self.timing: dict[str, Any] = {}
        self.joint_history = ObservationBuffer(retention_s=JOINT_HISTORY_S, max_samples=JOINT_HISTORY_LIMIT, clock=clock)
        self.messages = 0
        self.last_command: dict[str, Any] | None = None
        self.abort = threading.Event()
        self.moving = False
        self.feedback_error: str | None = None
        self.gripper_target: float | None = None
        # Diagnostic reference only. Never resend an old target to correct a
        # passive arm while its peer is executing.
        self.hold_reference: dict | None = None
        self.can_sample: tuple[int, float] | None = None


class ArxX5Driver:
    """Body driver for ARX X5 arms driven by the vendor ``normal`` controllers."""

    data_origin = "real"

    def __init__(
        self,
        transport: ArxTransport,
        *,
        body_id: str = "arx_arms",
        arms: dict[str, dict[str, str]] | None = None,
        calibration_path: Path | str | None = None,
        clock: Callable[[], float] = time.monotonic,
        publish_hz: float = PUBLISH_HZ,
        hold_s_default: float = 0.5,
        motion_limits: dict | None = None,
        tracking_limits: dict | None = None,
        eef_preflight: Callable | None = None,
        eef_limit_policy: str = "reject",
        controller_recovery: Callable | None = None,
        controller_lifecycle: Callable | None = None,
    ) -> None:
        self.motion_limits = MotionLimits.from_config(motion_limits)
        self.tracking_limits = TrackingLimits.from_config(tracking_limits)
        self._eef_preflight = eef_preflight
        self.eef_limit_policy = validate_eef_limit_policy(eef_limit_policy)
        self._kinematics: dict[str, NativeKinematics] = {}
        self.transport = transport
        self._controller_recovery = controller_recovery
        self._controller_lifecycle = controller_lifecycle
        self.body_id = body_id
        self.arms = validate_arms(arms)
        self._profiles = {device: controller_profile(spec) for device, spec in self.arms.items()}
        self._clock = clock
        self._publish_hz = validate_publish_hz(publish_hz)
        self._hold_s_default = float(hold_s_default)
        self._states: dict[str, _ArmState] = {d: _ArmState(clock) for d in self.arms}
        self._calibration_path = (
            Path(calibration_path).expanduser() if calibration_path else None
        )
        self._calibration_lock = threading.RLock()
        self.calibration: dict[str, dict[str, Any]] = self._load_calibration()
        for device_id, spec in self.arms.items():
            transport.subscribe_status(
                spec["status_topic"], lambda msg, d=device_id: self._on_status(d, msg)
            )

    # -- calibration persistence ------------------------------------------------------------------

    def _load_calibration(self) -> dict[str, dict[str, Any]]:
        data: dict[str, Any] = {}
        if self._calibration_path and self._calibration_path.exists():
            try:
                data = json.loads(self._calibration_path.read_text())
            except Exception:
                data = {}
        out = {}
        for device_id in self.arms:
            entry = data.get(device_id) or {}
            profile = self._profiles[device_id]
            out[device_id] = {
                **entry,
                "command": dict(entry.get("command") or self.arms[device_id].get("gripper_command_endpoints") or {
                    "closed_raw": profile.gripper_closed_raw,
                    "open_raw": profile.gripper_open_raw,
                }),
                "feedback": entry.get("feedback"),
                "measured_at": entry.get("measured_at"),
            }
        return out

    def _save_calibration(self) -> None:
        if self._calibration_path:
            self._calibration_path.parent.mkdir(parents=True, exist_ok=True)
            # Readers see either complete version, including if the process exits
            # during a write. Serialize calibrations of different arms as well.
            with self._calibration_lock:
                fd, name = tempfile.mkstemp(
                    prefix=".gripper-", dir=self._calibration_path.parent
                )
                try:
                    with os.fdopen(fd, "w") as stream:
                        json.dump(self.calibration, stream, indent=2, allow_nan=False)
                        stream.flush()
                        os.fsync(stream.fileno())
                    os.replace(name, self._calibration_path)
                finally:
                    Path(name).unlink(missing_ok=True)

    # -- feedback --------------------------------------------------------------------------------

    def _on_status(self, device_id: str, msg: dict[str, Any]) -> None:
        state = self._states[device_id]
        now = self._clock()
        try:
            received_mono = float(msg.get("received_mono", now))
            source_age = float(msg.get("source_age_s", 0.0))
            source_stamp = float(msg["stamp"]) if "source_age_s" in msg else None
            timing = dict(msg.get("timing", {}))
            from ...core.timing import timestamp_ns

            source_ns = (
                timestamp_ns(timing["source_timestamp_ns"])
                if "source_timestamp_ns" in timing
                else (None if source_stamp is None else round(source_stamp * 1e9))
            )
            if (
                not math.isfinite(source_age)
                or source_age < 0
                or not math.isfinite(received_mono)
                or received_mono > now
                or (source_stamp is not None and not math.isfinite(source_stamp))
                or (source_ns is not None and source_ns <= 0)
            ):
                raise ValueError("invalid RobotStatus source timestamp")
            fields = {
                k: [float(v) for v in msg[k]]
                for k in ("joint_pos", "joint_vel", "joint_cur", "end_pos")
            }
            if any(
                len(v) != (6 if k == "end_pos" else 7)
                or not all(math.isfinite(x) for x in v)
                for k, v in fields.items()
            ):
                raise ValueError("invalid RobotStatus length or nonfinite value")
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            with state.lock:
                state.feedback_error = str(exc)
                state.received_at = None
                state.joint_history.clear()
            return
        with state.lock:
            if (
                source_ns is not None
                and state.source_stamp_ns is not None
                and source_ns <= state.source_stamp_ns
            ):
                return  # replayed/out-of-order feedback cannot become fresh
            state.joint_pos = fields["joint_pos"]
            state.joint_vel = fields["joint_vel"]
            state.joint_cur = fields["joint_cur"]
            state.end_pos = fields["end_pos"]
            state.feedback_error = None
            state.received_at = received_mono - source_age
            state.source_stamp_ns = source_ns
            state.timing = timing
            if (
                "source_timestamp_ns" in timing
                and timing.get("source_clock") == "unix_realtime"
                and timing.get("clock_id")
            ):
                raw = fields["joint_pos"][6]
                opening, basis = self.raw_to_opening(device_id, raw)
                state.joint_history.append(timing, {
                    "moving": state.moving,
                    "joint_position_rad": fields["joint_pos"][:6],
                    "joint_velocity_raw": fields["joint_vel"][:6],
                    "joint_current_raw": fields["joint_cur"][:6],
                    "gripper_position_raw": raw,
                    "gripper_current_raw": fields["joint_cur"][6],
                    "gripper_opening": opening,
                    "gripper_opening_basis": basis,
                    "gripper_opening_measured": basis == MEASURED_BASIS,
                    "eef_position_m": fields["end_pos"][:3],
                    "eef_rpy_rad": fields["end_pos"][3:6],
                }, sequence=state.messages + 1)
            else:
                state.joint_history.clear()
            state.messages += 1
            state.feedback_changed.notify_all()

    def _snapshot(
        self, device_id: str
    ) -> tuple[list[float] | None, list[float] | None, float | None]:
        state = self._states[device_id]
        with state.lock:
            return (
                list(state.joint_pos) if state.joint_pos else None,
                list(state.end_pos) if state.end_pos else None,
                state.received_at,
            )

    def _can_rx_packets(self, device_id: str) -> int | None:
        path = (
            Path("/sys/class/net")
            / self.arms[device_id]["can"]
            / "statistics"
            / "rx_packets"
        )
        try:
            return int(path.read_text().strip())
        except (OSError, ValueError):
            return None

    def can_traffic(self, device_id: str, window_s: float = 0.2) -> dict[str, Any]:
        """RX packets on the arm's CAN interface over ``window_s`` (None when sysfs is unavailable, e.g. in tests)."""
        first = self._can_rx_packets(device_id)
        if first is None:
            return {"available": False}
        time.sleep(window_s)
        second = self._can_rx_packets(device_id)
        if second is None:
            return {"available": False}
        return {
            "available": True,
            "rx_packets_delta": second - first,
            "window_s": window_s,
            "interface": self.arms[device_id]["can"],
        }

    def _require_can_traffic(self, device_id: str) -> None:
        # Nonblocking after the first sample: never sleep 100 ms on a 50 Hz stream.
        state = self._states[device_id]
        current = self._can_rx_packets(device_id)
        if current is None:
            if getattr(self.transport, "requires_can", False):
                raise DeviceError(
                    ErrorCode.PRECONDITION_FAILED,
                    f"{device_id}: CAN interface is missing",
                )
            return
        now = self._clock()
        previous = state.can_sample
        if previous is None:
            traffic = self.can_traffic(device_id, 0.1)
            if not traffic.get("available") or traffic["rx_packets_delta"] <= 0:
                raise DeviceError(
                    ErrorCode.PRECONDITION_FAILED,
                    f"{device_id}: CAN RX is not advancing",
                    details=traffic,
                )
            current = self._can_rx_packets(device_id)
            if current is None:
                raise DeviceError(
                    ErrorCode.PRECONDITION_FAILED, f"{device_id}: CAN disappeared"
                )
            state.can_sample = (current, self._clock())
        elif current < previous[0] or (
            current == previous[0] and now - previous[1] > 0.25
        ):
            raise DeviceError(
                ErrorCode.PRECONDITION_FAILED,
                f"{device_id}: CAN RX stopped or interface was replaced; inspect hardware before restarting",
            )
        elif current > previous[0]:
            state.can_sample = (current, now)

    def _motors_online(self, device_id: str) -> bool | None:
        state = self._states[device_id]
        with state.lock:
            if state.joint_pos is None or state.joint_cur is None:
                return None
            return not (
                all(v == 0.0 for v in state.joint_pos)
                and all(v == 0.0 for v in state.joint_cur)
            )

    def _fresh_feedback(self, device_id: str) -> tuple[list[float], list[float]]:
        """Measured joints and EEF pose from feedback younger than FEEDBACK_STALE_S; nothing else is required."""
        joints, end_pos, received_at = self._snapshot(device_id)
        if joints is None or end_pos is None or received_at is None:
            raise DeviceError(
                ErrorCode.OBSERVATION_STALE,
                f"{device_id}: no RobotStatus yet (controller running? CAN up? same ROS_DOMAIN_ID?)",
                details={
                    "device_id": device_id,
                    "topic": self.arms[device_id]["status_topic"],
                },
            )
        age = self._clock() - received_at
        if age > FEEDBACK_STALE_S:
            raise DeviceError(
                ErrorCode.OBSERVATION_STALE,
                f"{device_id}: feedback is {age:.2f}s old",
                details={"device_id": device_id, "age_s": round(age, 3)},
            )
        return joints, end_pos

    def _require_fresh(self, device_id: str, *, teaching=False) -> tuple[list[float], list[float]]:
        motor_feedback = getattr(self.transport, "motor_feedback", None)
        if motor_feedback is not None:
            feedback = motor_feedback(self.arms[device_id]["side"])
            if not (
                feedback.get("available")
                and feedback.get("all_online")
                and feedback.get("all_fault_free")
            ):
                raise DeviceError(
                    ErrorCode.PRECONDITION_FAILED,
                    f"{device_id}: live motor CAN feedback check failed",
                    details=feedback,
                )
        diagnostics = getattr(self.transport, "diagnostics", None)
        if diagnostics is not None:
            result = diagnostics(self.arms[device_id]["side"])
            if not result.get('hardware_ready' if teaching else 'ready', False):
                raise DeviceError(
                    ErrorCode.PRECONDITION_FAILED,
                    f"{device_id}: "
                    + (
                        result.get("fault", {}).get("reason")
                        or "controller source/configuration/writer/fault check failed"
                    ),
                    details=result,
                )
        self._require_can_traffic(device_id)
        joints, end_pos = self._fresh_feedback(device_id)
        if any(abs(v) > JOINT_LIMIT_RAD for v in joints[:6]):
            raise DeviceError(
                ErrorCode.PRECONDITION_FAILED,
                f"{device_id}: joint feedback is outside the configured command envelope",
                details={"joints": joints[:6]},
            )
        if self._motors_online(device_id) is False:
            raise DeviceError(
                ErrorCode.PRECONDITION_FAILED,
                f"{device_id}: motors offline (feedback all zeros); power the arm and check CAN",
                details={"device_id": device_id, "can": self.arms[device_id]["can"]},
            )
        return joints, end_pos

    # -- gripper value spaces --------------------------------------------------------------------

    def _endpoints(
        self, device_id: str, *, measured: bool
    ) -> tuple[float, float] | None:
        cal = self.calibration[device_id]
        source = cal.get("feedback") if measured else cal.get("command")
        if not source:
            return None
        return float(source["closed_raw"]), float(source["open_raw"])

    def opening_to_raw(self, device_id: str, opening: float) -> float:
        closed, open_ = self._endpoints(device_id, measured=False)  # type: ignore[misc]
        opening = min(1.0, max(0.0, float(opening)))
        return closed + opening * (open_ - closed)

    def raw_to_opening(self, device_id: str, raw: float) -> tuple[float | None, str]:
        measured = self._endpoints(device_id, measured=True)
        endpoints, basis = (
            (measured, MEASURED_BASIS)
            if measured
            else (
                self._endpoints(device_id, measured=False),
                "command_endpoints_estimate",
            )
        )
        if endpoints is None:
            return None, "uncalibrated"
        closed, open_ = endpoints
        if closed == open_:
            return None, "degenerate"
        return max(0.0, min(1.0, (float(raw) - closed) / (open_ - closed))), basis

    # -- manifest --------------------------------------------------------------------------------

    def _device_manifest(self, device_id: str) -> DeviceManifest:
        spec = self.arms[device_id]
        return DeviceManifest(
            device_id=device_id,
            kind=KIND_ARM,
            hardware_profile=f"arx-x5-normal-end{self._profiles[device_id].end_type}",
            components=["j1", "j2", "j3", "j4", "j5", "j6", "gripper", "eef"],
            frames=[f"{device_id}/base"],
            signals={
                "moving": Signal("moving", "", (), "Body command execution state at source feedback ingestion; not proof of physical stillness"),
                "joint_history": Signal(
                    "joint_history", "", (),
                    "Opt-in: up to 2 seconds / 512 validated source-stamped joint samples; include explicitly.",
                ),
                "joint_position_rad": Signal("joint_position_rad", "rad", (6,)),
                "joint_velocity_raw": Signal("joint_velocity_raw", "vendor_raw", (6,)),
                "joint_current_raw": Signal("joint_current_raw", "vendor_raw", (6,)),
                "gripper_opening": Signal(
                    "gripper_opening",
                    "fraction",
                    (1,),
                    "1 open, 0 closed; basis in gripper_opening_basis",
                ),
                "gripper_opening_basis": Signal(
                    "gripper_opening_basis",
                    "",
                    (),
                    "measured_feedback_endpoints | command_endpoints_estimate | uncalibrated",
                ),
                "gripper_opening_measured": Signal(
                    "gripper_opening_measured", "", (), "true when gripper_opening comes from calibrated feedback"),
                "gripper_position_raw": Signal(
                    "gripper_position_raw",
                    "vendor_raw",
                    (1,),
                    "controller feedback joint_pos[6] (diagnostic)",
                ),
                "gripper_current_raw": Signal(
                    "gripper_current_raw",
                    "vendor_raw",
                    (1,),
                    "rises in magnitude while pressing on an object",
                ),
                "eef_position_m": Signal(
                    "eef_position_m",
                    "m",
                    (3,),
                    "controller end_pos[0:3], arm base frame",
                ),
                "eef_rpy_rad": Signal(
                    "eef_rpy_rad", "rad", (3,), "controller end_pos[3:6]"
                ),
            },
            commands={
                "joint_targets_rad": Command(
                    "joint_targets_rad",
                    "rad",
                    (6,),
                    -JOINT_LIMIT_RAD,
                    JOINT_LIMIT_RAD,
                    self.motion_limits.joint_speed_rad_s,
                    "absolute joint targets (mode 5)",
                ),
                "gripper_opening": Command(
                    "gripper_opening",
                    "fraction",
                    (1,),
                    0.0,
                    1.0,
                    self.motion_limits.gripper_speed_fraction_s,
                    "1 open, 0 closed; mapped to the vendor raw command through the stored endpoints",
                ),
            },
            procedures={
                "move_eef_group": Procedure(
                    "move_eef_group",
                    "Check each arm's reachable EEF path prefix before dispatch; one continuous trajectory "
                    "per arm, final arrival only. Holds grippers unless explicitly supplied. No collision planning.",
                    {"targets": {"type": "object"}, "max_segments": {"type": "integer", "minimum": 1, "maximum": 24},
                     "segment_m": num(0.001, .04), "max_duration_s": num(.2, 30.)}, ["targets"],
                    completion="eef_pose_feedback_verified"),
                **({name: Procedure(name, description,
                    {'site_ready': {'type': 'boolean', 'const': True}}, ['site_ready'],
                    completion='operator_mode_verified', permission='lifecycle')
                    for name, description in (
                        ('enter_soft_teach', 'Operator SOFT teaching: all arm joints AND gripper lose holding torque. Not electrical motor disable.'),
                        ('resume_teach_hold', 'Operator hold at freshly measured joints and gripper; never home or replay targets.'),
                    )} if hasattr(self.transport, 'teach_state') else {}),
                **({name: Procedure(
                    name, description,
                    {"site_ready": {"type": "boolean", "const": True}}, ["site_ready"],
                    completion="controller_lifecycle_verified", permission="lifecycle",
                ) for name, description in (
                    ("start_controller", "Start this owned arm controller on its existing CAN interface; vendor initialization can move motors."),
                    ("stop_controller", "Stop this owned controller process; this does not confirm motor disable or physical stop."),
                )} if self._controller_lifecycle is not None else {}),
                **({"recover": Procedure(
                    "recover",
                    "Recover this arm's owned controller after motor power cycling or a latched offline fault. Requires supported arm/object and a clear workspace; initialization reapplies torque and may move. Healthy controllers are left running. Does not home, calibrate the gripper, clear quarantine or resume old operations.",
                    {"site_ready": {"type": "boolean", "const": True}},
                    ["site_ready"],
                    completion="controller_feedback_verified",
                    effects="controller restart and motor enable",
                    permission="lifecycle",
                )} if self._controller_recovery is not None else {}),
                "move_joints": Procedure(
                    "move_joints",
                    "Interpolate six joint targets (absolute or relative) over duration_s; gripper holds.",
                    {
                        "kind": enum("absolute", "relative"),
                        "values": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 6,
                            "maxItems": 6,
                        },
                        "duration_s": num(0.2, 30),
                        "gripper_opening": num(0, 1),
                    },
                    ["kind", "values", "duration_s"],
                ),
                "move_eef": Procedure(
                    "move_eef",
                    "Move to an absolute controller-EEF pose in this arm's exact controller base frame. "
                    "Omitted target_rpy_rad holds the current orientation; [0,0,0] requests zero orientation. "
                    "Old relative fields are rejected. Travel/speed/acceleration and IK limits still apply. "
                    "Gripper holds unless gripper_opening is supplied.",
                    {**TARGET_PROPERTIES, "target_frame": {"type": "string", "const": f"{device_id}/base"},
                     "duration_s": num(0.2, 30), "gripper_opening": num(0, 1)},
                    [*TARGET_REQUIRED, "duration_s"],
                    completion="eef_pose_feedback_verified",
                ),
                "move_eef_path": Procedure(
                    "move_eef_path",
                    "Preflight and stream absolute controller-EEF waypoints in one exact arm base frame. "
                    "Omitted waypoint orientation holds the preceding orientation. All targets are "
                    "checked before dispatch; only the final endpoint gets arrival verification. "
                    "Truncation skips the remaining suffix. Gripper holds throughout.",
                    {"waypoints": WAYPOINTS_SCHEMA, "duration_s": num(0.2, 30),
                     "target_frame": {"type": "string", "const": f"{device_id}/base"}},
                    ["waypoints", "target_frame"],
                    completion="eef_pose_feedback_verified",
                ),
                "set_gripper": Procedure(
                    "set_gripper",
                    "Interpolate the gripper to an opening (1 open, 0 closed) over duration_s; joints hold.",
                    {"opening": num(0, 1), "duration_s": num(0.1, 10)},
                    ["opening"],
                    completion="command_stream_completed",
                ),
                "calibrate_gripper": Procedure(
                    "calibrate_gripper",
                    "Sweep the gripper from open to closed in steps with nothing between the fingers, record the feedback endpoints on the Body and reopen (Robotiq auto_calibrate equivalent).",
                    {
                        "steps": {"type": "integer", "minimum": 2, "maximum": 10},
                        "hold_s": num(0.1, 5),
                    },
                    [],
                    completion="calibration_recorded",
                    effects="gripper motion",
                ),
                "set_command_endpoints": Procedure(
                    "set_command_endpoints",
                    "Override the raw values the Body sends for opening 0 (closed_raw) and 1 (open_raw); never moves the gripper.",
                    {"closed_raw": num(), "open_raw": num()},
                    ["closed_raw", "open_raw"],
                    is_async=False,
                    completion="calibration_recorded",
                ),
                "joint_zero": Procedure(
                    "joint_zero",
                    "Move six joints to zero over duration_s (velocity capped); gripper must open.",
                    {
                        "duration_s": num(0.5, 30),
                        "gripper_action": enum("open"),
                    },
                    ["duration_s"],
                    completion="command_stream_completed",
                    effects="motion",
                ),
            },
            stream=StreamSpec(
                ["joint_targets_rad", "gripper_opening"],
                rate_hz_max=self._publish_hz,
                watchdog_s=0.25,
                description="actor target stream (complete mode-5 command per push)",
            ),
            observation_streams={"state": ObservationStreamSpec(
                ["moving", "joint_position_rad", "joint_velocity_raw", "joint_current_raw",
                 "gripper_position_raw", "gripper_current_raw", "gripper_opening",
                 "gripper_opening_basis", "gripper_opening_measured", "eef_position_m", "eef_rpy_rad"],
                retention_s=JOINT_HISTORY_S, max_samples=JOINT_HISTORY_LIMIT,
                source_semantics="controller_status_publication",
            )},
            calibration=self.calibration[device_id],
            unsupported=[
                "motor_online_confirmation",
                "grasp_ground_truth",
                "collision_planning",
            ],
            limitations=[
                "command completion is not physical arrival",
                f"CAN {spec['can']}",
            ],
            metadata={
                "side": spec["side"],
                "eef_coordinates": _arx_coordinate_metadata(device_id),
                "status_topic": spec["status_topic"],
                "command_topic": spec["command_topic"],
                "motion_limits": self.motion_limits.to_dict(),
                "passive_hold_policy": "deviation_warning_no_peer_cancel_v1",
                "joint_limits_rad": {
                    "lower": list(self._profiles[device_id].joint_lower),
                    "upper": list(self._profiles[device_id].joint_upper),
                },
                # Pinned by the environment (body/core/arm/contract.py); facts only, no prose.
                "identity": identity(self._profiles[device_id]),
                "model_sha256": self._profiles[device_id].model_sha256,
                "tracking_limits": self.tracking_limits.to_dict(),
                "tracking_recovery_protocol": "typed_deviation_v1_stop_then_replan",
                "eef_preflight": "sampled_current_branch_ik_and_sdk_joint_bounds",
                "eef_limit_policy": self.eef_limit_policy,
                "eef_xyz_policy": "clip_goal_then_preflight; attitude_and_joint_limits_unchanged",
                "eef_step_metric": "euclidean",
                "eef_path_max_waypoints": MAX_WAYPOINTS,
                "publish_hz": self._publish_hz,
                "trajectory_limit_scope": "commanded_targets_not_measured_motion",
            },
        )

    def manifest(self) -> BodyManifest:
        return BodyManifest(
            self.body_id,
            "arx_x5_normal",
            [self._device_manifest(d) for d in self.arms],
            description="ARX X5 arms via the vendor normal controllers",
        )

    def health(self) -> dict[str, Any]:
        arms: dict[str, Any] = {}
        for device_id, state in self._states.items():
            with state.lock:
                now = self._clock()
                arms[device_id] = {
                    "messages": state.messages,
                    "age_s": None
                    if state.received_at is None
                    else round(now - state.received_at, 4),
                    "moving": state.moving,
                    "feedback_error": state.feedback_error,
                }
            diagnostics = getattr(self.transport, "diagnostics", None)
            if diagnostics is not None:
                detail = diagnostics(self.arms[device_id]["side"])
                arms[device_id]["controller_ready"] = detail["ready"]
                arms[device_id]["fault"] = detail.get("fault")
                arms[device_id]["motor_feedback"] = detail.get("motor_feedback")
        return {
            "ok": all(
                v["age_s"] is not None
                and v["age_s"] < FEEDBACK_STALE_S
                and v.get("controller_ready", True)
                for v in arms.values()
            ),
            "arms": arms,
            "motion_limits": self.motion_limits.to_dict(),
            "eef_step_metric": "euclidean",
            "publish_hz": self._publish_hz,
        }

    # -- reads -----------------------------------------------------------------------------------

    def read(self, device_id: str, include: list[str] | None = None) -> dict[str, Any]:
        state = self._states[device_id]
        with state.lock:
            timing = dict(state.timing)
            history = None
            if include and "joint_history" in include:
                history = [
                    {"joint_position_rad": sample["payload"]["joint_position_rad"],
                     "timing": sample["timing"]}
                    for sample in state.joint_history.snapshot()
                ]
            joints, vel, cur, end, received_at, moving = (
                list(state.joint_pos) if state.joint_pos else None,
                list(state.joint_vel) if state.joint_vel else None,
                list(state.joint_cur) if state.joint_cur else None,
                list(state.end_pos) if state.end_pos else None,
                state.received_at,
                state.moving,
            )
        if joints is None:
            return {
                "signals": {},
                "availability": "missing",
                "freshness": "unknown",
                "age_s": None,
                "moving": moving,
                "source": {
                    "kind": "ros_message",
                    "topic": self.arms[device_id]["status_topic"],
                },
            }
        raw = joints[6]
        opening, basis = self.raw_to_opening(device_id, raw)
        signals: dict[str, Any] = {
            "joint_position_rad": [round(v, 5) for v in joints[:6]],
            "joint_velocity_raw": [round(v, 4) for v in (vel or [0.0] * 7)[:6]],
            "joint_current_raw": [round(v, 4) for v in (cur or [0.0] * 7)[:6]],
            "gripper_opening": None if opening is None else round(opening, 4),
            "gripper_opening_basis": basis,
            "gripper_opening_measured": basis == MEASURED_BASIS,
            "gripper_position_raw": round(raw, 5),
            "gripper_current_raw": round((cur or [0.0] * 7)[6], 4),
            "eef_position_m": [round(v, 4) for v in (end or [0.0] * 6)[:3]],
            "eef_rpy_rad": [round(v, 4) for v in (end or [0.0] * 6)[3:6]],
        }
        if include:
            if history is not None:
                signals["joint_history"] = history
            signals = {k: v for k, v in signals.items() if k in set(include)}
        age = None if received_at is None else self._clock() - received_at
        return {
            "signals": signals,
            "freshness": "unknown"
            if age is None
            else ("stale" if age > FEEDBACK_STALE_S else "fresh"),
            "age_s": None if age is None else round(age, 4),
            "timing": timing,
            "moving": moving,
            "source": {
                "kind": "ros_message",
                "topic": self.arms[device_id]["status_topic"],
            },
        }

    def read_observation_at(self, device_id, stream, at, *, mode="bracket", max_gap_s=0.05, include=None):
        if stream != "state":
            raise DeviceError(ErrorCode.NOT_SUPPORTED, "unknown arm observation stream")
        result = self._states[device_id].joint_history.query(at, mode=mode, max_gap_s=max_gap_s)
        for sample in result["samples"]:
            signals = sample.pop("payload")
            sample["signals"] = signals if include is None else {k: v for k, v in signals.items() if k in include}
        return result

    def check(self, device_id: str) -> dict[str, Any]:
        joints, _, received_at = self._snapshot(device_id)
        age = None if received_at is None else self._clock() - received_at
        fresh = age is not None and age <= FEEDBACK_STALE_S
        online = self._motors_online(device_id)
        plausible = joints is not None and all(
            abs(v) <= JOINT_LIMIT_RAD for v in joints[:6]
        )
        calibrated = bool(self.calibration[device_id].get("feedback"))
        traffic = self.can_traffic(device_id)
        can_state = (
            "unknown"
            if not traffic.get("available")
            else ("pass" if traffic["rx_packets_delta"] > 0 else "fail")
        )
        checks = {
            "feedback": {
                "state": "pass" if fresh else "fail",
                "reason": ""
                if fresh
                else (
                    "no RobotStatus yet" if age is None else f"feedback {age:.2f}s old"
                ),
            },
            "motors": {
                "state": "unknown"
                if online is None
                else ("pass" if online else "fail"),
                "reason": ""
                if online
                else "feedback is all zeros: motors offline or arm unpowered",
            },
            "can_traffic": {
                "state": can_state,
                "reason": ""
                if can_state != "fail"
                else f"no RX packets on {traffic.get('interface')}: motors offline or the controller's CAN socket is closed (frozen feedback); inspect the configured host instance and CAN link",
            },
            "gripper_calibration": {
                "state": "pass" if calibrated else "unknown",
                "reason": ""
                if calibrated
                else "feedback endpoints not measured yet: run calibrate_gripper (setup session)",
            },
        }
        checks["joint_envelope"] = {
            "state": "pass" if plausible else "fail",
            "reason": ""
            if plausible
            else "joint feedback outside configured envelope or unavailable",
        }
        ready = (
            fresh
            and plausible
            and online is True
            and (
                can_state == "pass"
                or not getattr(self.transport, "requires_can", False)
            )
        )
        diagnostics = getattr(self.transport, "diagnostics", None)
        if diagnostics is not None:
            detail = diagnostics(self.arms[device_id]["side"])
            checks["controller"] = {
                "state": "pass" if detail["ready"] else "fail",
                "reason": detail.get("fault", {}).get("reason", ""),
                "details": detail,
            }
            if "motor_feedback" in detail:
                feedback = detail["motor_feedback"]
                checks["motors"] = {
                    "state": "pass"
                    if feedback.get("available")
                    and feedback.get("all_online")
                    and feedback.get("all_fault_free")
                    else "fail",
                    "reason": ""
                    if feedback.get("all_online") and feedback.get("all_fault_free")
                    else "Per-motor feedback is missing, stale, disabled, or faulted.",
                    "details": feedback,
                }
            ready = ready and detail["ready"]
        if self._controller_lifecycle is not None:
            owner = self._controller_lifecycle(device_id, self.arms[device_id], action="status")
            checks["controller_process"] = {"state": "pass" if owner["owned"] else "fail", "details": owner}
            ready = ready and owner["owned"] and owner["running"] is True
        return {
            "readiness": "satisfied" if ready else "unsatisfied",
            "checks": checks,
            "physical_facts": {
                "can": self.arms[device_id]["can"],
                "can_traffic": traffic,
            },
        }

    # -- commands --------------------------------------------------------------------------------

    def _publish(
        self,
        device_id: str,
        joints: list[float],
        end_pos: list[float],
        gripper_raw: float,
        mode: int,
    ) -> None:
        command = {
            "joint_pos": [float(v) for v in joints[:6]],
            "end_pos": [float(v) for v in end_pos[:6]],
            "gripper": float(gripper_raw),
            "mode": int(mode),
        }
        self.transport.publish_command(self.arms[device_id]["command_topic"], command)
        self._states[device_id].last_command = command

    def _check_eef_path(self, device_id, joints, start, goal, *, path=None, path_length_m=None):
        try:
            if self._eef_preflight is not None:
                return self._eef_preflight(device_id, joints, start, goal, **(
                    {"path": path, "path_length_m": path_length_m} if path is not None else {}))
            return self._eef_checker(device_id).check(
                joints, start, goal, on_limit=self.eef_limit_policy, path=path, path_length_m=path_length_m,
                clip_xyz=True,
            )
        except DeviceError as exc:
            exc.details = {
                **(exc.details or {}),
                "target_frame": f"{device_id}/base",
                "start_eef": list(start),
                "start_joint_position_rad": list(joints),
                "requested_goal_end_pos": list(goal),
            }
            raise
        except Exception as exc:
            raise DeviceError(
                ErrorCode.PRECONDITION_FAILED,
                f"EEF preflight unavailable: {exc}",
                stage="preflight",
            ) from exc

    def _eef_checker(self, device_id):
        if device_id not in self._kinematics:
            from ...core.resources import runtime_root

            self._kinematics[device_id] = NativeKinematics(runtime_root(), profile=self._profiles[device_id])
        return EefPreflight(self._kinematics[device_id], profile=self._profiles[device_id])

    def _check_eef_goal(self, device_id, seeds, goal):
        """Endpoint-only IK evidence for a truncated path; it never admits a motion."""
        if self._eef_preflight is not None:
            return {"status": "not_checked", "reason": "injected_preflight_has_no_goal_check"}
        try:
            return self._eef_checker(device_id).goal_check(seeds, goal)
        except Exception as exc:
            return {"status": "not_checked", "reason": f"{type(exc).__name__}: {exc}"}

    def _tracking_fault(self, device_id, details):
        self._states[device_id].abort.set()
        # The existing hold uses fresh measured joints and retains the current
        # gripper target. A failed hold propagates as unknown, never as stopped.
        self.stream_hold(device_id)
        raise DeviceError(
            ErrorCode.TRACKING_ERROR,
            f"{device_id}: motion feedback did not satisfy the commanded path or arrival",
            details={
                **details,
                "tracking_limits": self.tracking_limits.to_dict(),
                "hold_command_sent": True,
                "physical_stop_confirmed": False,
            },
            stage="tracking",
            execution_outcome="partial",
        )

    def _prepare_eef_waypoints(
        self, device_id, joints, start, waypoints, duration_s, cancel
    ):
        segments, reports = [], []
        origin, seed = list(start), list(joints)
        truncated, limit = False, None
        # Resolve/check every segment before even a prefix can be streamed.
        nominal, goals, clippings = list(start), [], []
        try:
            for point in waypoints:
                nominal = resolve_goal(nominal, point, self.motion_limits)
                effective, clipping = clip_eef_xyz(nominal)
                goals.append(effective)
                clippings.append(clipping)
        except ValueError as exc:
            raise DeviceError(ErrorCode.LIMIT_VIOLATION, str(exc), stage="preflight") from exc
        for index, point in enumerate(waypoints):
            if cancel.is_set() or self._states[device_id].abort.is_set():
                return None, None
            goal = goals[index]
            try:
                report = self._check_eef_path(device_id, seed, origin, goal)
            except DeviceError as exc:
                # If the next span has no executable prefix, an earlier checked
                # endpoint can still be used, provided it has the braking margin.
                margin_ok = all(
                    lo + TRUNCATION_JOINT_MARGIN_RAD
                    <= q
                    <= hi - TRUNCATION_JOINT_MARGIN_RAD
                    for lo, q, hi in zip(self._profiles[device_id].joint_lower, seed, self._profiles[device_id].joint_upper)
                )
                if (
                    self.eef_limit_policy != "truncate"
                    or exc.code != ErrorCode.LIMIT_VIOLATION
                    or not segments
                    or not margin_ok
                ):
                    raise
                truncated, limit = (
                    True,
                    {"waypoint_index": index, "error": exc.to_dict()},
                )
                break
            if report.get("entry_recovery"):
                raise DeviceError(ErrorCode.PRECONDITION_FAILED,
                    "Joint entry recovery requires a single Move target, not legacy EEF waypoints", stage="preflight",
                    details={"entry_recovery": report["entry_recovery"], "hardware_command_sent": False})
            reports.append(report)
            truncated = bool(report.get("truncated"))
            actual_goal = report.get("effective_goal_end_pos", goal)
            segments.append(
                (
                    actual_goal,
                    max(0.2, point["duration_s"] * report.get("path_fraction", 1)),
                )
            )
            if truncated:
                limit = {"waypoint_index": index, "error": report.get("limit")}
                break
            if (clippings[index] or {}).get("requires_replan"):
                # Later waypoints may depend on reaching the original target.
                break
            seed = report["goal_joints_rad"]
            origin = list(actual_goal)
        try:
            trajectory = EefTrajectory(start, segments, self.motion_limits, duration_s)
        except ValueError as exc:
            raise DeviceError(
                ErrorCode.INVALID_ARGUMENTS, str(exc), stage="preflight"
            ) from exc
        return trajectory, {
            "checked": True,
            "truncated": truncated,
            "limit": limit,
            "requested_waypoints": len(waypoints),
            "executed_waypoints": len(segments),
            "skipped_waypoints": len(waypoints) - len(segments),
            "continuous_spans": len(trajectory.spans),
            "effective_goal_end_pos": trajectory.goal,
            "samples": sum(report.get("samples", 0) for report in reports),
            "elapsed_ms": sum(report.get("elapsed_ms", 0) for report in reports),
            "collision_checked": False,
            "workspace_clippings": [{"waypoint_index": i, **item}
                                    for i, item in enumerate(clippings[:len(segments)]) if item],
            "truncated_waypoint_fraction": reports[-1].get("path_fraction", 1)
            if reports
            else None,
        }

    def _wait_motion_arrival(
        self,
        device_id,
        goal,
        before_publish,
        final_sent_at,
        cancel,
        guard,
        *,
        eef,
        require_open=False,
        passive_hold=False,
    ):
        state = self._states[device_id]
        began = self._clock()
        window = ArrivalWindow(self.tracking_limits)
        while True:
            with state.command_lock:
                if cancel.is_set() or state.abort.is_set():
                    self.stream_hold(device_id)
                    return None
                self._require_fresh(device_id)
                with state.lock:
                    actual, joints, stamp, messages = (
                        list(state.end_pos) if eef else list(state.joint_pos[:6]),
                        list(state.joint_pos),
                        state.received_at,
                        state.messages,
                    )
                if eef:
                    pos, rot = pose_error(actual, goal)
                    errors = {"position_error_m": pos, "rotation_error_rad": rot}
                    in_tolerance = (
                        pos <= self.tracking_limits.settle_position_m
                        and rot <= self.tracking_limits.settle_rotation_rad
                    )
                else:
                    error = max(abs(a - b) for a, b in zip(actual, goal))
                    errors = {"max_joint_error_rad": error}
                    in_tolerance = error <= self.tracking_limits.settle_joint_rad
                if require_open:
                    opening, basis = self.raw_to_opening(device_id, joints[6])
                    open_ok = (
                        opening is not None
                        and 1 - opening <= self.tracking_limits.settle_gripper_opening
                    )
                    errors.update(
                        {
                            "gripper_opening": opening,
                            "gripper_opening_basis": basis,
                            "gripper_opening_measured": basis == MEASURED_BASIS,
                            "gripper_open_verified": open_ok,
                        }
                    )
                    in_tolerance = in_tolerance and open_ok
                if (
                    messages > before_publish
                    and stamp >= final_sent_at
                    and window.update(stamp, in_tolerance)
                ):
                    workspace = workspace_status(actual) if eef else None
                    return {
                        "verified": workspace is None or workspace["within_limits"],
                        "pose_tolerance_satisfied": True,
                        "workspace": workspace,
                        "reason": "measured_workspace_outside" if workspace and not workspace["within_limits"] else "settled",
                        "basis": "consecutive_robot_status_after_final_command",
                        **errors,
                        "confirmation_s": stamp - window.since,
                        "feedback_samples": window.samples,
                        "settle_wait_s": self._clock() - began,
                    }
                violation = guard.check(stamp, actual) if guard is not None else None
                if violation:
                    self._tracking_fault(device_id, violation)
                remaining = self.tracking_limits.settle_timeout_s - (
                    self._clock() - began
                )
                if remaining <= 0:
                    if passive_hold and not eef:
                        return {"verified": False, "reason": "arrival_timeout",
                                "severity": "warning", "space": "joint", **errors,
                                "settle_wait_s": self._clock() - began,
                                "peer_cancelled": False}
                    self._tracking_fault(
                        device_id,
                        {
                            "reason": "arrival_timeout",
                            "space": "eef" if eef else "joint",
                            **errors,
                        },
                    )
            # Wake on feedback; repeated polls cannot advance the confirmation.
            with state.feedback_changed:
                if state.messages == messages:
                    state.feedback_changed.wait(min(0.02, remaining))

    def _interpolate(
        self,
        device_id: str,
        *,
        goal_joints: list[float] | None = None,
        goal_end_pos: list[float] | None = None,
        eef_target: dict | None = None,
        eef_waypoints: list[dict] | None = None,
        goal_opening: float | None = None,
        duration_s: float,
        cancel: threading.Event,
        progress: Callable[[float], None] | None = None,
        prepared_motion: dict | None = None,
        passive_hold: bool = False,
    ) -> dict[str, Any]:
        if cancel.is_set():
            return {
                "completed": False,
                "partial": False,
                "hardware_command_sent": False,
            }
        state = self._states[device_id]
        with state.command_lock:
            state.abort.clear()
        joints, end_pos = self._require_fresh(device_id)
        start_joints, start_end, start_raw = joints[:6], end_pos[:6], joints[6]
        hold_reference = getattr(state, "hold_reference", None)
        if passive_hold and hold_reference is None:
            hold_reference = {"joints": list(start_joints), "pose": list(start_end),
                              "basis": "first_passive_hold_measured_start"}
            state.hold_reference = hold_reference
        preflight = None
        trajectory = None
        deferred_opening = None
        if eef_target is not None:
            try:
                goal_end_pos = resolve_goal(start_end, eef_target, self.motion_limits)
            except ValueError as exc:
                raise DeviceError(ErrorCode.LIMIT_VIOLATION, str(exc), stage="preflight") from exc
        requested_end = list(goal_end_pos) if goal_end_pos is not None else None
        workspace_clipping = None
        if goal_end_pos is not None and prepared_motion is None:
            goal_end_pos, workspace_clipping = clip_eef_xyz(goal_end_pos)
        if goal_joints is not None:
            check_joint_target(goal_joints, start=start_joints, profile=self._profiles[device_id])
        if eef_waypoints is not None:
            trajectory, preflight = self._prepare_eef_waypoints(
                device_id, start_joints, start_end, eef_waypoints, duration_s, cancel
            )
            if trajectory is None:
                return {
                    "completed": False,
                    "partial": False,
                    "hardware_command_sent": False,
                }
            goal_end_pos = trajectory.goal
        elif goal_end_pos is not None:
            if prepared_motion is not None:
                dp, dr = pose_error(start_end, prepared_motion["start"])
                if (max(abs(a-b) for a,b in zip(start_joints, prepared_motion["joints"])) > self.tracking_limits.preflight_joint_rad
                        or dp > self.tracking_limits.preflight_position_m or dr > self.tracking_limits.preflight_rotation_rad or list(goal_end_pos) != prepared_motion["goal"]):
                    raise DeviceError(ErrorCode.PRECONDITION_FAILED,
                        "Arm moved after continuous group preflight", stage="preflight")
                preflight = prepared_motion["report"]
            else:
                try:
                    preflight = self._check_eef_path(device_id, start_joints, start_end, goal_end_pos)
                except DeviceError as exc:
                    exc.details = {**(exc.details or {}), "original_requested_end_pos": requested_end,
                                   "workspace_clipping": workspace_clipping}
                    raise
            if preflight.get("entry_recovery"):
                current_joints, current_end = self._require_fresh(device_id)
                dp, dr = pose_error(current_end, start_end)
                if max(abs(a-b) for a,b in zip(current_joints[:6], start_joints)) > self.tracking_limits.preflight_joint_rad or dp > self.tracking_limits.preflight_position_m or dr > self.tracking_limits.preflight_rotation_rad:
                    raise DeviceError(ErrorCode.PRECONDITION_FAILED, "Arm moved during recovery preflight", stage="preflight")
                result = self._interpolate(device_id,
                    goal_joints=preflight["goal_joints_rad"], duration_s=duration_s,
                    cancel=cancel, progress=progress)
                raise DeviceError(ErrorCode.PATH_TRUNCATED,
                    "Joint entry recovery only; re-observe and replan original target",
                    stage="completed_prefix", execution_outcome="partial" if result.get("hardware_command_sent") else "none",
                    details={**result, "entry_recovery": preflight["entry_recovery"],
                             "requested_targets_reached": False, "original_requested_end_pos": requested_end})
            if preflight.get("truncated"):
                goal_end_pos = preflight["effective_goal_end_pos"]
                duration_s = max(0.2, duration_s * preflight["path_fraction"])
                deferred_opening, goal_opening = goal_opening, None
        if goal_end_pos is not None:
            current_joints, current_end = self._require_fresh(device_id)
            if (
                max(abs(a - b) for a, b in zip(current_joints[:6], start_joints)) > self.tracking_limits.preflight_joint_rad
                or pose_error(current_end, start_end)[0] > self.tracking_limits.preflight_position_m
                or pose_error(current_end, start_end)[1] > self.tracking_limits.preflight_rotation_rad
            ):
                raise DeviceError(
                    ErrorCode.PRECONDITION_FAILED,
                    "arm moved during EEF preflight; re-read and replan",
                    stage="preflight",
                )
        start_opening, _ = self.raw_to_opening(device_id, start_raw)
        held_raw = self._states[device_id].gripper_target
        goal_raw = (
            (start_raw if held_raw is None else held_raw)
            if goal_opening is None
            else self.opening_to_raw(device_id, goal_opening)
        )
        needed = 0.0
        if goal_joints is not None:
            needed = max(
                needed,
                max(abs(g - s) for g, s in zip(goal_joints, start_joints))
                / self.motion_limits.joint_speed_rad_s,
            )
        if goal_opening is not None and start_opening is not None:
            needed = max(
                needed,
                abs(goal_opening - start_opening)
                / self.motion_limits.gripper_speed_fraction_s,
            )
        duration = max(float(duration_s), QUINTIC_PEAK_SPEED * needed)
        if goal_end_pos is not None:
            duration = max(
                duration,
                self.motion_limits.eef_duration_s(
                    math.dist(goal_end_pos[:3], start_end[:3]),
                    rotation_distance(start_end[3:], goal_end_pos[3:]),
                ),
            )
        if trajectory is not None:
            duration = trajectory.duration
        mode = MODE_EEF if goal_end_pos is not None else MODE_JOINT
        if not passive_hold:
            # Clear only after admission, immediately before execution. A
            # rejected plan must not erase accumulated passive-hold evidence.
            state.hold_reference = None
        state.moving = True
        ticker = Ticker(self._publish_hz)
        began = self._clock()
        guard = (
            PoseTrackingGuard(self.tracking_limits, began, start_end)
            if goal_end_pos is not None
            else None
        )
        arrival = None
        hold_peak_error = 0.0
        sent = False
        completed = True
        clipped_command_count = 0
        clipped_command_examples = []
        max_xyz_correction = 0.
        last_sent_end = None
        # Progress counts published ticks, not wall time: a slow readiness check
        # pauses the trajectory instead of making the next setpoint jump ahead.
        ticks = 0
        try:
            while True:
                frac = (
                    1.0
                    if duration <= 0
                    else min(1.0, ticks / (duration * self._publish_hz))
                )
                blend = min(1.0, max(0.0, frac**3 * (10 + frac * (-15 + 6 * frac))))
                with state.command_lock:
                    if cancel.is_set() or state.abort.is_set():
                        completed = False
                        self.stream_hold(device_id)
                        break
                    fresh_joints, _ = self._require_fresh(device_id)
                    if passive_hold:
                        hold_peak_error = max(hold_peak_error, max(abs(a-b) for a,b in zip(fresh_joints[:6], goal_joints)))
                    if guard is not None:
                        _, actual, stamp = self._snapshot(device_id)
                        violation = guard.check(stamp, actual)
                        if violation:
                            self._tracking_fault(device_id, violation)
                    joints_cmd = (
                        [a + (b - a) * blend for a, b in zip(start_joints, goal_joints)]
                        if goal_joints is not None
                        else start_joints
                    )
                    end_cmd = (
                        trajectory.at(frac * duration)
                        if trajectory is not None
                        else (
                            (prepared_motion["path"].at(blend) if prepared_motion is not None
                             else interpolate_pose(start_end, goal_end_pos, blend))
                            if goal_end_pos is not None
                            else start_end
                        )
                    )
                    if mode == MODE_EEF:
                        end_cmd, clipping = clip_eef_xyz(end_cmd)
                        if clipping is not None:
                            clipped_command_count += 1
                            max_xyz_correction = max(max_xyz_correction, clipping["correction_distance_m"])
                            if len(clipped_command_examples) < 8:
                                clipped_command_examples.append({"time_fraction": frac, **clipping})
                    gripper_cmd = (
                        goal_raw
                        if goal_opening is None
                        else start_raw + (goal_raw - start_raw) * blend
                    )
                    with state.lock:
                        before_publish = state.messages
                    if guard is not None:
                        guard.sent(self._clock(), end_cmd)
                    sent = True
                    final_sent_at = self._clock()
                    self._publish(device_id, joints_cmd, end_cmd, gripper_cmd, mode)
                    last_sent_end = list(end_cmd) if mode == MODE_EEF else None
                    state.gripper_target = gripper_cmd
                if progress is not None:
                    progress(frac)
                if frac >= 1.0:
                    arrival = self._wait_motion_arrival(
                        device_id,
                        goal_end_pos if mode == MODE_EEF else joints_cmd,
                        before_publish,
                        final_sent_at,
                        cancel,
                        guard,
                        eef=mode == MODE_EEF,
                        require_open=goal_opening == 1.0,
                        passive_hold=passive_hold,
                    )
                    if arrival is None:
                        completed = False
                    break
                ticker.wait()
                ticks += 1
        except DeviceError as exc:
            exc.details = {**(exc.details or {}), "workspace_clipping": workspace_clipping,
                           "effective_command_end_pos": goal_end_pos,
                           "last_sent_end_pos": last_sent_end,
                           "workspace_path_clipping": {"command_count": clipped_command_count,
                               "max_correction_m": max_xyz_correction, "samples": clipped_command_examples}}
            if prepared_motion is not None:
                exc.details = {**(exc.details or {}),
                    "hardware_command_sent": sent,
                    "command_fraction": frac if sent else 0.,
                    "fraction_semantics": "commanded_time_fraction_not_measured_completion",
                    "start_end_pos": start_end, "requested_end_pos": requested_end,
                    "complete_goal_checked": bool(prepared_motion["report"].get("complete_goal_checked"))}
            # Feedback/CAN loss after dispatch leaves physical outcome unknown.
            # Never construct a hold from stale or disconnected feedback.
            if exc.execution_outcome == "partial":
                raise
            raise DeviceError(
                exc.code,
                str(exc),
                details=exc.details,
                stage=exc.stage,
                execution_outcome="unknown" if sent else "none",
            ) from exc
        finally:
            state.moving = False
        fb_joints, fb_end, _ = self._snapshot(device_id)
        measured_workspace = workspace_status(fb_end)
        fb_opening, basis = (
            (None, "unknown")
            if fb_joints is None
            else self.raw_to_opening(device_id, fb_joints[6])
        )
        result = {
            "mode": mode,
            "hardware_command_sent": sent,
            "duration_s_effective": round(duration, 3),
            "interpolation": "continuous_polyline_quintic_collinear_merge"
            if trajectory is not None
            else "quintic_zero_endpoint_velocity_acceleration",
            "orientation_interpolation": "quaternion_shortest_arc_slerp" if mode == MODE_EEF else None,
            "publish_hz": self._publish_hz,
            "completed": completed,
            "motion_limits": self.motion_limits.to_dict(),
            "eef_preflight": preflight,
            "eef_arrival": arrival if mode == MODE_EEF else None,
            "joint_arrival": arrival if mode == MODE_JOINT else None,
            "completion_basis": ("hold_command_finished_with_warning" if passive_hold and arrival and not arrival.get("verified")
                                 else "arm_feedback_settled") if completed else "interrupted",
            "gripper_arrival_required": goal_opening == 1.0,
            "partial": not completed,
            "feedback_joints_rad": None
            if fb_joints is None
            else [round(v, 4) for v in fb_joints[:6]],
            "eef_coordinates": _arx_coordinate_metadata(device_id) if goal_end_pos is not None else None,
            "frame_id": f"{device_id}/base" if goal_end_pos is not None else None,
            "start_end_pos": list(start_end) if goal_end_pos is not None else None,
            "requested_end_pos": requested_end,
            "workspace_clipping": workspace_clipping,
            "last_sent_end_pos": last_sent_end,
            "workspace_path_clipping": {"command_count": clipped_command_count,
                "max_correction_m": max_xyz_correction, "samples": clipped_command_examples,
                "sample_basis": "published commands, not measured poses"},
            "workspace_clippings": (preflight or {}).get("workspace_clippings", []),
            "requested_targets_reached": completed and (not passive_hold or bool(arrival and arrival.get("verified"))) and not (workspace_clipping or {}).get("requires_replan")
                and not any(item.get("requires_replan") for item in (preflight or {}).get("workspace_clippings", []))
                and not (preflight or {}).get("truncated"),
            "goal_end_pos": list(goal_end_pos) if goal_end_pos is not None else None,
            "requested_target": eef_target,
            "feedback_eef_position_m": None
            if fb_end is None
            else [round(v, 4) for v in fb_end[:3]],
            "feedback_eef_rpy_rad": None
            if fb_end is None
            else [round(v, 4) for v in fb_end[3:]],
            "feedback_gripper_opening": None
            if fb_opening is None
            else round(fb_opening, 4),
            "gripper_opening_basis": basis,
            "gripper_opening_measured": basis == MEASURED_BASIS,
            "feedback_gripper_raw": None
            if fb_joints is None
            else round(fb_joints[6], 4),
        }
        result["measured_workspace"] = measured_workspace
        result["workspace_recovery_required"] = not measured_workspace["within_limits"]
        if not measured_workspace["within_limits"]:
            result["requested_targets_reached"] = False
            if completed and not passive_hold:
                result["completion_basis"] = "measured_workspace_outside"
            result["prefix_reason"] = {"code": "measured_workspace_outside", "requires_replan": True,
                                       "workspace": measured_workspace}
        if completed and not passive_hold and measured_workspace["within_limits"]:
            state.hold_reference = {"joints": list(fb_joints[:6]), "pose": list(fb_end),
                                    "basis": "last_completed_active_action_feedback"}
        if passive_hold:
            final_error = None if fb_joints is None else max(abs(a-b) for a,b in zip(fb_joints[:6], goal_joints))
            hold_peak_error = max(hold_peak_error, final_error or 0., (arrival or {}).get("max_joint_error_rad", 0.))
            cumulative_joint_error = (None if fb_joints is None else
                max(abs(a-b) for a,b in zip(fb_joints[:6], hold_reference["joints"])))
            cumulative_pose_error = None if fb_end is None else pose_error(fb_end, hold_reference["pose"])
            result["hold_report"] = {
                "role": "passive_hold", "policy": "report_deviation_after_group_no_peer_cancel",
                "threshold_rad": self.tracking_limits.settle_joint_rad,
                "max_joint_error_rad": hold_peak_error, "final_joint_error_rad": final_error,
                "command_joint_target_rad": list(goal_joints),
                "actual_joints_rad": None if fb_joints is None else list(fb_joints[:6]),
                "arrival_verified": bool(arrival and arrival.get("verified")),
                "measured_workspace": measured_workspace,
                "reference": hold_reference,
                "cumulative_joint_error_rad": cumulative_joint_error,
                "cumulative_position_error_m": None if cumulative_pose_error is None else cumulative_pose_error[0],
                "cumulative_rotation_error_rad": None if cumulative_pose_error is None else cumulative_pose_error[1],
                "status": "interrupted" if not completed else "deviation_warning" if (
                    not measured_workspace["within_limits"]
                    or (cumulative_joint_error is not None and cumulative_joint_error > self.tracking_limits.settle_joint_rad)
                    or hold_peak_error > self.tracking_limits.settle_joint_rad or not (arrival or {}).get("verified")) else "within_tolerance",
                "peer_cancelled_for_deviation": False,
                "next_action": "Read fresh measured state and observations before planning the next action; do not replay previous motion.",
            }
        if completed and preflight and preflight.get("truncated"):
            raise DeviceError(
                ErrorCode.PATH_TRUNCATED,
                f"{device_id}: reached the safe prefix; remaining target was not executed",
                stage="completed_prefix",
                execution_outcome="partial",
                details={
                    **result,
                    "completed": False,
                    "partial": True,
                    "prefix_completed": True,
                    "deferred_gripper_opening": deferred_opening,
                },
            )
        return result

    def write(
        self,
        device_id: str,
        command: str,
        value: list[float],
        *,
        cancel: threading.Event,
    ) -> dict[str, Any]:
        if command == "joint_targets_rad":
            return self._interpolate(
                device_id,
                goal_joints=[float(v) for v in value],
                duration_s=0.5,
                cancel=cancel,
            )
        if command == "gripper_opening":
            return self._interpolate(
                device_id, goal_opening=float(value[0]), duration_s=0.5, cancel=cancel
            )
        raise DeviceError(ErrorCode.UNKNOWN_COMMAND, f"unknown command {command!r}")

    def _sleep(self, seconds: float, cancel: threading.Event) -> None:
        cancel.wait(max(0.0, seconds))

    def invoke(
        self,
        device_ids: list[str],
        procedure: str,
        params: dict[str, Any],
        *,
        cancel: threading.Event,
        progress: Callable[[float], None],
    ) -> dict[str, Any]:
        if procedure == "move_eef_group":
            from .continuous import move_group
            return move_group(self, device_ids, params, cancel, progress)
        if procedure == "joint_zero":
            results: dict[str, Any] = {}
            action = str(params.get("gripper_action", "open"))
            if action != "open":
                raise DeviceError(
                    ErrorCode.INVALID_ARGUMENTS,
                    "joint_zero requires gripper_action=open",
                )
            for i, device_id in enumerate(device_ids):
                results[device_id] = self._interpolate(
                    device_id,
                    goal_joints=[0.0] * 6,
                    goal_opening=1.0,
                    duration_s=max(2.0, float(params.get("duration_s", 3.0))),
                    cancel=cancel,
                    progress=lambda v, i=i, n=len(device_ids): progress((i + v) / n),
                )
                if not results[device_id]["completed"]:
                    return {"devices": results, "partial": True, "completed": False}
            return {
                "devices": results,
                "partial": False,
                "completed": True,
                "gripper_action": action,
            }
        if len(device_ids) != 1:
            raise DeviceError(
                ErrorCode.INVALID_ARGUMENTS, f"{procedure} takes exactly one device"
            )
        device_id = device_ids[0]
        if procedure in {'enter_soft_teach', 'resume_teach_hold'}:
            from .teaching import set_teaching
            return set_teaching(self, device_id, procedure, params, cancel)
        if procedure == "recover":
            return self._recover_controller(device_id, params, cancel, progress)
        if procedure in {"start_controller", "stop_controller"}:
            if self._controller_lifecycle is None or params != {"site_ready": True} or params["site_ready"] is not True:
                raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "controller lifecycle requires site_ready=true")
            state = self._states[device_id]
            if not state.command_lock.acquire(blocking=False):
                raise DeviceError(ErrorCode.RESOURCE_BUSY, "arm command is in progress")
            try:
                # The command lock is released between motion ticks; a running motion still owns the arm.
                if state.moving:
                    raise DeviceError(ErrorCode.RESOURCE_BUSY, "arm motion is in progress")
                result = self._controller_lifecycle(device_id, self.arms[device_id],
                    action="start" if procedure == "start_controller" else "stop", cancel=cancel)
                if result.get("started"):
                    self._reset_controller_cache(device_id)
                    self._wait_controller_feedback(device_id, cancel)
                progress(1.0)
                return result
            finally:
                state.command_lock.release()
        if procedure in {"move_joints", "move_eef", "set_gripper"}:
            duration = float(params.get("duration_s", 1.0))
            if not math.isfinite(duration) or not 0 < duration <= 30:
                raise DeviceError(
                    ErrorCode.INVALID_ARGUMENTS,
                    "duration_s must be finite and in (0, 30]",
                )
            for key, count in (
                ("values", 6),
                ("target_position_m", 3),
                ("target_rpy_rad", 3),
            ):
                if key in params:
                    vector = params[key]
                    if (
                        not isinstance(vector, list)
                        or len(vector) != count
                        or any(
                            isinstance(v, bool)
                            or not isinstance(v, (int, float))
                            or not math.isfinite(v)
                            for v in vector
                        )
                    ):
                        raise DeviceError(
                            ErrorCode.INVALID_ARGUMENTS,
                            f"{key} needs {count} finite numbers",
                        )
            if procedure == "move_joints" and params.get("kind") not in {
                "relative",
                "absolute",
            }:
                raise DeviceError(
                    ErrorCode.INVALID_ARGUMENTS, "unknown joint target kind"
                )
            if procedure == "set_gripper":
                opening = float(params["opening"])
                if not math.isfinite(opening) or not 0 <= opening <= 1:
                    raise DeviceError(
                        ErrorCode.INVALID_ARGUMENTS, "opening must be in [0, 1]"
                    )
        compound_opening = params.get("gripper_opening")
        if compound_opening is not None:
            compound_opening = float(compound_opening)
            if not math.isfinite(compound_opening) or not 0 <= compound_opening <= 1:
                raise DeviceError(
                    ErrorCode.INVALID_ARGUMENTS, "gripper_opening must be in [0, 1]"
                )
        if procedure == "move_joints":
            joints, _ = self._require_fresh(device_id)
            values = [float(v) for v in params["values"]]
            if params["kind"] == "relative":
                if max(abs(v) for v in values) > self.motion_limits.joint_step_rad:
                    raise DeviceError(
                        ErrorCode.LIMIT_VIOLATION,
                        f"relative delta exceeds {self.motion_limits.joint_step_rad} rad per call",
                        details={"values": values},
                    )
                goal = [a + b for a, b in zip(joints[:6], values)]
            else:
                goal = values
            check_joint_target(goal, start=joints[:6], profile=self._profiles[device_id])
            result = self._interpolate(
                device_id,
                goal_joints=goal,
                goal_opening=compound_opening,
                duration_s=float(params["duration_s"]),
                cancel=cancel,
                progress=progress,
            )
            return {
                **result,
                "start_joints_rad": [round(v, 4) for v in joints[:6]],
                "goal_joints_rad": [round(v, 4) for v in goal],
            }
        if procedure == "move_eef_path":
            try:
                validate_target([0, 0, 0], None, params.get("target_frame"), device_id)
                waypoints = validate_waypoints(params.get("waypoints"), self.motion_limits)
                duration = params.get("duration_s", 0.2)
                if (
                    isinstance(duration, bool)
                    or not isinstance(duration, (int, float))
                    or not math.isfinite(duration)
                    or not 0.2 <= duration <= 30
                ):
                    raise ValueError("duration_s must be in [0.2, 30]")
                if set(params) - {"waypoints", "duration_s", "target_frame"}:
                    raise ValueError(
                        "unsupported path fields; gripper holds throughout"
                    )
            except ValueError as exc:
                raise DeviceError(ErrorCode.INVALID_ARGUMENTS, str(exc)) from exc
            return self._interpolate(
                device_id,
                eef_waypoints=waypoints,
                duration_s=duration,
                cancel=cancel,
                progress=progress,
            )
        if procedure == "move_eef":
            try:
                if set(params) - {*TARGET_PROPERTIES, "duration_s", "gripper_opening"}:
                    raise ValueError("move_eef uses absolute targets; translation_m/rpy_delta_rad are no longer accepted")
                target = validate_target(params.get("target_position_m"), params.get("target_rpy_rad"), params.get("target_frame"), device_id)
            except ValueError as exc:
                raise DeviceError(ErrorCode.INVALID_ARGUMENTS, str(exc)) from exc
            return self._interpolate(
                device_id, eef_target=target, goal_opening=compound_opening,
                duration_s=float(params["duration_s"]), cancel=cancel, progress=progress,
            )
        if procedure == "set_gripper":
            result = self._interpolate(
                device_id,
                goal_opening=float(params["opening"]),
                duration_s=float(params.get("duration_s", 1.0)),
                cancel=cancel,
                progress=progress,
            )
            return {
                **result,
                "requested_opening": float(params["opening"]),
                "resolved_position_raw": round(
                    self.opening_to_raw(device_id, float(params["opening"])), 4
                ),
            }
        if procedure == "calibrate_gripper":
            return self._calibrate_gripper(
                device_id,
                steps=int(params.get("steps", 4)),
                hold_s=float(params.get("hold_s", self._hold_s_default)),
                cancel=cancel,
                progress=progress,
            )
        if procedure == "set_command_endpoints":
            closed, open_ = float(params["closed_raw"]), float(params["open_raw"])
            if closed == open_ or not all(math.isfinite(v) for v in (closed, open_)):
                raise DeviceError(
                    ErrorCode.INVALID_ARGUMENTS,
                    "closed_raw and open_raw must be finite and different",
                )
            with self._calibration_lock:
                self.calibration[device_id] = {
                    "command": {"closed_raw": closed, "open_raw": open_},
                    "feedback": None,
                    "measured_at": None,
                }
                self._save_calibration()
            return {
                "calibration": self.calibration[device_id],
                "hardware_command_sent": False,
                "completed": True,
            }
        raise DeviceError(
            ErrorCode.UNKNOWN_PROCEDURE, f"unknown procedure {procedure!r}"
        )

    def _recover_controller(self, device_id, params, cancel, progress):
        if self._controller_recovery is None:
            raise DeviceError(ErrorCode.NOT_SUPPORTED, "host controller recovery is not configured")
        if set(params) != {"site_ready"} or params["site_ready"] is not True:
            raise DeviceError(
                ErrorCode.INVALID_ARGUMENTS,
                "recover requires site_ready=true after confirming arm/object support and a clear workspace",
            )
        state = self._states[device_id]
        if not state.command_lock.acquire(blocking=False):
            raise DeviceError(ErrorCode.RESOURCE_BUSY, "arm command is in progress")
        try:
            if state.moving:
                raise DeviceError(ErrorCode.RESOURCE_BUSY, "arm motion is in progress")
            if cancel.is_set():
                return {"completed": False, "restarted": False, "partial": False}
            result = self._controller_recovery(
                device_id, self.arms[device_id], cancel=cancel,
                preflight=lambda: self._recovery_preflight(device_id, cancel),
                progress=progress,
            )
            if result.get("restarted"):
                self._reset_controller_cache(device_id)
                self._wait_controller_feedback(device_id, cancel)
            if result.get("completed"):
                progress(1.0)
            return result
        finally:
            state.command_lock.release()

    def _reset_controller_cache(self, device_id):
        state = self._states[device_id]
        with state.lock:
            state.last_command = None
            state.gripper_target = None
            state.joint_history.clear()
            state.can_sample = None

    def _wait_controller_feedback(self, device_id, cancel, *, timeout_s=20.0, stable_s=2.0):
        """Allow ROS service rediscovery; require continuous fresh Body feedback."""
        deadline, since, detail = time.monotonic() + timeout_s, None, {}
        while time.monotonic() < deadline and not cancel.is_set():
            detail = self.transport.diagnostics(self.arms[device_id]["side"], refresh=True)
            ready = detail.get("ready") is True
            if ready:
                try:
                    self._require_fresh(device_id)
                except DeviceError:
                    ready = False
            if ready:
                since = time.monotonic() if since is None else since
                if time.monotonic() - since >= stable_s:
                    return
            else:
                since = None
                if any(detail.get(k) is False for k in ("configuration", "sdk_identity_matches", "exclusive_writer")):
                    break
            cancel.wait(0.1)
        raise DeviceError(ErrorCode.OUTCOME_UNKNOWN,
            "Body controller service/feedback did not remain ready after restart",
            execution_outcome="unknown", details=detail)

    def _recovery_preflight(self, device_id, cancel):
        diagnostics = getattr(self.transport, "diagnostics", None)
        if diagnostics is None:
            raise DeviceError(ErrorCode.PRECONDITION_FAILED, "controller diagnostics unavailable")
        initial = None
        started = time.monotonic()
        while True:
            if cancel.is_set():
                raise DeviceError(ErrorCode.PRECONDITION_FAILED, "recovery cancelled before restart")
            detail = diagnostics(self.arms[device_id]["side"], refresh=True)
            feedback = detail.get("motor_feedback", {})
            motors = feedback.get("motors", {})
            joints, _, stamp = self._snapshot(device_id)
            valid = (
                all(detail.get(k) is True for k in ("configuration", "status_source", "exclusive_writer"))
                and feedback.get("available") is True
                and feedback.get("all_online") is True
                and set(motors) == {"1", "2", "4", "5", "6", "7", "8"}
                and all(m["fault"] in ((0, -1) if k in {"5", "6", "7", "8"} else (0,)) for k, m in motors.items())
                and joints is not None and stamp is not None
                and 0 <= self._clock() - stamp <= FEEDBACK_STALE_S
                and all(lo <= q <= hi for q, lo, hi in zip(joints[:6], self._profiles[device_id].joint_lower, self._profiles[device_id].joint_upper))
            )
            if not valid:
                raise DeviceError(
                    ErrorCode.PRECONDITION_FAILED,
                    "recover requires current feedback, no active motor fault and a verified exclusive controller",
                    details=detail,
                )
            if detail.get("ready") is True:
                return detail
            disabled = any(m["fault"] == -1 for m in motors.values())
            if not disabled and detail.get("fault", {}).get("kind") != "controller_offline_latched":
                raise DeviceError(ErrorCode.PRECONDITION_FAILED, "controller fault requires inspection", details=detail)
            if initial is None:
                initial = joints
            if max(abs(q - p) for q, p in zip(joints[:6], initial[:6])) > 0.005:
                raise DeviceError(ErrorCode.PRECONDITION_FAILED, "arm moved during recovery preflight; check support")
            if time.monotonic() - started >= 1.0:
                return detail
            cancel.wait(0.1)

    def _calibrate_gripper(
        self,
        device_id: str,
        *,
        steps: int,
        hold_s: float,
        cancel: threading.Event,
        progress: Callable[[float], None],
    ) -> dict[str, Any]:
        """Open → closed in ``steps`` → open, recording feedback at the extremes."""
        steps = max(2, steps)
        samples: list[dict[str, Any]] = []

        def hold_and_sample(opening: float, label: str) -> float:
            result = self._interpolate(
                device_id, goal_opening=opening, duration_s=0.6, cancel=cancel
            )
            if not result["completed"]:
                raise DeviceError(
                    ErrorCode.OPERATION_FINISHED,
                    "calibration cancelled",
                    execution_outcome="partial",
                )
            self._sleep(hold_s, cancel)
            values = []
            state = self._states[device_id]
            last_message = None
            deadline = self._clock() + 2.0
            command_span = abs(
                self.opening_to_raw(device_id, 1) - self.opening_to_raw(device_id, 0)
            )
            while self._clock() < deadline:
                if cancel.is_set() or state.abort.is_set():
                    raise DeviceError(
                        ErrorCode.OPERATION_FINISHED,
                        "calibration cancelled before sampling",
                        execution_outcome="partial",
                    )
                joints, _ = self._require_fresh(device_id)
                with state.lock:
                    message = state.messages
                if message != last_message:
                    values.append(joints[6])
                    values = values[-5:]
                    last_message = message
                if (
                    len(values) == 5
                    and max(values) - min(values) <= command_span * 0.02
                ):
                    break
                cancel.wait(0.025)
            else:
                raise DeviceError(
                    ErrorCode.PRECONDITION_FAILED,
                    "gripper feedback did not settle into five fresh samples",
                    details={"label": label, "feedback_raw": values},
                )
            fb = statistics.median(values)
            samples.append(
                {
                    "label": label,
                    "commanded_opening": opening,
                    "commanded_raw": round(self.opening_to_raw(device_id, opening), 4),
                    "feedback_raw": round(fb, 4),
                    "sample_count": len(values),
                    "spread_raw": round(max(values) - min(values), 5),
                }
            )
            return fb

        open_fb = hold_and_sample(1.0, "open")
        progress(0.1)
        for k in range(1, steps + 1):
            opening = 1.0 - k / steps
            fb = hold_and_sample(opening, f"step_{k}")
            progress(0.1 + 0.7 * k / steps)
        closed_fb = fb
        reopen_fb = hold_and_sample(1.0, "reopen")
        progress(1.0)
        command = self.calibration[device_id]["command"]
        span = abs(closed_fb - open_fb)
        commanded_span = command["open_raw"] - command["closed_raw"]
        tolerance = abs(commanded_span) * 0.10
        valid = (
            all(math.isfinite(v) for v in (open_fb, closed_fb, reopen_fb))
            and (open_fb - closed_fb) * commanded_span > 0
            and 0.5 * abs(commanded_span) <= span <= 1.5 * abs(commanded_span)
            and abs(open_fb - command["open_raw"]) <= tolerance
            and abs(closed_fb - command["closed_raw"]) <= tolerance
            and abs(reopen_fb - open_fb) <= abs(commanded_span) * 0.03
        )
        if not valid:
            raise DeviceError(
                ErrorCode.PRECONDITION_FAILED,
                "gripper endpoint travel/direction/reopen check failed; inspect obstruction and command endpoints",
                details={"samples": samples},
            )
        if cancel.is_set() or self._states[device_id].abort.is_set():
            raise DeviceError(
                ErrorCode.OPERATION_FINISHED,
                "calibration cancelled before saving",
                execution_outcome="partial",
            )
        with self._calibration_lock:
            self.calibration[device_id].update(
                {
                    "feedback": {
                        "closed_raw": round(closed_fb, 4),
                        "open_raw": round(statistics.mean((open_fb, reopen_fb)), 4),
                    },
                    "measured_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "method": "fresh_settled_endpoint_sweep_v1",
                    "quality": {
                        "reopen_verified": True,
                        "samples_per_position": 5,
                        "reopen_error_raw": round(abs(reopen_fb - open_fb), 5),
                    },
                }
            )
            self._save_calibration()
        return {
            "calibration": self.calibration[device_id],
            "samples": samples,
            "reopen_feedback_raw": round(reopen_fb, 4),
            "completed": True,
            "physical_confirmed": True,
        }

    # -- streams / safety --------------------------------------------------------------------------

    def stream_apply(self, device_id: str, targets: dict[str, list[float]]) -> None:
        """Publish one streamed setpoint; omitted joints hold their measured angles."""
        state = self._states[device_id]
        with state.command_lock:
            joints, end_pos = self._require_fresh(device_id)
            measured = joints[:6]
            joint_cmd = [float(v) for v in targets.get("joint_targets_rad", measured)]
            if "joint_targets_rad" in targets:
                check_joint_target(joint_cmd, profile=self._profiles[device_id])
                # The stream clamp bounds the change between pushes; this bounds the
                # first push and any target far from where the arm actually is.
                step = max(abs(a - b) for a, b in zip(joint_cmd, measured))
                if step > self.motion_limits.joint_step_rad:
                    raise DeviceError(
                        ErrorCode.LIMIT_VIOLATION,
                        f"{device_id}: streamed joint target is {step:.3f} rad from the measured joints; "
                        f"the limit is motion_limits.joint_step_rad={self.motion_limits.joint_step_rad}",
                        stage="preflight",
                    )
            gripper_raw = (
                self.opening_to_raw(device_id, float(targets["gripper_opening"][0]))
                if "gripper_opening" in targets
                else (joints[6] if state.gripper_target is None else state.gripper_target)
            )
            self._publish(device_id, joint_cmd, end_pos, float(gripper_raw), MODE_JOINT)
            state.gripper_target = float(gripper_raw)

    def stream_hold(self, device_id: str) -> None:
        """Hold the measured joints. Needs only fresh feedback: a vendor fault, a slow
        controller service or a second writer must not prevent a hold."""
        self._hold(device_id)

    def _hold(self, device_id: str) -> bool:
        """Hold the measured joints; returns False when an operator SOFT latch was kept instead."""
        state = self._states[device_id]
        with state.command_lock:
            if hasattr(self.transport, 'teach_state'):
                try:
                    mode = self.transport.teach_state(self.arms[device_id]['side'])
                except Exception:
                    # Unknown teaching state: holding the measured joints is the safe action.
                    mode = {}
                if mode.get('active') is True:
                    # Preserve the explicit operator SOFT latch on lease cleanup.
                    # This is not a torque hold or a certified physical stop.
                    self._require_fresh(device_id, teaching=True)
                    with state.lock:
                        velocities = list(state.joint_vel or [])
                    if len(velocities) != 7 or any(not math.isfinite(v) or abs(v) > .03 for v in velocities):
                        raise DeviceError(ErrorCode.PRECONDITION_FAILED,
                            'SOFT mode cannot actively stop a moving arm; support it and verify stationary feedback')
                    return False
            joints, end_pos = self._fresh_feedback(device_id)
            raw = joints[6] if state.gripper_target is None else state.gripper_target
            self._publish(device_id, joints[:6], end_pos, raw, MODE_JOINT)
            return True

    def estop(self, device_id: str) -> dict[str, Any]:
        state = self._states[device_id]
        # Abort before waiting for the command lock, so a running motion stops at its next
        # tick; set it again under the lock, because a motion starting meanwhile clears it.
        state.abort.set()
        with state.command_lock:
            state.abort.set()
            if not self._hold(device_id):
                return {'held': False, 'mode': 'soft_teach',
                        'note': 'SOFT latch preserved; no active torque hold or certified stop'}
        return {
            "held": True,
            "note": "interpolation aborted and current feedback republished as target; not a certified stop",
        }

    def close(self) -> None:
        self.transport.close()
        for solver in self._kinematics.values():
            solver.close()


def driver(
    *,
    body_id: str = "arx_arms",
    calibration_path: str | None = None,
    node_name: str = "rua_arx_body",
    arms: dict | None = None,
    profile: str | None = None,
    motion_limits: dict | None = None,
    tracking_limits: dict | None = None,
    eef_limit_policy: str = "reject",
    publish_hz: float = PUBLISH_HZ,
    controller_recovery: Callable | None = None,
    controller_lifecycle: Callable | None = None,
) -> ArxX5Driver:
    """Explicit factory for ``rua-body serve --driver uni_robot_agent.integrations.arx_one.body.drivers.arx_x5.driver:driver``."""
    # Reject invalid configuration before acquiring ROS/CAN resources.
    limits = MotionLimits.from_config(motion_limits)
    tracking = TrackingLimits.from_config(tracking_limits)
    validate_eef_limit_policy(eef_limit_policy)
    publish_hz = validate_publish_hz(publish_hz)
    if not isinstance(arms, dict) or not arms:
        raise ValueError("an explicit nonempty arms map is required")
    configured_arms = {
        device: {
            **spec,
            **({"profile": profile} if profile else {}),
        }
        for device, spec in arms.items()
    }
    configured_arms = validate_arms(configured_arms)
    transport = RclpyTransport(node_name, arms=configured_arms)
    try:
        return ArxX5Driver(
            transport,
            body_id=body_id,
            arms=configured_arms,
            calibration_path=calibration_path,
            motion_limits=limits.to_dict(),
            tracking_limits=tracking.to_dict(),
            eef_limit_policy=eef_limit_policy,
            publish_hz=publish_hz,
            controller_recovery=controller_recovery,
            controller_lifecycle=controller_lifecycle,
        )
    except BaseException:
        transport.close()
        raise


__all__ = [
    "validate_arms",
    "MODE_EEF",
    "MODE_JOINT",
    "ArxTransport",
    "ArxX5Driver",
    "RclpyTransport",
    "driver",
]
