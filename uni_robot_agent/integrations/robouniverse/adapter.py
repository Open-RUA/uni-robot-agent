"""Map Uni pose steps to the public RoboUniverse Panda joint-position client."""
import io
import time

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from uni_robot_agent.services.environment.contracts import (
    ArmCommand, ArmDescription, ArmState, CameraDescription, EnvironmentDescription,
    EnvironmentState, Pose, PoseCommand, RequestRejected, StepReceipt,
)
from uni_robot_agent.services.environment.snapshots import CameraFrame, CapturedObservation


ARM = "panda"
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


class UnsupportedLayout(ValueError):
    """The package declares an action or observation layout without a registered mapping."""


def _split(names, units, label):
    """Locate the seven arm joints by name; every other channel must be a metre-valued finger."""
    if len(set(names)) != len(names) or len(units) != len(names):
        raise UnsupportedLayout(f"{label} channel names or units are inconsistent")
    missing = [name for name in JOINTS if name not in names]
    if missing:
        raise UnsupportedLayout(f"{label} has no channel named {missing[0]}")
    joints = [names.index(name) for name in JOINTS]
    fingers = [i for i in range(len(names)) if i not in joints]
    if any(units[i] != "rad" for i in joints) or not 1 <= len(fingers) <= 2 or any(units[i] != "m" for i in fingers):
        raise UnsupportedLayout(f"{label} must hold seven radian joints and one or two metre fingers")
    return joints, fingers


class PandaLayout:
    """Channel positions of one Panda inside the package's action and observation vectors."""

    def __init__(self, spec):
        action, observation = spec["action"], spec["observation"]
        if action["kind"] != "joint_position_target":
            raise UnsupportedLayout(f"Action kind {action['kind']!r} is not a plain joint-position target")
        self.size = len(action["names"])
        self.joints, self.fingers = _split(action["names"], action["units"], "action")
        self.obs_joints, self.obs_fingers = _split(observation["joint_names"], observation["joint_units"], "observation")
        self.low, self.high = np.asarray(action["lower"], float), np.asarray(action["upper"], float)
        if (self.low.shape != (self.size,) or self.high.shape != (self.size,)
                or not np.all(np.isfinite(self.low)) or not np.all(np.isfinite(self.high))
                or not np.all(self.high > self.low)):
            raise UnsupportedLayout("Action limits must be finite and ordered")
        # All finger channels share one range: a single channel drives a mimic joint, two are symmetric.
        if (len(set(self.low[self.fingers])) != 1 or len(set(self.high[self.fingers])) != 1):
            raise UnsupportedLayout("Finger channels must share one range")
        self.finger_low, self.finger_high = float(self.low[self.fingers[0]]), float(self.high[self.fingers[0]])
        self.obs_size = len(observation["joint_names"])

    def opening(self, joint_position):
        fingers = np.asarray(joint_position)[self.obs_fingers]
        return float(np.clip((np.mean(fingers) - self.finger_low) / (self.finger_high - self.finger_low), 0, 1))

    def action(self, joints, opening):
        """Ordered action vector with the arm targets and every finger channel at the same opening."""
        vector = np.zeros(self.size)
        vector[self.joints] = np.clip(joints, self.low[self.joints], self.high[self.joints])
        vector[self.fingers] = np.clip(
            self.finger_low + opening * (self.finger_high - self.finger_low),
            self.low[self.fingers],
            self.high[self.fingers],
        )
        return vector


# Panda URDF joint origins, expressed in the respective parent link.
# https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_data/franka_panda/panda.urdf
OFFSETS = ((0, 0, .333), (0, 0, 0), (0, -.316, 0), (.0825, 0, 0),
           (-.0825, .384, 0), (0, 0, 0), (.088, 0, 0))
TWISTS = Rotation.from_euler("x", np.array([0, -90, 90, 90, -90, 90, 90])[:, None], degrees=True).as_matrix()
HAND_ROTATION = Rotation.from_euler("z", -45, degrees=True).as_matrix()


def panda_kinematics(joints):
    """Return base-relative grasp-site pose and its geometric Jacobian."""
    position, rotation = np.zeros(3), np.eye(3)
    origins, axes = [], []
    for offset, twist, angle in zip(OFFSETS, TWISTS, joints, strict=True):
        position = position + rotation @ offset
        rotation = rotation @ twist
        origins.append(position)
        axes.append(rotation[:, 2])
        rotation = rotation @ Rotation.from_euler("z", angle).as_matrix()
    # Flange is 107 mm from joint 7; nominal grasp site is 105 mm beyond hand.
    position = position + rotation[:, 2] * .212
    jacobian = np.vstack((np.cross(axes, position - origins).T, np.asarray(axes).T))
    return position, Rotation.from_matrix(rotation @ HAND_ROTATION), jacobian


class VerifiedAdapter:
    def __init__(self, client, instruction, episode_id):
        self.client, self.instruction, self.episode_id = client, instruction, episode_id
        spec = client.describe()
        observation = spec["observation"]
        self.layout = PandaLayout(spec)
        self.task_id = spec["task_id"]
        self.description = EnvironmentDescription(
            arms=[ArmDescription(name=ARM, joint_names=JOINTS, control_frame="panda_grasptarget",
                grasp_site_description="World origin is panda_link0. TCP lies 105 mm along hand +Z.",
                rotation_radius_m=.212)],
            cameras=[CameraDescription(name=c["id"], role="scene" if c["role"] == "external" else c["role"],
                                       has_depth=False, has_calibration=False)
                     for c in observation["cameras"]],
            control_frequency_hz=1 / spec["timing"]["control_dt_s"],
            max_episode_steps=spec["max_steps"], observation_interval_steps=1,
            metadata={"control_constraints": "All poses use the Panda base as the world frame."},
        )
        self.obs = client.observation
        if self.obs["episode_id"] != episode_id:
            raise ValueError("Harbor episode identity changed during startup")
        self.opening_command = self.measured_opening()
        self.ended = self.obs["step_index"] >= spec["max_steps"]
        self.outcome = None

    def episode_identity(self):
        return self.instruction, {"task_id": self.task_id, "benchmark": "RoboUniverse-Verified"}

    def measured_opening(self):
        return self.layout.opening(self.obs["proprio"]["joint_position"])

    def measured_joints(self):
        return np.asarray(self.obs["proprio"]["joint_position"])[self.layout.obs_joints]

    def read_state(self):
        joints = self.measured_joints()
        position, rotation, _ = panda_kinematics(joints)
        pose = Pose(position=position.tolist(), quaternion=np.roll(rotation.as_quat(), 1).tolist())
        arm = ArmState(control_pose=pose, joints=joints, gripper=self.measured_opening(),
                       gripper_command=self.opening_command)
        index = self.obs["step_index"]
        return EnvironmentState(arms={ARM: arm}, state_version=index, native_steps=index, ended=self.ended)

    def validate_commands(self, commands, mode):
        if self.ended:
            raise RequestRejected("Harbor episode has ended")
        for command in commands:
            if (mode != "pose" or not isinstance(command, PoseCommand) or command.solver != "native"
                    or set(command.arms) != {ARM} or not isinstance(command.arms[ARM], ArmCommand)):
                raise RequestRejected("Expected a Panda pose and scalar opening")

    def _step_client(self, action):
        before = self.obs["step_index"]
        # Mark ended before the RPC so an unresolved transport/5xx failure cannot leave
        # the adapter out of sync with a simulator that may already have advanced physics.
        self.ended = True
        try:
            reply = self.client.step_chunk([action.tolist()])
        except Exception as error:
            # HTTP 400 is a synchronous validation rejection before physics advances, so
            # restore active state and surface a recoverable RequestRejected to the planner.
            if getattr(error, "status", None) == 400:
                self.ended = False
                raise RequestRejected(f"Simulator rejected action: {error}") from error
            raise
        observed = reply["observation"]
        count = reply["executed_steps"]
        if observed["episode_id"] != self.episode_id or count not in (0, 1) or observed["step_index"] != before + count:
            raise RuntimeError("Harbor returned an inconsistent action receipt")
        self.obs = observed
        self.ended = reply["state"] != "active"
        return reply, count

    def _panda_ik_delta(self, joints, error):
        """Limit-aware active-set weighted DLS step for the 7-DOF Panda.

        Plain damped least-squares stalls when one joint hits a limit and gets
        clipped afterwards, because the remaining 6 redundant joints never
        compensate. Here we:
        1. Soft-weight joints whose normalized margin to either limit is < 15%.
        2. Add an error-gated null-space centering pull toward joint midpoints
           (scaled to 0 near zero task error so hold steps stay motionless).
        3. Iteratively clamp any joint that would exceed its limit, subtract its
           reachable contribution from the task RHS, zero its weight, and
           re-solve across the remaining joints, reporting saturated joint names.
        """
        _, _, jacobian = panda_kinematics(joints)
        low, high = self.layout.low[self.layout.joints], self.layout.high[self.layout.joints]
        mid = 0.5 * (low + high)
        half = np.maximum(0.5 * (high - low), 1e-6)
        margin = np.minimum(joints - low, high - joints) / half
        weights = np.clip(margin / 0.15, 0.05, 1.0)
        q_null = 0.20 * (mid - joints)

        def _solve(w, rhs):
            jw = jacobian * w[None, :]
            gram = jw @ jacobian.T + .025**2 * np.eye(6)
            j_pinv = (w[:, None] * jacobian.T) @ np.linalg.inv(gram)
            null_scale = min(1.0, float(np.linalg.norm(rhs)) / 0.02)
            return j_pinv @ rhs + null_scale * ((np.eye(7) - j_pinv @ jacobian) @ q_null)

        w = weights.copy()
        rhs = error.copy()
        fixed = np.zeros(7)
        saturated = []
        for _ in range(7):
            step = _solve(w, rhs)
            total = fixed + step
            violated = np.where(((joints + total > high) & (step > 0)) | ((joints + total < low) & (step < 0)))[0]
            if len(violated) == 0:
                break
            for idx in violated:
                rem = float(np.clip(joints[idx] + total[idx], low[idx], high[idx]) - joints[idx]) - fixed[idx]
                fixed[idx] += rem
                rhs = rhs - jacobian[:, idx] * rem
                w[idx] = 0.0
                saturated.append(JOINTS[idx])
        else:
            step = _solve(w, rhs)
        delta = fixed + step
        delta /= max(1., float(np.max(np.abs(delta))) / .14)
        return delta, sorted(set(saturated))

    def step_pose(self, command):
        self.validate_commands([command], "pose")
        target = command.arms[ARM]
        joints = self.measured_joints()
        position, rotation, _ = panda_kinematics(joints)
        goal = Rotation.from_quat(np.roll(target.pose.quaternion, -1))
        error = np.r_[target.pose.position - position, (goal * rotation.inv()).as_rotvec()]
        # One limit-aware damped IK update per control interval, relinearized at the next measured state.
        delta, saturated = self._panda_ik_delta(joints, error)
        action = self.layout.action(joints + delta, target.gripper)
        reply, count = self._step_client(action)
        if count:
            self.opening_command = target.gripper
        diagnostics = {"executed_steps": count, "reason": reply.get("reason")}
        if saturated:
            diagnostics["saturated_joints"] = saturated
        return StepReceipt(accepted=bool(count), state=self.read_state(), diagnostics=diagnostics)

    def step_joints(self, command):
        raise RequestRejected("Only pose commands are exposed")

    def step_native(self, command):
        raise RequestRejected("Only pose commands are exposed")

    def preview_joints(self, commands):
        raise RequestRejected("Joint previews are unavailable")

    def capture(self, cameras=None, *, high_resolution=False):
        images = self.obs["rgb"]
        names = list(images) if cameras is None else cameras
        if not names or len(set(names)) != len(names) or set(names) - images.keys():
            raise RequestRejected("Select unique Harbor cameras")
        frames = {}
        for name in names:
            data = self.client.request(images[name]["url"])
            with Image.open(io.BytesIO(data)) as image:
                frames[name] = CameraFrame(np.asarray(image.convert("RGB")))
        state = self.read_state()
        return CapturedObservation(self.episode_id, state.state_version, time.time(),
            state.model_dump(mode="json"), frames, terminated=self.ended)

    def finish(self, reason, status=None):
        if self.outcome is None:
            # Every accepted step passes through this adapter, so a simulator that is ahead of it was driven elsewhere
            # (or an unresolved step request still advanced it; simulator_steps lets a reviewer tell).
            known = self.obs["step_index"]
            actual = self.client.observe()["step_index"]
            response = self.client.finish()
            self.ended = True
            self.outcome = {"task_success": None, "reason": response["reason"], "native_steps": known,
                            "simulator_steps": actual, "bypassed_adapter": actual != known}
        return dict(self.outcome)

    def close(self):
        pass  # Harbor owns the simulator and the eventual verifier invocation.
