"""Map Uni pose steps to the public RoboUniverse AgileX dual Piper joint-position client.

The agent container sees only `describe()`. It does not see where each arm is bolted down, which lives in the
simulator's kinematics module and differs per task. A harness-side frame table (built offline, see
`FrameTable`) carries those bases. The lookup key is a digest of everything `describe()` exposes, and the lookup
fails closed: an unknown key, a table that disagrees with `describe()`, or a layout this module does not
register raises `UnsupportedLayout`; no frame is ever guessed.

World frame shown to the model: the origin is the horizontal midpoint of the two arm base origins on the
simulator's z = 0 plane (so heights equal simulator heights), +Z is up, +X is the horizontal direction from the left
base to the right base and +Y completes a right-handed frame. Both arms are commanded and reported in this one frame.
"""
import hashlib
import json
import re

import numpy as np
from scipy.spatial.transform import Rotation

from uni_robot_agent.integrations.robouniverse.adapter import UnsupportedLayout, VerifiedAdapter
from uni_robot_agent.services.environment.contracts import (
    ArmCommand, ArmDescription, ArmState, CameraDescription, EnvironmentDescription,
    EnvironmentState, Pose, PoseCommand, RequestRejected, StepReceipt,
)

ARMS = ("left", "right")
YAW_CHANNELS = frozenset({"base_yaw", "pedestal_yaw"})
FORMAT = "uni-robouniverse-dual-frames/1"
FRAME = "dual_arm_world"
# Grasp site: this far beyond the link6 origin along link6 +Z. The pinned URDF puts the finger joints 0.135 m out
# (the fingertips end there) and the packages' own tool points range 0.105 to 0.205 m, mostly 0.115 to 0.135 m, so
# 0.12 m is the pad centre. The site frame is link6's frame: +Z approach, fingers open along +-X. Not validated in
# simulation.
SITE_M = .12
FINGER_MAX_M = .05
KEY_FIELDS = ("kind", "names", "units", "lower", "upper", "gripper_mapping")
CHANNEL = re.compile(r"^(left|right)[/._](.+)$")
TRAILING = re.compile(r"(\d+)$")


def _plain(value):
    """JSON-comparable form: every number a float, so 0 and 0.0 key identically."""
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, (int, float, np.integer, np.floating)):
        return float(value)
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    raise UnsupportedLayout(f"describe() holds an unexpected {type(value).__name__}")


def describe_key(spec):
    """Digest of the identity and layout that `describe()` exposes for one package."""
    try:
        observation = spec["observation"]
        document = {
            "task_id": spec["task_id"],
            "action": {name: spec["action"][name] for name in KEY_FIELDS},
            "observation": {"joint_names": observation["joint_names"], "joint_units": observation["joint_units"],
                            "cameras": [{k: v for k, v in camera.items() if k != "calibration_path"}
                                        for camera in observation["cameras"]]},
            "timing": spec["timing"],
        }
    except (KeyError, TypeError) as error:
        raise UnsupportedLayout(f"describe() does not carry {error}") from error
    return hashlib.sha256(json.dumps(_plain(document), sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _arm_channels(names, units, arm, label):
    """Joint channels (chain order) and finger channels of one arm, as indices into a channel list."""
    if len(set(names)) != len(names) or len(units) != len(names):
        raise UnsupportedLayout(f"{label} channel names or units are inconsistent")
    joints, fingers = [], []
    for index, (name, unit) in enumerate(zip(names, units)):
        match = CHANNEL.match(name)
        if match is None or match.group(1) != arm:
            continue
        if unit == "rad":
            number = TRAILING.search(match.group(2))
            if number is None:
                raise UnsupportedLayout(f"{label} joint {name!r} has no joint number")
            joints.append((int(number.group(1)), index))
        elif unit == "m":
            fingers.append(index)
        else:
            raise UnsupportedLayout(f"{label} channel {name!r} has unit {unit!r}")
    joints.sort()
    if [number for number, _ in joints] != list(range(1, 7)):
        raise UnsupportedLayout(f"{label} {arm} arm must hold radian joints numbered 1 to 6")
    if not 1 <= len(fingers) <= 2:
        raise UnsupportedLayout(f"{label} {arm} arm must hold one or two metre fingers")
    return [index for _, index in joints], fingers


def _yaw_channel(names, units, label):
    """Locate the single optional mobile-base yaw channel (`base_yaw` / `pedestal_yaw`), or (None, None)."""
    found = [(index, name) for index, name in enumerate(names) if name in YAW_CHANNELS]
    if not found:
        return None, None
    if len(found) > 1:
        raise UnsupportedLayout(f"{label} declares multiple base yaw channels")
    index, name = found[0]
    if units[index] != "rad":
        raise UnsupportedLayout(f"{label} base yaw channel {name!r} has unit {units[index]!r}, expected 'rad'")
    return index, name


class DualPiperLayout:
    """Channel positions of two Piper arms (and optional mobile-base yaw) inside action and observation vectors."""

    def __init__(self, spec):
        try:
            self._build(spec)
        except (KeyError, TypeError, ValueError, IndexError) as error:
            if isinstance(error, UnsupportedLayout):
                raise
            raise UnsupportedLayout(f"describe() layout is malformed: {error!r}") from error

    def _build(self, spec):
        action, observation = spec["action"], spec["observation"]
        if action["kind"] != "joint_position_target":
            raise UnsupportedLayout(f"Action kind {action['kind']!r} is not a plain joint-position target")
        names, units = action["names"], action["units"]
        obs_names, obs_units = observation["joint_names"], observation["joint_units"]
        self.size, self.obs_size = len(names), len(obs_names)
        self.joints, self.fingers, self.obs_joints, self.obs_fingers = {}, {}, {}, {}
        for arm in ARMS:
            self.joints[arm], self.fingers[arm] = _arm_channels(names, units, arm, "action")
            self.obs_joints[arm], self.obs_fingers[arm] = _arm_channels(obs_names, obs_units, arm, "observation")
            if [names[i] for i in self.joints[arm]] != [obs_names[i] for i in self.obs_joints[arm]]:
                raise UnsupportedLayout("Observation joints must carry the action joint names")
        self.base_yaw, yaw_name = _yaw_channel(names, units, "action")
        self.obs_base_yaw, obs_yaw_name = _yaw_channel(obs_names, obs_units, "observation")
        if (self.base_yaw is None) != (self.obs_base_yaw is None) or yaw_name != obs_yaw_name:
            raise UnsupportedLayout("Action and observation base yaw channels must match")
        self.base_yaw_name = yaw_name
        extra_act = [self.base_yaw] if self.base_yaw is not None else []
        extra_obs = [self.obs_base_yaw] if self.obs_base_yaw is not None else []
        claimed = sorted(sum((self.joints[arm] + self.fingers[arm] for arm in ARMS), []) + extra_act)
        if claimed != list(range(self.size)):
            raise UnsupportedLayout("Every action channel must be a joint or finger of the left or right arm")
        claimed = sorted(sum((self.obs_joints[arm] + self.obs_fingers[arm] for arm in ARMS), []) + extra_obs)
        if claimed != list(range(self.obs_size)):
            raise UnsupportedLayout("Every observation channel must be a joint or finger of the left or right arm")
        self.low, self.high = np.asarray(action["lower"], float), np.asarray(action["upper"], float)
        if (self.low.shape != (self.size,) or self.high.shape != (self.size,)
                or not np.all(np.isfinite(self.low)) or not np.all(np.isfinite(self.high))
                or not np.all(self.high > self.low)):
            raise UnsupportedLayout("Action limits must be finite and ordered")
        # A finger has one sign and a closed and an open magnitude; every finger of an arm shares the magnitudes.
        self.finger_sign, self.finger_closed, self.finger_open = {}, {}, {}
        for arm in ARMS:
            ranges = set()
            self.finger_sign[arm] = []
            for channel in self.fingers[arm]:
                low, high = float(self.low[channel]), float(self.high[channel])
                if low >= 0:
                    sign, closed, opened = 1., low, high
                elif high <= 0:
                    sign, closed, opened = -1., -high, -low
                else:
                    raise UnsupportedLayout("A finger range must not straddle zero")
                if not 0 <= closed < opened <= FINGER_MAX_M + 1e-6:
                    raise UnsupportedLayout(f"Finger range [{low}, {high}] m is not a gripper opening")
                self.finger_sign[arm].append(sign)
                ranges.add((round(closed, 9), round(opened, 9)))
            if len(ranges) != 1:
                raise UnsupportedLayout("Finger channels of an arm must share one range")
            (self.finger_closed[arm], self.finger_open[arm]), = ranges

    def joint_limits(self, arm):
        return self.low[self.joints[arm]], self.high[self.joints[arm]]

    def base_yaw_limits(self):
        if self.base_yaw is None:
            return None
        return float(self.low[self.base_yaw]), float(self.high[self.base_yaw])

    def opening(self, arm, joint_position):
        """Opening in 0..1 from the measured finger displacements (closed at the nearer-to-zero limit)."""
        values = np.abs(np.asarray(joint_position)[self.obs_fingers[arm]])
        closed, opened = self.finger_closed[arm], self.finger_open[arm]
        return float(np.clip((np.mean(values) - closed) / (opened - closed), 0, 1))

    def action(self, joints, openings, base_yaw=None):
        """Ordered action vector from per-arm joint targets, openings, and optional base yaw, clipped to limits."""
        vector = np.zeros(self.size)
        for arm in ARMS:
            vector[self.joints[arm]] = np.clip(joints[arm], *self.joint_limits(arm))
            closed, opened = self.finger_closed[arm], self.finger_open[arm]
            for channel, sign in zip(self.fingers[arm], self.finger_sign[arm]):
                raw = sign * (closed + openings[arm] * (opened - closed))
                vector[channel] = float(np.clip(raw, self.low[channel], self.high[channel]))
        if self.base_yaw is not None:
            if base_yaw is None or not np.isfinite(base_yaw):
                raise ValueError("base_yaw target is required for a mobile-base layout")
            lo, hi = self.base_yaw_limits()
            vector[self.base_yaw] = float(np.clip(base_yaw, lo, hi))
        return vector


def _transform(position, quaternion_wxyz):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(np.roll(quaternion_wxyz, -1)).as_matrix()
    result[:3, 3] = position
    return result


def _rot_z(angle):
    c, s = float(np.cos(angle)), float(np.sin(angle))
    result = np.eye(4)
    result[0, 0], result[0, 1] = c, -s
    result[1, 0], result[1, 1] = s, c
    return result


def rotate_about_z(transform, pivot_xy, yaw):
    """Apply a Z-axis rotation by `yaw` around vertical line (pivot_xy[0], pivot_xy[1], z)."""
    if abs(yaw) < 1e-15:
        return transform.copy()
    px, py = float(pivot_xy[0]), float(pivot_xy[1])
    shift = np.eye(4)
    shift[0, 3], shift[1, 3] = px, py
    unshift = np.eye(4)
    unshift[0, 3], unshift[1, 3] = -px, -py
    return shift @ _rot_z(yaw) @ unshift @ transform


class FrameTable:
    """Per-package arm bases and the pinned Piper chains, keyed by `describe_key`.

    Layout: `chains` holds the six (xyz, rpy) joint origins of each pinned URDF chain, `shapes` the joint names and
    limits an entry was identified with, and `cases` maps a describe() digest to either:
    - `[chain, shape, left base, right base]` for a fixed-base dual-arm package, or
    - `[chain, shape, left base, right base, [pivot_x, pivot_y]]` for a mobile-base (`base_yaw`) package,
      where `left base` and `right base` are the simulator-frame arm bases at `base_yaw = 0` and `[pivot_x, pivot_y]`
      is the simulator-frame horizontal Z-rotation pivot.
    A base is position (3) then quaternion wxyz (4).
    """

    def __init__(self, document):
        if not isinstance(document, dict) or document.get("format") != FORMAT:
            raise UnsupportedLayout("Frame table has an unknown format")
        chains, shapes, cases = document.get("chains"), document.get("shapes"), document.get("cases")
        if (not isinstance(chains, dict) or not isinstance(shapes, list) or not isinstance(cases, dict)
                or not cases or not chains or not shapes):
            raise UnsupportedLayout("Frame table has no chains, shapes or cases")
        body = json.dumps({"chains": chains, "shapes": shapes, "cases": cases}, sort_keys=True,
                          separators=(",", ":")).encode()
        if document.get("digest") != hashlib.sha256(body).hexdigest():
            raise UnsupportedLayout("Frame table digest does not match its content")
        try:
            self.chains = {name: [(np.asarray(row[:3], float), Rotation.from_euler("xyz", row[3:]).as_matrix())
                                  for row in rows] for name, rows in chains.items()}
        except (TypeError, ValueError, IndexError) as error:
            raise UnsupportedLayout(f"Frame table chain is malformed: {error}") from error
        if any(len(chain) != 6 for chain in self.chains.values()):
            raise UnsupportedLayout("Every pinned chain must hold six joints")
        self.shapes, self.cases = shapes, cases

    @classmethod
    def load(cls, path):
        def no_duplicates(pairs):
            keys = [key for key, _ in pairs]
            if len(set(keys)) != len(keys):
                raise UnsupportedLayout("Frame table repeats a key")
            return dict(pairs)

        with open(path) as handle:
            return cls(json.load(handle, object_pairs_hook=no_duplicates))

    def lookup(self, spec, layout):
        """Chain origins, simulator-frame base transforms (at yaw=0), and optional Z-pivot for `spec`."""
        entry = self.cases.get(describe_key(spec))
        if entry is None:
            raise UnsupportedLayout("No frame table entry for this package; refusing to guess the arm bases")
        try:
            if len(entry) == 4:
                chain, shape, left, right = entry
                pivot_raw = None
            elif len(entry) == 5:
                chain, shape, left, right, pivot_raw = entry
            else:
                raise UnsupportedLayout("Frame table entry has an unexpected length")
            origins, shape = self.chains[str(chain)], self.shapes[shape]
            names = spec["action"]["names"]
            frames = {}
            for arm, base in zip(ARMS, (left, right)):
                joints, lower, upper = shape[arm]
                channels = layout.joints[arm]
                if (joints != [names[i] for i in channels]
                        or not np.allclose(lower, layout.low[channels], rtol=0, atol=1e-9)
                        or not np.allclose(upper, layout.high[channels], rtol=0, atol=1e-9)):
                    raise UnsupportedLayout(f"Frame table joints or limits disagree with describe() for the {arm} arm")
                if len(base) != 7 or not np.all(np.isfinite(base)) or abs(np.linalg.norm(base[3:]) - 1) > 1e-6:
                    raise UnsupportedLayout("Frame table base pose is malformed")
                frames[arm] = _transform(base[:3], base[3:])
            pivot = None
            if layout.base_yaw is not None:
                if pivot_raw is None:
                    raise UnsupportedLayout("Mobile-base layout requires a verified base_yaw pivot in the frame table")
                pivot_arr = np.asarray(pivot_raw, float)
                if pivot_arr.shape != (2,) or not np.all(np.isfinite(pivot_arr)):
                    raise UnsupportedLayout("Frame table base_yaw pivot is malformed")
                if "base_yaw" in shape:
                    yaw_name, yaw_low, yaw_high = shape["base_yaw"]
                    lo, hi = layout.base_yaw_limits()
                    if (yaw_name != layout.base_yaw_name
                            or not np.isclose(yaw_low, lo, rtol=0, atol=1e-9)
                            or not np.isclose(yaw_high, hi, rtol=0, atol=1e-9)):
                        raise UnsupportedLayout("Frame table base_yaw channel or limits disagree with describe()")
                pivot = pivot_arr
            elif pivot_raw is not None:
                raise UnsupportedLayout("Fixed-base layout must not bind to a mobile-base frame table entry")
        except (KeyError, TypeError, ValueError, IndexError) as error:
            if isinstance(error, UnsupportedLayout):
                raise
            raise UnsupportedLayout(f"Frame table entry is malformed: {error}") from error
        return origins, frames, pivot


def midpoint_frame(bases):
    """Simulator-from-world transform of the model's world frame, from the two simulator-frame bases."""
    left, right = bases["left"][:3, 3], bases["right"][:3, 3]
    across = (right - left) * [1, 1, 0]
    length = np.linalg.norm(across)
    if length < .05:
        raise UnsupportedLayout("The arm bases do not define a horizontal left-to-right axis")
    x, z = across / length, np.array([0., 0., 1.])
    result = np.eye(4)
    result[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    result[:3, 3] = (left + right) / 2 * [1, 1, 0]
    return result


def piper_kinematics(origins, base, joints, site=SITE_M):
    """Pose and geometric Jacobian of the point `site` m beyond link6, in the frame `base` is given in."""
    position, rotation = base[:3, 3].copy(), base[:3, :3].copy()
    joint_origins, axes = [], []
    for (offset, fixed), angle in zip(origins, joints, strict=True):
        position = position + rotation @ offset
        rotation = rotation @ fixed
        joint_origins.append(position)
        axes.append(rotation[:, 2])
        cosine, sine = np.cos(angle), np.sin(angle)
        rotation = rotation @ np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])
    position = position + rotation[:, 2] * site
    jacobian = np.vstack((np.cross(axes, position - joint_origins).T, np.asarray(axes).T))
    return position, Rotation.from_matrix(rotation), jacobian


class DualVerifiedAdapter(VerifiedAdapter):
    """Two Piper arms in one world frame (fixed or Z-turntable base); capture, finish and bypass come from VerifiedAdapter."""

    def __init__(self, client, instruction, episode_id, table):
        self.client, self.instruction, self.episode_id = client, instruction, episode_id
        spec = client.describe()
        observation = spec["observation"]
        self.layout = DualPiperLayout(spec)
        self.origins, bases, self.base_pivot_sim = table.lookup(spec, self.layout)
        self.bases_sim = bases
        self.frame = midpoint_frame(bases)
        inverse = np.linalg.inv(self.frame)
        self.bases = {arm: inverse @ bases[arm] for arm in ARMS}
        if self.base_pivot_sim is not None:
            pivot_world = inverse @ np.array([self.base_pivot_sim[0], self.base_pivot_sim[1], 0.0, 1.0])
            self.base_pivot = pivot_world[:2].copy()
        else:
            self.base_pivot = None
        self.task_id = spec["task_id"]
        names = spec["action"]["names"]
        mobile_note = (
            f" Both arm bases ride on a Z-revolute turntable ({self.layout.base_yaw_name}) pivoting at "
            f"{np.round(self.base_pivot, 3).tolist()} m in XY; coordinated arm moves automatically rotate the base "
            f"to reach targets around the turntable (for large 180-degree turns, pass an intermediate ~90-degree waypoint)."
            if self.base_pivot is not None else ""
        )
        arms = [ArmDescription(
            name=arm, joint_names=[names[i] for i in self.layout.joints[arm]], control_frame=FRAME,
            grasp_site_description=(
                f"World origin is the horizontal midpoint between the left and right arm bases at simulator height 0. "
                f"+X is horizontal from the left base towards the right base, +Z is up, +Y = Z x X. The {arm} arm base is at "
                f"{np.round(self.bases[arm][:3, 3], 3).tolist()} m in this frame (at zero base yaw).{mobile_note} "
                f"The site lies {SITE_M * 1000:.0f} mm beyond the last arm link origin along its +Z (approach) axis; "
                f"the fingers open along the site X axis."),
            rotation_radius_m=.15) for arm in ARMS]
        metadata = {
            "control_constraints": "Both arms receive absolute site poses and scalar openings in one command. "
                                   "All poses use the dual-arm world frame described per arm." + mobile_note,
            "world_frame": {"name": FRAME, "origin": "horizontal midpoint of the left and right arm base origins, at simulator height 0",
                            "x": "horizontal, left base to right base", "z": "up", "y": "z cross x"},
            "arm_bases": {arm: {"position_m": np.round(self.bases[arm][:3, 3], 4).tolist(),
                                "quaternion_wxyz": np.round(np.roll(Rotation.from_matrix(
                                    self.bases[arm][:3, :3]).as_quat(), 1), 4).tolist()} for arm in ARMS},
        }
        if self.base_pivot is not None:
            lo, hi = self.layout.base_yaw_limits()
            metadata["base_yaw"] = {
                "channel": self.layout.base_yaw_name,
                "pivot_xy_m": np.round(self.base_pivot, 4).tolist(),
                "limits_rad": [round(lo, 4), round(hi, 4)],
            }
        self.description = EnvironmentDescription(
            arms=arms,
            cameras=[CameraDescription(name=c["id"], role="scene" if c["role"] == "external" else c["role"],
                                       has_depth=False, has_calibration=False)
                     for c in observation["cameras"]],
            control_frequency_hz=1 / spec["timing"]["control_dt_s"],
            max_episode_steps=spec["max_steps"], observation_interval_steps=1,
            metadata=metadata,
        )
        self.obs = client.observation
        if self.obs["episode_id"] != episode_id:
            raise ValueError("Harbor episode identity changed during startup")
        self.opening_command = {arm: self.measured_opening(arm) for arm in ARMS}
        self.ended = self.obs["step_index"] >= spec["max_steps"]
        self.outcome = None

    def measured_opening(self, arm):
        return self.layout.opening(arm, self.obs["proprio"]["joint_position"])

    def measured_joints(self):
        position = np.asarray(self.obs["proprio"]["joint_position"])
        return {arm: position[self.layout.obs_joints[arm]] for arm in ARMS}

    def measured_base_yaw(self):
        if self.layout.obs_base_yaw is None:
            return None
        return float(self.obs["proprio"]["joint_position"][self.layout.obs_base_yaw])

    def current_base(self, arm, base_yaw=None):
        if self.base_pivot is None:
            return self.bases[arm]
        yaw = self.measured_base_yaw() if base_yaw is None else float(base_yaw)
        return rotate_about_z(self.bases[arm], self.base_pivot, yaw)

    def current_base_sim(self, arm, base_yaw=None):
        if self.base_pivot_sim is None:
            return self.bases_sim[arm]
        yaw = self.measured_base_yaw() if base_yaw is None else float(base_yaw)
        return rotate_about_z(self.bases_sim[arm], self.base_pivot_sim, yaw)

    def kinematics(self, arm, joints, base_yaw=None):
        return piper_kinematics(self.origins, self.current_base(arm, base_yaw), joints)

    def joint_delta(self, arm, joints, target, base_yaw=None):
        """One damped least squares update towards `target`, relinearized at the next measured state."""
        position, rotation, jacobian = self.kinematics(arm, joints, base_yaw)
        goal = Rotation.from_quat(np.roll(target.quaternion, -1))
        error = np.r_[target.position - position, (goal * rotation.inv()).as_rotvec()]
        delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + .025**2 * np.eye(6), error)
        return delta / max(1., np.max(np.abs(delta)) / .10)

    def coupled_delta(self, joints, base_yaw, targets):
        """One coupled 13-DOF damped least-squares update (base_yaw + left 6 + right 6) for turntable bases.

        The Z-revolute base column is `[z x (p_site - pivot); z]` in the dual-arm world frame. Weighting favors
        `base_yaw` for shared horizontal yaw motion, softens arm joints near their limits, projects null-space
        centering onto arm joints, and freezes `base_yaw` if it pushes against its own limit so arm joints still
        finish the step cleanly.
        """
        jacobian = np.zeros((12, 13))
        error = np.zeros(12)
        pivot3 = np.array([self.base_pivot[0], self.base_pivot[1], 0.0])
        z_axis = np.array([0.0, 0.0, 1.0])
        for index, arm in enumerate(ARMS):
            position, rotation, j_arm = self.kinematics(arm, joints[arm], base_yaw)
            target = targets[arm]
            goal = Rotation.from_quat(np.roll(target.quaternion, -1))
            row = index * 6
            error[row:row + 6] = np.r_[target.position - position, (goal * rotation.inv()).as_rotvec()]
            jacobian[row:row + 6, 0] = np.r_[np.cross(z_axis, position - pivot3), z_axis]
            jacobian[row:row + 6, 1 + row:7 + row] = j_arm

        weights = np.ones(13)
        weights[0] = 6.0
        q_null = np.zeros(13)
        for index, arm in enumerate(ARMS):
            lo, hi = self.layout.joint_limits(arm)
            mid = 0.5 * (lo + hi)
            half = np.maximum(0.5 * (hi - lo), 1e-6)
            margin = np.minimum(joints[arm] - lo, hi - joints[arm]) / half
            weights[1 + index * 6:7 + index * 6] = np.clip(margin / 0.15, 0.05, 1.0)
            q_null[1 + index * 6:7 + index * 6] = 0.30 * (mid - joints[arm])

        def _solve(w, rhs):
            jw = jacobian * w[None, :]
            gram = jw @ jacobian.T + .025**2 * np.eye(12)
            j_pinv = (w[:, None] * jacobian.T) @ np.linalg.inv(gram)
            # Gate null-space centering by task-error magnitude so pure hold steps stay stationary.
            null_scale = min(1.0, float(np.linalg.norm(rhs)) / 0.02)
            return j_pinv @ rhs + null_scale * ((np.eye(13) - j_pinv @ jacobian) @ q_null)

        delta = _solve(weights, error)
        yaw_lo, yaw_hi = self.layout.base_yaw_limits()
        if (base_yaw + delta[0] > yaw_hi and delta[0] > 0) or (base_yaw + delta[0] < yaw_lo and delta[0] < 0):
            rem_yaw = float(np.clip(base_yaw + delta[0], yaw_lo, yaw_hi) - base_yaw)
            arm_weights = weights.copy()
            arm_weights[0] = 0.0
            delta = _solve(arm_weights, error - jacobian[:, 0] * rem_yaw)
            delta[0] = rem_yaw
        delta /= max(1., np.max(np.abs(delta)) / .10)
        return float(delta[0]), {"left": delta[1:7], "right": delta[7:13]}

    def read_state(self):
        joints = self.measured_joints()
        base_yaw = self.measured_base_yaw()
        arms = {}
        for arm in ARMS:
            position, rotation, _ = self.kinematics(arm, joints[arm], base_yaw)
            pose = Pose(position=position.tolist(), quaternion=np.roll(rotation.as_quat(), 1).tolist())
            arms[arm] = ArmState(control_pose=pose, joints=joints[arm], gripper=self.measured_opening(arm),
                                 gripper_command=self.opening_command[arm])
        index = self.obs["step_index"]
        metadata = {"base_yaw_rad": base_yaw} if base_yaw is not None else {}
        return EnvironmentState(arms=arms, state_version=index, native_steps=index, ended=self.ended, metadata=metadata)

    def validate_commands(self, commands, mode):
        if self.ended:
            raise RequestRejected("Harbor episode has ended")
        for command in commands:
            if (mode != "pose" or not isinstance(command, PoseCommand) or command.solver != "native"
                    or set(command.arms) != set(ARMS)
                    or not all(isinstance(command.arms[arm], ArmCommand) for arm in ARMS)):
                raise RequestRejected("Expected a left and a right arm pose with scalar openings")

    def step_pose(self, command):
        self.validate_commands([command], "pose")
        measured = self.measured_joints()
        base_yaw = self.measured_base_yaw()
        if base_yaw is not None:
            dyaw, djoints = self.coupled_delta(measured, base_yaw, {arm: command.arms[arm].pose for arm in ARMS})
            targets = {arm: measured[arm] + djoints[arm] for arm in ARMS}
            yaw_target = base_yaw + dyaw
        else:
            targets = {arm: measured[arm] + self.joint_delta(arm, measured[arm], command.arms[arm].pose) for arm in ARMS}
            yaw_target = None
        action = self.layout.action(targets, {arm: command.arms[arm].gripper for arm in ARMS}, base_yaw=yaw_target)
        reply, count = self._step_client(action)
        if count:
            self.opening_command = {arm: command.arms[arm].gripper for arm in ARMS}
        return StepReceipt(accepted=bool(count), state=self.read_state(),
                           diagnostics={"executed_steps": count, "reason": reply.get("reason")})
