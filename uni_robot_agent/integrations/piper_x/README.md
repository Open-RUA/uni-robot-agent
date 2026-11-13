# Piper-X

Piper-X connects CAN arms and RealSense cameras to UNI's existing Direct tools.
The environment follows the mainline ARX ONE lifecycle and `wholepath` contract.
All Piper-specific code is in this directory; shared tool schemas, planner
adapters and providers are unchanged.

## Runtime layout

- `body/`: Body 2.0 leases, receipts, authentication and native hosting.
- `hardware/`: passive CAN feedback, pinned SDK writes, group path planning,
  geometry checks and RGB-D acquisition.
- `environment/`: state, image-time wrist geometry, native command translation
  and lifecycle stopping.
- `config.py` and `templates/`: explicit model, host, robot and calibration inputs.
- `session.py`: identity validation and shared tool composition. Physical
  episodes are not automatically retried and outcomes are assessed externally.

Native libraries load only in native runtime paths. CLI help and offline
validation do not open CAN or USB. The driver targets Linux SocketCAN; it does
not enable motors, change modes, reset faults or configure interfaces automatically.

## Installation and configuration

Install matching main and runtime distributions in separate environments. The
main package runs the official planner CLI; the runtime hosts Body and the
environment. Both distributions provide the `piper_x` extra. For source
development, use `pip install -e '.[piper_x]'`. Install the commissioned pyAgxArm
SDK separately in the native environment; its version and driver source hash
must match the model manifest.

Copy `templates/` to an installation-owned directory. Placeholders are incomplete
and grant no motion clearance. Supply:

1. `model.json`: six joints in CAN motor order, pinned URDF/collision assets,
   firmware/SDK identity, physical limits and gripper range.
2. `host.json`: CAN USB identity/channel, camera SDK serials and stream profiles,
   Body instances, token environment names and private paths.
3. `robot.json`: named arm roles, camera bindings, motion profiles, feedback
   bounds and a hash-pinned calibration file.
4. `calibration.json`: base transforms, flange-to-grasp transform, optical
   extrinsics, gripper calibration, table geometry and commissioning evidence
   bound to the selected hardware and capture settings.

TCP is a full transform, not an offset silently added by a tool. Where the
installed tool's TCP is flange +z 0.1425 m, record it in `T_flange_from_grasp`
with the actual orientation. Optical frames are calibrated independently.
Calibration acquisition and measured station files are external to this source.

An arm may declare six `kinematic_joint_offsets_rad` values in its calibration.
They default to zero. Model angles are native measured/controller angles plus
these offsets; reported joints, transmitted targets and native joint limits
remain in controller coordinates. This is software kinematic calibration, not
a firmware zero reset. FK, IK, link collision transforms and image-time wrist
poses use the same per-arm model. The other arm's model is unaffected.

With a nonzero correction, Body EEF feedback, tracking and Cartesian arrival use
`FK_corrected(q) @ inverse(FK_nominal(q)) @ native_flange`. This preserves the
native pose residual while applying the calibrated geometry. Unmodified native
pose signals remain available as `native_eef_position_m` and `native_eef_rpy_rad`.
Zero-offset installations retain their previous EEF values. Read-only mode loads
only the lightweight FK model for corrected arms, never a motion writer or
collision runtime.

Changing this model requires matching hand-eye/base calibration and fresh
independent validation, then updating the calibration file hash and preparing a
new environment. Do not install a corrected camera extrinsic with an uncorrected
robot model. Offsets alone do not satisfy any station acceptance check.

Public wrist camera names are `<arm-name>_wrist_<configured-camera-name>`.
Scene names are retained. Shared tools can then select the acting arm's wrist
views while the adapter preserves device identities and parent-frame calibration.
Arm names must not be substrings of each other; ambiguous names are rejected.

RGB-D packets with exactly zero Brown or inverse-Brown distortion coefficients
already have pinhole geometry. The observation adapter preserves their pixels
directly without allocating identity rectification maps. Nonzero distortion
and all source-time freshness checks retain their normal processing.

Camera resolution and frame rate are installation settings in `host.json`, not
planner-side image resizing. The driver reads intrinsics from the active color
profile and aligns depth to that profile. When changing capture settings, update
the calibration capture identity and file hash, then prepare the environment
again. Do not scale a previous intrinsic matrix by the image-size ratio: a
different sensor profile may change the field of view. Retain extrinsics only
while the physical camera mounting is unchanged and recheck alignment.

Validate and generate host configuration without hardware access:

```bash
python -m uni_robot_agent.integrations.piper_x validate --deployment /path/to/deployment.json
python -m uni_robot_agent.integrations.piper_x host-config \
  --deployment /path/to/deployment.json --output /path/to/host.resolved.json
```

These default to observation-only arms. After commissioning, add `--execute`
to both commands to validate and prepare physical execution. Keep credential
values in a private token file and environment. The host token file maps tokens
to Body principals; configured environment variables select those tokens.
An optional principal `device_ids` set limits both reads and physical actions.
Omitting the scope permits all devices; an empty set permits none. Device-scoped
queries exclude records spanning unauthorized devices, even for credentials
sharing a principal name. An `estop` without an explicit device selection acts
only on that credential's allowed devices. Native aggregate health details are
omitted for device-scoped credentials.

## Start an episode

Run each Body in its own native-runtime terminal; these instance names match
the templates:

```bash
python -m uni_robot_agent.integrations.piper_x.body.host.cli body \
  --config /path/to/host.resolved.json --instance arms
python -m uni_robot_agent.integrations.piper_x.body.host.cli body \
  --config /path/to/host.resolved.json --instance cameras
```

Start an environment with a new run ID and output directory:

```bash
python -m uni_robot_agent.integrations.piper_x environment \
  --deployment /path/to/deployment.json --execute \
  --run-id trial-001 --instruction 'Put the two fruits in the basket.' \
  --output-dir ./environment-runs/trial-001 --host 127.0.0.1 --port 19312
```

Use the official UNI CLI in the planner environment, with the configured robot
ID and a model configured for that planner:

```bash
uni-robot-agent run --integration piper_x --robot-id YOUR_ROBOT_ID \
  --env-url http://127.0.0.1:19312 --agent codex --model YOUR_MODEL_ID \
  --action-profile direct --no-sam3 --output-dir ./runs --monitor
```

`--no-sam3` disables SAM segmentation; calibrated RGB-D remains available.
Use the usual perception options if SAM is required. There is no private
evaluation runner or `--deploy local` recipe. Each trial uses a new environment
and official CLI session. UNI owns result, trajectory and monitor artifacts.
A model completion claim is not an external judgment: `task_success` stays null.

For observation-only use, omit `--execute` from preparation/startup and pass
`--tool-profile readonly` to the official CLI. That profile composes existing
scene, skill and episode tools only. A Direct session refuses an observation-only
environment before tool assembly.

## Motion, stopping and migration

`robot.json.motion_validation` selects the native preflight contract. Its
default, `collision`, preserves the commissioned swept-volume checks and
geometry requirements. Existing installations need no configuration change.
For supervised operation with the same collision-checking scope as ARX ONE,
explicitly select `command_limits` and record `operator_supervised: true` in
the calibration. This profile checks FK/IK, native joint bounds, command steps,
speeds, feedback freshness, pose tracking and measured arrival. It does **not**
check table, self or inter-arm collisions, or require camera-bracket/cable
geometry. Pinned model assets remain recorded, but collision enclosure evidence
is not a motion prerequisite in this profile. Deployment audit, Body metadata
and motion receipts identify the selected profile; they never claim collision
clearance for `command_limits`. Commissioning checks remain explicit evidence
or attributed skips, rather than being marked passed by this selection.

In `command_limits`, stationary feedback at most 0.02 rad outside a native joint
bound may trigger a separate entry prefix on the next requested operation. On
each arm with an outlying joint, all joints within 0.03 rad of a bound receive
targets 0.03 rad inside that bound, subject to the same step/speed limits.
Unaffected arms and gripper openings are retained. All transmitted destinations
remain inside native limits. The receipt reports `joint_entry_recovery` and
`requested_targets_reached: false`, requiring fresh observation and replanning.
Larger excursions are rejected. Feedback is never clipped, and this is neither
a zero reset nor an automatic action at startup. Collision mode remains unchanged.
Exact-bound feedback alone does not trigger entry. Applying the margin to all
boundary joints on the recovering arm avoids a peer joint drifting outside
while the original outlier recovers. The margin accommodates small measured
servo residuals; success still requires consecutive feedback strictly inside
the native bounds.

One `move` uses a mainline `PathCommand` and a group-only Body operation.
All arms are preflighted, including measured holds for idle arms. The public
command limit is 24 path segments. `grasp`, `release` and `hold` preserve their
control-period duration through `min_duration_s`. Bounded prefixes report
confirmed progress; unknown execution is never replayed. The shared replanning
executor owns observation barriers after failed actions.

Stop the environment with Ctrl-C or SIGTERM. Like ARX, its exit handler calls
`backend.finish("operator_cancelled")` on a separate thread outside the action
lock. Body closes admission and verifies fresh stationary feedback. Piper uses
measured joint-position holding for software stopping; this is not a certified
hardware emergency stop. Keep the station's physical stop available during use.
Stop evidence reports three distinct fresh stationary samples separately from
hold-request errors and quarantined operations. Stationary feedback does not
prove target arrival, clear quarantine, replay a command, or certify a failed
hold. These failures remain blocked for operator recovery. Upgrade and restart
the native arm Body and environment together to use the updated entry behavior.
On restart, quarantine is reconstructed from every unknown operation in the
journal, regardless of list pagination. Only persisted acknowledgements for the
specific device and operation can clear its quarantine; the receipt stays unknown.

Migration from experimental v2 requires matching packages and restarted native
services. Regenerate host configuration for `integrations.piper_x.body`. Old
`services.body`/`integrations.body_gripper` imports, `coordinated_path_v1`, the
64-segment budget, `continuous.py`, private `eval` runner and
`stop --request-id --query` command are not retained. Scalar environments do not
advertise device `/interrupts`; use the lifecycle stop above. Readonly
finalization does not acquire control or transmit a stop.

## Validation boundary

CPU tests use synthetic deployments, Body devices and local HTTP. They cover
identity/resource isolation, readonly operation, group commands, receipt
reconciliation without replay, partial/unknown outcomes, camera loss and
lifecycle stopping. They cannot establish physical clearance, calibration
accuracy, CAN timing or task success.

From a source checkout with the `piper_x` extra and pytest installed:

```bash
PYTHONPATH=.:tests python -m pytest tests/test_piper_x*.py
PYTHONPATH=. python scripts/validation/tool_composition.py
```

Two supervised dual-arm trials on 2026-11-13 used the official Codex CLI,
640x480 RGB-D at 15 Hz, the `command_limits` profile, calibrated per-arm
kinematics and the 0.03 rad entry margin documented above. Both trials placed
the apple with the left arm and the orange with the right arm into one basket.
One trial was accepted with confirmed idle stopping. The other was rejected
as `execution_uncertain`: its final right-arm withdrawal exceeded the existing
30 mm tracking threshold (30.66 mm), retaining the unknown receipt and
quarantine. Later stationary feedback did not change that outcome. The cause
of this tracking deviation remains unresolved; the successful trial is not
evidence that it has been fixed.

These are limited station observations, not a reliability benchmark or
validation of the collision profile. Neither trial had an automatic physical
task evaluator (`task_success` remained null); fruit placement was reviewed
from the final episode images. Keep station files, credentials, measured
calibration and run artifacts outside source. A new installation still needs
its own commissioning and validation.
The journal-recovery and authorization fixes added during PR review have CPU
regression coverage only; they were not deployed for those trials.
See [source attribution](SOURCE.md).
