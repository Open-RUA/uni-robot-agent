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

Public wrist camera names are `<arm-name>_wrist_<configured-camera-name>`.
Scene names are retained. Shared tools can then select the acting arm's wrist
views while the adapter preserves device identities and parent-frame calibration.
Arm names must not be substrings of each other; ambiguous names are rejected.

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

This migration has not been deployed or tested on hardware. Historical runs
of the experimental tree do not validate this revision. Keep station files,
credentials, measured calibration and run artifacts outside source.
See [source attribution](SOURCE.md).
