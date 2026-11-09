# ARX ONE Body

The Body is the device process. Here it owns the ARX X5 arms and the D405 cameras and serves them
through the Body RPC contract: per-device leases, idempotent operations keyed by request,
quarantine after unknown outcomes, command streams and an event log. It outlives agent episodes,
so controllers keep holding the arms when an episode or the agent stops. Importing this package
does not create a ROS node, open CAN or USB, or start a controller.

## Layout

| Path | Contents | For another robot |
|---|---|---|
| `core/` | Body framework: the `Driver` interface, contract, server, client, RPC, leases, operations, streams, observations, manifest, timing, runtime root and device locks; `arm/` adds the arm contract, end-effector paths and trajectory limits; `camera/` the camera contract | Reuse unchanged |
| `host/` | `rua-host`: host configuration, serving, installation, diagnostics, runtime build and the kit interface; it names no robot | Reuse unchanged |
| `drivers/fake/` | The smallest complete driver and kit: in-memory scalar devices, the smallest arm that meets the arm contract (`driver_args.kind: "arm"`) and a synthetic RGB-D camera that meets the camera contract | Start here |
| `drivers/arx_x5/` | X5 driver, SDK limits, safety preflight, continuous group motion, CAN feedback, J4 staging; `kit.py` and `host/` add CAN, controller lifecycle and install rules; `ros/` holds the controller packages, `models/` the URDF; `native/` builds them | Replace with yours |
| `drivers/realsense/` | D405 driver and native capture; `clock.py` turns SDK frame times into contract source times; `kit.py`; `capture/` holds the C++ helper, `sdk_patch/` the SDK clock repair; `native/` builds them | Replace or keep |
| `drivers/dependencies.py` | Locked third-party artifacts: fetch, verify and stage offline | Reuse with your own lock |
| `runtime.json` | Which kits the runtime packages, and the dependency lock | Edit |

Which Bodies exist and how they form a robot is client-side configuration in `../offline/fleet/`. A robot
that already has a device service (ROS 2 control, a vendor RPC) does not need a Body at all: the
episode environment talks to devices only through `environment/control.py` and `native_compat.py`,
so adapting those to another service is the alternative to writing a driver.

## Writing a driver

A driver is one class with the `Driver` methods (`core/driver.py`). The Body server around it adds
everything else: authentication, leases, request idempotency, operation tracking and quarantine,
the stream watchdog, the event log and the RPC. A driver never sees a token or a lease.

| Method | Must do |
|---|---|
| `manifest()` | Return the devices with their kind, signals, commands, procedures (parameter JSON schema) and metadata; fixed for the driver's lifetime |
| `health()` | `{"ok": bool, ...}` without touching hardware state |
| `read(device_id, include)` | `{"signals": {...}, "freshness": "fresh" \| "stale" \| "unknown", "age_s", "moving", "timing"}` from the latest feedback |
| `check(device_id)` | `{"readiness": "satisfied" \| "unsatisfied" \| "unknown", "checks": {...}}`: whether the device may be commanded |
| `invoke(device_ids, procedure, params, cancel=, progress=)` | Run one declared procedure to completion; poll `cancel`, stop safely when it is set and return `{"partial": true}` |
| `write(device_id, command, value, cancel=)`, `stream_apply(device_id, targets)` | Direct and streamed commands, for devices that declare `commands`/`stream`; otherwise raise `NOT_SUPPORTED` |
| `stream_hold(device_id)`, `estop(device_id)` | Hold at the measured position using only fresh feedback, even after a fault; `estop` also aborts running work and says what it did |

Optional: `data_origin` (`"real"` unless set), `read_observation_at` for source-stamped history and
`close()`. Raise `DeviceError(ErrorCode...., message)` for refusals; anything else is reported as an
unknown outcome and quarantines the device until an operator acknowledges it.

Steps:

1. Copy `drivers/fake/` to `drivers/<name>/`; keep the `driver(**driver_args)` factory and replace
   the class with yours. Run it locally with `body.host.serve --driver ...:driver` and test it in
   memory as `tests/integrations/arx_one/body/test_device_contracts.py` does.
2. In `kit.py`, validate `driver_args`, claim the hardware each Body uses and list the signals
   `doctor --connect` reads. Add `native/` only if you build vendor code.
3. List the kit in `runtime.json` and reference it from `host.json`.
4. If the episode environment drives the device, meet the arm or camera contract below.

## Arm contract

`core/arm/contract.py` states what an arm device must report for `environment/` to drive it, and
`contract.problems(describe_entry)` lists every gap; the environment service refuses a Body that has
any. Names only, no vendor fields:

| Part | Required | Optional |
|---|---|---|
| `kind` | `"arm"` | |
| Metadata | `identity` (facts the installation pins: model, firmware or SDK version and hash, limit profile), `joint_limits_rad` (`lower`/`upper` per joint), `motion_limits` (every `MotionLimits` field, positive), `eef_coordinates` (`eef.coordinate_metadata`) | `model_sha256` (checked against the configured URDF) |
| Signals | `joint_position_rad`, `eef_position_m`, `eef_rpy_rad`, `gripper_opening` (1 open, 0 closed) | `gripper_opening_measured` (false while the opening is an estimate; absent means measured) |
| Procedures | `move_eef` (absolute pose in `<device>/base`), `move_joints` (`kind`, `values`, `duration_s`), `set_gripper` (`opening`) | `move_eef_group` (one preflighted path for several arms; a target is a pose or `{"hold_joints": [...]}`): enables continuous motion and gripper-only holds. `calibrate_gripper` for the operator command |

Stopping and holding come from every driver's `estop` and `stream_hold`. Everything else a driver
declares, such as the ARX raw currents or `gripper_opening_basis`, is its own diagnostics: the
environment never reads it. The fake arm declares exactly the contract and nothing else, so its
manifest is the template.

## Camera contract

`core/camera/contract.py` states what a camera must report for `environment/` to observe through it;
`problems` lists the gaps and the environment service refuses a Body that has any.

| Part | Required | Optional |
|---|---|---|
| `kind` | `"camera"` | |
| Metadata | `identity` (facts such as model and serial number) | |
| Observation stream `rgbd` | Signals `rgb` (JPEG), `depth` (uint16 millimetres registered to the colour grid, `u16le_b64` or `png16_b64`), `intrinsics.color` (`fx`, `fy`, `cx`, `cy`, `width`, `height`, `model` from `none`, `brown_conrady`, `inverse_brown_conrady`, and five coefficients `d`) | `depth_png` (read instead of `depth` when declared) |
| Sample timing | `source_timestamp_ns`, `clock_id` and `source_clock` of the colour image; `source_times.rgb` and `source_times.depth` on the Body host clock; `source_times.pairing.valid` once both images are known to come from one capture | `source_time_error` (the sample is refused) |

How a driver derives source times from its SDK is its own concern; the environment only checks them
against the request and the host clock. The RealSense driver does it in `drivers/realsense/clock.py`
from SDK Global Time and the sensor counter; the synthetic camera in `drivers/fake/` shows the result
directly. Camera-to-world and camera-to-arm transforms are calibration, kept in `environment.json`.

## Commissioning

Commissioning pins three things per arm in `environment.json`: `expected_metadata` (`{"identity":
...}`), `joint_lower` and `joint_upper`. `operator_cli identity` prints them from the running Body and
shows how the configuration differs, and does the same for each camera's `expected_metadata`; see
[DEPLOYMENT.md](../DEPLOYMENT.md) §4. Put only facts in
`identity` (hashes, versions, profile names), never documentation links or prose: any edit to it
makes existing installations refuse the Body until they re-pin.

## Kits

The host never names a driver. Each driver package that runs on an installed host has a `kit.py`
(interface in `host/kit.py`) that tells the host:

- the Driver factory, how to validate its `driver_args` and host-level configuration, and which
  hardware each Body claims (no two Bodies may claim the same bus or serial);
- extra arguments at serve time, native files and prerequisites for `doctor`, and the signals that
  `doctor --connect` reads;
- root-owned files for `install-config` (permissions, device rules) and extra `rua-host` commands;
- what the runtime build packages: Python subpackages, native inputs, `native/build.sh` and
  `native/env.sh`, package dependencies and build-time consistency checks.

`host.json` lists the kits it uses under `kits` with their configuration, and each Body names its
`kit`. For ARX ONE, `kits.arx_x5` holds the ROS domain and the USB-CAN adapters and adds the
`controllers` and `can` commands; `kits.realsense` is empty. To support another device, add
`drivers/<name>/` with `driver.py` and `kit.py` (copy `drivers/fake/`), list it in `runtime.json`
and reference it from `host.json` (see Writing a driver above).

## Runtime

`host/build.py` packages `core/`, `host/` and the kits listed in `runtime.json` into a standalone
`runtime.pyz`, with their native sources, without the agent. `host/build_runtime.sh` checks every
input hash, runs each kit's `native/build.sh` (for ARX ONE: the ROS controller and the camera
helper) and produces a `.deb`; see [DEPLOYMENT.md](../DEPLOYMENT.md). The installed release
provides `rua-host`, which sources each kit's `env.sh` before running a command;
`../deploy/scripts/rua-host-local.sh RELEASE HOST_CONFIG COMMAND...` runs an unpacked release.

```bash
python -m uni_robot_agent.integrations.arx_one.offline.fleet demo   # in-memory fake Bodies
python -m uni_robot_agent.integrations.arx_one.body.host.serve \
  --driver uni_robot_agent.integrations.arx_one.body.drivers.fake.driver:driver   # serve one driver locally
rua-host validate --config host.json   # identities and configuration only
rua-host doctor --config host.json     # installed files and prerequisites; nothing starts
rua-host doctor --config host.json --connect --token-file observe-token --timeout-s 3
```

`doctor --connect` sends only health, describe, check and state reads. Successful reads do not
mean the robot is ready to move.

## Host lifecycle

- `rua-host install-config --config ...` (root) installs `host.json` and the files its kits
  need; for ARX ONE the CAN configuration, scoped sudo rules and udev rules. It starts nothing.
- `rua-host body --config ... --instance arms` serves the arms Body against running controllers;
  it does not start them. `--instance cams` opens the cameras and serves observations.
- `rua-host controllers status|diagnose|start|stop|recover --config ... --instance arms` (from
  the `arx_x5` kit) manages controller processes. **Starting a controller is a physical action**: the vendor constructor can
  home an arm and close its gripper. `--device arx.left_arm` scopes it to one arm.

Each instance has its own port, journal and state. Controller ownership is tracked by PID, start
time, executable and arguments, so installing a new release does not restart a running controller.
Process teardown is not a stop confirmation; use the Body's lease and stop evidence.

Tokens live in a local table `{token: {name, scopes, device_ids}}`. Reads need `observe`, motion
`operate`, controller lifecycle `lifecycle`. Generate your own and never publish them.

## Arms

The driver attaches to the vendor `normal` controllers over ROS 2. Every move is checked against
the SDK joint and end-effector limits by sampling the path with IK before dispatch; a path that
leaves the limits runs only its verified prefix. Both arms move as one group without intermediate
stops, and an idle arm holds its measured joints and reports deviations without cancelling the
other. Feedback is tracked on every control tick; a tracking fault holds the arm and is reported.
The gripper opening scale comes from calibrated open and closed feedback.

## Cameras

Each D405 is read by a C++ child process behind an ordinary-user pipe. The child verifies USB and
SDK identities against the host configuration, bounds SDK calls and stops when its pipe closes or
its 10 s lease lapses; it never resets USB or writes firmware. Depth is registered to the colour
frame with the SDK projection and nearest-pixel z-buffering, converted to millimetres, and holes
stay invalid. Each camera keeps a bounded frame queue and a source-stamped history; processing
overlaps across cameras with one packet in flight per camera, preserving order. Intrinsics and
timestamps stay paired with their source frames. The SDK clock repair keeps those timestamps
correct across the 32-bit microsecond wrap (see [DEPLOYMENT.md](../DEPLOYMENT.md)).
