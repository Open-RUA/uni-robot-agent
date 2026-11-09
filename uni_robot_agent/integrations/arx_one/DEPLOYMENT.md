# Deploying ARX ONE on a new machine

This repository contains the integration code, the Body runtime source and configuration
templates. Third-party binaries, the robot meshes, measured calibration and every
machine identity are not distributed. This guide lists what you supply and how.

| Item | In the repository | You supply |
|---|---|---|
| Integration, Body runtime and controller source | Yes | — |
| ARX X5 vendor SDK (`libarx_x5_src.so`) | Pinned URL, hashes and the J4 patch | Download and patch it (§2.1) |
| RealSense SDK (librealsense) | Clock-repair source overlay and build options | Build and package it (§2.2) |
| Eigen 3.4.0, can-utils | Pinned URLs and hashes | Fetched automatically (§2.3) |
| X5 meshes | URDF and `SHA256SUMS` | Download them (§2.4) |
| Host, site and environment configuration | `deploy/configs/*.example.*` templates | Your identities and paths (§4) |
| Hand-eye, wrist-camera, gripper and TCP calibration | Calibration code | Measure them on your robot (§5) |
| SAM 3 checkpoint, LingBot depth source and weights | Config templates | Download them (§6) |

Run the build on Ubuntu 24.04 AMD64 (ARM64 is also locked) with ROS Jazzy and CPython 3.12. The
commands below run from the repository root unless stated otherwise. Paths under `body/` and
`deploy/configs/` are relative to `uni_robot_agent/integrations/arx_one/`.

## 1. Python environment

```bash
python -m pip install -e '.[arx_one]'
```

The `arx_one` extra includes OpenCV for calibration and SciPy for geometry.

## 2. Third-party binaries

All binaries go into a dependency cache that the runtime build reads offline. The lock file
`body/drivers/dependencies.lock.json` pins every artifact by SHA-256. Hardware startup never
downloads anything, and a rebuilt SDK needs its own hashes before the lock is changed.

### 2.1 ARX X5 SDK with the J4 ±90° patch

The default lock uses a locally modified vendor SDK: only the J4 limits change from ±1.29 rad to
±π/2 (see `body/drivers/arx_x5/AMD64_J4_90.md`). It is not a vendor release. Download the official
library and apply the patch with the bundled transform:

```bash
curl -fsSL -o libarx_x5_src.official.so \
  https://raw.githubusercontent.com/ARXroboticsX/ARX_X5/c78328785ea23a81d908e1dcc551eca992f2f1e9/ROS2/X5_ws/src/arx_x5_ros2/arx_x5_controller/lib/arx_x5_src/libarx_x5_src.so
sha256sum libarx_x5_src.official.so   # d07d53d14da17f0a1db2753e3339f9e4c055dad72df613c811b9692c4bede85c
python - <<'PY'
from pathlib import Path
from uni_robot_agent.integrations.arx_one.body.drivers.dependencies import apply_arx_amd64_j4_90
Path("libarx_x5_src.so").write_bytes(apply_arx_amd64_j4_90(Path("libarx_x5_src.official.so").read_bytes()))
PY
sha256sum libarx_x5_src.so            # ec4a7e7f2528797c6e9fcf1b30e773165be81b8d8130ed28c00510fd651e67a7
```

To keep the vendor limits instead, build with `dependencies.multiarch.lock.json`, which downloads
the official library, and start from `deploy/configs/host.type2.example.json` (profile
`x5_2025_amd64_type2`) for your host and environment configuration.

### 2.2 RealSense SDK with the clock-wrap repair

The D405 clock is a 32-bit microsecond counter that wraps every 4294967.296 ms. Upstream
librealsense does not shift its last request time when it rebases at a wrap, so the next fit can
extrapolate far backwards and reject good frames until it rebuilds. The overlay in
`body/drivers/realsense/sdk_patch/` shifts that value with the others and bounds both
interpolation paths to [0, 1]; fitting, rejection gates and fallback are unchanged. Keep its
Apache 2.0 `LICENSE` when you distribute it.

The AMD64 lock pins librealsense 2.58.4 (`34d6c778e1134d8505adcd56bb57acbe7598a459`) with this
repair; ARM64 pins 2.58.3 (`dfd6aa91250f5c31521d72d627865417989bb4e7`). Your build has different
bytes, so record your own hashes:

```bash
git clone https://github.com/realsenseai/librealsense
git -C librealsense checkout 34d6c778e1134d8505adcd56bb57acbe7598a459
python uni_robot_agent/integrations/arx_one/body/drivers/realsense/sdk_patch/prepare_source.py \
  --source librealsense --output librealsense-clockfix
cmake -S librealsense-clockfix -B rs-build -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON -DBUILD_SHARED_LIBS=ON -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_WITH_DDS=OFF -DBUILD_WITH_NEON=OFF -DHWM_OVER_XU=ON -DBUILD_GLSL_EXTENSIONS=OFF \
  -DBUILD_ROSBAG2=OFF -DCHECK_FOR_UPDATES=OFF -DENABLE_CCACHE=OFF \
  -DCMAKE_INSTALL_PREFIX="$PWD/rs-install"
cmake --build rs-build -j && cmake --install rs-build
cp librealsense-clockfix/LICENSE librealsense-clockfix/NOTICE.md rs-install/
tar -czf librealsense-sdk-amd64-clock-wrap.tar.gz -C rs-install LICENSE NOTICE.md include lib
```

`prepare_source.py` checks that both upstream files match the recorded baseline before it writes
the repaired copy, and never overwrites its input. Then update `dependencies.lock.json`:

- `dependencies.librealsense.artifacts[0]`: `import_name` and `sha256` of the archive.
- `dependencies.librealsense.target_overrides.linux-amd64.shared_library_sha256` and
  `runtime_files.linux-amd64["realsense/lib/librealsense2.so.2.58.4"]`: the SHA-256 of
  `rs-install/lib/librealsense2.so.2.58.4`.

After installing, check that the running Body maps the new library (path and SHA-256) and watch the
cameras across at least two wraps; a short capture does not prove the repair is active.

### 2.3 Fetch the cache and build the runtime

```bash
python -m uni_robot_agent.integrations.arx_one.body.drivers.dependencies fetch \
  --cache build/native-dependencies/downloads --target linux-amd64 \
  --import-artifact libarx_x5_src.so \
  --import-artifact librealsense-sdk-amd64-clock-wrap.tar.gz
python -m uni_robot_agent.integrations.arx_one.body.host.build \
  --source . --output build/hardware-inputs --target linux-amd64 \
  --dependency-cache build/native-dependencies/downloads
bash build/hardware-inputs/build_runtime.sh build/hardware-inputs
```

`fetch` downloads Eigen and can-utils. `body.host.build` packages the kits listed in
`body/runtime.json` (here `arx_x5` and `realsense`); `build_runtime.sh` then runs each kit's
`native/build.sh`. For this robot that needs ROS Jazzy, colcon, CMake, a C++ toolchain and the ROS
dependencies declared under `body/drivers/arx_x5/ros/src/`; run it as an ordinary user. It verifies
every input hash, builds and tests the controller and camera helper, and produces a `.deb`.
Installing the package selects a release but starts nothing. See [body/README.md](body/README.md)
for `rua-host`.

### 2.4 X5 meshes

Only visualization needs them. Download them into
`body/drivers/arx_x5/models/arx_x5_2025/meshes/` as described in
`body/drivers/arx_x5/models/arx_x5_2025/README.md`.

## 3. Hardware

Two ARX X5 arms (type 2 gripper end) on two USB-CAN adapters, and three RealSense D405 cameras:
one fixed head camera and one on each wrist.

## 4. Configuration

Copy the templates from `deploy/configs/` and replace every `REPLACE_*` value and `/path/to` path. Keep
the copies outside the repository; they hold your machine identities. The placeholders fail
validation on purpose.

| File | From | Fill in |
|---|---|---|
| `host.json` | `host.example.json` (J4 ±90°, matches the default lock) or `host.type2.example.json` (official limits) | Operator user and token table path; under `kits.arx_x5` the ROS domain and USB-CAN serials; per Body its `kit` and `driver_args` (camera USB and SDK serials, gripper calibration path) |
| Token table | Format in [body/README.md](body/README.md) | Your own `observe`, `operate` and `lifecycle` tokens |
| Site file | `site.example.yaml` | Body URLs, the environment variable holding each Body's token (`token_env`), calibration bindings (§5) |
| `environment.json` | `environment.example.json` | Arm and camera transforms, grasp site, URDF path and hash, camera frame periods, motion profiles (§5); each arm's `expected_metadata`, `joint_lower` and `joint_upper` and each camera's `expected_metadata` from `operator_cli identity` (below) |
| Depth | `depth.example.json` or `depth-shared.example.json` | LingBot paths, or the shared depth service URL |
| SAM 3 | `sam3.example.toml` | Checkpoint path and GPU |

A camera's USB/ASIC serial and its SDK serial are separate identities; record both. The ARX
profile of a Body and of each arm must agree and match the physical gripper end and the installed
SDK. The robot, Body and instance names in the templates (`arx_one`, `arx_one_arms`, `arms`,
`cams`) are free choices; keep them consistent across the files.

```bash
rua-host validate --config /path/to/host.json
rua-host doctor --config /path/to/host.json
```

Once the Bodies run (§7), copy what they report for each arm and camera into `environment.json`. The
command only reads, and works on a configuration that is not complete yet:

```bash
python -m uni_robot_agent.integrations.arx_one.operator_cli --config environment.json identity
```

For each arm it prints `expected_metadata` (`{"identity": {...}}`: the controller profile, SDK and
model hashes the driver reports), `joint_lower` and `joint_upper`; for each camera its
`expected_metadata` (model and serial); plus `differs_from_config` and any `contract_problems`. Copy
these fields as printed. The environment service refuses a Body
whose identity or limits differ from them, so after a runtime upgrade that changes the SDK, model or
limits, run the command again and review the difference before copying it.

## 5. Calibration

None of these values transfer between robots; measure each on your installation.

| Value | Where it goes | How |
|---|---|---|
| Head camera to each arm base | `environment.json` `cameras.head.T_parent_from_optical` and each arm's `T_world_from_controller` | Eye-to-hand calibration (below) |
| Each wrist camera to its EEF | `cameras.<wrist>.T_parent_from_optical` | Eye-in-hand calibration (`solve-eye-in-hand`, below) |
| Grasp site (TCP) | Each arm's `T_native_from_grasp`, `grasp_site`, `grasp_site_description`, `rotation_radius_m` | Measure the offset from the vendor EEF to the gripper center |
| Gripper open and closed feedback | File named by `calibration_path` in `host.json` | `operator_cli calibrate-gripper ARM --yes` once per arm, with nothing between the fingers |
| Arrival tolerances | `motion_profiles` in `environment.json` | Choose for your robot's tracking precision; the template holds the defaults |

The vendor controller records the gripper's current position as closed each time it starts. Close
both grippers fully before starting the controllers, or the opening scale is wrong until the next
start with the grippers closed.

### Hand-eye tools

```bash
python -m uni_robot_agent.integrations.arx_one.offline.calibration inspect /path/to/record.json
python -m uni_robot_agent.integrations.arx_one.offline.calibration fit-points --input points.json --output record.json
python -m uni_robot_agent.integrations.arx_one.offline.calibration solve-hand-eye --input poses.json --output record.json
python -m uni_robot_agent.integrations.arx_one.offline.calibration solve-eye-in-hand --input wrist-poses.json --output record.json
python -m uni_robot_agent.integrations.arx_one.offline.calibration verify-hand-eye --record frozen.json \
  --input validation-poses.json --output report.json
```

Outputs never overwrite existing files; a rejected fit or failed verification is still written as
evidence and exits with code 2. Every output records its input SHA-256, and verification never
changes a record's status or refits it.

- Coordinates are metres; `T_A_B` maps B into A (`p_A = R_A_B p_B + t_A_B`). Matrices must be
  proper rotations. Inputs carry `source_frame`, `target_frame`, `camera_serial`, `intrinsics`,
  `mount_id` and `measured_at`.
- `fit-points`: at least six matched camera/base points spread in two axes, plus at least four
  independent validation points, and a `method` naming how the points were measured.
- Hand-eye inputs give each sample's `base_from_tool` from fresh, stationary arm feedback (the
  controller `/base` to the native EEF) and the detected `camera_from_target`.
- `solve-hand-eye` (fixed camera, board on the tool) and `solve-eye-in-hand` (camera on the tool,
  board fixed to the base): measured board geometry and stationary samples; at least 12 fitting
  poses and four held-out poses with rotation in two axes. Eye-in-hand writes `tool_from_camera`,
  which is the wrist camera's `T_parent_from_optical`. The OpenCV solver needs OpenCV 4 (the
  `arx_one` extra pins it).
  ChArUco uses `DEFAULT_CHARUCO`; AprilGrid needs measured rows, columns, tag size, gap and
  dictionary. There is no implicit board.
- `verify-hand-eye`: at least four new stationary samples against a frozen record that includes
  `tool_from_target` and the board geometry.
- `max_error_m` is at most 0.02 m. Passing fit or verification shows geometric closure only, not
  TCP accuracy, depth accuracy or reachability. Hand-eye transforms target the controller `/base`,
  not URDF `base_link`.

### Site bindings

A Site file binds each calibrated pair explicitly; there are no defaults or fallbacks to estimated
transforms. Put records under `deploy/configs/calibration/<robot>/` (ignored by Git) and refer to them as
`package:deploy/configs/calibration/...`, or give a path relative to the Site file:

```yaml
calibrations:
  head-left:
    robot: arx_one
    camera: head
    target_arm: left
    record: package:deploy/configs/calibration/arx_one/head-to-left.calibrated.json
    sha256: SHA256_OF_THAT_RECORD
    camera_serial: HEAD_CAMERA_SERIAL
    mount_id: YOUR_MOUNT_ID
    source_frame: rs.head/color_optical
    target_frame: arx.left_arm/base
```

Loading the Site checks hashes, identities and validated status offline. Only fixed cameras can be
bound this way; wrist cameras need time-correlated joints. Points outside a record's validation
volume are converted without an inherited error bound, and conversions are never `motion_ready`.

Site bindings serve the offline tools. An episode reads its transforms from `environment.json`:
copy them from validated records (`transform_m`), and keep each record and its SHA-256 with your
configuration so the values stay traceable.

## 6. Perception services

- **SAM 3:** run `deploy/scripts/run_sam.sh` with a checkpoint from the SAM 3 release. The PyPI
  `sam3` wheel does not install its tokenizer: copy `bpe_simple_vocab_16e6.txt.gz` from the SAM 3
  source to `assets/` under the service environment's `site-packages`, where `sam3` looks for it.
- **Depth completion (optional):** LingBot source at revision
  `f3a237e434ae987bc38281476d6cfb5df3e4d739` (record it in `source/revision.txt`) and v0.5 weights
  with SHA-256 `b60cf27ddbd0e51e9b59b03475c0d39d02d2e48ecf8dbb5866f04d46802b3c23`, in a separate
  Python environment where this package is importable. A run can start its own worker
  (`depth.example.json`) or use a resident service shared by runs (`depth-shared.example.json`):

  ```bash
  python -m uni_robot_agent.integrations.arx_one.perception.completion.service \
    --depth-config /path/to/depth.json --state-dir /path/to/state --port 18875
  ```

  The worker keeps the model loaded and serializes requests; queueing, cold start and inference
  share one deadline (default 120 s). Completed depth keeps every valid measured pixel and uses the
  model only at holes.

### Offline inspection

`offline` captures and inspects observations without an agent. `demo` uses a synthetic
Body; `capture` sends read requests only and never starts a controller or moves an arm. The other
commands work on an archive directory.

```bash
python -m uni_robot_agent.integrations.arx_one.offline.capture demo --output /tmp/observation-demo
python -m uni_robot_agent.integrations.arx_one.offline.capture capture --site site.yaml --robot arx_one \
  --camera head --timeout-s 3 --output ./capture
python -m uni_robot_agent.integrations.arx_one.offline.capture list --output ./capture
python -m uni_robot_agent.integrations.arx_one.offline.capture filter-depth --output ./capture OBSERVE_ID --camera head
python -m uni_robot_agent.integrations.arx_one.offline.capture complete-depth --output ./capture OBSERVE_ID \
  --camera head --depth-config /path/to/depth.json
python -m uni_robot_agent.integrations.arx_one.offline.capture back-project --output ./capture OBSERVE_ID \
  --camera head --pixels '[[120,160]]' --depth-source filled --depth-artifact DEPTH_ID
```

Run capture on the Body host so image and joint timestamps share one clock. Geometry queries name
the exact depth artifact; measured depth needs `--depth-source measured`.

## 7. Running

```bash
python -m uni_robot_agent.integrations.arx_one.operator_cli --config environment.json status
python -m uni_robot_agent.integrations.arx_one.operator_cli --config environment.json up \
  --runtime-root RELEASE --host-config host.json --yes
deploy/scripts/run_env.sh    # one physical episode
deploy/scripts/run_agent.sh  # the agent against that episode; choose any supported model
```

`up` starts the controllers, which is a physical action. `up` and `down` name their systemd units
`<prefix>-body-<instance>` and `<prefix>-controllers-<instance>-<time>`; `--unit-prefix` (default `rua`)
must match the units already running. `down`, `home`, `gripper`,
`calibrate-gripper` and `ack` are operator commands; the model cannot call them.

## Upgrading an earlier layout

A machine set up before the integration was regrouped needs these changes; deployed files keep
working until they are made.

- Move measured calibration from `configs/calibration/` to `deploy/configs/calibration/`; only the
  new path is ignored by Git and excluded from packages. Change Site records from
  `package:configs/...` to `package:deploy/configs/...`.
- Module paths: `arx_one.fleet` is `arx_one.offline.fleet`, the former `arx_one.offline` is
  `arx_one.offline.capture`, and `arx_one.calibration` is `arx_one.offline.calibration`. Launch
  scripts are under `deploy/scripts/`.
- `host.json` needs `"schema_version": 2`: host-level kit settings under `kits`, and a `kit` per Body
  (see §4 and `deploy/configs/host.example.json`). Rebuild the runtime afterwards (§2.3).
- In `environment.json`, each arm's `expected_metadata` is now `{"identity": {...}}`; the former
  `vendor_limit_profile` and `eef_coordinates` entries are gone (the coordinate convention is checked
  by the arm contract). Each camera's `serial` is replaced by `expected_metadata` (`{"identity":
  {"model": ..., "serial": ...}}`). Copy both from `operator_cli identity` (§4) after the new runtime
  serves.
- Units started by an earlier `operator_cli up` are named `rua-arx-one-...`; pass
  `--unit-prefix rua-arx-one` to `down` once, or stop them by hand, before `up` with the new default.
- The motion failure reason `sdk_limit` is now `command_limit`; update anything that matches on it.

## Not needed on another machine

- Our measured calibration, camera serials, tokens and deployment paths.
- Model credentials and model-specific launch settings: use the models and options the
  `uni-robot-agent` CLI supports.
- The local test console and migration scripts used during development.
