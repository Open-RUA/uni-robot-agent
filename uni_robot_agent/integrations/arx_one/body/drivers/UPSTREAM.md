# Third-party sources and RUA changes

URLs, revisions, target architectures, SHA-256 hashes and license locations are pinned in
[dependencies.lock.json](dependencies.lock.json) (default, AMD64) and
[dependencies.multiarch.lock.json](dependencies.multiarch.lock.json) (ARM64 and AMD64 with the
official ARX library). `dependencies.py` fetches or imports each artifact, verifies its hash and
stages it for the offline build; nothing unverified enters a release.

| Component | Pinned source | License |
| --- | --- | --- |
| ARX X5 SDK | `ARXroboticsX/ARX_X5`, ROS2 at `c78328785ea23a81d908e1dcc551eca992f2f1e9` (2026-04-28): three ABI headers and the per-architecture `libarx_x5_src` libraries | BSD-3-Clause |
| Eigen | `libeigen/eigen` 3.4.0 at `3147391d946bb4b6c68edd901f2add6ac1f31f8c` | MPL-2.0 (keep the archive's COPYING notices) |
| can-utils | Ubuntu noble `can-utils 2023.03-1` `.deb` for each target; only `slcand` is installed | GPL-2.0-or-later for `slcand` |
| librealsense | AMD64: 2.58.4 at `34d6c778e1134d8505adcd56bb57acbe7598a459` with the clock repair below, built with RSUSB; ARM64: 2.58.3 at `dfd6aa91250f5c31521d72d627865417989bb4e7`. Binaries are imported as locked artifacts; there is no public binary URL | Apache-2.0 |
| X5 model | `x5_2025.urdf` and meshes from the ARX and `arx5-sdk` sources, see [arx_x5/models/arx_x5_2025/README.md](arx_x5/models/arx_x5_2025/README.md) | BSD-3-Clause and MIT, in `models/arx_x5_2025/licenses/` |

## RUA changes to third-party code

**J4 range.** The default lock installs the ARX library with the two J4 bound constants changed
from ±1.29 rad to ±90°, the documented joint range. `dependencies.py` checks the official input
hash, rewrites only those constants and checks the output hash in the lock
(transforms `rua_arx_amd64_j4_90_v1` and `rua_arx_arm64_j4_90_v1`). This is an RUA modification,
not a manufacturer release; see [arx_x5/AMD64_J4_90.md](arx_x5/AMD64_J4_90.md).
`dependencies.multiarch.lock.json` keeps the official library and its limits.

**librealsense clock repair.** `realsense/sdk_patch/` replaces `src/global_timestamp_reader.cpp`
of the upstream 2.58.4 source (`prepare_source.py` checks the upstream file hash first). When
the D405's 32-bit microsecond clock wraps, the SDK shifts its stored sample times to the new
epoch but not the last request time, so the next fit extrapolates across the wrap; the repair
shifts that value too and clamps both interpolations to [0, 1]. The header is unchanged.

**Controller packages.** `arx_x5/ros/src/` holds the vendor `arx5_arm_msg` and `arm_control`
message packages and the `arx_x5_controller` node, derived from the ARX ROS2 sources
(BSD-3-Clause, see [arx_x5/ros/LICENSE](arx_x5/ros/LICENSE)). The controller keeps the vendor's
modes and wire messages and adds what the Body contract needs:

| Change | Purpose |
| --- | --- |
| Gripper feedback reference | Report the post-zero gripper reference instead of subtracting a stale vendor reference twice |
| Startup hold | Start from the measured joints and gripper after vendor startup and hold in joint mode, instead of returning to GO_HOME |
| Opened gripper after zeroing | `joint_zero` and home finish with an open gripper |
| Command limits | Configurable displacement, speed and acceleration with staged trajectories |
| Input freshness | VR and command inputs are checked for receipt-time freshness and range |
| Bounded responses | Joint and status serialization is bounded; `get_error_summary` returns at most 64 distinct fault codes plus the original count and a truncation flag. Any fault still blocks readiness |
| Constructor parameters | Keep `arx_x(150, 600, 10)` from the validated deployment, not upstream `arx_x(500, 2000, 10)` |

Keyboard examples and launch files that the Body does not use are left at the pinned upstream
source.

## Models and calibration

`arx_x5/ros/src/arx_x5_controller/x5_2025.urdf` is the single editable model;
`arx_x5/models/export.py` generates the portable copy and `--check` rejects drift. Camera
extrinsics and the mounting of the two arms are site calibration (`deploy/configs/calibration/`), not
part of the model. URDF `base_link` and the vendor EEF origin differ by the zero-pose
translation; do not interchange them when commanding Cartesian motion.
