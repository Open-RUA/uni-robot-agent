# Environment integrations

uni-robot-agent uses shared robot tools across simulator and physical environments.
Both `run` and `tui` require an explicit `--integration`; there is no default.
Choose an integration, then prepare its external services. Python extras install
harness-side dependencies; simulators, hardware runtimes, assets and model
checkpoints remain separate.

## Included integrations

| Integration | Use | External services |
|---|---|---|
| `robodojo` | Official RoboDojo simulation, task requirements and bundled gripper skills | RoboDojo environment; Hybrid adds an identified Pi0.5 model and a reviewer by default |
| `robouse` | Direct simulation tasks, including LIBERO and RoboCasa | Prepared profile-specific RoboUse environment |
| `libero_plus` | Official LIBERO-Plus perturbation tasks, Direct control | [Separate pinned LIBERO-Plus runtime](libero-extensions.md) |
| `robotwin` | RoboTwin 2.0 dual-arm tasks, Direct control | [Pinned RoboTwin runtime and assets](robotwin.md) |
| `libero_pro` | Selected official LIBERO-PRO suites, Direct control | [Separate pinned LIBERO-PRO runtime](libero-extensions.md#libero-pro) |
| `rmbench` | Memory-dependent dual-arm manipulation, Direct control | [Pinned RMBench runtime and assets](rmbench.md) |
| `calvin` | Single-task Panda episodes, Direct control | [Pinned CALVIN environment and annotations](calvin.md) |
| `robolab` | RoboLab DROID, Direct scalar-gripper control | Prepared RoboLab/Isaac environment; RGB cameras |
| `dexjoco` | MuJoCo Panda/Allegro manipulation through Hybrid | Pinned DexJoCo environment and an identified OpenPI actor |

These simulation integrations share scene, gripper, skill and episode tools.
RoboDojo, RoboUse and RoboLab declare Direct and Hybrid tool profiles; Hybrid
requires an admitted environment/controller and actor binding. LIBERO-Plus,
LIBERO-PRO, RoboTwin, RMBench and CALVIN declare Direct only.
RoboLab has no automatic setup/deploy recipe. Use
SAM3 only where the integration supports it; otherwise pass `--no-sam3`. Depth-based pixel
localization requires paired depth and calibration; RGB triangulation requires
at least two calibrated views.

## Real-robot release status

`arx_one` supports a dual-arm ARX ONE with its own Body runtime, on the shared gripper
tools. See [ARX ONE](../uni_robot_agent/integrations/arx_one/README.md); a new machine
supplies third-party binaries, meshes and calibration as described in its
[deployment guide](../uni_robot_agent/integrations/arx_one/DEPLOYMENT.md).
Public support for **Franka** is **Coming soon**.

## Inspect integration options

```bash
uni-robot-agent run --integration robouse --help
uni-robot-agent run --integration robodojo --help
```

See [getting started](getting-started.md) for installation and a first offline
configuration check. The CLI is `uni-robot-agent`; the Python package is `uni_robot_agent`.

## Prepared episodes and shared tools

`--env-url` points to an already prepared episode, including its full allocated
path when using a pool. The harness checks optional expected identity fields;
it does not use them to create or reset a simulator. The episode's metadata
provides its task instruction, arms, cameras, calibration and action binding.
`--instruction` overrides the instruction for the agent and actor.

| Tool family | Interface |
|---|---|
| Scene | `view_env_state` reads recorded RGB/state; `capture_scene` freezes RGB/state and available calibration and paired depth; `locate` queries the immutable snapshot using the advertised capabilities |
| Direct motion | `move(targets, profile="standard")`, `grasp`, `release`, `hold` |
| Actor | `actor_act` runs bounded chunks with fresh observations and optional reviewer decisions |
| Guidance | `list_skills()` lists shared general/ and current-environment Markdown paths recursively; `read_skill(path=...)` reads a file's content and SHA256; `--skill-writing` adds `write_skill` within the episode's namespace and requires an explicit external skills root |
| Completion | `finish` submits termination; the official task outcome stays outside model-visible feedback |

Tool definitions live in [`tools/gripper/`](../uni_robot_agent/tools/gripper/),
[`tools/hybrid/`](../uni_robot_agent/tools/hybrid/),
[`tools/scene/`](../uni_robot_agent/tools/scene/), [`tools/skills.py`](../uni_robot_agent/tools/skills.py)
and [`tools/episode.py`](../uni_robot_agent/tools/episode.py).

An environment adjusts these tools through its description's `metadata`; an
environment that declares nothing gets the defaults, with unchanged schemas.

| Key | Default | Effect |
|---|---|---|
| `finish_statuses` | `["success"]` | Statuses `finish` offers, from `success`, `failure` and `give_up`; they are recorded as `completion_claimed`, `task_failed` and `task_abandoned` |
| `motion_profiles` | Shared `standard`/`approach`/`precision` | Replaces whole named profiles (`max_step_m`, `position_tolerance_m`, `orientation_tolerance_rad`, all positive) with arrival criteria matched to the arm; the `move` description states the effective values |
| `physical_hardware` | false | A physical environment's results never count as a confirmed stop by themselves; its tools must confirm the stop for [replan after failure](architecture.md#replan-after-failure) |
| `normalized_opening_tolerance` | None | For arms without `gripper_opening_models`, checks an explicit 0-1 opening against the measured 0-1 opening within this tolerance, in `(0, 0.5]`. Only readings whose arm metadata sets `gripper_opening_measured: true` are judged; others stay unknown |

## RoboDojo

`--integration robodojo` adds the RoboDojo prompt and optional task-requirements
loader to the shared prepared-episode lifecycle. `--task` and `--run-id` identify
the expected episode. `--include-extra-task-requirements` loads the repository's
task requirements; it is enabled by default for RoboDojo and loads the simplified
`resources/prompts/robodojo/instruction_supplements.json` catalog. These supplements
clarify the task instruction, including the required sequence.
The full `task_requirements.json` remains a separate reference resource.

### Realistic observation settings

The paper's **Realistic** principle excludes simulator-provided depth and
task-verifier feedback from agent/tool interaction. RGB images, proprioception,
camera calibration, robot descriptions and task instructions remain available.
Static task requirements describe the task; they are separate from a verifier's
live assessment of progress or success. `finish` acknowledges submission without
returning the official outcome to the agent. Outcomes are saved for evaluation.

RoboDojo captures depth by default. For the paper's RGB-only input setting,
pass `--no-depth` when starting the environment through `run --deploy local`,
`deploy env` or the native environment server. This retains RGB and camera
calibration. With at least two calibrated RGB-only cameras, `locate` supports
multi-view triangulation. When connecting with `--env-url`, configure these
capabilities on the existing environment service before starting the agent;
the agent connection does not disable service-side depth acquisition.

The harness also supports optional depth localization and physical RGB-D sensors.
These capabilities do not establish that a run uses the paper's RGB-only setting.
See [runtime defaults](running.md#single-episode-cli) for capability selection.

### Minimal robot tools

All eight gripper integrations support `--tool-profile minimal`: RoboDojo,
RoboUse, RoboLab, RoboTwin, LIBERO-Plus, LIBERO-PRO, RMBench and CALVIN.
It exposes `move`, `hold` and `finish`, reusing the same schemas, controllers,
execution receipts and outcome storage as Direct. `move` already accepts a
gripper opening, so separate grasp/release tools are unnecessary.

Minimal selects immediate tool returns and omits skill, scene, Hybrid, observer,
workspace and background-job tools. Explicit background returns, tool debugging,
skill writing, observation profiles and Code as Policy are rejected. Planner
SDK native computation/file tools retain their existing configuration. Direct
remains the default; Direct/Hybrid tool visibility and scheduling are unchanged.

Start the environment with depth disabled (for example RoboDojo `--no-depth`).
An external `--env-url` must already describe a depth-disabled service. Minimal
supports calibrated RGB and cameras that explicitly declare no calibration;
it does not expose a depth setting or silently change the connected service.
Initial input includes RGB images, measured robot state and per-image calibration.
Codex, Claude Code, Gemini CLI, OpenCode and Antigravity accept these images as
multimodal user input. The shared artifact store retains images and snapshot
manifests; `initial_observation.json` records the initial image/state references.

Completed gripper actions in every profile reuse their final snapshot when it
contains current images and the camera's declared calibration. Otherwise they
capture once after the model-level action, not after every primitive. Returned
`camera_calibration` includes `intrinsic_K` and `T_world_from_optical` for the
attached image grids, together with `image_state_version` and `snapshot_id`.
The latter transform maps optical coordinates into world metres. This may add
capture latency on RoboDojo, whose ordinary tiled snapshots lack calibration.
Missing declared calibration or a stale capture produces an `artifact_error`
while preserving the acknowledged action feedback; it never replays motion.
Cameras declaring calibration unavailable, including distorted LIBERO-Plus
agent views, retain null matrices instead of fabricated geometry.

### Camera-calibration ablation

For RoboDojo, add `--no-camera-calibration` at environment startup to disable
camera intrinsics and extrinsics as well; this option requires `--no-depth`.
Use `run --deploy local`, `deploy env`, or the native RoboDojo server. The environment
declares `has_calibration=false`. RoboUse and RoboLab do not support this switch;
explicit calibration overrides are rejected for those integrations and for actor
or SAM3 deployment. In `run --env-url` mode, configure the external service at
startup instead; the agent connection cannot change its camera capabilities.
Captures and action records contain no camera matrices or depth data; image paths,
sizes, robot proprioception and robot geometry remain available. `locate` exposes
only 2D SAM segmentation, or is absent when SAM is unavailable. Triangulation,
pixel-to-world localization and plane fitting are not exposed.
Use matching agent and service versions; older clients may reject the added
camera capability field.

### Hybrid control

Hybrid requires `--action-model-url` and an expected
`--action-model-checkpoint-sha256`. Its reviewer is enabled by default and uses
`--vla-review-base-url` or the inherited `--provider-url`. Reviewer model, adapter
and credentials are independently configurable; see
[reviewer configuration](running.md#vla-reviewer). `--no-vla-review` leaves the
[actor execution budgets](running.md#actor-execution-budgets) unchanged.

### Coordinated arm targets

`move`, `grasp` and `release` accept one or two distinct arm targets.
RoboDojo applies them together in each native control step, including in serial
execution mode. Unlisted arms hold; an arm reaching tolerance holds its achieved
pose while the other continues. A fault on either arm stops the call.

List order does not set execution order. Separate tool calls remain separate
actions even when emitted together. This coordinates target tracking, without
planning collision-free paths or maintaining a rigid relative pose between hands.

`move` requires absolute world [x,y,z,roll,pitch,yaw] in metres and fixed-axis
XYZ radians (R=Rz(yaw)@Ry(pitch)@Rx(roll)). State, primitive poses and Hybrid
previews keep wxyz quaternions at the same site. Feedback adds six-value
`target_pose`/`actual_pose` alongside existing position/quaternion fields.
Retain measured XYZ for an in-place turn. Translation and rotation interpolate
together; dependent stages need separate calls and completed feedback checks. Each integration
registers `control_frame`, `grasp_site_description` and `rotation_radius_m` on its
arm descriptors; shared tools interpolate poses without loading a robot model.
RoboDojo merges a fixed-site URDF extension into the native X5 chain for FK/DLS,
and converts site targets once for native link6 IK. RoboUse supplies backend-specific
grasp-site and controller conversions.
The site is a geometric convention, not object attachment evidence.

Environment protocol 4 and tool schema 8 define this interface. Upgrade services
and clients together. When replaying saved commands, verify that their recorded
coordinate conventions and schema match the target environment.

### Active gripper guidance

The [RoboDojo index](../uni_robot_agent/resources/skills/robodojo/index.md)
links to the [shared skill index](../uni_robot_agent/resources/skills/general/index.md)
for gripper control, handover, pose targeting, contact, placement, verification
and recovery. It adds X5 and environment-specific supplements. The agent selects
references from the observed situation and available capabilities.

Start with [the Direct example](../examples/README.md#robodojo). For actor-backed
control, use [the Hybrid example](../examples/robodojo_hybrid.sh) and
[environment service scripts](../scripts/deploy/services/robodojo/).
[RoboDojo SOURCE.md](../uni_robot_agent/integrations/robodojo/SOURCE.md) records pinned
upstream revisions and retained licenses. The official environment server requires
`--native-root` pointing to a clean pinned official checkout, including its submodules.
External runtime trees are not packaged.

## RoboUse

`--integration robouse` composes the shared scene, gripper, skill and episode tools
for RoboUse tasks, including LIBERO and RoboCasa. The `robouse_cartesian` binding
supports public world-frame absolute poses or latest-ACK-relative deltas on
admitted LIBERO and RoboCasa controllers. RoboCasa retains the controller target
orientation and rejects rotation. Other RoboUse families remain Direct-only until
their native capability contracts are explicitly validated. Tasks and seeds reuse
the same binding when those contracts match. Each native backend keeps its own
dependencies, task admission, step budget, scoring and camera configuration.

Use `setup --integration robouse --runtime-profile PROFILE` to prepare a native
runtime, then `run --integration robouse --deploy local --task TASK` with the
model and provider settings. For an unreleased build, pass your matching runtime wheel to
`setup --wheel`; a published installation can fetch its matching version from the
package index. See [deployment](deployment.md).
See [deployment profiles](deployment.md#environment-profiles-and-limits),
[RoboUse scripts](../scripts/robouse/README.md) and the
[implementation](../uni_robot_agent/integrations/robouse/).

## RoboLab

`--integration robolab` composes the shared scene, gripper, skill and finish providers
for a prepared single-DROID episode. See [RoboLab setup](robolab.md) for runtime
requirements, coordinate frames and supported features. The integration uses RGB
and calibrated multi-view triangulation, with the optional `robolab_droid_joint`
actor binding and no SAM3 binding.

## DexJoCo

`--integration dexjoco` connects a prepared native device episode to the shared
Hybrid tools. The first admitted task is `pick_bucket`, using the public
single-arm Pi0.5 checkpoint. The binding maps measured TCP/finger state and
absolute TCP/finger actions to one coordinated native control step. The native
OSC and Allegro controllers retain their original physics and task budget.

The integration requires Hybrid and exposes actor execution with optional
student/stop review. Direct motion and geometric action corrections are not
provided. Official task success is recorded only at finalization. See
[DexJoCo setup and validation](dexjoco.md) for exact support and baseline limits.

## Model bindings and backends

All Hybrid integrations share `ActorClient`, `ActorService`, `HybridTools` and
one execution loop. The integration selects its binding through a factory;
`integrations/hybrid.py` owns actor/reviewer connections.

| Environment | Binding | Actor semantics |
|---|---|---|
| RoboDojo | `robodojo_pi05` | Explicit legacy Pi0.5 config, images, RNG and clipping |
| RoboDojo X5 | `robodojo_x5_joint` | Absolute X5 joints and continuous normalized openings at 25 Hz |
| RoboLab DROID | `robolab_droid_joint` | Seven measured joints plus normalized finger angle; joint and physical-gap targets |
| RoboUse LIBERO / RoboCasa | `robouse_cartesian` | World public-control-point absolute poses or latest-ACK-relative deltas; native rotation constraints |
| DexJoCo | `dexjoco_tcp_hand` | Absolute world TCP pose plus named Allegro finger positions in one native step |

Except for the pinned legacy Pi0.5 path, bindings validate semantics without
requiring a particular backend runtime. XPolicyLab can serve any of these
contracts when its configured model actually implements them. A compatible
checkpoint change is deployment configuration. A new representation needs an
explicit binding conversion or reusable capability, not another actor loop.
See [Hybrid extension and migration](hybrid.md), the
[XPolicyLab bridge](services.md#xpolicylab-bridge), and
[service profiles](services.md#openpi-deployment-profiles).

## Integration contract

Expose `get_integration_spec()` from an integration module. The
[`IntegrationSpec` and `IntegrationSession` contracts](../uni_robot_agent/integrations/base.py)
keep the runtime independent of a simulator or transport:

1. **Configuration:** declare CLI arguments, validation, typed `config_model`
   options and tools available in task runs. Declare `deployment_module` only
   when automatic preparation/deployment is supported.
2. **Lifecycle:** return a session from `create_session(IntegrationContext)`
   implementing `start`, `finalize` and `close`.
3. **Tools:** provide `tool_providers`, `episode`, `task_succeeded()` and
   `action_model_name`. Return structured `ToolOutput` values.
4. **Scheduling:** declare resource policies for physical operations and
   lifecycle barriers; transport order does not establish exclusion.
5. **Outcomes:** return `EpisodeOutcome` with acceptance and failure
   classification. Retryable infrastructure failures are not accepted evaluation
   results. Set `allow_automatic_retry=False` when replay is unsuitable.

Register environment prompt builders explicitly; otherwise the generic builder
is used. Keep heavy imports in service processes or behind startup so CLI help
and offline validation work without native runtimes.

## Integration layout

```text
integrations/<name>/
├── __init__.py             # integration registration
├── prompt.py               # environment-specific context
├── action_binding.py       # model observation/action conversion, when needed
└── environment/
    ├── adapter.py          # EnvironmentBackend implementation
    ├── native_compat.py    # pinned runtime compatibility checks
    ├── server.py           # prepare episode and mount shared routes
    └── ...                 # native geometry, controllers and kinematics
```

RoboDojo and RoboUse keep native adapters in `environment/adapter.py`.
Shared tools and HTTP protocols stay in `tools/` and
`services/`.

### Prepared episode composition

Declare ordered tool modules on `IntegrationSpec.tool_profiles` and reuse
`PreparedEpisodeSession(context, binding_factory=...)` for URL-backed episodes.
The selected `context.tool_modules` create providers against `session.episode`
and register auxiliary clients with `session.own()`. The session closes them even if later
setup fails, then releases its environment client. The outer session owns
`IntegrationContext.artifacts`; integrations use that store without closing it
or creating their own image copies.

Observation providers return `ObservationData.image_paths` pointing to ready
files in the shared store. Publish image bytes through the store at the provider
boundary, then pass `local_path(artifact_id)` to `ObservationHub`. Runtime paths are absolute;
persisted observation references are relative to the session directory.

RoboUse and RoboDojo reuse `integrations.gripper.create_session`; RoboDojo also
supplies a task-requirements loader. Another tool family can use a different callback
without adding action-name dispatch to the runtime or duplicating episode state.

See [architecture](architecture.md) for module ownership and
[services](services.md#add-an-actor-backend) for backend extensions.
