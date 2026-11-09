# Architecture

uni-robot-agent is a harness for robot-use agents, with Codex and Claude Code
runtimes. The agent selects operations from task instructions and observations;
the harness schedules tools, executes them and returns feedback. Environment,
action-model and perception services run independently, with their native dependencies.
The Python package is `uni_robot_agent`; the CLI is `uni-robot-agent`.

![uni-robot-agent architecture: agent backends, shared robot tools, independent services and native adapters](../assets/diagrams/uni-robot-agent-architecture.png)

The diagram shows the shared tool and service implementation, with RoboUse and
RoboDojo as simulator examples. `actor_act` is available in Hybrid
mode; `write_skill` requires `--skill-writing`. Agent collaboration, where
available, is supplied by the selected agent SDK.

## Design principles

The paper organizes the harness around **Unified**, **Efficient**, **Realistic**
and **Modular** interaction.

### Unified

Shared tools give operations and feedback a common meaning across environments,
including arguments, units and result formats. Environment adapters translate
native observations and commands, so adding an environment does not require
reimplementing every tool. Consistent interaction records also support combining
experience across simulation and physical robots for subsequent training.

### Efficient

A two-stage pipeline overlaps agent reasoning with tool execution. Background
calls return job handles so the agent can continue independent work; exclusive
calls sharing a resource execute in submission order. Reasoning can therefore
overlap sequential robot actions. Result-dependent decisions still wait for
completed feedback. Model batching and dispatch concurrency are separate
controls; see [concurrency controls](#concurrency-controls).

### Realistic

The paper's setting uses information available in physical deployments: RGB
images, robot proprioception, camera calibration, robot descriptions and task
instructions. Agents and tools operate without simulator-provided depth or
task-verifier feedback. Calibrated multi-view observations support spatial
grounding, while action feedback and images let the agent assess outcomes.
Official task outcomes remain available for evaluation and saved results;
`finish` returns a submission acknowledgement to the agent, not the verdict.

The software also supports depth-enabled configurations, including paired RGB-D
sensors on physical robots. RoboDojo captures simulator depth by default, so
the paper's RGB-only setting must be selected explicitly. See
[observation settings](integrations.md#realistic-observation-settings) for the
configuration and capability boundaries.

### Modular

Explicit interfaces separate agent runtimes, scheduling, tool behavior, services
and native backends. Researchers can replace runtimes, tools, environment
backends, action models, perception models and context policies through their
own contracts. The five logical layers map to the implementation as follows:

| Layer | Responsibility | Main implementation |
|---|---|---|
| Agent | Model interaction and context | `agents/`, `provider/`, `context/` |
| Orchestration | Scheduling, dispatch and background jobs | `execution/` and SDK tool adapters |
| Tool | Shared robot operations and feedback | `tools/` |
| Service | Domain contracts and runtime resource management | `services/` |
| Backend | Concrete simulator, robot and model implementations | Native integration adapters and service model backends |

`session.py` assembles these components and owns the run lifecycle. The module
map below gives the detailed boundaries. Reliable interaction is supported by
immutable observations, receipt reconciliation, coordinated arm/gripper control,
progress-based motion stopping, actionable feedback and visual context management.

## Execution flow

1. The CLI resolves and validates configuration. `--print-config` and
   `--validate-config` exit before opening services or creating run output.
2. Configuration resolves an integration's declared tool profile. `session.py`
   passes its modules to `IntegrationSession`, registers the assembled tools and
   policies, and builds the prompt from their contributions.
3. Codex or Claude Code invokes tools through its MCP adapter and the executor.
   Each provider owns its schemas and handlers; `Toolkit` registers them.
4. Tools read one shared `Episode` projection. Motion tools compose primitive
   HTTP steps; scene tools work with immutable observations and calibration.
5. A `run` drains outstanding execution, finalizes the episode, writes its
   outcome and closes resources. Event subscribers drive console/TUI output
   and the JSONL trajectory.

Tool-call success, agent completion and task success are separate. The `finish`
tool submits a termination request without revealing the environment's verdict
to the agent. `EpisodeOutcome` records official success, acceptance, retryability
and failure category.
See [result inspection](getting-started.md#inspect-the-result).

## Module map

Package paths are relative to [`uni_robot_agent/`](../uni_robot_agent/).

| Area | Responsibility |
|---|---|
| [`cli.py`](../uni_robot_agent/cli.py), [`config.py`](../uni_robot_agent/config.py) | CLI, process supervision, validation and redaction |
| [`session.py`](../uni_robot_agent/session.py) | Session assembly, task/interactive lifecycles and final outcomes |
| [`agents/`](../uni_robot_agent/agents/) | Codex and Claude Agent SDK adapters |
| [`tools/`](../uni_robot_agent/tools/) | Schemas, handlers, structured outputs and tool orchestration |
| [`execution/`](../uni_robot_agent/execution/) | Serial execution, background jobs and resource scheduling |
| [`integrations/prepared_episode.py`](../uni_robot_agent/integrations/prepared_episode.py) | Shared episode state, connection ownership and receipt reconciliation |
| [`integrations/gripper.py`](../uni_robot_agent/integrations/gripper.py) | Gripper tool composition, action bindings and execution policies |
| [`integrations/`](../uni_robot_agent/integrations/) | Environment adapters, prompts, configuration and model bindings |
| [`services/`](../uni_robot_agent/services/) | HTTP transport/hosting, environment, actor, perception and MCP adapters |
| [`provider/`](../uni_robot_agent/provider/), [`context/`](../uni_robot_agent/context/) | Model gateway, provider adapters, prompts and history policies |
| [`observation/`](../uni_robot_agent/observation/) | Optional observation snapshots, freshness and observer insights |
| [`artifacts/`](../uni_robot_agent/artifacts/), [`telemetry/`](../uni_robot_agent/telemetry/) | Run persistence, SFT export, tracing and logging |
| [`events/`](../uni_robot_agent/events/), [`ui/`](../uni_robot_agent/ui/) | Shared events, console rendering and Textual interface |
| [`resources/`](../uni_robot_agent/resources/) | Packaged prompts and environment skills |

## Shared tools and episode ownership

`PreparedEpisodeSession` owns connections and creates one `Episode` shared by
independently composed providers:

| Provider | Public tools |
|---|---|
| `GripperTools` | `move`, `grasp`, `release`, `hold` |
| `HybridTools` | `actor_act`, when an actor and compatible binding are configured |
| `SceneTools` | `view_env_state`, `capture_scene`, `locate` |
| `SkillTools` | `list_skills` discovers shared general/ and current-environment Markdown; `read_skill` reads either scope; optional `write_skill` writes only environment-local experience |
| `EpisodeTools` | `finish` |

Arm and camera names come from the environment description. Motion uses arm
poses and scalar gripper opening. RoboUse and RoboDojo reuse these providers
with their own environment adapters and action bindings. Native environments
provide descriptors, models/resources and backends, never their own tool definitions.

Both `run` and `tui` require an explicit environment `--integration`, such as
`robouse` or `robodojo`. The integration composes compatible tools, validates
options and selects environment-specific context. `integrations/gripper.py` is
shared composition code, not a selectable integration.

Physical actions, `capture_scene` and `finish` share the exclusive `episode`
resource queue. `locate` operates on immutable historical snapshots and can
overlap motion; the result is evidence about that snapshot, not a guarantee of
an object's current position. [`HybridTools`](../uni_robot_agent/tools/hybrid/provider.py)
owns actor collaboration independently of scalar-gripper tools. Its runner captures,
calls the actor service, requests a review and executes a bounded prefix through
[`execution.py`](../uni_robot_agent/tools/hybrid/execution.py). Every acknowledged
primitive immediately updates the existing episode; later failures retain that
confirmed progress. Sample completion, acknowledged commands, elapsed action
duration and partial or unknown execution are separate evidence.

An integration binding encodes already captured observations, validates the actor
profile, interprets predictions, assembles optional capabilities and compiles rows.
It owns neither inference nor an execution loop. The optional shared
`tools/hybrid/adapters/arm_gripper.py` component handles arm scope, opening and
command conversion. `tools/hybrid/corrections/cartesian.py` handles bounded pose
geometry. The core imports neither component nor a concrete integration.
Native FK, IK, calibration, limits and physical stepping remain in environment
adapters. See [the extension contract](hybrid.md).

### Explicit tool composition

`IntegrationSpec.tool_profiles` maps names to ordered `ToolModule` tuples;
`default_tool_profile` selects the default. Each module owns a provider factory,
execution policies and optional runtime prompt resource names. The prepared
session assembles the selected modules against one Episode and owns their clients.
`ToolAssembly` supplies providers, policies and prompt resources to the session.
It replaces the fixed provider tuple and the separate evaluation-name whitelist.

Each of the eight environment entry points declares its supported profiles.
Shared `DIRECT_TOOLS`, `HYBRID_TOOLS` and `MINIMAL_TOOLS` definitions avoid copying
tool lists. Minimal composes a selected `GripperTools` subset (`move`, `hold`)
with `EpisodeTools` (`finish`). It starts with a persisted multimodal observation;
see [minimal tool behavior and calibration](integrations.md#minimal-robot-tools).
Hybrid is instantiated only when selected; environment bindings still own native
semantic conversion. Duplicate module names and overlapping policy names fail
validation, and Toolkit continues to reject duplicate tool names.

Job controls remain owned by the executor, with visibility determined by return
mode unless debugging supplies an explicit selection. Debug-only declarations
can expose unavailable tools without opening their services. Prompt resources
follow the configured composition before debug visibility filtering, preserving
the existing ablation contract. Workspace and Observation retain their existing
session-level assembly paths.

## Service boundaries

```text
Domain client → HttpTransport → typed FastAPI routes → domain service → native backend
                                Uvicorn host
```

The environment service owns native state, snapshots, execution receipts and
one primitive step at a time for the scalar-gripper protocol through `/steps`.
Opt-in device episodes add typed bounded `/device-steps` and an independent
`/interrupts` channel; native real-time servo loops remain beside the hardware.
`tools/gripper/motion.py` composes these steps into higher-level programs and
checks stopping conditions. There are no server-side move, grasp or multi-action
prefix routes.

Actor and perception services each own a bounded `BatchWorker`. `Pi05Backend`
uses the OpenPI/JAX runtime; `Sam3Backend` supplies image
segmentation. Action bindings translate environment snapshots and model action
chunks. Service configuration identifies checkpoints and compatible profiles.

The Codex MCP HTTP adapter and Claude in-process SDK adapter share tool-result
encoding. The model gateway reuses the HTTP transport and host, while retaining
its own provider adapters, retries and trace handling. A lost physical or actor
response is reconciled through its receipt, never automatically replayed.
See [services](services.md) for deployment and extension contracts.

<a id="three-concurrency-controls"></a>

## Concurrency controls

| Setting | Controls |
|---|---|
| `--model-parallel-tool-calls` | Whether the provider gateway permits multiple tool calls per model response. Defaults to true; explicit use requires a provider URL. Native SDK behavior is unchanged without the gateway. |
| `--mcp-parallel-dispatch` | Whether Codex dispatches calls concurrently to the MCP server. Defaults to false. |
| `--tool-return-mode` | `background` by default; `immediate` returns completed results and hides job controls. Both allow model batches. |
| `--resource-scheduling` | `per-resource` by default; `global-serial` maps all calls onto one exclusive FIFO resource. |

Both switches accept `--no-` forms. MCP parallel dispatch is Codex-only; Claude
maps disabled model parallelism to Messages `disable_parallel_tool_use`.
Concurrent MCP dispatch does not establish model-output order at the executor boundary.

Each `ToolExecutionPolicy` selects a return mode (`immediate` or `background`)
and placement (`exclusive` or `parallel`). Exclusive jobs enter a FIFO resource
queue in submission order. A background reply may acknowledge enqueueing,
rather than completed motion. Use `wait_for_tool`, `wait_for_tools` or
`get_tool_result` to inspect completion. Cancelling queued work does not
interrupt a running physical action; `cancel_tool` and `flush_queue` return a
receipt for each cancelled job.

### Replan after failure

A policy can opt a resource into replanning after a failed physical action. It
is off unless declared, and environments that declare nothing keep the
scheduling above. The model queues `move` (to above the cup), `move` (down) and
`grasp`, and the first move stops halfway:

| | Without the policy | With `replan_on_failure` |
|---|---|---|
| Queued `move` and `grasp` | Run from a pose the plan did not expect | Cancelled before dispatch, each with a receipt (`replan_after_failure`) |
| A new action | Accepted | Refused with `replan_required` and the next step |
| Ending the episode | Allowed | Allowed (`ends_episode`) |

The resource accepts world-mutating work again once three things hold: the
failed result has been read, the environment confirmed that the actuators
stopped, and a `replan_observation` tool (scene `capture_scene`, on any
resource) that started after both returned a new `snapshot_id`. With unknown execution or an
unconfirmed stop it stays refused for the episode; recovery belongs to the
operator. Nothing is retried or replayed.

| Policy field | Meaning |
|---|---|
| `require_result_read` | In background mode, a call to a tool with this flag is refused while the last such call on its resource is unread |
| `replan_on_failure` | An incomplete result cancels queued world-mutating work and requires a replan |
| `replan_observation` | A successful result with a new `snapshot_id` can clear the requirement |
| `ends_episode` | Stays allowed while a replan is required |

Tools report what an action did in `ToolOutput.action` (`ActionOutcome`:
`complete`, `executed` = `none`/`some`/`unknown`, `stopped`), which is never
sent to the model. A failed result without it counts as unknown execution
unless it carries `no_execution: true`. Gripper tools fill it from motion
feedback, and count a program that failed after acknowledged steps as executed;
a stepped simulation counts as stopped after a known result, while a device
environment, or one that declares `physical_hardware`, must confirm the stop
itself. Admission applies to `call` and
`submit` alike.

Without these policies scheduling is unchanged. Job controls change for every
environment: `flush_queue` also lists each cancelled job, and a cancelled job's
result carries `no_execution`, `execution_status` and `reason`.

Coordinated arm targets are a separate per-tool capability; see
[RoboDojo](integrations.md#coordinated-arm-targets).

## Prompts and observations

Environment prompt builders supply the effective instruction, measured state,
calibration and task context. Providers contribute their tool guidance. The
shared assembler adds runtime, parallel, Hybrid, optional skill-writing and
observer sections when enabled, then saves `effective_prompt.md`. Registered
prompt sources preserve their assembly order.

`--instruction` overrides the prepared episode's instruction for both agent and
action model. All integrations can discover the shared general/ skill library
and their own environment namespace. RoboDojo additionally provides X5 geometry,
observation, motion and return supplements. Content is read on demand, not
preloaded in full. `list_skills()` takes no arguments and returns searched scopes
in `paths` and sorted recursive Markdown paths in `entries`. `read_skill(path=...)`
reads a Markdown file and returns content and SHA256. Discovery enumerates
current files, including additions not yet in a curated index. Missing directories
are not created; a missing environment directory does not hide general skills.
By default these tools read the packaged library. `UNI_ROBOT_AGENT_SKILLS_DIR`
selects one complete external skills root for listing, reading, and writing;
there is no overlay or fallback. `--skill-writing` requires that explicit
external root and permits Markdown writes within the episode namespace only.
Shared general/ skills stay read-only to the agent.
There is no special `memory/` directory. See [skills setup](running.md#skills).
This is an explicit local skill library, not automatic model training.

The default observation profile is `none`. Selecting a built-in profile
(for example `robouse` or `robodojo`) requires parallel execution and a
matching `--integration`. Profiles add snapshot and insight tools; freshness
and state versions describe evidence, not task completion.

Codex can use the identity or sliding-window context policy. History transforms
require an explicit provider URL. Claude Code uses its SDK's native model
configuration and currently supports the identity context policy.

## Extending the agent

- **Integration:** implement the [integration contract](integrations.md#integration-contract),
  including lifecycle handling, scheduling and official outcomes.
- **Agent:** implement the [`Agent` contract](../uni_robot_agent/agents/contracts.py), then
  register it with [`register_agent`](../uni_robot_agent/agents/registry.py). The adapter owns SDK events, cancellation
  and cleanup; the session owns shared robot resources.
- **Tool:** expose `ToolDef` objects through a `ToolProvider`, return structured
  `ToolOutput` and declare resource policies. Final schemas are saved in
  `tools.json` after assembly.
- **Service backend:** implement the domain's backend contract; reuse its
  client, routes and receipts. See [the actor example](services.md#add-an-actor-backend).
- **Prompt or history policy:** use [`context/`](../uni_robot_agent/context/) registries
  without coupling them to a transport.

See [CONTRIBUTING.md](../CONTRIBUTING.md) for development setup and validation.

## Tool definitions and names

uni-robot-agent tools use `ToolSpec` for explicit descriptions and JSON schemas,
then `bind_tools` binds those declarations to handlers. Fixed and
environment-dependent schemas follow this same path. `ToolDef` is the bound
runtime entry, not a second declaration API. Argument validation runs before
the handler; missing or unknown fields are not silently repaired.

Tool providers define local names that are unique within the harness toolkit.
The SDK's declarations determine the callable names visible in each session.
`passthrough.responses` preserves those declarations, names, namespaces and structured
history. Profiles that require flat tool lists encode each declared namespace
and local name into a distinct wire name, then restore the original route in
responses. Distinct namespaces may contain the same local name; conflicting
declarations or wire names fail before forwarding.

`volcengine.responses` maps native search to a function. Kimi's native search requires
`$web_search` on its wire; its training records use `web_search`. Qwen uses
ordinary function tools. Other training tool names retain the adapter's
declared names, including any flattened namespace.
Codex configured without a provider gateway and the Claude Code SDK retain
their native tool naming. MCP permissions and routing use those SDK names.
Prompts, skills and tool descriptions use local labels; actual calls use the
exact declared name, including its namespace or server prefix.


### Registered grasp-site boundary (environment protocol 4)

Each integration registers one grasp site per arm in `EnvironmentDescription.arms`.
`control_frame` names it; `grasp_site_description` explains the point, axes and opening
semantics. `ArmState.control_pose`, `ArmCommand.pose`, Direct arrival feedback and
Hybrid pose previews all refer to this same site in world metres / wxyz.
`move` accepts [x,y,z,roll,pitch,yaw] with fixed-axis XYZ radians and converts
to that internal quaternion representation. Retaining XYZ requests an in-place turn. There is no model-selectable wrist/center reference.
`rotation_radius_m` preserves conservative waypoint displacement budgets even when
the public site is displaced from a native controller point.

The shared motion tools do not load URDF or solve IK. Robot-model loading and native
conversion stay inside the integration: X5 URDF plus fixed-site extension for
RoboDojo FK/DLS and native grasp-site/OSC conversion in RoboUse.
Actor joint/native action vectors and camera extrinsics retain their native
conventions. Environment protocol 4 / tool schema 8 requires coordinated client/service upgrades.
Native task grading remains in its original frame, including finite return tolerances.

Provider profiles, protocol conversions, and vendor implementation ownership are
described in [Model providers](providers.md).

### Local service deployment

`services/deployment/` owns versioned runtime records, explicit preparation,
process groups and structured readiness. Each integration or model service
provides a lightweight `deployment.py` adapter and packaged recipe; native
libraries remain in the selected service interpreter. `setup` prepares resources,
`deploy` owns a foreground service, and `run --deploy local` owns an environment
plus a separately supervised agent process. External URLs grant no process ownership.
Signals and Linux orphan reaping are installed by the CLI, not by the reusable
`Deployment` handle or preparation function. See [deployment](deployment.md).

### Public Python entry points

`SessionConfig` and `AgentConfig` resolve into the same runtime settings as CLI.
`Session` owns assembly, subscriptions, executor draining and finalization; CLI
retains parsing, signal/process supervision and exit codes. Integration option
schemas live on `IntegrationSpec.config_model`; core SDK code has no concrete
environment branches. `ToolClient` uses `ToolExecutor`, including its background
job registry. `RunResult` views the existing result record, and `RunRecord` reads
it without starting runtimes. See [SDK](sdk.md).

### Read-only Web presentation

`ui/web/` serves one live run or a collection of recorded run directories. CLI owns
its host lifecycle; a managed deployment parent starts exactly one monitor and
passes ownership information to its agent child process. SDK `Session` does not own
Web servers. Static ES modules and CSS ship in the wheel.

`ToolExecution` records queue/start/finish/cancellation facts separately from SDK
acknowledgements. Execution context carries transport-neutral call and batch IDs
across worker threads. Codex MCP responses carry an opaque call receipt in `_meta`,
which the pinned SDK event adapter reads; Claude propagates its existing tool-use
identity. `ModelRequest` records gateway request wall time. Event JSONL preserves
canonical source metadata and IDs. These additions do not change tool schemas,
model-visible results, resource policy or physical cancellation behavior.

The monitor incrementally reads artifacts, projects them into display state and
serves idempotent SSE state updates. Slow browser clients never execute network
work in synchronous EventBus callbacks. A separate sampler only reads published
environment snapshots; it does not reset, capture, step or finish an environment.
Replay uses the same projection with no environment/model connection. Image routes
are restricted to referenced raster files inside the selected run directory.
