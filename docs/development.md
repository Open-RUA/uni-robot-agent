# Development boundaries and extension guide

Read this alongside the [architecture diagram](architecture.md) and
[repository instructions](../AGENTS.md). This guide describes where to implement
behavior; [Contributing](../CONTRIBUTING.md) covers installation and checks.

## Design philosophy

Use the paper's four [design principles](architecture.md#design-principles):
**Unified** tool semantics and feedback across environments; **Efficient**
overlap of reasoning and execution with explicit resource ordering; **Realistic**
interaction using deployable observations without privileged simulator inputs;
and **Modular** components connected by explicit interfaces. Document optional
capabilities and defaults separately from the paper's evaluation settings.
In particular, official task-verifier outcomes belong in evaluation records,
outside model-visible tool feedback.

Components communicate through explicit contracts so each can evolve without
reimplementing the others. Planner SDKs, model APIs, tool semantics, scheduling,
native environments and presentation have different owners. Put behavior with
its owner; extend a shared contract when a new use case actually needs it.

Existing components, including gripper tools, are open to modification. You can
improve an existing tool, add one to its family, or create a new family under
`tools/`. The environment supplies native execution and observations and selects
compatible providers. It does not define the model-facing tool implementation.

Share code when its semantics match. New tool families can reuse scene tools
and the prepared episode lifecycle. Keep embodiment-specific behavior in its
own family rather than forcing it into the scalar-gripper interface.

## Directory responsibilities

Paths below are relative to `uni_robot_agent/` unless noted otherwise.

| Path | Owns | Keep out |
|---|---|---|
| `cli.py`, `config.py` | Argument resolution, validation, process setup | Native robot control and SDK tool handlers |
| `session.py` | Top-level run lifecycle, assembly, finalization | Branches implementing individual tools or native robots |
| `agents/` | Planner SDK configuration, lifecycle, tool delivery, SDK events/records | Robot semantics and environment control |
| `provider/` | Model gateway, upstream codecs, model request retries | Physical action retries or tool execution |
| `context/` | Prompt/history assembly and context policies | Hardware queries and hidden task controllers |
| `tools/<family>/` | Existing or new tool sets: schemas, handlers, orchestration, feedback, resource policies | Concrete environment imports and native device connections |
| `tools/gripper/` | Direct scalar-gripper motion and companion arm orchestration | Actor collaboration, dexterous joint semantics and native SDK imports |
| `tools/hybrid/` | Actor/reviewer orchestration, generic bounded execution; explicitly selected domain adapters | Native drivers, concrete integration imports and model wire conventions |
| `tools/scene/` | Shared state viewing, snapshot capture, localization | Per-robot camera endpoints and device calibration procedures |
| `tools/skills.py`, `tools/episode.py`, `tools/presentation.py` | Runtime guidance, finish request, model-visible presentation | Native success evaluators or developer workflow instructions |
| `execution/` | Scheduling, resource exclusion, jobs and execution lifecycle | Environment-name or SDK-name dispatch |
| `integrations/base.py`, `integrations/prepared_episode.py` | Integration contracts, one acknowledged episode projection, owned connections | A separate state cache per provider |
| `integrations/gripper.py` | Compose shared providers and bindings for a tool family | Tool schemas or hardware connections |
| `integrations/<environment>/` | Registration/config, environment context, native backend, preparation, controllers, resource/model bindings | Model tool definitions or a copied service/executor |
| `services/environment/` | Typed command/state contracts, client, routes, sequencing, receipts, snapshots, interruption | High-level tool programs or an endpoint for every tool |
| `services/actor/`, `services/perception/` | Independent model services and their backends | Environment episode ownership |
| `services/mcp/`, shared HTTP utilities | SDK transport bridge, result encoding, connection/host mechanics | Tool semantics and duplicate dispatch |
| `observation/` | Observation coordination and freshness/insight policies | A second authoritative environment state |
| `artifacts/` | Shared artifact storage, persistence, export | Robot-specific or provider-specific decisions |
| `events/` | Shared event vocabulary and publication/subscription | SDK-specific parsing and robot control |
| `telemetry/` | Traces, logs, diagnostics and their persistence | Decisions that change task execution |
| `ui/` | Rendering, interaction and presentation projections | Native robot execution or authoritative episode state |
| `resources/prompts/`, `resources/skills/` | Packaged robot-facing guidance; general and environment scopes | Coding-agent instructions and deployment secrets |
| Repository `scripts/`, `examples/` | Launchers, diagnostics, reproducible usage | The only implementation of reusable production behavior |
| Repository `docs/` and future public tests | Maintained explanation and focused regression evidence | Claims that fixture success establishes physical success |

Planner adapters live in `agents/`; follow [the agent contract](agents.md).
Dependencies flow through contracts and supplied bindings. Integration assembly
may import shared providers; shared providers must not import a concrete robot.

## How the components fit together

The main control path is:

```text
CLI/config -> session assembly -> planner SDK adapter
                                   -> executor -> tool provider
                                                    -> typed service client
                                                    -> service -> native backend
```

Alongside that path, `provider/` connects the planner to a model API, `context/`
prepares model context, and `observation/` coordinates observation work. Events
connect execution to presentation and recording; the artifact store gives those
consumers consistent file identities. These components cooperate through their
interfaces rather than taking over one another's decisions.

| If you want to change… | Start here | Boundary to preserve |
|---|---|---|
| Run configuration or lifecycle | CLI/config and session assembly | Delegate SDK setup, tool behavior and native startup to their owners |
| Planner SDK support | `agents/` | Translate SDK behavior into shared execution/events, rather than scheduling or executing tools twice |
| Model vendor support | `provider/` | Convert upstream request/response formats; do not change robot behavior to suit a vendor |
| Prompt or history selection | `context/` and relevant prompt builders/resources | Consume supplied state and tool descriptions; do not query hardware from context assembly |
| Agent-visible robot operations | `tools/` | Define the operation here and use typed clients/bindings for native work |
| Concurrency or background jobs | `execution/` | Enforce declared resource policies without interpreting SDK/environment names |
| A physical or simulated environment | `integrations/<environment>/` | Supply native backend/data and compose existing or newly added tool families |
| Remote execution or model inference contract | The owning package in `services/` | Keep typed requests, ownership, admission and cleanup explicit |
| Observation scheduling or freshness | `observation/` | Coordinate observations; capture/calibration still belong to the environment and localization semantics to scene tools |
| Events and diagnostics | `events/`, `telemetry/` | Normalize native SDK events in its adapter; keep observation of execution separate from control |
| Stored images, records or exports | `artifacts/` and domain record producers | Preserve artifact identity and ownership; domain producers supply meaning |
| Console/TUI behavior | `ui/` | Render events/state; send user intent through session APIs instead of calling native hardware |

For example, a new planner need not introduce new robot tools, a new model API
need not introduce a new planner, and a UI redesign need not change scheduling.
A feature spanning layers should change each owning interface deliberately;
these boundaries do not require every contribution to touch only one directory.

## Add an environment to an existing family

1. Read `IntegrationSpec`, `PreparedEpisodeSession`, the selected family's
   composition, and the environment backend contract. Use RoboUse/RoboDojo for
   gripper structure. An example adapter does not establish hardware validation
   for a new device.
2. Prepare native services and publish an accurate environment description:
   device names, frames/units, limits, camera availability, calibration, task
   identity, and supported capabilities. Static validated facts become context;
   unsupported operations remain unavailable.
3. Implement native execution, observation and outcome handling in the
   environment adapter. Start the existing `EnvironmentService` and mount its
   routes. Native servo loops and high-frequency constraints stay with hardware.
4. Supply model/resource bindings where the family needs them: robot geometry,
   joint/control decoding, posture resources or storage. Bindings return domain
   data/results, not tool providers or arbitrary tool dispatch functions.
5. Declare the supported tool profiles and reuse the family composition and prepared
   lifecycle. Register additional clients with the session's ownership mechanism;
   use its artifact store. Do not clone the episode or close outer-owned stores.
6. Add focused tests and deployment instructions. When migrating an existing
   implementation, explain which features are retained, replaced or removed.

Adding an environment should principally change its integration, registration,
tests and documentation. If it reveals a missing shared capability, implement
that capability in the owning layer explicitly, rather than hiding a private
toolset inside the environment. Do not require unrelated backends to implement it.

## Declare tool compositions

An integration declares its profiles explicitly:

```python
from uni_robot_agent.integrations.gripper import (
    DIRECT_TOOLS, HYBRID_TOOLS, build_gripper_spec,
)

spec = build_gripper_spec(
    name="example",
    create_session=create_session,
    tool_profiles={"direct": DIRECT_TOOLS, "hybrid": HYBRID_TOOLS},
)
```

Declare only profiles whose bindings the integration can supply. A custom profile
is another ordered tuple of modules; it does not require a branch in the shared
session. `IntegrationSpec.default_tool_profile` defaults to `direct` and can be
set explicitly when constructing the spec.

Define a new `ToolModule` with a provider factory and that family's policies.
Factories receive the prepared session, reuse `session.episode`, and register
clients with `session.own()` before any later initialization can fail. Tool
schemas and handlers stay in `tools/`. The module's optional `runtime_prompts`
names refer to packaged runtime files and optional same-named integration notes.
Provider-specific prompt contributions are collected from instantiated providers.

A module may supply `declare_unavailable` for explicit debug ablations. It must
not connect services or grant disabled permissions; unavailable invocations must
report that state. Keep this declaration separate from the executable factory.
The executor owns JobTools and their foreground/background visibility.

Migration from the former assembly API: replace `configure_tools` callbacks and
`IntegrationSpec.eval_tool_names/tool_policies` with profile declarations.
`IntegrationContext.tool_modules` carries the selection; custom sessions expose
a `tool_assembly` built with `assemble_tools`. Native preparation validation can
use `PreparedEpisodeSession(validate_episode=...)`; native outcome handling
remains in the integration's session.

Run `PYTHONPATH=. python scripts/validation/tool_composition.py` for CPU
composition, configuration, visibility and resource-lifecycle checks.

## Add a tool or a capability

Choose the smallest coherent extension:

- Modify an existing tool when you are improving its behavior or contract.
  Changing `tools/gripper/` is a normal contribution; describe the effect on its
  callers and update tests and documentation.
- Add a tool to an existing family when its purpose, inputs and dependencies
  fit that family. Perception tools can evolve too.
- Create `tools/<family>/` when a distinct set of operations needs its own
  provider and semantics. Define schemas, handlers, feedback and resource policies
  there, then compose the provider through integration assembly. A new tool set
  does not need multiple production environments before it can be introduced.

Use typed environment commands for physical work. Pure geometry or library
work can use CPU bindings without a physical service command. Supply native
models/resources through bindings; do not move tool dispatch into those bindings.
Use existing service contracts where they fit and extend them when needed.

Registration and schemas should reflect supported capabilities. Do not advertise
an operation that the selected environment cannot implement, or fabricate scalar
grippers, step counters or success evaluators to force it into another contract.
For an additive capability, keep unrelated configurations unchanged. For an
intentional API change, update the affected clients/backends together and document
compatibility or migration instead of preserving obsolete paths indefinitely.

Native diagnostic endpoints and CLIs may remain inside the integration. They
are not automatically planner tools. Expose agent capabilities through shared
tool providers and keep native device diagnostics in the integration.

## Cameras, preparation, and runtime observations

The existing scene flow is `capture_scene` followed by `locate(snapshot_id=...)`.
Capture freezes observations from running streams; it does not commission or
calibrate cameras. View/state tools expose the recorded observation/state.

The environment adapter prepares RGB, depth, distortion handling, intrinsics,
extrinsics, units and timing so shared localization can interpret each snapshot.
An RGB-only view can coexist with a paired RGB-D view. Equal image dimensions or
intrinsics do not prove pixel alignment, calibration, or temporal pairing. Do not
take an RGB pixel from one snapshot and sample a fresh, unpaired depth frame.

Preparation records stable setup and validates available calibration. Runtime
checks still reject stale, mismatched or invalid observations; changed camera
profiles/calibration require re-preparation. A prompt statement does not replace
these checks. Missing depth may disable depth localization while calibrated
multi-view triangulation remains available according to the advertised contract.

## Lifecycle and physical execution

The shared episode projection follows acknowledged service state. Commands carry
identity and expected state version; receipts settle accepted or uncertain work.
Model-request retries in `provider/` do not authorize physical-command retries.
After a lost action acknowledgement, reconcile rather than resend the action.

Keep physical ordering/resource exclusion, snapshot identity, finalization and
ownership explicit. Interruption must use the independent interruption path,
not wait behind the movement it is meant to stop. A software hold/cancel is not
a hardware emergency stop. Preserve native limits and authorization semantics.
Task success comes from the environment or external assessment, not tool success.

## Add a planner

Implement the plugin under `agents/<name>/` and register its lazy import in
`agents/registry.py`. Use the shared executor, model runtime contract, events and
artifact lifecycle. Add an upstream adapter under `provider/` only when a new
wire conversion is actually needed. Do not add agent-name branches to robot tools,
environment adapters, or the shared session to reproduce SDK-specific behavior.
See [agent integration](agents.md) for the methods and validation requirements.

## Extend the other components

For a model vendor, implement its backend and registry entry in `provider/`
following [Model providers](providers.md)
and test request conversion, response delivery, error handling and recording.
Provider retries concern model exchanges, not physical actions. A new planner
using an existing protocol should reuse that provider path.

For a new hybrid embodiment, follow [the binding contract](hybrid.md). Reuse
`integrations.hybrid.configure_hybrid` and inject the integration's binding factory;
no central binding registry is needed.

For an action or perception model, add a backend to the corresponding service
and declare its input/output profile. Put environment-specific conversions in
the integration's binding. Keep native model loading in its service process;
see [service extension examples](services.md).

For scheduling changes, extend `execution/` against declared tool policies and
test resource ownership, cancellation and job completion. Tools declare what
resources they need; the executor decides when work can run. Service/native
layers still enforce their own execution and lifecycle constraints. A physical
tool family that opts into [replan after failure](architecture.md#replan-after-failure)
reports `ToolOutput.action` from its own feedback, so the executor never reads
tool-specific result fields or tool names.

For observation/context work, distinguish obtaining observations from choosing
what context the model sees. Observation providers own sampling; the observation
hub coordinates versions, concurrency and persistence, not sampling frequency
or native environment ownership. Apply prompt/history policies in `context/`;
use acknowledged episode state and artifact references rather than maintaining
a second authority.

For events, persistence or UI work, emit explicit identities and lifecycle data
at the source, then project them for consumers. Do not infer operation identity
from timing or screen order. Keep persistent artifact ownership in the store
and execution decisions in the runtime. UI actions may request work through
session interfaces; rendering itself does not execute a robot command.

## Validation and review

Add focused tests for new behavior, with explicit fixtures and optional native
runtime dependencies. Keep installation and package-content checks runnable
without simulator packages; see [release validation](packaging.md).

| Change | Evidence to include |
|---|---|
| Layering/import changes | Inspect dependency direction; import public modules without native runtimes |
| Environment protocol/lifecycle | CPU HTTP fixtures for receipts, stale state, uncertainty, shutdown and optional-capability absence |
| Shared tool/perception change | Focused checks of schemas, prompts, feedback, budgets and ordering |
| Planner/provider work | Session/transport checks for dispatch, cancellation and delivery |
| Packaging/dependencies/resources | Build/install checks from [Packaging](packaging.md) |
| Documentation only | Verify paths, links, commands and correspondence with actual contracts |

Scale validation to the affected behavior and callers. A behavior-preserving
refactor should compare existing contracts; an intentional behavior change should
test the new contract and describe its compatibility implications. Neither case
requires freezing existing tools or running a full benchmark for every edit.

Use the installed runtime that supports the changed subsystem. Report passes,
new failures, existing failures and skips separately. Compare affected
Direct/Hybrid and optional-capability configurations when changing shared paths.
Schema/prompt comparisons complement behavior tests; unchanged source in an
integration alone is insufficient. State hardware validation separately.

Review for behavior placed in the wrong layer: integration-local model tools,
tool-name dispatch in environment services, native robot SDKs in shared tools,
planner-specific scheduling, UI-owned physical execution, or duplicated state
authority. Static setup usually belongs in metadata; a new inspection tool is
appropriate when inspecting it is itself an agent capability, implemented in
`tools/`. Review the abstraction and contract rather than imposing a fixed tool list.

## Developer instructions versus robot prompts

`AGENTS.md` is the canonical coding-agent entry. `CLAUDE.md` imports it using
`@AGENTS.md` for Claude Code compatibility; keep one set of rules. Detailed docs
are read for the affected task. Avoid copying the entire architecture into each
entry or maintaining parallel per-agent versions.

These files are not packaged runtime skills or robot task prompts. Coding SDKs
may discover repository instructions from their working directory, so adding an
instruction file is not proof that a robot SDK's effective context is unchanged.
Verify instruction discovery when running experiments from this checkout. Do not
alter SDK defaults or runtime prompts as an incidental documentation change.

### SDK validation

Use `scripts/validation/sdk_release.py` to check an installed wheel outside the
checkout. Integration Python options extend `IntegrationOptions` and are
registered on `IntegrationSpec.config_model`; retain parity with CLI defaults and
use the existing integration validator. Session callbacks are output consumers,
not robot controllers. The release check in `scripts/validation/sdk_release.py`
installs a wheel with dependencies and checks public imports, offline configuration
and packaged resources; it does not establish HTTP execution, GPU, hardware or
provider availability.
