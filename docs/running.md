# Configuration and running the agent

For resource preparation and owned local services, see [Local deployment](deployment.md).

For uni-robot-agent installation and a first episode, see [Getting started](getting-started.md).
The installed CLI works outside the checkout; repository scripts below assume
its root as the working directory. Launch the CLI as `uni-robot-agent`,
`python -m uni_robot_agent`, or `python -m uni_robot_agent.cli`.

| Command | Purpose |
|---|---|
| `setup` | Install/cache a runtime and resources, or adopt an existing installation |
| `deploy env`, `deploy actor`, `deploy sam3` | Start one prepared service in the foreground |
| `run --deploy local` | Start a prepared environment and run the agent on one task |
| `run --env-url URL` | Run the agent in an independently prepared episode |
| `tui --env-url URL` | Open the interactive terminal for an existing episode |
| `monitor --run-dir PATH [PATH ...]` | View saved directories, with cards for multiple runs |
| `doctor` | Diagnose the installation and configuration offline |

Add `--monitor` to `run` for a live Web console. TUI does not accept local
deployment or monitor flags. The [Python SDK](sdk.md) exposes the same session
and service lifecycle without invoking CLI process management. General batch
evaluation is not part of these entry points.

## Configuration

Explicit CLI values override the supported environment variables, then declared
defaults. Configuration environment variables use the `UNI_ROBOT_AGENT_` prefix.
Use `--print-config` to inspect or `--validate-config` to validate
offline. Neither mode connects, resets, creates output directories or requires
GPUs. Secret values are excluded from configuration records; URL credentials
and queries are redacted.

| Environment variable | Purpose |
|---|---|
| `UNI_ROBOT_AGENT_API_KEY` | Main provider credential (Codex or Claude); reviewer fallback |
| `UNI_ROBOT_AGENT_VLA_REVIEW_API_KEY` | Independent action-chunk reviewer credential |
| `UNI_ROBOT_AGENT_PROVIDER_URL` | Default main-agent upstream URL |
| `UNI_ROBOT_AGENT_ENV_URL` | Default environment endpoint |
| `UNI_ROBOT_AGENT_ACTION_MODEL_URL` | Default action-model service URL |
| `UNI_ROBOT_AGENT_SAM3_URL` | Default perception endpoint |
| `UNI_ROBOT_AGENT_OUTPUT_DIR` | Default output parent directory |
| `UNI_ROBOT_AGENT_SKILLS_DIR` | Optional complete external skills root; required for skill writing |

Script settings such as `PYTHON`, `SIM_PYTHON`, `ACTION_MODEL_PYTHON`,
`HARNESS_PYTHON`, `ROBOUSE_PYTHON` and `ROBOUSE_WORKER_PYTHON` select interpreters.
Pass native source and checkpoint paths through the service CLI or TOML
configuration. These settings do not configure agent budgets.
Native SDK authentication and simulator-specific environment variables remain
separate from the harness CLI.

Unsupported historical environment variables are not interpreted. In particular, old
`UNI_ROBOT_AGENT_CONTEXT_*`, `UNI_ROBOT_AGENT_EXECUTION_MODE`, `UNI_ROBOT_AGENT_REASONING_EFFORT`,
`UNI_ROBOT_AGENT_TELEMETRY` and `UNI_ROBOT_AGENT_MCP_*` settings do not replace CLI flags.
See the [changelog](../CHANGELOG.md) when updating an earlier experiment script.

## Diagnose an installation

`doctor` checks one installation and environment without resolving a run
configuration. It is offline: it does not connect to services, download assets,
probe credentials or create directories. Use it when an episode cannot start or
when support needs the installed package, dependency and configuration state.

```bash
# Human-readable aligned report and summary
uni-robot-agent doctor

# Machine-readable report for a support ticket
uni-robot-agent doctor --json

# Require one integration and fail when it is unavailable
uni-robot-agent doctor --integration robodojo
```

Checks cover the Python version, the installed `uni-robot-agent` version and
main/runtime package split, required imports, the Codex and Claude Agent SDKs,
the optional `opencode` and `gemini` binaries, provider profiles, the known
integration adapters, the packaged or `UNI_ROBOT_AGENT_SKILLS_DIR` skills root,
output-directory writability and the presence of configuration environment
variables. Environment-variable checks report `set`/`unset` only; values are
never read or printed.

Each check is `ok`, `warn`, `fail` or `skip`; hints explain warnings and
failures. The command exits `0` when no check fails, including when warnings or
skips remain, and `1` when any check fails. A missing optional integration is a
warning by default; naming it with `--integration NAME` makes it a failure.

Binaries are located with `PATH` lookup only. Pass `--check-binaries` to also
run `opencode --version` or `gemini --version` with a short timeout. Use
`--output-dir PATH` to check a different parent directory than
`UNI_ROBOT_AGENT_OUTPUT_DIR` or the default `outputs`.

## Single-episode CLI

Choose an integration to see its complete help:

```bash
uni-robot-agent run --integration robodojo --help
uni-robot-agent run --integration robouse --help
```

`run` exposes tools for one task attempt. `tui` exposes interactive tools
and is included in the standard installation. Both commands require an explicit `--integration` and `--model`.
`--instruction` overrides natural-language instructions for `run`. In URL mode,
optional task, suite, seed and run identity arguments check the prepared episode;
they do not change it. With `run --deploy local`, deployment task/layout options
create the owned environment before the agent connects.

| Option | Default | Purpose |
|---|---|---|
| `--integration` | Required | Explicit environment integration, such as `robodojo` or `robouse` |
| `--agent` | `codex` | `codex` or `claude_code` |
| `--model` | Required | Provider model ID |
| `--max-turns` | `0` | Agent turn limit; `0` disables it |
| `--reasoning-effort` | `max` for Codex; `none` for Claude | Main Codex agent effort; Claude requires `none` |
| `--episode-timeout-s` | `10800` | Agent episode wall time for `run`, excluding managed environment startup; `0` disables it |
| `--tool-return-mode` | `background` | `immediate` waits for final results; `background` exposes job controls |
| `--resource-scheduling` | `per-resource` | Independent resource domains; `global-serial` uses one FIFO queue |
| `--observation-profile` | `none` | Integration observation profile; requires matching integration and per-resource scheduling |
| `--skill-writing` | Disabled | Enable writing Markdown experience within the integration's skill namespace |
| `--env-url` | Environment or required in connection mode | Existing endpoint; conflicts with `--deploy local` |
| `--deploy local` | Disabled | `run` only: own a prepared RoboDojo/RoboUse environment |
| `--monitor` | Disabled | `run` only: start a local Web console |
| `--monitor-port` | `8765` | Live monitor port; `0` selects an available port |
| `--action-model-url` | Environment or unset | Action-model endpoint required for hybrid actions |
| `--sam3-url` / `--no-sam3` | Explicit choice | Configure SAM3 or disable it |
| `--provider-url` | Environment or SDK native provider | Upstream base URL; required for Claude, also accepts `ANTHROPIC_BASE_URL` |
| `--provider-profile` | Protocol-specific passthrough | See the [provider profiles](providers.md#profiles); the selected planner determines input compatibility |
| `--provider-timeout-s` | `600` | Main agent HTTP request timeout |
| `--provider-retries` | `2` | Extra attempts per provider request |
| `--provider-retry-backoff-s` | `5` | Provider retry backoff |
| `--mcp-timeout-s` | `10800` for Codex | Codex timeout waiting for an MCP response |
| `--output-dir` | Environment or `outputs` | Parent of unique run directories |
| `--telemetry` | `standard` | `off`, `standard`, or `debug`; debug also saves raw provider requests |
| `--log-level` | `INFO` | `DEBUG`, `INFO`, `WARNING`, or `ERROR` |

`--no-sam3` overrides an environment-provided SAM3 URL; it conflicts with an
explicit `--sam3-url`.
Claude Code uses the uni-robot-agent gateway with a native Anthropic Messages upstream
and `--provider-profile passthrough.messages`. Provider timeout and retry settings apply to
Claude as well. Select `--claude-effort` for its reasoning configuration; the
Codex reasoning and MCP dispatch flags do not configure Claude. The optional
`--claude-max-budget-usd` sets an SDK cost limit.

Claude exposes its native tools (including file, shell and Agent tools) in
addition to the selected robot MCP tools. Native tools run through the SDK with
automatic hook approval and its sandbox disabled; the MCP `allowed_tools` list
is not a filesystem boundary. Helper and subagent model aliases use `--model`.
Only the main agent may submit the episode's `finish`; subagents return their
findings to their parent. Robot calls still pass through the shared executor.
Hooks wait for confirmed model-response delivery before allowing tools.

Tool ownership comes from SDK hook agent IDs. SDK task-start messages link
subagent text and nested agents to the spawning call. Missing SDK identity is
left unassigned rather than inferred from timing. Raw Messages exchanges remain
session-scoped; pure-text HTTP exchanges do not carry a reliable child identity.

Defaults are Direct actions,
assembled robot system instructions (`--codex-base-prompt robot`), RoboDojo extra
task requirements enabled, and a three-hour run budget with unlimited agent turns.
TUI sessions remain without an episode wall-time limit. Models, provider URLs,
credentials and service endpoints must be supplied by the user or deployment;
`--provider-profile` resolves to `passthrough.responses` for Codex or
`passthrough.messages` for Claude. Select the service-specific profile explicitly,
for example `aliyun.chat_completions` for Qwen on Alibaba Cloud.
No sampling temperature or top-p is imposed.

Tool debugging stays off by default. RoboDojo environment captures include depth
by default. To select the paper's **Realistic** RGB-only input setting, pass
`--no-depth` when starting the environment (`run --deploy local`, `deploy env`
or the native environment server), retaining camera calibration. With
`--env-url`, configure the existing environment service separately.
Depth supports geometry in `locate`; attached visual images remain RGB, so RGB
attachments alone do not establish that depth is disabled. RGB triangulation is
exposed when all configured cameras are RGB-only and at least two calibrated
cameras are available; otherwise localization follows the declared depth and
segmentation capabilities. Official task-verifier outcomes are kept outside
model-visible feedback. See [observation settings](integrations.md#realistic-observation-settings).
The RoboDojo IK joint-delta guard is on by default and can be disabled explicitly
with `--no-ik-joint-delta-guard` at the environment server boundary.

### Codex base instructions

The default `--codex-base-prompt robot` replaces the SDK's built-in coding
instructions with the complete assembled `Prompt.system`: environment
instructions, description, tool guidance and enabled runtime guidance. Only
`Prompt.user` is sent as task input. Robot mode requires a nonempty system prompt;
SDK-added skill discovery remains enabled.

For an ablation, select `--codex-base-prompt default` to keep the SDK's built-in
instructions and flattened input. The selected mode is recorded in `result.json`.
Robot mode archives the exact text in `codex_base_instructions.md` and
`codex_user_input.md` inside the session directory.

### Provider archive storage

Provider exchanges are archived by default (`--save-exchanges`). Codex request
views use `request.{codex,context,provider,training}.json.gz`; response bodies also
use gzip. `--telemetry debug` additionally archives the original `request.json.gz`.

To reduce storage, pass `--no-save-exchanges`. Training inputs are temporarily
written, then released for the exported thread after a complete training export
and clean trace persistence. Failed or partial exports and unexported threads
retain their inputs. `meta.json` (including usage), `report.json`, shared images,
trajectories and environment videos are retained. The archive switch takes
precedence over raw-request recording; it does not change telemetry or clean up
historical experiments. Claude's native Messages records have their own
[archive and export support](sdk.md#events-and-saved-records).

Each session stores shared image files in `images/`. Observations, provider
traces, `trajectory.jsonl` and `train.jsonl` reference paths relative to the
session directory. Different encodings or annotated images remain distinct.

### Tool calls and execution

Four settings control different stages:

| Setting | Default | Controls |
|---|---|---|
| `--model-parallel-tool-calls` | Enabled | Whether the provider gateway allows multiple tool calls per model response |
| `--mcp-parallel-dispatch` | Disabled | Whether Codex dispatches calls concurrently to the uni-robot-agent MCP service |
| `--tool-return-mode` | `background` | Tool result delivery, independent of model batching |
| `--resource-scheduling` | `per-resource` | Resource scheduling, independent of result delivery |

The first two support `--no-` forms. MCP parallel dispatch requires Codex;
`--no-model-parallel-tool-calls` also works with Claude by setting Messages
`disable_parallel_tool_use`. Explicit model-generation switches require a
provider URL. Without the gateway, native SDK model-generation behavior is unchanged.

Concurrent MCP dispatch does not guarantee entry order matching the model's
output. A background tool reply may acknowledge enqueueing rather than completed
motion. Physical actions sharing an episode resource remain queued by executor
policy; independent perception can overlap. See [architecture](architecture.md)
for ownership and lifecycle boundaries.

Codex's native collaboration events are normalized into agent activity records
for the console and TUI. This is SDK event handling; there is no separate
`--multi-agent` launcher option.

### Tool visibility debugging

`--tool-debug` makes `--tools` the complete agent-visible uni-robot-agent tool set for
both `run` and `tui`. Supply space-separated group selectors and/or tool names;
duplicates are removed. Unknown names/groups fail session assembly. An empty
list exposes no uni-robot-agent tools. Without `--tool-debug`, `--tools` is ignored and
the existing mode-selected surface is preserved. SDK-native tools are unaffected.

| Selector | Tools |
|---|---|
| `@skills` | `list_skills`, `read_skill` |
| `@scene` | `view_env_state`, `capture_scene`, `locate` |
| `@motion` | `move`, `grasp`, `release`, `hold` |
| `@workspace` | `read_text_file`, `write_text_file`, `list_dir` |
| `@observation` | `capture_observation`, `get_latest_observation`, `get_observation`, `list_observations`, `publish_insight`, `get_latest_insight`, `read_insight_updates` |
| `@jobs` | `get_tool_result`, `wait_for_tool`, `wait_for_tools`, `list_tool_jobs`, `cancel_tool`, `queue_status`, `flush_queue` |

Add `actor_act`, `write_skill`, `finish`, or any individual member explicitly.
For example, append to your usual episode command:

```bash
--tool-debug --tools @skills @scene @motion finish --tool-return-mode immediate
--tool-debug --tools @skills @scene @motion @jobs finish --tool-return-mode background
```

Visibility does not enable features: direct mode's `actor_act`, disabled
`write_skill`, and observation tools without an observation profile remain
unavailable on invocation. Backend configuration and validation stay in force.
Prompts and skills retain their configured mode guidance even if a referenced
tool is hidden. This intentionally isolates visibility for ablation.

Return mode, resource scheduling, model batching and MCP dispatch are unchanged.
Debug never adds/removes job controls automatically: background without `@jobs`
and immediate with `@jobs` are both valid. Hidden calls are rejected before
scheduling. Debug `tools.json` contains the exact executor schemas, including
controls; `tool_selection.json` records requested selectors, expanded names,
visible names and schemas. Normal-mode `tools.json` keeps its existing format.

### Skills

Skills use one library root. By default, `list_skills` and `read_skill` use
`uni_robot_agent/resources/skills/` in the installed package.
Set `UNI_ROBOT_AGENT_SKILLS_DIR` to select a complete external root instead. The
root contains `general/` and environment namespace directories such as
`robodojo/`, rather than pointing directly at one namespace. All skill tools use the selected root exclusively:
there is no overlay, packaged fallback, or automatic copying.

`list_skills()` takes no arguments and returns the searched scopes in `paths`
(e.g. `["general", "robodojo"]`) and sorted recursive Markdown paths in `entries`.
Every environment can read shared skills plus its own namespace, selected by
`metadata.skill_namespace` or the integration name. No other environment's
private namespace is exposed. `read_skill(path=...)` accepts
a Markdown file path, such as `general/gripper/pose-targeting/SKILL.md`, and returns
its `path`, `content`, and `sha256`. It does not accept directories. An empty
library returns `entries: []` without creating directories. A missing environment
namespace still allows discovery of general skills. Reads reject paths outside
these scopes; discovery excludes file symlinks crossing scope boundaries and does not
traverse directory symlinks.

`--skill-writing` additionally exposes `write_skill`. It requires an explicit
external skills root and permits `.md` files anywhere inside the current
environment namespace. Shared `general/` skills remain read-only to the agent,
even in an external library; maintain them through reviewed source changes.
The package resources remain read-only. There is no special `memory/` directory.
Environment skills contain reusable robot/control/environment knowledge, not
per-task recipes. General skills specify capability requirements and do not
reference a specific environment's private files.

An external library must contain the complete namespace layout described above.
Preserve the library and source snapshot used by each run for reproducibility.

To start from the bundled skills, choose a new directory and copy only the small
skills library once, using the Python environment where the package is installed:

```bash
export UNI_ROBOT_AGENT_SKILLS_DIR=/absolute/path/to/experiment/skills
python - <<'PYTHON'
import os
import shutil
from pathlib import Path
import uni_robot_agent.resources

source = Path(uni_robot_agent.resources.__file__).parent / "skills"
target = Path(os.environ["UNI_ROBOT_AGENT_SKILLS_DIR"])
shutil.copytree(source, target)  # Target must not already exist.
PYTHON
```

Then add `--skill-writing` to the episode command in that shell. For a uv tool
installation, prepare this directory from the checkout's
`uni_robot_agent/resources/skills/` instead. Store the external library on
persistent storage and use a distinct copy per run when skills can change.
Review written skills before sharing them across runs. No simulator,
checkpoint, run archive, or other large directory needs copying.

### Context

`--context-policy identity` preserves history without window tuning.
`sliding_window` supports Codex Responses and Gemini CLI GenerateContent through
the provider gateway. Both require
`--context-max-input-tokens` and `--context-target-input-tokens`.

| Sliding-window option | Default |
|---|---|
| `--context-keep-recent-blocks` | `4` |
| `--context-keep-observation-blocks` | `1` |
| `--context-min-recent-tool-images` | `3` |
| `--context-image-eviction-batch-size` | `20` |

Input-token budgets use an estimate, not a model tokenizer. The recent-history
counts refer to context blocks. `--context-min-recent-tool-images` controls the
image-eviction phase; whole-block trimming can still remove those images.
Explicit context budget/image options are rejected with `identity`. Choose
budgets within the limits supported by your model and provider.

Gemini trims native `contents` while keeping user prompts, system instructions,
tool definitions and generation settings. Call batches and all matching results
are indivisible; only images with tool-result provenance enter the image window.
The native CLI still retains its own history. See [external CLI context](external-cli-backends.md#gemini-context)
for retention and native-compression limits.

### Timeouts and outcomes

`--episode-timeout-s` limits agent episode wall time; `0` disables it. Its start
point depends on the connection mode:

- With `run --env-url`, the budget includes agent session startup and connection
  to the prepared environment. It does not start or supervise the external server.
  The CLI uses GNU `timeout` when available, otherwise a Python subprocess
  supervisor. At the deadline it sends INT, allowing 30 seconds before KILL.
  The Python fallback returns `124` after a timeout; GNU `timeout` can return
  `137` if forced termination is needed.
- With `run --deploy local`, environment startup has its separate
  `--startup-timeout-s` budget. The episode clock starts after the environment
  is ready and the agent subprocess is launched. Environment startup is excluded
  from `--episode-timeout-s`. The managed supervisor returns `124` on episode
  timeout and cleans up its owned processes; their shutdown grace is configured
  with `--shutdown-timeout-s` (default: 30 seconds).

Agent turns do not extend the deadline. Shutdown can extend total command time
beyond the episode budget, and forced termination may prevent final records.

Provider request timeouts/retries, MCP acknowledgements, service readiness and
episode wall time are separate budgets. The VLA reviewer keeps its own
request policy; see [reviewer configuration](#vla-reviewer).

Episode exit `75` signals a retryable failure; it does not itself retry the
episode. Exit `1` covers unsuccessful or unaccepted outcomes. Environment success
and agent completion are recorded separately. See [result inspection](getting-started.md#inspect-the-result).

## Integration options

| Integration | Episode configuration |
|---|---|
| `robodojo` | Shared gripper tools plus optional pinned task requirements; RoboDojo Pi0.5 requires an action-model URL and checkpoint SHA256 |
| `robouse` | Direct tasks and admitted LIBERO/RoboCasa Hybrid controllers; local deployment selects the native runtime profile |
| `robolab` | Direct and joint-policy Hybrid DROID tasks; connect to a prepared RoboLab/Isaac environment |

All three integrations accept `--action-profile direct|hybrid` (default `direct`).
Hybrid adds the independent `actor_act` tool and requires an actor URL and
`--action-model-checkpoint-sha256`. The binding defaults to the prepared environment
and can be selected with `--action-model-binding`; the SDK equivalent is
`integration_options["action_model_binding"]`. RoboDojo accepts `robodojo_pi05` or
`robodojo_x5_joint`, RoboLab accepts `robolab_droid_joint`, and RoboUse accepts
`robouse_cartesian` on its admitted controllers. Complete profile and environment
compatibility checks happen at session setup. See [Hybrid](hybrid.md) and the
[bridge deployment contract](services.md#xpolicylab-bridge).

Only RoboDojo supplies a task-requirements loader for
`--include-extra-task-requirements`. It is enabled by default and reads the packaged
`resources/prompts/robodojo/instruction_supplements.json`. The full
`task_requirements.json` is retained as a reference and is not loaded by default.
Use `--no-include-extra-task-requirements` to disable the supplements. Both
forms are RoboDojo-only options; RoboUse and RoboLab reject them.

### Actor execution budgets

Each `actor_act` chunk obtains a fresh observation and prediction, executes up to
`actions_per_chunk` actions, and discards the unused prediction. Let `H` be the
number of actions returned per prediction by the connected actor service.
`actions_per_chunk` defaults to `min(25, H, 40)` and accepts positive integers up
to `min(H, 40)`. Explicit values above that limit are rejected before inference.

| Binding | Current service output length | Default `actions_per_chunk` | Default `max_chunks` |
|---|---|---|---|
| RoboDojo Pi0.5 | 50 actions | 25 | 6 |
| X5, DROID or admitted RoboUse binding | Declared `H` actions | `min(25,H,40)` | 6 |

`max_chunks` accepts any positive integer; there is no shared ten-chunk cap.
The call stops at its requested chunk budget or earlier when the native episode
ends, the reviewer stops, or execution cannot continue. These defaults are starting
budgets; adjust them to the task and inspect the returned execution evidence.
The tool schema and execution use the same limits. Set `max_chunks=1` explicitly
for a single observation, prediction and execution round.

### VLA reviewer

The included Hybrid bindings enable the VLA reviewer by default. `--no-vla-review` disables it; enabling it with
direct control is invalid. Enabling or disabling review does not change the
actor execution budgets.

With `--no-sam3`, segmentation is unavailable. Depending on the prepared camera
capabilities, `locate` supports depth-based `method="pixels"`, calibrated RGB
`method="triangulate"`, or is absent when neither is possible. A configured SAM
service adds `method="segment"` and its arguments where supported.

The reviewer uses a separate model client to assess proposed action chunks. Configure
`--vla-review-model`, `--vla-review-base-url`, and `--vla-review-adapter`
(`passthrough.responses`, `volcengine.responses`, `moonshot.chat_completions`, `aliyun.chat_completions`, `deepseek.chat_completions`, or `minimax.chat_completions`). The model defaults to the main-agent model. Endpoint/profile inheritance requires
a Responses-compatible planner profile; a Messages planner requires an explicit
reviewer URL and adapter when review is enabled. The reviewer owns its own client
and request state; disabled review opens no client.

The credential precedence is `--vla-review-api-key`, then
`UNI_ROBOT_AGENT_VLA_REVIEW_API_KEY`, then `UNI_ROBOT_AGENT_API_KEY`. Credentials are excluded
from printed configuration. At execution time, the reviewer requires both an
explicit or inherited API endpoint and credential. Offline validation does not
check their availability. Its medium reasoning effort, 450-second timeout per
request, at most two transport attempts, and no HTTP-error retries are independent
of `--reasoning-effort`, `--provider-timeout-s`, `--provider-retries`, and
`--provider-retry-backoff-s`.
These settings configure the reviewer, not the Claude SDK's model transport.

## Service endpoints

Environment, actor and perception clients use HTTP(S) URLs. Environment episode
descriptions declare `protocol_version: "4"` and `tool_schema_version: "8"`;
actor and perception use their own versioned contracts. For example,
`--action-model-url http://127.0.0.1:8601` selects the shared actor API for
the selected actor runtime. Actor sessions carry explicit session, request
and sequence identities; policy state belongs to the explicit session.
See [Services](services.md) for routes, failure recovery and deployment TOML.

Service HTTP transport does not retry environment or model requests. Provider retries
remain in `ModelClient`, before any response is delivered to its consumer;
the reviewer separately selects its transport-only retry budget. A timed-out
environment or actor request must be reconciled through its receipt before
continuing, because native state or model RNG may already have advanced.

## Scripts

| Entry | Responsibility |
|---|---|
| [RoboDojo episode](../scripts/evaluation/robodojo/run_episode.sh) | Supply `--integration robodojo` and forward the episode CLI |
| [RoboDojo readiness wrapper](../scripts/evaluation/robodojo/run_when_ready.sh) | Wait for required endpoints, then run; `--startup-timeout-s` defaults to 360 |
| [Direct examples](../examples/README.md) | Prepare and run one RoboDojo, RoboLab or RoboUse task |
| [RoboDojo Hybrid example](../examples/robodojo_hybrid.sh) | Connect to prepared environment and actor services with an explicit model and checkpoint hash |
| [RoboDojo environment](../scripts/deploy/services/robodojo/start_environment.sh) | Start the official environment with `SIM_PYTHON` and explicit `--native-root` |
| [Actor service](../scripts/deploy/services/action_model/start_service.sh) | Start RoboDojo Pi0.5 with `ACTION_MODEL_PYTHON` and a service TOML configuration |
| [SAM3 service](../scripts/deploy/services/perception/start_sam3.sh) | Start perception in its prepared inference environment |

Configure service interpreters with `SIM_PYTHON`, `ACTION_MODEL_PYTHON` or
`PYTHON`. The [RoboUse launchers](../scripts/robouse/README.md) cover its supported
native backends, including LIBERO. For owned startup and cleanup, prefer the
[deployment CLI](deployment.md). No general batch evaluator is included.

## Public naming

The product, distribution, and CLI are `uni-robot-agent`; Python imports use
`uni_robot_agent`. Configuration environment variables use `UNI_ROBOT_AGENT_`.
Actor clients and services use `uni_robot_agent.actor.v2`; update both together.
Trace format 5 uses `$uni_robot_agent_image` references to the shared session image
directory. Existing records retain their original format. The source repository is
[Open-RUA/uni-robot-agent](https://github.com/Open-RUA/uni-robot-agent).

Use `--integration`, `--agent`, `--action-model-url`, `--action-model-checkpoint-sha256`,
`--vla-review`, and `--include-extra-task-requirements`.
Context retention uses `--context-keep-recent-blocks`,
`--context-keep-observation-blocks`, `--context-min-recent-tool-images`, and
`--context-image-eviction-batch-size`.
Configuration and result files use `agent_backend` and `action_model_url`.
Deployment scripts use `--action-model-*` flags and `ACTION_MODEL_*` variables.
Historical `--robot`, `--planner` and `--archive-policy` options are not supported.
Migrate scripts and result readers together.

### Tencent TokenHub and HY4

Use `tencent.responses` with a Responses planner such as Codex. Select the
TokenHub model ID available to your account; `hy4` is shown below. Supply your
own credential through `UNI_ROBOT_AGENT_API_KEY`:

```bash
export UNI_ROBOT_AGENT_API_KEY='replace-with-tokenhub-api-key'
uni-robot-agent run --integration robodojo --agent codex \
  --model hy4 --provider-profile tencent.responses \
  --provider-url https://tokenhub.tencentmaas.com/v1 \
  --reasoning-effort high --context-policy identity \
  --env-url http://127.0.0.1:18760 --action-profile direct --no-sam3 \
  --validate-config
```

`--validate-config` checks local configuration without contacting TokenHub or
robot services. Remove it to run against a prepared environment service.
This provider is an unreleased addition; use a source checkout containing
`tencent.responses` until a package release includes it.

The adapter preserves the supplied reasoning fields, reasoning history and
function-call/result pairing. Supported model IDs and reasoning effort values
are determined by your upstream service; the adapter does not remap them.
`--context-policy identity` preserves conversation selection before encoding.
Inline PNG images become lossless WebP; other images and external URLs remain
unchanged. Responses, including SSE events, errors and usage, are forwarded
without protocol conversion. The Hybrid reviewer can also select
`--vla-review-adapter tencent.responses` with its own model and endpoint.

### DeepSeek image history

Use `--provider-profile deepseek.chat_completions --context-policy identity` with the official
DeepSeek base URL and `deepseek-flash`. The adapter preserves full history and
reasoning, uploading original inline images to the [Files API](https://api-docs.deepseek.com/guides/files_api/).
Requests reuse file IDs within each client and credential; no sliding window,
resizing, or lossy image conversion is applied. External URLs and explicit
`detail=low` images remain inline because file IDs ignore the detail setting.

Local images and training exchanges remain replayable after remote file deletion;
exchange metadata records image SHA-256, file ID, expiry and upload failures.
Client shutdown attempts bounded deletion of only its own uploads, with a 24-hour
expiry as fallback. Account quotas (10,000 files / 25 GiB) still apply across
concurrent episodes. File references reduce request bytes, not inference tokens.
Token/cost reporting continues to use API usage; context-token estimates are unchanged.
In thinking mode, use automatic tool selection; DeepSeek rejects a forced named tool.


### MiniMax image and reasoning history

Use `--provider-profile minimax.chat_completions --provider-url https://api.minimax.cn/v1`
with `--model MiniMax-M3.1-Flash` and an explicit `--reasoning-effort` of
`low`, `medium`, `high`, `xhigh`, or `max`. M3.1 Flash cannot disable thinking.
The adapter reuses the shared Chat codec, preserves reasoning and tool-call
identity, and places tool images after each complete tool-result group.
PNG images are encoded as lossless WebP without changing pixels or dimensions;
OpenAI image detail `auto` maps to MiniMax `default`.

The documented request-body limit is 64 MB and the per-image limit is 10 MB.
Base64 expands binary image bytes by about one third. The 1M-token context
window does not remove this byte limit. This adapter does not truncate history
or upload images to a Files API. Use `--context-policy identity` to preserve
history and inspect actual request sizes before long runs.
Reference: https://platform.minimax.cn/docs/api-reference/text-openai-api

Measured on 2026-10-20 against `https://api.minimax.cn/v1` with
`MiniMax-M3.1-Flash`: 200 image blocks were accepted; 201 were rejected with
an explicit 200-image limit. A 94.51 MiB JSON request succeeded and a
126.01 MiB request returned HTTP 413. These bracket this tested route's body
limit; they do not establish a stable exact cutoff or replace the documented
64 MB budget. A decoded image above 10 MiB was rejected with a 10,485,760-byte
limit. Base64 bytes, per-image decoded bytes, image count, and context tokens
are separate constraints. A synthetic 100-tool-turn history containing
200 small images passed, but is not a long-horizon task-quality evaluation.
The adapter does not automatically prune history to fit these limits.

## Python sessions

The synchronous [SDK](sdk.md) uses the same execution and outcome pipeline.
`Session.run_agent()` returns `RunResult` instead of exiting the process. Tool-only
sessions record `SessionOpened` / `SessionClosed` plus tool events in the same
trajectory format; agent event names retain their historical encoding.
`ToolJob` provides polling, waiting and cancellation for one tool submission.

## Local Web monitor

Add `--monitor` to a single-task `run` to open a read-only Web console while the
agent is running. Both managed deployment and an existing environment URL work:

```bash
uni-robot-agent run \
  --integration robodojo --deploy local --task stack_bowls \
  --model "$MODEL" --action-profile direct --no-sam3 \
  --output-dir ./runs --monitor --monitor-port 8765
```

Keep the provider arguments and prepared runtime/cache arguments required by your
installation. The monitor does not download assets or prepare runtimes. Its URL
is printed before managed environment startup; deployment, run and failure states
are visible at the same URL. Defaults are `127.0.0.1:8765`; use port `0` for an
available OS-assigned port. The frontend is included in the wheel; no Node install
or frontend build is required.

View completed results directly from their existing directories:

```bash
# One run opens its console.
uni-robot-agent monitor --run-dir ./runs/robodojo_... --port 8765

# Multiple run directories open preview cards; click a card for its console.
uni-robot-agent monitor --run-dir ./runs/run_a ./runs/run_b --port 8765

# A parent directory discovers its recorded runs recursively.
uni-robot-agent monitor --run-dir ./runs --port 8765
```

This is local directory viewing, not uploading, cloud storage, publishing or batch
evaluation. Replay does not connect to the model or environment and does not
modify source results. Closing the browser does not stop a run. The monitor
started by `run` exits with that command; use `monitor --run-dir` to reopen the
saved result. The standalone monitor exits with Ctrl-C. For a cluster, run the
command beside the results and use your existing tunnel/port forwarding to reach
the loopback port; a printed cluster `127.0.0.1` is not your laptop's loopback.

The console shows camera observations, per-response tool batches, resource queues,
model request/execution overlap, elapsed time, environment step budget and official
outcomes. Blue means running, green means success and red means failure; labels
also distinguish infrastructure errors, cancellation and ungraded outcomes.

- All camera views share one playback time. Select a primary view from the dropdown
  or click a small camera tile; the other physical cameras continue alongside it.
  Changing the primary camera never seeks. Each view holds its latest observation
  at or before the playhead; a missing first frame is explicitly unavailable.
- Play follows recorded wall-clock timestamps, including multiple frames within
  one tool batch. Speed controls offer 1x/4x/16x. "Skip still intervals" explicitly
  jumps gaps longer than two seconds to the next recorded image or batch; turn it
  off for uninterrupted wall-clock playback. Slow image delivery can pause the
  playhead while images load. No synthetic/interpolated frames are generated.
- The seek rail shows proportional batch segments, image-update ticks and a shared
  cursor. Selecting a batch seeks to its recorded start. Previous/next buttons and
  arrow keys on the batch strip navigate batches. Return to latest resumes live
  following. Frame/call association is only exact when explicit IDs were recorded;
  appearing at the same playback time does not create an association.
- Completed replay also reads version-2 environment snapshot archives in
  `environment/` or `env/` under the selected directory. For historical split
  layouts (`env/` alongside `planner/<run>/`), pass their common episode directory,
  not only the planner subdirectory. An explicit shared snapshot ID must connect
  the archive to the run, and only that episode's RGB images are exposed. Archive
  indexing happens once per completed run at server startup/first read. Restart
  the monitor if you restore missing archive files later. Source files stay intact;
  smaller display previews are generated in memory. This reads existing captures
  and does not change the live environment's sampling or transport cadence.
- Camera labels count distinct recorded snapshots, not duplicate tool/mirror copies.
  Image transitions and selection feedback respect reduced-motion preferences.
- SDK acknowledgement can mean that work was queued. Actual execution completion
  is recorded independently. Codex transport receipts in MCP `_meta` associate
  SDK calls with runtime events once the receipt arrives; Claude carries its
  explicit tool-use ID into the execution context. No model-visible schemas,
  arguments or result content are changed by this metadata.
- Model request time includes transport, provider wait and retries; it is not a
  measurement of hidden reasoning time. Requests made outside the uni-robot-agent gateway
  may not have these intervals. Step usage is a budget, not task completion.
- Historical records without IDs, intervals, frames or scores show those fields
  as unavailable. They are not retroactively guessed. The display retains the
  latest 500 batches and 1,000 calls/request intervals. Live imagery is limited to
  2,000 frame entries; completed replay includes all archived frames. The reader
  retains source records in memory, so this is not an unbounded archival service.

Live monitors add `monitor.json`, `monitor_frames.jsonl` and content-addressed
`monitor_media/` images inside the run directory. Keep them alongside `result.json`,
`trajectory.jsonl` and `images/` when moving a run for replay.

Keep the associated environment `artifacts/` and `images/` directories for full
physical-step replay; planner tool-output images alone omit intermediate views.
