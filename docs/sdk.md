# Python SDK

The `uni-robot-agent` distribution contains both the `uni-robot-agent` command and
an importable `uni_robot_agent` package. Use Python 3.11+ and install into the
same environment as your application. After the first PyPI publication:

```bash
python -m pip install uni-robot-agent
# Or, in a uv project:
uv add uni-robot-agent
uni-robot-agent --help
```

During the source preview, use the [source or wheel installation](getting-started.md#install-from-source-or-a-wheel)
instead. A downloaded wheel can be installed directly without a checkout.
`uv tool install` isolates a CLI application; use `uv add` or `uv pip install`
to make the SDK importable in your Python project.
Installing the package does not install simulators, GPU drivers, assets or weights.

## Public entry points

| CLI workflow | Python API |
|---|---|
| Prepare resources with `setup` | `setup(SetupConfig(...), report=print)` |
| Start a service with `deploy` | `deploy(DeploymentConfig(...))` and its owned context |
| Run the agent on one task with `run` | `run(SessionConfig(agent=AgentConfig(...), ...))` |
| Implement a custom strategy | `Session`, `ToolClient`, `ToolJob`, `ToolOutput` |
| Inspect outcomes and artifacts | `RunResult`, `RunRecord.open(path)` |
| View saved directories | Use `uni-robot-agent monitor --run-dir PATH`; no SDK Web host API |

Import these names from `uni_robot_agent`. `SessionConfig` contains environment,
execution and output settings; `AgentConfig` contains optional planner settings.
There is no `deploy="local"` field on SessionConfig: nest a Session inside an
explicit Deployment when Python owns the environment.

## Connect and write a strategy

Start an environment service first and use its URL below. Constructing a Session
only validates configuration; entering its context begins the connected episode.
For local service ownership, see [local preparation and deployment](#local-preparation-and-deployment).

```python
from uni_robot_agent import Session, SessionConfig

config = SessionConfig(
    integration="robodojo",
    env_url="http://localhost:18760",
    no_sam3=True,
    output_dir="runs",
)
with Session(config) as session:
    schemas = session.tools.list()
    observation = session.tools.call("view_env_state")
    if not observation.success:
        raise RuntimeError(observation.error)
    # Develop your strategy here, using schemas from this environment.
    result = session.finish()

print(result.task_success, result.session_dir)
```

Creating `Session(config)` validates configuration offline. Entering the context
(or calling `start()`) connects and assembles the existing integration, tools,
executor and artifact store. No model or agent is required for direct tool use.
`tools.list()` returns the session's JSON tool schemas; `call(name, arguments)`
returns a `ToolOutput`, including `success`, `error`, `extra` and `images`.
A failed tool returns `success=False`; it is distinct from official task success.
The example observes once and then finalizes the episode; add your strategy
before `finish()` to attempt the task.

The integration declares its own options through `IntegrationSpec.config_model`.
For example, the RoboDojo integration accepts
`integration_options={"task": "expected-task-id", "run_id": "expected-run-id"}`. These values
assert the identity of an already prepared episode. They do not create or reset
it. RoboDojo alone also accepts `include_extra_task_requirements` (default `True`);
set it to `False` to omit the packaged `instruction_supplements.json` guidance.
RoboUse and RoboLab reject this field. Unknown options fail before connecting.
Each integration retains its own
defaults, validators and capabilities. A strategy
that calls environment-specific tools still depends on those capabilities.

`no_sam3=True` disables SAM3 segmentation. Depth localization or RGB triangulation
can remain available when the environment supplies the required camera data.
To enable segmentation on a supported integration, supply `sam3_url` instead.
Hybrid action models use `action_model_url` and
`integration_options={"action_profile": "hybrid", ...}`. A reviewer in a tool-only
session needs its own model, endpoint and compatible profile. No agent model is
invented for it. Existing integration restrictions continue to apply.

## Run a built-in agent

Set `UNI_ROBOT_AGENT_API_KEY` in the process environment, then select your model
and compatible provider URL. The environment must already be running:

```python
from uni_robot_agent import AgentConfig, SessionConfig, run

result = run(SessionConfig(
    integration="robodojo",
    env_url="http://localhost:18760",
    no_sam3=True,
    agent=AgentConfig(model="your-provider-model", provider_url="https://provider.example/v1"),
    output_dir="runs",
))
print(result.agent_finished, result.task_success, result.storage_errors)
```

`AgentConfig` selects `agent_backend="codex"` (default) or `"claude_code"` and
contains model transport, retry, context and backend options. Claude uses a
Messages-compatible provider (`provider_profile="passthrough.messages"`) and its `claude_effort` / `claude_max_budget_usd`
settings. Explicit options for a different backend are rejected.
Credentials can use `AgentConfig(api_key=...)` or the existing environment settings.
`AgentConfig(provider_headers={...})` adds or overrides main-agent upstream HTTP
headers, for example OpenRouter attribution fields or an `api-key` credential;
header values are excluded from repr and serialized configuration.
Secret keys are excluded from persisted public configuration; endpoint credentials
and query strings are stripped from that snapshot.

`run(config)` opens, runs, finalizes and closes one session. With an explicit
`Session`, `run_agent()` uses its assembled tools and also finalizes and closes
on completion. It can run only once. Agent/provider execution failures are captured
in `RunResult`; invalid configuration and connection/startup failures raise Python
exceptions. `KeyboardInterrupt` propagates after cleanup. The SDK never calls
CLI `main()`, installs signal handlers, replaces the process or calls `sys.exit()`.

`RunResult` exposes `task_success`, `agent_finished`, `accepted`, `retryable`,
`failure_category`, `storage_errors`, `session_dir` and the complete `record`.
`exit_code` is the same outcome mapping used by CLI. A retryable result is
information for the caller, not automatic episode replay. Physical integrations
that prohibit replay retain that policy.

## Tool jobs, deadlines and ownership

```python
with Session(config) as session:
    job = session.tools.submit("capture_scene", {})
    observation = job.result(timeout=30)
    result = session.finish()
```

`submit()` uses the existing executor and declared resource policies. Calls that
share an exclusive resource remain FIFO; independent resources may overlap.
`ToolJob` is a handle for one tool call in one episode. It provides `job_id`,
`poll()`, `result(timeout=...)` and `cancel()`. It is not a batch evaluation job.
A wait timeout raises `TimeoutError` without cancelling or replaying work.
Cancellation succeeds only for queued work; running physical work is not forcibly
interrupted. Tool errors and cancelled jobs produce unsuccessful `ToolOutput`s.

Use one calling thread per Session. Session is synchronous; in an existing async
application or notebook, run the whole synchronous operation with
`await asyncio.to_thread(run, config)`. This version has no `AsyncSession`, batch
scheduler, GPU allocator or remote worker API.

`episode_timeout_s` defaults to 10800 (0 disables). The planner receives the
session deadline and new SDK calls are rejected after expiry. This is not a
process-level hard kill for arbitrary user code or an already executing tool.
Use `job.result(timeout=...)` to bound a wait; service transport timeouts and the
existing executor cleanup policy still apply. Unfinished physical work is reported
as uncertain rather than being retried. `cancel()` requests planner cancellation;
it is not a hardware emergency stop.

When Codex is interrupted, `run_agent()` preserves completed counters and attempts
model-record finalization before re-raising `KeyboardInterrupt`. Inspect
`session.result` or the saved result directory; an interrupted trajectory may be
partial, so check its `train_partial` and `train_error` fields.

`finish()`, `close()` and context exit finalize once and close owned connections.
Finalization seals the episode and may cause its service to release native episode
resources. It does not stop an external server process. A finished Session cannot
be reused for another task. Environment creation/reset remains outside this API.

## Local preparation and deployment

Prepare resources once, separately from per-episode execution. For a published
installation on a supported Linux host:

```python
from uni_robot_agent import SetupConfig, setup

runtime = setup(SetupConfig(integration="robodojo"), report=print)
```

For an unreleased build, pass its matching runtime wheel explicitly:

```python
from pathlib import Path
from uni_robot_agent import SetupConfig, setup

runtime = setup(SetupConfig(
    integration="robodojo",
    wheel=Path("/path/to/uni_robot_agent_runtime-0.1.1-py3-none-any.whl"),
), report=print)
```

Omitting `wheel` installs `uni-robot-agent-runtime` at the CLI/SDK version from
the configured index, in the separate native environment.
RoboDojo's native runtime requires Python 3.11; pass
`python="/path/to/python3.11"` to `SetupConfig` if it is not on PATH. See
[deployment prerequisites](deployment.md#host-prerequisites-and-runtime-storage).

This can install dependencies and download assets; it does not launch the server.
Once prepared, own the environment explicitly for each episode:

```python
from uni_robot_agent import DeploymentConfig, Session, SessionConfig, deploy

with deploy(DeploymentConfig(
    integration="robodojo", task="stack_bowls", deploy_gpu=0,
)) as environment:
    with Session(SessionConfig(
        integration="robodojo", env_url=environment.url, no_sam3=True,
    )) as session:
        observation = session.tools.call("view_env_state")
```

`deploy(config)` starts an already prepared local runtime and returns a
`Deployment` handle (`url`, `info`, `output`, `close()`). Its context closes only
its owned process group. Actor and SAM3 use `service="actor"` / `"sam3"`, with the
same supported profiles as CLI. Nest the Session inside the Deployment so
connections and episode work finish before the server is stopped.

Explicit `setup(SetupConfig(...))` returns a `Runtime` and reuses the existing
preparation/cache implementation. Supply `report=print` for progress messages.
Neither Session nor deploy invokes setup. See [deployment](deployment.md) for
supported local runtimes and their system requirements. Local GPU selection is a
visible-device index; there is no cross-cluster resource scheduling.

## Events and saved records

Subscribe with `session.subscribe(callback)` before or after opening. Events
include session lifecycle, actual tool execution and optional agent activity.
Callback failures are recorded in `storage_errors` without aborting physical work.
Callbacks execute synchronously on the producer thread and should be short.

```python
from uni_robot_agent import RunRecord

record = RunRecord.open(result.session_dir)
print(record.result["task_success"])
for event in record.events():
    for image in event.get("images", []):
        image_path = record.artifact(image["path"])
```

SDK and CLI use the same `result.json`, `trajectory.jsonl` and artifact store.
Tool-only sessions record events and images without an agent start event. Result
file failures remain visible in the in-memory result's `storage_errors`; an
unwritten file cannot be opened with `RunRecord`. Record reading is independent
of model SDK initialization and does not connect to services.

Model archives retain each backend's existing support. Codex can export its training
trajectory; Claude records native model exchanges but currently reports that SFT
export is not implemented. This does not turn a successful agent run into a failed
episode. Check `record["train_error"]` separately from the official outcome.

## View SDK results

Use the same directory viewer after finalization:

```bash
uni-robot-agent monitor --run-dir ./runs --port 8765
```

Pass `result.session_dir` as the path for one session, or a parent directory for
multiple-run cards. Tool-only sessions have no model request/batch activity;
the viewer only shows what the records contain. Session itself does not start a
Web host or a live camera collector. See [monitoring and replay](running.md#local-web-monitor)
for the artifacts needed for synchronized camera replay.
