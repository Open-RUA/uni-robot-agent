# Changelog

## Unreleased

- Add an offline `uni-robot-agent doctor` command that checks the installed
  package split, required and optional dependencies, agent CLI binaries,
  provider profiles, integration imports, skills root, output-directory
  writability and environment-variable presence without connecting to services,
  probing credentials or creating directories.
- Add experimental OpenCode and Gemini CLI backends with Qwen protocol bridges,
  explicit tool-call identity, response delivery gates and isolated subprocess
  state in `tool_calling` interaction mode. Gemini requires a separately built
  ACP compatibility patch and manifest covering its split bundle and assets.
- Add offline configuration and package checks plus an inert Qwen image/tool
  smoke runner. OpenCode `1.19.5` and the patched Gemini CLI
  `0.69.0-nightly.20261104.g89e9d25d6` passed live image/tool smoke runs with
  `qwen3.8-flash`; the pinned Gemini source also passed its full build and
  type-check. See [tested versions and limits](docs/external-cli-backends.md).
- Reject incomplete Chat tool generations and emit complete Responses text events.
  Preserve Codex's Qwen thinking default while disabling it for the new adapters.
- Add a Direct RoboTwin 2.0 adapter with dual-arm TCP control, measured gripper
  state, native seed admission and calibrated cached observations.

- Add a separate Direct LIBERO-PRO entry point for pinned base and perturbation
  suites, reusing the LIBERO-family controller and native observation path.

- Add a Direct LIBERO-Plus environment adapter with official task/initial-state
  selection, native corrupted images and explicit evaluation budgets.
- Extract Hybrid into an independent tool family with one receipt-aware execution
  loop, capability-derived schemas and integration-owned semantic bindings.
- Generalize XPolicyLab wire mappings; actor v2 separates observation/action
  semantics and timing. Upgrade actor clients/services and migrate bridge configs.
- Add RoboLab DROID joint-policy binding and native nominal FK preview, plus
  RoboUse LIBERO full-pose and RoboCasa translation-only bindings. Checkpoints
  still require explicit compatibility validation.
- Upgrade scalar environment protocol to 4 and tool schema to 8. Hybrid uses
  `active_groups` and capability-specific `constraints`; retain unknown device
  receipts to prevent replay. See [migration](docs/hybrid.md).

- Add opt-in `code_as_policy` interaction for Codex and Claude Code: run ordinary
  Python programs against the registered robot tools, with completed feedback,
  local image paths, and the existing session lifecycle.

- Add an optional XPolicyLab WebSocket bridge for explicitly validated RoboDojo
  X5 joint policies, with declared camera selection and action horizon.
- Add `--action-model-binding robodojo_x5_joint` and its SDK option to reuse
  hybrid review and correction with compatible actors; retain the Pi0.5 default.
- Record actor identity/profile in proposal archives and reviewer context;
  preserve full output chunks while bounding FK preview and execution prefixes.

- Add the `tencent.responses` provider for Tencent TokenHub, including HY4,
  preserving native Responses fields and losslessly encoding inline PNG images.
- Document provider configuration and validate Tencent support in installed SDKs.

## 0.1.1 — 2026-11-04

- Generate native CuRobo robot configs during setup in a private asset overlay;
  shared downloaded assets remain unchanged. Detect missing configs before launch.
- Accept the audited X5 URDF's byte-equivalent CRLF snapshot alongside its LF
  form, retaining exact mesh checks and checking these assets before GPU startup.

- Install a matching `uni-robot-agent-runtime` in native service environments,
  excluding planner SDK and UI dependencies while preserving the main CLI/SDK.
- Align RoboDojo and RoboUse Factory FastAPI/Uvicorn constraints with Isaac Sim
  5.1; retain strict dependency checks.
- Build both distributions from one source tree and verify independent sdists,
  resource parity, native HTTP hosting, and upgrades from 0.1.0.
- Build the pinned IsaacLab fork with a documented, checksum-guarded Starlette
  metadata correction and a local version suffix, preserving native module bytes.
- Install all IsaacLab base extensions and verify their required imports before
  marking the simulator runtime ready.
- `setup --wheel` now requires the runtime wheel at the CLI version.

Known issue: on containers that do not reap zombie descendants, SDK shutdown
can wait about 35 extra seconds and report forced cleanup after the simulator
has exited. The validated runs left no live simulator or GPU allocation behind.

## 0.1.0 — 2026-11-03

Initial open-source release of uni-robot-agent, a robot agent built on Codex and
Claude Code.

### Agent and robot tools

- Default RoboDojo task supplements to the simplified instruction_supplements
  catalog; retain the full task_requirements catalog as a reference. Keep its
  CLI switch and Python configuration field exclusive to RoboDojo.
- Support pure RGB RoboDojo observations with depth and camera calibration disabled;
  expose only 2D segmentation and retain robot proprioception and robot geometry.
- Wait for calibrated camera data alongside RGB when capturing RoboDojo scenes,
  including with depth disabled. Capture errors report no physical execution
  and allow subsequent actions instead of leaving the client locked.
- Guide RoboDojo planners to reposition a free arm for a clearer wrist-camera
  view, then recapture calibrated observations and use the updated extrinsics.
- Observe scenes, plan actions and execute manipulation tasks through shared
  perception, motion, skill and completion tools.
- Use Direct motion or RoboDojo Hybrid control with Pi0.5 proposals and an optional
  reviewer. Preserve execution receipts and report partial execution explicitly.
- Read shared and environment skills; write environment-local experience only
  when an external skill library and skill writing are explicitly enabled.
- Connect model providers through protocol adapters while preserving tool identity,
  call/result pairing and available model traces.

### CLI, SDK and monitoring

- Reject camera-calibration overrides outside RoboDojo environment deployment,
  including connections to an existing service, instead of silently ignoring them.
- Document episode timeout start points separately for external services and
  managed local deployment.

- Simplify the Hybrid example to use normal budgets and task supplements;
  exclude default run artifacts from Git.
- Add minimal RoboDojo, RoboLab and RoboUse episode scripts with preparation
  instructions, explicit model selection and offline configuration checks.
- Select the Codex calculation interpreter with `--agent-python`, independently
  of the agent runtime and service interpreters.
- Prepare isolated runtimes and resources with `setup`; start environment, actor
  and SAM3 services separately with `deploy`; use `run --deploy local` to run the
  agent on one task and clean up owned processes.
- Expose typed Python configuration, direct tools and queued jobs, built-in agent
  execution, events, saved results, and explicit setup/deployment APIs.
- Monitor live runs and replay result directories locally, including synchronized
  camera views, tool batches, resource queues and native outcome/step metrics.
  Multiple saved directories open a card overview; replay does not launch tasks.
- Preserve completed Codex counters and finalize model records on interruption
  before propagating cancellation to the caller.
- Keep tool success, agent completion and native task success distinct. Record
  explicit false outcomes, zero counters and storage failures without replacing
  the original execution outcome.

### Distribution

- Remove the unimplemented physical-home entry point and obsolete RoboUse test
  claims; document the worker interpreter's required installed package.
- Make source launchers prefer their own checkout over inherited Python paths.
- Supply RoboDojo observation guidance from its integration, keeping shared
  observation code independent of environment names.

- Remove experiment-specific provider bridging and obsolete diagnostic probes
  from the public distribution.
- Prepare checked release candidates and SHA-256 manifests without uploading;
  run Linux/macOS installation checks and explicitly dispatch tagged releases
  to PyPI or TestPyPI through Trusted Publishing.
- Document package-index installation for the CLI and SDK, matching-version
  runtime setup, and source/wheel installation before the first publication.

- Include official RoboDojo and RoboUse runtime recipes; LIBERO tasks use RoboUse.
  Other included adapters retain their existing connection workflows and stated
  validation limits. See [integrations](docs/integrations.md).
- Include SDK type information, prompts, skills and local monitoring resources,
  plus a standalone installation/configuration validator.
- Use `uni-robot-agent` for the distribution/CLI and `uni_robot_agent` for imports.
- Use `UNI_ROBOT_AGENT_*` configuration and consistent names for actor protocols,
  RoboUse workers, tool metadata and default storage paths. Update agent and
  service runtimes together; select existing pose libraries with `--store` or
  `--hand-pose-store`. New provider records use trace format 5.
