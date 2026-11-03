# Changelog

## Unreleased

- Persist reward-independent rollout measurements and pre-start manifests for
  CLI/Python sessions, including explicit missing-data reasons and offline replay.
- Record upstream generation attempts, separate tool interface responsibility
  from direct effects, and preserve background query identity without repeating
  the original action's failure.

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
