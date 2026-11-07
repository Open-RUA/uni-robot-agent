# RoboTwin 2.0

`--integration robotwin` connects the shared Direct gripper tools to a separately
prepared RoboTwin environment service. The initial binding supports the official
`aloha-agilex` dual-arm parallel-gripper embodiment and `demo_clean` /
`demo_randomized` task configurations. Hybrid actors, other embodiments and
automatic `setup` / `--deploy local` recipes are not included.

## Prepare and run

Follow the [RoboTwin installation and asset instructions](https://github.com/RoboTwin-Platform/RoboTwin)
in an isolated Linux native-service environment. Install the matching
`uni-robot-agent-runtime[benchmarks]` wheel there. Native SAPIEN, motion planners,
robot assets and task assets remain external and retain their upstream licenses.
Use this pinned, unmodified source checkout:

```bash
git clone https://github.com/RoboTwin-Platform/RoboTwin.git /persistent/RoboTwin
git -C /persistent/RoboTwin checkout ea8b21121ebb3cd201ff5b3fe361944ac94eda3f
python -m uni_robot_agent.integrations.robotwin.server \
  --native-root /persistent/RoboTwin --task beat_block_hammer \
  --task-config demo_clean --seed 1 --run-id robotwin-hammer-1 \
  --output-dir /persistent/runs/robotwin-hammer-1 --no-depth --port 18783
```

The output directory must be new. The service checks the upstream expert's seed
admission, then resets the same seed for the agent. A rejected seed is a setup
exclusion, not an agent failure. Instruction selection uses upstream seen/unseen
configuration; `--instruction-type` can override it. Upstream may fall back to the
task name when description generation fails. Metadata records the selected
configuration, seed, native revision, budget and evaluation profile.

The tested native stack uses Python 3.11, SAPIEN 3.0.0b1, mplib 0.2.1 and
CuRobo v0.7.8. A newer CuRobo installation may expose an incompatible API.
On an RTX 5090, the native probe used PyTorch 2.7.0+cu128 and CuRobo extensions
built for SM 12.0 with CUDA 12.8 and GCC 14. Install the official embodiment and
object assets and run upstream's `scripts/update_embodiment_config_path.py` to
generate local embodiment configuration paths.

Connect from the planner interpreter:

```bash
uni-robot-agent run --integration robotwin --env-url http://127.0.0.1:18783 \
  --model MODEL --action-profile direct --no-sam3
```

Each service owns one episode. Native setup, stepping, observations and cleanup
run on one owner thread. Launch another service for each new trial.

## Control and evaluation

- Public poses are world-frame TCP centres, with `wxyz` quaternions. The adapter
  converts the 12 cm local-X offset to the pinned native EE reference. Every
  primitive action supplies both arms, including a held arm.
- Gripper openings are normalized from 0 (closed) to 1 (open). State reads use
  articulation joint positions; requested drive targets are reported separately.
- One service step invokes one native `take_action(..., action_type="ee")`.
  That action executes a planned trajectory with variable physics duration.
  The advertised 30 Hz is the upstream nominal policy rate: tool hold durations
  and timing limits use nominal action counts, not measured simulation seconds.
- Use the native per-task action limit, or explicitly record `--max-steps`.
  Step counts mean native policy actions, not individual physics steps. Direct
  tool trajectories and this timing differ from VLA action-stream evaluations;
  report them when comparing scores.
- Head and both wrist RGB cameras are available. `--depth` opts into metric depth.
  Calibration maps optical coordinates into world coordinates. Capture reuses
  cached observations without another physics step or render. Object state,
  rewards, expert trajectories and success diagnostics are not model inputs.
- Success comes from the native evaluator. Agent `finish` status does not
  override it. Missing assets, rendering failures and invalid task configuration
  are setup errors, not benchmark failures.

This is an episode integration, not a full benchmark runner or a reproduced
leaderboard result. Native rendering and complete task coverage require the
upstream runtime and assets; CPU contract tests do not establish those results.

## Native validation scope

A Linux/SAPIEN HTTP probe exercised `beat_block_hammer`, `demo_clean`, seed 1.
The upstream expert admitted this seed; seed 0 was excluded by its precheck.
Twenty native EE actions moved both TCPs by over 25 mm, saved all three RGB
views and exercised budget termination and cleanup. The adapter accepts
SAPIEN's native 3-by-4 camera extrinsics and converts them to homogeneous
transforms before inversion.

The probe did not solve the hammer task. Its renderer reported OIDN invalid-handle
errors: the saved images contained the scene, but visibly retained ray-tracing
noise. Rendering quality needs validation on the deployment's driver/runtime
before score comparisons. Other tasks, randomized configurations and complete
benchmark success rates have not been established by this probe.

A subsequent Qwen3.8-Flash Direct episode on the same task and seed executed
395 of 400 native actions and ended with native `task_success=false`. It used
RGB without depth, calibration, SAM, runtime skills and Codex `tool_calling`
with max reasoning. The planner stopped without calling `finish`; the recorded
run lasted approximately 94.5 minutes and did not report a provider exception.
The failed grasp attempts establish an unsuccessful task trial, not a diagnosed
collision-model defect: the planner's explanation needs independent native
contact/geometry checks. The known rendering limitation also remains.

This trial used the pre-merge implementation and does not verify later shared
harness changes or establish a benchmark success rate.
