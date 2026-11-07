# LIBERO-Plus and LIBERO-PRO

Stock LIBERO remains available through `--integration robouse`. The separate
`libero_plus` integration uses the official LIBERO-Plus task catalog, language,
initial states, corrupted observations and success predicates with the shared
Direct gripper tools. It does not change existing RoboUse tasks.

## Prepare a native runtime

Use an isolated Linux service environment with Python 3.11+, `robosuite==1.4.0`,
the `benchmarks` extra and the matching uni-robot-agent runtime distribution. Follow the
[upstream installation and asset instructions](https://github.com/sylvestf/LIBERO-plus)
for its other dependencies, including its image-corruption libraries. Do not
install stock LIBERO and its forks into the same interpreter.

Check that both the imported package and its distribution metadata resolve to
the intended environment; a shared site-packages directory can silently select
stock LIBERO or a different robosuite version. Plus also needs the ImageMagick
shared library for Wand, not just the Python package.

```bash
git clone https://github.com/sylvestf/LIBERO-plus.git /persistent/LIBERO-plus
git -C /persistent/LIBERO-plus checkout 4976dc30028e805ff8094b55501d532c48fec182
```

Prepare the official assets in that checkout. The server checks its pinned
revision and rejects tracked modifications. Native code and assets are external,
not included in the harness wheel. Native import, asset and rendering failures
are setup failures, not task failures. Automatic `setup` / `--deploy local`
recipes are not provided for this integration.

The official Plus archive includes parent directories before `assets/`. Place
the actual contents under `libero/libero/assets`, including the scene XMLs,
object meshes and textures needed by each selected task. Extracting only object
meshes is insufficient. PRO's generated BDDL and initial states are distributed
separately from its Git source; obtain the official files for the selected suite.

With PyTorch 2.6 or newer, the pinned upstream loader can reject official pickle
initial states under the new `weights_only=True` default. For verified, trusted
upstream initial-state files, set `TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1` in the
native service process. This enables pickle loading; do not use it with
untrusted state files.

## Start one episode

In the native service interpreter:

```bash
python -m uni_robot_agent.integrations.libero_plus.server \
  --native-root /persistent/LIBERO-plus --suite libero_spatial --task 0 \
  --init-state-index 0 --seed 0 --run-id plus-spatial-0 \
  --output-dir /persistent/runs/plus-spatial-0 --no-depth --port 18781
```

In the planner interpreter:

```bash
uni-robot-agent run --integration libero_plus --env-url http://127.0.0.1:18781 \
  --model MODEL --action-profile direct --no-sam3
```

`--task` on the service accepts an official task name or zero-based index.
The planner's optional `--task` checks the resolved **task name**, not that index.
Every service owns exactly one episode; start a new service for another trial.
The output directory must be new. Service creation, physics, images and teardown
run on a single owner thread.

## Evaluation semantics

- Select an explicit official initial-state index; invalid indices fail instead
  of wrapping. The simulator random seed is a separate field.
- Default settling is ten open-gripper no-op steps. Recorded policy budgets are
  220 / 280 / 300 / 520 for spatial / object / goal / long (`libero_10`). Override
  `--max-steps` and `--settle-steps` explicitly when reproducing another protocol.
- Upstream recommends **one trial per Plus task**, since perturbation variants
  already have separate task IDs. This service is an episode adapter, not a claim
  to have run the complete benchmark or reproduced a VLA leaderboard score.
- Capture reuses the native wrapper's last observation, including sensor noise.
  It never renders an uncorrupted replacement or advances physics. RGB and depth
  use the same vertical convention. Zoom/glass-distorted agent-view images have
  no advertised calibration or paired depth; wrist-camera capabilities remain.
- Only robot state and configured images are exposed. Object poses, native
  rewards and verifier diagnostics are not model-visible. `finish` cannot turn
  an unsuccessful trajectory into a success.
- Direct tool trajectories, camera/depth access, action budgets and training
  access must be reported when comparing against policy-learning baselines.

The native source retains its upstream license; no simulator code or assets are
vendored by this integration.

## LIBERO-PRO

`libero_pro` reuses the same OSC, snapshot and outcome adapter, with a separate
native process and pinned [LIBERO-PRO source](https://github.com/Zxy-MLlab/LIBERO-PRO).
Prepare its dependencies and official assets in a separate runtime, then use:

```bash
git clone https://github.com/Zxy-MLlab/LIBERO-PRO.git /persistent/LIBERO-PRO
git -C /persistent/LIBERO-PRO checkout eafdb809426b13153aa1e4c42d6601844217dfec
python -m uni_robot_agent.integrations.libero_pro.server \
  --native-root /persistent/LIBERO-PRO --suite libero_spatial_swap --task 0 \
  --init-state-index 0 --seed 0 --run-id pro-spatial-swap-0 \
  --output-dir /persistent/runs/pro-spatial-swap-0 --no-depth --port 18782
```

Connect the planner with `--integration libero_pro --env-url http://127.0.0.1:18782`.
The four base suites and their `_env`, `_object`, `_swap`, `_task`, `_lan` variants
are admitted. Initial states must exist for the exact selected task; the adapter
does not borrow another task's states, repair BDDL, generate OOD tasks or relabel
generated tasks as official. A missing task or asset stops preparation.

For multi-trial evaluation, enumerate official initial-state indices explicitly
and retain the native revision, suite, task name, seed and budget with each result.
The Plus one-trial recommendation does not apply to PRO. These integrations do
not expand RoboUse's separate task-admission list.

## Native validation scope

Real Linux/MuJoCo HTTP probes exercised Plus `libero_spatial` tasks 0 (table
texture) and 1390 (zoom blur), and PRO `libero_spatial_swap` task 0, each with
seed 0 and initial-state index 0. Each probe executed 20 native pose/gripper
actions, measured over 24 mm TCP displacement, saved camera images and verified
budget termination and cleanup. The zoom-blur probe withheld agent-view
calibration while retaining wrist calibration.

These bounded motion probes did not solve the tasks. Only their required assets
were unpacked; this validation does not establish complete suite coverage or a
benchmark success rate.

Subsequent Qwen3.8-Flash Direct episodes completed the Plus spatial task 0 and
PRO spatial-swap task 0 with native `task_success=true`, after 155 and 117
native actions respectively. Both used seed 0, initial-state index 0, RGB without
depth, camera calibration, SAM and runtime skills. The planner used Codex
`tool_calling`, max reasoning, a three-hour wall-clock budget and no turn limit.
The native service reported success independently of agent completion; the PRO
agent ended without calling `finish`.

These are two individual integration trials on the pre-merge implementation,
not suite-level scores or verification of later shared-harness changes. Native
SDK shell tools were available for scratch calculations and image inspection;
the trials are not presented as a fully audited benchmark evaluation.
