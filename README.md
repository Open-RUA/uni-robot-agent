<p align="center">
  <a href="assets/brand/uni-robot-agent-cover.png"><img src="assets/brand/uni-robot-agent-cover.png" alt="uni-robot-agent — A unified harness for robot use." width="100%"></a>
</p>

<p align="center">
  <strong>uni-robot-agent</strong> is an open-source harness for robot-use agents, with Codex and Claude Code runtimes.
</p>

<p align="center">
  <a href="https://pypi.org/project/uni-robot-agent/"><img src="https://img.shields.io/pypi/v/uni-robot-agent?color=62518B&amp;style=for-the-badge&amp;logo=pypi&amp;logoColor=white&amp;labelColor=242424" alt="Latest PyPI version"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-MIT-386749?style=for-the-badge&amp;labelColor=242424" alt="License: MIT"></a>
  <a href="https://uni-robot-agent.com/docs"><img src="https://img.shields.io/badge/docs-Read%20guide-295E91?style=for-the-badge&amp;logo=readthedocs&amp;logoColor=white&amp;labelColor=242424" alt="Read the documentation"></a>
  <a href="https://uni-robot-agent.com/"><img src="https://img.shields.io/badge/website-Homepage-98472F?style=for-the-badge&amp;labelColor=242424" alt="Project website"></a>
</p>

<p align="center">
  <a href="#quick-start"><strong>Quick start</strong></a> ·
  <a href="#python-sdk">Python SDK</a> ·
  <a href="#dashboard">Dashboard</a> ·
  <a href="#environments">Environments</a> ·
  <a href="#how-it-works">How it works</a> ·
  <a href="#contributing">Contributing</a> ·
  <a href="#license">License</a> ·
  <a href="#citation">Citation</a>
</p>

<a id="whats-new"></a>

## What's New

**Hybrid** now separates shared orchestration, reusable actor backends and integration-specific bindings. [DexJoCo](docs/dexjoco.md) adds dexterous-hand manipulation with OpenPI/Pi0.5. [Learn more](docs/hybrid.md).

**Minimal mode** offers a simple three-tool baseline (`move`, `hold`, `finish`) for studying robot-use agents with either RGB plus camera intrinsics/extrinsics or RGB-only observations ([`--tool-profile minimal`](docs/integrations.md#minimal-robot-tools)).

**Code as Policy** is here: control robots with code. Enable it with
`--interaction-mode code_as_policy`. [Learn more](docs/code-as-policy.md).

**v0.1.1** fixes fresh RoboDojo setup with isolated simulator dependencies and automatic robot configuration.
See [pip and uv installation](#quick-start) below.

<a id="features"></a>

## Features

Use one package to run a built-in agent, call robot tools from your own Python
strategy, and inspect live or saved episodes. Direct mode uses robot tools;
Hybrid mode adds a compatible action model such as Pi0.5. Prompts, skills and
the dashboard are included in the package.

Experimental [OpenCode and Gemini CLI adapters](docs/external-cli-backends.md)
add Qwen-backed runtime options. They require separate CLI installations and
live compatibility validation; Gemini also requires the documented ACP bridge.
The optional [Antigravity SDK backend](docs/antigravity.md) adds a pinned Python
SDK runtime for Qwen tool-calling runs.

The harness follows four design principles:

- **Unified:** shared tool interfaces and feedback across environments, with native APIs handled by adapters.
- **Efficient:** overlap agent reasoning with tool execution while preserving order for operations that share robot resources.
- **Realistic:** support the paper's RGB-only setting, using proprioception, calibration and robot descriptions without simulator depth or task-verifier feedback to the agent.
- **Modular:** replace the agent runtime, model provider, tools, environment or model services through explicit interfaces.

RoboDojo enables depth by default. Select `--no-depth` for the paper's RGB-only
setting and retain camera calibration. See [design principles](https://github.com/Open-RUA/uni-robot-agent/blob/main/docs/architecture.md#design-principles)
and [observation settings](https://github.com/Open-RUA/uni-robot-agent/blob/main/docs/integrations.md#realistic-observation-settings).

<a id="quick-start"></a>

## Quick start

**1. Install and prepare RoboDojo.** Use a Python 3.11 virtual environment on a [Linux GPU host](https://github.com/Open-RUA/uni-robot-agent/blob/main/docs/deployment.md#host-prerequisites-and-runtime-storage).

```bash
pip install uni-robot-agent  # or: uv pip install uni-robot-agent
uni-robot-agent setup --integration robodojo
```

**2. Run a task.** Set your vision model and Responses-compatible endpoint:

```bash
export UNI_ROBOT_AGENT_API_KEY='YOUR_API_KEY'
export UNI_ROBOT_AGENT_PROVIDER_URL='https://provider.example/v1'

uni-robot-agent run --integration robodojo --deploy local \
  --task stack_bowls --model YOUR_MODEL_ID \
  --action-profile direct --no-sam3 --output-dir ./runs --monitor
```

Open the printed dashboard URL; results are saved under `./runs`. [More options](https://github.com/Open-RUA/uni-robot-agent/blob/main/docs/getting-started.md).

<a id="python-sdk"></a>

## Python SDK

Run the agent against an already running environment. This uses the same API key
as the CLI; replace the model and provider placeholders with your settings:

```python
from uni_robot_agent import AgentConfig, SessionConfig, run

result = run(SessionConfig(
    integration="robodojo",
    env_url="http://127.0.0.1:18760",
    no_sam3=True,
    agent=AgentConfig(
        model="YOUR_MODEL_ID",
        provider_url="https://provider.example/v1",
    ),
    output_dir="runs",
))
print(result.agent_finished, result.task_success, result.session_dir)
```

`agent_finished` records agent completion; `task_success` is the environment's
reported outcome, or `None` when unavailable. Use `Session.tools` to implement a
custom strategy, `RunRecord` to read saved runs, and `setup()` / `deploy()` to own
local services explicitly. See the [SDK guide](https://github.com/Open-RUA/uni-robot-agent/blob/main/docs/sdk.md) and
[Python examples](https://github.com/Open-RUA/uni-robot-agent/tree/main/examples/sdk/).

<a id="dashboard"></a>

## Dashboard

Add `--monitor` to a CLI run to view camera streams, tool calls, execution timing
and available task outcomes. The monitor prints a local URL; no frontend build
or upload is required.

Replay saved episodes or browse all runs under a parent directory:

```bash
uni-robot-agent monitor --run-dir ./runs
```

Select an episode, play or pause the recording, switch cameras and jump between
tool calls. See [live monitoring and replay](https://github.com/Open-RUA/uni-robot-agent/blob/main/docs/running.md#local-web-monitor)
for remote-host access and other options.

<a id="environments"></a>

## Environments

### Simulation

| Integration | Environments and tasks | Guide |
|---|---|---|
| **RoboDojo** | Dual-arm manipulation with Direct tools or Hybrid Pi0.5 control | [Setup](https://github.com/Open-RUA/uni-robot-agent/blob/main/docs/deployment.md) |
| **RoboUse** | Selected tasks from LIBERO, RoboCasa, MetaWorld, RoboSuite, ManiSkill, Tabletop, Fetch and Kitchen; also Menagerie, composed-robot, Genesis and Factory task adapters | [Tasks](https://github.com/Open-RUA/uni-robot-agent/blob/main/scripts/robouse/README.md) |
| **LIBERO-Plus / PRO** | Official perturbation suites through separately prepared Direct services | [Integration](docs/libero-extensions.md) |
| **RoboTwin 2.0** | ALOHA AgileX dual-arm tasks through a separately prepared Direct service | [Integration](docs/robotwin.md) |
| **RMBench** | Native memory-dependent dual-arm tasks with four camera views | [Integration](docs/rmbench.md) |
| **CALVIN** | Native Panda single-task episodes; no five-task chain runner | [Integration](docs/calvin.md) |
| **RoboLab** | Single-DROID tasks in a separately prepared Isaac Sim / Isaac Lab runtime | [Integration](https://github.com/Open-RUA/uni-robot-agent/blob/main/docs/robolab.md) |
| **DexJoCo** | Dexterous-hand manipulation with a Panda arm and 16-DoF Allegro hand using Hybrid Pi0.5 control | [Integration](docs/dexjoco.md) |

### Real robots

| Platform | Public release status |
|---|---|
| **ARX ONE** | Available: dual ARX X5 arms and three RealSense D405 cameras behind a device process ([integration](uni_robot_agent/integrations/arx_one/README.md), [deployment](uni_robot_agent/integrations/arx_one/DEPLOYMENT.md)) |
| **Franka** | Coming soon |

The ARX ONE integration is also a template for connecting another real robot: its Body
framework and host tooling are robot-independent, and each device kind plugs in as a driver
with a host kit.

<a id="how-it-works"></a>

## How it works

<p align="center">
  <a href="assets/diagrams/uni-robot-agent-architecture.png"><img src="assets/diagrams/uni-robot-agent-architecture.png" alt="uni-robot-agent architecture: agent backends, shared robot tools, independent services, and native adapters." width="100%"></a>
</p>

The executor schedules tool calls from agents and the Python SDK, and shared robot tools access native backends through services. [Architecture guide](https://github.com/Open-RUA/uni-robot-agent/blob/main/docs/architecture.md)

<a id="contributing"></a>

## Contributing

Follow [CONTRIBUTING.md](https://github.com/Open-RUA/uni-robot-agent/blob/main/CONTRIBUTING.md) to set up a development environment,
or [open an issue](https://github.com/Open-RUA/uni-robot-agent/issues) with a minimal
reproduction, package version, platform and relevant logs.

<a id="license"></a>

## License

This project is licensed under the [MIT License](https://github.com/Open-RUA/uni-robot-agent/blob/main/LICENSE).

[Upstream code](https://github.com/Open-RUA/uni-robot-agent/blob/main/uni_robot_agent/integrations/robodojo/UPSTREAM_LICENSE), [fonts](https://github.com/Open-RUA/uni-robot-agent/blob/main/assets/brand/README.md#typography), and external assets and models retain their respective licenses.

<a id="citation"></a>

## Citation

If you use uni-robot-agent in your research, please cite our paper:

```bibtex
@article{yuan2026democratizing,
  title={Democratizing Robot-Use Agents: A Unified Harness with Open Models},
  author={Yuan, Jiarui and Zuo, Yuxin and Huang, Xiaoyu and Zhang, Wenbo and Park, Kwanyoung and Yang, Junlin and Zhang, Tianxiang and Chen, Ruoqu and Ouyang, Yutao and He, Bingxiang and others},
  journal={arXiv preprint arXiv:2611.01560},
  year={2026}
}
```
