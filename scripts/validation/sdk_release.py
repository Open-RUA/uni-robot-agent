"""Install a wheel with full dependencies and check SDK imports/configuration outside the checkout.

Usage: python scripts/validation/sdk_release.py WHEEL NEW_DIRECTORY [--installer uv]
An optional --wheelhouse supplies offline dependencies without skipping resolution.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import venv
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('wheel', type=Path)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--installer', choices=('pip', 'uv'), default='pip')
    parser.add_argument('--antigravity', action='store_true', help='Also install and check the optional SDK')
    parser.add_argument('--wheelhouse', type=Path)
    parser.add_argument('--upgrade-from', help='Install this published main-package version before upgrading')
    args = parser.parse_args()
    wheel, directory = args.wheel.resolve(), args.directory.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    environment = {key: value for key, value in os.environ.items()
                   if not key.startswith('UNI_ROBOT_AGENT_')
                   and key != 'ANTHROPIC_BASE_URL'}
    environment.update(PYTHONPATH='', PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1')
    target = directory / 'venv'
    venv.create(target, with_pip=True)
    python = target / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
    def command(parts):
        subprocess.run([str(part) for part in parts], cwd=directory, env=environment, check=True)
    install = [python, '-m', 'pip', 'install'] if args.installer == 'pip' else [
        shutil.which('uv') or 'uv', 'pip', 'install', '--link-mode', 'copy', '--python', python]
    source = ['--no-index', '--find-links', args.wheelhouse.resolve()] if args.wheelhouse else []
    if args.upgrade_from:
        command([*install, *source, f'uni-robot-agent=={args.upgrade_from}'])
    command([*install, *source, str(wheel) + ('[antigravity]' if args.antigravity else '')])
    command([python, '-I', '-c', '''import pathlib,sys,uni_robot_agent
for name in uni_robot_agent.__all__:
    getattr(uni_robot_agent, name)
assert pathlib.Path(uni_robot_agent.__file__).is_relative_to(sys.prefix)
assert pathlib.Path(uni_robot_agent.__file__).with_name('py.typed').is_file()
assert not any(n in sys.modules for n in ('isaacsim', 'mujoco', 'torch'))
print('Installed SDK:', uni_robot_agent.__file__)
'''])
    executable = python.parent / ('uni-robot-agent.exe' if os.name == 'nt' else 'uni-robot-agent')
    command([executable, '--version'])
    command([executable, '--help'])
    command([python, '-m', 'uni_robot_agent', '--version'])
    for integration in ('robodojo', 'robouse', 'robolab', 'robotwin', 'libero_plus',
                        'libero_pro', 'rmbench', 'calvin', 'arx_one'):
        command([python, '-m', 'uni_robot_agent', 'run', '--integration', integration,
                 '--model', 'validation-model', '--task', 'validation-task',
                 '--env-url', 'http://127.0.0.1:18760',
                 '--no-sam3', '--tool-profile', 'direct',
                 '--agent-python', python, '--validate-config'])
    for backend, runtime_options in (
        ('antigravity', []),
        ('opencode', ['--opencode-version', 'offline-candidate']),
        ('gemini_cli', ['--gemini-version', 'offline-candidate',
                        '--gemini-bridge-manifest', 'offline-manifest.json']),
    ):
        command([python, '-m', 'uni_robot_agent', 'run', '--integration', 'robouse',
                 '--agent', backend, '--model', 'qwen-validation',
                 '--env-url', 'http://127.0.0.1:18760', '--no-sam3',
                 '--provider-url', 'https://provider.invalid/v1', *runtime_options, '--validate-config'])
    if args.antigravity:
        command([python, '-I', '-c', '''from importlib.metadata import distribution
from pathlib import Path
from google.antigravity import Agent, LocalAgentConfig
from uni_robot_agent.agents.antigravity.config import SDK_VERSION
dist = distribution('google-antigravity')
assert dist.version == SDK_VERSION
assert Path(dist.locate_file('google/antigravity/bin/localharness')).is_file()
assert callable(Agent) and callable(LocalAgentConfig)
print('Optional Antigravity SDK:', dist.version)
'''])
    for integration, binding in (('robodojo', 'robodojo_pi05'),
                                 ('robodojo', 'robodojo_x5_joint'),
                                 ('robolab', 'robolab_droid_joint'),
                                 ('robouse', 'robouse_cartesian'),
                                 ('dexjoco', 'dexjoco_tcp_hand')):
        command([python, '-m', 'uni_robot_agent', 'run', '--integration', integration,
                 '--task', 'validation-task', '--model', 'validation-model',
                 '--env-url', 'http://127.0.0.1:18760', '--no-sam3',
                 '--tool-profile', 'hybrid', '--action-model-binding', binding,
                 '--action-model-url', 'http://127.0.0.1:18761',
                 '--action-model-checkpoint-sha256', 'a' * 64, '--no-vla-review',
                 '--agent-python', python, '--validate-config'])
    command([python, '-m', 'uni_robot_agent', 'run', '--integration', 'robodojo',
             '--agent', 'codex', '--model', 'hy4', '--provider-profile', 'tencent.responses',
             '--provider-url', 'https://tokenhub.tencentmaas.com/v1',
             '--reasoning-effort', 'high', '--context-policy', 'identity',
             '--env-url', 'http://127.0.0.1:18760', '--action-profile', 'direct',
             '--no-sam3', '--agent-python', python, '--validate-config'])
    command([python, '-I', '-c', """from pathlib import Path
from importlib.util import find_spec
import uni_robot_agent
from uni_robot_agent import Session, SessionConfig, SetupConfig, DeploymentConfig
from uni_robot_agent.context.prompt import get_prompt_builder
from uni_robot_agent.provider.registry import build_provider_adapter
assert build_provider_adapter('tencent.responses', client_protocol='responses').profile_name == 'tencent.responses'
from uni_robot_agent.integrations.robodojo.action_binding import create_binding as x5_binding
from uni_robot_agent.integrations.robolab.action_binding import create_binding as droid_binding
from uni_robot_agent.integrations.robouse.action_binding import create_binding as cartesian_binding
from uni_robot_agent.tools.hybrid.provider import HybridTools
from uni_robot_agent.integrations.dexjoco.action_binding import create_binding as dexjoco_binding
from uni_robot_agent.services.actor.backends.openpi.config import resolve_config, recipe_settings
import sys
assert all(callable(factory) for factory in (x5_binding, droid_binding, cartesian_binding, dexjoco_binding))
assert not any(name in sys.modules for name in ('isaacsim', 'mujoco', 'torch', 'scipy'))
root = Path(uni_robot_agent.__file__).parent
assert callable(get_prompt_builder('robodojo'))
assert callable(get_prompt_builder('robouse'))
for resource in ('ui/web/static/index.html', 'ui/web/static/app.js',
                 'agents/opencode/bridge.js', 'agents/gemini_cli/robotBridge.ts',
                 'integrations/robodojo/deployment.toml',
                 'integrations/robouse/recipes/libero.toml',
                 'integrations/dexjoco/deployment.toml', 'integrations/dexjoco/SOURCE.md',
                 'resources/prompts/dexjoco/hybrid.md', 'resources/skills/dexjoco/index.md',
                 'services/actor/recipes/robodojo_pi05.toml',
                 'services/actor/recipes/dexjoco_pick_bucket.toml'):
    assert (root / resource).is_file(), resource
session = Session(SessionConfig(integration='robouse',
    env_url='http://127.0.0.1:18760', no_sam3=True))
session.close()  # Validate configuration only; do not connect or finalize an episode.
for profile, widths in (("robodojo_pi05", (14,14,50)), ("dexjoco_pick_bucket", (23,22,30))):
    config = resolve_config(dict(profile=profile, native_root="/unused/source",
        checkpoint="/unused/checkpoint", checkpoint_sha256="a" * 64))
    assert (len(config.profile.observation_fields), len(config.profile.action_fields),
            config.profile.action_horizon) == widths
    assert recipe_settings(profile)["actor"]["model"] == "pi05"
    SetupConfig(service="actor", profile=profile).resolve()
from uni_robot_agent.services.environment.device_contracts import CoordinatedDeviceStep, DeviceRequest
step = CoordinatedDeviceStep(device="robot", joints={"hand": {"finger": .1}})
assert DeviceRequest.model_validate_json(DeviceRequest(command=step,
    expected_state_version=0).model_dump_json()).command == step
SetupConfig(integration="dexjoco").resolve()
DeploymentConfig(integration="dexjoco", task="pick_bucket", depth=False).resolve()
assert not any(name in sys.modules for name in ("mujoco", "jax", "openpi", "torch", "scipy"))
resolved = SessionConfig(integration='robouse', env_url='http://127.0.0.1:18760',
                         no_sam3=True, tool_profile='direct').resolve()
assert resolved.tool_profile == 'direct'
assert resolved.tool_modules == ('skills', 'scene', 'gripper', 'episode')
legacy = SessionConfig(integration='robouse', env_url='http://127.0.0.1:18760',
                       no_sam3=True, integration_options={'action_profile': 'direct'}).resolve()
assert legacy.tool_modules == resolved.tool_modules
dex = SessionConfig(integration='dexjoco', env_url='http://127.0.0.1:18788',
    action_model_url='http://127.0.0.1:18769', no_sam3=True, tool_profile='hybrid',
    integration_options={'action_model_checkpoint_sha256': 'a' * 64,
                         'vla_review': False}).resolve()
assert dex.tool_modules == ('skills', 'scene', 'hybrid', 'episode')
try:
    SessionConfig(integration='robouse', env_url='http://127.0.0.1:18760',
                  no_sam3=True, tool_profile='direct',
                  integration_options={'action_profile': 'hybrid'}).resolve()
except ValueError as error:
    assert 'select different' in str(error)
else:
    raise AssertionError('Conflicting tool selectors accepted')
SetupConfig(integration='robodojo')
DeploymentConfig(integration='robodojo', task='stack_bowls')
rgb = DeploymentConfig(integration='robodojo', task='stack_bowls',
                       depth=False, camera_calibration=False).resolve()
assert rgb.depth is False and rgb.camera_calibration is False
try:
    DeploymentConfig(integration='robodojo', task='stack_bowls',
                     camera_calibration=False).resolve()
except ValueError as error:
    assert 'requires --no-depth' in str(error)
else:
    raise AssertionError('Uncalibrated depth configuration accepted')
from uni_robot_agent.integrations.robodojo.prompt import load_task_requirements
assert load_task_requirements('make_kong') == ''
assert '2 cm' not in load_task_requirements('arrange_largest_number_random')
assert 'object itself more than 10 cm' in load_task_requirements('general_pickup')
for name in ('robodojo_internal', 'efe', 'franka_robotiq', 'franka_sharpa', '_franka', 'dexterous_hand'):
    assert find_spec(f'uni_robot_agent.integrations.{name}') is None, name
    assert not (root / 'integrations' / name).exists(), name
for name in ('gripper', 'libero', 'robodojo_internal', 'efe', 'franka_robotiq', 'franka_sharpa'):
    try:
        Session(SessionConfig(integration=name, env_url='http://127.0.0.1:18760', no_sam3=True))
    except ValueError as error:
        assert 'not available' in str(error)
    else:
        raise AssertionError(f'Unavailable integration accepted: {name}')
assert find_spec('uni_robot_agent.tools.dexterous_hand') is None
assert not (root / 'tools' / 'dexterous_hand').exists()
print('Offline SDK configuration, unavailable-entry errors and packaged resources: OK')
"""])
    command([python, '-m', 'pip', 'check'])
    if args.upgrade_from:
        # Detect module ownership left behind by an upgrade or another distribution.
        command([python, '-m', 'pip', 'uninstall', '-y', 'uni-robot-agent'])
        command([python, '-I', '-c', "import importlib.util; assert importlib.util.find_spec('uni_robot_agent') is None"])


if __name__ == '__main__':
    main()
