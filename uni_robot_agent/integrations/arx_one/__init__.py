"""ARX ONE embodiment using the shared gripper tools.

The Body runtime lives in body/ and runs as its own process. Model
environments import that package through this module, so keep it free of agent
dependencies at import time.
"""
from dataclasses import replace


def validate_cli(args):
    from uni_robot_agent.integrations.gripper import validate_gripper_cli
    if args.action_profile != "direct" or args.action_model_url:
        raise ValueError("ARX ONE supports Direct control; no actor binding is registered")
    validate_gripper_cli(args)


def get_integration_spec():
    from uni_robot_agent.integrations.gripper import build_gripper_spec
    from . import prompt  # noqa: F401 - register the prompt builder.
    from .session import create_session

    from uni_robot_agent.integrations.composition import ToolModule
    from uni_robot_agent.integrations.episode_tools import SKILLS_MODULE, SCENE_MODULE, EPISODE_MODULE
    from uni_robot_agent.tools.gripper.provider import tool_policies
    from .motion.tools import BodyGripperTools

    motion = ToolModule("gripper", lambda session: BodyGripperTools(session.episode), tool_policies())
    modules = tuple(replace(module, policies={
        name: replace(policy, require_result_read=True,
                      replan_on_failure=policy.mutates_world and not policy.ends_episode)
        if policy.mutates_world else policy
        for name, policy in module.policies.items()
    }) for module in (SKILLS_MODULE, SCENE_MODULE, motion, EPISODE_MODULE))
    spec = build_gripper_spec(name="arx_one", create_session=create_session,
                             tool_profiles={"direct": modules}, validate_cli=validate_cli)
    return replace(spec, allow_automatic_retry=False, debug_modules=())
