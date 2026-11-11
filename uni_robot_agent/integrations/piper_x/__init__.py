"""Piper-X uses the shared gripper/scene tools and prepared episode lifecycle."""

from uni_robot_agent.integrations.gripper import GripperOptions


class PiperOptions(GripperOptions):
    robot_id: str


def add_cli_args(parser):
    from uni_robot_agent.integrations.gripper import add_gripper_cli_args
    add_gripper_cli_args(parser)
    parser.add_argument("--robot-id", required=True, help="Expected physical robot identity; never inferred from URL")


def validate_cli(args):
    from uni_robot_agent.integrations.gripper import validate_gripper_cli
    if args.action_profile != "direct" or args.action_model_url:
        raise ValueError("Piper-X currently supports the direct gripper profile")
    validate_gripper_cli(args)


def get_integration_spec():
    from dataclasses import replace
    from uni_robot_agent.integrations.episode_tools import SKILLS_MODULE, SCENE_MODULE, EPISODE_MODULE
    from uni_robot_agent.integrations.gripper import DIRECT_TOOLS, build_gripper_spec
    from .session import create_session
    from . import prompt  # noqa: F401 - register context without native drivers.
    spec = build_gripper_spec(name="piper_x", create_session=create_session,
        tool_profiles={"direct": DIRECT_TOOLS, "readonly": (SKILLS_MODULE, SCENE_MODULE, EPISODE_MODULE)}, add_cli_args=add_cli_args,
        validate_cli=validate_cli, config_model=PiperOptions)
    return replace(spec, execution_mode="replanning", allow_automatic_retry=False, debug_modules=())
