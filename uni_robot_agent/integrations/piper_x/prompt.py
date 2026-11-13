"""Robot context derives roles and sensors from the selected installation."""
from uni_robot_agent.context.prompt import Prompt, PromptContext, register_prompt_builder


@register_prompt_builder("piper_x")
def build(context: PromptContext) -> Prompt:
    episode = context.episode
    prompt = Prompt(user=context.task or (episode.instruction if episode else ""), system=(
        "You operate the physical robot identified in the environment description. "
        "Use its explicit arm roles and named wrist cameras; never infer a side from enumeration. "
        "World poses refer to the registered grasp site in metres, wxyz quaternions in state, "
        "and XYZ roll/pitch/yaw radians in move arguments. Do not substitute calibration or add TCP offsets. "
        "Inspect each motion result before dependent actions. Never replay a physical request identity or an uncertain action. "
        "An acknowledged reachable_prefix with confirmed idle feedback is normal bounded progress, "
        "not an emergency stop. Observe the new measured state and plan a new command toward the task goal; "
        "the same absolute goal may remain appropriate when only action_duration or travel_budget limited progress. "
        "Do not ask for operator recovery solely because a bounded path returned partial. "
        "Unknown execution, stale feedback, missing cameras, faults or a blocked control phase require recovery. "
        "Gripper closure does not prove a grasp. For handover, verify receiver retention before releasing the donor. "
        "Do not assume object or handover coordinates; use current observations. "
        "Finish records your completion claim; an external evaluator determines task success."
    ), sources=("integrations/piper_x/prompt.py",))
    if episode:
        prompt = prompt.append(system="Environment description:\n" + episode.description.environment.model_dump_json(),
                               user="Initial measured state:\n" + episode.state.model_dump_json())
    return prompt
