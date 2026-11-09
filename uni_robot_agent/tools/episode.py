"""Submit a completion claim through the shared episode lifecycle."""
from uni_robot_agent.execution.scheduler import ToolExecutionPolicy
from uni_robot_agent.services.environment.contracts import RequestRejected
from uni_robot_agent.tools.toolkit import ToolOutput, ToolProvider, ToolSpec, bind_tools, schema

NAMES = ("finish",)


def tool_policies():
    # finish stays allowed while a failed action awaits a replan, so the model can still report.
    return {"finish": ToolExecutionPolicy(resource_key="episode", concurrency="exclusive", mutates_world=True,
                                          ends_episode=True)}


# Each status the completion tool can report, with the reason the episode records for it.
FINISH_REASONS = {"success": "completion_claimed", "failure": "task_failed", "give_up": "task_abandoned"}
SUCCESS_ONLY = ("success",)


def finish_statuses(metadata) -> tuple[str, ...]:
    """The statuses an environment declares in ``finish_statuses``; success only by default."""
    declared = tuple(metadata.get("finish_statuses", SUCCESS_ONLY))
    if not declared or len(set(declared)) != len(declared) or not set(declared) <= FINISH_REASONS.keys():
        raise ValueError(f"finish_statuses must be distinct values from {sorted(FINISH_REASONS)}")
    return declared


def tool_specs(statuses=SUCCESS_ONLY):
    if tuple(statuses) == SUCCESS_ONLY:
        description = ("Submit completion only when every task requirement is verified by current observations. "
                       "Summarize the evidence. Submission does not override the environment's official outcome.")
    else:
        meanings = {"success": "success only when every task requirement is verified by current observations",
                    "failure": "failure when the task cannot be completed",
                    "give_up": "give_up when you stop trying"}
        description = ("End this task: " + "; ".join(meanings[status] for status in statuses) + ". "
                       "Summarize the evidence. Submission does not override the environment's official outcome.")
    return [
        ToolSpec("finish", description,
            schema({"status": {"type": "string", "enum": list(statuses)},
                    "summary": {"type": "string", "minLength": 1}}, ("status", "summary")))
    ]


class EpisodeTools(ToolProvider):
    provider_id = "episode"

    def __init__(self, episode):
        self.episode = episode
        self.statuses = finish_statuses(episode.description.environment.metadata)

    def get_tools(self):
        return bind_tools(tool_specs(self.statuses), provider_id=self.provider_id,
                          handlers={"finish": self.finish}, invoke=self.episode.invoke)

    def finish(self, status, summary):
        if status not in self.statuses or not summary.strip():
            raise RequestRejected("finish requires success and the observed completion evidence"
                                  if self.statuses == SUCCESS_ONLY else
                                  "finish requires a declared status and the observed evidence")
        self.episode.finalize(reason=FINISH_REASONS[status], status=status, summary=summary)
        return ToolOutput(extra={"submitted": True})
