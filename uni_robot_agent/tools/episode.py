"""Submit a completion claim through the shared episode lifecycle."""
from uni_robot_agent.execution.scheduler import ToolExecutionPolicy
from uni_robot_agent.services.environment.contracts import RequestRejected
from uni_robot_agent.tools.toolkit import ToolOutput, ToolProvider, ToolSpec, bind_tools, schema

NAMES = ("finish",)


def tool_policies():
    return {"finish": ToolExecutionPolicy(resource_key="episode", concurrency="exclusive", mutates_world=True)}


def tool_specs():
    return [
        ToolSpec("finish",
            "Submit completion only when every task requirement is verified by current observations. Summarize the evidence. Submission does not override the environment's official outcome.",
            schema({"status": {"type": "string", "enum": ["success"]},
                    "summary": {"type": "string", "minLength": 1}}, ("status", "summary")))
    ]


class EpisodeTools(ToolProvider):
    provider_id = "episode"

    def __init__(self, episode):
        self.episode = episode

    def get_tools(self):
        return bind_tools(tool_specs(), provider_id=self.provider_id,
                          handlers={"finish": self.finish}, invoke=self.episode.invoke)

    def finish(self, status, summary):
        if status != "success" or not summary.strip():
            raise RequestRejected("finish requires success and the observed completion evidence",
                                  code="finish_claim_arguments", fault_owner="policy")
        self.episode.finalize(reason="completion_claimed", status="success", summary=summary)
        return ToolOutput(extra={"submitted": True})
