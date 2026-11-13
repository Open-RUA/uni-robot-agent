"""Prepared episode with explicit robot identity and externally assessed outcomes."""
from uni_robot_agent.integrations.base import EpisodeOutcome
from uni_robot_agent.integrations.prepared_episode import PreparedEpisodeSession


def validate_episode(session):
    info = session.episode.description
    if info.integration != "piper_x" or info.environment.metadata.get("robot_id") != session.context.args.robot_id:
        raise ValueError("Environment service is bound to another integration or robot_id")
    if session.context.args.tool_profile != "readonly" and not info.environment.metadata.get("execute"):
        raise ValueError("An observation-only environment requires --tool-profile readonly")


class Session(PreparedEpisodeSession):
    def finalize(self, *, agent_result, error):
        outcome = super().finalize(agent_result=agent_result, error=error)
        stop = outcome.metadata.get("stop", {})
        uncertain = outcome.metadata.get("execution_uncertain") or stop.get("status") not in {
            "confirmed_idle", "not_required"}
        self.result = EpisodeOutcome(success=None, accepted=not uncertain, retryable=False,
            failure_category="execution_uncertain" if uncertain else None,
            metadata={**outcome.metadata, "requires_external_task_assessment": True})
        return self.result


def create_session(context):
    return Session(context, validate_episode=validate_episode)
