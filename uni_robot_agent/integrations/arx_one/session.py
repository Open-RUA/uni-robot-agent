"""ARX ONE episode session; physical outcomes are assessed externally and never retried automatically."""
from uni_robot_agent.integrations.base import EpisodeOutcome
from uni_robot_agent.integrations.prepared_episode import PreparedEpisodeSession

# A missing or unrecognized stop result is never treated as a verified stop.
STOPPED = {"confirmed_idle", "not_required"}


class Session(PreparedEpisodeSession):
    def start(self):
        super().start()
        if self.episode.description.integration != "arx_one":
            self.close()
            raise ValueError("ARX ONE requires an arx_one environment")

    def finalize(self, *, agent_result, error):
        outcome = super().finalize(agent_result=agent_result, error=error)
        stop = outcome.metadata.get("stop", {})
        uncertain = outcome.metadata.get("execution_uncertain") or stop.get("status") not in STOPPED
        self.result = EpisodeOutcome(success=None, accepted=not uncertain, retryable=False,
            failure_category="execution_uncertain" if uncertain else None,
            metadata={**outcome.metadata, "requires_external_task_assessment": True})
        return self.result


def create_session(context):
    return Session(context)
