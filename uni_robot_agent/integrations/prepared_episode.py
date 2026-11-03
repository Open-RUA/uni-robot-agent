"""Lifecycle and acknowledged state for a prepared environment episode."""
from __future__ import annotations

from pathlib import Path

from pydantic import ValidationError

from uni_robot_agent.artifacts.run import write_json_atomic
from uni_robot_agent.integrations.base import EpisodeOutcome
from uni_robot_agent.services.environment.client import EnvironmentClient, EnvironmentRequestError
from uni_robot_agent.services.environment.contracts import FinishRequest, RequestRejected
from uni_robot_agent.services.environment.device_contracts import DeviceEpisodeInfo, parse_state, parse_reply
from uni_robot_agent.telemetry.logging import get_logger
from uni_robot_agent.tools.toolkit import ToolOutput

logger = get_logger("episode")


class Episode:
    """One projection of service receipts, shared by all episode tools."""

    def __init__(self, env, description, output, *, artifacts):
        self.env, self.description, self.output = env, description, Path(output)
        self.artifacts = artifacts
        # The service owns the authoritative snapshot sequence.  The episode
        # keeps a local mirror so actor/perception/prompt consumers can
        # use local files after the first HTTP fetch instead of downloading the
        # same remote image repeatedly.
        self.artifact_mirror = env.attach_artifact_store(self.artifacts)
        self.task_requirements = ""
        self.latest_snapshot = env.record(0)
        self.state = parse_state(self.latest_snapshot["state"], description)
        self.latest_feedback = None
        self.recent_observations = []
        self.finished = False
        self.outcome = None
        self.finish_request = None
        self.storage_errors = {}
        self.instruction = description.metadata.get("instruction", description.task)

    @property
    def remaining_steps(self):
        if isinstance(self.description, DeviceEpisodeInfo):
            return None
        return max(0, self.description.environment.max_episode_steps - self.state.native_steps)

    @property
    def task_success(self):
        return None if self.outcome is None else self.outcome["task_success"]

    def require_active(self):
        if self.finished or self.env.uncertain or self.state.ended:
            raise RequestRejected("The episode is not available for a new physical action")

    def invoke(self, handler, **arguments):
        try:
            return handler(**arguments)
        except Exception as error:
            if isinstance(error, EnvironmentRequestError):
                status = error.error.execution_status
                return ToolOutput(success=False, error=str(error), extra={
                    **(error.error.details if isinstance(self.description, DeviceEpisodeInfo) else {}),
                    "no_execution": status == "not_started", "execution_uncertain": status == "unknown",
                    "execution_status": status}, call_error={
                        "code": error.error.code, "fault_owner": (
                            "infrastructure" if error.error.code in {
                                "internal_error", "state_unavailable", "execution_unknown"} else "unknown"),
                        "execution_status": status})
            if isinstance(error, (RequestRejected, ValidationError, ValueError)):
                return ToolOutput(success=False, error=str(error), extra={"no_execution": True},
                                  call_error={"code": getattr(error, "code", "unattributed_rejection"),
                                              "fault_owner": getattr(error, "fault_owner", "unknown")})
            from uni_robot_agent.services.errors import HttpCallError
            import httpx
            fault_owner = "infrastructure" if isinstance(error, (HttpCallError, httpx.TransportError)) else "unknown"
            logger.exception("Tool %s failed", handler.__name__)
            return ToolOutput(success=False, error="Tool execution failed",
                              extra={"execution_uncertain": self.env.uncertain},
                              call_error={"code": type(error).__name__, "fault_owner": fault_owner})

    def publish(self, reply):
        self.state = reply.state_after
        self.latest_feedback = reply.feedback
        # Failed image publication cannot replace an acknowledged state with a stale one.
        self.latest_snapshot = reply.snapshot
        self.recent_observations.append({**({"command_index": self.state.completed_commands} if isinstance(self.description, DeviceEpisodeInfo)
                   else {"step_id": self.state.native_steps}),
            "state": self.state.model_dump(mode="json"), "feedback": reply.feedback,
            "snapshot_id": None if reply.snapshot is None else reply.snapshot["snapshot_id"]})
        self.recent_observations = self.recent_observations[-6:]
        return reply

    def accept_primitive(self, reply):
        """Keep acknowledged progress visible if a later step loses its ACK."""
        self.state = reply.state_after.model_copy(deep=True)
        self.latest_feedback = reply.feedback
        if reply.snapshot is not None:
            self.latest_snapshot = reply.snapshot

    def finalize(self, *, reason, status=None, summary=""):
        if not self.finished:
            if not self.env.uncertain:
                self.finish_request = {"reason": reason, "status": status, "summary": summary}
            result = self.env.finish(FinishRequest(reason=reason, status=status))
            self._record_finish(result)
        return self.outcome

    def _record_finish(self, result):
        # A local write failure cannot undo the service's confirmed outcome.
        self.outcome = result["outcome"]
        self.finished = True
        for name, value in (("finish_request.json", self.finish_request),
                            ("native_outcome.json", self.outcome)):
            try:
                write_json_atomic(self.output / name, value, ensure_ascii=True,
                                  allow_nan=False, trailing_newline=True)
            except Exception as error:
                self.storage_errors[name] = f"{type(error).__name__}: {error}"
                logger.warning("Could not persist %s: %s", name, error)

    def image_paths(self, manifest):
        return {camera: str(self.artifact_mirror.local_path(item["rgb"]))
                for camera, item in manifest["cameras"].items()}

    def close(self):
        self.env.close()


class PreparedEpisodeSession:
    """Own connections and assemble explicitly supplied tool providers."""

    def __init__(self, context, *, configure_tools, requirements_loader=None):
        self.context, self.args = context, context.args
        self.configure_tools = configure_tools
        self.requirements_loader = requirements_loader
        self.episode = self.result = None
        self.tool_providers = ()
        self.action_model_name = None
        self._resources = []
        self.closed = False

    def own(self, resource):
        self._resources.append(resource)
        return resource

    def start(self):
        args = self.args
        env = EnvironmentClient(args.env_url, tracer=self.context.tracer)
        try:
            info = env.connect()
            args._episode_identity = info.model_dump(mode="json")
            args.camera_depth_enabled = {camera.name: camera.has_depth
                                         for camera in info.environment.cameras}
            for key, actual in (("run_id", info.episode_id), ("task", info.metadata.get("task_id", info.metadata.get("task", info.task))),
                                ("suite", info.metadata.get("suite")), ("seed", info.metadata.get("seed"))):
                expected = getattr(args, key, None)
                if expected is not None and str(expected) != str(actual):
                    raise ValueError(f"Episode {key} does not match the expected configuration value")
            self.episode = Episode(env, info, self.context.session_dir, artifacts=self.context.artifacts)
            if getattr(args, "instruction", None) is not None:
                self.episode.instruction = args.instruction
            if self.requirements_loader is not None:
                self.episode.task_requirements = self.requirements_loader(info.task)
            self.tool_providers = tuple(self.configure_tools(self))
        except BaseException:
            self.close()
            if self.episode is None:
                env.close()
            raise

    def task_succeeded(self):
        return self.episode.task_success

    def finalize(self, *, agent_result, error):
        if self.result is None:
            episode = self.episode
            error = error or getattr(agent_result, "error", None)
            if episode.env.uncertain:
                try:
                    receipt = episode.env.reconcile()
                    if receipt and receipt.get("status") == "completed":
                        result = receipt["result"]
                        if "state_after" in result:
                            episode.publish(parse_reply(result, episode.description))
                        elif "outcome" in result:
                            episode._record_finish(result)
                except Exception:
                    logger.exception("Could not reconcile the pending episode request")
            if episode.env.uncertain:
                self.result = EpisodeOutcome(success=None, accepted=False, retryable=True,
                    failure_category="execution_uncertain", metadata={"execution_uncertain": True})
            else:
                reason = "agent_error" if error else getattr(agent_result, "stop_reason", None) or ("session_finished" if agent_result is None else "agent_stopped_early")
                outcome = episode.finalize(reason=reason)
                metadata = dict(outcome)
                if episode.storage_errors:
                    metadata["storage_errors"] = dict(episode.storage_errors)
                self.result = EpisodeOutcome(success=episode.task_success, accepted=True,
                    failure_category=None if episode.task_success else reason, metadata=metadata)
        return self.result

    def close(self):
        if not self.closed:
            self.closed = True
            for resource in reversed(self._resources):
                try:
                    resource.close()
                except Exception:
                    logger.exception("Could not close prepared session resource")
            if self.episode is not None:
                self.episode.close()
