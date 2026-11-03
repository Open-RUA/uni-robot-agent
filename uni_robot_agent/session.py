"""Runtime assembly and single-episode SDK, CLI and TUI lifecycles."""

from __future__ import annotations

import json
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any
from uuid import uuid4

from uni_robot_agent.artifacts.run import build_run_result, write_result_json
from uni_robot_agent.artifacts.records import RunResult
from uni_robot_agent.artifacts.measurements import RunMeasurements
from uni_robot_agent.tools.client import ToolClient
from uni_robot_agent.config import SessionConfig
from uni_robot_agent.events import SessionOpened, SessionClosed
from uni_robot_agent.artifacts.store import ArtifactStore
from uni_robot_agent.events.subscribers import (
    ConsoleSubscriber,
    EventJsonlSubscriber,
    EventSubscriptions,
)
from uni_robot_agent.config import resolve_context_config, resolve_agent_model_config, runtime_env
from uni_robot_agent.context.prompt import PromptContext, assemble_prompt, get_prompt_builder
from uni_robot_agent.events import ErrorEvent, EventBus, AgentRunFinished, Event
from uni_robot_agent.observation import load_observation_profile
from uni_robot_agent.agents import AgentResult, create_agent, get_agent_backend
from uni_robot_agent.agents.contracts import AgentInterrupted
from uni_robot_agent.integrations import load_integration
from uni_robot_agent.integrations.base import (
    IntegrationSession,
    IntegrationContext,
    EpisodeOutcome,
)
from uni_robot_agent.execution import (
    ParallelToolExecutor,
    ToolExecutor,
    connect_execution_events,
)
from uni_robot_agent.telemetry import TelemetrySession, create_telemetry
from uni_robot_agent.telemetry.logging import get_logger
from uni_robot_agent.tools.toolkit import Toolkit
from uni_robot_agent.tools.observation import ObservationTools
from uni_robot_agent.tools.selection import select_tools
from uni_robot_agent.tools.workspace import WorkspaceProvider


@dataclass
class SessionAssembly:
    """Shared assembly with environment resources owned by the session."""

    toolkit: Toolkit
    executor: ToolExecutor
    build_prompt: Callable[[str], Any]
    integration_session: IntegrationSession
    session_dir: Path
    artifacts: ArtifactStore
    observation_profile: Any | None
    telemetry: TelemetrySession
    allow_automatic_retry: bool = True


def assemble_session(args) -> SessionAssembly:
    """Assemble tools and executor through IntegrationSession without depending on transport details."""
    session = None
    executor = None
    observation_profile = None
    output_dir = args.output_dir or "outputs"
    session_id = (
        f"{args.integration}_{args.agent_backend or 'tools'}_{int(time.time())}_{uuid4().hex[:8]}"
    )
    session_dir = Path(getattr(args, "_session_dir", None) or Path(output_dir) / session_id).resolve()
    session_dir.mkdir(parents=True, exist_ok=True)
    (session_dir / "scratch").mkdir(exist_ok=True)
    telemetry = create_telemetry(
        session_dir, getattr(args, "telemetry", "off"), component="harness"
    )

    artifacts = ArtifactStore(session_dir)
    try:
        integration_mod = load_integration(args.integration)
        integration_spec = integration_mod.get_integration_spec()
        session = integration_spec.create_session(
            IntegrationContext(
                args=args,
                session_dir=session_dir,
                artifacts=artifacts,
                tracer=telemetry.tracer,
            )
        )
        session.start()

        telemetry.annotate(
            integration=args.integration,
            agent_backend=args.agent_backend,
            model=args.model,
            tool_return_mode=args.tool_return_mode,
            resource_scheduling=args.resource_scheduling,
            action_profile=getattr(args, "action_profile", None),
        )

        tool_providers = (*session.tool_providers, WorkspaceProvider())
        tools = [tool for provider in tool_providers for tool in provider.get_tools()]
        if getattr(args, "command", "run") == "run":
            visible_names = set(integration_spec.eval_tool_names)
            if visible_names and getattr(args, "skill_writing", False):
                visible_names.add("write_skill")
            tools = (
                [tool for tool in tools if tool.name in visible_names]
                if visible_names
                else tools
            )

        profile_name = getattr(args, "observation_profile", "none")
        if profile_name != "none":
            if args.resource_scheduling != "per-resource":
                raise ValueError(
                    "observation profiles currently require --resource-scheduling per-resource"
                )
            profile_mod = load_observation_profile(profile_name)
            observation_profile = profile_mod.create_observation_profile(
                session.episode,
                session_dir,
            )
            tools.extend(observation_profile.tools)

        # Debug tool filtering does not change prompt guidance for the configured modes.
        prompt_tool_names = tuple(sorted(tool.name for tool in tools))
        selection = None
        if getattr(args, "tool_debug", False):
            declared = [tool for provider in tool_providers for tool in provider.get_declared_tools()]
            declared.extend(observation_profile.tools if observation_profile is not None
                            else ObservationTools(None).get_declared_tools())
            selection = select_tools(declared, getattr(args, "tools", ()))
            tools = list(selection.tools)

        toolkit = Toolkit()
        # Preserve tool order so provider composition does not change model-visible schema order.
        toolkit.register_tools(sorted(tools, key=lambda tool: tool.name))
        policies = dict(integration_spec.tool_policies)
        listeners = ()
        if observation_profile is not None:
            policies.update(observation_profile.policies)
            listeners = observation_profile.listeners
        executor = ParallelToolExecutor(
            toolkit, policies, listeners=listeners, tracer=telemetry.tracer,
            tool_return_mode=args.tool_return_mode,
            resource_scheduling=args.resource_scheduling,
            control_tool_names=None if selection is None else selection.control_names,
        )
        schemas = toolkit.get_tool_schemas() if selection is None else executor.get_tool_schemas()
        (session_dir / "tools.json").write_text(
            json.dumps(schemas, indent=2, allow_nan=False) + "\n"
        )
        if selection is not None:
            (session_dir / "tool_selection.json").write_text(
                json.dumps(selection.record(schemas), indent=2, allow_nan=False) + "\n"
            )

        # The callback is internal bookkeeping; finish returns only a submission receipt.
        toolkit.set_solved_fn(session.task_succeeded)
        description = getattr(session.episode, "description", None)
        prompt_integration = getattr(description, "integration", args.integration)
        prompt_builder = get_prompt_builder(prompt_integration)
        prompt_context = PromptContext(
            integration_name=prompt_integration,
            task="",
            action_model_name=session.action_model_name,
            episode=session.episode,
            session_dir=session_dir,
            observation_profile=profile_name,
            action_profile=getattr(args, "action_profile", None),
            tool_return_mode=args.tool_return_mode,
            resource_scheduling=args.resource_scheduling,
            tool_names=prompt_tool_names,
            skill_writing=getattr(args, "skill_writing", False),
        )
        contributions = tuple(getattr(observation_profile, "prompt_contributions", ()))

        def build_prompt(task: str):
            context = replace(prompt_context, task=task)
            base_prompt = prompt_builder(context)
            for provider in session.tool_providers:
                for contribution in provider.prompt_contributions:
                    base_prompt = base_prompt.append(
                        system=contribution.system, user=contribution.user, source=contribution.source,
                    )
            return assemble_prompt(base_prompt, context, contributions)

        return SessionAssembly(
            toolkit=toolkit,
            executor=executor,
            build_prompt=build_prompt,
            integration_session=session,
            session_dir=session_dir,
            artifacts=artifacts,
            observation_profile=observation_profile,
            telemetry=telemetry,
            allow_automatic_retry=integration_spec.allow_automatic_retry,
        )
    except BaseException as exc:
        # Stop producers and finish persistence before publishing even a failed startup.
        for resource in (executor, observation_profile, session, telemetry):
            close_resource(resource)
        artifact_storage = finish_artifacts(artifacts)
        try:
            record = build_run_result(
                args, agent_result=None, error_msg=str(exc), elapsed_s=0.0,
                outcome=EpisodeOutcome(success=None, accepted=False,
                                       failure_category="session_start_failed"),
                artifact_storage=artifact_storage,
            )
            write_result_json(session_dir, record)
        except Exception as write_error:
            logger.warning("Failed to record startup failure: %s", write_error)
        raise


def close_resource(resource) -> None:
    if resource is None or not callable(getattr(resource, "close", None)):
        return
    try:
        resource.close()
    except Exception as exc:  # noqa: BLE001 - one close failure must not block remaining cleanup.
        logger.warning("Failed to close run resource: %s", exc)


def finish_artifacts(artifacts: ArtifactStore) -> dict:
    """Finish the sole image writer without changing the official task outcome."""
    error = None
    try:
        artifacts.close()
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        logger.warning("Could not finish artifact storage: %s", error)
    status = artifacts.status()
    if error:
        status["close_error"] = error
    if status["failed"] or status["pending"]:
        logger.warning("Incomplete artifact storage: %s", status)
    return status


logger = get_logger("session")


class Session:
    """Synchronous ownership of one episode, its tools, optional agent and records.

    Use one calling thread per Session. Background tool jobs share its executor;
    cancel() may be called by another thread to request agent cancellation.
    """

    def __init__(self, config: SessionConfig):
        self._initialize(config.resolve())

    def _initialize(self, args, *, console=False):
        self._args = args
        self._console = console
        self._assembly = None
        self._agent = None
        self._agent_result = None
        self._agent_started = False
        self._closed = False
        self._started = time.time()
        self._deadline = None
        self._tools = None
        self._measurements = None
        self.result: RunResult | None = None
        self.events = EventBus()
        self._subscriptions = EventSubscriptions(self.events)

    @classmethod
    def _from_resolved(cls, args, *, console=False):
        session = cls.__new__(cls)
        session._initialize(args, console=console)
        return session

    @property
    def session_dir(self) -> Path:
        if self._assembly is None:
            raise RuntimeError("Session has not started")
        return self._assembly.session_dir

    @property
    def tools(self) -> ToolClient:
        self._require_active()
        return self._tools

    def _require_active(self):
        if self._closed or self._assembly is None:
            raise RuntimeError("Session is not open")
        if self._deadline is not None and time.monotonic() >= self._deadline:
            raise TimeoutError("Session episode budget exhausted")

    def start(self) -> Session:
        if self._closed:
            raise RuntimeError("Session is closed; create a new Session for a new episode")
        if self._assembly is not None:
            self._require_active()
            return self
        args = self._args
        self._started = time.time()
        from uni_robot_agent.config import public_config
        if not getattr(args, "_session_dir", None):
            args._session_dir = str(Path(args.output_dir or "outputs") /
                f"{args.integration}_{args.agent_backend or 'tools'}_{int(time.time())}_{uuid4().hex[:8]}")
        self._measurements = RunMeasurements(args._session_dir, public_config(args))
        timeout_s = args.episode_timeout_s or None
        if deadline := getattr(args, "_episode_deadline", None):
            timeout_s = max(0.001, deadline - time.time())
        if timeout_s:
            self._deadline = time.monotonic() + timeout_s
        try:
            if args.agent_backend is not None:
                self._agent = create_agent(args.agent_backend, model=args.model, timeout_s=timeout_s,
                    **get_agent_backend(args.agent_backend).config_options(args))
            self._assembly = assemble_session(args)
            assembly = self._assembly
            if self._console:
                self._subscriptions.add(ConsoleSubscriber())
            self._subscriptions.add(EventJsonlSubscriber(assembly.session_dir, artifacts=assembly.artifacts))
            self._subscriptions.add(self._measurements)
            assembly.toolkit.set_event_bus(self.events)
            connect_execution_events(assembly.executor, self.events)
            self._tools = ToolClient(assembly.executor, self._require_active)
            if args.instruction is None:
                args.instruction = assembly.integration_session.episode.instruction
            self.events.emit(SessionOpened(integration=args.integration, instruction=args.instruction,
                unobserved_model_roles=("actor_reviewer",) if getattr(args, "vla_review", False) else ()))
            return self
        except BaseException as error:
            if self._assembly is not None:
                self.finish(error=f"Session startup failed: {error}")
            else:
                self._closed = True
                try:
                    self._measurements.finish({"error": f"Session startup failed: {type(error).__name__}",
                                               "task_success": None, "accepted": False})
                except Exception:
                    logger.exception("Could not persist the failed startup attempt")
                close_resource(self._agent)
                close_resource(self._subscriptions)
            raise

    def run_agent(self) -> RunResult:
        if self._args.agent_backend is None:
            raise ValueError("run_agent requires SessionConfig.agent")
        if self._agent_started:
            raise RuntimeError("This session has already run an agent")
        try:
            self.start()
            self._require_active()
        except BaseException as error:
            self.finish(error=f"Agent startup failed: {error}")
            raise
        self._agent_started = True
        args, assembly = self._args, self._assembly
        error_msg = None
        try:
            logger.info("Task: %s", args.instruction)
            with assembly.telemetry.tracer.span("agent.run", agent_backend=args.agent_backend, model=args.model):
                self._agent_result = _run_agent(self._agent,
                    model_config=resolve_agent_model_config(args), context_config=resolve_context_config(args),
                    save_raw_request=args.telemetry == "debug", save_exchanges=args.save_exchanges,
                    artifacts=assembly.artifacts, tracer=assembly.telemetry.tracer,
                    prompt=assembly.build_prompt(args.instruction), task=args.instruction,
                    toolkit=assembly.toolkit, executor=assembly.executor, max_turns=args.max_turns,
                    event_bus=self.events, session_dir=assembly.session_dir)
            _log_agent_result(self._agent_result)
        except KeyboardInterrupt as error:
            self.cancel()
            error_msg = "Interrupted by user (Ctrl-C)"
            if isinstance(error, AgentInterrupted):
                self._agent_result = error.result
            else:
                self.events.emit(AgentRunFinished(success=False, message=error_msg))
            raise
        except Exception as error:
            error_msg = f"Agent crashed: {type(error).__name__}: {error}"
            logger.exception(error_msg)
            self.events.emit(ErrorEvent(message=error_msg))
            self.events.emit(AgentRunFinished(success=False, message=error_msg))
        finally:
            self.finish(error=error_msg)
        return self.result

    def subscribe(self, callback: Callable[[Event], None]) -> None:
        """Observe events; callback failures are recorded without aborting execution."""
        if self._closed:
            raise RuntimeError("Session is closed")
        self._subscriptions.add(callback)

    def cancel(self) -> None:
        """Request planner cancellation; this is not a physical emergency stop."""
        if self._agent is not None and not self._closed:
            self._agent.cancel("interrupted")

    def finish(self, *, error: str | None = None) -> RunResult | None:
        """Drain work, finalize once, persist and close owned connections."""
        if self._closed:
            return self.result
        self._closed = True
        if self._assembly is None:
            close_resource(self._subscriptions)
            return None
        assembly = self._assembly
        try:
            self.result = _finalize_run(assembly, self._subscriptions, self._args,
                self._agent_result, error, self._started, bus=self.events,
                measurements=self._measurements)
            return self.result
        finally:
            for resource in (self._agent, assembly.integration_session, assembly.telemetry):
                close_resource(resource)

    def close(self) -> None:
        self.finish()

    def __enter__(self) -> Session:
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.finish(error=f"{exc_type.__name__}: {exc}" if exc_type else None)


def run_session(config: SessionConfig) -> RunResult:
    """Run one configured agent and return its official episode result."""
    if config.agent is None:
        raise ValueError("run requires SessionConfig.agent; use Session for direct tools")
    with Session(config) as session:
        return session.run_agent()


def run(args) -> None:
    """CLI-owned presentation wraps the SDK session without changing its configuration."""
    monitor = None
    state, error = "finished", None
    try:
        if getattr(args, "monitor", False) and not getattr(args, "_monitor_parent", False):
            from uni_robot_agent.ui.web.runtime import LiveMonitor
            if not getattr(args, "_session_dir", None):
                args._session_dir = str(Path(args.output_dir or "outputs") / f"{args.integration}_{int(time.time())}_{uuid4().hex[:8]}")
            monitor = LiveMonitor(args, args._session_dir).start()
            if args.env_url:
                monitor.attach(args.env_url)
        _run_cli(args)
    except BaseException as exc:
        state = "cancelled" if isinstance(exc, KeyboardInterrupt) else "error"
        error = str(exc)
        raise
    finally:
        if monitor is not None:
            monitor.close(state, error)


def _run_cli(args) -> None:
    """CLI adapter retaining process deadlines and exit status outside the Python API."""
    if deadline := runtime_env("EPISODE_DEADLINE"):
        args._episode_deadline = float(deadline)
    logger.info("Starting: integration=%s, agent=%s, return=%s, scheduling=%s",
                args.integration, args.agent_backend, args.tool_return_mode, args.resource_scheduling)
    logger.info("Limits: max_turns=%s, timeout=%s, reasoning_effort=%s",
                args.max_turns, args.episode_timeout_s or "none", args.reasoning_effort)
    session = Session._from_resolved(args, console=True)
    try:
        session.start()
    except Exception as error:
        logger.error("Integration session startup failed: %s", error)
        raise SystemExit(1) from error
    try:
        result = session.run_agent()
    except KeyboardInterrupt:
        result = session.result
    if result is not None and result.exit_code:
        raise SystemExit(result.exit_code)


def _log_agent_result(result):
    if result.train_path and result.train_partial:
        logger.warning(
            "Partial training trajectory saved: %s (%s)",
            result.train_path,
            result.train_stop_reason,
        )
    elif result.train_path:
        logger.info("Training trajectory saved: %s", result.train_path)
    if result.train_error:
        if result.train_debug_path:
            logger.warning(
                "Training trajectory validation failed: %s; diagnostic trajectory saved: %s",
                result.train_error,
                result.train_debug_path,
            )
        else:
            logger.warning("Training trajectory not saved: %s", result.train_error)

    if result.success:
        logger.info(
            "Completed (turns=%d): %s", result.turns_used, result.message[:200]
        )
    else:
        logger.error(
            "Failed (turns=%d): %s",
            result.turns_used,
            (result.error or result.message)[:200],
        )

def _finalize_run(assembly, subscriptions, args, result, error_msg, started, *, bus=None, measurements=None) -> RunResult:
    """Drain physical work, seal the episode, then publish its result."""
    lifecycle_outcome: EpisodeOutcome | None = (
        EpisodeOutcome(
            success=None,
            accepted=False,
            retryable=True,
            failure_category="execution_uncertain",
        )
        if result is not None and result.cleanup_error
        else None
    )
    # Drain background physical actions before reading the official outcome;
    # otherwise terminated would describe an intermediate state.
    try:
        assembly.executor.close()
    except Exception as exc:
        logger.error("Runtime did not finish physical actions: %s", exc)
        lifecycle_outcome = lifecycle_outcome or EpisodeOutcome(
            success=None,
            accepted=False,
            retryable=True,
            failure_category="execution_uncertain",
        )

    close_resource(assembly.observation_profile)

    # Integration sessions own sealing and outcome reads; finalize before closing resources.
    # Persistent simulators and hardware both require this order, not agent-reported success.
    storage_errors = None
    try:
        finalized = assembly.integration_session.finalize(
            agent_result=result,
            error=error_msg,
        )
        # Keep file errors even when lifecycle policy replaces the evaluation outcome.
        storage_errors = finalized.metadata.get("storage_errors")
        outcome = lifecycle_outcome or finalized
    except Exception as e:  # noqa: BLE001 - persist results and reclaim resources even on failure.
        logger.warning("Integration session finalization failed: %s", e)
        outcome = lifecycle_outcome or EpisodeOutcome(
            success=None,
            accepted=False,
            failure_category="session_finalize_failed",
        )

    provider_error = getattr(result, "provider_error", None)
    if provider_error and lifecycle_outcome is None:
        # Mark provider failures as infrastructure errors; the integration retry policy applies below.
        outcome = EpisodeOutcome(
            success=None,
            accepted=False,
            retryable=True,
            failure_category="provider_transport",
            metadata=outcome.metadata,
        )
    # Physical episodes must not be replayed by provider/lifecycle overrides.
    if not assembly.allow_automatic_retry and outcome.retryable:
        outcome = replace(outcome, retryable=False)
    task_success = outcome.success

    if bus is not None:
        try:
            bus.emit(SessionClosed(task_success=outcome.success, accepted=outcome.accepted,
                                   failure_category=outcome.failure_category))
        except Exception as error:
            # User callbacks cannot prevent cleanup or erase the official verdict.
            storage_errors = {**(storage_errors or {}), "session_events": str(error)}
    close_resource(subscriptions)
    storage_errors = {**(storage_errors or {}), **subscriptions.errors} or None
    artifact_storage = finish_artifacts(assembly.artifacts)
    finally_error = None
    record = build_run_result(
            args,
            agent_result=result,
            error_msg=error_msg,
            elapsed_s=time.time() - started,
            outcome=outcome,
            storage_errors=storage_errors,
            artifact_storage=artifact_storage,
        )
    try:
        if measurements is not None:
            try:
                measurements.finish(record)
                record["measurements_path"] = "measurements.json"
            except Exception as exc:
                record.setdefault("storage_errors", {})["measurements"] = type(exc).__name__
        record = write_result_json(assembly.session_dir, record)
    except Exception as e:  # noqa: BLE001 - result-write failure must not bypass resource cleanup.
        finally_error = e

    if finally_error:
        logger.error("Failed to write result.json: %s", finally_error)
        record.setdefault("storage_errors", {})["result.json"] = str(finally_error)

    exit_code = _result_exit_code(
        outcome=outcome,
        task_success=task_success,
        agent_result=result,
        error_msg=error_msg,
    )
    if finally_error and not exit_code:
        exit_code = 1
    return RunResult(assembly.session_dir, record, exit_code)


def run_tui(args) -> None:
    """Start the single-task TUI."""
    try:
        from uni_robot_agent.ui.textual.app import UniRobotAgentApp
    except ImportError:
        print("TUI dependencies are missing. Reinstall uni-robot-agent with pip or uv tool.", file=sys.stderr)
        sys.exit(1)

    agent_options = get_agent_backend(args.agent_backend).config_options(args)
    agent_options["model_config"] = resolve_agent_model_config(args)
    agent_options["context_config"] = resolve_context_config(args)
    agent_options["save_raw_request"] = args.telemetry == "debug"
    agent_options["save_exchanges"] = args.save_exchanges

    assembly = assemble_session(args)

    bus = EventBus()
    subscriptions = EventSubscriptions(bus)

    try:
        subscriptions.add(EventJsonlSubscriber(assembly.session_dir, artifacts=assembly.artifacts))
        assembly.toolkit.set_event_bus(bus)
        app = UniRobotAgentApp(
            integration_name=args.integration,
            agent_backend=args.agent_backend,
            model=args.model,
            agent_options=agent_options,
            toolkit=assembly.toolkit,
            executor=assembly.executor,
            prompt_builder=assembly.build_prompt,
            bus=bus,
            session_dir=assembly.session_dir,
            artifacts=assembly.artifacts,
            tracer=assembly.telemetry.tracer,
            max_turns=args.max_turns,
        )
        try:
            app.run()
        finally:
            app.close()
    finally:
        for resource in (
            assembly.executor,
            assembly.observation_profile,
            assembly.integration_session,
            assembly.telemetry,
            subscriptions,
        ):
            close_resource(resource)
        finish_artifacts(assembly.artifacts)


class SingleTaskSession:
    """One task per environment session, independent of the terminal frontend."""

    def __init__(
        self,
        *,
        agent_backend,
        model,
        agent_options,
        toolkit,
        executor,
        prompt_builder,
        bus,
        session_dir,
        artifacts,
        tracer,
        max_turns=50,
    ):
        self._agent_backend = agent_backend
        self._model = model
        options = dict(agent_options or {})
        self._model_config = options.pop("model_config", None)
        self._context_config = options.pop("context_config", {})
        self._save_raw_request = options.pop("save_raw_request", False)
        self._save_exchanges = options.pop("save_exchanges", True)
        self._agent = create_agent(agent_backend, model=model, **options)
        self._toolkit = toolkit
        self._executor = executor
        self._prompt_builder = prompt_builder
        self._bus = bus
        self._session_dir = Path(session_dir).resolve()
        self._artifacts = artifacts
        self._tracer = tracer
        self._max_turns = max_turns
        self._started = False
        self.turns = 0

    def run(self, task: str) -> AgentResult:
        if self._started:
            raise RuntimeError("This session has already run a task; start a new session")
        self._started = True
        with self._tracer.span(
            "agent.run", agent_backend=self._agent_backend, model=self._model
        ):
            result = _run_agent(
                self._agent,
                model_config=self._model_config,
                context_config=self._context_config,
                save_raw_request=self._save_raw_request,
                save_exchanges=self._save_exchanges,
                artifacts=self._artifacts,
                tracer=self._tracer,
                prompt=self._prompt_builder(task),
                task=task,
                toolkit=self._toolkit,
                executor=self._executor,
                max_turns=self._max_turns,
                event_bus=self._bus,
                session_dir=self._session_dir,
            )
        self.turns = result.turns_used
        return result

    def cancel(self) -> None:
        self._agent.cancel("tui_closed")

    def close(self) -> None:
        self._agent.close()


def _result_exit_code(*, outcome, task_success, agent_result, error_msg) -> int:
    """Return 75 for retryable outcomes, 0 for success, and 1 for other failures."""
    if outcome.retryable:
        return 75
    if not outcome.accepted:
        return 1
    if task_success is None:
        ok = not error_msg and (agent_result is None or agent_result.success)
    else:
        ok = task_success and not error_msg
    return 0 if ok else 1


def _run_agent(
    agent,
    *,
    model_config,
    context_config,
    artifacts=None,
    save_raw_request=False,
    save_exchanges=True,
    tracer=None,
    **request,
):
    """Own the model endpoint for one SDK run and finalize its immutable records."""
    from uni_robot_agent.provider import ProviderGateway

    session_dir = request.get("session_dir")
    event_bus = request.get("event_bus")
    gateway = None
    model_runtime = None
    if model_config is not None and model_config.base_url:
        model_runtime = agent.model_runtime(model_config.adapter, event_bus)
        gateway = ProviderGateway(
            model_config.base_url,
            adapter=model_runtime.adapter,
            protocol=model_runtime.protocol,
            trace_dir=Path(session_dir) / model_runtime.trace_directory if session_dir else None,
            artifacts=artifacts,
            request_timeout_s=model_config.request_timeout_s,
            max_retries=model_config.max_retries,
            retry_backoff_initial_s=model_config.retry_backoff_initial_s,
            model_parallel_tool_calls=model_config.parallel_tool_calls,
            save_raw_request=save_raw_request,
            save_exchanges=save_exchanges,
            tracer=tracer,
            model_binding=model_runtime.binding,
            event_bus=event_bus,
            **context_config,
        )
    result = None
    interruption = None
    cleanup_errors = []

    def cancel_agent(reason):
        try:
            agent.cancel(reason)
        except Exception as exc:
            cleanup_errors.append(f"Cancellation failed: {exc}")

    try:
        endpoint = gateway.start() if gateway is not None else None
        result = agent.run(model_endpoint=endpoint, **request)
        if result.stop_reason or result.error:
            cancel_agent(result.stop_reason or "agent_error")
    except AgentInterrupted as error:
        result = error.result
        interruption = error
        cancel_agent("interrupted")
    except BaseException:
        cancel_agent("interrupted")
        raise
    finally:
        try:
            agent.close()
        except Exception as exc:
            cleanup_errors.append(f"Agent cleanup failed: {exc}")
        if gateway is not None:
            try:
                gateway.stop()
            except Exception as exc:
                cleanup_errors.append(f"Provider cleanup failed: {exc}")
    if cleanup_errors:
        result.cleanup_error = "; ".join(cleanup_errors)
        result.error = result.error or result.cleanup_error
        result.train_error = result.cleanup_error
        logger.error(result.cleanup_error)
        if event_bus is not None:
            event_bus.emit(ErrorEvent(message=result.cleanup_error))
        if gateway is not None:
            gateway.persist_report(
                {
                    "cleanup_error": result.cleanup_error,
                    "records_frozen": False,
                    "agent_error": result.error,
                    "train_error": result.train_error,
                    "generated_at": time.time(),
                }
            )
    elif gateway is not None:
        try:
            model_runtime.finalize_records(gateway, result, session_dir, event_bus)
        except Exception as exc:
            result.train_error = f"Model record finalization failed: {exc}"
            result.error = result.error or result.train_error
            logger.exception("Model record finalization failed")
            if event_bus is not None:
                event_bus.emit(ErrorEvent(message=result.train_error))
    else:
        agent.finalize_without_provider(result, session_dir)
    task_success = None
    solved = getattr(request.get("toolkit"), "task_succeeded", None)
    if callable(solved):
        try:
            task_success = solved()
        except Exception as exc:
            logger.warning("Failed to read task_success: %s", exc)
    if event_bus is not None:
        event_bus.emit(
            AgentRunFinished(
                success=result.success,
                task_success=task_success,
                turns=result.turns_used,
                steps=result.steps_used,
                tool_calls=result.tool_calls,
                message=result.error or result.message,
                elapsed_s=result.elapsed_s,
            )
        )
    if interruption is not None:
        raise interruption
    return result
