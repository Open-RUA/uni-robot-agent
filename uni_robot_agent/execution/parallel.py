"""Execution runtime with background jobs.

Unconfigured tools run directly and synchronously. Integration specs independently
choose resource scheduling and whether callers wait or receive a job handle.
"""

from __future__ import annotations

from dataclasses import replace
from contextvars import copy_context
import threading
import uuid
from collections.abc import Iterable
from typing import Any

from uni_robot_agent.execution import (
    QueueSnapshotListener,
    ExecutionResult,
    ToolLifecycleEvent,
    ToolLifecycleListener,
    ToolExecutor,
)
from uni_robot_agent.execution.scheduler import JobRegistry, ResourceScheduler, ToolExecutionPolicy
from uni_robot_agent.telemetry import NullTracer
from uni_robot_agent.telemetry.logging import get_logger
from uni_robot_agent.tools.jobs import JobTools
from uni_robot_agent.tools.toolkit import Toolkit, ToolOutput

logger = get_logger("runtime.parallel")


class ParallelToolExecutor(ToolExecutor):
    """Add background execution and resource scheduling around existing Toolkit handlers."""

    def __init__(
        self,
        toolkit: Toolkit,
        policies: dict[str, ToolExecutionPolicy] | None = None,
        *,
        max_parallel: int = 4,
        listeners: Iterable[ToolLifecycleListener] | None = None,
        tracer=None,
        tool_return_mode: str = "background",
        resource_scheduling: str = "per-resource",
        control_tool_names: Iterable[str] | None = None,
    ) -> None:
        if tool_return_mode not in {"immediate", "background"}:
            raise ValueError(f"Invalid tool return mode: {tool_return_mode}")
        if resource_scheduling not in {"per-resource", "global-serial"}:
            raise ValueError(f"Invalid resource scheduling: {resource_scheduling}")
        self._tool_return_mode = tool_return_mode
        self._resource_scheduling = resource_scheduling
        self._toolkit = toolkit
        self._policies = policies or {}
        self._listeners = tuple(listeners or ())
        self._tracer = tracer or NullTracer()
        self._queue_lock = threading.Lock()
        self._queue_listeners: list[QueueSnapshotListener] = []
        self._job_calls = {}
        self._reported_cancellations = set()
        self._registry = JobRegistry()
        self._scheduler = ResourceScheduler(
            self._registry,
            max_parallel=max_parallel,
            on_change=self._emit_queue_snapshot,
        )
        self._controls = JobTools(self._registry, self._scheduler)
        available_controls = set(self._controls.get_tool_names())
        if control_tool_names is None:
            control_tool_names = available_controls if tool_return_mode == "background" else ()
        self._control_names = set(control_tool_names)
        if unknown := self._control_names - available_controls:
            raise ValueError(f"Unknown control tools: {sorted(unknown)}")
        overlap = self._control_names.intersection(toolkit.get_tool_names())
        if overlap:
            raise ValueError(f"tool names reserved by runtime controls: {sorted(overlap)}")

    def get_tool_schemas(self) -> list[dict[str, Any]]:
        return self._toolkit.get_tool_schemas() + [
            spec for spec in self._controls.get_tool_schemas()
            if spec["name"] in self._control_names
        ]

    def call(self, name: str, arguments: dict[str, Any]) -> ExecutionResult:
        return self._call(name, arguments)

    def submit(self, name: str, arguments: dict[str, Any]) -> ExecutionResult:
        return self._call(name, arguments, background=True)

    def job_result(self, job_id: str, timeout: float | None = 0) -> ExecutionResult:
        job = self._registry.wait(job_id, timeout)
        if job is None:
            raise KeyError(job_id)
        if job.status in {"queued", "running"}:
            return ExecutionResult.accepted(job.job_id, job.resource_key, job.status, job.queue_position)
        return self._controls.result_for(job, include_job_metadata=False)

    def cancel_job(self, job_id: str) -> bool:
        if self._registry.get(job_id) is None:
            raise KeyError(job_id)
        return self._scheduler.cancel(job_id)

    def _call(self, name: str, arguments: dict[str, Any], *, background=False) -> ExecutionResult:
        if name in self._control_names:
            return self._control_call(name, arguments)

        # Reject hidden names before scheduling or notifying observation listeners.
        if name not in self._toolkit.get_tool_names():
            return self._control_call(name, arguments, unknown=True)

        policy = self._policies.get(name, ToolExecutionPolicy())
        # Foreground overrides waiting only; resource ownership remains independent.
        if self._tool_return_mode == "immediate":
            policy = replace(policy, return_mode="immediate")
        if background:
            policy = replace(policy, return_mode="background")
        if self._resource_scheduling == "global-serial":
            policy = replace(policy, resource_key="__global__", concurrency="exclusive")
        # concurrency chooses execution placement; mode chooses whether the caller waits.
        if policy.return_mode == "immediate" and policy.concurrency == "parallel":
            return ExecutionResult.completed(
                self._execute_with_events(name, arguments, policy)
            )

        job = self._registry.create(name, arguments, policy.resource_key)
        parent_context = self._tracer.capture()
        call_context = copy_context()
        from uni_robot_agent.execution.correlation import current_call
        with self._queue_lock:
            self._job_calls[job.job_id] = current_call()
        self.emit_execution("queued", job.job_id, name, arguments, policy.resource_key, job_id=job.job_id)
        try:
            self._scheduler.submit(
                job,
                lambda: call_context.run(self._execute_scheduled,
                    name, arguments, policy, job.job_id, parent_context,
                ),
                concurrency=policy.concurrency,
            )
        except Exception as exc:
            self._registry.fail(job.job_id, f"{type(exc).__name__}: {exc}")
            self.emit_execution("finished", job.job_id, name, arguments, policy.resource_key,
                                job_id=job.job_id, success=False, error=str(exc))
            if policy.return_mode == "background":
                return self._controls.result_for(job)
            result = self._controls.result_for(job, include_job_metadata=False)
            self._registry.discard(job.job_id)
            return result

        if policy.return_mode == "immediate":
            completed_job = self._registry.wait(job.job_id)
            result = self._controls.result_for(
                completed_job,
                include_job_metadata=False,
            )
            self._registry.discard(job.job_id)
            return result

        # Read the registry: a submitted job may already be running or complete.
        # Inferring state from its static policy would expose stale state to the agent.
        return ExecutionResult.accepted(
            job.job_id,
            policy.resource_key,
            job.status,
            job.queue_position,
        )

    def _control_call(self, name, arguments, *, unknown=False):
        from uni_robot_agent.execution import execution_context
        execution_id = uuid.uuid4().hex
        self.emit_execution("queued", execution_id, name, arguments, "control")
        output = None
        with execution_context(execution_id):
            self.emit_execution("started", execution_id, name, arguments, "control")
            try:
                if unknown:
                    output = self._toolkit.execute_tool(name, arguments)
                    return ExecutionResult.completed(output)
                with self._tracer.span("tool.control", tool=name):
                    result = self._controls.call(name, arguments)
                output = result.output or ToolOutput(extra=result.to_dict())
                assessment = None
                # A query can return an original failed job. Its interface did
                # succeed; the original execution retains its own assessment.
                ids = ([arguments.get("job_id")] if name in {"get_tool_result", "wait_for_tool"}
                       else arguments.get("job_ids") if name == "wait_for_tools" else None)
                if (isinstance(ids, list) and ids and not output.call_error
                        and all(isinstance(i, str) and self._registry.get(i) is not None for i in ids)):
                    from uni_robot_agent.tools.assessment import VERSION
                    assessment = {"version": VERSION, "status": "success", "fault_owner": "none",
                                  "reason": "job_query_returned", "penalty_eligible": False}
                self._toolkit.record_result(name, arguments, output, call_assessment=assessment)
                return result
            finally:
                self.emit_execution("finished", execution_id, name, arguments, "control",
                    success=output.success if output is not None else False,
                    error=output.error if output is not None else "control_exception")

    def _execute(self, name: str, arguments: dict[str, Any]) -> ToolOutput:
        return self._toolkit.execute_tool(name, arguments)

    def _execute_with_events(
        self,
        name: str,
        arguments: dict[str, Any],
        policy: ToolExecutionPolicy,
        job_id: str | None = None,
    ) -> ToolOutput:
        """Execute and notify optional extensions without changing tool outcomes on listener failure."""
        execution_id = job_id or uuid.uuid4().hex[:12]
        self._emit(ToolLifecycleEvent(
            phase="started",
            execution_id=execution_id,
            tool_name=name,
            arguments=dict(arguments),
            resource_key=policy.resource_key,
            mutates_world=policy.mutates_world,
            job_id=job_id,
        ))
        def execute() -> ToolOutput:
            output: ToolOutput | None = None
            error: Exception | None = None
            try:
                from uni_robot_agent.execution import execution_context
                with execution_context(execution_id):
                    output = self._execute(name, arguments)
                return output
            except Exception as exc:
                error = exc
                raise
            finally:
                self._emit(ToolLifecycleEvent(
                    phase="finished",
                    execution_id=execution_id,
                    tool_name=name,
                    arguments=dict(arguments),
                    resource_key=policy.resource_key,
                    mutates_world=policy.mutates_world,
                    job_id=job_id,
                    output=output,
                    error=f"{type(error).__name__}: {error}" if error else None,
                ))

        if not self._tracer.enabled:
            return execute()
        with self._tracer.span(
            "tool.execute",
            tool=name,
            provider=self._toolkit.get_tool_provider_id(name),
            resource=policy.resource_key,
            mode=policy.return_mode,
            job_id=job_id,
        ):
            return execute()

    def _execute_scheduled(
        self,
        name: str,
        arguments: dict[str, Any],
        policy: ToolExecutionPolicy,
        job_id: str,
        parent_context,
    ) -> ToolOutput:
        """Propagate submitter identity explicitly into scheduler threads."""
        with self._tracer.use_context(parent_context):
            job = self._registry.get(job_id)
            if (
                self._tracer.enabled
                and job is not None
                and job.started_mono_ns is not None
            ):
                self._tracer.record_duration(
                    "runtime.queue_wait",
                    job.started_mono_ns - job.created_mono_ns,
                    attributes={
                        "tool": name,
                        "resource": policy.resource_key,
                        "job_id": job_id,
                    },
                )
            return self._execute_with_events(name, arguments, policy, job_id)

    def _emit(self, event: ToolLifecycleEvent) -> None:
        self.emit_execution(event.phase, event.execution_id, event.tool_name, event.arguments,
            event.resource_key, job_id=event.job_id,
            success=(event.output.success if event.output is not None else False) if event.phase == "finished" else None,
            error=event.error)
        for listener in self._listeners:
            try:
                listener.on_tool_event(event)
            except Exception:
                logger.exception(
                    "Tool lifecycle listener failed (ignored): phase=%s tool=%s",
                    event.phase,
                    event.tool_name,
                )

    def add_queue_listener(self, listener: QueueSnapshotListener) -> None:
        with self._queue_lock:
            self._queue_listeners.append(listener)

    def _emit_queue_snapshot(self, snapshot: dict[str, Any]) -> None:
        with self._queue_lock:
            listeners = tuple(self._queue_listeners)
            cancelled = [job for job in self._registry.list()
                         if job.status == "cancelled" and job.job_id not in self._reported_cancellations]
            self._reported_cancellations.update(job.job_id for job in cancelled)
        from uni_robot_agent.execution.correlation import call_context
        for job in cancelled:
            identity = self._job_calls.get(job.job_id, {})
            with call_context(identity.get("call_id", ""), identity.get("batch_id", "")):
                self.emit_execution("cancelled", job.job_id, job.tool_name, job.arguments,
                                    job.resource_key, job_id=job.job_id)
        for listener in listeners:
            try:
                listener(snapshot)
            except Exception:
                logger.exception("Tool queue listener failed (ignored)")

    @property
    def tool_execution_seq(self) -> int:
        return self._toolkit.tool_execution_seq

    def close(self) -> None:
        # Wait for physical actions before returning; the caller closes the environment
        # client next, which would otherwise disconnect RPCs still needed by those actions.
        unfinished = self._scheduler.close()
        if unfinished:
            raise RuntimeError(
                f"Runtime closed with unfinished background jobs: {', '.join(unfinished)}"
            )
