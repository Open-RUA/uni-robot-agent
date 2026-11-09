"""Declarative control tools for background jobs in the parallel runtime."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

from uni_robot_agent.execution.scheduler import JobRecord, JobRegistry, ResourceScheduler
from uni_robot_agent.tools.toolkit import (
    ToolDef, ToolOutput, ToolProvider, ToolSpec, bind_tools, schema,
)

if TYPE_CHECKING:
    from uni_robot_agent.execution import ExecutionResult


def tool_specs() -> list[ToolSpec]:
    return [
        ToolSpec("get_tool_result",
            "Get the current status or final result of a background tool job.",
            schema({"job_id": {"type": "string"}}, required=("job_id",)), readonly=True),
        ToolSpec("wait_for_tool",
            "Wait for a background tool job; timeout limits waiting and does not interrupt it.",
            schema({
                "job_id": {"type": "string"},
                "timeout": {"type": "number", "minimum": 0},
            }, required=("job_id",)), readonly=True),
        ToolSpec("wait_for_tools",
            "Wait for several background tool jobs with one shared timeout and "
            "return their current or final results in input order.",
            schema({
                "job_ids": {
                    "type": "array", "items": {"type": "string"},
                    "minItems": 1, "uniqueItems": True,
                },
                "timeout": {"type": "number", "minimum": 0},
            }, required=("job_ids",)), readonly=True),
        ToolSpec("list_tool_jobs", "List background tool jobs and their statuses.",
            schema({}), readonly=True),
        ToolSpec("cancel_tool",
            "Cancel a queued background tool job; running physical actions are not forcefully interrupted.",
            schema({"job_id": {"type": "string"}}, required=("job_id",))),
        ToolSpec("queue_status", "Inspect resource queues and background job statuses.",
            schema({"resource": {"type": "string"}}), readonly=True),
        ToolSpec("flush_queue", "Cancel all queued jobs for a specified resource.",
            schema({"resource": {"type": "string"}}, required=("resource",))),
    ]


class JobTools(ToolProvider):
    """Tool declarations and handlers for inspecting the parallel scheduler."""

    provider_id = "jobs"

    def __init__(self, registry: JobRegistry, scheduler: ResourceScheduler, *, on_result_read=None) -> None:
        self._on_result_read = on_result_read
        self._registry = registry
        self._scheduler = scheduler
        tools = bind_tools(
            tool_specs(),
            provider_id=self.provider_id,
            handlers={
                "get_tool_result": self.get_tool_result,
                "wait_for_tool": self.wait_for_tool,
                "wait_for_tools": self.wait_for_tools,
                "list_tool_jobs": self.list_tool_jobs,
                "cancel_tool": self.cancel_tool,
                "queue_status": self.queue_status,
                "flush_queue": self.flush_queue,
            },
        )
        self._tools = {item.name: item for item in tools}

    def get_tools(self) -> list[ToolDef]:
        return list(self._tools.values())

    def get_tool_schemas(self) -> list[dict[str, Any]]:
        return [item.spec for item in self._tools.values()]

    def get_tool_names(self) -> list[str]:
        return list(self._tools)

    def call(self, name: str, arguments: dict[str, Any]) -> ExecutionResult:
        tool_def = self._tools.get(name)
        if tool_def is None:
            return self._completed(ToolOutput(
                success=False,
                error=f"Unknown parallel control tool: {name}",
            ))
        result = tool_def.handler(**arguments)
        return self._completed(result) if isinstance(result, ToolOutput) else result

    def get_tool_result(self, job_id: str) -> ExecutionResult:
        return self.result_for(
            self._registry.get(job_id),
            missing_id=job_id,
        )

    def wait_for_tool(
        self,
        job_id: str,
        timeout: float | None = None,
    ) -> ExecutionResult:
        job = self._registry.wait(
            job_id,
            float(timeout) if timeout is not None else None,
        )
        return self.result_for(job, missing_id=job_id)

    def wait_for_tools(
        self,
        job_ids: list[str],
        timeout: float | None = None,
    ) -> ExecutionResult:
        # Share one deadline across jobs so N waits do not multiply the timeout.
        deadline = None if timeout is None else time.monotonic() + float(timeout)
        jobs: list[JobRecord | None] = []
        for job_id in job_ids:
            remaining = (
                None if deadline is None else max(0.0, deadline - time.monotonic())
            )
            jobs.append(self._registry.wait(job_id, remaining))

        results: list[dict[str, Any]] = []
        images = []
        all_done = True
        all_succeeded = True
        for job_id, job in zip(job_ids, jobs, strict=True):
            if job is None:
                results.append({
                    "job_id": job_id,
                    "status": "missing",
                    "error": f"Unknown job: {job_id}",
                })
                all_done = False
                all_succeeded = False
                continue

            item = job.summary()
            if job.status in {"queued", "running"}:
                all_done = False
                all_succeeded = False
            else:
                del item["status"]
                result = self.result_for(job, include_job_metadata=False).output
                if result is not None:
                    item["output"] = result.to_dict(for_model=True)
                    item["image_count"] = len(result.images)
                    images.extend(result.images)
                    all_succeeded = all_succeeded and result.success
                else:
                    all_succeeded = False
            results.append(item)

        return self._completed(ToolOutput(
            success=all_succeeded,
            extra={"all_done": all_done, "results": results},
            images=images,
        ))

    def list_tool_jobs(self) -> ExecutionResult:
        jobs = [job.summary() for job in self._registry.list()]
        return self._completed(ToolOutput(extra={"jobs": jobs}))

    def cancel_tool(self, job_id: str) -> ExecutionResult:
        job = self._registry.get(job_id)
        if job is None:
            return self._missing_job(job_id)
        cancelled = self._scheduler.cancel(job_id)
        if cancelled and self._on_result_read is not None:
            self._on_result_read(job)
        return self._completed(ToolOutput(
            success=cancelled,
            error=None if cancelled else f"Job is already {job.status}",
            extra={
                "job_id": job_id,
                "status": "cancelled" if cancelled else job.status,
            },
        ))

    def queue_status(self, resource: str | None = None) -> ExecutionResult:
        return self._completed(
            ToolOutput(extra=self._scheduler.queue_status(resource))
        )

    def flush_queue(self, resource: str) -> ExecutionResult:
        cancelled_jobs = self._scheduler.cancel_pending(resource, None, reason="explicit_queue_flush")
        for job in cancelled_jobs:
            if self._on_result_read is not None:
                self._on_result_read(job)
        return self._completed(ToolOutput(extra={
            "resource": resource, "cancelled": len(cancelled_jobs),
            "cancelled_jobs": [dict(job.summary(), **job.cancellation) for job in cancelled_jobs],
        }))

    def result_for(
        self,
        job: JobRecord | None,
        *,
        missing_id: str | None = None,
        include_job_metadata: bool = True,
    ) -> ExecutionResult:
        """Translate scheduler state into the public runtime result contract."""
        if job is None:
            return self._missing_job(missing_id)
        if job.status in {"queued", "running"}:
            return self._execution_result(
                status="running",
                job_id=job.job_id,
                resource_key=job.resource_key,
                queue_position=job.queue_position,
            )
        if job.status == "succeeded":
            output = job.result
            if not isinstance(output, ToolOutput):
                output = ToolOutput(success=False, error="Background job returned an invalid result")
            if include_job_metadata:
                output = ToolOutput(
                    success=output.success,
                    error=output.error,
                    obs=output.obs,
                    extra={
                        **output.extra,
                        "job_id": job.job_id,
                    },
                    images=output.images,
                    action=output.action,
                )
        else:
            extra = (
                {"job_id": job.job_id}
                if include_job_metadata else {}
            )
            output = ToolOutput(
                success=False,
                error=job.error or f"Job {job.status}",
                extra={**extra, **job.cancellation},
            )
        if self._on_result_read is not None:
            self._on_result_read(job)
        return self._completed(output)

    def _missing_job(self, job_id: str | None) -> ExecutionResult:
        return self._completed(ToolOutput(
            success=False,
            error=f"Unknown job: {job_id}",
        ))

    @staticmethod
    def _execution_result(**kwargs) -> ExecutionResult:
        # Import lazily so ``uni_robot_agent.tools.jobs`` remains safe to import first.
        from uni_robot_agent.execution import ExecutionResult

        return ExecutionResult(**kwargs)

    @classmethod
    def _completed(cls, output: ToolOutput) -> ExecutionResult:
        return cls._execution_result(status="completed", output=output)
