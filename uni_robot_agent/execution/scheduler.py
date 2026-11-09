"""Generic background execution primitives.

Own job registration, resource queues, and thread lifecycles without depending
on Toolkit, MCP, or environments. ParallelToolExecutor connects them to tool calls.
"""

from __future__ import annotations

import threading
import time
import uuid
from collections import deque
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Any, Literal

JobStatus = Literal["queued", "running", "succeeded", "failed", "cancelled"]


@dataclass(frozen=True)
class ToolExecutionPolicy:
    """Tool return mode and execution placement; physical worlds normally use exclusive."""

    resource_key: str = "episode"
    return_mode: Literal["immediate", "background"] = "immediate"
    concurrency: Literal["exclusive", "parallel"] = "parallel"
    # Optional extensions use this to advance world versions; scheduling ignores it.
    mutates_world: bool = False
    # Background mode: a new call on this resource is refused until this tool's last
    # result has been read with wait_for_tool or get_tool_result.
    require_result_read: bool = False
    # An incomplete result (ToolOutput.action) cancels queued world-mutating work on
    # the resource and refuses new work until the model replans; see execution/replan.py.
    replan_on_failure: bool = False
    # A successful result of this tool is a new observation that a replan can start from.
    replan_observation: bool = False
    # Ending the episode stays allowed while a replan is required.
    ends_episode: bool = False


@dataclass
class JobRecord:
    job_id: str
    tool_name: str
    arguments: dict[str, Any]
    resource_key: str
    status: JobStatus = "queued"
    created_at: float = field(default_factory=time.time)
    created_mono_ns: int = field(default_factory=time.monotonic_ns, repr=False)
    started_at: float | None = None
    started_mono_ns: int | None = field(default=None, repr=False)
    finished_at: float | None = None
    result: Any = None
    error: str | None = None
    queue_position: int | None = None
    cancellation: dict[str, Any] = field(default_factory=dict)
    done: threading.Event = field(default_factory=threading.Event, repr=False)

    def summary(self) -> dict[str, Any]:
        return {
            "job_id": self.job_id,
            "tool_name": self.tool_name,
            "resource": self.resource_key,
            "status": self.status,
            "queue_position": self.queue_position,
            "created_at": self.created_at,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "error": self.error,
        }


class JobRegistry:
    """Thread-safe job registry owning all state transitions."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._jobs: dict[str, JobRecord] = {}

    def create(
        self,
        tool_name: str,
        arguments: dict[str, Any],
        resource_key: str,
    ) -> JobRecord:
        job = JobRecord(
            job_id=uuid.uuid4().hex[:12],
            tool_name=tool_name,
            arguments=dict(arguments),
            resource_key=resource_key,
        )
        with self._lock:
            self._jobs[job.job_id] = job
        return job

    def get(self, job_id: str) -> JobRecord | None:
        with self._lock:
            return self._jobs.get(job_id)

    def discard(self, job_id: str) -> None:
        """Remove completed internal jobs created only for synchronous queueing."""
        with self._lock:
            job = self._jobs.get(job_id)
            if job is not None and job.done.is_set():
                self._jobs.pop(job_id, None)

    def list(self, resource_key: str | None = None) -> list[JobRecord]:
        with self._lock:
            jobs = list(self._jobs.values())
        if resource_key is not None:
            jobs = [job for job in jobs if job.resource_key == resource_key]
        return sorted(jobs, key=lambda job: job.created_at)

    def mark_running(self, job_id: str) -> bool:
        with self._lock:
            job = self._jobs[job_id]
            if job.status == "cancelled":
                return False
            job.status = "running"
            job.started_at = time.time()
            job.started_mono_ns = time.monotonic_ns()
            job.queue_position = None
            return True

    def complete(self, job_id: str, result: Any) -> None:
        with self._lock:
            job = self._jobs[job_id]
            job.status = "succeeded"
            job.result = result
            job.finished_at = time.time()
            job.done.set()

    def fail(self, job_id: str, error: str) -> None:
        with self._lock:
            job = self._jobs[job_id]
            job.status = "failed"
            job.error = error
            job.finished_at = time.time()
            job.done.set()

    def cancel_if_queued(self, job_id: str, *, reason: str = "cancelled_by_request") -> bool:
        with self._lock:
            job = self._jobs[job_id]
            if job.status != "queued":
                return False
            job.status = "cancelled"
            job.cancellation = {"no_execution": True, "execution_status": "cancelled_before_dispatch", "reason": reason}
            job.queue_position = None
            job.finished_at = time.time()
            job.done.set()
            return True

    def set_queue_positions(self, job_ids: list[str]) -> None:
        with self._lock:
            for position, job_id in enumerate(job_ids, 1):
                job = self._jobs.get(job_id)
                if job and job.status == "queued":
                    job.queue_position = position

    def wait(self, job_id: str, timeout: float | None = None) -> JobRecord | None:
        job = self.get(job_id)
        if job is None:
            return None
        job.done.wait(timeout)
        return job


class ResourceScheduler:
    """Provide exclusive FIFO resource queues and a bounded pool for parallel work."""

    def __init__(
        self,
        registry: JobRegistry,
        *,
        max_parallel: int = 4,
        on_change: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self._registry = registry
        self._on_change = on_change
        self._parallel = ThreadPoolExecutor(
            max_workers=max_parallel,
            thread_name_prefix="uni_robot_agent-tool",
        )
        self._lock = threading.RLock()
        self._queues: dict[str, deque[tuple[str, Callable[[], Any]]]] = {}
        self._workers: dict[str, threading.Thread] = {}
        self._wakeups: dict[str, threading.Condition] = {}
        self._closed = False

    def submit(
        self,
        job: JobRecord,
        fn: Callable[[], Any],
        *,
        concurrency: Literal["exclusive", "parallel"],
    ) -> None:
        if concurrency == "parallel":
            with self._lock:
                if self._closed:
                    raise RuntimeError("resource scheduler is closed")
            self._parallel.submit(self._run, job.job_id, fn)
            self._notify()
            return

        with self._lock:
            if self._closed:
                raise RuntimeError("resource scheduler is closed")
            queue = self._queues.setdefault(job.resource_key, deque())
            queue.append((job.job_id, fn))
            job.queue_position = len(queue)
            condition = self._wakeups.setdefault(
                job.resource_key, threading.Condition(self._lock)
            )
            worker = self._workers.get(job.resource_key)
            if worker is None or not worker.is_alive():
                worker = threading.Thread(
                    target=self._exclusive_worker,
                    args=(job.resource_key, condition),
                    name=f"uni_robot_agent-{job.resource_key}",
                    daemon=True,
                )
                self._workers[job.resource_key] = worker
                worker.start()
            condition.notify()
            self._refresh_positions(job.resource_key)
        self._notify()

    def _exclusive_worker(self, resource_key: str, condition: threading.Condition) -> None:
        while True:
            with condition:
                queue = self._queues[resource_key]
                while not queue and not self._closed:
                    condition.wait()
                if self._closed and not queue:
                    return
                job_id, fn = queue.popleft()
                self._refresh_positions(resource_key)
            self._notify()
            self._run(job_id, fn)

    def _run(self, job_id: str, fn: Callable[[], Any]) -> None:
        if not self._registry.mark_running(job_id):
            return
        self._notify()
        try:
            self._registry.complete(job_id, fn())
        except Exception as exc:
            self._registry.fail(job_id, f"{type(exc).__name__}: {exc}")
        finally:
            self._notify()

    def cancel(self, job_id: str) -> bool:
        cancelled = self._registry.cancel_if_queued(job_id)
        if cancelled:
            self._notify()
        return cancelled

    def cancel_pending(self, resource_key: str, tool_names: set[str] | None, *, reason: str) -> list[JobRecord]:
        """Cancel queued work (of the named tools, or all), keeping each receipt; running jobs are untouched."""
        with self._lock:
            cancelled = [job for job in self._registry.list(resource_key)
                         if (tool_names is None or job.tool_name in tool_names)
                         and self._registry.cancel_if_queued(job.job_id, reason=reason)]
            queue = self._queues.get(resource_key)
            if queue is not None:
                ids = {job.job_id for job in cancelled}
                retained = [(jid, fn) for jid, fn in queue if jid not in ids]
                queue.clear()
                queue.extend(retained)
                self._refresh_positions(resource_key)
        self._notify()
        return cancelled

    def flush(self, resource_key: str) -> int:
        with self._lock:
            queue = self._queues.get(resource_key)
            if not queue:
                return 0
            cancelled = 0
            for job_id, _ in list(queue):
                cancelled += int(self._registry.cancel_if_queued(job_id))
            queue.clear()
            self._refresh_positions(resource_key)
        self._notify()
        return cancelled

    def queue_status(self, resource_key: str | None = None) -> dict[str, Any]:
        with self._lock:
            resources = [resource_key] if resource_key else sorted(self._queues)
            queues = {
                name: [job_id for job_id, _ in self._queues.get(name, ())]
                for name in resources
            }
        return {
            "queues": queues,
            "jobs": [job.summary() for job in self._registry.list(resource_key)],
        }

    def active_snapshot(self) -> dict[str, Any]:
        """Return current jobs for the status view; events retain historical results."""
        jobs = [
            job.summary()
            for job in self._registry.list()
            if job.status in {"queued", "running"}
        ]
        resources: dict[str, dict[str, list[dict[str, Any]]]] = {}
        for job in jobs:
            state = resources.setdefault(
                job["resource"], {"running": [], "queued": []},
            )
            state[job["status"]].append(job)
        for state in resources.values():
            state["queued"].sort(
                key=lambda item: item["queue_position"] or 0,
            )
        return {"resources": resources}

    def _notify(self) -> None:
        if self._on_change is not None:
            self._on_change(self.active_snapshot())

    def _refresh_positions(self, resource_key: str) -> None:
        queue = self._queues.get(resource_key, ())
        self._registry.set_queue_positions([job_id for job_id, _ in queue])

    def close(self, *, drain_timeout: float = 30.0) -> list[str]:
        """Stop scheduling and wait for running jobs.

        Running tool calls may outlast the drain timeout.
        Closing the environment client before they finish would interrupt their RPCs.
        Return unfinished job IDs so the caller can report them.
        """
        with self._lock:
            if self._closed:
                return []
            self._closed = True
            for condition in self._wakeups.values():
                condition.notify_all()

        deadline = time.monotonic() + max(0.0, drain_timeout)
        for worker in self._workers.values():
            worker.join(timeout=max(0.0, deadline - time.monotonic()))
        self._parallel.shutdown(wait=False, cancel_futures=True)
        for job in self._registry.list():
            self._registry.cancel_if_queued(job.job_id)
            self._registry.wait(job.job_id, timeout=max(0.0, deadline - time.monotonic()))

        unfinished = [
            job.job_id for job in self._registry.list()
            if job.status in {"queued", "running"}
        ]
        return unfinished
