"""Replan after failure: when a physical action does not complete, its old plan stops.

``ParallelToolExecutor`` owns one ``ReplanTracker`` and calls it under its admission
lock. When a tool whose policy sets ``replan_on_failure`` returns an incomplete
``ActionOutcome`` that may have moved something, its resource needs a replan:

1. Queued world-mutating calls on that resource are cancelled before dispatch and keep
   their receipts. A call that was already dequeued is refused when it starts.
2. New world-mutating calls on the resource are refused (``reason: replan_required``).
   Tools whose policy sets ``ends_episode`` stay allowed, so the model can still report.
3. The requirement clears once the failed result has been read, the environment has
   confirmed that the actuators stopped, and a ``replan_observation`` tool (on any resource)
   that started after both returns a new snapshot. The model then plans a new action from it.

When execution is unknown or the stop is not confirmed, the requirement never clears
within the episode: recovery belongs to the operator. Nothing here dispatches, retries
or replays an action.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field, replace
from typing import Any

from uni_robot_agent.tools.toolkit import ActionOutcome, ToolOutput

CANCELLED = {"no_execution": True, "execution_status": "cancelled_before_dispatch",
             "reason": "replan_after_failure"}


def action_outcome(output: ToolOutput) -> ActionOutcome | None:
    """The outcome a tool reported; a failed tool that did not say counts as unknown."""
    if output.action is not None:
        return output.action
    if output.success and not output.error:
        return None
    if output.extra.get("no_execution") is True:
        return ActionOutcome(complete=False, executed="none")
    return ActionOutcome(complete=False, executed="unknown")


def requires_replan(outcome: ActionOutcome | None) -> bool:
    """An incomplete action that may have moved something invalidates the queued plan."""
    return outcome is not None and not outcome.complete and outcome.executed != "none"


@dataclass
class PendingReplan:
    epoch: int
    failed_job_id: str | None
    reason: str
    executed: str
    stop_confirmed: bool
    result_read: bool
    failed_snapshot_id: str | None
    cancelled_jobs: list[dict[str, Any]] = field(default_factory=list)

    @property
    def phase(self) -> str:
        if not self.result_read:
            return "result_unread"
        if not self.stop_confirmed:
            return "stop_unconfirmed"
        return "observation_required"

    def report(self, observation_tools: list[str]) -> dict[str, Any]:
        return {
            "phase": self.phase,
            "failed_job_id": self.failed_job_id,
            "reason": self.reason,
            "executed": self.executed,
            "stop_confirmed": self.stop_confirmed,
            "cancelled_jobs": deepcopy(self.cancelled_jobs),
            "next_action": self._next_action(observation_tools),
        }

    def _next_action(self, observation_tools: list[str]) -> str:
        if self.phase == "result_unread":
            return f"Read the result of job {self.failed_job_id} with wait_for_tool or get_tool_result."
        if self.phase == "stop_unconfirmed":
            return ("The stop was not confirmed or execution is unknown. No new action can run until "
                    "an operator recovers the robot; report the condition.")
        if not observation_tools:
            return "No observation tool can start a replan in this environment; report the condition."
        return (f"Call {' or '.join(observation_tools)} for a new observation, then plan a NEW action "
                "from the measured state. Do not resubmit cancelled actions.")


class ReplanTracker:
    """Per-resource replan requirements; the executor holds its admission lock around every call."""

    def __init__(self, observation_tools: list[str]):
        self.observation_tools = observation_tools
        self.pending: dict[str, PendingReplan] = {}
        self.epochs: dict[str, int] = {}

    def epoch(self, resource: str) -> int:
        return self.epochs.get(resource, 0)

    def begin(self, resource: str, job_id: str | None, output: ToolOutput, outcome: ActionOutcome,
              cancelled: list[dict[str, Any]], *, result_read: bool) -> ToolOutput:
        """Start (or extend) a replan requirement and attach it to the failed result."""
        epoch = self.epochs[resource] = self.epoch(resource) + 1
        stop_confirmed = outcome.stopped is True and outcome.executed != "unknown"
        previous = self.pending.get(resource)
        if previous is not None:
            # A later failure cannot erase an earlier unknown execution or unconfirmed stop.
            stop_confirmed = stop_confirmed and previous.stop_confirmed
            cancelled = previous.cancelled_jobs + cancelled
        item = self.pending[resource] = PendingReplan(
            epoch=epoch, failed_job_id=job_id, reason=output.error or "action_incomplete",
            executed=outcome.executed, stop_confirmed=stop_confirmed,
            result_read=result_read or job_id is None,
            failed_snapshot_id=output.extra.get("snapshot_id"), cancelled_jobs=cancelled)
        # The model sees this report by reading the result, so it describes the step after that.
        output.extra["replan"] = replace(item, result_read=True).report(self.observation_tools)
        return output

    def result_read(self, resource: str, job_id: str) -> None:
        item = self.pending.get(resource)
        if item is not None and item.failed_job_id == job_id:
            item.result_read = True

    def rejection(self, resource: str) -> ToolOutput | None:
        item = self.pending.get(resource)
        if item is None:
            return None
        return ToolOutput(success=False, error="A failed action requires a new plan first",
                          extra={"no_execution": True, "reason": "replan_required",
                                 "replan": item.report(self.observation_tools)})

    def observation_tokens(self) -> dict[str, int]:
        """Taken when an observation starts: the replans it may clear, by resource and epoch.

        An observation sees the whole scene, so it can clear a replan on any resource; only one
        that started after a replan's prerequisites can clear that replan.
        """
        return {resource: item.epoch for resource, item in self.pending.items()
                if item.result_read and item.stop_confirmed}

    def observed(self, tokens: dict[str, int] | None, output: ToolOutput) -> ToolOutput:
        if not self.pending:
            return output
        snapshot_id = output.extra.get("snapshot_id")
        usable = output.success and not output.error and bool(snapshot_id)
        cleared = [resource for resource, item in self.pending.items()
                   if usable and (tokens or {}).get(resource) == item.epoch and snapshot_id != item.failed_snapshot_id]
        last = None
        for resource in cleared:
            last = self.pending.pop(resource)
        if self.pending:
            output.extra["replan"] = next(iter(self.pending.values())).report(self.observation_tools)
        else:
            output.extra["replan"] = {
                "phase": "ready_to_replan", "failed_job_id": last.failed_job_id, "snapshot_id": snapshot_id,
                "next_action": "Plan a NEW action from this observation. Do not resubmit cancelled actions."}
        return output
