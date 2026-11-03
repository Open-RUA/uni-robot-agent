"""Reward-independent run facts, replayed from the existing trajectory event stream."""
from __future__ import annotations

import hashlib
import json
import math
import re
import subprocess
import threading
from collections import Counter
from pathlib import Path

from uni_robot_agent.artifacts.run import write_json_atomic

VERSION = "measurements_1"


def redact(value):
    """Filter credential fields before events/configuration reach disk."""
    if isinstance(value, dict):
        return {k: "[REDACTED]" if re.search(
            r"(^|[_-])(api[_-]?key|authorization|password|secret|access[_-]?token|refresh[_-]?token|cookie)($|[_-])",
            str(k), re.I) else redact(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [redact(v) for v in value]
    if isinstance(value, str):
        return re.sub(r"(?i)\bBearer\s+[^\s\"']+", "Bearer [REDACTED]", value)
    return value


def _union(intervals):
    result = []
    for start, end in sorted(intervals):
        if end < start:
            continue
        if result and start <= result[-1][1]:
            result[-1] = (result[-1][0], max(result[-1][1], end))
        else:
            result.append((start, end))
    return result


def _length(intervals):
    return sum(end - start for start, end in _union(intervals))


def _subtract(intervals, busy):
    return _length(intervals) - _length([
        (max(a, c), min(b, d)) for a, b in _union(intervals)
        for c, d in _union(busy) if max(a, c) < min(b, d)])


def summarize(records):
    """Pure online/offline projection; missing or conflicting evidence stays null."""
    seen, rows, issues = {}, [], []
    for row in records:
        key = (row.get("session_id"), row.get("event_seq"))
        if key[1] is None:
            issues.append("missing_event_identity")
        elif key in seen:
            if seen[key] != row:
                issues.append("conflicting_event_identity")
            continue
        seen[key] = row
        rows.append(row)
    rows.sort(key=lambda row: row.get("event_seq", 0))
    if [r.get("event_seq") for r in rows] != list(range(1, len(rows) + 1)):
        issues.append("event_sequence_gap")
    if len({r.get("session_id") for r in rows}) != 1:
        issues.append("mixed_session_identity")
    clocks = {r.get("clock_id") for r in rows}
    timing_ok = len(clocks) == 1 and None not in clocks and all(
        type(r.get("monotonic_s")) in (int, float) and math.isfinite(r["monotonic_s"]) for r in rows)
    if not timing_ok:
        issues.append("missing_or_mixed_clock_domain")

    def of_type(kind):
        return [r for r in rows if r.get("type") == kind]

    opened, closed = of_type("SessionOpened"), of_type("SessionClosed")
    total = (closed[-1]["monotonic_s"] - opened[0]["monotonic_s"]
             if timing_ok and opened and closed else None)
    reasons = {key: [] for key in ("T", "W", "K", "L_gen", "L_think", "N_policy_call_fail")}
    if total is None or total < 0:
        total = None
        reasons["T"].append("missing_valid_observation_to_close_boundary")
    if not closed:
        issues.append("session_not_closed")

    executions, results, busy, pending = {}, {}, [], []
    for row in of_type("ToolExecution"):
        identity = row.get("execution_id")
        if not identity:
            issues.append("missing_execution_id")
            continue
        executions.setdefault(identity, {})[row["phase"]] = row
    for row in of_type("ToolResult"):
        identity = row.get("execution_id")
        if not identity:
            issues.append("unpaired_tool_result")
        elif identity in results and results[identity] != row:
            issues.append("conflicting_tool_result")
        else:
            results[identity] = row
    for identity, phases in executions.items():
        if "finished" not in phases and "cancelled" not in phases:
            pending.append(identity)
        elif "finished" in phases and "started" not in phases:
            issues.append("missing_execution_start")
        if "started" in phases and "finished" in phases and timing_ok:
            busy.append((phases["started"]["monotonic_s"], phases["finished"]["monotonic_s"]))
    if pending:
        issues.append("unfinished_tool_executions")
    assessments = {identity: row.get("call_assessment", {}) for identity, row in results.items()}
    # Cancelled queued work is not a measured policy error; retain its identity.
    unresolved = [identity for identity, phases in executions.items()
                  if "cancelled" not in phases and (
                      identity not in assessments or assessments[identity].get("penalty_eligible") is None)]
    known_failures = sum(a.get("penalty_eligible") is True for a in assessments.values())
    if unresolved:
        reasons["N_policy_call_fail"].append("unresolved_call_responsibility_or_result")

    attempts = {}
    for row in of_type("ProviderAttempt"):
        attempts.setdefault((row.get("exchange_id"), row.get("attempt")), {})[row["phase"]] = row
    model = {}
    for row in of_type("ModelRequest"):
        model.setdefault(row.get("exchange_id"), {})[row["phase"]] = row
    delivered = all("delivered" in phases or "delivery_failed" in phases for phases in model.values())
    token_totals = {}
    for name, field in (("L_gen", "generated_tokens"), ("L_think", "think_tokens"), ("L_other", "other_tokens")):
        values = [phases.get("finished", {}).get(field) for phases in attempts.values()]
        valid = bool(values) and all(type(v) is int and v >= 0 for v in values)
        valid = valid and all("started" in phases and "finished" in phases for phases in attempts.values())
        valid = valid and bool(model) and delivered
        valid = valid and not any(r.get("unobserved_model_roles") for r in opened)
        token_totals[name] = sum(values) if valid else None
        if not valid:
            reasons.setdefault(name, []).append("missing_provider_usage_or_unclosed_request")
            if any(r.get("unobserved_model_roles") for r in opened):
                reasons[name].append("unobserved_model_roles")

    # SDK native tools (shell, files, subagents) are busy work too. Their IDs
    # are paired explicitly; never match repeated calls by name or timestamp.
    sdk = {}
    for row in of_type("AgentActivity"):
        if row.get("kind") in {"tool_started", "tool_completed", "tool_failed"}:
            key = (row.get("agent_id"), row.get("item_id") or row.get("transport_call_id"))
            if not key[1]:
                reasons["W"].append("missing_sdk_tool_identity")
            sdk.setdefault(key, {})[row["kind"]] = row
    for phases in sdk.values():
        start = phases.get("tool_started")
        end = phases.get("tool_completed") or phases.get("tool_failed")
        if start and end and timing_ok:
            busy.append((start["monotonic_s"], end["monotonic_s"]))
        else:
            reasons["W"].append("unpaired_sdk_tool_lifecycle")
    starts, ends = of_type("SessionStart"), of_type("SessionEnd")
    if not timing_ok or not starts or len(starts) != len(ends) or pending or not model or not delivered or "missing_execution_start" in issues:
        reasons["W"].append("incomplete_agent_model_or_execution_lifecycle")
    wait = None
    if not reasons["W"]:
        active = [(start["monotonic_s"], end["monotonic_s"]) for start, end in zip(starts, ends)]
        if any(b < a for a, b in active):
            reasons["W"].append("invalid_agent_boundary")
        else:
            wait = max(0.0, _subtract(active, busy))
    if issues:
        # Keep individually known values, but never advertise joint completeness.
        reasons["K"].extend(i for i in issues if i in {
            "missing_event_identity", "conflicting_event_identity", "missing_execution_id",
            "event_sequence_gap", "mixed_session_identity", "unpaired_tool_result"})
    if reasons["K"]:
        reasons["N_policy_call_fail"].append("incomplete_event_stream")
        for name in token_totals:
            token_totals[name] = None
            reasons.setdefault(name, []).append("incomplete_event_stream")
        wait = None
        reasons["W"].append("incomplete_event_stream")
    k = len(executions) if not reasons["K"] else None
    metrics = {"T": total, "W": wait, "K": k, **token_totals,
               "N_policy_call_fail": known_failures if not reasons["N_policy_call_fail"] else None}
    return {"version": VERSION, "clock_id": next(iter(clocks)) if timing_ok else None,
            "time_boundary": "SessionOpened_after_initial_observation_to_SessionClosed_after_drain",
            "wait_scope": "agent_active_time_minus_union_of_executor_and_sdk_tool_intervals",
            "call_scope": "executor_entries_including_job_controls; SDK_native_work_separate",
            "metrics": metrics, "known_policy_call_failures": known_failures,
            "BTR": wait / total if wait is not None and total is not None and total > 0 else None,
            "call_outcomes": dict(Counter(a.get("status", "unknown") for a in assessments.values())),
            "fault_owners": dict(Counter(a.get("fault_owner", "unknown") for a in assessments.values())),
            "effect_outcomes": dict(Counter(r.get("effect_assessment", {}).get("status", "unknown") for r in results.values())),
            "unresolved_attempt_ids": unresolved, "pending_execution_ids": pending,
            "missing_reasons": {k: sorted(set(v)) for k, v in reasons.items()},
            "integrity_issues": sorted(set(issues)),
            "core_complete": all(v is not None for v in metrics.values()) and not issues,
            "native_outcome": closed[-1] if closed else None}


class RunMeasurements:
    """Create provenance before startup, then consume the canonical event records."""

    def __init__(self, directory, config):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.records = []
        self._lock = threading.Lock()
        self.manifest = {"version": VERSION, "run_id": self.directory.name,
                         "config": redact(config), "phase": "starting", "outcome": None,
                         "source": self._source_identity()}
        write_json_atomic(self.directory / "manifest.json", self.manifest)

    @staticmethod
    def _source_identity():
        root = Path(__file__).resolve().parents[2]
        try:
            def git(*args):
                return subprocess.check_output(["git", "-C", str(root), *args], stderr=subprocess.DEVNULL)
            if Path(git("rev-parse", "--show-toplevel").decode().strip()).resolve() != root.resolve():
                raise OSError("Installed package is not the enclosing repository's source")
            diff = git("diff", "HEAD", "--", ".")
            untracked = git("ls-files", "--others", "--exclude-standard").decode().splitlines()
            files = {name: hashlib.sha256((root / name).read_bytes()).hexdigest()
                     for name in untracked if (root / name).is_file()}
            return {"commit": git("rev-parse", "HEAD").decode().strip(),
                    "dirty": bool(diff or files), "diff_sha256": hashlib.sha256(diff).hexdigest(),
                    "untracked_sha256": files}
        except (OSError, subprocess.CalledProcessError):
            from importlib.metadata import version, PackageNotFoundError
            try:
                package_version = version("uni-robot-agent")
            except PackageNotFoundError:
                package_version = None
            return {"commit": None, "package_version": package_version, "reason": "git_identity_unavailable"}

    def __call__(self, event):
        from uni_robot_agent.events.subscribers import event_to_record
        with self._lock:
            self.records.append(event_to_record(event))

    def finish(self, result):
        with self._lock:
            summary = summarize(self.records)
        summary["result"] = redact(result)
        summary["artifacts_sha256"] = {name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name in ("effective_prompt.md", "tools.json")
            if (path := self.directory / name).is_file()}
        if result.get("storage_errors"):
            summary["core_complete"] = False
            summary["integrity_issues"].append("storage_errors")
        write_json_atomic(self.directory / "measurements.json", summary, allow_nan=False)
        self.manifest.update(phase="closed" if summary["native_outcome"] else "startup_failed",
                             outcome=redact(result), measurements_sha256=hashlib.sha256(
                                 (self.directory / "measurements.json").read_bytes()).hexdigest())
        write_json_atomic(self.directory / "manifest.json", self.manifest)
        return summary
