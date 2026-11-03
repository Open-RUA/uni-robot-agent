# Rollout measurements and tool judgments

Companion contract for [implementation PR #1](https://github.com/Open-RUA/uni-robot-agent/pull/1).
The implementation records facts for later reward experiments; it does not enable
an efficiency reward or change native grading.

## Review order

| Area | Read first | Question |
|---|---|---|
| Raw measurements | `events/core.py`, `provider/usage.py` | Are identities and actual provider counts preserved? |
| Judgments | `tools/assessment.py`, `tools/toolkit.py` | Are interface status, fault ownership and direct effect separate? |
| Aggregation | `artifacts/measurements.py` | Do missing evidence and repeated records behave correctly? |
| Lifecycle | `session.py` | Does a failed startup survive, and does recording leave execution unchanged? |
| Regression | `tests/test_measurements.py` | Do retries, parallel queries, storage failures and replay agree? |

Paths are relative to `uni_robot_agent/`, except `tests/`. Detailed explanations
live here so the implementation diff contains only code, focused tests and CI.

## Artifacts

- `manifest.json`: created before startup; redacted configuration, run identity,
  source commit and dirty hashes. Failed startup remains an attempt.
- `trajectory.jsonl`: the existing canonical event ledger, extended with clock,
  request/attempt identities, provider usage and tool assessments.
- `measurements.json`: derived facts, missing reasons, completeness and native result.
  Finalization reads the flushed ledger; it does not keep a second event buffer.
- `result.json`: existing official outcome, plus a `measurements_path` reference.

`RunRecord.open(directory).measurements` reads the summary. Recompute its derived
fields with `summarize(RunRecord.open(directory).events())`; `result` and artifact
hashes are finalization metadata. An interrupted run retains its manifest and
flushed event prefix.

## Measurements

| Quantity | Definition |
|---|---|
| T, seconds | Initial observation available (`SessionOpened`) to finalization after draining (`SessionClosed`); excludes startup, includes sealing. |
| W, seconds | Agent-active intervals minus the union of executor and SDK-native tool intervals. Includes decision/transport overhead when no tool is busy. |
| BTR | W/T when both are known and T>0; diagnostic only. |
| K | Distinct executor entries, including invalid requests, explicit retries and background controls; excludes primitive steps and SDK shell/file calls. |
| L_gen | Provider-reported generated tokens from every upstream attempt, including discarded retries; never estimated from text. |
| L_think / L_other | Reported reasoning tokens / L_gen minus reasoning. Missing partition stays null even when the total is known. |
| N_policy_call_fail | Distinct, explicitly policy-owned interface failures. Unresolved responsibility makes the complete count null; the known lower bound is retained. |

Deduplicate by `(session_id,event_seq)`, correlate tools by `execution_id` and
`call_id/job_id`, and provider attempts by `(exchange_id,attempt)`. Independent
retries count again; repeated reads of the same job do not repeat its failure.
Compaction stays within the same run. Only local bus-receipt timestamps from the
same clock domain are subtracted; SDK receipt latency is part of W.

Missing model delivery, unmatched SDK calls or unfinished execution make W null.
Missing usage for any model request makes the full token total null. Inspect
`missing_reasons` and `integrity_issues`; null is never replaced with zero.

## Tool judgments

| Existing evidence | Interface / owner | Direct effect |
|---|---|---|
| Invalid schema or unknown tool | failure / policy | unknown; not executed |
| Legal motion, targets not reached | success / none | failure under this call's tolerances |
| Legal segmentation, found=false | success / none | target not found |
| Typed service or transport fault | failure or unknown / infrastructure | unknown |
| Rejection without reliable attribution | failure or unknown / unknown | unknown |
| Query of an existing failed job | successful query; original action kept separately | not applicable |
| Finish acknowledged, native task fails | success / none | not applicable |

Only typed policy faults are eligible for a downstream calling penalty. Do not
attribute responsibility by matching words such as `stale` or `rejected`.
Current typed cases cover schema, unknown tools and explicit finish/localization
argument violations. Other native rejections retain unknown responsibility.

Use the released `move`/`locate` contracts and metric `opening_reached` feedback,
not legacy `move_to`/`back_project` or the normalized 0.05 gripper threshold.
Opening arrival does not prove grasping; valid geometry does not prove identity.
No verifier captures another image, executes a motion or grades the task again.

## Finish and coverage

`finish(status="success", summary=...)` submits the model's claim and returns an
acknowledgement. Native success/score remain in saved results, outside agent
feedback. Existing sealing prevents repeated final grading.

This draft covers Python `Session` and CLI `run`. TUI summary and full Hybrid
reviewer usage remain follow-ups; reviewer-enabled token totals are explicitly
incomplete. Actor effects retain feedback without a unified success label.

Run `python -B -m pytest tests/test_measurements.py` for CPU contracts. Real
RoboDojo validation must separately check native results, recording coverage,
replay equality and unchanged tool behavior. A small pilot checks integration;
paired task/layout samples are needed to assess success-rate changes.
