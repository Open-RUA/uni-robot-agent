# Rollout measurements and tool judgments

Each Python `Session` and CLI `run` records facts independently of any reward
function. No efficiency coefficient, success gate, policy update or additional
environment action is enabled by recording. Native evaluation stays in the
environment's existing finalization path.

## Saved records

| Artifact | Contract |
|---|---|
| `manifest.json` | Created before agent/environment startup. Contains redacted resolved configuration, run identity, available source commit/dirty-diff hash and untracked-file hashes. Startup failure remains an attempt. Installed wheels may have no Git identity; that limitation is explicit. |
| `trajectory.jsonl` | Existing event ledger, extended with process clock/sequence identities, provider attempts and call/effect judgments. Image bodies remain in the existing artifact store. |
| `measurements.json` | Derived metrics, known counts, missing reasons, unresolved identities, native result and hashes of the effective prompt/tool declarations when present. |
| `result.json` | Existing official outcome and storage status, with a `measurements_path` reference. |

`RunRecord.open(directory).measurements` reads the summary without connecting to
an environment. For offline verification, pass `RunRecord.events()` to
`uni_robot_agent.artifacts.measurements.summarize`. Its derived fields must match
the saved summary; `result` and artifact hashes are finalization metadata.

Recording is integrated into `Session`/CLI `run`. The separate interactive TUI
continues to save its existing event stream but does not yet produce this run
summary. An abrupt process kill leaves the starting manifest and flushed event
prefix; replay reports an unclosed run rather than inventing a final outcome.

## Measurements

| Quantity | Definition and coverage |
|---|---|
| `T` (seconds) | `SessionOpened` after initial observation is available, through `SessionClosed` after execution draining and environment finalization. Startup/warmup before the observation is excluded; native finalization is included. |
| `W` (seconds) | Agent-active intervals minus the union of executor execution and SDK-native tool intervals. Includes decision, model transport and delivery overhead while no tool is busy. Overlapping tool/subagent work is counted once. This is a versioned runtime interval measure, not a model's self-reported reasoning time. Missing model delivery, SDK tool pairs or pending executions makes it null. SDK event receipt latency is included; cross-process clocks are not subtracted. |
| `BTR` | `W/T` when both are known and `T>0`; diagnostic only. |
| `K` | Distinct executor entries, including invalid requests, explicit retries and background control calls. Internal primitive steps and SDK-native shell/file calls are not added to this count; their activity remains in the ledger. Cancelled queued attempts retain their identity. |
| `L_gen` | Sum of provider-reported generated/output tokens for **every upstream attempt**, including discarded retries and responses lost during delivery. Never estimated from text length. A transport failure without usage makes the complete sum null. |
| `L_think`, `L_other` | Provider-reported reasoning partition and `L_gen-L_think`. If no trustworthy partition is supplied, both stay null while an independently known total is retained. No assumption that absent reasoning usage means zero. |
| `N_policy_call_fail` | Count of distinct attempts with a typed policy-owned interface failure. An unresolved attempt makes the complete count null; `known_policy_call_failures` preserves the known lower bound. |

All gateway requests, including context/compaction requests that pass through it,
share the same run. Compaction is not another rollout. Without a gateway, actual
provider usage is unavailable and the summary says so. Hybrid reviewer calls
currently use a separate model client; enabling that role marks full token
totals incomplete rather than reporting gateway-only counts as full coverage.
Unknown or unobserved
work cannot be reconstructed from SDK token estimates. `core_complete` requires
all core quantities and a consistent closed event stream; inspect missing reasons
instead of treating null as zero.

Events use `(session_id,event_seq)` for deduplication, `execution_id` for tool
attempts, `call_id/batch_id/job_id` for correlation, and `(exchange_id,attempt)`
for upstream requests. Independent repetitions retain different execution IDs.
Only timestamps with the same explicit `clock_id` are comparable. SDK-provided
wall times remain useful for presentation; measurement uses local bus receipt
times. Conflicting duplicate records invalidate completeness.

## Call status versus direct effect

`ToolOutput.success` means the interface returned a valid result. It does not
mean that the arm arrived, the object was grasped or the task succeeded.
Judgments are stored on `ToolResult`, outside model-visible feedback:

| Existing evidence | Interface / responsibility | Direct effect |
|---|---|---|
| Schema validation rejects parameters; unregistered tool | failure / policy | unknown, no confirmed execution |
| A legal motion returns `targets_reached=false` | success / none | failure under that call's controller tolerances |
| A legal segmentation returns `found=false` | success / none | failure to find the target |
| Typed HTTP/transport fault at the environment boundary | failure or unknown / infrastructure | unknown |
| Other rejection or exception without reliable responsibility evidence | failure or unknown / unknown | unknown |
| Query returns a known failed background job | successful query; original action assessed separately | not applicable |
| Finish submission succeeds but native task fails | success / none | not applicable; native failure remains in the result |

Do not infer policy responsibility by matching words such as `rejected`, `stale`
or `timeout`. A `RequestRejected` can represent either a bad request or a domain
condition. Until a producer provides a typed owner, retain unknown responsibility.
This implementation attributes schema/unknown-tool faults and explicitly typed
finish/localization argument-combination violations to policy. Untyped native
rejections remain attribution gaps for follow-up.

Effect assessment follows the **released** tools: `move` consumes its returned
arrival/target flags; `grasp`/`release` consume metric `opening_reached` feedback;
`locate` consumes depth/triangulation validity or segmentation feedback. Do not
import legacy `move_to`, `back_project` or normalized 0.05 gripper rules.
Opening arrival does not establish attachment or release, geometric validity
does not establish object identity, and holding does not establish stability.
Hybrid `actor_act` retains its feedback with an unknown aggregate effect until
a separate direct-effect contract is implemented. No second image capture,
motion, or task grading call is made by any verifier.

## Finish and native task completion

The current `finish(status="success", summary=...)` submits the model's claim
and returns an acknowledgement. It does not return privileged native scores to
the agent. The environment remains authoritative, and the session drains jobs
before reading the official final outcome. Existing episode/service sealing
prevents a second final check when the session closes after `finish`.

RoboDojo's native adapter retains its own task grader, step limits and
`final_check=True` behavior on eligible explicit finishes. The recorder neither
changes these task criteria nor substitutes the model's completion claim. Keep
native success/score, agent completion, interface status and effect status as
separate fields. Preserve each environment's native grading when extending
this recording to RoboUse or other integrations.

## Validation

Run `python -B -m pytest tests/test_measurements.py`. Coverage includes serial
and background attempts, repeated errors, result-query deduplication, retries,
missing token partitions, SDK interval overlap, JSON/replay, startup failure,
credential filtering and finish/native-outcome separation. These CPU fixtures
do not establish a real simulator rollout or hardware validation.

Maintain source ownership: producer semantics in `tools/` and `integrations/`,
attempt usage in `provider/`, clock/event identity in `events/`, derived records
in `artifacts/`, lifecycle assembly in `session.py`. Reward functions and
training ingestion belong downstream of these facts.
