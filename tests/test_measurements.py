"""CPU contract checks; these fixtures do not constitute simulator rollouts."""
import json
from types import SimpleNamespace

import pytest

from uni_robot_agent.artifacts.measurements import RunMeasurements, summarize
from uni_robot_agent.events import EventBus
from uni_robot_agent.events.core import ProviderAttempt
from uni_robot_agent.events.subscribers import EventJsonlSubscriber, event_to_record
from uni_robot_agent.execution import SerialToolExecutor, ParallelToolExecutor, connect_execution_events
from uni_robot_agent.execution.scheduler import ToolExecutionPolicy
from uni_robot_agent.tools.assessment import assess_call, assess_effect
from uni_robot_agent.tools.toolkit import Toolkit, ToolOutput, ToolSpec, bind_tools, schema
from uni_robot_agent.provider.usage import token_counts


def records():
    rows = [
        ("SessionOpened", 0, {}), ("SessionStart", 1, {}),
        ("ModelRequest", 1, {"exchange_id": "m", "phase": "started"}),
        ("ProviderAttempt", 1, {"exchange_id": "m", "attempt": 1, "phase": "started"}),
        ("ProviderAttempt", 2, {"exchange_id": "m", "attempt": 1, "phase": "finished",
                                "generated_tokens": 12, "think_tokens": 4, "other_tokens": 8}),
        ("ModelRequest", 2, {"exchange_id": "m", "phase": "delivered"}),
        ("ToolExecution", 3, {"execution_id": "a", "phase": "queued"}),
        ("ToolExecution", 3, {"execution_id": "a", "phase": "started"}),
        ("ToolResult", 7, {"execution_id": "a", "call_assessment": {
            "status": "success", "fault_owner": "none", "penalty_eligible": False}}),
        ("ToolExecution", 7, {"execution_id": "a", "phase": "finished"}),
        ("SessionEnd", 9, {}), ("SessionClosed", 10, {"task_success": False}),
    ]
    return [dict(type=kind, monotonic_s=at, clock_id="clock", session_id="run", event_seq=i + 1, **fields)
            for i, (kind, at, fields) in enumerate(rows)]


def test_replay_json_roundtrip_and_compact_is_one_episode():
    source = records()
    source.append(dict(type="AgentActivity", kind="compacted", monotonic_s=8,
                       clock_id="clock", session_id="run", event_seq=13))
    expected = summarize(source)
    assert summarize(json.loads(json.dumps(source))) == expected
    assert summarize(source + source) == expected
    assert expected["metrics"] == {"T": 10, "W": 4, "K": 1, "L_gen": 12,
        "L_think": 4, "L_other": 8, "N_policy_call_fail": 0}
    assert expected["core_complete"]


def test_overlapping_native_tool_intervals_count_once():
    source = records()
    for seq, kind, at in ((13, "tool_started", 2), (14, "tool_completed", 4)):
        source.append(dict(type="AgentActivity", kind=kind, agent_id="sub", item_id="shell1",
                           event_seq=seq, monotonic_s=at, clock_id="clock", session_id="run"))
    assert summarize(source)["metrics"]["W"] == 3
    source.pop()
    assert summarize(source)["metrics"]["W"] is None


@pytest.mark.parametrize("drop,metric", [("SessionClosed", "T"), ("ProviderAttempt", "L_gen"),
                                        ("ToolResult", "N_policy_call_fail")])
def test_missing_evidence_is_not_zero(drop, metric):
    result = summarize([r for r in records() if r["type"] != drop])
    assert result["metrics"][metric] is None
    assert not result["core_complete"]


def test_mixed_clocks_and_conflicting_replay():
    source = records()
    source[2]["clock_id"] = "other_process"
    result = summarize(source)
    assert result["metrics"]["T"] is None
    assert result["metrics"]["W"] is None
    duplicate = {**source[0], "monotonic_s": 100}
    assert summarize(source + [duplicate])["metrics"]["K"] is None


def test_every_model_request_requires_its_own_provider_usage():
    source = records()
    for phase in ("started", "delivered"):
        source.append(dict(type="ModelRequest", exchange_id="unobserved", phase=phase,
            event_seq=len(source) + 1, monotonic_s=8, clock_id="clock", session_id="run"))
    result = summarize(source)
    assert result["metrics"]["L_gen"] is None
    assert not result["core_complete"]


def test_retry_usage_counts_discarded_generation_and_missing_partition():
    source = records()
    source += [dict(type="ProviderAttempt", phase=phase, exchange_id="m", attempt=2,
        event_seq=13+i, monotonic_s=2, clock_id="clock", session_id="run", **extra)
        for i, (phase, extra) in enumerate([
            ("started", {}), ("finished", {"generated_tokens": 7, "think_tokens": 2, "other_tokens": 5})])]
    assert summarize(source)["metrics"]["L_gen"] == 19
    source[-1].pop("think_tokens")
    result = summarize(source)
    assert result["metrics"]["L_gen"] == 19
    assert result["metrics"]["L_think"] is None


@pytest.mark.parametrize("protocol,usage,expected", [
    ("responses", {"output_tokens": 10, "output_tokens_details": {"reasoning_tokens": 3}}, (10, 3, 7)),
    ("chat_completions", {"completion_tokens": 10, "completion_tokens_details": {"reasoning_tokens": 2}}, (10, 2, 8)),
    ("messages", {"output_tokens": 10}, (10, None, None)),
    ("responses", {"output_tokens": True}, (None, None, None)),
    ("responses", {"output_tokens": 10, "output_tokens_details": {"reasoning_tokens": 11}}, (10, None, None)),
])
def test_actual_usage_no_text_estimation(protocol, usage, expected):
    result = token_counts(json.dumps({"usage": usage, "type": "message", "content": [],
                                     "stop_reason": "end_turn"}).encode(), protocol)
    assert tuple(result[k] for k in ("generated_tokens", "think_tokens", "other_tokens")) == expected


@pytest.mark.parametrize("parallel", [False, True])
def test_independent_bad_attempts_accumulate_but_queries_do_not_repenalize(parallel):
    bus = EventBus()
    toolkit = Toolkit()
    toolkit.set_event_bus(bus)
    toolkit.register_tools(bind_tools([ToolSpec("test", "Fixture", schema({"n": {"type": "integer"}}, ["n"]))],
                                     provider_id="fixture", handlers={"test": lambda n: ToolOutput()}))
    executor = ParallelToolExecutor(toolkit, policies={"test": ToolExecutionPolicy(return_mode="background")}) if parallel else SerialToolExecutor(toolkit)
    connect_execution_events(executor, bus)
    try:
        for _ in range(2):
            result = executor.call("test", {"n": "bad"})
            if result.job_id:
                executor.job_result(result.job_id, timeout=None)
                executor.call("get_tool_result", {"job_id": result.job_id})
                executor.call("get_tool_result", {"job_id": result.job_id})
        executor.call("does_not_exist", {})
    finally:
        executor.close()
    source = [event_to_record(e) for e in bus.replay()]
    summary = summarize(source)
    assert summary["known_policy_call_failures"] == 3
    assert summary["metrics"]["N_policy_call_fail"] == 3
    assert summary["metrics"]["K"] >= 3
    for row in source:
        if row["type"] == "ToolResult":
            assert "call_error" not in row["output"]


def test_domains_do_not_become_policy_errors():
    for output in ({"success": True, "found": False},
                   {"success": True, "feedback": {"pose_reached": False}}):
        assert assess_call(output)["penalty_eligible"] is False
    assert assess_call({"success": False, "error": "rejected"})["fault_owner"] == "unknown"
    assert assess_call({"success": False, "error": "service", "call_error": {
        "fault_owner": "infrastructure", "code": "transport"}})["penalty_eligible"] is False


def test_assessment_failure_preserves_executed_tool_result(monkeypatch):
    from uni_robot_agent.tools import assessment
    bus, toolkit = EventBus(), Toolkit()
    toolkit.set_event_bus(bus)
    output = ToolOutput(extra={"feedback": {"steps_executed": 1}})
    toolkit.register_tools(bind_tools([ToolSpec("hold", "Fixture", schema({}))],
        provider_id="fixture", handlers={"hold": lambda: output}))
    def broken_assessor(*args, **kwargs):
        raise ValueError("fixture diagnostic failure")
    monkeypatch.setattr(assessment, "assess_effect", broken_assessor)
    assert toolkit.execute_tool("hold", {}) is output
    event = event_to_record(bus.replay()[-1])
    assert event["call_assessment"]["status"] == "success"
    assert event["effect_assessment"]["reason"] == "assessment_error"
    assert event["output"] == output.to_dict()


def test_released_effect_contract_uses_metric_opening_not_legacy_threshold():
    output = {"success": True, "feedback": {"steps_executed": 3, "arms": {
        "left": {"opening_reached": False, "opening_error_m": .003, "opening_tolerance_m": .002}}}}
    assert assess_effect("grasp", output)["status"] == "failure"
    assert assess_effect("finish", {"success": True, "submitted": True})["reason"] == "not_applicable"
    assert assess_effect("actor_act", output)["status"] == "unknown"


def test_manifest_precedes_execution_and_failure_survives(tmp_path):
    recorder = RunMeasurements(tmp_path, {"model": "fixture", "api_key": "DO_NOT_SAVE"})
    assert (tmp_path / "manifest.json").is_file()
    recorder.finish({"task_success": None, "accepted": False})
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert manifest["phase"] == "startup_failed"
    assert "DO_NOT_SAVE" not in (tmp_path / "manifest.json").read_text()


def test_session_startup_error_preserves_attempt(tmp_path, monkeypatch):
    import uni_robot_agent.session as module
    args = SimpleNamespace(output_dir=str(tmp_path), integration="fixture", agent_backend=None,
                           episode_timeout_s=None, instruction="fixture")
    def fail(args):
        raise RuntimeError("fixture startup failure")
    monkeypatch.setattr(module, "assemble_session", fail)
    session = module.Session._from_resolved(args)
    with pytest.raises(RuntimeError, match="fixture startup failure"):
        session.start()
    saved = json.loads(next(tmp_path.glob("*/measurements.json")).read_text())
    assert saved["metrics"]["T"] is None
    assert saved["result"]["accepted"] is False


def test_jsonl_redacts_credentials(tmp_path):
    from uni_robot_agent.artifacts.store import ArtifactStore
    from uni_robot_agent.events import ToolResult
    store = ArtifactStore(tmp_path)
    sink = EventJsonlSubscriber(tmp_path, artifacts=store)
    event = ToolResult(arguments={"authorization": "TOP_SECRET"},
                       output={"nested": {"api_key": "TOP_SECRET"}, "error": "Bearer TOP_SECRET"})
    try:
        sink(event)
    finally:
        sink.close()
        store.close()
    assert "TOP_SECRET" not in (tmp_path / "trajectory.jsonl").read_text()


def test_provider_attempt_has_stable_identity_after_json():
    bus = EventBus()
    bus.emit(ProviderAttempt(exchange_id="m", attempt=1, phase="started"))
    row = json.loads(json.dumps(event_to_record(bus.replay()[0])))
    assert row["session_id"] and row["clock_id"] and row["event_seq"] == 1


def test_finish_submission_does_not_reveal_or_replace_native_failure(tmp_path):
    from unittest.mock import Mock
    from uni_robot_agent.integrations.prepared_episode import Episode
    from uni_robot_agent.tools.episode import EpisodeTools
    episode = Episode.__new__(Episode)
    episode.finished, episode.outcome, episode.finish_request = False, None, None
    episode.output, episode.storage_errors = tmp_path, {}
    episode.env = SimpleNamespace(uncertain=False, finish=Mock(return_value={
        "outcome": {"task_success": False, "native_score": 0.0}}))
    output = EpisodeTools(episode).finish("success", "Fixture completion claim")
    assert output.to_dict(for_model=True) == {"call_ok": True, "submitted": True}
    assert episode.task_success is False
    episode.finalize(reason="session_finalize")
    assert episode.env.finish.call_count == 1
    assert assess_call(output.to_dict())["status"] == "success"


def test_model_client_records_every_retry_without_exposing_responses(monkeypatch):
    from uni_robot_agent.provider import client
    from uni_robot_agent.provider.registry import build_provider_adapter
    bodies = [json.dumps({"status": "failed", "error": {"code": "retry", "message": "fixture"},
                          "usage": {"output_tokens": 7}}).encode(),
              json.dumps({"usage": {"output_tokens": 11}}).encode()]
    emitted = []
    monkeypatch.setattr(client, "_send_provider_request", lambda *a, **kw: (200, {}, bodies.pop(0)))
    model = client.ModelClient("http://localhost", max_retries=1, retry_backoff_initial_s=0,
        adapter=build_provider_adapter("passthrough.responses", client_protocol="responses"),
        attempt_observer=lambda exchange, attempt, phase, status, counts: emitted.append((attempt, phase, counts)))
    try:
        model.send(target="http://localhost/responses", request_bytes=b"{}", headers={},
                   exchange={"index": 0, "provider_attempts": []})
    finally:
        model.close()
    assert [(a, p) for a, p, _ in emitted] == [(1, "started"), (1, "finished"), (2, "started"), (2, "finished")]
    assert sum(c.get("generated_tokens", 0) for _, _, c in emitted) == 18


def test_hold_native_end_and_pixel_projection():
    assert assess_effect("hold", {"success": True, "feedback": {
        "steps_executed": 2, "requested_steps": 10, "stop_reason": "native_end"}})["status"] == "success"
    assert assess_effect("locate", {"success": True, "method": "pixels", "geometry": {
        "valid_count": 1, "points_xyz": [[.1, .2, .3]]}})["status"] == "success"
    assert assess_effect("locate", {"success": True, "method": "pixels", "geometry": {
        "valid_count": 1, "points_xyz": [[float("nan"), .2, .3]]}})["status"] == "unknown"


def test_session_closes_then_summary_matches_actual_jsonl(tmp_path, monkeypatch):
    import uni_robot_agent.session as module
    from uni_robot_agent.artifacts.store import ArtifactStore
    from uni_robot_agent.artifacts.records import RunRecord
    from uni_robot_agent.integrations.base import EpisodeOutcome
    from uni_robot_agent.telemetry import NullTracer
    from unittest.mock import Mock
    args = SimpleNamespace(output_dir=str(tmp_path), integration="fixture", agent_backend=None,
        model="fixture", episode_timeout_s=None, instruction="fixture", max_turns=50, reasoning_effort="none")
    integration = SimpleNamespace(episode=SimpleNamespace(instruction="fixture"), close=Mock(),
        finalize=Mock(return_value=EpisodeOutcome(success=False, metadata={"score": 0.2, "steps": 3})))

    def assemble(args):
        directory = __import__("pathlib").Path(args._session_dir)
        toolkit = Toolkit()
        toolkit.register_tools(bind_tools([ToolSpec("hold", "Fixture", schema({}))], provider_id="fixture",
            handlers={"hold": lambda: ToolOutput(extra={"feedback": {
                "steps_executed": 1, "requested_steps": 1}})}))
        return module.SessionAssembly(toolkit, SerialToolExecutor(toolkit), lambda _: None,
            integration, directory, ArtifactStore(directory), None,
            SimpleNamespace(close=lambda: None, tracer=NullTracer()))
    monkeypatch.setattr(module, "assemble_session", assemble)
    session = module.Session._from_resolved(args).start()
    assert session.tools.call("hold").success
    result = session.finish()
    assert result.record["task_success"] is False
    assert result.record["score"] == 0.2
    saved = RunRecord.open(result.session_dir)
    summary = saved.measurements
    replayed = summarize(saved.events())
    for key in replayed:
        assert replayed[key] == summary[key]
    assert summary["metrics"]["K"] == 1
    assert summary["metrics"]["T"] >= 0
    assert integration.finalize.call_count == 1
    session.finish()
    assert integration.finalize.call_count == 1


@pytest.mark.parametrize("disconnect", [False, True])
def test_gateway_emits_delivery_terminal_even_if_trace_write_fails(disconnect):
    import asyncio
    from unittest.mock import Mock
    from uni_robot_agent.provider.gateway import _GatewayResponse
    exchange = {"index": 0}
    gateway = SimpleNamespace(_model_binding=None, _model_event=Mock(),
                              _trace=SimpleNamespace(finish=Mock(side_effect=OSError("fixture disk failure"))))
    async def is_disconnected():
        return disconnect
    response = _GatewayResponse(gateway, SimpleNamespace(is_disconnected=is_disconnected), exchange,
        None, status=200, headers={}, body=b"{}", complete=True)
    async def receive():
        return {"type": "http.request", "body": b""}
    async def send(message):
        pass
    with pytest.raises(OSError, match="fixture disk failure"):
        asyncio.run(response({"type": "http"}, receive, send))
    assert gateway._model_event.call_args.args == (exchange, "delivery_failed" if disconnect else "delivered")


def test_only_explicit_typed_rejections_are_policy_owned():
    from uni_robot_agent.integrations.prepared_episode import Episode
    from uni_robot_agent.services.environment.contracts import RequestRejected
    episode = Episode.__new__(Episode)
    for owner in ("policy", "unknown"):
        def reject():
            raise RequestRejected("fixture rejection", code="fixture_contract", fault_owner=owner)
        output = episode.invoke(reject)
        assessment = assess_call({**output.to_dict(), "call_error": output.call_error})
        assert assessment["fault_owner"] == owner
        assert assessment["penalty_eligible"] is (True if owner == "policy" else None)


def test_typed_environment_service_failure_is_infrastructure():
    from uni_robot_agent.integrations.prepared_episode import Episode
    from uni_robot_agent.services.environment.client import EnvironmentRequestError
    from uni_robot_agent.services.environment.contracts import ProtocolError
    episode = Episode.__new__(Episode)
    episode.description = object()
    def fail():
        raise EnvironmentRequestError(ProtocolError(code="state_unavailable", message="fixture service error",
                                                    execution_status="unknown"))
    output = episode.invoke(fail)
    assessment = assess_call({**output.to_dict(), "call_error": output.call_error})
    assert assessment["status"] == "unknown"
    assert assessment["fault_owner"] == "infrastructure"
    assert assessment["penalty_eligible"] is False


def test_unobserved_hybrid_reviewer_invalidates_full_token_total():
    source = records()
    source[0]["unobserved_model_roles"] = ["actor_reviewer"]
    result = summarize(source)
    assert result["metrics"]["L_gen"] is None
    assert "unobserved_model_roles" in result["missing_reasons"]["L_gen"]


def test_startup_storage_failure_does_not_mask_original_error(tmp_path, monkeypatch):
    import uni_robot_agent.session as module
    args = SimpleNamespace(output_dir=str(tmp_path), integration="fixture", agent_backend=None,
                           episode_timeout_s=None, instruction="fixture")
    def fail(args):
        raise RuntimeError("original startup error")
    def disk_failure(*args):
        raise OSError("fixture disk full")
    monkeypatch.setattr(module, "assemble_session", fail)
    monkeypatch.setattr(module.RunMeasurements, "finish", disk_failure)
    session = module.Session._from_resolved(args)
    with pytest.raises(RuntimeError, match="original startup error"):
        session.start()
    assert session._closed
