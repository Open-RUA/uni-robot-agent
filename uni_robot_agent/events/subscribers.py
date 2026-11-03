"""Event selection, UI delivery, and compatible trajectory persistence."""

from __future__ import annotations

import json
import logging
import threading
from collections.abc import Callable
from dataclasses import asdict
from pathlib import Path
from typing import IO, TYPE_CHECKING, Any, Literal

from uni_robot_agent.artifacts.store import ArtifactStore
from uni_robot_agent.events.core import (
    AgentActivity, AgentRunFinished, AgentRunStarted, AssistantText, ErrorEvent,
    Event, EventBus, ToolCall, ToolCallBatch, ToolResult,
)

if TYPE_CHECKING:
    from uni_robot_agent.ui.console import ConsoleRenderer


class ConsoleSubscriber:
    """Choose the canonical tool stream and deliver it to the console renderer."""

    def __init__(self, renderer: ConsoleRenderer | None = None) -> None:
        if renderer is None:
            from uni_robot_agent.ui.console import ConsoleRenderer
            renderer = ConsoleRenderer()
        self._renderer = renderer
        self._tool_event_source: Literal["agent", "execution"] = "execution"

    def __call__(self, event: Event) -> None:
        if isinstance(event, AgentRunStarted):
            self._tool_event_source = event.tool_event_source
        if self._tool_event_source == "agent":
            if isinstance(event, (ToolCall, ToolCallBatch)):
                return
            if isinstance(event, ToolResult):
                if not event.success:
                    self._renderer.render_execution_failure(event)
                return
        self._renderer.render(event)

    def close(self) -> None:
        self._renderer.close()


class TuiSubscriber:
    """Select events without importing Textual or knowing its application state."""

    def __init__(
        self,
        render: Callable[[Event], None],
        dispatch: Callable[..., Any],
    ) -> None:
        self._render = render
        self._dispatch = dispatch
        self._tool_event_source: Literal["agent", "execution"] = "execution"

    def __call__(self, event: Event) -> None:
        if isinstance(event, AgentRunStarted):
            self._tool_event_source = event.tool_event_source
        if self._tool_event_source == "agent":
            if isinstance(event, ToolCall):
                return
            if isinstance(event, ToolResult):
                if event.success:
                    return
                event = ErrorEvent(
                    timestamp_unix_s=event.timestamp_unix_s,
                    session_id=event.session_id,
                    event_seq=event.event_seq,
                    source_step=event.tool_execution_seq,
                    turn=event.turn,
                    message=f"runtime {event.tool_name}: {event.output}",
                )
        if isinstance(event, (
            AgentRunStarted, AgentRunFinished, AssistantText, ToolCall,
            ToolCallBatch, ToolResult, ErrorEvent, AgentActivity,
        )):
            self._dispatch(self._render, event)


class EventSubscriptions:
    """Own subscription registration and deterministic detach/close."""

    def __init__(self, bus: EventBus) -> None:
        self._bus = bus
        self._subscribers: list[tuple[Any, Callable[[Event], None]]] = []
        self._errors: dict[str, str] = {}
        self._error_lock = threading.Lock()

    @property
    def errors(self) -> dict[str, str]:
        with self._error_lock:
            return dict(self._errors)

    def _record_error(self, name: str, error: Exception) -> None:
        message = f"{type(error).__name__}: {error}"
        with self._error_lock:
            self._errors[name] = message
        logging.getLogger(__name__).warning("Event output failed (%s): %s", name, message)

    def add(self, subscriber: Callable[[Event], None]) -> None:
        name = f"event_output/{len(self._subscribers)}/{type(subscriber).__name__}"
        lock = threading.Lock()
        failed = False

        def deliver(event: Event) -> None:
            nonlocal failed
            # Serialize each output and stop delivering to a broken sink. Output
            # failures must never replace an already executed physical result.
            with lock:
                if failed:
                    return
                try:
                    subscriber(event)
                except Exception as exc:
                    failed = True
                    self._record_error(name, exc)

        self._bus.subscribe(deliver)
        self._subscribers.append((subscriber, deliver))

    def close(self) -> None:
        subscribers, self._subscribers = self._subscribers, []
        # Detach every callback before closing outputs, even when one close fails.
        for _, callback in subscribers:
            self._bus.unsubscribe(callback)
        first_error: Exception | None = None
        for index in reversed(range(len(subscribers))):
            subscriber, _ = subscribers[index]
            close = getattr(subscriber, "close", None)
            if close is not None:
                try:
                    close()
                except Exception as exc:
                    self._record_error(f"event_output/{index}/{type(subscriber).__name__}/close", exc)
                    first_error = first_error or exc
        if first_error is not None:
            raise first_error


class EventJsonlSubscriber:
    """Append session events with references to the shared image store."""

    def __init__(self, session_dir: str | Path, *, artifacts: ArtifactStore):
        directory = Path(session_dir)
        directory.mkdir(parents=True, exist_ok=True)
        self._artifacts = artifacts
        self._path = directory / "trajectory.jsonl"
        # Line buffering exposes each event to readers without reopening on agent starts.
        self._file: IO[str] | None = open(self._path, "a", buffering=1, encoding="utf-8")

    @property
    def path(self) -> Path | None:
        return self._path

    def __call__(self, event: Event) -> None:
        if self._file is None:
            return

        record = event_to_record(event)
        if isinstance(event, ToolResult) and event.images:
            for image, item in zip(event.images, record["images"]):
                ref = self._artifacts.put_bytes(image.data, image.mime_type)
                item["artifact_id"] = ref["artifact_id"]
                item["path"] = self._artifacts.relative_path(ref["artifact_id"])
        self._file.write(json.dumps(record, ensure_ascii=False, default=str) + "\n")

        if isinstance(event, AgentRunFinished):
            # Physical actions can still drain after the agent stops.
            self._file.flush()

    def close(self) -> None:
        if self._file:
            stream, self._file = self._file, None
            try:
                stream.flush()
            finally:
                stream.close()


def event_to_record(event: Event) -> dict[str, Any]:
    """Convert an Event dataclass into a JSONL-compatible dictionary."""
    record = asdict(event)
    record["timestamp"] = record.pop("timestamp_unix_s")
    if isinstance(event, AssistantText):
        record["step"] = record.pop("agent_step")
    elif isinstance(event, (ToolCall, ErrorEvent)):
        record["step"] = record.pop("source_step")
    elif isinstance(event, ToolResult):
        record["step"] = record.pop("tool_execution_seq")
    if isinstance(event, AgentRunStarted):
        record["task"] = record.pop("instruction")
    if isinstance(event, ToolResult):
        record["images"] = [
            {
                "name": image.name,
                "mime_type": image.mime_type,
                "size_bytes": len(image.data),
            }
            for image in event.images
        ]
    record["type"] = {AgentRunStarted: "SessionStart", AgentRunFinished: "SessionEnd"}.get(
        type(event), type(event).__name__,
    )
    # Omit empty fields to reduce record size.
    from uni_robot_agent.artifacts.measurements import redact
    return redact({
        k: v
        for k, v in record.items()
        if v is not None and v != "" and v != {} and v != []
    })
