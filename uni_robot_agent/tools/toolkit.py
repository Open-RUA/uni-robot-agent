"""Tool declarations, validated bindings, results, and the Toolkit registry."""

from __future__ import annotations

import threading
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any, Callable, Literal

from uni_robot_agent.telemetry.logging import get_logger

logger = get_logger("toolkit")


@dataclass
class ToolDef:
    """Runtime entry produced by binding a ToolSpec to its handler."""

    name: str
    spec: dict[str, Any]  # Tool name, description and JSON input_schema.
    handler: Callable[..., Any]
    readonly: bool = False
    provider_id: str = "unknown"


@dataclass(frozen=True)
class ImageAttachment:
    """Binary image attached to a tool result, kept outside the JSON payload."""

    data: bytes = field(repr=False)
    mime_type: str = "image/png"
    name: str | None = None


@dataclass(frozen=True)
class ActionOutcome:
    """What a physical action did, for the executor's replan handling; never sent to the model.

    ``complete`` means the requested action finished as planned. ``executed`` is
    ``"none"`` when nothing reached the hardware, ``"some"`` when it did, and
    ``"unknown"`` when a lost acknowledgement leaves it open. ``stopped`` is true only
    when the environment confirmed the actuators are stationary afterwards, and
    ``None`` when it does not say.
    """

    complete: bool
    executed: Literal["none", "some", "unknown"]
    stopped: bool | None = None


@dataclass
class ToolOutput:
    """Common return type for tool handlers.

    success means the call completed with a valid result, not that a subgoal or
    episode succeeded. Put domain outcomes and terminated/truncated state in extra.
    Common fields provide consistency; extra carries tool-specific fields.
    The framework serializes through .to_dict() before returning to the agent.
    """

    success: bool = True
    error: str | None = None
    obs: dict[str, Any] | None = None
    extra: dict[str, Any] = field(default_factory=dict)
    images: list[ImageAttachment] = field(default_factory=list)
    # Read by the executor only; to_dict() never serializes it.
    action: ActionOutcome | None = None

    def to_dict(self, *, for_model: bool = False) -> dict[str, Any]:
        """Keep internal call status; name its meaning explicitly on the model wire."""
        d: dict[str, Any] = ({"call_ok": self.success and not bool(self.error)}
                             if for_model else {"success": self.success})
        if self.error:
            d["error"] = self.error
        if self.obs is not None:
            d["obs"] = self.obs
        d.update(self.extra)
        return d


class ToolProvider:
    """Supply explicitly bound tools and optional provider-owned instructions."""

    provider_id = "unknown"

    @property
    def prompt_contributions(self):
        """Optional instructions owned by this tool provider."""
        return ()

    def get_tools(self) -> list[ToolDef]:
        raise NotImplementedError

    def get_declared_tools(self) -> list[ToolDef]:
        """Declare selectable bindings without enabling optional backends or permissions."""
        return self.get_tools()


class Toolkit:
    """Register tools, expose schemas, and dispatch calls."""

    def __init__(self):
        self._tools: dict[str, ToolDef] = {}
        self._event_bus: Any = None
        self._tool_execution_seq: int = 0
        self._sequence_lock = threading.Lock()
        self._solved_fn: Callable[[], bool | None] | None = None

    def register_tools(self, tools: list[ToolDef]) -> None:
        for tool_def in tools:
            if tool_def.name in self._tools:
                raise ValueError(f"tool already registered: {tool_def.name}")
            self._tools[tool_def.name] = tool_def

    def set_solved_fn(self, fn: Callable[[], bool | None]) -> None:
        """Set the official task outcome callback read at agent completion."""
        self._solved_fn = fn

    def task_succeeded(self) -> bool | None:
        """Return official task completion, or None when no callback is available."""
        if self._solved_fn is None:
            return None
        result = self._solved_fn()
        return None if result is None else bool(result)

    def set_event_bus(self, bus: Any) -> None:
        """Inject the EventBus used by execute_tool() to emit ToolResult."""
        self._event_bus = bus

    def get_tool_schemas(self) -> list[dict[str, Any]]:
        """Return tool declarations for encoding by the selected agent adapter."""
        return [t.spec for t in self._tools.values()]

    def execute_tool(self, name: str, input_dict: dict[str, Any], *,
                     result_transform: Callable[[ToolOutput], ToolOutput] | None = None) -> ToolOutput:
        """Execute a tool, return its structured result, and emit ToolResult.

        ``result_transform`` lets the executor annotate the result before it is recorded.
        """
        tool_def = self._tools.get(name)
        if tool_def is None:
            return ToolOutput(success=False, error=f"Unknown tool: {name}")

        # Handlers may run concurrently; allocate execution sequence numbers under a separate lock.
        with self._sequence_lock:
            self._tool_execution_seq += 1
            tool_execution_seq = self._tool_execution_seq

        # Emit ToolResult even when a handler raises so the failure is recorded.
        try:
            result = tool_def.handler(**input_dict)
        except Exception as e:
            import traceback

            tb_lines = traceback.format_exc().splitlines()
            logger.error("tool %s raised:\n%s", name, "\n".join(tb_lines))
            result = ToolOutput(
                success=False,
                error=f"{type(e).__name__}: {e}",
                extra={"traceback": "\n".join(tb_lines[-10:])},
            )
        else:
            if not isinstance(result, ToolOutput):
                raise TypeError(
                    f"tool {name!r} returned {type(result).__name__}, "
                    f"expected ToolOutput"
                )

        if result_transform is not None:
            result = result_transform(result)
        self.record_result(name, input_dict, result, tool_execution_seq=tool_execution_seq)
        return result

    def record_result(self, name: str, arguments: dict[str, Any], result: ToolOutput,
                      *, tool_execution_seq: int = 0) -> None:
        """Record a receipt, including admission failures with no executed step."""
        result_dict = result.to_dict()
        if self._event_bus:
            from uni_robot_agent.events import ToolResult as ToolResultEvent
            from uni_robot_agent.execution import current_execution_id
            from uni_robot_agent.execution.correlation import current_call

            self._event_bus.emit(
                ToolResultEvent(
                    tool_name=name,
                    arguments=arguments,
                    success=result_dict.get("success", True),
                    output=result_dict,
                    images=tuple(result.images),
                    tool_execution_seq=tool_execution_seq,
                    execution_id=current_execution_id() or "", **current_call(),
                )
            )

    def get_tool_names(self) -> list[str]:
        return list(self._tools.keys())

    def get_tool_provider_id(self, name: str) -> str:
        tool_def = self._tools.get(name)
        return tool_def.provider_id if tool_def is not None else "unknown"

    @property
    def tool_execution_seq(self) -> int:
        return self._tool_execution_seq

    def close(self) -> None:
        pass


@dataclass(frozen=True)
class ToolSpec:
    """A model-facing schema independent of the object handling the call."""

    name: str
    description: str
    input_schema: dict
    readonly: bool = False


def schema(properties, required=()):
    return {"type": "object", "properties": properties, "required": list(required),
            "additionalProperties": False}

def constraint_summary(input_schema):
    """Keep bounds visible in descriptions when a client simplifies JSON Schema."""
    rows = []

    def visit(node, path):
        labels = []
        for key, label in (("minimum", ">="), ("exclusiveMinimum", ">"),
                           ("maximum", "<="), ("exclusiveMaximum", "<"),
                           ("minItems", "min items="), ("maxItems", "max items="),
                           ("minLength", "min length="), ("maxLength", "max length=")):
            if key in node:
                labels.append(f"{label}{node[key]}")
        if node.get("uniqueItems"):
            labels.append("unique items")
        if labels:
            rows.append(f"{path or 'arguments'}: {', '.join(labels)}")
        for name, child in node.get("properties", {}).items():
            visit(child, f"{path}.{name}" if path else name)
        if isinstance(node.get("items"), dict):
            visit(node["items"], path + "[]")

    visit(input_schema, "")
    return " Parameter constraints: " + "; ".join(rows) + "." if rows else ""


def bind_tools(specs, *, provider_id, handlers, invoke=None):
    """Validate arguments before dispatching to explicitly owned handlers."""
    import jsonschema

    def bind(item, handler):
        def handle(**arguments):
            try:
                jsonschema.validate(arguments, item.input_schema)
            except jsonschema.ValidationError as error:
                field = ".".join(map(str, error.absolute_path)) or "arguments"
                return ToolOutput(success=False, error=f"{field}: {error.message}",
                                  extra={"no_execution": True, "validation": {
                                      "field": field, "constraint": error.validator,
                                      "allowed": error.validator_value, "received": error.instance}})
            return handler(**arguments) if invoke is None else invoke(handler, **arguments)

        return ToolDef(
            name=item.name,
            spec={"name": item.name, "description": item.description + constraint_summary(item.input_schema),
                  "input_schema": deepcopy(item.input_schema)},
            handler=handle, readonly=item.readonly, provider_id=provider_id,
        )

    return [bind(item, handlers[item.name]) for item in specs]
