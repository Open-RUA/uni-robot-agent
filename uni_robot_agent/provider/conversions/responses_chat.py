"""Responses and Chat Completions conversion, independent of model providers."""

from __future__ import annotations

import copy
import json
from typing import TYPE_CHECKING, Any
from urllib.parse import urlsplit

from uni_robot_agent.provider.protocols.sse import encode_events
from uni_robot_agent.provider.protocols.chat_completions import parse_response
from uni_robot_agent.provider.conversions.tool_names import flatten_request, restore_response
from uni_robot_agent.telemetry.logging import get_logger

logger = get_logger("provider.conversions.responses_chat")
if TYPE_CHECKING:
    from uni_robot_agent.provider.contracts import DecodedResponse, EncodedRequest
_IMAGE_TYPES = {"input_image", "output_image", "image"}
_NON_MESSAGE_ITEMS = {"additional_tools"}


def target_url(upstream_base_url: str, request_path: str) -> str:
    parsed = urlsplit(request_path)
    path = parsed.path
    if path.startswith("/v1/"):
        path = path[len("/v1"):]
    if path in {"/responses", "/", ""}:
        path = "/chat/completions"
    target = upstream_base_url.rstrip("/") + path
    return f"{target}?{parsed.query}" if parsed.query else target


def function_tool(tool: dict[str, Any]) -> dict[str, Any]:
    """Express a Responses declaration as an ordinary Responses function."""
    if tool.get("type") == "web_search":
        return {
            "type": "function", "name": "web_search", "description": "Search the web.",
            "parameters": {
                "type": "object", "properties": {"query": {"type": "string"}},
                "required": ["query"], "additionalProperties": False,
            },
        }
    if not tool.get("name"):
        raise ValueError(f"Cannot map Responses tool type to Chat: {tool.get('type')!r}")
    value = dict(tool)
    if value.get("type") == "custom":
        value.pop("format", None)
        value["parameters"] = {
            "type": "object", "properties": {"input": {"type": "string"}},
            "required": ["input"], "additionalProperties": False,
        }
    value["type"] = "function"
    return value


def encode_request(
    request: dict[str, Any], *, native_search_tool: dict[str, Any] | None = None,
) -> EncodedRequest:
    from uni_robot_agent.provider.contracts import EncodedRequest

    flat, routes = flatten_request(request)
    messages = responses_messages(flat.get("instructions"), flat.get("input", []))
    _validate_chat_messages(messages)
    tools = []
    for tool in flat.get("tools", []):
        if tool.get("type") == "web_search" and native_search_tool is not None:
            tools.append(copy.deepcopy(native_search_tool))
        else:
            function = function_tool(tool)
            tools.append({"type": "function", "function": {
                key: value for key, value in function.items() if key != "type"
            }})
    wire_names = [tool["function"]["name"] for tool in tools]
    if len(wire_names) != len(set(wire_names)):
        raise ValueError("Tool name collision after Chat protocol conversion")
    stream = bool(request.get("stream", False))
    body = {"model": flat.get("model"), "messages": messages, "stream": stream}
    if tools:
        body["tools"] = tools
    for key in ("temperature", "top_p", "max_tokens", "parallel_tool_calls"):
        if key in flat:
            body[key] = flat[key]
    by_name = {
        original.get("name", original.get("type")): converted
        for original, converted in zip(flat.get("tools", []), tools)
    }
    if "tool_choice" in flat:
        body["tool_choice"] = _chat_tool_choice(flat["tool_choice"], by_name)
    search = by_name.get("web_search") if routes.get("web_search", {}).get("type") == "web_search" else None
    training = copy.deepcopy(flat)
    for declarations in [training.get("tools", []), *[
        item.get("tools", []) for item in training.get("input", [])
        if isinstance(item, dict) and item.get("type") == "additional_tools"
    ]]:
        for index, tool in enumerate(declarations):
            if tool.get("type") == "web_search":
                declarations[index] = function_tool(tool)
    if "tool_choice" in training:
        training["tool_choice"] = _training_tool_choice(training["tool_choice"])
    return EncodedRequest(body, {
        "routes": routes, "responses_stream": stream,
        "search_wire_name": search["function"]["name"] if search else None,
    }, training)


def _chat_tool_choice(choice: Any, declarations: dict[str, Any]) -> Any:
    if not isinstance(choice, dict):
        return choice
    if choice.get("type") == "allowed_tools":
        return {"type": "allowed_tools", "allowed_tools": {
            "mode": choice["mode"],
            "tools": [_chat_tool_choice(tool, declarations) for tool in choice["tools"]],
        }}
    name = choice.get("name", choice.get("type"))
    if name not in declarations:
        raise ValueError(f"Tool choice is not declared: {name!r}")
    declaration = declarations[name]
    return {"type": declaration["type"], "function": {"name": declaration["function"]["name"]}}


def _training_tool_choice(choice: Any) -> Any:
    if not isinstance(choice, dict):
        return choice
    if choice.get("type") == "web_search":
        return {"type": "function", "name": "web_search"}
    return {key: [_training_tool_choice(item) for item in value] if key == "tools" else value
            for key, value in choice.items()}


def decode_response(response: bytes, decode_state: dict[str, Any]) -> DecodedResponse:
    from uni_robot_agent.provider.contracts import DecodedResponse

    training = response_json(parse_response(response))
    search_name = decode_state.get("search_wire_name")
    if search_name:
        for item in training["output"]:
            if item.get("type") == "function_call" and item.get("name") == search_name:
                item["name"] = "web_search"
                item["id"] = item["call_id"] or item["id"]
    client = restore_response(training, decode_state["routes"])
    stream = decode_state["responses_stream"]
    return DecodedResponse(encode_response(client, stream=stream), encode_response(training, stream=stream))



def response_json(body: dict[str, Any]) -> dict[str, Any]:
    response_id = body.get("id") or "resp_chat"
    message = (body.get("choices") or [{}])[0].get("message") or {}
    output = []
    if reasoning := str(message.get("reasoning_content") or "").strip():
        output.append({"id": f"rs_{response_id}", "type": "reasoning", "status": "completed",
                       "summary": [{"type": "summary_text", "text": reasoning}]})
    if message.get("content"):
        output.append({"type": "message", "role": "assistant",
                       "content": [{"type": "output_text", "text": message["content"]}]})
    for call in message.get("tool_calls") or []:
        function = call.get("function") or {}
        output.append({
            "id": f"fc_{call.get('id') or function.get('name') or 'tool'}",
            "type": "function_call", "status": "completed", "call_id": call.get("id"),
            "name": function.get("name"), "arguments": function.get("arguments", "{}"),
        })
    result = {"id": response_id, "status": "completed", "output": output}
    if usage := body.get("usage"):
        result["usage"] = {
            "input_tokens": usage["prompt_tokens"], "output_tokens": usage["completion_tokens"],
            "total_tokens": usage["total_tokens"],
            "input_tokens_details": {"cached_tokens": (usage.get("prompt_tokens_details") or {}).get("cached_tokens", 0)},
            "output_tokens_details": {"reasoning_tokens": (usage.get("completion_tokens_details") or {}).get("reasoning_tokens", 0)},
        }
    return result



def encode_response(response: dict[str, Any], *, stream: bool) -> bytes:
    if not stream:
        return json.dumps(response, ensure_ascii=False).encode()
    events = []
    for index, item in enumerate(response["output"]):
        kind = item.get("type")
        if kind in {"function_call", "custom_tool_call"}:
            field = "arguments" if kind == "function_call" else "input"
            event_prefix = "response.function_call_arguments" if kind == "function_call" else "response.custom_tool_call_input"
            started = dict(item, status="in_progress", **{field: ""})
            events.append({"type": "response.output_item.added", "output_index": index, "item": started})
            if item.get(field):
                events.extend([
                    {"type": event_prefix + ".delta", "output_index": index,
                     "item_id": item.get("id"), "delta": item[field]},
                    {"type": event_prefix + ".done", "output_index": index,
                     "item_id": item.get("id"), field: item[field]},
                ])
        events.append({"type": "response.output_item.done", "output_index": index, "item": item})
    events.append({"type": "response.completed", "response": response})
    for sequence, event in enumerate(events):
        event["sequence_number"] = sequence
    return encode_events(events, has_done=True)



def responses_messages(instructions: Any, items: list[Any]) -> list[dict[str, Any]]:
    messages: list[dict[str, Any]] = []
    pending_reasoning: list[str] = []
    assistant: dict[str, Any] | None = None
    call_names: dict[str, str] = {}

    def flush_assistant() -> None:
        nonlocal assistant
        if assistant is None:
            if pending_reasoning:
                messages.append({
                    "role": "assistant",
                    "content": "",
                    "reasoning_content": _take_reasoning(pending_reasoning),
                })
            return
        if pending_reasoning:
            assistant["reasoning_content"] = _take_reasoning(pending_reasoning)
        messages.append(assistant)
        assistant = None

    def ensure_assistant() -> dict[str, Any]:
        nonlocal assistant
        if assistant is None:
            assistant = {"role": "assistant", "content": ""}
        return assistant

    def append_assistant_content(content: str) -> None:
        if not content:
            return
        message = ensure_assistant()
        previous = message.get("content") or ""
        message["content"] = f"{previous}\n{content}" if previous else content

    if instructions:
        messages.append({"role": "system", "content": instructions})
    if isinstance(items, str):
        return messages + [{"role": "user", "content": items}]
    index = 0
    while index < len(items):
        item = items[index]
        if not isinstance(item, dict):
            index += 1
            continue
        kind = item.get("type") or ("message" if "role" in item else None)
        if kind == "reasoning":
            reasoning = _text(item.get("summary") or item.get("content")).strip()
            if reasoning:
                pending_reasoning.append(reasoning)
        elif kind == "message":
            role = item.get("role")
            if role in {"system", "developer"}:
                flush_assistant()
                messages.append({"role": "system", "content": _text(item.get("content"))})
            elif role == "user":
                flush_assistant()
                messages.append({"role": "user", "content": _content_parts(item.get("content"))})
            elif role == "assistant":
                append_assistant_content(_text(item.get("content")))
        elif kind in {"function_call", "custom_tool_call"}:
            message = ensure_assistant()
            tool_calls = message.setdefault("tool_calls", [])
            while index < len(items):
                call = items[index]
                if not isinstance(call, dict) or call.get("type") not in {
                    "function_call", "custom_tool_call"
                }:
                    break
                call_id = call.get("call_id")
                name = call.get("name")
                arguments = call.get("arguments")
                if arguments is None:
                    arguments = json.dumps({"input": call.get("input", "")}, ensure_ascii=False)
                tool_calls.append({
                    "id": call_id,
                    "type": "function",
                    "function": {
                        "name": name,
                        "arguments": arguments,
                    },
                })
                if isinstance(call_id, str) and isinstance(name, str) and name:
                    call_names[call_id] = name
                index += 1
            continue
        elif kind in {"function_call_output", "custom_tool_call_output"}:
            flush_assistant()
            call_id = item.get("call_id")
            # The Chat representation keeps image_url in tool messages. Keep images in their matching
            # results; extra user messages would interrupt parallel tool sequences.
            content = _content_parts(item.get("output"))
            tool_message: dict[str, Any] = {
                "role": "tool",
                "tool_call_id": call_id,
                "content": content,
            }
            if isinstance(call_id, str) and call_names.get(call_id):
                tool_message["name"] = call_names[call_id]
            messages.append(tool_message)
        elif kind not in _NON_MESSAGE_ITEMS:
            logger.warning("Chat codec ignored unsupported input item: %r", kind)
        index += 1
    flush_assistant()
    return messages


def _text(value: Any) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, list):
        return "\n".join(str(block.get("text", "")) for block in value if isinstance(block, dict))
    return json.dumps(value, ensure_ascii=False) if value is not None else ""


def _validate_chat_messages(messages: list[dict[str, Any]]) -> None:
    outstanding: dict[str, str] = {}
    previous_role: str | None = None

    for index, message in enumerate(messages):
        role = message.get("role")
        if role == "assistant":
            if previous_role == "assistant":
                raise ValueError(f"consecutive assistant messages at index {index}")
            if outstanding:
                raise ValueError(
                    f"assistant message at index {index} interrupts pending tool calls"
                )
            tool_calls = message.get("tool_calls") or []
            if not isinstance(tool_calls, list):
                raise ValueError(f"assistant tool_calls at index {index} must be a list")
            for call in tool_calls:
                if not isinstance(call, dict):
                    raise ValueError(f"invalid tool call at message index {index}")
                call_id = call.get("id")
                function = call.get("function") or {}
                name = function.get("name")
                if not isinstance(call_id, str) or not call_id:
                    raise ValueError(f"tool call at index {index} has no id")
                if call_id in outstanding:
                    raise ValueError(f"duplicate tool call id: {call_id}")
                if not isinstance(name, str) or not name:
                    raise ValueError(f"tool call {call_id} has no function name")
                outstanding[call_id] = name
        elif role == "tool":
            call_id = message.get("tool_call_id")
            if not isinstance(call_id, str) or not call_id:
                raise ValueError(f"tool message at index {index} has no tool_call_id")
            if call_id not in outstanding:
                raise ValueError(f"tool result has no matching call: {call_id}")
            expected_name = outstanding.pop(call_id)
            name = message.get("name")
            if name is not None and name != expected_name:
                raise ValueError(
                    f"tool result {call_id} names {name!r}, expected {expected_name!r}"
                )
        else:
            if outstanding:
                raise ValueError(
                    f"{role!r} message at index {index} interrupts pending tool calls"
                )
        previous_role = role

    if outstanding:
        raise ValueError(f"unmatched tool calls: {sorted(outstanding)}")


def _has_image(value: Any) -> bool:
    return isinstance(value, list) and any(
        isinstance(block, dict) and block.get("type") in _IMAGE_TYPES
        for block in value
    )


def _image_part(block: dict[str, Any]) -> dict[str, Any] | None:
    """Convert Responses image blocks into Chat image_url parts."""
    url = block.get("image_url") or block.get("url")
    # Responses accepts image_url as either a data URL string or {"url": ...}.
    if isinstance(url, dict):
        url = url.get("url")
    if not isinstance(url, str) or not url:
        data, mime = block.get("data"), block.get("mime_type") or block.get("mimeType")
        if not isinstance(data, str) or not data:
            return None
        url = f"data:{mime or 'image/png'};base64,{data}"
    part: dict[str, Any] = {"type": "image_url", "image_url": {"url": url}}
    if detail := block.get("detail"):
        part["image_url"]["detail"] = detail
    return part


def _content_parts(value: Any) -> Any:
    """Preserve image content; keep plain text as strings to retain the request shape."""
    if not _has_image(value):
        return _text(value)
    parts: list[dict[str, Any]] = []
    for block in value:
        if not isinstance(block, dict):
            continue
        if block.get("type") in _IMAGE_TYPES:
            if part := _image_part(block):
                parts.append(part)
            else:
                logger.warning("Chat codec dropped an unreadable image block")
            continue
        if text := str(block.get("text") or ""):
            parts.append({"type": "text", "text": text})
    return parts or ""


def _take_reasoning(parts: list[str]) -> str:
    text = "\n\n".join(parts)
    parts.clear()
    return text
