"""Read provider-reported counts without estimating tokens from response text."""
from uni_robot_agent.provider.protocols.responses import response_usage


def token_counts(body: bytes, protocol: str) -> dict:
    try:
        if protocol == "chat_completions":
            from uni_robot_agent.provider.protocols.chat_completions import parse_response
            usage = parse_response(body).get("usage") or {}
        elif protocol == "messages":
            from uni_robot_agent.provider.protocols.messages import response_usage as messages_usage
            usage = messages_usage(body)
        else:
            usage = response_usage(body) or {}
    except (ValueError, TypeError, KeyError):
        usage = {}
    if not isinstance(usage, dict):
        usage = {}

    def count(value):
        return value if type(value) is int and value >= 0 else None

    generated = count(usage.get("output_tokens", usage.get("completion_tokens")))
    details = usage.get("output_tokens_details", usage.get("completion_tokens_details")) or {}
    if not isinstance(details, dict):
        details = {}
    think = count(details.get("reasoning_tokens"))
    if generated is not None and think is not None and think > generated:
        think = None
    return {"generated_tokens": generated, "think_tokens": think,
            "other_tokens": generated - think if generated is not None and think is not None else None,
            "usage_complete": generated is not None}
