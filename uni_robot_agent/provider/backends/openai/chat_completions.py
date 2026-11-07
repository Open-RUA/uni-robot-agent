"""Generic OpenAI-compatible Chat Completions settings on the shared codec."""

from __future__ import annotations

from typing import Any

from uni_robot_agent.provider.conversions import responses_chat
from uni_robot_agent.provider.contracts import DecodedResponse, EncodedRequest


class OpenAIChatCompletionsAdapter:
    """Translate Responses input for OpenAI's Chat Completions API and compatible servers."""

    profile_name = "openai.chat_completions"

    def target_url(self, upstream_base_url: str, request_path: str) -> str:
        return responses_chat.target_url(upstream_base_url, request_path)

    def encode_request(self, request: dict[str, Any]) -> EncodedRequest:
        encoded = responses_chat.encode_request(request)
        if encoded.body["stream"]:
            # Chat streaming reports usage only when requested; keep an explicit
            # stream_options value from the caller instead of overriding it.
            encoded.body["stream_options"] = request.get(
                "stream_options", {"include_usage": True}
            )
        return encoded

    def decode_response(self, response: bytes, decode_state: dict[str, Any]) -> DecodedResponse:
        return responses_chat.decode_response(response, decode_state)
