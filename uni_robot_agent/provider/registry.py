"""Provider profiles and supported client/upstream protocol pairs."""

from dataclasses import dataclass
from functools import partial
from typing import Callable

from uni_robot_agent.provider.backends.tencent.responses import TencentResponsesAdapter
from uni_robot_agent.provider.contracts import ProviderAdapter
from uni_robot_agent.provider.backends.passthrough import PassthroughAdapter
from uni_robot_agent.provider.backends.aliyun.chat_completions import (
    AliyunChatCompletionsAdapter,
)
from uni_robot_agent.provider.backends.aliyun.messages import AliyunMessagesAdapter
from uni_robot_agent.provider.backends.aliyun.gemini_chat_completions import AliyunGeminiChatAdapter
from uni_robot_agent.provider.backends.volcengine.responses import (
    VolcengineResponsesAdapter,
)
from uni_robot_agent.provider.backends.moonshot.chat_completions import (
    MoonshotChatCompletionsAdapter,
)
from uni_robot_agent.provider.backends.deepseek.chat_completions import (
    DeepSeekChatCompletionsAdapter,
)
from uni_robot_agent.provider.backends.minimax.chat_completions import (
    MiniMaxChatCompletionsAdapter,
)
from uni_robot_agent.provider.backends.openai.chat_completions import (
    OpenAIChatCompletionsAdapter,
)


@dataclass(frozen=True)
class ProviderProfile:
    name: str
    client_protocol: str
    upstream_protocol: str
    factory: Callable[[], ProviderAdapter]


_PROFILES = {
    profile.name: profile
    for profile in (
        ProviderProfile('aliyun.gemini_chat_completions', 'gemini', 'chat_completions', AliyunGeminiChatAdapter),
        ProviderProfile(
            "passthrough.responses",
            "responses",
            "responses",
            partial(PassthroughAdapter, "responses"),
        ),
        ProviderProfile(
            "passthrough.messages",
            "messages",
            "messages",
            partial(PassthroughAdapter, "messages"),
        ),
        ProviderProfile(
            "aliyun.chat_completions",
            "responses",
            "chat_completions",
            AliyunChatCompletionsAdapter,
        ),
        ProviderProfile(
            "aliyun.messages", "messages", "messages", AliyunMessagesAdapter
        ),
        ProviderProfile(
            "tencent.responses", "responses", "responses", TencentResponsesAdapter
        ),
        ProviderProfile(
            "volcengine.responses", "responses", "responses", VolcengineResponsesAdapter
        ),
        ProviderProfile(
            "moonshot.chat_completions",
            "responses",
            "chat_completions",
            MoonshotChatCompletionsAdapter,
        ),
        ProviderProfile(
            "deepseek.chat_completions",
            "responses",
            "chat_completions",
            DeepSeekChatCompletionsAdapter,
        ),
        ProviderProfile(
            "minimax.chat_completions",
            "responses",
            "chat_completions",
            MiniMaxChatCompletionsAdapter,
        ),
        ProviderProfile(
            "openai.chat_completions",
            "responses",
            "chat_completions",
            OpenAIChatCompletionsAdapter,
        ),
    )
}


def list_provider_profiles(*, client_protocol: str | None = None) -> tuple[str, ...]:
    return tuple(
        name
        for name, profile in _PROFILES.items()
        if client_protocol is None or profile.client_protocol == client_protocol
    )


def validate_provider_profile(name: str, *, client_protocol: str) -> ProviderProfile:
    try:
        profile = _PROFILES[name]
    except KeyError:
        raise ValueError(
            f"Unknown provider profile {name!r}; choose from {', '.join(list_provider_profiles())}"
        ) from None
    if profile.client_protocol != client_protocol:
        raise ValueError(
            f"Provider profile {name!r} requires {profile.client_protocol} input, "
            f"not {client_protocol}; choose from {', '.join(list_provider_profiles(client_protocol=client_protocol))}"
        )
    return profile


def build_provider_adapter(name: str, *, client_protocol: str) -> ProviderAdapter:
    return validate_provider_profile(name, client_protocol=client_protocol).factory()
