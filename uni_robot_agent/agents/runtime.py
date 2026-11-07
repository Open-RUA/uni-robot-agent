"""Model services supplied by an agent to the shared session owner."""
from dataclasses import dataclass
from typing import Callable
from uni_robot_agent.provider.contracts import ProviderAdapter
from uni_robot_agent.provider.contracts import ModelBinding, ModelProtocol

@dataclass(frozen=True)
class ModelRuntime:
    trace_directory: str
    adapter: ProviderAdapter
    protocol: ModelProtocol
    binding: ModelBinding | None
    finalize_records: Callable


def add_model_arguments(parser):
    """Shared transport options, independent of installed agent SDKs."""
    import argparse
    from uni_robot_agent.config import _nonnegative, _positive
    from uni_robot_agent.context.history import list_context_policies
    from uni_robot_agent.provider.registry import list_provider_profiles
    options = parser.add_argument_group("Model transport and context")
    options.add_argument(
        "--reasoning-effort",
        choices=("none", "minimal", "low", "medium", "high", "xhigh", "max"),
        default=None,
        help="Reasoning effort (default: max for Codex, none for Claude)",
    )
    options.add_argument("--provider-url")
    options.add_argument(
        "--provider-header",
        dest="provider_headers",
        action="append",
        default=None,
        metavar="NAME=VALUE",
        help="Repeatable custom upstream HTTP header; replaces the SDK header with the same name",
    )
    options.add_argument(
        "--provider-profile",
        choices=list_provider_profiles(),
        default=None,
        help="Provider service and protocol (default: passthrough for the selected agent protocol)",
    )
    options.add_argument(
        "--provider-timeout-s",
        type=_positive,
        default=600,
        help="Main agent HTTP request timeout in seconds",
    )
    options.add_argument(
        "--provider-retries",
        type=_nonnegative,
        default=2,
        help="Additional attempts for a main agent HTTP request",
    )
    options.add_argument("--provider-retry-backoff-s", type=_nonnegative, default=5)
    options.add_argument(
        "--model-parallel-tool-calls",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Allow multiple model tool calls per response",
    )
    context = parser.add_argument_group("Context policy")
    context.add_argument(
        "--context-policy", choices=list_context_policies(), default="identity"
    )
    context.add_argument(
        "--context-max-input-tokens",
        type=_positive,
        help="Maximum estimated input tokens",
    )
    context.add_argument(
        "--context-target-input-tokens",
        type=_positive,
        help="Target estimated input tokens after trimming",
    )
    for name, default, value_type in (
        ("keep-recent-blocks", 4, _nonnegative),
        ("keep-observation-blocks", 1, _nonnegative),
        ("min-recent-tool-images", 3, _nonnegative),
        ("image-eviction-batch-size", 20, _positive),
    ):
        context.add_argument(
            "--context-" + name,
            type=value_type,
            default=default,
        )
