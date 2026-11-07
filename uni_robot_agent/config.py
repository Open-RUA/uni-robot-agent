"""Resolve and validate configuration before connecting to any service."""

from __future__ import annotations

import argparse
import os
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

from uni_robot_agent.context.prompt import get_prompt_builder
from uni_robot_agent.provider.headers import parse_provider_headers


def _nonnegative(value: str) -> int:
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be a nonnegative integer")
    return number


def _positive(value: str) -> int:
    number = _nonnegative(value)
    if number == 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def runtime_env(name, default=None):
    """Read a setting from the product's environment-variable namespace."""
    return os.environ.get("UNI_ROBOT_AGENT_" + name, default)


def resolve_config(args, parser, integration_spec, argv=()):
    """CLI adapter: translate shared validation errors into argparse diagnostics."""
    explicit = {word.split("=", 1)[0] for word in argv if word.startswith("--")}
    for owner, options in getattr(parser, "agent_options", {}).items():
        if owner != args.agent_backend and (used := explicit & options):
            parser.error(f"{sorted(used)[0]} requires --agent {owner}")
    try:
        return resolve_settings(args, integration_spec, explicit=explicit)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))


def resolve_settings(args, integration_spec, *, explicit=(), agent_enabled=True):
    """Resolve one runtime configuration without parsing arguments or exiting."""
    explicit = set(explicit)
    raw_headers = getattr(args, "provider_headers", None)
    if raw_headers is None:
        raw_headers = getattr(args, "_provider_headers", None)
    args._provider_headers = parse_provider_headers(raw_headers)
    # The raw CLI entries contain secret values; keep only the internal parsed copy.
    vars(args).pop("provider_headers", None)
    from uni_robot_agent.agents.registry import get_agent_backend
    if agent_enabled:
        get_agent_backend(args.agent_backend).resolve_defaults(args)
    # Read environment variables at the entry point; business modules receive resolved values.
    if args.no_sam3 and args.sam3_url:
        raise ValueError("--sam3-url and --no-sam3 are mutually exclusive")
    for name in (
        "env_url",
        "action_model_url",
        "sam3_url",
        "provider_url",
        "output_dir",
    ):
        if getattr(args, name) is None:
            setattr(args, name, runtime_env(name.upper()))
    args._api_key = getattr(args, "_api_key", None) or runtime_env("API_KEY")
    if hasattr(args, "_vla_review_api_key") and args._vla_review_api_key is None:
        args._vla_review_api_key = runtime_env("VLA_REVIEW_API_KEY", args._api_key)
    args.output_dir = args.output_dir or "outputs"
    if args.no_sam3:
        args.sam3_url = None
    from urllib.parse import urlsplit

    for name in ("env_url", "action_model_url", "sam3_url", "provider_url"):
        value = getattr(args, name)
        if value:
            try:
                url = urlsplit(value)
                valid = (
                    url.scheme in {"http", "https", "socket", "tcp", "ws", "wss"}
                    and url.hostname
                    and (url.port is None or url.port > 0)
                )
            except ValueError:
                valid = False
            if not valid:
                raise ValueError(
                    f"--{name.replace('_', '-')} must be a valid absolute service URL"
                )
    if args.env_url and urlsplit(args.env_url).scheme not in {"http", "https"}:
        raise ValueError("--env-url must be a prepared episode HTTP(S) URL")
    if args.context_policy == "identity" and any(
        flag.startswith("--context-") and flag != "--context-policy"
        for flag in explicit
    ):
        raise ValueError(
            "context budget/image options require --context-policy sliding_window"
        )
    local = getattr(args, "deploy", None) == "local"
    if getattr(args, "camera_calibration", None) is not None and not local:
        raise ValueError("--camera-calibration/--no-camera-calibration require --deploy local; "
                         "configure an external environment service at startup instead")
    if local and args.env_url:
        raise ValueError("--deploy local and --env-url (including environment variables) are mutually exclusive")
    if not args.env_url and not local:
        raise ValueError("--env-url or UNI_ROBOT_AGENT_ENV_URL is required (or select --deploy local)")
    if local:
        from uni_robot_agent.services.deployment.config import validate_deployment
        validate_deployment(args)
    if args.observation_profile != "none":
        if args.resource_scheduling != "per-resource":
            raise ValueError(
                "--observation-profile requires --resource-scheduling per-resource"
            )
        if args.observation_profile != args.integration:
            raise ValueError(
                "--observation-profile must match --integration or be none"
            )
        from uni_robot_agent.observation import load_observation_profile

        load_observation_profile(args.observation_profile)
    if agent_enabled:
        agent_backend = get_agent_backend(args.agent_backend)
        from uni_robot_agent.provider.registry import validate_provider_profile
        args.provider_profile = args.provider_profile or f"passthrough.{agent_backend.model_protocol}"
        validate_provider_profile(args.provider_profile, client_protocol=agent_backend.model_protocol)
        agent_backend.validate_config(args, explicit)
        if args.context_policy != "identity" and not args.provider_url:
            raise ValueError(
                "context policy requires --provider-url or UNI_ROBOT_AGENT_PROVIDER_URL"
            )
        context = resolve_context_config(args)
        if args.context_policy == "sliding_window":
            from uni_robot_agent.context.history import create_context_policy

            create_context_policy(args.context_policy, **context["context_options"])
    get_prompt_builder(args.integration)
    integration_spec.validate_cli(args)
    if getattr(args, "skill_writing", False):
        from uni_robot_agent.tools.skills import skill_root

        skill_root(writing=True)
    return args

def public_config(args) -> dict:
    from urllib.parse import urlsplit, urlunsplit

    result = {
        k: str(v) if isinstance(v, Path) else v
        for k, v in vars(args).items()
        if k not in {"print_config", "provider_headers", "validate_config"}
        and not k.startswith("_")
    }
    # URLs may contain credentials or query tokens; strip them from all configuration output.
    for name, value in result.items():
        if (name.endswith("_url") or name.endswith("_endpoint")) and value:
            parsed = urlsplit(value)
            result[name] = urlunsplit(
                (parsed.scheme, parsed.netloc.rsplit("@", 1)[-1], parsed.path, "", "")
            )
    return result


def resolve_context_config(args) -> dict:
    """Collect context policy settings independently of SDK selection."""
    policy = getattr(args, "context_policy", "identity")

    options: dict[str, int] = {}
    if policy == "sliding_window":
        maximum = getattr(args, "context_max_input_tokens", None)
        target = getattr(args, "context_target_input_tokens", None)
        if maximum is None or target is None:
            raise ValueError(
                "sliding_window requires --context-max-input-tokens and "
                "--context-target-input-tokens"
            )
        options = {
            "max_estimated_input_tokens": maximum,
            "target_estimated_input_tokens": target,
            "reserve_output_tokens": 0,
            "keep_recent_blocks": args.context_keep_recent_blocks,
            "keep_observation_blocks": args.context_keep_observation_blocks,
            "min_recent_tool_images": args.context_min_recent_tool_images,
            "image_eviction_batch_size": args.context_image_eviction_batch_size,
        }
    return {"context_policy": policy, "context_options": options}


@dataclass(frozen=True)
class AgentModelConfig:
    """Resolved main-agent transport settings, separate from SDK options."""

    base_url: str | None
    api_key: str | None
    adapter: str
    request_timeout_s: float
    max_retries: int
    retry_backoff_initial_s: float
    parallel_tool_calls: bool
    provider_headers: Mapping[str, str] | None = None


def resolve_agent_model_config(args) -> AgentModelConfig:
    return AgentModelConfig(
        base_url=args.provider_url,
        api_key=args._api_key,
        adapter=args.provider_profile,
        request_timeout_s=args.provider_timeout_s,
        max_retries=args.provider_retries,
        retry_backoff_initial_s=args.provider_retry_backoff_s,
        parallel_tool_calls=args.model_parallel_tool_calls,
        provider_headers=dict(getattr(args, "_provider_headers", None) or {}),
    )




class AgentConfig(BaseModel):
    """Built-in planner settings; credentials are excluded from serialized configuration."""

    model_config = ConfigDict(extra="forbid", frozen=True)
    model: str = Field(min_length=1)
    agent_backend: str = "codex"
    reasoning_effort: Literal["none", "minimal", "low", "medium", "high", "xhigh", "max"] | None = None
    max_turns: int = Field(default=0, ge=0)
    provider_url: str | None = None
    provider_profile: str | None = None
    api_key: str | None = Field(default=None, exclude=True, repr=False)
    provider_headers: dict[str, str] | None = Field(
        default=None, exclude=True, repr=False
    )
    provider_timeout_s: int = Field(default=600, gt=0)
    provider_retries: int = Field(default=2, ge=0)
    provider_retry_backoff_s: int = Field(default=5, ge=0)
    model_parallel_tool_calls: bool = True
    context_policy: Literal["identity", "sliding_window"] = "identity"
    context_max_input_tokens: int | None = Field(default=None, gt=0)
    context_target_input_tokens: int | None = Field(default=None, gt=0)
    context_keep_recent_blocks: int = Field(default=4, ge=0)
    context_keep_observation_blocks: int = Field(default=1, ge=0)
    context_min_recent_tool_images: int = Field(default=3, ge=0)
    context_image_eviction_batch_size: int = Field(default=20, gt=0)
    agent_python: str | None = None
    opencode_binary: str = "opencode"
    opencode_version: str | None = None
    gemini_binary: str = "gemini"
    gemini_version: str | None = None
    gemini_bridge_manifest: str | None = None
    codex_base_prompt: Literal["default", "robot"] = "robot"
    codex_web_search: Literal["disabled", "cached", "live"] | None = None
    mcp_parallel_dispatch: bool = False
    mcp_timeout_s: int | None = Field(default=None, gt=0)
    claude_max_budget_usd: float | None = Field(default=None, gt=0, allow_inf_nan=False)
    claude_effort: Literal["low", "medium", "high", "max"] | None = None


class SessionConfig(BaseModel):
    """One prepared episode. Integration-specific options belong to its registered schema."""

    model_config = ConfigDict(extra="forbid", frozen=True)
    integration: str = Field(min_length=1)
    env_url: str | None = None
    agent: AgentConfig | None = None
    integration_options: dict[str, Any] = Field(default_factory=dict, repr=False)
    instruction: str | None = None
    action_model_url: str | None = None
    sam3_url: str | None = None
    no_sam3: bool = False
    output_dir: Path | None = None
    episode_timeout_s: int = Field(default=10800, ge=0)
    interaction_mode: Literal["tool_calling", "code_as_policy"] = "tool_calling"
    tool_return_mode: Literal["immediate", "background"] = "background"
    resource_scheduling: Literal["per-resource", "global-serial"] = "per-resource"
    observation_profile: str = "none"
    skill_writing: bool = False
    tool_debug: bool = False
    tools: list[str] = Field(default_factory=list)
    save_exchanges: bool = True
    telemetry: Literal["off", "standard", "debug"] = "standard"

    def resolve(self):
        """Validate offline and return the internal shared runtime settings."""
        from uni_robot_agent.integrations import load_integration

        spec = load_integration(self.integration).get_integration_spec()
        options = spec.config_model.model_validate(self.integration_options)
        values = self.model_dump(exclude={"agent", "integration_options"})
        values.update({name: getattr(options, name) for name in type(options).model_fields})
        # Tool-only sessions retain model transport defaults for shared tool composition,
        # but do not load, validate or instantiate an agent backend.
        agent = self.agent
        values.update(agent.model_dump(exclude={"api_key"}) if agent else {
            name: field.get_default() for name, field in AgentConfig.model_fields.items()
            if name not in {"model", "api_key"}
        })
        values.update(
            command="run",
            deploy=None,
            _api_key=agent.api_key if agent else None,
            _provider_headers=agent.provider_headers if agent else None,
        )
        if self.agent is None:
            values.update(agent_backend=None, model=None)
        if "vla_review_api_key" in values:
            values["_vla_review_api_key"] = values.pop("vla_review_api_key")
        explicit = {"--" + name.replace("_", "-") for name in self.model_fields_set}
        if self.agent:
            explicit.update("--" + name.replace("_", "-") for name in agent.model_fields_set)
            if agent.agent_backend != 'gemini_cli' and any(
                name.startswith('gemini_') for name in agent.model_fields_set
            ):
                raise ValueError("Gemini settings require agent_backend='gemini_cli'")
            if agent.agent_backend != 'opencode' and any(
                name.startswith('opencode_') for name in agent.model_fields_set
            ):
                raise ValueError("OpenCode settings require agent_backend='opencode'")
            if agent.agent_backend != "codex" and any(
                name == "agent_python" or name.startswith(("codex_", "mcp_"))
                for name in agent.model_fields_set
            ):
                raise ValueError("Codex settings require agent_backend='codex'")
            if agent.agent_backend != "claude_code" and any(
                name.startswith("claude_") for name in agent.model_fields_set
            ):
                raise ValueError("Claude settings require agent_backend='claude_code'")
        return resolve_settings(SimpleNamespace(**values), spec, explicit=explicit,
                                agent_enabled=self.agent is not None)
