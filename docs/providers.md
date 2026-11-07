# Model providers

Provider profiles describe a service endpoint and its wire protocol, independently
of the planner SDK and model name. A model identifier such as `qwen3.8-flash`
does not identify the hosting service or select an adapter automatically.

## Profiles

| Profile | Client input | Upstream format |
|---|---|---|
| `passthrough.responses` | Responses | Responses |
| `passthrough.messages` | Messages | Messages |
| `aliyun.chat_completions` | Responses | Chat Completions |
| `aliyun.gemini_chat_completions` | GenerateContent | Chat Completions |
| `aliyun.messages` | Messages | Messages |
| `tencent.responses` | Responses | Responses |
| `volcengine.responses` | Responses | Responses |
| `moonshot.chat_completions` | Responses | Chat Completions |
| `deepseek.chat_completions` | Responses | Chat Completions |
| `minimax.chat_completions` | Responses | Chat Completions |
| `openai.chat_completions` | Responses | Chat Completions |

Codex declares Responses input; Claude Code declares Messages input. An omitted
`--provider-profile` resolves to `passthrough.responses` or `passthrough.messages`
respectively, and the resolved name is recorded in configuration. Explicit
incompatible combinations are rejected before connecting to services.

Experimental OpenCode defaults to `aliyun.chat_completions` with Qwen thinking
disabled; Codex retains that profile's existing thinking default. Gemini CLI
defaults to `aliyun.gemini_chat_completions`, also with thinking disabled.
See [External CLI backends](external-cli-backends.md) for compatibility limits.
Changing a Google base URL alone does not make Gemini CLI speak Chat Completions.

`aliyun` targets Alibaba Cloud Model Studio. It retains the tested Qwen parameter
rules, including the model-specific Qwen3.8 Messages effort mapping. It is not
a generic adapter for every model hosted by Alibaba or every service hosting
Qwen. `volcengine` targets Ark; `moonshot` targets Kimi's service. Supply the
correct service URL and credentials separately. Passthrough preserves payloads;
it does not discover or guarantee the remote endpoint's capabilities.

`openai.chat_completions` targets the OpenAI Chat Completions API and compatible
servers such as vLLM, SGLang and Ollama. The base URL should include the API
version, for example `https://api.openai.com/v1`. The profile uses only the
shared Responses-to-Chat conversion: for streaming requests it adds
`stream_options={"include_usage": True}` unless the caller supplied
`stream_options`, because OpenAI reports streaming usage only when requested.
It does not implement OpenAI's Responses API (use `passthrough.responses` for
that protocol) and it does not add vendor-specific thinking, search, token-limit
or reasoning-effort fields.

`tencent.responses` targets Tencent TokenHub's Responses endpoint, including
HY4. It preserves reasoning fields, tool-call identifiers, history and raw
responses. Inline PNG images are encoded as lossless WebP without changing
pixels or dimensions; other image formats and remote URLs pass through. It
uses the shared provider credentials and HTTP transport, with no Tencent SDK
or native robot dependency. This profile requires Responses input; it does not
implement Tencent's other APIs or a Claude Code Messages adapter. See the
[Tencent configuration example](running.md#tencent-tokenhub-and-hy4).

The Hybrid reviewer uses Responses input independently of the main planner.
Its adapter choices come from the same registry filtered for Responses. It can
inherit a compatible Responses planner's endpoint/profile. For a Messages
or GenerateContent planner, an enabled reviewer needs an explicit `--vla-review-base-url` and
`--vla-review-adapter`. When review availability depends on the prepared action
binding, the check runs during assembly before model requests. A disabled
reviewer opens no model client.

## Ownership and extension

Paths are relative to `uni_robot_agent/provider/`.

| Module | Responsibility |
|---|---|
| `contracts.py` | Request/response values, provider resource sessions, client protocol and SDK binding interfaces |
| `registry.py` | Profile names, compatible client/upstream pairs, adapter factories and validation |
| `client.py` | Context preparation, upstream requests, bounded retries and provider-session cleanup |
| `gateway.py` | Local HTTP ingress, buffered forwarding, delivery and binding commit/revocation |
| `protocols/` | Responses, Chat Completions and Messages parsing, protocol errors, SSE and Messages keepalive |
| `conversions/` | Responses/Chat translation and reversible tool-name mapping |
| `backends/<service>/` | Service-specific parameters, endpoints and optional resource management |
| `recording/` | Client/context/provider/training views and archived image references |
| `image_encoding.py` | Shared image encoding used by service backends |

Add an adapter under its service directory and a `ProviderProfile` in the
registry. Implement `target_url`, `encode_request`, and `decode_response`, with
explicit per-request decode state and training views. Declare the input and
upstream protocols; only register conversion pairs that are implemented. Use an
optional provider session for resources such as DeepSeek Files uploads and leases.
Its `prepare_request`, `finish_request`, and `close` hooks have distinct lifetimes.

Use `list_provider_profiles`, `validate_provider_profile`, and
`build_provider_adapter(name, client_protocol=...)` for all callers. Adding a
Messages backend does not require a Claude-specific branch, and adding a
Responses backend does not require changes to Codex or the shared gripper tools.
Planner SDK bindings remain under `agents/`; providers do not execute robot tools.

Context selection precedes conversion. The client checks raw upstream errors
using the upstream protocol before conversion. The gateway delivers the resulting
client protocol and commits SDK bindings only after successful delivery. Keep
this ordering, cancellation, image evidence, tool identity, and retry behavior
covered by tests. Messages heartbeat frames do not authorize tool execution.

## Python interface

Import public clients and factories from `uni_robot_agent.provider` and interface
types from `provider.contracts`. Implementations live under `backends/`,
conversions under `conversions/`, and exchange recording under `recording/`.
The DeepSeek broker entry point is
`python -m uni_robot_agent.provider.backends.deepseek.file_broker`.
The gateway binding argument is `model_binding`; use the `ModelBinding` contract.

Provider profiles select protocol conversion and backend behavior. The context
policy named `identity` controls conversation selection independently.

Adapters may implement `RequestValidatingAdapter.validate_request` to validate
the original wire history before context selection. Gemini's Qwen adapter uses
this boundary so unsupported content cannot disappear through history pruning.
Validation traverses native parts without encoding or copying image payloads;
provider encoding still operates on the selected effective request.
