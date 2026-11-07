"""Offline installation and configuration diagnostics for ``uni-robot-agent doctor``.

The module keeps only standard-library imports at module scope. Optional
integrations, provider profiles and package metadata are loaded on demand, so
importing this module (or rendering CLI help) does not require an agent SDK,
model runtime or native simulator. Checks never connect to a service, read a
credential value or create a directory.
"""

from __future__ import annotations

import importlib
import importlib.metadata
import json
import os
import platform
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping

STATUSES = frozenset({"ok", "warn", "fail", "skip"})
MINIMUM_PYTHON = (3, 11)
PACKAGE_NAMES = ("uni-robot-agent", "uni-robot-agent-runtime")

# Module name and the distribution that provides its version metadata.
REQUIRED_MODULES = (
    ("rich", "rich"),
    ("pydantic", "pydantic"),
    ("jsonschema", "jsonschema"),
    ("httpx", "httpx"),
    ("fastapi", "fastapi"),
    ("uvicorn", "uvicorn"),
    ("numpy", "numpy"),
    ("PIL", "pillow"),
)

# Base dependencies in pyproject.toml; each pins one planner backend.
AGENT_SDKS = (
    ("openai-codex", "Codex"),
    ("claude-agent-sdk", "Claude Code"),
)

# Optional managed-backend executables; never executed unless requested.
BACKEND_BINARIES = ("opencode", "gemini")

KNOWN_INTEGRATIONS = (
    "robodojo",
    "robouse",
    "robolab",
    "robotwin",
    "libero_plus",
    "libero_pro",
)

# (variable, hint). Presence is reported; values are never read.
# Every listed setting is optional for an offline installation, so an unset
# variable is a warning rather than a failure.
ENVIRONMENT_VARIABLES = (
    (
        "UNI_ROBOT_AGENT_API_KEY",
        "Set it before running an episode; doctor reports presence only and never reads the value.",
    ),
    (
        "UNI_ROBOT_AGENT_PROVIDER_URL",
        "Set it, or pass --provider-url when a gateway provider profile is selected.",
    ),
    (
        "UNI_ROBOT_AGENT_ENV_URL",
        "Set it, or pass --env-url/--deploy local for each run.",
    ),
    (
        "UNI_ROBOT_AGENT_SKILLS_DIR",
        "Optional: set it to a complete external skills library to replace packaged skills.",
    ),
)


@dataclass(frozen=True)
class Check:
    """One diagnostic result with no reference to secrets or live services."""

    name: str
    status: str
    detail: str
    hint: str | None = None

    def __post_init__(self) -> None:
        if self.status not in STATUSES:
            raise ValueError(f"invalid check status {self.status!r}; choose from {sorted(STATUSES)}")
        if not self.name:
            raise ValueError("check name must not be empty")

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "detail": self.detail,
            "hint": self.hint,
        }


@dataclass(frozen=True)
class DoctorReport:
    """Structured outcome of every check, in stable execution order."""

    checks: tuple[Check, ...] = ()

    @property
    def ok(self) -> bool:
        return not self.failures

    @property
    def failures(self) -> tuple[Check, ...]:
        return tuple(check for check in self.checks if check.status == "fail")

    @property
    def warnings(self) -> tuple[Check, ...]:
        return tuple(check for check in self.checks if check.status == "warn")

    def as_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "failures": len(self.failures),
            "warnings": len(self.warnings),
            "checks": [check.as_dict() for check in self.checks],
        }


# Indirection points keep checks deterministic under unittest.mock.patch.
def _import_module(name: str) -> Any:
    return importlib.import_module(name)


def _distribution_version(name: str) -> str:
    return importlib.metadata.version(name)


def _which(name: str) -> str | None:
    return shutil.which(name)


def _access(path: Path, mode: int) -> bool:
    return os.access(path, mode)


def _packaged_skills_root() -> Path:
    return Path(__file__).resolve().parent / "resources" / "skills"


def _package_version() -> str:
    from uni_robot_agent import __version__

    return __version__


def _python_version_info() -> Any:
    return sys.version_info


def _binary_version(path: str) -> tuple[str | None, str | None]:
    """Return (first output line, None) or (None, error description)."""
    try:
        completed = subprocess.run(
            [path, "--version"],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as error:
        return None, f"{type(error).__name__}: {error}"
    lines = (completed.stdout or completed.stderr or "").strip().splitlines()
    first = lines[0].strip() if lines else ""
    if completed.returncode != 0:
        return None, f"exit status {completed.returncode}"
    return first, None


def check_python_version() -> Check:
    info = _python_version_info()
    major, minor, micro = info[0], info[1], info[2]
    detail = f"{major}.{minor}.{micro} ({platform.python_implementation()}, {sys.executable})"
    if (major, minor) < MINIMUM_PYTHON:
        return Check(
            "python-version",
            "fail",
            detail,
            "Install Python 3.11 or newer and recreate the virtual environment.",
        )
    return Check("python-version", "ok", detail)


def check_package_version() -> Check:
    hint = (
        "Install the CLI with python -m pip install uni-robot-agent; "
        "the importable package is not from an installed distribution."
    )
    try:
        version = _package_version()
    except Exception as error:
        return Check(
            "package-version",
            "fail",
            f"cannot read uni_robot_agent.__version__: {type(error).__name__}: {error}",
            hint,
        )
    if not version or version == "unknown (not installed)":
        return Check("package-version", "fail", str(version or "empty version"), hint)
    return Check("package-version", "ok", f"uni-robot-agent {version}")


def check_package_split() -> Check:
    installed: dict[str, str] = {}
    for name in PACKAGE_NAMES:
        try:
            installed[name] = _distribution_version(name)
        except importlib.metadata.PackageNotFoundError:
            continue
        except Exception as error:
            return Check(
                "package-split",
                "fail",
                f"cannot read metadata for {name}: {type(error).__name__}: {error}",
                "Reinstall the package in a clean environment and check its metadata.",
            )
    main_version = installed.get("uni-robot-agent")
    runtime = installed.get("uni-robot-agent-runtime")
    if main_version and runtime:
        return Check(
            "package-split",
            "fail",
            f"both installed: uni-robot-agent {main_version}, uni-robot-agent-runtime {runtime}",
            "Recreate the environment with only one package; the main and runtime wheels must be installed in separate environments.",
        )
    if not main_version:
        detail = (
            f"missing uni-robot-agent; only uni-robot-agent-runtime {runtime} is installed"
            if runtime
            else "neither uni-robot-agent nor uni-robot-agent-runtime is installed"
        )
        return Check(
            "package-split",
            "fail",
            detail,
            "Install the agent CLI package with python -m pip install uni-robot-agent.",
        )
    return Check("package-split", "ok", f"uni-robot-agent {main_version}")


def _import_check(
    name: str,
    module: str,
    distribution: str,
    *,
    required: bool,
    hint: str | None = None,
) -> Check:
    try:
        _import_module(module)
    except Exception as error:
        status = "fail" if required else "warn"
        return Check(
            name,
            status,
            f"cannot import {module}: {type(error).__name__}: {error}",
            hint
            or (
                "Reinstall the standard package: python -m pip install --force-reinstall uni-robot-agent"
                if required
                else f"Install the {distribution} dependency to use this feature."
            ),
        )
    try:
        version = _distribution_version(distribution)
    except importlib.metadata.PackageNotFoundError:
        return Check(name, "ok", f"{module} importable (distribution metadata missing)")
    except Exception:
        return Check(name, "ok", f"{module} importable (version unavailable)")
    return Check(name, "ok", f"{module} {version}")


def check_required_dependencies() -> list[Check]:
    return [
        _import_check(f"dependency:{module}", module, distribution, required=True)
        for module, distribution in REQUIRED_MODULES
    ]


def check_agent_sdks() -> list[Check]:
    checks = []
    for distribution, backend in AGENT_SDKS:
        try:
            version = _distribution_version(distribution)
        except importlib.metadata.PackageNotFoundError:
            checks.append(
                Check(
                    f"sdk:{distribution}",
                    "warn",
                    "not installed",
                    f"Install the pinned SDK with the standard installation; the {backend} backend cannot run without it.",
                )
            )
        except Exception as error:
            checks.append(
                Check(
                    f"sdk:{distribution}",
                    "warn",
                    f"version unavailable: {type(error).__name__}: {error}",
                    f"Reinstall the pinned SDK if the {backend} backend is required.",
                )
            )
        else:
            checks.append(Check(f"sdk:{distribution}", "ok", f"{distribution} {version}"))
    return checks


def check_backend_binaries(*, check_version: bool = False) -> list[Check]:
    checks = []
    for name in BACKEND_BINARIES:
        path = _which(name)
        if path is None:
            checks.append(
                Check(
                    f"binary:{name}",
                    "skip",
                    "not found",
                    f"Install {name} and put it on PATH to enable that managed backend; other backends work without it.",
                )
            )
            continue
        if not check_version:
            checks.append(Check(f"binary:{name}", "ok", f"found at {path}"))
            continue
        version, error = _binary_version(path)
        if error is not None:
            checks.append(
                Check(
                    f"binary:{name}",
                    "warn",
                    f"found at {path}; --version failed: {error}",
                    f"Check that {name} runs in this environment; doctor does not use it otherwise.",
                )
            )
        else:
            suffix = f" ({version})" if version else ""
            checks.append(Check(f"binary:{name}", "ok", f"found at {path}{suffix}"))
    return checks


def check_provider_profiles() -> Check:
    try:
        from uni_robot_agent.provider.registry import list_provider_profiles

        names = list_provider_profiles()
    except Exception as error:
        return Check(
            "provider-profiles",
            "fail",
            f"cannot load provider registry: {type(error).__name__}: {error}",
            "Reinstall the standard package; provider profiles are required to resolve --provider-profile.",
        )
    return Check(
        "provider-profiles",
        "ok",
        f"{len(names)} profiles: {', '.join(names)}",
    )


def check_integrations(*, requested: str | None = None) -> list[Check]:
    from uni_robot_agent.integrations import load_integration

    names: Iterable[str] = (requested,) if requested else KNOWN_INTEGRATIONS
    checks = []
    for name in names:
        required = name == requested
        try:
            load_integration(name)
        except Exception as error:
            status = "fail" if required else "warn"
            hint = (
                f"Install this integration's dependencies or check the name; available: {', '.join(KNOWN_INTEGRATIONS)}."
                if required
                else "Install the adapter's optional dependencies (see docs/getting-started.md#integration-extras) to use it."
            )
            checks.append(
                Check(
                    f"integration:{name}",
                    status,
                    f"not importable: {type(error).__name__}: {error}",
                    hint,
                )
            )
        else:
            checks.append(
                Check(
                    f"integration:{name}",
                    "ok",
                    "importable; get_integration_spec present",
                )
            )
    return checks


def check_skills_root(*, environ: Mapping[str, str] | None = None) -> Check:
    environment = os.environ if environ is None else environ
    configured = environment.get("UNI_ROBOT_AGENT_SKILLS_DIR")
    if configured:
        root = Path(configured).expanduser()
        source = "UNI_ROBOT_AGENT_SKILLS_DIR"
    else:
        root = _packaged_skills_root()
        source = "packaged"
    if not root.exists():
        return Check(
            "skills-root",
            "fail",
            f"{source} skills root does not exist: {root}",
            "Point UNI_ROBOT_AGENT_SKILLS_DIR at a complete skills library or reinstall the package."
            if configured
            else "Reinstall the package; the packaged skills root is missing.",
        )
    if not root.is_dir():
        return Check(
            "skills-root",
            "fail",
            f"{source} skills root is not a directory: {root}",
            "Point UNI_ROBOT_AGENT_SKILLS_DIR at a directory, not a file.",
        )
    general = root / "general"
    if not general.is_dir():
        return Check(
            "skills-root",
            "fail",
            f"{source} skills root has no general/ directory: {root}",
            "A complete skills library contains general/ and the environment namespace directories.",
        )
    if not _access(root, os.R_OK | os.X_OK):
        return Check(
            "skills-root",
            "fail",
            f"{source} skills root is not readable: {root}",
            "Fix permissions on the skills library so the agent can read it.",
        )
    return Check("skills-root", "ok", f"{root} ({source}, general/ present)")


def check_output_dir(
    output_dir: str | os.PathLike[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
) -> Check:
    environment = os.environ if environ is None else environ
    configured = (
        os.fspath(output_dir)
        if output_dir is not None
        else environment.get("UNI_ROBOT_AGENT_OUTPUT_DIR")
    )
    target = Path(configured).expanduser() if configured else Path("outputs")
    hint = "Pass --output-dir or set UNI_ROBOT_AGENT_OUTPUT_DIR to a writable directory."
    if target.exists():
        if not target.is_dir():
            return Check("output-dir", "fail", f"{target} exists and is not a directory", hint)
        if _access(target, os.W_OK | os.X_OK):
            return Check("output-dir", "ok", f"writable: {target}")
        return Check("output-dir", "fail", f"not writable: {target}", hint)
    parent = target.parent
    while not parent.exists():
        if parent == parent.parent:
            return Check("output-dir", "fail", f"no existing parent for {target}", hint)
        parent = parent.parent
    if not parent.is_dir():
        return Check("output-dir", "fail", f"nearest existing parent is not a directory: {parent}", hint)
    if _access(parent, os.W_OK | os.X_OK):
        return Check(
            "output-dir",
            "ok",
            f"{target} does not exist yet; create it under writable parent {parent}",
        )
    return Check("output-dir", "fail", f"{target} does not exist and parent {parent} is not writable", hint)


def check_environment_variables(*, environ: Mapping[str, str] | None = None) -> list[Check]:
    environment = os.environ if environ is None else environ
    checks = []
    for name, hint in ENVIRONMENT_VARIABLES:
        present = bool(environment.get(name))
        if present:
            checks.append(Check(f"env:{name}", "ok", "set"))
        else:
            checks.append(Check(f"env:{name}", "warn", "unset", hint))
    return checks


def run_checks(
    *,
    integration: str | None = None,
    output_dir: str | os.PathLike[str] | None = None,
    check_binaries: bool = False,
    environ: Mapping[str, str] | None = None,
) -> DoctorReport:
    """Run every offline check and return an ordered, structured report."""
    environment = os.environ if environ is None else environ
    checks: list[Check] = [
        check_python_version(),
        check_package_version(),
        check_package_split(),
    ]
    checks.extend(check_required_dependencies())
    checks.extend(check_agent_sdks())
    checks.extend(check_backend_binaries(check_version=check_binaries))
    checks.append(check_provider_profiles())
    checks.extend(check_integrations(requested=integration))
    checks.append(check_skills_root(environ=environment))
    checks.append(check_output_dir(output_dir, environ=environment))
    checks.extend(check_environment_variables(environ=environment))
    return DoctorReport(tuple(checks))


def summary(report: DoctorReport) -> str:
    ok = sum(1 for check in report.checks if check.status == "ok")
    skipped = sum(1 for check in report.checks if check.status == "skip")
    text = (
        f"doctor: {ok} ok, {len(report.warnings)} warnings, "
        f"{len(report.failures)} failures"
    )
    if skipped:
        text += f", {skipped} skipped"
    return text


def render_json(report: DoctorReport, *, indent: int = 2) -> str:
    return json.dumps(report.as_dict(), ensure_ascii=False, indent=indent)


def render_table(report: DoctorReport) -> str:
    status_width = max([len("STATUS")] + [len(check.status.upper()) for check in report.checks])
    name_width = max([len("CHECK")] + [len(check.name) for check in report.checks])
    lines = [f"{'STATUS':<{status_width}}  {'CHECK':<{name_width}}  DETAIL"]
    for check in report.checks:
        lines.append(
            f"{check.status.upper():<{status_width}}  {check.name:<{name_width}}  {check.detail}"
        )
    hints = [
        check
        for check in report.checks
        if check.status in {"warn", "fail"} and check.hint
    ]
    if hints:
        lines.append("")
        lines.extend(f"hint ({check.name}): {check.hint}" for check in hints)
    lines.append("")
    lines.append(summary(report))
    return "\n".join(lines)
