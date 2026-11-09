"""Host kits: what one driver kind adds to the generic host, discovered from body/drivers/<name>/kit.py.

The host never names a driver. Each driver package that runs on an installed host
provides a ``kit`` object; ``host.json`` selects kits per Body and gives each kit
its host-level configuration.

Rules for a kit:

- ``validate`` and ``resources`` run whenever host.json is loaded, by every rua-host command and
  by install-config as root: keep them fast, import nothing heavy and never touch hardware.
- ``driver_args`` adds factory arguments; a key that host.json already sets is an error, so
  configuration always wins visibly.
- ``commands`` maps extra rua-host subcommands to ``module:function``; the function receives the
  remaining arguments and returns the exit code. Built-in names cannot be redefined.
- ``native/env.sh`` is sourced by the rua-host wrapper for every command, with
  ``RUA_RUNTIME_ROOT``, ``RUA_HOST_CONFIG`` and ``RUA_HOST_COMMAND`` set and the command's
  arguments in ``$@`` (``--config`` overrides the default configuration). It must do nothing
  when host.json does not configure the kit, and must never fail.
- ``native/build.sh`` runs once during the Linux build with ``BUILD``, ``SOURCE`` (the kit's
  native inputs), ``DEPS`` (staged locked dependencies), ``WORK`` (its own scratch directory),
  ``DEST`` (the release root, shared by all kits), ``PKG`` (the package root), ``TARGET`` and
  ``VERSION``. It installs under ``DEST/native/``, ``DEST/licenses/<name>/`` and, for privileged
  helpers, ``PKG/usr/local/sbin/``.
"""

from __future__ import annotations

import importlib
import importlib.util
import pkgutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

DRIVERS = __package__.rsplit(".", 1)[0] + ".drivers"


@dataclass(frozen=True)
class InstalledFile:
    """A root-owned file written by ``rua-host install-config``; ``check`` validates it before writing."""

    path: str
    content: str
    mode: int = 0o644
    check: tuple[str, ...] = ()


@dataclass(frozen=True)
class Build:
    """What the runtime build takes from a kit.

    ``packages`` are Python subpackages packaged with the kit's top-level modules,
    ``native_inputs`` map paths in the kit directory to paths in the native-source
    archive (under the kit name), ``exclude`` names locally built files inside them,
    and ``deb_depends`` are added to the package. ``native/build.sh``, when listed,
    runs during the Linux build; ``native/env.sh`` is sourced by ``rua-host``.
    """

    packages: tuple[str, ...] = ()
    native_inputs: tuple[tuple[str, str], ...] = ()
    exclude: tuple[str, ...] = ()
    deb_depends: tuple[str, ...] = ()


class Kit(Protocol):
    name: str
    #: ``module:attribute`` of the Driver factory served for this kit's Bodies.
    factory: str
    #: Extra ``rua-host`` subcommands, ``name -> module:function(argv)``.
    commands: dict[str, str]
    build: Build

    def validate(self, host: dict[str, Any], config: dict[str, Any], bodies: dict[str, dict[str, Any]]) -> None:
        """Check the kit configuration and its Bodies' ``driver_args``; may normalize ``driver_args``."""

    def resources(self, body: dict[str, Any]) -> list[tuple[str, str]]:
        """Hardware this Body claims, such as a bus or a serial number; no two Bodies may share one."""

    def driver_args(self, host: dict[str, Any], instance: str) -> dict[str, Any]:
        """Factory arguments added at serve time to the configured ``driver_args``."""

    def native_files(self, root: Path) -> dict[str, Path]:
        """Installed native files the Body needs, checked by ``doctor``."""

    def requirements(self) -> dict[str, bool]:
        """Runtime prerequisites outside the release, such as a system Python module."""

    def probe(self, body: dict[str, Any]) -> tuple[list[str], list[str]]:
        """Device IDs and state signals that ``doctor --connect`` reads; reads only."""

    def install_files(self, host: dict[str, Any], config: dict[str, Any], operator: str) -> list[InstalledFile]:
        """Root-owned files (permissions, device rules) the kit needs on the host."""

    def after_install(self) -> list[tuple[str, ...]]:
        """Commands run once after the files are written, such as reloading device rules."""

    def check_build(self, kit_dir: Path, lock: dict[str, Any], target: str) -> dict[str, Any]:
        """Reject inconsistent build inputs; returns fields recorded in release.json."""


@dataclass
class BaseKit:
    """Defaults for kits that need no host integration beyond serving a driver."""

    name: str
    factory: str
    commands: dict[str, str] = field(default_factory=dict)
    build: Build = field(default_factory=Build)

    def validate(self, host, config, bodies):
        if config:
            raise ValueError(f"kit {self.name} takes no host configuration")

    def resources(self, body):
        return []

    def driver_args(self, host, instance):
        return {}

    def native_files(self, root):
        return {}

    def requirements(self):
        return {}

    def probe(self, body):
        return [], []

    def install_files(self, host, config, operator):
        return []

    def after_install(self):
        return []

    def check_build(self, kit_dir, lock, target):
        return {}


def load_kit(name: str) -> Kit:
    if not name.isidentifier():
        raise ValueError(f"invalid kit name {name!r}")
    try:
        module = importlib.import_module(f"{DRIVERS}.{name}.kit")
    except ModuleNotFoundError as exc:
        if exc.name in (f"{DRIVERS}.{name}", f"{DRIVERS}.{name}.kit"):
            raise ValueError(f"kit {name!r} is not installed") from None
        raise
    kit = module.kit
    if kit.name != name:
        raise ValueError(f"kit module {name} declares {kit.name!r}")
    return kit


def installed_kits() -> list[str]:
    """Driver packages in this checkout or runtime archive that provide a kit."""
    drivers = importlib.import_module(DRIVERS)
    names = []
    for info in pkgutil.iter_modules(drivers.__path__):
        if info.ispkg and importlib.util.find_spec(f"{DRIVERS}.{info.name}.kit") is not None:
            names.append(info.name)
    return sorted(names)


def factory(spec: str):
    module, _, attribute = spec.partition(":")
    if not module or not attribute:
        raise ValueError("factory must be module:attribute")
    return getattr(importlib.import_module(module), attribute)
