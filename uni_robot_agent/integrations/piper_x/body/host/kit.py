"""Lightweight Body host plugins; validation never opens hardware.

Explicit module:attribute references adapt native integrations without importing
one in the shared service. Packaging uses the common wheel/runtime distribution.
"""
from __future__ import annotations
import importlib
import importlib.util
import pkgutil
from dataclasses import dataclass, field
from typing import Any, Protocol

DRIVERS = __package__.rsplit(".", 1)[0] + ".drivers"


class Kit(Protocol):
    name: str
    factory: str
    commands: dict[str, str]

    def validate(self, host: dict, config: dict, bodies: dict) -> None: ...
    def resources(self, body: dict) -> list[tuple[str, str]]: ...
    def driver_args(self, host: dict, instance: str) -> dict[str, Any]: ...


@dataclass
class BaseKit:
    name: str
    factory: str
    commands: dict[str, str] = field(default_factory=dict)

    def validate(self, host, config, bodies):
        if config:
            raise ValueError(f"kit {self.name} takes no host configuration")

    def resources(self, body):
        return []

    def driver_args(self, host, instance):
        return {}


def load_kit(name: str) -> Kit:
    if ":" in name:
        # Explicit plugin reference, validated without constructing a driver.
        # Importing a kit must not import native libraries or open hardware.
        kit = factory(name)
        if not callable(getattr(kit, "resources", None)):
            raise ValueError("Kit reference must expose the host kit contract")
        return kit
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
