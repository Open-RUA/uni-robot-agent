"""The interface a robot driver implements; the Body server adds leases, operations, streams and events.

See ``body/drivers/fake`` for the smallest complete driver.

Threading: the server calls ``read``, ``check`` and ``health`` from request threads at any time;
``write`` and ``invoke`` run one at a time per device on an operation worker; ``stream_apply``
and ``stream_hold`` run on the stream watchdog; ``estop`` may run concurrently with all of
them and must not wait for a running motion to finish. A driver signals failure by raising
``DeviceError`` (``core/contract.py``), with ``execution_outcome="unknown"`` whenever hardware
may have acted: the server then quarantines the device instead of letting anyone retry.
"""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Any, Protocol

from .manifest import BodyManifest


class Driver(Protocol):
    """One Body's hardware. Optional members: ``data_origin`` (``"real"`` unless set, for example
    ``"fake"``), ``read_observation_at`` (backs the observation streams the manifest declares) and
    ``close()`` (called once when the Body stops)."""

    def manifest(self) -> BodyManifest:
        """Devices, signals, commands, procedures and streams; fixed for the driver's lifetime."""

    def health(self) -> dict[str, Any]:
        """Return ``{"ok": bool, ...}``; extra fields are reported as diagnostics. Never touches hardware state."""

    def read(self, device_id: str, include: list[str] | None = None) -> dict[str, Any]:
        """Return ``{"signals": {...}, "freshness": "fresh|stale|unknown", "age_s": float, "source": {...}}``."""

    def check(self, device_id: str) -> dict[str, Any]:
        """Return ``{"readiness": "satisfied|unsatisfied|unknown", "checks": {name: {"state": ..., "reason": ...}}}``."""

    def write(self, device_id: str, command: str, value: list[float], *, cancel: threading.Event) -> dict[str, Any]:
        """Apply one declared command (already clamped to its manifest limits); return the result record."""

    def invoke(self, device_ids: list[str], procedure: str, params: dict[str, Any], *,
               cancel: threading.Event, progress: Callable[[float], None]) -> dict[str, Any]:
        """Run one declared procedure to completion; poll ``cancel`` and stop safely when it is set.

        The result should say whether the goal was reached and whether hardware acted, for
        example ``{"completed": bool, "partial": bool, "hardware_command_sent": bool, ...}``.
        """

    def stream_apply(self, device_id: str, targets: dict[str, list[float]]) -> None:
        """Apply one streamed setpoint; omitted targets hold their measured values."""

    def stream_hold(self, device_id: str) -> None:
        """Hold the measured position. Must work with only fresh feedback, even after a fault."""

    def estop(self, device_id: str) -> dict[str, Any]:
        """Abort running work and hold; return what was done. This is not a certified emergency stop."""
