"""Per-device exclusive control leases.

* ``acquire`` is atomic over all requested devices and idempotent for the same
  principal + request_key.
* Leases expire after ``ttl_s`` without ``renew``; expiry revokes the lease and
  notifies the server (which cancels unissued work on those devices).
* The holder is the only principal allowed to write to the devices.
* Revocation callbacks run after the table lock is released, so a slow driver
  hold in a callback never blocks renewals of other leases.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Iterable
from contextlib import contextmanager
from typing import Any

from .contract import DeviceError, ErrorCode, Lease, new_id


class LeaseTable:
    def __init__(
        self,
        *,
        ttl_s: float = 5.0,
        clock: Callable[[], float] = time.monotonic,
        on_revoke: Callable[[Lease, str], None] | None = None,
    ) -> None:
        self.ttl_s = float(ttl_s)
        if not math.isfinite(self.ttl_s) or self.ttl_s <= 0:
            raise ValueError("lease ttl_s must be positive and finite")
        self._clock = clock
        self._on_revoke = on_revoke
        self._lock = threading.RLock()
        self._leases: dict[str, Lease] = {}
        self._by_device: dict[str, str] = {}  # device_id -> control_id
        self._by_request: dict[tuple[str, str], str] = {}  # (principal, request_key) -> control_id
        self._generation = 0
        self._pending: list[tuple[Lease, str]] = []

    # -- queries -----------------------------------------------------------------------------

    def holder(self, device_id: str) -> Lease | None:
        with self._locked():
            self._expire_locked()
            control_id = self._by_device.get(device_id)
            return self._leases.get(control_id) if control_id else None

    def get(self, control_id: str) -> Lease | None:
        with self._locked():
            self._expire_locked()
            return self._leases.get(control_id)

    def active(self) -> list[Lease]:
        with self._locked():
            self._expire_locked()
            return [lease for lease in self._leases.values() if lease.state == "active"]

    # -- mutations ---------------------------------------------------------------------------

    def acquire(self, principal: str, device_ids: Iterable[str], request_key: str) -> Lease:
        devices = tuple(dict.fromkeys(str(d) for d in device_ids))
        if not devices:
            raise DeviceError(ErrorCode.INVALID_ARGUMENTS, "device_ids must not be empty")
        with self._locked():
            self._expire_locked()
            existing_id = self._by_request.get((principal, request_key))
            if existing_id and existing_id in self._leases:
                existing = self._leases[existing_id]
                if existing.state == "active":
                    if existing.device_ids != devices:
                        raise DeviceError(ErrorCode.IDEMPOTENCY_CONFLICT, "request_key reused for a different device set", details={"request_key": request_key})
                    return existing
            conflicts = {}
            for device in devices:
                holder_id = self._by_device.get(device)
                if holder_id and holder_id in self._leases and self._leases[holder_id].state == "active":
                    conflicts[device] = {"control_id": holder_id, "principal": self._leases[holder_id].principal}
            if conflicts:
                raise DeviceError(ErrorCode.RESOURCE_BUSY, "device already under exclusive control", details={"conflicts": conflicts})
            self._generation += 1
            now = self._clock()
            lease = Lease(
                control_id=new_id("ctl"),
                principal=principal,
                device_ids=devices,
                ttl_s=self.ttl_s,
                expires_at=now + self.ttl_s,
                generation=self._generation,
                request_key=request_key,
            )
            self._leases[lease.control_id] = lease
            for device in devices:
                self._by_device[device] = lease.control_id
            self._by_request[(principal, request_key)] = lease.control_id
            return lease

    def renew(self, principal: str, control_id: str) -> Lease:
        with self._locked():
            self._expire_locked()
            lease = self._leases.get(control_id)
            if lease is None or lease.state != "active":
                raise DeviceError(ErrorCode.CONTROL_EXPIRED, "control lease is not active", details={"control_id": control_id, "state": None if lease is None else lease.state})
            if lease.principal != principal:
                raise DeviceError(ErrorCode.CONTROL_NOT_OWNED, "control lease belongs to another principal", details={"control_id": control_id})
            lease.expires_at = self._clock() + self.ttl_s
            return lease

    def release(self, principal: str, control_id: str) -> Lease:
        with self._locked():
            self._expire_locked()
            lease = self._leases.get(control_id)
            if lease is None:
                raise DeviceError(ErrorCode.CONTROL_EXPIRED, "unknown control_id", details={"control_id": control_id})
            if lease.principal != principal:
                raise DeviceError(ErrorCode.CONTROL_NOT_OWNED, "control lease belongs to another principal", details={"control_id": control_id})
            if lease.state == "active":
                self._revoke_locked(lease, "released")
            return lease

    def require(self, principal: str, control_id: str | None, device_ids: Iterable[str] = ()) -> Lease:
        """Return the active lease covering ``device_ids`` or raise."""
        if not control_id:
            raise DeviceError(ErrorCode.CONTROL_EXPIRED, "control_id is required for writes", details={"control_id": None})
        with self._locked():
            self._expire_locked()
            lease = self._leases.get(control_id)
            if lease is None or lease.state != "active":
                raise DeviceError(ErrorCode.CONTROL_EXPIRED, "control lease is not active", details={"control_id": control_id, "state": None if lease is None else lease.state})
            if lease.principal != principal:
                raise DeviceError(ErrorCode.CONTROL_NOT_OWNED, "control lease belongs to another principal", details={"control_id": control_id})
            missing = [d for d in device_ids if d not in lease.device_ids]
            if missing:
                raise DeviceError(ErrorCode.CONTROL_NOT_OWNED, "lease does not cover the requested devices", details={"control_id": control_id, "missing": missing})
            return lease

    def revoke_all(self, reason: str = "revoked") -> list[Lease]:
        with self._locked():
            revoked = []
            for lease in list(self._leases.values()):
                if lease.state == "active":
                    self._revoke_locked(lease, reason)
                    revoked.append(lease)
            return revoked

    def sweep(self) -> list[Lease]:
        with self._locked():
            return self._expire_locked()

    # -- internals ---------------------------------------------------------------------------

    def _expire_locked(self) -> list[Lease]:
        now = self._clock()
        expired = []
        for lease in list(self._leases.values()):
            if lease.state == "active" and lease.expires_at <= now:
                self._revoke_locked(lease, "expired")
                expired.append(lease)
        return expired

    @contextmanager
    def _locked(self):
        try:
            with self._lock:
                yield
        finally:
            with self._lock:
                pending, self._pending = self._pending, []
            for lease, reason in pending:
                try:
                    self._on_revoke(lease, reason)
                except Exception:  # pragma: no cover - callbacks must not break the table
                    pass

    def _revoke_locked(self, lease: Lease, reason: str) -> None:
        lease.state = reason
        for device in lease.device_ids:
            if self._by_device.get(device) == lease.control_id:
                del self._by_device[device]
        if self._on_revoke is not None:
            self._pending.append((lease, reason))

    def snapshot(self) -> dict[str, Any]:
        with self._locked():
            self._expire_locked()
            now = self._clock()
            return {lease.control_id: lease.to_dict(now) for lease in self._leases.values() if lease.state == "active"}


__all__ = ["LeaseTable"]
