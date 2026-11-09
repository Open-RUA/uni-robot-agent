"""Device-owned observation history; independent of command streams and RPCs."""

import math
import threading
import time
from bisect import bisect_left
from collections import deque
from copy import deepcopy

from .contract import DeviceError, ErrorCode
from .timing import TIMESTAMP_TOLERANCE_NS, timestamp_ns


def validate_query(at, mode, max_gap_s):
    try:
        if not isinstance(at, dict):
            raise ValueError("at must contain source time and clock identity")
        target = timestamp_ns(at.get("source_timestamp_ns"))
        if not isinstance(at.get("clock_id"), str) or not at["clock_id"]:
            raise ValueError("clock_id is required")
        if at.get("source_clock") != "unix_realtime":
            raise ValueError("only explicit Unix source times can be compared")
        if mode not in ("bracket", "nearest", "nearest_complete", "after"):
            raise ValueError("mode must be bracket, nearest, nearest_complete or after")
        if (
            type(max_gap_s) not in (int, float)
            or not math.isfinite(max_gap_s)
            or not 0 < max_gap_s <= 2
        ):
            raise ValueError("max_gap_s must be positive and at most 2 s")
        return target
    except (TypeError, ValueError) as exc:
        raise DeviceError(ErrorCode.INVALID_ARGUMENTS, str(exc)) from exc


def _receive_pair(timing):
    """Validate clocks without treating delivery time as source time."""
    span = timing.get("clock_pair_span_ns")
    if not (type(span) is int or isinstance(span, str) and span.isdecimal()):
        raise ValueError("missing integer clock sampling span")
    if not 0 <= int(span) <= TIMESTAMP_TOLERANCE_NS:
        raise ValueError("clock sampling span exceeds 1 ms")
    if "received_realtime_ns" in timing:
        wall = timestamp_ns(timing.get("received_realtime_ns"))
        mono = timestamp_ns(timing.get("received_monotonic_ns"))
        domain = "monotonic"
    else:
        wall = timestamp_ns(timing.get("callback_realtime_ns"))
        mono = timestamp_ns(timing.get("callback_monotonic_raw_ns"))
        domain = "monotonic_raw"
    if timestamp_ns(timing.get("source_timestamp_ns")) > wall + TIMESTAMP_TOLERANCE_NS:
        raise ValueError("source timestamp is later than receipt")
    return wall, mono, domain


class ObservationBuffer:
    """Bounded snapshots copied on ingestion, never populated by a query.

    Producers supply an immutable generation key and a payload byte estimate.
    Arrival monotonic time bounds retention, source integer ns selects samples.
    The horizon is an upper bound: count and byte caps can evict earlier.
    """

    def __init__(
        self, *, retention_s=2.0, max_samples=512, max_bytes=2**20, clock=time.monotonic
    ):
        if (
            not math.isfinite(retention_s)
            or retention_s <= 0
            or type(max_samples) is not int
            or max_samples <= 0
            or type(max_bytes) is not int
            or max_bytes <= 0
        ):
            raise ValueError("history limits must be positive")
        self.retention_s = retention_s
        self.max_samples = max_samples
        self.max_bytes = max_bytes
        self._clock = clock
        self._lock = threading.RLock()
        self._rows = deque()
        self._bytes = 0
        self._identity = None
        self._diagnostic = {"accepted_total": 0, "rejected_total": 0,
                            "consecutive_rejected": 0,
                            "expired_total": 0, "capacity_evicted_total": 0,
                            "clear_total": 0}

    def clear(self, *, reason="explicit_clear"):
        with self._lock:
            self._diagnostic.update(clear_total=self._diagnostic["clear_total"] + 1,
                                    last_clear={"reason": reason, "removed_count": len(self._rows),
                                                "monotonic_s": self._clock()})
            self._rows.clear()
            self._bytes = 0
            self._identity = None
            self._diagnostic["consecutive_rejected"] = 0

    def begin_generation(self, timing, generation):
        """Invalidate history on a producer/clock epoch change, even for a bad first frame."""
        identity = (timing.get("clock_id"), timing.get("source_clock", "unix_realtime"), generation)
        with self._lock:
            if identity != self._identity:
                self.clear(reason="generation_changed")
                self._identity = identity

    def _prune(self):
        cutoff = self._clock() - self.retention_s
        while self._rows and (
            self._rows[0][0] < cutoff
            or len(self._rows) > self.max_samples
            or self._bytes > self.max_bytes
        ):
            category = "expired_total" if self._rows[0][0] < cutoff else "capacity_evicted_total"
            self._diagnostic[category] += 1
            self._diagnostic["last_eviction"] = {"reason": category, "monotonic_s": self._clock()}
            self._bytes -= self._rows.popleft()[1]

    def record_rejection(self, reason, *, sequence=None):
        with self._lock:
            self._diagnostic["rejected_total"] += 1
            self._diagnostic["consecutive_rejected"] += 1
            self._diagnostic["last_rejection"] = {"reason": str(reason)[:512],
                "sequence": sequence, "monotonic_s": self._clock()}

    def __len__(self):
        with self._lock:
            self._prune()
            return len(self._rows)

    def append(
        self, timing, payload, *, generation=None, sequence=None, size_bytes=1024
    ):
        stamp = timestamp_ns(timing.get("source_timestamp_ns"))
        identity = (timing.get("clock_id"), timing.get("source_clock"), generation)
        if not identity[0] or identity[1] != "unix_realtime":
            raise ValueError("history requires a named Unix source clock")
        if type(size_bytes) is not int or size_bytes <= 0:
            raise ValueError("payload byte estimate must be positive")
        with self._lock:
            self._prune()
            if identity != self._identity:
                self.clear(reason="generation_changed")
                self._identity = identity
            if self._rows and stamp <= self._rows[-1][2]["stamp"]:
                self.record_rejection("non_increasing_source_timestamp", sequence=sequence)
                return False  # duplicates and replay cannot replace newer source data
            if size_bytes > self.max_bytes:
                self.clear(reason="payload_exceeds_capacity")
                self.record_rejection("payload_exceeds_capacity", sequence=sequence)
                return False
            row = {
                "stamp": stamp,
                "timing": deepcopy(timing),
                "generation": generation,
                "sequence": sequence,
                "payload": deepcopy(payload),
            }
            self._rows.append((self._clock(), size_bytes, row))
            self._diagnostic["accepted_total"] += 1
            self._diagnostic["consecutive_rejected"] = 0
            self._diagnostic["last_accepted"] = {"sequence": sequence, "source_timestamp_ns": str(stamp),
                                                  "monotonic_s": self._clock(),
                                                  "identity": identity}
            self._bytes += size_bytes
            self._prune()
            return True

    def snapshot(self):
        with self._lock:
            self._prune()
            return deepcopy([row for _, _, row in self._rows])

    def diagnostics(self):
        """Bounded metadata, including the last good frame after it expires.

        A retained frame is not proof that an arbitrary target-time query will
        succeed. This method reports availability without relaxing query rules.
        """
        with self._lock:
            self._prune()
            return self._diagnostics_locked()

    def _diagnostics_locked(self):
        # A failed query has already pruned its rows. Do not prune a second
        # time while assembling that error: its two counts must describe the
        # same retained set even if a frame expires during serialization.
        last = self._diagnostic.get("last_accepted")
        return deepcopy({**self._diagnostic,
            "identity": self._identity,
            "retained_count": len(self._rows), "payload_bytes": self._bytes,
            "last_accepted_age_s": max(0., self._clock() - last["monotonic_s"]) if last else None})

    def query(self, at, *, mode="bracket", max_gap_s=0.05):
        target = validate_query(at, mode, max_gap_s)
        with self._lock:
            self._prune()
            rows = [row for _, _, row in self._rows]
            retained = {
                "count": len(rows),
                "payload_bytes": self._bytes,
                "oldest_source_timestamp_ns": str(rows[0]["stamp"]) if rows else None,
                "newest_source_timestamp_ns": str(rows[-1]["stamp"]) if rows else None,
            }

            def fail(reason, message):
                raise DeviceError(
                    ErrorCode.OBSERVATION_STALE,
                    message,
                    details={"reason": reason, "retained": retained,
                             "history_diagnostics": self._diagnostics_locked()},
                )

            if not rows:
                fail("pending", "no source observations retained")
            if at["clock_id"] != rows[-1]["timing"]["clock_id"]:
                fail("clock_mismatch", "target and device use different host clocks")
            # An invalid sample must not complete the post-target watermark or
            # hide a usable candidate. Keep the retained diagnostics separate.
            valid, rejected = [], []
            for row in rows:
                try:
                    _receive_pair(row["timing"])
                except ValueError as exc:
                    rejected.append(str(exc))
                else:
                    valid.append(row)
            if not valid:
                fail("invalid_clock", rejected[-1])
            rows = valid
            retained["valid_count"] = len(rows)
            retained["invalid_count"] = len(rejected)
            stamps = [row["stamp"] for row in rows]
            upper = bisect_left(stamps, target)
            if mode == "bracket":
                if target < stamps[0]:
                    fail("before_history", "target predates retained source history")
                if upper == len(rows):
                    fail("pending", "a post-target observation has not arrived")
                lower = upper if stamps[upper] == target else upper - 1
                selected = rows[lower : upper + 1]
                gap = stamps[upper] - stamps[lower]
            elif mode == "after":
                # RGB and depth can straddle the target despite sharing a frame
                # number. Require both source times to cover this fixed target.
                while upper < len(rows):
                    times = rows[upper]["timing"].get("source_times", {})
                    if all(timestamp_ns(times[s]["source_timestamp_ns"]) >= target
                           for s in ("rgb", "depth") if s in times):
                        break
                    upper += 1
                if upper == len(rows):
                    fail("pending", "a post-target observation has not arrived")
                selected = [rows[upper]]
                gap = stamps[upper] - target
            else:
                if mode == "nearest_complete" and upper == len(rows):
                    fail("pending", "a post-target observation has not arrived")
                # Nearest among retained samples, not a prediction about future arrivals.
                indices = [i for i in (upper - 1, upper) if 0 <= i < len(rows)]
                index = min(indices, key=lambda i: (abs(stamps[i] - target), stamps[i]))
                selected = [rows[index]]
                gap = abs(stamps[index] - target)
            if gap > max_gap_s * 1e9:
                fail("gap", "source samples exceed the requested time gap")
            try:
                checked = selected
                if mode == "nearest_complete" and 0 < upper < len(rows):
                    checked = rows[upper - 1:upper + 1]
                pairs = [_receive_pair(row["timing"]) for row in checked]
                if len(pairs) == 2:
                    a, b = pairs
                    if (
                        a[2] != b[2]
                        or b[1] <= a[1]
                        or abs((b[0] - a[0]) - (b[1] - a[1])) > TIMESTAMP_TOLERANCE_NS
                    ):
                        raise ValueError(
                            "host clock discontinuity inside observation bracket"
                        )
            except ValueError as exc:
                fail("invalid_clock", str(exc))
            return {
                "at": deepcopy(at),
                "mode": mode,
                "gap_s": gap / 1e9,
                "retained": retained,
                "clock_mapping_validated": False,
                "hardware_synchronized": False,
                "samples": [
                    {
                        "timing": deepcopy(row["timing"]),
                        "sequence": row["sequence"],
                        "generation": row["generation"],
                        "offset_s": (row["stamp"] - target) / 1e9,
                        "payload": deepcopy(row["payload"]),
                    }
                    for row in selected
                ],
            }
