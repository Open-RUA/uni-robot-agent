"""Distinct-feedback settling, adapted from ARX ONE PR #28 (9b176d6), MIT.

This verifies a caller's tolerance predicate, not task success or physical stop.
The caller owns feedback freshness, command boundaries and hardware tolerances.
"""
from __future__ import annotations

import math


class ArrivalWindow:
    """Require three new samples spanning a continuous in-tolerance interval."""

    def __init__(self, *, duration_s: float = .05, max_gap_s: float = .04):
        if not all(math.isfinite(x) and x > 0 for x in (duration_s, max_gap_s)):
            raise ValueError("Arrival window durations must be finite and positive")
        self.duration_s = duration_s
        self.max_gap_s = max_gap_s
        self.last_stamp: float | None = None
        self.since: float | None = None
        self.samples = 0

    def update(self, stamp: float, in_tolerance: bool) -> bool:
        if not math.isfinite(stamp):
            raise ValueError("Arrival sample timestamp must be finite")
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        gap = self.last_stamp is not None and stamp - self.last_stamp > self.max_gap_s
        self.last_stamp = stamp
        if gap or not in_tolerance:
            self.since, self.samples = None, 0
        if not in_tolerance:
            return False
        if self.since is None:
            self.since = stamp
        self.samples += 1
        return self.samples >= 3 and stamp - self.since + 1e-9 >= self.duration_s
