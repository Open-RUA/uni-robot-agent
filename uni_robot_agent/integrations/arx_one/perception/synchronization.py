"""Match a camera source time to bounded, source-stamped joint history."""

import math
from bisect import bisect_left

import numpy as np

from ..body.core.timing import TIMESTAMP_TOLERANCE_NS, timestamp_ns


class JointHistoryPending(LookupError):
    """A later source sample may complete the image's joint bracket."""


def joint_state_at_time(
    history: list[dict], image_time: dict, *, max_gap_s=0.05
) -> dict:
    """Interpolate at the reported frame time; this does not certify its clock map.

    JointHistoryPending means later feedback may complete the bracket. All
    other invalid histories fail immediately, without extrapolation.
    """
    if (
        isinstance(max_gap_s, bool)
        or not math.isfinite(max_gap_s)
        or not 0 < max_gap_s <= 0.2
    ):
        raise ValueError("max_gap_s must be positive and at most 0.2 s")
    if not isinstance(image_time, dict) or not isinstance(history, list):
        raise ValueError("image time and joint history must be structured records")
    target = timestamp_ns(image_time.get("source_timestamp_ns"))
    clock_id = image_time.get("clock_id")
    if not clock_id or image_time.get("source_clock") != "unix_realtime":
        raise ValueError("image host time is unavailable")
    if not history:
        raise JointHistoryPending("joint source history is empty")
    stamps, joints = [], []
    for sample in history:
        if not isinstance(sample, dict) or not isinstance(sample.get("timing"), dict):
            raise ValueError("joint history sample has no clock metadata")
        timing = sample.get("timing", {})
        if (
            timing.get("clock_id") != clock_id
            or timing.get("source_clock") != "unix_realtime"
        ):
            raise ValueError("image and joint history use different host clocks")
        if timing.get("source_semantics") != "controller_status_publication":
            raise ValueError("joint history does not contain controller source times")
        stamp = timestamp_ns(timing.get("source_timestamp_ns"))
        received = timestamp_ns(timing.get("received_realtime_ns"))
        timestamp_ns(timing.get("received_monotonic_ns"))
        if stamp > received + TIMESTAMP_TOLERANCE_NS:
            raise ValueError("joint source timestamp is later than reception")
        span = timing.get("clock_pair_span_ns")
        if not (type(span) is int or isinstance(span, str) and span.isdecimal()):
            raise ValueError("joint clock sampling span must be integer nanoseconds")
        span = int(span)
        if not 0 <= span <= TIMESTAMP_TOLERANCE_NS:
            raise ValueError("joint clock sampling span is missing or exceeds 1 ms")
        if stamps and stamp <= stamps[-1]:
            raise ValueError("joint source times must strictly increase")
        q = np.asarray(sample.get("joint_position_rad"), dtype=float)
        if q.ndim != 1 or not q.size or not np.isfinite(q).all() or (joints and q.shape != joints[0].shape):
            raise ValueError("joint history needs consistently shaped finite measured joint angles")
        stamps.append(stamp)
        joints.append(q)
    if target < stamps[0]:
        raise ValueError("image predates retained joint source history")
    if target > stamps[-1]:
        raise JointHistoryPending("joint history does not yet reach the image time")
    upper = bisect_left(stamps, target)
    lower = upper if stamps[upper] == target else upper - 1
    gap = stamps[upper] - stamps[lower]
    if gap > max_gap_s * 1e9:
        raise ValueError("joint source samples around the image have too large a gap")
    if lower != upper:
        a, b = (history[i]["timing"] for i in (lower, upper))
        wall_delta = timestamp_ns(b["received_realtime_ns"]) - timestamp_ns(
            a["received_realtime_ns"]
        )
        mono_delta = timestamp_ns(b.get("received_monotonic_ns")) - timestamp_ns(
            a.get("received_monotonic_ns")
        )
        if mono_delta <= 0 or abs(wall_delta - mono_delta) > TIMESTAMP_TOLERANCE_NS:
            raise ValueError("host clock discontinuity inside joint bracket")
    alpha = 0.0 if gap == 0 else (target - stamps[lower]) / gap
    q = joints[lower] + alpha * (joints[upper] - joints[lower])
    return {
        "joint_position_rad": q.tolist(),
        "source_timestamp_ns": str(target),
        "clock_id": clock_id,
        "bracket_source_timestamp_ns": [str(stamps[lower]), str(stamps[upper])],
        "bracket_gap_s": gap / 1e9,
        "interpolation_fraction": alpha,
        "basis": "joint_interpolation_at_camera_source_time",
        "image_time": dict(image_time),
        "clock_mapping_validated": False,
        "hardware_synchronized": False,
    }
