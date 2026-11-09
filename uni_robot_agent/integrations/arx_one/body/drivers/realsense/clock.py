"""RealSense source times: SDK Global Time checked against the Body host clock, and RGB-D pairing.

The camera contract (``core/camera/contract.py``) only asks for the resulting ``source_times``; this is
how the RealSense driver produces them.
"""

from copy import deepcopy
from decimal import Decimal, InvalidOperation

from ...core.timing import TIMESTAMP_TOLERANCE_NS, timestamp_ns


def camera_source_time(source: dict, timing: dict, *, stream="rgb") -> dict:
    """Check a raw SDK frame time, without correcting it or claiming exposure sync."""
    if not isinstance(source, dict) or not isinstance(timing, dict):
        raise ValueError("camera source/callback clock metadata is invalid")
    rgb = source.get(stream, {})
    if not isinstance(rgb, dict) or rgb.get("timestamp_domain") != "Global Time":
        raise ValueError("camera needs SDK Global Time mapped to the Body host")
    clock_id = timing.get("clock_id")
    if not isinstance(clock_id, str) or not clock_id.startswith("linux-boot:") or not clock_id[11:]:
        raise ValueError("camera host clock identity is missing")
    try:
        milliseconds = Decimal(str(rgb["timestamp_ms"]))
        if not milliseconds.is_finite() or milliseconds <= 0:
            raise ValueError("camera source timestamp is invalid")
        source_ns = timestamp_ns(int(milliseconds * 1_000_000))
        callback_ns = timestamp_ns(timing["callback_realtime_ns"])
        span = timing["clock_pair_span_ns"]
        if not (type(span) is int or isinstance(span, str) and span.isdecimal()):
            raise ValueError("camera clock sampling span must be integer nanoseconds")
        span = int(span)
    except (KeyError, TypeError, InvalidOperation, OverflowError) as exc:
        raise ValueError("camera source/callback clock metadata is missing or invalid") from exc
    if not 0 <= span <= TIMESTAMP_TOLERANCE_NS:
        raise ValueError("camera clock sampling span exceeds 1 ms")
    if source_ns > callback_ns + TIMESTAMP_TOLERANCE_NS:
        raise ValueError("camera source timestamp is later than its callback")
    return {
        "source_timestamp_ns": str(source_ns),
        "clock_id": clock_id,
        "source_clock": "unix_realtime",
        "source_semantics": "sdk_frame_timestamp_not_verified_exposure_midpoint",
        "callback_minus_source_ms": (callback_ns - source_ns) / 1e6,
        "clock_mapping": "sdk_global_time_estimate",
        "clock_mapping_validated": False,
    }


#: RGB and depth exposures of one packet may differ by at most this much.
RGBD_MAX_SENSOR_SKEW_US = 20_000


def rgbd_source_times(source: dict, timing: dict) -> dict:
    """Validate one RGB-D packet from a same-rate camera, independently of packet IDs.

    The camera driver supplies, per stream, an equal ``frame_number`` and a
    ``metadata["Sensor Timestamp"]`` from a wrapping 32-bit microsecond device counter.
    """
    times = {name: camera_source_time(source, timing, stream=name) for name in ("rgb", "depth")}
    rgb, depth = source["rgb"], source["depth"]
    if int(rgb["frame_number"]) != int(depth["frame_number"]):
        raise ValueError("RGB/depth source frame numbers differ (same-rate pair required)")
    # Device metadata is a wrapping microsecond counter, not Unix time.
    a = int(rgb["metadata"]["Sensor Timestamp"])
    b = int(depth["metadata"]["Sensor Timestamp"])
    skew_us = ((a - b + 2**31) % 2**32) - 2**31
    if abs(skew_us) > RGBD_MAX_SENSOR_SKEW_US:
        raise ValueError(f"RGB/depth exposure midpoint skew exceeds {RGBD_MAX_SENSOR_SKEW_US / 1000:g} ms")
    times["pairing"] = {"valid": True, "sensor_skew_ms": skew_us / 1000,
                        "policy": "equal_frame_counter_and_sensor_skew_20ms",
                        "hardware_synchronized": False}
    return times


class CameraClockMonitor:
    """Detect SDK mapping discontinuity; never substitute callback time for source."""
    def __init__(self):
        self.previous = {}
        self.generation = None
        self.good = 0
        self.rejected = 0
        self.last_error = None
        self.anchors = {}
        self.clock_epoch = 0
        self.host_pair = None
        self.diagnostic = {}
        self.last_rejection = None
        self.consecutive_rejected = 0

    def diagnostics(self):
        """Bounded evidence only; never changes clock admission or timestamps."""
        return deepcopy({"current": self.diagnostic,
                         "last_rejection": self.last_rejection,
                         "rejected_total": self.rejected,
                         "consecutive_rejected": self.consecutive_rejected,
                         "consistent_samples": self.good, "clock_epoch": self.clock_epoch})

    def check(self, source, timing, generation):
        diagnostic_timing = timing if isinstance(timing, dict) else {}
        self.diagnostic = {"stage": "host_clock", "generation": generation,
                           "callback_realtime_ns": diagnostic_timing.get("callback_realtime_ns"),
                           "clock_pair_span_ns": diagnostic_timing.get("clock_pair_span_ns"),
                           "streams": {}}
        if generation != self.generation:
            self.previous, self.good, self.generation = {}, 0, generation
            self.anchors = {}
            self.host_pair = None
            self.clock_epoch += 1
        try:
            pair = (timestamp_ns(timing["callback_realtime_ns"]),
                    timestamp_ns(timing["callback_monotonic_raw_ns"]))
            previous_host = self.host_pair
            self.host_pair = pair
            if previous_host is not None:
                wall, mono = previous_host
                self.diagnostic["host_step_ns"] = (pair[0]-wall) - (pair[1]-mono)
                if pair[1] <= mono or abs((pair[0]-wall) - (pair[1]-mono)) > TIMESTAMP_TOLERANCE_NS:
                    self.clock_epoch += 1
                    raise ValueError("host realtime clock stepped relative to monotonic")
            self.diagnostic["stage"] = "source_time_and_rgbd_pair"
            # Keep raw values even if source-time validation itself rejects the frame.
            for name in ("rgb", "depth"):
                raw_source = source.get(name, {}) if isinstance(source, dict) else {}
                raw_source = raw_source if isinstance(raw_source, dict) else {}
                metadata = raw_source.get("metadata", {})
                metadata = metadata if isinstance(metadata, dict) else {}
                self.diagnostic["streams"][name] = {
                    "timestamp_ms": raw_source.get("timestamp_ms"),
                    "timestamp_domain": raw_source.get("timestamp_domain"),
                    "frame_number": raw_source.get("frame_number"),
                    "hardware_frame_timestamp_us": metadata.get("Frame Timestamp"),
                    "sensor_timestamp_us": metadata.get("Sensor Timestamp"),
                }
            times = rgbd_source_times(source, timing)
            now = {}
            for name in ("rgb", "depth"):
                self.diagnostic["stage"] = name + "_source_age"
                value = times[name]
                metrics = self.diagnostic["streams"][name]
                metrics["callback_minus_source_ms"] = value["callback_minus_source_ms"]
                if value["callback_minus_source_ms"] > 500:
                    raise ValueError(f"{name} source frame is older than 500 ms")
                raw = int(source[name]["metadata"]["Frame Timestamp"])
                mapped = int(value["source_timestamp_ns"])
                seq = int(source[name]["frame_number"])
                now[name] = (raw, mapped, seq)
                anchor_raw, anchor_mapped = self.anchors.get(name, (raw, mapped))
                raw_elapsed = ((raw-anchor_raw) % 2**32) * 1000
                self.diagnostic["stage"] = name + "_mapping_drift"
                metrics["mapping_drift_ns"] = (mapped-anchor_mapped)-raw_elapsed
                metrics["mapping_drift_limit_ns"] = 5_000_000 + raw_elapsed * .001
                if abs((mapped-anchor_mapped)-raw_elapsed) > 5_000_000 + raw_elapsed * .001:
                    raise ValueError(f"{name} SDK clock mapping drifted relative to hardware time")
                if name not in self.anchors or raw_elapsed >= 5_000_000_000:
                    self.anchors[name] = (raw, mapped)
                if name in self.previous:
                    old_raw, old_mapped, old_seq = self.previous[name]
                    delta = (raw - old_raw) % 2**32
                    self.diagnostic["stage"] = name + "_counter"
                    metrics.update(previous_frame_number=old_seq, hardware_delta_us=delta)
                    if seq <= old_seq or not 0 < delta < 10_000_000:
                        raise ValueError(f"{name} source counter discontinuity")
                    self.diagnostic["stage"] = name + "_mapping_jump"
                    metrics["mapping_jump_ns"] = (mapped - old_mapped) - delta * 1000
                    if abs((mapped - old_mapped) - delta * 1000) > 5_000_000:
                        raise ValueError(f"{name} SDK clock mapping jumped relative to hardware time")
            self.previous = now
            self.good += 1
            self.last_error = None
            self.consecutive_rejected = 0
            self.diagnostic["stage"] = "warming" if self.good < 3 else "accepted"
            for name in ("rgb", "depth"):
                times[name]["clock_mapping_validated"] = False
                times[name]["clock_health"] = "consistent" if self.good >= 3 else "warming"
            if self.good < 3:
                return times, "camera clock needs three consistent source samples"
            return times, None
        except (ValueError, TypeError, KeyError) as exc:
            self.previous, self.good, self.anchors = {}, 0, {}
            self.rejected += 1
            self.last_error = str(exc)
            self.consecutive_rejected += 1
            self.diagnostic["error"] = self.last_error
            self.last_rejection = deepcopy(self.diagnostic)
            return {}, self.last_error
