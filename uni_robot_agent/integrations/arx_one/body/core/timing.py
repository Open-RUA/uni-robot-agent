"""Clock identity and integer timestamps for comparing times from processes on the same Linux boot."""

from functools import lru_cache
from pathlib import Path

TIMESTAMP_TOLERANCE_NS = 1_000_000


@lru_cache(maxsize=1)
def host_clock_id() -> str | None:
    try:
        return "linux-boot:" + Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except OSError:
        return None


def timestamp_ns(value) -> int:
    """Read integer nanoseconds without accepting float rounding or booleans."""
    if isinstance(value, bool) or not isinstance(value, (str, int)):
        raise ValueError("timestamp must be integer nanoseconds")
    if isinstance(value, str) and not value.isdecimal():
        raise ValueError("timestamp must be integer nanoseconds")
    result = int(value)
    if result <= 0:
        raise ValueError("timestamp must be positive")
    return result
