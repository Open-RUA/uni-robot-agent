"""Host system helpers: token authentication and identity-checked process ownership."""

import json
import os
import signal
import time
from pathlib import Path

from ..core.contract import Principal


def load_tokens(path):
    data = json.loads(Path(path).expanduser().read_text())
    if not isinstance(data, dict) or not data:
        raise ValueError("Body requires a nonempty token table")
    if any(not isinstance(key, str) or not key or not isinstance(value, dict)
           or not isinstance(value.get("name"), str) or not value["name"] for key, value in data.items()):
        raise ValueError("invalid token table structure")
    return {key: Principal.from_dict(value) for key, value in data.items()}


PROC = Path("/proc")


def identity(pid, *, proc=None):
    try:
        base = (PROC if proc is None else proc) / str(int(pid))
        stat = (base / "stat").read_text().rsplit(") ", 1)[1].split()
        if stat[0] == "Z":
            return None
        return {
            "pid": int(pid),
            "start": stat[19],
            "exe": os.readlink(base / "exe"),
            "argv": (base / "cmdline").read_bytes().decode().rstrip("\0").split("\0"),
        }
    except (OSError, ValueError, IndexError):
        return None


def stop_owned(item):
    # Vendor destructor can block joining its receiver; bound that wait and
    # verify the original process identity before any escalation.
    for sig, wait_s in [(signal.SIGTERM, 5), (signal.SIGKILL, 2)]:
        current = identity(item["pid"])
        if current is None:
            return
        if current != item:
            raise RuntimeError("PID identity changed; refusing to signal it")
        os.kill(item["pid"], sig)
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if identity(item["pid"]) is None:
                return
            time.sleep(0.05)
    raise RuntimeError("Controller failed to exit")
