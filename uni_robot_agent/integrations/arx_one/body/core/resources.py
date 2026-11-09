"""Host-local resources shared by drivers and the host: the installed runtime root and device ownership locks."""

import fcntl
import hashlib
import os
from pathlib import Path


def runtime_root():
    return Path(os.environ.get("RUA_RUNTIME_ROOT", "/opt/rua-hardware/current"))


class ResourceLocks:
    def __init__(self, resources, directory=None):
        self.files = []
        root = Path(
            directory
            or os.environ.get(
                "RUA_LOCK_DIR", str(Path.home() / ".local/state/rua/locks")
            )
        )
        root.mkdir(parents=True, exist_ok=True)
        try:
            for resource in sorted(set(resources)):
                name = hashlib.sha256(resource.encode()).hexdigest()
                handle = (root / (name + ".lock")).open("a")
                try:
                    fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BaseException:
                    handle.close()
                    raise RuntimeError(f"hardware resource already owned: {resource}")
                self.files.append(handle)
        except BaseException:
            self.close()
            raise

    def close(self):
        for handle in self.files:
            handle.close()
        self.files.clear()
