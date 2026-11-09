"""Ordinary-user owned Linux capture subprocess using the bounded native wire.

The native process owns USB; only its parent can use these anonymous pipes.
No privileged helper, shell, network listener, or independent capture service.
"""

import json
import logging
import os
import select
import struct
import subprocess
import threading
import time
from collections import deque
from pathlib import Path


REQUEST = struct.Struct("<8sHHIQQ")
RESPONSE = struct.Struct("<8sHHIIIII")


BINARY = Path(__file__).resolve().parent / "capture/rua-camera-linux-capture"


def _wait(fd, *, write, deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError("Linux capture IPC deadline exceeded")
    readable, writable, _ = select.select([] if write else [fd], [fd] if write else [], [], remaining)
    if not (writable if write else readable):
        raise TimeoutError("Linux capture IPC deadline exceeded")


def _read_exact(fd, count, deadline):
    result = bytearray()
    while len(result) < count:
        _wait(fd, write=False, deadline=deadline)
        data = os.read(fd, count-len(result))
        if not data:
            raise ConnectionError("Linux capture process disconnected")
        result.extend(data)
    return bytes(result)


class LinuxHelperConnection:
    def __init__(self, cameras=None):
        if os.geteuid() == 0:
            raise PermissionError("Linux camera capture must not run as root")
        self.lock = threading.Lock()
        self.closed = False
        self.diagnostics = deque(maxlen=32)
        from ...core.resources import runtime_root
        installed = runtime_root()/'native/cameras/rua-camera-linux-capture'
        binary = installed if installed.is_file() else BINARY
        args = [str(binary), "--stdio"]
        for role, spec in (cameras or {}).items():
            args.append(f"--camera={role},{spec['serial']},{spec['sdk_serial']}")
        self.process = subprocess.Popen(args, stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        self.log_thread = threading.Thread(target=self._drain_diagnostics,daemon=True)
        self.log_thread.start()

    def _drain_diagnostics(self):
        next_log = 0
        try:
            while data := self.process.stderr.readline(2048):
                line = data.decode("utf-8",errors="replace").strip()
                self.diagnostics.append(line)
                if time.monotonic() >= next_log:
                    logging.getLogger(__name__).warning("%s",line)
                    next_log = time.monotonic()+1
        except (OSError,ValueError):
            pass

    def request(self, operation, *, role_mask=0, after_sequence=0):
        with self.lock:
            if self.closed:
                raise ConnectionError("Linux capture connection is closed")
            deadline = time.monotonic()+2
            data = REQUEST.pack(b"THRGBD01",1,operation,role_mask,after_sequence,0)
            fd = self.process.stdin.fileno()
            while data:
                _wait(fd,write=True,deadline=deadline)
                data = data[os.write(fd,data):]
            fd = self.process.stdout.fileno()
            header = RESPONSE.unpack(_read_exact(fd,RESPONSE.size,deadline))
            magic,version,echo,status,meta_size,rgb_size,depth_size,reserved = header
            if (magic != b"THRGBR01" or version != 1 or echo != operation or reserved != 0
                    or status not in range(8) or meta_size > 65536
                    or rgb_size not in (0,640*480*3) or depth_size not in (0,640*480*2)
                    or bool(rgb_size) != bool(depth_size)
                    or ((rgb_size or depth_size) and (operation not in (4, 7) or status != 0))):
                raise ValueError("Invalid Linux native capture response")
            metadata = json.loads(_read_exact(fd,meta_size,deadline)) if meta_size else {}
            if not isinstance(metadata,dict):
                raise ValueError("Invalid Linux native metadata")
            return status,metadata,_read_exact(fd,rgb_size,deadline),_read_exact(fd,depth_size,deadline)

    def close(self):
        if self.closed:
            return
        self.closed = True
        # Do not take request lock: a blocked request must be interruptible.
        try:
            if self.process.poll() is None:
                self.process.terminate()
        except ProcessLookupError:
            pass
        try:
            self.process.wait(timeout=7)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=2)
        self.log_thread.join(1)
        for stream in (self.process.stdin,self.process.stdout,self.process.stderr):
            try: stream.close()
            except OSError: pass
