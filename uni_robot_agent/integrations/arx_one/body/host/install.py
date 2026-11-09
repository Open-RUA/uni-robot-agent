"""Install a validated host configuration and the files its kits need; no hardware start."""

import argparse
import json
import os
import pwd
import re
import subprocess
import tempfile
from pathlib import Path

from .config import load_host
from .kit import InstalledFile, load_kit

CONFIG_DIR = "etc/rua"
#: Kits may install configuration only; programs come from the release package.
ALLOWED_PREFIXES = ("/etc/",)
#: System directories that must never become the operator-owned state directory.
PROTECTED = {"/", "/etc", "/usr", "/var", "/var/lib", "/opt", "/home", "/tmp", "/root", "/run", "/srv"}


def planned_files(host):
    """Every root-owned file this host needs, validated before anything is written."""
    operator = host["operator"]
    data = {key: value for key, value in host.items() if key != "_path"}
    files = [InstalledFile(f"/{CONFIG_DIR}/host.json", json.dumps(data, indent=2) + "\n")]
    for name, config in host["kits"].items():
        files += load_kit(name).install_files(host, config, operator)
    normalized = []
    for item in files:
        path = os.path.normpath(item.path)
        if not item.path.startswith("/") or ".." in Path(item.path).parts or not path.startswith(ALLOWED_PREFIXES):
            raise ValueError(f"kits may install only absolute paths under {ALLOWED_PREFIXES}: {item.path}")
        normalized.append(path)
    if len(set(normalized)) != len(normalized):
        raise ValueError("two kits install the same file")
    for item in files:
        if item.check:
            with tempfile.NamedTemporaryFile(mode="w") as handle:
                handle.write(item.content)
                handle.flush()
                subprocess.run([part.replace("{}", handle.name) for part in item.check],
                               check=True, capture_output=True)
    return [InstalledFile(path, item.content, item.mode, item.check) for path, item in zip(normalized, files)]


def write_atomically(path, content, mode):
    """Replace ``path`` in one step with the final mode; never follow a symlink at the destination."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink():
        raise ValueError(f"refusing to replace a symlink: {path}")
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
    try:
        os.fchmod(fd, mode)
        with os.fdopen(fd, "w") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise


def check_state_dir(path, user):
    """The state directory becomes operator-owned, so it must be a dedicated directory."""
    resolved = os.path.normpath(path)
    protected = PROTECTED | {str(Path(item).resolve()) for item in PROTECTED}
    if resolved in protected or str(Path(resolved).resolve()) in protected or len(Path(resolved).parts) < 3:
        raise ValueError(f"state_dir must be a dedicated directory, not {resolved}")
    target = Path(resolved)
    if target.exists() and any(target.iterdir()) and user is not None and target.stat().st_uid not in (0, user.pw_uid):
        raise ValueError(f"state_dir {resolved} is owned by another user")


def install(config, *, root=Path("/")):
    """Write the host files under ``root``; only ``/`` is a real installation and requires root."""
    root = Path(root)
    real = root == Path("/")
    if real and os.geteuid() != 0:
        raise PermissionError("host configuration installation requires root")
    host = load_host(config)
    if not re.fullmatch("[a-z_][a-z0-9_-]*", host["operator"]):
        raise ValueError("invalid operator")
    user = pwd.getpwnam(host["operator"]) if real else None
    check_state_dir(host["state_dir"], user)
    files = planned_files(host)
    (root / CONFIG_DIR).mkdir(mode=0o755, parents=True, exist_ok=True)
    for item in files:
        write_atomically(root / item.path.lstrip("/"), item.content, item.mode)
    if real:
        for name in host["kits"]:
            for command in load_kit(name).after_install():
                subprocess.run(list(command), check=True)
    state = root / host["state_dir"].lstrip("/")
    state.mkdir(parents=True, exist_ok=True)
    if user is not None:
        os.chown(state, user.pw_uid, user.pw_gid)
    print(json.dumps({"host_id": host["host_id"], "instances": list(host["bodies"]),
                      "files": [item.path for item in files], "hardware_started": False}))
    return files


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", required=True)
    a = p.parse_args(argv)
    install(a.config)
    return 0
