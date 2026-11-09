"""Root-owned narrowly scoped USB-CAN lifecycle helper; never emits motor frames."""

import argparse
import errno
import fcntl
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import time

from ....core.resources import runtime_root
from ....host.system import identity as process_identity


STATE_DIR = Path("/run/rua-can")
CONFIG = Path("/etc/rua/arms.json")
PROC = Path("/proc")
IP = "/usr/sbin/ip"
SLCAND = "/opt/rua-hardware/current/native/bin/slcand"
SLCAND_EXECUTABLES = (SLCAND, "/usr/bin/slcand")
MUTATION_STARTED = False


def mutation():
    global MUTATION_STARTED
    MUTATION_STARTED = True


def set_link(cfg, state):
    mutation()
    run(IP, "link", "set", cfg["can_interface"], state)


def spawn_slcand(cfg, log):
    try:
        child = subprocess.Popen(
            [
                SLCAND,
                "-o",
                "-f",
                "-c",
                "-s8",
                "-F",
                cfg["device"],
                cfg["can_interface"],
            ],
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
    except OSError as error:
        # Popen reports these synchronous exec failures after reaping its child.
        # Preserve any earlier recovery effects, but do not invent a new effect.
        if error.errno not in (
            errno.ENOENT,
            errno.EACCES,
            errno.ENOEXEC,
            errno.ENOTDIR,
        ):
            mutation()
        raise
    except BaseException:
        mutation()
        raise
    mutation()
    return child


def run(*args):
    return subprocess.run(args, capture_output=True, text=True, check=True).stdout


def process(pid):
    return process_identity(pid, proc=PROC)


def identity(cfg):
    path = Path(cfg["device"])
    if not path.is_char_device():
        raise RuntimeError(f"USB-CAN missing: {path}")
    properties = dict(
        line.split("=", 1)
        for line in run(
            "/usr/bin/udevadm", "info", "--query=property", "--name=" + str(path)
        ).splitlines()
        if "=" in line
    )
    for prop, key in [
        ("ID_SERIAL_SHORT", "serial"),
        ("ID_VENDOR_ID", "vendor_id"),
        ("ID_MODEL_ID", "product_id"),
    ]:
        if properties.get(prop) != cfg[key]:
            raise RuntimeError(f"{path}: {prop} does not match configured {key}")
    return path.resolve()


def net_status(cfg):
    iface = cfg["can_interface"]
    net = Path("/sys/class/net") / iface
    if not net.exists():
        return {"interface": iface, "present": False, "up": False}
    if int((net / "type").read_text()) != 280:
        raise RuntimeError(f"{iface} exists but is not CAN")
    counters = {
        p.name: int(p.read_text())
        for p in (net / "statistics").iterdir()
        if p.name
        in (
            "rx_packets",
            "tx_packets",
            "rx_errors",
            "tx_errors",
            "rx_dropped",
            "tx_dropped",
        )
    }
    return {
        "interface": iface,
        "present": True,
        "up": bool(int((net / "flags").read_text(), 16) & 1),
        "counters": counters,
    }


def matching_process(cfg, tty):
    matches = []
    for path in PROC.iterdir():
        if not path.name.isdigit():
            continue
        item = process(path.name)
        if not item or item["exe"] not in SLCAND_EXECUTABLES:
            continue
        argv = item["argv"]
        if len(argv) < 3:
            continue
        endpoint = Path(argv[-2])
        if argv[-1] == cfg["can_interface"] or (
            endpoint.exists() and endpoint.resolve() == tty
        ):
            if (
                argv[-1] != cfg["can_interface"]
                or not endpoint.exists()
                or endpoint.resolve() != tty
            ):
                raise RuntimeError(
                    "A slcand process owns this tty or CAN name with a different mapping"
                )
            if argv[1:-2] != ["-o", "-f", "-c", "-s8", "-F"]:
                raise RuntimeError(
                    "Existing slcand has unrecognized options; stop it manually first"
                )
            matches.append(item)
    if len(matches) > 1:
        raise RuntimeError("Multiple slcand owners detected")
    return matches[0] if matches else None


def device_snapshot(cfg):
    tty = identity(cfg)
    info = tty.stat()
    return {"tty": str(tty), "device_id": (info.st_dev, info.st_ino, info.st_rdev)}


def serial_fds(item, snapshot):
    """Compare the open file, not argv's symlink which may have been replaced."""
    if process(item["pid"]) != item:
        raise RuntimeError("slcand identity changed during descriptor inspection")
    descriptors = []
    for fd in (PROC / str(item["pid"]) / "fd").iterdir():
        target = os.readlink(fd)
        name = Path(target.removesuffix(" (deleted)")).name
        if not (name.startswith(("ttyACM", "ttyUSB")) or target == snapshot["tty"]):
            continue
        info = fd.stat()
        if not stat.S_ISCHR(info.st_mode):
            raise RuntimeError("slcand serial descriptor is not a character device")
        descriptors.append(
            {
                "fd": fd.name,
                "target": target,
                "device_id": (info.st_dev, info.st_ino, info.st_rdev),
            }
        )
    if process(item["pid"]) != item:
        raise RuntimeError("slcand identity changed during descriptor inspection")
    if len(descriptors) != 1:
        raise RuntimeError("Cannot verify exactly one slcand serial descriptor")
    descriptor = descriptors[0]
    descriptor["current"] = (
        descriptor["target"] == snapshot["tty"]
        and descriptor["device_id"] == snapshot["device_id"]
    )
    return descriptor


def no_controller(cfg):
    """A CAN repair must never disconnect or revive a running controller."""
    for path in PROC.iterdir():
        if not path.name.isdigit():
            continue
        item = process(path.name)
        if (
            not item
            or Path(item["exe"].removesuffix(" (deleted)")).name != "X5Controller"
        ):
            continue
        mappings = [
            arg.split(":=", 1)[1].strip("\"'")
            for arg in item["argv"]
            if arg.startswith("arm_can_id:=")
        ]
        if not mappings or cfg["can_interface"] in mappings:
            raise RuntimeError(
                "Controller is running or has an unknown CAN mapping; stop it before CAN recovery"
            )


def saved_owner(arm, item):
    try:
        saved = json.loads(state_path(arm).read_text())
    except (OSError, ValueError):
        raise RuntimeError(
            "Stale slcand has no verified RUA ownership record"
        ) from None
    if saved != item:
        raise RuntimeError(
            "Stale slcand does not match the recorded RUA process identity"
        )


def state_path(arm):
    return STATE_DIR / (arm + ".json")


def save(arm, item):
    tmp = state_path(arm).with_suffix(".tmp")
    tmp.write_text(json.dumps(item))
    tmp.chmod(0o600)
    tmp.replace(state_path(arm))


def terminate(arm, cfg, item, precondition=None):
    for sig, seconds in [(signal.SIGTERM, 2), (signal.SIGKILL, 1)]:
        current = process(item["pid"])
        if current is None:
            break
        if current != item:
            raise RuntimeError(
                "PID identity changed; refusing to signal another process"
            )
        if precondition:
            precondition()
        if net_status(cfg)["present"]:
            set_link(cfg, "down")
        # Recheck after bringing down the link and before each signal.
        if process(item["pid"]) != item:
            raise RuntimeError(
                "PID identity changed; refusing to signal another process"
            )
        if precondition:
            precondition()
        mutation()
        os.kill(item["pid"], sig)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and process(item["pid"]) is not None:
            time.sleep(0.05)
    if process(item["pid"]) is not None:
        raise RuntimeError("slcand did not exit")
    state_path(arm).unlink(missing_ok=True)


def start(arm, cfg):
    snapshot = device_snapshot(cfg)
    tty = Path(snapshot["tty"])
    existing = matching_process(cfg, tty)
    recovery = None
    if existing:
        descriptor = serial_fds(existing, snapshot)
        link = net_status(cfg)
        if link["present"] and descriptor["current"]:
            if not link["up"]:
                no_controller(cfg)
                if (
                    device_snapshot(cfg) != snapshot
                    or serial_fds(existing, snapshot) != descriptor
                ):
                    raise RuntimeError(
                        "USB-CAN changed during activation; retry startup after the device settles"
                    )
                no_controller(cfg)
                set_link(cfg, "up")
            save(arm, existing)
            return dict(net_status(cfg), pid=existing["pid"], reused=True), None
        saved_owner(arm, existing)

        def confirm_stale():
            no_controller(cfg)
            saved_owner(arm, existing)
            if (
                device_snapshot(cfg) != snapshot
                or serial_fds(existing, snapshot) != descriptor
            ):
                raise RuntimeError(
                    "USB-CAN ownership changed during recovery; refusing to signal"
                )
            if net_status(cfg)["present"] and descriptor["current"]:
                raise RuntimeError(
                    "CAN interface recovered during preflight; retry without restarting it"
                )

        confirm_stale()
        recovery = {
            "reason": "stale_serial_descriptor"
            if not descriptor["current"]
            else "missing_interface",
            "previous_pid": existing["pid"],
            "previous_serial_target": descriptor["target"],
        }
        terminate(arm, cfg, existing, precondition=confirm_stale)
        if device_snapshot(cfg) != snapshot:
            raise RuntimeError(
                "USB-CAN changed after stale owner exited; retry after the device settles"
            )
        if matching_process(cfg, tty):
            raise RuntimeError("A new slcand owner appeared during recovery")
    if net_status(cfg)["present"]:
        raise RuntimeError("Existing CAN interface has no verified slcand owner")
    no_controller(cfg)
    busy = subprocess.run(["/usr/bin/fuser", str(tty)], capture_output=True)
    if busy.returncode == 0:
        raise RuntimeError(f"{tty} is held by another process")
    if busy.returncode != 1:
        raise RuntimeError(f"Cannot verify whether {tty} is in use")
    if device_snapshot(cfg) != snapshot:
        raise RuntimeError("USB-CAN identity changed before startup")
    no_controller(cfg)
    with (STATE_DIR / (arm + ".log")).open("ab") as log:
        child = spawn_slcand(cfg, log)
    item = process(child.pid)
    if item is None:
        raise RuntimeError("slcand failed to start")
    save(arm, item)
    try:
        deadline = time.monotonic() + 3
        while not net_status(cfg)["present"]:
            if child.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError(
                    "slcand failed to create interface; inspect /run/rua-can/"
                    + arm
                    + ".log"
                )
            time.sleep(0.05)
        # Re-read after exec to record the final executable and full argv.
        item = process(child.pid)
        if item is None or item["exe"] != SLCAND:
            raise RuntimeError("slcand exited before interface activation")
        if (
            device_snapshot(cfg) != snapshot
            or not serial_fds(item, snapshot)["current"]
        ):
            raise RuntimeError("USB-CAN changed during slcand startup")
        no_controller(cfg)
        save(arm, item)
        set_link(cfg, "up")
        result = dict(net_status(cfg), pid=child.pid, reused=False)
        if recovery:
            result["recovery"] = recovery
        return result, item
    except BaseException:
        current = process(child.pid)
        if current and current["exe"] == SLCAND:
            terminate(arm, cfg, current, precondition=lambda: no_controller(cfg))
        raise


def stop(arm, cfg):
    no_controller(cfg)
    # A device can disappear during unplug; recorded process identity still works.
    item = None
    path = state_path(arm)
    if path.exists():
        saved = json.loads(path.read_text())
        current = process(saved["pid"])
        if current and current != saved:
            raise RuntimeError("Saved CAN PID was reused; refusing to signal it")
        item = current
    if item is None and Path(cfg["device"]).exists():
        item = matching_process(cfg, identity(cfg))
    if item:
        terminate(arm, cfg, item, precondition=lambda: no_controller(cfg))
    elif net_status(cfg)["present"]:
        raise RuntimeError("Cannot stop interface without a verified owner")
    return dict(net_status(cfg), stopped=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("start", "stop", "status"))
    parser.add_argument(
        "arm", help="configured CAN key, or both for all configured keys"
    )
    args = parser.parse_args()
    if os.geteuid() != 0:
        raise RuntimeError("Use the installed sudo-authorized rua-can command")
    os.umask(0o077)
    STATE_DIR.mkdir(mode=0o700, exist_ok=True)
    if STATE_DIR.is_symlink() or STATE_DIR.stat().st_uid != 0:
        raise RuntimeError("Invalid runtime state directory owner")
    cfg = json.loads(CONFIG.read_text())
    selected = tuple(cfg) if args.arm == "both" else (args.arm,)
    if any(k not in cfg for k in selected):
        raise ValueError("unknown configured CAN key")
    if any(
        not k
        or any(
            c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-"
            for c in k
        )
        for k in cfg
    ):
        raise ValueError("unsafe CAN configuration key")
    results, started = {}, []
    with (STATE_DIR / "lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            for arm in selected:
                if args.action == "start":
                    results[arm], item = start(arm, cfg[arm])
                    if item:
                        started.append((arm, item))
                elif args.action == "stop":
                    results[arm] = stop(arm, cfg[arm])
                else:
                    results[arm] = net_status(cfg[arm])
        except BaseException:
            for arm, item in reversed(started):
                terminate(
                    arm, cfg[arm], item, precondition=lambda: no_controller(cfg[arm])
                )
            raise
    print(json.dumps(results))


def command(argv):
    """rua-host can (run as root through rua-can): manage the CAN adapters of the installed release.

    Failures print one JSON line to stderr, including whether any effect had started, and exit 1.
    """
    global SLCAND, SLCAND_EXECUTABLES
    SLCAND = str(runtime_root() / "native/bin/slcand")
    # Retain ownership of pinned adapters across package upgrades; never
    # restart a live CAN adapter merely because `current` changed.
    installed = tuple(str(p.resolve()) for p in runtime_root().parent.glob("*/native/bin/slcand") if p.is_file())
    SLCAND_EXECUTABLES = (SLCAND, *installed)
    sys.argv = ["rua-can", *argv]
    try:
        main()
    except Exception as exc:
        print(json.dumps({"code": "can_helper_failed", "message": str(exc), "effects_started": MUTATION_STARTED}),
              file=sys.stderr)
        return 1
    return 0
