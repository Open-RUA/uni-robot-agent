"""Explicit, instance-scoped X5 controller lifecycle for installed hosts (rua-host controllers)."""

import argparse
import fcntl
import hashlib
import json
import subprocess
import time
from contextlib import ExitStack, contextmanager
from pathlib import Path

from ....core.resources import runtime_root
from ..limits import controller_profile
from ....host.config import load_host
from ....host.system import identity, stop_owned


PROC = Path("/proc")


def controllers():
    result = []
    for p in PROC.iterdir():
        if p.name.isdigit():
            item = identity(p.name)
            if item and Path(item["exe"].removesuffix(" (deleted)")).name == "X5Controller":
                result.append(item)
    return result

def controller_argv(executable, spec):
    controller = spec["controller"]
    namespace, name = controller.rsplit("/", 1)
    args = [
        str(executable),
        "--ros-args",
        "-r",
        f"__node:={name}",
        "-r",
        f"__ns:={namespace or '/'}",
        "-r",
        f"/arx_joy:={controller}/arx_joy",
    ]
    params = {
        "arm_can_id": spec["can"],
        "arm_control_type": "normal",
        "arm_end_type": controller_profile(spec).end_type,
        "catch_control_mode": 0,
        "go_home_position": [0.0] * 6,
        "arm_pub_topic_name": spec["status_topic"],
        "arm_sub_topic_name": spec["command_topic"],
        "vr_sub_topic_name": controller + "/vr_cmd",
        "vr_ack_topic": controller + "/vr_ack",
        "vr_takeover_topic": controller + "/vr_takeover",
        "vr_release_topic": controller + "/vr_release",
    }
    for k, v in params.items():
        args.extend(["-p", f"{k}:={json.dumps(v)}"])
    return args


def wait_ready(device, spec, item, *, cancel=None):
    from ..driver import ArxX5Driver, RclpyTransport

    driver = ArxX5Driver(
        RclpyTransport("rua_startup_probe", exclusive=False, arms={device: spec}),
        arms={device: spec},
    )
    try:
        deadline = time.monotonic() + 20
        stable_since = None
        while time.monotonic() < deadline:
            if cancel is not None and cancel.is_set():
                raise RuntimeError("controller recovery cancelled during startup")
            if identity(item["pid"]) != item:
                raise RuntimeError("controller exited during startup")
            status = driver.check(device)
            if status["readiness"] == "satisfied":
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 2.0:
                    status["startup_stable_s"] = round(
                        time.monotonic() - stable_since, 3
                    )
                    return status
            else:
                stable_since = None
            time.sleep(0.2)
        raise RuntimeError(f"controller feedback/CAN check failed: {status}")
    finally:
        driver.close()


def owned_controller(spec, path, known):
    saved = json.loads(path.read_text()) if path.exists() else None
    matching = [
        p for p in known
        if f'arm_can_id:="{spec["can"]}"' in p["argv"]
        or f"arm_can_id:={spec['can']}" in p["argv"]
    ]
    if matching and (
        len(matching) != 1
        or matching[0] != saved
        or Path(saved["exe"]).name != "X5Controller"
        or saved["argv"] != controller_argv(Path(saved["exe"]), spec)
    ):
        raise RuntimeError("foreign or mismatched controller; explicit handover required")
    return matching[0] if matching else None


def start_controller(executable, spec, path, device, *, cancel=None):
    if cancel is not None and cancel.is_set():
        raise RuntimeError("controller startup cancelled")
    verify_controller_profile(executable, spec)
    with path.with_suffix(".log").open("ab") as log:
        child = subprocess.Popen(
            controller_argv(executable, spec),
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
    time.sleep(0.15)
    item = identity(child.pid)
    if not item:
        raise RuntimeError("controller did not start")
    try:
        temporary = path.with_name(path.name + ".tmp")
        temporary.write_text(json.dumps(item))
        temporary.replace(path)
        status = (
            wait_ready(device, spec, item)
            if cancel is None
            else wait_ready(device, spec, item, cancel=cancel)
        )
        return item, status
    except BaseException:
        stop_owned(item)
        path.unlink(missing_ok=True)
        raise


def verify_controller_profile(executable, spec):
    """Reject mismatched SDK/model bytes before the constructor can touch CAN."""
    profile = controller_profile(spec)
    base = Path(executable).resolve().parent.parent.parent
    for file, expected in (
        (base / "lib" / profile.sdk_filename, profile.sdk_sha256),
        (base / "share/arx_x5_controller" / profile.model_filename, profile.model_sha256),
    ):
        if hashlib.sha256(file.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f"controller profile mismatch before startup: {file.name}")


@contextmanager
def controller_lock(state, can_key):
    """Allow different arms; exclude same-arm and whole-host maintenance."""
    from ....core.contract import DeviceError, ErrorCode

    with ExitStack() as stack:
        for name, mode in (("lock", fcntl.LOCK_SH), (can_key + ".lock", fcntl.LOCK_EX)):
            lock = stack.enter_context((state / name).open("a"))
            try:
                fcntl.flock(lock, mode | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise DeviceError(
                    ErrorCode.RESOURCE_BUSY, f"{can_key} controller lifecycle is busy"
                ) from exc
        yield


def recover(config, instance, device, spec, *, preflight, cancel, progress):
    """Restart one owned controller through the driver's explicit invoke operation."""
    from ....core.contract import DeviceError, ErrorCode

    host = load_host(config)
    body = host["bodies"][instance]
    if body["kit"] != "arx_x5" or body["driver_args"]["arms"].get(device) != spec:
        raise DeviceError(ErrorCode.PRECONDITION_FAILED, "Body/host arm mapping changed")
    state = Path(host["state_dir"]) / instance / "controllers"
    state.mkdir(parents=True, exist_ok=True)
    path = state / (spec["can_key"] + ".json")
    with controller_lock(state, spec["can_key"]):
        try:
            item = owned_controller(spec, path, controllers())
            if item is None:
                raise RuntimeError("no owned controller; inspect the host before initial startup")
            if cancel.is_set():
                return {"completed": False, "restarted": False, "partial": False}
            before = preflight()
            if before["ready"]:
                return {"completed": True, "restarted": False, "reason": "already_ready"}
            # Reuse the exact installed executable already owned by this arm.
            executable = Path(item["exe"])
            if not executable.is_file():
                raise RuntimeError("owned controller executable is unavailable")
            verify_controller_profile(executable, spec)
            can_path = Path("/sys/class/net") / spec["can"]
            ifindex = (can_path / "ifindex").read_text()
            if not int((can_path / "flags").read_text(), 16) & 1:
                raise RuntimeError("CAN interface is down")
        except (OSError, ValueError, RuntimeError) as exc:
            raise DeviceError(ErrorCode.PRECONDITION_FAILED, str(exc)) from exc
        progress(0.2)
        if cancel.is_set():
            return {"completed": False, "restarted": False, "partial": False}
        try:
            stop_owned(item)
            path.unlink(missing_ok=True)
            if cancel.is_set():
                raise RuntimeError("recovery cancelled after controller stop")
            if (can_path / "ifindex").read_text() != ifindex:
                raise RuntimeError("CAN interface was replaced during recovery")
            if not int((can_path / "flags").read_text(), 16) & 1:
                raise RuntimeError("CAN interface went down during recovery")
            progress(0.5)
            current, status = start_controller(
                executable, spec, path, device, cancel=cancel
            )
            if (can_path / "ifindex").read_text() != ifindex:
                stop_owned(current)
                path.unlink(missing_ok=True)
                raise RuntimeError("CAN interface was replaced during startup")
        except Exception as exc:
            raise DeviceError(
                ErrorCode.OUTCOME_UNKNOWN,
                f"controller recovery did not complete: {exc}",
                execution_outcome="unknown",
                details={"device_id": device, "previous_pid": item["pid"]},
            ) from exc
        progress(0.9)
        return {
            "completed": True,
            "restarted": True,
            "previous_pid": item["pid"],
            "controller_pid": current["pid"],
            "readiness": status,
            "can_preserved": True,
            "old_operations_resumed": False,
        }


def lifecycle(config, instance, device, spec, *, action, cancel=None):
    """Body-owned start/stop/status; preserve the existing CAN interface."""
    from ....core.contract import DeviceError, ErrorCode

    host = load_host(config)
    if host["bodies"][instance]["driver_args"]["arms"].get(device) != spec:
        raise DeviceError(ErrorCode.PRECONDITION_FAILED, "Body/host arm mapping changed")
    state = Path(host["state_dir"]) / instance / "controllers"
    state.mkdir(parents=True, exist_ok=True)
    path = state / (spec["can_key"] + ".json")
    if action == "status":
        try:
            item = owned_controller(spec, path, controllers())
            return {"owned": True, "running": item is not None, "pid": item["pid"] if item else None}
        except (OSError, ValueError, RuntimeError) as exc:
            return {"owned": False, "running": None, "error": str(exc)}
    if action not in ("start", "stop"):
        raise ValueError("unsupported controller lifecycle action")
    with controller_lock(state, spec["can_key"]):
        try:
            item = owned_controller(spec, path, controllers())
            if cancel is not None and cancel.is_set():
                return {"completed": False, "hardware_command_sent": False}
            if action == "start":
                if item:
                    return {"completed": True, "started": False, "controller_pid": item["pid"]}
                executable = runtime_root() / "native/ros/lib/arx_x5_controller/X5Controller"
                verify_controller_profile(executable, spec)
                net = Path("/sys/class/net") / spec["can"]
                ifindex = (net / "ifindex").read_text()
                if not int((net / "flags").read_text(), 16) & 1:
                    raise RuntimeError("CAN interface is down; operator start does not reconfigure CAN")
        except (OSError, ValueError, RuntimeError) as exc:
            raise DeviceError(ErrorCode.PRECONDITION_FAILED, str(exc)) from exc
        try:
            if action == "stop":
                if item:
                    stop_owned(item)
                path.unlink(missing_ok=True)
                return {"completed": True, "stopped": True, "can_preserved": True,
                        "motors_disabled_confirmed": False}
            current, status = start_controller(executable, spec, path, device, cancel=cancel)
            if (net / "ifindex").read_text() != ifindex:
                stop_owned(current)
                path.unlink(missing_ok=True)
                raise RuntimeError("CAN interface changed during startup")
            return {"completed": True, "started": True, "controller_pid": current["pid"],
                    "readiness": status, "can_preserved": True}
        except Exception as exc:
            raise DeviceError(ErrorCode.OUTCOME_UNKNOWN, f"controller {action} incomplete: {exc}",
                              execution_outcome="unknown") from exc


def run(action, config, instance, *, device_id=None):
    host = load_host(config)
    body = host["bodies"][instance]
    if body["kit"] != "arx_x5":
        raise ValueError("controller lifecycle requires an arm Body")
    body_profile = body["driver_args"].get("profile")
    arms = {
        device: {
            **spec,
            **({"profile": body_profile} if body_profile else {}),
        }
        for device, spec in body["driver_args"]["arms"].items()
    }
    if device_id is not None:
        if device_id not in arms:
            raise ValueError("device is not part of the selected arm Body")
        arms = {device_id: arms[device_id]}
    if action == "diagnose":
        # A passive probe: do not create/start a controller or touch CAN setup.
        from ..driver import ArxX5Driver, RclpyTransport

        driver = ArxX5Driver(
            RclpyTransport("rua_fault_probe", exclusive=False, arms=arms),
            arms=arms,
            calibration_path=body["driver_args"].get("calibration_path"),
        )
        try:
            time.sleep(1.25)
            print(
                json.dumps({device: driver.check(device) for device in arms}),
                flush=True,
            )
        finally:
            driver.close()
        return
    state = Path(host["state_dir"]) / instance / "controllers"
    state.mkdir(parents=True, exist_ok=True)
    executable = runtime_root() / "native/ros/lib/arx_x5_controller/X5Controller"
    with (state / "lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        known = controllers()
        prepared = []
        for device, spec in arms.items():
            path = state / (spec["can_key"] + ".json")
            # An installed Body update must not force a healthy native controller
            # to restart. Its saved executable remains pinned until explicit stop.
            # Still require the exact live PID/start-time/executable/arguments and
            # the current instance's configuration before managing that process.
            item = owned_controller(spec, path, known)
            if action == "start" and item is None:
                verify_controller_profile(executable, spec)
            prepared.append((device, spec, path, item))
        results = {}
        for device, spec, path, item in prepared:
            if action == "status":
                results[device] = item
                continue
            if action == "stop":
                if item:
                    stop_owned(item)
                path.unlink(missing_ok=True)
                results[device] = {"stopped": True, "can_preserved": True}
                continue
            if item:
                results[device] = wait_ready(device, spec, item)
                continue
            if not executable.is_file():
                raise RuntimeError("native controller is not installed")
            result = subprocess.run(
                ["sudo", "-n", "/usr/local/sbin/rua-can", "start", spec["can_key"]],
                capture_output=True,
                text=True,
            )
            if result.returncode:
                # rua-can reports its failure, including any started effect, as JSON on stderr.
                raise RuntimeError(f"rua-can start {spec['can_key']} failed: {result.stderr.strip() or result.returncode}")
            status = json.loads(result.stdout)[spec["can_key"]]
            if not status.get("up"):
                raise RuntimeError("CAN not up; refusing controller startup")
            _, results[device] = start_controller(executable, spec, path, device)
        print(json.dumps(results), flush=True)


def recover_via_body(config, instance, device, token_file):
    """Manual recovery uses the same authenticated invoke and renewable lease."""
    from ....core.client import ControlSession, HttpDeviceClient
    from ....core.contract import TERMINAL_STATES, DeviceError, ErrorCode, new_request_key

    host = load_host(config)
    body = host["bodies"][instance]
    if body["kit"] != "arx_x5" or device not in body["driver_args"]["arms"]:
        raise ValueError("recover requires one arm device in the selected Body")
    token = Path(token_file).expanduser().read_text().strip()
    if not token:
        raise ValueError("token file is empty")
    address = host.get("listen", "127.0.0.1")
    if address == "0.0.0.0":
        address = "127.0.0.1"
    if address == "::":
        address = "::1"
    if ":" in address and not address.startswith("["):
        address = f"[{address}]"
    client = HttpDeviceClient(f"http://{address}:{body['port']}", token=token)
    manifest = client.manifest()
    target = manifest.device(device)
    if manifest.body_id != body["body_id"] or target is None:
        raise ValueError("Body identity/device mismatch")
    if "recover" not in target.procedures:
        raise ValueError("Body does not advertise recover; update the installed Body runtime")
    request_key = new_request_key()
    print(json.dumps({"device_id": device, "request_key": request_key}), flush=True)
    with ControlSession(client) as control:
        control_id = control.acquire([device])
        try:
            op = client.invoke(device, "recover", {"site_ready": True},
                               control_id=control_id, request_key=request_key)
        except DeviceError as exc:
            if exc.code == ErrorCode.TRANSPORT_ERROR:
                raise DeviceError(ErrorCode.OUTCOME_UNKNOWN,
                                  "dispatch response lost; inspect lookup_request before retrying",
                                  execution_outcome="unknown", details={"request_key": request_key}) from exc
            raise
        print(json.dumps({"operation_id": op["operation_id"], "state": op["state"]}), flush=True)
        done = None
        try:
            done = client.wait_operation(op["operation_id"], timeout_s=45.0)
            if done["state"] not in TERMINAL_STATES:
                client.cancel_operation(op["operation_id"], control_id=control_id)
                done = client.wait_operation(op["operation_id"], timeout_s=5.0)
            if done["state"] not in TERMINAL_STATES:
                raise DeviceError(ErrorCode.OUTCOME_UNKNOWN, "recovery has no confirmed terminal outcome",
                                  execution_outcome="unknown")
            return done
        except DeviceError as exc:
            raise DeviceError(ErrorCode.OUTCOME_UNKNOWN, "recovery feedback or cancellation could not be confirmed",
                              execution_outcome="unknown", details={"operation_id": op["operation_id"],
                                                                    "request_key": request_key}) from exc
        finally:
            if done is None or done["state"] not in TERMINAL_STATES:
                try:
                    client.cancel_operation(op["operation_id"], control_id=control_id)
                except DeviceError:
                    pass


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("action", choices=["start", "stop", "status", "diagnose", "recover"])
    p.add_argument("--config", required=True)
    p.add_argument("--instance", required=True)
    p.add_argument("--device", help="Limit lifecycle action to one device, e.g. arx.right_arm")
    p.add_argument("--token-file", help="File containing an operate+lifecycle Body token")
    p.add_argument("--site-ready", action="store_true", help="Confirm arm/object support, people clear, and permission to reapply torque")
    a = p.parse_args(argv)
    if a.action == "recover":
        if not (a.device and a.token_file and a.site_ready):
            p.error("recover requires --device, --token-file and --site-ready")
        from ....core.contract import DeviceError

        try:
            result = recover_via_body(a.config, a.instance, a.device, a.token_file)
        except DeviceError as exc:
            result = {"state": "unknown" if exc.execution_outcome == "unknown" else "failed", "error": exc.to_dict()}
        except (OSError, ValueError) as exc:
            result = {"state": "failed", "error": {"message": str(exc)}}
        print(json.dumps(result), flush=True)
        return 0 if result.get("state") == "succeeded" else 1
    if a.token_file or a.site_ready:
        p.error("--token-file and --site-ready are only supported for recover")
    run(a.action, a.config, a.instance, device_id=a.device)
    return 0
